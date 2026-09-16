#pragma once

/**
 * @file tp_node_store.h
 * @brief TPCP v1 - the flash-backed store that loads and persists TpNodeConfig.
 *
 * Records are appended in 256-byte units (eight 32-byte flash blocks) so a
 * save never has to erase; the newest valid record wins the boot scan, and
 * the partition is erased only when it fills up. Reads are memory-mapped and
 * free, so the boot scan runs to completion in one call.
 *
 * Programming is incremental - one block per service() call - and only ever
 * advances while the control loop reports a de-energized bridge, because
 * flashing stalls the single-bank CPU for several PWM periods. This is the
 * only file in node/ that touches hardware, which is why it is separated: the
 * config record, the identity and the discovery engine stay host-testable.
 *
 * The record is serialized through a byte cursor rather than hand-packed
 * words: with sixty-odd fields the offsets are what an edit gets wrong, and a
 * cursor cannot get them wrong.
 *
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#include "../../core/phy/flash_eeprom.h"  // FlashEEPROM + tp_flash_programming_window
#include "tp_node_config.h"                   // TpNodeConfig and the record layout constants

// ============================================================================
// Record cursor
// ============================================================================

/**
 * @class TpRecordCursor
 * @brief Sequential little-endian reader / writer over a fixed-size record buffer.
 *
 * Every accessor advances the cursor by the width it moved; a cursor that
 * runs past the end stops moving and reads zeros, so a layout mistake can
 * corrupt nothing beyond the record itself.
 */
class TpRecordCursor {
public:
    /** @brief Bind to a buffer of @p size bytes. */
    TpRecordCursor(uint8_t* buf, uint32_t size) : buf_(buf), size_(size) {}

    /** @brief Write one byte. */
    void u8(uint8_t v) {
        if (pos_ < size_) {
            buf_[pos_++] = v;                        // Store and advance
        }
    }
    /** @brief Write a little-endian 16-bit word. */
    void u16(uint16_t v) { u8((uint8_t)v); u8((uint8_t)(v >> 8)); }
    /** @brief Write a little-endian 32-bit word. */
    void u32(uint32_t v) { u16((uint16_t)v); u16((uint16_t)(v >> 16)); }
    /** @brief Write a signed 32-bit word. */
    void i32(int32_t v) { u32((uint32_t)v); }
    /** @brief Write the low 48 bits of a signed 64-bit value. */
    void i48(int64_t v) { u32((uint32_t)v); u16((uint16_t)((uint64_t)v >> 32)); }
    /** @brief Write a float as bfloat16. */
    void bf16(float v) { u16(tp_pack_bf16(v)); }

    /** @brief Read one byte. */
    uint8_t get_u8() { return (pos_ < size_) ? buf_[pos_++] : 0u; }
    /** @brief Read a little-endian 16-bit word. */
    uint16_t get_u16() { const uint16_t lo = get_u8(); return (uint16_t)(lo | ((uint16_t)get_u8() << 8)); }
    /** @brief Read a little-endian 32-bit word. */
    uint32_t get_u32() { const uint32_t lo = get_u16(); return lo | ((uint32_t)get_u16() << 16); }
    /** @brief Read a signed 32-bit word. */
    int32_t get_i32() { return (int32_t)get_u32(); }
    /** @brief Read a sign-extended 48-bit value. */
    int64_t get_i48() {
        const uint64_t lo = get_u32();                                 // Bits 0-31
        const uint64_t hi = get_u16();                                 // Bits 32-47
        return (int64_t)((lo | (hi << 32)) << 16) >> 16;               // Sign-extend from bit 47
    }
    /** @brief Read a bfloat16 as float. */
    float get_bf16() { return tp_unpack_bf16(get_u16()); }

    /** @brief Bytes consumed so far. */
    uint32_t position() const { return pos_; }

private:
    uint8_t* buf_;      ///< Record bytes.
    uint32_t size_;     ///< Record size.
    uint32_t pos_ = 0;  ///< Cursor.
};

// ============================================================================
// Persistent configuration store
// ============================================================================

/**
 * @class TpNodeStore
 * @brief Loads and persists TpNodeConfig in the dedicated flash partition.
 */
class TpNodeStore {
public:
    /** State of the write engine. */
    enum class State : uint8_t {
        Idle = 0,   ///< Nothing in progress.
        Space,      ///< Looking for a free record slot.
        Erase,      ///< Partition full: erasing before restarting at zero.
        Write,      ///< Programming block block_ of the record.
        Verify      ///< Reading the record back to confirm it took.
    };

    /**
     * @brief Bind the partition driver.
     * @param mem FlashEEPROM instance wired to the configuration partition.
     */
    void attach(FlashEEPROM* mem) { flash_ = mem; }

    /**
     * @brief Scan the partition and restore the newest valid record.
     *
     * Pure reads, so this is safe to call at boot with the control loop
     * running and never stalls the CPU.
     *
     * @param[out] cfg Destination configuration; left untouched when no valid
     *                 record exists, so the caller's defaults survive.
     * @return true when a record was found and applied.
     */
    bool load(TpNodeConfig& cfg) {
        loaded_ = false;                                    // No record yet
        stored_version_ = 0u;                               // Nothing known
        if (flash_ == nullptr || flash_->get_max_size() < TP_CONFIG_BYTES) {
            return false;                                   // Partition not usable
        }
        const uint32_t region = flash_->get_max_size();     // Partition size in bytes
        uint32_t words[TP_CONFIG_WORDS];                    // Candidate record buffer
        uint32_t addr = 0;                                  // Scan cursor
        bool found = false;                                 // A valid record was seen

        while ((addr + TP_CONFIG_BYTES) <= region) {        // While a whole record fits
            if (!flash_->read_block(addr, &words[0])) {
                break;                                      // Read fault: keep what we have
            }
            if ((words[0] & 0xFFFFu) != TP_CONFIG_MAGIC) {
                addr += 32u;                                // Not a record header: step one block
                continue;
            }
            bool ok = true;                                 // Whole record readable
            for (uint32_t b = 1; b < TP_CONFIG_BLOCKS; ++b) {
                if (!flash_->read_block(addr + (b * 32u), &words[b * 8u])) {
                    ok = false;                             // Truncated record at the tail
                    break;
                }
            }
            if (!ok) {
                break;
            }
            if (crc32(words, TP_CONFIG_WORDS - 1u) == words[TP_CONFIG_WORDS - 1u]) {
                const uint8_t ver = (uint8_t)((words[0] >> 16) & 0xFFu);  // Layout version
                if (ver == TP_CONFIG_VERSION) {
                    unpack(words, cfg);                     // Newer record overwrites the older
                    found = true;
                }
                stored_version_ = ver;                      // Report even a foreign version
            }
            addr += TP_CONFIG_BYTES;                        // Step over the whole record
        }
        loaded_ = found;
        return found;
    }

    /**
     * @brief Arm a save of the given configuration.
     *
     * The record is serialized immediately into a RAM buffer so the caller may
     * keep editing the live configuration while the write drains.
     *
     * @param cfg Configuration to persist.
     * @return true when the write engine was armed.
     */
    bool begin_save(const TpNodeConfig& cfg) {
        if (flash_ == nullptr || flash_->get_max_size() < TP_CONFIG_BYTES) {
            return false;                                   // Partition not usable
        }
        if (state_ != State::Idle) {
            return false;                                   // A save is already draining
        }
        pack(cfg, buf_);                                    // Serialize + CRC now
        state_ = State::Space;                              // Start by finding a slot
        addr_ = 0;                                          // Scan cursor
        block_ = 0;                                         // First block
        save_ok_ = false;                                   // Result not known yet
        save_attempted_ = true;                             // Distinguishes "failed" from "never ran"
        return true;
    }

    /**
     * @brief Advance the write engine by at most one flash operation.
     *
     * Call from thread mode. Programming is skipped entirely unless the control
     * loop reports a de-energized bridge, because a flash stall freezes the
     * control ISR with whatever vector is latched - the invariant the whole
     * tp_flash_programming_window mechanism exists to protect.
     *
     * @return true when the engine is idle (nothing left to do).
     */
    bool service() {
        if (state_ == State::Idle) {
            return true;                                    // Nothing in flight
        }
        if (state_ == State::Space) {
            // Slot search is read-only: no stall, so it does not need the window.
            const uint32_t region = flash_->get_max_size();  // Partition size
            uint32_t probe[8];                               // One block of scratch
            while ((addr_ + TP_CONFIG_BYTES) <= region) {    // While a record still fits
                if (!flash_->read_block(addr_, probe)) {
                    state_ = State::Idle;                    // Read fault: give up cleanly
                    return true;
                }
                if (probe[0] == 0xFFFFFFFFu) {
                    state_ = State::Write;                   // Erased slot found
                    return false;
                }
                addr_ += TP_CONFIG_BYTES;                    // Step over the occupied record
            }
            addr_ = 0;                                       // Exhausted: restart at zero
            state_ = State::Erase;                           // ... after erasing the partition
            return false;
        }

        // Everything below programs flash and therefore stalls the CPU.
        if (!tp_flash_programming_window) {
            return false;                                    // Bridge is live: try again later
        }

        switch (state_) {
            case State::Erase:
                if (!flash_->erase(TP_FLASH_ERASE_KEY)) {
                    state_ = State::Idle;                    // Erase refused: abandon the save
                    return true;
                }
                state_ = State::Write;                       // Partition is blank now
                return false;
            case State::Write:
                if (!flash_->write(addr_ + (block_ * 32u), &buf_[block_ * 8u])) {
                    state_ = State::Idle;                    // Program fault: abandon the save
                    return true;
                }
                block_++;                                    // Next block of the record
                if (block_ >= TP_CONFIG_BLOCKS) {
                    state_ = State::Verify;                  // Whole record committed
                }
                return false;
            case State::Verify: {
                uint32_t back[TP_CONFIG_WORDS];              // Read-back buffer
                bool ok = true;                              // Every block read back
                for (uint32_t b = 0; b < TP_CONFIG_BLOCKS; ++b) {
                    ok = ok && flash_->read_block(addr_ + (b * 32u), &back[b * 8u]);
                }
                save_ok_ = ok && (crc32(back, TP_CONFIG_WORDS - 1u) == back[TP_CONFIG_WORDS - 1u]);
                state_ = State::Idle;                        // Done either way
                return true;
            }
            default:
                state_ = State::Idle;                        // Unreachable
                return true;
        }
    }

    /**
     * @brief Abandon a save that is still waiting for a programming window.
     *
     * Safe at any point: the engine only ever holds a serialized copy of the
     * record, and a partially written record fails its CRC on the next scan, so
     * nothing that has already been programmed can be mistaken for valid.
     */
    void abort() {
        state_ = State::Idle;   // Stop the engine
        save_ok_ = false;       // Report the save as failed
    }

    /** @brief true while a save is still draining. */
    bool is_busy() const { return state_ != State::Idle; }

    /** @brief true when the last completed save verified successfully. */
    bool is_saved() const { return save_ok_; }

    /** @brief true when a save has been attempted at all since boot. */
    bool save_attempted() const { return save_attempted_; }

    /** @brief true when the boot scan restored a stored record. */
    bool is_loaded() const { return loaded_; }

    /** @brief Layout version of the restored record (0 when none was found). */
    uint8_t stored_version() const { return stored_version_; }

    /** @brief Bytes still free in the partition, for the persistence diagnostics. */
    uint32_t free_bytes() {
        if (flash_ == nullptr) {
            return 0u;                                       // Nothing bound
        }
        const uint32_t region = flash_->get_max_size();      // Partition size
        uint32_t probe[8];                                   // Scratch block
        uint32_t addr = 0;                                   // Scan cursor
        while ((addr + TP_CONFIG_BYTES) <= region) {         // While a record still fits
            if (!flash_->read_block(addr, probe)) {
                break;                                       // Read fault: report what we know
            }
            if (probe[0] == 0xFFFFFFFFu) {
                break;                                       // First erased slot ends the scan
            }
            addr += TP_CONFIG_BYTES;                         // Step over the occupied record
        }
        return region - addr;
    }

    /**
     * @brief Erase the whole configuration partition.
     *
     * Used by the factory-reset keyword. Stalls the CPU for the erase, so the
     * caller must have an open programming window.
     *
     * @return true when the partition was erased.
     */
    bool erase_all() {
        if (flash_ == nullptr || !tp_flash_programming_window) {
            return false;                                    // Not bound, or the bridge is live
        }
        loaded_ = false;                                     // Nothing stored any more
        stored_version_ = 0u;                                // ... and no version either
        return flash_->erase(TP_FLASH_ERASE_KEY);
    }

private:
    /**
     * @brief Serialize a configuration into the record layout.
     *
     * Physical values are stored as bfloat16, the same encoding the motor
     * profile uses, so a value read over CAN is bit-identical to what is in
     * flash. Word 0 keeps the magic / version / address header the boot scan
     * recognizes; the CRC of everything before it is the last word.
     *
     * @param c Source configuration.
     * @param w Destination word array (TP_CONFIG_WORDS entries).
     */
    static void pack(const TpNodeConfig& c, uint32_t* w) {
        uint8_t* bytes = (uint8_t*)w;                          // Little-endian byte view of the words
        for (uint32_t i = 0; i < TP_CONFIG_BYTES; ++i) {
            bytes[i] = 0u;                                     // Reserved bytes read back as zero
        }
        TpRecordCursor s(bytes, TP_CONFIG_BYTES - 4u);         // Everything but the CRC word
        s.u16(TP_CONFIG_MAGIC);                                // Record marker
        s.u8(TP_CONFIG_VERSION);                               // Layout version
        s.u8(c.node_id);                                       // Node address
        s.u8(c.flags);                                         // Behaviour flags
        s.u8(c.safe_state);                                    // De-energized policy
        s.u8(c.watchdog_action);                               // Watchdog reaction
        s.u8(c.bus_type);                                      // Frame format
        s.u8(c.comm_mode);                                     // Response policy
        s.u8(c.burst_step);                                    // Burst period
        s.u16(c.arb_kbps);                                     // Arbitration rate
        s.u16(c.data_kbps);                                    // Data-phase rate
        s.bf16(c.heartbeat_s);                                 // PulseBeat period
        s.bf16(c.heartbeat_delay_s);                           // PulseBeat delay
        s.bf16(c.watchdog_s);                                  // Watchdog timeout
        s.u8(c.hb_retry);                                      // FaultBeat retries
        s.u8(c.hb_retry_force);                                // Retry mode
        for (uint32_t i = 0; i < TP_TELEMETRY_SLOTS; ++i) {    // Telemetry slots
            s.u8(c.telemetry[i].reg);
            s.u8(c.telemetry[i].flags);
            s.u16(c.telemetry[i].period_ms);
        }
        s.bf16(c.oc_limit_a);                                  // Overcurrent A
        s.bf16(c.oc_limit_b);                                  // Overcurrent B
        s.bf16(c.oc_limit_c);                                  // Overcurrent C
        s.bf16(c.peak_current_a);                              // Peak current
        s.u8(c.oc_disbalance);                                 // Disbalance
        s.u8(c.oc_alpha);                                      // Disbalance filter
        s.u8(c.oc_recovery);                                   // OC recovery
        s.bf16(c.ov_limit_v);                                  // Overvoltage
        s.bf16(c.uv_limit_v);                                  // Undervoltage
        s.bf16(c.supply_hyst_v);                               // Supply hysteresis
        s.u8(c.supply_recovery);                               // Supply recovery
        s.bf16(c.ot_limit_c);                                  // Overtemperature trip
        s.bf16(c.ot_hyst_c);                                   // Overtemperature hysteresis
        s.bf16(c.ot_warn_c);                                   // Overtemperature warning
        s.bf16(c.op_limit_w);                                  // Overpower trip
        s.bf16(c.op_limit_a);                                  // Bus overcurrent trip
        s.bf16(c.pt_baseline_w);                               // Power baseline
        s.bf16(c.pt_capacity_j);                               // Energy budget
        s.u8(c.pt_overdrive_s);                                // Overdrive duration
        s.u8(c.motor_temp_base_c);                             // Motor thermal baseline
        s.bf16(c.motor_derating);                              // Motor derating
        s.bf16(c.motor_rth);                                   // Motor thermal resistance
        s.bf16(c.current_limit_a);                             // Control envelope
        s.bf16(c.vel_limit_rps);                               // Velocity envelope
        s.bf16(c.accel_limit);                                 // Acceleration envelope
        s.bf16(c.jerk_limit);                                  // Jerk envelope
        s.bf16(c.err_vel_rps);                                 // Fault velocity limit
        s.bf16(c.err_accel);                                   // Fault acceleration limit
        s.bf16(c.err_jerk);                                    // Fault jerk limit
        s.u8(c.axis_id);                                       // Axis id
        s.u8(c.axis_inverted);                                 // Axis direction
        s.i32(c.axis_offset_q16);                              // Axis offset
        s.u8(c.led_mode);                                      // LED mode
        s.u8(c.led_on_off);                                    // LED timing
        s.u8(c.led_id);                                        // LED element
        s.u8(c.led_r);                                         // LED red
        s.u8(c.led_g);                                         // LED green
        s.u8(c.led_b);                                         // LED blue
        s.bf16(c.gear_backlash);                               // Backlash
        s.u16((uint16_t)c.gear_ratio_mul);                     // Ratio numerator
        s.u16(c.gear_ratio_div);                               // Ratio denominator
        for (uint32_t i = 0; i < TP_IO_FUNCTIONS; ++i) {       // IO map
            s.u8(c.io_map[i].on_pin);
            s.u8(c.io_map[i].off_pin);
            s.u8(c.io_map[i].flags);
        }
        for (uint32_t i = 0; i < 2u; ++i) {                    // Encoders
            s.u8(c.encoder[i].type_iface);
            s.u8(c.encoder[i].flags);
            s.u32(c.encoder[i].ppr);
        }
        s.u16(c.homing_flags);                                 // Homing flags
        s.bf16(c.homing_quick_vel);                            // Coarse speed
        s.bf16(c.homing_quick_torque);                         // Coarse torque
        s.bf16(c.homing_fine_retract);                         // Fine retract
        s.bf16(c.homing_fine_vel);                             // Fine speed
        s.bf16(c.homing_fine_torque);                          // Fine torque
        s.bf16(c.ref_offset_cw);                               // CW reference offset
        s.bf16(c.ref_offset_ccw);                              // CCW reference offset
        s.u32(c.ref_ang_cw);                                   // CW reference angle
        s.u32(c.ref_ang_ccw);                                  // CCW reference angle
        s.u32(c.ref_ang_zero);                                 // Zero reference angle
        s.i48(c.zero_offset_cw_q32);                           // Zero to CW endstop
        s.i48(c.zero_offset_ccw_q32);                          // Zero to CCW endstop
        s.i48(c.field_cw_q32);                                 // CW travel
        s.i48(c.field_ccw_q32);                                // CCW travel
        s.i48(c.internal_offset_q32);                          // Internal offset
        w[TP_CONFIG_WORDS - 1u] = crc32(w, TP_CONFIG_WORDS - 1u);  // Integrity of everything above
    }

    /**
     * @brief Deserialize a validated record into a configuration.
     * @param w Source word array (CRC and version already checked).
     * @param c Destination configuration.
     */
    static void unpack(const uint32_t* w, TpNodeConfig& c) {
        TpRecordCursor s((uint8_t*)w, TP_CONFIG_BYTES - 4u);   // Byte view, CRC excluded
        (void)s.get_u16();                                     // Magic
        (void)s.get_u8();                                      // Version
        c.node_id = s.get_u8();
        c.flags = s.get_u8();
        c.safe_state = s.get_u8();
        c.watchdog_action = s.get_u8();
        c.bus_type = s.get_u8();
        c.comm_mode = s.get_u8();
        c.burst_step = s.get_u8();
        c.arb_kbps = s.get_u16();
        c.data_kbps = s.get_u16();
        c.heartbeat_s = s.get_bf16();
        c.heartbeat_delay_s = s.get_bf16();
        c.watchdog_s = s.get_bf16();
        c.hb_retry = s.get_u8();
        c.hb_retry_force = s.get_u8();
        for (uint32_t i = 0; i < TP_TELEMETRY_SLOTS; ++i) {
            c.telemetry[i].reg = s.get_u8();
            c.telemetry[i].flags = s.get_u8();
            c.telemetry[i].period_ms = s.get_u16();
        }
        c.oc_limit_a = s.get_bf16();
        c.oc_limit_b = s.get_bf16();
        c.oc_limit_c = s.get_bf16();
        c.peak_current_a = s.get_bf16();
        c.oc_disbalance = s.get_u8();
        c.oc_alpha = s.get_u8();
        c.oc_recovery = s.get_u8();
        c.ov_limit_v = s.get_bf16();
        c.uv_limit_v = s.get_bf16();
        c.supply_hyst_v = s.get_bf16();
        c.supply_recovery = s.get_u8();
        c.ot_limit_c = s.get_bf16();
        c.ot_hyst_c = s.get_bf16();
        c.ot_warn_c = s.get_bf16();
        c.op_limit_w = s.get_bf16();
        c.op_limit_a = s.get_bf16();
        c.pt_baseline_w = s.get_bf16();
        c.pt_capacity_j = s.get_bf16();
        c.pt_overdrive_s = s.get_u8();
        c.motor_temp_base_c = s.get_u8();
        c.motor_derating = s.get_bf16();
        c.motor_rth = s.get_bf16();
        c.current_limit_a = s.get_bf16();
        c.vel_limit_rps = s.get_bf16();
        c.accel_limit = s.get_bf16();
        c.jerk_limit = s.get_bf16();
        c.err_vel_rps = s.get_bf16();
        c.err_accel = s.get_bf16();
        c.err_jerk = s.get_bf16();
        c.axis_id = s.get_u8();
        c.axis_inverted = s.get_u8();
        c.axis_offset_q16 = s.get_i32();
        c.led_mode = s.get_u8();
        c.led_on_off = s.get_u8();
        c.led_id = s.get_u8();
        c.led_r = s.get_u8();
        c.led_g = s.get_u8();
        c.led_b = s.get_u8();
        c.gear_backlash = s.get_bf16();
        c.gear_ratio_mul = (int16_t)s.get_u16();
        c.gear_ratio_div = s.get_u16();
        for (uint32_t i = 0; i < TP_IO_FUNCTIONS; ++i) {
            c.io_map[i].on_pin = s.get_u8();
            c.io_map[i].off_pin = s.get_u8();
            c.io_map[i].flags = s.get_u8();
        }
        for (uint32_t i = 0; i < 2u; ++i) {
            c.encoder[i].type_iface = s.get_u8();
            c.encoder[i].flags = s.get_u8();
            c.encoder[i].ppr = s.get_u32();
        }
        c.homing_flags = s.get_u16();
        c.homing_quick_vel = s.get_bf16();
        c.homing_quick_torque = s.get_bf16();
        c.homing_fine_retract = s.get_bf16();
        c.homing_fine_vel = s.get_bf16();
        c.homing_fine_torque = s.get_bf16();
        c.ref_offset_cw = s.get_bf16();
        c.ref_offset_ccw = s.get_bf16();
        c.ref_ang_cw = s.get_u32();
        c.ref_ang_ccw = s.get_u32();
        c.ref_ang_zero = s.get_u32();
        c.zero_offset_cw_q32 = s.get_i48();
        c.zero_offset_ccw_q32 = s.get_i48();
        c.field_cw_q32 = s.get_i48();
        c.field_ccw_q32 = s.get_i48();
        c.internal_offset_q32 = s.get_i48();
    }

    /**
     * @brief CRC-32 (IEEE 802.3, reflected) over a word array.
     *
     * Table-less: 32 shifts per word. The record is checked only at boot and
     * after a save, so a 512-byte lookup table would cost more flash than the
     * loop ever costs in time.
     *
     * @param w Words to cover.
     * @param n Number of words.
     * @return CRC of the covered words.
     */
    static uint32_t crc32(const uint32_t* w, uint32_t n) {
        uint32_t crc = 0xFFFFFFFFu;                     // Standard initial value
        for (uint32_t i = 0; i < n; ++i) {
            crc ^= w[i];                                // Absorb one whole word
            for (uint32_t b = 0; b < 32u; ++b) {        // One bit per iteration
                // Reflected polynomial 0xEDB88320; the mask is a branchless
                // "subtract one from the LSB" trick that yields 0 or all-ones.
                crc = (crc >> 1) ^ (0xEDB88320u & (uint32_t)(-(int32_t)(crc & 1u)));
            }
        }
        return ~crc;                                    // Standard final inversion
    }

    FlashEEPROM* flash_ = nullptr;        ///< Configuration partition driver.
    State    state_ = State::Idle;        ///< Write-engine state.
    uint32_t addr_ = 0;                   ///< Byte offset of the record being written.
    uint32_t block_ = 0;                  ///< Block of the record being programmed.
    uint32_t buf_[TP_CONFIG_WORDS] = {0}; ///< Serialized record awaiting programming.
    bool     save_ok_ = false;            ///< Last save verified.
    bool     save_attempted_ = false;     ///< A save has been armed since boot.
    bool     loaded_ = false;             ///< Boot scan restored a record.
    uint8_t  stored_version_ = 0;         ///< Layout version of the restored record.
};
