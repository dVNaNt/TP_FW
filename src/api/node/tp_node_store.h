#pragma once

/**
 * @file tp_node_store.h
 * @brief TPCP v1 - the flash-backed store that loads and persists TpNodeConfig.
 *
 * Records are appended in 64-byte units so a save never has to erase; the
 * newest valid record wins the boot scan, and the partition is erased only
 * when it fills up. Reads are memory-mapped and free, so the boot scan runs to
 * completion in one call.
 *
 * Programming is incremental - one block per service() call - and only ever
 * advances while the control loop reports a de-energized bridge, because
 * flashing stalls the single-bank CPU for several PWM periods. This is the
 * only file in node/ that touches hardware, which is why it is separated: the
 * config record, the identity and the discovery engine stay host-testable.
 *
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#include "../../lib/drivers/flash_eeprom.h"  // FlashEEPROM + tp_flash_programming_window
#include "tp_node_config.h"                   // TpNodeConfig and the record layout constants
// ============================================================================
// Persistent configuration store
// ============================================================================

/**
 * @class TpNodeStore
 * @brief Loads and persists TpNodeConfig in the dedicated flash partition.
 *
 * Records are appended in 64-byte units (two 32-byte flash blocks) so a save
 * never has to erase; the newest valid record wins the boot scan. When the
 * partition fills up it is erased once and the record restarts at offset zero.
 *
 * Reads are memory-mapped and cost nothing, so the boot scan runs to
 * completion in one call. Programming is incremental - one block per
 * service() call - and only ever happens while the control loop reports a
 * de-energized bridge.
 */
class TpNodeStore {
public:
    /** State of the write engine. */
    enum class State : uint8_t {
        Idle = 0,   ///< Nothing in progress.
        Space,      ///< Looking for a free record slot.
        Erase,      ///< Partition full: erasing before restarting at zero.
        Write0,     ///< Programming the first block of the record.
        Write1,     ///< Programming the second block of the record.
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
            if (!flash_->read_block(addr + 32u, &words[8])) {
                break;                                      // Truncated record at the tail
            }
            if (crc32(words, TP_CONFIG_WORDS - 1u) == words[TP_CONFIG_WORDS - 1u]) {
                unpack(words, cfg);                         // Newer record overwrites the older
                stored_version_ = (uint8_t)((words[0] >> 16) & 0xFFu);
                found = true;
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
                    state_ = State::Write0;                  // Erased slot found
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
                state_ = State::Write0;                      // Partition is blank now
                return false;
            case State::Write0:
                if (!flash_->write(addr_, &buf_[0])) {
                    state_ = State::Idle;                    // Program fault: abandon the save
                    return true;
                }
                state_ = State::Write1;                      // First block committed
                return false;
            case State::Write1:
                if (!flash_->write(addr_ + 32u, &buf_[8])) {
                    state_ = State::Idle;                    // Program fault: abandon the save
                    return true;
                }
                state_ = State::Verify;                      // Whole record committed
                return false;
            case State::Verify: {
                uint32_t back[TP_CONFIG_WORDS];              // Read-back buffer
                save_ok_ = flash_->read_block(addr_, &back[0]) &&
                           flash_->read_block(addr_ + 32u, &back[8]) &&
                           (crc32(back, TP_CONFIG_WORDS - 1u) == back[TP_CONFIG_WORDS - 1u]);
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

    /**
     * @brief true when a save has been attempted at all since boot.
     *
     * Distinguishes "no save has happened" from "a save failed". Without it a
     * freshly booted node reports a write failure it never suffered, because
     * is_saved() is false in both cases.
     */
    bool save_attempted() const { return save_attempted_; }

    /** @brief true when the boot scan restored a stored record. */
    bool is_loaded() const { return loaded_; }

    /** @brief Layout version of the restored record (0 when none was found). */
    uint8_t stored_version() const { return stored_version_; }

    /**
     * @brief Bytes still free in the partition, for the persistence status chunk.
     */
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
     * Used by the factory-reset command. Stalls the CPU for the erase, so the
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
     * @brief Serialize a configuration into the 16-word record layout.
     *
     * Physical values are stored as bfloat16 pairs, the same encoding the motor
     * profile uses, so one word holds two limits.
     *
     * @param c Source configuration.
     * @param w Destination word array (TP_CONFIG_WORDS entries).
     */
    static void pack(const TpNodeConfig& c, uint32_t* w) {
        w[0] = (uint32_t)TP_CONFIG_MAGIC                       // Bits 0-15: record marker
             | ((uint32_t)TP_CONFIG_VERSION << 16)             // Bits 16-23: layout version
             | ((uint32_t)c.node_id << 24);                    // Bits 24-31: node address
        w[1] = (uint32_t)c.flags                               // Bits 0-7: behaviour flags
             | ((uint32_t)c.safe_state << 8)                   // Bits 8-15: de-energized policy
             | ((uint32_t)c.heartbeat_ms << 16);               // Bits 16-31: heartbeat period
        w[2] = (uint32_t)c.watchdog_ms                         // Bits 0-15: watchdog timeout
             | ((uint32_t)c.watchdog_action << 16);            // Bits 16-23: watchdog reaction
        w[3] = (uint32_t)c.arb_kbps                            // Bits 0-15: arbitration rate
             | ((uint32_t)c.data_kbps << 16);                  // Bits 16-31: data-phase rate
        for (uint32_t i = 0; i < TP_TELEMETRY_SLOTS; ++i) {    // One word per telemetry slot
            w[4 + i] = (uint32_t)c.telemetry[i].opcode
                     | ((uint32_t)c.telemetry[i].flags << 8)
                     | ((uint32_t)c.telemetry[i].period_ms << 16);
        }
        w[8]  = pair(c.oc_limit_a, c.oc_limit_b);              // Phase A / phase B overcurrent
        w[9]  = pair(c.oc_limit_c, c.ov_limit_v);              // Phase C overcurrent / overvoltage
        w[10] = pair(c.uv_limit_v, c.ot_limit_c);              // Undervoltage / overtemperature
        w[11] = pair(c.ot_hyst_c, c.op_limit_w);               // Thermal hysteresis / overpower
        w[12] = pair(c.op_limit_a, c.current_limit_a);         // Bus current / control envelope
        w[13] = pair(c.vel_limit_rps, c.accel_limit);          // Velocity / acceleration envelope
        w[14] = pair(c.jerk_limit, 0.0f);                      // Jerk envelope / reserved
        w[15] = crc32(w, TP_CONFIG_WORDS - 1u);                // Integrity of everything above
    }

    /**
     * @brief Deserialize a validated record into a configuration.
     * @param w Source word array (CRC already checked).
     * @param c Destination configuration.
     */
    static void unpack(const uint32_t* w, TpNodeConfig& c) {
        c.node_id = (uint8_t)((w[0] >> 24) & 0xFFu);
        c.flags = (uint8_t)(w[1] & 0xFFu);
        c.safe_state = (uint8_t)((w[1] >> 8) & 0xFFu);
        c.heartbeat_ms = (uint16_t)((w[1] >> 16) & 0xFFFFu);
        c.watchdog_ms = (uint16_t)(w[2] & 0xFFFFu);
        c.watchdog_action = (uint8_t)((w[2] >> 16) & 0xFFu);
        c.arb_kbps = (uint16_t)(w[3] & 0xFFFFu);
        c.data_kbps = (uint16_t)((w[3] >> 16) & 0xFFFFu);
        for (uint32_t i = 0; i < TP_TELEMETRY_SLOTS; ++i) {
            c.telemetry[i].opcode = (uint8_t)(w[4 + i] & 0xFFu);
            c.telemetry[i].flags = (uint8_t)((w[4 + i] >> 8) & 0xFFu);
            c.telemetry[i].period_ms = (uint16_t)((w[4 + i] >> 16) & 0xFFFFu);
        }
        c.oc_limit_a = lo(w[8]);
        c.oc_limit_b = hi(w[8]);
        c.oc_limit_c = lo(w[9]);
        c.ov_limit_v = hi(w[9]);
        c.uv_limit_v = lo(w[10]);
        c.ot_limit_c = hi(w[10]);
        c.ot_hyst_c = lo(w[11]);
        c.op_limit_w = hi(w[11]);
        c.op_limit_a = lo(w[12]);
        c.current_limit_a = hi(w[12]);
        c.vel_limit_rps = lo(w[13]);
        c.accel_limit = hi(w[13]);
        c.jerk_limit = lo(w[14]);
    }

    /** @brief Pack two floats into one word as a bfloat16 pair (low, high). */
    static uint32_t pair(float a, float b) {
        return (uint32_t)tp_pack_bf16(a) | ((uint32_t)tp_pack_bf16(b) << 16);
    }

    /** @brief Unpack the low bfloat16 half of a word. */
    static float lo(uint32_t w) { return tp_unpack_bf16((uint16_t)(w & 0xFFFFu)); }

    /** @brief Unpack the high bfloat16 half of a word. */
    static float hi(uint32_t w) { return tp_unpack_bf16((uint16_t)(w >> 16)); }

    /**
     * @brief CRC-32 (IEEE 802.3, reflected) over a word array.
     *
     * Table-less: 32 shifts per word. The record is 15 words and is checked
     * only at boot and after a save, so a 512-byte lookup table would cost more
     * flash than the loop ever costs in time.
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
    uint32_t buf_[TP_CONFIG_WORDS] = {0}; ///< Serialized record awaiting programming.
    bool     save_ok_ = false;            ///< Last save verified.
    bool     save_attempted_ = false;     ///< A save has been armed since boot.
    bool     loaded_ = false;             ///< Boot scan restored a record.
    uint8_t  stored_version_ = 0;         ///< Layout version of the restored record.
};
