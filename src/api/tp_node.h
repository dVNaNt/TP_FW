#pragma once

/**
 * @file tp_node.h
 * @brief Node identity, persistent configuration and the UID discovery engine.
 *
 * Three things a drive on a shared bus must own before it can be talked to at
 * all, and all three live here:
 *
 * 1. **Identity** - a 56-bit UID derived from the MCU's factory-programmed
 *    unique device identifier. It never changes, is unique per silicon die and
 *    is what the discovery protocol arbitrates on.
 * 2. **Configuration** - the node address plus everything else the master can
 *    persist (link bit rates, protection limits, safe-state policy, watchdog,
 *    telemetry slots), stored in its own flash partition with a magic header,
 *    a version and a CRC32.
 * 3. **Discovery** - the state machine that answers the master's byte-by-byte
 *    UID search so an unaddressed drive can be found and given an address on a
 *    bus it shares with any number of identical drives.
 *
 * ## Why the configuration gets its own flash partition
 *
 * The motor-profile engine appends a record per save into the user partition
 * and erases the WHOLE partition when it runs out of slots (about 60 saves).
 * A node address living there would be silently destroyed by an unrelated
 * profile save, and a drive that forgets its bus address is a field failure.
 * NVM_CONFIG_SECTION_SIZE gives it a partition nothing else writes.
 *
 * ## Why the record is written from thread mode
 *
 * Flash programming stalls the single-bank CPU - tens of milliseconds for an
 * erase - and that stall freezes the control ISR with whatever voltage vector
 * is latched on the bridge. The store therefore only ever programs while
 * tp_flash_programming_window is true, i.e. while the last control pass
 * commanded zero current. Enforcing that is the caller's job (see
 * TpNodeStore::service).
 *
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#include "../lib/drivers/flash_eeprom.h"  // FlashEEPROM + tp_flash_programming_window
#include "tp_wire.h"                      // Protocol enumerations and codecs

/** Number of independently configurable cyclic telemetry producers. */
#define TP_TELEMETRY_SLOTS 4u

/** Magic word identifying a node-configuration record inside the partition. */
#define TP_CONFIG_MAGIC 0x7C0Fu  // "TunePulse Config", low half of word 0

/** Layout version of the configuration record; bumped on any field change. */
#define TP_CONFIG_VERSION 1u

/** Words in one configuration record (two 32-byte flash blocks). */
#define TP_CONFIG_WORDS 16u

/** Bytes in one configuration record. */
#define TP_CONFIG_BYTES (TP_CONFIG_WORDS * 4u)

/** Key FlashEEPROM demands before it will erase a partition. */
#define TP_FLASH_ERASE_KEY 0xDEADC0DEu

// ============================================================================
// Build-time defaults
// ============================================================================
//
// These are the values a node uses when its configuration partition is empty.
// They exist as build flags because the shipping defaults - no address, CAN-FD
// at 1 Mbit/s / 5 Mbit/s - are correct for a deployed drive but awkward on a
// bench: an unaddressed node answers nothing but discovery, and a classic-CAN
// USB adapter cannot talk to an FD link at all.
//
//   -D TP_DEFAULT_NODE_ID=1        first bring-up without running discovery
//   -D TP_DEFAULT_LINK_FLAGS=4     TP_CFG_CLASSIC_LINK: classic CAN 2.0A
//   -D TP_DEFAULT_ARB_KBPS=500     match a cheap adapter's bit rate
//
// A STORED configuration always wins over these, so once a node has been saved
// the build flags stop having any effect on it.

/** Node address a node adopts when nothing is stored. */
#ifndef TP_DEFAULT_NODE_ID
#define TP_DEFAULT_NODE_ID TP_NODE_UNASSIGNED
#endif

/** Behaviour flags (TP_CFG_*) a node adopts when nothing is stored. */
#ifndef TP_DEFAULT_LINK_FLAGS
#define TP_DEFAULT_LINK_FLAGS 0u
#endif

/** Arbitration bit rate in kbit/s a node adopts when nothing is stored. */
#ifndef TP_DEFAULT_ARB_KBPS
#define TP_DEFAULT_ARB_KBPS 1000u
#endif

/** CAN-FD data-phase bit rate in kbit/s a node adopts when nothing is stored. */
#ifndef TP_DEFAULT_DATA_KBPS
#define TP_DEFAULT_DATA_KBPS 5000u
#endif

// ============================================================================
// Node configuration
// ============================================================================

/** @brief One cyclic telemetry producer: what to send, how often, and when. */
struct TpTelemetrySlot {
    uint8_t  opcode;     ///< Status opcode to emit (0x01-0x0F); 0 = slot unused.
    uint8_t  flags;      ///< TpTelemetryFlag bits.
    uint16_t period_ms;  ///< Emission period [ms]; 0 means SYNC-triggered only.
};

/** Flag bits of TpNodeConfig::flags. */
#define TP_CFG_AUTO_ENABLE  (1u << 0)  ///< Enable the output stage as soon as the drive is calibrated.
#define TP_CFG_PROMISCUOUS  (1u << 1)  ///< Accept every identifier (bus monitoring).
#define TP_CFG_CLASSIC_LINK (1u << 2)  ///< Run classic CAN 2.0A instead of CAN-FD.

/**
 * @brief Everything the master can change and persist about this node.
 *
 * The struct is the RAM shadow: registers read and write it directly, and
 * apply() pushes it into the live subsystems. That indirection exists because
 * several protection limits are write-only in the driver layer (the proxies
 * program a DAC threshold and keep nothing), so the shadow is the only place
 * a configured value can be read back from.
 */
struct TpNodeConfig {
    uint8_t  node_id;          ///< Bus address, 0 = unassigned.
    uint8_t  flags;            ///< TP_CFG_* bits.
    uint8_t  safe_state;       ///< TpSafeState used while disabled or stopped.
    uint8_t  watchdog_action;  ///< TpSafeState entered when the watchdog expires.
    uint16_t heartbeat_ms;     ///< Heartbeat period [ms]; 0 disables the heartbeat.
    uint16_t watchdog_ms;      ///< Master-liveness timeout [ms]; 0 disables it.
    uint16_t arb_kbps;         ///< Arbitration bit rate [kbit/s].
    uint16_t data_kbps;        ///< CAN-FD data-phase bit rate [kbit/s].

    TpTelemetrySlot telemetry[TP_TELEMETRY_SLOTS];  ///< Cyclic telemetry configuration.

    float oc_limit_a;      ///< Phase-A hardware overcurrent threshold [A].
    float oc_limit_b;      ///< Phase-B hardware overcurrent threshold [A].
    float oc_limit_c;      ///< Phase-C hardware overcurrent threshold [A].
    float ov_limit_v;      ///< Overvoltage trip [V].
    float uv_limit_v;      ///< Undervoltage trip [V].
    float ot_limit_c;      ///< Overtemperature trip [degC].
    float ot_hyst_c;       ///< Overtemperature release hysteresis [degC].
    float op_limit_w;      ///< Overpower trip [W].
    float op_limit_a;      ///< Bus overcurrent trip [A].
    float current_limit_a; ///< Control current envelope [A].
    float vel_limit_rps;   ///< Velocity envelope [RPS].
    float accel_limit;     ///< Acceleration envelope [RPS^2].
    float jerk_limit;      ///< Jerk envelope [RPS^3].

    /**
     * @brief Load the compiled-in defaults.
     *
     * These mirror the values tunepulse_init() has always programmed, so a
     * board with an erased configuration partition behaves exactly like one
     * built before this layer existed.
     */
    void set_defaults() {
        node_id = TP_DEFAULT_NODE_ID;             // Normally unassigned: discovery hands out an address
        flags = TP_DEFAULT_LINK_FLAGS;            // Manual enable, CAN-FD, not promiscuous
        safe_state = TP_SAFE_FREEWHEEL;           // Coast when de-energized
        watchdog_action = TP_SAFE_FREEWHEEL;      // Coast when the master goes quiet
        heartbeat_ms = 100u;                      // 10 Hz liveness
        watchdog_ms = 0u;                         // Watchdog off until the master arms it
        arb_kbps = TP_DEFAULT_ARB_KBPS;           // 1 Mbit/s arbitration by default
        data_kbps = TP_DEFAULT_DATA_KBPS;         // 5 Mbit/s FD data phase by default
        for (uint32_t i = 0; i < TP_TELEMETRY_SLOTS; ++i) {
            telemetry[i].opcode = 0u;             // Slot unused
            telemetry[i].flags = 0u;              // Disabled
            telemetry[i].period_ms = 0u;          // No timer
        }
        oc_limit_a = 4.0f;                        // Matches the DRV8323H board wiring
        oc_limit_b = 4.0f;                        // Matches the DRV8323H board wiring
        oc_limit_c = 6.2f;                        // Phase C carries the sum current
        ov_limit_v = 51.0f;                       // Bus capacitor rating headroom
        uv_limit_v = 16.0f;                       // Gate-driver dropout margin
        ot_limit_c = 85.0f;                       // Silicon derating point
        ot_hyst_c = 10.0f;                        // Release at 75 degC, no chatter
        op_limit_w = 60.0f;                       // Thermal budget of the stage
        op_limit_a = 2.5f;                        // Bus current budget
        current_limit_a = 1.0f;                   // Conservative default envelope
        vel_limit_rps = 50.0f;                    // 3000 rpm
        accel_limit = 200.0f;                     // Reaches the velocity limit in 0.25 s
        jerk_limit = 2000.0f;                     // Smooth ramp shaping
    }
};

// ============================================================================
// Node identity
// ============================================================================

/**
 * @class TpNodeIdentity
 * @brief The immutable 56-bit UID this node is discovered by.
 *
 * The STM32 carries a 96-bit factory identifier (lot number, wafer number, die
 * X/Y). The protocol frame has room for 56 bits next to the node address, so
 * the 96 bits are folded into 56 with an FNV-1a mix. The fold is deterministic
 * (the same die always produces the same UID across reboots and reflashes) and
 * spreads every input bit across the output, so two dies that differ only in
 * their wafer coordinate still land far apart. With 2^56 codes, a bus of a few
 * hundred drives has a collision probability far below any other failure mode
 * on the board.
 */
class TpNodeIdentity {
public:
    /** @brief Compute the UID once; subsequent calls return the cached value. */
    void init() {
        if (ready_) {
            return;                                   // Already folded
        }
#ifdef UID_BASE
        const volatile uint32_t* uid = (const volatile uint32_t*)UID_BASE;  // Factory identifier
        const uint32_t w0 = uid[0];                   // Lot number / wafer coordinates
        const uint32_t w1 = uid[1];                   // Lot number continued
        const uint32_t w2 = uid[2];                   // Lot number continued
#else
        // Host builds and boards without a factory identifier: a fixed value
        // keeps the protocol testable, but such a node must be given its
        // address explicitly because discovery cannot tell two of them apart.
        const uint32_t w0 = 0x54554E45u;              // "TUNE"
        const uint32_t w1 = 0x50554C53u;              // "PULS"
        const uint32_t w2 = 0x45000001u;              // "E" + instance
#endif
        // FNV-1a over the twelve identifier bytes: one multiply and one xor per
        // byte, no table, and every input bit reaches every output bit.
        uint64_t h = 0xCBF29CE484222325ull;           // FNV offset basis
        h = fold(h, w0);                              // Mix word 0
        h = fold(h, w1);                              // Mix word 1
        h = fold(h, w2);                              // Mix word 2
        // Fold the top 8 bits back in rather than discarding them, so no input
        // information is thrown away when the hash is truncated to 56 bits.
        h = (h ^ (h >> 56)) & 0x00FFFFFFFFFFFFFFull;
        // A UID of all zeros would tie in every arbitration round, so map that
        // one degenerate value onto a fixed non-zero code.
        if (h == 0ull) {
            h = 0x00FFFFFFFFFFFFFFull;
        }
        for (uint32_t i = 0; i < TP_UID_BYTES; ++i) {
            uid_[i] = (uint8_t)(h >> (8u * i));       // Little-endian byte order
        }
        raw_ = h;                                     // Cache the packed form
        ready_ = true;
    }

    /** @brief The 56-bit UID as seven bytes, least significant byte first. */
    const uint8_t* bytes() const { return uid_; }

    /** @brief The 56-bit UID packed into a 64-bit word (top byte is zero). */
    uint64_t raw() const { return raw_; }

    /** @brief Low 32 bits of the UID, used as the guard on address changes. */
    uint32_t guard() const { return (uint32_t)raw_; }

    /**
     * @brief Test whether this node's UID starts with a given prefix.
     * @param prefix Prefix bytes, least significant byte first.
     * @param len    Prefix length in bytes (0 matches every node).
     * @return true when the first len bytes match.
     */
    bool prefix_matches(const uint8_t* prefix, uint8_t len) const {
        if (len > TP_UID_BYTES) {
            return false;                             // Longer than a UID: never matches
        }
        for (uint8_t i = 0; i < len; ++i) {
            if (uid_[i] != prefix[i]) {
                return false;                         // First mismatch decides
            }
        }
        return true;
    }

private:
    /**
     * @brief Mix one 32-bit word into an FNV-1a accumulator, byte by byte.
     * @param h Current accumulator.
     * @param w Word to absorb.
     * @return Updated accumulator.
     */
    static uint64_t fold(uint64_t h, uint32_t w) {
        for (uint32_t i = 0; i < 4u; ++i) {
            h ^= (uint64_t)((w >> (8u * i)) & 0xFFu);  // Absorb one byte
            h *= 0x100000001B3ull;                     // FNV prime
        }
        return h;
    }

    uint8_t  uid_[TP_UID_BYTES] = {0};  ///< Byte view handed to the discovery engine.
    uint64_t raw_ = 0;                  ///< Packed 56-bit view.
    bool     ready_ = false;            ///< init() has run.
};

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

// ============================================================================
// UID discovery engine
// ============================================================================

/**
 * @class TpDiscovery
 * @brief Answers the master's byte-by-byte UID search.
 *
 * The master isolates one node at a time by walking the UID tree. For search
 * step i it broadcasts the prefix it has already resolved; every node whose
 * UID starts with that prefix replies with a DATA-LESS frame whose identifier
 * is 0x500 | UID[i]. Because a CAN identifier is arbitrated bit by bit and a
 * dominant bit wins, the frame that survives arbitration is automatically the
 * one carrying the LOWEST byte value - the bus itself performs the minimum
 * search, at zero cost in software. The master records that byte and moves to
 * step i+1. After seven steps it holds a complete UID and assigns an address.
 *
 * A node that loses arbitration must not let the controller re-send its stale
 * reply into the next round, where it would answer a question that was never
 * asked. Two independent defences cover that: every new prefix request aborts
 * whatever is still pending in the transmitter, and a node only ever replies
 * once per request. Disabling automatic retransmission (DAR) on the link makes
 * the first defence unnecessary, but the protocol does not require it.
 */
class TpDiscovery {
public:
    /** @brief Enter discovery mode and reset the search state. */
    void begin() {
        active_ = true;      // Answer prefix requests from now on
        isolated_ = false;   // Not yet singled out by the master
    }

    /** @brief Leave discovery mode. */
    void end() {
        active_ = false;     // Ignore prefix requests again
        isolated_ = false;   // Clear any isolation
    }

    /** @brief true while the node participates in a UID search. */
    bool is_active() const { return active_; }

    /**
     * @brief Decide whether to answer one prefix request.
     *
     * A node that has already been isolated in this search round stays quiet so
     * the master can find the remaining devices without it winning every
     * arbitration again.
     *
     * @param id         Node identity (source of the UID).
     * @param prefix     Prefix bytes received from the master.
     * @param len        Prefix length in bytes (0 .. TP_UID_BYTES-1).
     * @param[out] reply Identifier to transmit with DLC 0 when the answer is yes.
     * @return true when this node must reply.
     */
    bool on_prefix(const TpNodeIdentity& id, const uint8_t* prefix, uint8_t len, uint16_t& reply) {
        if (!active_ || isolated_) {
            return false;                             // Not searching, or already found
        }
        if (len >= TP_UID_BYTES) {
            return false;                             // Nothing left to disclose
        }
        if (!id.prefix_matches(prefix, len)) {
            return false;                             // This node is not in the master's subtree
        }
        reply = (uint16_t)(TP_ID_DISCOVERY_BASE | id.bytes()[len]);  // Next UID byte rides the identifier
        return true;
    }

    /**
     * @brief Test whether an assignment frame addresses this node.
     *
     * The frame carries the full 7-byte UID plus the address to assign, so an
     * assignment can never land on the wrong drive even if two UIDs share
     * their first six bytes.
     *
     * @param id      Node identity.
     * @param payload The 8 frame bytes: UID[0..6] followed by the node address.
     * @param[out] assigned Address to adopt when the UID matches.
     * @return true when this node is the addressee.
     */
    bool on_assign(const TpNodeIdentity& id, const uint8_t* payload, uint8_t& assigned) {
        if (!id.prefix_matches(payload, TP_UID_BYTES)) {
            return false;                             // Meant for a different drive
        }
        assigned = payload[TP_UID_BYTES];             // Byte 7 carries the address
        isolated_ = true;                             // Stay quiet for the rest of the search
        return true;
    }

private:
    bool active_ = false;    ///< Discovery mode entered.
    bool isolated_ = false;  ///< Already isolated in this search round.
};
