#pragma once

/**
 * @file tp_engine.h
 * @brief TunePulse CAN Protocol engine: dispatch, registers, motion, telemetry, discovery.
 *
 * The engine is the whole protocol above the wire format. It owns:
 *
 * - **Dispatch** - decodes one received frame, classifies it by CMD group
 *   (register read / write, read-only, write-only command, burst), executes it
 *   against the firmware and answers on the response channel.
 * - **Registers** - packs every readable register from live state and applies
 *   every writable one, including the batch and range reads that stream many
 *   registers per request.
 * - **Motion** - assembles multi-frame motion tasks into executor segments,
 *   applies single-frame setpoints, and decodes the bit-packed burst stream.
 * - **Telemetry** - the PulseBeat plus four configurable cyclic producers,
 *   each able to run on a timer or on the bus time sync.
 * - **Safety** - feeds the control loop's master-liveness watchdog on every
 *   addressed frame and emits an emergency frame the instant the fault byte
 *   changes.
 * - **Discovery and identity** - answers the binary HashUID search, adopts an
 *   assigned address, reprograms the acceptance filters and persists it.
 * - **Persistence** - drives the non-blocking configuration store and gates
 *   every flash write behind the de-energized programming window.
 *
 * ## Where it runs
 *
 * poll() runs in THREAD mode from loop(). Nothing here touches control state
 * directly: every command that changes what the motor does goes through the
 * DriveControl mailbox, which the control ISR drains between passes. That is
 * what makes it safe for a protocol handler to be preempted at any instruction
 * by a 20 kHz control loop.
 *
 * ## Transport independence
 *
 * The engine never names a peripheral. It moves frames through two function
 * pointers (read one frame, write one frame) and asks the transport to rebind
 * its filters through a third. tp_can.h binds those to FDCAN1 and the two USB
 * transports.
 *
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#include "../lib/dev_log.h"      // Non-blocking diagnostic logging
#include "../lib/lib.h"          // TunePulseHeart
#include "tp_chunks.h"           // Frame layouts
#include "tp_node.h"             // Identity, configuration, store, discovery
#include "tp_registers.h"        // Register map and configuration application
#include "tp_wire.h"             // Identifier map, CMD map, codecs

// ============================================================================
// Transport interface
// ============================================================================

/**
 * @brief Fetch one pending frame, non-blocking.
 * @param[out] id  Full 11-bit identifier the frame arrived with.
 * @param[out] f   Frame payload, always 8 bytes (short frames are zero-padded).
 * @param[out] len Real payload length (1..8).
 * @return true when a frame was delivered, false when the interface is empty.
 */
typedef bool (*TpFrameRead)(uint16_t& id, TpFrame& f, uint8_t& len);

/**
 * @brief Transmit one 8-byte frame. Must not block; may drop when the queue is full.
 * @param id Identifier to transmit with.
 * @param f  Frame payload.
 * @return true when the frame was queued.
 */
typedef bool (*TpFrameWrite)(uint16_t id, const TpFrame& f);

/**
 * @brief Rebind the transport's acceptance filters to a node address.
 * @param node_id     Address to accept traffic for (255 = unassigned).
 * @param promiscuous true to additionally accept every identifier.
 */
typedef void (*TpBindFilters)(uint8_t node_id, bool promiscuous);

/** @brief Reboot the device; mode 1 jumps into the USB DFU bootloader. */
typedef void (*TpRebootFn)(uint8_t mode);

/**
 * @brief Health of the link to the position sensor, as the board sees it.
 *
 * Frame validation is board code (what a valid frame looks like, how the bus
 * is reset and what the sensor's status bits mean are properties of one SPI
 * link and one chip), so the engine does not read it from the generic
 * Encoder object: the board fills this snapshot on request through the probe
 * registered with bind_encoder_link(). Diagnostics record 0x43/0x07 is the
 * wire form of it; a board that registers no probe answers NOT_IMPLEMENTED.
 */
struct TpEncoderLinkInfo {
    bool     check_proven;   ///< The integrity check has proven itself and failing frames are dropped.
    bool     link_down;      ///< The link is declared down: nothing is ingested, the encoder fault is raised.
    uint8_t  sensor_status;  ///< The sensor's own status bits from the last accepted frame (low 3 bits).
    uint16_t relinks;        ///< Bus resets since boot.
    uint32_t frames_bad;     ///< Frames that failed the check since boot.
    uint32_t frames_ok;      ///< Frames that passed since boot.
};

/** @brief Fill a TpEncoderLinkInfo snapshot (board-provided). */
typedef void (*TpEncoderLinkProbe)(TpEncoderLinkInfo& out);

// ============================================================================
// Protocol engine
// ============================================================================

/**
 * @class TpProtocol
 * @brief The TunePulse CAN Protocol implementation for one drive.
 */
class TpProtocol {
public:
    // ------------------------------------------------------------------
    // Wiring
    // ------------------------------------------------------------------

    /**
     * @brief Bind the firmware and the transport.
     * @param tp      Live firmware state.
     * @param rd      Frame read callback.
     * @param wr      Frame write callback.
     * @param filters Acceptance-filter rebind callback.
     */
    void attach(TunePulseHeart* tp, TpFrameRead rd, TpFrameWrite wr, TpBindFilters filters);

    /**
     * @brief Bind the configuration partition and the reboot hook.
     * @param cfg_flash FlashEEPROM instance wired to the configuration partition.
     * @param reboot    Reboot / DFU entry function.
     */
    void bind_platform(FlashEEPROM* cfg_flash, TpRebootFn reboot);

    /**
     * @brief Register the board's position-sensor link probe (optional).
     * @param probe Fills a TpEncoderLinkInfo; nullptr leaves 0x43/0x07 unimplemented.
     */
    void bind_encoder_link(TpEncoderLinkProbe probe) { encoder_link_ = probe; }

    /**
     * @brief Bring the protocol up: identity, stored configuration, filters.
     *
     * Safe to call before the control loop is running; it performs flash READS
     * only, which are memory-mapped and never stall the CPU.
     */
    void begin();

    // ------------------------------------------------------------------
    // Service
    // ------------------------------------------------------------------

    /**
     * @brief Service the protocol: receive, execute, emit, persist.
     *
     * Call as often as possible from thread mode. Every stage is bounded, so
     * one call can never take an unpredictable amount of time: at most
     * MAX_FRAMES_PER_POLL frames are decoded, at most REPLY_BUDGET batch
     * replies are emitted, at most one flash operation is started, and the
     * telemetry scheduler emits at most one frame per slot.
     */
    void poll();

    // ------------------------------------------------------------------
    // Accessors
    // ------------------------------------------------------------------

    /** @brief The live configuration shadow. */
    TpNodeConfig& config() { return config_; }

    /** @brief The immutable device identity. */
    TpNodeIdentity& identity() { return identity_; }

    /** @brief The persistent configuration store. */
    TpNodeStore& store() { return store_; }

    /** @brief Frames rejected with a NACK since boot (saturating). */
    uint16_t rejected() const { return rejected_; }

    /** @brief Frames decoded and executed since boot (saturating). */
    uint16_t handled() const { return handled_; }

    /**
     * @brief Publish the transport's link health for the communication diagnostics.
     * @param tx_err    Hardware transmit error counter.
     * @param rx_err    Hardware receive error counter.
     * @param bus_flags bit0 error-passive, bit1 bus-off, bit2 warning, bit3 gateway armed.
     * @param dropped   Frames lost to RX ring overflow.
     */
    void set_bus_counters(uint8_t tx_err, uint8_t rx_err, uint8_t bus_flags, uint16_t dropped) {
        bus_tx_err_ = tx_err;      // Hardware transmit error counter
        bus_rx_err_ = rx_err;      // Hardware receive error counter
        bus_flags_ = bus_flags;    // Warning / error-passive / bus-off / gateway bits
        bus_dropped_ = dropped;    // Frames lost to RX ring overflow
    }

private:
    /** Frames decoded per poll() call, so one burst cannot monopolize loop(). */
    static constexpr uint32_t MAX_FRAMES_PER_POLL = 16u;

    /** Batch / range replies emitted per poll() call, so a range read cannot flood the TX ring. */
    static constexpr uint32_t REPLY_BUDGET = 4u;

    /** Delay between arming a reboot and executing it, so the reply gets out. */
    static constexpr uint32_t REBOOT_DELAY_MS = 20u;

    /** How long a pending save may wait for a de-energized programming window. */
    static constexpr uint32_t SAVE_TIMEOUT_MS = 2000u;

    /** Bytes in the batch bitmap (one bit per read CMD 0x00 - 0x7F). */
    static constexpr uint32_t BATCH_BITMAP_BYTES = 16u;

    /** Largest amplitude a MIDI note may drive, as a Q15 fraction of the bus voltage. */
    static constexpr uint32_t MIDI_MAX_AMP_Q15 = 8192u;

    /** Parts of a multi-frame motion task that have arrived (bits of seg_parts_). */
    static constexpr uint8_t SEG_PART_1 = 0x01u;
    static constexpr uint8_t SEG_PART_2 = 0x02u;
    static constexpr uint8_t SEG_PART_3 = 0x04u;
    static constexpr uint8_t SEG_PART_4 = 0x08u;

    // ==================================================================
    // Dispatch (engine/tp_engine_dispatch_impl.h)
    // ==================================================================

    /**
     * @brief Route one received frame by identifier and CMD group.
     * @param id  Identifier the frame arrived with.
     * @param f   Frame payload (zero-padded to 8 bytes).
     * @param len Real payload length.
     */
    void dispatch(uint16_t id, const TpFrame& f, uint8_t len);

    /**
     * @brief Execute one command-channel frame and answer it.
     * @param f         Frame payload.
     * @param broadcast true when the frame was addressed to every node.
     */
    void handle_frame(const TpFrame& f, bool broadcast);

    /**
     * @brief Handle the two discovery registers, which an unaddressed node accepts too.
     * @param f         Frame payload.
     * @param broadcast true when the frame reached every node.
     * @return true when the frame was a discovery register (handled or ignored).
     */
    bool handle_discovery(const TpFrame& f, bool broadcast);

    /** @brief Queue the registers named by a batch read (0x40). */
    TpError handle_batch_read(const TpFrame& f);

    /** @brief Start the range walk named by a range read (0x41). */
    TpError handle_range_read(const TpFrame& f);

    /** @brief Emit up to REPLY_BUDGET queued batch / range replies. */
    void service_batch();

    /** @brief Transmit a reply on this node's response identifier (comm-mode gated). */
    void respond(const TpFrame& f);

    /** @brief Whether a frame of this kind gets an answer under the comm policy. */
    bool answers(bool is_read, bool is_setpoint) const;

    /** @brief Build a generic acknowledgement. */
    static void ack(TpFrame& f, uint8_t cmd, uint8_t detail) {
        f.u64 = 0;                     // Reserved bytes transmit as zero
        f.ack.cmd = TP_CMD_ACK;        // Generic success reply
        f.ack.echo = cmd;              // What was executed
        f.ack.error = TP_ERR_NONE;     // No error by definition
        f.ack.detail = detail;         // Context
    }

    /** @brief Build a generic rejection and count it. */
    void nack(TpFrame& f, uint8_t cmd, TpError err, uint8_t detail);

    // ==================================================================
    // Register reads (engine/tp_engine_regread_impl.h)
    // ==================================================================

    /**
     * @brief Pack one readable register into a reply frame.
     *
     * @param cmd Read-form CMD.
     * @param sub Sub identifier (ignored for plain registers).
     * @param req The request frame, for keyed reads (IO function, table sample, address).
     * @param rsp Destination frame.
     * @return TP_ERR_NONE and a filled frame, otherwise the reason it cannot be read.
     */
    TpError read_register(uint8_t cmd, uint8_t sub, const TpFrame& req, TpFrame& rsp);

    /** @brief Pack a 0x01 mechanics record. */
    TpError read_axis(uint8_t sub, const TpFrame& req, TpFrame& rsp);

    /** @brief Pack a 0x50 - 0x58 status register. */
    TpError read_status(uint8_t cmd, TpFrame& rsp);

    /** @brief Pack a 0x43 diagnostics record. */
    TpError read_diag(uint8_t sub, TpFrame& rsp);

    /** @brief Pack the 0x00 direct memory access reply. */
    TpError read_direct(const TpFrame& req, TpFrame& rsp);

    /** @brief Pack a 0x2F calibration table sample or size. */
    TpError read_cal_table(const TpFrame& req, TpFrame& rsp);

    /** @brief Fill the PulseBeat frame (also used verbatim as the emergency frame). */
    void pack_pulsebeat(TpFrame& f, bool advance);

    // ==================================================================
    // Register writes (engine/tp_engine_regwrite_impl.h)
    // ==================================================================

    /**
     * @brief Apply one register write and build the read-form reply.
     * @param f   Request frame (CMD has the write bit set).
     * @param rsp Destination for the read-back reply.
     * @return TP_ERR_NONE when applied, otherwise the reason for rejection.
     */
    TpError write_register(const TpFrame& f, TpFrame& rsp);

    /** @brief Apply a 0x01 mechanics record write. */
    TpError write_axis(const TpFrame& f);

    /** @brief Apply a 0x20 System Control write. */
    TpError write_system_control(const TpFrame& f);

    /** @brief Apply a 0x21 Motor Properties 1 write. */
    TpError write_motor_props_1(const TpFrame& f);

    /** @brief Apply a 0x22 Motor Properties 2 write. */
    TpError write_motor_props_2(const TpFrame& f);

    /** @brief Apply a 0x10 Bus Settings write. */
    TpError write_bus(const TpFrame& f);

    /** @brief Apply a 0x2D Watchdog write. */
    TpError write_watchdog(const TpFrame& f);

    /** @brief Apply the 0x00 direct memory write. */
    TpError write_direct(const TpFrame& f);

    /** @brief Resolve a direct-access window address into a pointer (nullptr when unmapped). */
    volatile uint32_t* direct_pointer(uint32_t addr, bool for_write) const;

    // ==================================================================
    // Motion (engine/tp_engine_motion_impl.h)
    // ==================================================================

    /** @brief Handle one Motion Task frame (0xC0 - 0xC3). */
    TpError handle_motion_task(const TpFrame& f);

    /** @brief Apply a single-frame (SegmentID 0) motion task as a setpoint. */
    TpError apply_setpoint(const TpFrame& f);

    /** @brief Push the fully assembled multi-frame segment into the queue. */
    TpError commit_segment();

    /** @brief Handle one bit-packed burst frame (0xE0 - 0xFF). */
    TpError handle_burst(const TpFrame& f);

    /** @brief Convert a wire torque (N*m, or A without Kt) into the executor's normalized scale. */
    uint16_t torque_norm(float torque) const;

    /** @brief Convert a wire torque into a q-axis current [A]. */
    float torque_to_amps(float torque) const;

    /** @brief Current [A] to wire torque units (N*m, or A without Kt). */
    float amps_to_torque(float amps) const;

    /** @brief Duration of a point-to-point move under the velocity and acceleration given. */
    static uint32_t scurve_duration_us(float distance_rev, float vel, float acc);

    /** @brief Node time [us]: micros() re-based by the last bus sync. */
    uint32_t node_time_us() const;

    // ==================================================================
    // System (engine/tp_engine_system_impl.h)
    // ==================================================================

    /** @brief Energize or release the output stage with the protocol's guards. */
    TpError set_enable(bool on);

    /** @brief Start the calibration routines named by TP_CAL_* bits. */
    TpError start_calibration(uint8_t bits);

    /** @brief Advance a multi-step calibration: the angle sweep after the identification. */
    void service_calibration();

    /** @brief TP_CAL_* bits of the routines currently running. */
    uint8_t calibration_running() const;

    /** @brief TP_CAL_* bits of the results currently valid. */
    uint8_t calibration_valid() const;

    /** @brief Handle a 0xCF Save Config command. */
    TpError handle_save(const TpFrame& f);

    /** @brief Handle a 0xD0 MIDI Control command. */
    TpError handle_midi(const TpFrame& f);

    /** @brief Advance the MIDI envelope and end the note on time. */
    void service_midi(uint32_t now);

    /**
     * @brief Adopt a node address: rebind the filters and persist it.
     * @param id      New address (TP_NODE_UNASSIGNED to return to discovery).
     * @param persist true to write it into the configuration partition.
     */
    void adopt_node_id(uint8_t id, bool persist);

    /** @brief Energize the drive by itself once it becomes ready, if configured. */
    void service_auto_enable();

    /** @brief Arm a deferred reboot so the reply can still leave the node. */
    void arm_reboot(uint8_t mode);

    /** @brief Clear latched faults in the driver layer. */
    void clear_faults();

    /** @brief Drive the pending configuration save and time it out. */
    void service_store(uint32_t now);

    // ==================================================================
    // Telemetry (engine/tp_engine_telemetry_impl.h)
    // ==================================================================

    /** @brief Emit the PulseBeat and due telemetry slots. */
    void service_telemetry(uint32_t now);

    /** @brief Emit every sync-triggered telemetry slot and re-schedule the beat. */
    void on_sync(uint32_t now);

    /** @brief Emit an emergency frame whenever the fault byte changes. */
    void emit_emergency_on_change();

    /** @brief Pack and transmit one register on the telemetry identifier. */
    bool emit_register(uint8_t reg);

    // ==================================================================
    // State derivation (engine/tp_engine_status_impl.h)
    // ==================================================================

    /** @brief Collapse the firmware's lifecycle into one TpDriveState value. */
    uint8_t drive_state() const;

    /** @brief The active control mode as a wire value. */
    uint8_t control_mode() const;

    /** @brief Assemble the warning byte from every subsystem that can raise one. */
    uint8_t warn_flags() const;

    /** @brief Motor temperature estimate [degC] from the thermal model (NaN without one). */
    float motor_temperature() const;

    /** @brief Consolidated usage of the current / power limits as a signed fraction. */
    float usage_ratio() const;

    /** @brief Motor load fraction (applied current against the envelope), signed. */
    float load_ratio() const;

    /** @brief Normalized following error against the spring width. */
    float following_error_ratio() const;

    /** @brief true when the output stage is currently energized by command. */
    bool is_enabled() const;

    /** @brief Reload the control loop's master-liveness watchdog. */
    void feed_watchdog();

    /** @brief Read a little-endian 32-bit field out of a frame at a byte offset. */
    static uint32_t read_u32(const TpFrame& f, uint32_t off) {
        return (uint32_t)f.b8[off] | ((uint32_t)f.b8[off + 1u] << 8) |
               ((uint32_t)f.b8[off + 2u] << 16) | ((uint32_t)f.b8[off + 3u] << 24);
    }

    /** @brief Verify at bring-up that the register map is sorted. */
    void verify_register_table() const;

    // ==================================================================
    // State
    // ==================================================================

    TunePulseHeart*   tp_ = nullptr;            ///< Live firmware state.
    TpFrameRead       read_ = nullptr;          ///< Transport RX callback.
    TpFrameWrite      write_ = nullptr;         ///< Transport TX callback.
    TpBindFilters     bind_filters_ = nullptr;  ///< Transport filter rebind callback.
    TpRebootFn        reboot_ = nullptr;        ///< Reset / DFU entry.
    TpEncoderLinkProbe encoder_link_ = nullptr; ///< Board probe behind diagnostics 0x43/0x07.

    TpNodeConfig   config_;     ///< Live configuration shadow.
    TpNodeIdentity identity_;   ///< Immutable device identity.
    TpNodeStore    store_;      ///< Persistent configuration store.

    // ---- Telemetry ---------------------------------------------------
    uint32_t hb_next_ms_ = 0;                         ///< Next PulseBeat due time.
    uint32_t tlm_next_ms_[TP_TELEMETRY_SLOTS] = {0};  ///< Next due time per telemetry slot.
    uint8_t  hb_seq_ = 0;                             ///< PulseBeat sequence counter.
    uint8_t  hb_failed_ = 0;                          ///< Consecutive beats the transport refused.
    uint8_t  last_fault_ = 0;                         ///< Fault byte at the previous emergency check.

    // ---- Counters ----------------------------------------------------
    uint16_t rejected_ = 0;      ///< Frames answered with a NACK.
    uint16_t handled_ = 0;       ///< Frames decoded and executed.
    uint8_t  bus_tx_err_ = 0;    ///< Transport transmit error counter.
    uint8_t  bus_rx_err_ = 0;    ///< Transport receive error counter.
    uint8_t  bus_flags_ = 0;     ///< Transport warning / error-passive / bus-off / gateway bits.
    uint16_t bus_dropped_ = 0;   ///< Frames lost to transport ring overflow.

    // ---- Batch and range reads ---------------------------------------
    uint8_t  batch_bits_[BATCH_BITMAP_BYTES] = {0};  ///< Pending batch reads, one bit per CMD.
    bool     batch_pending_ = false;                 ///< Any bit set in batch_bits_.
    uint8_t  batch_sub_ = 0;                         ///< Record cursor inside the sub-space being emitted.
    bool     range_active_ = false;                  ///< A range walk is in progress.
    uint8_t  range_cmd_ = 0;                         ///< Register being walked.
    uint32_t range_cursor_ = 0;                      ///< Next sub / address / sample.
    uint32_t range_end_ = 0;                         ///< Last sub / address / sample (inclusive).

    // ---- Motion task assembly ----------------------------------------
    uint8_t  seg_id_ = 0;         ///< SegmentID being assembled (0 = none).
    uint8_t  seg_axis_ = 0;       ///< AxisID of the segment being assembled.
    uint8_t  seg_parts_ = 0;      ///< SEG_PART_* bits received so far.
    uint16_t seg_flags_ = 0;      ///< Motion Task 1 flags word.
    uint32_t seg_time_ = 0;       ///< Motion Task 1 time [100 us].
    int32_t  seg_pos_q16_ = 0;    ///< Motion Task 2 position, q1.15.16.
    uint16_t seg_torque_raw_ = TP_BF16_NAN;  ///< Motion Task 3 torque, bfloat16.
    uint32_t seg_vel_raw_ = 0;    ///< Motion Task 3 velocity, float24.
    uint16_t seg_acc_raw_ = 0;    ///< Motion Task 4 acceleration, bfloat16.
    uint16_t seg_jerk_raw_ = 0;   ///< Motion Task 4 jerk, bfloat16.
    uint8_t  seg_io_pos_ = 0;     ///< Motion Task 4 positive IO mask.
    uint8_t  seg_io_neg_ = 0;     ///< Motion Task 4 negative IO mask.
    uint8_t  seg_dropped_ = 0;    ///< Segments discarded incomplete (saturating).

    // ---- Burst stream --------------------------------------------------
    int32_t  burst_ref_q16_ = 0;      ///< Absolute position the masked field is resolved against.
    uint32_t burst_last_t21_ = 0;     ///< Time tag of the previous PVT point.
    uint8_t  burst_last_frame_ = 0;   ///< Frame counter of the previous PVcTF point.
    bool     burst_primed_ = false;   ///< A previous point exists to measure the duration from.
    uint32_t burst_last_ms_ = 0;      ///< When the previous burst frame arrived.
    int32_t  sync_offset_us_ = 0;     ///< Node time = micros() + this, re-based by Bus Settings.

    // ---- MIDI -----------------------------------------------------------
    bool     midi_active_ = false;  ///< A timed note is playing.
    uint32_t midi_start_ms_ = 0;    ///< When the note started.
    uint32_t midi_end_ms_ = 0;      ///< When the note ends (0 = never).
    uint32_t midi_attack_ms_ = 0;   ///< Attack length.
    uint32_t midi_decay_ms_ = 0;    ///< Decay length.
    uint16_t midi_freq_ = 0;        ///< Note frequency [Hz].
    uint16_t midi_amp_ = 0;         ///< Peak amplitude, Q15.
    uint16_t midi_last_amp_ = 0;    ///< Amplitude last posted (avoids mailbox churn).

    // ---- Calibration and lifecycle -------------------------------------
    uint8_t  cal_pending_ = 0;         ///< TP_CAL_* routines still to start (sequenced).
    bool     auto_enable_done_ = false;  ///< The one-shot auto-enable has already fired.
    uint32_t save_deadline_ms_ = 0;    ///< Deadline for a save waiting on the flash window.
    bool     reboot_pending_ = false;  ///< A reboot is armed.
    uint8_t  reboot_mode_ = 0;         ///< 0 = warm reset, 1 = USB DFU.
    uint32_t reboot_at_ms_ = 0;        ///< When the armed reboot fires.
};

// ============================================================================
// Method definitions
// ============================================================================
//
// TpProtocol is one class, but its methods split cleanly into eight concerns,
// so the bodies live in engine/ as out-of-class inline definitions. The
// declarations above are the index; each header below defines one group.

#include "engine/tp_engine_lifecycle_impl.h"  // attach, bind_platform, begin, poll
#include "engine/tp_engine_dispatch_impl.h"   // Identifier routing, CMD groups, batch reads, replies
#include "engine/tp_engine_regread_impl.h"    // Packing every readable register
#include "engine/tp_engine_regwrite_impl.h"   // Applying every writable register
#include "engine/tp_engine_motion_impl.h"     // Motion tasks, setpoints, burst stream
#include "engine/tp_engine_system_impl.h"     // Enable, calibration, save, MIDI, node id, reboot
#include "engine/tp_engine_telemetry_impl.h"  // PulseBeat, telemetry slots, emergency frames
#include "engine/tp_engine_status_impl.h"     // Drive state / warnings / normalized values
