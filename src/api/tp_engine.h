#pragma once

/**
 * @file tp_engine.h
 * @brief TunePulse CAN Protocol engine: dispatch, telemetry, safety, discovery.
 *
 * The engine is the whole protocol above the wire format. It owns:
 *
 * - **Dispatch** - decodes one received frame, routes it by identifier
 *   (network management / setpoint stream / request channel) and by opcode,
 *   executes it against the firmware and answers on the response channel.
 * - **Telemetry** - a heartbeat plus four independently configurable cyclic
 *   producers, each able to run on a timer or on the SYNC broadcast.
 * - **Safety** - feeds the control loop's master-liveness watchdog on every
 *   addressed frame and emits an emergency frame the instant the fault byte
 *   changes.
 * - **Discovery and identity** - answers the UID search, adopts an assigned
 *   address, reprograms the acceptance filters and persists the result.
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
 * The engine never names a peripheral. It moves frames through three function
 * pointers (read one frame, write one frame, write an identifier-only frame)
 * and asks the transport to rebind its filters through a fourth. tp_can.h binds
 * those to FDCAN1; a USB CDC or RS-485 binding would implement the same four.
 *
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#include "../lib/dev_log.h"      // Non-blocking diagnostic logging
#include "../lib/lib.h"          // TunePulseHeart
#include "tp_chunks.h"           // Frame layouts
#include "tp_node.h"             // Identity, configuration, store, discovery
#include "tp_registers.h"        // Register map and configuration application
#include "tp_wire.h"             // Identifier map, opcodes, codecs

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
 * @brief Transmit an identifier-only frame (DLC 0), used by discovery arbitration.
 * @param id Identifier to transmit.
 * @return true when the frame was queued.
 */
typedef bool (*TpFrameWriteEmpty)(uint16_t id);

/**
 * @brief Rebind the transport's acceptance filters to a node address.
 * @param node_id     Address to accept traffic for (0 = network management only).
 * @param promiscuous true to additionally accept every identifier.
 */
typedef void (*TpBindFilters)(uint8_t node_id, bool promiscuous);

/** @brief Reboot the device; mode 1 jumps into the USB DFU bootloader. */
typedef void (*TpRebootFn)(uint8_t mode);

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
     * @param wr_id   Identifier-only write callback (discovery).
     * @param filters Acceptance-filter rebind callback.
     */
    void attach(TunePulseHeart* tp, TpFrameRead rd, TpFrameWrite wr, TpFrameWriteEmpty wr_id, TpBindFilters filters);

    /**
     * @brief Bind the configuration partition and the reboot hook.
     * @param cfg_flash FlashEEPROM instance wired to the configuration partition.
     * @param reboot    Reboot / DFU entry function.
     */
    void bind_platform(FlashEEPROM* cfg_flash, TpRebootFn reboot);

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
     * MAX_FRAMES_PER_POLL frames are decoded, at most one flash operation is
     * started, and the telemetry scheduler emits at most one frame per slot.
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

    /** @brief Requests decoded and executed since boot (saturating). */
    uint16_t handled() const { return handled_; }

    /** @brief Publish the transport's error counters for the COMM_STATUS chunk. */
    void set_bus_counters(uint8_t tx_err, uint8_t rx_err, uint8_t bus_flags, uint16_t dropped) {
        bus_tx_err_ = tx_err;      // Hardware transmit error counter
        bus_rx_err_ = rx_err;      // Hardware receive error counter
        bus_flags_ = bus_flags;    // Warning / error-passive / bus-off bits
        bus_dropped_ = dropped;    // Frames lost to RX ring overflow
    }

private:
    /** Frames decoded per poll() call, so one burst cannot monopolize loop(). */
    static constexpr uint32_t MAX_FRAMES_PER_POLL = 16u;

    /** Delay between arming a reboot and executing it, so the reply gets out. */
    static constexpr uint32_t REBOOT_DELAY_MS = 20u;

    /** How long a pending save may wait for a de-energized programming window. */
    static constexpr uint32_t SAVE_TIMEOUT_MS = 2000u;

    // ==================================================================
    // Dispatch
    // ==================================================================

    /**
     * @brief Route one received frame by identifier.
     * @param id  Identifier the frame arrived with.
     * @param f   Frame payload (zero-padded to 8 bytes).
     * @param len Real payload length.
     */
    void dispatch(uint16_t id, const TpFrame& f, uint8_t len);

    /**
     * @brief Handle one network-management broadcast.
     * @param id  Network-management identifier.
     * @param f   Frame payload.
     * @param len Real payload length.
     */
    void handle_nmt(uint16_t id, const TpFrame& f, uint8_t len);

    /**
     * @brief Execute one broadcast global-control command.
     * @param f Frame payload; byte 0 is the TpGlobalCmd.
     */
    void handle_global(const TpFrame& f);

    /**
     * @brief Handle one frame from the request channel and answer it.
     * @param f         Frame payload.
     * @param broadcast true when the frame was addressed to every node.
     */
    void handle_request(const TpFrame& f, bool broadcast);

    // ==================================================================
    // Real-time control
    // ==================================================================

    /**
     * @brief Execute one control frame.
     *
     * Shared by the request channel and the fire-and-forget setpoint stream, so
     * a command behaves identically whichever channel carried it.
     *
     * @param f       Frame payload.
     * @param request true when the caller will answer; false on the setpoint stream.
     * @return TP_ERR_NONE on success, otherwise the reason for rejection.
     */
    TpError handle_control(const TpFrame& f, bool request);

    /**
     * @brief Queue one bit-packed PVT / PVcTF segment.
     *
     * PVT: the trailing field is a duration in 100 us units and the segment is
     * a time-parameterized Hermite spline - the classic streaming trajectory.
     * PVcTF: the trailing field is a normalized force limit and the segment
     * runs open loop until its displacement is reached, which is how a force
     * controlled move ends on contact instead of on the clock.
     *
     * @param f Frame payload.
     * @return TP_ERR_NONE on success, otherwise the reason for rejection.
     */
    TpError queue_burst(const TpFrame& f);

    /**
     * @brief Queue a jerk-limited point-to-point move.
     *
     * The ramp shape comes from the configured motion envelope (TP_OP_MOTION_LIMITS),
     * so the frame only has to carry where to go and how fast to cruise. The
     * profile is emitted as a Hermite segment whose duration is derived from
     * the trapezoidal time for the requested displacement, which the executor
     * then interpolates with continuous velocity at both ends.
     *
     * @param f Frame payload.
     * @return TP_ERR_NONE on success, otherwise the reason for rejection.
     */
    TpError queue_scurve(const TpFrame& f);

    /**
     * @brief Apply a queue-control command.
     * @param f Frame payload.
     * @return TP_ERR_NONE on success, otherwise the reason for rejection.
     */
    TpError handle_queue_ctl(const TpFrame& f);

    /**
     * @brief Apply a Motor Properties write (0x09).
     *
     * Every field is "leave unchanged" when zero, which is what makes the chunk
     * usable as a partial update without a separate mask.
     *
     * @param f Frame payload.
     * @return TP_ERR_NONE on success, otherwise the reason for rejection.
     */
    TpError apply_motor_properties(const TpFrame& f);

    // ==================================================================
    // Registers
    // ==================================================================

    /** @brief Build the accessor context for one register operation. */
    TpRegContext reg_context();

    /** @brief Execute a register read and build the reply. */
    void handle_reg_read(const TpFrame& f, TpFrame& tx);

    /** @brief Execute a register write and build the reply. */
    void handle_reg_write(const TpFrame& f, TpFrame& tx);

    /** @brief Answer a register introspection request. */
    void handle_reg_info(const TpFrame& f, TpFrame& tx);

    // ==================================================================
    // Persistence
    // ==================================================================

    /**
     * @brief Execute a save / load / factory-reset command.
     *
     * Saving programs flash, which stalls the CPU for milliseconds with the
     * control ISR frozen and whatever vector is latched still on the bridge.
     * The command is therefore refused unless the drive is currently
     * de-energized - the same invariant every other NVM writer in this
     * firmware obeys.
     *
     * @param f Frame payload.
     * @return TP_ERR_NONE on success, otherwise the reason for rejection.
     */
    TpError handle_persistence(const TpFrame& f);

    // ==================================================================
    // Calibration
    // ==================================================================

    /**
     * @brief Start or abort a calibration routine.
     * @param f Frame payload.
     * @return TP_ERR_NONE on success, otherwise the reason for rejection.
     */
    TpError handle_calibration(const TpFrame& f);

    // ==================================================================
    // System
    // ==================================================================

    /** @brief Read or change the node address. */
    void handle_node_id(const TpFrame& f, TpFrame& tx);

    /**
     * @brief Adopt a node address: rebind the filters and optionally persist it.
     * @param id      New address.
     * @param persist true to write it into the configuration partition.
     */
    void adopt_node_id(uint8_t id, bool persist);

    /** @brief Configure one cyclic telemetry slot, or read it back. */
    void handle_telemetry_cfg(const TpFrame& f, TpFrame& tx);

    /** @brief Configure the master-liveness watchdog, or read it back. */
    void handle_watchdog_cfg(const TpFrame& f, TpFrame& tx);

    /**
     * @brief Energize the drive by itself once it becomes ready, if configured.
     *
     * The drive does NOT energize on its own by default: coming out of a
     * calibration into a live position loop without the master having asked for
     * it is a surprise nobody wants on a bench. A machine that must come back up
     * unattended after a power cycle sets TP_CFG_AUTO_ENABLE (register 0x0003
     * bit 0), and this fires exactly once, as soon as the angle table is valid
     * and no fault is latched.
     */
    void service_auto_enable();

    /** @brief Arm a deferred reboot so the reply can still leave the node. */
    void arm_reboot(uint8_t mode);

    /** @brief Clear latched faults in the driver layer. */
    void clear_faults();

    // ==================================================================
    // Telemetry
    // ==================================================================

    /**
     * @brief Emit due telemetry.
     * @param now Current millisecond timebase.
     */
    void service_telemetry(uint32_t now);

    /** @brief Emit every SYNC-triggered telemetry slot. */
    void on_sync();

    /**
     * @brief Emit an emergency frame whenever the fault byte changes.
     *
     * The emergency identifier is the lowest on the bus, so a drive that trips
     * announces it ahead of every command and every other node's telemetry -
     * which is the entire point of putting faults at 0x000 + node.
     */
    void emit_emergency_on_change();

    /**
     * @brief Pack and transmit one status chunk on the telemetry identifier.
     * @param opcode Status opcode to emit.
     */
    void emit_chunk(uint8_t opcode);

    // ==================================================================
    // Status chunk packing
    // ==================================================================

    /**
     * @brief Fill a frame with one status chunk.
     * @param opcode Status opcode (0x01-0x0F, plus 0x1C / 0x2B / 0x32).
     * @param f      Destination frame.
     * @return true when the opcode names a status chunk this build can produce.
     */
    bool pack_chunk(uint8_t opcode, TpFrame& f);

    /** @brief Fill the heartbeat chunk (also used verbatim as the emergency frame). */
    void pack_heartbeat(TpFrame& f);

    // ==================================================================
    // State derivation
    // ==================================================================

    /** @brief Collapse the firmware's lifecycle into one TpDriveState value. */
    uint8_t drive_state() const;

    /** @brief The active control mode as a wire value. */
    uint8_t control_mode() const;

    /** @brief Assemble the warning byte from every subsystem that can raise one. */
    uint8_t warn_flags() const;

    /** @brief Temperature as a signed byte; -128 marks "no data". */
    uint8_t temp_byte() const;

    /** @brief Normalized load: applied current against the envelope, -128 on overload. */
    uint8_t load_byte() const;

    /**
     * @brief Rough conversion efficiency as a 0-255 byte.
     *
     * Mechanical power is not observed, so efficiency is estimated from the
     * copper loss the identified resistance predicts for the applied current.
     * 255 means "not available", which is what a drive with no identified
     * resistance reports.
     */
    uint8_t efficiency_byte(float p_elec) const;

    /** @brief true when the output stage is currently energized by command. */
    bool is_enabled() const;

    /** @brief Reload the control loop's master-liveness watchdog. */
    void feed_watchdog();

    // ==================================================================
    // Reply helpers
    // ==================================================================

    /** @brief Transmit a reply on this node's response identifier. */
    void respond(const TpFrame& f);

    /** @brief Build a generic acknowledgement. */
    static void ack(TpFrame& f, uint8_t opcode) {
        f.u64 = 0;                     // Reserved bytes transmit as zero
        f.ack.cmd = TP_OP_ACK;         // Generic success reply
        f.ack.opcode = opcode;         // What was executed
        f.ack.error = TP_ERR_NONE;     // No error by definition
    }

    /** @brief Build a generic rejection and count it. */
    void nack(TpFrame& f, uint8_t opcode, TpError err, uint8_t detail);

    /** @brief Read a little-endian 32-bit field out of a frame at a byte offset. */
    static uint32_t read_u32(const TpFrame& f, uint32_t off) {
        return (uint32_t)f.b8[off] | ((uint32_t)f.b8[off + 1u] << 8) |
               ((uint32_t)f.b8[off + 2u] << 16) | ((uint32_t)f.b8[off + 3u] << 24);
    }

    /** @brief Read a little-endian 24-bit field out of a three-byte array. */
    static uint32_t read_u24(const uint8_t* p) {
        return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16);
    }

    /**
     * @brief Verify at bring-up that the register table is sorted.
     *
     * Register lookup is a binary search, which silently returns "unknown
     * register" for entries that sit out of order. A future edit that appends
     * a register in the wrong place would therefore fail in the field and pass
     * every build; this check turns that into one log line at boot.
     */
    void verify_register_table() const;

    // ==================================================================
    // State
    // ==================================================================

    TunePulseHeart*   tp_ = nullptr;            ///< Live firmware state.
    TpFrameRead       read_ = nullptr;          ///< Transport RX callback.
    TpFrameWrite      write_ = nullptr;         ///< Transport TX callback.
    TpFrameWriteEmpty write_id_ = nullptr;      ///< Transport identifier-only TX callback.
    TpBindFilters     bind_filters_ = nullptr;  ///< Transport filter rebind callback.
    TpRebootFn        reboot_ = nullptr;        ///< Reset / DFU entry.

    TpNodeConfig   config_;     ///< Live configuration shadow.
    TpNodeIdentity identity_;   ///< Immutable device identity.
    TpNodeStore    store_;      ///< Persistent configuration store.
    TpDiscovery    discovery_;  ///< UID search participant.

    uint32_t hb_next_ms_ = 0;                         ///< Next heartbeat due time.
    uint32_t tlm_next_ms_[TP_TELEMETRY_SLOTS] = {0};  ///< Next due time per telemetry slot.
    uint8_t  hb_seq_ = 0;                             ///< Heartbeat sequence counter.
    uint8_t  last_fault_ = 0;                         ///< Fault byte at the previous emergency check.
    uint16_t rejected_ = 0;                           ///< Frames answered with a NACK.
    uint16_t handled_ = 0;                            ///< Requests decoded and executed.

    uint8_t  bus_tx_err_ = 0;    ///< Transport transmit error counter.
    uint8_t  bus_rx_err_ = 0;    ///< Transport receive error counter.
    uint8_t  bus_flags_ = 0;     ///< Transport warning / error-passive / bus-off bits.
    uint16_t bus_dropped_ = 0;   ///< Frames lost to transport ring overflow.

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
// TpProtocol is one class, but its ~40 methods split cleanly into nine
// concerns, so the bodies live in engine/ as out-of-class inline definitions.
// The declarations above are the index; each header below defines one group.
// Nothing here changes what the class IS - this is file organization only.

#include "engine/tp_engine_lifecycle_impl.h"  // attach, bind_platform, begin, poll
#include "engine/tp_engine_dispatch_impl.h"   // Identifier routing and the opcode switch
#include "engine/tp_engine_control_impl.h"    // Modes, targets, limits, trajectory queue
#include "engine/tp_engine_registers_impl.h"  // Register read / write / introspection
#include "engine/tp_engine_persist_impl.h"    // Save, load, defaults, calibration
#include "engine/tp_engine_system_impl.h"     // Node id, telemetry cfg, watchdog, reboot
#include "engine/tp_engine_telemetry_impl.h"  // Cyclic telemetry and emergency frames
#include "engine/tp_engine_pack_impl.h"       // Status chunk packing
#include "engine/tp_engine_status_impl.h"     // Drive state / warning / normalized bytes
