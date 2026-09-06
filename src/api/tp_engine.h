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
    void attach(TunePulseHeart* tp, TpFrameRead rd, TpFrameWrite wr, TpFrameWriteEmpty wr_id, TpBindFilters filters) {
        tp_ = tp;             // Firmware state
        read_ = rd;           // RX path
        write_ = wr;          // TX path
        write_id_ = wr_id;    // Discovery TX path
        bind_filters_ = filters;  // Filter control
    }

    /**
     * @brief Bind the configuration partition and the reboot hook.
     * @param cfg_flash FlashEEPROM instance wired to the configuration partition.
     * @param reboot    Reboot / DFU entry function.
     */
    void bind_platform(FlashEEPROM* cfg_flash, TpRebootFn reboot) {
        store_.attach(cfg_flash);  // Persistence backend
        reboot_ = reboot;          // Reset vector
    }

    /**
     * @brief Bring the protocol up: identity, stored configuration, filters.
     *
     * Safe to call before the control loop is running; it performs flash READS
     * only, which are memory-mapped and never stall the CPU.
     */
    void begin() {
        identity_.init();                      // Fold the factory UID down to 56 bits
        config_.set_defaults();                // Start from the compiled-in defaults
        const bool restored = store_.load(config_);  // Overlay the stored record, if any

        if (tp_ != nullptr) {
            tp_apply_config(*tp_, config_);    // Push limits and envelopes into the hardware
        }
        if (bind_filters_ != nullptr) {
            bind_filters_(config_.node_id, (config_.flags & TP_CFG_PROMISCUOUS) != 0u);
        }
        verify_register_table();               // Catch an unsorted map at bring-up, not in the field

        TP_LOG_VALS("TPCP: node/uid/config", (unsigned)config_.node_id,
                    (unsigned long)identity_.guard(), restored ? "stored" : "defaults");
        hb_next_ms_ = millis();                // First heartbeat on the next service pass
    }

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
    void poll() {
        const uint32_t now = millis();  // One timebase read for the whole pass

        // ---- Receive and execute ----------------------------------------
        if (read_ != nullptr) {
            uint16_t id = 0;                            // Identifier of the received frame
            TpFrame f;                                  // Received payload
            uint8_t len = 0;                            // Real payload length
            uint32_t budget = MAX_FRAMES_PER_POLL;      // Bound the work per call
            while ((budget-- > 0u) && read_(id, f, len)) {
                dispatch(id, f, len);
            }
        }

        // ---- Emergency: report a fault the instant it changes -------------
        emit_emergency_on_change();

        // ---- Cyclic telemetry --------------------------------------------
        service_telemetry(now);

        // ---- Persistence -------------------------------------------------
        // Programming is skipped internally unless the control loop reports a
        // de-energized bridge, so this call is cheap while the drive is live.
        if (store_.is_busy()) {
            if (!store_.service()) {
                // Still draining. A save that can never find a programming
                // window (the drive was re-energized before it completed) would
                // otherwise leave the engine busy forever and block every later
                // save, so it is abandoned after a bounded wait and reported
                // through PARAM_STATUS as a failed write.
                if (save_deadline_ms_ == 0u) {
                    save_deadline_ms_ = now + SAVE_TIMEOUT_MS;
                } else if ((int32_t)(now - save_deadline_ms_) >= 0) {
                    store_.abort();
                    save_deadline_ms_ = 0u;
                    TP_LOG("TPCP: configuration save abandoned - no programming window");
                }
            } else {
                save_deadline_ms_ = 0u;      // Finished: arm a fresh deadline next time
            }
        } else {
            save_deadline_ms_ = 0u;          // Idle: nothing to time out
        }

        // ---- Auto-enable -------------------------------------------------
        service_auto_enable();

        // ---- Deferred reboot ---------------------------------------------
        // A reboot is never executed inside the frame handler: the reply has to
        // reach the master first, so the request only arms this and the reset
        // happens once the transmit queue has drained.
        if (reboot_pending_ && (reboot_ != nullptr) && ((int32_t)(now - reboot_at_ms_) >= 0)) {
            reboot_(reboot_mode_);
        }
    }

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
    void dispatch(uint16_t id, const TpFrame& f, uint8_t len) {
        if (tp_is_nmt(id)) {
            handle_nmt(id, f, len);                     // Broadcast network management
            return;
        }
        if (config_.node_id == TP_NODE_UNASSIGNED) {
            return;                                     // No address: answer nothing else
        }
        if (tp_is_setpoint_for(id, config_.node_id)) {
            feed_watchdog();                            // The master is alive
            handle_control(f, false);                   // Fire and forget: never answered
            return;
        }
        if (tp_is_request_for(id, config_.node_id)) {
            feed_watchdog();                            // The master is alive
            if (handled_ != 0xFFFFu) {
                handled_++;                             // Saturating: a wrapped counter hides a storm
            }
            const bool broadcast = (id == (uint16_t)(TP_ID_REQUEST_BASE + TP_NODE_BROADCAST));
            handle_request(f, broadcast);
            return;
        }
        // Anything else reached us through the promiscuous filter: ignore it.
    }

    /**
     * @brief Handle one network-management broadcast.
     * @param id  Network-management identifier.
     * @param f   Frame payload.
     * @param len Real payload length.
     */
    void handle_nmt(uint16_t id, const TpFrame& f, uint8_t len) {
        switch (id) {
            case TP_NMT_DISCOVER_BEGIN:
                // Only nodes without an address take part, so a running system
                // is never disturbed by a discovery sweep on the same bus.
                if (config_.node_id == TP_NODE_UNASSIGNED) {
                    discovery_.begin();
                }
                break;

            case TP_NMT_DISCOVER_PREFIX: {
                if (len < 1u) {
                    break;                              // Prefix length byte missing
                }
                const uint8_t plen = f.b8[0];           // Bytes of UID already resolved
                if (plen > TP_DISCOVER_PREFIX_MAX) {
                    break;                              // Nothing left to disclose
                }
                uint16_t reply = 0;                     // Identifier that carries our next UID byte
                if (discovery_.on_prefix(identity_, &f.b8[1], plen, reply) && (write_id_ != nullptr)) {
                    write_id_(reply);                   // CAN arbitration finds the lowest byte for us
                }
                break;
            }

            case TP_NMT_DISCOVER_ASSIGN: {
                if (len < 8u) {
                    break;                              // Needs the full UID plus the address
                }
                uint8_t assigned = 0;                   // Address the master wants us to take
                if (!discovery_.on_assign(identity_, f.b8, assigned)) {
                    break;                              // Meant for a different drive
                }
                if ((assigned == TP_NODE_UNASSIGNED) || (assigned == TP_NODE_BROADCAST)) {
                    break;                              // Reserved addresses are not assignable
                }
                adopt_node_id(assigned, true);          // Take it, refilter, persist
                TpFrame tx;                             // Confirmation frame
                tx.u64 = 0;
                tx.ack.cmd = TP_OP_ACK;
                tx.ack.opcode = TP_OP_SYS_NODE_ID;
                tx.ack.detail = assigned;
                respond(tx);                            // Answered on our brand-new address
                break;
            }

            case TP_NMT_DISCOVER_END:
                discovery_.end();                       // Back to normal operation
                break;

            case TP_NMT_GLOBAL_CONTROL:
                if (len >= 1u) {
                    handle_global(f);
                }
                break;

            case TP_NMT_SYNC:
                on_sync();                              // Latch and emit synchronized telemetry
                break;

            case TP_NMT_TIME:
            default:
                break;                                  // Informational or reserved
        }
    }

    /**
     * @brief Execute one broadcast global-control command.
     * @param f Frame payload; byte 0 is the TpGlobalCmd.
     */
    void handle_global(const TpFrame& f) {
        if (tp_ == nullptr) {
            return;                                     // Not bound to a drive
        }
        switch (f.b8[0]) {
            case TP_GLOBAL_STOP:
                tp_->drive.post_u32(DriveReq::Stop, TP_STOP_QUICK);   // Clear the queue and hold
                tp_->drive.post_u32(DriveReq::SetEnable, 0u);         // Then de-energize
                break;
            case TP_GLOBAL_DISABLE:
                tp_->drive.post_u32(DriveReq::SetEnable, 0u);         // Output stage off
                break;
            case TP_GLOBAL_ENABLE:
                tp_->drive.post_u32(DriveReq::SetEnable, 1u);         // Output stage on
                break;
            case TP_GLOBAL_FAULT_RESET:
                clear_faults();                                       // Drop latched faults
                break;
            case TP_GLOBAL_HEARTBEAT:
                emit_chunk(TP_OP_HEARTBEAT);                          // Answer an inventory sweep
                break;
            case TP_GLOBAL_REBOOT:
                if (read_u32(f, 4) == TP_REBOOT_KEY) {
                    arm_reboot(0u);                                   // Guarded warm reset
                }
                break;
            default:
                break;                                                // Unknown sub-command
        }
    }

    /**
     * @brief Handle one frame from the request channel and answer it.
     * @param f         Frame payload.
     * @param broadcast true when the frame was addressed to every node.
     */
    void handle_request(const TpFrame& f, bool broadcast) {
        TpFrame tx;                        // Reply under construction
        tx.u64 = 0;                        // Start from a clean frame
        bool reply = true;                 // Cleared by handlers that answer nothing

        // Bit-packed motion segments are legal on the request channel too;
        // byte 0 with bit 7 set can never be an opcode.
        if (tp_burst_is(f.b8)) {
            const TpError e = queue_burst(f);
            // Name the kind of segment in the acknowledgement: the request
            // carried no opcode byte to echo, so this is the only way the
            // master can tell a PVT accept from a PVcTF accept.
            const uint8_t label = ((f.b8[0] & TP_BURST_CONST_T) != 0u) ? (uint8_t)TP_OP_MOTION_PVCTF
                                                                       : (uint8_t)TP_OP_MOTION_PVT;
            if (e != TP_ERR_NONE) {
                nack(tx, label, e, 0u);
            } else {
                ack(tx, label);
            }
            if (!broadcast) {
                respond(tx);
            }
            return;
        }

        const uint8_t op = f.b8[0];        // Opcode being requested

        // Motor Properties is the one opcode that is both a status chunk and a
        // writable one, so it MUST be taken before the read shortcut below -
        // otherwise a write would be silently answered with the unchanged
        // state and never applied.
        if (op == TP_OP_MOTOR_PROPERTIES) {
            const TpError e = apply_motor_properties(f);
            if (e != TP_ERR_NONE) {
                nack(tx, op, e, 0u);
            } else {
                pack_chunk(TP_OP_MOTOR_PROPERTIES, tx);  // Answer with the applied state
            }
            if (!broadcast) {
                respond(tx);
            }
            return;
        }

        // Status chunks: pure reads, always answered, safe to broadcast.
        if (pack_chunk(op, tx)) {
            respond(tx);                   // Even a broadcast read gets an answer:
            return;                        // every node replies on its own identifier.
        }

        switch (op) {
            // ---- Real-time control ---------------------------------------
            case TP_OP_CONTROL_MODE:
            case TP_OP_TARGET_POSITION:
            case TP_OP_TARGET_VELOCITY:
            case TP_OP_TARGET_TORQUE:
            case TP_OP_MOTION_LIMITS:
            case TP_OP_STOP:
            case TP_OP_ANCHOR:
            case TP_OP_MOTION_SCURVE:
            case TP_OP_MOTION_QUEUE_CTL: {
                const TpError e = handle_control(f, true);
                if (e == TP_ERR_NONE) {
                    ack(tx, op);
                } else {
                    nack(tx, op, e, 0u);
                }
                break;
            }

            // ---- Registers -----------------------------------------------
            case TP_OP_REG_READ:
                handle_reg_read(f, tx);
                break;
            case TP_OP_REG_WRITE:
                handle_reg_write(f, tx);
                break;
            case TP_OP_REG_INFO:
                handle_reg_info(f, tx);
                break;

            // ---- Persistence ---------------------------------------------
            case TP_OP_PARAM_SAVE:
            case TP_OP_PARAM_LOAD:
            case TP_OP_PARAM_DEFAULTS: {
                const TpError e = handle_persistence(f);
                if (e == TP_ERR_NONE) {
                    ack(tx, op);
                } else {
                    nack(tx, op, e, 0u);
                }
                break;
            }

            // ---- Calibration ---------------------------------------------
            case TP_OP_CALIB_START:
            case TP_OP_CALIB_ABORT: {
                const TpError e = handle_calibration(f);
                if (e == TP_ERR_NONE) {
                    ack(tx, op);
                } else {
                    nack(tx, op, e, 0u);
                }
                break;
            }

            // ---- System --------------------------------------------------
            case TP_OP_SYS_ENABLE:
                if (tp_ == nullptr) {
                    nack(tx, op, TP_ERR_WRONG_STATE, 0u);
                } else if ((f.sys.arg != 0u) && !tp_->angle_calibrator.is_calibrated()) {
                    // Energizing without an angle table would command an
                    // arbitrary electrical angle: refuse rather than jerk.
                    nack(tx, op, TP_ERR_NOT_CALIBRATED, 0u);
                } else if ((f.sys.arg != 0u) && (tp_->driver_phy.get_fault() != 0u)) {
                    nack(tx, op, TP_ERR_FAULT_ACTIVE, tp_->driver_phy.get_fault());
                } else {
                    tp_->drive.post_u32(DriveReq::SetEnable, (f.sys.arg != 0u) ? 1u : 0u);
                    ack(tx, op);
                }
                break;
            case TP_OP_SYS_FAULT_RESET:
                clear_faults();
                ack(tx, op);
                break;
            case TP_OP_SYS_REBOOT:
                if (f.reboot.key != TP_REBOOT_KEY) {
                    nack(tx, op, TP_ERR_ACCESS_DENIED, 0u);
                } else {
                    arm_reboot(f.reboot.mode);
                    ack(tx, op);
                }
                break;
            case TP_OP_SYS_NODE_ID:
                handle_node_id(f, tx);
                break;
            case TP_OP_SYS_TELEMETRY:
                handle_telemetry_cfg(f, tx);
                break;
            case TP_OP_SYS_WATCHDOG:
                handle_watchdog_cfg(f, tx);
                break;

            // ---- Reserved for bulk transfer / firmware update -------------
            case 0x40u:
            case 0x41u:
            case 0x42u:
                nack(tx, op, TP_ERR_NOT_IMPLEMENTED, 0u);
                break;

            case TP_OP_NOP:
                reply = false;                       // A zero frame is padding, not a command
                break;

            default:
                nack(tx, op, TP_ERR_UNKNOWN_OPCODE, 0u);
                break;
        }

        // A broadcast write is executed by every node but answered by none:
        // 250 simultaneous acknowledgements would be a burst, not information.
        if (reply && !broadcast) {
            respond(tx);
        }
    }

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
    TpError handle_control(const TpFrame& f, bool request) {
        (void)request;                                  // Behaviour is identical on both channels
        if (tp_ == nullptr) {
            return TP_ERR_WRONG_STATE;                  // Not bound to a drive
        }
        if (tp_burst_is(f.b8)) {
            return queue_burst(f);                      // Bit-packed PVT / PVcTF segment
        }

        switch (f.b8[0]) {
            case TP_OP_CONTROL_MODE: {
                if (f.control_mode.mode > (uint8_t)TP_MODE_MOTION) {
                    return TP_ERR_BAD_PARAM;            // Undefined mode
                }
                if ((f.control_mode.enable != 0u) && !tp_->angle_calibrator.is_calibrated()) {
                    return TP_ERR_NOT_CALIBRATED;       // No angle table: refuse to energize
                }
                if (!tp_->drive.post_u32(DriveReq::SetMode, f.control_mode.mode)) {
                    return TP_ERR_BUSY;                 // Mailbox full
                }
                if (f.control_mode.enable != 0u) {
                    tp_->drive.post_u32(DriveReq::SetEnable, 1u);
                }
                return TP_ERR_NONE;
            }

            case TP_OP_TARGET_POSITION: {
                const int64_t pos = tp_unpack_i48(f.target_position.pos);  // Q16.16 revolutions
                const DriveReq op = ((f.target_position.flags & TP_POS_FLAG_RELATIVE) != 0u)
                                        ? DriveReq::TargetPosRel
                                        : DriveReq::TargetPosAbs;
                if ((f.target_position.flags & TP_POS_FLAG_SET_MODE) != 0u) {
                    tp_->drive.post_u32(DriveReq::SetMode, TP_MODE_POSITION);
                }
                return tp_->drive.post_i64(op, pos) ? TP_ERR_NONE : TP_ERR_BUSY;
            }

            case TP_OP_TARGET_VELOCITY: {
                const float vel = tp_unpack_f24(read_u24(f.target_velocity.velocity));  // [RPS]
                if (vel != vel) {
                    return TP_ERR_BAD_PARAM;            // NaN fails the self-compare
                }
                if (f.target_velocity.accel != 0u) {
                    const float acc = tp_unpack_bf16(f.target_velocity.accel);
                    if (acc > 0.0f) {
                        config_.accel_limit = acc;      // Keep the shadow in step
                        tp_->drive.post_f32(DriveReq::AccelLimit, acc);
                    }
                }
                if ((f.target_velocity.flags & TP_TGT_FLAG_SET_MODE) != 0u) {
                    tp_->drive.post_u32(DriveReq::SetMode, TP_MODE_VELOCITY);
                }
                return tp_->drive.post_f32(DriveReq::TargetVel, vel) ? TP_ERR_NONE : TP_ERR_BUSY;
            }

            case TP_OP_TARGET_TORQUE: {
                const float cur = tp_unpack_bf16(f.target_torque.current);  // [A]
                if (cur != cur) {
                    return TP_ERR_BAD_PARAM;            // NaN fails the self-compare
                }
                if (f.target_torque.limit != 0u) {
                    const float lim = tp_unpack_bf16(f.target_torque.limit);
                    if (lim > 0.0f) {
                        config_.current_limit_a = lim;  // Keep the shadow in step
                        tp_->drive.post_f32(DriveReq::CurrentLimit, lim);
                    }
                }
                if ((f.target_torque.flags & TP_TGT_FLAG_SET_MODE) != 0u) {
                    tp_->drive.post_u32(DriveReq::SetMode, TP_MODE_TORQUE);
                }
                return tp_->drive.post_f32(DriveReq::TargetCurrent, cur) ? TP_ERR_NONE : TP_ERR_BUSY;
            }

            case TP_OP_MOTION_LIMITS: {
                const float vel = tp_unpack_f24(read_u24(f.motion_limits.velocity));  // [RPS]
                const float acc = tp_unpack_bf16(f.motion_limits.accel);              // [RPS^2]
                const float jrk = tp_unpack_bf16(f.motion_limits.jerk);               // [RPS^3]
                if (vel > 0.0f) {
                    config_.vel_limit_rps = vel;
                    tp_->drive.post_f32(DriveReq::VelocityLimit, vel);
                }
                if (acc > 0.0f) {
                    config_.accel_limit = acc;
                    tp_->drive.post_f32(DriveReq::AccelLimit, acc);
                }
                if (jrk > 0.0f) {
                    config_.jerk_limit = jrk;
                    tp_->drive.post_f32(DriveReq::JerkLimit, jrk);
                }
                return TP_ERR_NONE;
            }

            case TP_OP_STOP:
                if (f.stop.mode > TP_STOP_QUICK) {
                    return TP_ERR_BAD_PARAM;            // Undefined stop style
                }
                return tp_->drive.post_u32(DriveReq::Stop, f.stop.mode) ? TP_ERR_NONE : TP_ERR_BUSY;

            case TP_OP_ANCHOR: {
                const int64_t pos = tp_unpack_i48(f.anchor.pos);  // New value of "here"
                return tp_->drive.post_i64(DriveReq::Anchor, pos) ? TP_ERR_NONE : TP_ERR_BUSY;
            }

            case TP_OP_MOTION_SCURVE:
                return queue_scurve(f);

            case TP_OP_MOTION_QUEUE_CTL:
                return handle_queue_ctl(f);

            default:
                return TP_ERR_UNKNOWN_OPCODE;
        }
    }

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
    TpError queue_burst(const TpFrame& f) {
        MotionCommandTM seg;                                    // Segment under construction
        seg.motion.displacement = tp_burst_disp(f.b8);          // Q16.16 revolutions
        seg.motion.end_velocity = tp_burst_vel(f.b8);           // [RPS]
        if (seg.motion.end_velocity != seg.motion.end_velocity) {
            return TP_ERR_BAD_PARAM;                            // NaN fails the self-compare
        }
        const uint16_t tail = tp_burst_tail(f.b8);              // Duration or force
        if ((f.b8[0] & TP_BURST_CONST_T) != 0u) {
            // Force-controlled: run open loop until the displacement is
            // reached, with the trailing field capping the torque.
            seg.motion.duration = 0u;
            seg.torque = tail;
            seg.mode = MotionConfig(MotionMode::FREERUN, TorqueTrigger::DISABLED);
        } else {
            // Time-parameterized spline: 100 us units on the wire keep a
            // 16-bit field useful out to 6.5 seconds per segment.
            seg.motion.duration = (uint32_t)tail * 100u;
            seg.torque = UINT16_MAX;                            // No per-segment torque cap
            seg.mode = MotionConfig(MotionMode::HERMITE, TorqueTrigger::DISABLED);
        }
        return tp_->drive.post_segment(seg) ? TP_ERR_NONE : TP_ERR_QUEUE_FULL;
    }

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
    TpError queue_scurve(const TpFrame& f) {
        const int32_t disp_q16 = tp_unpack_i24(f.scurve.target);  // Q16.16 revolutions
        float vel = (f.scurve.velocity != 0u) ? tp_unpack_bf16(f.scurve.velocity)
                                              : config_.vel_limit_rps;
        if ((vel != vel) || (vel <= 0.0f)) {
            return TP_ERR_BAD_PARAM;                             // NaN or non-positive cruise speed
        }
        if (vel > config_.vel_limit_rps) {
            vel = config_.vel_limit_rps;                         // Clamp into the envelope
        }
        if ((f.scurve.flags & TP_SCURVE_RELATIVE) == 0u) {
            // Absolute targets are only meaningful against the live position,
            // which the ISR owns; the queue works in displacements, so an
            // absolute move is not expressible as a queued segment.
            return TP_ERR_BAD_PARAM;
        }
        const float rev = (float)disp_q16 * (1.0f / 65536.0f);   // Displacement in revolutions
        const float mag = (rev < 0.0f) ? -rev : rev;             // Distance to travel
        if (mag <= 0.0f) {
            return TP_ERR_BAD_PARAM;                             // A zero move is not a move
        }
        // Trapezoidal time estimate: cruise time plus one full acceleration
        // ramp. The executor's Hermite interpolation supplies the smooth ends,
        // so this only has to be the right duration, not the exact profile.
        const float acc = (config_.accel_limit > 0.0f) ? config_.accel_limit : 1.0f;
        const float t_ramp = vel / acc;                          // Time to reach cruise speed
        const float t_total = (mag / vel) + t_ramp;              // Cruise plus both half ramps
        uint32_t dur_us = (uint32_t)(t_total * 1000000.0f);      // Executor works in microseconds
        if (dur_us < 1000u) {
            dur_us = 1000u;                                      // Never shorter than 1 ms
        }

        MotionCommandTM seg;                                     // Segment under construction
        seg.motion.displacement = disp_q16;                      // Where to go, relative
        seg.motion.duration = dur_us;                            // How long it may take
        seg.motion.end_velocity = 0.0f;                          // Point-to-point ends at rest
        seg.torque = UINT16_MAX;                                 // No per-segment torque cap
        seg.mode = MotionConfig(MotionMode::HERMITE, TorqueTrigger::DISABLED);
        return tp_->drive.post_segment(seg) ? TP_ERR_NONE : TP_ERR_QUEUE_FULL;
    }

    /**
     * @brief Apply a queue-control command.
     * @param f Frame payload.
     * @return TP_ERR_NONE on success, otherwise the reason for rejection.
     */
    TpError handle_queue_ctl(const TpFrame& f) {
        switch (f.queue_ctl.action) {
            case TP_QUEUE_NOP:
                return TP_ERR_NONE;                              // Nothing requested
            case TP_QUEUE_CLEAR:
                return tp_->drive.post(DriveReq::QueueClear) ? TP_ERR_NONE : TP_ERR_BUSY;
            case TP_QUEUE_PAUSE:
                return tp_->drive.post_u32(DriveReq::QueuePause, 1u) ? TP_ERR_NONE : TP_ERR_BUSY;
            case TP_QUEUE_RESUME:
                return tp_->drive.post_u32(DriveReq::QueuePause, 0u) ? TP_ERR_NONE : TP_ERR_BUSY;
            case TP_QUEUE_TIME_SCALE:
                return tp_->drive.post_i32(DriveReq::TimeScale, f.queue_ctl.time_scale) ? TP_ERR_NONE
                                                                                        : TP_ERR_BUSY;
            default:
                return TP_ERR_BAD_PARAM;                         // Undefined action
        }
    }

    /**
     * @brief Apply a Motor Properties write (0x09).
     *
     * Every field is "leave unchanged" when zero, which is what makes the chunk
     * usable as a partial update without a separate mask.
     *
     * @param f Frame payload.
     * @return TP_ERR_NONE on success, otherwise the reason for rejection.
     */
    TpError apply_motor_properties(const TpFrame& f) {
        if (tp_ == nullptr) {
            return TP_ERR_WRONG_STATE;                           // Not bound to a drive
        }
        // A read is a frame with an all-zero payload: nothing to apply.
        if ((f.motor_props.temp_lim == 0u) && (f.motor_props.resistance == 0u) &&
            (f.motor_props.inductance == 0u) && (f.motor_props.torque == 0u)) {
            return TP_ERR_NONE;
        }
        if (is_enabled()) {
            return TP_ERR_WRONG_STATE;                           // Never retune a live loop
        }
        if (f.motor_props.temp_lim != 0u) {
            tp_->motor_params.set_max_temp(f.motor_props.temp_lim);
        }
        if (f.motor_props.resistance != 0u) {
            const float r = tp_unpack_bf16(f.motor_props.resistance);
            if (!tp_reg_positive(r)) {
                return TP_ERR_OUT_OF_RANGE;
            }
            tp_->motor_params.set_resistance(r);
        }
        if (f.motor_props.inductance != 0u) {
            const float l = tp_unpack_bf16(f.motor_props.inductance);
            if (!tp_reg_positive(l)) {
                return TP_ERR_OUT_OF_RANGE;
            }
            tp_->motor_params.set_inductance_d(l);               // Symmetric machine assumption
            tp_->motor_params.set_inductance_q(l);
        }
        if (f.motor_props.torque != 0u) {
            const float kt = tp_unpack_bf16(f.motor_props.torque);
            if (!tp_reg_positive(kt)) {
                return TP_ERR_OUT_OF_RANGE;
            }
            tp_->motor_params.set_torque_constant(kt);
        }
        return TP_ERR_NONE;
    }

    // ==================================================================
    // Registers
    // ==================================================================

    /** @brief Build the accessor context for one register operation. */
    TpRegContext reg_context() {
        TpRegContext c;         // Context handed to the accessors
        c.tp = tp_;             // Live firmware state
        c.cfg = &config_;       // Configuration shadow
        c.id = &identity_;      // Device identity
        c.dirty = false;        // No persisted register touched yet
        c.hw_reapply = false;   // No hardware reprogramming needed yet
        return c;
    }

    /** @brief Execute a register read and build the reply. */
    void handle_reg_read(const TpFrame& f, TpFrame& tx) {
        const TpRegDesc* d = tp_find_register(f.reg.reg);        // Look the register up
        if (d == nullptr) {
            nack(tx, TP_OP_REG_READ, TP_ERR_UNKNOWN_REG, (uint8_t)(f.reg.reg & 0xFFu));
            return;
        }
        if (((d->flags & TP_RF_READ) == 0u) || (d->get == nullptr)) {
            nack(tx, TP_OP_REG_READ, TP_ERR_WRITE_ONLY, (uint8_t)(f.reg.reg & 0xFFu));
            return;
        }
        TpRegContext c = reg_context();                          // Accessor context
        uint32_t value = 0;                                      // Read-back value
        if (!d->get(c, value)) {
            nack(tx, TP_OP_REG_READ, TP_ERR_WRONG_STATE, (uint8_t)(f.reg.reg & 0xFFu));
            return;
        }
        tx.reg.cmd = TP_OP_REG_READ;                             // Echo the opcode
        tx.reg.info = (uint8_t)d->type;                          // Let a generic tool decode it
        tx.reg.reg = f.reg.reg;                                  // Echo the identifier
        tx.reg.value = value;                                    // The value itself
    }

    /** @brief Execute a register write and build the reply. */
    void handle_reg_write(const TpFrame& f, TpFrame& tx) {
        const TpRegDesc* d = tp_find_register(f.reg.reg);        // Look the register up
        if (d == nullptr) {
            nack(tx, TP_OP_REG_WRITE, TP_ERR_UNKNOWN_REG, (uint8_t)(f.reg.reg & 0xFFu));
            return;
        }
        if (((d->flags & TP_RF_WRITE) == 0u) || (d->set == nullptr)) {
            nack(tx, TP_OP_REG_WRITE, TP_ERR_READ_ONLY, (uint8_t)(f.reg.reg & 0xFFu));
            return;
        }
        if (((d->flags & TP_RF_DISABLED) != 0u) && is_enabled()) {
            // Retuning a protection threshold or the machine's identity under
            // load moves a trip point beneath a live current: refuse it.
            nack(tx, TP_OP_REG_WRITE, TP_ERR_WRONG_STATE, (uint8_t)(f.reg.reg & 0xFFu));
            return;
        }
        TpRegContext c = reg_context();                          // Accessor context
        if (!d->set(c, tp_reg_normalize(d->type, f.reg.value))) {
            nack(tx, TP_OP_REG_WRITE, TP_ERR_OUT_OF_RANGE, (uint8_t)(f.reg.reg & 0xFFu));
            return;
        }
        if (c.hw_reapply && (tp_ != nullptr)) {
            tp_apply_config(*tp_, config_);                      // Push the change into hardware
        }
        // Answer with the value the drive holds at the moment of the reply, not
        // with the value that was sent, so a clamped or rounded write is
        // visible immediately. Registers that route through the control
        // mailbox (targets, mode, enable) take effect on the next control pass,
        // so their reply can still show the previous value - read the register
        // again to confirm one of those.
        uint32_t applied = f.reg.value;                          // Fallback if there is no getter
        if (d->get != nullptr) {
            (void)d->get(c, applied);
        }
        tx.reg.cmd = TP_OP_REG_WRITE;                            // Echo the opcode
        tx.reg.info = (uint8_t)d->type;                          // Type of the value
        tx.reg.reg = f.reg.reg;                                  // Echo the identifier
        tx.reg.value = applied;                                  // What is in force now
    }

    /** @brief Answer a register introspection request. */
    void handle_reg_info(const TpFrame& f, TpFrame& tx) {
        const TpRegDesc* d = tp_find_register(f.reg.reg);        // Look the register up
        if (d == nullptr) {
            nack(tx, TP_OP_REG_INFO, TP_ERR_UNKNOWN_REG, (uint8_t)(f.reg.reg & 0xFFu));
            return;
        }
        tx.reg.cmd = TP_OP_REG_INFO;                             // Echo the opcode
        tx.reg.info = (uint8_t)d->type;                          // Payload interpretation
        tx.reg.reg = d->id;                                      // Echo the identifier
        // Byte 4 carries the access flags, byte 5 the group (the identifier's
        // high byte), bytes 6-7 the total register count so a tool can tell
        // when it has walked the whole map.
        tx.reg.value = (uint32_t)d->flags | ((uint32_t)(d->id >> 8) << 8) |
                       ((uint32_t)TP_REGISTER_COUNT << 16);
    }

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
    TpError handle_persistence(const TpFrame& f) {
        if (tp_ == nullptr) {
            return TP_ERR_WRONG_STATE;                           // Not bound to a drive
        }
        const uint8_t target = f.param.target;                   // Which store to act on
        switch (f.b8[0]) {
            case TP_OP_PARAM_SAVE: {
                if (store_.is_busy() || tp_->motor_params.is_flash_busy()) {
                    return TP_ERR_BUSY;                          // A transfer is already draining
                }
                if (is_enabled()) {
                    return TP_ERR_WRONG_STATE;                   // Disable the stage before writing flash
                }
                bool armed = false;                              // At least one engine started
                if ((target == TP_PARAM_ALL) || (target == TP_PARAM_CONFIG)) {
                    armed |= store_.begin_save(config_);
                }
                if ((target == TP_PARAM_ALL) || (target == TP_PARAM_MOTOR)) {
                    tp_->motor_params.begin_save();              // Gated internally on full calibration
                    armed = true;
                }
                return armed ? TP_ERR_NONE : TP_ERR_NVM;
            }
            case TP_OP_PARAM_LOAD: {
                if (is_enabled()) {
                    return TP_ERR_WRONG_STATE;                   // Reloading limits under load is unsafe
                }
                if ((target == TP_PARAM_ALL) || (target == TP_PARAM_CONFIG)) {
                    config_.set_defaults();                      // Anything not stored falls back
                    (void)store_.load(config_);                  // Overlay the stored record
                    tp_apply_config(*tp_, config_);              // Push it into the hardware
                }
                if ((target == TP_PARAM_ALL) || (target == TP_PARAM_MOTOR)) {
                    tp_->motor_params.begin_load(MOTOR_ID_ANY);  // Non-blocking rescan
                }
                return TP_ERR_NONE;
            }
            case TP_OP_PARAM_DEFAULTS: {
                if (f.param.key != TP_DEFAULTS_KEY) {
                    return TP_ERR_ACCESS_DENIED;                 // Guarded: this can wipe the node address
                }
                if (is_enabled()) {
                    return TP_ERR_WRONG_STATE;                   // Same flash-window reasoning as above
                }
                if (target == TP_PARAM_ALL) {
                    // Full factory reset: erase the stored record as well, so
                    // the node really does come back on compiled-in defaults
                    // after a reboot. This DOES surrender the node address -
                    // the drive comes back unaddressed and has to be found by
                    // a discovery sweep again - which is exactly what a factory
                    // reset means, and why it needs the key.
                    if (!store_.erase_all()) {
                        return TP_ERR_NVM;                       // No programming window, or no partition
                    }
                    config_.set_defaults();
                    tp_apply_config(*tp_, config_);
                    TP_LOG("TPCP: factory reset - stored configuration erased");
                    return TP_ERR_NONE;
                }
                // Target CONFIG resets the live shadow only: reversible until
                // it is saved, and it keeps the address so the bench does not
                // lose contact with the drive mid-session.
                const uint8_t keep_id = config_.node_id;
                config_.set_defaults();
                config_.node_id = keep_id;
                tp_apply_config(*tp_, config_);
                return TP_ERR_NONE;
            }
            default:
                return TP_ERR_UNKNOWN_OPCODE;
        }
    }

    // ==================================================================
    // Calibration
    // ==================================================================

    /**
     * @brief Start or abort a calibration routine.
     * @param f Frame payload.
     * @return TP_ERR_NONE on success, otherwise the reason for rejection.
     */
    TpError handle_calibration(const TpFrame& f) {
        if (tp_ == nullptr) {
            return TP_ERR_WRONG_STATE;                           // Not bound to a drive
        }
        if (f.b8[0] == TP_OP_CALIB_ABORT) {
            // Neither AngleCalibrator nor MotorCalibration exposes an abort
            // entry point: both are advanced from the control ISR and run to
            // their own terminal state. Pretending to abort by de-energizing
            // would report success while the sweep kept running, so this says
            // plainly that the operation is not available. A reboot
            // (TP_OP_SYS_REBOOT) is the way to stop a calibration.
            return TP_ERR_NOT_IMPLEMENTED;
        }
        if (tp_->driver_phy.get_fault() != 0u) {
            return TP_ERR_FAULT_ACTIVE;                          // Clear faults before moving the rotor
        }
        // Calibration drives the rotor open loop, so it must own the machine:
        // any queued trajectory and any commanded mode is dropped first.
        tp_->drive.post(DriveReq::QueueClear);
        tp_->drive.post_u32(DriveReq::SetMode, TP_MODE_DISABLED);
        tp_->drive.post_u32(DriveReq::SetEnable, 0u);

        switch (f.calib.kind) {
            case TP_CALIB_CURRENT_OFFSET:
                tp_->driver_phy.current.calibrate();             // Re-measure the amplifier offsets
                return TP_ERR_NONE;
            case TP_CALIB_ELECTRICAL:
                tp_->current_driver.setup_status.set_driver_status(OperationStatus::DriverCalibration);
                tp_->current_driver.start_motor_calibration();   // Resistance + inductance identification
                return TP_ERR_NONE;
            case TP_CALIB_ANGLE:
                // FORCE demands a physical sweep. Without it a drive with a
                // stored profile reloads its saved table and the shaft never
                // moves, which looks exactly like a request that did nothing.
                tp_->angle_calibrator.calibrate((f.calib.flags & TP_CALIB_FLAG_FORCE) != 0u);
                return TP_ERR_NONE;
            case TP_CALIB_FULL:
                // Starting both engines at once would run the HFI
                // identification and the angle sweep concurrently on the same
                // bridge, which is not a sequence this firmware implements.
                // The master sequences it instead: start ELECTRICAL, poll
                // CALIBRATION_STATUS (0x0C) until the stage reports success,
                // then start ANGLE.
                return TP_ERR_NOT_IMPLEMENTED;
            default:
                return TP_ERR_BAD_PARAM;                         // Undefined routine
        }
    }

    // ==================================================================
    // System
    // ==================================================================

    /** @brief Read or change the node address. */
    void handle_node_id(const TpFrame& f, TpFrame& tx) {
        // A frame with a zero address byte is a read: the master is asking who
        // it is talking to, which must never be able to change anything.
        if (f.node_id.node_id == 0u) {
            tx.node_id.cmd = TP_OP_SYS_NODE_ID;
            tx.node_id.node_id = config_.node_id;
            tx.node_id.flags = 0u;
            tx.node_id.state = (uint8_t)((store_.is_loaded() ? 1u : 0u) |
                                         (discovery_.is_active() ? 2u : 0u));
            tx.node_id.guard = identity_.guard();
            return;
        }
        if (f.node_id.guard != identity_.guard()) {
            // The guard is the low half of this node's UID: without it a
            // broadcast address change would renumber the whole bus at once.
            nack(tx, TP_OP_SYS_NODE_ID, TP_ERR_ACCESS_DENIED, 0u);
            return;
        }
        if (f.node_id.node_id > TP_NODE_MAX) {
            nack(tx, TP_OP_SYS_NODE_ID, TP_ERR_BAD_PARAM, 0u);
            return;
        }
        const bool persist = (f.node_id.flags & 0x01u) != 0u;    // Store it as well as adopt it
        adopt_node_id(f.node_id.node_id, persist);
        tx.node_id.cmd = TP_OP_SYS_NODE_ID;                      // Answered on the NEW address
        tx.node_id.node_id = config_.node_id;
        tx.node_id.flags = persist ? 0x01u : 0x00u;
        tx.node_id.state = store_.is_loaded() ? 1u : 0u;
        tx.node_id.guard = identity_.guard();
    }

    /**
     * @brief Adopt a node address: rebind the filters and optionally persist it.
     * @param id      New address.
     * @param persist true to write it into the configuration partition.
     */
    void adopt_node_id(uint8_t id, bool persist) {
        config_.node_id = id;                                    // Update the shadow
        if (bind_filters_ != nullptr) {
            // Rebinding first means the confirmation frame is already sent from
            // the new address, so the master sees the change take effect.
            bind_filters_(id, (config_.flags & TP_CFG_PROMISCUOUS) != 0u);
        }
        if (persist && !store_.is_busy()) {
            (void)store_.begin_save(config_);                    // Drains inside the flash window
        }
        TP_LOG_VAL("TPCP: node address adopted", (unsigned)id);
    }

    /** @brief Configure one cyclic telemetry slot, or read it back. */
    void handle_telemetry_cfg(const TpFrame& f, TpFrame& tx) {
        if (f.telemetry.slot >= TP_TELEMETRY_SLOTS) {
            nack(tx, TP_OP_SYS_TELEMETRY, TP_ERR_BAD_PARAM, f.telemetry.slot);
            return;
        }
        TpTelemetrySlot& slot = config_.telemetry[f.telemetry.slot];  // Slot being addressed
        // Bit 7 of the flags byte marks an APPLY. Deriving "is this a write?"
        // from a non-zero body instead would make an all-zero body ambiguous,
        // and an all-zero body is exactly how a slot gets switched off - so a
        // slot could be configured but never disabled again.
        const bool is_write = (f.telemetry.flags & 0x80u) != 0u;
        if (is_write) {
            if (f.telemetry.opcode > TP_OP_HEARTBEAT) {
                nack(tx, TP_OP_SYS_TELEMETRY, TP_ERR_BAD_PARAM, f.telemetry.opcode);
                return;
            }
            slot.opcode = f.telemetry.opcode;                    // What to emit
            slot.flags = (uint8_t)(f.telemetry.flags & 0x7Fu);   // Timer / SYNC / event (apply bit stripped)
            slot.period_ms = f.telemetry.period;                 // How often
            tlm_next_ms_[f.telemetry.slot] = millis();           // Emit on the next pass
        }
        tx.telemetry.cmd = TP_OP_SYS_TELEMETRY;                  // Echo the current configuration
        tx.telemetry.slot = f.telemetry.slot;
        tx.telemetry.opcode = slot.opcode;
        tx.telemetry.flags = slot.flags;
        tx.telemetry.period = slot.period_ms;
    }

    /** @brief Configure the master-liveness watchdog, or read it back. */
    void handle_watchdog_cfg(const TpFrame& f, TpFrame& tx) {
        const bool is_write = (f.watchdog.flags & 0x80u) != 0u;  // Bit 7 marks a write
        if (is_write) {
            if (f.watchdog.action > TP_SAFE_HOLD) {
                nack(tx, TP_OP_SYS_WATCHDOG, TP_ERR_BAD_PARAM, f.watchdog.action);
                return;
            }
            config_.watchdog_ms = ((f.watchdog.flags & 0x01u) != 0u) ? f.watchdog.timeout : 0u;
            config_.watchdog_action = f.watchdog.action;
            if (tp_ != nullptr) {
                // The safe state the watchdog falls into IS the drive's safe
                // state: one policy, so a timeout and a disable behave alike.
                tp_->drive.post_u32(DriveReq::SetSafeState, config_.watchdog_action);
                const uint32_t ticks = (config_.watchdog_ms == 0u)
                                           ? 0u
                                           : ((uint32_t)config_.watchdog_ms * tp_->drive.frequency_hz()) / 1000u;
                tp_->drive.post_u32(DriveReq::ArmWatchdog, ticks);
            }
        }
        uint8_t flags = (config_.watchdog_ms != 0u) ? 0x01u : 0x00u;  // Enabled bit
        if (tp_ != nullptr) {
            DriveTelemetry t;                                    // Live control snapshot
            tp_->drive.read_telemetry(t);
            if ((t.flags & DRIVE_TLM_WATCHDOG) != 0u) {
                flags |= 0x02u;                                  // Currently expired
            }
        }
        tx.watchdog.cmd = TP_OP_SYS_WATCHDOG;                    // Echo the effective configuration
        tx.watchdog.action = config_.watchdog_action;
        tx.watchdog.flags = flags;
        tx.watchdog.timeout = config_.watchdog_ms;
    }

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
    void service_auto_enable() {
        if (auto_enable_done_ || ((config_.flags & TP_CFG_AUTO_ENABLE) == 0u) || (tp_ == nullptr)) {
            return;                                          // Not configured, or already done
        }
        if (!tp_->angle_calibrator.is_calibrated() || (tp_->driver_phy.get_fault() != 0u)) {
            return;                                          // Not ready yet: try again next pass
        }
        auto_enable_done_ = true;                            // One shot for the life of this boot
        tp_->drive.post_u32(DriveReq::SetMode, TP_MODE_POSITION);  // Hold where the calibration left it
        tp_->drive.post_u32(DriveReq::SetEnable, 1u);
        TP_LOG("TPCP: auto-enable armed the output stage");
    }

    /** @brief Arm a deferred reboot so the reply can still leave the node. */
    void arm_reboot(uint8_t mode) {
        reboot_mode_ = mode;                        // 0 = warm reset, 1 = USB DFU
        reboot_at_ms_ = millis() + REBOOT_DELAY_MS; // Let the transmit queue drain first
        reboot_pending_ = true;                     // poll() executes it
        if (tp_ != nullptr) {
            tp_->drive.post_u32(DriveReq::SetEnable, 0u);  // Never reset an energized bridge
        }
    }

    /** @brief Clear latched faults in the driver layer. */
    void clear_faults() {
        if (tp_ == nullptr) {
            return;                                 // Not bound to a drive
        }
        // Every latched source is released individually through the fluent
        // trigger proxy - there is no blanket clear in the driver layer, and
        // reaching into fault_flags_ directly would bypass the per-subsystem
        // recovery timers that decide when a fault may actually be released.
        //
        // Clearing is safe: a condition that is still present re-asserts its
        // bit on the very next control pass, so this cannot mask a real fault.
        tp_->driver_phy.set_fault().overvoltage(false);   // Release the OV latch
        tp_->driver_phy.set_fault().undervoltage(false);  // Release the UV latch
        tp_->driver_phy.set_fault().overcurrent(false);   // Release the OC latch
        tp_->driver_phy.set_fault().overpower(false);     // Release the OP latch
        tp_->driver_phy.set_fault().phase(false);         // Release the phase-stage latch
        tp_->driver_phy.set_fault().overheat(false);      // Release the OT latch
        tp_->driver_phy.set_fault().external(false);      // Release the external-fault latch
    }

    // ==================================================================
    // Telemetry
    // ==================================================================

    /**
     * @brief Emit due telemetry.
     * @param now Current millisecond timebase.
     */
    void service_telemetry(uint32_t now) {
        // Heartbeat first: it is the frame a supervisor keys its own timeout on.
        if (config_.heartbeat_ms != 0u) {
            if ((int32_t)(now - hb_next_ms_) >= 0) {
                hb_next_ms_ = now + config_.heartbeat_ms;   // Schedule the next one
                emit_chunk(TP_OP_HEARTBEAT);
            }
        }
        for (uint32_t i = 0; i < TP_TELEMETRY_SLOTS; ++i) {
            const TpTelemetrySlot& s = config_.telemetry[i];  // Slot under consideration
            if ((s.opcode == 0u) || ((s.flags & TP_TLM_ENABLE) == 0u)) {
                continue;                                    // Slot unused or disabled
            }
            if ((s.flags & TP_TLM_ON_SYNC) != 0u) {
                continue;                                    // Driven by SYNC, not by the timer
            }
            if (s.period_ms == 0u) {
                continue;                                    // No timer configured
            }
            if ((int32_t)(now - tlm_next_ms_[i]) >= 0) {
                tlm_next_ms_[i] = now + s.period_ms;         // Schedule the next emission
                emit_chunk(s.opcode);
            }
        }
    }

    /** @brief Emit every SYNC-triggered telemetry slot. */
    void on_sync() {
        for (uint32_t i = 0; i < TP_TELEMETRY_SLOTS; ++i) {
            const TpTelemetrySlot& s = config_.telemetry[i];  // Slot under consideration
            if ((s.opcode != 0u) && ((s.flags & TP_TLM_ENABLE) != 0u) &&
                ((s.flags & TP_TLM_ON_SYNC) != 0u)) {
                emit_chunk(s.opcode);                        // Synchronized across every node
            }
        }
    }

    /**
     * @brief Emit an emergency frame whenever the fault byte changes.
     *
     * The emergency identifier is the lowest on the bus, so a drive that trips
     * announces it ahead of every command and every other node's telemetry -
     * which is the entire point of putting faults at 0x000 + node.
     */
    void emit_emergency_on_change() {
        if (tp_ == nullptr) {
            return;                                          // Not bound to a drive
        }
        const uint8_t fault = tp_->driver_phy.get_fault();   // Current fault byte
        if (fault == last_fault_) {
            return;                                          // Nothing changed
        }
        last_fault_ = fault;                                 // Latch the new state either way
        if (config_.node_id == TP_NODE_UNASSIGNED) {
            return;                                          // No address: a node must stay silent
        }
        TpFrame f;                                           // Emergency frame
        pack_heartbeat(f);                                   // Same layout as the heartbeat
        if (write_ != nullptr) {
            write_((uint16_t)(TP_ID_EMCY_BASE + config_.node_id), f);
        }
    }

    /**
     * @brief Pack and transmit one status chunk on the telemetry identifier.
     * @param opcode Status opcode to emit.
     */
    void emit_chunk(uint8_t opcode) {
        if (config_.node_id == TP_NODE_UNASSIGNED) {
            return;                                          // No address: a node must stay silent
        }
        TpFrame f;                                           // Frame under construction
        f.u64 = 0;                                           // Start from a clean frame
        if (!pack_chunk(opcode, f) || (write_ == nullptr)) {
            return;                                          // Unknown chunk or no transport
        }
        write_((uint16_t)(TP_ID_TELEMETRY_BASE + config_.node_id), f);
    }

    // ==================================================================
    // Status chunk packing
    // ==================================================================

    /**
     * @brief Fill a frame with one status chunk.
     * @param opcode Status opcode (0x01-0x0F, plus 0x1C / 0x2B / 0x32).
     * @param f      Destination frame.
     * @return true when the opcode names a status chunk this build can produce.
     */
    bool pack_chunk(uint8_t opcode, TpFrame& f) {
        if (tp_ == nullptr) {
            return false;                                    // Not bound to a drive
        }
        f.u64 = 0;                                           // Clean slate for every branch
        switch (opcode) {
            case TP_OP_DEVICE_INFO:
                f.device_info.cmd = TP_OP_DEVICE_INFO;
                f.device_info.hw_ver = TP_HW_VERSION;
                f.device_info.fw_ver = TP_FW_VERSION;
                f.device_info.device = TP_DEVICE_CODE;
                return true;

            case TP_OP_DEVICE_UID: {
                f.device_uid.cmd = TP_OP_DEVICE_UID;
                const uint8_t* uid = identity_.bytes();      // 56-bit identity
                for (uint32_t i = 0; i < TP_UID_BYTES; ++i) {
                    f.device_uid.uid[i] = uid[i];
                }
                return true;
            }

            case TP_OP_DEVICE_STATUS: {
                const uint32_t up = millis() / 1000u;        // Whole seconds since boot
                f.device_status.cmd = TP_OP_DEVICE_STATUS;
                f.device_status.uptime[0] = (uint8_t)(up);
                f.device_status.uptime[1] = (uint8_t)(up >> 8);
                f.device_status.uptime[2] = (uint8_t)(up >> 16);
                f.device_status.status = (uint32_t)tp_->driver_phy.get_fault() << TP_STATUS_FAULT_SHIFT |
                                         (uint32_t)warn_flags() << TP_STATUS_WARN_SHIFT |
                                         (uint32_t)drive_state() << TP_STATUS_STATE_SHIFT |
                                         (uint32_t)control_mode() << TP_STATUS_MODE_SHIFT;
                return true;
            }

            case TP_OP_MOTOR_STATUS: {
                DriveTelemetry t;                            // Live control snapshot
                tp_->drive.read_telemetry(t);
                const MotorType mt = tp_->motor_params.get_motor_type();
                const PhasePattern pp = tp_->motor_params.get_phase_pattern();
                f.motor_status.cmd = TP_OP_MOTOR_STATUS;
                f.motor_status.mode = t.mode & 0x0Fu;
                f.motor_status.motor_type = (mt == MotorType_STEP) ? 1u : ((mt == MotorType_BLDC) ? 2u : 0u);
                f.motor_status.phase_conn = (pp == PhasePattern_CBA) ? 2u : ((pp == PhasePattern_ACB) ? 3u : 1u);
                f.motor_status.pole_pairs = (uint8_t)((tp_->motor_params.get_pole_pairs() > 255)
                                                          ? 255
                                                          : tp_->motor_params.get_pole_pairs());
                f.motor_status.temp = temp_byte();
                f.motor_status.direction = (tp_->motor_params.get_direction() < 0) ? 1u : 0u;
                f.motor_status.enabled = ((t.flags & DRIVE_TLM_ENABLED) != 0u) ? 1u : 0u;
                f.motor_status.calibrated = tp_->angle_calibrator.is_calibrated() ? 1u : 0u;
                f.motor_status.identified = (tp_->motor_params.get_resistance() > 0.0f) ? 1u : 0u;
                f.motor_status.profile_loaded = tp_->motor_params.is_loaded() ? 1u : 0u;
                return true;
            }

            case TP_OP_ELECTRIC_STATUS: {
                const float p_elec = tp_->driver_phy.power.get_power_w();   // Bus-side power [W]
                f.electric_status.cmd = TP_OP_ELECTRIC_STATUS;
                f.electric_status.efficiency = efficiency_byte(p_elec);
                f.electric_status.power = tp_pack_bf16(p_elec);
                f.electric_status.supply = tp_pack_bf16(tp_->driver_phy.supply.get_voltage());
                f.electric_status.current = tp_pack_bf16(tp_->last_current_a_);
                return true;
            }

            case TP_OP_MOTION_STATUS_1: {
                DriveTelemetry t;                            // Live control snapshot
                tp_->drive.read_telemetry(t);
                f.motion_status1.cmd = TP_OP_MOTION_STATUS_1;
                f.motion_status1.motion_mode = t.motion_mode;
                tp_pack_i48(f.motion_status1.position, (int64_t)t.position_q16);
                return true;
            }

            case TP_OP_MOTION_STATUS_2: {
                DriveTelemetry t;                            // Live control snapshot
                tp_->drive.read_telemetry(t);
                // A frozen observer must not be reported as motion: with the
                // bridge released the rotor is coasting or stopped, and the
                // last value the loop computed says nothing about either.
                const bool live = (t.flags & DRIVE_TLM_LOOP_LIVE) != 0u;
                const float vel = live ? tp_->position_controller.getEstimatedVelocityRevS() : 0.0f;
                const uint32_t v24 = tp_pack_f24(vel);       // float24 velocity
                f.motion_status2.cmd = TP_OP_MOTION_STATUS_2;
                f.motion_status2.velocity[0] = (uint8_t)(v24);
                f.motion_status2.velocity[1] = (uint8_t)(v24 >> 8);
                f.motion_status2.velocity[2] = (uint8_t)(v24 >> 16);
                f.motion_status2.accel =
                    tp_pack_bf16(live ? tp_->position_controller.getEstimatedAccelerationRevS2() : 0.0f);
                f.motion_status2.torque = tp_pack_bf16(tp_->last_current_a_);
                return true;
            }

            case TP_OP_MOTION_STATUS_3: {
                DriveTelemetry t;                            // Live control snapshot
                tp_->drive.read_telemetry(t);
                const int32_t perr = t.target_pos_q16 - t.position_q16;      // Following error, Q16.16
                // Same staleness rule as MOTION_STATUS_2: report no velocity
                // error rather than a frozen one when the loop is not running.
                const bool live = (t.flags & DRIVE_TLM_LOOP_LIVE) != 0u;
                const float verr = live
                                       ? (tp_->position_controller.getEstimatedVelocityRevS() - t.target_vel_rps)
                                       : 0.0f;
                f.motion_status3.cmd = TP_OP_MOTION_STATUS_3;
                tp_pack_i24(f.motion_status3.pos_err, perr);
                f.motion_status3.vel_err = tp_pack_bf16(verr);
                f.motion_status3.queue = (t.queue_depth > 15u) ? 15u : t.queue_depth;
                f.motion_status3.time_scale = tp_pack_timescale(t.time_scale_q10);
                return true;
            }

            case TP_OP_MOTOR_PROPERTIES:
                f.motor_props.cmd = TP_OP_MOTOR_PROPERTIES;
                f.motor_props.temp_lim = tp_->motor_params.get_max_temp();
                f.motor_props.resistance = tp_pack_bf16(tp_->motor_params.get_resistance());
                f.motor_props.inductance = tp_pack_bf16(tp_->motor_params.get_effective_inductance());
                f.motor_props.torque = tp_pack_bf16(tp_->motor_params.get_torque_constant());
                return true;

            case TP_OP_ENCODER_STATUS:
                f.encoder_status.cmd = TP_OP_ENCODER_STATUS;
                f.encoder_status.health = 0u;                // Sensor supervision is not implemented yet
                f.encoder_status.angle = (uint16_t)(tp_->encoder.get_angle() >> 16);
                f.encoder_status.rotations = tp_->encoder.get_rotations();
                return true;

            case TP_OP_DRIVER_STATUS:
                f.driver_status.cmd = TP_OP_DRIVER_STATUS;
                f.driver_status.state = drive_state();
                f.driver_status.mode = control_mode();
                f.driver_status.fault = tp_->driver_phy.get_fault();
                f.driver_status.warn = warn_flags();
                f.driver_status.op_status = (uint8_t)tp_->current_driver.get_driver_status();
                f.driver_status.motor_status = (uint8_t)tp_->current_driver.get_motor_status();
                return true;

            case TP_OP_CALIBRATION_STATUS: {
                const MotorStatus ms = tp_->current_driver.get_motor_status();
                f.calib_status.cmd = TP_OP_CALIBRATION_STATUS;
                // 0xFF marks "idle": no routine is running, the drive is calibrated.
                f.calib_status.kind = (tp_->motor_status == MotorStatus::Success) ? 0xFFu
                                                                                  : (uint8_t)TP_CALIB_FULL;
                f.calib_status.stage = (uint8_t)ms;
                // The identification walks a known sequence of MotorStatus
                // values, so the stage number doubles as a coarse progress bar.
                f.calib_status.progress = (uint8_t)((ms == MotorStatus::Success) ? 255u
                                                                                 : (((uint32_t)ms * 255u) / 12u));
                f.calib_status.flags = (uint8_t)((tp_->angle_calibrator.is_calibrated() ? 0x01u : 0u) |
                                                 (tp_->angle_calibrator.is_settling() ? 0x02u : 0u) |
                                                 (tp_->angle_calibrator.is_persisting() ? 0x04u : 0u) |
                                                 ((ms == MotorStatus::CalibError) ? 0x08u : 0u));
                f.calib_status.cal_id = tp_->motor_params.get_cal_id();
                f.calib_status.pole_pairs = (uint16_t)tp_->motor_params.get_pole_pairs();
                return true;
            }

            case TP_OP_COMM_STATUS:
                f.comm_status.cmd = TP_OP_COMM_STATUS;
                f.comm_status.bus_state = bus_flags_;
                f.comm_status.tx_errors = bus_tx_err_;
                f.comm_status.rx_errors = bus_rx_err_;
                f.comm_status.dropped = bus_dropped_;
                f.comm_status.rejected = rejected_;
                return true;

            case TP_OP_POWER_STATUS: {
                const float temp = tp_->driver_phy.temperature.get_temperature_c();
                const float head = config_.ot_limit_c - temp;               // Degrees left to the trip
                f.power_status.cmd = TP_OP_POWER_STATUS;
                f.power_status.load = load_byte();
                f.power_status.bus_curr = tp_pack_bf16(tp_->driver_phy.power.get_supply_current_usage_a());
                f.power_status.bus_power = tp_pack_bf16(tp_->driver_phy.power.get_power_w());
                f.power_status.headroom = (uint8_t)((head < 0.0f) ? 0.0f : ((head > 255.0f) ? 255.0f : head));
                return true;
            }

            case TP_OP_HEARTBEAT:
                pack_heartbeat(f);
                return true;

            case TP_OP_MOTION_QUEUE_STAT: {
                DriveTelemetry t;                            // Live control snapshot
                tp_->drive.read_telemetry(t);
                f.queue_stat.cmd = TP_OP_MOTION_QUEUE_STAT;
                f.queue_stat.state = (uint8_t)(((t.flags & DRIVE_TLM_ENABLED) != 0u ? 0x01u : 0u) |
                                               ((t.flags & DRIVE_TLM_PAUSED) != 0u ? 0x02u : 0u) |
                                               ((t.flags & DRIVE_TLM_HALTED) != 0u ? 0x04u : 0u) |
                                               ((t.flags & DRIVE_TLM_STARVING) != 0u ? 0x08u : 0u));
                f.queue_stat.queued = t.queue_depth;
                f.queue_stat.free_slots = t.queue_free;
                f.queue_stat.time_scale = t.time_scale_q10;
                f.queue_stat.dropped = t.dropped_segments;
                return true;
            }

            case TP_OP_PARAM_STATUS:
                f.param_status.cmd = TP_OP_PARAM_STATUS;
                f.param_status.state = (uint8_t)((store_.is_busy() || tp_->motor_params.is_flash_busy() ? 0x01u : 0u) |
                                                 (store_.is_loaded() ? 0x02u : 0u) |
                                                 (tp_->motor_params.is_loaded() ? 0x04u : 0u) |
                                                 // Only a save that actually ran and failed
                                                 // sets the failure bit; a node that has simply
                                                 // never saved is not in an error state.
                                                 ((store_.save_attempted() && !store_.is_saved())
                                                      ? 0x08u : 0u));
                f.param_status.config_ver = store_.stored_version();
                f.param_status.motor_cal_id = tp_->motor_params.get_cal_id();
                f.param_status.config_free = (uint16_t)store_.free_bytes();
                f.param_status.user_free = 0u;               // The motor engine does not expose it
                return true;

            case TP_OP_CALIB_RESULT:
                f.calib_result.cmd = TP_OP_CALIB_RESULT;
                f.calib_result.flags =
                    (uint8_t)(((tp_->motor_params.get_resistance() > 0.0f) ? 0x01u : 0u) |
                              (tp_->angle_calibrator.is_calibrated() ? 0x02u : 0u) |
                              ((tp_->current_driver.get_motor_status() == MotorStatus::CalibError) ? 0x04u : 0u));
                f.calib_result.resistance = tp_pack_bf16(tp_->motor_params.get_resistance());
                f.calib_result.induct_d = tp_pack_bf16(tp_->motor_params.get_inductance_d());
                f.calib_result.induct_q = tp_pack_bf16(tp_->motor_params.get_inductance_q());
                return true;

            default:
                return false;                                // Not a status chunk
        }
    }

    /** @brief Fill the heartbeat chunk (also used verbatim as the emergency frame). */
    void pack_heartbeat(TpFrame& f) {
        DriveTelemetry t;                                    // Live control snapshot
        tp_->drive.read_telemetry(t);
        f.u64 = 0;                                           // Clean slate
        f.heartbeat.cmd = TP_OP_HEARTBEAT;
        f.heartbeat.state = drive_state();
        f.heartbeat.mode = control_mode();
        f.heartbeat.fault = tp_->driver_phy.get_fault();
        f.heartbeat.warn = warn_flags();
        f.heartbeat.supply = tp_pack_volts_q2(tp_->driver_phy.supply.get_voltage());
        f.heartbeat.temp = temp_byte();
        f.heartbeat.load = load_byte();
        f.heartbeat.seq = (uint8_t)(hb_seq_++ & 0x0Fu);       // Lets a master spot a lost heartbeat
        f.heartbeat.queue = (t.queue_depth > 15u) ? 15u : t.queue_depth;
    }

    // ==================================================================
    // State derivation
    // ==================================================================

    /** @brief Collapse the firmware's lifecycle into one TpDriveState value. */
    uint8_t drive_state() const {
        if (reboot_pending_) {
            return TP_STATE_REBOOTING;                       // Reset already armed
        }
        if (tp_->driver_phy.get_fault() != 0u) {
            return TP_STATE_FAULT;                           // A latched fault wins over everything
        }
        if (discovery_.is_active()) {
            return TP_STATE_CONFIG;                          // Being addressed by the master
        }
        if (!tp_->driver_phy.current.is_calibrated()) {
            return TP_STATE_BOOT;                            // Current-sense offsets unknown
        }
        if (!tp_->angle_calibrator.is_calibrated() ||
            (tp_->current_driver.get_motor_status() != MotorStatus::Success)) {
            return TP_STATE_CALIBRATING;                     // Identification or sweep in progress
        }
        DriveTelemetry t;                                    // Live control snapshot
        tp_->drive.read_telemetry(t);
        // "Enabled" is the output-stage arm; it is NOT the same as "controlling
        // the motor". A drive can be enabled while its mode is DISABLED or its
        // watchdog has expired, in which case the safe-state gate has released
        // the bridge and the rotor is free. Reporting READY there would claim
        // the drive is holding position when it is coasting, so the state is
        // taken from whether the loop is actually running.
        if (((t.flags & DRIVE_TLM_ENABLED) == 0u) || ((t.flags & DRIVE_TLM_LOOP_LIVE) == 0u)) {
            return TP_STATE_IDLE;                            // De-energized or not regulating
        }
        return ((t.flags & DRIVE_TLM_MOVING) != 0u) ? TP_STATE_RUNNING : TP_STATE_READY;
    }

    /** @brief The active control mode as a wire value. */
    uint8_t control_mode() const {
        DriveTelemetry t;                                    // Live control snapshot
        tp_->drive.read_telemetry(t);
        return t.mode & 0x0Fu;
    }

    /** @brief Assemble the warning byte from every subsystem that can raise one. */
    uint8_t warn_flags() const {
        uint8_t w = 0;                                       // Assembled warning bits
        DriveTelemetry t;                                    // Live control snapshot
        tp_->drive.read_telemetry(t);
        if (!tp_->angle_calibrator.is_calibrated()) {
            w |= TP_WARN_NOT_CALIBRATED;                     // No angle table
        }
        if (tp_->motor_params.is_flash_busy() || store_.is_busy()) {
            w |= TP_WARN_FLASH_BUSY;                         // An NVM transfer is draining
        }
        if ((t.flags & DRIVE_TLM_WATCHDOG) != 0u) {
            w |= TP_WARN_WATCHDOG;                           // The master went quiet
        }
        if ((t.flags & DRIVE_TLM_STARVING) != 0u) {
            w |= TP_WARN_QUEUE_LOW;                          // One segment left to execute
        }
        if (t.dropped_segments != 0u) {
            w |= TP_WARN_QUEUE_DROP;                         // A segment was rejected
        }
        if ((bus_dropped_ != 0u) || ((bus_flags_ & 0x03u) != 0u)) {
            w |= TP_WARN_COMM;                               // Frames lost or the bus is degraded
        }
        const float temp = tp_->driver_phy.temperature.get_temperature_c();
        if (temp > (config_.ot_limit_c * 0.8f)) {
            w |= TP_WARN_TEMPERATURE;                        // Within 20 % of the thermal trip
        }
        const float v = tp_->driver_phy.supply.get_voltage();
        if ((v < (config_.uv_limit_v * 1.1f)) || (v > (config_.ov_limit_v * 0.9f))) {
            w |= TP_WARN_SUPPLY;                             // Approaching a rail limit
        }
        return w;
    }

    /** @brief Temperature as a signed byte; -128 marks "no data". */
    uint8_t temp_byte() const {
        const float t = tp_->driver_phy.temperature.get_temperature_c();
        if (t != t) {
            return 0x80u;                                    // NaN: report "no data"
        }
        return tp_pack_temp_c(t);
    }

    /** @brief Normalized load: applied current against the envelope, -128 on overload. */
    uint8_t load_byte() const {
        if (tp_->driver_phy.get_fault() != 0u) {
            return 0x80u;                                    // Overload marker
        }
        const float lim = (config_.current_limit_a > 0.0f) ? config_.current_limit_a : 1.0f;
        float ratio = tp_->last_current_a_ / lim;            // Signed fraction of the envelope
        if (ratio > 1.0f) {
            ratio = 1.0f;                                    // Saturate positive
        }
        if (ratio < -1.0f) {
            ratio = -1.0f;                                   // Saturate negative
        }
        return (uint8_t)(int8_t)(int32_t)(ratio * 127.0f);
    }

    /**
     * @brief Rough conversion efficiency as a 0-255 byte.
     *
     * Mechanical power is not observed, so efficiency is estimated from the
     * copper loss the identified resistance predicts for the applied current.
     * 255 means "not available", which is what a drive with no identified
     * resistance reports.
     */
    uint8_t efficiency_byte(float p_elec) const {
        const float r = tp_->motor_params.get_resistance();  // Identified winding resistance
        if ((r <= 0.0f) || (p_elec <= 0.1f)) {
            return 255u;                                     // Cannot be estimated
        }
        const float i = tp_->last_current_a_;                // Applied q-axis current
        const float loss = i * i * r;                        // Copper loss [W]
        float eff = (p_elec - loss) / p_elec;                // Fraction delivered to the shaft
        if (eff < 0.0f) {
            eff = 0.0f;                                      // Clamp a nonsensical estimate
        }
        if (eff > 1.0f) {
            eff = 1.0f;                                      // Clamp a nonsensical estimate
        }
        return (uint8_t)(eff * 254.0f);                      // 255 stays reserved for "unknown"
    }

    /** @brief true when the output stage is currently energized by command. */
    bool is_enabled() const {
        if (tp_ == nullptr) {
            return false;                                    // Not bound to a drive
        }
        DriveTelemetry t;                                    // Live control snapshot
        tp_->drive.read_telemetry(t);
        return (t.flags & DRIVE_TLM_ENABLED) != 0u;
    }

    /** @brief Reload the control loop's master-liveness watchdog. */
    void feed_watchdog() {
        if (tp_ != nullptr) {
            tp_->drive.feed_watchdog();                      // One 32-bit store, ISR-safe
        }
    }

    // ==================================================================
    // Reply helpers
    // ==================================================================

    /** @brief Transmit a reply on this node's response identifier. */
    void respond(const TpFrame& f) {
        if (write_ != nullptr) {
            write_((uint16_t)(TP_ID_RESPONSE_BASE + config_.node_id), f);
        }
    }

    /** @brief Build a generic acknowledgement. */
    static void ack(TpFrame& f, uint8_t opcode) {
        f.u64 = 0;                     // Reserved bytes transmit as zero
        f.ack.cmd = TP_OP_ACK;         // Generic success reply
        f.ack.opcode = opcode;         // What was executed
        f.ack.error = TP_ERR_NONE;     // No error by definition
    }

    /** @brief Build a generic rejection and count it. */
    void nack(TpFrame& f, uint8_t opcode, TpError err, uint8_t detail) {
        f.u64 = 0;                     // Reserved bytes transmit as zero
        f.ack.cmd = TP_OP_NACK;        // Generic failure reply
        f.ack.opcode = opcode;         // What was refused
        f.ack.error = (uint8_t)err;    // Why it was refused
        f.ack.detail = detail;         // Context: register id, fault mask, stage
        if (rejected_ != 0xFFFFu) {
            rejected_++;               // Saturating: a wrapped counter hides a storm
        }
    }

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
    void verify_register_table() const {
        for (uint32_t i = 1; i < TP_REGISTER_COUNT; ++i) {
            if (TP_REGISTERS[i - 1u].id >= TP_REGISTERS[i].id) {
                TP_LOG_VAL("TPCP: REGISTER TABLE NOT SORTED at index", (unsigned)i);
                return;                                      // One report is enough
            }
        }
    }

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
