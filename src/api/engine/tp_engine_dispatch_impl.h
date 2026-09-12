#pragma once

/**
 * @file tp_engine_dispatch_impl.h
 * @brief Frame routing: identifier class, then opcode.
 *
 * dispatch() classifies by identifier - network management, the setpoint
 * stream, or the request channel - and handle_request() fans the request
 * channel out by opcode into every other group.
 *
 * Two orderings here are load-bearing. A bit-packed motion burst is tested for
 * FIRST, because it carries no opcode byte at all. And MOTOR_PROPERTIES is
 * handled before the generic "read a status chunk" shortcut, because it is the
 * one opcode that is both readable and writable.
 *
 * respond() and nack() live here too: they are the reply mechanism the whole
 * dispatch tree shares, and nack() is what counts a rejection.
 *
 * Part of TpProtocol, declared in tp_engine.h. Including that header pulls this
 * one in at the end; including this one first pulls tp_engine.h, whose own
 * include of this file is then skipped by #pragma once - so the class is always
 * complete before these definitions are parsed, whichever order is used.
 *
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#include "../tp_engine.h"  // class TpProtocol - the declarations these bodies define

inline void TpProtocol::dispatch(uint16_t id, const TpFrame& f, uint8_t len) {
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

inline void TpProtocol::handle_nmt(uint16_t id, const TpFrame& f, uint8_t len) {
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

inline void TpProtocol::handle_global(const TpFrame& f) {
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

inline void TpProtocol::handle_request(const TpFrame& f, bool broadcast) {
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

inline void TpProtocol::respond(const TpFrame& f) {
    if (write_ != nullptr) {
        write_((uint16_t)(TP_ID_RESPONSE_BASE + config_.node_id), f);
    }
}

inline void TpProtocol::nack(TpFrame& f, uint8_t opcode, TpError err, uint8_t detail) {
    f.u64 = 0;                     // Reserved bytes transmit as zero
    f.ack.cmd = TP_OP_NACK;        // Generic failure reply
    f.ack.opcode = opcode;         // What was refused
    f.ack.error = (uint8_t)err;    // Why it was refused
    f.ack.detail = detail;         // Context: register id, fault mask, stage
    if (rejected_ != 0xFFFFu) {
        rejected_++;               // Saturating: a wrapped counter hides a storm
    }
}
