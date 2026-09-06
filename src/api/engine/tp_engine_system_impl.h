#pragma once

/**
 * @file tp_engine_system_impl.h
 * @brief Node address, telemetry slots, watchdog and lifecycle commands.
 *
 * The system group covers everything that changes what the node IS rather
 * than what the motor does: its address, which telemetry it emits, how it
 * reacts to a silent master, and the reset / auto-enable path.
 *
 * adopt_node_id() rebinds the hardware acceptance filters BEFORE anything else
 * so the node never spends a window listening on its old address.
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

inline void TpProtocol::handle_node_id(const TpFrame& f, TpFrame& tx) {
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

inline void TpProtocol::adopt_node_id(uint8_t id, bool persist) {
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

inline void TpProtocol::handle_telemetry_cfg(const TpFrame& f, TpFrame& tx) {
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

inline void TpProtocol::handle_watchdog_cfg(const TpFrame& f, TpFrame& tx) {
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

inline void TpProtocol::service_auto_enable() {
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

inline void TpProtocol::arm_reboot(uint8_t mode) {
    reboot_mode_ = mode;                        // 0 = warm reset, 1 = USB DFU
    reboot_at_ms_ = millis() + REBOOT_DELAY_MS; // Let the transmit queue drain first
    reboot_pending_ = true;                     // poll() executes it
    if (tp_ != nullptr) {
        tp_->drive.post_u32(DriveReq::SetEnable, 0u);  // Never reset an energized bridge
    }
}

inline void TpProtocol::clear_faults() {
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
