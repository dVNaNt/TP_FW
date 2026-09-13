#pragma once

/**
 * @file tp_engine_telemetry_impl.h
 * @brief PulseBeat scheduling, cyclic telemetry slots and emergency frames.
 *
 * The PulseBeat and the four configurable slots each carry their own due
 * time, so a slow slot never delays a fast one. Slots flagged ON_SYNC are
 * emitted from the bus time sync instead of from a timer, and the sync also
 * re-phases the PulseBeat by the configured delay so several drives do not
 * beat on top of one another.
 *
 * emit_emergency_on_change() is edge-triggered on the driver fault byte: an
 * emergency frame is sent once when the fault set changes, not repeatedly
 * while it persists. A PulseBeat the transport refuses for the configured
 * number of retries is escalated to the emergency channel too (a FaultBeat).
 *
 * Part of TpProtocol, declared in tp_engine.h.
 *
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#include "../tp_engine.h"  // class TpProtocol - the declarations these bodies define

inline void TpProtocol::service_telemetry(uint32_t now) {
    if ((config_.node_id == TP_NODE_UNASSIGNED) || (config_.comm_mode == TP_COMM_SILENT) || (write_ == nullptr)) {
        return;                                          // Unaddressed or silent: a node says nothing
    }
    // PulseBeat first: it is the frame a supervisor keys its own timeout on.
    if (config_.heartbeat_s > 0.0f) {
        if ((int32_t)(now - hb_next_ms_) >= 0) {
            const uint32_t period_ms = (uint32_t)(config_.heartbeat_s * 1000.0f);  // Beat period
            hb_next_ms_ = now + ((period_ms == 0u) ? 1u : period_ms);              // Schedule the next one
            TpFrame f;                                   // Beat under construction
            pack_pulsebeat(f, true);
            if (write_((uint16_t)(TP_ID_TELEMETRY_BASE + config_.node_id), f)) {
                hb_failed_ = 0u;                         // Delivered: the streak is over
            } else {
                // The transport refused it. With retries configured the beat
                // is escalated to the emergency channel once the streak is
                // long enough - a master that hears a FaultBeat knows the
                // telemetry channel itself is in trouble.
                if (hb_failed_ != 0xFFu) {
                    hb_failed_++;
                }
                if ((config_.hb_retry != 0u) && (config_.hb_retry != 127u) && (hb_failed_ >= config_.hb_retry)) {
                    write_((uint16_t)(TP_ID_EMCY_BASE + config_.node_id), f);
                    hb_failed_ = 0u;
                }
                if (config_.hb_retry_force != 0u) {
                    hb_next_ms_ = now + 1u;              // Retry at once instead of next period
                }
            }
        }
    }
    for (uint32_t i = 0; i < TP_TELEMETRY_SLOTS; ++i) {
        const TpTelemetrySlot& s = config_.telemetry[i];  // Slot under consideration
        if ((s.reg == 0u) || ((s.flags & TP_TLM_ENABLE) == 0u)) {
            continue;                                    // Slot unused or disabled
        }
        if ((s.flags & TP_TLM_ON_SYNC) != 0u) {
            continue;                                    // Driven by the sync, not by the timer
        }
        if (s.period_ms == 0u) {
            continue;                                    // No timer configured
        }
        if ((int32_t)(now - tlm_next_ms_[i]) >= 0) {
            tlm_next_ms_[i] = now + s.period_ms;         // Schedule the next emission
            emit_register(s.reg);
        }
    }
}

inline void TpProtocol::on_sync(uint32_t now) {
    // The beat is re-phased by the configured delay: with every drive given
    // a different delay a bus of them beats in sequence rather than at once.
    hb_next_ms_ = now + (uint32_t)(config_.heartbeat_delay_s * 1000.0f);
    for (uint32_t i = 0; i < TP_TELEMETRY_SLOTS; ++i) {
        const TpTelemetrySlot& s = config_.telemetry[i];  // Slot under consideration
        if ((s.reg != 0u) && ((s.flags & TP_TLM_ENABLE) != 0u) && ((s.flags & TP_TLM_ON_SYNC) != 0u)) {
            emit_register(s.reg);                        // Synchronized across every node
        }
    }
}

inline void TpProtocol::emit_emergency_on_change() {
    if (tp_ == nullptr) {
        return;                                          // Not bound to a drive
    }
    const uint8_t fault = tp_->fault_byte();   // Current fault byte
    if (fault == last_fault_) {
        return;                                          // Nothing changed
    }
    last_fault_ = fault;                                 // Latch the new state either way
    if ((config_.node_id == TP_NODE_UNASSIGNED) || (config_.comm_mode == TP_COMM_SILENT) || (write_ == nullptr)) {
        return;                                          // A node must stay silent without an address
    }
    TpFrame f;                                           // Emergency frame
    pack_pulsebeat(f, false);                            // Same layout as the beat
    write_((uint16_t)(TP_ID_EMCY_BASE + config_.node_id), f);
}

inline bool TpProtocol::emit_register(uint8_t reg) {
    if ((config_.node_id == TP_NODE_UNASSIGNED) || (config_.comm_mode == TP_COMM_SILENT) || (write_ == nullptr)) {
        return false;                                    // A node must stay silent without an address
    }
    const TpRegDesc* d = tp_find_register(reg);          // Sub-space registers emit their record 0
    if (d == nullptr) {
        return false;                                    // Unknown register
    }
    TpFrame req;                                         // Synthesized request (record 0)
    req.u64 = 0;
    req.b8[0] = reg;
    TpFrame f;                                           // Frame under construction
    f.u64 = 0;
    if (read_register(reg, 0u, req, f) != TP_ERR_NONE) {
        return false;                                    // Not readable right now
    }
    if (tp_cmd_is_pulsebeat(reg)) {
        pack_pulsebeat(f, true);                         // A streamed beat advances the counter
    }
    return write_((uint16_t)(TP_ID_TELEMETRY_BASE + config_.node_id), f);
}
