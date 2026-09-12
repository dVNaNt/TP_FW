#pragma once

/**
 * @file tp_engine_telemetry_impl.h
 * @brief Cyclic telemetry scheduling and emergency frames.
 *
 * The heartbeat and the four configurable slots each carry their own due
 * time, so a slow slot never delays a fast one. Slots flagged ON_SYNC are
 * emitted from the SYNC handler instead of from a timer.
 *
 * emit_emergency_on_change() is edge-triggered on the driver fault byte: an
 * emergency frame is sent once when the fault set changes, not repeatedly
 * while it persists.
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

inline void TpProtocol::service_telemetry(uint32_t now) {
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

inline void TpProtocol::on_sync() {
    for (uint32_t i = 0; i < TP_TELEMETRY_SLOTS; ++i) {
        const TpTelemetrySlot& s = config_.telemetry[i];  // Slot under consideration
        if ((s.opcode != 0u) && ((s.flags & TP_TLM_ENABLE) != 0u) &&
            ((s.flags & TP_TLM_ON_SYNC) != 0u)) {
            emit_chunk(s.opcode);                        // Synchronized across every node
        }
    }
}

inline void TpProtocol::emit_emergency_on_change() {
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

inline void TpProtocol::emit_chunk(uint8_t opcode) {
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
