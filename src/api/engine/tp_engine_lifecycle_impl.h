#pragma once

/**
 * @file tp_engine_lifecycle_impl.h
 * @brief Bring-up and the thread-mode service loop.
 *
 * attach() and bind_platform() wire the engine to a transport and a flash
 * partition; begin() restores the stored configuration and applies it; poll()
 * is the whole thread-mode half of the protocol.
 *
 * poll() runs five bounded stages per call - drain at most MAX_FRAMES_PER_POLL
 * frames, check for a fault edge, service telemetry timers, advance a pending
 * flash save, then fire an armed reboot. Every stage is bounded so one busy
 * burst can never monopolize loop().
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

inline void TpProtocol::attach(TunePulseHeart* tp,
                               TpFrameRead rd,
                               TpFrameWrite wr,
                               TpFrameWriteEmpty wr_id,
                               TpBindFilters filters) {
    tp_ = tp;             // Firmware state
    read_ = rd;           // RX path
    write_ = wr;          // TX path
    write_id_ = wr_id;    // Discovery TX path
    bind_filters_ = filters;  // Filter control
}

inline void TpProtocol::bind_platform(FlashEEPROM* cfg_flash, TpRebootFn reboot) {
    store_.attach(cfg_flash);  // Persistence backend
    reboot_ = reboot;          // Reset vector
}

inline void TpProtocol::begin() {
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

inline void TpProtocol::poll() {
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

inline void TpProtocol::verify_register_table() const {
    for (uint32_t i = 1; i < TP_REGISTER_COUNT; ++i) {
        if (TP_REGISTERS[i - 1u].id >= TP_REGISTERS[i].id) {
            TP_LOG_VAL("TPCP: REGISTER TABLE NOT SORTED at index", (unsigned)i);
            return;                                      // One report is enough
        }
    }
}
