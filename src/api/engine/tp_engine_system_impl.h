#pragma once

/**
 * @file tp_engine_system_impl.h
 * @brief Enable, calibration, persistence, MIDI, node address and lifecycle commands.
 *
 * The system group covers everything that changes what the node IS rather
 * than what the motor does: whether its stage is energized, which
 * calibrations it has run, what it has stored, its address, how it reacts to
 * a reset request, and - as the one playful exception - what it sings.
 *
 * adopt_node_id() rebinds the hardware acceptance filters BEFORE anything else
 * so the node never spends a window listening on its old address.
 *
 * Part of TpProtocol, declared in tp_engine.h.
 *
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#include "../tp_engine.h"  // class TpProtocol - the declarations these bodies define

inline TpError TpProtocol::set_enable(bool on) {
    if (on) {
        if (!tp_->angle_calibrator.is_calibrated()) {
            // Energizing without an angle table would command an arbitrary
            // electrical angle: refuse rather than jerk.
            return TP_ERR_NOT_CALIBRATED;
        }
        if (tp_->driver_phy.get_fault() != 0u) {
            return TP_ERR_FAULT_ACTIVE;                  // Clear the latch first
        }
        if (tp_->current_driver.is_identifying()) {
            return TP_ERR_BUSY;                          // The identification owns the bridge
        }
    }
    return tp_->drive.post_u32(DriveReq::SetEnable, on ? 1u : 0u) ? TP_ERR_NONE : TP_ERR_BUSY;
}

// ============================================================================
// Calibration
// ============================================================================

inline uint8_t TpProtocol::calibration_running() const {
    uint8_t bits = 0;                                    // Routines currently owning the bridge
    if (!tp_->driver_phy.current.is_calibrated()) {
        bits |= TP_CAL_I;                                // Offsets still being measured
    }
    // A runtime identification that ended in CalibError has finished, badly;
    // it is not running, so a master may request another one.
    const MotorStatus ms = tp_->current_driver.get_motor_calibration_status();
    if ((tp_->current_driver.is_identifying() && (ms != MotorStatus::CalibError)) ||
        (tp_->motor_status != MotorStatus::Success)) {
        bits |= TP_CAL_ELECTRICAL;                       // Identification (boot or runtime)
    }
    if (tp_->angle_calibrator.is_running()) {
        bits |= TP_CAL_P;                                // Angle sweep, settling or storing
    }
    return (uint8_t)(bits | cal_pending_);               // Plus what is queued to run next
}

inline uint8_t TpProtocol::calibration_valid() const {
    uint8_t bits = 0;                                    // Results the drive holds
    if (tp_->driver_phy.current.is_calibrated()) {
        bits |= TP_CAL_I;                                // Current-sense offsets known
    }
    if (tp_->motor_params.get_resistance() > 0.0f) {
        bits |= TP_CAL_ELECTRICAL;                       // R, L and the wiring were identified
    }
    if (tp_->angle_calibrator.is_calibrated()) {
        bits |= TP_CAL_P;                                // Angle table valid
    }
    return bits;
}

inline TpError TpProtocol::start_calibration(uint8_t bits) {
    if ((bits & (TP_CAL_F | TP_CAL_M)) != 0u) {
        return TP_ERR_NOT_IMPLEMENTED;                   // No flux or mechanical routine exists
    }
    if (tp_->driver_phy.get_fault() != 0u) {
        return TP_ERR_FAULT_ACTIVE;                      // Clear faults before moving the rotor
    }
    if ((calibration_running() & ~cal_pending_) != 0u) {
        return TP_ERR_BUSY;                              // One routine at a time
    }
    // Calibration drives the rotor open loop, so it must own the machine:
    // any queued trajectory and any commanded mode is dropped first.
    tp_->drive.post(DriveReq::QueueClear);
    tp_->drive.post_u32(DriveReq::SetMode, TP_MODE_DISABLED);
    tp_->drive.post_u32(DriveReq::SetEnable, 0u);

    if ((bits & TP_CAL_I) != 0u) {
        tp_->driver_phy.current.calibrate();             // Re-measure the amplifier offsets
    }
    if ((bits & TP_CAL_ELECTRICAL) != 0u) {
        // One routine identifies the wiring, the resistance and the inductance
        // together; asking for any of the three runs all of it.
        tp_->current_driver.setup_status.set_driver_status(OperationStatus::DriverCalibration);
        tp_->current_driver.start_motor_calibration();
        if ((bits & TP_CAL_P) != 0u) {
            cal_pending_ = TP_CAL_P;                     // The sweep follows once R and L are known
        }
        return TP_ERR_NONE;
    }
    if ((bits & TP_CAL_P) != 0u) {
        // A requested sweep is always a physical one. Without the force a
        // drive with a stored profile reloads its saved table and the shaft
        // never moves, which looks exactly like a request that did nothing.
        tp_->angle_calibrator.calibrate(true);
    }
    return TP_ERR_NONE;
}

inline void TpProtocol::service_calibration() {
    if ((cal_pending_ == 0u) || (tp_ == nullptr)) {
        return;                                          // Nothing sequenced
    }
    if (tp_->current_driver.get_motor_calibration_status() == MotorStatus::CalibError) {
        // Checked BEFORE is_identifying(): a failed identification reports
        // itself as still identifying (its status is not Success), so the
        // queued sweep stayed pending forever and the diagnostics kept
        // showing the angle calibration as running.
        cal_pending_ = 0u;                               // Drop the queued sweep
        TP_LOG("TPCP: identification failed - angle sweep not started");
        return;                                          // Do not sweep on a motor that failed
    }
    if (tp_->current_driver.is_identifying()) {
        return;                                          // The identification still owns the bridge
    }
    cal_pending_ = 0u;                                   // One shot either way
    tp_->angle_calibrator.calibrate(true);               // Second step of the sequence
    TP_LOG("TPCP: identification done - starting the angle sweep");
}

// ============================================================================
// Persistence
// ============================================================================

inline TpError TpProtocol::handle_save(const TpFrame& f) {
    if (tp_ == nullptr) {
        return TP_ERR_WRONG_STATE;                       // Not bound to a drive
    }
    const uint32_t catalog = tp_unpack_u24(f.save.catalog);  // Which stores
    const bool want_config = (catalog & TP_CATALOG_CONFIG) != 0u;
    const bool want_motor = (catalog & TP_CATALOG_MOTOR) != 0u;
    if (!want_config && !want_motor) {
        return TP_ERR_BAD_PARAM;                         // Nothing selected
    }
    switch (f.save.keyword) {
        case TP_SAVE_KEY: {
            if (store_.is_busy() || tp_->motor_params.is_flash_busy()) {
                return TP_ERR_BUSY;                      // A transfer is already draining
            }
            if (is_enabled()) {
                return TP_ERR_WRONG_STATE;               // Disable the stage before writing flash
            }
            bool armed = false;                          // At least one engine started
            if (want_config) {
                armed |= store_.begin_save(config_);
            }
            if (want_motor) {
                tp_->motor_params.begin_save();          // Gated internally on full calibration
                armed = true;
            }
            return armed ? TP_ERR_NONE : TP_ERR_NVM;
        }
        case TP_DEFAULTS_KEY: {
            if (is_enabled()) {
                return TP_ERR_WRONG_STATE;               // Reloading limits under load is unsafe
            }
            if (want_config) {
                // Resets the live shadow only: reversible until it is saved,
                // and it keeps the address so the bench does not lose contact
                // with the drive mid-session.
                const uint8_t keep_id = config_.node_id;
                config_.set_defaults();
                config_.node_id = keep_id;
                tp_apply_config(*tp_, config_);
            }
            if (want_motor) {
                tp_->motor_params.begin_load(MOTOR_ID_ANY);  // Back to the stored profile
            }
            return TP_ERR_NONE;
        }
        case TP_ERASE_KEY: {
            if (is_enabled()) {
                return TP_ERR_WRONG_STATE;               // Same flash-window reasoning as above
            }
            if (!want_config) {
                return TP_ERR_NOT_IMPLEMENTED;           // The motor profile has no erase path
            }
            // Full factory reset: erase the stored record, so the node really
            // does come back on compiled-in defaults after a reboot. This DOES
            // surrender the node address - the drive comes back unaddressed and
            // has to be found by a discovery sweep again - which is exactly what
            // a factory reset means, and why it needs its own keyword.
            if (!store_.erase_all()) {
                return TP_ERR_NVM;                       // No programming window, or no partition
            }
            config_.set_defaults();
            tp_apply_config(*tp_, config_);
            TP_LOG("TPCP: factory reset - stored configuration erased");
            return TP_ERR_NONE;
        }
        default:
            return TP_ERR_ACCESS_DENIED;                 // Guarded: a stray frame must not save or wipe
    }
}

inline void TpProtocol::service_store(uint32_t now) {
    // Programming is skipped internally unless the control loop reports a
    // de-energized bridge, so this call is cheap while the drive is live.
    if (!store_.is_busy()) {
        save_deadline_ms_ = 0u;                          // Idle: nothing to time out
        return;
    }
    if (store_.service()) {
        save_deadline_ms_ = 0u;                          // Finished: arm a fresh deadline next time
        return;
    }
    // Still draining. A save that can never find a programming window (the
    // drive was re-energized before it completed) would otherwise leave the
    // engine busy forever and block every later save, so it is abandoned
    // after a bounded wait and reported through the diagnostics as failed.
    if (save_deadline_ms_ == 0u) {
        save_deadline_ms_ = now + SAVE_TIMEOUT_MS;
    } else if ((int32_t)(now - save_deadline_ms_) >= 0) {
        store_.abort();
        save_deadline_ms_ = 0u;
        TP_LOG("TPCP: configuration save abandoned - no programming window");
    }
}

// ============================================================================
// MIDI
// ============================================================================

inline TpError TpProtocol::handle_midi(const TpFrame& f) {
    if (tp_ == nullptr) {
        return TP_ERR_WRONG_STATE;                       // Not bound to a drive
    }
    if ((f.midi.wave_mode & 0x0Fu) != TP_WAVE_SINE) {
        return TP_ERR_BAD_PARAM;                         // The generator only produces a sine
    }
    const uint32_t now = millis();                       // Note start
    if (f.midi.tone == 0u) {
        // Silence: end the note, leave the mode to the master.
        midi_active_ = false;
        midi_freq_ = 0u;
        midi_last_amp_ = 0u;
        tp_->drive.post_u32(DriveReq::SetTone, 0u);
        return TP_ERR_NONE;
    }
    // MIDI note number -> frequency: 440 Hz * 2^((n - 69) / 12), through the
    // FPU's exp2 rather than a table. Bounded to what a winding can radiate:
    // below the floor the coil is close to a short at audio rates, above the
    // ceiling the inductance swallows the note (and the generator refuses it).
    const float hz = 440.0f * __builtin_exp2f(((float)f.midi.tone - 69.0f) * (1.0f / 12.0f));
    if ((hz < 40.0f) || (hz > 2000.0f)) {
        return TP_ERR_OUT_OF_RANGE;
    }
    if (!tp_->angle_calibrator.is_calibrated()) {
        return TP_ERR_NOT_CALIBRATED;                    // Tone mode still drives the bridge
    }
    if (tp_->driver_phy.get_fault() != 0u) {
        return TP_ERR_FAULT_ACTIVE;
    }
    float duration_ms = 0.0f;                            // 0 = until the next command
    if (tp_bf16_is_finite(f.midi.duration)) {
        duration_ms = tp_unpack_bf16(f.midi.duration);
        if (duration_ms < 0.0f) {
            return TP_ERR_BAD_PARAM;
        }
    }
    midi_freq_ = (uint16_t)(hz + 0.5f);
    midi_amp_ = (uint16_t)(((uint32_t)f.midi.volume * MIDI_MAX_AMP_Q15) / 255u);
    midi_start_ms_ = now;
    midi_end_ms_ = (duration_ms > 0.0f) ? (now + (uint32_t)duration_ms) : 0u;
    midi_attack_ms_ = (uint32_t)(duration_ms * (float)f.midi.attack / 255.0f);
    midi_decay_ms_ = (uint32_t)(duration_ms * (float)f.midi.decay / 255.0f);
    midi_active_ = true;
    midi_last_amp_ = 0xFFFFu;                            // Force the first envelope post
    if (control_mode() != TP_MODE_TONE) {
        tp_->drive.post_u32(DriveReq::SetMode, TP_MODE_TONE);
    }
    tp_->drive.post_u32(DriveReq::SetEnable, 1u);        // A note needs the bridge
    service_midi(now);                                   // First envelope sample right away
    return TP_ERR_NONE;
}

inline void TpProtocol::service_midi(uint32_t now) {
    if (!midi_active_ || (tp_ == nullptr)) {
        return;                                          // Nothing playing
    }
    if ((midi_end_ms_ != 0u) && ((int32_t)(now - midi_end_ms_) >= 0)) {
        midi_active_ = false;                            // Note over: silence, stay in tone mode
        midi_last_amp_ = 0u;
        tp_->drive.post_u32(DriveReq::SetTone, 0u);
        return;
    }
    // Linear attack and decay shares of the duration; an untimed note plays
    // at full amplitude. Amplitude only re-posts when it changes, so a held
    // note costs the mailbox nothing.
    uint32_t amp = midi_amp_;                            // Envelope value this pass
    if (midi_end_ms_ != 0u) {
        const uint32_t elapsed = now - midi_start_ms_;   // Into the note
        const uint32_t remaining = midi_end_ms_ - now;   // Until its end
        if ((midi_attack_ms_ != 0u) && (elapsed < midi_attack_ms_)) {
            amp = (midi_amp_ * elapsed) / midi_attack_ms_;
        } else if ((midi_decay_ms_ != 0u) && (remaining < midi_decay_ms_)) {
            amp = (midi_amp_ * remaining) / midi_decay_ms_;
        }
    }
    if ((uint16_t)amp != midi_last_amp_) {
        midi_last_amp_ = (uint16_t)amp;
        tp_->drive.post_u32(DriveReq::SetTone, (uint32_t)midi_freq_ | (amp << 16));
    }
}

// ============================================================================
// Node address and lifecycle
// ============================================================================

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
    tp_->driver_phy.set_fault().encoder(false);       // Release the encoder latch (re-asserted while the link is down)
    // A failed runtime identification parks the drive in CalibError, which
    // refuses every enable until a reboot. The fault reset is the operator's
    // "try again": drop back to the last known parameters and let the drive
    // run; a new identification may be requested afterwards.
    tp_->current_driver.recover_identification();
}
