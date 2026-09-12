#pragma once

/**
 * @file tp_engine_persist_impl.h
 * @brief Saving, loading and calibration commands.
 *
 * Both persistence targets are covered: the node-configuration partition and
 * the motor-profile partition. Every operation is refused while the output
 * stage is energized, because flashing stalls the single-bank CPU for several
 * PWM periods and would latch the last voltage vector on the bridge.
 *
 * The calibration handler clears the queue, drops out of the active mode and
 * de-energizes before starting an identification run.
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

inline TpError TpProtocol::handle_persistence(const TpFrame& f) {
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

inline TpError TpProtocol::handle_calibration(const TpFrame& f) {
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
