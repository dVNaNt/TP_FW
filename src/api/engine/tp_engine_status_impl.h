#pragma once

/**
 * @file tp_engine_status_impl.h
 * @brief Collapsing firmware state into the protocol's status values.
 *
 * The protocol reports coarse, compact values: one drive state, one control
 * mode, a warning byte, and a few normalized fractions for temperature, load
 * and following error. These helpers are where the firmware's
 * full-resolution state is reduced to them.
 *
 * All are const and read-only, and every normalized value reserves an
 * explicit "no data" marker rather than reporting a misleading zero.
 *
 * Part of TpProtocol, declared in tp_engine.h.
 *
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#include "../tp_engine.h"  // class TpProtocol - the declarations these bodies define

inline uint8_t TpProtocol::drive_state() const {
    if (reboot_pending_) {
        return TP_STATE_REBOOTING;                       // Reset already armed
    }
    if (tp_->driver_phy.get_fault() != 0u) {
        return TP_STATE_FAULT;                           // A latched fault wins over everything
    }
    if (config_.node_id == TP_NODE_UNASSIGNED) {
        return TP_STATE_UNADDRESSED;                     // Waiting to be found by the master
    }
    if (!tp_->driver_phy.current.is_calibrated()) {
        return TP_STATE_BOOT;                            // Current-sense offsets unknown
    }
    if (!tp_->angle_calibrator.is_calibrated() || tp_->current_driver.is_identifying() ||
        (tp_->motor_status != MotorStatus::Success)) {
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

inline uint8_t TpProtocol::control_mode() const {
    DriveTelemetry t;                                    // Live control snapshot
    tp_->drive.read_telemetry(t);
    return t.mode & 0x0Fu;
}

inline uint8_t TpProtocol::warn_flags() const {
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
    if ((t.dropped_segments != 0u) || (seg_dropped_ != 0u)) {
        w |= TP_WARN_QUEUE_DROP;                         // A segment was rejected or lost
    }
    if ((bus_dropped_ != 0u) || ((bus_flags_ & 0x03u) != 0u)) {
        w |= TP_WARN_COMM;                               // Frames lost or the bus is degraded
    }
    const float temp = tp_->driver_phy.temperature.get_temperature_c();
    if (temp > config_.ot_warn_c) {
        w |= TP_WARN_TEMPERATURE;                        // Above the warning threshold
    }
    const float v = tp_->driver_phy.supply.get_voltage();
    if ((v < (config_.uv_limit_v + config_.supply_hyst_v)) || (v > (config_.ov_limit_v - config_.supply_hyst_v))) {
        w |= TP_WARN_SUPPLY;                             // Inside the hysteresis band of a rail limit
    }
    return w;
}

inline float TpProtocol::motor_temperature() const {
    // No winding sensor exists: the temperature is estimated from the copper
    // loss through the configured thermal resistance, on top of the driver
    // temperature. Without a model there is honestly no data.
    if (!(config_.motor_rth > 0.0f)) {
        return __builtin_nanf("");
    }
    const float r = tp_->motor_params.get_resistance();  // Identified winding resistance
    const float i = tp_->last_current_a_;                // Applied q-axis current
    const float loss = i * i * r;                        // Copper loss [W]
    return tp_->driver_phy.temperature.get_temperature_c() + (loss * config_.motor_rth);
}

inline float TpProtocol::load_ratio() const {
    if (tp_->driver_phy.get_fault() != 0u) {
        return 2.0f;                                     // Beyond any limit: reported as overload
    }
    const float lim = (config_.current_limit_a > 0.0f) ? config_.current_limit_a : 1.0f;  // Envelope
    return tp_->last_current_a_ / lim;                   // Signed fraction of the envelope
}

inline float TpProtocol::usage_ratio() const {
    // The consolidated usage is the tightest of the current and power
    // limits, signed by the direction of the power flow (regeneration is
    // negative).
    float usage = load_ratio();                          // Current against the envelope
    if (usage < 0.0f) {
        usage = -usage;
    }
    const float p = tp_->driver_phy.power.get_power_w(); // Signed bus power
    const float p_lim = (config_.op_limit_w > 0.0f) ? config_.op_limit_w : 1.0f;  // Overpower trip
    const float p_ratio = ((p < 0.0f) ? -p : p) / p_lim; // Power against its trip
    if (p_ratio > usage) {
        usage = p_ratio;
    }
    return (p < 0.0f) ? -usage : usage;
}

inline float TpProtocol::following_error_ratio() const {
    DriveTelemetry t;                                    // Live control snapshot
    tp_->drive.read_telemetry(t);
    const float err = (float)(t.target_pos_q16 - t.position_q16) * (1.0f / 65536.0f);  // [rev]
    const float eps = tp_->position_controller.epsilon_rev;  // Spring saturation width
    return (eps > 0.0f) ? (err / eps) : 0.0f;
}

inline bool TpProtocol::is_enabled() const {
    if (tp_ == nullptr) {
        return false;                                    // Not bound to a drive
    }
    DriveTelemetry t;                                    // Live control snapshot
    tp_->drive.read_telemetry(t);
    return (t.flags & DRIVE_TLM_ENABLED) != 0u;
}

inline void TpProtocol::feed_watchdog() {
    if (tp_ != nullptr) {
        tp_->drive.feed_watchdog();                      // One 32-bit store, ISR-safe
    }
}
