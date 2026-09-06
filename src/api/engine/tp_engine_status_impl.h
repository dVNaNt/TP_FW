#pragma once

/**
 * @file tp_engine_status_impl.h
 * @brief Collapsing firmware state into the protocol status bytes.
 *
 * The protocol reports coarse, compact values: one drive state, one control
 * mode, a warning byte, and three normalized bytes for temperature, load and
 * efficiency. These helpers are where the firmware full-resolution state is
 * reduced to them.
 *
 * All are const and read-only. Each of the normalized bytes reserves -128 as
 * an explicit "no data" marker rather than reporting a misleading zero.
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

inline uint8_t TpProtocol::drive_state() const {
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

inline uint8_t TpProtocol::temp_byte() const {
    const float t = tp_->driver_phy.temperature.get_temperature_c();
    if (t != t) {
        return 0x80u;                                    // NaN: report "no data"
    }
    return tp_pack_temp_c(t);
}

inline uint8_t TpProtocol::load_byte() const {
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

inline uint8_t TpProtocol::efficiency_byte(float p_elec) const {
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
