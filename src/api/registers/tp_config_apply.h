#pragma once

/**
 * @file tp_config_apply.h
 * @brief TPCP v1 - pushing the configuration shadow into the live subsystems.
 *
 * Called once at boot after the stored record is restored, and again whenever
 * a register write touches something the hardware caches: a comparator
 * threshold, a control envelope, the watchdog reload.
 *
 * Every target is reached either through a DAC threshold setter or through the
 * DriveControl mailbox, never by writing control state directly - this runs in
 * THREAD mode and the control loop owns that state.
 *
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#include "../../core/lib.h"  // TunePulseHeart and the drive mailbox
#include "../tp_node.h"      // TpNodeConfig: the shadow being applied

// ============================================================================
// Configuration -> hardware
// ============================================================================

/**
 * @brief Watchdog reload in control ticks for a timeout in seconds (0 = disabled).
 * @param tp        Live firmware state (supplies the control-loop rate).
 * @param timeout_s Timeout [s]; non-positive or NaN disables the watchdog.
 */
static inline uint32_t tp_watchdog_ticks(const TunePulseHeart& tp, float timeout_s) {
    if (!(timeout_s > 0.0f)) {
        return 0u;                                                  // Off (also catches NaN)
    }
    const float ticks = timeout_s * (float)tp.drive.frequency_hz();  // Seconds -> control passes
    return (ticks > 4.0e9f) ? 0xFFFFFFFFu : (uint32_t)ticks;        // Saturate absurd values
}

/**
 * @brief Push the configuration shadow into the live subsystems.
 *
 * Programming the analog protection thresholds while the stage is energized
 * would move a trip point under a live current, so the registers that reach
 * them all carry TP_RF_DISABLED and the engine refuses those writes unless the
 * output stage is off.
 *
 * @param tp  Live firmware state.
 * @param cfg Configuration to apply.
 */
static inline void tp_apply_config(TunePulseHeart& tp, const TpNodeConfig& cfg) {
    // Analog protection thresholds (comparator DAC references).
    tp.driver_phy.current.overcurrent.set_limit_amps()
        .oc_ia(cfg.oc_limit_a)
        .oc_ib(cfg.oc_limit_b)
        .oc_ic(cfg.oc_limit_c);
    tp.driver_phy.supply.set_limit_volts()
        .uv_lim_volts(cfg.uv_limit_v)
        .ov_lim_volts(cfg.ov_limit_v);
    tp.driver_phy.temperature.set_limit_celsius()
        .ot_lim_celsius(cfg.ot_limit_c)
        .ot_hyst_celsius(cfg.ot_hyst_c);
    tp.driver_phy.power.overpower.set_limit()
        .op_lim_watts(cfg.op_limit_w)
        .op_lim_amps(cfg.op_limit_a);

    // Control envelopes and policy: routed through the mailbox so the ISR
    // adopts them between passes instead of mid-computation. The envelope
    // can never exceed the configured peak current.
    //
    // The gearbox ratio is installed first: the configuration record is
    // written in OUTPUT-SHAFT units (what the master sees), the control loop
    // runs in MOTOR-SHAFT units, so the envelope crosses the ratio once here.
    tp.gearbox.set_ratio(cfg.gear_ratio_mul, static_cast<int16_t>(cfg.gear_ratio_div));
    const float envelope = (cfg.current_limit_a > cfg.peak_current_a) ? cfg.peak_current_a : cfg.current_limit_a;
    tp.drive.post_f32(DriveReq::CurrentLimit, envelope);                 // Currents never cross the gearbox
    tp.drive.post_f32(DriveReq::VelocityLimit, tp.gearbox.vel_ext2int(cfg.vel_limit_rps));
    tp.drive.post_f32(DriveReq::AccelLimit, tp.gearbox.acc_ext2int(cfg.accel_limit));
    tp.drive.post_f32(DriveReq::JerkLimit, tp.gearbox.jerk_ext2int(cfg.jerk_limit));
    tp.drive.post_u32(DriveReq::SetSafeState, cfg.safe_state);

    // Watchdog reload is expressed in control ticks so the ISR never divides.
    tp.drive.post_u32(DriveReq::ArmWatchdog, tp_watchdog_ticks(tp, cfg.watchdog_s));

    // Status LED: a steady or blinking colour is applied here; mode 2 leaves
    // the element to the firmware (the control loop paints faults red).
    if (cfg.led_mode != 2u) {
        tp.led.set().led(cfg.led_r, cfg.led_g, cfg.led_b);
    }
}
