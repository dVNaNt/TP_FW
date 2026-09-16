/**
 * @file current_control.h
 * @brief High-level current-control motor driver object.
 *
 * Aggregates the physical driver (DriverPWM), the FOC current controller
 * (foc-current.h), the motor description (Motor) and the calibration state
 * machines into the single application-facing driver used by TunePulseHeart.
 *
 * ## Angular velocity estimation
 *
 * The electrical angular velocity fed to FocCurrentController is produced by
 * a second-order Type-II PLL (FilterPLL, math/filters/pll.h) tracking the
 * input angle at the fixed FOC tick rate. This replaces the raw differentiation
 * of quantized angle samples, which was dominated by quantization noise.
 * The PLL bandwidth can be retuned at runtime via CurrentControl::pll_.set_fc().
 *
 * ## Overcurrent protection interaction
 *
 * While the physical layer reports an active overcurrent (OC) fault,
 * tick_control_foc() bypasses regulation entirely: the FOC controller state
 * (PI integrators, feedforward step memory, power filters) is reset and a
 * zero voltage vector is emitted. This prevents the integrators from winding
 * up against the Hi-Z outputs during the fault and eliminates the recovery
 * transient when the drive returns to active operation.
 *
 * ## Electrical angle prediction (transport-delay compensation)
 *
 * The phase currents are sampled by the ADC at the start of a PWM period,
 * while the voltage vector computed from them is loaded into the PWM
 * registers and only reaches the bridge at the center of the NEXT period.
 * The whole control path therefore lags the rotor by 1.5 PWM periods, which
 * at speed produces a growing d/q cross-coupling error (the applied vector
 * falls behind the rotor and drifts toward the d axis).
 *
 * tick_control_foc() compensates this by advancing the inverse-Park
 * (output) angle by omega_e * 1.5 * T_pwm, taken from the PLL velocity
 * estimate. The forward Park transform for the measured currents keeps the
 * raw sampled angle: it must stay matched to the actual sampling instant.
 * The prediction is applied only in the closed-loop FOC branch (never
 * during the electrical identification, which steers the angle itself) and
 * is suppressed while the PLL is coasting through a glitch run, when its
 * velocity estimate is stale.
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#pragma once
// 3-phase PWM driver with FOC (Field Oriented Control) current regulation

#include <cmath>  // For sqrtf and fabsf
#include "../../common.h"
#include "../../phy/pwm_driver/driver_pwm.h"
#include "../../math/filters/pll.h"                  // Second-order PLL for smooth angular velocity
#include "../../math/helpers/frequency_generator.h"  // Frequency generator for HFI
#include "../../math/helpers/stability_check.h"      // Stability check for current monitoring
#include "../../math/helpers/timeout_timer.h"        // Timeout timer for countdown operations
#include "../motor.h"                                // Motor parameters structure
#include "foc-current.h"                             // FOC current controller with feedforward
#include "hfi_sense.h"                               // HFI sense for inductance measurement
#include "motor_calibration.h"

static constexpr float test_pwr = 10.0;
struct CurrentControl {
public:
    Motor& motor;
    DriverStatus setup_status;
    MotorStatus motor_calib_status;
    FilterMED3 med3_id_;
    FilterMED3 med3_iq_;
    FilterEMA_f32 lpf_id_;
    FilterEMA_f32 lpf_iq_;
    FilterAdaptiveAlphaEMA_f32 lpf_adaptive_alpha_ = FilterAdaptiveAlphaEMA_f32(20000, 500, 3500);
    FocCurrentController foc_current_;
    FilterPLL pll_;                                        // Angle tracker: smooth omega instead of raw quantized differentiation
    static constexpr float CTRL_SAMPLE_FREQ = 20000.0f;    // FOC tick rate [Hz] (must match FocCurrentController::kInvDt)
    static constexpr float PLL_FC_DEFAULT = 250.0f;        // Default PLL bandwidth [Hz]
    static constexpr float TWO_PI_F = 6.283185307179586f;  // [rad] full turn
    /**
     * Transport delay between current sampling and voltage application,
     * expressed in PWM periods. The ADC samples at the start of a period and
     * the computed vector is applied at the center of the next one, so the
     * loop sees 1.5 periods of lag; the output angle is advanced by exactly
     * that much to land the voltage on the rotor position it was computed for.
     */
    static constexpr float PWM_ADVANCE_PERIODS = 1.5f;
    static constexpr uint32_t CALIBRATION_ZERO_CYCLES = 50;
    MotorCalibration motor_calib;
    int16_t angle_ = 0;
    timeout_timer driver_timeout;
    float power = 0;
    Vec2D_f32 current_dq = {0, 0};
    Vec2D_f32 current_ab = {0, 0};
    Vec2D_f32 voltage_ab_ = {0, 0};
    Vec2D_f32 voltage_dq_ = {0, 0};
    bool is_saturated = false;
    DriverPWM& drv;

    CurrentControl(Motor& motor_ref, DriverPWM& drv)
        : motor(motor_ref),
          setup_status(),
          motor_calib_status(MotorStatus::NotCalibrated),
          driver_timeout(CALIB_PAUSE_CYCLES),
          drv(drv),
          motor_calib(drv),
          pll_(CTRL_SAMPLE_FREQ, PLL_FC_DEFAULT) {}

    MotorStatus get_motor_status() const { return motor.get_status(); }
    OperationStatus get_driver_status() const { return setup_status.get_driver_status(); }

    int16_t get_angle() const { return angle_; }

    void start_motor_calibration() {
        // if (setup_status.get_driver_status() != OperationStatus::Ready)
        //     return;
        motor_calib.start_calibration(motor, motor_calib_status, angle_, test_pwr);
    }

    MotorStatus get_motor_calibration_status() const { return motor_calib_status; }

    /**
     * @brief True while the electrical identification owns the bridge.
     *
     * Covers every stage of MotorCalibration, including a terminal CalibError:
     * in that state the routine still answers every tick and the FOC loop is
     * not to be trusted, so the caller keeps routing passes here rather than
     * to closed-loop control.
     */
    bool is_identifying() const { return motor_calib_status != MotorStatus::Success; }

    /**
     * @brief Re-derive the FOC current-loop gains from the Motor profile.
     *
     * The loop is tuned from the winding resistance and inductance once, when
     * the identification finishes. A profile edited afterwards - R or L
     * written through the register map - would otherwise leave the loop on
     * its old gains until the next identification. Refused while an
     * identification is running, because that routine re-tunes on completion
     * from the values it measures itself.
     *
     * Thread-mode callers must hold the drive disabled: the controller object
     * is rebuilt in place, and a pass that regulated with a half-built one
     * would command garbage.
     *
     * @return true when the loop was re-tuned.
     */
    bool retune() {
        if (is_identifying()) {
            return false;                    // The identification will tune on its own completion
        }
        init_foc_controller();
        return true;
    }

    /**
     * @brief Leave the CalibError state a failed identification parked the
     *        drive in, keeping the last known motor parameters.
     *
     * A failed runtime identification reports Success for nothing afterwards:
     * is_identifying() stays true, the closed loop is locked out and every
     * enable is refused until a reboot. The operator's fault reset calls this
     * to make the drive usable again with the parameters it already had (the
     * routine only publishes a value once its own pass has measured it, so a
     * failure leaves the previous ones in place). The current controller is
     * re-initialised from those parameters. A new identification may be
     * requested afterwards. No effect unless the drive is in CalibError.
     */
    void recover_identification() {
        if (motor_calib_status != MotorStatus::CalibError) {
            return;                                   // Nothing to recover from
        }
        motor_calib_status = MotorStatus::Success;    // Identification no longer owns the bridge
        motor.set_status(MotorStatus::Success);       // Profile reports a usable motor again
        init_foc_controller();                        // Gains from the parameters we still have; status Ready
    }

    /**
     * @brief Abort a running identification (see MotorCalibration::abort()).
     * @param reason Short reason for the console dump.
     */
    void abort_identification(const char* reason, uint8_t fault_bits = 0) {
        if (is_identifying() && (motor_calib_status != MotorStatus::CalibError)) {
            motor_calib.abort(motor_calib_status, motor, setup_status, reason, fault_bits);
        }
    }

private:
    void init_foc_controller() {
        // All control constants come from the Motor parameter structure
        // (identified values, or flash-restored ones when available).
        float R_ohm = motor.get_resistance();
        float L_h = motor.get_effective_inductance();
        float torque_nm = motor.get_max_torque();
        const float rated_current_a = 1.5f;

        // Fallbacks for parameters not yet identified/stored.
        if (torque_nm <= 0.0f) {
            torque_nm = 0.42f;
        }
        float k_bemf = torque_nm / (50 * rated_current_a);
        foc_current_ = FocCurrentController(drv.motor_ctrl.get_motor_type(), R_ohm, L_h, rated_current_a, k_bemf);
        setup_status.set_driver_status(OperationStatus::Ready);
    }

public:
    int16_t angular_speed = 0;
    float fc_pll = PLL_FC_DEFAULT;
    float fc_pll_prev = PLL_FC_DEFAULT;
    Vec2D_f32 tick_control_foc(int16_t angle, float target_current) {
        // The electrical identification steers the angle itself (a fixed test
        // vector, then a slow rotation), so the caller's angle is only adopted
        // while no identification is running. Keyed on the identification
        // state rather than on the Motor status: after boot the Motor reports
        // Success for good, so an identification started later was handed the
        // encoder angle every pass and measured against a moving vector.
        if (!is_identifying())
            angle_ = angle;

        // Track the angle with the second-order PLL. It provides a smooth,
        // quantization-noise-free angular velocity estimate (omega) together
        // with the filtered angle, instead of differentiating raw quantized
        // samples. angular_speed is derived back to LSB-per-tick units so the
        // adaptive-alpha LPF keeps its original scaling.
        if (fc_pll != fc_pll_prev) {
            pll_.set_fc(fc_pll);
            fc_pll_prev = fc_pll;
        }

        pll_.tick(angle_);
        angular_speed = (int16_t)(pll_.get_omega() * 65536.0f / CTRL_SAMPLE_FREQ);

        // Overcurrent guard: while the physical-layer OC protection is
        // engaged, the PWM outputs are forced into HiZ, so regulating against
        // them would wind up the PI integrators against an invisible load and
        // produce a violent torque kick on fault release. Instead, freeze and
        // reset the entire FOC state machine and emit zero voltage; the
        // controller resumes from a clean state once the fault clears.
        if (drv.get_fault()) {
            foc_current_.reset();                 // Zero d/q integrators, step memory and power filters
            voltage_dq_ = Vec2D_f32(0.0f, 0.0f);  // Invalidate stored voltage used by the power estimate
            is_saturated = false;                 // Clear saturation flag (no regulation happened)
            return Vec2D_f32(0.0f, 0.0f);         // Zero alpha-beta voltage command
        }

        // Preparing the angles for the Park / inverse-Park transforms. The
        // measured-current Park transform always uses the raw sampled angle;
        // the output (inverse-Park) rotation defaults to it too, but the
        // closed-loop FOC branch below advances it by the transport delay.
        Pol2D angle_polar = Pol2D(angle_, INT16_MAX);
        Vec2D angle_sincos = angle_polar.convert();
        Vec2D angle_sincos_inv(angle_sincos.cos, -angle_sincos.sin);
        Vec2D out_sincos = angle_sincos;  // Rotation applied to the voltage vector

        current_ab = drv.motor_ctrl.get_current_ab_amps();

        Vec2D_f32 current_dq_raw = current_ab.rotate(angle_sincos_inv);
        current_dq = current_dq_raw;

        Vec2D_f32 voltage_dq(0, 0);

        switch (setup_status.get_driver_status()) {
            case OperationStatus::Ready: {
                if (motor_calib_status == MotorStatus::Success) {
                    constexpr float ANGLE_COEF = 3.14159f / 32768.0f;

                    // Electrical angular velocity from the PLL: PLL omega is in
                    // mechanical turns/s -> multiply by 2*pi.
                    float omega_e_rad_s = pll_.get_omega() * TWO_PI_F;
                    float voltage_limit_fraction = 0.50f;
                    float theta_el_rad = (float)angle_ * ANGLE_COEF;

                    // Predict the electrical angle for the instant the voltage
                    // actually reaches the bridge: currents were sampled at the
                    // start of this PWM period, the vector computed here is
                    // applied at the center of the next one, so the output
                    // angle is advanced by omega_e * 1.5 * T_pwm. In 65536-LSB
                    // per electrical turn units the advance is
                    // omega[turn/s] * 65536 * 1.5 / 20000; the int16 addition
                    // wraps across the turn boundary automatically. Suppressed
                    // while the PLL is coasting through a glitch run: its
                    // omega is stale then, so the raw angle is safer.
                    float advance_lsb = 0.0f;                          // Angle advance [LSB]
                    if (pll_.is_locked()) {
                        advance_lsb = pll_.get_omega() * 65536.0f * PWM_ADVANCE_PERIODS / CTRL_SAMPLE_FREQ;
                    }
                    Pol2D pred_polar = Pol2D((int16_t)(angle_ + (int32_t)lrintf(advance_lsb)), INT16_MAX);
                    out_sincos = pred_polar.convert();                 // Predicted output rotation

                    voltage_dq = foc_current_.tick(target_current, current_dq_raw.flux, current_dq_raw.torque,
                                                   drv.supply.get_voltage(), voltage_limit_fraction, omega_e_rad_s);

                    is_saturated = foc_current_.is_saturated;

                    break;
                }

                uint8_t alpha = lpf_adaptive_alpha_.tick(abs(angular_speed));
                lpf_id_.set_alpha(alpha);
                lpf_iq_.set_alpha(alpha);

                // int16_t filtered_id = med3_id_.tick(current_dq_raw.flux);
                // int16_t filtered_iq = med3_iq_.tick(current_dq_raw.torque);

                float filtered_id = lpf_id_.tick_lpf(current_dq_raw.flux);
                float filtered_iq = lpf_iq_.tick_lpf(current_dq_raw.torque);

                Vec2D_f32 current_dq_filt = Vec2D_f32(filtered_id, filtered_iq);

                Vec2D_f32 current_ab_filt = current_dq_filt.rotate(angle_sincos);
                MotorStatus prev_status = motor_calib_status;

                motor_calib.tick_calibration(drv.supply.get_voltage(), drv.power.get_power_w(), current_ab_filt.a,
                                             current_ab_filt.b, current_dq_raw.flux, current_dq_raw.torque, motor_calib_status,
                                             motor, setup_status, angle_);

                if (prev_status != MotorStatus::Success && motor_calib_status == MotorStatus::Success) {
                    init_foc_controller();
                } else if (prev_status != MotorStatus::CalibError && motor_calib_status == MotorStatus::CalibError) {
                    voltage_ab_ = Vec2D_f32(0.0, 0.0);
                }

                voltage_dq = Vec2D_f32(motor_calib.get_voltage_d(), motor_calib.get_voltage_q());
                break;
            }

            case OperationStatus::DriverCalibration:

                setup_status.current_not_calibrated = 0;
                setup_status.current_sensors_faulty = 0;
                setup_status.set_driver_status(OperationStatus::Ready);
                start_motor_calibration();

                break;

            default:
                return Vec2D_f32(NAN, NAN);
        }

        voltage_dq_ = voltage_dq;
        return voltage_dq.rotate(out_sincos);  // Rotate to the (predicted) output angle
    }
};