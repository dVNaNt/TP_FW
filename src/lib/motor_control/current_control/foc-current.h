/**
 * @file foc-current.h
 * @brief FOC Current Controller with Feedforward Compensation and Dynamic Bandwidth
 *
 * This module implements a high-performance Field-Oriented Control (FOC) current controller
 * for PMSM and stepper motors. The controller combines model-based feedforward compensation
 * with PI feedback to achieve fast, accurate current tracking with minimal steady-state error.
 *
 * Key Features:
 * - Model-based feedforward compensation for resistive, inductive, and back-EMF terms
 * - Cross-coupling compensation between d-q axes
 * - Dynamic Bandwidth (Gain Scheduling): Fc scales with electrical velocity to reduce zero-speed noise
 * - Circular voltage limiting to preserve voltage vector angle
 * - Conditional integrator anti-windup to prevent saturation
 * - Power tracking: active electrical power, copper losses, and mechanical power estimations
 * - Regenerative braking resistor mode: motor winding used as braking resistor during recuperation
 * - Active Supply Power Limiting: Prevents exceeding maximum supply power budget
 *
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#pragma once
#include <cmath>
#include "../../common.h"

struct FocCurrentController {
    // ============================================================
    //                 Output Structure
    // ============================================================
    Vec2D_f32 VoltageDQ = {0.0, 0.0};
    bool is_saturated = false;
    // ============================================================
    //                 Controller Timing Constants
    // ============================================================
    static constexpr float kDt = 1.0f / 20000.0f;
    static constexpr float kInvDt = 20000.0f;

    // ============================================================
    //                 Motor Model Parameters
    // ============================================================
    float R = 0.0f;
    float L = 0.0f;
    MotorType motor_type = MotorType_Invalid;
    float power_scale_factor = 1.5f;

    // ============================================================
    //                 Dynamic Bandwidth (Gain Scheduling)
    // ============================================================
    float bw_min_hz = 0.0f;
    float bw_max_hz = 0.0f;
    float bw_slope_hz_per_rads = 0.0f;

    // ============================================================
    //                 PI Controller Gains
    // ============================================================
    float kp = 0.0f;
    float ki = 0.0f;
    float k_aw = 0.0f;

    // ============================================================
    //                 PI Controller State
    // ============================================================
    float integrator_d = 0.0f;
    float integrator_q = 0.0f;

    // ============================================================
    //                 Previous Step Memory
    // ============================================================
    float prev_id_target = 0.0f;
    float prev_iq_target = 0.0f;
    float prev_ud_out = 0.0f;  // Added for predictive power computation
    float prev_uq_out = 0.0f;  // Added for predictive power computation

    // ============================================================
    //                 Calculated Power Metrics [W]
    // ============================================================
    float pwr_el = 0.0f;
    float pwr_loss = 0.0f;
    float pwr_mech = 0.0f;

    // ============================================================
    //         Regenerative Braking Parameters (configurable)
    // ============================================================

    /**
     * @brief Minimum mechanical power generation threshold [W].
     *
     * If |pwr_mech| is below this value, braking injection is fully disabled.
     * This prevents ADC noise and current measurement errors from triggering
     * false braking at low speeds. Accounts for passive system consumption.
     * Default: 0.1 W (100 mW passive consumption).
     */
    float brake_pwr_threshold = 0.1f;

    /**
     * @brief Hard cap on braking D-axis current injection [A].
     *
     * Prevents thermal overload. Set to motor's continuous current rating.
     * Default: 5.0 A.
     */
    float brake_id_max_a = 6.0f;

    /**
     * @brief Maximum safe power dissipation budget [W].
     * * Limits the maximum power that can be safely dumped into the motor windings.
     * If generated power exceeds this budget, the braking torque (Iq target) is
     * proportionally throttled to prevent thermal overload or overvoltage.
     * Default: 10.0 W.
     */
    float peak_loss_pwr = 10.0f;

    float cmd_q_a = 0.0f;

    /**
     * @brief Maximum allowed current from the power supply [A].
     * Used to limit the total electrical power drawn by the controller.
     */
    float supply_current_a = 2.0f;

    /**
     * @brief Base IIR low-pass filter coefficient for power components [0..1].
     */
    float brake_filter_alpha = 0.05f;

    /**
     * @brief Voltage threshold above which the filter alpha starts increasing [V].
     */
    float brake_voltage_threshold = 48.0f;

    /**
     * @brief Voltage range over the threshold to reach maximum voltage-dependent alpha increase [V].
     */
    float brake_voltage_range = 5.0f;

    /**
     * @brief Scale factor for speed dependency in dynamic alpha calculation.
     */
    float brake_speed_scale = 0.001f;
    float target_id_ = 0.0f;
    float target_iq_ = 0.0f;

private:
    static inline float clampf(float x, float lo, float hi) { return (x < lo) ? lo : ((x > hi) ? hi : x); }

public:
    // ============================================================
    //                 Initialization
    // ============================================================

    FocCurrentController() { reset(); }

    FocCurrentController(MotorType type,
                         float phase_resistance_ohm,
                         float phase_inductance_h,
                         float max_torque_nm,
                         float max_current_a,
                         float pole_pairs,
                         float bw_min = 150.0f,
                         float bw_max = 1500.0f,
                         float bw_slope = 1.59155f) {
        motor_type = type;
        R = phase_resistance_ohm;
        L = phase_inductance_h;

        power_scale_factor = (motor_type == MotorType_STEP) ? 1.0f : 1.5f;

        bw_min_hz = bw_min;
        bw_max_hz = bw_max;
        bw_slope_hz_per_rads = bw_slope;

        k_aw = (L > 0.0f) ? ((R * 0.25f) / L) : 0.0f;

        reset();
    }

    void reset() {
        integrator_d = 0.0f;
        integrator_q = 0.0f;
        prev_id_target = 0.0f;
        prev_iq_target = 0.0f;
        prev_ud_out = 0.0f;
        prev_uq_out = 0.0f;
        pwr_el = 0.0f;
        pwr_loss = 0.0f;
        pwr_mech = 0.0f;
    }

    // ============================================================
    //                 Getters for Power Metrics
    // ============================================================
    float max_current_slew_rate_a_s = 600.0f;
    float get_pwr() const { return pwr_el; }
    float get_pwr_loss() const { return pwr_loss; }
    float get_pwr_mech() const { return pwr_mech; }

    // ============================================================
    //                 Main regulator step
    // ============================================================
    Vec2D_f32 tick(float iq_target_a,
                   float id_measured_a,
                   float iq_measured_a,
                   float bus_voltage_v,
                   float voltage_limit_fraction,
                   float omega_el_rad_s) {
        is_saturated = false;
        float id_target_a = 0.0f;
        cmd_q_a = iq_target_a;

        // --------------------------------------------------------
        // 0) Regenerative braking (generation limiting)
        // --------------------------------------------------------
        // Above-threshold generated power (positive when generating)
        // Note: pwr_mech has already been filtered on the previous step
        const float gen_pwr = -pwr_mech;
        const float demand = (gen_pwr > brake_pwr_threshold) ? (gen_pwr - brake_pwr_threshold) : 0.0f;

        if (demand > 0.0f) {
            // Priority 1: dump power through the D-axis within the safe budget
            // We dissipate as much as possible up to the peak_loss_pwr limit
            const float pwr_to_dissipate = clampf(demand, 0.0f, peak_loss_pwr);
            const float id_brake = sqrtf(pwr_to_dissipate / (power_scale_factor * R));

            // Inject current into the positive D-axis; hard-limited by the thermal budget
            id_target_a += clampf(id_brake, 0.0f, brake_id_max_a);

            // Priority 2: circular power limiting for the Q-axis (braking torque)
            // Compute the ratio (k) between actual generation and our dissipation capability
            const float k = peak_loss_pwr / gen_pwr;

            // Compute the permissible Iq amplitude based on the previous command
            // (+0.01f guarantees the algorithm can start when prev_iq was 0)
            const float max_allowed_iq_mag = (fabsf(prev_iq_target) + 0.01f) * k;

            // Clamp the requested Iq so regeneration power fits into the budget
            if (iq_target_a > 0.0f) {
                iq_target_a = clampf(iq_target_a, 0.0f, max_allowed_iq_mag);
            } else {
                iq_target_a = clampf(iq_target_a, -max_allowed_iq_mag, 0.0f);
            }
        }

        // --------------------------------------------------------
        // 0.5) Positive power limiting (consumption clamp)
        // --------------------------------------------------------
        // Maximum power available from the power supply
        const float max_supply_pwr = supply_current_a * bus_voltage_v;

        // Predictive estimate of electrical power (using previous-tick voltages)
        // While accelerating the power is positive
        const float predicted_pwr = power_scale_factor * (prev_ud_out * id_target_a + prev_uq_out * iq_target_a);

        // If the projected power exceeds the supply limit
        if (predicted_pwr > max_supply_pwr && predicted_pwr > 0.001f) {
            // Compute the current scaling factor
            const float pwr_scale = max_supply_pwr / predicted_pwr;

            // Only the torque current Iq is clamped.
            // Id is left unchanged since it may be used for field weakening
            iq_target_a *= pwr_scale;
        }

        target_id_ = id_target_a;
        target_iq_ = iq_target_a;

        // --------------------------------------------------------
        // 1) Dynamic bandwidth computation (gain scheduling)
        // --------------------------------------------------------
        float current_bw = bw_min_hz + bw_slope_hz_per_rads * fabsf(omega_el_rad_s);
        current_bw = clampf(current_bw, bw_min_hz, bw_max_hz);

        const float omega_c = 2.0f * 3.14159265358979f * current_bw;

        kp = L * omega_c;
        ki = R * omega_c * 0.25f;

        // --------------------------------------------------------
        // 2) Maximum permissible voltage-vector magnitude
        // --------------------------------------------------------
        const float u_max = bus_voltage_v * voltage_limit_fraction;
        const float u_max_sq = u_max * u_max;

        // --------------------------------------------------------
        // 3) Derivatives of the current command
        // --------------------------------------------------------
        const float did_target_dt = (id_target_a - prev_id_target) * kInvDt;
        const float diq_target_dt = (iq_target_a - prev_iq_target) * kInvDt;

        prev_id_target = id_target_a;
        prev_iq_target = iq_target_a;

        // --------------------------------------------------------
        // 4) D-axis feedforward
        // --------------------------------------------------------
        const float ud_ff_resistive = R * id_target_a;
        const float ud_ff_dynamic = L * did_target_dt;
        const float ud_ff_cross = -omega_el_rad_s * L * iq_target_a;

        // --------------------------------------------------------
        // 5) Q-axis feedforward
        // --------------------------------------------------------
        const float uq_ff_resistive = R * iq_target_a;
        const float uq_ff_dynamic = L * diq_target_dt;
        const float uq_ff_cross = omega_el_rad_s * L * id_target_a;

        // --------------------------------------------------------
        // 6) Current errors for the PI controllers
        // --------------------------------------------------------
        const float err_d = id_target_a - id_measured_a;
        const float err_q = iq_target_a - iq_measured_a;

        // --------------------------------------------------------
        // 7) PI correction summed with feedforward
        // --------------------------------------------------------
        const float ud_pi = kp * err_d + integrator_d;
        const float uq_pi = kp * err_q + integrator_q;

        const float ud_unlimited = ud_ff_resistive + ud_ff_dynamic + ud_ff_cross + ud_pi;
        const float uq_unlimited = uq_ff_resistive + uq_ff_dynamic + uq_ff_cross + uq_pi;

        float ud_out = ud_unlimited;
        float uq_out = uq_unlimited;

        // --------------------------------------------------------
        // 8) Circular voltage limit
        // --------------------------------------------------------
        const float u_sq = ud_unlimited * ud_unlimited + uq_unlimited * uq_unlimited;
        Vec2D_f32 out;
        if (u_sq > u_max_sq) {
            const float scale = u_max / sqrtf(u_sq + 1e-9f);
            ud_out *= scale;
            uq_out *= scale;
        }
        is_saturated = true;
        // --------------------------------------------------------
        // 9) Anti-windup (Back-calculation)
        // --------------------------------------------------------
        const float ud_err_aw = ud_out - ud_unlimited;
        const float uq_err_aw = uq_out - uq_unlimited;

        integrator_d += (ki * err_d + k_aw * ud_err_aw) * kDt;
        integrator_q += (ki * err_q + k_aw * uq_err_aw) * kDt;

        out.d = ud_out;
        out.q = uq_out;

        // Store voltages for predictive power control on the next tick
        prev_ud_out = ud_out;
        prev_uq_out = uq_out;

        // --------------------------------------------------------
        // 10) Power estimation and dynamic filtering
        // --------------------------------------------------------
        // Compute instantaneous power values
        const float inst_pwr_el = power_scale_factor * (ud_out * id_measured_a + uq_out * iq_measured_a);
        const float inst_pwr_loss = power_scale_factor * R * (id_measured_a * id_measured_a + iq_measured_a * iq_measured_a);

        // Dynamic filter alpha: depends on BOTH speed AND voltage
        const float voltage_excess = clampf(bus_voltage_v - brake_voltage_threshold, 0.0f, brake_voltage_range);
        const float voltage_factor = voltage_excess / brake_voltage_range;  // 0.0 to 1.0

        // Raise the filter cutoff above the voltage threshold, speed-dependent
        const float dynamic_alpha =
            clampf(brake_filter_alpha + voltage_factor * fabsf(omega_el_rad_s) * brake_speed_scale, brake_filter_alpha, 1.0f);

        // Filter both components with the same alpha
        pwr_el += dynamic_alpha * (inst_pwr_el - pwr_el);
        pwr_loss += dynamic_alpha * (inst_pwr_loss - pwr_loss);

        // Mechanical power is the difference of the filtered components
        pwr_mech = pwr_el - pwr_loss;

        return out;
    }
};