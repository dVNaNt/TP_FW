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
 * - Back-EMF (BEMF) compensation: uq_bemf = omega_e * psi_f, where the flux linkage psi_f
 *   is estimated from the torque constant (holding torque / current ratio) and is exact
 *   for both 2-phase steppers and 3-phase PMSM in the synchronous dq frame
 * - Soft BEMF deadband: the compensation is gated by a forbidden-zone epsilon on
 *   |omega_e| and smoothly activated above it with a C2-continuous analog ramp,
 *   preventing limit cycles around standstill
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
                         float max_current_a,
                         float k_e = 0.0f,
                         float bw_min = 150.0f,
                         float bw_max = 1500.0f,
                         float bw_slope = 1.59155f) {
        motor_type = type;
        R = phase_resistance_ohm;
        L = phase_inductance_h;

        power_scale_factor = (motor_type == MotorType_STEP) ? 1.0f : 1.5f;
        flux_linkage_wb = k_e;


        bw_min_hz = bw_min;
        bw_max_hz = bw_max;
        bw_slope_hz_per_rads = bw_slope;

        k_aw = (L > 0.0f) ? (R / L) : 0.0f;

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
    float current_bw = 0;
    float k_cc = 1.0;
    float k_r = 1.00;
    float k_l = 1.0;
    float l_ff = 1.0;

    // ============================================================
    //                 Back-EMF Compensation
    // ============================================================

    /**
     * @brief Rotor permanent-magnet flux linkage (psi_f) [Wb].
     *
     * Estimated at construction time from the torque constant:
     *   kt = max_torque_nm / max_current_a  [Nm/A]
     *   psi_f = kt / power_scale_factor
     * Rationale:
     * - 2-phase stepper (power_scale_factor = 1.0): T = psi_f * iq  =>  psi_f = kt
     * - 3-phase PMSM (power_scale_factor = 1.5): with T = 1.5 * p * psi_f * iq
     *   and omega_el = p * omega_mech, mechanical power is
     *   P = T * omega_mech = 1.5 * psi_f * omega_el * iq, hence psi_f = kt / 1.5
     * This is a coarse estimate (ignores saturation and cogging), but sufficient
     * for feedforward BEMF cancellation. Set manually if the datasheet value
     * of psi_f (or ke_rms_line_line / sqrt(3) / omega_el) is known.
     */
    float flux_linkage_wb = 0.0f;

    /**
     * @brief BEMF compensation gain (0 disables the compensation).
     *
     * Multiplies the omega_e * psi_f feedforward term. Values below 1.0 make
     * the PI controller absorb the residual error (safer for coarse psi_f
     * estimates); values above 1.0 over-compensate.
     * Default: 1.0 (full compensation).
     */
    float k_bemf = 1.0f;

    /**
     * @brief Soft deadband (forbidden zone) on electrical speed for BEMF
     *        feedforward [rad/s].
     *
     * Below |omega_e| = bemf_omega_deadband the BEMF compensation is fully
     * disabled (gain = 0). Rationale: near standstill the PLL omega estimate
     * is dominated by quantization/measurement noise; feeding it back through
     * the BEMF term closes a limit-cycle loop
     * (omega noise -> uq -> torque -> shaft -> encoder -> PLL). Inside the
     * forbidden zone the (tiny, speed-proportional) residual EMF is absorbed
     * by the PI integrator instead, which does not create a positive feedback
     * path. Default: 5.0 rad/s.
     */
    float bemf_omega_deadband = 5.0f;

    /**
     * @brief Width of the smooth activation ramp above the deadband [rad/s].
     *
     * The BEMF gain ramps from 0 (at |omega_e| = bemf_omega_deadband) to 1
     * (at |omega_e| = bemf_omega_deadband + bemf_omega_soft_range) using a
     * C2-continuous smootherstep curve, so the feedforward voltage has no
     * step or kink in either value or slope as the motor crosses the zone
     * boundary. A hard (step) activation here would itself excite limit
     * cycles via torque kicks at every boundary crossing.
     * Set to 0 for a hard deadband (not recommended).
     * Default: 10.0 rad/s.
     */
    float bemf_omega_soft_range = 10.0f;

    /**
     * @brief C2-continuous analog activation window (smootherstep).
     *
     * Maps |omega| to a gain in [0, 1]:
     *   |omega| <= deadband                          -> 0 (fully inhibited)
     *   deadband < |omega| < deadband + soft_range   -> 6t^5 - 15t^4 + 10t^3,
     *                                                   t = (|omega| - deadband) / soft_range
     *   |omega| >= deadband + soft_range             -> 1 (fully active)
     *
     * The smootherstep polynomial (Perlin, 2002) is chosen over linear or
     * 3t^2-2t^3 (smoothstep) ramps because its first AND second derivatives
     * vanish at both ends: the BEMF feedforward - and therefore the torque -
     * is continuous in value, slope and curvature across the activation
     * region, eliminating excitation of mechanical resonance when crossing
     * the zone boundary. Complexity: O(1), 4 FMA-class operations.
     *
     * @param abs_omega  Absolute electrical speed [rad/s] (caller applies fabsf).
     * @param deadband   Forbidden-zone threshold epsilon [rad/s].
     * @param soft_range Width of the activation ramp [rad/s] (guarded against <= 0).
     * @return Activation gain in [0, 1].
     */
    static inline float bemf_activation_gain(float abs_omega, float deadband, float soft_range) {
        const float range = (soft_range > 1e-6f) ? soft_range : 1e-6f;              // Guard: hard deadband fallback
        const float t = clampf((abs_omega - deadband) / range, 0.0f, 1.0f);         // Normalized ramp position
        return t * t * t * (t * (t * 6.0f - 15.0f) + 10.0f);                        // Smootherstep 6t^5-15t^4+10t^3
    }

    float ubemf = 0;
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

        // // --------------------------------------------------------
        // // 0) Regenerative braking (generation limiting)
        // // --------------------------------------------------------
        // // Above-threshold generated power (positive when generating)
        // // Note: pwr_mech has already been filtered on the previous step
        // const float gen_pwr = -pwr_mech;
        // const float demand = (gen_pwr > brake_pwr_threshold) ? (gen_pwr - brake_pwr_threshold) : 0.0f;

        // if (demand > 0.0f) {
        //     // Priority 1: dump power through the D-axis within the safe budget
        //     // We dissipate as much as possible up to the peak_loss_pwr limit
        //     const float pwr_to_dissipate = clampf(demand, 0.0f, peak_loss_pwr);
        //     const float id_brake = sqrtf(pwr_to_dissipate / (power_scale_factor * R));

        //     // Inject current into the positive D-axis; hard-limited by the thermal budget
        //     id_target_a += clampf(id_brake, 0.0f, brake_id_max_a);

        //     // Priority 2: circular power limiting for the Q-axis (braking torque)
        //     // Compute the ratio (k) between actual generation and our dissipation capability
        //     const float k = peak_loss_pwr / gen_pwr;

        //     // Compute the permissible Iq amplitude based on the previous command
        //     // (+0.01f guarantees the algorithm can start when prev_iq was 0)
        //     const float max_allowed_iq_mag = (fabsf(prev_iq_target) + 0.01f) * k;

        //     // Clamp the requested Iq so regeneration power fits into the budget
        //     if (iq_target_a > 0.0f) {
        //         iq_target_a = clampf(iq_target_a, 0.0f, max_allowed_iq_mag);
        //     } else {
        //         iq_target_a = clampf(iq_target_a, -max_allowed_iq_mag, 0.0f);
        //     }
        // }

        // // --------------------------------------------------------
        // // 0.5) Positive power limiting (consumption clamp)
        // // --------------------------------------------------------
        // // Maximum power available from the power supply
        // const float max_supply_pwr = supply_current_a * bus_voltage_v;

        // // Predictive estimate of electrical power (using previous-tick voltages)
        // // While accelerating the power is positive
        // const float predicted_pwr = power_scale_factor * (prev_ud_out * id_target_a + prev_uq_out * iq_target_a);

        // // If the projected power exceeds the supply limit
        // if (predicted_pwr > max_supply_pwr && predicted_pwr > 0.001f) {
        //     // Compute the current scaling factor
        //     const float pwr_scale = max_supply_pwr / predicted_pwr;

        //     // Only the torque current Iq is clamped.
        //     // Id is left unchanged since it may be used for field weakening
        //     iq_target_a *= pwr_scale;
        // }

        target_id_ = id_target_a;
        target_iq_ = iq_target_a;

        // --------------------------------------------------------
        // 1) Dynamic bandwidth computation (gain scheduling)
        // --------------------------------------------------------
        float current_bw_ = bw_min_hz + bw_slope_hz_per_rads * fabsf(omega_el_rad_s);
        current_bw = clampf(current_bw_, bw_min_hz, bw_max_hz);

        const float omega_c = 2.0f * 3.14159265358979f * current_bw;

        kp = L * omega_c * k_l;
        ki = R * omega_c * k_r;

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
        const float ud_ff_dynamic = L * (did_target_dt * l_ff);
        const float ud_ff_cross = -omega_el_rad_s * L * iq_target_a;

        // --------------------------------------------------------
        // 5) Q-axis feedforward
        // --------------------------------------------------------
        const float uq_ff_resistive = R * iq_target_a;
        const float uq_ff_dynamic = L * (diq_target_dt * l_ff);
        const float uq_ff_cross = omega_el_rad_s * L * id_target_a;
        // Back-EMF feedforward: the permanent-magnet flux cutting the stator
        // windings induces a q-axis EMF of omega_e * psi_f [V]. Since this term
        // is a pure disturbance on the q axis (it does not depend on the
        // currents), canceling it in feedforward removes the speed-proportional
        // lag the PI integrator would otherwise have to wind up against.
        // Sign: the EMF opposes the applied voltage, so the compensation term
        // carries the same sign as omega (it "adds" voltage into rotation).
        //
        // Soft deadband (anti-limit-cycle): the raw PLL omega is noisy at low
        // speed, and feeding that noise straight into the q-axis voltage closes
        // the positive feedback loop
        //   omega noise -> uq_ff_bemf -> torque -> shaft motion -> encoder -> PLL,
        // which manifests as self-oscillation (limit cycle) around standstill.
        // The compensation is therefore gated by a forbidden zone of width
        // bemf_omega_deadband on |omega_e| and smoothly activated above it via
        // the C2-continuous smootherstep ramp. Multiplying omega by the gain
        // (instead of gating the final voltage) preserves the odd symmetry of
        // the term: gain(|omega|) is even, omega is odd, so the product is odd
        // and passes through zero exactly at omega = 0 with no sign flip.
        // Inside the zone the residual EMF (omega_e * psi_f, tiny at low speed)
        // is absorbed by the PI integrator, which has no such feedback path.
        const float abs_omega_el = fabsf(omega_el_rad_s);                                            // Magnitude for the gating decision
        const float bemf_gain = bemf_activation_gain(abs_omega_el, bemf_omega_deadband, bemf_omega_soft_range);  // 0..1 analog activation
        const float uq_ff_bemf = omega_el_rad_s * bemf_gain * flux_linkage_wb * k_bemf;              // Noise-gated EMF cancellation
        ubemf = uq_ff_bemf;
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

        const float ud_unlimited = ud_ff_resistive + ud_ff_dynamic + ud_ff_cross * k_cc + ud_pi;
        const float uq_unlimited = uq_ff_resistive + uq_ff_dynamic + uq_ff_cross * k_cc + uq_ff_bemf + uq_pi;

        float ud_out = ud_unlimited;
        float uq_out = uq_unlimited;

        // --------------------------------------------------------
        // 8) Circular voltage limit
        // --------------------------------------------------------
        const float u_sq = ud_unlimited * ud_unlimited + uq_unlimited * uq_unlimited;
        Vec2D_f32 out;
        if (u_sq > u_max_sq) {
            const float scale = u_max / sqrtf(u_sq + 1e-12f);
            ud_out = ud_unlimited * scale;
            uq_out = uq_unlimited * scale;
            is_saturated = true;
        } else {
            is_saturated = false;
        }
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

        // // --------------------------------------------------------
        // // 10) Power estimation and dynamic filtering
        // // --------------------------------------------------------
        // // Compute instantaneous power values
        // const float inst_pwr_el = power_scale_factor * (ud_out * id_measured_a + uq_out * iq_measured_a);
        // const float inst_pwr_loss = power_scale_factor * R * (id_measured_a * id_measured_a + iq_measured_a * iq_measured_a);

        // // Dynamic filter alpha: depends on BOTH speed AND voltage
        // const float voltage_excess = clampf(bus_voltage_v - brake_voltage_threshold, 0.0f, brake_voltage_range);
        // const float voltage_factor = voltage_excess / brake_voltage_range;  // 0.0 to 1.0

        // // Raise the filter cutoff above the voltage threshold, speed-dependent
        // const float dynamic_alpha =
        //     clampf(brake_filter_alpha + voltage_factor * fabsf(omega_el_rad_s) * brake_speed_scale, brake_filter_alpha, 1.0f);

        // // Filter both components with the same alpha
        // pwr_el += dynamic_alpha * (inst_pwr_el - pwr_el);
        // pwr_loss += dynamic_alpha * (inst_pwr_loss - pwr_loss);

        // // Mechanical power is the difference of the filtered components
        // pwr_mech = pwr_el - pwr_loss;

        return out;
    }
};