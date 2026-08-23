#pragma once

/**
 * @file foc_vector_synthesis.h
 * @brief Cascaded position/velocity torque-command synthesizer ("epsilon
 *        spring + damper") with a dynamic-bandwidth kinematic observer.
 *
 * MotorVectorSynthesis converts a measured position and a motion target into
 * a q-axis current command for the FOC current loop. It combines:
 *
 * 1. A third-order "ABY" position/velocity/acceleration observer driven by
 *    the measured position, whose bandwidth (cutoff frequency) adapts to the
 *    commanded velocity and is slew-rate limited for smoothness.
 * 2. An "epsilon spring": a saturating, S-curve-shaped position error term
 *    that produces full current only beyond @c epsilon_rev of error.
 * 3. A velocity damper built from a fast-tanh approximation of the estimated
 *    velocity error.
 * 4. Cascade anti-windup: when the downstream voltage loop reports saturation,
 *    the output current may only decrease relative to the last applied value,
 *    preventing integral/torque windup against a saturated plant.
 *
 * Position units are revolutions encoded as int32 Q16.16 (65536 counts =
 * 1 revolution); currents are in milliamperes.
 */

#include <stdint.h>
#include <cmath>  // для fabsf, sinf, sqrtf и т.д.
#include "common.h"

/**
 * @brief Position->current synthesis block (outer loop of the cascade).
 */
struct MotorVectorSynthesis {
    // ── Epsilon-spring parameters ─────────────────────────────────
    /**
     * Dead-band half-width of the position spring in revolutions. Within
     * +/- this error the spring command stays near zero (smooth engagement).
     */
    float epsilon_rev = 0.0075f;



    // ── Variable-bandwidth ABY observer parameters ────────────────
    float dt = 0.00005f;               ///< Observer step time [s] (50 us = 20 kHz control rate).
    float obs_fc_min_hz = 50.0f;       ///< Minimum observer cutoff frequency [Hz].
    float obs_fc_max_hz = 300.0f;      ///< Maximum observer cutoff frequency [Hz].
    /**
     * Cutoff-frequency gain versus target speed [Hz per rev/s].
     * NOTE: the historical inline comment claimed "+120 Hz per 1 rev/s"
     * while the actual coefficient is 150 Hz/(rev/s).
     */
    float obs_fc_per_rev_s = 150.0f;   // +120 Гц на 1 rev/s target_vel
    float obs_fc_slew_hz_s = 2000.0f;  ///< Slew-rate limit of the cutoff frequency [Hz/s].
    float b0_ff_rev_s2_per_ma = 0.0f;  ///< Feedforward acceleration per mA of applied current (may be 0).
    float obs_acc_limit_rev_s2 = 200000.0f; ///< Saturation limit on the estimated acceleration [rev/s^2].

    // ── Damping parameters ────────────────────────────────────────
    float damping_gain = 0.25f;  ///< Damper gain (fraction of max current at large velocity errors).
    float v_soft_rev_s = 0.25f;  ///< Softening width of the damper around zero velocity error [rev/s].

private:
    /// Scalar clamp to [lo, hi].
    static inline float clampf(float x, float lo, float hi) { return (x < lo) ? lo : ((x > hi) ? hi : x); }
    /// Floating-point sign extraction (+1 / -1 / 0).
    static inline float signf(float x) { return (x > 0.0f) ? 1.0f : ((x < 0.0f) ? -1.0f : 0.0f); }

    // ── ABY observer state ──
    float obs_pos_rev = 0.0f;     ///< Estimated rotor position [rev].
    float obs_vel_rev_s = 0.0f;   ///< Estimated rotor velocity [rev/s].
    float obs_acc_rev_s2 = 0.0f;  ///< Estimated rotor acceleration [rev/s^2].
    float obs_fc_hz = 50.0f;      ///< Current (slew-limited) observer cutoff frequency [Hz].

    /// Last actually applied q-current, used for feedforward and anti-windup clamping [mA].
    float last_applied_iq_ma = 0.0f;

    /// True until the first tick() call seeds the observer with the measurement.
    bool is_first_tick = true;

public:
    /// Default constructor.
    MotorVectorSynthesis() {}

    /// @return Estimated rotor position [rev].
    float getEstimatedPositionRev() const { return obs_pos_rev; }
    /// @return Estimated rotor velocity [rev/s].
    float getEstimatedVelocityRevS() const { return obs_vel_rev_s; }
    /// @return Estimated rotor acceleration [rev/s^2].
    float getEstimatedAccelerationRevS2() const { return obs_acc_rev_s2; }
    /// @return Current slew-limited observer cutoff frequency [Hz].
    float getObserverFcHz() const { return obs_fc_hz; }

    /**
     * @brief One synthesis step: produce the q-axis current command.
     *
     * Pipeline:
     * 1. Seed state on first call.
     * 2. Adapt the observer bandwidth toward
     *    fc_min + obs_fc_per_rev_s * |target_vel|, slew-limited.
     * 3. Integrate the third-order observer (Luenberger-style gains
     *    l1=3w, l2=3w^2, l3=w^3 derived from the cutoff w=2*pi*fc).
     * 4. Compute spring current from the saturated position error shaped by
     *    an S-curve approximation, and damping current from the fast-tanh of
     *    the normalized velocity error.
     * 5. Apply cascade anti-windup if the voltage loop is saturated, then
     *    clamp to [-max_c, +max_c] and remember the applied value.
     *
     * @param current_pos      Measured position [Q16.32 counts; 65536 = 1 rev].
     * @param target_pos       Target position in the same Q16 encoding.
     * @param target_vel       Target velocity [rev/s] (used for BW adaptation).
     * @param max_c            Maximum allowed current magnitude [mA].
     * @param is_volt_saturated Saturation flag reported by the FOC voltage loop.
     * @return Commanded q-axis current [mA], within [-max_c, +max_c].
     */
    float tick(int32_t current_pos,
                    int32_t target_pos,
                    float target_vel,
                    float max_c,
                    bool is_volt_saturated) {
        const float inv_q16 = 1.0f / 65536.0f;
        const float meas_pos_rev = (float)current_pos * inv_q16;

        // ====================================================================
        // 0. Initialization: seed the observer with the first measurement
        // ====================================================================
        if (is_first_tick) {
            obs_pos_rev = meas_pos_rev;
            obs_vel_rev_s = 0.0f;
            obs_acc_rev_s2 = 0.0f;
            obs_fc_hz = obs_fc_min_hz;
            last_applied_iq_ma = 0.0f;
            is_first_tick = false;
        }

        // ====================================================================
        // 1. Dynamic ABY observer (kinematic core)
        // ====================================================================
        // Target bandwidth grows linearly with the commanded speed magnitude.
        float fc_target_hz = obs_fc_min_hz + obs_fc_per_rev_s * fabsf(target_vel);
        fc_target_hz = clampf(fc_target_hz, obs_fc_min_hz, obs_fc_max_hz);

        // Slew-limit the bandwidth change so gains never jump abruptly.
        float max_df = obs_fc_slew_hz_s * dt;
        float df = clampf(fc_target_hz - obs_fc_hz, -max_df, max_df);
        obs_fc_hz += df;

        // Third-order pole-placement gains from the desired cutoff frequency.
        const float w = 2.0f * (float)M_PI * obs_fc_hz;
        const float l1 = 3.0f * w;
        const float l2 = 3.0f * w * w;
        const float l3 = w * w * w;

        // Optional model-based feedforward of acceleration from applied current.
        const float acc_ff = b0_ff_rev_s2_per_ma * last_applied_iq_ma;
        const float err = meas_pos_rev - obs_pos_rev;

        // Semi-implicit Euler integration of the corrected kinematic model.
        obs_pos_rev += dt * (obs_vel_rev_s + l1 * err);
        obs_vel_rev_s += dt * (obs_acc_rev_s2 + acc_ff + l2 * err);
        obs_acc_rev_s2 += dt * (l3 * err);

        obs_acc_rev_s2 = clampf(obs_acc_rev_s2, -obs_acc_limit_rev_s2, obs_acc_limit_rev_s2);

        // ====================================================================
        // 2. Control law: spring + damper
        // ====================================================================
        // Position error converted from Q16 counts to revolutions.
        float ep = (float)(target_pos - current_pos) * inv_q16;

        // Normalize the error by the epsilon dead-band and saturate to [-1, 1].
        float t_linear = clampf(ep / (epsilon_rev + 1e-9f), -1.0f, 1.0f);
        // Smooth S-curve shaping of the saturated error (Hermite smoothstep
        // t^2*(3-2t) combined with sign preservation via t*|t|). The historical
        // comment called it "sign(x)*sin(x)^2", which it is not exactly.
        float sin_pow2_fast = t_linear * fabs(t_linear) * (3.0f - 2.0f * t_linear);

        // Spring torque command: proportional to the shaped error.
        float Iq_spring = max_c * sin_pow2_fast;

        // Velocity error between the OBSERVER estimate and the target.
        float vel_err = obs_vel_rev_s - target_vel;

        // Fast tanh approximation: x/(1+|x|), bounded in (-1, 1).
        float norm_vel = vel_err / (v_soft_rev_s + 0.05f);
        float fast_tanh = norm_vel / (1.0f + fabsf(norm_vel));

        // Damper opposes the velocity error; |tanh|*tanh keeps it odd-symmetric.
        float Iq_damp = -max_c * damping_gain * fast_tanh * fast_tanh * signf(fast_tanh);

        // Total requested torque-producing current before limiting.
        float Iq_unlimited = Iq_spring + Iq_damp;
        float Iq = Iq_unlimited;

        // ====================================================================
        // 2.5 CASCADE ANTI-WINDUP (clamping) driven by the FocCurrentController
        // ====================================================================
        if (is_volt_saturated) {
            // With the voltage already at its limit the demanded current must
            // not grow further: decreasing is allowed (to brake or correct an
            // overshoot), increasing towards deeper saturation is blocked.
            if ((Iq_unlimited > 0.0f && Iq_unlimited > last_applied_iq_ma) ||
                (Iq_unlimited < 0.0f && Iq_unlimited < last_applied_iq_ma)) {
                // Trying to push more current than last step -> clamp it.
                Iq = last_applied_iq_ma;
            }
        }

        // Hard safety clamp to the allowed current range.
        Iq = clampf(Iq, -max_c, max_c);
    
        // Remember the actually applied current for observer feedforward and
        // for the anti-windup comparison on the next step.
        last_applied_iq_ma = Iq;

        return Iq;
    }
};
