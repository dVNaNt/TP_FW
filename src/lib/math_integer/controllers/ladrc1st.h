/**
 * @file ladrc_1st.h
 * @brief First‑order Linear Active Disturbance Rejection Control (LADRC) module
 *
 * Implements a first‑order LADRC controller with a second‑order Linear Extended
 * State Observer (LESO) for single‑input single‑output systems that exhibit
 * dominating first‑order behavior (e.g. motor speed loops, temperature loops,
 * voltage regulators). The controller internally estimates the total disturbance
 * (including plant nonlinearities and external load changes) and compensates it
 * via feedforward, while the outer control law behaves like a PI‑type controller
 * with a closed‑loop pole at -wc.
 *
 * Algorithm:
 * 1. Plant model: ẏ = b0·u + f(t), where f(t) is the generalized disturbance.
 * 2. Expansion: z1 = y, z2 = f(t) (extended state).
 * 3. LESO (second‑order Luenberger observer):
 *    ż1 = β1·(y − z1) + z2 + b0·u
 *    ż2 = β2·(y − z1)
 *    with β1 = 2·ωo and β2 = ωo² for the second‑order LESO.
 * 4. Control law (equivalent to PI on normalized integrator):
 *    v = kp·e + ki·∫e dt, e = r − z1
 *    u = (v − z2)/b0
 *    where kp = 2·ωc, ki = ωc² so that the closed‑loop behaves like 1/(s+ωc).
 *
 * Requirements:
 * - The sampling period `dt` must be fixed and known a‑priori.
 * - The nominal plant gain `b0` should be chosen close to the working‑point gain.
 * - Typical tuning: ωo ≈ 3–5·ωc for robust disturbance estimation.
 *
 * Usage:
 * 1. Instantiate LADRC1st with dt and initial bandwidths.
 * 2. Optionally call set_b0() if the nominal gain changes.
 * 3. Each control cycle call tick(ref, y_meas, u_limit) to get the control output.
 *
 * Dependencies:
 * - None (self‑contained, using basic float math).
 * - Designed for embedded C++ environments (STM32‑style, fixed‑rate motor control).
 *
 * References:
 * - G. Herbst, "A Simulative Study on Active Disturbance Rejection Control",
 *   arXiv:1908.04596, 2019.
 * - "First‑order LADRC Notes & Code" (CSDN, 2023).
 *
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#pragma once

#include <cmath>

/**
 * @brief First‑order Linear Active Disturbance Rejection Controller (LADRC1st)
 *
 * Encapsulates a first‑order LADRC controller with a second‑order Linear Extended
 * State Observer (LESO). The controller takes a reference and a measured output
 * and returns a control signal u, while internally estimating the generalized
 * disturbance and compensating for it. This structure is suitable for motor speed
 * loops, DC‑link regulators, or any SISO system with first‑order‑like behavior.
 *
 * Key parameters:
 * - wc: controller bandwidth (rad/s), sets closed‑loop speed (pole ≈ -wc).
 * - wo: observer bandwidth (rad/s), governs disturbance estimation speed.
 * - b0: nominal plant gain (u→ẏ), used both for LESO and the control law.
 * - dt: constant sampling period (seconds), fixed at construction.
 *
 * States:
 * - z1: estimated output (≈ y).
 * - z2: estimated generalized disturbance (≈ f(t)).
 * - xi: integral of tracking error e = ref - z1 (for PI‑like action).
 */
struct LADRC1st {
    // Public tunable parameters
    float wc;        ///< Controller bandwidth (rad/s)
    float wo;        ///< Observer bandwidth (rad/s)
    float b0;        ///< Nominal plant gain (u → ẏ)
    const float dt;  ///< Fixed sampling period (seconds)

    // Internal observer states
    float z1;  ///< Estimated output (≈ y)
    float z2;  ///< Estimated generalized disturbance

    // Internal controller state (integral of error)
    float xi;  ///< xi = ∫e dt, where e = ref - z1

    // Precomputed gains for efficiency (b0, wc, wo → beta1, beta2, kp, ki)
    float beta1;  ///< ESO gain: β1 = 2·wo
    float beta2;  ///< ESO gain: β2 = wo²
    float kp;     ///< Proportional gain: kp = 2·wc
    float ki;     ///< Integral gain: ki = wc²

    /**
     * @brief Constructor
     * @param dt_ Sampling period in seconds (fixed)
     * @param wc_ Controller bandwidth (rad/s, default 100)
     * @param wo_ Observer bandwidth (rad/s, default 300)
     * @param b0_ Nominal plant gain (default 1.0)
     *
     * Initializes controller and observer gains from bandwidths and b0,
     * and zeros all internal states.
     */
    LADRC1st(float dt_, float wc_, float wo_, float b0_)
        : wc(wc_), wo(wo_), b0(b0_), dt(dt_), z1(0.0f), z2(0.0f), xi(0.0f) {
        updateGains();
    }

    /**
     * @brief Reset all dynamic states
     * @param y0 Optional initial estimated output (defaults to 0.0)
     *
     * Sets z1 = y0, z2 = 0, xi = 0 so that the ESO and controller
     * start from a known state; useful after mode switches or faults.
     */
    void reset(float y0 = 0.0f) {
        z1 = y0;
        z2 = 0.0f;
        xi = 0.0f;
    }

    /**
     * @brief Set controller bandwidth and recompute gains
     * @param wc_ New controller bandwidth (rad/s)
     *
     * Recomputes kp and ki such that:
     * - kp = 2·wc = 2·wc_
     * - ki = wc² = wc_²
     * This changes the closed‑loop pole location but keeps the LADRC structure.
     */
    void set_bandwidth(float wc_) {
        wc = wc_;
        updateGains();
    }

    /**
     * @brief Set nominal plant gain and recompute gains
     * @param b0_ New nominal plant gain (u → ẏ)
     *
     * Updates b0 and the controller gain pair (kp, ki) so that the
     * PI‑like virtual control v = kp·e + ki·∫e dt is adapted to the
     * new operating point. The ESO still uses b0 for plant gain.
     */
    void set_b0(float b0_) {
        b0 = b0_;
        updateGains();
    }

    /**
     * @brief Single control step (tick) with dynamic output saturation
     * @param ref      Reference (setpoint)
     * @param y_meas   Measured output
     * @param u_limit  Maximum absolute control output (dynamically calculated limit, e.g. available Vbus)
     * @return Computed and saturated control signal u
     *
     * Algorithm flow:
     * 1. Compute tracking error: e = ref - z1.
     * 2. Compute proportional and integral action.
     * 3. Compute virtual control v and raw control output u.
     * 4. Clamp output u to [-u_limit, u_limit] and recalculate integrator state (Anti-Windup).
     * 5. LESO update using the ACTUAL clamped u (prevents observer divergence).
     *
     * Complexity: O(1) per tick, suitable for fixed‑rate embedded control loops.
     */
    float tick(float ref, float y_meas, float u_limit) {
        // 1. Compute tracking error using estimated output (robust against noise)
        const float e = ref - z1;

        // 2. Compute proportional term and tentative integral term
        const float v_prop = kp * e;
        float v_int = xi + ki * e * dt;

        // 3. Disturbance compensation: raw control signal (u_raw = (v - z2) / b0)
        float u = (v_prop + v_int - z2) / b0;

        // 4. Output Saturation and Exact Back-Calculation Anti-Windup
        if (u > u_limit) {
            u = u_limit;
            // Back-calculate integrator: (v_prop + v_int - z2) / b0 = u_limit
            v_int = u_limit * b0 + z2 - v_prop;
        } else if (u < -u_limit) {
            u = -u_limit;
            // Back-calculate integrator: (v_prop + v_int - z2) / b0 = -u_limit
            v_int = -u_limit * b0 + z2 - v_prop;
        }
        
        // Save the bounded integrator state
        xi = v_int;

        // 5. LESO update: forward Euler using the CLAMPED u
        const float y_err = y_meas - z1;
        const float dz1 = beta1 * y_err + z2 + b0 * u;
        const float dz2 = beta2 * y_err;

        z1 += dz1 * dt;
        z2 += dz2 * dt;

        return u;
    }

private:
    /**
     * @brief Recompute ESO and controller gains from wc, wo, b0
     *
     * Sets:
     * - beta1 = 2·wo
     * - beta2 = wo²
     * - kp   = 2·wc
     * - ki   = wc²
     * Called internally by set_bandwidth() and set_b0(); also called at construction.
     */
    void updateGains() {
        // ESO gains for second‑order LESO (β1 = 2·wo, β2 = wo²)
        beta1 = 2.0f * wo;
        beta2 = wo * wo;

        // Controller gains for first‑order LADRC (kp = 2·wc, ki = wc²)
        kp = 2.0f * wc;
        ki = wc * wc;
    }
};
