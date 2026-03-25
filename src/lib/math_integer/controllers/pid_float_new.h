#pragma once

#include <cstdint>
#include <cmath>

/// Stable float PID controller with dynamic symmetric output limit
/// and back-calculation anti-windup.
///
/// All I/O in int32_t (same units as encoder Q16.16 or raw counts).
/// Internal computation in float for numerical stability.
///
/// Anti-windup method: back-calculation
///   integrator += Ki·error·dt + Kb·(output_sat - output_unsat)·dt
///   This is the IEC 61131 / industrial servo standard.
///   Kb = sqrt(Ki/Kp) is the optimal back-calc gain (Astrom & Hagglund).
struct PID {
public:
    // =========================================================================
    // Tuning parameters
    // =========================================================================

    float kp   = 1.0f;    // Proportional gain
    float ki   = 0.0f;    // Integral gain     [1/s]
    float kd   = 0.0f;    // Derivative gain   [s]
    float kb   = 0.0f;    // Back-calculation anti-windup gain
                          // Recommended: sqrt(ki/kp), or 0 to disable

    float dt   = 0.00005f; // Timestep [s] — set to 1/Fs

    // Derivative low-pass filter coefficient (0 = no filter, 1 = off)
    // alpha = 1 - exp(-2π·f_cut·dt)
    // e.g. at 20kHz, f_cut=500Hz → alpha ≈ 0.147
    float d_filter_alpha = 0.1f;

    // =========================================================================
    // Constructor
    // =========================================================================

    PID() = default;

    PID(float kp, float ki, float kd, float dt, float kb = 0.0f)
        : kp(kp), ki(ki), kd(kd), kb(kb), dt(dt) {}

    // =========================================================================
    // Main update — called every control tick
    //
    //   setpoint : desired value   [int32_t, same units as real]
    //   real     : measured value  [int32_t]
    //   limit    : symmetric output clamp, must be > 0
    //              output is guaranteed in [-limit, +limit]
    //
    // Returns output in int32_t, same units as input.
    // =========================================================================

    int32_t tick(int32_t setpoint, int32_t real, int32_t limit) {
        // -- 1. Error ----------------------------------------------------------
        float error = (float)(setpoint - real);

        // -- 2. Proportional ---------------------------------------------------
        float p_term = kp * error;

        // -- 3. Derivative (filtered) -----------------------------------------
        //
        // Derivative on measurement (not on error) to avoid derivative kick
        // when setpoint changes abruptly.
        //
        //   d_raw  = -(real_f - prev_real_f) / dt
        //   d_filt = d_filt_prev + alpha * (d_raw - d_filt_prev)
        //
        float real_f = (float)real;
        float d_raw  = -(real_f - prev_real_) / dt;
        d_filt_      += d_filter_alpha * (d_raw - d_filt_);
        float d_term  = kd * d_filt_;
        prev_real_    = real_f;

        // -- 4. Unsaturated output (P + I + D) --------------------------------
        float output_unsat = p_term + integrator_ + d_term;

        // -- 5. Symmetric output clamp ----------------------------------------
        float lim_f      = (float)limit;
        float output_sat = output_unsat;
        if      (output_sat >  lim_f) output_sat =  lim_f;
        else if (output_sat < -lim_f) output_sat = -lim_f;

        // -- 6. Integrator update with back-calculation anti-windup -----------
        //
        // Standard clamping alone freezes the integrator at a fixed value and
        // creates slow recovery when limit changes. Back-calculation feeds the
        // saturation error back through Kb, actively driving the integrator
        // toward the saturation boundary instead of freezing it.
        //
        //   Δ_antiwindup = Kb · (output_sat - output_unsat)
        //
        // When not saturated: output_sat == output_unsat → zero correction.
        // When saturated:     correction pulls integrator back.
        //
        float sat_error = output_sat - output_unsat;

        // Dynamic integrator limit: integrator can never exceed the output
        // limit on its own — prevents slow recovery on limit changes.
        float integ_new = integrator_
                        + (ki * error + kb * sat_error) * dt;

        // Hard clamp integrator to [-limit, +limit]
        // This is a second safety net on top of back-calculation.
        // It handles the case where limit suddenly decreases.
        if      (integ_new >  lim_f) integ_new =  lim_f;
        else if (integ_new < -lim_f) integ_new = -lim_f;

        integrator_ = integ_new;

        // -- 7. Return ---------------------------------------------------------
        return (int32_t)output_sat;
    }

    // =========================================================================
    // Utilities
    // =========================================================================

    // Full state reset (call after fault, mode switch, or re-enable)
    inline void reset() {
        integrator_ = 0.0f;
        d_filt_     = 0.0f;
        prev_real_  = 0.0f;
    }

    // Preset integrator to a known value (bumpless transfer)
    // e.g. when switching from open-loop to closed-loop:
    //   pid.preset_integrator(current_output - kp * current_error)
    inline void preset_integrator(float value) {
        integrator_ = value;
    }

    // Read internal state (for monitoring / auto-tune)
    inline float get_integrator()  const { return integrator_; }
    inline float get_d_filtered()  const { return d_filt_;     }

    // Compute optimal Kb from current Kp/Ki (call after changing gains)
    // Based on Astrom & Hagglund "PID Controllers" eq. 3.28
    inline void auto_kb() {
        kb = (kp > 0.0f && ki > 0.0f)
            ? sqrtf(ki / kp)
            : 0.0f;
    }

private:
    float integrator_ = 0.0f;  // integral accumulator
    float d_filt_     = 0.0f;  // filtered derivative state
    float prev_real_  = 0.0f;  // previous measurement for derivative
};
