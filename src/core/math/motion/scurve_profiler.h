/**
 * @file scurve_profiler.h
 * @brief 7-segment S-Curve motion profile generator with Q16 fixed-point outputs.
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#pragma once

#include <cmath>
#include <climits>
#include <algorithm>
#include <cstring> // For safe std::memcpy
#include "motion_command.h"

/**
 * @class SCurveProfiler
 * @brief Time-optimal, jerk-limited kinematic trajectory generator.
 * @details Optimized for embedded systems (MCU). Uses purely 32-bit float 
 * kinematics and Delta-DDA to eliminate `double` precision overhead while 
 * maintaining perfect integer displacement matching.
 */
class SCurveProfiler {
public:
    // ============================================================
    // Public Constants & Types
    // ============================================================

    static constexpr float kQ16Scale = 65536.0f;
    static constexpr float kInvQ16Scale = 1.0f / 65536.0f;
    static constexpr int kMaxSegments = 7;

    // ============================================================
    // Public API
    // ============================================================

    [[nodiscard]] static inline int32_t toQ16(float x) noexcept {
        return static_cast<int32_t>(x * kQ16Scale);
    }

    /**
     * @brief Generates S-Curve profile and directly pushes to the provided callback.
     * @tparam PushCb Callable type, e.g. a lambda capturing `buf_.write` or `executor.push`
     */
    template <typename PushCb>
    static int generate(
        PushCb&& push_cb,
        int32_t displacement,
        uint32_t time_us,
        float max_vel,
        float max_acc,
        float max_jrk, 
        uint16_t torque) noexcept
    {
        if (displacement == 0) [[unlikely]] return 0;
        if (max_vel <= 0.0f || max_acc <= 0.0f || max_jrk <= 0.0f) [[unlikely]] return 0;

        // Local on-stack buffer used to generate segments before pushing them
        // Allows safely committing a final correction segment (loss compensation).
        MotionCommand out[kMaxSegments];

        const float sg = (displacement < 0) ? -1.0f : 1.0f;
        const float D = static_cast<float>(displacement < 0 ? -displacement : displacement) * kInvQ16Scale;

        const float inv_A = 1.0f / max_acc;
        const float inv_J = 1.0f / max_jrk;

        const float t_j = max_acc * inv_J;
        const float t_j_sq = t_j * t_j;
        const float dv_ramp = max_acc * t_j;

        // --- Lambda computing one half of the acceleration profile ---
        auto accel_half = [&](float v_half) noexcept -> HalfProfile {
            if (v_half <= 0.0f) return {0.0f, 0.0f};

            if (v_half >= dv_ramp) {
                const float tc = (v_half - dv_ramp) * inv_A;
                const float d = max_acc * t_j_sq + 1.5f * max_acc * t_j * tc + 0.5f * max_acc * tc * tc;
                return {2.0f * t_j + tc, d};
            } else {
                const float tj2 = sqrtf(v_half * inv_J);
                return {2.0f * tj2, v_half * tj2};
            }
        };

        const HalfProfile full_ah = accel_half(max_vel);
        float v_peak, t_acc_half, d_acc_half;

        if (D >= 2.0f * full_ah.d) {
            v_peak = max_vel;
            t_acc_half = full_ah.t;
            d_acc_half = full_ah.d;
        } else {
            const float d_lim = 2.0f * max_acc * t_j_sq;

            if (D >= d_lim) {
                const float disc = t_j_sq + 4.0f * D * inv_A;
                const float tc = 0.5f * (-3.0f * t_j + sqrtf(disc)); 
                v_peak = dv_ramp + max_acc * ((tc > 0.0f) ? tc : 0.0f);   // Plain compare: no libm fmaxf
            } else {
                const float cbrt_val = fast_cbrtf(D * sqrtf(max_jrk) * 0.5f);
                const float v_cbrt = cbrt_val * cbrt_val;
                v_peak = (v_cbrt < max_vel) ? v_cbrt : max_vel;               // Plain compare: no libm fminf
            }

            const HalfProfile ah = accel_half(v_peak);
            t_acc_half = ah.t;
            d_acc_half = ah.d;
        }

        const float d_rest = D - 2.0f * d_acc_half;                          // What the ramps leave for the cruise
        const float d_cruise = (d_rest > 0.0f) ? d_rest : 0.0f;              // Plain compare: no libm fmaxf
        const float t_cruise = (v_peak > 0.0f) ? (d_cruise / v_peak) : 0.0f;

        // --- Time scaling (float only) ---
        float scale_t = 1.0f;
        float scale_v = 1.0f;

        if (time_us > 0U) {
            const float t_min = 2.0f * t_acc_half + t_cruise;
            const float t_req = static_cast<float>(time_us) * 1.0e-6f;

            if (t_req > t_min && t_min > 0.0f) {
                scale_t = t_req / t_min;
                scale_v = t_min / t_req;
            }
        }

        const float kEps = 1.0e-5f; // Realistic epsilon for f32
        const bool has_flat_acc = (v_peak >= dv_ramp) && ((v_peak - dv_ramp) * inv_A > kEps);
        const bool has_cruise = (t_cruise > kEps);

        int n = 0;

        // --- Delta-DDA accumulators (no double, no absolute coordinates) ---
        // Everything here stays 32-bit on purpose: a float <-> int64 conversion
        // on this target goes through libgcc's double-precision helpers and
        // drags ~2.5 KiB of soft-double into an image that otherwise links none.
        float rem_disp_q16 = 0.0f;
        float rem_time_us = 0.0f;

        int32_t total_emitted_disp = 0;
        uint32_t total_emitted_time = 0;   // Sum of the durations pushed [us]; bounded by kMaxTimeUs below

        // Lambda generating and splitting segments
        auto emit = [&](float delta_d, float delta_t, float v_end) noexcept {
            // Compute the exact step in float, adding the previous step remainder
            const float step_disp_f = delta_d * sg * kQ16Scale + rem_disp_q16;
            float step_time_f = delta_t * scale_t * 1000000.0f + rem_time_us;

            // A single phase longer than this is capped (~66 minutes): keeps the
            // conversion below inside uint32 without an int64 in the path.
            constexpr float kMaxTimeUs = 4.0e9f;
            if (step_time_f > kMaxTimeUs) [[unlikely]] step_time_f = kMaxTimeUs;

            // Quantize to integers (float -> uint32 is one FPU instruction)
            int32_t disp_step = static_cast<int32_t>(step_disp_f);
            uint32_t time_step = static_cast<uint32_t>(step_time_f);

            // Keep fractional remainders for the next call
            rem_disp_q16 = step_disp_f - static_cast<float>(disp_step);
            rem_time_us = step_time_f - static_cast<float>(time_step);

            if (time_step == 0U) [[unlikely]] return;
            if (n >= kMaxSegments) [[unlikely]] return;

            // Account for what is actually pushed into the buffer
            total_emitted_disp += disp_step;
            total_emitted_time += time_step;

            const float vel = sg * v_end * scale_v;
            // The executor measures segment time as an unsigned 32-bit
            // difference and reads bit 31 as "negative", so one segment may
            // not exceed 2^31 - 1 us; a longer phase is split in two.
            constexpr uint32_t kMaxDurUs = 0x7FFFFFFFU;

            // Split a phase longer than the executor's limit. With the cap above
            // a phase is at most 4e9 us, so a single split always suffices.
            if (time_step > kMaxDurUs) [[unlikely]] {
                if (n < kMaxSegments - 1) {
                    // Proportional slice of the displacement, in float: a 24-bit
                    // mantissa is exact to 1/128 of a count at 2^31, and the
                    // second half takes the exact remainder anyway.
                    const int32_t disp_slice = static_cast<int32_t>(
                        static_cast<float>(disp_step) * (static_cast<float>(kMaxDurUs) / static_cast<float>(time_step)));

                    out[n++] = {kMaxDurUs, disp_slice, vel};

                    time_step -= kMaxDurUs;
                    disp_step -= disp_slice;
                } else {
                    time_step = kMaxDurUs;   // No slot left to split into: run the phase at the limit
                }
            }

            if (n < kMaxSegments) {
                out[n++] = {time_step, disp_step, vel};
            }
        };

        // --- Geometry computation ---
        float d1 = 0, d2 = 0, d3 = 0;
        float t1 = 0, t2 = 0, t3 = 0;
        float v1 = 0, v2 = 0;

        if (has_flat_acc) {
            t1 = t_j;
            v1 = 0.5f * max_jrk * t_j_sq;
            d1 = max_jrk * t_j_sq * t_j * (1.0f / 6.0f);

            t2 = (v_peak - dv_ramp) * inv_A;
            v2 = v1 + max_acc * t2;
            d2 = t2 * (v1 + 0.5f * max_acc * t2);

            t3 = t_j;
            d3 = d_acc_half - d1 - d2;
        } else if (v_peak > 0.0f) {
            t1 = t_acc_half * 0.5f;
            v1 = v_peak * 0.5f;
            d1 = max_jrk * t1 * t1 * t1 * (1.0f / 6.0f);
            t3 = t1;
            d3 = d_acc_half - d1;
        }

        // --- Segment generation ---
        emit(d1, t1, v1);

        if (has_flat_acc) emit(d2, t2, v2);

        if (has_cruise) {
            emit(d3, t3, v_peak);
            emit(d_cruise, t_cruise, v_peak);
            emit(d3, t3, v_peak - v1);
        } else {
            emit(d3 * 2.0f, t3 * 2.0f, v_peak - v1);
        }

        if (has_flat_acc) emit(d2, t2, v1);

        emit(d1, t1, 0.0f);

        // --- Perfect final-condition fitting ---
        if (n > 0) {
            out[n - 1].end_velocity = 0.0f;

            // Compensate lost fractional float micro-steps directly in the last segment
            out[n - 1].displacement += (displacement - total_emitted_disp);

            // Fit exactly into the requested time - only when the profile was
            // actually stretched to it. A request shorter than the time-optimal
            // profile is ignored above, and trimming the last segment to it
            // would end the move with a velocity kick instead of at rest.
            if ((time_us > 0U) && (scale_t > 1.0f)) {
                // The emitted total only differs from the request by the
                // quantization remainders, so the signed difference fits an
                // int32 and the wrapping subtraction recovers it exactly.
                const int32_t dur_fix = static_cast<int32_t>(time_us - total_emitted_time);
                const uint32_t last = out[n - 1].duration;
                const uint32_t new_dur = last + static_cast<uint32_t>(dur_fix);   // Wrapping add

                // Apply only when the result is a valid, positive segment length
                // (a negative fix larger than the segment would wrap past zero).
                const bool shrinks_past_zero = (dur_fix < 0) && (static_cast<uint32_t>(-dur_fix) >= last);
                if (!shrinks_past_zero && (new_dur <= (UINT32_MAX >> 1))) {
                    out[n - 1].duration = new_dur;
                }
            }
        }

        // --- Push finished segments through the callback ---
        for (int i = 0; i < n; ++i) {
            MotionCommandTM tm;
            tm.motion = out[i];
            tm.torque = torque;
            tm.mode = MotionMode::HERMITE;
            push_cb(tm);
        }

        return n;
    }

private:
    struct HalfProfile {
        float t; 
        float d; 
    };

    /**
     * @brief Fast cube root without UB (Undefined Behavior).
     */
    [[nodiscard]] static inline float fast_cbrtf(float x) noexcept {
        if (x == 0.0f) return 0.0f;
        const float ax = (x < 0.0f) ? -x : x;

        uint32_t i;
        std::memcpy(&i, &ax, sizeof(i)); // Safe bit-cast (C++11 and later)
        i = i / 3u + 0x2A517D3Cu;
        
        float r;
        std::memcpy(&r, &i, sizeof(r));

        r = (2.0f * r + ax / (r * r)) * 0.33333333f;
        r = (2.0f * r + ax / (r * r)) * 0.33333333f;

        return (x < 0.0f) ? -r : r;
    }
};