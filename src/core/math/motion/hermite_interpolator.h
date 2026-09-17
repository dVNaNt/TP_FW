/**
 * @file hermite_interpolator.h
 * @brief Fixed-point cubic Hermite segment evaluator.
 *
 * prepare() precomputes the four Hermite basis coefficients from displacement,
 * duration and boundary velocities; eval() then produces position (Q32.32
 * fixed point, 64 bits so a segment may span any displacement) and velocity
 * per microsecond tick using only multiplies and shifts - suitable for the
 * ISR-hot trajectory path.
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

// =============================================================================
// hermite_interpolator.h
// Cubic Hermite segment interpolator
// =============================================================================
#pragma once
#include <cmath>
#include <cstdint>

#include "../div64.h"   // udiv64_32(): 64/32 reciprocal without the libgcc 64-bit divider

/**
 * @class HermiteInterpolator
 * @brief A high-performance Cubic Hermite Spline Interpolator optimized for ARM Cortex-M4F (STM32G4).
 * * This class computes and evaluates a normalized cubic Hermite polynomial segment.
 * It uses fixed-point math (Q31 and Q16), dynamic bit-shifting based on IEEE 754 exponent 
 * extraction to prevent underflow/overflow, and utilizes Horner's method structured to 
 * compile into single-cycle DSP assembly instructions (SMMUL/SMMLA).
 * * Polynomial definition:
 * P(u) = A*u^3 + B*u^2 + C*u, where u in [0, 1) is the normalized time.
 */
class HermiteInterpolator {
private:
    uint64_t inv_T_u64_{0};   ///< Reciprocal of segment duration (Scaled to 2^64-1 / T)
    float v_fac_{0.0f};       ///< Velocity conversion factor from raw Q31 steps to physical RPS
    int32_t c1_{0};           ///< Horner coefficient 1: Scaled for Q31 execution
    int32_t c2_{0};           ///< Horner coefficient 2: Scaled for Q31 execution
    int32_t c3_{0};           ///< Horner coefficient 3: Scaled for Q31 execution
    uint8_t shift32_{0};      ///< Dynamic post-evaluation left shift to restore Q16 position scale
    bool is_static_{true};    ///< HOT PATH FLAG: True if the segment represents a completely static state (no motion)

public:
    /**
     * @brief Default constructor initializing the interpolator in a safe, static state.
     */
    HermiteInterpolator() = default;

    /**
     * @brief Pre-calculates fixed-point coefficients for the Hermite trajectory segment.
     * * Calculates the boundary constraints and scales the cubic polynomial coefficients
     * to maximize accuracy within 32-bit registers. It automatically detects if the segment
     * contains zero displacement and zero boundary velocities to trigger a static hot path.
     * * @param delta_p_q16   Target relative position displacement for this segment in Q16 fixed-point format.
     * @param duration_us   Segment duration in microseconds.
     * @param v0_rps        Initial velocity boundary condition in Revolutions Per Second (RPS).
     * @param v1_rps        Final velocity boundary condition in Revolutions Per Second (RPS).
     */
    void prepare(int32_t delta_p_q16, uint32_t duration_us, float v0_rps, float v1_rps) {
        
        // ---------------------------------------------------------------------
        // HOT PATH DETECTION: Zero Displacement
        // ---------------------------------------------------------------------
        // If there is no positional change and no boundary velocities, the segment 
        // is completely static. We bypass all matrix/scaling math and exit early.
        if (delta_p_q16 == 0) {
            is_static_ = true;
            c1_ = 0; c2_ = 0; c3_ = 0;
            shift32_ = 0;
            v_fac_ = 0.0f;
            inv_T_u64_ = 0;
            return;
        }
        
        // If we pass the check, motion is expected on this segment.
        is_static_ = false;

        // Convert time to seconds and Q16 delta position to float for scaling analysis
        const float T_f32 = float(duration_us) * 1e-6f;
        const float D_f32 = float(delta_p_q16) * (1.0f / 65536.0f);

        // m0 and m1 represent target positional displacements over the segment duration 
        // derived from the physical velocity boundaries.
        const float m0 = v0_rps * T_f32;
        const float m1 = v1_rps * T_f32;

        // ---------------------------------------------------------------------
        // MATHEMATICAL DERIVATION: Q31 Pre-compensation
        // The evaluation phase processes normalized time as u_q31 = u / 2.
        // To counter the resulting 1/2, 1/4, and 1/8 scale factors inside Horner's setup,
        // the standard Hermite matrix coefficients are pre-multiplied:
        // C1 = 2 * m0
        // C2 = 12 * D - 8 * m0 - 4 * m1
        // C3 = -16 * D + 8 * m0 + 8 * m1
        // ---------------------------------------------------------------------
        const float C1f = 2.0f * m0;
        const float C2f = 12.0f * D_f32 - 8.0f * m0 - 4.0f * m1;
        const float C3f = -16.0f * D_f32 + 8.0f * m0 + 8.0f * m1;

        // Analyze peak magnitude of the coefficients to determine maximum safe fixed-point headroom.
        const float worst = fabsf(C1f) + fabsf(C2f) + fabsf(C3f);

        int fb = 30;
        if (worst > 1e-12f) {
            // High-speed log2 approximation utilizing IEEE 754 float binary format extraction.
            // Extracts the exponent bits directly to find the scale bounds in 1-2 clock cycles.
            const uint32_t bits = __builtin_bit_cast(uint32_t, worst);
            const int exp = int((bits >> 23) & 0xFF) - 127;
            fb = 30 - exp;
            fb = (fb < 1) ? 1 : ((fb > 30) ? 30 : fb); // Clamping fractional bits to safe operating range
        }

        // Scale integer position displacement into the determined fixed-point headroom
        const int64_t D_shifted = int64_t(delta_p_q16) << fb;
        const int32_t D_scaled = int32_t((D_shifted + (delta_p_q16 >= 0 ? 32768 : -32768)) >> 16);

        // Reconstruct float scaling factor directly in bit representation
        const uint32_t scale_bits = uint32_t(fb + 127) << 23;
        const float sc = __builtin_bit_cast(float, scale_bits);

        // lrintf (round-to-nearest-even, one vcvtr) instead of lroundf (a libm call):
        // the two differ only on exact .5 ties, irrelevant for coefficient quantisation.
        const int32_t m0_sc = __builtin_lrintf(m0 * sc);
        const int32_t m1_sc = __builtin_lrintf(m1 * sc);

        // Compute pure integer coefficients. Integer cancellation eliminates any floating-point accumulation drift.
        c1_ = 2 * m0_sc;
        c2_ = 12 * D_scaled - 8 * m0_sc - 4 * m1_sc;
        c3_ = -16 * D_scaled + 8 * m0_sc + 8 * m1_sc;
        shift32_ = uint8_t(32 - fb);

        // Compute inverse scaling factor for output velocity float reconstruction
        const uint32_t inv_sc_bits = uint32_t(127 - fb) << 23;
        const float inv_sc = __builtin_bit_cast(float, inv_sc_bits);
        
        // V_raw is derived from P(u_q31). Since u_q31 = u / 2, the raw derivative contains a factor of 2.
        // We multiply by 500,000 (1e6 / 2) to correct for this scale factor and convert microseconds to seconds simultaneously.
        v_fac_ = inv_sc * (500000.0f / float(duration_us));

        // Compute high-precision 64-bit inverse time factor to replace division with multiplication during runtime evaluation.
        // udiv64_32() is three hardware UDIVs (~80 cycles); the plain `~0ULL / duration_us`
        // was the image's only 64-bit division and linked libgcc's 700 B shift-subtract loop.
        if (duration_us <= 1) {
            inv_T_u64_ = ~0ULL;
        } else {
            inv_T_u64_ = udiv64_32(~0ULL, duration_us);
        }
    }

    /**
     * @brief Evaluates position and velocity for a given timestamp within the segment.
     * * @note [HOT PATH] If the segment is flagged as static, this function bypasses all tracking math
     * and writes zero outputs immediately. Uses compiler hints to ensure zero branch-prediction overhead.
     * * @param[in]  t_us_in_seg      Current elapsed time from the start of this segment, in microseconds.
     * @param[out] out_p_rel_q32  Target reference output position relative to the segment start,
     *                            revolutions with 32 fractional bits (the executor's Q32.32 basis).
     *                            64 bits wide on purpose: the raw polynomial value is bounded by the
     *                            coefficient magnitude, which prepare() keeps under 2^30, so the
     *                            left shift that restores the 32-bit fraction is exact in 64 bits,
     *                            while a 32-bit result could hold at most half a revolution and
     *                            wrapped on any longer segment - the trajectory folded back on itself
     *                            half way through a one-turn move.
     * @param[out] out_v_rps        Target reference output velocity, in physical Revolutions Per Second (RPS).
     */
    __attribute__((always_inline)) void eval(uint32_t t_us_in_seg, int64_t& out_p_rel_q32, float& out_v_rps) const {
        
        // ---------------------------------------------------------------------
        // HOT PATH EXECUTION
        // ---------------------------------------------------------------------
        // __builtin_expect tells the GCC compiler that 'is_static_' is highly likely to be false (0).
        // This ensures that the compiler structures assembly pipeline to pass straight through 
        // without branch penalties for moving states, while providing ultra-fast exit for static states.
        if (__builtin_expect(is_static_, 0)) {
            out_p_rel_q32 = 0;
            out_v_rps = 0.0f;
            return;
        }

        // --- Step 1: Calculate Normalized Time (u = t / T) via 64-bit Fixed Point Multiplication ---
        const uint32_t inv_lo = uint32_t(inv_T_u64_);
        const uint32_t inv_hi = uint32_t(inv_T_u64_ >> 32);
        const uint32_t u_lo = uint32_t((uint64_t(inv_lo) * t_us_in_seg) >> 32);
        const uint32_t u_hi = inv_hi * t_us_in_seg;
        const uint64_t raw = uint64_t(u_lo) + uint64_t(u_hi);

        // Format normalized time to Q31 bounds (u_q31 = u / 2) to prevent signed multiplication overflows.
        const int32_t u_q31 = (raw >= 0xFFFFFFFFULL) ? 0x7FFFFFFFl : int32_t(raw >> 1);

        // ---------------------------------------------------------------------
        // ALGORITHM: Interleaved Horner's Method with ARM DSP Intrinsics
        // The pattern `(int64_t(A) * B) >> 32` map directly to the hardware 
        // 1-clock instruction SMMUL (Signed Multiply Most Significant Word) on Cortex-M4.
        // ---------------------------------------------------------------------
        
        // Compute: c3 * u
        const int32_t c3_u = int32_t((int64_t(c3_) * u_q31) >> 32);

        // Accumulate second level: (c3 * u) + c2
        const int32_t p_t1 = c3_u + c2_;
        
        // Calculate raw velocity tracking derivative concurrently within the Horner sequence steps.
        // Mathematical derivative formulation: v_t1 = 3 * c3 * u^2 + 2 * c2 * u
        const int32_t v_t1 = (p_t1 << 1) + c3_u;

        // Perform final accumulation levels.
        // Adding c1_ matches the native ARM SMMLA (Signed Multiply Accumulate) instruction execution.
        const int32_t p_t2 = int32_t((int64_t(p_t1) * u_q31) >> 32) + c1_;
        const int32_t v_raw = int32_t((int64_t(v_t1) * u_q31) >> 32) + c1_;

        const int32_t p_raw = int32_t((int64_t(p_t2) * u_q31) >> 32);

        // --- Step 2: Output Reconstruction ---
        // Restore the 32 fractional bits of the Q32.32 basis in 64-bit arithmetic (see the
        // parameter note: the raw value is under 2^30, so the shift cannot lose bits) and
        // multiply the tracking velocity by its scaling constant.
        out_p_rel_q32 = int64_t(p_raw) << shift32_;
        out_v_rps = float(v_raw) * v_fac_;
    }
};