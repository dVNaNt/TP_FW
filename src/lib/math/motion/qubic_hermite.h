// =============================================================================
// hermite_interpolator.h
// Cubic Hermite segment interpolator for STM32G4 (Cortex-M4F)
// =============================================================================
#pragma once
#include <cmath>
#include <cstdint>

// ARM Math/DSP Intrinsics (Include if using CMSIS, or rely on compiler builtins)
// #include <arm_math.h>

class HermiteInterpolator {
private:
    uint64_t inv_T_u64_;    // Reciprocal of duration (2^64-1 / T)
    float v_fac_;           // Float multiplier for velocity to physical RPS
    int32_t c1_, c2_, c3_;  // Horner polynomial coefficients (Scaled for Q31 U)
    uint8_t shift32_;       // Final left shift for position scale

public:
    HermiteInterpolator() = default;

    // -------------------------------------------------------------------------
    // prepare()
    // Calculates coefficients for the normalized cubic Hermite polynomial:
    // P(u) = A u^3 + B u^2 + C u, where u is normalized time [0, 1]
    // A = m0 + m1 - 2*Delta_p
    // B = 3*Delta_p - 2*m0 - m1
    // C = m0
    // -------------------------------------------------------------------------
    void prepare(int32_t delta_p_q16, uint32_t duration_us, float v0_rps, float v1_rps) {
        const float T_f32 = float(duration_us) * 1e-6f;
        const float D_f32 = float(delta_p_q16) * (1.0f / 65536.0f);

        // m0 and m1 are total position displacement over the segment based on initial/final velocity
        const float m0 = v0_rps * T_f32;
        const float m1 = v1_rps * T_f32;

        // ---------------------------------------------------------------------
        // MATH EXPLANATION: Q31 Pre-compensation
        // The evaluation step multiplies variables by u_q31 and shifts right by 32.
        // This is mathematically equivalent to multiplying by (u / 2).
        // To evaluate P(u) using Horner's method with a factor of (u/2), we must 
        // pre-multiply the coefficients to offset the 1/8, 1/4, and 1/2 scaling factors:
        // C1_f = 2 * C
        // C2_f = 4 * B
        // C3_f = 8 * A
        // ---------------------------------------------------------------------
        const float C1f = 2.0f * m0;
        const float C2f = 12.0f * D_f32 - 8.0f * m0 - 4.0f * m1;
        const float C3f = -16.0f * D_f32 + 8.0f * m0 + 8.0f * m1;

        // Find the maximum absolute value to determine optimal fixed-point scaling
        const float worst = fabsf(C1f) + fabsf(C2f) + fabsf(C3f);

        int fb = 30;
        if (worst > 1e-12f) {
            // Fast log2 calculation by extracting the IEEE 754 exponent
            const uint32_t bits = __builtin_bit_cast(uint32_t, worst);
            const int exp = int((bits >> 23) & 0xFF) - 127;
            fb = 30 - exp;
            fb = (fb < 1) ? 1 : ((fb > 30) ? 30 : fb);
        }

        const int64_t D_shifted = int64_t(delta_p_q16) << fb;
        const int32_t D_scaled = int32_t((D_shifted + (delta_p_q16 >= 0 ? 32768 : -32768)) >> 16);

        const uint32_t scale_bits = uint32_t(fb + 127) << 23;
        const float sc = __builtin_bit_cast(float, scale_bits);

        const int32_t m0_sc = __builtin_lroundf(m0 * sc);
        const int32_t m1_sc = __builtin_lroundf(m1 * sc);

        // Integer cancellation is preserved, completely avoiding floating-point drift
        c1_ = 2 * m0_sc;
        c2_ = 12 * D_scaled - 8 * m0_sc - 4 * m1_sc;
        c3_ = -16 * D_scaled + 8 * m0_sc + 8 * m1_sc;
        shift32_ = uint8_t(32 - fb);

        const uint32_t inv_sc_bits = uint32_t(127 - fb) << 23;
        const float inv_sc = __builtin_bit_cast(float, inv_sc_bits);
        
        // V_raw is mathematically derived from P(u_q31). Since u_q31 = u / 2, 
        // the raw derivative is scaled by 2. We multiply by 500,000 (1e6 / 2) 
        // to simultaneously correct the factor of 2 and convert microseconds to seconds.
        v_fac_ = inv_sc * (500000.0f / float(duration_us));

        if (duration_us <= 1) {
            inv_T_u64_ = ~0ULL;
        } else {
            inv_T_u64_ = ~0ULL / duration_us;
        }
    }

    // -------------------------------------------------------------------------
    // eval()
    // -------------------------------------------------------------------------
    __attribute__((always_inline)) void eval(uint32_t t_us_in_seg, int32_t& out_p_rel_q16_32, float& out_v_rps) const {
        // High-precision fixed-point normalized time calculation u = t / T
        const uint32_t inv_lo = uint32_t(inv_T_u64_);
        const uint32_t inv_hi = uint32_t(inv_T_u64_ >> 32);
        const uint32_t u_lo = uint32_t((uint64_t(inv_lo) * t_us_in_seg) >> 32);
        const uint32_t u_hi = inv_hi * t_us_in_seg;
        const uint64_t raw = uint64_t(u_lo) + uint64_t(u_hi);

        const int32_t u_q31 = (raw >= 0xFFFFFFFFULL) ? 0x7FFFFFFFl : int32_t(raw >> 1);

        // ---------------------------------------------------------------------
        // MATH EXPLANATION: Horner's Method with DSP Intrinsics
        // The operation `(int64_t(A) * B) >> 32` compiles directly to the 
        // ARM SMMUL (Signed Multiply Most Significant Word) instruction (1 cycle).
        // ---------------------------------------------------------------------
        const int32_t c3_u = int32_t((int64_t(c3_) * u_q31) >> 32);

        const int32_t p_t1 = c3_u + c2_;
        
        // V(u) derivative calculation mixed directly into the Horner evaluation
        // Derivative of Horner formulation: v_t1 = 3 * c3 * u_q31^2 + 2 * c2 * u_q31
        const int32_t v_t1 = (p_t1 << 1) + c3_u;

        // Adding c1_ matches the SMMLA (Signed Multiply Accumulate) instruction (1 cycle)
        const int32_t p_t2 = int32_t((int64_t(p_t1) * u_q31) >> 32) + c1_;
        const int32_t v_raw = int32_t((int64_t(v_t1) * u_q31) >> 32) + c1_;

        const int32_t p_raw = int32_t((int64_t(p_t2) * u_q31) >> 32);

        out_p_rel_q16_32 = p_raw << shift32_;
        out_v_rps = float(v_raw) * v_fac_;
    }
};