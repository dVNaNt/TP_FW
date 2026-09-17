/**
 * @file div64.h
 * @brief Exact int64 / int16 division on Cortex-M4 without the 64-bit library divider.
 *
 * The STM32G4 core has a 32-bit hardware divider (UDIV, 2-12 cycles) and a
 * single-cycle 32x32->64 multiplier (UMULL / UMLAL), but no 64-bit divide: a
 * plain `int64_t / int16_t` in C++ becomes a call to libgcc's
 * `__aeabi_ldivmod` / `__udivmoddi4` - a shift-subtract loop of up to 64
 * iterations (a few hundred cycles, 700 B of flash). This header gives two
 * exact replacements, both truncating toward zero like C:
 *
 * 1. `DivMagic16` - divisor invariant over many divisions (a segment
 *    duration, a sample rate, a compile-time constant). set() precomputes a
 *    65-bit reciprocal once with four UDIVs; div() is then one 64x64->128
 *    high multiply (four UMULL/UMLAL) plus a fix-up and a shift: ~30 cycles,
 *    branch-free apart from the |d| == 1 case, no data-dependent timing.
 *    Constructed from a constant it folds completely at compile time.
 *
 * 2. `div64_16()` - divisor different on every call. Schoolbook long
 *    division in base 2^16 with the hardware divider: because |d| < 2^16 the
 *    partial remainder is always below 2^16, so every partial dividend fits
 *    32 bits and three UDIVs produce the full 64-bit quotient. ~35-45 cycles
 *    (UDIV is operand-dependent), no setup, no state.
 *
 * 3. `udiv64_32()` - unsigned 64-bit dividend by a full 32-bit divisor
 *    (a reciprocal of a duration in microseconds, for instance). The high
 *    word is divided directly; the remaining 64/32 -> 32 step is Knuth's
 *    Algorithm D in base 2^16 (Hacker's Delight 9-4 `divlu`): normalize the
 *    divisor, estimate each of the two quotient digits with a UDIV by the
 *    divisor's top digit and correct the estimate at most twice. Three UDIVs,
 *    ~80 cycles, exact. This is the one that replaces the library call at a
 *    segment boundary, where the divisor is 32 bits wide.
 *
 * ## Why the multiply is exact (Granlund & Montgomery, PLDI 1994)
 *
 * For an N-bit unsigned dividend u and a divisor d with l = ceil(log2 d),
 * take m = floor(2^(N+l) / d) + 1. Then floor(m * u / 2^(N+l)) == floor(u / d)
 * for EVERY u < 2^N: the error m*d - 2^(N+l) lies in (0, 2^l] and is
 * swallowed by the final shift. m has N+1 bits, so it is stored minus its
 * implicit top bit (m' = m - 2^N) and the missing term u * 2^N is added back
 * as `t + ((u - t) >> 1)` (Hacker's Delight 10-8), which cannot overflow
 * because t = floor(m' * u / 2^N) <= u. For N = 64 and d < 2^16:
 * 2^(64+l) = d * 2^64 + e * 2^64 with e = 2^l - d < d, hence
 * m' = floor(e * 2^64 / d) + 1, a 64-bit number obtained by long division of
 * the five base-2^16 digits (e, 0, 0, 0, 0) - four UDIVs, no 64-bit divide.
 *
 * ## Edge cases
 *
 * - d == 0: `DivMagic16` treats it as 1 (identity), `div64_16()` returns 0
 *   and `udiv64_32()` returns an unspecified value (UDIV by zero yields 0
 *   with the default CCR.DIV_0_TRP = 0). None of them traps.
 * - INT64_MIN / -1 has no int64 representation and wraps to INT64_MIN.
 * - |d| == INT16_MIN is 32768, a power of two: the result is a pure shift.
 *
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#pragma once

#include <stdint.h>

/**
 * @brief High 64 bits of the 128-bit product a * b.
 *
 * Four 32x32->64 partial products, each cross term accumulated onto the
 * carry of the previous column so GCC emits one UMULL and three UMLAL (all
 * single-cycle on Cortex-M4). No partial sum exceeds 64 bits: a product of
 * two 32-bit words is at most 2^64 - 2^33 + 1, leaving room for the 32-bit
 * carry added to it.
 */
__attribute__((always_inline)) static inline uint64_t umulhi64(uint64_t a, uint64_t b) {
    const uint32_t a_lo = (uint32_t)a;                          // Low word of a
    const uint32_t a_hi = (uint32_t)(a >> 32);                  // High word of a
    const uint32_t b_lo = (uint32_t)b;                          // Low word of b
    const uint32_t b_hi = (uint32_t)(b >> 32);                  // High word of b
    const uint64_t p_ll = (uint64_t)a_lo * b_lo;                // UMULL: bits 0..63; only its top word matters
    const uint64_t c1 = (p_ll >> 32) + (uint64_t)a_hi * b_lo;   // UMLAL: column 32 with the first cross term
    const uint64_t c2 = (uint32_t)c1 + (uint64_t)a_lo * b_hi;   // UMLAL: column 32 with the second cross term
    return ((c1 >> 32) + (c2 >> 32)) + (uint64_t)a_hi * b_hi;   // UMLAL: column 64 plus both carries
}

/**
 * @brief Precomputed reciprocal of an int16 divisor for exact int64 division.
 *
 * Usage:
 * @code
 *   DivMagic16 by_dt;                 // Divisor changes rarely
 *   by_dt.set(dt);                    // Four UDIVs, once per change
 *   int64_t q = by_dt.div(numerator); // ~30 cycles, exact, every sample
 *
 *   constexpr DivMagic16 by_1000(1000);   // Folds at compile time
 *   int64_t ms = by_1000.div(us);         // Multiply + shift, no setup
 * @endcode
 */
struct DivMagic16 {
    uint64_t m_ = 0;    ///< Reciprocal minus its implicit 2^64 bit (m' above).
    uint8_t sh_ = 0;    ///< Final shift: ceil(log2 |d|) - 1, in 0..15.
    uint8_t neg_ = 0;   ///< 1 when the divisor is negative.
    uint8_t one_ = 1;   ///< 1 when |d| <= 1: the quotient is the dividend itself.

    /** @brief Default: divide by 1. */
    constexpr DivMagic16() = default;

    /** @brief Construct for divisor @p d; constexpr so a constant folds entirely. */
    constexpr explicit DivMagic16(int16_t d) { set(d); }

    /**
     * @brief Precompute the reciprocal of @p d.
     *
     * Four 32-bit UDIVs (one per base-2^16 digit of e * 2^64 / |d|); nothing
     * 64-bit is divided. |INT16_MIN| = 32768 is handled through uint32.
     *
     * @param d Divisor; 0 is treated as 1.
     */
    constexpr void set(int16_t d) {
        const uint32_t ad = (d < 0) ? (uint32_t)(-(int32_t)d) : (uint32_t)d;  // |d|, 0..32768
        neg_ = (d < 0) ? 1u : 0u;                                             // Sign of the divisor
        one_ = (ad <= 1u) ? 1u : 0u;                                          // Identity (and d == 0)
        if (one_) {                                                           // Nothing to precompute
            m_ = 0u;
            sh_ = 0u;
            return;
        }
        const uint32_t l = 32u - (uint32_t)__builtin_clz(ad - 1u);            // ceil(log2 ad), 1..15
        uint32_t rem = (1u << l) - ad;                                        // e = 2^l - ad, 0 <= e < ad
        uint64_t m = 0u;                                                      // Quotient digits, MSB first
        for (uint32_t i = 0u; i < 4u; ++i) {                                  // Four zero digits of e * 2^64
            const uint32_t t = rem << 16;                                     // Partial dividend, < 2^32
            const uint32_t q = t / ad;                                        // UDIV: digit, < 2^16
            rem = t - q * ad;                                                 // MLS: remainder, < ad
            m = (m << 16) | q;                                                // Append the digit
        }
        m_ = m + 1u;                                                          // floor(e * 2^64 / ad) + 1
        sh_ = (uint8_t)(l - 1u);                                              // Post-shift
    }

    /**
     * @brief Exact @p n / d, truncated toward zero.
     *
     * The magnitude is divided unsigned (INT64_MIN maps to 2^63 without
     * overflow) and the sign is restored from the XOR of the operand signs.
     */
    __attribute__((always_inline)) int64_t div(int64_t n) const {
        const uint32_t n_neg = (uint32_t)((uint64_t)n >> 63);                 // Sign of the dividend
        const uint64_t u = n_neg ? (0u - (uint64_t)n) : (uint64_t)n;          // |n|
        uint64_t q;                                                           // |n| / |d|
        if (one_) {                                                           // |d| <= 1
            q = u;
        } else {
            const uint64_t t = umulhi64(m_, u);                               // floor(m' * u / 2^64)
            const uint64_t v = t + ((u - t) >> 1);                            // Add the implicit u * 2^64 term
            // v >> sh_ with sh_ in 0..15, spelled as 32-bit shifts so the
            // compiler does not emit the generic >= 32 path. (hi << 1) << (31 - sh)
            // is hi << (32 - sh) without the undefined shift by 32 at sh == 0.
            const uint32_t v_lo = (uint32_t)v;                                // Low word
            const uint32_t v_hi = (uint32_t)(v >> 32);                        // High word
            const uint32_t q_lo = (v_lo >> sh_) | ((v_hi << 1) << (31u - sh_));   // Low word of the quotient
            const uint32_t q_hi = v_hi >> sh_;                                // High word of the quotient
            q = ((uint64_t)q_hi << 32) | q_lo;
        }
        return ((n_neg ^ neg_) != 0u) ? (int64_t)(0u - q) : (int64_t)q;       // Restore the sign
    }
};

/**
 * @brief Exact @p n / @p d for a divisor that changes every call.
 *
 * Long division in base 2^16 with the hardware divider. After the high word
 * is divided, the remainder is below |d| < 2^16, so each of the two remaining
 * partial dividends (remainder << 16 | next digit) fits 32 bits and its
 * quotient digit fits 16: three UDIVs, three MLS, no 64-bit arithmetic
 * beyond assembling the result. Truncates toward zero.
 *
 * @param n Dividend.
 * @param d Divisor; 0 yields 0 (UDIV by zero returns 0, no trap).
 */
__attribute__((always_inline)) static inline int64_t div64_16(int64_t n, int16_t d) {
    const uint32_t ad = (d < 0) ? (uint32_t)(-(int32_t)d) : (uint32_t)d;      // |d|, 0..32768
    const uint32_t n_neg = (uint32_t)((uint64_t)n >> 63);                     // Sign of the dividend
    const uint64_t u = n_neg ? (0u - (uint64_t)n) : (uint64_t)n;              // |n|
    const uint32_t hi = (uint32_t)(u >> 32);                                  // Digit 2 (32 bits)
    const uint32_t lo = (uint32_t)u;                                          // Digits 1 and 0 (16 bits each)
    const uint32_t q2 = hi / ad;                                              // UDIV: high quotient word
    uint32_t rem = hi - q2 * ad;                                              // MLS: < ad < 2^16
    uint32_t t = (rem << 16) | (lo >> 16);                                    // Partial dividend, < 2^32
    const uint32_t q1 = t / ad;                                               // UDIV: digit, < 2^16
    rem = t - q1 * ad;                                                        // MLS
    t = (rem << 16) | (lo & 0xFFFFu);                                         // Last partial dividend
    const uint32_t q0 = t / ad;                                               // UDIV: digit, < 2^16
    const uint64_t q = ((uint64_t)q2 << 32) | (q1 << 16) | q0;                // Assemble |n| / |d|
    return (((n_neg ^ (uint32_t)(d < 0)) != 0u)) ? (int64_t)(0u - q) : (int64_t)q;  // Restore the sign
}

/**
 * @brief Exact unsigned @p n / @p d for a 32-bit divisor, 64-bit quotient.
 *
 * Step 1 divides the high word with one UDIV, leaving a remainder r < d.
 * Step 2 is Knuth's Algorithm D (Hacker's Delight 9-4, `divlu`) on the
 * 64-bit value (r : n_lo), whose quotient fits 32 bits because r < d:
 *  - normalize d so its top bit is set (s = clz d) and shift the dividend by
 *    the same amount; the digits are then base 2^16 with the divisor's top
 *    digit vn1 >= 2^15, which bounds every estimate error by 2;
 *  - each quotient digit is estimated as UDIV by vn1 and corrected while
 *    q * vn0 exceeds b * rhat + next digit (at most two decrements, the
 *    `rhat < b` exit keeps b * rhat inside 32 bits);
 *  - all intermediate values fit uint32: q1 <= 2^16 + 1 so q1 * vn0 < 2^32,
 *    and the partial dividends are formed modulo 2^32 on purpose (the true
 *    value is in range once the digit is correct).
 *
 * @param n Dividend.
 * @param d Divisor; 0 gives an unspecified result without trapping.
 */
static inline uint64_t udiv64_32(uint64_t n, uint32_t d) {
    const uint32_t n_hi = (uint32_t)(n >> 32);                    // High word of the dividend
    const uint32_t n_lo = (uint32_t)n;                            // Low word of the dividend
    const uint32_t q_hi = n_hi / d;                               // UDIV: high quotient word
    const uint32_t r = n_hi - q_hi * d;                           // MLS: remainder into the low step, r < d

    // --- 64/32 -> 32: (r : n_lo) / d with r < d (Knuth D, base 2^16) ---
    const uint32_t b = 65536u;                                    // Digit base
    const uint32_t s = (uint32_t)__builtin_clz(d);                // Normalization shift, 0..31 (CLZ)
    const uint32_t v = d << s;                                    // Normalized divisor, top bit set
    const uint32_t vn1 = v >> 16;                                 // Its top digit, >= 2^15
    const uint32_t vn0 = v & 0xFFFFu;                             // Its low digit
    const uint32_t un32 = (r << s) | ((s != 0u) ? (n_lo >> (32u - s)) : 0u);  // Normalized dividend, top 32 bits
    const uint32_t un10 = n_lo << s;                              // Normalized dividend, low 32 bits
    const uint32_t un1 = un10 >> 16;                              // Its digits
    const uint32_t un0 = un10 & 0xFFFFu;

    uint32_t q1 = un32 / vn1;                                     // UDIV: first digit estimate, <= 2^16 + 1
    uint32_t rhat = un32 - q1 * vn1;                              // MLS: partial remainder, < vn1
    while ((q1 >= b) || (q1 * vn0 > b * rhat + un1)) {           // Estimate too large (at most twice)
        --q1;
        rhat += vn1;
        if (rhat >= b) {                                          // Further tests would overflow and cannot fire
            break;
        }
    }
    const uint32_t un21 = un32 * b + un1 - q1 * v;                // Multiply and subtract, modulo 2^32

    uint32_t q0 = un21 / vn1;                                     // UDIV: second digit estimate
    rhat = un21 - q0 * vn1;                                       // MLS
    while ((q0 >= b) || (q0 * vn0 > b * rhat + un0)) {           // Same correction for the low digit
        --q0;
        rhat += vn1;
        if (rhat >= b) {
            break;
        }
    }
    const uint32_t q_lo = q1 * b + q0;                            // Low quotient word
    return ((uint64_t)q_hi << 32) | q_lo;
}
