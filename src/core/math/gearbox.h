/**
 * @file gearbox.h
 * @brief Gear-ratio conversion between the motor shaft and the output shaft.
 *
 * A positioning system frequently drives a load through a gearbox: the motor
 * turns `mul` times for every `div` turns of the output shaft. Every position
 * command, measured position, velocity, acceleration and jerk limit then has
 * to cross that boundary, and it has to cross it exactly: a ratio stored as a
 * float loses whole revolutions after a few hundred turns, while the integer
 * ratio `mul/div` with int16 terms does not drift at all.
 *
 * The class keeps the ratio in three interchangeable forms:
 *  - `mul_` / `div_` (int16) - the exact integer ratio, fit for the network
 *    protocol and for flash storage;
 *  - `ratio_ = mul/div` and `inv_ratio_ = div/mul` (float) - precomputed once
 *    in set_ratio(), used by every float32 conversion (velocity, acceleration,
 *    jerk) so the hot path costs a single FMUL;
 *  - two DivMagic16 reciprocals - precomputed exact int64 / int16 divisions
 *    (see div64.h), so an integer position conversion is one 64x64->128 high
 *    multiply plus a shift (~30 cycles) instead of libgcc's shift-subtract
 *    loop.
 *
 * ## Integer position pipeline (pos_int2ext / pos_ext2int)
 *
 * Positions are Q32.32 fixed point in revolutions inside an int64. The raw
 * conversion `pos * mul / div` needs up to 47 headroom bits for an extreme
 * int16 ratio, which the 32.32 split cannot always provide. Both methods
 * therefore:
 *  1. shift the input right by `kPosShift` (8 bits), trading the sub-revolution
 *     fractional tail for headroom - the low bits that survive the round trip
 *     are lost anyway (well above the tolerated 5-bit fractional loss), while
 *     the revolution integer part is kept intact;
 *  2. multiply by the int16 factor and divide by the other one with DivMagic16;
 *  3. shift the result back left by `kPosShift`, restoring the Q32.32 scale.
 *
 * ## Overflow behaviour
 *
 * Position is cyclic: crossing the 2^32-revolution boundary is normal, and an
 * oversized value must simply wrap, not saturate or trap. All multiplies and
 * shifts are therefore done in uint64 arithmetic, where wrap-around is fully
 * defined by the standard; the wrapped bit pattern is reinterpreted as int64
 * before the sign-exact DivMagic16 division, so the conversion remains
 * consistent across the wrap (a point one count before the boundary maps to
 * one count after it). DivMagic16 itself also copes with INT64_MIN.
 *
 * Divisors of zero are degenerate and are mapped to 1 in set_ratio() (the
 * identity ratio), which keeps both the float and the integer paths finite.
 *
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#pragma once

#include <stdint.h>

#include "div64.h"



/**
 * @brief Motor-to-output ratio holder and converter.
 *
 * `direct()` converts motor-shaft quantities to output-shaft ones (x mul/div),
 * `inverse()` does the opposite (x div/mul). Integer position conversions are
 * exact modulo the intentional low-bit loss of the pre-shift; float velocity,
 * acceleration and jerk conversions reuse the precomputed float multipliers.
 */
class Gearbox {
private:
    int16_t mul_;          ///< Motor turns per ratio step (direct-ratio numerator)
    int16_t div_;          ///< Output turns per ratio step (direct-ratio denominator)
    float ratio_;          ///< Precomputed mul / div, motor -> output multiplier
    float inv_ratio_;      ///< Precomputed div / mul, output -> motor multiplier
    DivMagic16 by_div_;    ///< Precomputed reciprocal of div_ for exact int64 division
    DivMagic16 by_mul_;    ///< Precomputed reciprocal of mul_ for exact int64 division

public:
    /// Fractional bits sacrificed on the position input to bound the intermediate product.
    static constexpr int kPosShift = 8;

    /** @brief Default: 1:1 direct drive, no conversion error anywhere. */
    constexpr Gearbox()
        : mul_(1)          // Identity numerator
        , div_(1)          // Identity denominator
        , ratio_(1.0f)     // Motor == output
        , inv_ratio_(1.0f) // Output == motor
    {}

    /**
     * @brief Install a new ratio and precompute every derived multiplier.
     *
     * Both float multipliers are computed here so the conversion methods never
     * divide at runtime, and both DivMagic16 reciprocals are rebuilt (four
     * UDIVs each, once per ratio change instead of once per conversion).
     *
     * @param mul Motor turns per output turn (numerator of the direct ratio).
     * @param div Output turns per motor turn (denominator of the direct ratio);
     *            0 is remapped to 1 on either side to keep the ratio finite.
     */
    void set_ratio(int16_t mul, int16_t div) {
        mul_ = (mul == 0) ? static_cast<int16_t>(1) : mul;   // Degenerate numerator -> identity
        div_ = (div == 0) ? static_cast<int16_t>(1) : div;   // Degenerate denominator -> identity
        ratio_ = static_cast<float>(mul_) / static_cast<float>(div_);     // Motor -> output float gain
        inv_ratio_ = static_cast<float>(div_) / static_cast<float>(mul_); // Output -> motor float gain
        by_div_.set(div_); // Reciprocal for the integer direct conversion
        by_mul_.set(mul_); // Reciprocal for the integer inverse conversion
    }

    /** @brief Convert a motor-shaft position (Q32.32 rev) to the output shaft. */
    int64_t direct(int64_t pos_mot) const {
        return pos_int2ext(pos_mot); // Wrapper over the exact integer pipeline
    }

    /** @brief Convert an output-shaft position (Q32.32 rev) back to the motor shaft. */
    int64_t inverse(int64_t pos_out) const {
        return pos_ext2int(pos_out); // Wrapper over the exact integer pipeline
    }

    /**
     * @brief Exact-ish motor -> output position: pos * mul / div.
     *
     * Shifts the input down by kPosShift first (headroom, at the cost of the
     * low fractional bits), multiplies by mul_, divides by div_ with the
     * precomputed exact reciprocal and shifts back up. Any overflow wraps
     * through the int64 circle by design, matching the cyclic Q32.32 position.
     *
     * @param pos Motor position, Q32.32 revolutions.
     * @return Output position, Q32.32 revolutions (low fractional bits zeroed).
     */
    int64_t pos_int2ext(int64_t pos) const {
        uint64_t p = static_cast<uint64_t>(pos >> kPosShift);        // Drop the fractional tail for headroom
        p *= static_cast<uint64_t>(static_cast<int64_t>(mul_));      // Scale up; wrap-around is intended and defined
        return static_cast<int64_t>(static_cast<uint64_t>(           // Reinterpret the wrapped pattern as signed
            by_div_.div(static_cast<int64_t>(p))) << kPosShift);     // Exact /div_, restore the Q32.32 scale
    }

    /**
     * @brief Exact-ish output -> motor position: pos * div / mul.
     *
     * Mirrors pos_int2ext(): pre-shift, multiply by div_, divide by mul_ with
     * the precomputed exact reciprocal, restore the shift. Overflow wraps.
     *
     * @param pos Output position, Q32.32 revolutions.
     * @return Motor position, Q32.32 revolutions (low fractional bits zeroed).
     */
    int64_t pos_ext2int(int64_t pos) const {
        uint64_t p = static_cast<uint64_t>(pos >> kPosShift);        // Drop the fractional tail for headroom
        p *= static_cast<uint64_t>(static_cast<int64_t>(div_));      // Scale up; wrap-around is intended and defined
        return static_cast<int64_t>(static_cast<uint64_t>(           // Reinterpret the wrapped pattern as signed
            by_mul_.div(static_cast<int64_t>(p))) << kPosShift);     // Exact /mul_, restore the Q32.32 scale
    }

    /** @brief Velocity motor -> output, rev/s. Single precomputed FMUL. */
    float vel_int2ext(float vel) const {
        return vel * ratio_; // Linear in the ratio
    }

    /** @brief Velocity output -> motor, rev/s. Single precomputed FMUL. */
    float vel_ext2int(float vel) const {
        return vel * inv_ratio_; // Linear in the inverted ratio
    }

    /** @brief Acceleration motor -> output, rev/s^2. Same linear gain as velocity. */
    float acc_int2ext(float acc) const {
        return acc * ratio_; // First derivative of velocity scales linearly
    }

    /** @brief Acceleration output -> motor, rev/s^2. Same linear gain as velocity. */
    float acc_ext2int(float acc) const {
        return acc * inv_ratio_; // First derivative of velocity scales linearly
    }

    /** @brief Jerk motor -> output, rev/s^3. Same linear gain as velocity. */
    float jerk_int2ext(float jerk) const {
        return jerk * ratio_; // Second derivative of velocity scales linearly
    }

    /** @brief Jerk output -> motor, rev/s^3. Same linear gain as velocity. */
    float jerk_ext2int(float jerk) const {
        return jerk * inv_ratio_; // Second derivative of velocity scales linearly
    }
};

