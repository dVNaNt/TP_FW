/**
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#pragma once

#include "../common.h"

/// @brief Compute sine and cosine for a normalized angle using a 64-point
///        quarter-wave lookup table with 8-bit linear interpolation.
///
/// The 16-bit input angle is treated as an unsigned bit field:
/// @code
///   bits [15:14]  quadrant index        (0 = Q1, 1 = Q2, 2 = Q3, 3 = Q4)
///   bits [13:8]   LUT index             (0..63, selects two adjacent samples)
///   bits [7:0]    interpolation fraction (0..255, sub-sample position)
/// @endcode
///
/// The LUT stores sin(i * PI/2 / 64) * 32767 for i in [0..64], covering the
/// first quarter wave (0° to 90°). All values are non-negative [0, 32767].
/// Cosine is derived from the same table via the mirrored index (64 - idx),
/// exploiting the identity cos(x) = sin(PI/2 - x).
///
/// @param angle    Input angle in i1.15 format. Full range [-32768, 32767]
///                 maps linearly to [-PI, PI). Passed as signed int16_t but
///                 internally reinterpreted as uint16_t for bit extraction.
/// @param out_sin  Output: sine value in i1.15 format, range [-32767, 32767].
/// @param out_cos  Output: cosine value in i1.15 format, range [-32767, 32767].
///
/// @note INT16_MIN (-32768 / 0x8000) is architecturally impossible on output:
///       - LUT values are strictly in [0, 32767] (no negatives by construction).
///       - Linear interpolation of two non-negative values stays in [0, 32767].
///       - Negating a value in [0, 32767] produces [-32767, 0]. Never -32768.
///       Therefore no runtime clamp is required inside this function.
///
/// @note Array bounds are safe by construction:
///       - idx in [0..63]  → idx+1 in [1..64]     → both within _LUT[65].
///       - idx_c = 64-idx in [1..64] → idx_c-1 in [0..63] → no underflow.
static inline void angle2sincos(int16_t angle, int16_t* out_sin, int16_t* out_cos) {

    // Quarter-wave sine LUT: 65 entries covering sin(0°) to sin(90°) in i1.15.
    // Generated offline: round(sin(i * PI/2 / 64) * 32767) for i in [0..64].
    // All values in [0x0000, 0x7FFF]. 0x8000 (INT16_MIN) is provably absent.
    // Placed in .rodata as a function-local static constexpr: single instance,
    // zero runtime initialisation cost, 130 bytes of Flash.
    static constexpr int16_t _LUT[65] = {
        0x0000, 0x0324, 0x0648, 0x096A, 0x0C8C, 0x0FAB, 0x12C8, 0x15E2, // idx  0.. 7
        0x18F9, 0x1C0B, 0x1F1A, 0x2223, 0x2528, 0x2826, 0x2B1F, 0x2E11, // idx  8..15
        0x30FB, 0x33DF, 0x36BA, 0x398C, 0x3C56, 0x3F17, 0x41CE, 0x447A, // idx 16..23
        0x471C, 0x49B4, 0x4C3F, 0x4EBF, 0x5133, 0x539B, 0x55F5, 0x5842, // idx 24..31
        0x5A82, 0x5CB3, 0x5ED7, 0x60EB, 0x62F1, 0x64E8, 0x66CF, 0x68A6, // idx 32..39
        0x6A6D, 0x6C23, 0x6DC9, 0x6F5E, 0x70E2, 0x7254, 0x73B5, 0x7504, // idx 40..47
        0x7641, 0x776B, 0x7884, 0x7989, 0x7A7C, 0x7B5C, 0x7C29, 0x7CE3, // idx 48..55
        0x7D89, 0x7E1D, 0x7E9C, 0x7F09, 0x7F61, 0x7FA6, 0x7FD8, 0x7FF5, // idx 56..63
        0x7FFF                                                             // idx 64 = sin(90°)
    };

    const uint16_t u     = (uint16_t)angle;   // Reinterpret i1.15 bits as unsigned for slicing
    const uint16_t quad  = u >> 14;            // Top 2 bits: quadrant selector (0..3)
    const uint16_t sub   = u & 0x3FFFu;        // Lower 14 bits: position within quadrant
    const uint16_t idx   = sub >> 8;           // Upper 6 of 14 bits: LUT index (0..63)
    const uint16_t frac  = sub & 0xFFu;        // Lower 8 of 14 bits: interpolation weight (0..255)
    const uint16_t idx_c = 64u - idx;          // Mirrored index for cosine via cos(x)=sin(PI/2-x)

    const int32_t a0 = _LUT[idx];              // sin sample at idx     (for sin interpolation)
    const int32_t a1 = _LUT[idx + 1];          // sin sample at idx+1   (next adjacent point)
    const int32_t b0 = _LUT[idx_c];            // cos sample at idx_c   (for cos interpolation)
    const int32_t b1 = _LUT[idx_c - 1];        // cos sample at idx_c-1 (next adjacent point, descending)

    // Linear interpolation in 32-bit to avoid overflow.
    // Result range: [0, 32767] since both endpoints and frac are non-negative.
    const int32_t a = a0 + (((a1 - a0) * int32_t(frac)) >> 8); // Interpolated sin magnitude
    const int32_t b = b0 + (((b1 - b0) * int32_t(frac)) >> 8); // Interpolated cos magnitude

    // Apply quadrant sign mapping. Both a and b are in [0, 32767], so
    // negation produces at most -32767 — safe to cast directly to int16_t.
    switch (quad) {
        case 0: *out_sin = int16_t( a); *out_cos = int16_t( b); break; // Q1: sin+, cos+
        case 1: *out_sin = int16_t( b); *out_cos = int16_t(-a); break; // Q2: sin+, cos-
        case 2: *out_sin = int16_t(-a); *out_cos = int16_t(-b); break; // Q3: sin-, cos-
        default:*out_sin = int16_t(-b); *out_cos = int16_t( a); break; // Q4: sin-, cos+
    }
}

/// Scales sine and cosine values by a given scale factor in i1.15 format.
///
/// ### Arguments
/// * `input` - A tuple `(sine, cosine)` representing sine and cosine components as `i1.15`.
/// * `scale` - A scaling factor in `i1.15`. Typically used to adjust amplitude.
///
/// ### Returns
/// * A tuple `(scaled_sine, scaled_cosine)` representing the scaled values.
/// 
/// ### Notes
/// * Uses `i32` internally to avoid overflow during scaling.
static inline void scale_sincos(int16_t in_sin, int16_t in_cos, int16_t scale, int16_t* out_sin, int16_t* out_cos) {
    // Convert to i32 to avoid overflow during calculations
    int32_t a = (int32_t)in_sin;
    int32_t b = (int32_t)in_cos;
    int32_t sc = (int32_t)scale;

    // Scale the sine and cosine values, shifting right to retain `i16` precision
    *out_sin = (int16_t)((a * sc) >> 15);
    *out_cos = (int16_t)((b * sc) >> 15);
}

/// Rotates a vector represented by sine and cosine components as `i1.15` using another vector (offset),
/// also represented by sine and cosine components.
///
/// ### Arguments
/// * `source` - A tuple `(source_sin, source_cos)` representing the sine and cosine components
///              of the vector to be rotated as `i1.15`.
/// * `offset` - A tuple `(offset_sin, offset_cos)` representing the sine and cosine components
///              of the rotation angle as `i1.15`.
///
/// ### Returns
/// * A tuple `(out_sin, out_cos)` - The sine and cosine components of the rotated vector as `i1.15`.
/// 
/// ### Notes
/// * Uses `i32` internally for calculations to avoid overflow.
/// * The final results are converted back to `i1.15` for consistency.
static inline void rotate_sincos(int16_t source_sin, int16_t source_cos, int16_t offset_sin, int16_t offset_cos, int16_t* out_sin, int16_t* out_cos) {
    // Convert input types to i32 to avoid overflow during calculations.
    int32_t ss = (int32_t)source_sin;
    int32_t sc = (int32_t)source_cos;
    int32_t os = (int32_t)offset_sin;
    int32_t oc = (int32_t)offset_cos;

    // Calculate the sine component of the rotated vector:
    // out_sin = source_sin * offset_cos + source_cos * offset_sin
    int32_t rs = ss * oc + sc * os;

    // Calculate the cosine component of the rotated vector:
    // out_cos = source_cos * offset_cos - source_sin * offset_sin
    int32_t rc = sc * oc - ss * os;

    // Scale back to `i16` by shifting right 15 bits
    *out_sin = (int16_t)(rs >> 15);
    *out_cos = (int16_t)(rc >> 15);
}


