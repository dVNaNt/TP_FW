#pragma once

/**
 * @file tp_codecs.h
 * @brief TPCP v1 - the compact fixed-point and mini-float codecs.
 *
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#include <stdint.h>  // Fixed-width integer types

// ============================================================================
// Fixed-point codecs
// ============================================================================
//
// The protocol uses four compact float formats plus two fixed-point integer
// formats. All of them are pure bit manipulation - no FPU work, no branches on
// the hot path - so a 20 kHz control loop can pack telemetry without cost.
//
//  bfloat16 (1.8.7)  : the top 16 bits of an IEEE-754 float. Full float range,
//                      ~0.4 % relative error. Used for physical parameters.
//  float24  (1.8.15) : the top 24 bits of an IEEE-754 float. Full range,
//                      ~0.008 % relative error. Used for velocity.
//  Q1.31.16 (48 bit) : signed position in revolutions with 1/65536 resolution,
//                      +/-32768 revolutions. The native encoder format widened.
//  Q1.7.16  (24 bit) : signed following error, +/-128 revolutions.
//  Q1.3.8   (12 bit) : signed time scale, 256 = 1.0x, range -8.0 .. +7.996.
//  Q1.1.6   (8 bit)  : signed normalized error, 64 = 1.0, range -2.0 .. +1.98.
// ============================================================================

/**
 * @brief Pack an IEEE-754 float into bfloat16 (1 sign / 8 exponent / 7 mantissa).
 *
 * Truncating conversion: the low 16 bits of the representation are discarded.
 * This is the same encoding the motor parameter records use in flash, so a
 * value read over CAN is bit-identical to the value stored in NVM.
 *
 * @param value Single-precision input.
 * @return 16-bit bfloat16 payload.
 */
static inline uint16_t tp_pack_bf16(float value) {
    uint32_t bits;                          // Raw IEEE-754 representation
    __builtin_memcpy(&bits, &value, 4);     // Bit-cast float -> uint32 (no aliasing UB)
    return (uint16_t)(bits >> 16);          // Keep sign, exponent and 7 mantissa bits
}

/**
 * @brief Unpack a bfloat16 payload back into a float.
 * @param raw 16-bit bfloat16 payload.
 * @return Reconstructed single-precision value.
 */
static inline float tp_unpack_bf16(uint16_t raw) {
    uint32_t bits = ((uint32_t)raw) << 16;  // Restore the discarded mantissa as zeros
    float value;                            // Reconstructed float
    __builtin_memcpy(&value, &bits, 4);     // Bit-cast uint32 -> float
    return value;
}

/**
 * @brief Pack an IEEE-754 float into float24 (1 sign / 8 exponent / 15 mantissa).
 *
 * float32 and float24 share the sign and exponent fields, so the conversion is
 * a pure truncation of the 8 lowest mantissa bits: two shifts, no branching.
 *
 * @param value Single-precision input.
 * @return 24-bit payload, right-aligned in a 32-bit container.
 */
static inline uint32_t tp_pack_f24(float value) {
    uint32_t bits;                          // Raw IEEE-754 representation
    __builtin_memcpy(&bits, &value, 4);     // Bit-cast float -> uint32
    return (bits >> 8) & 0x00FFFFFFu;       // Drop the 8 lowest mantissa bits
}

/**
 * @brief Unpack a float24 payload back into a float.
 *
 * A pure left shift: float24 is the TOP 24 bits of an IEEE-754 float, so
 * restoring it only has to put those bits back and leave the discarded low
 * mantissa bits as zero.
 *
 * There is deliberately no sign extension here. Sign extension belongs to
 * two's-complement integers; IEEE-754 is sign-magnitude, and its sign bit
 * already lands on bit 31 after the shift. Replicating it downwards - which
 * the original draft codec did - overwrites bits 30..24, i.e. the entire
 * exponent, turning every negative value into a huge number, an infinity or a
 * NaN. That defect is what this comment exists to keep from coming back.
 *
 * @param raw 24-bit payload, right-aligned.
 * @return Reconstructed single-precision value.
 */
static inline float tp_unpack_f24(uint32_t raw) {
    const uint32_t bits = (raw & 0x00FFFFFFu) << 8;  // Restore sign, exponent and 15 mantissa bits
    float value;                                     // Reconstructed float
    __builtin_memcpy(&value, &bits, 4);              // Bit-cast uint32 -> float
    return value;
}

/**
 * @brief Store a signed Q16.16 value as a little-endian 48-bit field.
 * @param dst    Destination buffer, at least 6 bytes.
 * @param q16_16 Signed value with 16 fractional bits (revolutions).
 */
static inline void tp_pack_i48(uint8_t* dst, int64_t q16_16) {
    uint64_t u = (uint64_t)q16_16;          // Two's-complement bit pattern
    dst[0] = (uint8_t)(u);                  // Bits 0-7
    dst[1] = (uint8_t)(u >> 8);             // Bits 8-15
    dst[2] = (uint8_t)(u >> 16);            // Bits 16-23
    dst[3] = (uint8_t)(u >> 24);            // Bits 24-31
    dst[4] = (uint8_t)(u >> 32);            // Bits 32-39
    dst[5] = (uint8_t)(u >> 40);            // Bits 40-47
}

/**
 * @brief Load a little-endian 48-bit field into a signed Q16.16 value.
 *
 * Sign extension is branchless: shift bit 47 up to bit 63, then shift back
 * arithmetically.
 *
 * @param src Source buffer, at least 6 bytes.
 * @return Signed value with 16 fractional bits.
 */
static inline int64_t tp_unpack_i48(const uint8_t* src) {
    uint64_t bits = (uint64_t)src[0]              // Bits 0-7
                  | ((uint64_t)src[1] << 8)       // Bits 8-15
                  | ((uint64_t)src[2] << 16)      // Bits 16-23
                  | ((uint64_t)src[3] << 24)      // Bits 24-31
                  | ((uint64_t)src[4] << 32)      // Bits 32-39
                  | ((uint64_t)src[5] << 40);     // Bits 40-47
    return (int64_t)(bits << 16) >> 16;           // Sign-extend from bit 47
}

/**
 * @brief Store the low 24 bits of a signed value as a little-endian field.
 * @param dst Destination buffer, at least 3 bytes.
 * @param v   Signed value (Q16.16 following error, or a raw 24-bit quantity).
 */
static inline void tp_pack_i24(uint8_t* dst, int32_t v) {
    uint32_t u = (uint32_t)v;               // Two's-complement bit pattern
    dst[0] = (uint8_t)(u);                  // Bits 0-7
    dst[1] = (uint8_t)(u >> 8);             // Bits 8-15
    dst[2] = (uint8_t)(u >> 16);            // Bits 16-23
}

/**
 * @brief Load a little-endian 24-bit field as a sign-extended 32-bit value.
 * @param src Source buffer, at least 3 bytes.
 * @return Sign-extended signed value.
 */
static inline int32_t tp_unpack_i24(const uint8_t* src) {
    uint32_t bits = (uint32_t)src[0]              // Bits 0-7
                  | ((uint32_t)src[1] << 8)       // Bits 8-15
                  | ((uint32_t)src[2] << 16);     // Bits 16-23
    return (int32_t)(bits << 8) >> 8;             // Sign-extend from bit 23
}

/**
 * @brief Convert a Q10 time scale (1024 = 1.0x) into the 12-bit Q1.3.8 field.
 *
 * Integer only: Q10 -> Q8 is a shift by two, followed by a symmetric
 * saturation into the 12-bit signed range.
 *
 * @param scale_q10 Signed time scale with 10 fractional bits.
 * @return 12-bit payload, right-aligned, two's complement.
 */
static inline uint16_t tp_pack_timescale(int32_t scale_q10) {
    int32_t q8 = scale_q10 >> 2;            // 1024 per unit -> 256 per unit
    if (q8 > 2047) {
        q8 = 2047;                          // Saturate at +7.996x
    }
    if (q8 < -2048) {
        q8 = -2048;                         // Saturate at -8.0x
    }
    return (uint16_t)((uint32_t)q8 & 0x0FFFu);  // Mask into the 12-bit field
}

/**
 * @brief Convert a 12-bit Q1.3.8 time-scale field back into Q10.
 * @param raw 12-bit payload, right-aligned.
 * @return Signed time scale with 10 fractional bits (1024 = 1.0x).
 */
static inline int32_t tp_unpack_timescale(uint16_t raw) {
    int16_t q8 = (int16_t)((uint16_t)(raw << 4)) >> 4;  // Sign-extend the 12-bit field
    return (int32_t)q8 << 2;                            // Q8 -> Q10
}

/**
 * @brief Pack a normalized ratio into the 8-bit Q1.1.6 error field.
 *
 * Q1.1.6 covers -2.0 .. +1.984 with 1/64 resolution: enough to show the master
 * how close the drive is to its following-error limit in a single byte.
 *
 * @param ratio Normalized value (typically error divided by its limit).
 * @return 8-bit two's-complement payload.
 */
static inline uint8_t tp_pack_q1_1_6(float ratio) {
    int32_t q = (int32_t)(ratio * 64.0f + (ratio >= 0.0f ? 0.5f : -0.5f));  // Scale and round away from zero
    if (q > 127) {
        q = 127;                            // Saturate at +1.984
    }
    if (q < -128) {
        q = -128;                           // Saturate at -2.0
    }
    return (uint8_t)((int8_t)q);            // Two's-complement byte
}

/**
 * @brief Encode a supply voltage as an unsigned byte with 0.25 V resolution.
 * @param volts Supply voltage in volts (0 .. 63.75 V).
 * @return Encoded byte; saturates instead of wrapping.
 */
static inline uint8_t tp_pack_volts_q2(float volts) {
    int32_t q = (int32_t)(volts * 4.0f + 0.5f);  // Convert to quarter-volt steps, rounded
    if (q < 0) {
        q = 0;                              // Clamp negatives (sensor glitch) to zero
    }
    if (q > 255) {
        q = 255;                            // Saturate at 63.75 V
    }
    return (uint8_t)q;
}

/**
 * @brief Encode a temperature as a signed byte in whole degrees Celsius.
 * @param celsius Temperature in degrees Celsius.
 * @return Encoded byte; INT8_MIN (-128) is reserved for "no data".
 */
static inline uint8_t tp_pack_temp_c(float celsius) {
    int32_t q = (int32_t)(celsius + (celsius >= 0.0f ? 0.5f : -0.5f));  // Round to the nearest degree
    if (q > 127) {
        q = 127;                            // Saturate at +127 degC
    }
    if (q < -127) {
        q = -127;                           // Keep -128 free as the "no data" marker
    }
    return (uint8_t)((int8_t)q);
}

