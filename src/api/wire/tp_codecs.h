#pragma once

/**
 * @file tp_codecs.h
 * @brief TPCP v1 - the compact fixed-point and mini-float codecs.
 *
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#include <stdint.h>  // Fixed-width integer types

#include "tp_ids.h"  // TP_UID_BYTES

// ============================================================================
// Encodings
// ============================================================================
//
// The protocol uses two mini-float formats and a family of fixed-point
// integers. All of them are pure bit manipulation - no FPU work, no branches on
// the hot path - so a 20 kHz control loop can pack telemetry without cost.
//
//  bfloat16 (1.8.7)  : the top 16 bits of an IEEE-754 float. Full float range,
//                      ~0.4 % relative error. Used for physical parameters.
//                      NaN (0x7FC0) marks "unknown" / "leave unchanged".
//  float24  (1.8.15) : the top 24 bits of an IEEE-754 float. Full range,
//                      ~0.008 % relative error. Used for velocity.
//  q1.23.24 (48 bit) : signed position in revolutions with 24 fractional bits,
//                      +/-8388608 revolutions. The wire form of a position.
//  q1.23.32 (56 bit) : signed revolutions with 32 fractional bits - the raw
//                      encoder word beside its turn count.
//  q1.15.32 (48 bit) : signed revolutions with 32 fractional bits, +/-32768 rev.
//  q1.15.16 (32 bit) : signed revolutions with 16 fractional bits - the
//                      firmware's native encoder unit.
//  q1.7.16  (24 bit) : signed following error / masked position, +/-128 rev.
//  q1.3.8   (12 bit) : signed time scale, 256 = 1.0x, range -8.0 .. +7.996.
//  q1.1.4   (6 bit)  : signed normalized error, 16 = 1.0, range -2.0 .. +1.94.
//
// Multi-byte integers are little-endian; the burst frame (see
// chunks/tp_chunk_motion.h) is the documented exception.
// ============================================================================

/** bfloat16 quiet NaN: "not available" on reads, "leave unchanged" on writes. */
#define TP_BF16_NAN 0x7FC0u

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

/** @brief True when a bfloat16 payload is a finite number (not NaN, not infinity). */
static inline bool tp_bf16_is_finite(uint16_t raw) {
    return (raw & 0x7F80u) != 0x7F80u;      // Exponent all ones marks Inf / NaN
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

/** @brief True when a float24 payload is a finite number. */
static inline bool tp_f24_is_finite(uint32_t raw) {
    return (raw & 0x7F8000u) != 0x7F8000u;  // Exponent all ones marks Inf / NaN
}

// ============================================================================
// Little-endian integer fields
// ============================================================================

/** @brief Store an unsigned 24-bit value little-endian. */
static inline void tp_pack_u24(uint8_t* dst, uint32_t v) {
    dst[0] = (uint8_t)(v);                  // Bits 0-7
    dst[1] = (uint8_t)(v >> 8);             // Bits 8-15
    dst[2] = (uint8_t)(v >> 16);            // Bits 16-23
}

/** @brief Load an unsigned 24-bit little-endian value. */
static inline uint32_t tp_unpack_u24(const uint8_t* src) {
    return (uint32_t)src[0] | ((uint32_t)src[1] << 8) | ((uint32_t)src[2] << 16);
}

/** @brief Store the low 24 bits of a signed value little-endian (q1.7.16). */
static inline void tp_pack_i24(uint8_t* dst, int32_t v) {
    tp_pack_u24(dst, (uint32_t)v);          // Two's-complement bit pattern
}

/** @brief Load a little-endian 24-bit field as a sign-extended 32-bit value. */
static inline int32_t tp_unpack_i24(const uint8_t* src) {
    return (int32_t)(tp_unpack_u24(src) << 8) >> 8;  // Sign-extend from bit 23
}

/**
 * @brief Store a signed value as a little-endian 48-bit field.
 * @param dst Destination buffer, at least 6 bytes.
 * @param v   Signed value (q1.23.24 position or q1.15.32 offset).
 */
static inline void tp_pack_i48(uint8_t* dst, int64_t v) {
    const uint64_t u = (uint64_t)v;         // Two's-complement bit pattern
    for (uint32_t i = 0; i < 6u; ++i) {
        dst[i] = (uint8_t)(u >> (8u * i));  // Byte i of the little-endian field
    }
}

/**
 * @brief Load a little-endian 48-bit field into a sign-extended 64-bit value.
 * @param src Source buffer, at least 6 bytes.
 */
static inline int64_t tp_unpack_i48(const uint8_t* src) {
    uint64_t bits = 0;                      // Assembled field
    for (uint32_t i = 0; i < 6u; ++i) {
        bits |= (uint64_t)src[i] << (8u * i);
    }
    return (int64_t)(bits << 16) >> 16;     // Sign-extend from bit 47
}

/**
 * @brief Store a signed value as a little-endian 56-bit field (q1.23.32).
 * @param dst Destination buffer, at least 7 bytes.
 */
static inline void tp_pack_i56(uint8_t* dst, int64_t v) {
    const uint64_t u = (uint64_t)v;         // Two's-complement bit pattern
    for (uint32_t i = 0; i < 7u; ++i) {
        dst[i] = (uint8_t)(u >> (8u * i));  // Byte i of the little-endian field
    }
}

/**
 * @brief Load a little-endian 56-bit field into a sign-extended 64-bit value.
 * @param src Source buffer, at least 7 bytes.
 */
static inline int64_t tp_unpack_i56(const uint8_t* src) {
    uint64_t bits = 0;                      // Assembled field
    for (uint32_t i = 0; i < 7u; ++i) {
        bits |= (uint64_t)src[i] << (8u * i);
    }
    return (int64_t)(bits << 8) >> 8;       // Sign-extend from bit 55
}

/** @brief Store the 48-bit HashUID little-endian (the top 16 bits of the word are ignored). */
static inline void tp_pack_uid(uint8_t* dst, uint64_t uid) {
    for (uint32_t i = 0; i < TP_UID_BYTES; ++i) {
        dst[i] = (uint8_t)(uid >> (8u * i));  // Byte i of the little-endian field
    }
}

/** @brief Load a little-endian 48-bit HashUID into a 64-bit word. */
static inline uint64_t tp_unpack_uid(const uint8_t* src) {
    uint64_t uid = 0;                       // Assembled identity
    for (uint32_t i = 0; i < TP_UID_BYTES; ++i) {
        uid |= (uint64_t)src[i] << (8u * i);
    }
    return uid;
}

// ============================================================================
// Position conversions
// ============================================================================
//
// The firmware measures position in q1.15.16 (int32, 1/65536 rev). The wire
// carries q1.23.24 for absolute positions and q1.15.32 for the stored offsets,
// so a conversion is a shift in either direction.

/** @brief Widen a native q1.15.16 position into the wire's q1.23.24. */
static inline int64_t tp_q16_to_q24(int32_t q16) {
    return (int64_t)q16 << 8;               // Eight more fractional bits
}

/**
 * @brief Narrow a wire q1.23.24 position into the native q1.15.16, saturating.
 *
 * The native range is +/-32768 revolutions; a wire value outside it is
 * clamped rather than wrapped, so a bad frame can at worst command the far
 * end of the range instead of an arbitrary point.
 */
static inline int32_t tp_q24_to_q16(int64_t q24) {
    const int64_t q16 = q24 >> 8;           // Drop the eight extra fractional bits
    if (q16 > 2147483647LL) {
        return 2147483647;                  // Saturate positive
    }
    if (q16 < -2147483648LL) {
        return (int32_t)-2147483648LL;      // Saturate negative
    }
    return (int32_t)q16;
}

/** @brief Widen a native q1.15.16 value into q1.15.32 (48-bit offsets). */
static inline int64_t tp_q16_to_q32(int32_t q16) {
    return (int64_t)q16 << 16;              // Sixteen more fractional bits
}

/** @brief Narrow a q1.15.32 value into the native q1.15.16 (truncating). */
static inline int32_t tp_q32_to_q16(int64_t q32) {
    return (int32_t)(q32 >> 16);            // The 48-bit range fits int32 exactly
}

// ============================================================================
// Small normalized fields
// ============================================================================

/**
 * @brief Convert a Q10 time scale (1024 = 1.0x) into the 12-bit q1.3.8 field.
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
 * @brief Convert a 12-bit q1.3.8 time-scale field back into Q10.
 * @param raw 12-bit payload, right-aligned.
 * @return Signed time scale with 10 fractional bits (1024 = 1.0x).
 */
static inline int32_t tp_unpack_timescale(uint16_t raw) {
    int16_t q8 = (int16_t)((uint16_t)(raw << 4)) >> 4;  // Sign-extend the 12-bit field
    return (int32_t)q8 << 2;                            // Q8 -> Q10
}

/**
 * @brief Pack a normalized ratio into the 6-bit q1.1.4 error field of the PulseBeat.
 *
 * q1.1.4 covers -2.0 .. +1.9375 with 1/16 resolution: enough to show the
 * master how close the drive is to its following-error limit in six bits.
 *
 * @param ratio Normalized value (typically error divided by its limit).
 * @return 6-bit two's-complement payload, right-aligned.
 */
static inline uint8_t tp_pack_q1_1_4(float ratio) {
    int32_t q = (int32_t)(ratio * 16.0f + (ratio >= 0.0f ? 0.5f : -0.5f));  // Scale and round away from zero
    if (q > 31) {
        q = 31;                             // Saturate at +1.9375
    }
    if (q < -32) {
        q = -32;                            // Saturate at -2.0
    }
    return (uint8_t)((uint32_t)q & 0x3Fu);  // Six-bit two's complement
}

/**
 * @brief Encode a temperature as the protocol's unsigned Celsius byte.
 *
 * 0 means "no data" and 255 means "over range", so the usable span is
 * 1 .. 254 degC. Sub-zero readings clamp to 1 rather than aliasing "no data".
 *
 * @param celsius Temperature in degrees Celsius (NaN = no data).
 */
static inline uint8_t tp_pack_temp_u8(float celsius) {
    if (celsius != celsius) {
        return 0u;                          // NaN: no data
    }
    if (celsius >= 254.5f) {
        return 255u;                        // Over range
    }
    int32_t q = (int32_t)(celsius + 0.5f);  // Round to the nearest degree
    if (q < 1) {
        q = 1;                              // Keep 0 free as the "no data" marker
    }
    return (uint8_t)q;
}

/**
 * @brief Encode a signed fraction as the int8 usage / load byte (-128 = overload).
 * @param ratio Fraction of the limit in use; negative values mean regeneration.
 */
static inline uint8_t tp_pack_usage_i8(float ratio) {
    if (ratio != ratio) {
        return 0u;                          // NaN: nothing to report
    }
    if (ratio > 1.0f) {
        return 0x80u;                       // Beyond the limit: -128 marks overload
    }
    if (ratio < -1.0f) {
        ratio = -1.0f;                      // Saturate the regen side
    }
    return (uint8_t)(int8_t)(int32_t)(ratio * 127.0f);
}
