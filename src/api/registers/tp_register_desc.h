#pragma once

/**
 * @file tp_register_desc.h
 * @brief TPCP v1 - the register descriptor model and its value helpers.
 *
 * A register is described by a type, a flag set and a pair of accessor
 * function pointers. The accessors in the table are capture-less lambdas,
 * which decay to plain function pointers - so the whole map lands in flash and
 * costs no RAM.
 *
 * TpRegContext is what an accessor is handed: the three places a register can
 * live (live firmware state, the configuration shadow, the identity) plus two
 * out-flags the engine reads back - `dirty` to schedule a save, `hw_reapply`
 * to re-run tp_apply_config once the write lands.
 *
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#include "../../lib/lib.h"  // TunePulseHeart, reached through TpRegContext
#include "../tp_node.h"      // TpNodeConfig and TpNodeIdentity, the other two context targets
// ============================================================================
// Register descriptor model
// ============================================================================

/** How the 32-bit register payload is to be interpreted. */
enum TpRegType : uint8_t {
    TP_T_U8  = 0,  ///< Unsigned 8-bit, zero-extended.
    TP_T_U16 = 1,  ///< Unsigned 16-bit, zero-extended.
    TP_T_U32 = 2,  ///< Unsigned 32-bit.
    TP_T_I8  = 3,  ///< Signed 8-bit, sign-extended.
    TP_T_I16 = 4,  ///< Signed 16-bit, sign-extended.
    TP_T_I32 = 5,  ///< Signed 32-bit.
    TP_T_F32 = 6,  ///< IEEE-754 single precision, raw bit pattern.
    TP_T_Q16 = 7   ///< Signed Q16.16 fixed point (revolutions).
};

/** Access and behaviour flags of one register. */
enum TpRegFlag : uint8_t {
    TP_RF_READ     = 1u << 0,  ///< Readable.
    TP_RF_WRITE    = 1u << 1,  ///< Writable.
    TP_RF_PERSIST  = 1u << 2,  ///< Included in the persisted configuration record.
    TP_RF_DISABLED = 1u << 3,  ///< Writable only while the output stage is disabled.
    TP_RF_REBOOT   = 1u << 4   ///< Takes effect after a save and a reboot.
};

/** Shorthand for the common read/write combination. */
#define TP_RF_RW (TP_RF_READ | TP_RF_WRITE)

/**
 * @brief Everything an accessor needs to reach the firmware.
 *
 * Passed by reference into every accessor so the table itself stays free of
 * global references and can be unit tested against a stub heart.
 */
struct TpRegContext {
    TunePulseHeart* tp;         ///< Live firmware state.
    TpNodeConfig*   cfg;        ///< Persisted configuration shadow.
    TpNodeIdentity* id;         ///< Immutable device identity (UID).
    bool            dirty;      ///< Set when a write touched a persisted register.
    bool            hw_reapply; ///< Set when a write needs tp_apply_config() to run.
};

/** @brief One register: identifier, type, access flags and its accessors. */
struct TpRegDesc {
    uint16_t  id;     ///< Register identifier.
    TpRegType type;   ///< Payload interpretation.
    uint8_t   flags;  ///< TpRegFlag bits.
    bool (*get)(TpRegContext& c, uint32_t& out);  ///< Read accessor (nullptr = write only).
    bool (*set)(TpRegContext& c, uint32_t value); ///< Write accessor (nullptr = read only).
};
// ============================================================================
// Accessor helpers
// ============================================================================

/** @brief Reinterpret a float as its raw 32-bit pattern (register payload). */
static inline uint32_t tp_reg_from_f32(float v) {
    uint32_t bits;                        // Raw representation
    __builtin_memcpy(&bits, &v, 4);       // Bit-cast without aliasing UB
    return bits;
}

/** @brief Reinterpret a raw 32-bit register payload as a float. */
static inline float tp_reg_to_f32(uint32_t v) {
    float value;                          // Reconstructed float
    __builtin_memcpy(&value, &v, 4);      // Bit-cast without aliasing UB
    return value;
}

/** @brief Reject non-finite and negative values on a physical limit write. */
static inline bool tp_reg_positive(float v) {
    return (v == v) && (v > 0.0f) && (v < 1.0e9f);  // NaN fails the self-compare
}
