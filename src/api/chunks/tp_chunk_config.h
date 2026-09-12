#pragma once

/**
 * @file tp_chunk_config.h
 * @brief TPCP v1 - register access and persistence frame layouts (opcodes 0x20 - 0x2F).
 *
 * One layout (TpRegAccess) serves register read, write and introspection alike:
 * on a request the info byte is zero, on a reply it carries the register type so
 * a generic host tool can decode a value it has no local map for.
 * 
 * The persistence frames cover both stores - the node-configuration partition
 * and the motor-profile partition - selected by TpParamTarget.
 *
 * Every struct is followed immediately by its own static_assert on sizeof: the
 * bitfield and padding rules that make these layouts land exactly on eight
 * bytes are compiler behaviour, so the assertion is the only thing standing
 * between a careless edit and a silent wire-format break. Keep them adjacent.
 *
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#include "../tp_wire.h"  // Opcodes, identifier map and the fixed-point codecs

// ============================================================================
// Register access and persistence (opcodes 0x20 - 0x2F)
// ============================================================================

/**
 * @brief 0x20 / 0x21 / 0x22 Register access frame.
 *
 * The same layout serves read, write and introspection. On a request the info
 * byte is zero; on a reply it carries the register type (TpRegType) so a
 * generic tool can decode the value without a local copy of the register map.
 */
struct TpRegAccess {
    uint8_t  cmd;    ///< Byte 0: TP_OP_REG_READ / TP_OP_REG_WRITE / TP_OP_REG_INFO.
    uint8_t  info;   ///< Byte 1: reply only - TpRegType of this register.
    uint16_t reg;    ///< Bytes 2-3: register identifier (LE).
    uint32_t value;  ///< Bytes 4-7: register value, encoded per TpRegType.
};
static_assert(sizeof(TpRegAccess) == 8, "TpRegAccess must fill one frame");

/** @brief 0x28 / 0x29 / 0x2A Persistence command: save, load or restore defaults. */
struct TpParamCmd {
    uint8_t  cmd;       ///< Byte 0: TP_OP_PARAM_SAVE / _LOAD / _DEFAULTS.
    uint8_t  target;    ///< Byte 1: TpParamTarget.
    uint16_t reserved;  ///< Bytes 2-3: reserved, write as zero.
    uint32_t key;       ///< Bytes 4-7: TP_DEFAULTS_KEY, required by TP_OP_PARAM_DEFAULTS only.
};
static_assert(sizeof(TpParamCmd) == 8, "TpParamCmd must fill one frame");

/** Storage areas addressable by the persistence commands. */
enum TpParamTarget : uint8_t {
    TP_PARAM_ALL    = 0x0u,  ///< Node configuration and motor profile together.
    TP_PARAM_CONFIG = 0x1u,  ///< Node configuration record only (address, limits, telemetry).
    TP_PARAM_MOTOR  = 0x2u   ///< Motor profile record only (identity and electrical parameters).
};

/** @brief 0x2B Param Status (R): state of the non-blocking persistence engine. */
struct TpParamStatus {
    uint8_t  cmd;          ///< Byte 0: TP_OP_PARAM_STATUS.
    uint8_t  state;        ///< Byte 1: bit0 busy, bit1 config loaded, bit2 motor loaded, bit3 last write failed.
    uint8_t  config_ver;   ///< Byte 2: version of the stored configuration record.
    uint8_t  motor_cal_id; ///< Byte 3: calibration identifier of the stored motor profile.
    uint16_t config_free;  ///< Bytes 4-5: free bytes left in the configuration partition.
    uint16_t user_free;    ///< Bytes 6-7: free bytes left in the motor-profile partition.
};
static_assert(sizeof(TpParamStatus) == 8, "TpParamStatus must fill one frame");
