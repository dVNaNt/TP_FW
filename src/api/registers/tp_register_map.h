#pragma once

/**
 * @file tp_register_map.h
 * @brief TPCP v1 - the register map: which CMDs exist, their access rules and sub-space sizes.
 *
 * The map is the authority a batch or range read consults before touching a
 * register, and the rule set the dispatcher applies before a write: whether
 * the register is writable, whether it is a sub-space and how many records
 * it holds, whether writing it requires the output stage to be off, and
 * whether it lands in the persisted record.
 *
 * The packing and unpacking of each register's payload lives in the engine
 * (engine/tp_engine_regread_impl.h and tp_engine_regwrite_impl.h): those
 * functions need the whole drive, the store and the engine's own state, and
 * a table of function pointers would only have moved the same switch behind
 * an indirection.
 *
 * The table MUST stay sorted by CMD: tp_find_register is a binary search.
 *
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#include "../tp_wire.h"  // CMD map and the sub identifier counts

// ============================================================================
// Register descriptor
// ============================================================================

/** Access and behaviour flags of one register. */
enum TpRegFlag : uint8_t {
    TP_RF_READ     = 1u << 0,  ///< Readable.
    TP_RF_WRITE    = 1u << 1,  ///< Writable (RW registers answer a write with their read form).
    TP_RF_SUB      = 1u << 2,  ///< Byte 1 is a sub identifier; `subs` records exist.
    TP_RF_PERSIST  = 1u << 3,  ///< Fields land in the persisted configuration record.
    TP_RF_DISABLED = 1u << 4,  ///< Writable only while the output stage is disabled.
    TP_RF_KEYED    = 1u << 5   ///< Byte 2 selects the record (IO map: the device function).
};

/** Shorthand for the common read/write combination. */
#define TP_RF_RW (TP_RF_READ | TP_RF_WRITE)

/** @brief One register: address, access flags and sub-space size. */
struct TpRegDesc {
    uint8_t cmd;    ///< Read-form CMD (0x00 - 0x7F).
    uint8_t flags;  ///< TpRegFlag bits.
    uint8_t subs;   ///< Records in the sub-space (1 for a plain register).
};

/**
 * @brief The register map, sorted by CMD so lookup is a binary search.
 *
 * Batch reads (0x40 / 0x41) are commands rather than registers and are not
 * listed: they cannot be nested.
 */
static const TpRegDesc TP_REGISTERS[] = {
    {TP_REG_DIRECT,             TP_RF_RW,                                        1u},
    {TP_REG_AXIS,               TP_RF_RW | TP_RF_SUB | TP_RF_PERSIST,            TP_SUB_AXIS_COUNT},
    {TP_REG_TELEMETRY,          TP_RF_RW | TP_RF_SUB | TP_RF_PERSIST,            TP_TELEMETRY_SLOTS},
    {TP_REG_BUS,                TP_RF_RW | TP_RF_PERSIST,                        1u},
    {TP_REG_DISCOVERY_ASSIGN,   TP_RF_RW,                                        1u},
    {TP_REG_DISCOVERY_SEARCH,   TP_RF_RW,                                        1u},
    {TP_REG_SYSTEM_CONTROL,     TP_RF_RW | TP_RF_PERSIST,                        1u},
    {TP_REG_MOTOR_PROPS_1,      TP_RF_RW | TP_RF_PERSIST,                        1u},
    {TP_REG_MOTOR_PROPS_2,      TP_RF_RW | TP_RF_DISABLED,                       1u},
    {TP_REG_POWER_LIMITS,       TP_RF_RW | TP_RF_PERSIST,                        1u},
    {TP_REG_MOTOR_THERMAL,      TP_RF_RW | TP_RF_PERSIST,                        1u},
    {TP_REG_DRIVER_THERMAL,     TP_RF_RW | TP_RF_PERSIST,                        1u},
    {TP_REG_SUPPLY_PROTECTION,  TP_RF_RW | TP_RF_PERSIST | TP_RF_DISABLED,       1u},
    {TP_REG_CURRENT_PROTECTION, TP_RF_RW | TP_RF_PERSIST | TP_RF_DISABLED,       1u},
    {TP_REG_MOTION_LIMITS,      TP_RF_RW | TP_RF_PERSIST,                        1u},
    {TP_REG_ERROR_LIMITS,       TP_RF_RW | TP_RF_PERSIST,                        1u},
    {TP_REG_WATCHDOG,           TP_RF_RW | TP_RF_PERSIST,                        1u},
    {TP_REG_CAL_TABLE,          TP_RF_READ,                                      1u},
    {TP_REG_CURRENTS,           TP_RF_READ | TP_RF_SUB,                          TP_SUB_CURRENTS_COUNT},
    {TP_REG_DIAG,               TP_RF_READ | TP_RF_SUB,                          TP_SUB_DIAG_COUNT},
    {TP_REG_DEVICE_INFO,        TP_RF_READ,                                      1u},
    {TP_REG_SYSTEM_STATUS,      TP_RF_READ,                                      1u},
    {TP_REG_MOTOR_STATUS,       TP_RF_READ,                                      1u},
    {TP_REG_MOTION_STATUS_1,    TP_RF_READ,                                      1u},
    {TP_REG_MOTION_STATUS_2,    TP_RF_READ,                                      1u},
    {TP_REG_MOTION_STATUS_3,    TP_RF_READ,                                      1u},
    {TP_REG_HOMING_TRAVEL,      TP_RF_READ,                                      1u},
    {TP_REG_ENCODER_POSITION,   TP_RF_READ,                                      1u},
    {TP_REG_DRIVER_STATUS,      TP_RF_READ,                                      1u},
    {TP_CMD_PULSEBEAT,          TP_RF_READ,                                      1u}
};

/** Number of registers in the map. */
static constexpr uint32_t TP_REGISTER_COUNT = sizeof(TP_REGISTERS) / sizeof(TP_REGISTERS[0]);

/**
 * @brief Find a register descriptor by read-form CMD.
 *
 * Binary search over the sorted table: at most five comparisons for the
 * whole map. A PulseBeat CMD (0x70 - 0x7F) is folded onto its base entry.
 *
 * @param cmd Read-form CMD (the write bit is stripped first).
 * @return Descriptor pointer, or nullptr when the CMD is unknown.
 */
static inline const TpRegDesc* tp_find_register(uint8_t cmd) {
    uint8_t key = tp_cmd_register(cmd);       // Strip the write bit
    if (tp_cmd_is_pulsebeat(key)) {
        key = TP_CMD_PULSEBEAT;               // Any beat counter names the same register
    }
    uint32_t lo = 0;                          // Inclusive lower bound
    uint32_t hi = TP_REGISTER_COUNT;          // Exclusive upper bound
    while (lo < hi) {
        const uint32_t mid = (lo + hi) / 2u;  // Midpoint of the remaining window
        if (TP_REGISTERS[mid].cmd == key) {
            return &TP_REGISTERS[mid];        // Exact hit
        }
        if (TP_REGISTERS[mid].cmd < key) {
            lo = mid + 1u;                    // Search the upper half
        } else {
            hi = mid;                         // Search the lower half
        }
    }
    return nullptr;                           // Not in the map
}
