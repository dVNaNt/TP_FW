#pragma once

/**
 * @file tp_cmds.h
 * @brief TPCP v1 - byte 0 of every frame: the CMD byte, its groups and the guard keys.
 *
 * The CMD byte is a register address with two group bits folded in. Bits 7..5
 * classify the frame without a table lookup:
 *
 * | Bit 7 | Bit 6 | Bit 5 | Range       | Group                          |
 * |:-----:|:-----:|:-----:|:------------|:-------------------------------|
 * |   0   |   0   |   x   | 0x00 - 0x3F | Read of a read/write register  |
 * |   1   |   0   |   x   | 0x80 - 0xBF | Write of a read/write register |
 * |   0   |   1   |   x   | 0x40 - 0x7F | Read-only register             |
 * |   1   |   1   |   0   | 0xC0 - 0xDF | Write-only register            |
 * |   1   |   1   |   1   | 0xE0 - 0xFF | Burst mode (bit-packed stream) |
 *
 * So a read/write register has ONE address (0x00..0x3F): reading it sends that
 * address, writing it sends the address with bit 7 set, and the answer to
 * either is the register's read form carrying the values now in force.
 *
 * Several registers are SUB-SPACES: byte 1 selects one of up to 256 records
 * behind a single CMD (the 0x01 axis group, the 0x02 telemetry slots, the
 * 0x42 currents and the 0x43 diagnostics). The sub identifiers live here too.
 *
 * The guard keys live here rather than with the commands they protect because
 * they are part of the CMD's contract: a save frame without its keyword is not
 * a valid frame.
 *
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#include <stdint.h>  // Fixed-width integer types

// ============================================================================
// Group classification
// ============================================================================

/** Bit 7 of the CMD byte: set on a write of a read/write register. */
#define TP_CMD_WRITE 0x80u

/** Mask of the two group bits of a read/write or read-only CMD. */
#define TP_CMD_GROUP_MASK 0xC0u

/** Mask of the three group bits that separate write-only from burst frames. */
#define TP_CMD_BURST_MASK 0xE0u

/** Group bits of a read/write register read (0x00 - 0x3F). */
#define TP_CMD_GROUP_RW_READ 0x00u

/** Group bits of a read-only register (0x40 - 0x7F). */
#define TP_CMD_GROUP_RO 0x40u

/** Group bits of a read/write register write (0x80 - 0xBF). */
#define TP_CMD_GROUP_RW_WRITE 0x80u

/** Group bits of a write-only register (0xC0 - 0xDF). */
#define TP_CMD_GROUP_WO 0xC0u

/** Group bits of a burst-mode frame (0xE0 - 0xFF). */
#define TP_CMD_GROUP_BURST 0xE0u

/** @brief True when byte 0 is a bit-packed burst frame (bits 7..5 all set). */
static inline bool tp_cmd_is_burst(uint8_t cmd) {
    return (cmd & TP_CMD_BURST_MASK) == TP_CMD_GROUP_BURST;
}

/** @brief True when byte 0 addresses a write-only register (0xC0 - 0xDF). */
static inline bool tp_cmd_is_write_only(uint8_t cmd) {
    return (cmd & TP_CMD_BURST_MASK) == TP_CMD_GROUP_WO;
}

/** @brief True when byte 0 addresses a read-only register (0x40 - 0x7F). */
static inline bool tp_cmd_is_read_only(uint8_t cmd) {
    return (cmd & TP_CMD_GROUP_MASK) == TP_CMD_GROUP_RO;
}

/** @brief True when byte 0 reads a read/write register (0x00 - 0x3F). */
static inline bool tp_cmd_is_rw_read(uint8_t cmd) {
    return (cmd & TP_CMD_GROUP_MASK) == TP_CMD_GROUP_RW_READ;
}

/** @brief True when byte 0 writes a read/write register (0x80 - 0xBF). */
static inline bool tp_cmd_is_rw_write(uint8_t cmd) {
    return (cmd & TP_CMD_GROUP_MASK) == TP_CMD_GROUP_RW_WRITE;
}

/** @brief True for any frame that only reads (RW read or RO). */
static inline bool tp_cmd_is_read(uint8_t cmd) {
    return (cmd & TP_CMD_WRITE) == 0u;  // Bit 7 clear: 0x00 - 0x7F
}

/** @brief The register address of a CMD byte with the write bit stripped (RW group only). */
static inline uint8_t tp_cmd_register(uint8_t cmd) {
    return (uint8_t)(cmd & 0x7Fu);  // Same address for the read and the write form
}

// ============================================================================
// Register addresses
// ============================================================================

/**
 * @brief Register addresses (the read form; writes set TP_CMD_WRITE).
 *
 * The read/write block (0x00 - 0x3F) and the read-only block (0x40 - 0x7F)
 * are register maps; the write-only block (0xC0 - 0xDF) holds commands that
 * have no state to read back. Registers marked (sub) carry a sub identifier
 * in byte 1 - see the TP_SUB_* enumerations below.
 */
enum TpCmd : uint8_t {
    // ---- 0x00 - 0x3F: read/write registers ----------------------------
    TP_REG_DIRECT             = 0x00u,  ///< Direct memory access: 24-bit windowed address + 32-bit value.
    TP_REG_AXIS               = 0x01u,  ///< (sub) Axis, LED, gearbox, IO map, encoders, homing, limits.
    TP_REG_TELEMETRY          = 0x02u,  ///< (sub) Cyclic telemetry slot configuration (sub = slot).
    TP_REG_BUS                = 0x10u,  ///< Bus type, response policy, bit rates, master time sync.
    TP_REG_DISCOVERY_ASSIGN   = 0x11u,  ///< HashUID48 + address: assignment, reset and ping.
    TP_REG_DISCOVERY_SEARCH   = 0x12u,  ///< Binary UID search: depth + prefix.
    TP_REG_SYSTEM_CONTROL     = 0x20u,  ///< Enable, burst step, calibration requests, actions.
    TP_REG_MOTOR_PROPS_1      = 0x21u,  ///< Control mode, motor type, wiring, poles, ratings.
    TP_REG_MOTOR_PROPS_2      = 0x22u,  ///< Thermal rating, resistance, inductance, Kt.
    TP_REG_POWER_LIMITS       = 0x23u,  ///< Overdrive duration, power baseline / peak / capacity.
    TP_REG_MOTOR_THERMAL      = 0x24u,  ///< Motor temperature model and limit.
    TP_REG_DRIVER_THERMAL     = 0x25u,  ///< Driver overtemperature warn / fault / hysteresis.
    TP_REG_SUPPLY_PROTECTION  = 0x26u,  ///< Supply hysteresis, recovery, OV, UV.
    TP_REG_CURRENT_PROTECTION = 0x27u,  ///< Disbalance, recovery, peak current, OC trip.
    TP_REG_MOTION_LIMITS      = 0x2Au,  ///< Velocity / acceleration / jerk envelope.
    TP_REG_ERROR_LIMITS       = 0x2Bu,  ///< Same envelope, applied under fault conditions.
    TP_REG_WATCHDOG           = 0x2Du,  ///< PulseBeat period / delay / retries and the watchdog.
    TP_REG_CAL_TABLE          = 0x2Fu,  ///< Calibration table sample access.

    // ---- 0x40 - 0x7F: read-only registers -----------------------------
    TP_REG_BATCH_READ         = 0x40u,  ///< Up to seven registers (or ranges) in one request.
    TP_REG_RANGE_READ         = 0x41u,  ///< One register, a range of sub identifiers / addresses.
    TP_REG_CURRENTS           = 0x42u,  ///< (sub) Supply current + d/q or alpha/beta currents.
    TP_REG_DIAG               = 0x43u,  ///< (sub) Link, calibration, queue, persistence, encoder.
    TP_REG_DEVICE_INFO        = 0x50u,  ///< Hardware version, firmware version, device code.
    TP_REG_SYSTEM_STATUS      = 0x51u,  ///< IO word, uptime, fault / warning / state word.
    TP_REG_MOTOR_STATUS       = 0x52u,  ///< State, identity bits, motor temperature, power, torque.
    TP_REG_MOTION_STATUS_1    = 0x53u,  ///< Following errors, queue depth, time scale.
    TP_REG_MOTION_STATUS_2    = 0x54u,  ///< Motion mode + 48-bit absolute position.
    TP_REG_MOTION_STATUS_3    = 0x55u,  ///< Velocity, acceleration, torque.
    TP_REG_HOMING_TRAVEL      = 0x56u,  ///< Sensorless homing travel (not implemented: NACK).
    TP_REG_ENCODER_POSITION   = 0x57u,  ///< Raw absolute encoder position, q1.23.32.
    TP_REG_DRIVER_STATUS      = 0x58u,  ///< Faults, warnings, driver temperature, usage, supply.
    TP_CMD_ACK                = 0x5Eu,  ///< Generic success reply to a write-only command.
    TP_CMD_NACK               = 0x5Fu,  ///< Generic reject: echoes the CMD plus a TpError code.
    TP_CMD_PULSEBEAT          = 0x70u,  ///< 0x70 - 0x7F: PulseBeat, low nibble = beat counter.

    // ---- 0xC0 - 0xDF: write-only registers ----------------------------
    TP_CMD_MOTION_TASK_1      = 0xC0u,  ///< Segment header: mode, flags, time.
    TP_CMD_MOTION_TASK_2      = 0xC1u,  ///< Segment position (absolute or displacement).
    TP_CMD_MOTION_TASK_3      = 0xC2u,  ///< Segment torque and velocity.
    TP_CMD_MOTION_TASK_4      = 0xC3u,  ///< Segment acceleration, jerk, IO trigger masks.
    TP_CMD_SAVE_CONFIG        = 0xCFu,  ///< Persist / restore / erase the stored configuration.
    TP_CMD_MIDI               = 0xD0u,  ///< Play a note through the motor coils.

    // ---- 0xE0 - 0xFF: burst mode ---------------------------------------
    TP_CMD_BURST              = 0xE0u   ///< Base of the bit-packed PVT / PVcTF stream.
};

/** Number of beat values the PulseBeat CMD nibble can carry (0x70 .. 0x7F). */
#define TP_PULSEBEAT_MASK 0x0Fu

/** @brief True when byte 0 is a PulseBeat frame (0x70 - 0x7F). */
static inline bool tp_cmd_is_pulsebeat(uint8_t cmd) {
    return (cmd & 0xF0u) == TP_CMD_PULSEBEAT;
}

// ============================================================================
// Sub identifiers
// ============================================================================

/** Sub identifiers of the 0x01 axis / mechanics group. */
enum TpSubAxis : uint8_t {
    TP_SUB_AXIS_CONTROL    = 0x00u,  ///< Axis id, direction inversion, position offset.
    TP_SUB_LED             = 0x01u,  ///< Status LED colour and blink pattern.
    TP_SUB_GEARBOX         = 0x02u,  ///< Backlash and gear ratio.
    TP_SUB_IO_MAP          = 0x03u,  ///< Pin assignment of one device function (byte 2 = function).
    TP_SUB_ENCODER_1       = 0x04u,  ///< Primary encoder type, interface, resolution.
    TP_SUB_ENCODER_2       = 0x05u,  ///< Secondary encoder type, interface, resolution.
    TP_SUB_HOMING_QUICK    = 0x06u,  ///< Coarse homing search settings.
    TP_SUB_HOMING_FINE     = 0x07u,  ///< Fine homing search settings.
    TP_SUB_REF_CW          = 0x08u,  ///< CW reference limit: expected offset + reference angle.
    TP_SUB_REF_CCW         = 0x09u,  ///< CCW reference limit: expected offset + reference angle.
    TP_SUB_POS_ZERO        = 0x0Au,  ///< Reference angle of the work-field zero.
    TP_SUB_ZERO_OFFSET_CW  = 0x0Bu,  ///< Distance from zero to the CW endstop, q1.15.32.
    TP_SUB_ZERO_OFFSET_CCW = 0x0Cu,  ///< Distance from zero to the CCW endstop, q1.15.32.
    TP_SUB_FIELD_CW        = 0x0Du,  ///< Allowed travel from zero in the CW direction, q1.15.32.
    TP_SUB_FIELD_CCW       = 0x0Eu,  ///< Allowed travel from zero in the CCW direction, q1.15.32.
    TP_SUB_INTERNAL_OFFSET = 0x0Fu,  ///< System zero to encoder internal position, q1.15.32.
    TP_SUB_AXIS_COUNT      = 0x10u   ///< Number of records in the group.
};

/** Sub identifiers of the 0x42 currents group. */
enum TpSubCurrents : uint8_t {
    TP_SUB_CURRENTS_DQ    = 0x00u,  ///< Supply current, q-axis current, d-axis current.
    TP_SUB_CURRENTS_AB    = 0x01u,  ///< Supply current, alpha (coil A) current, beta (coil B) current.
    TP_SUB_CURRENTS_COUNT = 0x02u   ///< Number of records in the group.
};

/** Sub identifiers of the 0x43 diagnostics group. */
enum TpSubDiag : uint8_t {
    TP_SUB_DIAG_COMM         = 0x00u,  ///< Bus state, error counters, drops, rejects.
    TP_SUB_DIAG_CALIBRATION  = 0x01u,  ///< Calibration routine, stage, progress, flags.
    TP_SUB_DIAG_QUEUE        = 0x02u,  ///< Executor state, queued / free slots, time scale, drops.
    TP_SUB_DIAG_PERSIST      = 0x03u,  ///< Persistence engine state and partition free space.
    TP_SUB_DIAG_ENCODER      = 0x04u,  ///< Raw single-turn angle and revolution count.
    TP_SUB_DIAG_CALIB_RESULT = 0x05u,  ///< Identified resistance and d/q inductances.
    TP_SUB_DIAG_FOC          = 0x06u,  ///< Electrical angle in use and the d/q voltage command.
    TP_SUB_DIAG_ENCODER_LINK = 0x07u,  ///< Encoder frame integrity: check state, failure counters.
    TP_SUB_DIAG_IDENT_FAIL   = 0x08u,  ///< Why the last electrical identification aborted.
    TP_SUB_DIAG_COUNT        = 0x09u   ///< Number of records in the group.
};

/** Number of independently configurable cyclic telemetry slots (0x02 sub = slot). */
#define TP_TELEMETRY_SLOTS 4u

// ============================================================================
// Guard keys and magic values
// ============================================================================

/** Keyword of a 0xCF frame that saves the selected catalog to flash. */
#define TP_SAVE_KEY 0x000010ADu

/** Keyword of a 0xCF frame that reloads the compiled-in defaults into the live shadow. */
#define TP_DEFAULTS_KEY 0x0000DEF0u

/** Keyword of a 0xCF frame that erases the stored record (factory reset, address included). */
#define TP_ERASE_KEY 0x0000E7A5u

/** Catalog value of a 0xCF frame that selects every store. */
#define TP_CATALOG_ALL 0xFFFFFFu

/** Catalog bit selecting the node configuration record. */
#define TP_CATALOG_CONFIG 0x000001u

/** Catalog bit selecting the motor profile record. */
#define TP_CATALOG_MOTOR 0x000002u

/** Data word of a 0x2F read that asks for the table size instead of a sample. */
#define TP_CAL_TABLE_SIZE_KEY 0xCA1B513Eu

/** System Control enable byte: energize the output stage. */
#define TP_SYS_ENABLE 0xE1u

/** System Control enable byte: release the output stage. */
#define TP_SYS_DISABLE 0x0Fu

/** System Control action byte: clear every latched fault. */
#define TP_SYS_FAULT_RESET 0xC1u

/** System Control action byte: warm reset once the reply has left. */
#define TP_SYS_REBOOT 0xB0u

/** System Control action byte: reboot into the USB DFU bootloader. */
#define TP_SYS_REBOOT_DFU 0xDFu

/** System Control calibration bits (request on write, validity on read). */
#define TP_CAL_P 0x01u  ///< Encoder (angle table) calibration.
#define TP_CAL_I 0x02u  ///< Current-sense offset calibration.
#define TP_CAL_E 0x04u  ///< Phase connection and motor check.
#define TP_CAL_R 0x08u  ///< Resistance identification.
#define TP_CAL_L 0x10u  ///< Inductance identification.
#define TP_CAL_F 0x20u  ///< Flux linkage identification (not implemented).
#define TP_CAL_M 0x40u  ///< Mechanical calibration (not implemented).

/** The three electrical identification requests, which one routine serves. */
#define TP_CAL_ELECTRICAL (TP_CAL_E | TP_CAL_R | TP_CAL_L)
