#pragma once

/**
 * @file tp_chunk_system.h
 * @brief TPCP v1 - calibration, system control and generic reply layouts (0x30 - 0x3F, 0x7E - 0x7F).
 *
 * Calibration commands and their result, plus the system group: enable, fault
 * reset, reboot, node address, telemetry slot configuration and the master
 * watchdog.
 * 
 * The two generic replies (TpAck / TpNack) close the file. They are not a
 * 0x30-range concern, but they are the universal answer to every command frame
 * in this group and in every other, so they sit with the commands rather than
 * in a file of their own.
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
// Calibration and system control (opcodes 0x30 - 0x3F)
// ============================================================================

/** @brief 0x30 / 0x31 Calibration command: start or abort an identification run. */
struct TpCalibCmd {
    uint8_t  cmd;       ///< Byte 0: TP_OP_CALIB_START / TP_OP_CALIB_ABORT.
    uint8_t  kind;      ///< Byte 1: TpCalibKind. CURRENT_OFFSET, ELECTRICAL and ANGLE are
                        ///< implemented; FULL is rejected with TP_ERR_NOT_IMPLEMENTED because
                        ///< the two engines have to be sequenced by the master (start
                        ///< ELECTRICAL, poll 0x0C until it succeeds, then start ANGLE).
    uint8_t  flags;     ///< Byte 2: bit0 persist the result when the run succeeds.
    uint8_t  reserved;  ///< Byte 3: reserved, write as zero.
    uint32_t pad;       ///< Bytes 4-7: reserved, write as zero.
};
static_assert(sizeof(TpCalibCmd) == 8, "TpCalibCmd must fill one frame");

/** @brief 0x32 Calibration Result (R): the identified electrical parameters. */
struct TpCalibResult {
    uint8_t  cmd;         ///< Byte 0: TP_OP_CALIB_RESULT.
    uint8_t  flags;       ///< Byte 1: bit0 electrical valid, bit1 angle table valid, bit2 last run failed.
    uint16_t resistance;  ///< Bytes 2-3: identified resistance [Ohm], bfloat16.
    uint16_t induct_d;    ///< Bytes 4-5: identified d-axis inductance [H], bfloat16.
    uint16_t induct_q;    ///< Bytes 6-7: identified q-axis inductance [H], bfloat16.
};
static_assert(sizeof(TpCalibResult) == 8, "TpCalibResult must fill one frame");

/** @brief 0x38 / 0x39 System command: enable/disable the stage, clear faults. */
struct TpSysCmd {
    uint8_t  cmd;       ///< Byte 0: TP_OP_SYS_ENABLE / TP_OP_SYS_FAULT_RESET.
    uint8_t  arg;       ///< Byte 1: enable flag, or the fault mask to clear (0 = all).
    uint16_t reserved;  ///< Bytes 2-3: reserved, write as zero.
    uint32_t pad;       ///< Bytes 4-7: reserved, write as zero.
};
static_assert(sizeof(TpSysCmd) == 8, "TpSysCmd must fill one frame");

/** @brief 0x3A System Reboot (W): restart, optionally into the USB DFU bootloader. */
struct TpRebootCmd {
    uint8_t  cmd;       ///< Byte 0: TP_OP_SYS_REBOOT.
    uint8_t  mode;      ///< Byte 1: 0 = warm reset, 1 = jump to the USB DFU bootloader.
    uint16_t reserved;  ///< Bytes 2-3: reserved, write as zero.
    uint32_t key;       ///< Bytes 4-7: must equal TP_REBOOT_KEY or the frame is rejected.
};
static_assert(sizeof(TpRebootCmd) == 8, "TpRebootCmd must fill one frame");

/** @brief 0x3B Node Address (RW): read the address, or change it under a UID guard. */
struct TpNodeIdCmd {
    uint8_t  cmd;      ///< Byte 0: TP_OP_SYS_NODE_ID.
    uint8_t  node_id;  ///< Byte 1: requested address on a write, current address in the reply.
                       ///< Zero selects a READ (0 is the unassigned address and is
                       ///< therefore never a legal assignment target anyway).
    uint8_t  flags;    ///< Byte 2: bit0 persist the new address immediately.
    uint8_t  state;    ///< Byte 3: reply only - bit0 address stored in NVM, bit1 discovery active.
    uint32_t guard;    ///< Bytes 4-7: low 32 bits of the device UID; a write without it is rejected.
};
static_assert(sizeof(TpNodeIdCmd) == 8, "TpNodeIdCmd must fill one frame");

/** @brief 0x3C Telemetry Slot (RW): configure one cyclic telemetry producer. */
struct TpTelemetryCmd {
    uint8_t  cmd;      ///< Byte 0: TP_OP_SYS_TELEMETRY.
    uint8_t  slot;     ///< Byte 1: slot index (0 .. TP_TELEMETRY_SLOTS-1).
    uint8_t  opcode;   ///< Byte 2: status opcode to emit (0x01 - 0x0F); 0 frees the slot.
    uint8_t  flags;    ///< Byte 3: bit7 = apply this frame (clear it to read the slot back),
                       ///< bits 0-6 = TpTelemetryFlag. The apply bit is what lets an
                       ///< all-zero body switch a slot off instead of reading it.
    uint16_t period;   ///< Bytes 4-5: emission period in milliseconds (0 = on SYNC only).
    uint16_t reserved; ///< Bytes 6-7: reserved, write as zero.
};
static_assert(sizeof(TpTelemetryCmd) == 8, "TpTelemetryCmd must fill one frame");

/** @brief 0x3D Watchdog (RW): master-liveness timeout and the reaction to it. */
struct TpWatchdogCmd {
    uint8_t  cmd;       ///< Byte 0: TP_OP_SYS_WATCHDOG.
    uint8_t  action;    ///< Byte 1: TpSafeState entered when the timeout expires.
    uint8_t  flags;     ///< Byte 2: bit7 = apply this frame (clear it to read back);
                        ///< bit0 = watchdog enabled; the reply also sets bit1 when it has expired.
    uint8_t  reserved;  ///< Byte 3: reserved, write as zero.
    uint16_t timeout;   ///< Bytes 4-5: timeout in milliseconds (0 disables the watchdog).
    uint16_t pad;       ///< Bytes 6-7: reserved, write as zero.
};
static_assert(sizeof(TpWatchdogCmd) == 8, "TpWatchdogCmd must fill one frame");

// ============================================================================
// Generic replies (opcodes 0x7E - 0x7F)
// ============================================================================

/** @brief 0x7E / 0x7F Generic acknowledge or reject reply. */
struct TpAck {
    uint8_t  cmd;       ///< Byte 0: TP_OP_ACK or TP_OP_NACK.
    uint8_t  opcode;    ///< Byte 1: the opcode this reply refers to.
    uint8_t  error;     ///< Byte 2: TpError (always TP_ERR_NONE in an ACK).
    uint8_t  detail;    ///< Byte 3: opcode-specific context (register id low byte, stage, ...).
    uint32_t reserved;  ///< Bytes 4-7: reserved, transmitted as zero.
};
static_assert(sizeof(TpAck) == 8, "TpAck must fill one frame");
