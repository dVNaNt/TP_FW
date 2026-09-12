#pragma once

/**
 * @file tp_opcodes.h
 * @brief TPCP v1 - byte 0 of every frame: the opcode space and its guard keys.
 *
 * The opcode is allocated in blocks of sixteen so that a range test alone
 * classifies a frame: 0x00-0x0F status reads (which double as cyclic-telemetry
 * chunk ids), 0x10-0x1F real-time control and motion, 0x20-0x2F registers and
 * persistence, 0x30-0x3F calibration and system, 0x40-0x6F reserved for bulk
 * transfer, 0x7E/0x7F the generic replies.
 *
 * Bit 7 is deliberately left clear across the whole space: that is what lets a
 * bit-packed motion burst frame (which sets it) be told apart from an opcode
 * without a mode flag. See TpMotionBurst in chunks/tp_chunk_motion.h.
 *
 * The two guard keys live here rather than with the commands they protect
 * because they are part of the opcode's contract - a reboot or a defaults-wipe
 * frame without its key is not a valid frame.
 *
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#include <stdint.h>  // Fixed-width integer types

// ============================================================================
// Opcode space (byte 0 of every frame)
// ============================================================================

/**
 * @brief Protocol opcodes.
 *
 * Groups: 0x00-0x0F status reads (also valid as cyclic-telemetry chunk ids),
 * 0x10-0x1F real-time control and motion, 0x20-0x2F parameter registers and
 * persistence, 0x30-0x3F calibration and system control, 0x40-0x6F reserved
 * for bulk transfer, 0x7E/0x7F generic acknowledge / reject.
 *
 * A read is requested by sending the opcode with a zero payload; the drive
 * answers with the same opcode and the populated payload. Opcodes marked (W)
 * carry their arguments in the request payload and are answered with ACK (or
 * with their own status opcode where noted).
 */
enum TpOpcode : uint8_t {
    // ---- 0x00-0x0F: status reads / telemetry chunks -----------------------
    TP_OP_NOP                = 0x00u,  ///< No operation; a zero-filled frame is always ignored.
    TP_OP_DEVICE_INFO        = 0x01u,  ///< (R) hardware version, firmware version, device code.
    TP_OP_DEVICE_UID         = 0x02u,  ///< (R) 56-bit device UID.
    TP_OP_DEVICE_STATUS      = 0x03u,  ///< (R) uptime seconds + 32-bit fault/warning word.
    TP_OP_MOTOR_STATUS       = 0x04u,  ///< (R) control mode, motor type, phase order, poles, temperature.
    TP_OP_ELECTRIC_STATUS    = 0x05u,  ///< (R) efficiency, power, supply voltage, phase current.
    TP_OP_MOTION_STATUS_1    = 0x06u,  ///< (R) motion mode + 48-bit absolute position.
    TP_OP_MOTION_STATUS_2    = 0x07u,  ///< (R) velocity, acceleration, torque.
    TP_OP_MOTION_STATUS_3    = 0x08u,  ///< (R) position error, velocity error, queue depth, time scale.
    TP_OP_MOTOR_PROPERTIES   = 0x09u,  ///< (RW) temperature limit, resistance, inductance, torque constant.
    TP_OP_ENCODER_STATUS     = 0x0Au,  ///< (R) raw encoder angle, revolution count, encoder health.
    TP_OP_DRIVER_STATUS      = 0x0Bu,  ///< (R) driver lifecycle state, fault byte, warning byte, control mode.
    TP_OP_CALIBRATION_STATUS = 0x0Cu,  ///< (R) calibration stage, progress and result flags.
    TP_OP_COMM_STATUS        = 0x0Du,  ///< (R) CAN and protocol counters (tx, rx, dropped, errors).
    TP_OP_POWER_STATUS       = 0x0Eu,  ///< (R) bus current, mechanical power estimate, thermal headroom.
    TP_OP_HEARTBEAT          = 0x0Fu,  ///< (R) compact liveness chunk; emitted cyclically on its own.

    // ---- 0x10-0x1F: real-time control and motion --------------------------
    TP_OP_CONTROL_MODE       = 0x10u,  ///< (W) select control mode and enable/disable the output stage.
    TP_OP_TARGET_POSITION    = 0x11u,  ///< (W) absolute or relative position target, Q1.31.16.
    TP_OP_TARGET_VELOCITY    = 0x12u,  ///< (W) velocity target plus acceleration limit.
    TP_OP_TARGET_TORQUE      = 0x13u,  ///< (W) q-axis current target plus current limit.
    TP_OP_MOTION_LIMITS      = 0x14u,  ///< (W) absolute velocity / acceleration / jerk envelope.
    TP_OP_STOP               = 0x15u,  ///< (W) controlled stop; byte 1 selects the stop style.
    TP_OP_ANCHOR             = 0x16u,  ///< (W) redefine the current position without moving.
    // A PVT / PVcTF segment is never sent as an opcode frame: it needs all 64
    // bits (see TpMotionBurst) and travels as the bit-packed burst frame
    // instead. These two identifiers exist so the acknowledgement can name
    // which kind of segment was queued; sending them as a request opcode is
    // rejected with TP_ERR_UNKNOWN_OPCODE.
    TP_OP_MOTION_PVT         = 0x18u,  ///< (ACK label) a PVT segment was queued.
    TP_OP_MOTION_PVCTF       = 0x19u,  ///< (ACK label) a PVcTF segment was queued.
    TP_OP_MOTION_SCURVE      = 0x1Au,  ///< (W) queue a jerk-limited point-to-point move.
    TP_OP_MOTION_QUEUE_CTL   = 0x1Bu,  ///< (W) clear / pause / resume the queue, set the time scale.
    TP_OP_MOTION_QUEUE_STAT  = 0x1Cu,  ///< (R) queue depth, executor state, time scale.

    // ---- 0x20-0x2F: parameter registers and persistence -------------------
    TP_OP_REG_READ           = 0x20u,  ///< (R) read one register by 16-bit identifier.
    TP_OP_REG_WRITE          = 0x21u,  ///< (W) write one register; answered with the value actually applied.
    TP_OP_REG_INFO           = 0x22u,  ///< (R) register introspection: type, access flags, group.
    TP_OP_PARAM_SAVE         = 0x28u,  ///< (W) persist the configuration into NVM.
    TP_OP_PARAM_LOAD         = 0x29u,  ///< (W) reload the configuration from NVM.
    TP_OP_PARAM_DEFAULTS     = 0x2Au,  ///< (W) restore compiled-in defaults (does not persist by itself).
    TP_OP_PARAM_STATUS       = 0x2Bu,  ///< (R) state of the persistence engine.

    // ---- 0x30-0x3F: calibration and system control ------------------------
    TP_OP_CALIB_START        = 0x30u,  ///< (W) start a calibration routine (see TpCalibKind).
    TP_OP_CALIB_ABORT        = 0x31u,  ///< (W) abort the running calibration.
    TP_OP_CALIB_RESULT       = 0x32u,  ///< (R) identified resistance / inductance / offset.
    TP_OP_SYS_ENABLE         = 0x38u,  ///< (W) enable or disable the output stage.
    TP_OP_SYS_FAULT_RESET    = 0x39u,  ///< (W) clear latched faults.
    TP_OP_SYS_REBOOT         = 0x3Au,  ///< (W) reboot, optionally into the USB DFU bootloader.
    TP_OP_SYS_NODE_ID        = 0x3Bu,  ///< (RW) read or change this node's address.
    TP_OP_SYS_TELEMETRY      = 0x3Cu,  ///< (RW) configure one cyclic telemetry slot.
    TP_OP_SYS_WATCHDOG       = 0x3Du,  ///< (RW) configure the master-liveness watchdog.

    // ---- 0x7E-0x7F: generic replies ---------------------------------------
    TP_OP_ACK                = 0x7Eu,  ///< Generic success reply: echoes the opcode that was executed.
    TP_OP_NACK               = 0x7Fu   ///< Generic reject: echoes the opcode plus a TpError code.
};

/** Key required in bytes 1..4 of TP_OP_SYS_REBOOT to make a reboot take effect. */
#define TP_REBOOT_KEY 0x544F4F42u  // ASCII "BOOT" little-endian: guards against a stray frame

/** Key required in bytes 1..4 of TP_OP_PARAM_DEFAULTS to wipe the stored configuration. */
#define TP_DEFAULTS_KEY 0x54455344u  // ASCII "DSET" little-endian
