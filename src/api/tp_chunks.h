#pragma once

/**
 * @file tp_chunks.h
 * @brief TunePulse CAN Protocol (TPCP) v1 - the 8-byte frame payload layouts.
 *
 * This header is the aggregate of the frame layouts plus the union that ties
 * them together. The layouts themselves live in chunks/, split by opcode
 * family so each file matches one section of CAN_PROTOCOL.md:
 *
 * | Header                       | Opcodes            | Contents               |
 * |:-----------------------------|:-------------------|:-----------------------|
 * | `chunks/tp_chunk_status.h`   | 0x01 - 0x0F        | Status and telemetry   |
 * | `chunks/tp_chunk_motion.h`   | 0x10 - 0x1F        | Control and motion     |
 * | `chunks/tp_chunk_config.h`   | 0x20 - 0x2F        | Registers, persistence |
 * | `chunks/tp_chunk_system.h`   | 0x30 - 0x3F, 0x7E+ | Calib, system, replies |
 *
 * Include THIS file, not the parts: union TpFrame below names every layout, so
 * it can only be declared once all four families are visible. A host tool needs
 * exactly this file and tp_wire.h to be bit-compatible with the drive.
 *
 * Every protocol frame is exactly 8 bytes and is described here by a typed
 * view. Serialization is zero-copy: TpFrame is a union of a raw byte array, a
 * 64-bit word and one typed view per opcode, so filling a view IS the packing
 * and reading a view IS the unpacking. No memcpy, no serialization loops.
 *
 * Layout rules (identical for every view):
 * - Byte 0 is the opcode.
 * - Multi-byte integers are little-endian and, wherever the layout allows it,
 *   naturally aligned so the compiler emits a single load/store.
 * - Fields that cannot be aligned are declared as explicit byte arrays and go
 *   through the codecs in tp_wire.h.
 * - Sub-byte fields are C bitfields allocated from the LSB upward.
 *
 * The ONE exception to the little-endian rule is the bit-packed motion burst
 * frame (see TpMotionBurst): it is a 64-bit bit field, not a struct of
 * integers, and is laid out MSB-first so that its two format flags land in
 * byte 0 where they can be told apart from an opcode.
 *
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#include "chunks/tp_chunk_status.h"  // 0x01-0x0F status and telemetry layouts
#include "chunks/tp_chunk_motion.h"  // 0x10-0x1F control, targets and trajectory
#include "chunks/tp_chunk_config.h"  // 0x20-0x2F register access and persistence
#include "chunks/tp_chunk_system.h"  // 0x30-0x3F calibration/system + generic replies

// ============================================================================
// Universal frame container
// ============================================================================

/**
 * @brief The 8-byte protocol frame with one typed view per opcode.
 *
 * The transport moves b8[]; the handlers read and write the typed views. All
 * views alias the same 64 bits, so no conversion step exists at all.
 */
union TpFrame {
    uint8_t  b8[8];  ///< Raw byte access, used by the transport layer.
    uint64_t u64;    ///< Whole-frame word: one instruction to clear or compare a frame.

    TpDeviceInfo        device_info;        ///< View: 0x01.
    TpDeviceUid         device_uid;         ///< View: 0x02.
    TpDeviceStatus      device_status;      ///< View: 0x03.
    TpMotorStatus       motor_status;       ///< View: 0x04.
    TpElectricStatus    electric_status;    ///< View: 0x05.
    TpMotionStatus1     motion_status1;     ///< View: 0x06.
    TpMotionStatus2     motion_status2;     ///< View: 0x07.
    TpMotionStatus3     motion_status3;     ///< View: 0x08.
    TpMotorProperties   motor_props;        ///< View: 0x09.
    TpEncoderStatus     encoder_status;     ///< View: 0x0A.
    TpDriverStatus      driver_status;      ///< View: 0x0B.
    TpCalibrationStatus calib_status;       ///< View: 0x0C.
    TpCommStatus        comm_status;        ///< View: 0x0D.
    TpPowerStatus       power_status;       ///< View: 0x0E.
    TpHeartbeat         heartbeat;          ///< View: 0x0F (telemetry and emergency channels).

    TpControlModeCmd    control_mode;       ///< View: 0x10.
    TpTargetPosition    target_position;    ///< View: 0x11.
    TpTargetVelocity    target_velocity;    ///< View: 0x12.
    TpTargetTorque      target_torque;      ///< View: 0x13.
    TpMotionLimits      motion_limits;      ///< View: 0x14.
    TpStopCmd           stop;               ///< View: 0x15.
    TpAnchorCmd         anchor;             ///< View: 0x16.
    TpMotionBurst       burst;              ///< View: bit-packed PVT / PVcTF segment.
    TpMotionScurve      scurve;             ///< View: 0x1A.
    TpMotionQueueCtl    queue_ctl;          ///< View: 0x1B.
    TpMotionQueueStat   queue_stat;         ///< View: 0x1C.

    TpRegAccess         reg;                ///< View: 0x20 / 0x21 / 0x22.
    TpParamCmd          param;              ///< View: 0x28 / 0x29 / 0x2A.
    TpParamStatus       param_status;       ///< View: 0x2B.

    TpCalibCmd          calib;              ///< View: 0x30 / 0x31.
    TpCalibResult       calib_result;       ///< View: 0x32.
    TpSysCmd            sys;                ///< View: 0x38 / 0x39.
    TpRebootCmd         reboot;             ///< View: 0x3A.
    TpNodeIdCmd         node_id;            ///< View: 0x3B.
    TpTelemetryCmd      telemetry;          ///< View: 0x3C.
    TpWatchdogCmd       watchdog;           ///< View: 0x3D.

    TpAck               ack;                ///< View: 0x7E / 0x7F.
};
static_assert(sizeof(TpFrame) == 8, "TpFrame must be exactly one classic CAN payload");
