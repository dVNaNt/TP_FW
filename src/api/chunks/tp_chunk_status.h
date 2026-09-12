#pragma once

/**
 * @file tp_chunk_status.h
 * @brief TPCP v1 - status and telemetry frame layouts (opcodes 0x01 - 0x0F).
 *
 * The read-only half of the protocol: everything a master polls or subscribes
 * to. Each of these doubles as a cyclic-telemetry chunk, which is why the
 * opcode and the chunk id are the same number - a telemetry slot is configured
 * by naming the opcode it should emit.
 * 
 * TpHeartbeat closes the group and is the one layout sent unsolicited on two
 * different channels: as the periodic heartbeat, and verbatim as the emergency
 * frame when a fault edge is detected.
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
// Status and telemetry chunks (opcodes 0x01 - 0x0F)
// ============================================================================

/** @brief 0x01 Device Info (R): hardware revision, firmware revision, device code. */
struct TpDeviceInfo {
    uint8_t  cmd;      ///< Byte 0: TP_OP_DEVICE_INFO.
    uint8_t  hw_ver;   ///< Byte 1: hardware revision tag.
    uint16_t fw_ver;   ///< Bytes 2-3: firmware version (major << 8 | minor).
    uint32_t device;   ///< Bytes 4-7: device / board type code.
};
static_assert(sizeof(TpDeviceInfo) == 8, "TpDeviceInfo must fill one frame");

/** @brief 0x02 Device UID (R): the 56-bit unique identifier used by discovery. */
struct TpDeviceUid {
    uint8_t cmd;                 ///< Byte 0: TP_OP_DEVICE_UID.
    uint8_t uid[TP_UID_BYTES];   ///< Bytes 1-7: UID, byte 0 first (search order).
};
static_assert(sizeof(TpDeviceUid) == 8, "TpDeviceUid must fill one frame");

/**
 * @brief 0x03 Device Status (R): uptime plus the aggregated status word.
 *
 * The status word packs everything a supervisor needs in a single 32-bit read:
 * bits 0-7 fault flags (DriverPWM::Fault, 1:1), bits 8-15 warning flags
 * (TpWarn), bits 16-19 drive state (TpDriveState), bits 20-23 control mode
 * (TpControlMode), bits 24-31 reserved.
 */
struct TpDeviceStatus {
    uint8_t  cmd;        ///< Byte 0: TP_OP_DEVICE_STATUS.
    uint8_t  uptime[3];  ///< Bytes 1-3: uptime in seconds, uint24 LE (194 days).
    uint32_t status;     ///< Bytes 4-7: fault | warn << 8 | state << 16 | mode << 20.
};
static_assert(sizeof(TpDeviceStatus) == 8, "TpDeviceStatus must fill one frame");

/** Bit positions inside the TpDeviceStatus / register status word. */
#define TP_STATUS_FAULT_SHIFT 0u   ///< Bits 0-7: DriverPWM fault flags.
#define TP_STATUS_WARN_SHIFT  8u   ///< Bits 8-15: TpWarn flags.
#define TP_STATUS_STATE_SHIFT 16u  ///< Bits 16-19: TpDriveState.
#define TP_STATUS_MODE_SHIFT  20u  ///< Bits 20-23: TpControlMode.

/**
 * @brief 0x04 Motor Status (R): identity and live state of the connected machine.
 *
 * Read only in v1. The draft listed this chunk as writable, but every field it
 * carries is reachable through the register map (0x0100 motor type, 0x0101 pole
 * pairs, 0x0102 phase order, 0x0103 direction) where each one gets its own
 * range check and the "output stage must be disabled" guard. A second,
 * weaker write path to the same fields would only be a way to get them wrong.
 */
struct TpMotorStatus {
    uint8_t cmd;  ///< Byte 0: TP_OP_MOTOR_STATUS.
    union {
        struct {
            uint8_t mode : 4;        ///< Bits 0-3: active TpControlMode.
            uint8_t motor_type : 2;  ///< Bits 4-5: 0 none, 1 stepper, 2 BLDC, 3 error.
            uint8_t phase_conn : 2;  ///< Bits 6-7: 0 none, 1 ABC, 2 CAB, 3 ACB.
        };
        uint8_t flags1;              ///< Whole-byte view of byte 1.
    };
    uint8_t pole_pairs;  ///< Byte 2: pole pairs (1-255).
    uint8_t temp;        ///< Byte 3: winding/driver temperature in degC, 255 = no data.
    union {
        struct {
            uint32_t direction : 1;    ///< Bit 0: 0 = CW, 1 = CCW.
            uint32_t enabled : 1;      ///< Bit 1: output stage energized.
            uint32_t calibrated : 1;   ///< Bit 2: angle table valid.
            uint32_t identified : 1;   ///< Bit 3: electrical parameters identified.
            uint32_t profile_loaded : 1;  ///< Bit 4: profile restored from NVM at boot.
            uint32_t reserved : 27;    ///< Bits 5-31: reserved, transmitted as zero.
        };
        uint32_t flags4;               ///< Whole-word view of bytes 4-7.
    };
};
static_assert(sizeof(TpMotorStatus) == 8, "TpMotorStatus must fill one frame");

/** @brief 0x05 Electric Status (R): conversion efficiency and the power triple. */
struct TpElectricStatus {
    uint8_t  cmd;         ///< Byte 0: TP_OP_ELECTRIC_STATUS.
    uint8_t  efficiency;  ///< Byte 1: 0-255 mapped to 0-100 %, 255 = not available.
    uint16_t power;       ///< Bytes 2-3: instantaneous motor power [W], bfloat16.
    uint16_t supply;      ///< Bytes 4-5: DC bus voltage [V], bfloat16.
    uint16_t current;     ///< Bytes 6-7: q-axis current magnitude [A], bfloat16.
};
static_assert(sizeof(TpElectricStatus) == 8, "TpElectricStatus must fill one frame");

/** @brief 0x06 Motion Status 1 (R): executing mode plus the absolute position. */
struct TpMotionStatus1 {
    uint8_t cmd;          ///< Byte 0: TP_OP_MOTION_STATUS_1.
    uint8_t motion_mode;  ///< Byte 1: segment mode currently executing (MotionMode).
    uint8_t position[6];  ///< Bytes 2-7: measured position, Q1.31.16 LE (revolutions).
};
static_assert(sizeof(TpMotionStatus1) == 8, "TpMotionStatus1 must fill one frame");

/** @brief 0x07 Motion Status 2 (R): observed velocity, acceleration and torque. */
struct TpMotionStatus2 {
    uint8_t  cmd;          ///< Byte 0: TP_OP_MOTION_STATUS_2.
    uint8_t  velocity[3];  ///< Bytes 1-3: observed velocity [RPS], float24.
    uint16_t accel;        ///< Bytes 4-5: observed acceleration [RPS^2], bfloat16.
    uint16_t torque;       ///< Bytes 6-7: applied q-axis current [A], bfloat16.
};
static_assert(sizeof(TpMotionStatus2) == 8, "TpMotionStatus2 must fill one frame");

/**
 * @brief 0x08 Motion Status 3 (R): following errors, queue depth and time scale.
 *
 * Bytes 6-7 pack the queue depth (bits 0-3) and the active time scale as a
 * 12-bit Q1.3.8 value (bits 4-15, 256 = 1.0x).
 */
struct TpMotionStatus3 {
    uint8_t  cmd;         ///< Byte 0: TP_OP_MOTION_STATUS_3.
    uint8_t  pos_err[3];  ///< Bytes 1-3: position error [rev], Q1.7.16 LE.
    uint16_t vel_err;     ///< Bytes 4-5: velocity error [RPS], bfloat16.
    union {
        struct {
            uint16_t queue : 4;        ///< Bits 0-3: queued segments, saturating at 15.
            uint16_t time_scale : 12;  ///< Bits 4-15: Q1.3.8 time multiplier.
        };
        uint16_t queue_ts;             ///< Whole-word view of bytes 6-7.
    };
};
static_assert(sizeof(TpMotionStatus3) == 8, "TpMotionStatus3 must fill one frame");

/** @brief 0x09 Motor Properties (RW): thermal limit and the electrical constants. */
struct TpMotorProperties {
    uint8_t  cmd;         ///< Byte 0: TP_OP_MOTOR_PROPERTIES.
    uint8_t  temp_lim;    ///< Byte 1: overtemperature trip point [degC].
    uint16_t resistance;  ///< Bytes 2-3: winding resistance [Ohm], bfloat16.
    uint16_t inductance;  ///< Bytes 4-5: effective inductance [H], bfloat16.
    uint16_t torque;      ///< Bytes 6-7: torque constant Kt [N*m/A], bfloat16.
};
static_assert(sizeof(TpMotorProperties) == 8, "TpMotorProperties must fill one frame");

/** @brief 0x0A Encoder Status (R): raw sensor angle, turn count and health bits. */
struct TpEncoderStatus {
    uint8_t  cmd;         ///< Byte 0: TP_OP_ENCODER_STATUS.
    uint8_t  health;      ///< Byte 1: sensor health bits (0 = no error reported).
    uint16_t angle;       ///< Bytes 2-3: raw single-turn angle, top 16 bits of the sensor word.
    int32_t  rotations;   ///< Bytes 4-7: accumulated full revolutions (signed).
};
static_assert(sizeof(TpEncoderStatus) == 8, "TpEncoderStatus must fill one frame");

/** @brief 0x0B Driver Status (R): lifecycle state, faults, warnings, sub-states. */
struct TpDriverStatus {
    uint8_t cmd;  ///< Byte 0: TP_OP_DRIVER_STATUS.
    union {
        struct {
            uint8_t state : 4;  ///< Bits 0-3: TpDriveState.
            uint8_t mode : 4;   ///< Bits 4-7: TpControlMode.
        };
        uint8_t state_mode;     ///< Whole-byte view of byte 1.
    };
    uint8_t  fault;         ///< Byte 2: DriverPWM fault flags, 1:1.
    uint8_t  warn;          ///< Byte 3: TpWarn flags.
    uint8_t  op_status;     ///< Byte 4: OperationStatus of the current-control layer.
    uint8_t  motor_status;  ///< Byte 5: MotorStatus of the identification state machine.
    uint16_t flags;         ///< Bytes 6-7: reserved diagnostic bits, transmitted as zero.
};
static_assert(sizeof(TpDriverStatus) == 8, "TpDriverStatus must fill one frame");

/** @brief 0x0C Calibration Status (R): stage and progress of the running routine. */
struct TpCalibrationStatus {
    uint8_t  cmd;         ///< Byte 0: TP_OP_CALIBRATION_STATUS.
    uint8_t  kind;        ///< Byte 1: TpCalibKind currently running (255 = idle).
    uint8_t  stage;       ///< Byte 2: MotorStatus stage value of the state machine.
    uint8_t  progress;    ///< Byte 3: coarse progress, 0-255 mapped to 0-100 %.
    uint8_t  flags;       ///< Byte 4: bit0 calibrated, bit1 settling, bit2 persisting, bit3 error.
    uint8_t  cal_id;      ///< Byte 5: identifier of the stored calibration table.
    uint16_t pole_pairs;  ///< Bytes 6-7: pole pairs counted by the sweep.
};
static_assert(sizeof(TpCalibrationStatus) == 8, "TpCalibrationStatus must fill one frame");

/** @brief 0x0D Comm Status (R): link health counters for bus diagnostics. */
struct TpCommStatus {
    uint8_t  cmd;        ///< Byte 0: TP_OP_COMM_STATUS.
    uint8_t  bus_state;  ///< Byte 1: bit0 error-passive, bit1 bus-off, bit2 warning.
    uint8_t  tx_errors;  ///< Byte 2: transmit error counter (hardware TEC, saturating).
    uint8_t  rx_errors;  ///< Byte 3: receive error counter (hardware REC, saturating).
    uint16_t dropped;    ///< Bytes 4-5: frames lost to RX ring overflow (saturating).
    uint16_t rejected;   ///< Bytes 6-7: protocol frames rejected with a NACK (saturating).
};
static_assert(sizeof(TpCommStatus) == 8, "TpCommStatus must fill one frame");

/** @brief 0x0E Power Status (R): bus-side power draw and thermal headroom. */
struct TpPowerStatus {
    uint8_t  cmd;        ///< Byte 0: TP_OP_POWER_STATUS.
    uint8_t  load;       ///< Byte 1: normalized load, int8 (-128 = overload).
    uint16_t bus_curr;   ///< Bytes 2-3: DC bus current [A], bfloat16.
    uint16_t bus_power;  ///< Bytes 4-5: DC bus power [W], bfloat16.
    uint8_t  headroom;   ///< Byte 6: thermal headroom to the trip point [degC], saturating.
    uint8_t  reserved;   ///< Byte 7: reserved, transmitted as zero.
};
static_assert(sizeof(TpPowerStatus) == 8, "TpPowerStatus must fill one frame");

/**
 * @brief 0x0F Heartbeat: the compact liveness chunk.
 *
 * Emitted cyclically on the telemetry identifier (0x600 + node) and, unchanged,
 * on the emergency identifier (0x000 + node) the instant the fault byte
 * changes. A master therefore needs exactly one decoder for both channels.
 *
 * Seven payload bytes were chosen to answer the four questions a supervisor
 * asks every period: is the node alive (seq), what is it doing (state, mode,
 * queue), is anything wrong (fault, warn), and how hard is it working
 * (supply, temperature, load).
 */
struct TpHeartbeat {
    uint8_t cmd;  ///< Byte 0: TP_OP_HEARTBEAT.
    union {
        struct {
            uint8_t state : 4;  ///< Bits 0-3: TpDriveState.
            uint8_t mode : 4;   ///< Bits 4-7: TpControlMode.
        };
        uint8_t state_mode;     ///< Whole-byte view of byte 1.
    };
    uint8_t fault;   ///< Byte 2: DriverPWM fault flags, 1:1 (0 = healthy).
    uint8_t warn;    ///< Byte 3: TpWarn flags.
    uint8_t supply;  ///< Byte 4: DC bus voltage, 0.25 V per LSB (0-63.75 V).
    uint8_t temp;    ///< Byte 5: temperature in whole degC, int8 (-128 = no data).
    uint8_t load;    ///< Byte 6: normalized load, int8 (-128 = overload).
    union {
        struct {
            uint8_t seq : 4;    ///< Bits 0-3: heartbeat sequence, increments every frame.
            uint8_t queue : 4;  ///< Bits 4-7: queued motion segments, saturating at 15.
        };
        uint8_t seq_queue;      ///< Whole-byte view of byte 7.
    };
};
static_assert(sizeof(TpHeartbeat) == 8, "TpHeartbeat must fill one frame");
