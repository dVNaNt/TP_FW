#pragma once

/**
 * @file tp_chunks.h
 * @brief TunePulse CAN Protocol (TPCP) v1 - the 8-byte frame payload layouts.
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

#include "tp_wire.h"  // Opcodes, identifier map and fixed-point codecs

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

// ============================================================================
// Real-time control frames (opcodes 0x10 - 0x1F)
// ============================================================================

/** @brief 0x10 Control Mode (W): select the control mode and the output state. */
struct TpControlModeCmd {
    uint8_t cmd;       ///< Byte 0: TP_OP_CONTROL_MODE.
    uint8_t mode;      ///< Byte 1: requested TpControlMode.
    uint8_t enable;    ///< Byte 2: 0 = leave disabled, 1 = enable the output stage.
    uint8_t reserved;  ///< Byte 3: reserved, write as zero.
    uint32_t pad;      ///< Bytes 4-7: reserved, write as zero.
};
static_assert(sizeof(TpControlModeCmd) == 8, "TpControlModeCmd must fill one frame");

/** @brief 0x11 Target Position (W): absolute or relative position command. */
struct TpTargetPosition {
    uint8_t cmd;      ///< Byte 0: TP_OP_TARGET_POSITION.
    uint8_t flags;    ///< Byte 1: bit0 relative (0 = absolute), bit1 also switch to POSITION mode.
    uint8_t pos[6];   ///< Bytes 2-7: target position, Q1.31.16 LE (revolutions).
};
static_assert(sizeof(TpTargetPosition) == 8, "TpTargetPosition must fill one frame");

/** Flag bits of TpTargetPosition::flags and TpAnchorCmd::flags. */
#define TP_POS_FLAG_RELATIVE  (1u << 0)  ///< Interpret the payload as a displacement.
#define TP_POS_FLAG_SET_MODE  (1u << 1)  ///< Switch to POSITION mode as part of the command.

/** @brief 0x12 Target Velocity (W): velocity command plus its acceleration limit. */
struct TpTargetVelocity {
    uint8_t  cmd;          ///< Byte 0: TP_OP_TARGET_VELOCITY.
    uint8_t  velocity[3];  ///< Bytes 1-3: target velocity [RPS], float24 (signed).
    uint16_t accel;        ///< Bytes 4-5: acceleration limit [RPS^2], bfloat16 (0 = unchanged).
    uint8_t  flags;        ///< Byte 6: bit1 also switch to VELOCITY mode.
    uint8_t  reserved;     ///< Byte 7: reserved, write as zero.
};
static_assert(sizeof(TpTargetVelocity) == 8, "TpTargetVelocity must fill one frame");

/** @brief 0x13 Target Torque (W): direct q-axis current command with its envelope. */
struct TpTargetTorque {
    uint8_t  cmd;       ///< Byte 0: TP_OP_TARGET_TORQUE.
    uint8_t  flags;     ///< Byte 1: bit1 also switch to TORQUE mode.
    uint16_t current;   ///< Bytes 2-3: q-axis current target [A], bfloat16 (signed).
    uint16_t limit;     ///< Bytes 4-5: current magnitude limit [A], bfloat16 (0 = unchanged).
    uint16_t reserved;  ///< Bytes 6-7: reserved, write as zero.
};
static_assert(sizeof(TpTargetTorque) == 8, "TpTargetTorque must fill one frame");

/** Flag bit shared by the target frames: adopt the matching control mode. */
#define TP_TGT_FLAG_SET_MODE (1u << 1)  ///< Switch to the mode implied by this command.

/**
 * @brief 0x14 Motion Limits (W): the absolute velocity / acceleration / jerk envelope.
 *
 * This is the frame the original draft called "Motion ABSOLUTE Limits". It
 * bounds every generated trajectory, including the point-to-point moves of
 * TP_OP_MOTION_SCURVE, which take their ramp shape from here.
 */
struct TpMotionLimits {
    uint8_t  cmd;          ///< Byte 0: TP_OP_MOTION_LIMITS.
    uint8_t  velocity[3];  ///< Bytes 1-3: velocity limit [RPS], float24.
    uint16_t accel;        ///< Bytes 4-5: acceleration limit [RPS^2], bfloat16.
    uint16_t jerk;         ///< Bytes 6-7: jerk limit [RPS^3], bfloat16.
};
static_assert(sizeof(TpMotionLimits) == 8, "TpMotionLimits must fill one frame");

/** @brief 0x15 Stop (W): controlled stop with a selectable end state. */
struct TpStopCmd {
    uint8_t  cmd;       ///< Byte 0: TP_OP_STOP.
    uint8_t  mode;      ///< Byte 1: TpStopMode.
    uint16_t reserved;  ///< Bytes 2-3: reserved, write as zero.
    uint32_t pad;       ///< Bytes 4-7: reserved, write as zero.
};
static_assert(sizeof(TpStopCmd) == 8, "TpStopCmd must fill one frame");

/** @brief 0x16 Anchor (W): redefine the current position without moving the rotor. */
struct TpAnchorCmd {
    uint8_t cmd;     ///< Byte 0: TP_OP_ANCHOR.
    uint8_t flags;   ///< Byte 1: reserved, write as zero.
    uint8_t pos[6];  ///< Bytes 2-7: value the current position takes on, Q1.31.16 LE.
};
static_assert(sizeof(TpAnchorCmd) == 8, "TpAnchorCmd must fill one frame");

/**
 * @brief Bit-packed motion burst frame (PVT / PVcTF).
 *
 * This is the one frame that is NOT byte-field oriented: the original draft
 * needs 22 + 24 + 16 = 62 payload bits plus two format flags, which is the
 * whole 64-bit frame with nothing left over for an opcode byte. The flags
 * therefore live in the two most significant bits of byte 0, and because every
 * opcode is <= 0x7F, "byte 0 has bit 7 set" unambiguously marks a burst frame.
 *
 * Wire layout, MSB first (the documented exception to the little-endian rule):
 *
 * | Byte | Bits | Field                                                     |
 * |:----:|:----:|:----------------------------------------------------------|
 * | 0    | 7    | burst, always 1                                            |
 * | 0    | 6    | const_t: 0 = PVT (bytes 6-7 are a duration), 1 = PVcTF     |
 * | 0    | 5-0  | displacement bits 21-16                                    |
 * | 1-2  | 7-0  | displacement bits 15-0 (Q1.5.16: 6 signed rev + 16 angle)  |
 * | 3-5  | 7-0  | end velocity, float24 (1.8.15) [RPS]                       |
 * | 6-7  | 7-0  | duration in 100 us units, or normalized force when const_t |
 *
 * Displacement is a 22-bit two's-complement value covering +/-32 revolutions
 * with 1/65536 resolution - the same units the encoder and the executor use.
 */
struct TpMotionBurst {
    uint8_t b[8];  ///< Raw frame bytes; use the helpers below to access fields.
};
static_assert(sizeof(TpMotionBurst) == 8, "TpMotionBurst must fill one frame");

/** Byte-0 mask marking a frame as a bit-packed motion burst. */
#define TP_BURST_MARK 0x80u

/** Byte-0 mask selecting the constant-duration (force controlled) variant. */
#define TP_BURST_CONST_T 0x40u

/**
 * @brief True when a received frame is a bit-packed motion burst.
 * @param b Frame bytes.
 */
static inline bool tp_burst_is(const uint8_t* b) {
    return (b[0] & TP_BURST_MARK) != 0u;  // Opcodes are all <= 0x7F, so bit 7 is unambiguous
}

/**
 * @brief Extract the 22-bit signed displacement of a motion burst frame.
 * @param b Frame bytes.
 * @return Displacement in Q16.16 revolutions, sign-extended to 32 bits.
 */
static inline int32_t tp_burst_disp(const uint8_t* b) {
    uint32_t raw = ((uint32_t)(b[0] & 0x3Fu) << 16)  // Bits 21-16 from byte 0
                 | ((uint32_t)b[1] << 8)             // Bits 15-8
                 | (uint32_t)b[2];                   // Bits 7-0
    return (int32_t)(raw << 10) >> 10;               // Sign-extend from bit 21
}

/**
 * @brief Extract the float24 end velocity of a motion burst frame.
 * @param b Frame bytes.
 * @return End velocity in revolutions per second.
 */
static inline float tp_burst_vel(const uint8_t* b) {
    uint32_t raw = ((uint32_t)b[3] << 16)  // Bits 23-16
                 | ((uint32_t)b[4] << 8)   // Bits 15-8
                 | (uint32_t)b[5];         // Bits 7-0
    return tp_unpack_f24(raw);             // float24 -> float
}

/**
 * @brief Extract the trailing 16-bit field (duration or force) of a burst frame.
 * @param b Frame bytes.
 * @return Duration in 100 us units, or the normalized force when const_t is set.
 */
static inline uint16_t tp_burst_tail(const uint8_t* b) {
    return (uint16_t)(((uint16_t)b[6] << 8) | (uint16_t)b[7]);  // MSB first
}

/**
 * @brief Build a motion burst frame from its three fields.
 * @param b        Destination frame bytes.
 * @param const_t  false for a PVT segment (tail = duration), true for PVcTF (tail = force).
 * @param disp_q16 Displacement in Q16.16 revolutions (clamped to +/-32 revolutions).
 * @param vel_rps  End velocity in revolutions per second.
 * @param tail     Duration in 100 us units, or the normalized force.
 */
static inline void tp_burst_pack(uint8_t* b, bool const_t, int32_t disp_q16, float vel_rps, uint16_t tail) {
    if (disp_q16 > 0x1FFFFF) {
        disp_q16 = 0x1FFFFF;                                  // Saturate at +32 revolutions
    }
    if (disp_q16 < -0x200000) {
        disp_q16 = -0x200000;                                 // Saturate at -32 revolutions
    }
    const uint32_t d = (uint32_t)disp_q16 & 0x003FFFFFu;      // Keep the 22 payload bits
    const uint32_t v = tp_pack_f24(vel_rps);                  // Velocity as float24
    b[0] = (uint8_t)(TP_BURST_MARK                            // Burst marker
                     | (const_t ? TP_BURST_CONST_T : 0u)      // Format flag
                     | (uint8_t)((d >> 16) & 0x3Fu));         // Displacement bits 21-16
    b[1] = (uint8_t)((d >> 8) & 0xFFu);                       // Displacement bits 15-8
    b[2] = (uint8_t)(d & 0xFFu);                              // Displacement bits 7-0
    b[3] = (uint8_t)((v >> 16) & 0xFFu);                      // Velocity bits 23-16
    b[4] = (uint8_t)((v >> 8) & 0xFFu);                       // Velocity bits 15-8
    b[5] = (uint8_t)(v & 0xFFu);                              // Velocity bits 7-0
    b[6] = (uint8_t)(tail >> 8);                              // Tail high byte
    b[7] = (uint8_t)(tail & 0xFFu);                           // Tail low byte
}

/** @brief 0x1A Motion S-curve (W): jerk-limited point-to-point move request. */
struct TpMotionScurve {
    uint8_t  cmd;       ///< Byte 0: TP_OP_MOTION_SCURVE.
    uint8_t  flags;     ///< Byte 1: bit0 relative target, bit2 append instead of replace.
    uint8_t  target[3]; ///< Bytes 2-4: target or displacement [rev], Q1.7.16 LE.
    uint16_t velocity;  ///< Bytes 5-6: cruise velocity [RPS], bfloat16 (0 = use the stored limit).
    uint8_t  reserved;  ///< Byte 7: reserved, write as zero.
} __attribute__((packed));
static_assert(sizeof(TpMotionScurve) == 8, "TpMotionScurve must fill one frame");

/** Flag bits of TpMotionScurve::flags. */
#define TP_SCURVE_RELATIVE (1u << 0)  ///< Target is a displacement from the current position.
#define TP_SCURVE_APPEND   (1u << 2)  ///< Queue behind the running move instead of replacing it.

/** @brief 0x1B Motion Queue Control (W): queue lifecycle and playback rate. */
struct TpMotionQueueCtl {
    uint8_t cmd;         ///< Byte 0: TP_OP_MOTION_QUEUE_CTL.
    uint8_t action;      ///< Byte 1: TpQueueAction.
    int16_t time_scale;  ///< Bytes 2-3: playback rate, Q10 (1024 = 1.0x, 0 = pause, negative = reverse).
    uint32_t reserved;   ///< Bytes 4-7: reserved, write as zero.
};
static_assert(sizeof(TpMotionQueueCtl) == 8, "TpMotionQueueCtl must fill one frame");

/** Actions accepted in TpMotionQueueCtl::action. */
enum TpQueueAction : uint8_t {
    TP_QUEUE_NOP        = 0x0u,  ///< Apply the time scale only.
    TP_QUEUE_CLEAR      = 0x1u,  ///< Drop every queued segment and hold the current position.
    TP_QUEUE_PAUSE      = 0x2u,  ///< Freeze trajectory time (equivalent to time scale 0).
    TP_QUEUE_RESUME     = 0x3u,  ///< Resume at the configured time scale.
    TP_QUEUE_TIME_SCALE = 0x4u   ///< Set the time scale explicitly from bytes 2-3.
};

/** @brief 0x1C Motion Queue Status (R): depth and state of the executor queue. */
struct TpMotionQueueStat {
    uint8_t cmd;         ///< Byte 0: TP_OP_MOTION_QUEUE_STAT.
    uint8_t state;       ///< Byte 1: bit0 enabled, bit1 paused, bit2 halted, bit3 starving.
    uint8_t queued;      ///< Byte 2: segments waiting to execute (saturating at 255).
    uint8_t free_slots;  ///< Byte 3: free queue slots (saturating at 255).
    int16_t time_scale;  ///< Bytes 4-5: active playback rate, Q10.
    uint16_t dropped;    ///< Bytes 6-7: segments rejected because the queue was full.
};
static_assert(sizeof(TpMotionQueueStat) == 8, "TpMotionQueueStat must fill one frame");

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
