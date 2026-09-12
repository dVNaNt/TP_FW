#pragma once

/**
 * @file tp_chunk_motion.h
 * @brief TPCP v1 - real-time control and motion frame layouts (opcodes 0x10 - 0x1F).
 *
 * The write half of the hot path: mode selection, the three target kinds,
 * envelope limits, stop / anchor, and the trajectory queue.
 * 
 * The bit-packed motion burst (TpMotionBurst) is the one exception to every
 * other layout rule in the protocol. It is a 64-bit bit field rather than a
 * struct of integers, and it is laid out MSB-first so that its two format flags
 * land in byte 0 with bit 7 set - which no opcode ever has. That is what lets a
 * burst frame share the request channel with opcode frames and still be told
 * apart by inspecting a single byte. Its accessors live here beside it.
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
