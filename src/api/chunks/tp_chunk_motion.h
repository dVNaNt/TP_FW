#pragma once

/**
 * @file tp_chunk_motion.h
 * @brief TPCP v1 - write-only command layouts (0xC0 - 0xDF) and the burst frame (0xE0 - 0xFF).
 *
 * The write half of the hot path. A motion segment is described by up to four
 * Motion Task frames sharing one SegmentID: the header (mode, flags, time),
 * the position, the torque / velocity pair and the acceleration / jerk / IO
 * masks. SegmentID 0 marks a single-frame setpoint that is applied at once.
 *
 * The bit-packed burst frame is the one exception to every other layout rule
 * in the protocol. It is a 64-bit bit field rather than a struct of integers,
 * and it is laid out MSB-first: its three group bits land at the top of byte
 * 0 (0xE0 - 0xFF), which no register address ever occupies. Its accessors
 * live here beside it.
 *
 * Every struct is followed immediately by its own static_assert on sizeof.
 *
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#include "tp_chunk_config.h"  // TP_PACKED and the wire definitions

// ============================================================================
// 0xC0 - 0xC3 Motion tasks
// ============================================================================

/**
 * @brief 0xC0 Motion Task 1: segment header.
 *
 * The flags word is laid out from the LSB upward exactly as the specification
 * lists the fields: mode (4), isAbs (1), PosMode (2), TorqMode (2), TimeMode
 * (2), io_trig (1), isVel (1), isAcc (1), isJrk (1), reserved (1).
 */
struct TP_PACKED TpMotionTask1 {
    uint8_t  cmd;         ///< Byte 0: TP_CMD_MOTION_TASK_1.
    uint8_t  segment_id;  ///< Byte 1: segment identity shared by every part (0 = single frame).
    uint8_t  axis_id;     ///< Byte 2: axis this segment addresses (0 = this node whatever its axis id).
    uint16_t flags;       ///< Bytes 3-4: see the TP_MT1_* accessors.
    uint8_t  time[3];     ///< Bytes 5-7: duration or time trigger [100 us], u24.
};
static_assert(sizeof(TpMotionTask1) == 8, "TpMotionTask1 must fill one frame");

/** Field accessors of TpMotionTask1::flags. */
#define TP_MT1_MODE(f)      ((uint8_t)((f) & 0x0Fu))          ///< Bits 0-3: TpMotionTaskMode.
#define TP_MT1_IS_ABS(f)    ((((f) >> 4) & 0x1u) != 0u)       ///< Bit 4: absolute position.
#define TP_MT1_POS_MODE(f)  ((uint8_t)(((f) >> 5) & 0x3u))    ///< Bits 5-6: TpMtPosMode.
#define TP_MT1_TORQ_MODE(f) ((uint8_t)(((f) >> 7) & 0x3u))    ///< Bits 7-8: TpMtTorqueMode.
#define TP_MT1_TIME_MODE(f) ((uint8_t)(((f) >> 9) & 0x3u))    ///< Bits 9-10: TpMtTimeMode.
#define TP_MT1_IO_TRIG(f)   ((((f) >> 11) & 0x1u) != 0u)      ///< Bit 11: IO trigger used.
#define TP_MT1_IS_VEL(f)    ((((f) >> 12) & 0x1u) != 0u)      ///< Bit 12: velocity used.
#define TP_MT1_IS_ACC(f)    ((((f) >> 13) & 0x1u) != 0u)      ///< Bit 13: acceleration used.
#define TP_MT1_IS_JRK(f)    ((((f) >> 14) & 0x1u) != 0u)      ///< Bit 14: jerk used.

/** @brief Build a TpMotionTask1::flags word from its fields. */
static inline uint16_t tp_mt1_flags(uint8_t mode, bool is_abs, uint8_t pos_mode, uint8_t torq_mode,
                                    uint8_t time_mode, bool io_trig, bool is_vel, bool is_acc, bool is_jrk) {
    return (uint16_t)((mode & 0x0Fu) | (is_abs ? 0x10u : 0u) | ((pos_mode & 0x3u) << 5) |
                      ((torq_mode & 0x3u) << 7) | ((time_mode & 0x3u) << 9) | (io_trig ? 0x800u : 0u) |
                      (is_vel ? 0x1000u : 0u) | (is_acc ? 0x2000u : 0u) | (is_jrk ? 0x4000u : 0u));
}

/** @brief 0xC1 Motion Task 2: absolute position or displacement. */
struct TP_PACKED TpMotionTask2 {
    uint8_t cmd;         ///< Byte 0: TP_CMD_MOTION_TASK_2.
    uint8_t segment_id;  ///< Byte 1: same SegmentID as Motion Task 1 (0 = position setpoint, absolute).
    uint8_t pos[6];      ///< Bytes 2-7: position [rev], q1.23.24 little-endian.
};
static_assert(sizeof(TpMotionTask2) == 8, "TpMotionTask2 must fill one frame");

/** @brief 0xC2 Motion Task 3: segment torque and velocity. */
struct TP_PACKED TpMotionTask3 {
    uint8_t  cmd;          ///< Byte 0: TP_CMD_MOTION_TASK_3.
    uint8_t  segment_id;   ///< Byte 1: same SegmentID as Motion Task 1 (0 = velocity / torque setpoint).
    uint16_t torque;       ///< Bytes 2-3: torque [N*m] (amperes when Kt is unknown), bfloat16; NaN = unused.
    uint8_t  reserved;     ///< Byte 4: reserved, write as zero.
    uint8_t  velocity[3];  ///< Bytes 5-7: velocity [RPS], float24; NaN = unused.
};
static_assert(sizeof(TpMotionTask3) == 8, "TpMotionTask3 must fill one frame");

/** @brief 0xC3 Motion Task 4: acceleration, jerk and the IO trigger masks. */
struct TP_PACKED TpMotionTask4 {
    uint8_t  cmd;          ///< Byte 0: TP_CMD_MOTION_TASK_4.
    uint8_t  segment_id;   ///< Byte 1: same SegmentID as Motion Task 1.
    uint16_t accel;        ///< Bytes 2-3: acceleration [RPS^2], bfloat16.
    uint16_t jerk;         ///< Bytes 4-5: jerk [RPS^3], bfloat16.
    uint8_t  io_mask_pos;  ///< Byte 6: probe inputs that end the segment when they go active.
    uint8_t  io_mask_neg;  ///< Byte 7: probe inputs that end the segment when they go inactive.
};
static_assert(sizeof(TpMotionTask4) == 8, "TpMotionTask4 must fill one frame");

// ============================================================================
// 0xCF Save config, 0xD0 MIDI
// ============================================================================

/** @brief 0xCF Save Config: persist, restore or erase stored configuration. */
struct TP_PACKED TpSaveConfig {
    uint8_t  cmd;         ///< Byte 0: TP_CMD_SAVE_CONFIG.
    uint8_t  catalog[3];  ///< Bytes 1-3: TP_CATALOG_* bits (0xFFFFFF = everything), u24.
    uint32_t keyword;     ///< Bytes 4-7: TP_SAVE_KEY / TP_DEFAULTS_KEY / TP_ERASE_KEY.
};
static_assert(sizeof(TpSaveConfig) == 8, "TpSaveConfig must fill one frame");

/** @brief 0xD0 MIDI Control: play one note through the motor coils. */
struct TP_PACKED TpMidi {
    uint8_t  cmd;        ///< Byte 0: TP_CMD_MIDI.
    uint8_t  wave_mode;  ///< Byte 1: bits 0-3 TpMidiWaveform, bits 4-7 mode (reserved).
    uint8_t  tone;       ///< Byte 2: MIDI note number (69 = A4 = 440 Hz); 0 = silence.
    uint8_t  volume;     ///< Byte 3: 0..255 = 0..100 % of the maximum safe amplitude.
    uint16_t duration;   ///< Bytes 4-5: note duration [ms], bfloat16; 0 = until the next command.
    uint8_t  attack;     ///< Byte 6: attack share of the duration, 0..255 = 0..100 %.
    uint8_t  decay;      ///< Byte 7: decay share of the duration, 0..255 = 0..100 %.
};
static_assert(sizeof(TpMidi) == 8, "TpMidi must fill one frame");

// ============================================================================
// 0xE0 - 0xFF Burst mode
// ============================================================================

/**
 * @brief Bit-packed burst frame (PVT or PVcTF, selected by System Control's burst step).
 *
 * Wire layout, MSB first (the documented exception to the little-endian rule):
 *
 * | Byte | Bits | PVT (burst step = 0)                | PVcTF (burst step > 0)              |
 * |:----:|:----:|:------------------------------------|:------------------------------------|
 * | 0    | 7-5  | 111                                 | 111                                 |
 * | 0    | 4-0  | time21 bits 20-16                   | frame counter (5 bits)              |
 * | 1-2  | 7-0  | time21 bits 15-0                    | segment force [N*m], bfloat16       |
 * | 3-4  | 7-0  | velocity [RPS], bfloat16            | velocity [RPS], bfloat16            |
 * | 5-7  | 7-0  | position, q1.7.16 masked absolute   | position, q1.7.16 masked absolute   |
 *
 * time21 is (system time [us] / 250) mod 2^21 - an absolute time tag, so a
 * lost frame shifts nothing. The position is the low 24 bits of the absolute
 * q1.15.16 position: the node reconstructs the nearest absolute value to its
 * queue tail, which is unambiguous while segments stay under 128 revolutions.
 */
struct TP_PACKED TpBurst {
    uint8_t b[8];  ///< Raw frame bytes; use the helpers below to access fields.
};
static_assert(sizeof(TpBurst) == 8, "TpBurst must fill one frame");

/** @brief True when a received frame is a bit-packed burst (bits 7..5 of byte 0 set). */
static inline bool tp_burst_is(const uint8_t* b) {
    return tp_cmd_is_burst(b[0]);
}

/** @brief The 21-bit time tag of a PVT burst frame [250 us units]. */
static inline uint32_t tp_burst_time21(const uint8_t* b) {
    return ((uint32_t)(b[0] & 0x1Fu) << 16) | ((uint32_t)b[1] << 8) | (uint32_t)b[2];
}

/** @brief The 5-bit frame counter of a PVcTF burst frame. */
static inline uint8_t tp_burst_frame(const uint8_t* b) {
    return (uint8_t)(b[0] & 0x1Fu);
}

/** @brief The bfloat16 force of a PVcTF burst frame (bytes 1-2, MSB first). */
static inline uint16_t tp_burst_force_raw(const uint8_t* b) {
    return (uint16_t)(((uint16_t)b[1] << 8) | (uint16_t)b[2]);
}

/** @brief The bfloat16 velocity of a burst frame (bytes 3-4, MSB first). */
static inline uint16_t tp_burst_vel_raw(const uint8_t* b) {
    return (uint16_t)(((uint16_t)b[3] << 8) | (uint16_t)b[4]);
}

/** @brief The masked q1.7.16 position of a burst frame, sign-extended (bytes 5-7, MSB first). */
static inline int32_t tp_burst_pos24(const uint8_t* b) {
    const uint32_t raw = ((uint32_t)b[5] << 16) | ((uint32_t)b[6] << 8) | (uint32_t)b[7];
    return (int32_t)(raw << 8) >> 8;  // Sign-extend from bit 23
}

/**
 * @brief Reconstruct an absolute q1.15.16 position from its 24-bit masked form.
 *
 * The masked value is the low 24 bits of the absolute position. The absolute
 * position congruent to it that lies nearest to @p reference is returned, so
 * the result is exact as long as the true position is within 128 revolutions
 * of the reference.
 *
 * @param masked    24-bit masked position (sign-extended, as tp_burst_pos24 returns it).
 * @param reference Absolute position to resolve against [rev], q1.15.16.
 */
static inline int32_t tp_burst_unmask(int32_t masked, int32_t reference) {
    const int32_t delta = (int32_t)(((uint32_t)masked - (uint32_t)reference) << 8) >> 8;  // Nearest, mod 2^24
    return reference + delta;
}

/**
 * @brief Pack a PVT burst frame.
 * @param b       Destination frame bytes.
 * @param time21  Time tag [250 us], masked to 21 bits.
 * @param vel_rps Velocity at the point [RPS].
 * @param pos_q16 Absolute position [rev], q1.15.16 (only the low 24 bits travel).
 */
static inline void tp_burst_pack_pvt(uint8_t* b, uint32_t time21, float vel_rps, int32_t pos_q16) {
    const uint16_t v = tp_pack_bf16(vel_rps);                     // Velocity as bfloat16
    const uint32_t p = (uint32_t)pos_q16 & 0x00FFFFFFu;           // Masked position
    b[0] = (uint8_t)(TP_CMD_GROUP_BURST | (uint8_t)((time21 >> 16) & 0x1Fu));  // Group bits + time 20-16
    b[1] = (uint8_t)((time21 >> 8) & 0xFFu);                      // Time bits 15-8
    b[2] = (uint8_t)(time21 & 0xFFu);                             // Time bits 7-0
    b[3] = (uint8_t)(v >> 8);                                     // Velocity high byte
    b[4] = (uint8_t)(v & 0xFFu);                                  // Velocity low byte
    b[5] = (uint8_t)((p >> 16) & 0xFFu);                          // Position bits 23-16
    b[6] = (uint8_t)((p >> 8) & 0xFFu);                           // Position bits 15-8
    b[7] = (uint8_t)(p & 0xFFu);                                  // Position bits 7-0
}

/**
 * @brief Pack a PVcTF burst frame.
 * @param b        Destination frame bytes.
 * @param frame    5-bit frame counter.
 * @param force    Segment force [N*m].
 * @param vel_rps  Velocity at the point [RPS].
 * @param pos_q16  Absolute position [rev], q1.15.16 (only the low 24 bits travel).
 */
static inline void tp_burst_pack_pvctf(uint8_t* b, uint8_t frame, float force, float vel_rps, int32_t pos_q16) {
    tp_burst_pack_pvt(b, 0u, vel_rps, pos_q16);                   // Group bits, velocity, position
    const uint16_t f = tp_pack_bf16(force);                       // Force as bfloat16
    b[0] = (uint8_t)(TP_CMD_GROUP_BURST | (frame & 0x1Fu));       // Group bits + frame counter
    b[1] = (uint8_t)(f >> 8);                                     // Force high byte
    b[2] = (uint8_t)(f & 0xFFu);                                  // Force low byte
}
