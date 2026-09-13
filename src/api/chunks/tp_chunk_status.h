#pragma once

/**
 * @file tp_chunk_status.h
 * @brief TPCP v1 - read-only register layouts (CMD 0x40 - 0x7F) and the generic replies.
 *
 * The read-only half of the protocol: everything a master polls, batches or
 * subscribes to. Any of these can be named in a telemetry slot and streamed
 * on 0x500 + node, which is why the layouts carry their own CMD in byte 0 -
 * a streamed frame decodes exactly like an answered read.
 *
 * TpPulseBeat closes the group and is the one layout sent unsolicited on two
 * different channels: as the periodic PulseBeat, and verbatim as the
 * emergency frame when a fault edge is detected.
 *
 * Every struct is followed immediately by its own static_assert on sizeof: the
 * bitfield and padding rules that make these layouts land exactly on eight
 * bytes are compiler behaviour, so the assertion is the only thing standing
 * between a careless edit and a silent wire-format break. Keep them adjacent.
 *
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#include "tp_chunk_config.h"  // TP_PACKED and the wire definitions

// ============================================================================
// 0x40 / 0x41 Batch reads
// ============================================================================

/**
 * @brief 0x40 Batch read request: up to seven read registers in one frame.
 *
 * TP_REG_BATCH_READ (0x40) itself is the filler: trailing fillers are
 * ignored, and a single filler BETWEEN two registers turns them into an
 * inclusive range. The node answers with one frame per register, each on its
 * response identifier.
 */
struct TP_PACKED TpBatchRead {
    uint8_t cmd;      ///< Byte 0: TP_REG_BATCH_READ.
    uint8_t regs[7];  ///< Bytes 1-7: read registers (0x00 - 0x7F), 0x40 = filler / range marker.
};
static_assert(sizeof(TpBatchRead) == 8, "TpBatchRead must fill one frame");

/**
 * @brief 0x41 Range read request: one register, a range of sub identifiers.
 *
 * For a sub-space register the range walks sub identifiers; for 0x00 it walks
 * direct-access addresses in 4-byte steps; for 0x2F it walks table samples.
 */
struct TP_PACKED TpRangeRead {
    uint8_t cmd;        ///< Byte 0: TP_REG_RANGE_READ.
    uint8_t reg;        ///< Byte 1: read register to walk.
    uint8_t start[3];   ///< Bytes 2-4: first sub identifier / address / sample (u24).
    uint8_t end[3];     ///< Bytes 5-7: last sub identifier / address / sample, inclusive (u24).
};
static_assert(sizeof(TpRangeRead) == 8, "TpRangeRead must fill one frame");

// ============================================================================
// 0x42 / 0x43 Sub-space reads
// ============================================================================

/** @brief 0x42/0x00 and 0x42/0x01 System currents: supply plus a current pair. */
struct TP_PACKED TpCurrents {
    uint8_t  cmd;     ///< Byte 0: TP_REG_CURRENTS.
    uint8_t  sub;     ///< Byte 1: TP_SUB_CURRENTS_DQ (q, d) or TP_SUB_CURRENTS_AB (alpha, beta).
    uint16_t supply;  ///< Bytes 2-3: supply current draw [A], bfloat16.
    uint16_t first;   ///< Bytes 4-5: q-axis or alpha current [A], bfloat16.
    uint16_t second;  ///< Bytes 6-7: d-axis or beta current [A], bfloat16.
};
static_assert(sizeof(TpCurrents) == 8, "TpCurrents must fill one frame");

/** @brief 0x43/0x00 Communication diagnostics: link health counters. */
struct TP_PACKED TpDiagComm {
    uint8_t  cmd;        ///< Byte 0: TP_REG_DIAG.
    uint8_t  sub;        ///< Byte 1: TP_SUB_DIAG_COMM.
    uint8_t  bus_state;  ///< Byte 2: bit0 error-passive, bit1 bus-off, bit2 warning, bit3 gateway armed.
    uint8_t  tx_errors;  ///< Byte 3: transmit error counter (hardware TEC).
    uint8_t  rx_errors;  ///< Byte 4: receive error counter (hardware REC).
    uint8_t  dropped;    ///< Byte 5: frames lost to RX ring overflow (saturating).
    uint16_t rejected;   ///< Bytes 6-7: frames answered with a NACK (saturating).
};
static_assert(sizeof(TpDiagComm) == 8, "TpDiagComm must fill one frame");

/** @brief 0x43/0x01 Calibration diagnostics: routine, stage, progress, result flags. */
struct TP_PACKED TpDiagCalibration {
    uint8_t cmd;         ///< Byte 0: TP_REG_DIAG.
    uint8_t sub;         ///< Byte 1: TP_SUB_DIAG_CALIBRATION.
    uint8_t running;     ///< Byte 2: TP_CAL_* bits of the routine currently running (0 = idle).
    uint8_t stage;       ///< Byte 3: MotorStatus stage of the electrical identification.
    uint8_t progress;    ///< Byte 4: coarse progress, 0-255 mapped to 0-100 %.
    uint8_t flags;       ///< Byte 5: bit0 angle table valid, bit1 settling, bit2 persisting, bit3 last run failed, bit4 electrical identified, bit5 profile loaded.
    uint8_t cal_id;      ///< Byte 6: identifier of the stored calibration table.
    uint8_t pole_pairs;  ///< Byte 7: pole pairs counted by the sweep (saturating).
};
static_assert(sizeof(TpDiagCalibration) == 8, "TpDiagCalibration must fill one frame");

/** @brief 0x43/0x02 Queue diagnostics: executor state and capacity. */
struct TP_PACKED TpDiagQueue {
    uint8_t cmd;         ///< Byte 0: TP_REG_DIAG.
    uint8_t sub;         ///< Byte 1: TP_SUB_DIAG_QUEUE.
    uint8_t state;       ///< Byte 2: bit0 enabled, bit1 paused, bit2 halted, bit3 starving.
    uint8_t queued;      ///< Byte 3: segments waiting to execute.
    uint8_t free_slots;  ///< Byte 4: free queue slots.
    int16_t time_scale;  ///< Bytes 5-6: active playback rate, Q10 (1024 = 1.0x).
    uint8_t dropped;     ///< Byte 7: segments rejected or discarded incomplete (saturating).
};
static_assert(sizeof(TpDiagQueue) == 8, "TpDiagQueue must fill one frame");

/** @brief 0x43/0x03 Persistence diagnostics: store state and free space. */
struct TP_PACKED TpDiagPersist {
    uint8_t  cmd;          ///< Byte 0: TP_REG_DIAG.
    uint8_t  sub;          ///< Byte 1: TP_SUB_DIAG_PERSIST.
    uint8_t  state;        ///< Byte 2: bit0 busy, bit1 config restored, bit2 motor profile restored, bit3 last save failed.
    uint8_t  config_ver;   ///< Byte 3: layout version of the stored configuration record.
    uint8_t  cal_id;       ///< Byte 4: calibration identifier of the stored motor profile.
    uint16_t config_free;  ///< Bytes 5-6: free bytes left in the configuration partition.
    uint8_t  reserved;     ///< Byte 7: reserved, transmitted as zero.
};
static_assert(sizeof(TpDiagPersist) == 8, "TpDiagPersist must fill one frame");

/** @brief 0x43/0x04 Encoder diagnostics: raw single-turn angle and turn count. */
struct TP_PACKED TpDiagEncoder {
    uint8_t  cmd;        ///< Byte 0: TP_REG_DIAG.
    uint8_t  sub;        ///< Byte 1: TP_SUB_DIAG_ENCODER.
    uint16_t angle;      ///< Bytes 2-3: raw single-turn angle, top 16 bits of the sensor word.
    int32_t  rotations;  ///< Bytes 4-7: accumulated full revolutions (signed).
};
static_assert(sizeof(TpDiagEncoder) == 8, "TpDiagEncoder must fill one frame");

/** @brief 0x43/0x05 Calibration result: the identified electrical parameters. */
struct TP_PACKED TpDiagCalibResult {
    uint8_t  cmd;         ///< Byte 0: TP_REG_DIAG.
    uint8_t  sub;         ///< Byte 1: TP_SUB_DIAG_CALIB_RESULT.
    uint16_t resistance;  ///< Bytes 2-3: identified resistance [Ohm], bfloat16.
    uint16_t induct_d;    ///< Bytes 4-5: identified d-axis inductance [H], bfloat16.
    uint16_t induct_q;    ///< Bytes 6-7: identified q-axis inductance [H], bfloat16.
};
static_assert(sizeof(TpDiagCalibResult) == 8, "TpDiagCalibResult must fill one frame");

/**
 * @brief 0x43/0x06 FOC frame diagnostics: the rotor electrical angle the
 *        current loop is using right now and the voltage it commands.
 *
 * The angle is the calibrated encoder-derived rotor angle in the FOC frame
 * (65536 per electrical period), i.e. the frame the d/q currents of 0x42/0x00
 * are measured in. Comparing it with the angle a known alignment ought to
 * produce (the electrical identification parks the rotor at d = 0) exposes a
 * wrong or stale angle table directly, without moving anything: a frame that
 * is 180 degrees off turns the position loop into positive feedback and the
 * rotor runs away at the voltage-limited speed with a small measured current.
 */
struct TP_PACKED TpDiagFoc {
    uint8_t  cmd;         ///< Byte 0: TP_REG_DIAG.
    uint8_t  sub;         ///< Byte 1: TP_SUB_DIAG_FOC.
    uint16_t angle_el;    ///< Bytes 2-3: electrical angle used by FOC, 65536 per period.
    uint16_t u_d;         ///< Bytes 4-5: commanded d-axis voltage [V], bfloat16.
    uint16_t u_q;         ///< Bytes 6-7: commanded q-axis voltage [V], bfloat16.
};
static_assert(sizeof(TpDiagFoc) == 8, "TpDiagFoc must fill one frame");

/**
 * @brief 0x43/0x07 Encoder link diagnostics: how many sensor frames failed
 *        their integrity check and whether failing frames are being dropped.
 *
 * A healthy link shows a rising ok count and zero failures. Failures point at
 * the cable, the magnet or the sensor supply; the drive drops them once the
 * check has proven itself (flag bit 0), so they cost a stale sample each
 * rather than a phantom revolution.
 */
struct TP_PACKED TpDiagEncoderLink {
    uint8_t  cmd;         ///< Byte 0: TP_REG_DIAG.
    uint8_t  sub;         ///< Byte 1: TP_SUB_DIAG_ENCODER_LINK.
    uint8_t  flags;       ///< Byte 2: bit0 check proven (bad frames dropped), bit1 link down (encoder fault),
                          ///<         bit2 sensor overspeed, bit3 weak magnetic field, bit4 sensor undervoltage.
    uint8_t  relinks;     ///< Byte 3: bus resets since boot, one per 64 consecutive failures (saturating).
    uint16_t frames_bad;  ///< Bytes 4-5: frames that failed the check since boot (saturating).
    uint16_t frames_ok;   ///< Bytes 6-7: frames that passed since boot (wrapping liveness counter).
};
static_assert(sizeof(TpDiagEncoderLink) == 8, "TpDiagEncoderLink must fill one frame");

/**
 * @brief 0x44 Black box: one part of one control-rate record, or the ring's state.
 *
 * The drive keeps the last N closed-loop passes (lib/flight_recorder.h) and
 * freezes them shortly after a fault. A record is 24 bytes and a frame
 * carries four data bytes, so it is read in parts: part 0 answers with the
 * ring's state (depth, valid count, frozen, fault byte), parts 1-6 with the
 * fields of record `index` (0 = oldest). Dev builds only: an image without
 * the recorder answers NOT_IMPLEMENTED.
 */
struct TP_PACKED TpTrace {
    uint8_t  cmd;     ///< Byte 0: TP_REG_TRACE.
    uint8_t  part;    ///< Byte 1: 0 state; 1 angle_el u16 + iq mA; 2 id mA + uq 10 mV; 3 ud 10 mV + icmd mA;
                      ///<         4 ia mA + ib mA; 5 ua 10 mV + ub 10 mV; 6 vbus 10 mV + perr 1/1000 rev.
    uint16_t index;   ///< Bytes 2-3: record index (0 = oldest); the depth in a part-0 reply.
    uint16_t a;       ///< Bytes 4-5: first field of the part; valid count in a part-0 reply.
    uint16_t b;       ///< Bytes 6-7: second field; part 0: low byte flags (bit0 frozen), high byte fault at the freeze.
};
static_assert(sizeof(TpTrace) == 8, "TpTrace must fill one frame");

/**
 * @brief 0x43/0x08 Identification failure: why the last electrical
 *        identification aborted, with the electrical conditions at that moment.
 *
 * The drive prints the same on its console, but a bench rarely has the
 * console attached; this record is that dump in six bytes. Reason codes:
 * 0 none since boot, 1 phase pattern not recognised, 2 underpower (open
 * phase, low bus), 3 overpower during the slow rotation (stalled or loaded
 * rotor), 4 a driver fault interrupted the routine (its fault byte follows),
 * 5 the HFI inductance was missing on a stage or the two probe frequencies
 * disagreed beyond the accepted spread, 255 unclassified.
 */
struct TP_PACKED TpDiagIdentFail {
    uint8_t cmd;        ///< Byte 0: TP_REG_DIAG.
    uint8_t sub;        ///< Byte 1: TP_SUB_DIAG_IDENT_FAIL.
    uint8_t reason;     ///< Byte 2: reason code (see above).
    uint8_t stage;      ///< Byte 3: MotorStatus stage that was running at the abort.
    uint8_t fault;      ///< Byte 4: driver fault byte at the abort (reason 4), else 0.
    uint8_t volts_x10;  ///< Byte 5: injected test voltage at the abort [0.1 V], saturating.
    int8_t  ia_x20;     ///< Byte 6: phase A current at the abort [0.05 A], saturating.
    int8_t  ib_x20;     ///< Byte 7: phase B current at the abort [0.05 A], saturating.
};
static_assert(sizeof(TpDiagIdentFail) == 8, "TpDiagIdentFail must fill one frame");

// ============================================================================
// 0x50 - 0x58 Status registers
// ============================================================================

/** @brief 0x50 Device code and firmware version. */
struct TP_PACKED TpDeviceInfo {
    uint8_t  cmd;     ///< Byte 0: TP_REG_DEVICE_INFO.
    uint8_t  hw_ver;  ///< Byte 1: hardware revision tag.
    uint16_t fw_ver;  ///< Bytes 2-3: firmware version (major << 8 | minor).
    uint32_t device;  ///< Bytes 4-7: device / board type code.
};
static_assert(sizeof(TpDeviceInfo) == 8, "TpDeviceInfo must fill one frame");

/**
 * @brief 0x51 System Status: IO word, uptime and the aggregated fault word.
 *
 * The fault word packs everything a supervisor needs in one read: bits 0-7
 * fault flags (DriverPWM::Fault, 1:1), bits 8-15 warning flags (TpWarn),
 * bits 16-19 drive state (TpDriveState), bits 20-23 control mode
 * (TpControlMode), bits 24-31 reserved.
 */
struct TP_PACKED TpSystemStatus {
    uint8_t  cmd;     ///< Byte 0: TP_REG_SYSTEM_STATUS.
    uint8_t  io;      ///< Byte 1: endstop / probe / homing input word.
    uint16_t uptime;  ///< Bytes 2-3: seconds since boot, bfloat16.
    uint32_t fault;   ///< Bytes 4-7: fault | warn << 8 | state << 16 | mode << 20.
};
static_assert(sizeof(TpSystemStatus) == 8, "TpSystemStatus must fill one frame");

/** Bit positions inside the TpSystemStatus fault word. */
#define TP_STATUS_FAULT_SHIFT 0u   ///< Bits 0-7: DriverPWM fault flags.
#define TP_STATUS_WARN_SHIFT  8u   ///< Bits 8-15: TpWarn flags.
#define TP_STATUS_STATE_SHIFT 16u  ///< Bits 16-19: TpDriveState.
#define TP_STATUS_MODE_SHIFT  20u  ///< Bits 20-23: TpControlMode.

/**
 * @brief 0x52 Motor Status: lifecycle, machine identity bits, temperature, power, torque.
 *
 * The specification leaves bytes 1 and 2 to be defined; here byte 1 packs the
 * drive state and control mode, byte 2 the identity and readiness bits.
 */
struct TP_PACKED TpMotorStatus {
    uint8_t  cmd;     ///< Byte 0: TP_REG_MOTOR_STATUS.
    uint8_t  status;  ///< Byte 1: bits 0-3 TpDriveState, bits 4-7 TpControlMode.
    uint8_t  info;    ///< Byte 2: bits 0-1 motor type, bits 2-3 phase connection, bit4 direction (1 = CCW), bit5 enabled, bit6 identified, bit7 calibrated.
    uint8_t  temp;    ///< Byte 3: motor temperature [degC]; 0 = no data, 255 = over range.
    uint16_t power;   ///< Bytes 4-5: instantaneous motor power [W], bfloat16.
    uint16_t torque;  ///< Bytes 6-7: instantaneous torque [N*m], bfloat16 (NaN when Kt is unknown).
};
static_assert(sizeof(TpMotorStatus) == 8, "TpMotorStatus must fill one frame");

/** @brief 0x53 Motion Status 1: following errors, queue depth and time scale. */
struct TP_PACKED TpMotionStatus1 {
    uint8_t  cmd;         ///< Byte 0: TP_REG_MOTION_STATUS_1.
    uint8_t  pos_err[3];  ///< Bytes 1-3: position error [rev], q1.7.16 little-endian.
    uint16_t vel_err;     ///< Bytes 4-5: velocity error [RPS], bfloat16.
    uint16_t queue_ts;    ///< Bytes 6-7: bits 0-3 queued segments (saturating), bits 4-15 time scale q1.3.8.
};
static_assert(sizeof(TpMotionStatus1) == 8, "TpMotionStatus1 must fill one frame");

/** @brief 0x54 Motion Status 2: executing motion mode plus the absolute position. */
struct TP_PACKED TpMotionStatus2 {
    uint8_t cmd;          ///< Byte 0: TP_REG_MOTION_STATUS_2.
    uint8_t motion_mode;  ///< Byte 1: executor mode of the running segment (0xFF = halted / none).
    uint8_t position[6];  ///< Bytes 2-7: measured position [rev], q1.23.24 little-endian.
};
static_assert(sizeof(TpMotionStatus2) == 8, "TpMotionStatus2 must fill one frame");

/** @brief 0x55 Motion Status 3: observed velocity, acceleration and torque. */
struct TP_PACKED TpMotionStatus3 {
    uint8_t  cmd;          ///< Byte 0: TP_REG_MOTION_STATUS_3.
    uint8_t  velocity[3];  ///< Bytes 1-3: observed velocity [RPS], float24.
    uint16_t accel;        ///< Bytes 4-5: observed acceleration [RPS^2], bfloat16.
    uint16_t torque;       ///< Bytes 6-7: applied torque [N*m], bfloat16 (NaN when Kt is unknown).
};
static_assert(sizeof(TpMotionStatus3) == 8, "TpMotionStatus3 must fill one frame");

/** @brief 0x56 / 0x57 56-bit position registers: homing travel, raw encoder position. */
struct TP_PACKED TpPosition56 {
    uint8_t cmd;       ///< Byte 0: TP_REG_HOMING_TRAVEL or TP_REG_ENCODER_POSITION.
    uint8_t value[7];  ///< Bytes 1-7: value [rev], q1.23.32 little-endian.
};
static_assert(sizeof(TpPosition56) == 8, "TpPosition56 must fill one frame");

/**
 * @brief 0x58 Driver Status: faults, warnings, temperature, usage and supply.
 *
 * The specification numbers this register 0x45, which lies inside the 0x42 -
 * 0x4F sub-space; it lives at 0x58 instead so byte 1 can carry the fault byte
 * rather than a sub identifier.
 */
struct TP_PACKED TpDriverStatus {
    uint8_t  cmd;        ///< Byte 0: TP_REG_DRIVER_STATUS.
    uint8_t  fault;      ///< Byte 1: DriverPWM fault flags, 1:1 (0 = healthy).
    uint8_t  warn;       ///< Byte 2: TpWarn flags.
    uint8_t  op_status;  ///< Byte 3: OperationStatus of the current-control layer.
    uint8_t  temp;       ///< Byte 4: driver temperature [degC]; 0 = no data, 255 = over range.
    int8_t   usage;      ///< Byte 5: consolidated limit usage, -128..127 = -100..100 %.
    uint16_t supply;     ///< Bytes 6-7: DC bus voltage [V], bfloat16.
};
static_assert(sizeof(TpDriverStatus) == 8, "TpDriverStatus must fill one frame");

// ============================================================================
// 0x5E / 0x5F Generic replies
// ============================================================================

/** @brief 0x5E / 0x5F Generic acknowledge or reject reply. */
struct TP_PACKED TpAck {
    uint8_t  cmd;       ///< Byte 0: TP_CMD_ACK or TP_CMD_NACK.
    uint8_t  echo;      ///< Byte 1: the CMD byte this reply refers to.
    uint8_t  error;     ///< Byte 2: TpError (always TP_ERR_NONE in an ACK).
    uint8_t  detail;    ///< Byte 3: command-specific context (sub id, segment id, fault mask ...).
    uint32_t reserved;  ///< Bytes 4-7: reserved, transmitted as zero.
};
static_assert(sizeof(TpAck) == 8, "TpAck must fill one frame");

// ============================================================================
// 0x70 - 0x7F PulseBeat
// ============================================================================

/**
 * @brief 0x7x PulseBeat: the compact liveness frame.
 *
 * Emitted cyclically on 0x500 + node and, unchanged, on 0x100 + node the
 * instant the fault byte changes (or when the beat could not be sent for the
 * configured number of retries). A master needs exactly one decoder for both.
 *
 * The bit fields are declared as whole bytes / words and accessed through
 * the helpers below, so the layout is identical on every compiler.
 */
struct TP_PACKED TpPulseBeat {
    uint8_t  cmd;           ///< Byte 0: bits 4-7 = 0x7, bits 0-3 = beat counter.
    uint16_t queue_ts;      ///< Bytes 1-2: bits 0-3 queued segments, bits 4-15 time scale q1.3.8.
    uint8_t  status_fault;  ///< Byte 3: bits 0-3 TpDriveState, bits 4-7 TpFaultGroup.
    uint16_t warn_mode_err; ///< Bytes 4-5: bits 0-3 warning group, bits 4-9 control mode, bits 10-15 following error q1.1.4.
    int8_t   load;          ///< Byte 6: motor load incl. derating; -128 = overload.
    uint8_t  io;            ///< Byte 7: endstop / probe / homing input word.
};
static_assert(sizeof(TpPulseBeat) == 8, "TpPulseBeat must fill one frame");

/** @brief Assemble bytes 1-2 of a PulseBeat from the queue depth and a Q10 time scale. */
static inline uint16_t tp_beat_queue_ts(uint8_t queued, int32_t scale_q10) {
    const uint16_t q = (queued > 15u) ? 15u : queued;              // Saturating 4-bit depth
    return (uint16_t)(q | ((uint16_t)tp_pack_timescale(scale_q10) << 4));
}

/** @brief Assemble bytes 4-5 of a PulseBeat from the warning group, mode and error ratio. */
static inline uint16_t tp_beat_warn_mode_err(uint8_t warn_group, uint8_t mode, float err_ratio) {
    return (uint16_t)((warn_group & 0x0Fu) | ((uint16_t)(mode & 0x3Fu) << 4) |
                      ((uint16_t)tp_pack_q1_1_4(err_ratio) << 10));
}

/**
 * @brief Collapse a fault byte into the 4-bit fault group of the PulseBeat.
 * @param fault DriverPWM fault flags.
 * @return TpFaultGroup: 0 none, 1 + bit index for one fault, 0xF for several.
 */
static inline uint8_t tp_fault_group(uint8_t fault) {
    if (fault == 0u) {
        return TP_FAULT_GROUP_NONE;                                 // Healthy
    }
    if ((fault & (uint8_t)(fault - 1u)) != 0u) {
        return TP_FAULT_GROUP_MULTIPLE;                             // More than one bit set
    }
    uint8_t idx = 0;                                                // Bit index of the single fault
    while (((fault >> idx) & 1u) == 0u) {
        idx++;
    }
    return (uint8_t)(idx + 1u);
}

/**
 * @brief Collapse a warning byte into the 4-bit warning group of the PulseBeat.
 * @param warn TpWarn flags.
 * @return 0 when clear, otherwise 1 + index of the most significant set bit.
 */
static inline uint8_t tp_warn_group(uint8_t warn) {
    uint8_t group = 0;                                              // 0 = no warning
    for (uint8_t i = 0; i < 8u; ++i) {
        if ((warn & (1u << i)) != 0u) {
            group = (uint8_t)(i + 1u);                              // Highest set bit wins
        }
    }
    return group;
}
