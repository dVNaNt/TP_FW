#pragma once

/**
 * @file tp_register_ids.h
 * @brief TPCP v1 - the register identifier space.
 *
 * Identifiers are allocated in groups of 0x0100 by subsystem: 0x00xx identity
 * and communication, 0x01xx motor identity, 0x02xx protection limits, 0x03xx
 * control loop, 0x04xx motion envelope, 0x05xx live telemetry mirrors.
 *
 * The numeric order is load-bearing. TP_REGISTERS is binary-searched, so the
 * table must stay sorted by identifier; keeping the ids in one ascending list
 * here is what makes a mis-ordered table edit visible at a glance.
 *
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#include <stdint.h>  // Fixed-width integer types
// ============================================================================
// Register identifiers
// ============================================================================

/** Register identifiers, grouped by subsystem. */
enum TpRegId : uint16_t {
    // ---- 0x00xx identity, node address, communication --------------------
    TP_REG_NODE_ID          = 0x0000,  ///< u8  RW P  Bus address (0 = unassigned).
    TP_REG_ARB_BITRATE      = 0x0001,  ///< u32 RW PR Arbitration bit rate [bit/s].
    TP_REG_DATA_BITRATE     = 0x0002,  ///< u32 RW PR CAN-FD data-phase bit rate [bit/s].
    TP_REG_LINK_FLAGS       = 0x0003,  ///< u8  RW PR TP_CFG_* behaviour flags.
    TP_REG_HEARTBEAT_MS     = 0x0004,  ///< u16 RW P  Heartbeat period [ms], 0 = off.
    TP_REG_WATCHDOG_MS      = 0x0005,  ///< u16 RW P  Master-liveness timeout [ms], 0 = off.
    TP_REG_WATCHDOG_ACTION  = 0x0006,  ///< u8  RW P  TpSafeState entered on timeout.
    TP_REG_SAFE_STATE       = 0x0007,  ///< u8  RW P  TpSafeState while disabled.
    TP_REG_HW_VERSION       = 0x0008,  ///< u8  R     Hardware revision.
    TP_REG_FW_VERSION       = 0x0009,  ///< u16 R     Firmware version.
    TP_REG_DEVICE_CODE      = 0x000A,  ///< u32 R     Board type code.
    TP_REG_UPTIME_S         = 0x000B,  ///< u32 R     Seconds since boot.
    TP_REG_PROTOCOL_VERSION = 0x000C,  ///< u16 R     Protocol version.
    TP_REG_UID_LOW          = 0x000D,  ///< u32 R     Device UID, low 32 bits.
    TP_REG_UID_HIGH         = 0x000E,  ///< u32 R     Device UID, high 24 bits.

    // ---- 0x01xx motor identity and electrical parameters -----------------
    TP_REG_MOTOR_TYPE       = 0x0100,  ///< u8  RW D  MotorType (4 = stepper, 6 = BLDC).
    TP_REG_POLE_PAIRS       = 0x0101,  ///< u16 RW D  Pole pairs.
    TP_REG_PHASE_PATTERN    = 0x0102,  ///< u8  RW D  Phase order code (1 ABC, 2 CAB, 3 ACB).
    TP_REG_DIRECTION        = 0x0103,  ///< i8  RW D  Rotation direction (+1 / -1).
    TP_REG_RESISTANCE       = 0x0104,  ///< f32 RW    Winding resistance [Ohm].
    TP_REG_INDUCTANCE_D     = 0x0105,  ///< f32 RW    d-axis inductance [H].
    TP_REG_INDUCTANCE_Q     = 0x0106,  ///< f32 RW    q-axis inductance [H].
    TP_REG_TORQUE_CONSTANT  = 0x0107,  ///< f32 RW    Torque constant Kt [N*m/A].
    TP_REG_FLUX_LINKAGE     = 0x0108,  ///< f32 RW    PM flux linkage [Wb].
    TP_REG_CAL_ID           = 0x0109,  ///< u8  R     Calibration table identifier.
    TP_REG_CAL_OFFSET       = 0x010A,  ///< i32 R     Angle calibration offset.
    TP_REG_MAX_TORQUE       = 0x010B,  ///< f32 RW    Motor torque rating [N*m].

    // ---- 0x02xx protection limits ----------------------------------------
    TP_REG_CURRENT_LIMIT    = 0x0200,  ///< f32 RW P  Control current envelope [A].
    TP_REG_MAX_VOLTAGE      = 0x0201,  ///< f32 RW    Motor voltage rating [V].
    TP_REG_MAX_POWER        = 0x0202,  ///< f32 RW    Motor power rating [W].
    TP_REG_OC_LIMIT_A       = 0x0203,  ///< f32 RW P  Phase-A hardware overcurrent [A].
    TP_REG_OC_LIMIT_B       = 0x0204,  ///< f32 RW P  Phase-B hardware overcurrent [A].
    TP_REG_OC_LIMIT_C       = 0x0205,  ///< f32 RW P  Phase-C hardware overcurrent [A].
    TP_REG_OV_LIMIT         = 0x0206,  ///< f32 RW P  Overvoltage trip [V].
    TP_REG_UV_LIMIT         = 0x0207,  ///< f32 RW P  Undervoltage trip [V].
    TP_REG_OT_LIMIT         = 0x0208,  ///< f32 RW P  Overtemperature trip [degC].
    TP_REG_OT_HYSTERESIS    = 0x0209,  ///< f32 RW P  Overtemperature release band [degC].
    TP_REG_OP_LIMIT_W       = 0x020A,  ///< f32 RW P  Overpower trip [W].
    TP_REG_OP_LIMIT_A       = 0x020B,  ///< f32 RW P  Bus overcurrent trip [A].
    TP_REG_TEMP_LIMIT       = 0x020C,  ///< u8  RW    Motor thermal rating [degC].
    TP_REG_CALIB_CURRENT    = 0x020D,  ///< f32 RW    Identification current [mA].

    // ---- 0x03xx control loop ---------------------------------------------
    TP_REG_CONTROL_MODE     = 0x0300,  ///< u8  RW    Active TpControlMode.
    TP_REG_ENABLE           = 0x0301,  ///< u8  RW    Output stage enable.
    TP_REG_EPSILON_REV      = 0x0302,  ///< f32 RW    Spring saturation width [rev].
    TP_REG_DAMPING_GAIN     = 0x0303,  ///< f32 RW    Velocity damper gain.
    TP_REG_V_SOFT_RPS       = 0x0304,  ///< f32 RW    Damper softening velocity [RPS].
    TP_REG_OBS_FC_MIN       = 0x0305,  ///< f32 RW    Observer bandwidth at standstill [Hz].
    TP_REG_OBS_FC_MAX       = 0x0306,  ///< f32 RW    Observer bandwidth at speed [Hz].
    TP_REG_OBS_FC_PER_REV   = 0x0307,  ///< f32 RW    Observer bandwidth slope [Hz per rev/s].
    TP_REG_OBS_FC_SLEW      = 0x0308,  ///< f32 RW    Observer bandwidth slew rate [Hz/s].
    TP_REG_OBS_ACC_LIMIT    = 0x0309,  ///< f32 RW    Observer acceleration clamp [rev/s^2].

    // ---- 0x04xx motion envelope ------------------------------------------
    TP_REG_VEL_LIMIT        = 0x0400,  ///< f32 RW P  Velocity envelope [RPS].
    TP_REG_ACCEL_LIMIT      = 0x0401,  ///< f32 RW P  Acceleration envelope [RPS^2].
    TP_REG_JERK_LIMIT       = 0x0402,  ///< f32 RW P  Jerk envelope [RPS^3].
    TP_REG_TIME_SCALE       = 0x0403,  ///< i16 RW    Trajectory playback rate, Q10.
    TP_REG_QUEUE_DEPTH      = 0x0404,  ///< u8  R     Segments waiting in the queue.
    TP_REG_TARGET_POSITION  = 0x0405,  ///< q16 RW    Position target [rev].
    TP_REG_TARGET_VELOCITY  = 0x0406,  ///< f32 RW    Velocity target [RPS].
    TP_REG_TARGET_CURRENT   = 0x0407,  ///< f32 RW    q-axis current target [A].

    // ---- 0x05xx live telemetry mirrors (read only) ------------------------
    TP_REG_POSITION         = 0x0500,  ///< q16 R     Measured position [rev].
    TP_REG_VELOCITY         = 0x0501,  ///< f32 R     Observed velocity [RPS].
    TP_REG_APPLIED_CURRENT  = 0x0502,  ///< f32 R     Current commanded to the FOC loop [A].
    TP_REG_SUPPLY_VOLTAGE   = 0x0503,  ///< f32 R     DC bus voltage [V].
    TP_REG_POWER            = 0x0504,  ///< f32 R     Motor power [W].
    TP_REG_TEMPERATURE      = 0x0505,  ///< f32 R     Driver temperature [degC].
    TP_REG_FAULT_FLAGS      = 0x0506,  ///< u8  R     DriverPWM fault flags.
    TP_REG_ENCODER_ANGLE    = 0x0507,  ///< u32 R     Raw encoder angle word.
    TP_REG_ENCODER_TURNS    = 0x0508,  ///< i32 R     Accumulated revolutions.
    TP_REG_CONTROL_TICKS    = 0x0509,  ///< u32 R     Completed control passes.
    TP_REG_BUS_CURRENT      = 0x050A,  ///< f32 R     DC bus current [A].
    TP_REG_TONE_FREQ        = 0x050B,  ///< u16 RW    Tone frequency [Hz]; 0 is silence.
    TP_REG_TONE_AMP         = 0x050C   ///< u16 RW    Tone amplitude, Q15 fraction of bus volts.
};
