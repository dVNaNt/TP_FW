#pragma once

/**
 * @file tp_registers.h
 * @brief Typed parameter register map for the TunePulse CAN protocol.
 *
 * Fixed status chunks answer "how is the drive doing"; the register map answers
 * "what is the drive configured to do". Every tunable quantity in the firmware
 * gets a 16-bit identifier, a type, access flags and a pair of accessors, so a
 * host tool can read and write anything without a dedicated opcode per field
 * and without a compiled-in copy of the firmware's data structures.
 *
 * ## Identifier grouping
 *
 * | Range         | Group                                                    |
 * |:--------------|:---------------------------------------------------------|
 * | 0x0000-0x00FF | Identity, node address, link and communication settings   |
 * | 0x0100-0x01FF | Motor identity and identified electrical parameters       |
 * | 0x0200-0x02FF | Protection limits (current, voltage, thermal, power)      |
 * | 0x0300-0x03FF | Control loop: mode, spring/damper, observer bandwidth     |
 * | 0x0400-0x04FF | Motion envelope and trajectory playback                   |
 * | 0x0500-0x05FF | Read-only live telemetry mirrors                          |
 *
 * ## Values on the wire
 *
 * Every register value travels as a 32-bit word. TpRegType says how to read it:
 * integers are zero/sign extended, floats are the raw IEEE-754 bit pattern, and
 * Q16.16 values are the native fixed-point encoding used by the encoder. The
 * type is echoed in every reply, so a generic tool can decode a register it has
 * never seen before.
 *
 * ## Why a configuration shadow exists
 *
 * Several protection limits are write-only in the driver layer: the proxies
 * program a comparator threshold and keep nothing readable. TpNodeConfig is
 * therefore the authoritative copy of every configured value; registers read
 * and write the shadow, and tp_apply_config() pushes it into the hardware. That
 * also makes save/load trivially correct - the thing that gets persisted is
 * exactly the thing the registers report.
 *
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#include "../lib/lib.h"  // TunePulseHeart: the live firmware state
#include "tp_node.h"     // TpNodeConfig: the persisted configuration shadow
#include "tp_wire.h"     // Wire enumerations and codecs

// ============================================================================
// Register descriptor model
// ============================================================================

/** How the 32-bit register payload is to be interpreted. */
enum TpRegType : uint8_t {
    TP_T_U8  = 0,  ///< Unsigned 8-bit, zero-extended.
    TP_T_U16 = 1,  ///< Unsigned 16-bit, zero-extended.
    TP_T_U32 = 2,  ///< Unsigned 32-bit.
    TP_T_I8  = 3,  ///< Signed 8-bit, sign-extended.
    TP_T_I16 = 4,  ///< Signed 16-bit, sign-extended.
    TP_T_I32 = 5,  ///< Signed 32-bit.
    TP_T_F32 = 6,  ///< IEEE-754 single precision, raw bit pattern.
    TP_T_Q16 = 7   ///< Signed Q16.16 fixed point (revolutions).
};

/** Access and behaviour flags of one register. */
enum TpRegFlag : uint8_t {
    TP_RF_READ     = 1u << 0,  ///< Readable.
    TP_RF_WRITE    = 1u << 1,  ///< Writable.
    TP_RF_PERSIST  = 1u << 2,  ///< Included in the persisted configuration record.
    TP_RF_DISABLED = 1u << 3,  ///< Writable only while the output stage is disabled.
    TP_RF_REBOOT   = 1u << 4   ///< Takes effect after a save and a reboot.
};

/** Shorthand for the common read/write combination. */
#define TP_RF_RW (TP_RF_READ | TP_RF_WRITE)

/**
 * @brief Everything an accessor needs to reach the firmware.
 *
 * Passed by reference into every accessor so the table itself stays free of
 * global references and can be unit tested against a stub heart.
 */
struct TpRegContext {
    TunePulseHeart* tp;         ///< Live firmware state.
    TpNodeConfig*   cfg;        ///< Persisted configuration shadow.
    TpNodeIdentity* id;         ///< Immutable device identity (UID).
    bool            dirty;      ///< Set when a write touched a persisted register.
    bool            hw_reapply; ///< Set when a write needs tp_apply_config() to run.
};

/** @brief One register: identifier, type, access flags and its accessors. */
struct TpRegDesc {
    uint16_t  id;     ///< Register identifier.
    TpRegType type;   ///< Payload interpretation.
    uint8_t   flags;  ///< TpRegFlag bits.
    bool (*get)(TpRegContext& c, uint32_t& out);  ///< Read accessor (nullptr = write only).
    bool (*set)(TpRegContext& c, uint32_t value); ///< Write accessor (nullptr = read only).
};

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

// ============================================================================
// Accessor helpers
// ============================================================================

/** @brief Reinterpret a float as its raw 32-bit pattern (register payload). */
static inline uint32_t tp_reg_from_f32(float v) {
    uint32_t bits;                        // Raw representation
    __builtin_memcpy(&bits, &v, 4);       // Bit-cast without aliasing UB
    return bits;
}

/** @brief Reinterpret a raw 32-bit register payload as a float. */
static inline float tp_reg_to_f32(uint32_t v) {
    float value;                          // Reconstructed float
    __builtin_memcpy(&value, &v, 4);      // Bit-cast without aliasing UB
    return value;
}

/** @brief Reject non-finite and negative values on a physical limit write. */
static inline bool tp_reg_positive(float v) {
    return (v == v) && (v > 0.0f) && (v < 1.0e9f);  // NaN fails the self-compare
}

// ============================================================================
// The register table
// ============================================================================

/**
 * @brief The register map, sorted by identifier so lookup is a binary search.
 *
 * Accessors are capture-less lambdas, which decay to plain function pointers:
 * the whole table is a constant array in flash and costs no RAM. Each accessor
 * returns false to reject a value, which the engine turns into a NACK carrying
 * TP_ERR_OUT_OF_RANGE.
 */
static const TpRegDesc TP_REGISTERS[] = {
    // ---- 0x00xx identity, node address, communication --------------------
    {TP_REG_NODE_ID, TP_T_U8, TP_RF_RW | TP_RF_PERSIST,
     [](TpRegContext& c, uint32_t& v) { v = c.cfg->node_id; return true; },
     [](TpRegContext& c, uint32_t v) {
         if (v > TP_NODE_MAX) return false;              // 0xFF is the broadcast address
         c.cfg->node_id = (uint8_t)v; c.dirty = true; return true;
     }},
    {TP_REG_ARB_BITRATE, TP_T_U32, TP_RF_RW | TP_RF_PERSIST | TP_RF_REBOOT,
     [](TpRegContext& c, uint32_t& v) { v = (uint32_t)c.cfg->arb_kbps * 1000u; return true; },
     [](TpRegContext& c, uint32_t v) {
         if ((v < 10000u) || (v > 1000000u)) return false;  // 10 kbit/s .. 1 Mbit/s
         c.cfg->arb_kbps = (uint16_t)(v / 1000u); c.dirty = true; return true;
     }},
    {TP_REG_DATA_BITRATE, TP_T_U32, TP_RF_RW | TP_RF_PERSIST | TP_RF_REBOOT,
     [](TpRegContext& c, uint32_t& v) { v = (uint32_t)c.cfg->data_kbps * 1000u; return true; },
     [](TpRegContext& c, uint32_t v) {
         if ((v < 100000u) || (v > 8000000u)) return false;  // 100 kbit/s .. 8 Mbit/s
         c.cfg->data_kbps = (uint16_t)(v / 1000u); c.dirty = true; return true;
     }},
    {TP_REG_LINK_FLAGS, TP_T_U8, TP_RF_RW | TP_RF_PERSIST | TP_RF_REBOOT,
     [](TpRegContext& c, uint32_t& v) { v = c.cfg->flags; return true; },
     [](TpRegContext& c, uint32_t v) { c.cfg->flags = (uint8_t)v; c.dirty = true; return true; }},
    {TP_REG_HEARTBEAT_MS, TP_T_U16, TP_RF_RW | TP_RF_PERSIST,
     [](TpRegContext& c, uint32_t& v) { v = c.cfg->heartbeat_ms; return true; },
     [](TpRegContext& c, uint32_t v) {
         if (v > 60000u) return false;                   // A minute is already meaningless
         c.cfg->heartbeat_ms = (uint16_t)v; c.dirty = true; return true;
     }},
    {TP_REG_WATCHDOG_MS, TP_T_U16, TP_RF_RW | TP_RF_PERSIST,
     [](TpRegContext& c, uint32_t& v) { v = c.cfg->watchdog_ms; return true; },
     [](TpRegContext& c, uint32_t v) {
         if (v > 60000u) return false;                   // Same reasoning as the heartbeat
         c.cfg->watchdog_ms = (uint16_t)v; c.dirty = true; c.hw_reapply = true; return true;
     }},
    {TP_REG_WATCHDOG_ACTION, TP_T_U8, TP_RF_RW | TP_RF_PERSIST,
     [](TpRegContext& c, uint32_t& v) { v = c.cfg->watchdog_action; return true; },
     [](TpRegContext& c, uint32_t v) {
         if (v > TP_SAFE_HOLD) return false;             // Undefined safe state
         c.cfg->watchdog_action = (uint8_t)v; c.dirty = true; return true;
     }},
    {TP_REG_SAFE_STATE, TP_T_U8, TP_RF_RW | TP_RF_PERSIST,
     [](TpRegContext& c, uint32_t& v) { v = c.cfg->safe_state; return true; },
     [](TpRegContext& c, uint32_t v) {
         if (v > TP_SAFE_HOLD) return false;             // Undefined safe state
         c.cfg->safe_state = (uint8_t)v; c.dirty = true; c.hw_reapply = true; return true;
     }},
    {TP_REG_HW_VERSION, TP_T_U8, TP_RF_READ,
     [](TpRegContext&, uint32_t& v) { v = TP_HW_VERSION; return true; }, nullptr},
    {TP_REG_FW_VERSION, TP_T_U16, TP_RF_READ,
     [](TpRegContext&, uint32_t& v) { v = TP_FW_VERSION; return true; }, nullptr},
    {TP_REG_DEVICE_CODE, TP_T_U32, TP_RF_READ,
     [](TpRegContext&, uint32_t& v) { v = TP_DEVICE_CODE; return true; }, nullptr},
    {TP_REG_UPTIME_S, TP_T_U32, TP_RF_READ,
     [](TpRegContext&, uint32_t& v) { v = millis() / 1000u; return true; }, nullptr},
    {TP_REG_PROTOCOL_VERSION, TP_T_U16, TP_RF_READ,
     [](TpRegContext&, uint32_t& v) { v = TP_PROTOCOL_VERSION; return true; }, nullptr},
    {TP_REG_UID_LOW, TP_T_U32, TP_RF_READ,
     [](TpRegContext& c, uint32_t& v) { v = (uint32_t)c.id->raw(); return true; }, nullptr},
    {TP_REG_UID_HIGH, TP_T_U32, TP_RF_READ,
     [](TpRegContext& c, uint32_t& v) { v = (uint32_t)(c.id->raw() >> 32); return true; }, nullptr},

    // ---- 0x01xx motor identity and electrical parameters -----------------
    {TP_REG_MOTOR_TYPE, TP_T_U8, TP_RF_RW | TP_RF_DISABLED,
     [](TpRegContext& c, uint32_t& v) { v = (uint8_t)c.tp->motor_params.get_motor_type(); return true; },
     [](TpRegContext& c, uint32_t v) {
         if ((v != (uint32_t)MotorType_STEP) && (v != (uint32_t)MotorType_BLDC)) return false;
         c.tp->motor_params.set_motor_type((int8_t)v); return true;
     }},
    {TP_REG_POLE_PAIRS, TP_T_U16, TP_RF_RW | TP_RF_DISABLED,
     [](TpRegContext& c, uint32_t& v) { v = (uint32_t)c.tp->motor_params.get_pole_pairs(); return true; },
     [](TpRegContext& c, uint32_t v) {
         if ((v == 0u) || (v > 4095u)) return false;     // Same range the record header allows
         c.tp->motor_params.set_pole_pairs((int32_t)v); return true;
     }},
    {TP_REG_PHASE_PATTERN, TP_T_U8, TP_RF_RW | TP_RF_DISABLED,
     [](TpRegContext& c, uint32_t& v) {
         // Same 1/2/3 codes the MOTOR_STATUS chunk and the flash record use
         // (PHASE_CODE_ABC / _CAB / _ACB), so one wiring pattern has exactly
         // one numeric identity everywhere it appears.
         const PhasePattern p = c.tp->motor_params.get_phase_pattern();
         v = (p == PhasePattern_CBA) ? 2u : ((p == PhasePattern_ACB) ? 3u : 1u);
         return true;
     },
     [](TpRegContext& c, uint32_t v) {
         switch (v) {
             case 1u: c.tp->motor_params.set_phase_pattern(PhasePattern_ABC); return true;
             case 2u: c.tp->motor_params.set_phase_pattern(PhasePattern_CBA); return true;
             case 3u: c.tp->motor_params.set_phase_pattern(PhasePattern_ACB); return true;
             default: return false;                      // Undefined wiring pattern
         }
     }},
    {TP_REG_DIRECTION, TP_T_I8, TP_RF_RW | TP_RF_DISABLED,
     [](TpRegContext& c, uint32_t& v) { v = (uint32_t)(int32_t)c.tp->motor_params.get_direction(); return true; },
     [](TpRegContext& c, uint32_t v) {
         const int32_t d = (int32_t)v;                   // Already sign-extended by the engine
         if ((d != 1) && (d != -1)) return false;        // Only the two polarities are meaningful
         c.tp->motor_params.set_direction((int16_t)d); return true;
     }},
    {TP_REG_RESISTANCE, TP_T_F32, TP_RF_RW,
     [](TpRegContext& c, uint32_t& v) { v = tp_reg_from_f32(c.tp->motor_params.get_resistance()); return true; },
     [](TpRegContext& c, uint32_t v) {
         const float f = tp_reg_to_f32(v);
         if (!tp_reg_positive(f)) return false;
         c.tp->motor_params.set_resistance(f); return true;
     }},
    {TP_REG_INDUCTANCE_D, TP_T_F32, TP_RF_RW,
     [](TpRegContext& c, uint32_t& v) { v = tp_reg_from_f32(c.tp->motor_params.get_inductance_d()); return true; },
     [](TpRegContext& c, uint32_t v) {
         const float f = tp_reg_to_f32(v);
         if (!tp_reg_positive(f)) return false;
         c.tp->motor_params.set_inductance_d(f); return true;
     }},
    {TP_REG_INDUCTANCE_Q, TP_T_F32, TP_RF_RW,
     [](TpRegContext& c, uint32_t& v) { v = tp_reg_from_f32(c.tp->motor_params.get_inductance_q()); return true; },
     [](TpRegContext& c, uint32_t v) {
         const float f = tp_reg_to_f32(v);
         if (!tp_reg_positive(f)) return false;
         c.tp->motor_params.set_inductance_q(f); return true;
     }},
    {TP_REG_TORQUE_CONSTANT, TP_T_F32, TP_RF_RW,
     [](TpRegContext& c, uint32_t& v) { v = tp_reg_from_f32(c.tp->motor_params.get_torque_constant()); return true; },
     [](TpRegContext& c, uint32_t v) {
         const float f = tp_reg_to_f32(v);
         if (!tp_reg_positive(f)) return false;
         c.tp->motor_params.set_torque_constant(f); return true;
     }},
    {TP_REG_FLUX_LINKAGE, TP_T_F32, TP_RF_RW,
     [](TpRegContext& c, uint32_t& v) { v = tp_reg_from_f32(c.tp->motor_params.get_flux_linkage()); return true; },
     [](TpRegContext& c, uint32_t v) {
         const float f = tp_reg_to_f32(v);
         if (!tp_reg_positive(f)) return false;
         c.tp->motor_params.set_flux_linkage(f); return true;
     }},
    {TP_REG_CAL_ID, TP_T_U8, TP_RF_READ,
     [](TpRegContext& c, uint32_t& v) { v = c.tp->motor_params.get_cal_id(); return true; }, nullptr},
    {TP_REG_CAL_OFFSET, TP_T_I32, TP_RF_READ,
     [](TpRegContext& c, uint32_t& v) { v = (uint32_t)c.tp->motor_params.get_calibration_offset(); return true; }, nullptr},
    {TP_REG_MAX_TORQUE, TP_T_F32, TP_RF_RW,
     [](TpRegContext& c, uint32_t& v) { v = tp_reg_from_f32(c.tp->motor_params.get_max_torque()); return true; },
     [](TpRegContext& c, uint32_t v) {
         const float f = tp_reg_to_f32(v);
         if (!tp_reg_positive(f)) return false;
         c.tp->motor_params.set_max_torque(f); return true;
     }},

    // ---- 0x02xx protection limits ----------------------------------------
    {TP_REG_CURRENT_LIMIT, TP_T_F32, TP_RF_RW | TP_RF_PERSIST,
     [](TpRegContext& c, uint32_t& v) { v = tp_reg_from_f32(c.cfg->current_limit_a); return true; },
     [](TpRegContext& c, uint32_t v) {
         const float f = tp_reg_to_f32(v);
         if (!tp_reg_positive(f) || (f > 20.0f)) return false;   // Beyond any CLN17-class stage
         c.cfg->current_limit_a = f; c.dirty = true; c.hw_reapply = true; return true;
     }},
    {TP_REG_MAX_VOLTAGE, TP_T_F32, TP_RF_RW,
     [](TpRegContext& c, uint32_t& v) { v = tp_reg_from_f32(c.tp->motor_params.get_max_voltage()); return true; },
     [](TpRegContext& c, uint32_t v) {
         const float f = tp_reg_to_f32(v);
         if (!tp_reg_positive(f)) return false;
         c.tp->motor_params.set_max_voltage(f); return true;
     }},
    {TP_REG_MAX_POWER, TP_T_F32, TP_RF_RW,
     [](TpRegContext& c, uint32_t& v) { v = tp_reg_from_f32(c.tp->motor_params.get_max_power()); return true; },
     [](TpRegContext& c, uint32_t v) {
         const float f = tp_reg_to_f32(v);
         if (!tp_reg_positive(f)) return false;
         c.tp->motor_params.set_max_power(f); return true;
     }},
    {TP_REG_OC_LIMIT_A, TP_T_F32, TP_RF_RW | TP_RF_PERSIST | TP_RF_DISABLED,
     [](TpRegContext& c, uint32_t& v) { v = tp_reg_from_f32(c.cfg->oc_limit_a); return true; },
     [](TpRegContext& c, uint32_t v) {
         const float f = tp_reg_to_f32(v);
         if (!tp_reg_positive(f) || (f > 20.0f)) return false;
         c.cfg->oc_limit_a = f; c.dirty = true; c.hw_reapply = true; return true;
     }},
    {TP_REG_OC_LIMIT_B, TP_T_F32, TP_RF_RW | TP_RF_PERSIST | TP_RF_DISABLED,
     [](TpRegContext& c, uint32_t& v) { v = tp_reg_from_f32(c.cfg->oc_limit_b); return true; },
     [](TpRegContext& c, uint32_t v) {
         const float f = tp_reg_to_f32(v);
         if (!tp_reg_positive(f) || (f > 20.0f)) return false;
         c.cfg->oc_limit_b = f; c.dirty = true; c.hw_reapply = true; return true;
     }},
    {TP_REG_OC_LIMIT_C, TP_T_F32, TP_RF_RW | TP_RF_PERSIST | TP_RF_DISABLED,
     [](TpRegContext& c, uint32_t& v) { v = tp_reg_from_f32(c.cfg->oc_limit_c); return true; },
     [](TpRegContext& c, uint32_t v) {
         const float f = tp_reg_to_f32(v);
         if (!tp_reg_positive(f) || (f > 20.0f)) return false;
         c.cfg->oc_limit_c = f; c.dirty = true; c.hw_reapply = true; return true;
     }},
    {TP_REG_OV_LIMIT, TP_T_F32, TP_RF_RW | TP_RF_PERSIST | TP_RF_DISABLED,
     [](TpRegContext& c, uint32_t& v) { v = tp_reg_from_f32(c.cfg->ov_limit_v); return true; },
     [](TpRegContext& c, uint32_t v) {
         const float f = tp_reg_to_f32(v);
         if (!tp_reg_positive(f) || (f > 60.0f)) return false;   // Bus capacitor rating
         c.cfg->ov_limit_v = f; c.dirty = true; c.hw_reapply = true; return true;
     }},
    {TP_REG_UV_LIMIT, TP_T_F32, TP_RF_RW | TP_RF_PERSIST | TP_RF_DISABLED,
     [](TpRegContext& c, uint32_t& v) { v = tp_reg_from_f32(c.cfg->uv_limit_v); return true; },
     [](TpRegContext& c, uint32_t v) {
         const float f = tp_reg_to_f32(v);
         if (!tp_reg_positive(f) || (f > 60.0f)) return false;
         c.cfg->uv_limit_v = f; c.dirty = true; c.hw_reapply = true; return true;
     }},
    {TP_REG_OT_LIMIT, TP_T_F32, TP_RF_RW | TP_RF_PERSIST,
     [](TpRegContext& c, uint32_t& v) { v = tp_reg_from_f32(c.cfg->ot_limit_c); return true; },
     [](TpRegContext& c, uint32_t v) {
         const float f = tp_reg_to_f32(v);
         if (!tp_reg_positive(f) || (f > 150.0f)) return false;  // Silicon absolute maximum
         c.cfg->ot_limit_c = f; c.dirty = true; c.hw_reapply = true; return true;
     }},
    {TP_REG_OT_HYSTERESIS, TP_T_F32, TP_RF_RW | TP_RF_PERSIST,
     [](TpRegContext& c, uint32_t& v) { v = tp_reg_from_f32(c.cfg->ot_hyst_c); return true; },
     [](TpRegContext& c, uint32_t v) {
         const float f = tp_reg_to_f32(v);
         if (!tp_reg_positive(f) || (f > 50.0f)) return false;
         c.cfg->ot_hyst_c = f; c.dirty = true; c.hw_reapply = true; return true;
     }},
    {TP_REG_OP_LIMIT_W, TP_T_F32, TP_RF_RW | TP_RF_PERSIST,
     [](TpRegContext& c, uint32_t& v) { v = tp_reg_from_f32(c.cfg->op_limit_w); return true; },
     [](TpRegContext& c, uint32_t v) {
         const float f = tp_reg_to_f32(v);
         if (!tp_reg_positive(f) || (f > 1000.0f)) return false;
         c.cfg->op_limit_w = f; c.dirty = true; c.hw_reapply = true; return true;
     }},
    {TP_REG_OP_LIMIT_A, TP_T_F32, TP_RF_RW | TP_RF_PERSIST,
     [](TpRegContext& c, uint32_t& v) { v = tp_reg_from_f32(c.cfg->op_limit_a); return true; },
     [](TpRegContext& c, uint32_t v) {
         const float f = tp_reg_to_f32(v);
         if (!tp_reg_positive(f) || (f > 20.0f)) return false;
         c.cfg->op_limit_a = f; c.dirty = true; c.hw_reapply = true; return true;
     }},
    {TP_REG_TEMP_LIMIT, TP_T_U8, TP_RF_RW,
     [](TpRegContext& c, uint32_t& v) { v = c.tp->motor_params.get_max_temp(); return true; },
     [](TpRegContext& c, uint32_t v) {
         if ((v == 0u) || (v > 200u)) return false;
         c.tp->motor_params.set_max_temp((uint8_t)v); return true;
     }},
    {TP_REG_CALIB_CURRENT, TP_T_F32, TP_RF_RW | TP_RF_DISABLED,
     [](TpRegContext& c, uint32_t& v) { v = tp_reg_from_f32(c.tp->motor_params.get_current_limit()); return true; },
     [](TpRegContext& c, uint32_t v) {
         const float f = tp_reg_to_f32(v);
         if (!tp_reg_positive(f) || (f > 5000.0f)) return false;  // Milliamperes
         c.tp->motor_params.set_current_limit(f); return true;
     }},

    // ---- 0x03xx control loop ---------------------------------------------
    {TP_REG_CONTROL_MODE, TP_T_U8, TP_RF_RW,
     [](TpRegContext& c, uint32_t& v) {
         DriveTelemetry t; c.tp->drive.read_telemetry(t); v = t.mode; return true;
     },
     [](TpRegContext& c, uint32_t v) {
         if (v > (uint32_t)TP_MODE_TONE) return false;
         return c.tp->drive.post_u32(DriveReq::SetMode, v);
     }},
    {TP_REG_ENABLE, TP_T_U8, TP_RF_RW,
     [](TpRegContext& c, uint32_t& v) {
         DriveTelemetry t; c.tp->drive.read_telemetry(t);
         v = ((t.flags & DRIVE_TLM_ENABLED) != 0u) ? 1u : 0u; return true;
     },
     [](TpRegContext& c, uint32_t v) { return c.tp->drive.post_u32(DriveReq::SetEnable, v ? 1u : 0u); }},
    {TP_REG_EPSILON_REV, TP_T_F32, TP_RF_RW,
     [](TpRegContext& c, uint32_t& v) { v = tp_reg_from_f32(c.tp->position_controller.epsilon_rev); return true; },
     [](TpRegContext& c, uint32_t v) {
         const float f = tp_reg_to_f32(v);
         if (!tp_reg_positive(f) || (f > 1.0f)) return false;
         c.tp->position_controller.epsilon_rev = f; return true;
     }},
    {TP_REG_DAMPING_GAIN, TP_T_F32, TP_RF_RW,
     [](TpRegContext& c, uint32_t& v) { v = tp_reg_from_f32(c.tp->position_controller.damping_gain); return true; },
     [](TpRegContext& c, uint32_t v) {
         const float f = tp_reg_to_f32(v);
         if ((f != f) || (f < 0.0f) || (f > 10.0f)) return false;  // NaN fails the self-compare
         c.tp->position_controller.damping_gain = f; return true;
     }},
    {TP_REG_V_SOFT_RPS, TP_T_F32, TP_RF_RW,
     [](TpRegContext& c, uint32_t& v) { v = tp_reg_from_f32(c.tp->position_controller.v_soft_rev_s); return true; },
     [](TpRegContext& c, uint32_t v) {
         const float f = tp_reg_to_f32(v);
         if (!tp_reg_positive(f) || (f > 100.0f)) return false;
         c.tp->position_controller.v_soft_rev_s = f; return true;
     }},
    {TP_REG_OBS_FC_MIN, TP_T_F32, TP_RF_RW,
     [](TpRegContext& c, uint32_t& v) { v = tp_reg_from_f32(c.tp->position_controller.obs_fc_min_hz); return true; },
     [](TpRegContext& c, uint32_t v) {
         const float f = tp_reg_to_f32(v);
         if (!tp_reg_positive(f) || (f > 5000.0f)) return false;
         c.tp->position_controller.obs_fc_min_hz = f; return true;
     }},
    {TP_REG_OBS_FC_MAX, TP_T_F32, TP_RF_RW,
     [](TpRegContext& c, uint32_t& v) { v = tp_reg_from_f32(c.tp->position_controller.obs_fc_max_hz); return true; },
     [](TpRegContext& c, uint32_t v) {
         const float f = tp_reg_to_f32(v);
         if (!tp_reg_positive(f) || (f > 5000.0f)) return false;
         c.tp->position_controller.obs_fc_max_hz = f; return true;
     }},
    {TP_REG_OBS_FC_PER_REV, TP_T_F32, TP_RF_RW,
     [](TpRegContext& c, uint32_t& v) { v = tp_reg_from_f32(c.tp->position_controller.obs_fc_per_rev_s); return true; },
     [](TpRegContext& c, uint32_t v) {
         const float f = tp_reg_to_f32(v);
         if ((f != f) || (f < 0.0f) || (f > 10000.0f)) return false;
         c.tp->position_controller.obs_fc_per_rev_s = f; return true;
     }},
    {TP_REG_OBS_FC_SLEW, TP_T_F32, TP_RF_RW,
     [](TpRegContext& c, uint32_t& v) { v = tp_reg_from_f32(c.tp->position_controller.obs_fc_slew_hz_s); return true; },
     [](TpRegContext& c, uint32_t v) {
         const float f = tp_reg_to_f32(v);
         if (!tp_reg_positive(f)) return false;
         c.tp->position_controller.obs_fc_slew_hz_s = f; return true;
     }},
    {TP_REG_OBS_ACC_LIMIT, TP_T_F32, TP_RF_RW,
     [](TpRegContext& c, uint32_t& v) { v = tp_reg_from_f32(c.tp->position_controller.obs_acc_limit_rev_s2); return true; },
     [](TpRegContext& c, uint32_t v) {
         const float f = tp_reg_to_f32(v);
         if (!tp_reg_positive(f)) return false;
         c.tp->position_controller.obs_acc_limit_rev_s2 = f; return true;
     }},

    // ---- 0x04xx motion envelope ------------------------------------------
    {TP_REG_VEL_LIMIT, TP_T_F32, TP_RF_RW | TP_RF_PERSIST,
     [](TpRegContext& c, uint32_t& v) { v = tp_reg_from_f32(c.cfg->vel_limit_rps); return true; },
     [](TpRegContext& c, uint32_t v) {
         const float f = tp_reg_to_f32(v);
         if (!tp_reg_positive(f) || (f > 1000.0f)) return false;
         c.cfg->vel_limit_rps = f; c.dirty = true;
         return c.tp->drive.post_f32(DriveReq::VelocityLimit, f);
     }},
    {TP_REG_ACCEL_LIMIT, TP_T_F32, TP_RF_RW | TP_RF_PERSIST,
     [](TpRegContext& c, uint32_t& v) { v = tp_reg_from_f32(c.cfg->accel_limit); return true; },
     [](TpRegContext& c, uint32_t v) {
         const float f = tp_reg_to_f32(v);
         if (!tp_reg_positive(f)) return false;
         c.cfg->accel_limit = f; c.dirty = true;
         return c.tp->drive.post_f32(DriveReq::AccelLimit, f);
     }},
    {TP_REG_JERK_LIMIT, TP_T_F32, TP_RF_RW | TP_RF_PERSIST,
     [](TpRegContext& c, uint32_t& v) { v = tp_reg_from_f32(c.cfg->jerk_limit); return true; },
     [](TpRegContext& c, uint32_t v) {
         const float f = tp_reg_to_f32(v);
         if (!tp_reg_positive(f)) return false;
         c.cfg->jerk_limit = f; c.dirty = true;
         return c.tp->drive.post_f32(DriveReq::JerkLimit, f);
     }},
    {TP_REG_TIME_SCALE, TP_T_I16, TP_RF_RW,
     [](TpRegContext& c, uint32_t& v) {
         DriveTelemetry t; c.tp->drive.read_telemetry(t);
         v = (uint32_t)(int32_t)t.time_scale_q10; return true;
     },
     [](TpRegContext& c, uint32_t v) { return c.tp->drive.post_i32(DriveReq::TimeScale, (int32_t)v); }},
    {TP_REG_QUEUE_DEPTH, TP_T_U8, TP_RF_READ,
     [](TpRegContext& c, uint32_t& v) {
         DriveTelemetry t; c.tp->drive.read_telemetry(t); v = t.queue_depth; return true;
     }, nullptr},
    {TP_REG_TARGET_POSITION, TP_T_Q16, TP_RF_RW,
     [](TpRegContext& c, uint32_t& v) {
         DriveTelemetry t; c.tp->drive.read_telemetry(t);
         v = (uint32_t)t.target_pos_q16; return true;
     },
     [](TpRegContext& c, uint32_t v) {
         return c.tp->drive.post_i64(DriveReq::TargetPosAbs, (int64_t)(int32_t)v);
     }},
    {TP_REG_TARGET_VELOCITY, TP_T_F32, TP_RF_RW,
     [](TpRegContext& c, uint32_t& v) {
         DriveTelemetry t; c.tp->drive.read_telemetry(t);
         v = tp_reg_from_f32(t.target_vel_rps); return true;
     },
     [](TpRegContext& c, uint32_t v) {
         const float f = tp_reg_to_f32(v);
         if (f != f) return false;                       // Reject NaN outright
         return c.tp->drive.post_f32(DriveReq::TargetVel, f);
     }},
    {TP_REG_TARGET_CURRENT, TP_T_F32, TP_RF_RW,
     [](TpRegContext& c, uint32_t& v) {
         DriveTelemetry t; c.tp->drive.read_telemetry(t);
         v = tp_reg_from_f32(t.applied_current_a); return true;
     },
     [](TpRegContext& c, uint32_t v) {
         const float f = tp_reg_to_f32(v);
         if (f != f) return false;                       // Reject NaN outright
         return c.tp->drive.post_f32(DriveReq::TargetCurrent, f);
     }},

    // ---- 0x05xx live telemetry mirrors (read only) ------------------------
    {TP_REG_POSITION, TP_T_Q16, TP_RF_READ,
     [](TpRegContext& c, uint32_t& v) { v = (uint32_t)c.tp->encoder_pos; return true; }, nullptr},
    {TP_REG_VELOCITY, TP_T_F32, TP_RF_READ,
     [](TpRegContext& c, uint32_t& v) {
         v = tp_reg_from_f32(c.tp->position_controller.getEstimatedVelocityRevS()); return true;
     }, nullptr},
    {TP_REG_APPLIED_CURRENT, TP_T_F32, TP_RF_READ,
     [](TpRegContext& c, uint32_t& v) { v = tp_reg_from_f32(c.tp->last_current_a_); return true; }, nullptr},
    {TP_REG_SUPPLY_VOLTAGE, TP_T_F32, TP_RF_READ,
     [](TpRegContext& c, uint32_t& v) { v = tp_reg_from_f32(c.tp->driver_phy.supply.get_voltage()); return true; }, nullptr},
    {TP_REG_POWER, TP_T_F32, TP_RF_READ,
     [](TpRegContext& c, uint32_t& v) { v = tp_reg_from_f32(c.tp->driver_phy.power.get_power_w()); return true; }, nullptr},
    {TP_REG_TEMPERATURE, TP_T_F32, TP_RF_READ,
     [](TpRegContext& c, uint32_t& v) { v = tp_reg_from_f32(c.tp->driver_phy.temperature.get_temperature_c()); return true; }, nullptr},
    {TP_REG_FAULT_FLAGS, TP_T_U8, TP_RF_READ,
     [](TpRegContext& c, uint32_t& v) { v = c.tp->driver_phy.get_fault(); return true; }, nullptr},
    {TP_REG_ENCODER_ANGLE, TP_T_U32, TP_RF_READ,
     [](TpRegContext& c, uint32_t& v) { v = c.tp->encoder.get_angle(); return true; }, nullptr},
    {TP_REG_ENCODER_TURNS, TP_T_I32, TP_RF_READ,
     [](TpRegContext& c, uint32_t& v) { v = (uint32_t)c.tp->encoder.get_rotations(); return true; }, nullptr},
    {TP_REG_CONTROL_TICKS, TP_T_U32, TP_RF_READ,
     [](TpRegContext& c, uint32_t& v) { v = c.tp->counter; return true; }, nullptr},
    {TP_REG_BUS_CURRENT, TP_T_F32, TP_RF_READ,
     [](TpRegContext& c, uint32_t& v) {
         v = tp_reg_from_f32(c.tp->driver_phy.power.get_supply_current_usage_a()); return true;
     }, nullptr},
    {TP_REG_TONE_FREQ, TP_T_U16, TP_RF_RW,
     [](TpRegContext& c, uint32_t& v) { v = c.tp->drive.tone_freq(); return true; },
     [](TpRegContext& c, uint32_t v) {
         // Bounded to what a motor can actually radiate. Below the floor the
         // winding is close to a short at audio rates and the note is felt
         // rather than heard; above the ceiling the inductance swallows it.
         // 2000 Hz is FrequencyGenerator's own ceiling: above it
         // set_frequency_target() refuses the note and the drive would go
         // quiet with no error to show for it.
         if ((v != 0u) && ((v < 40u) || (v > 2000u))) return false;
         return c.tp->drive.post_u32(DriveReq::SetTone,
                                     (v & 0xFFFFu) | ((uint32_t)c.tp->drive.tone_amp() << 16));
     }},
    {TP_REG_TONE_AMP, TP_T_U16, TP_RF_RW,
     [](TpRegContext& c, uint32_t& v) { v = c.tp->drive.tone_amp(); return true; },
     [](TpRegContext& c, uint32_t v) {
         // Q15 fraction of the supply rail. Capped well below full scale: this
         // path is open loop, so nothing but the winding resistance limits the
         // current a large amplitude would draw at a low note.
         if (v > 8192u) return false;
         return c.tp->drive.post_u32(DriveReq::SetTone,
                                     (uint32_t)c.tp->drive.tone_freq() | (v << 16));
     }}
};

/** Number of registers in the map. */
static constexpr uint32_t TP_REGISTER_COUNT = sizeof(TP_REGISTERS) / sizeof(TP_REGISTERS[0]);

/**
 * @brief Find a register descriptor by identifier.
 *
 * Binary search over the sorted table: at most six comparisons for the whole
 * map, so a register access costs the same whether the map has 50 entries or
 * 500.
 *
 * @param id Register identifier.
 * @return Descriptor pointer, or nullptr when the identifier is unknown.
 */
static inline const TpRegDesc* tp_find_register(uint16_t id) {
    uint32_t lo = 0;                          // Inclusive lower bound
    uint32_t hi = TP_REGISTER_COUNT;          // Exclusive upper bound
    while (lo < hi) {
        const uint32_t mid = (lo + hi) / 2u;  // Midpoint of the remaining window
        if (TP_REGISTERS[mid].id == id) {
            return &TP_REGISTERS[mid];        // Exact hit
        }
        if (TP_REGISTERS[mid].id < id) {
            lo = mid + 1u;                    // Search the upper half
        } else {
            hi = mid;                         // Search the lower half
        }
    }
    return nullptr;                           // Not in the map
}

/**
 * @brief Sign-extend a raw register payload according to its type.
 *
 * The wire carries 32 bits regardless of the declared width, so a host that
 * writes -1 into an i8 register sends 0xFFFFFFFF. Narrow unsigned types are
 * masked instead, which is what makes an out-of-range write fail its accessor
 * range check rather than silently wrapping.
 *
 * @param type Declared register type.
 * @param raw  Payload as received.
 * @return Normalized value handed to the write accessor.
 */
static inline uint32_t tp_reg_normalize(TpRegType type, uint32_t raw) {
    switch (type) {
        case TP_T_U8:  return raw & 0xFFu;                        // Truncate to the declared width
        case TP_T_U16: return raw & 0xFFFFu;                      // Truncate to the declared width
        case TP_T_I8:  return (uint32_t)(int32_t)(int8_t)raw;     // Sign-extend from 8 bits
        case TP_T_I16: return (uint32_t)(int32_t)(int16_t)raw;    // Sign-extend from 16 bits
        default:       return raw;                                // 32-bit types pass through
    }
}

// ============================================================================
// Configuration -> hardware
// ============================================================================

/**
 * @brief Push the configuration shadow into the live subsystems.
 *
 * Called once at boot after the stored record is restored, and again whenever a
 * register write touches something the hardware caches (a comparator threshold,
 * the control envelope, the watchdog reload).
 *
 * Programming the analog protection thresholds while the stage is energized
 * would move a trip point under a live current, so the registers that reach
 * them all carry TP_RF_DISABLED and the engine refuses those writes unless the
 * output stage is off.
 *
 * @param tp  Live firmware state.
 * @param cfg Configuration to apply.
 */
static inline void tp_apply_config(TunePulseHeart& tp, const TpNodeConfig& cfg) {
    // Analog protection thresholds (comparator DAC references).
    tp.driver_phy.current.overcurrent.set_limit_amps()
        .oc_ia(cfg.oc_limit_a)
        .oc_ib(cfg.oc_limit_b)
        .oc_ic(cfg.oc_limit_c);
    tp.driver_phy.supply.set_limit_volts()
        .uv_lim_volts(cfg.uv_limit_v)
        .ov_lim_volts(cfg.ov_limit_v);
    tp.driver_phy.temperature.set_limit_celsius()
        .ot_lim_celsius(cfg.ot_limit_c)
        .ot_hyst_celsius(cfg.ot_hyst_c);
    tp.driver_phy.power.overpower.set_limit()
        .op_lim_watts(cfg.op_limit_w)
        .op_lim_amps(cfg.op_limit_a);

    // Control envelopes and policy: routed through the mailbox so the ISR
    // adopts them between passes instead of mid-computation.
    tp.drive.post_f32(DriveReq::CurrentLimit, cfg.current_limit_a);
    tp.drive.post_f32(DriveReq::VelocityLimit, cfg.vel_limit_rps);
    tp.drive.post_f32(DriveReq::AccelLimit, cfg.accel_limit);
    tp.drive.post_f32(DriveReq::JerkLimit, cfg.jerk_limit);
    tp.drive.post_u32(DriveReq::SetSafeState, cfg.safe_state);

    // Watchdog reload is expressed in control ticks so the ISR never divides.
    const uint32_t ticks = (cfg.watchdog_ms == 0u)
                               ? 0u
                               : ((uint32_t)cfg.watchdog_ms * tp.drive.frequency_hz()) / 1000u;
    tp.drive.post_u32(DriveReq::ArmWatchdog, ticks);
}
