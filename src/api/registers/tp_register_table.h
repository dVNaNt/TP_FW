#pragma once

/**
 * @file tp_register_table.h
 * @brief TPCP v1 - the register map itself, plus lookup and value normalization.
 *
 * The table is the single sorted array TP_REGISTERS. It MUST stay sorted by
 * identifier: tp_find_register below is a binary search, which silently
 * reports "unknown register" for an entry that sits out of order, and
 * TpProtocol::verify_register_table re-checks the ordering once at boot so a
 * bad edit shows up as one log line rather than a field failure.
 *
 * For that reason the map is deliberately NOT split into per-group arrays,
 * even though it is by far the largest file in src/api/ - it is one data
 * structure, and cutting it into several would break the search.
 *
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#include "tp_register_desc.h"  // TpRegDesc, TpRegContext, TpRegType, the value helpers
#include "tp_register_ids.h"   // TpRegId: the identifier this table is sorted by
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
