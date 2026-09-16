#pragma once

/**
 * @file tp_engine_regread_impl.h
 * @brief Packing every readable register from live state.
 *
 * One case per register, each filling the matching typed view of the frame
 * union. This is the read half of the protocol in a single place: if a value
 * is visible to a master, it is packed here. The same function serves an
 * answered read, a batch or range reply and a telemetry emission, so the
 * three can never disagree about a layout.
 *
 * The functions are near-pure - they read live state and write a frame - with
 * one exception: pack_pulsebeat() may advance the beat counter, so a master
 * can detect a dropped beat.
 *
 * Positions and velocities leave through to_wire_pos() / to_wire_vel(), which
 * apply the axis offset and direction inversion of register 0x01/0x00, so the
 * master sees the axis it configured rather than the raw encoder.
 *
 * Part of TpProtocol, declared in tp_engine.h.
 *
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#include "../tp_engine.h"  // class TpProtocol - the declarations these bodies define

/** @brief Apply the axis offset and direction to a native position for the wire. */
static inline int32_t tp_axis_to_wire(const TpNodeConfig& c, int32_t pos_q16) {
    const int32_t shifted = pos_q16 + c.axis_offset_q16;  // Work-field zero
    return (c.axis_inverted != 0u) ? -shifted : shifted;   // Configured direction
}

/** @brief Undo the axis offset and direction of a wire position (absolute). */
static inline int32_t tp_axis_from_wire(const TpNodeConfig& c, int32_t pos_q16) {
    const int32_t raw = (c.axis_inverted != 0u) ? -pos_q16 : pos_q16;  // Configured direction
    return raw - c.axis_offset_q16;                                    // Encoder zero
}

/** @brief Apply the axis direction to a displacement or velocity (no offset). */
static inline float tp_axis_sign(const TpNodeConfig& c, float v) {
    return (c.axis_inverted != 0u) ? -v : v;
}

inline TpError TpProtocol::read_register(uint8_t cmd, uint8_t sub, const TpFrame& req, TpFrame& rsp) {
    if (tp_ == nullptr) {
        return TP_ERR_WRONG_STATE;                       // Not bound to a drive
    }
    rsp.u64 = 0;                                         // Clean slate for every branch
    rsp.b8[0] = cmd;                                     // Every reply carries the read-form CMD
    switch (cmd) {
        case TP_REG_DIRECT:
            return read_direct(req, rsp);

        case TP_REG_AXIS:
            return read_axis(sub, req, rsp);

        case TP_REG_TELEMETRY: {
            if (sub >= TP_TELEMETRY_SLOTS) {
                return TP_ERR_UNKNOWN_SUB;               // No such slot
            }
            const TpTelemetrySlot& s = config_.telemetry[sub];  // Slot being read
            rsp.telemetry.sub = sub;
            rsp.telemetry.reg = s.reg;
            rsp.telemetry.flags = s.flags;
            rsp.telemetry.period_ms = s.period_ms;
            return TP_ERR_NONE;
        }

        case TP_REG_BUS: {
            rsp.bus.type_comm = (uint8_t)((config_.bus_type & 0x0Fu) | ((config_.comm_mode & 0x0Fu) << 4));
            rsp.bus.baud1 = tp_baud_code(config_.arb_kbps);
            rsp.bus.baud2 = tp_baud_code(config_.data_kbps);
            rsp.bus.sync = node_time_us() / 50u;         // Node clock in the master's 50 us units
            return TP_ERR_NONE;
        }

        case TP_REG_DISCOVERY_ASSIGN:
            rsp.discovery.addr = config_.node_id;
            tp_pack_uid(rsp.discovery.uid, identity_.raw());
            return TP_ERR_NONE;

        case TP_REG_DISCOVERY_SEARCH:
            rsp.search.depth = TP_UID_BITS;              // A batch read reports the whole identity
            tp_pack_uid(rsp.search.prefix, identity_.raw());
            return TP_ERR_NONE;

        case TP_REG_SYSTEM_CONTROL:
            rsp.system.enable = is_enabled() ? TP_SYS_ENABLE : TP_SYS_DISABLE;
            rsp.system.burst_step = config_.burst_step;
            rsp.system.calibration = calibration_valid();
            rsp.system.action = calibration_running();
            return TP_ERR_NONE;

        case TP_REG_MOTOR_PROPS_1: {
            const MotorType mt = tp_->motor_params.get_motor_type();       // Machine type
            const PhasePattern pp = tp_->motor_params.get_phase_pattern(); // Wiring
            const uint8_t type = (mt == MotorType_STEP) ? 1u : ((mt == MotorType_BLDC) ? 2u : 0u);
            const uint8_t conn = (pp == PhasePattern_CBA) ? 2u : ((pp == PhasePattern_ACB) ? 3u : 1u);
            const int16_t dir = tp_->motor_params.get_direction();         // +1 / -1 / 0
            const uint8_t dir2 = (dir < 0) ? 0x3u : ((dir > 0) ? 0x1u : 0x2u);  // 2-bit signed: -1, +1, -2
            int32_t poles = tp_->motor_params.get_pole_pairs();            // Pole pairs
            if (poles < 0) {
                poles = 0;                                                 // Never negative on the wire
            }
            if (poles > 4095) {
                poles = 4095;                                              // Saturate the 12-bit field
            }
            rsp.motor1.mode_type = (uint8_t)(control_mode() | (type << 4));
            rsp.motor1.conn_dir_poles = (uint16_t)(conn | (dir2 << 2) | ((uint16_t)poles << 4));
            rsp.motor1.max_voltage = tp_pack_bf16(tp_->motor_params.get_max_voltage());
            rsp.motor1.max_current = tp_pack_bf16(config_.current_limit_a);
            return TP_ERR_NONE;
        }

        case TP_REG_MOTOR_PROPS_2:
            rsp.motor2.temp = tp_->motor_params.get_max_temp();
            rsp.motor2.resistance = tp_pack_bf16(tp_->motor_params.get_resistance());
            rsp.motor2.inductance = tp_pack_bf16(tp_->motor_params.get_effective_inductance());
            rsp.motor2.kt = tp_pack_bf16(tp_->motor_params.get_torque_constant());
            return TP_ERR_NONE;

        case TP_REG_POWER_LIMITS:
            rsp.power_limits.overdrive = config_.pt_overdrive_s;
            rsp.power_limits.baseline = tp_pack_bf16(config_.pt_baseline_w);
            rsp.power_limits.peak = tp_pack_bf16(config_.op_limit_w);
            rsp.power_limits.capacity = tp_pack_bf16(config_.pt_capacity_j);
            return TP_ERR_NONE;

        case TP_REG_MOTOR_THERMAL:
            rsp.motor_thermal.temp_now = tp_pack_temp_u8(motor_temperature());
            rsp.motor_thermal.temp_base = config_.motor_temp_base_c;
            rsp.motor_thermal.temp_limit = tp_->motor_params.get_max_temp();
            rsp.motor_thermal.derating = tp_pack_bf16(config_.motor_derating);
            rsp.motor_thermal.rth = tp_pack_bf16(config_.motor_rth);
            return TP_ERR_NONE;

        case TP_REG_DRIVER_THERMAL:
            rsp.driver_thermal.temp_now = tp_pack_temp_u8(tp_->driver_phy.temperature.get_temperature_c());
            rsp.driver_thermal.ot_warn = (uint8_t)(config_.ot_warn_c + 0.5f);
            rsp.driver_thermal.ot_fault = (uint8_t)(config_.ot_limit_c + 0.5f);
            rsp.driver_thermal.hysteresis = (uint8_t)(config_.ot_hyst_c + 0.5f);
            return TP_ERR_NONE;

        case TP_REG_SUPPLY_PROTECTION:
            rsp.supply_prot.hysteresis = tp_pack_bf16(config_.supply_hyst_v);
            rsp.supply_prot.recovery = config_.supply_recovery;
            rsp.supply_prot.ov = tp_pack_bf16(config_.ov_limit_v);
            rsp.supply_prot.uv = tp_pack_bf16(config_.uv_limit_v);
            return TP_ERR_NONE;

        case TP_REG_CURRENT_PROTECTION:
            rsp.current_prot.disbalance = config_.oc_disbalance;
            rsp.current_prot.alpha = config_.oc_alpha;
            rsp.current_prot.recovery = config_.oc_recovery;
            rsp.current_prot.peak = tp_pack_bf16(config_.peak_current_a);
            rsp.current_prot.oc = tp_pack_bf16(config_.oc_limit_a);
            return TP_ERR_NONE;

        case TP_REG_MOTION_LIMITS:
            tp_pack_u24(rsp.limits.velocity, tp_pack_f24(config_.vel_limit_rps));
            rsp.limits.accel = tp_pack_bf16(config_.accel_limit);
            rsp.limits.jerk = tp_pack_bf16(config_.jerk_limit);
            return TP_ERR_NONE;

        case TP_REG_ERROR_LIMITS:
            tp_pack_u24(rsp.limits.velocity, tp_pack_f24(config_.err_vel_rps));
            rsp.limits.accel = tp_pack_bf16(config_.err_accel);
            rsp.limits.jerk = tp_pack_bf16(config_.err_jerk);
            return TP_ERR_NONE;

        case TP_REG_WATCHDOG:
            rsp.watchdog.retry = (uint8_t)((config_.hb_retry & 0x7Fu) | (config_.hb_retry_force ? 0x80u : 0u));
            rsp.watchdog.period = tp_pack_bf16(config_.heartbeat_s);
            rsp.watchdog.delay = tp_pack_bf16(config_.heartbeat_delay_s);
            rsp.watchdog.timeout = tp_pack_bf16(config_.watchdog_s);
            return TP_ERR_NONE;

        case TP_REG_CAL_TABLE:
            return read_cal_table(req, rsp);

        case TP_REG_TRACE:
            return read_trace(req, rsp);

        case TP_REG_CURRENTS: {
            rsp.currents.sub = sub;
            rsp.currents.supply = tp_pack_bf16(tp_->driver_phy.power.get_supply_current_usage_a());
            if (sub == TP_SUB_CURRENTS_DQ) {
                // The q current is measured in the FOC frame; the master sees
                // it in encoder units, the same sign as the torque it commands.
                rsp.currents.first = tp_pack_bf16(tp_->torque_sign() * tp_->current_driver.current_dq.torque);
                rsp.currents.second = tp_pack_bf16(tp_->current_driver.current_dq.flux);
                return TP_ERR_NONE;
            }
            if (sub == TP_SUB_CURRENTS_AB) {
                const Vec2D_f32 ab = tp_->driver_phy.motor_ctrl.get_current_ab_amps();  // Coil currents
                rsp.currents.first = tp_pack_bf16(ab.a);
                rsp.currents.second = tp_pack_bf16(ab.b);
                return TP_ERR_NONE;
            }
            return TP_ERR_UNKNOWN_SUB;
        }

        case TP_REG_DIAG:
            return read_diag(sub, req, rsp);

        case TP_REG_DEVICE_INFO:
        case TP_REG_SYSTEM_STATUS:
        case TP_REG_MOTOR_STATUS:
        case TP_REG_MOTION_STATUS_1:
        case TP_REG_MOTION_STATUS_2:
        case TP_REG_MOTION_STATUS_3:
        case TP_REG_HOMING_TRAVEL:
        case TP_REG_ENCODER_POSITION:
        case TP_REG_DRIVER_STATUS:
            return read_status(cmd, rsp);

        default:
            if (tp_cmd_is_pulsebeat(cmd)) {
                pack_pulsebeat(rsp, false);          // A read does not consume a beat
                return TP_ERR_NONE;
            }
            return TP_ERR_UNKNOWN_CMD;               // Not a readable register
    }
}

inline TpError TpProtocol::read_axis(uint8_t sub, const TpFrame& req, TpFrame& rsp) {
    rsp.b8[1] = sub;                                     // Every record echoes its sub identifier
    switch (sub) {
        case TP_SUB_AXIS_CONTROL:
            rsp.axis.axis_id = config_.axis_id;
            rsp.axis.flags = config_.axis_inverted ? 0x01u : 0x00u;
            rsp.axis.offset_q16 = config_.axis_offset_q16;
            return TP_ERR_NONE;
        case TP_SUB_LED:
            rsp.led.mode = config_.led_mode;
            rsp.led.on_off = config_.led_on_off;
            rsp.led.led_id = config_.led_id;
            rsp.led.r = config_.led_r;
            rsp.led.g = config_.led_g;
            rsp.led.b = config_.led_b;
            return TP_ERR_NONE;
        case TP_SUB_GEARBOX:
            rsp.gearbox.backlash = tp_pack_bf16(config_.gear_backlash);
            rsp.gearbox.ratio_mul = config_.gear_ratio_mul;
            rsp.gearbox.ratio_div = config_.gear_ratio_div;
            return TP_ERR_NONE;
        case TP_SUB_IO_MAP: {
            const uint8_t function = req.io_map.function;        // Record keyed by device function
            const uint8_t slot = tp_io_function_slot(function);  // Storage slot
            if (slot >= TP_IO_FUNCTIONS) {
                return TP_ERR_BAD_PARAM;                         // Not a defined function
            }
            rsp.io_map.function = function;
            rsp.io_map.on_pin = config_.io_map[slot].on_pin;
            rsp.io_map.off_pin = config_.io_map[slot].off_pin;
            rsp.io_map.flags = config_.io_map[slot].flags;
            return TP_ERR_NONE;
        }
        case TP_SUB_ENCODER_1:
        case TP_SUB_ENCODER_2: {
            const TpEncoderCfg& e = config_.encoder[sub - TP_SUB_ENCODER_1];  // Declared encoder
            rsp.encoder_cfg.type_iface = e.type_iface;
            rsp.encoder_cfg.flags = e.flags;
            rsp.encoder_cfg.ppr = e.ppr;
            return TP_ERR_NONE;
        }
        case TP_SUB_HOMING_QUICK:
            rsp.homing_quick.flags = config_.homing_flags;
            rsp.homing_quick.velocity = tp_pack_bf16(config_.homing_quick_vel);
            rsp.homing_quick.torque = tp_pack_bf16(config_.homing_quick_torque);
            return TP_ERR_NONE;
        case TP_SUB_HOMING_FINE:
            rsp.homing_fine.retract = tp_pack_bf16(config_.homing_fine_retract);
            rsp.homing_fine.velocity = tp_pack_bf16(config_.homing_fine_vel);
            rsp.homing_fine.torque = tp_pack_bf16(config_.homing_fine_torque);
            return TP_ERR_NONE;
        case TP_SUB_REF_CW:
            rsp.ref_limit.offset = tp_pack_bf16(config_.ref_offset_cw);
            rsp.ref_limit.ref_angle = config_.ref_ang_cw;
            return TP_ERR_NONE;
        case TP_SUB_REF_CCW:
            rsp.ref_limit.offset = tp_pack_bf16(config_.ref_offset_ccw);
            rsp.ref_limit.ref_angle = config_.ref_ang_ccw;
            return TP_ERR_NONE;
        case TP_SUB_POS_ZERO:
            rsp.pos_zero.ref_angle = config_.ref_ang_zero;
            return TP_ERR_NONE;
        case TP_SUB_ZERO_OFFSET_CW:
            tp_pack_i48(rsp.pos_offset.offset, config_.zero_offset_cw_q32);
            return TP_ERR_NONE;
        case TP_SUB_ZERO_OFFSET_CCW:
            tp_pack_i48(rsp.pos_offset.offset, config_.zero_offset_ccw_q32);
            return TP_ERR_NONE;
        case TP_SUB_FIELD_CW:
            tp_pack_i48(rsp.pos_offset.offset, config_.field_cw_q32);
            return TP_ERR_NONE;
        case TP_SUB_FIELD_CCW:
            tp_pack_i48(rsp.pos_offset.offset, config_.field_ccw_q32);
            return TP_ERR_NONE;
        case TP_SUB_INTERNAL_OFFSET:
            tp_pack_i48(rsp.pos_offset.offset, config_.internal_offset_q32);
            return TP_ERR_NONE;
        default:
            return TP_ERR_UNKNOWN_SUB;
    }
}

inline TpError TpProtocol::read_status(uint8_t cmd, TpFrame& rsp) {
    DriveTelemetry t;                                    // Live control snapshot
    tp_->drive.read_telemetry(t);
    const bool live = (t.flags & DRIVE_TLM_LOOP_LIVE) != 0u;  // The observer advanced this pass
    switch (cmd) {
        case TP_REG_DEVICE_INFO:
            rsp.device_info.hw_ver = TP_HW_VERSION;
            rsp.device_info.fw_ver = TP_FW_VERSION;
            rsp.device_info.device = TP_DEVICE_CODE;
            return TP_ERR_NONE;

        case TP_REG_SYSTEM_STATUS:
            rsp.system_status.io = tp_->io_inputs_;
            rsp.system_status.uptime = tp_pack_bf16((float)(millis() / 1000u));
            rsp.system_status.fault = (uint32_t)tp_->fault_byte() << TP_STATUS_FAULT_SHIFT |
                                      (uint32_t)warn_flags() << TP_STATUS_WARN_SHIFT |
                                      (uint32_t)drive_state() << TP_STATUS_STATE_SHIFT |
                                      (uint32_t)control_mode() << TP_STATUS_MODE_SHIFT;
            return TP_ERR_NONE;

        case TP_REG_MOTOR_STATUS: {
            const MotorType mt = tp_->motor_params.get_motor_type();       // Machine type
            const PhasePattern pp = tp_->motor_params.get_phase_pattern(); // Wiring
            const uint8_t type = (mt == MotorType_STEP) ? 1u : ((mt == MotorType_BLDC) ? 2u : 0u);
            const uint8_t conn = (pp == PhasePattern_CBA) ? 2u : ((pp == PhasePattern_ACB) ? 3u : 1u);
            rsp.motor_status.status = (uint8_t)(drive_state() | (control_mode() << 4));
            rsp.motor_status.info = (uint8_t)(type | (conn << 2) |
                                              ((tp_->motor_params.get_direction() < 0) ? 0x10u : 0u) |
                                              (((t.flags & DRIVE_TLM_ENABLED) != 0u) ? 0x20u : 0u) |
                                              ((tp_->motor_params.get_resistance() > 0.0f) ? 0x40u : 0u) |
                                              (tp_->angle_calibrator.is_calibrated() ? 0x80u : 0u));
            rsp.motor_status.temp = tp_pack_temp_u8(motor_temperature());
            rsp.motor_status.power = tp_pack_bf16(tp_->driver_phy.power.get_power_w());
            rsp.motor_status.torque = tp_pack_bf16(amps_to_torque(tp_->last_current_a_));
            return TP_ERR_NONE;
        }

        case TP_REG_MOTION_STATUS_1: {
            int32_t perr = t.target_pos_q16 - t.position_q16;            // Following error, q1.15.16
            if (perr > 0x7FFFFF) {
                perr = 0x7FFFFF;                                         // Saturate the 24-bit field
            }
            if (perr < -0x800000) {
                perr = -0x800000;                                        // Saturate the 24-bit field
            }
            // A frozen observer must not be reported as a velocity error: with
            // the bridge released the last value the loop computed says
            // nothing about the coasting rotor.
            const float verr = live ? (tp_->position_controller.getEstimatedVelocityRevS() - t.target_vel_rps) : 0.0f;
            tp_pack_i24(rsp.motion1.pos_err, (int32_t)tp_axis_sign(config_, (float)perr));
            rsp.motion1.vel_err = tp_pack_bf16(tp_axis_sign(config_, verr));
            rsp.motion1.queue_ts = tp_beat_queue_ts(t.queue_depth, t.time_scale_q10);
            return TP_ERR_NONE;
        }

        case TP_REG_MOTION_STATUS_2:
            rsp.motion2.motion_mode = t.motion_mode;
            tp_pack_i48(rsp.motion2.position, tp_q16_to_q24(tp_axis_to_wire(config_, t.position_q16)));
            return TP_ERR_NONE;

        case TP_REG_MOTION_STATUS_3: {
            const float vel = live ? tp_->position_controller.getEstimatedVelocityRevS() : 0.0f;
            const float acc = live ? tp_->position_controller.getEstimatedAccelerationRevS2() : 0.0f;
            tp_pack_u24(rsp.motion3.velocity, tp_pack_f24(tp_axis_sign(config_, vel)));
            rsp.motion3.accel = tp_pack_bf16(tp_axis_sign(config_, acc));
            rsp.motion3.torque = tp_pack_bf16(amps_to_torque(tp_->last_current_a_));
            return TP_ERR_NONE;
        }

        case TP_REG_HOMING_TRAVEL:
            return TP_ERR_NOT_IMPLEMENTED;                               // No sensorless homing yet

        case TP_REG_ENCODER_POSITION: {
            // q1.23.32: the turn count above the raw 32-bit single-turn word.
            const int64_t raw = ((int64_t)tp_->encoder.get_rotations() << 32) | (int64_t)tp_->encoder.get_angle();
            tp_pack_i56(rsp.position56.value, raw);
            return TP_ERR_NONE;
        }

        case TP_REG_DRIVER_STATUS:
            rsp.driver_status.fault = tp_->fault_byte();
            rsp.driver_status.warn = warn_flags();
            rsp.driver_status.op_status = (uint8_t)tp_->current_driver.get_driver_status();
            rsp.driver_status.temp = tp_pack_temp_u8(tp_->driver_phy.temperature.get_temperature_c());
            rsp.driver_status.usage = (int8_t)tp_pack_usage_i8(usage_ratio());
            rsp.driver_status.supply = tp_pack_bf16(tp_->driver_phy.supply.get_voltage());
            return TP_ERR_NONE;

        default:
            return TP_ERR_UNKNOWN_CMD;
    }
}

inline TpError TpProtocol::read_diag(uint8_t sub, const TpFrame& req, TpFrame& rsp) {
    rsp.b8[1] = sub;                                     // Every record echoes its sub identifier
    switch (sub) {
        case TP_SUB_DIAG_COMM:
            rsp.diag_comm.bus_state = bus_flags_;
            rsp.diag_comm.tx_errors = bus_tx_err_;
            rsp.diag_comm.rx_errors = bus_rx_err_;
            rsp.diag_comm.dropped = (bus_dropped_ > 255u) ? 255u : (uint8_t)bus_dropped_;
            rsp.diag_comm.rejected = rejected_;
            return TP_ERR_NONE;

        case TP_SUB_DIAG_CALIBRATION: {
            // The identification's own stage word: Motor::status_ is only
            // driven through NotCalibrated by the boot sequence, so a runtime
            // run would otherwise read "done, 100 %" from its first millisecond.
            const MotorStatus ms = tp_->current_driver.get_motor_calibration_status();
            const int32_t poles = tp_->motor_params.get_pole_pairs();      // Counted pole pairs
            const uint8_t running = calibration_running();
            rsp.diag_calib.running = running;
            if ((running & TP_CAL_ELECTRICAL) != 0u) {
                // The identification walks a known sequence of MotorStatus
                // values, so the stage number doubles as a coarse progress bar.
                rsp.diag_calib.stage = (uint8_t)ms;
                rsp.diag_calib.progress = (uint8_t)((ms == MotorStatus::Success) ? 255u : (((uint32_t)ms * 255u) / 12u));
            } else {
                // Otherwise the angle sweep is the routine of interest: its
                // CalStage rides in the stage byte with bit 7 set so a master
                // can tell the two stage vocabularies apart, and the progress
                // follows the samples collected.
                rsp.diag_calib.stage = (uint8_t)(0x80u | tp_->angle_calibrator.stage());
                rsp.diag_calib.progress = tp_->angle_calibrator.progress_255();
            }
            rsp.diag_calib.flags = (uint8_t)((tp_->angle_calibrator.is_calibrated() ? 0x01u : 0u) |
                                             (tp_->angle_calibrator.is_settling() ? 0x02u : 0u) |
                                             (tp_->angle_calibrator.is_persisting() ? 0x04u : 0u) |
                                             (((ms == MotorStatus::CalibError) || tp_->angle_calibrator.is_failed()) ? 0x08u : 0u) |
                                             ((tp_->motor_params.get_resistance() > 0.0f) ? 0x10u : 0u) |
                                             (tp_->motor_params.is_loaded() ? 0x20u : 0u));
            rsp.diag_calib.cal_id = tp_->motor_params.get_cal_id();
            rsp.diag_calib.pole_pairs = (uint8_t)((poles < 0) ? 0 : ((poles > 255) ? 255 : poles));
            return TP_ERR_NONE;
        }

        case TP_SUB_DIAG_QUEUE: {
            DriveTelemetry t;                                // Live control snapshot
            tp_->drive.read_telemetry(t);
            const uint32_t dropped = (uint32_t)t.dropped_segments + seg_dropped_;  // Both loss paths
            rsp.diag_queue.state = (uint8_t)(((t.flags & DRIVE_TLM_ENABLED) != 0u ? 0x01u : 0u) |
                                             ((t.flags & DRIVE_TLM_PAUSED) != 0u ? 0x02u : 0u) |
                                             ((t.flags & DRIVE_TLM_HALTED) != 0u ? 0x04u : 0u) |
                                             ((t.flags & DRIVE_TLM_STARVING) != 0u ? 0x08u : 0u));
            rsp.diag_queue.queued = t.queue_depth;
            rsp.diag_queue.free_slots = t.queue_free;
            rsp.diag_queue.time_scale = t.time_scale_q10;
            rsp.diag_queue.dropped = (dropped > 255u) ? 255u : (uint8_t)dropped;
            return TP_ERR_NONE;
        }

        case TP_SUB_DIAG_PERSIST:
            rsp.diag_persist.state = (uint8_t)((store_.is_busy() || tp_->motor_params.is_flash_busy() ? 0x01u : 0u) |
                                               (store_.is_loaded() ? 0x02u : 0u) |
                                               (tp_->motor_params.is_loaded() ? 0x04u : 0u) |
                                               // Only a save that actually ran and failed sets the
                                               // failure bit; a node that has never saved is not in
                                               // an error state.
                                               ((store_.save_attempted() && !store_.is_saved()) ? 0x08u : 0u));
            rsp.diag_persist.config_ver = store_.stored_version();
            rsp.diag_persist.cal_id = tp_->motor_params.get_cal_id();
            rsp.diag_persist.config_free = (uint16_t)store_.free_bytes();
            return TP_ERR_NONE;

        case TP_SUB_DIAG_ENCODER:
            rsp.diag_encoder.angle = (uint16_t)(tp_->encoder.get_angle() >> 16);
            rsp.diag_encoder.rotations = tp_->encoder.get_rotations();
            return TP_ERR_NONE;

        case TP_SUB_DIAG_CALIB_RESULT:
            rsp.diag_result.resistance = tp_pack_bf16(tp_->motor_params.get_resistance());
            rsp.diag_result.induct_d = tp_pack_bf16(tp_->motor_params.get_inductance_d());
            rsp.diag_result.induct_q = tp_pack_bf16(tp_->motor_params.get_inductance_q());
            return TP_ERR_NONE;

        case TP_SUB_DIAG_FOC: {
            // The angle the last control pass handed to the current loop and
            // the d/q voltage that loop produced: a live view of the FOC frame.
            const Vec2D_f32 u = tp_->current_driver.voltage_dq_;      // Last commanded d/q voltage [V]
            rsp.diag_foc.angle_el = (uint16_t)tp_->current_driver.get_angle();
            rsp.diag_foc.u_d = tp_pack_bf16(u.flux);
            rsp.diag_foc.u_q = tp_pack_bf16(u.torque);
            return TP_ERR_NONE;
        }

        case TP_SUB_DIAG_ENCODER_LINK: {
            if (encoder_link_ == nullptr) {
                return TP_ERR_NOT_IMPLEMENTED;                            // This board validates no frames
            }
            TpEncoderLinkInfo e;                                          // Snapshot filled by the board
            encoder_link_(e);
            rsp.diag_enc_link.flags = (uint8_t)((e.check_proven ? 0x01u : 0u) | (e.link_down ? 0x02u : 0u) |
                                                ((e.sensor_status & 0x07u) << 2));  // Sensor's own status bits
            rsp.diag_enc_link.relinks = (uint8_t)((e.relinks > 255u) ? 255u : e.relinks);
            rsp.diag_enc_link.frames_bad = (uint16_t)((e.frames_bad > 0xFFFFu) ? 0xFFFFu : e.frames_bad);
            rsp.diag_enc_link.frames_ok = (uint16_t)e.frames_ok;          // Wraps: a liveness counter
            return TP_ERR_NONE;
        }

        case TP_SUB_DIAG_IDENT_FAIL: {
            const MotorCalibration& mc = tp_->current_driver.motor_calib;   // Identification routine
            const float v = mc.last_failure_volts() * 10.0f;                 // 0.1 V units
            const float ia = mc.last_failure_ia() * 20.0f;                   // 0.05 A units
            const float ib = mc.last_failure_ib() * 20.0f;
            rsp.diag_ident.reason = mc.last_failure_code();
            rsp.diag_ident.stage = mc.last_failure_stage();
            rsp.diag_ident.fault = mc.last_failure_fault();
            rsp.diag_ident.volts_x10 = (uint8_t)((v < 0.0f) ? 0.0f : ((v > 255.0f) ? 255.0f : v));
            rsp.diag_ident.ia_x20 = (int8_t)((ia < -127.0f) ? -127.0f : ((ia > 127.0f) ? 127.0f : ia));
            rsp.diag_ident.ib_x20 = (int8_t)((ib < -127.0f) ? -127.0f : ((ib > 127.0f) ? 127.0f : ib));
            return TP_ERR_NONE;
        }

        case TP_SUB_DIAG_RIPPLE: {
            // Keyed: byte 2 of the request names the first of the two bins.
            const auto& map = tp_->ripple_map;                                   // The feed-forward table
            const uint8_t first = (uint8_t)(req.b8[2] & (map.kBins - 1u));       // Wrap into the map
            rsp.diag_ripple.index = first;
            rsp.diag_ripple.flags = (uint8_t)((map.is_enabled() ? 0x01u : 0u) | (map.is_learning() ? 0x02u : 0u));
            rsp.diag_ripple.bin0 = map.bin_ma(first);
            rsp.diag_ripple.bin1 = map.bin_ma((uint16_t)(first + 1u));
            return TP_ERR_NONE;
        }

        default:
            return TP_ERR_UNKNOWN_SUB;
    }
}

inline TpError TpProtocol::read_direct(const TpFrame& req, TpFrame& rsp) {
    const uint32_t addr = tp_unpack_u24(req.direct.addr);      // Windowed address
    volatile uint32_t* p = direct_pointer(addr, false);        // Resolve the window
    if (p == nullptr) {
        return TP_ERR_OUT_OF_RANGE;                             // Unmapped or misaligned
    }
    tp_pack_u24(rsp.direct.addr, addr);
    rsp.direct.value = *p;                                      // One volatile word read
    return TP_ERR_NONE;
}

inline TpError TpProtocol::read_cal_table(const TpFrame& req, TpFrame& rsp) {
    const uint8_t table = (uint8_t)((req.cal_table.select >> 5) & 0x7u);  // Which table
    if (table != TP_TABLE_ENCODER) {
        return TP_ERR_NOT_IMPLEMENTED;                          // Only the angle table exists
    }
    const uint16_t count = tp_->motor_params.cal_table.get_sample_count();  // Samples stored
    rsp.cal_table.select = req.cal_table.select;
    if (req.cal_table.data == TP_CAL_TABLE_SIZE_KEY) {
        rsp.cal_table.sample = count;                           // Size query
        rsp.cal_table.data = 0u;
        return TP_ERR_NONE;
    }
    if (req.cal_table.sample >= count) {
        return TP_ERR_OUT_OF_RANGE;                             // Beyond the stored samples
    }
    rsp.cal_table.sample = req.cal_table.sample;
    rsp.cal_table.data = tp_->motor_params.cal_table.get_sample(req.cal_table.sample);
    return TP_ERR_NONE;
}

inline TpError TpProtocol::read_trace(const TpFrame& req, TpFrame& rsp) {
#ifndef TP_TRACE_DEPTH
    (void)req;
    (void)rsp;
    return TP_ERR_NOT_IMPLEMENTED;                              // No recorder in this image
#else
    const auto& fr = tp_->flight;                               // The black box
    rsp.trace.part = req.trace.part;
    if (req.trace.part == 0u) {
        rsp.trace.index = fr.depth();                           // Ring depth
        rsp.trace.a = fr.count();                               // Valid records
        rsp.trace.b = (uint16_t)((fr.frozen() ? 0x01u : 0u) | ((uint16_t)fr.freeze_fault() << 8));
        return TP_ERR_NONE;
    }
    FlightRecord r;                                             // The requested record
    if (!fr.read(req.trace.index, r)) {
        return TP_ERR_OUT_OF_RANGE;                             // Beyond the valid records
    }
    rsp.trace.index = req.trace.index;
    switch (req.trace.part) {
        case 1u:
            rsp.trace.a = r.angle_el;
            rsp.trace.b = (uint16_t)r.iq_ma;
            return TP_ERR_NONE;
        case 2u:
            rsp.trace.a = (uint16_t)r.id_ma;
            rsp.trace.b = (uint16_t)r.uq_cv;
            return TP_ERR_NONE;
        case 3u:
            rsp.trace.a = (uint16_t)r.ud_cv;
            rsp.trace.b = (uint16_t)r.icmd_ma;
            return TP_ERR_NONE;
        case 4u:
            rsp.trace.a = (uint16_t)r.ia_ma;
            rsp.trace.b = (uint16_t)r.ib_ma;
            return TP_ERR_NONE;
        case 5u:
            rsp.trace.a = (uint16_t)r.ua_cv;
            rsp.trace.b = (uint16_t)r.ub_cv;
            return TP_ERR_NONE;
        case 6u:
            rsp.trace.a = (uint16_t)r.vbus_cv;
            rsp.trace.b = (uint16_t)r.perr_mrev;
            return TP_ERR_NONE;
        default:
            return TP_ERR_BAD_PARAM;                            // No such part
    }
#endif
}

inline void TpProtocol::pack_pulsebeat(TpFrame& f, bool advance) {
    DriveTelemetry t;                                    // Live control snapshot
    tp_->drive.read_telemetry(t);
    const uint8_t seq = (uint8_t)(hb_seq_ & TP_PULSEBEAT_MASK);  // Beat counter
    if (advance) {
        hb_seq_++;                                       // Lets a master spot a lost beat
    }
    f.u64 = 0;                                           // Clean slate
    f.beat.cmd = (uint8_t)(TP_CMD_PULSEBEAT | seq);
    f.beat.queue_ts = tp_beat_queue_ts(t.queue_depth, t.time_scale_q10);
    f.beat.status_fault = (uint8_t)(drive_state() | (tp_fault_group(tp_->fault_byte()) << 4));
    f.beat.warn_mode_err = tp_beat_warn_mode_err(tp_warn_group(warn_flags()), control_mode(), following_error_ratio());
    f.beat.load = (int8_t)tp_pack_usage_i8(load_ratio());
    f.beat.io = tp_->io_inputs_;
}
