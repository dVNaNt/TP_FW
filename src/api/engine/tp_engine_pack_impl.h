#pragma once

/**
 * @file tp_engine_pack_impl.h
 * @brief Snapshotting drive state into the status chunk layouts.
 *
 * One case per status opcode, each filling the matching typed view of the
 * frame union. This is the read half of the protocol in a single place: if a
 * value is visible to a master, it is packed here.
 *
 * The functions are near-pure - they read live state and write a frame - with
 * one exception: pack_heartbeat() advances the heartbeat sequence counter, so
 * a master can detect a dropped heartbeat.
 *
 * Part of TpProtocol, declared in tp_engine.h. Including that header pulls this
 * one in at the end; including this one first pulls tp_engine.h, whose own
 * include of this file is then skipped by #pragma once - so the class is always
 * complete before these definitions are parsed, whichever order is used.
 *
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#include "../tp_engine.h"  // class TpProtocol - the declarations these bodies define

inline bool TpProtocol::pack_chunk(uint8_t opcode, TpFrame& f) {
    if (tp_ == nullptr) {
        return false;                                    // Not bound to a drive
    }
    f.u64 = 0;                                           // Clean slate for every branch
    switch (opcode) {
        case TP_OP_DEVICE_INFO:
            f.device_info.cmd = TP_OP_DEVICE_INFO;
            f.device_info.hw_ver = TP_HW_VERSION;
            f.device_info.fw_ver = TP_FW_VERSION;
            f.device_info.device = TP_DEVICE_CODE;
            return true;

        case TP_OP_DEVICE_UID: {
            f.device_uid.cmd = TP_OP_DEVICE_UID;
            const uint8_t* uid = identity_.bytes();      // 56-bit identity
            for (uint32_t i = 0; i < TP_UID_BYTES; ++i) {
                f.device_uid.uid[i] = uid[i];
            }
            return true;
        }

        case TP_OP_DEVICE_STATUS: {
            const uint32_t up = millis() / 1000u;        // Whole seconds since boot
            f.device_status.cmd = TP_OP_DEVICE_STATUS;
            f.device_status.uptime[0] = (uint8_t)(up);
            f.device_status.uptime[1] = (uint8_t)(up >> 8);
            f.device_status.uptime[2] = (uint8_t)(up >> 16);
            f.device_status.status = (uint32_t)tp_->driver_phy.get_fault() << TP_STATUS_FAULT_SHIFT |
                                     (uint32_t)warn_flags() << TP_STATUS_WARN_SHIFT |
                                     (uint32_t)drive_state() << TP_STATUS_STATE_SHIFT |
                                     (uint32_t)control_mode() << TP_STATUS_MODE_SHIFT;
            return true;
        }

        case TP_OP_MOTOR_STATUS: {
            DriveTelemetry t;                            // Live control snapshot
            tp_->drive.read_telemetry(t);
            const MotorType mt = tp_->motor_params.get_motor_type();
            const PhasePattern pp = tp_->motor_params.get_phase_pattern();
            f.motor_status.cmd = TP_OP_MOTOR_STATUS;
            f.motor_status.mode = t.mode & 0x0Fu;
            f.motor_status.motor_type = (mt == MotorType_STEP) ? 1u : ((mt == MotorType_BLDC) ? 2u : 0u);
            f.motor_status.phase_conn = (pp == PhasePattern_CBA) ? 2u : ((pp == PhasePattern_ACB) ? 3u : 1u);
            f.motor_status.pole_pairs = (uint8_t)((tp_->motor_params.get_pole_pairs() > 255)
                                                      ? 255
                                                      : tp_->motor_params.get_pole_pairs());
            f.motor_status.temp = temp_byte();
            f.motor_status.direction = (tp_->motor_params.get_direction() < 0) ? 1u : 0u;
            f.motor_status.enabled = ((t.flags & DRIVE_TLM_ENABLED) != 0u) ? 1u : 0u;
            f.motor_status.calibrated = tp_->angle_calibrator.is_calibrated() ? 1u : 0u;
            f.motor_status.identified = (tp_->motor_params.get_resistance() > 0.0f) ? 1u : 0u;
            f.motor_status.profile_loaded = tp_->motor_params.is_loaded() ? 1u : 0u;
            return true;
        }

        case TP_OP_ELECTRIC_STATUS: {
            const float p_elec = tp_->driver_phy.power.get_power_w();   // Bus-side power [W]
            f.electric_status.cmd = TP_OP_ELECTRIC_STATUS;
            f.electric_status.efficiency = efficiency_byte(p_elec);
            f.electric_status.power = tp_pack_bf16(p_elec);
            f.electric_status.supply = tp_pack_bf16(tp_->driver_phy.supply.get_voltage());
            f.electric_status.current = tp_pack_bf16(tp_->last_current_a_);
            return true;
        }

        case TP_OP_MOTION_STATUS_1: {
            DriveTelemetry t;                            // Live control snapshot
            tp_->drive.read_telemetry(t);
            f.motion_status1.cmd = TP_OP_MOTION_STATUS_1;
            f.motion_status1.motion_mode = t.motion_mode;
            tp_pack_i48(f.motion_status1.position, (int64_t)t.position_q16);
            return true;
        }

        case TP_OP_MOTION_STATUS_2: {
            DriveTelemetry t;                            // Live control snapshot
            tp_->drive.read_telemetry(t);
            // A frozen observer must not be reported as motion: with the
            // bridge released the rotor is coasting or stopped, and the
            // last value the loop computed says nothing about either.
            const bool live = (t.flags & DRIVE_TLM_LOOP_LIVE) != 0u;
            const float vel = live ? tp_->position_controller.getEstimatedVelocityRevS() : 0.0f;
            const uint32_t v24 = tp_pack_f24(vel);       // float24 velocity
            f.motion_status2.cmd = TP_OP_MOTION_STATUS_2;
            f.motion_status2.velocity[0] = (uint8_t)(v24);
            f.motion_status2.velocity[1] = (uint8_t)(v24 >> 8);
            f.motion_status2.velocity[2] = (uint8_t)(v24 >> 16);
            f.motion_status2.accel =
                tp_pack_bf16(live ? tp_->position_controller.getEstimatedAccelerationRevS2() : 0.0f);
            f.motion_status2.torque = tp_pack_bf16(tp_->last_current_a_);
            return true;
        }

        case TP_OP_MOTION_STATUS_3: {
            DriveTelemetry t;                            // Live control snapshot
            tp_->drive.read_telemetry(t);
            const int32_t perr = t.target_pos_q16 - t.position_q16;      // Following error, Q16.16
            // Same staleness rule as MOTION_STATUS_2: report no velocity
            // error rather than a frozen one when the loop is not running.
            const bool live = (t.flags & DRIVE_TLM_LOOP_LIVE) != 0u;
            const float verr = live
                                   ? (tp_->position_controller.getEstimatedVelocityRevS() - t.target_vel_rps)
                                   : 0.0f;
            f.motion_status3.cmd = TP_OP_MOTION_STATUS_3;
            tp_pack_i24(f.motion_status3.pos_err, perr);
            f.motion_status3.vel_err = tp_pack_bf16(verr);
            f.motion_status3.queue = (t.queue_depth > 15u) ? 15u : t.queue_depth;
            f.motion_status3.time_scale = tp_pack_timescale(t.time_scale_q10);
            return true;
        }

        case TP_OP_MOTOR_PROPERTIES:
            f.motor_props.cmd = TP_OP_MOTOR_PROPERTIES;
            f.motor_props.temp_lim = tp_->motor_params.get_max_temp();
            f.motor_props.resistance = tp_pack_bf16(tp_->motor_params.get_resistance());
            f.motor_props.inductance = tp_pack_bf16(tp_->motor_params.get_effective_inductance());
            f.motor_props.torque = tp_pack_bf16(tp_->motor_params.get_torque_constant());
            return true;

        case TP_OP_ENCODER_STATUS:
            f.encoder_status.cmd = TP_OP_ENCODER_STATUS;
            f.encoder_status.health = 0u;                // Sensor supervision is not implemented yet
            f.encoder_status.angle = (uint16_t)(tp_->encoder.get_angle() >> 16);
            f.encoder_status.rotations = tp_->encoder.get_rotations();
            return true;

        case TP_OP_DRIVER_STATUS:
            f.driver_status.cmd = TP_OP_DRIVER_STATUS;
            f.driver_status.state = drive_state();
            f.driver_status.mode = control_mode();
            f.driver_status.fault = tp_->driver_phy.get_fault();
            f.driver_status.warn = warn_flags();
            f.driver_status.op_status = (uint8_t)tp_->current_driver.get_driver_status();
            f.driver_status.motor_status = (uint8_t)tp_->current_driver.get_motor_status();
            return true;

        case TP_OP_CALIBRATION_STATUS: {
            const MotorStatus ms = tp_->current_driver.get_motor_status();
            f.calib_status.cmd = TP_OP_CALIBRATION_STATUS;
            // 0xFF marks "idle": no routine is running, the drive is calibrated.
            f.calib_status.kind = (tp_->motor_status == MotorStatus::Success) ? 0xFFu
                                                                              : (uint8_t)TP_CALIB_FULL;
            f.calib_status.stage = (uint8_t)ms;
            // The identification walks a known sequence of MotorStatus
            // values, so the stage number doubles as a coarse progress bar.
            f.calib_status.progress = (uint8_t)((ms == MotorStatus::Success) ? 255u
                                                                             : (((uint32_t)ms * 255u) / 12u));
            f.calib_status.flags = (uint8_t)((tp_->angle_calibrator.is_calibrated() ? 0x01u : 0u) |
                                             (tp_->angle_calibrator.is_settling() ? 0x02u : 0u) |
                                             (tp_->angle_calibrator.is_persisting() ? 0x04u : 0u) |
                                             ((ms == MotorStatus::CalibError) ? 0x08u : 0u));
            f.calib_status.cal_id = tp_->motor_params.get_cal_id();
            f.calib_status.pole_pairs = (uint16_t)tp_->motor_params.get_pole_pairs();
            return true;
        }

        case TP_OP_COMM_STATUS:
            f.comm_status.cmd = TP_OP_COMM_STATUS;
            f.comm_status.bus_state = bus_flags_;
            f.comm_status.tx_errors = bus_tx_err_;
            f.comm_status.rx_errors = bus_rx_err_;
            f.comm_status.dropped = bus_dropped_;
            f.comm_status.rejected = rejected_;
            return true;

        case TP_OP_POWER_STATUS: {
            const float temp = tp_->driver_phy.temperature.get_temperature_c();
            const float head = config_.ot_limit_c - temp;               // Degrees left to the trip
            f.power_status.cmd = TP_OP_POWER_STATUS;
            f.power_status.load = load_byte();
            f.power_status.bus_curr = tp_pack_bf16(tp_->driver_phy.power.get_supply_current_usage_a());
            f.power_status.bus_power = tp_pack_bf16(tp_->driver_phy.power.get_power_w());
            f.power_status.headroom = (uint8_t)((head < 0.0f) ? 0.0f : ((head > 255.0f) ? 255.0f : head));
            return true;
        }

        case TP_OP_HEARTBEAT:
            pack_heartbeat(f);
            return true;

        case TP_OP_MOTION_QUEUE_STAT: {
            DriveTelemetry t;                            // Live control snapshot
            tp_->drive.read_telemetry(t);
            f.queue_stat.cmd = TP_OP_MOTION_QUEUE_STAT;
            f.queue_stat.state = (uint8_t)(((t.flags & DRIVE_TLM_ENABLED) != 0u ? 0x01u : 0u) |
                                           ((t.flags & DRIVE_TLM_PAUSED) != 0u ? 0x02u : 0u) |
                                           ((t.flags & DRIVE_TLM_HALTED) != 0u ? 0x04u : 0u) |
                                           ((t.flags & DRIVE_TLM_STARVING) != 0u ? 0x08u : 0u));
            f.queue_stat.queued = t.queue_depth;
            f.queue_stat.free_slots = t.queue_free;
            f.queue_stat.time_scale = t.time_scale_q10;
            f.queue_stat.dropped = t.dropped_segments;
            return true;
        }

        case TP_OP_PARAM_STATUS:
            f.param_status.cmd = TP_OP_PARAM_STATUS;
            f.param_status.state = (uint8_t)((store_.is_busy() || tp_->motor_params.is_flash_busy() ? 0x01u : 0u) |
                                             (store_.is_loaded() ? 0x02u : 0u) |
                                             (tp_->motor_params.is_loaded() ? 0x04u : 0u) |
                                             // Only a save that actually ran and failed
                                             // sets the failure bit; a node that has simply
                                             // never saved is not in an error state.
                                             ((store_.save_attempted() && !store_.is_saved())
                                                  ? 0x08u : 0u));
            f.param_status.config_ver = store_.stored_version();
            f.param_status.motor_cal_id = tp_->motor_params.get_cal_id();
            f.param_status.config_free = (uint16_t)store_.free_bytes();
            f.param_status.user_free = 0u;               // The motor engine does not expose it
            return true;

        case TP_OP_CALIB_RESULT:
            f.calib_result.cmd = TP_OP_CALIB_RESULT;
            f.calib_result.flags =
                (uint8_t)(((tp_->motor_params.get_resistance() > 0.0f) ? 0x01u : 0u) |
                          (tp_->angle_calibrator.is_calibrated() ? 0x02u : 0u) |
                          ((tp_->current_driver.get_motor_status() == MotorStatus::CalibError) ? 0x04u : 0u));
            f.calib_result.resistance = tp_pack_bf16(tp_->motor_params.get_resistance());
            f.calib_result.induct_d = tp_pack_bf16(tp_->motor_params.get_inductance_d());
            f.calib_result.induct_q = tp_pack_bf16(tp_->motor_params.get_inductance_q());
            return true;

        default:
            return false;                                // Not a status chunk
    }
}

inline void TpProtocol::pack_heartbeat(TpFrame& f) {
    DriveTelemetry t;                                    // Live control snapshot
    tp_->drive.read_telemetry(t);
    f.u64 = 0;                                           // Clean slate
    f.heartbeat.cmd = TP_OP_HEARTBEAT;
    f.heartbeat.state = drive_state();
    f.heartbeat.mode = control_mode();
    f.heartbeat.fault = tp_->driver_phy.get_fault();
    f.heartbeat.warn = warn_flags();
    f.heartbeat.supply = tp_pack_volts_q2(tp_->driver_phy.supply.get_voltage());
    f.heartbeat.temp = temp_byte();
    f.heartbeat.load = load_byte();
    f.heartbeat.seq = (uint8_t)(hb_seq_++ & 0x0Fu);       // Lets a master spot a lost heartbeat
    f.heartbeat.queue = (t.queue_depth > 15u) ? 15u : t.queue_depth;
}
