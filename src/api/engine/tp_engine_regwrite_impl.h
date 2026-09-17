#pragma once

/**
 * @file tp_engine_regwrite_impl.h
 * @brief Applying every writable register.
 *
 * A register write is looked up in the map, checked against its access rules
 * (writable, output stage must be off), applied field by field, pushed into
 * the hardware when a cached threshold changed, and answered with the read
 * form of the same register - so a clamped or rounded value is visible in the
 * very reply that acknowledges the write.
 *
 * The convention for "leave unchanged" is the natural absent value of each
 * field: NaN for a bfloat16 / float24, zero for a byte that is a magic value
 * or a physical quantity that can never be zero. Fields that are plain
 * values with a meaningful zero (the burst step, the disbalance filter, an
 * axis offset) are always applied.
 *
 * Part of TpProtocol, declared in tp_engine.h.
 *
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#include "../tp_engine.h"  // class TpProtocol - the declarations these bodies define

/** @brief A bfloat16 field that carries a usable finite value, decoded. */
static inline bool tp_take_bf16(uint16_t raw, float& out) {
    if (!tp_bf16_is_finite(raw)) {
        return false;                                    // NaN / Inf: leave unchanged
    }
    out = tp_unpack_bf16(raw);
    return true;
}

/** @brief A bfloat16 field that carries a finite value strictly above zero, decoded. */
static inline bool tp_take_bf16_pos(uint16_t raw, float& out) {
    float v;                                             // Decoded value
    if (!tp_take_bf16(raw, v) || !(v > 0.0f)) {
        return false;                                    // Absent, NaN or non-positive
    }
    out = v;
    return true;
}

inline TpError TpProtocol::write_register(const TpFrame& f, TpFrame& rsp) {
    if (tp_ == nullptr) {
        return TP_ERR_WRONG_STATE;                       // Not bound to a drive
    }
    const uint8_t reg = tp_cmd_register(f.b8[0]);        // Address without the write bit
    const TpRegDesc* d = tp_find_register(reg);          // Access rules
    if (d == nullptr) {
        return TP_ERR_UNKNOWN_CMD;                       // Not a register
    }
    if ((d->flags & TP_RF_WRITE) == 0u) {
        return TP_ERR_READ_ONLY;                         // Read-only register
    }
    if (((d->flags & TP_RF_DISABLED) != 0u) && is_enabled()) {
        // Retuning a protection threshold or the machine's identity under
        // load moves a trip point beneath a live current: refuse it.
        return TP_ERR_WRONG_STATE;
    }
    const uint8_t sub = ((d->flags & TP_RF_SUB) != 0u) ? f.b8[1] : 0u;  // Record selector
    bool reapply = false;                                // Push the shadow into the hardware afterwards
    TpError e = TP_ERR_NONE;                             // Outcome
    float v;                                             // Decoded scratch value
    switch (reg) {
        case TP_REG_DIRECT:
            e = write_direct(f);
            break;

        case TP_REG_AXIS:
            e = write_axis(f);
            break;

        case TP_REG_TELEMETRY: {
            if (sub >= TP_TELEMETRY_SLOTS) {
                return TP_ERR_UNKNOWN_SUB;               // No such slot
            }
            const uint8_t target = f.telemetry.reg;      // Register the slot should emit
            if (target != 0u) {
                const TpRegDesc* t = tp_find_register(target);  // Must be a readable register
                if ((target > 0x7Fu) || (t == nullptr) || ((t->flags & TP_RF_READ) == 0u) ||
                    (target == TP_REG_BATCH_READ) || (target == TP_REG_RANGE_READ)) {
                    return TP_ERR_BAD_PARAM;
                }
            }
            TpTelemetrySlot& s = config_.telemetry[sub]; // Slot being written
            s.reg = target;                              // What to emit (0 frees the slot)
            s.flags = f.telemetry.flags;                 // Timer / sync
            s.period_ms = f.telemetry.period_ms;         // How often
            tlm_next_ms_[sub] = millis();                // Emit on the next pass
            break;
        }

        case TP_REG_BUS:
            e = write_bus(f);
            break;

        case TP_REG_DISCOVERY_ASSIGN:
        case TP_REG_DISCOVERY_SEARCH:
            return TP_ERR_WRONG_STATE;                   // Dispatched before the register path

        case TP_REG_SYSTEM_CONTROL:
            e = write_system_control(f);
            break;

        case TP_REG_MOTOR_PROPS_1:
            e = write_motor_props_1(f);
            reapply = true;                              // The envelope may have moved
            break;

        case TP_REG_MOTOR_PROPS_2:
            e = write_motor_props_2(f);
            break;

        case TP_REG_POWER_LIMITS:
            // Zero, NaN and Inf are ignored field by field, as the specification says.
            config_.pt_overdrive_s = f.power_limits.overdrive;                    // 0 disables overdrive
            if (tp_take_bf16(f.power_limits.baseline, v) && (v >= 0.0f)) {
                config_.pt_baseline_w = v;
            }
            if (tp_take_bf16_pos(f.power_limits.peak, v) && (v <= 1000.0f)) {
                config_.op_limit_w = v;                                           // The overpower trip
                reapply = true;
            }
            if (tp_take_bf16(f.power_limits.capacity, v) && (v >= 0.0f)) {
                config_.pt_capacity_j = v;
            }
            break;

        case TP_REG_MOTOR_THERMAL:
            if (f.motor_thermal.temp_base != 0u) {
                config_.motor_temp_base_c = f.motor_thermal.temp_base;
            }
            if (f.motor_thermal.temp_limit != 0u) {
                if (f.motor_thermal.temp_limit > 200u) {
                    return TP_ERR_OUT_OF_RANGE;          // Beyond any winding insulation class
                }
                tp_->motor_params.set_max_temp(f.motor_thermal.temp_limit);
            }
            if (tp_take_bf16(f.motor_thermal.derating, v) && (v >= 0.0f)) {
                config_.motor_derating = v;
            }
            if (tp_take_bf16(f.motor_thermal.rth, v) && (v >= 0.0f)) {
                config_.motor_rth = v;
            }
            break;

        case TP_REG_DRIVER_THERMAL:
            if (f.driver_thermal.ot_warn != 0u) {
                config_.ot_warn_c = (float)f.driver_thermal.ot_warn;
            }
            if (f.driver_thermal.ot_fault != 0u) {
                if (f.driver_thermal.ot_fault > 150u) {
                    return TP_ERR_OUT_OF_RANGE;          // Silicon absolute maximum
                }
                config_.ot_limit_c = (float)f.driver_thermal.ot_fault;
                reapply = true;
            }
            if (f.driver_thermal.hysteresis != 0u) {
                if (f.driver_thermal.hysteresis > 50u) {
                    return TP_ERR_OUT_OF_RANGE;
                }
                config_.ot_hyst_c = (float)f.driver_thermal.hysteresis;
                reapply = true;
            }
            break;

        case TP_REG_SUPPLY_PROTECTION:
            if (tp_take_bf16(f.supply_prot.hysteresis, v) && (v >= 0.0f)) {
                config_.supply_hyst_v = v;
            }
            if (f.supply_prot.recovery != 0u) {
                config_.supply_recovery = f.supply_prot.recovery;
            }
            if (tp_take_bf16_pos(f.supply_prot.ov, v)) {
                if (v > 60.0f) {
                    return TP_ERR_OUT_OF_RANGE;          // Bus capacitor rating
                }
                config_.ov_limit_v = v;
                reapply = true;
            }
            if (tp_take_bf16_pos(f.supply_prot.uv, v)) {
                if (v > 60.0f) {
                    return TP_ERR_OUT_OF_RANGE;
                }
                config_.uv_limit_v = v;
                reapply = true;
            }
            break;

        case TP_REG_CURRENT_PROTECTION:
            config_.oc_disbalance = f.current_prot.disbalance;               // Plain values, always applied
            config_.oc_alpha = f.current_prot.alpha;
            if (f.current_prot.recovery != 0u) {
                config_.oc_recovery = f.current_prot.recovery;
            }
            if (tp_take_bf16_pos(f.current_prot.peak, v)) {
                if (v > 20.0f) {
                    return TP_ERR_OUT_OF_RANGE;          // Beyond any CLN17-class stage
                }
                config_.peak_current_a = v;
                if (config_.current_limit_a > v) {
                    config_.current_limit_a = v;         // The envelope can never exceed the peak
                }
                reapply = true;
            }
            if (tp_take_bf16_pos(f.current_prot.oc, v)) {
                if (v > 20.0f) {
                    return TP_ERR_OUT_OF_RANGE;
                }
                // Phases A and B carry one coil each; phase C is the return
                // of both on a stepper and sees their sum, so its threshold
                // keeps the board's 6.2 / 4.0 ratio above the two coil trips.
                config_.oc_limit_a = v;
                config_.oc_limit_b = v;
                config_.oc_limit_c = v * 1.55f;
                reapply = true;
            }
            break;

        case TP_REG_MOTION_LIMITS:
        case TP_REG_ERROR_LIMITS: {
            const bool fault_set = (reg == TP_REG_ERROR_LIMITS);   // Which envelope
            const uint32_t vraw = tp_unpack_u24(f.limits.velocity);  // float24 velocity
            if (tp_f24_is_finite(vraw)) {
                const float vel = tp_unpack_f24(vraw);
                if (vel > 0.0f) {
                    if (vel > 1000.0f) {
                        return TP_ERR_OUT_OF_RANGE;
                    }
                    (fault_set ? config_.err_vel_rps : config_.vel_limit_rps) = vel;
                }
            }
            if (tp_take_bf16_pos(f.limits.accel, v)) {
                (fault_set ? config_.err_accel : config_.accel_limit) = v;
            }
            if (tp_take_bf16_pos(f.limits.jerk, v)) {
                (fault_set ? config_.err_jerk : config_.jerk_limit) = v;
            }
            reapply = !fault_set;                        // The live envelope goes through the mailbox
            break;
        }

        case TP_REG_WATCHDOG:
            e = write_watchdog(f);
            break;

        default:
            return TP_ERR_READ_ONLY;                     // Everything else has no write path
    }
    if (e != TP_ERR_NONE) {
        return e;
    }
    if (reapply) {
        tp_apply_config(*tp_, config_);                  // Push the change into hardware
    }
    // Answer with the values the drive holds now. Fields routed through the
    // control mailbox (mode, enable) take effect on the next control pass, so
    // their read-back can still show the previous value; read again to confirm.
    return read_register(reg, sub, f, rsp);
}

inline TpError TpProtocol::write_axis(const TpFrame& f) {
    switch (f.b8[1]) {
        case TP_SUB_AXIS_CONTROL:
            config_.axis_id = f.axis.axis_id;
            config_.axis_inverted = (f.axis.flags & 0x01u) ? 1u : 0u;
            config_.axis_offset_q16 = f.axis.offset_q16;
            return TP_ERR_NONE;
        case TP_SUB_LED:
            if (f.led.mode > 2u) {
                return TP_ERR_BAD_PARAM;                 // Undefined LED mode
            }
            config_.led_mode = f.led.mode;
            config_.led_on_off = f.led.on_off;
            config_.led_id = f.led.led_id;
            config_.led_r = f.led.r;
            config_.led_g = f.led.g;
            config_.led_b = f.led.b;
            if (config_.led_mode != 2u) {
                tp_->led.set().led(config_.led_r, config_.led_g, config_.led_b);  // Applied at once
            }
            return TP_ERR_NONE;
        case TP_SUB_GEARBOX: {
            float v;                                     // Decoded backlash
            if (tp_take_bf16(f.gearbox.backlash, v) && (v >= 0.0f)) {
                config_.gear_backlash = v;               // Stored; compensation is not implemented yet
            }
            if ((f.gearbox.ratio_div == 0u) || (f.gearbox.ratio_div > 32767u)) {
                return TP_ERR_BAD_PARAM;                 // Division by zero, or beyond the int16 API
            }
            config_.gear_ratio_mul = f.gearbox.ratio_mul;
            config_.gear_ratio_div = f.gearbox.ratio_div;
            // Live at once, like the LED above: the very next read of any
            // position or velocity is reported in the new output-shaft units.
            tp_->gearbox.set_ratio(config_.gear_ratio_mul, static_cast<int16_t>(config_.gear_ratio_div));
            return TP_ERR_NONE;
        }
        case TP_SUB_IO_MAP: {
            const uint8_t slot = tp_io_function_slot(f.io_map.function);  // Storage slot
            if (slot >= TP_IO_FUNCTIONS) {
                return TP_ERR_BAD_PARAM;                 // Not a defined function
            }
            config_.io_map[slot].on_pin = f.io_map.on_pin;
            config_.io_map[slot].off_pin = f.io_map.off_pin;
            config_.io_map[slot].flags = (uint8_t)(f.io_map.flags & 0x07u);
            return TP_ERR_NONE;
        }
        case TP_SUB_ENCODER_1:
        case TP_SUB_ENCODER_2: {
            TpEncoderCfg& e = config_.encoder[f.b8[1] - TP_SUB_ENCODER_1];  // Declared encoder
            e.type_iface = f.encoder_cfg.type_iface;
            e.flags = (uint8_t)(f.encoder_cfg.flags & 0x03u);
            e.ppr = f.encoder_cfg.ppr;
            return TP_ERR_NONE;
        }
        case TP_SUB_HOMING_QUICK: {
            float v;                                     // Decoded scratch value
            config_.homing_flags = (uint16_t)(f.homing_quick.flags & 0x007Fu);
            if (tp_take_bf16(f.homing_quick.velocity, v) && (v >= 0.0f)) {
                config_.homing_quick_vel = v;
            }
            if (tp_take_bf16(f.homing_quick.torque, v) && (v >= 0.0f)) {
                config_.homing_quick_torque = v;
            }
            return TP_ERR_NONE;
        }
        case TP_SUB_HOMING_FINE: {
            float v;                                     // Decoded scratch value
            if (tp_take_bf16(f.homing_fine.retract, v) && (v >= 0.0f)) {
                config_.homing_fine_retract = v;
            }
            if (tp_take_bf16(f.homing_fine.velocity, v) && (v >= 0.0f)) {
                config_.homing_fine_vel = v;
            }
            if (tp_take_bf16(f.homing_fine.torque, v) && (v >= 0.0f)) {
                config_.homing_fine_torque = v;
            }
            return TP_ERR_NONE;
        }
        case TP_SUB_REF_CW:
        case TP_SUB_REF_CCW: {
            float v;                                     // Decoded offset
            if (tp_take_bf16(f.ref_limit.offset, v)) {
                if ((v < 0.0f) || (v >= 64.0f)) {
                    return TP_ERR_OUT_OF_RANGE;          // Must be less than 64 revolutions
                }
                (f.b8[1] == TP_SUB_REF_CW ? config_.ref_offset_cw : config_.ref_offset_ccw) = v;
            }
            (f.b8[1] == TP_SUB_REF_CW ? config_.ref_ang_cw : config_.ref_ang_ccw) = f.ref_limit.ref_angle;
            return TP_ERR_NONE;
        }
        case TP_SUB_POS_ZERO:
            config_.ref_ang_zero = f.pos_zero.ref_angle;
            return TP_ERR_NONE;
        case TP_SUB_ZERO_OFFSET_CW:
            config_.zero_offset_cw_q32 = tp_unpack_i48(f.pos_offset.offset);
            return TP_ERR_NONE;
        case TP_SUB_ZERO_OFFSET_CCW:
            config_.zero_offset_ccw_q32 = tp_unpack_i48(f.pos_offset.offset);
            return TP_ERR_NONE;
        case TP_SUB_FIELD_CW:
            config_.field_cw_q32 = tp_unpack_i48(f.pos_offset.offset);
            return TP_ERR_NONE;
        case TP_SUB_FIELD_CCW:
            config_.field_ccw_q32 = tp_unpack_i48(f.pos_offset.offset);
            return TP_ERR_NONE;
        case TP_SUB_INTERNAL_OFFSET:
            config_.internal_offset_q32 = tp_unpack_i48(f.pos_offset.offset);
            return TP_ERR_NONE;
        default:
            return TP_ERR_UNKNOWN_SUB;
    }
}

inline TpError TpProtocol::write_system_control(const TpFrame& f) {
    // Faults are cleared first so an enable in the same frame is not refused
    // for the very fault the frame asked to clear.
    switch (f.system.action) {
        case 0u:
            break;                                       // No action
        case TP_SYS_FAULT_RESET:
            clear_faults();
            break;
        case TP_SYS_REBOOT:
            arm_reboot(0u);
            break;
        case TP_SYS_REBOOT_DFU:
            arm_reboot(1u);
            break;
        case TP_SYS_RIPPLE_LEARN:
            tp_->ripple_map.post(RippleMap<6>::Learn);   // The control ISR starts averaging
            break;
        case TP_SYS_RIPPLE_COMMIT:
            tp_->ripple_map.post(RippleMap<6>::Commit);  // ... folds the residual in and enables the map
            break;
        case TP_SYS_RIPPLE_CLEAR:
            tp_->ripple_map.post(RippleMap<6>::Clear);   // ... forgets the map
            break;
        default:
            return TP_ERR_BAD_PARAM;                     // Not a defined action
    }
    switch (f.system.enable) {
        case 0u:
            break;                                       // No change
        case TP_SYS_ENABLE: {
            const TpError e = set_enable(true);
            if (e != TP_ERR_NONE) {
                return e;
            }
            break;
        }
        case TP_SYS_DISABLE:
            (void)set_enable(false);
            break;
        default:
            return TP_ERR_BAD_PARAM;                     // The two magic values guard against stray writes
    }
    config_.burst_step = f.system.burst_step;            // Plain value: 0 selects the time-tagged stream
    if (f.system.calibration != 0u) {
        return start_calibration(f.system.calibration);
    }
    return TP_ERR_NONE;
}

inline TpError TpProtocol::write_motor_props_1(const TpFrame& f) {
    const uint8_t mode = (uint8_t)(f.motor1.mode_type & 0x0Fu);        // Requested control mode
    const uint8_t type = (uint8_t)(f.motor1.mode_type >> 4);           // Motor type code
    const uint8_t conn = (uint8_t)(f.motor1.conn_dir_poles & 0x3u);    // Phase connection code
    const uint8_t dir2 = (uint8_t)((f.motor1.conn_dir_poles >> 2) & 0x3u);  // 2-bit signed direction
    const uint16_t poles = (uint16_t)(f.motor1.conn_dir_poles >> 4);   // Pole pairs
    if (mode > TP_MODE_TONE) {
        return TP_ERR_BAD_PARAM;                                       // Undefined control mode
    }
    if (type == 3u) {
        return TP_ERR_BAD_PARAM;                                       // "Error" is not assignable
    }
    // The machine identity feeds the FOC angle mapping; changing it under a
    // live loop would command an arbitrary electrical angle. Only a field
    // that DIFFERS from the current identity counts as a change: a master
    // doing a read-modify-write to touch the mode sends the identity back
    // unchanged, and refusing that would make the mode unreachable on an
    // enabled drive.
    const MotorType cur_type = tp_->motor_params.get_motor_type();           // Current machine type
    const PhasePattern cur_pp = tp_->motor_params.get_phase_pattern();       // Current wiring
    const uint8_t cur_type_code = (cur_type == MotorType_STEP) ? 1u : ((cur_type == MotorType_BLDC) ? 2u : 0u);
    const uint8_t cur_conn = (cur_pp == PhasePattern_CBA) ? 2u : ((cur_pp == PhasePattern_ACB) ? 3u : 1u);
    const int16_t cur_dir = tp_->motor_params.get_direction();               // Current direction
    const int32_t cur_poles = tp_->motor_params.get_pole_pairs();            // Current pole pairs
    const bool dir_write = ((dir2 == 0x1u) && (cur_dir != 1)) || ((dir2 == 0x3u) && (cur_dir != -1));
    const bool identity_write = ((type != 0u) && (type != cur_type_code)) || ((conn != 0u) && (conn != cur_conn)) ||
                                dir_write || ((poles != 0u) && ((int32_t)poles != cur_poles));
    if (identity_write && (is_enabled() || tp_->current_driver.is_identifying())) {
        return TP_ERR_WRONG_STATE;
    }
    if (type == 1u) {
        tp_->motor_params.set_motor_type(MotorType_STEP);
    } else if (type == 2u) {
        tp_->motor_params.set_motor_type(MotorType_BLDC);
    }
    switch (conn) {
        case 1u: tp_->motor_params.set_phase_pattern(PhasePattern_ABC); break;
        case 2u: tp_->motor_params.set_phase_pattern(PhasePattern_CBA); break;
        case 3u: tp_->motor_params.set_phase_pattern(PhasePattern_ACB); break;
        default: break;                                                // 0 = not connected: unchanged
    }
    if (dir2 == 0x1u) {
        tp_->motor_params.set_direction(1);                            // +1: CCW
    } else if (dir2 == 0x3u) {
        tp_->motor_params.set_direction(-1);                           // -1: CW
    }
    if (poles != 0u) {
        tp_->motor_params.set_pole_pairs((int32_t)poles);
    }
    float v;                                                           // Decoded scratch value
    if (tp_take_bf16_pos(f.motor1.max_voltage, v)) {
        tp_->motor_params.set_max_voltage(v);
    }
    if (tp_take_bf16_pos(f.motor1.max_current, v)) {
        if (v > 20.0f) {
            return TP_ERR_OUT_OF_RANGE;                                // Beyond any CLN17-class stage
        }
        config_.current_limit_a = (v > config_.peak_current_a) ? config_.peak_current_a : v;
    }
    if (mode != control_mode()) {
        if (!tp_->drive.post_u32(DriveReq::SetMode, mode)) {
            return TP_ERR_BUSY;                                        // Mailbox full
        }
    }
    return TP_ERR_NONE;
}

inline TpError TpProtocol::write_motor_props_2(const TpFrame& f) {
    if (f.motor2.temp != 0u) {
        if (f.motor2.temp > 200u) {
            return TP_ERR_OUT_OF_RANGE;
        }
        tp_->motor_params.set_max_temp(f.motor2.temp);
    }
    float r = 0.0f, l = 0.0f, kt = 0.0f;                               // Decoded constants
    // A value that round-trips to the bfloat16 the drive already reports is
    // not a change: a read-modify-write that touches the thermal rating alone
    // must not rebuild the current loop, nor be refused for it.
    const bool have_r = tp_take_bf16_pos(f.motor2.resistance, r) &&
                        (f.motor2.resistance != tp_pack_bf16(tp_->motor_params.get_resistance()));
    const bool have_l = tp_take_bf16_pos(f.motor2.inductance, l) &&
                        (f.motor2.inductance != tp_pack_bf16(tp_->motor_params.get_effective_inductance()));
    const bool have_kt = tp_take_bf16_pos(f.motor2.kt, kt);
    if (have_r || have_l) {
        // The loop is tuned from R and L, so the edit is applied to the live
        // controller now rather than waiting for the next identification.
        // Rebuilding it under an identification or an angle sweep would hand
        // a control pass a half-built object.
        if (tp_->current_driver.is_identifying() || !tp_->angle_calibrator.is_calibrated()) {
            return TP_ERR_WRONG_STATE;
        }
        if (have_r) {
            tp_->motor_params.set_resistance(r);
        }
        if (have_l) {
            tp_->motor_params.set_inductance_d(l);                     // Symmetric machine assumption
            tp_->motor_params.set_inductance_q(l);
        }
        if (!tp_->current_driver.retune()) {
            return TP_ERR_OUT_OF_RANGE;                                // The loop refused the constants
        }
    }
    if (have_kt) {
        tp_->motor_params.set_torque_constant(kt);
    }
    return TP_ERR_NONE;
}

inline TpError TpProtocol::write_bus(const TpFrame& f) {
    const uint8_t type = (uint8_t)(f.bus.type_comm & 0x0Fu);          // Frame format
    const uint8_t comm = (uint8_t)(f.bus.type_comm >> 4);              // Response policy
    if ((type > TP_BUS_FD_NOBRS) || (comm > TP_COMM_EVERY)) {
        return TP_ERR_BAD_PARAM;                                       // Undefined selector
    }
    if (f.bus.baud1 != TP_BAUD_KEEP) {
        const uint16_t kbps = tp_baud_kbps(f.bus.baud1);               // Arbitration rate
        if ((kbps == 0u) || (kbps > 1000u)) {
            return TP_ERR_BAD_PARAM;                                   // Arbitration tops out at 1 Mbit/s
        }
        config_.arb_kbps = kbps;                                       // Takes effect at the next boot
    }
    if (f.bus.baud2 != TP_BAUD_KEEP) {
        const uint16_t kbps = tp_baud_kbps(f.bus.baud2);               // Data-phase rate
        if (kbps == 0u) {
            return TP_ERR_BAD_PARAM;
        }
        config_.data_kbps = kbps;                                      // Takes effect at the next boot
    }
    config_.bus_type = type;                                           // Takes effect at the next boot
    config_.comm_mode = comm;                                          // Takes effect at once
    if (f.bus.sync != 0u) {
        // Re-base the node clock on the master's: from here on the burst
        // stream's time tags and this node's agree, and sync-triggered
        // telemetry goes out now.
        sync_offset_us_ = (int32_t)(f.bus.sync * 50u) - (int32_t)micros();
        on_sync(millis());
    }
    return TP_ERR_NONE;
}

inline TpError TpProtocol::write_watchdog(const TpFrame& f) {
    const uint8_t retries = (uint8_t)(f.watchdog.retry & 0x7Fu);       // FaultBeat retries
    if (retries != 127u) {
        config_.hb_retry = retries;                                    // 127 = leave unchanged
    }
    config_.hb_retry_force = (f.watchdog.retry & 0x80u) ? 1u : 0u;
    float v;                                                           // Decoded scratch value
    if (tp_take_bf16(f.watchdog.period, v)) {
        if ((v < 0.0f) || (v > 60.0f)) {
            return TP_ERR_OUT_OF_RANGE;                                // A minute is already meaningless
        }
        config_.heartbeat_s = v;                                       // 0 disables the beat
        hb_next_ms_ = millis();                                        // Re-phase on the new period
    }
    if (tp_take_bf16(f.watchdog.delay, v) && (v >= 0.0f) && (v <= 60.0f)) {
        config_.heartbeat_delay_s = v;
    }
    if (tp_take_bf16(f.watchdog.timeout, v)) {
        if ((v < 0.0f) || (v > 60.0f)) {
            return TP_ERR_OUT_OF_RANGE;
        }
        config_.watchdog_s = v;                                        // 0 disables the watchdog
        tp_->drive.post_u32(DriveReq::ArmWatchdog, tp_watchdog_ticks(*tp_, v));
    }
    return TP_ERR_NONE;
}

inline volatile uint32_t* TpProtocol::direct_pointer(uint32_t addr, bool for_write) const {
    const uint8_t window = (uint8_t)((addr >> 20) & 0x0Fu);            // Memory window
    const uint32_t off = addr & 0x000FFFFFu;                           // Byte offset inside it
    if ((off & 3u) != 0u) {
        return nullptr;                                                // Words only
    }
    uintptr_t base = 0;                                                // Window base address
    uint32_t size = 0x100000u;                                         // Window size (default: full)
    switch (window) {
        case TP_MEM_FLASH:   base = 0x08000000u; size = 0x20000u; if (for_write) { return nullptr; } break;
        case TP_MEM_SRAM:    base = 0x20000000u; size = 0x8000u;  break;
        case TP_MEM_CCM:     base = 0x10000000u; size = 0x2800u;  break;
        case TP_MEM_APB1:    base = 0x40000000u; size = 0x10000u; break;
        case TP_MEM_APB2:    base = 0x40010000u; size = 0x10000u; break;
        case TP_MEM_AHB1:    base = 0x40020000u; size = 0x10000u; break;
        case TP_MEM_AHB2:    base = 0x48000000u; size = 0x2000u;  break;
        case TP_MEM_AHB2_ADC: base = 0x50000000u; size = 0x80000u; break;
        case TP_MEM_HEART:   base = (uintptr_t)tp_; size = (uint32_t)sizeof(TunePulseHeart); break;
        case TP_MEM_CORTEX:  base = 0xE0000000u; size = 0x100000u; break;
        default:             return nullptr;                           // Undefined window
    }
    if ((off + 4u) > size) {
        return nullptr;                                                // Past the end of the window
    }
    return (volatile uint32_t*)(base + off);
}

inline TpError TpProtocol::write_direct(const TpFrame& f) {
    const uint32_t addr = tp_unpack_u24(f.direct.addr);                // Windowed address
    if (((addr >> 20) & 0x0Fu) == TP_MEM_FLASH) {
        return TP_ERR_ACCESS_DENIED;                                   // Flash is never written here
    }
    volatile uint32_t* p = direct_pointer(addr, true);                 // Resolve the window
    if (p == nullptr) {
        return TP_ERR_OUT_OF_RANGE;                                    // Unmapped or misaligned
    }
    *p = f.direct.value;                                               // One volatile word write
    return TP_ERR_NONE;
}
