#pragma once

/**
 * @file tp_engine_control_impl.h
 * @brief Real-time control, targets and the trajectory queue.
 *
 * Mode selection, the three target kinds, envelope limits, stop / anchor, and
 * the two ways a segment reaches the queue: a bit-packed PVT / PVcTF burst or
 * an s-curve segment sized from the configured envelope.
 *
 * Nothing here writes control state directly. Every effect is posted into the
 * DriveControl mailbox, which the 20 kHz ISR drains between passes - that is
 * what makes it safe for these handlers to be preempted at any instruction.
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

inline TpError TpProtocol::handle_control(const TpFrame& f, bool request) {
    (void)request;                                  // Behaviour is identical on both channels
    if (tp_ == nullptr) {
        return TP_ERR_WRONG_STATE;                  // Not bound to a drive
    }
    if (tp_burst_is(f.b8)) {
        return queue_burst(f);                      // Bit-packed PVT / PVcTF segment
    }

    switch (f.b8[0]) {
        case TP_OP_CONTROL_MODE: {
            if (f.control_mode.mode > (uint8_t)TP_MODE_MOTION) {
                return TP_ERR_BAD_PARAM;            // Undefined mode
            }
            if ((f.control_mode.enable != 0u) && !tp_->angle_calibrator.is_calibrated()) {
                return TP_ERR_NOT_CALIBRATED;       // No angle table: refuse to energize
            }
            if (!tp_->drive.post_u32(DriveReq::SetMode, f.control_mode.mode)) {
                return TP_ERR_BUSY;                 // Mailbox full
            }
            if (f.control_mode.enable != 0u) {
                tp_->drive.post_u32(DriveReq::SetEnable, 1u);
            }
            return TP_ERR_NONE;
        }

        case TP_OP_TARGET_POSITION: {
            const int64_t pos = tp_unpack_i48(f.target_position.pos);  // Q16.16 revolutions
            const DriveReq op = ((f.target_position.flags & TP_POS_FLAG_RELATIVE) != 0u)
                                    ? DriveReq::TargetPosRel
                                    : DriveReq::TargetPosAbs;
            if ((f.target_position.flags & TP_POS_FLAG_SET_MODE) != 0u) {
                tp_->drive.post_u32(DriveReq::SetMode, TP_MODE_POSITION);
            }
            return tp_->drive.post_i64(op, pos) ? TP_ERR_NONE : TP_ERR_BUSY;
        }

        case TP_OP_TARGET_VELOCITY: {
            const float vel = tp_unpack_f24(read_u24(f.target_velocity.velocity));  // [RPS]
            if (vel != vel) {
                return TP_ERR_BAD_PARAM;            // NaN fails the self-compare
            }
            if (f.target_velocity.accel != 0u) {
                const float acc = tp_unpack_bf16(f.target_velocity.accel);
                if (acc > 0.0f) {
                    config_.accel_limit = acc;      // Keep the shadow in step
                    tp_->drive.post_f32(DriveReq::AccelLimit, acc);
                }
            }
            if ((f.target_velocity.flags & TP_TGT_FLAG_SET_MODE) != 0u) {
                tp_->drive.post_u32(DriveReq::SetMode, TP_MODE_VELOCITY);
            }
            return tp_->drive.post_f32(DriveReq::TargetVel, vel) ? TP_ERR_NONE : TP_ERR_BUSY;
        }

        case TP_OP_TARGET_TORQUE: {
            const float cur = tp_unpack_bf16(f.target_torque.current);  // [A]
            if (cur != cur) {
                return TP_ERR_BAD_PARAM;            // NaN fails the self-compare
            }
            if (f.target_torque.limit != 0u) {
                const float lim = tp_unpack_bf16(f.target_torque.limit);
                if (lim > 0.0f) {
                    config_.current_limit_a = lim;  // Keep the shadow in step
                    tp_->drive.post_f32(DriveReq::CurrentLimit, lim);
                }
            }
            if ((f.target_torque.flags & TP_TGT_FLAG_SET_MODE) != 0u) {
                tp_->drive.post_u32(DriveReq::SetMode, TP_MODE_TORQUE);
            }
            return tp_->drive.post_f32(DriveReq::TargetCurrent, cur) ? TP_ERR_NONE : TP_ERR_BUSY;
        }

        case TP_OP_MOTION_LIMITS: {
            const float vel = tp_unpack_f24(read_u24(f.motion_limits.velocity));  // [RPS]
            const float acc = tp_unpack_bf16(f.motion_limits.accel);              // [RPS^2]
            const float jrk = tp_unpack_bf16(f.motion_limits.jerk);               // [RPS^3]
            if (vel > 0.0f) {
                config_.vel_limit_rps = vel;
                tp_->drive.post_f32(DriveReq::VelocityLimit, vel);
            }
            if (acc > 0.0f) {
                config_.accel_limit = acc;
                tp_->drive.post_f32(DriveReq::AccelLimit, acc);
            }
            if (jrk > 0.0f) {
                config_.jerk_limit = jrk;
                tp_->drive.post_f32(DriveReq::JerkLimit, jrk);
            }
            return TP_ERR_NONE;
        }

        case TP_OP_STOP:
            if (f.stop.mode > TP_STOP_QUICK) {
                return TP_ERR_BAD_PARAM;            // Undefined stop style
            }
            return tp_->drive.post_u32(DriveReq::Stop, f.stop.mode) ? TP_ERR_NONE : TP_ERR_BUSY;

        case TP_OP_ANCHOR: {
            const int64_t pos = tp_unpack_i48(f.anchor.pos);  // New value of "here"
            return tp_->drive.post_i64(DriveReq::Anchor, pos) ? TP_ERR_NONE : TP_ERR_BUSY;
        }

        case TP_OP_MOTION_SCURVE:
            return queue_scurve(f);

        case TP_OP_MOTION_QUEUE_CTL:
            return handle_queue_ctl(f);

        default:
            return TP_ERR_UNKNOWN_OPCODE;
    }
}

inline TpError TpProtocol::queue_burst(const TpFrame& f) {
    MotionCommandTM seg;                                    // Segment under construction
    seg.motion.displacement = tp_burst_disp(f.b8);          // Q16.16 revolutions
    seg.motion.end_velocity = tp_burst_vel(f.b8);           // [RPS]
    if (seg.motion.end_velocity != seg.motion.end_velocity) {
        return TP_ERR_BAD_PARAM;                            // NaN fails the self-compare
    }
    const uint16_t tail = tp_burst_tail(f.b8);              // Duration or force
    if ((f.b8[0] & TP_BURST_CONST_T) != 0u) {
        // Force-controlled: run open loop until the displacement is
        // reached, with the trailing field capping the torque.
        seg.motion.duration = 0u;
        seg.torque = tail;
        seg.mode = MotionConfig(MotionMode::FREERUN, TorqueTrigger::DISABLED);
    } else {
        // Time-parameterized spline: 100 us units on the wire keep a
        // 16-bit field useful out to 6.5 seconds per segment.
        seg.motion.duration = (uint32_t)tail * 100u;
        seg.torque = UINT16_MAX;                            // No per-segment torque cap
        seg.mode = MotionConfig(MotionMode::HERMITE, TorqueTrigger::DISABLED);
    }
    return tp_->drive.post_segment(seg) ? TP_ERR_NONE : TP_ERR_QUEUE_FULL;
}

inline TpError TpProtocol::queue_scurve(const TpFrame& f) {
    const int32_t disp_q16 = tp_unpack_i24(f.scurve.target);  // Q16.16 revolutions
    float vel = (f.scurve.velocity != 0u) ? tp_unpack_bf16(f.scurve.velocity)
                                          : config_.vel_limit_rps;
    if ((vel != vel) || (vel <= 0.0f)) {
        return TP_ERR_BAD_PARAM;                             // NaN or non-positive cruise speed
    }
    if (vel > config_.vel_limit_rps) {
        vel = config_.vel_limit_rps;                         // Clamp into the envelope
    }
    if ((f.scurve.flags & TP_SCURVE_RELATIVE) == 0u) {
        // Absolute targets are only meaningful against the live position,
        // which the ISR owns; the queue works in displacements, so an
        // absolute move is not expressible as a queued segment.
        return TP_ERR_BAD_PARAM;
    }
    const float rev = (float)disp_q16 * (1.0f / 65536.0f);   // Displacement in revolutions
    const float mag = (rev < 0.0f) ? -rev : rev;             // Distance to travel
    if (mag <= 0.0f) {
        return TP_ERR_BAD_PARAM;                             // A zero move is not a move
    }
    // Trapezoidal time estimate: cruise time plus one full acceleration
    // ramp. The executor's Hermite interpolation supplies the smooth ends,
    // so this only has to be the right duration, not the exact profile.
    const float acc = (config_.accel_limit > 0.0f) ? config_.accel_limit : 1.0f;
    const float t_ramp = vel / acc;                          // Time to reach cruise speed
    const float t_total = (mag / vel) + t_ramp;              // Cruise plus both half ramps
    uint32_t dur_us = (uint32_t)(t_total * 1000000.0f);      // Executor works in microseconds
    if (dur_us < 1000u) {
        dur_us = 1000u;                                      // Never shorter than 1 ms
    }

    MotionCommandTM seg;                                     // Segment under construction
    seg.motion.displacement = disp_q16;                      // Where to go, relative
    seg.motion.duration = dur_us;                            // How long it may take
    seg.motion.end_velocity = 0.0f;                          // Point-to-point ends at rest
    seg.torque = UINT16_MAX;                                 // No per-segment torque cap
    seg.mode = MotionConfig(MotionMode::HERMITE, TorqueTrigger::DISABLED);
    return tp_->drive.post_segment(seg) ? TP_ERR_NONE : TP_ERR_QUEUE_FULL;
}

inline TpError TpProtocol::handle_queue_ctl(const TpFrame& f) {
    switch (f.queue_ctl.action) {
        case TP_QUEUE_NOP:
            return TP_ERR_NONE;                              // Nothing requested
        case TP_QUEUE_CLEAR:
            return tp_->drive.post(DriveReq::QueueClear) ? TP_ERR_NONE : TP_ERR_BUSY;
        case TP_QUEUE_PAUSE:
            return tp_->drive.post_u32(DriveReq::QueuePause, 1u) ? TP_ERR_NONE : TP_ERR_BUSY;
        case TP_QUEUE_RESUME:
            return tp_->drive.post_u32(DriveReq::QueuePause, 0u) ? TP_ERR_NONE : TP_ERR_BUSY;
        case TP_QUEUE_TIME_SCALE:
            return tp_->drive.post_i32(DriveReq::TimeScale, f.queue_ctl.time_scale) ? TP_ERR_NONE
                                                                                    : TP_ERR_BUSY;
        default:
            return TP_ERR_BAD_PARAM;                         // Undefined action
    }
}

inline TpError TpProtocol::apply_motor_properties(const TpFrame& f) {
    if (tp_ == nullptr) {
        return TP_ERR_WRONG_STATE;                           // Not bound to a drive
    }
    // A read is a frame with an all-zero payload: nothing to apply.
    if ((f.motor_props.temp_lim == 0u) && (f.motor_props.resistance == 0u) &&
        (f.motor_props.inductance == 0u) && (f.motor_props.torque == 0u)) {
        return TP_ERR_NONE;
    }
    if (is_enabled()) {
        return TP_ERR_WRONG_STATE;                           // Never retune a live loop
    }
    if (f.motor_props.temp_lim != 0u) {
        tp_->motor_params.set_max_temp(f.motor_props.temp_lim);
    }
    if (f.motor_props.resistance != 0u) {
        const float r = tp_unpack_bf16(f.motor_props.resistance);
        if (!tp_reg_positive(r)) {
            return TP_ERR_OUT_OF_RANGE;
        }
        tp_->motor_params.set_resistance(r);
    }
    if (f.motor_props.inductance != 0u) {
        const float l = tp_unpack_bf16(f.motor_props.inductance);
        if (!tp_reg_positive(l)) {
            return TP_ERR_OUT_OF_RANGE;
        }
        tp_->motor_params.set_inductance_d(l);               // Symmetric machine assumption
        tp_->motor_params.set_inductance_q(l);
    }
    if (f.motor_props.torque != 0u) {
        const float kt = tp_unpack_bf16(f.motor_props.torque);
        if (!tp_reg_positive(kt)) {
            return TP_ERR_OUT_OF_RANGE;
        }
        tp_->motor_params.set_torque_constant(kt);
    }
    return TP_ERR_NONE;
}
