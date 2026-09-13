#pragma once

/**
 * @file tp_engine_motion_impl.h
 * @brief Motion tasks, single-frame setpoints and the burst stream.
 *
 * A segment reaches the queue three ways: as a set of Motion Task frames
 * sharing a SegmentID (assembled here and pushed once every part the header
 * announced has arrived), as a single-frame setpoint (SegmentID 0, applied at
 * once as a cyclic position / velocity / torque command), or as a bit-packed
 * burst frame (PVT or PVcTF, one segment per frame).
 *
 * The frame kind implies the control mode: a lone position frame puts the
 * drive in POSITION mode, a lone velocity frame in VELOCITY, a lone torque
 * frame in TORQUE, and anything that lands in the queue in MOTION. A master
 * therefore never has to name the mode separately for a cyclic setpoint.
 *
 * Torque travels in N*m when the torque constant (register 0x22) is known and
 * in amperes otherwise; the same rule applies to every torque field in the
 * protocol, so a bench without a Kt still drives the current directly.
 *
 * Nothing here writes control state directly. Every effect is posted into the
 * DriveControl mailbox, which the 20 kHz ISR drains between passes - that is
 * what makes it safe for these handlers to be preempted at any instruction.
 *
 * Part of TpProtocol, declared in tp_engine.h.
 *
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#include "../tp_engine.h"  // class TpProtocol - the declarations these bodies define

inline float TpProtocol::torque_to_amps(float torque) const {
    const float kt = tp_->motor_params.get_torque_constant();  // N*m per ampere (0 = unknown)
    return (kt > 0.0f) ? (torque / kt) : torque;
}

inline float TpProtocol::amps_to_torque(float amps) const {
    const float kt = tp_->motor_params.get_torque_constant();  // N*m per ampere (0 = unknown)
    return (kt > 0.0f) ? (amps * kt) : amps;
}

inline uint16_t TpProtocol::torque_norm(float torque) const {
    const float amps = torque_to_amps((torque < 0.0f) ? -torque : torque);       // Magnitude in amperes
    const float lim = (config_.current_limit_a > 0.0f) ? config_.current_limit_a : 1.0f;  // Envelope
    float norm = (amps / lim) * 65535.0f;                                        // Executor scale
    if (norm > 65535.0f) {
        norm = 65535.0f;                                                         // Saturate at full scale
    }
    return (uint16_t)norm;
}

inline uint32_t TpProtocol::scurve_duration_us(float distance_rev, float vel, float acc) {
    // Trapezoidal time estimate: cruise time plus one full acceleration ramp.
    // The executor's Hermite interpolation supplies the smooth ends, so this
    // only has to be the right duration, not the exact profile.
    const float mag = (distance_rev < 0.0f) ? -distance_rev : distance_rev;  // Distance to travel
    const float v = (vel > 0.0f) ? vel : 1.0f;                               // Guard the divide
    const float a = (acc > 0.0f) ? acc : 1.0f;                               // Guard the divide
    const float t = (mag / v) + (v / a);                                     // Cruise plus both half ramps
    if (t < 0.001f) {
        return 1000u;                                                        // Never shorter than 1 ms
    }
    if (t > 4000.0f) {
        return 4000000000u;                                                  // Cap at ~66 minutes
    }
    return (uint32_t)(t * 1000000.0f);
}

inline uint32_t TpProtocol::node_time_us() const {
    return (uint32_t)((int32_t)micros() + sync_offset_us_);  // Re-based by the last bus sync
}

// ============================================================================
// Motion tasks
// ============================================================================

inline TpError TpProtocol::handle_motion_task(const TpFrame& f) {
    if (tp_ == nullptr) {
        return TP_ERR_WRONG_STATE;                       // Not bound to a drive
    }
    const uint8_t cmd = f.b8[0];                         // Which part
    const uint8_t seg = f.b8[1];                         // SegmentID
    if (seg == 0u) {
        return apply_setpoint(f);                        // Single frame: a cyclic setpoint
    }

    if (cmd == TP_CMD_MOTION_TASK_1) {
        // A header opens a segment. A frame for another axis group is not
        // ours - it is accepted silently so a broadcast path never draws a
        // NACK from every node it was not meant for.
        const uint8_t axis = f.mt1.axis_id;              // Axis the segment addresses
        if ((axis != 0u) && (axis != config_.axis_id)) {
            return TP_ERR_NONE;
        }
        if ((seg_parts_ != 0u) && (seg_id_ != seg)) {
            // The previous segment never completed: its parts were lost, or
            // the master moved on. Count it so the queue diagnostics show it.
            if (seg_dropped_ != 0xFFu) {
                seg_dropped_++;
            }
        }
        seg_id_ = seg;
        seg_axis_ = axis;
        seg_parts_ = SEG_PART_1;
        seg_flags_ = f.mt1.flags;
        seg_time_ = tp_unpack_u24(f.mt1.time);
        seg_pos_q16_ = 0;
        seg_torque_raw_ = TP_BF16_NAN;
        seg_vel_raw_ = 0x7FC000u;                        // float24 NaN
        seg_acc_raw_ = TP_BF16_NAN;
        seg_jerk_raw_ = TP_BF16_NAN;
        seg_io_pos_ = 0u;
        seg_io_neg_ = 0u;
    } else {
        if ((seg_parts_ == 0u) || (seg_id_ != seg)) {
            return TP_ERR_SEGMENT;                       // A part of a segment we are not assembling
        }
        switch (cmd) {
            case TP_CMD_MOTION_TASK_2:
                seg_pos_q16_ = tp_q24_to_q16(tp_unpack_i48(f.mt2.pos));
                seg_parts_ |= SEG_PART_2;
                break;
            case TP_CMD_MOTION_TASK_3:
                seg_torque_raw_ = f.mt3.torque;
                seg_vel_raw_ = tp_unpack_u24(f.mt3.velocity);
                seg_parts_ |= SEG_PART_3;
                break;
            case TP_CMD_MOTION_TASK_4:
                seg_acc_raw_ = f.mt4.accel;
                seg_jerk_raw_ = f.mt4.jerk;
                seg_io_pos_ = f.mt4.io_mask_pos;
                seg_io_neg_ = f.mt4.io_mask_neg;
                seg_parts_ |= SEG_PART_4;
                break;
            default:
                return TP_ERR_UNKNOWN_CMD;
        }
    }

    // The header says which parts the segment needs; commit as soon as the
    // last of them is in, so the order the master sends them in is free.
    uint8_t needed = SEG_PART_1;                         // Parts the header announced
    if (TP_MT1_POS_MODE(seg_flags_) != TP_MT_POS_UNUSED) {
        needed |= SEG_PART_2;
    }
    if (TP_MT1_IS_VEL(seg_flags_) || (TP_MT1_TORQ_MODE(seg_flags_) != TP_MT_TORQUE_UNUSED)) {
        needed |= SEG_PART_3;
    }
    if (TP_MT1_IS_ACC(seg_flags_) || TP_MT1_IS_JRK(seg_flags_) || TP_MT1_IO_TRIG(seg_flags_)) {
        needed |= SEG_PART_4;
    }
    if ((seg_parts_ & needed) != needed) {
        return TP_ERR_NONE;                              // Still waiting for a part
    }
    const TpError e = commit_segment();                  // Every part is in: build and push
    seg_parts_ = 0u;                                     // Assembly slot free again
    seg_id_ = 0u;
    return e;
}

inline TpError TpProtocol::commit_segment() {
    const uint8_t mode = TP_MT1_MODE(seg_flags_);        // TpMotionTaskMode
    const bool is_abs = TP_MT1_IS_ABS(seg_flags_);       // Position is absolute
    const uint8_t pos_mode = TP_MT1_POS_MODE(seg_flags_);
    const uint8_t torq_mode = TP_MT1_TORQ_MODE(seg_flags_);
    const uint8_t time_mode = TP_MT1_TIME_MODE(seg_flags_);
    const bool has_pos = (seg_parts_ & SEG_PART_2) != 0u;
    const bool has_dyn = (seg_parts_ & SEG_PART_3) != 0u;
    const bool has_lim = (seg_parts_ & SEG_PART_4) != 0u;

    // ---- Decode the optional quantities ---------------------------------
    float vel = 0.0f;                                    // Segment velocity [RPS], axis sign applied
    bool have_vel = false;                               // A finite velocity was carried
    if (has_dyn && TP_MT1_IS_VEL(seg_flags_) && tp_f24_is_finite(seg_vel_raw_)) {
        vel = tp_axis_sign(config_, tp_unpack_f24(seg_vel_raw_));
        have_vel = true;
    }
    float torque = 0.0f;                                 // Segment torque in wire units
    bool have_torque = false;                            // A finite torque was carried
    if (has_dyn && (torq_mode != TP_MT_TORQUE_UNUSED) && tp_bf16_is_finite(seg_torque_raw_)) {
        torque = tp_unpack_bf16(seg_torque_raw_);
        have_torque = true;
    }
    float acc = config_.accel_limit;                     // Acceleration for profile shaping
    if (has_lim && TP_MT1_IS_ACC(seg_flags_) && tp_bf16_is_finite(seg_acc_raw_) && (tp_unpack_bf16(seg_acc_raw_) > 0.0f)) {
        acc = tp_unpack_bf16(seg_acc_raw_);
    }
    float jerk = config_.jerk_limit;                     // Jerk for profile shaping
    if (has_lim && TP_MT1_IS_JRK(seg_flags_) && tp_bf16_is_finite(seg_jerk_raw_) && (tp_unpack_bf16(seg_jerk_raw_) > 0.0f)) {
        jerk = tp_unpack_bf16(seg_jerk_raw_);
    }
    // Absolute targets are converted by the ISR against the queue tail; a
    // displacement only needs the axis direction applied.
    const int32_t pos = has_pos ? (is_abs ? tp_axis_from_wire(config_, seg_pos_q16_)
                                          : ((config_.axis_inverted != 0u) ? -seg_pos_q16_ : seg_pos_q16_))
                                : 0;
    const uint32_t duration_us = seg_time_ * 100u;       // Wire time [100 us] -> microseconds

    // ---- Segment skeleton --------------------------------------------------
    MotionCommandTM seg;                                 // Segment under construction
    seg.motion.displacement = pos;
    seg.motion.duration = (time_mode != TP_MT_TIME_UNUSED) ? duration_us : 0u;
    seg.motion.end_velocity = have_vel ? vel : 0.0f;
    seg.torque = UINT16_MAX;                             // No torque cap unless asked
    TorqueTrigger trig = TorqueTrigger::DISABLED;        // Torque trigger kind
    if (have_torque) {
        seg.torque = torque_norm(torque);                // Limit or threshold, same scale
        if (torq_mode == TP_MT_TORQUE_TRIGGER) {
            trig = TorqueTrigger::HIGHER_THAN;           // Reaching the torque ends the segment
        }
    }
    uint8_t io_pos = 0u;                                 // Pins that end the segment when active
    uint8_t io_neg = 0x3Fu;                              // Pins that end it when inactive (0x3F = none)
    if (has_lim && TP_MT1_IO_TRIG(seg_flags_)) {
        io_pos = (uint8_t)(seg_io_pos_ & 0x3Fu);
        io_neg = (uint8_t)(~seg_io_neg_ & 0x3Fu);        // The executor keeps this mask inverted
    }

    // ---- Per-mode construction ---------------------------------------------
    MotionMode base = MotionMode::HERMITE;               // Executor base mode
    bool queued = true;                                  // Lands in the executor queue
    switch (mode) {
        case TP_MT_HOLD:
            seg.motion.displacement = 0;                 // Hold: no travel, optional duration
            seg.motion.end_velocity = 0.0f;
            break;
        case TP_MT_FREERUN:
        case TP_MT_FREERUN_LERP:
            if (!have_vel) {
                return TP_ERR_BAD_PARAM;                 // An open-loop run needs a velocity
            }
            base = (mode == TP_MT_FREERUN) ? MotionMode::FREERUN : MotionMode::FREERUN_LERP;
            // The executor treats a non-zero displacement as the position that
            // ends the run; a target and a trigger mean the same thing to it.
            if (pos_mode == TP_MT_POS_UNUSED) {
                seg.motion.displacement = 0;
            }
            break;
        case TP_MT_FREEWHEEL:
            tp_->drive.post_u32(DriveReq::Stop, TP_SAFE_FREEWHEEL);
            queued = false;
            break;
        case TP_MT_BRAKE:
            tp_->drive.post_u32(DriveReq::Stop, TP_SAFE_BRAKE);
            queued = false;
            break;
        case TP_MT_SCURVE: {
            if (!has_pos) {
                return TP_ERR_BAD_PARAM;                 // A move needs a destination
            }
            float cruise = have_vel ? ((vel < 0.0f) ? -vel : vel) : config_.vel_limit_rps;  // Cruise speed
            if (cruise > config_.vel_limit_rps) {
                cruise = config_.vel_limit_rps;          // Clamp into the envelope
            }
            // An absolute destination's distance is only known to the ISR, so
            // the duration is sized from the distance to the CURRENT queue
            // tail as the telemetry last reported it - exact for a queue that
            // has drained, and a fair estimate otherwise.
            DriveTelemetry t;                            // Live control snapshot
            tp_->drive.read_telemetry(t);
            const int32_t dist_q16 = is_abs ? (pos - t.queue_tail_q16) : pos;  // Distance to travel
            seg.motion.duration = scurve_duration_us((float)dist_q16 * (1.0f / 65536.0f), cruise, acc);
            seg.motion.end_velocity = 0.0f;              // Point-to-point ends at rest
            (void)jerk;                                  // Shape comes from the Hermite ends
            break;
        }
        case TP_MT_PVT:
            if (!has_pos || (seg.motion.duration == 0u)) {
                return TP_ERR_BAD_PARAM;                 // A PVT point needs a position and a time
            }
            break;
        case TP_MT_ANCHOR: {
            if (!has_pos) {
                return TP_ERR_BAD_PARAM;                 // Nothing to anchor to
            }
            // Redefine "here" WITHOUT moving: every position crosses the wire
            // through the axis offset of 0x01/0x00 (reported = (native +
            // offset) x direction), so the anchor is a change of that offset
            // and nothing else. The control loop keeps holding the same
            // physical spot; only the number the master sees for it changes.
            // (Posting the anchor value as a new position TARGET, as this used
            // to, sent the rotor there at full speed - a 25 rev "set position"
            // became a 25 rev move.) The offset lives in the node
            // configuration: save it to keep the anchor across a reboot.
            DriveTelemetry t;                            // Live control snapshot
            tp_->drive.read_telemetry(t);
            const int32_t wire_now = tp_axis_to_wire(config_, t.position_q16);       // Where the master sees us
            const int32_t wire_new = is_abs ? seg_pos_q16_ : (wire_now + seg_pos_q16_);  // Where it wants us
            const int32_t signed_new = (config_.axis_inverted != 0u) ? -wire_new : wire_new;  // Undo the direction
            config_.axis_offset_q16 = signed_new - t.position_q16;                    // reported == wire_new
            queued = false;
            break;
        }
        case TP_MT_LIMIT:
            // Adopt the carried limits as the live envelope, like a Motion
            // Limits write without the persistence.
            if (have_vel && (vel != 0.0f)) {
                config_.vel_limit_rps = (vel < 0.0f) ? -vel : vel;
                tp_->drive.post_f32(DriveReq::VelocityLimit, config_.vel_limit_rps);
            }
            if (has_lim && TP_MT1_IS_ACC(seg_flags_)) {
                config_.accel_limit = acc;
                tp_->drive.post_f32(DriveReq::AccelLimit, acc);
            }
            if (has_lim && TP_MT1_IS_JRK(seg_flags_)) {
                config_.jerk_limit = jerk;
                tp_->drive.post_f32(DriveReq::JerkLimit, jerk);
            }
            queued = false;
            break;
        case TP_MT_RESET:
            tp_->drive.post(DriveReq::QueueClear);
            tp_->drive.post_u32(DriveReq::Stop, 0u);     // Hold where the queue was cut
            queued = false;
            break;
        case TP_MT_HOMING:
        case TP_MT_STEP_DIR:
        default:
            return TP_ERR_NOT_IMPLEMENTED;
    }
    if (!queued) {
        return TP_ERR_NONE;                              // Applied immediately
    }

    seg.mode = MotionConfig(base, trig, io_pos, io_neg);
    if (control_mode() != TP_MODE_MOTION) {
        tp_->drive.post_u32(DriveReq::SetMode, TP_MODE_MOTION);  // The queue only advances in MOTION
    }
    return tp_->drive.post_segment(seg, is_abs && has_pos) ? TP_ERR_NONE : TP_ERR_QUEUE_FULL;
}

inline TpError TpProtocol::apply_setpoint(const TpFrame& f) {
    switch (f.b8[0]) {
        case TP_CMD_MOTION_TASK_1: {
            // A lone header carries no target: only the modes that are an
            // action in themselves make sense here.
            switch (TP_MT1_MODE(f.mt1.flags)) {
                case TP_MT_HOLD:
                    return tp_->drive.post_u32(DriveReq::Stop, 0u) ? TP_ERR_NONE : TP_ERR_BUSY;
                case TP_MT_FREEWHEEL:
                    return tp_->drive.post_u32(DriveReq::Stop, TP_SAFE_FREEWHEEL) ? TP_ERR_NONE : TP_ERR_BUSY;
                case TP_MT_BRAKE:
                    return tp_->drive.post_u32(DriveReq::Stop, TP_SAFE_BRAKE) ? TP_ERR_NONE : TP_ERR_BUSY;
                case TP_MT_RESET:
                    tp_->drive.post(DriveReq::QueueClear);
                    return tp_->drive.post_u32(DriveReq::Stop, 0u) ? TP_ERR_NONE : TP_ERR_BUSY;
                case TP_MT_HOMING:
                case TP_MT_STEP_DIR:
                    return TP_ERR_NOT_IMPLEMENTED;
                default:
                    return TP_ERR_BAD_PARAM;             // Needs the parts a SegmentID would carry
            }
        }
        case TP_CMD_MOTION_TASK_2: {
            // Cyclic synchronous position: the frame IS the position mode.
            const int32_t pos = tp_axis_from_wire(config_, tp_q24_to_q16(tp_unpack_i48(f.mt2.pos)));
            if (control_mode() != TP_MODE_POSITION) {
                tp_->drive.post_u32(DriveReq::SetMode, TP_MODE_POSITION);
            }
            return tp_->drive.post_i64(DriveReq::TargetPosAbs, pos) ? TP_ERR_NONE : TP_ERR_BUSY;
        }
        case TP_CMD_MOTION_TASK_3: {
            // Cyclic synchronous velocity when a velocity is carried, torque
            // otherwise; a torque beside a velocity caps the envelope.
            const uint32_t vraw = tp_unpack_u24(f.mt3.velocity);   // float24 velocity
            const bool have_vel = tp_f24_is_finite(vraw);
            const bool have_torque = tp_bf16_is_finite(f.mt3.torque);
            if (!have_vel && !have_torque) {
                return TP_ERR_BAD_PARAM;                 // Nothing to command
            }
            if (have_vel) {
                if (control_mode() != TP_MODE_VELOCITY) {
                    tp_->drive.post_u32(DriveReq::SetMode, TP_MODE_VELOCITY);
                }
                if (have_torque) {
                    float lim = torque_to_amps(tp_unpack_bf16(f.mt3.torque));  // Envelope for the run
                    if (lim < 0.0f) {
                        lim = -lim;
                    }
                    if ((lim > 0.0f) && (lim <= config_.peak_current_a)) {
                        config_.current_limit_a = lim;
                        tp_->drive.post_f32(DriveReq::CurrentLimit, lim);
                    }
                }
                const float vel = tp_axis_sign(config_, tp_unpack_f24(vraw));  // Velocity [RPS]
                return tp_->drive.post_f32(DriveReq::TargetVel, vel) ? TP_ERR_NONE : TP_ERR_BUSY;
            }
            if (control_mode() != TP_MODE_TORQUE) {
                tp_->drive.post_u32(DriveReq::SetMode, TP_MODE_TORQUE);
            }
            const float amps = tp_axis_sign(config_, torque_to_amps(tp_unpack_bf16(f.mt3.torque)));  // Current [A]
            return tp_->drive.post_f32(DriveReq::TargetCurrent, amps) ? TP_ERR_NONE : TP_ERR_BUSY;
        }
        case TP_CMD_MOTION_TASK_4: {
            // A lone limits frame adjusts the live acceleration / jerk envelope.
            float v;                                     // Decoded value
            if (tp_take_bf16_pos(f.mt4.accel, v)) {
                config_.accel_limit = v;
                tp_->drive.post_f32(DriveReq::AccelLimit, v);
            }
            if (tp_take_bf16_pos(f.mt4.jerk, v)) {
                config_.jerk_limit = v;
                tp_->drive.post_f32(DriveReq::JerkLimit, v);
            }
            return TP_ERR_NONE;
        }
        default:
            return TP_ERR_UNKNOWN_CMD;
    }
}

// ============================================================================
// Burst stream
// ============================================================================

inline TpError TpProtocol::handle_burst(const TpFrame& f) {
    if (tp_ == nullptr) {
        return TP_ERR_WRONG_STATE;                       // Not bound to a drive
    }
    const uint32_t now = millis();                       // For the re-prime timeout
    DriveTelemetry t;                                    // Live control snapshot
    tp_->drive.read_telemetry(t);

    // The stream is resolved against the previous point. After a pause the
    // previous point is stale, so the reference and the clock are taken from
    // the drive again: the queue tail is where the next segment starts.
    if (!burst_primed_ || ((now - burst_last_ms_) > 1000u)) {
        burst_ref_q16_ = tp_axis_to_wire(config_, t.queue_tail_q16);
        burst_primed_ = false;
    }
    burst_last_ms_ = now;

    // ---- Position: masked absolute, nearest to the reference ----------------
    const int32_t wire_pos = tp_burst_unmask(tp_burst_pos24(f.b8), burst_ref_q16_);  // Absolute, wire units
    burst_ref_q16_ = wire_pos;                           // Next frame resolves against this one
    const int32_t pos = tp_axis_from_wire(config_, wire_pos);  // Native absolute target

    // ---- Velocity ----------------------------------------------------------
    const uint16_t vraw = tp_burst_vel_raw(f.b8);        // bfloat16 velocity
    const float vel = tp_bf16_is_finite(vraw) ? tp_axis_sign(config_, tp_unpack_bf16(vraw)) : 0.0f;

    // ---- Duration and torque: the two stream formats ------------------------
    MotionCommandTM seg;                                 // Segment under construction
    seg.motion.displacement = pos;                       // Absolute: the ISR converts it
    seg.motion.end_velocity = vel;
    seg.torque = UINT16_MAX;
    if (config_.burst_step == 0u) {
        // PVT: an absolute time tag in 250 us units, modulo 2^21. The
        // duration is the distance from the previous tag, or from the node
        // clock for the first point of a run.
        const uint32_t t21 = tp_burst_time21(f.b8);      // This point's time tag
        const uint32_t ref = burst_primed_ ? burst_last_t21_ : ((node_time_us() / 250u) & 0x1FFFFFu);
        uint32_t dt = (t21 - ref) & 0x1FFFFFu;           // Ticks since the reference, wrapped
        if ((dt == 0u) || (dt > 0x8000u)) {
            dt = 4u;                                     // Past or absurdly far: one millisecond
        }
        seg.motion.duration = dt * 250u;
        burst_last_t21_ = t21;
    } else {
        // PVcTF: a fixed period per frame; the 5-bit counter says how many
        // periods elapsed, so a lost frame stretches the segment instead of
        // shifting everything after it.
        const uint8_t frame = tp_burst_frame(f.b8);      // This point's counter
        uint32_t steps = burst_primed_ ? ((frame - burst_last_frame_) & 0x1Fu) : 1u;
        if (steps == 0u) {
            steps = 1u;                                  // Same counter twice: treat as one period
        }
        seg.motion.duration = steps * (uint32_t)config_.burst_step * 250u;
        burst_last_frame_ = frame;
        const uint16_t fraw = tp_burst_force_raw(f.b8);  // bfloat16 force
        if (tp_bf16_is_finite(fraw)) {
            seg.torque = torque_norm(tp_unpack_bf16(fraw));
        }
    }
    burst_primed_ = true;
    seg.mode = MotionConfig(MotionMode::HERMITE, TorqueTrigger::DISABLED);
    if (control_mode() != TP_MODE_MOTION) {
        tp_->drive.post_u32(DriveReq::SetMode, TP_MODE_MOTION);  // The queue only advances in MOTION
    }
    return tp_->drive.post_segment(seg, true) ? TP_ERR_NONE : TP_ERR_QUEUE_FULL;
}
