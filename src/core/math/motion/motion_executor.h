/**
 * @file motion_executor.h
 * @brief Trajectory execution core: consumes queued MotionCommandTM segments
 *        and produces a time-parameterized position stream (Q32 fixed point).
 *
 * MotionExecutor is the pipeline stage between the command queue and the
 * motion governor. Segments are evaluated by HermiteInterpolator (or held /
 * freewheeled) according to their base mode:
 *
 * - HERMITE : time-based cubic Hermite segment; with displacement 0 it acts
 *   as a bounded "hold" for `dur` microseconds.
 * - FREERUN : open-loop mode; the segment ends only when one of its trigger
 *   conditions (Time / IO / Torque / Position) fires.
 * - ANCHOR  : instantaneous (zero-tick) shift of the mathematical position
 *   basis; used to re-zero coordinates without physical motion.
 *
 * Segments may carry hardware/software triggers; a triggered segment is
 * non-reversible (it can finish early but never rewinds).
 *
 * The tick path is branch-optimized for pipelining: state transitions are
 * driven by precomputed durations and flags stored per segment.
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */
#pragma once

#include <climits>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include "hermite_interpolator.h"
#include "history_buffer.h"
#include "motion_command.h"

/**
 * @class MotionExecutor
 * @brief Trajectory generation core, optimized for the pipeline (branch-free tick).
 *
 * ### Segment semantics and transition rules
 *
 * | Base mode    | Duration (`dur`) | Displacement (`disp`) | Triggers | Reversible | Description |
 * |:-------------|:----------------:|:---------------------:|:--------:|:----------:|:------------|
 * | **HERMITE**  | `> 0`            | `!= 0`                | NO       | **YES**    | Classic time-based Hermite segment. |
 * | **HERMITE**  | `> 0`            | `0`                   | NO       | **YES**    | **Hold Limited**: hold position for `dur`. |
 * | **HERMITE**  | any              | any                   | **YES**  | NO         | Hermite/Hold with trigger (non-reversible!); finishes early. |
 * | **HERMITE**  | `0`              | `0`                   | NO       | NO         | **Hold Unlimited**: holds until a new command arrives. |
 * | **FREERUN**  | any              | any                   | any      | NO         | Open loop; ends on `Time`, `IO`, `Torque` or `Pos` condition. |
 * | **ANCHOR**   | any              | any                   | ignored  | **YES**    | **Instantaneous** (0 ticks) shift of the mathematical basis. |
 *
 * @tparam BUF_SIZE Depth of the internal command ring buffer (segments).
 */
template <size_t BUF_SIZE = 128>
class MotionExecutor {
public:
    void init(uint32_t now_us, int64_t initial_pos_q32 = 0) {
        buf_.clear();
        anchor_q32_ = initial_pos_q32;
        real_pos_q32_ = initial_pos_q32;
        seg_target_pos_q32_ = initial_pos_q32;
        seg_start_us_ = now_us;
        last_tick_us_ = now_us;
        seg_dur_us_ = 0;
        seg_cmd_start_vel_ = 0.0f;
        seg_cmd_end_vel_ = 0.0f;
        seg_torque_ = 0.0f;

        // A fresh queue is a HALTED queue: it holds the initial position at the
        // anchor and starts the first segment the tick it arrives, exactly like
        // a queue that ran dry. Starting in FREERUN instead bound the target to
        // the measured position with a zero torque envelope, so an axis put in
        // motion mode with nothing queued yet went limp until a segment came.
        base_mode_ = static_cast<MotionMode>(0xFF);
        trigger_flags_ = 0;
        is_reversible_ = false;

        initialized_ = true;
        enabled_ = true;
        paused_ = false;
    }

    __attribute__((always_inline)) void update_real_pos(int64_t pos_q32) { real_pos_q32_ = pos_q32; }

    bool push(MotionCommandTM cmd) {
        if (!std::isfinite(cmd.motion.end_velocity) || std::isnan(cmd.motion.end_velocity))
            return false;
        return buf_.write(cmd);
    }

    bool is_enabled() const { return enabled_; }
    bool is_paused() const { return paused_; }

    /// @brief Segments queued ahead of the one currently executing.
    size_t queued() const { return buf_.future_count(); }

    /// @brief Free slots the queue can still accept without evicting history.
    size_t free_slots() const { return buf_.free_count(); }

    /// @brief Numeric MotionMode of the executing segment; 0xFF marks the HALT state.
    uint8_t active_mode_code() const { return static_cast<uint8_t>(base_mode_); }

    void set_enable(bool en) { enabled_ = en; }
    bool set_pause(bool p) {
        paused_ = p;
        return true;
    }

    /**
     * @brief Main processing loop. Called at a high rate.
     */
    __attribute__((always_inline)) MotionExecutorOutput
    tick(uint32_t now_us, int16_t time_scale, uint16_t current_torque, uint8_t current_io, bool& halt, bool& warn) {
        halt = false;
        warn = false;

        if (__builtin_expect(!initialized_, 0))
            return {0, NAN, 0.0f};

        if (time_scale < 0 && !is_reversible_) {
            halt = true;
            enter_halt_mode(now_us);
            return {real_pos_q32_, 0.0f, seg_torque_};
        }

        const uint32_t dt = now_us - last_tick_us_;
        last_tick_us_ = now_us;

        if (__builtin_expect(!enabled_ || paused_, 0)) {
            seg_start_us_ += dt;
            if (!enabled_) return {real_pos_q32_, NAN, seg_torque_};
            return eval_segment(now_us - seg_start_us_, time_scale);
        }

        const uint32_t t_in_seg = now_us - seg_start_us_;

        if (time_scale >= 0) {
            // Instant trigger check. Returns true immediately for ANCHOR.
            if (__builtin_expect(needs_forward_transition(t_in_seg, current_torque, current_io), 0)) {
                do_forward_transition(now_us);
            }
        } else {
            // Unsigned overflow means negative elapsed time
            if (__builtin_expect(t_in_seg > 0x80000000u, 0)) {
                do_reverse_transition(now_us, halt);
            }
        }

        // A queue that ran dry (forward playback entered HALT above, or an
        // earlier pass did) is halted: the caller keeps the full torque
        // envelope so the axis HOLDS the anchor instead of going limp. Only the
        // reverse path used to raise the flag, so a finished forward queue
        // reported a zero torque limit and a loaded axis fell the moment its
        // last segment completed.
        if (static_cast<uint8_t>(base_mode_) == 0xFF) {
            halt = true;
        }
        if (!halt) {
            warn = (time_scale >= 0) ? (buf_.future_count() == 1) : (buf_.history_count() == 1);
        }

        return eval_segment(now_us - seg_start_us_, time_scale);
    }

private:
    // Pre-computed trigger flags (bitmask)
    static constexpr uint8_t FLAG_TIME   = 1 << 0;
    static constexpr uint8_t FLAG_TORQUE = 1 << 1;
    static constexpr uint8_t FLAG_IO     = 1 << 2;
    static constexpr uint8_t FLAG_POS    = 1 << 3;

    HistoryBuffer<MotionCommandTM, BUF_SIZE> buf_;
    HermiteInterpolator hermite_;

    int64_t anchor_q32_ = 0;
    int64_t real_pos_q32_ = 0;
    int64_t seg_target_pos_q32_ = 0;
    
    uint32_t seg_start_us_ = 0;
    uint32_t last_tick_us_ = 0;
    uint32_t seg_dur_us_ = 0;
    float seg_cmd_start_vel_ = 0.0f;
    float seg_cmd_end_vel_ = 0.0f;
    float seg_torque_ = 0.0f;
    uint16_t seg_torque_threshold_ = UINT16_MAX;

    bool initialized_ = false;
    bool enabled_ = true;
    bool paused_ = false;

    // --- State cache ---
    MotionMode base_mode_ = MotionMode::FREERUN;
    uint8_t trigger_flags_ = 0;
    bool is_reversible_ = false;
    
    uint8_t active_pos_mask_ = 0;
    uint8_t active_neg_mask_ = 0;
    TorqueTrigger active_torque_trigger_ = TorqueTrigger::DISABLED;

    static __attribute__((always_inline)) int64_t sat_add_i64(int64_t a, int64_t b) {
        if (b > 0 && a > INT64_MAX - b) return INT64_MAX;
        if (b < 0 && a < INT64_MIN - b) return INT64_MIN;
        return a + b;
    }

    __attribute__((always_inline)) void enter_halt_mode(uint32_t now_us) {
        trigger_flags_ = 0;
        is_reversible_ = false;
        seg_start_us_ = now_us;
        seg_dur_us_ = 0;
        seg_cmd_start_vel_ = 0.0f;
        seg_cmd_end_vel_ = 0.0f;
        seg_torque_ = 0.0f;
        base_mode_ = static_cast<MotionMode>(0xFF); // HALT marker
    }

    /**
     * @brief Segment entry. One-time parameter parsing; keeps tick() branch-free.
     */
    void enter_segment(const MotionCommandTM& cmd, uint32_t start_us, float v0) {
        seg_start_us_ = start_us;
        seg_dur_us_ = cmd.motion.duration;
        seg_cmd_start_vel_ = v0;
        seg_cmd_end_vel_ = cmd.motion.end_velocity;
        seg_torque_threshold_ = cmd.torque;
        
        base_mode_ = cmd.mode.getMode();
        active_torque_trigger_ = cmd.mode.getTorqueTrigger();
        active_pos_mask_ = cmd.mode.getPosMask();
        active_neg_mask_ = cmd.mode.getNegMask();
        
        trigger_flags_ = 0;
        is_reversible_ = false;

        // Elegant check for configured IO triggers. 
        // If neg_mask == 0x3F (default), the trigger is disabled.
        bool has_io_trigger = (active_pos_mask_ != 0) || (active_neg_mask_ != 0x3F);
        bool has_torque_trigger = (active_torque_trigger_ != TorqueTrigger::DISABLED);

        if (has_io_trigger) trigger_flags_ |= FLAG_IO;
        if (has_torque_trigger) trigger_flags_ |= FLAG_TORQUE;
        if (seg_dur_us_ > 0) trigger_flags_ |= FLAG_TIME;

        if (base_mode_ == MotionMode::ANCHOR) {
            // ANCHOR: instantaneous update. Takes no time, executes right away!
            is_reversible_ = true;
            trigger_flags_ = FLAG_TIME; // Force the transition (time >= 0 fires instantly)
            seg_dur_us_ = 0;
            buf_.trim_to_active();
            seg_torque_ = 1.0f; // Pass-through
            seg_cmd_end_vel_ = v0; // Inherit velocity to avoid stopping
            return;
        }

        if (base_mode_ == MotionMode::HERMITE) {
            if (has_io_trigger || has_torque_trigger || (seg_dur_us_ == 0 && trigger_flags_ == 0)) {
                // If triggers are set OR this is Hold Unlimited (dur=0, trig=0), the mode is NON-REVERSIBLE
                is_reversible_ = false;
                buf_.trim_all_history();
                seg_torque_ = has_torque_trigger ? 1.0f : ((cmd.torque != UINT16_MAX) ? (cmd.torque * (1.0f / 65535.0f)) : 1.0f);
            } else {
                // Reversible Hermite (including classic Hold Limited)
                is_reversible_ = true;
                buf_.trim_to_active();
                seg_torque_ = (cmd.torque != UINT16_MAX) ? (cmd.torque * (1.0f / 65535.0f)) : 1.0f;
            }

            if (seg_dur_us_ > 0) {
                hermite_.prepare(cmd.motion.displacement, seg_dur_us_, v0, cmd.motion.end_velocity);
            }
        } else {
            // FREERUN (non-reversible modes)
            buf_.trim_all_history();
            seg_torque_ = has_torque_trigger ? 1.0f : ((cmd.torque != UINT16_MAX) ? (cmd.torque * (1.0f / 65535.0f)) : 1.0f);
            
            anchor_q32_ = real_pos_q32_; // Bind to reality, reset the offset
            seg_target_pos_q32_ = sat_add_i64(anchor_q32_, (int64_t)cmd.motion.displacement << 16);
            
            if (cmd.motion.displacement != 0) trigger_flags_ |= FLAG_POS;
        }
    }

    /**
     * @brief Ultra-fast O(1) bitwise trigger check.
     */
    __attribute__((always_inline)) bool needs_forward_transition(uint32_t t_in_seg, uint16_t current_torque, uint8_t current_io) const {
        if (static_cast<uint8_t>(base_mode_) == 0xFF) return buf_.has_future(); // HALT recovery

        // Infinite Hold (or FREERUN) without triggers. Transitions as soon as a command arrives.
        if (trigger_flags_ == 0) return buf_.has_future();

        if (trigger_flags_ & FLAG_TIME) {
            if (t_in_seg >= seg_dur_us_) return true; // For ANCHOR always fires (0 >= 0)
        }

        if (trigger_flags_ & FLAG_TORQUE) {
            if (active_torque_trigger_ == TorqueTrigger::HIGHER_THAN && current_torque >= seg_torque_threshold_) return true;
            if (active_torque_trigger_ == TorqueTrigger::LOWER_THAN && current_torque <= seg_torque_threshold_) return true;
        }

        if (trigger_flags_ & FLAG_IO) {
            // Elegant branchless check (bitwise):
            // Direct mask: look for at least one matching one.
            if ((current_io & active_pos_mask_) != 0) return true;
            // Inverted mask: look for at least one matching zero. 
            // (~current_io has ones where the pin reads 0). 
            if ((~current_io & ~active_neg_mask_ & 0x3F) != 0) return true;
        }

        if (trigger_flags_ & FLAG_POS) {
            if (seg_target_pos_q32_ > anchor_q32_) {
                if (real_pos_q32_ >= seg_target_pos_q32_) return true;
            } else if (seg_target_pos_q32_ < anchor_q32_) {
                if (real_pos_q32_ <= seg_target_pos_q32_) return true;
            }
        }

        return false;
    }

    __attribute__((noinline)) void do_forward_transition(uint32_t now_us) {
        constexpr size_t kMaxTransitions = BUF_SIZE + 2;

        for (size_t guard = 0; guard < kMaxTransitions; ++guard) {
            if (static_cast<uint8_t>(base_mode_) == 0xFF && !buf_.has_active()) {
                float v0 = 0.0f;
                MotionCommandTM prev;
                if (buf_.peek_active(prev)) v0 = prev.motion.end_velocity;

                if (!buf_.advance_cursor()) return;
                
                MotionCommandTM first;
                if (buf_.peek_active(first)) enter_segment(first, now_us, v0);

                // The segment has only just begun: leave unless it is one that
                // finishes on the spot (ANCHOR, or a hold with a successor
                // already queued). Falling through unconditionally treated the
                // segment as complete in the same pass - its displacement was
                // folded into the anchor untravelled, its successor was started
                // with a start time one duration in the future, and the wrapped
                // elapsed time then completed every following segment one tick
                // apart until the queue was empty. Whatever had been queued by
                // then was swallowed; whatever arrived later played normally.
                if (!needs_forward_transition(0, 0, 0)) return;
                continue;
            }

            MotionCommandTM active;
            if (!buf_.peek_active(active)) {
                enter_halt_mode(now_us);
                return;
            }

            // Mathematically exact basis shift ONLY for reversible segments and anchors
            if (base_mode_ == MotionMode::HERMITE || base_mode_ == MotionMode::ANCHOR) {
                anchor_q32_ = sat_add_i64(anchor_q32_, (int64_t)active.motion.displacement << 16);
            } else {
                anchor_q32_ = real_pos_q32_; // Bind to reality to protect against jerks
            }

            float exit_v = active.motion.end_velocity;
            if (base_mode_ == MotionMode::HERMITE && seg_dur_us_ == 0) {
                exit_v = 0.0f; // Hold Unlimited exit
            } else if (base_mode_ == MotionMode::ANCHOR) {
                exit_v = seg_cmd_end_vel_; // Pass the exit velocity through the ANCHOR
            }

            uint32_t seg_end_us = (trigger_flags_ & FLAG_TIME) ? (seg_start_us_ + seg_dur_us_) : now_us;

            if (!buf_.has_future()) {
                buf_.advance_cursor();
                enter_halt_mode(seg_end_us);
                return;
            }

            buf_.advance_cursor();
            MotionCommandTM next;
            if (buf_.peek_active(next)) enter_segment(next, seg_end_us, exit_v);
            
            // ANCHOR resets this condition to true and the loop continues (instantly!)
            if (!needs_forward_transition(0, 0, 0)) return; 
        }
    }

    __attribute__((noinline)) void do_reverse_transition(uint32_t now_us, bool& halt) {
        constexpr size_t kMaxTransitions = BUF_SIZE + 2;

        for (size_t guard = 0; guard < kMaxTransitions; ++guard) {
            const uint32_t t_in_seg = now_us - seg_start_us_;
            if (t_in_seg <= 0x80000000u) return;

            if (!buf_.has_history()) {
                halt = true;
                enter_halt_mode(now_us);
                return;
            }

            MotionCommandTM prev;
            if (!buf_.retreat_cursor(prev)) {
                halt = true;
                enter_halt_mode(now_us);
                return;
            }

            // Correctly roll the math back
            if ((prev.mode.getMode() == MotionMode::HERMITE && prev.motion.duration > 0) ||
                 prev.mode.getMode() == MotionMode::ANCHOR) {
                anchor_q32_ = sat_add_i64(anchor_q32_, -((int64_t)prev.motion.displacement << 16));
            }

            MotionCommandTM before_prev;
            float v0 = 0.0f;
            if (buf_.peek_prev_history(before_prev)) {
                v0 = before_prev.motion.end_velocity;
            }

            const uint32_t reentry_start = seg_start_us_ - prev.motion.duration;
            enter_segment(prev, reentry_start, v0);
        }
    }

    __attribute__((always_inline)) MotionExecutorOutput eval_segment(uint32_t t_in_seg, int16_t time_scale) const {
        MotionExecutorOutput out{anchor_q32_, 0.0f, seg_torque_};
        static constexpr float inv_time_scale = 1.0f / 1024.0f;

        switch (base_mode_) {
            case MotionMode::HERMITE: {
                if (seg_dur_us_ > 0) { // Regular Hermite or Hold Limited
                    uint32_t t = (t_in_seg > seg_dur_us_) ? seg_dur_us_ : t_in_seg;
                    int64_t dp_q32 = 0;                    // Displacement from the segment start, Q32.32
                    hermite_.eval(t, dp_q32, out.velocity_rps);
                    out.position_q32 = sat_add_i64(anchor_q32_, dp_q32);
                    out.velocity_rps *= time_scale * inv_time_scale;
                } else { // Hold Unlimited
                    out.position_q32 = anchor_q32_;
                    out.velocity_rps = 0.0f;
                }
                break;
            }
            case MotionMode::FREERUN_LERP: {
                out.position_q32 = real_pos_q32_;
                float v = seg_cmd_end_vel_;
                if (seg_dur_us_ > 0) {
                    float progress = (float)t_in_seg / (float)seg_dur_us_;
                    if (progress > 1.0f) progress = 1.0f;
                    if (progress < 0.0f) progress = 0.0f;
                    v = seg_cmd_start_vel_ + (seg_cmd_end_vel_ - seg_cmd_start_vel_) * progress;
                }
                out.velocity_rps = v * time_scale * inv_time_scale;
                break;
            }
            case MotionMode::FREERUN: {
                out.position_q32 = real_pos_q32_;
                out.velocity_rps = seg_cmd_end_vel_ * time_scale * inv_time_scale;
                break;
            }
            case MotionMode::ANCHOR:
            default: { // ANCHOR and HALT (0xFF)
                // ANCHOR never actually runs (in the normal flow) since it is skipped instantly,
                // but if it is the last segment (ending at HALT), its velocity is inherited.
                out.position_q32 = anchor_q32_;
                out.velocity_rps = seg_cmd_end_vel_ * time_scale * inv_time_scale;
                break;
            }
        }
        return out;
    }
};