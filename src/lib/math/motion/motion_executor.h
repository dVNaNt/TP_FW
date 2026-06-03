// =============================================================================
// motion_executor.h
// Reversible trajectory execution engine for STM32G4 / Cortex-M4F
//
// Features:
//   - Q32.32 / Q16.16 precise fixed-point positioning
//   - Wrap-safe 32-bit hardware time tracking
//   - Seamless reversible multi-segment jumps
//   - O(1) hot path execution
//   - Bypass mode (enable/disable) with time freezing
//   - Non-blocking Unpause at boundaries (idles in HOLD until pushed)
//
// Command modes (derived at push time):
//   HERMITE : duration > 0 and displacement != INT32_MIN
//   HOLD    : duration == 0, displacement == 0, end_velocity == 0
//   FREERUN : displacement == INT32_MIN (Acts as a reverse barrier, wipes history)
//
// Hot path cycle estimate: ~25-35 cycles on Cortex-M4F.
// =============================================================================
#pragma once

#include <cstdint>
#include <cstddef>
#include <cmath>
#include <climits>
#include "qubic_hermite.h"
#include "history_buffer.h"

// =============================================================================
// MotionCommand
// =============================================================================
struct MotionCommand {
    uint32_t duration     = 0;          // Segment duration in microseconds
    int32_t  displacement = INT32_MIN;  // Q16.16 turns, INT32_MIN = FREERUN barrier
    float    end_velocity = INFINITY;   // Segment end velocity (RPS)
    float    torque       = INFINITY;   // Pass-through [0..1]

    inline bool has_duration()     const { return duration != 0; }
    inline bool has_displacement() const { return displacement != INT32_MIN; }
    inline bool has_velocity()     const { return !std::isinf(end_velocity); }
    inline bool has_torque()       const { return !std::isinf(torque); }
};


// =============================================================================
// MotionExecutorOutput
// =============================================================================
struct MotionExecutorOutput {
    int64_t position_q32;
    float   velocity_rps;
    float   torque;
};

// =============================================================================
// MotionExecutor
// =============================================================================
template <size_t BUF_SIZE = 128>
class MotionExecutor {
public:
    void init(uint32_t now_us, int64_t initial_pos_q32 = 0) {
        buf_.clear();
        anchor_q32_      = initial_pos_q32;
        real_pos_q32_    = initial_pos_q32;
        seg_start_us_    = now_us;
        last_tick_us_    = now_us;
        seg_dur_us_      = 0;
        seg_cmd_end_vel_ = 0.0f;
        seg_torque_      = 0.0f;
        seg_mode_        = Mode::HOLD;

        initialized_     = true;
        enabled_         = true;
        paused_          = false;
        last_time_scale_ = 0;
    }

    __attribute__((always_inline))
    void update_real_pos(int64_t pos_q32) { real_pos_q32_ = pos_q32; }

    bool push(MotionCommand cmd) {
        if (!cmd.has_torque())   cmd.torque       = 0.0f;
        if (!cmd.has_velocity()) cmd.end_velocity = 0.0f;

        if (!(cmd.torque >= 0.0f && cmd.torque <= 1.0f)) return false;
        if (!std::isfinite(cmd.end_velocity)) return false;

        const bool is_freerun = !cmd.has_displacement();
        const bool is_hold    = (cmd.duration == 0 && cmd.displacement == 0 && cmd.end_velocity == 0.0f);

        if (is_freerun && cmd.duration != 0) return false;
        if (!is_freerun && !is_hold && cmd.duration == 0) return false;

        return buf_.write(cmd);
    }

    // -------------------------------------------------------------------------
    // Control API
    // -------------------------------------------------------------------------
    bool is_enabled() const { return enabled_; }
    bool is_paused()  const { return paused_; }

    void set_enable(bool en) { enabled_ = en; }

    // Sets or clears the pause state unconditionally.
    // If set to unpaused (p == false) at a trajectory boundary (no future/past), 
    // it will idle in HOLD mode until new commands arrive, smoothly executing them.
    bool set_pause(bool p) {
        paused_ = p;
        return true;
    }

    // -------------------------------------------------------------------------
    // tick()
    // Hot path, highly optimized. Time is internally frozen if disabled or paused.
    // -------------------------------------------------------------------------
    __attribute__((always_inline))
    MotionExecutorOutput tick(uint32_t now_us, int16_t time_scale) {
        if (__builtin_expect(!initialized_, 0)) return {0, NAN, 0.0f};

        const uint32_t dt = now_us - last_tick_us_;
        last_tick_us_    = now_us;
        last_time_scale_ = time_scale;

        // Bypass / Freeze logic
        if (__builtin_expect(!enabled_ || paused_, 0)) {
            seg_start_us_ += dt; // Keeps (now_us - seg_start_us_) constant

            if (!enabled_) {
                return {real_pos_q32_, NAN, seg_torque_};
            }
            // If paused, return the exact frozen evaluated state
            return eval_segment(now_us - seg_start_us_, time_scale);
        }

        const uint32_t t_in_seg = now_us - seg_start_us_;

        if (time_scale >= 0) {
            if (__builtin_expect(needs_forward_transition(t_in_seg), 0))
                do_forward_transition(now_us);
        } else {
            if (__builtin_expect(needs_reverse_transition(t_in_seg), 0))
                do_reverse_transition(now_us);
        }

        return eval_segment(now_us - seg_start_us_, time_scale);
    }

private:
    enum class Mode : uint8_t { HOLD, HERMITE, FREERUN };

    HistoryBuffer<MotionCommand, BUF_SIZE> buf_;
    HermiteInterpolator hermite_;

    int64_t  anchor_q32_      = 0;
    int64_t  real_pos_q32_    = 0;
    uint32_t seg_start_us_    = 0;
    uint32_t last_tick_us_    = 0;
    uint32_t seg_dur_us_      = 0;
    float    seg_cmd_end_vel_ = 0.0f;
    float    seg_torque_      = 0.0f;
    
    bool     initialized_     = false;
    bool     enabled_         = true;
    bool     paused_          = false;
    int16_t  last_time_scale_ = 0;
    Mode     seg_mode_        = Mode::HOLD;

    static __attribute__((always_inline)) int64_t sat_add_i64(int64_t a, int64_t b) {
        if (b > 0 && a > INT64_MAX - b) return INT64_MAX;
        if (b < 0 && a < INT64_MIN - b) return INT64_MIN;
        return a + b;
    }

    __attribute__((always_inline))
    bool needs_forward_transition(uint32_t t_in_seg) const {
        if (seg_mode_ == Mode::HOLD) return buf_.has_future();
        if (seg_dur_us_ > 0 && t_in_seg >= seg_dur_us_) return true;
        if (seg_mode_ == Mode::FREERUN && buf_.has_future()) return true;
        return false;
    }

    __attribute__((always_inline))
    bool needs_reverse_transition(uint32_t t_in_seg) const {
        if (seg_mode_ == Mode::HOLD) return false;
        return t_in_seg > 0x80000000u; // Crossed left boundary backwards
    }

    // Configures the segment and prepares the interpolator.
    // For FREERUN, wipes the history to ensure it's not reversible into.
    void enter_segment(const MotionCommand& cmd, uint32_t start_us, float v0) {
        seg_start_us_    = start_us;
        seg_dur_us_      = cmd.duration;
        seg_cmd_end_vel_ = cmd.end_velocity;
        seg_torque_      = cmd.torque;

        if (!cmd.has_displacement()) {
            seg_mode_   = Mode::FREERUN;
            anchor_q32_ = real_pos_q32_;
            buf_.trim_all_history(); // FREERUN acts as a solid barrier
            return;
        }

        if (cmd.displacement == 0 && cmd.duration == 0 && cmd.end_velocity == 0.0f) {
            seg_mode_ = Mode::HOLD;
            return;
        }

        seg_mode_ = Mode::HERMITE;

        if (__builtin_expect(cmd.duration == 0, 0)) {
            anchor_q32_ = sat_add_i64(anchor_q32_, (int64_t)cmd.displacement << 16);
            seg_mode_   = Mode::HOLD;
            return;
        }

        hermite_.prepare(cmd.displacement, cmd.duration, v0, cmd.end_velocity);
        buf_.trim_to_active();
    }

    // Transitions state forward. Will gracefully idle in HOLD if future is depleted,
    // avoiding forced pause so unpause behavior works universally.
    __attribute__((noinline))
    void do_forward_transition(uint32_t now_us) {
        constexpr size_t kMaxTransitions = BUF_SIZE + 2;

        for (size_t guard = 0; guard < kMaxTransitions; ++guard) {
            if (seg_mode_ == Mode::HOLD && !buf_.has_active()) {
                float v0 = 0.0f;
                MotionCommand prev;
                // If there's a barrier/completed segment before us, use its end_velocity to dock smoothly
                if (buf_.peek_active(prev)) {
                    v0 = prev.end_velocity;
                }
                
                if (!buf_.advance_cursor()) return;
                MotionCommand first;
                if (buf_.peek_active(first)) enter_segment(first, now_us, v0);
                continue;
            }

            MotionCommand active;
            if (!buf_.peek_active(active)) return;

            const uint32_t t_in_seg = now_us - seg_start_us_;

            if (seg_dur_us_ == 0) {
                if (!buf_.has_future()) return;
                if (seg_mode_ == Mode::FREERUN) anchor_q32_ = real_pos_q32_;
                
                const float exit_v = seg_cmd_end_vel_;
                if (!buf_.advance_cursor()) return;

                MotionCommand next;
                if (buf_.peek_active(next)) enter_segment(next, now_us, exit_v);
                continue;
            }

            if (t_in_seg < seg_dur_us_) return;

            if (seg_mode_ == Mode::HERMITE) {
                anchor_q32_ = sat_add_i64(anchor_q32_, (int64_t)active.displacement << 16);
            } else if (seg_mode_ == Mode::FREERUN) {
                anchor_q32_ = real_pos_q32_;
            }

            const float exit_v = active.end_velocity;
            const uint32_t seg_end_us = seg_start_us_ + seg_dur_us_;

            if (!buf_.has_future()) {
                buf_.advance_cursor();
                seg_mode_        = Mode::HOLD;
                seg_start_us_    = now_us;
                seg_dur_us_      = 0;
                seg_cmd_end_vel_ = 0.0f;
                // We purposefully do NOT auto-pause. System idles in HOLD mode.
                return;
            }

            buf_.advance_cursor();
            MotionCommand next;
            if (buf_.peek_active(next)) enter_segment(next, seg_end_us, exit_v);
        }
    }

    // Transitions state backwards.
    // Handles FREERUN barriers by safely popping the current segment to future
    // and caching the docking velocity for smooth forward re-entry.
    __attribute__((noinline))
    void do_reverse_transition(uint32_t now_us) {
        constexpr size_t kMaxTransitions = BUF_SIZE + 2;

        for (size_t guard = 0; guard < kMaxTransitions; ++guard) {
            const uint32_t t_in_seg = now_us - seg_start_us_;
            if (t_in_seg <= 0x80000000u) return; // Inside segment

            MotionCommand prev;
            if (!buf_.retreat_cursor(prev)) {
                // Reached start of history or a FREERUN barrier. Pop to future.
                buf_.pop_active_to_future();
                
                MotionCommand barrier_cmd;
                float dock_v = 0.0f;
                // Peek at the barrier element (which is now right behind the future)
                // We extract its end velocity to guarantee seamless forward docking later.
                if (buf_.peek_active(barrier_cmd)) {
                    dock_v = barrier_cmd.end_velocity;
                }

                seg_mode_        = Mode::HOLD;
                seg_start_us_    = now_us;
                seg_dur_us_      = 0;
                seg_cmd_end_vel_ = dock_v; // Cache docking velocity
                // We purposefully do NOT auto-pause. Ready to seamlessly go forward.
                return;
            }

            if (prev.has_displacement()) {
                anchor_q32_ = sat_add_i64(anchor_q32_, -((int64_t)prev.displacement << 16));
            }

            MotionCommand before_prev;
            const float v0 = buf_.peek_prev_history(before_prev) ? before_prev.end_velocity : 0.0f;

            const uint32_t reentry_start = now_us - prev.duration;
            enter_segment(prev, reentry_start, v0);
        }
    }

    __attribute__((always_inline))
    MotionExecutorOutput eval_segment(uint32_t t_in_seg, int16_t time_scale) const {
        MotionExecutorOutput out{anchor_q32_, 0.0f, seg_torque_};

        static constexpr float inv_time_scale = 1.0f/1024.0f;

        switch (seg_mode_) {
        case Mode::HERMITE: {
            const uint32_t t = (seg_dur_us_ > 0 && t_in_seg > seg_dur_us_) ? seg_dur_us_ : t_in_seg;
            int32_t dp_q16_32 = 0;
            hermite_.eval(t, dp_q16_32, out.velocity_rps);
            out.position_q32 = anchor_q32_ + (int64_t)dp_q16_32;
            out.velocity_rps *= time_scale * inv_time_scale;
            break;
        }

        case Mode::FREERUN:
            out.position_q32 = real_pos_q32_;
            out.velocity_rps = seg_cmd_end_vel_ * time_scale * inv_time_scale;
            break;

        case Mode::HOLD:
        default:
            break;
        }

        return out;
    }
};