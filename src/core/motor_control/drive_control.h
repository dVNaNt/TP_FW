#pragma once

/**
 * @file drive_control.h
 * @brief Runtime control-mode machine, command mailbox and trajectory front end.
 *
 * This is the layer that turns the communication stack into actual motion. It
 * owns everything that a remote master can change about how the drive behaves
 * and hands the control loop a single resolved answer per pass:
 *
 * - the active control mode (disabled / torque / velocity / position / motion),
 * - the resolved position, velocity and current targets for this pass,
 * - the safe state to fall back to when the drive is disabled, stopped or the
 *   master goes quiet.
 *
 * ## Threading model
 *
 * Commands originate in THREAD mode (the protocol engine polled from loop()),
 * while every target is consumed by the 20 kHz control ISR. Rather than
 * letting two contexts share mutable control state, all mutation is funnelled
 * through one lock-free single-producer/single-consumer mailbox:
 *
 *   thread  -> post()      -> request ring -> drain()  -> ISR-owned state
 *   ISR     -> publish()   -> seqlock snapshot         -> read_telemetry() -> thread
 *
 * The consequence is that MotionExecutor, the mode variable and every setpoint
 * are touched by the ISR only. The thread side never takes a lock, never
 * blocks the ISR, and a full mailbox degrades to a rejected command (reported
 * back to the master) instead of a torn setpoint.
 *
 * The telemetry direction uses a sequence lock because the ISR is the writer:
 * the reader (thread) may spin, the writer never can. This is the same pattern
 * the encoder uses for its 64-bit position.
 *
 * ## Watchdog
 *
 * A master-liveness watchdog is armed by writing a reload value in control-loop
 * ticks. The ISR decrements the counter every pass; the protocol engine reloads
 * it with a single 32-bit store whenever an addressed frame arrives. When the
 * counter reaches zero the drive drops into its configured safe state and
 * raises a warning, and it stays there until a new command arrives.
 *
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#include "../common.h"
#include "../math/motion/motion_executor.h"  // Trajectory queue and segment evaluation

/** Depth of the command mailbox in entries; must be a power of two. */
#ifndef DRIVE_REQUEST_QUEUE_DEPTH
#define DRIVE_REQUEST_QUEUE_DEPTH 32u
#endif

/** Depth of the trajectory segment queue owned by the executor. */
#ifndef DRIVE_MOTION_QUEUE_DEPTH
#define DRIVE_MOTION_QUEUE_DEPTH 32u
#endif

/**
 * @brief Control modes. The numeric values match TpControlMode on the wire, so
 *        a mode travels between the protocol and the control loop uncast.
 */
enum class DriveMode : uint8_t {
    Disabled = 0,  ///< Output stage held in the safe state; no regulation runs.
    Torque   = 1,  ///< Direct q-axis current command, no position loop.
    Velocity = 2,  ///< Velocity command; the position target ramps at that rate.
    Position = 3,  ///< Absolute position command fed to the position loop.
    Motion   = 4,  ///< Trajectory queue executed by MotionExecutor.
    Tone     = 5   ///< Audio-frequency excitation: the coils are driven as a speaker.
};

/**
 * @brief Behaviour when the drive is disabled, stopped or the watchdog expires.
 *        Values match TpSafeState on the wire.
 */
enum class DriveSafeState : uint8_t {
    Freewheel = 0,  ///< High impedance: the rotor coasts (all gates off).
    Brake     = 1,  ///< All low-side switches on: dynamic braking.
    Hold      = 2   ///< Keep the position loop closed on the position at entry.
};

/**
 * @brief What the control loop must do with the result of one DriveControl pass.
 *
 * The control loop maps these onto the sentinel voltages the PWM backend
 * understands; DriveControl never touches hardware itself.
 */
enum class DriveAction : uint8_t {
    RunPosition = 0,  ///< Run the position loop against target_pos / target_vel.
    RunTorque   = 1,  ///< Bypass the position loop, command torque_target directly.
    Freewheel   = 2,  ///< Release the bridge (Hi-Z).
    Brake       = 3,  ///< Short the phases through the low side.
    Tone        = 4   ///< Drive the alpha-beta vector from the tone generator.
};

/** @brief Resolved control targets produced by one DriveControl::tick() pass. */
struct DriveOutput {
    DriveAction action;          ///< What the control loop should do this pass.
    int32_t     target_pos_q16;  ///< Position target [rev], Q16.16 (encoder units).
    float       target_vel_rps;  ///< Velocity target / damper reference [RPS].
    float       max_current_a;   ///< Current envelope for this pass [A].
    float       torque_target_a; ///< Direct q-axis current target [A] (RunTorque only).
    uint16_t    tone_freq_hz;    ///< Tone frequency [Hz] (Tone only; 0 = silent).
    uint16_t    tone_amp_q15;    ///< Tone amplitude, Q15 fraction of bus volts (Tone only).
};

/** @brief Snapshot of the control state published by the ISR for telemetry. */
struct DriveTelemetry {
    int32_t  position_q16;      ///< Measured position [rev], Q16.16.
    int32_t  target_pos_q16;    ///< Active position target [rev], Q16.16.
    int32_t  queue_tail_q16;    ///< Position the last queued segment ends at [rev], Q16.16.
    float    target_vel_rps;    ///< Active velocity target [RPS].
    float    applied_current_a; ///< Current commanded to the FOC loop [A].
    uint8_t  mode;              ///< Active DriveMode.
    uint8_t  motion_mode;       ///< MotionMode of the executing segment.
    uint8_t  queue_depth;       ///< Segments waiting in the executor queue.
    uint8_t  queue_free;        ///< Slots the queue can still accept.
    uint8_t  flags;             ///< See DRIVE_TLM_* below.
    uint16_t dropped_segments;  ///< Segments rejected because a queue was full.
    int16_t  time_scale_q10;    ///< Active trajectory playback rate, Q10.
};

/** Bits of DriveTelemetry::flags. */
#define DRIVE_TLM_ENABLED   (1u << 0)  ///< Output stage enabled by command.
#define DRIVE_TLM_WATCHDOG  (1u << 1)  ///< Master-liveness watchdog has expired.
#define DRIVE_TLM_PAUSED    (1u << 2)  ///< Trajectory playback is paused.
#define DRIVE_TLM_HALTED    (1u << 3)  ///< Executor ran out of segments and halted.
#define DRIVE_TLM_STARVING  (1u << 4)  ///< One segment left: the queue is about to run dry.
#define DRIVE_TLM_MOVING    (1u << 5)  ///< A trajectory or velocity ramp is in progress.
#define DRIVE_TLM_LOOP_LIVE (1u << 6)  ///< The position loop ran this pass, so its observer is current.

/** Command opcodes accepted by the DriveControl mailbox. */
enum class DriveReq : uint8_t {
    None = 0,        ///< Empty slot (never posted).
    SetMode,         ///< u32 = DriveMode value.
    SetEnable,       ///< u32 = 0 disable / 1 enable.
    TargetPosAbs,    ///< i64 = absolute position target, Q16.16.
    TargetPosRel,    ///< i64 = displacement added to the current target, Q16.16.
    TargetVel,       ///< f32 = velocity target [RPS].
    TargetCurrent,   ///< f32 = q-axis current target [A].
    CurrentLimit,    ///< f32 = current envelope [A].
    VelocityLimit,   ///< f32 = velocity envelope [RPS].
    AccelLimit,      ///< f32 = acceleration envelope [RPS^2].
    JerkLimit,       ///< f32 = jerk envelope [RPS^3].
    Stop,            ///< u32 = TpStopMode value.
    PushSegment,     ///< seg = one trajectory segment for the executor queue.
    PushSegmentAbs,  ///< seg = segment whose displacement field is an ABSOLUTE target, Q16.16.
    QueueClear,      ///< Drop every queued segment and hold.
    QueuePause,      ///< u32 = 0 resume / 1 pause.
    TimeScale,       ///< i32 = playback rate, Q10 (1024 = 1.0x).
    SetSafeState,    ///< u32 = DriveSafeState value.
    ArmWatchdog,     ///< u32 = reload value in control ticks (0 disables).
    SetTone          ///< u32 = frequency [Hz] in bits 0-15, Q15 amplitude in bits 16-31.
};

/**
 * @brief One mailbox entry: an opcode plus its single argument.
 *
 * The argument is a union so that a request stays one cache-friendly object no
 * matter which command it carries. MotionCommandTM has default member
 * initializers, which deletes the implicit union constructor, so the union
 * declares its own trivial one that simply activates the widest scalar member.
 */
struct DriveRequest {
    DriveReq op = DriveReq::None;  ///< What to do.

    /** Command argument; exactly one member is meaningful per opcode. */
    union Arg {
        int64_t         i64;  ///< 64-bit signed argument (positions).
        int32_t         i32;  ///< 32-bit signed argument (time scale).
        uint32_t        u32;  ///< 32-bit unsigned argument (enums, flags).
        float           f32;  ///< Single-precision argument (physical setpoints).
        MotionCommandTM seg;  ///< A whole trajectory segment.

        /// Activate the widest scalar member so the union is default constructible.
        Arg() : i64(0) {}
    } arg;
};

/**
 * @class DriveControl
 * @brief Mode machine, command mailbox and trajectory front end of one drive.
 *
 * Instantiated once inside TunePulseHeart. post() is called from thread mode,
 * tick() exclusively from the control ISR.
 */
class DriveControl {
public:
    /** Mask derived from the mailbox depth (power of two, checked below). */
    static constexpr uint32_t REQ_MASK = DRIVE_REQUEST_QUEUE_DEPTH - 1u;
    static_assert((DRIVE_REQUEST_QUEUE_DEPTH & REQ_MASK) == 0u, "Mailbox depth must be a power of two");

    /**
     * @brief Construct the control front end.
     * @param freq_hz Control-loop frequency in Hz; sets the velocity ramp step
     *                and the microsecond clock handed to the executor.
     */
    explicit DriveControl(uint32_t freq_hz) : freq_hz_(freq_hz ? freq_hz : 1u) {
        // Velocity ramp step: how much Q32.32 position one revolution per second
        // advances in a single control pass. Precomputed once so the ISR only
        // multiplies. 2^32 / f gives the Q32 increment per (rev/s) per tick.
        vel_to_pos_q32_ = 4294967296.0f / (float)freq_hz_;
        // Microseconds per control pass in Q16.16, so a non-integer period
        // (any carrier frequency that does not divide 1 MHz) still accumulates
        // an exact microsecond clock over time instead of drifting.
        us_per_tick_q16_ = (uint32_t)((1000000.0f * 65536.0f) / (float)freq_hz_);
    }

    // ------------------------------------------------------------------
    // Thread-side interface (protocol engine, console, any non-ISR caller)
    // ------------------------------------------------------------------

    /**
     * @brief Post one command into the mailbox.
     *
     * Wait-free: a single store publishes the entry. When the mailbox is full
     * the command is rejected so the caller can answer the master with a
     * BUSY/QUEUE_FULL reply instead of silently losing the request.
     *
     * @param req Fully populated request.
     * @return true when the command was accepted.
     */
    /** @brief Tone frequency currently commanded [Hz]. */
    uint16_t tone_freq() const { return tone_freq_hz_; }

    /** @brief Tone amplitude currently commanded, Q15 fraction of bus volts. */
    uint16_t tone_amp() const { return tone_amp_q15_; }

    bool post(const DriveRequest& req) {
        const uint32_t next = (req_w_ + 1u) & REQ_MASK;  // Slot after the one being filled
        if (next == req_r_) {
            req_dropped_++;                              // Mailbox full: report it, drop the command
            return false;
        }
        req_ring_[req_w_] = req;                         // Fill the slot
        __DMB();                                         // Payload visible before the index moves
        req_w_ = next;                                   // Publish the slot to the ISR
        return true;
    }

    /** @brief Convenience wrapper: post a command carrying no argument. */
    bool post(DriveReq op) {
        DriveRequest r;      // Request under construction
        r.op = op;           // Opcode
        r.arg.u32 = 0u;      // Argument unused
        return post(r);
    }

    /** @brief Convenience wrapper: post a command with a 32-bit unsigned argument. */
    bool post_u32(DriveReq op, uint32_t value) {
        DriveRequest r;      // Request under construction
        r.op = op;           // Opcode
        r.arg.u32 = value;   // Unsigned argument
        return post(r);
    }

    /** @brief Convenience wrapper: post a command with a 32-bit signed argument. */
    bool post_i32(DriveReq op, int32_t value) {
        DriveRequest r;      // Request under construction
        r.op = op;           // Opcode
        r.arg.i32 = value;   // Signed argument
        return post(r);
    }

    /** @brief Convenience wrapper: post a command with a 64-bit signed argument. */
    bool post_i64(DriveReq op, int64_t value) {
        DriveRequest r;      // Request under construction
        r.op = op;           // Opcode
        r.arg.i64 = value;   // Wide signed argument (positions)
        return post(r);
    }

    /** @brief Convenience wrapper: post a command with a floating-point argument. */
    bool post_f32(DriveReq op, float value) {
        DriveRequest r;      // Request under construction
        r.op = op;           // Opcode
        r.arg.f32 = value;   // Physical setpoint
        return post(r);
    }

    /**
     * @brief Convenience wrapper: queue one trajectory segment.
     * @param seg      Segment; its displacement is relative to the queue tail.
     * @param absolute true when the displacement field holds an ABSOLUTE
     *                 target instead: the ISR converts it against the tail
     *                 of the queue, which only it knows exactly.
     */
    bool post_segment(const MotionCommandTM& seg, bool absolute = false) {
        DriveRequest r;                                                 // Request under construction
        r.op = absolute ? DriveReq::PushSegmentAbs : DriveReq::PushSegment;  // Opcode
        r.arg.seg = seg;                                                // Whole segment by value
        return post(r);
    }

    /**
     * @brief Reload the master-liveness watchdog.
     *
     * One 32-bit store, safe to call from thread mode while the ISR decrements
     * the same word: the store either lands before or after a decrement, and
     * both orders leave a valid counter.
     */
    void feed_watchdog() { wd_counter_ = wd_reload_; }

    /** @brief Reload value currently armed, in control-loop ticks (0 = disabled). */
    uint32_t watchdog_reload() const { return wd_reload_; }

    /** @brief Control-loop frequency this instance was configured with [Hz]. */
    uint32_t frequency_hz() const { return freq_hz_; }

    /** @brief Commands rejected because the mailbox was full (saturating counter). */
    uint32_t requests_dropped() const { return req_dropped_; }

    /**
     * @brief Mailbox slots still free: how many post() calls succeed before the ISR drains.
     *
     * Lets a caller that must queue a GROUP of requests (a profiled move is up
     * to seven segments) refuse the whole group instead of leaving a partial
     * one in the mailbox. One slot of the ring is always kept empty to tell a
     * full ring from an empty one. Read from thread mode while the ISR
     * advances the consumer index; a stale value only under-reports the room.
     */
    uint32_t request_room() const { return (req_r_ - req_w_ - 1u) & REQ_MASK; }

    /**
     * @brief Read a consistent snapshot of the control state.
     *
     * Sequence lock with the ISR as the writer: the reader retries while a
     * publish is in flight. The ISR itself never waits.
     *
     * @param[out] out Destination snapshot.
     */
    void read_telemetry(DriveTelemetry& out) const {
        uint32_t s1;  // Sequence sampled before the copy
        uint32_t s2;  // Sequence sampled after the copy
        do {
            s1 = tlm_seq_.load(std::memory_order_relaxed);   // Even means "no write in flight"
            std::atomic_thread_fence(std::memory_order_acquire);
            out = tlm_;                                      // Copy the published snapshot
            std::atomic_thread_fence(std::memory_order_acquire);
            s2 = tlm_seq_.load(std::memory_order_relaxed);   // Unchanged means the copy is intact
        } while ((s1 != s2) || ((s1 & 1u) != 0u));
    }

    // ------------------------------------------------------------------
    // ISR-side interface (control loop only)
    // ------------------------------------------------------------------

    /**
     * @brief Resolve one control pass: drain commands, advance the trajectory,
     *        and hand the control loop its targets.
     *
     * @param measured_pos_q16 Measured mechanical position [rev], Q16.16.
     * @param applied_current_a Current commanded on the previous pass [A]; feeds
     *                          the executor torque trigger and the telemetry.
     * @param io_bits          Endstop / probe input state for the IO triggers.
     * @param inhibit          true when the control loop cannot energize this
     *                         pass (an NVM programming window is open). Commands
     *                         and the watchdog still run, but trajectory time is
     *                         frozen and a zero-current command is returned, so a
     *                         flash stall never advances a queued trajectory.
     * @return Resolved targets plus the action the control loop must take.
     */
    DriveOutput tick(int32_t measured_pos_q16, float applied_current_a, uint8_t io_bits, bool inhibit = false) {
        drain_requests();                                     // Apply everything the master asked for
        if (!inhibit) {
            now_us_q16_ += us_per_tick_q16_;                  // Advance the exact microsecond clock
        }
        const uint32_t now_us = (uint32_t)(now_us_q16_ >> 16);  // Whole microseconds for the executor

        // ---- Master-liveness watchdog -----------------------------------
        // Decrement only while armed; once it hits zero the drive is latched
        // into its safe state until a new command arrives (which reloads it).
        if (wd_reload_ != 0u) {
            if (wd_counter_ != 0u) {
                wd_counter_--;                                // Still hearing from the master
                wd_expired_ = false;                          // Clear a previous timeout
            } else {
                wd_expired_ = true;                           // Master went quiet
            }
        } else {
            wd_expired_ = false;                              // Watchdog disabled
        }

        DriveOutput out;                                      // Result of this pass
        out.target_vel_rps = 0.0f;                            // Default: no velocity feed-forward
        out.max_current_a = current_limit_a_;                 // Default: full configured envelope
        out.torque_target_a = 0.0f;                           // Default: no direct torque command
        out.tone_freq_hz = 0u;                                // Default: silent
        out.tone_amp_q15 = 0u;                                // Default: silent

        // ---- NVM programming window --------------------------------------
        // The caller owns a de-energized window and will publish it to the
        // flash engines. Hold every target on the measured position and command
        // zero current, exactly as the control loop did before this layer
        // existed, so the published window keeps describing the real vector.
        if (inhibit) {
            target_pos_q32_ = (int64_t)measured_pos_q16 << 16;
            queue_tail_q32_ = target_pos_q32_;             // A move planned now starts from here
            exec_.update_real_pos(target_pos_q32_);
            out.action = DriveAction::RunTorque;
            out.target_pos_q16 = measured_pos_q16;
            out.torque_target_a = 0.0f;
            publish(measured_pos_q16, out, applied_current_a);
            return out;
        }

        // ---- Safe-state gate --------------------------------------------
        // Disabled by command, tripped by the watchdog, or parked by a stop:
        // every one of them resolves through the same configured safe state,
        // so there is exactly one place where the drive can be de-energized.
        const bool inhibited = (!enabled_) || wd_expired_ || (mode_ == DriveMode::Disabled);
        if (inhibited) {
            // Keep every target glued to the measured position so that
            // re-enabling the drive can never produce a step command.
            target_pos_q32_ = (int64_t)measured_pos_q16 << 16;
            queue_tail_q32_ = target_pos_q32_;             // A move planned while parked starts from here
            exec_.update_real_pos(target_pos_q32_);
            out.target_pos_q16 = measured_pos_q16;
            switch (safe_state_) {
                case DriveSafeState::Brake:
                    out.action = DriveAction::Brake;          // Short the phases (low side on)
                    break;
                case DriveSafeState::Hold:
                    out.action = DriveAction::RunPosition;    // Stay stiff at the captured position
                    break;
                case DriveSafeState::Freewheel:
                default:
                    out.action = DriveAction::Freewheel;      // Release the bridge
                    break;
            }
            publish(measured_pos_q16, out, applied_current_a);
            return out;
        }

        // ---- Active modes -----------------------------------------------
        switch (mode_) {
            case DriveMode::Tone: {
                // The coils are a speaker this pass: no regulator runs, and the
                // caller drives the bridge straight from the tone generator.
                // The position basis still tracks reality so leaving tone mode
                // does not command a jump.
                target_pos_q32_ = (int64_t)measured_pos_q16 << 16;
                exec_.update_real_pos(target_pos_q32_);
                out.action = DriveAction::Tone;
                out.target_pos_q16 = measured_pos_q16;
                out.tone_freq_hz = tone_freq_hz_;
                out.tone_amp_q15 = tone_amp_q15_;
                break;
            }
            case DriveMode::Torque: {
                // Open the position loop entirely; keep the position target
                // tracking reality so a later mode switch is bumpless.
                target_pos_q32_ = (int64_t)measured_pos_q16 << 16;
                exec_.update_real_pos(target_pos_q32_);
                out.action = DriveAction::RunTorque;
                out.target_pos_q16 = measured_pos_q16;
                out.torque_target_a = clamp_current(target_current_a_);
                break;
            }
            case DriveMode::Velocity: {
                // Ramp the position target at the commanded rate. The rate is
                // slewed towards the request by the acceleration envelope, so
                // a step in the velocity command never becomes a torque step.
                ramp_velocity();
                // The per-pass step is converted through int32_t on purpose: a direct
                // float -> int64_t cast is __aeabi_f2lz, which libgcc implements with
                // double arithmetic (2 KiB of soft-float). vel_to_pos_q32_ is 2^32 / f_ctrl
                // (214748 per rev/s at 20 kHz), so int32_t covers |v| < 10000 rev/s.
                const int32_t step_q32 = (int32_t)(active_vel_rps_ * vel_to_pos_q32_);  // Q32.32 increment, one vcvt
                target_pos_q32_ += step_q32;                                              // Sign-extended into the Q32.32 basis
                exec_.update_real_pos(target_pos_q32_);
                out.action = DriveAction::RunPosition;
                out.target_pos_q16 = (int32_t)(target_pos_q32_ >> 16);
                out.target_vel_rps = active_vel_rps_;
                break;
            }
            case DriveMode::Position: {
                // Straight position command; the profile is whatever the
                // master streams in, so no ramp is applied here.
                exec_.update_real_pos((int64_t)measured_pos_q16 << 16);
                out.action = DriveAction::RunPosition;
                out.target_pos_q16 = (int32_t)(target_pos_q32_ >> 16);
                break;
            }
            case DriveMode::Motion: {
                // Trajectory queue. The executor owns the position basis, so
                // it is told the measured position every pass and returns the
                // interpolated target for this instant.
                exec_.update_real_pos((int64_t)measured_pos_q16 << 16);
                const uint16_t torque_norm = current_to_norm(applied_current_a);  // Torque trigger input
                bool halt = false;                            // Set when the queue ran dry
                bool warn = false;                            // Set when one segment is left
                const MotionExecutorOutput mo =
                    exec_.tick(now_us, (int16_t)time_scale_q10_, torque_norm, io_bits, halt, warn);
                exec_halted_ = halt;                          // Remember for telemetry
                exec_starving_ = warn;                        // Remember for telemetry
                target_pos_q32_ = mo.position_q32;            // Adopt the interpolated basis
                out.action = DriveAction::RunPosition;
                out.target_pos_q16 = (int32_t)(target_pos_q32_ >> 16);
                // A NaN velocity means "executor disabled"; treat it as zero so
                // the damper reference can never be poisoned.
                out.target_vel_rps = (mo.velocity_rps == mo.velocity_rps) ? mo.velocity_rps : 0.0f;
                // Per-segment torque limit scales the envelope for this pass.
                // A HALTED executor reports zero torque (it has no segment to
                // take a limit from), which would collapse the envelope to zero
                // and let a loaded axis fall the moment the queue ran dry. A
                // starved queue must HOLD the last position, so the full
                // envelope is kept whenever the executor is halted.
                out.max_current_a = current_limit_a_ * (halt ? 1.0f : mo.torque);
                break;
            }
            case DriveMode::Disabled:
            default: {
                // Unreachable: the safe-state gate above already handled it.
                out.action = DriveAction::Freewheel;
                out.target_pos_q16 = measured_pos_q16;
                break;
            }
        }

        // The queue tail is where the NEXT queued segment starts, and the
        // thread side sizes a profiled move from it before the segment even
        // reaches this ISR. Outside Motion mode the target moves without the
        // queue (a velocity ramp, a setpoint, a torque command tracking the
        // rotor), and a drained queue holds wherever it stopped; in both
        // cases the tail must follow the target, or the first S-curve after a
        // jog is sized from a stale point (a 25 rev move planned as a 0 rev
        // one collapses into a step the loop chases at full speed).
        if ((mode_ != DriveMode::Motion) || exec_halted_) {
            queue_tail_q32_ = target_pos_q32_;             // Nothing queued: the tail is the target
        }

        publish(measured_pos_q16, out, applied_current_a);
        return out;
    }

    /**
     * @brief Re-seed every target from the measured position.
     *
     * Called by the control loop whenever the position loop is (re-)armed, for
     * example right after the angle calibration commits, so the drive holds
     * where it is instead of snapping to a stale target.
     *
     * @param measured_pos_q16 Position to adopt [rev], Q16.16.
     * @param now_us           Current microsecond clock for the executor.
     */
    void rearm(int32_t measured_pos_q16, uint32_t now_us) {
        target_pos_q32_ = (int64_t)measured_pos_q16 << 16;  // Adopt reality as the target
        active_vel_rps_ = 0.0f;                             // No residual ramp
        target_vel_rps_ = 0.0f;                             // No pending velocity request
        exec_.init(now_us, target_pos_q32_);                // Rebase the trajectory queue
        queue_tail_q32_ = target_pos_q32_;                  // Nothing queued: the tail is the target
        exec_halted_ = false;                               // Fresh queue is not halted
        exec_starving_ = false;                             // ... and not starving
    }

    /** @brief Microsecond clock derived from the control-loop tick count. */
    uint32_t now_us() const { return (uint32_t)(now_us_q16_ >> 16); }

private:
    // ------------------------------------------------------------------
    // Command application (ISR context only)
    // ------------------------------------------------------------------

    /** @brief Apply every command waiting in the mailbox. */
    void drain_requests() {
        while (req_r_ != req_w_) {                 // While the producer published entries
            const DriveRequest r = req_ring_[req_r_];  // Copy out before releasing the slot
            __DMB();                               // Payload read before the index moves
            req_r_ = (req_r_ + 1u) & REQ_MASK;     // Release the slot back to the producer
            apply(r);                              // Execute it
        }
    }

    /**
     * @brief Execute one mailbox command.
     * @param r Request to apply.
     */
    void apply(const DriveRequest& r) {
        switch (r.op) {
            case DriveReq::SetTone: {
                // Frequency and amplitude travel together so a note change is
                // one mailbox slot and can never be torn across two passes.
                tone_freq_hz_ = (uint16_t)(r.arg.u32 & 0xFFFFu);
                tone_amp_q15_ = (uint16_t)(r.arg.u32 >> 16);
                break;
            }
            case DriveReq::SetMode: {
                const DriveMode m = (DriveMode)r.arg.u32;  // Requested mode
                if (m != mode_) {
                    enter_mode(m);                         // Bumpless transition
                }
                break;
            }
            case DriveReq::SetEnable:
                enabled_ = (r.arg.u32 != 0u);              // Arm or disarm the output stage
                if (enabled_) {
                    // Re-arming must not inherit a stale target: the safe-state
                    // gate has been holding every target at the measured
                    // position, so nothing else is needed here.
                    wd_counter_ = wd_reload_;              // Give the master a fresh window
                }
                break;
            case DriveReq::TargetPosAbs:
                target_pos_q32_ = r.arg.i64 << 16;         // Q16.16 command -> Q32.32 basis
                queue_tail_q32_ = target_pos_q32_;         // A fresh setpoint is where a queue would start
                break;
            case DriveReq::TargetPosRel:
                target_pos_q32_ += r.arg.i64 << 16;        // Displace the existing target
                queue_tail_q32_ = target_pos_q32_;         // Keep the queue tail on the target
                break;
            case DriveReq::TargetVel:
                target_vel_rps_ = clamp_velocity(r.arg.f32);  // Envelope-limited velocity request
                break;
            case DriveReq::TargetCurrent:
                target_current_a_ = r.arg.f32;             // Direct torque command
                break;
            case DriveReq::CurrentLimit:
                if (r.arg.f32 > 0.0f) {
                    current_limit_a_ = r.arg.f32;          // New current envelope
                }
                break;
            case DriveReq::VelocityLimit:
                if (r.arg.f32 > 0.0f) {
                    vel_limit_rps_ = r.arg.f32;            // New velocity envelope
                }
                break;
            case DriveReq::AccelLimit:
                if (r.arg.f32 > 0.0f) {
                    accel_limit_rps2_ = r.arg.f32;         // New acceleration envelope
                }
                break;
            case DriveReq::JerkLimit:
                if (r.arg.f32 > 0.0f) {
                    jerk_limit_rps3_ = r.arg.f32;          // Stored for the profile generators
                }
                break;
            case DriveReq::Stop:
                apply_stop((uint8_t)r.arg.u32);            // Controlled stop with an end state
                break;
            case DriveReq::PushSegment:
                push_segment(r.arg.seg);                   // Relative to the queue tail
                break;
            case DriveReq::PushSegmentAbs: {
                // The wire carried an absolute target; the queue works in
                // displacements from wherever the previous segment ends, and
                // only this side knows that point exactly.
                MotionCommandTM seg = r.arg.seg;           // Copy: the displacement gets rewritten
                seg.motion.displacement = seg.motion.displacement - (int32_t)(queue_tail_q32_ >> 16);
                push_segment(seg);
                break;
            }
            case DriveReq::QueueClear:
                exec_.init(now_us(), target_pos_q32_);     // Drop the queue, hold where we are
                queue_tail_q32_ = target_pos_q32_;         // Nothing queued: the tail is the target
                exec_halted_ = false;                      // Fresh queue is not halted
                exec_starving_ = false;                    // ... and not starving
                break;
            case DriveReq::QueuePause:
                exec_.set_pause(r.arg.u32 != 0u);          // Freeze or resume trajectory time
                paused_ = (r.arg.u32 != 0u);               // Mirror it for telemetry
                break;
            case DriveReq::TimeScale:
                time_scale_q10_ = clamp_time_scale(r.arg.i32);  // Playback rate, Q10
                break;
            case DriveReq::SetSafeState:
                if (r.arg.u32 <= (uint32_t)DriveSafeState::Hold) {
                    safe_state_ = (DriveSafeState)r.arg.u32;    // New de-energized behaviour
                }
                break;
            case DriveReq::ArmWatchdog:
                wd_reload_ = r.arg.u32;                    // Reload value in control ticks
                wd_counter_ = r.arg.u32;                   // Start a fresh window immediately
                wd_expired_ = false;                       // Arming clears a previous timeout
                break;
            case DriveReq::None:
            default:
                break;                                     // Nothing to do
        }
    }

    /**
     * @brief Switch control modes without producing a torque step.
     *
     * Every mode entry re-seeds the state the new mode integrates from, so the
     * first pass in the new mode commands exactly what the last pass of the
     * old mode did.
     *
     * @param m Mode to enter.
     */
    void enter_mode(DriveMode m) {
        switch (m) {
            case DriveMode::Velocity:
                // Start the ramp from the velocity that is actually being
                // tracked right now rather than from zero.
                target_vel_rps_ = 0.0f;
                break;
            case DriveMode::Motion:
                // Rebase the trajectory queue onto the present target so the
                // first segment starts exactly where the drive already is.
                exec_.init(now_us(), target_pos_q32_);
                queue_tail_q32_ = target_pos_q32_;
                exec_halted_ = false;
                exec_starving_ = false;
                break;
            case DriveMode::Torque:
                target_current_a_ = 0.0f;   // Never inherit a torque command across a mode switch
                tone_freq_hz_ = 0u;         // ... nor a note that would keep the coils singing
                break;
            case DriveMode::Position:
            case DriveMode::Disabled:
            default:
                break;                      // Targets already track the measured position
        }
        mode_ = m;                          // Commit the new mode
    }

    /**
     * @brief Execute a controlled stop.
     * @param stop_mode Wire-level stop style (see TpStopMode).
     */
    /**
     * @note A stop that names an end state (FREEWHEEL / BRAKE) also becomes the
     *       drive's RUNTIME safe state, so a later disable behaves the same way
     *       the operator just asked for. It does not touch the persisted
     *       configuration: the next SetSafeState or configuration apply
     *       restores the configured policy.
     */
    void apply_stop(uint8_t stop_mode) {
        exec_.init(now_us(), target_pos_q32_);  // Always drop the pending trajectory
        queue_tail_q32_ = target_pos_q32_;      // Nothing queued: the tail is the target
        exec_halted_ = false;                   // Fresh queue is not halted
        exec_starving_ = false;                 // ... and not starving
        target_vel_rps_ = 0.0f;                 // Cancel any velocity request
        active_vel_rps_ = 0.0f;                 // ... and the ramp that tracks it
        switch (stop_mode) {
            case 1u:  // TP_STOP_FREEWHEEL
                safe_state_ = DriveSafeState::Freewheel;  // Coast
                mode_ = DriveMode::Disabled;              // De-energize through the safe-state gate
                break;
            case 2u:  // TP_STOP_BRAKE
                safe_state_ = DriveSafeState::Brake;      // Short the phases
                mode_ = DriveMode::Disabled;              // De-energize through the safe-state gate
                break;
            case 0u:  // TP_STOP_HOLD
            case 3u:  // TP_STOP_QUICK
            default:
                mode_ = DriveMode::Position;              // Stay stiff at the present target
                break;
        }
    }

    // ------------------------------------------------------------------
    // Helpers
    // ------------------------------------------------------------------

    /**
     * @brief Queue one segment and advance the queue tail by its displacement.
     *
     * The tail is where the LAST queued segment ends: the origin every later
     * relative segment and every absolute-to-relative conversion measures
     * from. A FREERUN segment ends on a trigger rather than at its
     * displacement, so the tail is an estimate for those; a master that
     * streams absolute targets after a triggered segment re-anchors it with
     * the next absolute push anyway.
     *
     * @param seg Segment with a RELATIVE displacement.
     */
    void push_segment(const MotionCommandTM& seg) {
        if (!exec_.push(seg)) {
            segments_dropped_++;                                   // Executor queue full: count it
            return;
        }
        if (seg.mode.getMode() != MotionMode::ANCHOR) {
            queue_tail_q32_ += (int64_t)seg.motion.displacement << 16;  // The segment ends here
        }
    }

    /** @brief Slew the tracked velocity towards the request under the accel limit. */
    void ramp_velocity() {
        const float step = accel_limit_rps2_ / (float)freq_hz_;  // Largest change per control pass
        const float err = target_vel_rps_ - active_vel_rps_;     // Remaining difference
        if (err > step) {
            active_vel_rps_ += step;        // Accelerate at the limit
        } else if (err < -step) {
            active_vel_rps_ -= step;        // Decelerate at the limit
        } else {
            active_vel_rps_ = target_vel_rps_;  // Close enough: land exactly on the request
        }
    }

    /** @brief Clamp a current command into the configured envelope. */
    float clamp_current(float a) const {
        if (a > current_limit_a_) {
            return current_limit_a_;    // Saturate positive
        }
        if (a < -current_limit_a_) {
            return -current_limit_a_;   // Saturate negative
        }
        return a;
    }

    /** @brief Clamp a velocity command into the configured envelope. */
    float clamp_velocity(float v) const {
        if (v > vel_limit_rps_) {
            return vel_limit_rps_;      // Saturate positive
        }
        if (v < -vel_limit_rps_) {
            return -vel_limit_rps_;     // Saturate negative
        }
        return v;
    }

    /** @brief Clamp a Q10 time scale into the range the executor accepts (int16). */
    static int32_t clamp_time_scale(int32_t q10) {
        if (q10 > 32767) {
            return 32767;               // +32x playback
        }
        if (q10 < -32768) {
            return -32768;              // -32x playback (reverse)
        }
        return q10;
    }

    /**
     * @brief Map an applied current onto the 0..65535 torque scale the executor
     *        compares its torque triggers against.
     * @param a Applied q-axis current [A].
     */
    uint16_t current_to_norm(float a) const {
        const float mag = (a < 0.0f) ? -a : a;                   // Trigger on magnitude
        const float lim = (current_limit_a_ > 0.0f) ? current_limit_a_ : 1.0f;  // Avoid a divide by zero
        float norm = (mag / lim) * 65535.0f;                     // Scale into the trigger domain
        if (norm > 65535.0f) {
            norm = 65535.0f;                                     // Saturate at full scale
        }
        return (uint16_t)norm;
    }

    /**
     * @brief Publish the telemetry snapshot for the thread side.
     *
     * Sequence lock: the counter goes odd before the copy and even after it, so
     * a reader that sees an unchanged even counter around its own copy knows
     * the data is consistent.
     */
    void publish(int32_t measured_pos_q16, const DriveOutput& out, float applied_current_a) {
        tlm_seq_.fetch_add(1, std::memory_order_relaxed);    // Odd: write in progress
        std::atomic_thread_fence(std::memory_order_release);

        tlm_.position_q16 = measured_pos_q16;                // Where the rotor actually is
        tlm_.target_pos_q16 = out.target_pos_q16;            // Where it is being told to be
        tlm_.queue_tail_q16 = (int32_t)(queue_tail_q32_ >> 16);  // Where the queue currently ends
        tlm_.target_vel_rps = out.target_vel_rps;            // Velocity reference in force
        tlm_.applied_current_a = applied_current_a;          // Torque commanded last pass
        tlm_.mode = (uint8_t)mode_;                          // Active control mode
        tlm_.motion_mode = exec_mode_code();                 // Segment kind being executed
        tlm_.queue_depth = queue_depth();                    // Segments still to run
        tlm_.queue_free = queue_free();                      // Slots a host may still fill
        tlm_.dropped_segments = (uint16_t)segments_dropped_; // Segments lost to a full queue
        tlm_.time_scale_q10 = (int16_t)time_scale_q10_;      // Playback rate

        uint8_t flags = 0;                                   // Assembled status bits
        if (enabled_) {
            flags |= DRIVE_TLM_ENABLED;                      // Output stage armed
        }
        if (wd_expired_) {
            flags |= DRIVE_TLM_WATCHDOG;                     // Master went quiet
        }
        if (paused_) {
            flags |= DRIVE_TLM_PAUSED;                       // Trajectory time frozen
        }
        if (exec_halted_) {
            flags |= DRIVE_TLM_HALTED;                       // Queue ran dry
        }
        if (exec_starving_) {
            flags |= DRIVE_TLM_STARVING;                     // One segment left
        }
        if ((out.target_pos_q16 != measured_pos_q16) && (mode_ != DriveMode::Disabled)) {
            flags |= DRIVE_TLM_MOVING;                       // A target/measured gap means motion
        }
        if (out.action == DriveAction::RunPosition) {
            // The observer inside the position controller only advances when
            // that controller is actually ticked. Freewheeling or braking
            // leaves its velocity and acceleration frozen at whatever they were
            // when the loop last ran, so telemetry has to say when they are
            // stale rather than report a spinning rotor that has long stopped.
            flags |= DRIVE_TLM_LOOP_LIVE;
        }
        tlm_.flags = flags;

        std::atomic_thread_fence(std::memory_order_release);
        tlm_seq_.fetch_add(1, std::memory_order_relaxed);    // Even: snapshot complete
    }

    /** @brief Segments waiting in the executor queue, saturated to a byte. */
    uint8_t queue_depth() const {
        const size_t n = exec_.queued();  // Future segments held by the executor
        return (n > 255u) ? 255u : (uint8_t)n;
    }

    /**
     * @brief Slots the queue can still accept, saturated to a byte.
     *
     * Taken from the executor rather than derived as depth-minus-capacity: the
     * queue depth counts only segments waiting BEHIND the one executing, and
     * the ring also retains history behind the cursor, so subtracting it from
     * the nominal depth over-reports free space by at least one - exactly when
     * the queue is full and a host is relying on the number to pace itself.
     */
    uint8_t queue_free() const {
        const size_t n = exec_.free_slots();  // True free count from the ring itself
        return (n > 255u) ? 255u : (uint8_t)n;
    }

    /** @brief Numeric MotionMode of the executing segment, 0xFF while halted. */
    uint8_t exec_mode_code() const { return exec_.active_mode_code(); }

    // ------------------------------------------------------------------
    // State (ISR-owned unless noted)
    // ------------------------------------------------------------------

    const uint32_t freq_hz_;            ///< Control-loop frequency [Hz].
    float    vel_to_pos_q32_ = 0.0f;    ///< Q32.32 position step per (rev/s) per control pass.
    uint32_t us_per_tick_q16_ = 0u;     ///< Microseconds per control pass, Q16.16.
    uint64_t now_us_q16_ = 0u;          ///< Accumulated microsecond clock, Q16.16.

    DriveMode      mode_ = DriveMode::Disabled;             ///< Active control mode.
    DriveSafeState safe_state_ = DriveSafeState::Freewheel; ///< Behaviour while de-energized.
    bool           enabled_ = false;                        ///< Output stage armed by command.
    bool           paused_ = false;                         ///< Trajectory playback paused.

    int64_t target_pos_q32_ = 0;        ///< Position target [rev], Q32.32 (the executor's basis).
    int64_t queue_tail_q32_ = 0;        ///< Position the last queued segment ends at [rev], Q32.32.
    float   target_vel_rps_ = 0.0f;     ///< Requested velocity [RPS].
    float   active_vel_rps_ = 0.0f;     ///< Velocity actually being tracked after the ramp [RPS].
    float   target_current_a_ = 0.0f;   ///< Direct q-axis current command [A].
    uint16_t tone_freq_hz_ = 0u;        ///< Tone frequency [Hz]; 0 is silence.
    uint16_t tone_amp_q15_ = 0u;        ///< Tone amplitude, Q15 fraction of bus volts.

    float current_limit_a_ = 1.0f;      ///< Current envelope [A].
    float vel_limit_rps_ = 50.0f;       ///< Velocity envelope [RPS].
    float accel_limit_rps2_ = 200.0f;   ///< Acceleration envelope [RPS^2].
    float jerk_limit_rps3_ = 2000.0f;   ///< Jerk envelope [RPS^3], used by profile generation.
    int32_t time_scale_q10_ = 1024;     ///< Trajectory playback rate, Q10 (1024 = 1.0x).

    MotionExecutor<DRIVE_MOTION_QUEUE_DEPTH> exec_;  ///< Trajectory queue and interpolator.
    bool exec_halted_ = false;          ///< Executor ran out of segments.
    bool exec_starving_ = false;        ///< One segment left in the queue.
    uint32_t segments_dropped_ = 0u;    ///< Segments rejected by a full executor queue.

    volatile uint32_t wd_reload_ = 0u;   ///< Watchdog reload [control ticks], 0 = disabled.
    volatile uint32_t wd_counter_ = 0u;  ///< Watchdog countdown, reloaded from thread mode.
    bool wd_expired_ = false;            ///< Latched timeout state.

    // ---- Command mailbox (producer: thread, consumer: ISR) ---------------
    DriveRequest req_ring_[DRIVE_REQUEST_QUEUE_DEPTH];  ///< Pending commands.
    volatile uint32_t req_w_ = 0u;       ///< Producer index (thread mode).
    volatile uint32_t req_r_ = 0u;       ///< Consumer index (ISR).
    volatile uint32_t req_dropped_ = 0u; ///< Commands lost to a full mailbox.

    // ---- Telemetry snapshot (producer: ISR, consumer: thread) ------------
    DriveTelemetry tlm_ = {};                   ///< Published control state.
    mutable std::atomic<uint32_t> tlm_seq_{0};  ///< Sequence lock guarding tlm_.
};

/** @brief Public read-only view of DriveControl state used by the API layer. */
using DriveState = DriveTelemetry;
