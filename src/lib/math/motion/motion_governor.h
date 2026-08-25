/**
 * @file motion_governor.h
 * @brief Time-scaling governor for the motion pipeline.
 *
 * Converts a desired feed-rate/speed override into the integer `time_scale`
 * consumed by MotionExecutor::tick(). Positive values play segments forward,
 * negative values reverse them (only allowed when the active segment is
 * reversible), zero freezes trajectory time. The governor implements smooth
 * ramping between scale values so that speed changes do not produce torque
 * steps at the position loop input.
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

// #pragma once

// #include "../../common.h"
// #include <stdint.h>
// #include <math.h>

// #include "observer.h"         
// #include "motion_executor.h" 


// // ============================================================================
// // MOTION GOVERNOR CLASS
// // ============================================================================

// template <size_t QueueSize>
// class MotionGovernor {
// private:
//     LinearObserver observer;
//     // TrajectoryExecutor<QueueSize> executor;  // TEMPORARILY DISABLED: TrajectoryExecutor not defined

//     // Dynamic bandwidth parameters (Bandwidth Scheduling)
//     float min_observer_fc;
//     float max_observer_fc;
//     float fc_transition_vel; // Velocity (rev/s) at which bandwidth reaches its maximum

//     int32_t current_time_scale_q10; // Q10: 1024 = 1.0x (normal time flow)

//     /// @brief Dynamically computes the LESO bandwidth from the target velocity
//     inline float schedule_observer_bandwidth(float target_vel) {
//         float abs_vel = fabsf(target_vel);
        
//         // At low speeds encoder noise is worst due to quantization.
//         // Therefore the bandwidth narrows down to min_observer_fc. 
//         if (abs_vel >= fc_transition_vel) {
//             return max_observer_fc;
//         }

//         // Linear interpolation between min and max depending on speed
//         float progress = abs_vel / fc_transition_vel;
//         return min_observer_fc + (max_observer_fc - min_observer_fc) * progress;
//     }

// public:
//     MotionGovernor(uint32_t freq_hz, float inertia_b0 = 0.0f) 
//         : observer(freq_hz, inertia_b0), 
//           // executor(),  // TEMPORARILY DISABLED
//           min_observer_fc(50.0f),       // 50 Hz default (standstill)
//           max_observer_fc(250.0f),      // 250 Hz default (fast rotation)
//           fc_transition_vel(5.0f),      // Switch to maximum bandwidth at 5 rev/s
//           current_time_scale_q10(1024)  // Normal time flow (1.0x)
//     {
//     }

//     // --- Parameter configuration ---

//     /// @brief Forcibly sets the time-flow scale (used for overrides)
//     /// @param scale_q10 1024 = 1.0x, -1024 = reverse, 0 = pause, 512 = 0.5x
//     inline void set_time_scale(int32_t scale_q10) {
//         current_time_scale_q10 = scale_q10;
//     }

//     /// @brief Returns the current (recommended) time scale for the system timer
//     inline int32_t get_recommended_time_scale() const {
//         // Future: autonomous trajectory braking logic could live here,
//         // if Actual Pos lags Target Pos too far.
//         // For now we simply return the externally supplied scale.
//         return current_time_scale_q10;
//     }

//     /// @brief Configure dynamic filter-bandwidth change limits
//     inline void set_observer_bandwidth_limits(float min_fc, float max_fc, float transition_vel) {
//         min_observer_fc = min_fc;
//         max_observer_fc = max_fc;
//         fc_transition_vel = transition_vel;
//     }
    
//     // --- Access to internal modules ---

//     inline LinearObserver& get_observer() { return observer; }
//     // inline TrajectoryExecutor<QueueSize>& get_executor() { return executor; }  // TEMPORARILY DISABLED

//     // --- Trajectory queue control ---

//     inline bool enqueue_command(const MotionCommand& cmd) {
//         // return executor.enqueue_command(cmd);  // TEMPORARILY DISABLED
//         return false;
//     }

//     inline void clear_queue() {
//         // executor.clear_queue();  // TEMPORARILY DISABLED
//     }

//     /// @brief Returns true while a displacement polynomial is currently executing
//     inline bool is_trajectory_active() const {
//         // return executor.is_active(); // Requires an is_active() getter in executor  // TEMPORARILY DISABLED
//         return false;
//     }

//     /// @brief Helper method loading a smooth profile for a 20-revolution move
//     void inject_dummy_scurve() {
//         // TEMPORARILY DISABLED
//         // MotionCommand cmd;
//         // cmd.displacement = 20 * 65536; // 20 revolutions (Q16.16)
//         // cmd.velocity = 5.0f;           // 5 rev/s
//         // cmd.acceleration = 3.0f;       // 3 rev/s^2
//         // cmd.jerk = 1.0f;               // 1 rev/s^3
//         // cmd.torque = 0.0f;
//         // executor.enqueue_command(cmd);
//     }

//     // --- Main computation loop ---

//     /// @brief Governor main method. Call on every hardware control-loop tick.
//     /// @param motion_time_us Virtual time from the system timer
//     /// @param real_pos_q32_32 Raw accumulated encoder value (Q32.32)
//     /// @param applied_torque_ff Torque (Iq current) applied on the PREVIOUS tick
//     /// @return Structure with target and actual values
//     inline __attribute__((always_inline)) ObserverState tick(  // TEMPORARILY CHANGED RETURN TYPE FROM GovernorOutput
//         uint32_t motion_time_us, 
//         int64_t real_pos_q32_32, 
//         float applied_torque_ff) 
//     {
//         // TEMPORARILY SIMPLIFIED - just return observer state
//         // 2. Compute a safe LESO bandwidth from the TARGET velocity
//         float active_fc = schedule_observer_bandwidth(0.0f);  // Use 0 velocity for now

//         // 3. Estimate the actual physics via the LESO
//         ObserverState obs_state = observer.tick(real_pos_q32_32, applied_torque_ff, active_fc);

//         return obs_state;  // Return observer state directly
//     }

// public:
//         /// @brief Generates an ideal 4-phase S-curve for a short move.
//     /// @param current_motion_us Current virtual timer time used for a seamless start
//     void inject_dummy_scurve(uint32_t current_motion_us) {
//         // TEMPORARILY DISABLED - TrajectoryExecutor not available
//         // Original code commented out
//     }
// };