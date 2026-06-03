/*
 * Copyright (c) 2026 creapunk. All rights reserved.
 * Licensed under the Apache License, Version 2.0
 *
 * Detailed functional description:
 * - What the code does: Implements a 3rd-Order Linear Extended State Observer (LESO).
 *   It estimates a motor's internal position, velocity, and load disturbance based 
 *   on raw encoder readings and the known applied control torque.
 * - Why it exists: To replace noisy discrete derivatives of encoder signals with 
 *   smooth, delay-free velocity and acceleration estimates required by advanced 
 *   control algorithms like LADRC (Linear Active Disturbance Rejection Control) 
 *   and Impedance control.
 * - How it fits into the overall system: Acts as the primary kinematic estimator 
 *   in the motor control loop. It consumes raw encoder data at the hardware tick 
 *   rate and feeds clean state data back to the motion controller.
 * - Key algorithms or techniques used: 
 *   - Luenberger-style state observer 
 *   - Forward Euler numerical integration
 *   - Bandwidth-parameterized observer gains
 */

#pragma once // Prevent multiple inclusions of this header file

#include "../../common.h" // Include common system-wide definitions and macros
#include <stdint.h>       // Include standard integer types for fixed-width data
#include <math.h>         // Include math library for floating-point constants

/// @brief State vector representing the estimates produced by the Extended State Observer.
struct ObserverState { // Define a structure to hold the output estimates of the observer
    int64_t pos_q32;   // Absolute estimated position in Q32.32 format (upper 32 bits = rounds, lower 32 = angle)
    float vel;         // Estimated physical velocity of the rotor (in RPS or Rad/s depending on input scale)
    float acc;         // Total estimated acceleration (sum of external disturbance and known control input)
    float dist;        // Estimated generalized disturbance (z3 in LADRC), representing load torque and unmodeled dynamics
}; // End of ObserverState structure definition

/**
 * @brief 3rd-Order Linear Extended State Observer (LESO) for motion control.
 * 
 * High-level overview:
 * The LESO tracks the position of a motor and predicts its velocity and internal
 * disturbances. It uses the known control input (torque) and the error between 
 * the estimated position and actual position to continuously correct its internal state.
 * 
 * Step-by-step logic flow:
 * 1. Read raw encoder position and calculate the tracking error.
 * 2. Calculate the observer bandwidth limits to ensure integration stability.
 * 3. Compute error correction gains (l1, l2, l3) based on the bandwidth.
 * 4. Apply the forward Euler method to calculate state derivatives.
 * 5. Integrate derivatives to update internal velocity, disturbance, and position.
 * 6. Transform internal continuous tracking variables to the user-requested coordinate frame.
 * 
 * Mathematical formulas:
 * Tracking Error: e = Position_actual - Position_estimated
 * d(Position)/dt = Velocity_estimated + l1 * e
 * d(Velocity)/dt = Disturbance_estimated + b0 * Torque_applied + l2 * e
 * d(Disturbance)/dt = l3 * e
 * 
 * Algorithm complexity: O(1) per step (constant time execution suitable for real-time ISR).
 * Edge cases: Handles encoder wrap-around natively using 32-bit integer overflow mechanics.
 * 
 * Reference: "Active Disturbance Rejection Control" by J. Han / Zhiqiang Gao.
 */
class LinearObserver { // Define the LinearObserver class
private: // Begin private access modifier block
    // --- Internal Mathematical State --- // Section marker for internal variables
    int64_t internal_pos_q32; // Uninterrupted absolute internal accumulator for position
    int32_t internal_pos_i32 = 0;
    
    float vel;  // Internal estimated velocity variable
    float dist; // Internal estimated disturbance variable
    float acc;  // Internal estimated total acceleration variable

    // --- Output Coordinate Frame --- // Section marker for output mapping variables
    int64_t pos_offset_q32; // Offset applied to the internal position to form the output position
    bool is_reversed;       // Flag to invert the output direction of all kinematic variables
    bool is_initialized;    // Flag indicating if the observer has synced with the initial encoder value

    float dt;          // Integration time step in seconds (1.0 / frequency)
    float current_b0;  // System gain coefficient relating control input to acceleration
    float max_safe_fc; // Maximum allowable bandwidth frequency clamped for Euler stability

public: // Begin public access modifier block
    /// @brief Constructor initializing the observer with operating frequency and system inertia.
    explicit LinearObserver(uint32_t freq_hz, float initial_b0 = 0.0f) // Constructor with frequency and b0 parameters
        : current_b0(initial_b0), // Initialize the system gain coefficient
          internal_pos_q32(0),    // Initialize the internal position accumulator to zero
          vel(0.0f),              // Initialize the estimated velocity to zero
          dist(0.0f),             // Initialize the estimated disturbance to zero
          acc(0.0f),              // Initialize the estimated total acceleration to zero
          pos_offset_q32(0),      // Initialize the output position offset to zero
          is_reversed(false),     // Initialize the reverse direction flag to false
          is_initialized(false) { // Initialize the warm-up synchronization flag to false
        
        set_freq(freq_hz); // Call internal method to calculate dt and max_safe_fc based on frequency
    } // End of constructor body

    /// @brief Executes one iteration of the discrete observer.
    /// @param real_pos_q32 The raw measured position from the encoder in Q32.32 format.
    /// @param prev_trq The control effort applied to the motor during the previous tick.
    /// @param target_fc The desired cutoff frequency (bandwidth) of the observer in Hz.
    /// @return ObserverState containing the estimated kinematics in the modified output frame.
    inline __attribute__((always_inline)) ObserverState tick(int64_t real_pos_q32, float prev_trq, float target_fc) { // Execute one estimator step
        
        // __builtin_expect informs the compiler that initialization happens rarely, optimizing the hot path
        if (__builtin_expect(!is_initialized, 0)) { // Check if the observer requires initial warm-up synchronization
            internal_pos_q32 = real_pos_q32;        // Force internal position to match actual position immediately
            is_initialized = true;                  // Mark observer as initialized so this block is skipped next time
        } // End of initialization block

        // Get raw angle (lower 32 bits natively handle integer wrap-around calculation)
        int32_t current_raw_angle = (int32_t)real_pos_q32; // Cast 64-bit position to 32-bit to isolate the fractional angle

        // Clamp target_fc for Euler integration stability boundary limit
        float safe_fc = (target_fc > max_safe_fc) ? max_safe_fc : target_fc; // Restrict bandwidth to the Nyquist-derived limit
        float w = 2.0f * (float)M_PI * safe_fc; // Convert frequency in Hz to angular frequency in rad/s
        
        // Calculate the linear observer gains based on pole placement formulas
        float l1 = 3.0f * w;         // Calculate proportional gain for position error
        float l2 = 3.0f * w * w;     // Calculate proportional gain for velocity error
        float l3 = w * w * w;        // Calculate integral gain for disturbance estimation

        float u_acc = current_b0 * prev_trq; // Calculate the expected acceleration from the known applied torque

        // Calculate tracking error using purely internal uninterrupted variables to prevent jumps
        int32_t err_angle_q32 = current_raw_angle - (int32_t)internal_pos_q32; // Subtract estimated angle from actual angle
        float err = (float)err_angle_q32 * 2.3283064365386962890625e-10f; // Scale Q32 integer error to a normalized float value

        // Calculate Forward Euler derivatives for state propagation
        float d_pos  = vel + l1 * err;           // Compute position derivative using estimated velocity and error correction
        float d_vel  = dist + u_acc + l2 * err;  // Compute velocity derivative using disturbance, input, and error correction
        float d_dist = l3 * err;                 // Compute disturbance derivative using purely error correction

        // Perform Euler Integration to update the internal states
        vel  += d_vel * dt;  // Update estimated velocity by integrating the velocity derivative
        dist += d_dist * dt; // Update estimated disturbance by integrating the disturbance derivative
        
        int32_t delta_pos_q32 = (int32_t)((d_pos * dt) * 4294967296.0f); // Convert float position derivative back to Q32 discrete step
        internal_pos_q32 += delta_pos_q32; // Accumulate the discrete step into the absolute internal position
        acc = dist + u_acc; // Compute total perceived acceleration as the sum of disturbance and input

        ObserverState out_state; // Instantiate an empty structure to hold the output state
        if (is_reversed) { // Check if the user requested a reversed coordinate system
            out_state.pos_q32 = -(internal_pos_q32 + pos_offset_q32); // Invert and offset the output position
            out_state.vel     = -vel;  // Invert the output velocity
            out_state.acc     = -acc;  // Invert the output total acceleration
            out_state.dist    = -dist; // Invert the output estimated disturbance
        } else { // Standard, non-reversed coordinate system path
            out_state.pos_q32 = internal_pos_q32 + pos_offset_q32; // Apply the coordinate offset to the internal position
            out_state.vel     = vel;   // Pass the internal velocity directly to the output
            out_state.acc     = acc;   // Pass the internal acceleration directly to the output
            out_state.dist    = dist;  // Pass the internal disturbance directly to the output
        } // End of coordinate transformation logic

        internal_pos_i32 = internal_pos_q32;

        return out_state; // Return the fully populated output state to the caller
    } // End of tick function

    /// @brief Safely forces a new absolute position on the output without disturbing the estimator math.
    inline void reset_pos(int64_t new_pos_q32) { // Define method to zero or offset the user-facing position
        if (is_reversed) { // Check if coordinate system is currently inverted
            pos_offset_q32 = -new_pos_q32 - internal_pos_q32; // Calculate offset required for reversed mapping
        } else { // Coordinate system is standard
            pos_offset_q32 = new_pos_q32 - internal_pos_q32; // Calculate offset required for standard mapping
        } // End of offset calculation
    } // End of reset_pos method

    /// @brief Performs a full reset of the observer states, usually after motor disable/enable transitions.
    inline void reset_observer() { // Define method to reset kinematics while preserving coordinates
        is_initialized = false;    // Clear initialization flag to force a warm-up sync on the next tick
        vel = 0.0f;                // Reset internal velocity estimate to zero
        acc = 0.0f;                // Reset internal acceleration estimate to zero
        dist = 0.0f;               // Reset internal disturbance estimate to zero
        // Output position offset is intentionally preserved to maintain the user's reference frame
    } // End of reset_observer method

    /// @brief Reverses the output coordinate system. Does not affect estimator stability.
    inline void set_invert(bool revert) { // Define method to change rotational direction mapping
        is_reversed = revert;             // Store the requested boolean state into the class variable
    } // End of set_revert method

    inline void set_b0(float new_b0) { // Define method to dynamically update the system gain
        current_b0 = new_b0;           // Apply the newly provided b0 parameter to the internal state
    } // End of set_b0 method
    
    inline float get_b0() const { // Define getter method to read the current system gain
        return current_b0;        // Return the currently active b0 parameter
    } // End of get_b0 method
    
    inline void set_freq(uint32_t freq_hz) { // Define method to configure integration step size based on timer frequency
        float freq = (float)freq_hz;
        dt = 1.0f / freq;          // Calculate delta time by taking the inverse of frequency
        max_safe_fc = (0.25f * freq) / (2.0f * (float)M_PI); // Compute the highest safe bandwidth before Euler instability occurs
    } // End of set_freq method
}; // End of LinearObserver class definition