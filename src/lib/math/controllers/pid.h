#pragma once

#include "../../common.h"

/// A Proportional-Integral-Derivative (PID) controller implementation
/// to calculate corrective action for controlling dynamic systems.
///
/// **Note**
/// - Based on integer implementation and works with i16 range
/// - Works with constant dt only
/// - Has integral anti-windup
struct PID_Int {
private:
    // Constants controlling fast vs. slow math operations
    static constexpr bool FAST_MATH = true;        // Use fast bit-shift math
    static constexpr int32_t SLOW_MATH_SCALE = 2;  // Scale factor for slow math (do not change!)

    int32_t kp_;             // Proportional gain coefficient (scaled)
    int32_t ki_;             // Integral gain coefficient (scaled)
    int32_t kd_;             // Derivative gain coefficient (scaled)
    int32_t kff_;            // Feed-forward gain coefficient (scaled)
    int32_t integral_;       // Accumulator for the integral term
    int32_t previous_error_; // Stores the previous error value
    int16_t output_;         // The PID controller output

    /// Scale gain coefficient based on math mode
    /// @param gain Raw gain value (-10000% to 10000%)
    /// @return Scaled gain for internal calculations
    static inline int32_t scaleGain(int32_t gain) {
        int32_t clamped = clamp_i32(gain, 10000); // Clamp to valid range
        if (FAST_MATH) {
            return (clamped << 7) / 100; // Fast bit-shift scaling
        }
        return clamped; // No scaling for slow math
    }

public:
    /// Constructor: Initialize PID controller with gain coefficients
    /// @param kp Proportional gain (-10000% to 10000%)
    /// @param ki Integral gain (-10000% to 10000%)
    /// @param kd Derivative gain (-10000% to 10000%)
    /// @param kff Feed-forward gain (-10000% to 10000%)
    PID_Int(int32_t kp, int32_t ki, int32_t kd, int32_t kff)
        : kp_(scaleGain(kp))       // Scale and store proportional gain
        , ki_(scaleGain(ki))       // Scale and store integral gain
        , kd_(scaleGain(kd))       // Scale and store derivative gain
        , kff_(scaleGain(kff))     // Scale and store feed-forward gain
        , integral_(0)             // Initialize integral accumulator to zero
        , previous_error_(0)       // Initialize previous error to zero
        , output_(0)               // Initialize output to zero
    {}

    /// Update the PID controller calculations
    /// @param error Difference between desired and measured values
    /// @param feedfwd Feed-forward value to anticipate system response
    /// @param limit Maximum output limit (positive or negative)
    inline void tick(int16_t error, int16_t feedfwd, int16_t limit) {
        // Convert inputs to i32 for fixed-point math
        int32_t error_i32 = static_cast<int32_t>(error);       // Cast error to 32-bit
        int32_t feedfwd_i32 = static_cast<int32_t>(feedfwd);   // Cast feedforward to 32-bit
        int32_t limit_i32 = static_cast<int32_t>(limit);       // Cast limit to 32-bit

        // ######################## PROPORTIONAL TERM #################################
        int32_t p = FAST_MATH 
            ? ((error_i32 * kp_) >> 7)                         // Fast: bit-shift division
            : ((error_i32 * kp_) >> SLOW_MATH_SCALE);          // Slow: scaled division

        // ########################## INTEGRAL TERM ###################################
        // Tustin's method (trapezoidal rule) for integrating error with smoothing
        integral_ += (error_i32 + previous_error_) >> 1;       // Average of current and previous error

        // Clamp integral with anti-windup to prevent saturation
        integral_ = clamp_i32(integral_, (limit_i32 << 7) / ki_); // Limit integral accumulation

        // Calculate integral term
        int32_t i = FAST_MATH 
            ? ((integral_ * ki_) >> 7)                         // Fast: bit-shift division
            : ((integral_ * ki_) >> SLOW_MATH_SCALE);          // Slow: scaled division

        // ######################### DERIVATIVE TERM ##################################
        // Calculate derivative by finding the difference in error
        int32_t derivative = error_i32 - previous_error_;      // Rate of error change

        // Calculate derivative term
        int32_t d = FAST_MATH 
            ? ((derivative * kd_) >> 7)                        // Fast: bit-shift division
            : ((derivative * kd_) >> SLOW_MATH_SCALE);         // Slow: scaled division

        // Update previous error for the next calculation
        previous_error_ = error_i32;                           // Store current error

        // ######################## FEED-FORWARD TERM #################################
        int32_t ff = FAST_MATH 
            ? ((feedfwd_i32 * kff_) >> 7)                      // Fast: bit-shift division
            : ((feedfwd_i32 * kff_) >> SLOW_MATH_SCALE);       // Slow: scaled division

        // ############################## OUTPUT ######################################
        // Calculate total output by combining all components
        int32_t out = p + i + d + ff;                          // Sum P + I + D + FF

        // Apply fixed-point math correction for slow math mode
        if (!FAST_MATH) {
            out = out / (100 >> SLOW_MATH_SCALE);              // Scale correction
        }

        // Clamp final output to specified limits
        output_ = static_cast<int16_t>(clamp_i32(out, limit_i32)); // Store clamped output
    }

    /// Get the current PID output value
    /// @return Calculated output as 16-bit integer
    inline int16_t output() const {
        return output_; // Return stored output
    }

    /// Reset the controller state (integral and previous error)
    inline void reset() {
        integral_ = 0;       // Clear integral accumulator
        previous_error_ = 0; // Clear previous error
        output_ = 0;         // Clear output
    }

    /// Update proportional gain
    /// @param kp New proportional gain (-10000% to 10000%)
    inline void setKp(int32_t kp) {
        kp_ = scaleGain(kp); // Scale and store new gain
    }

    /// Update integral gain
    /// @param ki New integral gain (-10000% to 10000%)
    inline void setKi(int32_t ki) {
        ki_ = scaleGain(ki); // Scale and store new gain
    }

    /// Update derivative gain
    /// @param kd New derivative gain (-10000% to 10000%)
    inline void setKd(int32_t kd) {
        kd_ = scaleGain(kd); // Scale and store new gain
    }

    /// Update feed-forward gain
    /// @param kff New feed-forward gain (-10000% to 10000%)
    inline void setKff(int32_t kff) {
        kff_ = scaleGain(kff); // Scale and store new gain
    }
};
