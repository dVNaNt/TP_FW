#pragma once

#include "../../common.h"

/// A Proportional-Integral-Derivative (PID) controller implementation
/// to calculate corrective action for controlling dynamic systems.
///
/// **Note**
/// - Based on integer implementation and works with i16 range
/// - Works with constant dt only
/// - Has integral anti-windup
typedef struct {
    /// Proportional gain coefficient: -10000% to 10000%.
    /// Controls the reaction to the current error magnitude.
    int32_t kp;

    /// Integral gain coefficient: -10000% to 10000%.
    /// Controls the reaction based on the accumulation of past errors.
    int32_t ki;

    /// Derivative gain coefficient: -10000% to 10000%.
    /// Controls the reaction to the rate of error change.
    int32_t kd;

    /// Feed-forward gain coefficient: -10000% to 10000%.
    /// Adds an anticipated value to the output to help the system respond faster.
    int32_t kff;

    /// Accumulator for the integral term
    int32_t integral;
    /// Stores the previous error value for derivative and integral calculation
    int32_t previous_error;
    /// The PID controller output
    int16_t output;
} PID_Int;

// Constants controlling fast vs. slow math operations
enum {
    PID_INT_FAST_MATH = 1,
    PID_INT_SLOW_MATH_SCALE = 2  // Do not change!
};

// Constructor for the PID controller
//
// # Arguments
// * `kp` - Proportional gain coefficient
// * `ki` - Integral gain coefficient
// * `kd` - Derivative gain coefficient
// * `kff` - Feed-forward gain coefficient
//
// # Returns
// A new instance of the PID controller with the given gain coefficients.
static inline PID_Int pid_int_new(int32_t kp, int32_t ki, int32_t kd, int32_t kff) {
    // Adjusts and fits each gain coefficient within a valid range
    PID_Int self;
    self.kp = tunepulse_clamp_i32(kp, 10000);
    self.ki = tunepulse_clamp_i32(ki, 10000);
    self.kd = tunepulse_clamp_i32(kd, 10000);
    self.kff = tunepulse_clamp_i32(kff, 10000);

    if (PID_INT_FAST_MATH) {
        self.kp = (self.kp << 7) / 100;
        self.ki = (self.ki << 7) / 100;
        self.kd = (self.kd << 7) / 100;
        self.kff = (self.kff << 7) / 100;
    }

    self.integral = 0;       // Initialize the integral accumulator
    self.previous_error = 0; // Initialize the previous error
    self.output = 0;         // Initialize the output
    return self;
}

// Update the PID controller calculations
//
// # Arguments
// * `error` - The difference between the desired and measured values
// * `feedfwd` - A feed-forward value used to anticipate the system response
// * `limit` - The maximum output limit (positive or negative)
//
// This method computes the new PID output based on the provided error, feed-forward value,
// and output limits, considering the proportional, integral, derivative, and feed-forward components.
static inline void pid_int_tick(PID_Int* self, int16_t error, int16_t feedfwd, int16_t limit) {
    // Convert inputs as i32 to allow fixed point math
    int32_t error_i32 = (int32_t)error;
    int32_t feedfwd_i32 = (int32_t)feedfwd;
    int32_t limit_i32 = (int32_t)limit;

    // ######################## PROPORTIONAL TERM #################################
    int32_t p = PID_INT_FAST_MATH ? ((error_i32 * self->kp) >> 7)
                                  : ((error_i32 * self->kp) >> PID_INT_SLOW_MATH_SCALE); // Maximum possible value: ±100 * ±2^15

    // ########################## INTEGRAL TERM ###################################
    // Tustin's method (trapezoidal rule) for integrating the error with smoothing
    self->integral += (error_i32 + self->previous_error) >> 1;

    // Clamp integral to avoid with anti-windup
    self->integral = tunepulse_clamp_i32(self->integral, limit_i32); // Maximum accumulation: ±2^15

    // Calculate integral term
    int32_t i = PID_INT_FAST_MATH ? ((self->integral * self->ki) >> 7)
                                  : ((self->integral * self->ki) >> PID_INT_SLOW_MATH_SCALE); // Maximum possible value: ±100 * ±2^15

    // ######################### DERIVATIVE TERM ##################################
    // Calculate derivative by finding the difference in error
    int32_t derivative = error_i32 - self->previous_error; // Maximum value: ±2 * ±2^15 = ±2^16

    // Calculate derivative term
    int32_t d = PID_INT_FAST_MATH ? ((derivative * self->kd) >> 7)
                                  : ((derivative * self->kd) >> PID_INT_SLOW_MATH_SCALE); // Maximum possible value: ±100 * ±2^16

    // Update previous error for the next calculation
    self->previous_error = error_i32;

    // ######################## FEED-FORWARD TERM #################################
    int32_t ff = PID_INT_FAST_MATH ? ((feedfwd_i32 * self->kff) >> 7)
                                   : ((feedfwd_i32 * self->kff) >> PID_INT_SLOW_MATH_SCALE); // Maximum possible value: ±100 * ±2^15

    // ############################## OUTPUT ######################################
    // Calculate the total output by combining all components
    int32_t output = p + i + d + ff; // Maximum possible value: ±500 * 2^15

    // Apply fixed-point math correction to the output
    if (!PID_INT_FAST_MATH) {
        output = output / (100 >> PID_INT_SLOW_MATH_SCALE);
    }
    

    // Clamp the final output to ensure it stays within the specified limits
    self->output = (int16_t)tunepulse_clamp_i32(output, limit_i32);
}

/// Retrieve the output value of the PID controller
/// # Returns
/// The calculated output as a 16-bit integer value.
static inline int16_t pid_int_output(const PID_Int* self) {
    return self->output;
}


