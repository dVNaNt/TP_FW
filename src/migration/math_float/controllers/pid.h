#pragma once

#include "../../common.h"

// Implements a floating-point PID controller, providing proportional, integral, derivative,
// and feedforward control functionalities for precise process control in various applications.
//
// Key Features:
// - Configurable proportional, integral, derivative, and feedforward gains.
// - Integrates error using Tustin's method with anti-windup via integral clamping.
// - Calculates control output with derivative and feedforward contributions.
// - Provides output retrieval for use in control loops.
//
// Licensed under the Apache License, Version 2.0
// Copyright 2024 Anton Khrustalev, creapunk.com

/// Implements the PID controller for precise process control.
typedef struct {
    float kp;             // Proportional gain.
    float ki;             // Integral gain.
    float kd;             // Derivative gain.
    float kff;            // Feedforward gain.
    float integral;       // Accumulated integral of error.
    float previous_error; // Previous error value for derivative calculation.
    float output;         // Current output of the PID controller.
} PID_Flt;

/// Creates a new PID controller with specified gains.
static inline PID_Flt pid_flt_new(float kp, float ki, float kd, float kff) {
    PID_Flt p = {
        .kp = kp,                  // Sets the proportional gain.
        .ki = ki,                  // Sets the integral gain.
        .kd = kd,                  // Sets the derivative gain.
        .kff = kff,                // Sets the feedforward gain.
        .integral = 0.0f,          // Initializes the integral term to zero.
        .previous_error = 0.0f,    // Initializes the previous error to zero.
        .output = 0.0f,            // Initializes the output to zero.
    };
    return p;
}

/// Updates the PID controller with the current error, feedforward value, and output limit.
static inline void pid_flt_tick(PID_Flt* self, float error, float feedfwd, float limit) {
    // Calculate proportional term
    float p = self->kp * error; // Computes the proportional component.

    // Tustin's method for integrating the error with smoothing
    self->integral += (error + self->previous_error) * 0.5f; // Integrates the error using Tustin's method.

    // Clamp integral to avoid with anti-windup
    if (self->integral > limit) self->integral = limit;
    if (self->integral < -limit) self->integral = -limit; // Clamps the integral term to prevent windup.

    // Calculate integral term
    float i = self->ki * self->integral; // Computes the integral component.

    // Calculate derivative term
    float d = self->kd * (error - self->previous_error); // Computes the derivative component.

    // Update previous error for the next calculation
    self->previous_error = error; // Updates the previous error.

    // Calculate the total output with clamping
    float out = p + i + d + (self->kff * feedfwd); // Includes feedforward term
    if (out > limit) out = limit;
    if (out < -limit) out = -limit;
    self->output = out; // Calculates and clamps the total PID output.
}

/// Retrieves the current output of the PID controller.
static inline float pid_flt_output(const PID_Flt* self) {
    return self->output; // Returns the current output value.
}


