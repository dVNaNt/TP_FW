#pragma once

// Implements the MotorPulse module, responsible for converting angles to motor pulses,
// managing motor direction, microstepping, and error correction for precise stepper motor control.
//
// Licensed under the Apache License, Version 2.0
// Copyright 2024 Anton Khrustalev, creapunk.com

#include "../../common.h"

/// A struct representing a converter from angle to motor pulses
/// This struct is used to determine the number of pulses needed to rotate a stepper motor
/// by a specified angle. It keeps track of motor direction, microstepping, and error correction.
typedef struct {
    /// Number of steps needed by the motor driver to reach the target angle
    int16_t steps;
    /// Motor rotation direction (true for one direction, false for the other)
    bool direction;

    // UTILS
    /// Output microstepping value
    int32_t ustep;
    /// The previous angle, used to calculate the delta for each tick
    int16_t prev_angle;
    /// Accumulated error, used for more accurate step calculation
    int32_t error;
} Angle2Pulse;

/// Calculates the microstepping value based on the provided power
static inline int32_t angle2pulse_ustep_calc(uint16_t usteps_pow) {
    // Limit microstepping to a power of 9 or below to ensure stability and prevent overflow
    uint16_t usteps = (usteps_pow > 9u) ? 9u : usteps_pow;
    // Calculate the microstepping value by shifting bits to adjust precision
    return (int32_t)(UINT16_MAX >> (2 + usteps));
}

/// Constructs a new `MotorPulse` instance
static inline Angle2Pulse angle2pulse_new(uint16_t usteps_pow) {
    Angle2Pulse a;
    a.ustep = angle2pulse_ustep_calc(usteps_pow); // Calculate microstepping value based on power
    a.steps = 0;                                  // Initialize steps to zero
    a.direction = false;                          // Set initial direction to false
    a.prev_angle = 0;                             // Initialize previous angle to zero
    a.error = 0;                                  // Initialize error to zero
    return a;
}

/// Updates the motor control parameters based on the desired angle change
static inline void angle2pulse_tick(Angle2Pulse* self, int16_t angle, int16_t* out_dir, int16_t* out_steps) {
    // Calculate the accumulated error using the difference between the current and previous angle
    self->error += (int32_t)(angle - self->prev_angle);

    // Update the previous angle to the current angle
    self->prev_angle = angle;

    // Determine the motor direction based on the sign of the error
    bool direction = self->error < 0;

    // Calculate the number of microsteps required to reach the target, with rounding
    int32_t step_shift = (abs(self->error) + (self->ustep >> 1)) / self->ustep;

    // Calculate the angle shift in microsteps to determine position change
    int32_t angle_shift = step_shift * self->ustep;

    // Adjust the integral error to avoid accumulation of rounding errors
    self->error -= angle_shift - (angle_shift << 1) * (int32_t)direction;

    // Update the number of steps needed to reach the target angle
    self->steps = (int16_t)step_shift;

    // Avoid toggling the direction pin unnecessarily if no steps are required
    if (step_shift > 0) {
        self->direction = direction;
    }

    // Return the current motor direction and the number of steps needed
    *out_dir = (int16_t)(self->direction ? 1 : 0);
    *out_steps = self->steps;
}

/// Sets the microstepping division based on the provided power
static inline void angle2pulse_set_ustep_div(Angle2Pulse* self, uint16_t ustep_pow) {
    self->ustep = angle2pulse_ustep_calc(ustep_pow); // Recalculate microstepping value
}


