#pragma once

#include "../../common.h"
#include "../../math_integer/trigonometry.h" // Imports trigonometry module as math
#include "../calibration/angle_calibrator.h"
#include "../types.h"
#include "../motor.h"
#include "angle2pulse.h"

typedef struct {
    // COMMON
    /// Duty of brake mode
    int16_t enable;
    Motor motor;

    ControlMode control_mode;

    DriverStatus status;

    Angle2Pulse angle2pulse;

    // ####### Related to step-dir driver ########
    /// Motor resistance
    int16_t angle;
    /// Motor resistance
    int16_t current;

    /// Motor rotation direction
    int32_t direction;

    int16_t ch_1234[4];
} DriverPulse;

static inline void driver_pulse_mode_check(const DriverPulse* self, const int16_t ab[2], int16_t out[2]) {
    switch (self->control_mode) {
        case ControlMode_CurrentAB:
            out[0] = ab[0];
            out[1] = ab[1];
            break;
        case ControlMode_VoltageAB:
        default:
            out[0] = 0;
            out[1] = 0;
            break;
    }
}

static inline DriverPulse driver_pulse_new(Motor motor, ControlMode control_mode) {
    DriverPulse d;
    d.enable = 0;
    d.angle = 0;
    d.current = 0;
    d.direction = motor.get_direction();
    d.motor = motor;
    d.control_mode = control_mode;
    d.status = DriverStatus_Ready;
    d.ch_1234[0] = d.ch_1234[1] = d.ch_1234[2] = d.ch_1234[3] = 0;
    d.angle2pulse = angle2pulse_new(4);
    return d;
}

static inline void driver_pulse_tick_control(DriverPulse* self, int16_t angle, int16_t amplitude, int16_t supply, int16_t out[4]) {
    int16_t voltage_ab[2];
    switch (self->status) {
        case DriverStatus_Ready:
            voltage_ab[0] = angle;
            voltage_ab[1] = amplitude;
            break;
        case DriverStatus_Error:
        case DriverStatus_Calibrating:
        default:
            voltage_ab[0] = 0;
            voltage_ab[1] = 0;
            break;
    }
    int16_t current_ab[2];
    driver_pulse_mode_check(self, voltage_ab, current_ab);
    int16_t dir, steps;
    angle2pulse_tick(&self->angle2pulse, current_ab[0], &dir, &steps);
    self->ch_1234[0] = self->enable;
    self->ch_1234[1] = dir;
    self->ch_1234[2] = steps;
    self->ch_1234[3] = voltage_ab[1];
    for (size_t i = 0; i < 4; ++i) {
        out[i] = self->ch_1234[i];
    }
    (void)supply;
}

static inline bool driver_pulse_change_motor_mode(DriverPulse* self, MotorType motor_type) {
    (void)motor_type;
    return self->motor.get_motor_type() == motor_type;
}

static inline bool driver_pulse_change_phase_mode(DriverPulse* self, PhasePattern connection) {
    (void)connection;
    return self->motor.get_phase_pattern() == connection;
}

static inline bool driver_pulse_change_control_mode(DriverPulse* self, ControlMode mode) {
    return self->control_mode == mode;
}

static inline const int16_t* driver_pulse_get_control(const DriverPulse* self) {
    return self->ch_1234;
}


