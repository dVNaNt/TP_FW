#pragma once

// Common includes for the translated TunePulse algo headers.
#include <stdint.h>
#include <limits.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stddef.h>
#include "math_integer/trigonometry.h"

// Stub macros for logging that mirror the defmt_rtt usage in the original Rust code.
// Projects can redefine these before including the migration headers to hook into
// a concrete logging implementation.
#ifndef TUNEPULSE_LOG_INFO
#define TUNEPULSE_LOG_INFO(fmt, ...) ((void)0)
#endif

#ifndef TUNEPULSE_LOG_WARN
#define TUNEPULSE_LOG_WARN(fmt, ...) ((void)0)
#endif

#ifndef TUNEPULSE_LOG_ERROR
#define TUNEPULSE_LOG_ERROR(fmt, ...) ((void)0)
#endif

// Utility clamp helper used across several headers.
static inline int32_t tunepulse_clamp_i32(int32_t value, int32_t limit) {
    if (value > limit) {
        return limit;
    }
    if (value < -limit) {
        return -limit;
    }
    return value;
}



// Forward declaration for conversion
struct Pol2D;

/// 2D Vector in Cartesian coordinates (sin, cos) using i1.15 fixed-point format
struct Vec2D {
    int16_t sin;  // Sine component (y-axis) in i1.15 format
    int16_t cos;  // Cosine component (x-axis) in i1.15 format

    /// Constructor
    Vec2D(int16_t s = 0, int16_t c = 0) : sin(s), cos(c) {}

    /// Scale the vector by a coefficient in i1.15 format
    /// @param scale_factor Scaling factor in i1.15 (32768 = 1.0)
    /// @return Scaled vector
    Vec2D scale(int16_t scale_factor) const {
        int32_t s = (int32_t)sin;
        int32_t c = (int32_t)cos;
        int32_t sc = (int32_t)scale_factor;
        
        return Vec2D(
            (int16_t)((s * sc) >> 15),
            (int16_t)((c * sc) >> 15)
        );
    }

    /// Rotate the vector by another vector (rotation offset)
    /// @param rotation Rotation vector (sin, cos) in i1.15 format
    /// @return Rotated vector
    Vec2D rotate(const Vec2D& rotation) const {
        int32_t ss = (int32_t)sin;
        int32_t sc = (int32_t)cos;
        int32_t rs = (int32_t)rotation.sin;
        int32_t rc = (int32_t)rotation.cos;

        // Rotation formula:
        // out_sin = sin * rotation.cos + cos * rotation.sin
        // out_cos = cos * rotation.cos - sin * rotation.sin
        int32_t new_sin = ss * rc + sc * rs;
        int32_t new_cos = sc * rc - ss * rs;

        return Vec2D(
            (int16_t)(new_sin >> 15),
            (int16_t)(new_cos >> 15)
        );
    }

    /// Check if sin/cos values are properly normalized
    /// Validates that sin²+cos² ≤ 32767² * 2 (allows for some numerical tolerance)
    /// @param sin_q15 Sine value in Q15 format
    /// @param cos_q15 Cosine value in Q15 format
    /// @return true if normalized, false if out of range
    bool validate(int16_t threshold = INT16_MAX) {
        int32_t sq_sin = (int32_t)sin;           
        int32_t sq_cos = (int32_t)cos;           
        int32_t th = (int32_t)threshold;
        uint32_t sum = (uint32_t)(sq_sin * sq_sin) + (uint32_t)(sq_cos * sq_cos);  // Calculate sin² + cos²

        // Check if sum is within valid range
        return sum <= (th * th); 
    }
};

/// 2D Vector in Polar coordinates (angle, amplitude) using i1.15 fixed-point format
struct Pol2D {
    int16_t angle;      // Angle in normalized format (INT16_MIN to INT16_MAX = -π to π)
    int16_t amplitude;  // Amplitude (magnitude) in i1.15 format

    /// Constructor
    Pol2D(int16_t ang = 0, int16_t amp = 0) : angle(ang), amplitude(amp) {}

    /// Scale the polar vector by a coefficient in i1.15 format
    /// @param scale_factor Scaling factor in i1.15 (32768 = 1.0)
    /// @return Scaled polar vector
    Pol2D scale(int16_t scale_factor) const {
        int32_t amp = (int32_t)amplitude;
        int32_t sc = (int32_t)scale_factor;
        
        return Pol2D(
            angle,  // Angle remains unchanged
            (int16_t)((amp * sc) >> 15)
        );
    }

    /// Rotate the polar vector by an angle offset
    /// @param angle_offset Angle offset to add (normalized format)
    /// @return Rotated polar vector
    Pol2D rotate(int16_t angle_offset) const {
        // Simple angle addition (wraps around automatically due to int16_t overflow)
        return Pol2D(
            (int16_t)(angle + angle_offset),
            amplitude  // Amplitude remains unchanged
        );
    }

    /// Convert polar coordinates to Cartesian (Vec2D)
    /// Note: This method requires angle2sincos from math_integer/trigonometry.h
    /// Include trigonometry.h before using this method
    /// @return Cartesian vector (sin, cos)
    Vec2D convert() const {
        // Declare output variables for sin and cos
        int16_t s, c;
        
        // Use angle2sincos to convert angle to sin/cos components
        // This function is defined in math_integer/trigonometry.h
        angle2sincos(angle, &s, &c);
        
        // Scale the sin/cos by amplitude
        int32_t sin_scaled = ((int32_t)s * (int32_t)amplitude) >> 15;
        int32_t cos_scaled = ((int32_t)c * (int32_t)amplitude) >> 15;
        
        return Vec2D((int16_t)sin_scaled, (int16_t)cos_scaled);
    }
};

/// Frequency configuration structure
/// Manages core frequency, PWM frequency, and PWM period with automatic calculation
struct Frequency {
private:
    int32_t frequency_core_;  // Core/system frequency in Hz
    int32_t frequency_pwm_;   // PWM frequency in Hz
    int32_t period_pwm_;      // PWM period (calculated from core and PWM frequencies)

public:
    /// Constructor with default values
    /// @param freq_core Core frequency in Hz (default: 170000000 = 170 MHz for STM32G4)
    /// @param freq_pwm PWM frequency in Hz (default: 20000 = 20 kHz)
    Frequency(int32_t freq_core = 170000000, int32_t freq_pwm = 18000)
        : frequency_core_(freq_core)
        , frequency_pwm_(freq_pwm)
        , period_pwm_(0)
    {
        calculate_period();
    }

    /// Get core frequency in Hz
    /// @return Core frequency
    inline int32_t get_frequency_core() const {
        return frequency_core_;
    }

    /// Get PWM frequency in Hz
    /// @return PWM frequency
    inline int32_t get_frequency_pwm() const {
        return frequency_pwm_;
    }

    /// Get PWM period (timer ticks per PWM cycle)
    /// @return PWM period
    inline int32_t get_period_pwm() const {
        return period_pwm_;
    }

    /// Set core frequency and recalculate period
    /// @param freq_core Core frequency in Hz
    void set_frequency_core(int32_t freq_core) {
        frequency_core_ = freq_core;
        calculate_period();
    }

    /// Set PWM frequency and recalculate period
    /// @param freq_pwm PWM frequency in Hz
    void set_frequency_pwm(int32_t freq_pwm) {
        frequency_pwm_ = freq_pwm;
        calculate_period();
    }

private:
    /// Calculate PWM period from core and PWM frequencies
    /// Period = FrequencyCore / FrequencyPWM
    void calculate_period() {
        if (frequency_pwm_ > 0) {
            period_pwm_ = frequency_core_ / frequency_pwm_;
        } else {
            period_pwm_ = 0;
        }
    }
};


