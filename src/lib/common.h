#pragma once

// Common includes for the translated TunePulse algo headers.
#include <stdint.h>
#include <limits.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stddef.h>
#include "vectors.h"

#include "motor_driver/types.h"

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
// Must be declared before including headers that use it.
static inline int32_t clamp_i32(int32_t value, int32_t limit) {
    if (value > limit) {
        return limit;
    }
    if (value < -limit) {
        return -limit;
    }
    return value;
}


// Include headers that depend on the utility functions above
#include "math_integer/controllers/pid.h"
#include "math_integer/filters/filter1st.h"
#include "math_integer/helpers/stability_check.h"





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


