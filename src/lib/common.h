#pragma once

// Common includes for the translated TunePulse algo headers.
#include <limits.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <cmath>
#include <math.h>
#include <atomic>  // C++ atomic operations support

inline float signf(float x) { return (x > 0.0f) ? 1.0f : ((x < 0.0f) ? -1.0f : 0.0f); }
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

enum Channel : uint8_t {
    CH1 = 0,  // Channel 1 (array index 0)
    CH2 = 1,  // Channel 2 (array index 1)
    CH3 = 2   // Channel 3 (array index 2)
};

inline int16_t fast_sign_int(int32_t val) {
    return (val >> 31) | 1; 
    // Returns -1 for negative values and 1 for 0 and positive values.
    // Zero is considered positive.
}

// SerialUSB Logging utill
#include "dev_log.h"

// Include headers that depend on the utility functions above
#include "math/helpers/fifo_buffer.h"
#include "math/controllers/pi.h"
#include "math/filters/ema.h"
#include "math/filters/median3.h"
#include "math/helpers/stability_check.h"
#include "motor_driver/types.h"
#include "vectors.h"




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
    Frequency(int32_t freq_core = 170000000, int32_t freq_pwm = 20000)
        : frequency_core_(freq_core), frequency_pwm_(freq_pwm), period_pwm_(0) {
        calculate_period();
    }

    /// Get core frequency in Hz
    /// @return Core frequency
    inline int32_t get_frequency_core() const { return frequency_core_; }

    /// Get PWM frequency in Hz
    /// @return PWM frequency
    inline int32_t get_frequency_pwm() const { return frequency_pwm_; }

    /// Get PWM period (timer ticks per PWM cycle)
    /// @return PWM period
    inline int32_t get_period_pwm() const { return period_pwm_; }

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
