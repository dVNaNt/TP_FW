#pragma once

/**
 * @file common.h
 * @brief Common includes and small shared utilities for the TunePulse
 *        algorithm headers.
 *
 * Provides:
 * - Standard C/C++ headers used across the library (stdint, math, atomics).
 * - Small inline math helpers: signf(), clamp_i32(), fast_sign_int().
 * - The Channel enum used to index 3-phase quantities (CH1..CH3).
 * - The Frequency helper struct that keeps core/PWM frequencies and derives
 *   the PWM timer period.
 * - Include-order-critical utilities: clamp_i32() must be visible before the
 *   headers that use it, hence it is defined here ahead of those includes.
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */
#include <limits.h>
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <atomic>  // C++ atomic operations support
#include <cmath>


/**
 * @brief Floating-point sign extraction.
 * @param x Input value.
 * @return +1.0f if x > 0, -1.0f if x < 0, 0.0f if x == 0.
 */
inline float signf(float x) {
    return (x > 0.0f) ? 1.0f : ((x < 0.0f) ? -1.0f : 0.0f);
}
// Utility clamp helper used across several headers.
// Must be declared before including headers that use it.
/**
 * @brief Symmetric saturation (clamp) of a signed 32-bit value.
 *
 * @param value Input value to clamp.
 * @param limit Symmetric limit; result lies in [-limit, +limit].
 * @return value clamped to the range [-limit, +limit].
 *
 * @note Assumes @p limit is non-negative; a negative limit would produce an
 *       empty range (value > limit and value < -limit cannot both be fixed).
 */
static inline int32_t clamp_i32(int32_t value, int32_t limit) {
    if (value > limit) {
        return limit;
    }
    if (value < -limit) {
        return -limit;
    }
    return value;
}

/**
 * @brief Logical channel identifiers used to index 3-phase quantities.
 *
 * Values are chosen to match zero-based array indices, so a Channel value can
 * be used directly to subscript per-phase arrays (currents, offsets, ...).
 */
enum Channel : uint8_t {
    CH1 = 0,  ///< Channel 1 (array index 0)
    CH2 = 1,  ///< Channel 2 (array index 1)
    CH3 = 2   ///< Channel 3 (array index 2)
};

// ============================================================================
// Shared PWM-driver protection vocabulary
// ============================================================================

/// Callback that programs a hardware comparator (DAC) threshold.
/// @param threshold 16-bit threshold value for the analog comparator
typedef void (*CompThresholdCallback)(uint16_t);

/// Default fault recovery timer length in control-loop ticks.
/// At the nominal 20 kHz control rate this equals 500 ms.
static constexpr int32_t kDefaultRecoveryCycles = 10000;

/**
 * @brief Branchless integer sign extraction.
 *
 * Uses an arithmetic right shift of the sign bit: for negative inputs the
 * shift yields all-ones (-1), which OR-ed with 1 stays -1; for zero or
 * positive inputs the shift yields 0, which OR-ed with 1 becomes +1.
 *
 * @param val Input value.
 * @return -1 if val < 0, otherwise +1 (zero is treated as positive).
 */
inline int16_t fast_sign_int(int32_t val) {
    return (val >> 31) | 1;
    // Returns -1 for negative values and 1 for 0 and positive values.
    // Zero is considered positive.
}

// SerialUSB Logging utill
#include "dev_log.h"

// Include headers that depend on the utility functions above
#include "math/controllers/pi.h"
#include "math/filters/ema.h"
#include "math/filters/median3.h"
#include "math/helpers/fifo_buffer.h"
#include "math/helpers/stability_check.h"
#include "motor_control/types.h"
#include "vectors.h"

/**
 * @brief Frequency configuration helper.
 *
 * Manages the core (system) frequency and the PWM carrier frequency, and
 * automatically derives the PWM timer period as
 * @c period = frequency_core / frequency_pwm.
 *
 * @note This computes the period for edge-aligned counting. For
 *       center-aligned (up/down) PWM the hardware period must be half of
 *       this value (see PWM_TIMER_PERIOD in lib.h which divides by 2*freq).
 */
struct Frequency {
private:
    int32_t frequency_core_;  ///< Core/system frequency in Hz.
    int32_t frequency_pwm_;   ///< PWM carrier frequency in Hz.
    int32_t period_pwm_;      ///< PWM period in timer ticks (core_freq / pwm_freq).

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
