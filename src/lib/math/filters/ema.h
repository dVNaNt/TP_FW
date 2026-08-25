/**
 * @file ema.h
 * @brief Integer Exponential Moving Average low-pass filter.
 *
 * FilterEMA implements y += alpha*(x-y) with an 8-bit alpha (0..255) so the
 * whole filter is multiply/shift only. Used for supply-voltage smoothing,
 * current balance supervision and other slow feedback signals.
 */

#pragma once

// Implements first-order filtering module providing both low-pass and high-pass
// filtering functionality using shared internal state for efficiency.
//
// Key Features:
// - Single structure with shared state for both LPF and HPF operations
// - Separate tick methods: tick_lpf() and tick_hpf()
// - Static helper to calculate alpha from cutoff and sampling frequencies
// - Fully integer-based arithmetic for zero-overhead performance on MCUs
// - Provides additional 16-bit precision for error accumulation
//
// Alpha Semantics:
// - Alpha 0 = infinite filtering (output never changes)
// - Alpha 255 = no filtering (output = input)
// - Alpha calculated as: alpha = 2 * PI * (fc / fs) * 255
//
// Detailed Operation:
// LPF: Implements exponential moving average: output = (1-α)*prev + α*input
// HPF: Removes DC component by subtracting LPF result from input
//
// Licensed under the Apache License, Version 2.0
// Copyright 2024 Anton Khrustalev, creapunk.com

#include "../../common.h"

// Defining the EMA struct that implements first-order filtering logic
struct FilterEMA {
private:
    int32_t state_;  // Internal filter state (scaled << 16 for precision)
    int16_t output_; // Current filter output
    int16_t alpha_;  // Filter coefficient (0..255, where 0=infinite filtering, 255=no filtering)

public:
    /// Default constructor
    FilterEMA() : state_(0), output_(0), alpha_(255) {}

    /// Constructor to initialize the filter with input value and alpha
    /// @param input_default Initial value for filter state
    /// @param alpha_value Filter coefficient (0-255)
    FilterEMA(int16_t input_default, uint8_t alpha_value) 
        : state_(((int32_t)input_default) << 16)  // Initialize state with scaled input
        , output_(input_default)                   // Initialize output
        , alpha_((int16_t)alpha_value)            // Store alpha coefficient
    {}

    /// Low-pass filter tick: Smooth input signal
    /// @param input Current input sample
    /// @return Filtered output (low-pass)
    int16_t tick_lpf(int16_t input) {
        // Convert input to 32-bit scaled format (Q16.16)
        int32_t current = ((int32_t)input) << 16;

        // LPF math: state = state + alpha * (input - state) / 256
        // Alpha 0 = infinite filtering (state never changes)
        // Alpha 255 = no filtering (state = input)

        // Calculate difference between input and current state
        int32_t diff = current - state_;

        // Scale down difference for alpha multiplication (divide by 256)
        diff >>= 8;

        // Update state: state += alpha * diff
        state_ += (diff * (int32_t)alpha_);

        // Extract output from scaled state (convert back to int16)
        output_ = (int16_t)(((uint32_t)state_) >> 16);

        return output_;  // Return filtered output
    }

    /// High-pass filter tick: Remove DC component from input
    /// @param input Current input sample
    /// @return Filtered output (high-pass)
    int16_t tick_hpf(int16_t input) {
        // Convert input to 32-bit scaled format (Q16.16)
        int32_t current = ((int32_t)input) << 16;

        // HPF math: First update DC estimate (LPF), then subtract from input
        // dc_estimate = dc_estimate + alpha * (input - dc_estimate) / 256
        // output = input - dc_estimate

        // Calculate error between input and DC estimate (state)
        int32_t error = current - state_;

        // Scale down error for alpha multiplication (divide by 256)
        error >>= 8;

        // Update DC estimate: state += alpha * error
        state_ += (error * (int32_t)alpha_);

        // Calculate HPF output: input - DC component
        int16_t dc_component = (int16_t)(state_ >> 16);  // Extract DC estimate
        output_ = input - dc_component;                   // Subtract DC from input

        return output_;  // Return high-pass filtered output
    }

    /// Get current filter output
    /// @return Last calculated output value
    inline int16_t get_output() const {
        return output_;  // Return cached output
    }


    /// Set filter alpha coefficient
    /// @param alpha_value Filter coefficient (0-255)
    inline void set_alpha(uint8_t alpha_value) {
        alpha_ = (int16_t)alpha_value;  // Update alpha value
    }

    /// Reset filter to specific value
    /// @param value New initial value for filter state
    void reset(int16_t value) {
        output_ = value;                           // Reset output
        state_ = ((int32_t)value) << 16;          // Reset state with scaled value
    }

    /// Calculate alpha from cutoff and sampling frequencies
    /// Formula: alpha = 2 * PI * (fc / fs) * 255
    /// @param fc_hz Cutoff frequency in Hz
    /// @param fs_hz Sampling frequency in Hz
    /// @return Calculated alpha value (1-255)
    void set_cutoff(uint16_t fcut_hz, uint16_t fsampl_hz) {
        if (fsampl_hz == 0) return; // Guard against division by zero

        // Alpha = 2*pi * Ts * Fc * 255
        // Alpha = (2*pi * 255) * (Fc / Fs)
        // Const = 2 * 3.14159 * 255 ≈ 1602
        
        uint32_t alpha_calc = (1602UL * fcut_hz) / fsampl_hz;

        if (alpha_calc > 255) alpha_calc = 255;
        if (alpha_calc < 1) alpha_calc = 1;
        
        alpha_ = (int16_t)alpha_calc;
    }
};


// Adaptive alpha EMA filter that adjusts filtering strength based on angular velocity
// 
// Key Features:
// - Runtime: Zero division, single multiplication, minimal branching
// - Initialization: Integer-only math (no float), safe for MCUs without FPU
// - Guaranteed [1, 255] output range by construction (pre-calculated limits)
// - Automatically clamps frequency to user-defined [f_min, f_max]
// - Implicit 3x safety margin via slope calculation
//
// Licensed under the Apache License, Version 2.0
// Copyright 2026 Anton Khrustalev, creapunk.com
struct FilterAdaptiveAlphaEMA {
private:
    uint16_t min_speed_clamp_;  // Speed threshold corresponding to max(f_min, alpha=1)
    uint16_t max_speed_clamp_;  // Speed threshold corresponding to min(f_max, alpha=255)
    uint16_t alpha_slope_;      // Pre-calculated slope: alpha_units / speed_unit
    uint8_t alpha_;             // Current calculated alpha value (1-255)

public:
    FilterAdaptiveAlphaEMA() 
        : min_speed_clamp_(0), max_speed_clamp_(0), alpha_slope_(0), alpha_(1) {}

    /// Constructor with frequency range initialization
    /// @param f_sample Sampling frequency in Hz (e.g., 20000)
    /// @param f_min_hz Minimum cutoff frequency in Hz
    /// @param f_max_hz Maximum cutoff frequency in Hz
    FilterAdaptiveAlphaEMA(uint16_t f_sample, uint16_t f_min_hz, uint16_t f_max_hz) {
        // 1. Set Slope (Alpha per d_angle unit)
        // alpha = 2 * PI * f_cutoff / f_sample
        // f_cutoff = f_electrical * 3 (safety margin)
        // f_electrical = (d_angle / 65536) * f_sample
        // --> alpha = 2 * PI * 3 * (d_angle / 65536)
        // 
        // We want alpha in range [0..255]
        // Slope = (2 * PI * 3 * 255) = 4806.6 -> 4806
        // This coefficient is independent of f_sample
        alpha_slope_ = 4806; 

        // 2. Calculate Speed Limits based on User Frequency constraints
        // d_angle = (F_hz * 65536) / F_sample
        // Use uint32_t for intermediate calculation to prevent overflow
        uint32_t scale_num = 65536;
        uint16_t speed_at_f_min = (uint16_t)((f_min_hz * scale_num) / f_sample);
        uint16_t speed_at_f_max = (uint16_t)((f_max_hz * scale_num) / f_sample);

        // 3. Calculate Speed Limits based on Alpha constraints [1..255]
        // alpha = (speed * slope) >> 16
        // speed = (alpha << 16) / slope
        // 
        // speed_at_alpha_1 = (1 * 65536) / 4806 = 13
        uint16_t speed_at_alpha_1 = (uint16_t)((1UL << 16) / alpha_slope_) + 1; 
        
        // speed_at_alpha_255 = (255 * 65536) / 4806 = 3477
        uint16_t speed_at_alpha_255 = (uint16_t)((255UL << 16) / alpha_slope_);

        // 4. Intersect constraints (Take the strictest limits)
        
        // Lower bound: Max of (User Min Freq, Alpha=1)
        min_speed_clamp_ = (speed_at_f_min > speed_at_alpha_1) ? speed_at_f_min : speed_at_alpha_1;
        
        // Upper bound: Min of (User Max Freq, Alpha=255)
        max_speed_clamp_ = (speed_at_f_max < speed_at_alpha_255) ? speed_at_f_max : speed_at_alpha_255;

        // Safety check: ensure min <= max (in case user inputs crazy f_min > f_max)
        if (min_speed_clamp_ > max_speed_clamp_) {
            min_speed_clamp_ = max_speed_clamp_;
        }
        
        alpha_ = 100; // Default safe value
    }

    /// Update alpha based on angular velocity
    /// @param d_angle Angular velocity in INT16 format (change of angle per tick)
    /// @return Calculated alpha value (1-255)
    inline uint8_t tick(int16_t d_angle) {
        // 1. Absolute value
        uint16_t speed = (d_angle < 0) ? (uint16_t)(-d_angle) : (uint16_t)d_angle;

        // 2. Clamp Speed to pre-calculated Safe Range
        if (speed < min_speed_clamp_) {
            speed = min_speed_clamp_;
        } else if (speed > max_speed_clamp_) {
            speed = max_speed_clamp_;
        }

        // 3. Calculate Alpha (Single multiplication)
        // alpha = (speed * slope) / 65536
        alpha_ = (uint8_t)(((uint32_t)speed * alpha_slope_) >> 16);
        
        return alpha_;
    }

    inline uint8_t get_alpha() const { return alpha_; }
};


#pragma once

#include <stdint.h>
#include <math.h>


// First-order float32 LPF / HPF.
//
// Public API intentionally mirrors the original integer FilterEMA:
//
// - alpha = 0   -> alpha_f = 0.0f, output holds indefinitely
// - alpha = 255 -> alpha_f = 1.0f, LPF output equals input exactly
//
// LPF:
//   y[k] = y[k - 1] + alpha * (x[k] - y[k - 1])
//
// HPF:
//   dc[k] = dc[k - 1] + alpha * (x[k] - dc[k - 1])
//   y[k]  = x[k] - dc[k]
//
// Unlike the original Q16.16 implementation, state and output are both
// native float32 values; there is no internal fixed-point quantization.
//
// Licensed under the Apache License, Version 2.0
// Copyright 2024 Anton Khrustalev, creapunk.com
struct FilterEMA_f32 {
private:
    static constexpr float INV_ALPHA_SCALE = 1.0f / 255.0f;

    float state_;      // Internal LPF / DC-estimate state.
    float output_;     // Cached latest LPF or HPF result.
    uint8_t alpha_;    // 0..255, mapped exactly to [0.0f .. 1.0f].

    inline float alpha_float() const {
        return static_cast<float>(alpha_) * INV_ALPHA_SCALE;
    }

public:
    /// Default constructor: zero state, alpha = 1.0.
    FilterEMA_f32()
        : state_(0.0f),
          output_(0.0f),
          alpha_(255) {}

    /**
     * @brief Constructor with initial state and alpha.
     * @param input_default Initial filter state/output.
     * @param alpha_value Coefficient: 0 = hold, 255 = direct pass-through.
     */
    FilterEMA_f32(float input_default, uint8_t alpha_value)
        : state_(input_default),
          output_(input_default),
          alpha_(alpha_value) {}

    /**
     * @brief Execute one low-pass-filter update.
     * @param input Current input sample.
     * @return Filtered output.
     */
    inline float tick_lpf(float input) {
        const float alpha = alpha_float();

        // Equivalent to:
        // state_ = (1.0f - alpha) * state_ + alpha * input;
        //
        // This form needs one subtraction, one multiply and one addition,
        // and is numerically stable for alpha close to zero.
        state_ += alpha * (input - state_);
        output_ = state_;

        return output_;
    }

    /**
     * @brief Execute one high-pass-filter update.
     * @param input Current input sample.
     * @return High-pass output, input minus updated DC estimate.
     */
    inline float tick_hpf(float input) {
        const float alpha = alpha_float();

        // First update the low-frequency/DC estimate.
        state_ += alpha * (input - state_);

        // Then remove it from the original input.
        output_ = input - state_;

        return output_;
    }

    /// @return Last output produced by tick_lpf() or tick_hpf().
    inline float get_output() const {
        return output_;
    }

    /**
     * @brief Set alpha in the retained uint8 representation.
     * @param alpha_value 0 -> 0.0f, 255 -> 1.0f.
     */
    inline void set_alpha(uint8_t alpha_value) {
        alpha_ = alpha_value;
    }

    /// @return Raw uint8 alpha, useful for telemetry/configuration.
    inline uint8_t get_alpha() const {
        return alpha_;
    }

    /**
     * @brief Reset both state and cached output.
     * @param value New initial state in physical float units.
     */
    inline void reset(float value) {
        state_ = value;
        output_ = value;
    }

    /**
     * @brief Calculate and set alpha from cutoff and sampling frequencies.
     *
     * Uses the same small-signal approximation as the original:
     *     alpha = 2*pi*fc/fs
     *
     * It then quantizes this to uint8 alpha such that:
     *     alpha_u8 = round(alpha_float * 255)
     *
     * @param fcut_hz Requested cutoff frequency, Hz.
     * @param fsampl_hz Sampling frequency, Hz.
     */
    inline void set_cutoff(uint16_t fcut_hz, uint16_t fsampl_hz) {
        if (fsampl_hz == 0u) {
            return;
        }

        float alpha = TWO_PI *
                      (static_cast<float>(fcut_hz) /
                       static_cast<float>(fsampl_hz));

        if (alpha <= 0.0f) {
            alpha_ = 0u;
            return;
        }

        if (alpha >= 1.0f) {
            alpha_ = 255u;
            return;
        }

        alpha_ = static_cast<uint8_t>(alpha * 255.0f + 0.5f);
    }
};


/**
 * @brief Adaptive-alpha first-order filter helper.
 *
 * This class intentionally keeps exactly the original public API and returns
 * uint8 alpha. It is appropriate when FilterEMA::set_alpha() remains the
 * configuration interface.
 *
 * Runtime result:
 * - tick() returns alpha in 1..255 for a valid configured interval
 * - that alpha maps to float coefficient alpha / 255.0f in FilterEMA
 *
 * The speed input still uses signed angle increment per tick because it is
 * naturally an encoder-domain quantity.
 *
 * Licensed under the Apache License, Version 2.0
 * Copyright 2026 Anton Khrustalev, creapunk.com
 */
struct FilterAdaptiveAlphaEMA_f32 {
private:
    static constexpr float TWO_PI_TIMES_SAFETY = 18.849555921538759f;
    static constexpr float INV_65536 = 1.0f / 65536.0f;

    uint16_t min_speed_clamp_;
    uint16_t max_speed_clamp_;
    float alpha_slope_;   // Float alpha per d_angle unit, before uint8 mapping.
    uint8_t alpha_;

    static inline uint16_t clamp_u16(float value) {
        if (value <= 0.0f) {
            return 0u;
        }

        if (value >= 65535.0f) {
            return 65535u;
        }

        return static_cast<uint16_t>(value + 0.5f);
    }

public:
    FilterAdaptiveAlphaEMA_f32()
        : min_speed_clamp_(0u),
          max_speed_clamp_(0u),
          alpha_slope_(0.0f),
          alpha_(1u) {}

    /**
     * @brief Configure alpha adaptation limits.
     *
     * The relation follows the original 3x electrical-frequency safety margin:
     *
     *   f_electrical = abs(d_angle) * f_sample / 65536
     *   f_cutoff     = 3 * f_electrical
     *   alpha_float  = 2*pi * f_cutoff / f_sample
     *                = 6*pi * abs(d_angle) / 65536
     *
     * f_sample cancels from the runtime slope, but it remains necessary to
     * convert user cutoff-frequency limits into d_angle limits.
     *
     * @param f_sample Sampling frequency in Hz.
     * @param f_min_hz Minimum permitted cutoff frequency in Hz.
     * @param f_max_hz Maximum permitted cutoff frequency in Hz.
     */
    FilterAdaptiveAlphaEMA_f32(
        uint16_t f_sample,
        uint16_t f_min_hz,
        uint16_t f_max_hz
    )
        : min_speed_clamp_(0u),
          max_speed_clamp_(0u),
          alpha_slope_(0.0f),
          alpha_(1u) {

        if (f_sample == 0u) {
            return;
        }

        // alpha_float = abs(d_angle) * (6*pi / 65536).
        alpha_slope_ = TWO_PI_TIMES_SAFETY * INV_65536;

        const float sample_scale = 65536.0f /
                                   static_cast<float>(f_sample);

        const uint16_t speed_at_f_min =
            clamp_u16(static_cast<float>(f_min_hz) * sample_scale);

        const uint16_t speed_at_f_max =
            clamp_u16(static_cast<float>(f_max_hz) * sample_scale);

        // Convert representable alpha endpoints into speed bounds.
        const float alpha_min = 1.0f / 255.0f;
        const float speed_at_alpha_1_f = alpha_min / alpha_slope_;
        const float speed_at_alpha_255_f = 1.0f / alpha_slope_;

        uint16_t speed_at_alpha_1 =
            clamp_u16(speed_at_alpha_1_f + 1.0f);

        uint16_t speed_at_alpha_255 =
            clamp_u16(speed_at_alpha_255_f);

        min_speed_clamp_ = (speed_at_f_min > speed_at_alpha_1)
            ? speed_at_f_min
            : speed_at_alpha_1;

        max_speed_clamp_ = (speed_at_f_max < speed_at_alpha_255)
            ? speed_at_f_max
            : speed_at_alpha_255;

        if (min_speed_clamp_ > max_speed_clamp_) {
            min_speed_clamp_ = max_speed_clamp_;
        }

        alpha_ = 100u;
    }

    /**
     * @brief Convert angular increment to alpha.
     * @param d_angle Signed encoder-angle increment per sample.
     * @return uint8 alpha in the interval 1..255.
     */
    inline uint8_t tick(int16_t d_angle) {
        // Avoid undefined signed overflow for INT16_MIN.
        int32_t signed_speed = static_cast<int32_t>(d_angle);
        uint16_t speed = static_cast<uint16_t>(
            signed_speed < 0 ? -signed_speed : signed_speed
        );

        if (speed < min_speed_clamp_) {
            speed = min_speed_clamp_;
        } else if (speed > max_speed_clamp_) {
            speed = max_speed_clamp_;
        }

        float alpha = static_cast<float>(speed) * alpha_slope_;

        if (alpha <= (1.0f / 255.0f)) {
            alpha_ = 1u;
        } else if (alpha >= 1.0f) {
            alpha_ = 255u;
        } else {
            alpha_ = static_cast<uint8_t>(alpha * 255.0f + 0.5f);
        }

        return alpha_;
    }

    inline uint8_t get_alpha() const {
        return alpha_;
    }
};