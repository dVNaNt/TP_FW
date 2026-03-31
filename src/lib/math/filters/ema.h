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
        if (fsampl_hz == 0) return; // Защита от деления на 0

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
