#pragma once

// Implements the high-pass filtering module, removing DC components and low-frequency
// drift from sensor data. Essential for HFI signal processing to extract high-frequency response.
//
// Key Features:
// - Implements a high-pass filter (HPF) by subtracting the low-pass filtered signal from the input.
// - Allows dynamic adjustment of the filter coefficient (`alpha`).
// - Fully integer-based arithmetic for zero-overhead performance on MCUs.
// - Handles signed 16-bit data (int16_t), which is typical for current measurements (ADC).
//
// Detailed Operation:
// The `FilterHPF` struct maintains an internal estimate of the DC component (low-frequency part).
// In each `tick`, it updates this DC estimate using a low-pass filter logic and then calculates
// the high-frequency output by subtracting the DC estimate from the current input.
// Ideally used to remove the fundamental current component in motor control to isolate HFI ripples.
//
// Licensed under the Apache License, Version 2.0
// Copyright 2024 Anton Khrustalev, creapunk.com

#include "../../common.h"

// Defining the FilterHPF struct that implements the high-pass filtering logic.
struct FilterHPF {
    // Filter coefficient (0u..255u)
    // Higher alpha = Faster DC tracking (higher cutoff frequency)
    // Lower alpha = Slower DC tracking (lower cutoff frequency)
    int32_t alpha;
    
    int16_t output;       // The high-frequency component (Input - DC)
    int32_t dc_estimate;  // Internal state: the estimated DC/Low-Frequency component (scaled << 16)

    /// Default constructor
    FilterHPF() : alpha(0), output(0), dc_estimate(0) {}

    /// Constructor to initialize the filter with the input and alpha
    FilterHPF(int16_t input_default, uint8_t alpha_value) {
        alpha = (int32_t)alpha_value;
        output = 0; // Initially, if input matches DC estimate, AC part is 0
        dc_estimate = ((int32_t)input_default) << 16;
    }

    /// Math call
    /// @param input Current signal sample (e.g., measured current)
    /// @return The high-frequency component of the signal (Input - DC)
    int16_t tick(int16_t input) {
        // Convert input to 32-bit scaled format for precision
        int32_t current_scaled = ((int32_t)input) << 16;

        // 1. Update DC Estimate (Low Pass Filter logic)
        // dc_estimate += alpha * (input - dc_estimate)
        // Since alpha is 0..255, we effectively divide by 256 via bit shifts
        
        // Calculate error between current input and estimated DC
        int32_t error = current_scaled - dc_estimate;

        // Scale down error to apply alpha (effectively error / 256)
        error >>= 8;

        // Apply correction to DC estimate
        dc_estimate += (error * alpha);

        // 2. Calculate HPF Output
        // Output = Input - DC_Estimate
        // We take the top 16 bits of dc_estimate to get back to int16 range
        int16_t dc_component = (int16_t)(dc_estimate >> 16);
        
        output = input - dc_component;

        return output;
    }

    /// Function to retrieve the latest high-frequency component
    int16_t get_output() const {
        return output;
    }
    
    /// Function to retrieve the estimated DC component (for debugging)
    int16_t get_dc_component() const {
        return (int16_t)(dc_estimate >> 16);
    }

    /// Function to set the alpha value
    /// @param alpha_value 0-255. Recommended ~10-30 for HFI current cleaning.
    void set_alpha(uint8_t alpha_value) {
        alpha = (int32_t)alpha_value;
    }

    /// Function to reset the filter
    /// Useful when motor state changes drastically to avoid long settling time
    void reset(int16_t current_value) {
        output = 0;
        dc_estimate = ((int32_t)current_value) << 16;
    }
};
