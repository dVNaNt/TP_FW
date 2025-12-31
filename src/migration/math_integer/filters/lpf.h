#pragma once

// Implements the position filtering module, providing low-pass filtering
// functionality to smooth out sensor data and reduce noise in position measurements.
//
// Key Features:
// - Implements a low-pass filter (LPF) for smoothing position data.
// - Allows dynamic adjustment of the filter coefficient (`alpha`).
// - Efficiently updates filtered output with minimal computational overhead.
// - Provides additional 8bit for error storage improoving accuracy of result over time
//
// Detailed Operation:
// The `FilterLPF` struct implements a low-pass filter to smooth incoming position
// data. It maintains an internal state (`temp`) that holds the scaled filtered value.
// The filter operates by calculating the difference between the current input and the
// previous filtered value, scaling this difference by the filter coefficient (`alpha`),
// and updating the internal state accordingly. The `tick` method performs the filtering
// operation, while `get_output` retrieves the current filtered value. The `set_alpha`
// method allows dynamic adjustment of the filter coefficient to modify the filter's
// responsiveness.
//
// Licensed under the Apache License, Version 2.0
// Copyright 2024 Anton Khrustalev, creapunk.com

#include "../../common.h"

// Defining the FilterLPF struct that implements the position filtering logic.
struct FilterLPF {
    // Filter coefficient (0u..255u = 0.0f..1.0f)
    int32_t alpha;
    uint16_t output;
    int32_t temp; // Stores scaled filtered value

    /// Default constructor
    FilterLPF() : alpha(0), output(0), temp(0) {}

    /// Constructor to initialize the filter with the input and alpha
    FilterLPF(uint16_t input_default, uint8_t alpha_value) {
        alpha = (int32_t)alpha_value;
        output = input_default;
        temp = ((int32_t)input_default) << 16;
    }

    /// Math call
    uint16_t tick(uint16_t input) {
        // Convert the input to a 32-bit integer and shift left by 16 bits to allow wrapping as i32
        int32_t current = ((int32_t)input) << 16;

        // LPF filter math: filtered = alpha * (input - prev)
        // For integer alpha u8: filtered = alpha * (input - prev) / 256 + current

        // Get difference between previous (temp is i32 scaled) and current
        int32_t diff = temp - current;

        // Downscale difference to allow alpha scale
        diff >>= 8;

        // Calculate rest part of lpf, temp now will be as prev for next iter (i32 scaled)
        temp = (diff * alpha) + current;

        // Convert to u32 to allow correct bitshift and scale back to u16
        output = (uint16_t)(((uint32_t)temp) >> 16);

        return output;
    }

    /// Function to retrieve the output value
    uint16_t get_output() const {
        return output;
    }

    /// Function to set the alpha value
    void set_alpha(uint8_t alpha_value) {
        alpha = (int32_t)alpha_value;
    }

    /// Function to reset the filter with a new value
    void reset(uint16_t value) {
        output = value;
        temp = ((int32_t)value) << 16;
    }
};


