/**
 * @file current_sense.h
 * @brief Three-channel current sensing with automatic calibration and overcurrent protection
 *
 * This module provides high-performance, integer-only current sensing for three independent
 * motor phase currents (A, B, C). It processes raw ADC readings from current sense amplifiers
 * and converts them to normalized Q15 fixed-point values or absolute milliamp measurements.
 *
 * Negative current (current sinking) = current flowing INTO the motor. Positive current (current sourcing) - regenerative current
 * flow FROM the motor
 *
 * FUNCTIONAL OVERVIEW:
 *
 * The current sensing system operates in three stages:
 * 1. Zero-current calibration: Automatically measures ADC offset when motor is idle
 * 2. Normalization: Converts bipolar ADC readings to Q15 format (-32767 to +32767)
 * 3. Overcurrent protection: Monitors current levels and triggers hardware comparators
 *
 * KEY FEATURES:
 *
 * - Multi-channel support: Independent processing for 3 current sensors
 * - Automatic calibration: Self-calibrating zero-current offset over N samples
 * - Flexible scaling: Per-channel full-scale ranges with automatic normalization
 * - Q15 fixed-point: Fast integer arithmetic optimized for embedded systems
 * - Overcurrent protection: Two-level (soft/hard) limits with hardware comparator integration
 * - Missing sensor detection: Graceful handling of disconnected or failed sensors
 * - Lazy conversion: Milliamp values computed only when requested to minimize CPU load
 *
 * ALGORITHM DETAILS:
 *
 * Calibration Process:
 * - Accumulates N samples of ADC readings when motor is at zero current
 * - Calculates average to determine zero-current offset (cal_val_)
 * - Independent per-channel calibration allows different sensor characteristics
 * - Typical calibration time: 100 samples at 20kHz PWM = 5ms
 *
 * Normalization Algorithm:
 * - Converts raw ADC to bipolar value: bipolar = ADC_raw - cal_val_
 * - Scales to Q15 format: normalized = (bipolar * scale_factor) >> 15
 * - Scale factor computed as: scale = (full_scale_ch / full_scale_max) * 32768
 * - Largest configured range maps to ±32767 (INT16_MAX), smaller ranges scale proportionally
 *
 * Overcurrent Protection:
 * - Soft limit: Software-monitored threshold, sets error flag for control loop response
 * - Hard limit: Hardware-enforced threshold, updates DAC-driven comparator for instant shutdown
 * - Comparator threshold calculation: DAC_value = (limit_Q15 << 15) / scale + cal_val_
 * - Hardware comparators can trigger immediate PWM shutdown without CPU intervention
 *
 * USAGE PATTERN:
 *
 * 1. Construction: CurrentSense(ch1_fs_ma, ch2_fs_ma, ch3_fs_ma)
 * 2. Configuration: set_calibration_samples(100), set_overcurrent_hard_ma(CH1, 5000)
 * 3. Calibration: calibrate() when motor is idle
 * 4. Runtime: tick() called from ADC interrupt with new readings
 * 5. Reading: norm() for Q15 values, get_current_ma() for absolute measurements
 *
 * SENSOR COMPATIBILITY:
 *
 * - Supports bidirectional current sense amplifiers (e.g., INA240, MAX4080)
 * - Handles inverted sensors via negative full-scale configuration
 * - Gracefully handles missing sensors (full_scale = 0 or ADC = UINT16_MAX)
 * - Compatible with both inline and low-side current sensing topologies
 *
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */


#pragma once  // Ensure this header is included only once during compilation


#include "../../../common.h"  // Include common migration definitions and types




// Function pointer type for comparator overcurrent threshold update callback
// Takes threshold value in ADC units (uint16_t) and returns nothing
typedef void (*DAC_SET_COMP_OC_Callback)(uint16_t threshold);


struct CurrentSense {  // Define the CurrentSense structure for 3-channel current sensing
private:
    // ---------------------------------------------------------------------------
    // Private data members
    // ---------------------------------------------------------------------------


    // Callback function pointers for updating comparator thresholds (per channel)
    DAC_SET_COMP_OC_Callback comp_oc_callback_[3] = {nullptr, nullptr,
                                                     nullptr};  // Array of callback pointers for channels A, B, C


    // Absolute current values in milliamperes
    Vec3D_i32 out_ma_ = {INT32_MIN, INT32_MIN, INT32_MIN};  // Absolute current values in milliamperes for each channel


    // Full-scale current amplitude in mA for each channel (0 => sensor missing)
    const Vec3D_i32 full_scale_ma_;  // Maximum current range in milliamps for each channel (0 means sensor absent)


    // Cached maximum full-scale range among channels (mA)
    int32_t full_scale_max_ma;  // The largest full-scale value among all three channels


    // Per-channel calibration
    Vec3D_i32 cal_val_ = {0, 0, 0};  // Calibration accumulator during sampling, then zero-current offset after calibration
    Vec3D cal_n_ = {0, 0, 0};        // Current calibration sample counter (0 to n_samples+1)


    // Precomputed per-channel ratio in Q15:
    //   scale[ch] = full_scale_ma[ch] / full_scale_max_ma * 2^15
    // Range: 0..32768
    Vec3D scale_ = {0, 0, 0};  // Scaling ratios in Q15 fixed-point format for normalization


    // Hard overcurrent limit in Q15 format (per channel).
    Vec3D overcurrent_hard_norm_ = {INT16_MAX, INT16_MAX, INT16_MAX};  // Save in case of recalibration


    Vec3D overcurrent_soft_norm_ = {INT16_MAX, INT16_MAX, INT16_MAX};


    // Normalized outputs (Q15-like), INT16_MIN => sensor missing
    Vec3D out_norm_ = {INT16_MIN, INT16_MIN,
                       INT16_MIN};  // Normalized current outputs in Q15 format (INT16_MIN indicates missing sensor)


    // Calibration configuration
    int16_t n_samples = 100;  // Number of samples for calibration (1 to 1000)


    bool is_changed = false;  // Flag indicating that new data is available and needs conversion
    bool valid = false;       // False by default untill at least one sensor cal started AND full scale != 0



    CurrentSense(int16_t ch1_fs_ma,
                 int16_t ch2_fs_ma,
                 int16_t ch3_fs_ma)  // Constructor: Create and initialize with full-scale ranges and error manager reference
        : full_scale_ma_((int32_t)ch1_fs_ma, (int32_t)ch2_fs_ma, (int32_t)ch3_fs_ma) {
        // Validate and calculate ratios
        if (!(ch1_fs_ma == 0 && ch2_fs_ma == 0 && ch3_fs_ma == 0)) {
            // Find maximum absolute full-scale value among all channels
            int32_t max_scale = 0;
            for (uint8_t ch = 0u; ch < 3u; ++ch) {
                const int32_t abs_fs = (full_scale_ma_[ch] >= 0) ? full_scale_ma_[ch] : -full_scale_ma_[ch];
                if (abs_fs > max_scale) {
                    max_scale = abs_fs;
                }
            }


            full_scale_max_ma = max_scale;  // Cache the maximum full-scale value


            // Q15 ratio: (abs(full_scale_ma) << 15) / max_scale
            // Negative full_scale_ma means inverted sensor
            for (uint8_t ch = 0u; ch < 3u; ++ch) {
                scale_[ch] = (full_scale_ma_[ch] << 14) / max_scale;
            }
        }
    }


    // ---------------------------------------------------------------------------
    // Private helper methods
    // ---------------------------------------------------------------------------


    // Main fast tick: store inputs, progress calibrations (if active), compute normalized outputs. Zero value means sensor
    // inactive
    void tick(const uint16_t currents[3]) {  // Main processing function called each cycle with new ADC readings array
        is_changed = true;                   // Mark that new data is available


        // Short exit
        if (!valid) {                                              // Check if all sensors are missing (no valid full-scale ranges)
            out_norm_.a = out_norm_.b = out_norm_.c = INT16_MIN;  // Set channel 1 2 3 output to sensor missing indicator
            return;                                                // Exit early since no valid sensors are configured
        }


        int8_t oc_error_soft = 0;


        // Calibration (independent per channel) - three-stage process
        for (uint8_t ch = 0u; ch < 3u; ++ch) {     // Iterate through all three channels
            const int16_t cal_n_val = cal_n_[ch];  // Read current calibration counter value
            // Normal run
            if (cal_n_val > n_samples) {
                if (scale_[ch] == 0 or
                    currents[ch] == UINT16_MAX) {  // Check if this specific channel's sensor is missing or invalid
                    out_norm_[ch] = INT16_MIN;     // Set output to sensor missing indicator
                    continue;                      // Skip to next channel
                }


                const int32_t bipolar = (int32_t)currents[ch] - cal_val_[ch];  // Calculate ADC delta from zero-current offset


                // Scale delta by channel ratio: Q15 ratio => Q15 output
                const int32_t scaled = (bipolar * (int32_t)scale_[ch]) >> 15;


                oc_error_soft |= (scaled > overcurrent_soft_norm_[ch]) << ch;


                out_norm_[ch] = clamp_i32(scaled, INT16_MAX);  // Saturate result to 16-bit range and store as normalized output
            }  // Calibration
            else {
                if (cal_n_val < n_samples) {                 // Stage A: Accumulating samples (cal_n < n_samples)
                    cal_val_[ch] += (int32_t)currents[ch];   // Add current raw reading to calibration accumulator
                } else {                                     // Stage B: Calculate average (cal_n == n_samples)
                    cal_val_[ch] = cal_val_[ch] / n_samples;  // Set zero-current offset to calculated average


                    // Update OC_HARD DAC threshold for this specific channel after calibration completes
                    update_oc_hard_dac_threshold(ch);
                }


                cal_n_[ch] = cal_n_val + 1u;  // Increment calibration sample counter
                out_norm_[ch] = INT16_MIN;    // Set output to sensor missing indicator
            }
        }


    }


    // Update OC_HARD DAC threshold value for a specific channel
    // Converts Q15 normalized threshold to ADC format by reversing the normalization process
    // ch: Channel index (0, 1, or 2)
    void update_oc_hard_dac_threshold(uint8_t ch) {
        // Check if channel is valid (has scale factor and is calibrated)
        if (scale_[ch] == 0 || cal_n_[ch] <= n_samples) {
            return;
        }


        // Convert Q15 threshold to ADC delta (bipolar value)
        // overcurrent_hard_norm_[ch] is in Q15 format, scale is also Q15
        // Result: ADC delta from zero point
        int32_t dac_delta = -((int32_t)overcurrent_hard_norm_[ch] << 15) / scale_[ch];


        // Add calibration offset to get absolute ADC value
        int32_t dac_value = dac_delta + cal_val_[ch];


        // Clamp to valid ADC range [0, UINT16_MAX]
        if (dac_value < 0)
            dac_value = 0;
        if (dac_value > UINT16_MAX)
            dac_value = UINT16_MAX;


        // Call the callback function if it's set for this channel
        if (comp_oc_callback_[ch] != nullptr) {
            comp_oc_callback_[ch]((uint16_t)dac_value);  // Invoke callback with new threshold value
        }
    }


    friend struct Feedback;


public:
    // Set the number of calibration samples (1 to 1000)
    inline void set_calibration_samples(int32_t n) {        // Set the number of samples to use for calibration
        n_samples = (n < 1) ? 1 : ((n > 1000) ? 1000 : n);  // Clamp value between 1 and 1000
    }


    // Start calibration for a specific channel
    // Resets cal_n to 0 and cal_val to 0 to begin accumulating samples
    void calibrate(Channel channel) {  // Begin zero-current calibration for specified channel
        valid = full_scale_max_ma != 0;
        cal_n_[channel] = 0u;   // Reset calibration sample counter to zero
        cal_val_[channel] = 0;  // Reset calibration accumulator to zero
    }


    // Start calibration for all channels
    inline void calibrate() {  // Begin zero-current calibration for all channels
        calibrate(CH1);        // Calibrate channel 1
        calibrate(CH2);        // Calibrate channel 2
        calibrate(CH3);        // Calibrate channel 3
    }


    inline bool is_calibrated(Channel ch) const {  // Check if a specific channel is calibrated
        return (cal_n_[ch] > n_samples);           // Return true if calibration is complete (cal_n > n_samples)
    }


    bool is_calibrated() const {  // Check if all channels are calibrated (overload with no parameters)
        return (cal_n_[0] > n_samples) && (cal_n_[1] > n_samples) && (cal_n_[2] > n_samples);  // Return the all calibrated flag
    }


    // Returns pointer to 3-element array of normalized currents (Q15), INT16_MIN => sensor missing.
    inline const int16_t* norm() const {  // Get pointer to array of all normalized current outputs
        return out_norm_.arr;             // Return pointer to the 3-element normalized output array
    }


    inline int16_t norm_ch(Channel ch) const {  // Get normalized current output for a specific channel
        return out_norm_[ch];                   // Return the normalized output for the specified channel
    }


    // Get maximum full-scale current range among all channels (in milliamps)
    inline int32_t get_full_scale_max_ma() const { return full_scale_max_ma; }


    // =========================================================================
    // CONVERSION METHODS: Between normalized Q15 and amperes
    // =========================================================================


    // Convert amperes to normalized Q15 value
    // amps: Current in amperes (float)
    // Returns: Normalized current in Q15 format (-32767..32767)
    int16_t amps2norm(float amps) const {
        if (full_scale_max_ma <= 0)
            return 0;                                          // Avoid division by zero
        const float fs_amps = full_scale_max_ma / 1000.0f;     // Convert max scale mA to A
        int32_t norm = (int32_t)((amps * 32768.0f) / fs_amps); // Scale amps to Q15
        return clamp_i32(norm, INT16_MAX);                     // Return clamped normalized value
    }


    // Convert normalized Q15 value to amperes
    // norm: Normalized current in Q15 format (-32767..32767)
    // Returns: Current in amperes (float)
    inline float norm2amps(int16_t norm) const {
        const float fs_amps = full_scale_max_ma / 1000.0f;      // Convert max scale mA to A
        return ((float)norm * fs_amps) / 32768.0f;              // Scale Q15 to amperes
    }


    // Get absolute current value in milliamperes for a specific channel
    // Converts from normalized Q15 format to milliamperes only when is_changed is true для уменьшения долгих операций
    // Lazy initialization optimized using branchless ternary logic
    const Vec3D_i32 get_current_ma() {
        if (is_changed) {  // Only calculate if new ADC data was parsed
            out_ma_[0] = (out_norm_[0] == INT16_MIN) ? INT32_MIN : (((int32_t)out_norm_[0] * full_scale_max_ma) >> 15);
            out_ma_[1] = (out_norm_[1] == INT16_MIN) ? INT32_MIN : (((int32_t)out_norm_[1] * full_scale_max_ma) >> 15);
            out_ma_[2] = (out_norm_[2] == INT16_MIN) ? INT32_MIN : (((int32_t)out_norm_[2] * full_scale_max_ma) >> 15);
            is_changed = false;
        }
        return out_ma_;
    }


    // =========================================================================
    // OVERCURRENT LIMIT METHODS
    // =========================================================================


    // Set soft overcurrent limit in normalized Q15 format
    inline void set_overcurrent_soft_norm(Channel ch, int16_t limit) {
        overcurrent_soft_norm_[ch] = limit;  // Store soft limit in Q15 format
    }


    // Set soft overcurrent limit in milliamps
    void set_overcurrent_soft_ma(Channel ch, int32_t limit) {
        overcurrent_soft_norm_[ch] = amps2norm((float)limit / 1000.0f);  // Convert mA to A and use amps2norm
    }


    // Set hard overcurrent limit in milliamps for a specific channel
    void set_overcurrent_hard_amps(Channel ch, float limit) {
        overcurrent_hard_norm_[ch] = amps2norm(limit);  // Convert mA to A and use amps2norm
        update_oc_hard_dac_threshold(ch);             // Recalculate DAC threshold for this channel
    }


    // =========================================================================
    // COMPARATOR CALLBACK METHODS
    // =========================================================================


    // Set callback function for comparator threshold update for a specific channel
    // ch: Channel to set callback for (CH1, CH2, or CH3)
    // callback: Function pointer to be called when threshold is updated (or nullptr to clear)
    inline void set_overcurrent_hard_callback(Channel ch, DAC_SET_COMP_OC_Callback callback) {
        comp_oc_callback_[ch] = callback;  // Store callback pointer for specified channel
    }
};  // End of CurrentSense structure definition