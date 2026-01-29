#pragma once  // Ensure this header is included only once during compilation

// Sequence: calibrating offset -> recalculating normalized full scale -> convert

// 3-channel current sensing helper (integer-only, fast path).
// - Input: 3x uint16_t raw current sensor ADC values
// - Per-channel zero calibration over N ticks (independent)
// - Per-channel full-scale current range in mA (0 => sensor missing)
// - Output: 3x int16_t normalized currents, where the *largest* configured range
//   is treated as INT16_MAX (Q15), with INT16_MIN reserved as "sensor missing".

#include "../../common.h"  // Include common migration definitions and types

// Function pointer type for comparator overcurrent threshold update callback
// Takes threshold value in ADC units (uint16_t) and returns nothing
typedef void (*DAC_SET_COMP_OC_Callback)(uint16_t threshold);

struct CurrentSense {  // Define the CurrentSense structure for 3-channel current sensing
public:
    // Channel enumeration for type-safe channel selection
    enum Channel : uint8_t {
        CH1 = 0,  // Channel 1 (array index 0)
        CH2 = 1,  // Channel 2 (array index 1)
        CH3 = 2   // Channel 3 (array index 2)
    };

private:
    // ---------------------------------------------------------------------------
    // Private data members
    // ---------------------------------------------------------------------------

    // Absolute current values in milliamperes
    Vec3D_i32 out_ma_;  // Absolute current values in milliamperes for each channel

    // Full-scale current amplitude in mA for each channel (0 => sensor missing)
    Vec3D_i32 full_scale_ma_;  // Maximum current range in milliamps for each channel (0 means sensor absent)

    // Cached maximum full-scale range among channels (mA)
    int32_t full_scale_max_ma;  // The largest full-scale value among all three channels

    // Per-channel calibration
    Vec3D_i32 cal_val_;  // Calibration accumulator during sampling, then zero-current offset after calibration
    Vec3D cal_n_;        // Current calibration sample counter (0 to n_samples+1)

    // Precomputed per-channel ratio in Q15:
    //   scale[ch] = full_scale_ma[ch] / full_scale_max_ma * 2^15
    // Range: 0..32768
    Vec3D scale_;  // Scaling ratios in Q15 fixed-point format for normalization

    // Hard overcurrent limit in Q15 format (per channel).
    Vec3D overcurrent_hard_norm_;  // Save in case of recalibration

    Vec3D overcurrent_soft_norm_;

    // Normalized outputs (Q15-like), INT16_MIN => sensor missing
    Vec3D out_norm_;  // Normalized current outputs in Q15 format (INT16_MIN indicates missing sensor)

    // Calibration configuration
    int16_t n_samples;  // Number of samples for calibration (1 to 1000)

    uint8_t oc_error_soft;
    uint8_t oc_error_hard;

    bool all_calibrated;  // Flag indicating all channels are calibrated
    bool is_changed;      // Flag indicating that new data is available and needs conversion
    bool valid;           // False by default untill at least one sensor cal started AND full scale != 0

    // Callback function pointers for updating comparator thresholds (per channel)
    DAC_SET_COMP_OC_Callback comp_oc_callback_[3];  // Array of callback pointers for channels A, B, C

    // ---------------------------------------------------------------------------
    // Private helper methods
    // ---------------------------------------------------------------------------

    static inline int16_t sat_i32_to_i16_no_min(int32_t x) {  // Saturate 32-bit integer to 16-bit range, avoiding INT16_MIN
        // Reserve INT16_MIN as a sentinel "sensor missing".
        if (x > INT16_MAX)
            return INT16_MAX;  // Clamp to maximum positive 16-bit value if input exceeds it
        if (x < -INT16_MAX)
            return (int16_t)(-INT16_MAX);  // Clamp to -32767 (not -32768) to reserve INT16_MIN
        return (int16_t)x;                 // Return the value as 16-bit integer if within valid range
    }

    // Main fast tick: store inputs, progress calibrations (if active), compute normalized outputs. Zero value means sensor
    // inactive
    void tick(const uint16_t currents[3]) {  // Main processing function called each cycle with new ADC readings array
        is_changed = true;                   // Mark that new data is available

        // Short exit
        if (!valid) {                 // Check if all sensors are missing (no valid full-scale ranges)
            out_norm_.a = INT16_MIN;  // Set channel 0 output to sensor missing indicator
            out_norm_.b = INT16_MIN;  // Set channel 1 output to sensor missing indicator
            out_norm_.c = INT16_MIN;  // Set channel 2 output to sensor missing indicator
            return;                   // Exit early since no valid sensors are configured
        }

        const int32_t raw[3] = {(int32_t)currents[0], (int32_t)currents[1],
                                (int32_t)currents[2]};  // Store raw ADC readings in local array

        oc_error_soft = 0;

        // Calibration (independent per channel) - three-stage process
        for (uint8_t ch = 0u; ch < 3u; ++ch) {     // Iterate through all three channels
            const int16_t cal_n_val = cal_n_[ch];  // Read current calibration counter value
            // Normal run
            if (cal_n_val > n_samples) {
                if (scale_[ch] == 0 or raw[ch] == UINT16_MAX) {  // Check if this specific channel's sensor is missing or invalid
                    out_norm_[ch] = INT16_MIN;                   // Set output to sensor missing indicator
                    continue;                                    // Skip to next channel
                }

                const int32_t bipolar = (int32_t)raw[ch] - cal_val_[ch];  // Calculate ADC delta from zero-current offset

                // Scale delta by channel ratio: Q15 ratio => Q15 output
                const int32_t scaled = (bipolar * (int32_t)scale_[ch]) >> 15;

                oc_error_soft |= (scaled > overcurrent_soft_norm_[ch]) << ch;

                out_norm_[ch] = sat_i32_to_i16_no_min(scaled);  // Saturate result to 16-bit range and store as normalized output
            }  // Calibration
            else if (cal_n_val < n_samples) {  // Stage A: Accumulating samples (cal_n < n_samples)
                cal_val_[ch] += raw[ch];       // Add current raw reading to calibration accumulator
                cal_n_[ch] = cal_n_val + 1u;   // Increment calibration sample counter
                out_norm_[ch] = INT16_MIN;     // Set output to sensor missing indicator
            }  // Calibration finish
            else {                                        // Stage B: Calculate average (cal_n == n_samples)
                cal_val_[ch] = cal_val_[ch] / n_samples;  // Set zero-current offset to calculated average
                // Calculate noise amplitude
                cal_n_[ch] = n_samples + 1u;  // Increment cal_n to n_samples+1 to move to stage C

                // Check if all channels are calibrated
                all_calibrated = (cal_n_[0] > n_samples) && (cal_n_[1] > n_samples) && (cal_n_[2] > n_samples);

                // Update OC_HARD DAC threshold for this specific channel after calibration completes
                update_oc_hard_dac_threshold(ch);

                out_norm_[ch] = INT16_MIN;  // Set output to sensor missing indicator
            }
        }
    }

    friend struct Feedback;

public:
    CurrentSense(int16_t ch1_fs_ma,
                 int16_t ch2_fs_ma,
                 int16_t ch3_fs_ma)  // Constructor: Create and initialize with full-scale ranges
        : full_scale_ma_((int32_t)ch1_fs_ma, (int32_t)ch2_fs_ma, (int32_t)ch3_fs_ma),
          full_scale_max_ma(0),
          scale_(0, 0, 0),
          cal_n_(0, 0, 0),
          cal_val_(0, 0, 0),
          n_samples(100),
          all_calibrated(false),
          out_norm_(INT16_MIN, INT16_MIN, INT16_MIN),
          out_ma_(INT32_MIN, INT32_MIN, INT32_MIN),
          is_changed(false),
          valid(false),
          oc_error_soft(0),
          overcurrent_soft_norm_(INT16_MAX, INT16_MAX, INT16_MAX),  // Initialize soft overcurrent limit to maximum
          overcurrent_hard_norm_(INT16_MAX,
                                 INT16_MAX,
                                 INT16_MAX),            // Initialize hard overcurrent limits to maximum for all channels
          comp_oc_callback_{nullptr, nullptr, nullptr}  // Initialize callback pointers to null for all channels
    {
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

    // Set the number of calibration samples (1 to 1000)
    inline void set_calibration_samples(int32_t n) {        // Set the number of samples to use for calibration
        n_samples = (n < 1) ? 1 : ((n > 1000) ? 1000 : n);  // Clamp value between 1 and 1000
    }

    // Start calibration for a specific channel
    // Resets cal_n to 0 and cal_val to 0 to begin accumulating samples
    void calibrate(Channel channel) {  // Begin zero-current calibration for specified channel
        valid = full_scale_max_ma != 0;
        cal_n_[channel] = 0u;    // Reset calibration sample counter to zero
        cal_val_[channel] = 0;   // Reset calibration accumulator to zero
        all_calibrated = false;  // Reset all calibrated flag since we're starting a new calibration
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

    inline bool is_calibrated() const {  // Check if all channels are calibrated (overload with no parameters)
        return all_calibrated;           // Return the all calibrated flag
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
    // CONVERSION METHODS: Between normalized Q15 and milliamps
    // =========================================================================

    // Convert milliamps to normalized Q15 value
    // ma: Current in milliamps
    // Returns: Normalized current in Q15 format (-32767..32767)
    int16_t ma2norm(int32_t ma) const {
        if (full_scale_max_ma <= 0)
            return 0;                                   // Avoid division by zero
        int32_t norm = (ma << 15) / full_scale_max_ma;  // Scale milliamps to Q15
        if (norm > INT16_MAX)
            return INT16_MAX;  // Clamp to max positive
        if (norm < -INT16_MAX)
            return -INT16_MAX;  // Clamp to max negative (avoid INT16_MIN)
        return (int16_t)norm;   // Return normalized value
    }

    // Convert normalized Q15 value to milliamps
    // norm: Normalized current in Q15 format (-32767..32767)
    // Returns: Current in milliamps
    inline int32_t norm2ma(int16_t norm) const {
        return ((int32_t)norm * full_scale_max_ma) >> 15;  // Scale Q15 to milliamps
    }

    // Get absolute current value in milliamperes for a specific channel
    // Converts from normalized Q15 format to milliamperes only when is_changed is true
    const Vec3D_i32 get_current_ma() {
        if (is_changed) {
            // Convert normalized values to milliamperes for all channels
            for (uint8_t i = 0u; i < 3u; ++i) {
                if (out_norm_[i] == INT16_MIN) {
                    out_ma_[i] = INT32_MIN;  // Sensor missing or not calibrated
                } else {
                    // Convert Q15 normalized value to milliamperes
                    // out_norm is in range [-32767, 32767] representing [-full_scale_max_ma, full_scale_max_ma]
                    out_ma_[i] = ((int32_t)out_norm_[i] * full_scale_max_ma) >> 15;
                }
            }
            is_changed = false;  // Reset flag after conversion
        }

        // Return absolute value
        return out_ma_;
    }

    // =========================================================================
    // OVERCURRENT LIMIT METHODS
    // =========================================================================

    // // Set soft overcurrent limit in normalized Q15 format
    // inline void set_overcurrent_soft_norm(int16_t limit) {
    //     overcurrent_soft_norm = limit;  // Store soft limit in Q15 format
    // }

    // // Set soft overcurrent limit in milliamps
    // void set_overcurrent_soft_ma(int32_t limit) {
    //     overcurrent_soft_norm = ma2norm(limit);  // Convert milliamps to Q15 and store
    // }

    // Set hard overcurrent limit in normalized Q15 format for a specific channel
    inline void set_overcurrent_hard_norm(Channel ch, int16_t limit) {
        overcurrent_hard_norm_[ch] = limit;  // Store hard limit in Q15 format for specified channel
        update_oc_hard_dac_threshold(ch);    // Recalculate DAC threshold for this channel
    }

    // Set hard overcurrent limit in normalized Q15 format for all channels (same value)
    inline void set_overcurrent_hard_norm(int16_t limit) {
        overcurrent_hard_norm_.a = limit;  // Store hard limit for channel 1
        overcurrent_hard_norm_.b = limit;  // Store hard limit for channel 2
        overcurrent_hard_norm_.c = limit;  // Store hard limit for channel 3
        update_oc_hard_dac_thresholds();   // Recalculate DAC thresholds for all channels
    }

    // Set hard overcurrent limit in milliamps for a specific channel
    void set_overcurrent_hard_ma(Channel ch, int32_t limit) {
        overcurrent_hard_norm_[ch] = ma2norm(limit);  // Convert milliamps to Q15 and store for specified channel
        update_oc_hard_dac_threshold(ch);             // Recalculate DAC threshold for this channel
    }

    // Set hard overcurrent limit in milliamps for all channels (same value)
    void set_overcurrent_hard_ma(int32_t limit) {
        int16_t norm_limit = ma2norm(limit);    // Convert milliamps to Q15
        overcurrent_hard_norm_.a = norm_limit;  // Store for channel 1
        overcurrent_hard_norm_.b = norm_limit;  // Store for channel 2
        overcurrent_hard_norm_.c = norm_limit;  // Store for channel 3
        update_oc_hard_dac_thresholds();        // Recalculate DAC thresholds for all channels
    }

    // Get hard overcurrent limit in normalized Q15 format for a specific channel
    inline int16_t get_overcurrent_hard_norm(Channel ch) const {
        return overcurrent_hard_norm_[ch];  // Return hard limit for specified channel
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
        int32_t dac_delta = ((int32_t)overcurrent_hard_norm_[ch] << 15) / scale_[ch];

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

    // Update OC_HARD DAC threshold values for all channels
    void update_oc_hard_dac_thresholds() {
        for (uint8_t ch = 0u; ch < 3u; ++ch) {
            update_oc_hard_dac_threshold(ch);  // Update each channel individually
        }
    }

    // // Check if current exceeds soft limit (absolute value comparison)
    // inline bool is_overcurrent_soft_norm(int32_t current_norm) const {
    //     int32_t abs_current = (current_norm >= 0) ? current_norm : -current_norm;  // Get absolute value
    //     return (int32_t)abs_current >= overcurrent_soft_norm;                      // Compare with soft limit
    // }

    // Check if current exceeds hard limit for a specific channel (absolute value comparison)
    inline bool is_overcurrent_hard_norm(Channel ch, int16_t current_norm) const {
        int16_t abs_current = (current_norm >= 0) ? current_norm : -current_norm;  // Get absolute value
        return abs_current >= overcurrent_hard_norm_[ch];  // Compare with hard limit for specified channel
    }

    // // Check if any channel exceeds soft overcurrent limit
    // inline bool is_any_overcurrent_soft() const {
    //     return is_overcurrent_soft_norm(out_norm_.a) ||  // Check channel 1
    //            is_overcurrent_soft_norm(out_norm_.b) ||  // Check channel 2
    //            is_overcurrent_soft_norm(out_norm_.c);    // Check channel 3
    // }

    // Check if any channel exceeds hard overcurrent limit
    inline bool is_any_overcurrent_hard() const {
        return is_overcurrent_hard_norm(CH1, out_norm_.a) ||  // Check channel 1
               is_overcurrent_hard_norm(CH2, out_norm_.b) ||  // Check channel 2
               is_overcurrent_hard_norm(CH3, out_norm_.c);    // Check channel 3
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
