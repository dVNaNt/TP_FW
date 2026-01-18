#pragma once  // Ensure this header is included only once during compilation

// Sequence: calibrating offset -> recalculating normalized full scale -> convert

// 3-channel current sensing helper (integer-only, fast path).
// - Input: 3x uint16_t raw current sensor ADC values
// - Per-channel zero calibration over N ticks (independent)
// - Per-channel full-scale current range in mA (0 => sensor missing)
// - Output: 3x int16_t normalized currents, where the *largest* configured range
//   is treated as INT16_MAX (Q15), with INT16_MIN reserved as "sensor missing".

#include "../../common.h"  // Include common migration definitions and types

// Helper struct for passing 3 raw current values by value
struct RawCurrents3 {
    uint16_t values[3];
};

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

    // Full-scale current amplitude in mA for each channel (0 => sensor missing)
    Vec3D_i32 full_scale_ma_;  // Maximum current range in milliamps for each channel (0 means sensor absent)

    // Cached maximum full-scale range among channels (mA)
    int32_t full_scale_max_ma;  // The largest full-scale value among all three channels

    // Precomputed per-channel ratio in Q15:
    //   scale[ch] = full_scale_ma[ch] / full_scale_max_ma * 2^15
    // Range: 0..32768
    Vec3D_i32 scale_;  // Scaling ratios in Q15 fixed-point format for normalization

    // Per-channel calibration
    Vec3D_i32 cal_n_;    // Current calibration sample counter (0 to n_samples+1)
    Vec3D_i32 cal_val_;  // Calibration accumulator during sampling, then zero-current offset after calibration

    // Noise measurement during calibration
    Vec3D_i32 cal_min_;  // Minimum value during calibration for noise calculation
    Vec3D_i32 cal_max_;  // Maximum value during calibration for noise calculation
    Vec3D_i32 noise_amplitude_;  // Measured noise amplitude (max - min) for each channel
    Vec3D_i32 noise_amplitude_norm_;  // Normalized noise amplitude (Q15 format, relative to max noise)
    int32_t max_noise_norm;  // Maximum noise amplitude among all channels in normalized form (Q15, relative to INT16_MAX full amplitude)

    // Calibration configuration
    int32_t n_samples;    // Number of samples for calibration (1 to 1000)
    bool all_calibrated;  // Flag indicating all channels are calibrated

    // Normalized outputs (Q15-like), INT16_MIN => sensor missing
    Vec3D out_norm_;  // Normalized current outputs in Q15 format (INT16_MIN indicates missing sensor)

    // Absolute current values in milliamperes
    Vec3D_i32 out_ma_;  // Absolute current values in milliamperes for each channel

    bool is_changed;    // Flag indicating that new data is available and needs conversion

    bool valid; // False by default untill at least one sensor cal started AND full scale != 0

    int16_t overcurrent_soft_norm;  // Soft overcurrent limit in Q15 format
    int16_t overcurrent_hard_norm;  // Hard overcurrent limit in Q15 format

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

    // Main fast tick: store inputs, progress calibrations (if active), compute normalized outputs. Zero value means sensor inactive
    void tick(const uint16_t currents[3]) {  // Main processing function called each cycle with new ADC readings array
        is_changed = true;  // Mark that new data is available
        
        // Short exit
        if (!valid) {                                          // Check if all sensors are missing (no valid full-scale ranges)
            out_norm_.a = INT16_MIN;                           // Set channel 0 output to sensor missing indicator
            out_norm_.b = INT16_MIN;                           // Set channel 1 output to sensor missing indicator
            out_norm_.c = INT16_MIN;                           // Set channel 2 output to sensor missing indicator
            return;                                            // Exit early since no valid sensors are configured
        }

        const int32_t raw[3] = {(int32_t)currents[0], (int32_t)currents[1], (int32_t)currents[2]};  // Store raw ADC readings in local array

        // Calibration (independent per channel) - three-stage process
        for (uint8_t ch = 0u; ch < 3u; ++ch) {          // Iterate through all three channels
            const int32_t cal_n_val = cal_n_[ch];        // Read current calibration counter value
            // Normal run
            if (cal_n_val > n_samples) {
                if (scale_[ch] == 0 or raw[ch] == UINT16_MAX) {  // Check if this specific channel's sensor is missing or invalid
                    out_norm_[ch] = INT16_MIN;              // Set output to sensor missing indicator
                    continue;                              // Skip to next channel
                }

                const int32_t bipolar = (int32_t)raw[ch] - cal_val_[ch];  // Calculate ADC delta from zero-current offset
                                
                // Scale delta by channel ratio: Q15 ratio => Q15 output
                const int32_t scaled = (bipolar * scale_[ch]) >> 15;
                
                out_norm_[ch] = sat_i32_to_i16_no_min(scaled);  // Saturate result to 16-bit range and store as normalized output
            } // Calibration
            else if (cal_n_val < n_samples) 
            {                // Stage A: Accumulating samples (cal_n < n_samples)
                cal_val_[ch] += raw[ch];                 // Add current raw reading to calibration accumulator
                // Track min/max for noise measurement
                if (raw[ch] < cal_min_[ch]) cal_min_[ch] = raw[ch];
                if (raw[ch] > cal_max_[ch]) cal_max_[ch] = raw[ch];
                cal_n_[ch] = cal_n_val + 1u;             // Increment calibration sample counter
                out_norm_[ch] = INT16_MIN;               // Set output to sensor missing indicator
            } // Calibration finish
            else {        // Stage B: Calculate average (cal_n == n_samples)
                cal_val_[ch] = cal_val_[ch] / n_samples;  // Set zero-current offset to calculated average
                // Calculate noise amplitude
                noise_amplitude_[ch] = cal_max_[ch] - cal_min_[ch];
                cal_n_[ch] = n_samples + 1u;             // Increment cal_n to n_samples+1 to move to stage C

                // Check if all channels are calibrated
                all_calibrated = (cal_n_[0] > n_samples) && (cal_n_[1] > n_samples) && (cal_n_[2] > n_samples);
                
                // Normalize noise amplitudes when all channels are calibrated
                if (all_calibrated) {
                    int32_t max_noise = noise_amplitude_[0];
                    if (noise_amplitude_[1] > max_noise) max_noise = noise_amplitude_[1];
                    if (noise_amplitude_[2] > max_noise) max_noise = noise_amplitude_[2];
                    
                    // Q15 normalization: (noise << 15) / max_noise
                    if (max_noise) {
                        noise_amplitude_norm_[0] = (noise_amplitude_[0] << 15) / max_noise;
                        noise_amplitude_norm_[1] = (noise_amplitude_[1] << 15) / max_noise;
                        noise_amplitude_norm_[2] = (noise_amplitude_[2] << 15) / max_noise;
                    }
                    
                    // Calculate max noise normalized to INT16_MAX (full sensor amplitude)
                    // max_noise is in ADC units (0-65535), normalize to Q15 (INT16_MAX = 32767)
                    // Formula: (max_noise * INT16_MAX) / UINT16_MAX
                    max_noise_norm = (max_noise * INT16_MAX) / UINT16_MAX;
                }
                out_norm_[ch] = INT16_MIN;  // Set output to sensor missing indicator
            }
        }
    }

    friend struct Feedback;

public:
    CurrentSense(int16_t ch1_fs_ma, int16_t ch2_fs_ma, int16_t ch3_fs_ma)  // Constructor: Create and initialize with full-scale ranges
        : full_scale_ma_((int32_t)ch1_fs_ma, (int32_t)ch2_fs_ma, (int32_t)ch3_fs_ma),
          full_scale_max_ma(0),
          scale_(0, 0, 0),
          cal_n_(0, 0, 0),
          cal_val_(0, 0, 0),
          cal_min_(INT32_MAX, INT32_MAX, INT32_MAX),
          cal_max_(INT32_MIN, INT32_MIN, INT32_MIN),
          noise_amplitude_(0, 0, 0),
          noise_amplitude_norm_(0, 0, 0),
          max_noise_norm(0),
          n_samples(100),
          all_calibrated(false),
          out_norm_(INT16_MIN, INT16_MIN, INT16_MIN),
          out_ma_(INT32_MIN, INT32_MIN, INT32_MIN),
          is_changed(false),
          valid(false),
          overcurrent_soft_norm(INT16_MAX),  // Initialize soft overcurrent limit to maximum
          overcurrent_hard_norm(INT16_MAX)   // Initialize hard overcurrent limit to maximum
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
                scale_[ch] = (full_scale_ma_[ch]  << 14) / max_scale;
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
        cal_n_[channel] = 0u;                  // Reset calibration sample counter to zero
        cal_val_[channel] = 0;                 // Reset calibration accumulator to zero
        cal_min_[channel] = INT32_MAX;         // Reset minimum value for noise measurement
        cal_max_[channel] = INT32_MIN;         // Reset maximum value for noise measurement
        noise_amplitude_norm_[channel] = 0;    // Reset normalized noise amplitude
        max_noise_norm = 0;                   // Reset maximum noise normalized value
        all_calibrated = false;               // Reset all calibrated flag since we're starting a new calibration
    }

    // Start calibration for all channels
    inline void calibrate() {  // Begin zero-current calibration for all channels
        calibrate(CH1);        // Calibrate channel 1
        calibrate(CH2);        // Calibrate channel 2
        calibrate(CH3);        // Calibrate channel 3
    }

    inline bool is_calibrated(Channel ch) const {  // Check if a specific channel is calibrated
        return (cal_n_[ch] > n_samples);            // Return true if calibration is complete (cal_n > n_samples)
    }

    inline bool is_calibrated() const {  // Check if all channels are calibrated (overload with no parameters)
        return all_calibrated;           // Return the all calibrated flag
    }

    // Returns pointer to 3-element array of normalized currents (Q15), INT16_MIN => sensor missing.
    inline const int16_t* norm() const {  // Get pointer to array of all normalized current outputs
        return out_norm_.arr;                  // Return pointer to the 3-element normalized output array
    }

    inline int16_t norm_ch(Channel ch) const {  // Get normalized current output for a specific channel
        return out_norm_[ch];                    // Return the normalized output for the specified channel
    }


    // Get normalized noise amplitude for a specific channel (Q15 format, relative to max noise)
    inline int32_t get_noise_amplitude_norm(Channel ch) const {
        return noise_amplitude_norm_[ch];
    }

    // Get pointer to array of all noise amplitudes (raw ADC units)
    inline const int32_t* get_noise_amplitudes() const {
        return noise_amplitude_.arr;
    }

    // Get pointer to array of all normalized noise amplitudes (Q15 format)
    inline const int32_t* get_noise_amplitudes_norm() const {
        return noise_amplitude_norm_.arr;
    }
    
    // Get maximum noise amplitude among all channels in normalized form (Q15, relative to INT16_MAX full amplitude)
    // Returns value in range [0, INT16_MAX] where INT16_MAX represents full sensor amplitude
    inline int32_t get_max_noise_norm() const {
        return max_noise_norm;
    }
    
    // Get maximum full-scale current range among all channels (in milliamps)
    inline int32_t get_full_scale_max_ma() const {
        return full_scale_max_ma;
    }

    // =========================================================================
    // CONVERSION METHODS: Between normalized Q15 and milliamps
    // =========================================================================
    
    // Convert milliamps to normalized Q15 value
    // ma: Current in milliamps
    // Returns: Normalized current in Q15 format (-32767..32767)
    int16_t ma2norm(int32_t ma) const {
        if (full_scale_max_ma <= 0) return 0;           // Avoid division by zero
        int32_t norm = (ma << 15) / full_scale_max_ma;  // Scale milliamps to Q15
        if (norm > INT16_MAX) return INT16_MAX;         // Clamp to max positive
        if (norm < -INT16_MAX) return -INT16_MAX;       // Clamp to max negative (avoid INT16_MIN)
        return (int16_t)norm;                           // Return normalized value
    }
    
    // Convert normalized Q15 value to milliamps
    // norm: Normalized current in Q15 format (-32767..32767)
    // Returns: Current in milliamps
    inline int32_t norm2ma(int16_t norm) const {
        return ((int32_t)norm * full_scale_max_ma) >> 15;  // Scale Q15 to milliamps
    }

    // Get absolute current value in milliamperes for a specific channel
    // Converts from normalized Q15 format to milliamperes only when is_changed is true
    const int32_t* get_current_ma() {
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
        return out_ma_.arr;
    }

    // =========================================================================
    // OVERCURRENT LIMIT METHODS
    // =========================================================================

    // Set soft overcurrent limit in normalized Q15 format
    inline void set_overcurrent_soft_norm(int16_t limit) {
        overcurrent_soft_norm = limit;  // Store soft limit in Q15 format
    }
    

    // Set soft overcurrent limit in milliamps
    void set_overcurrent_soft_ma(int32_t limit) {
        overcurrent_soft_norm = ma2norm(limit);  // Convert milliamps to Q15 and store
    }

    // Set hard overcurrent limit in normalized Q15 format
    inline void set_overcurrent_hard_norm(int16_t limit) {
        overcurrent_hard_norm = limit;  // Store hard limit in Q15 format
    }

    // Set hard overcurrent limit in milliamps
    void set_overcurrent_hard_ma(int32_t limit) {
        overcurrent_hard_norm = ma2norm(limit);  // Convert milliamps to Q15 and store
    }

    // Check if current exceeds soft limit (absolute value comparison)
    inline bool is_overcurrent_soft_norm(int32_t current_norm) const {
        int32_t abs_current = (current_norm >= 0) ? current_norm : -current_norm;  // Get absolute value
        return (int32_t)abs_current >= overcurrent_soft_norm;  // Compare with soft limit
    }

    // Check if current exceeds hard limit (absolute value comparison)
    inline bool is_overcurrent_hard_norm(int16_t current_norm) const {
        int16_t abs_current = (current_norm >= 0) ? current_norm : -current_norm;  // Get absolute value
        return abs_current >= overcurrent_hard_norm;  // Compare with hard limit
    }

    // Check if any channel exceeds soft overcurrent limit
    inline bool is_any_overcurrent_soft() const {
        return is_overcurrent_soft_norm(out_norm_.a) ||  // Check channel 1
               is_overcurrent_soft_norm(out_norm_.b) ||  // Check channel 2
               is_overcurrent_soft_norm(out_norm_.c);    // Check channel 3
    }

    // Check if any channel exceeds hard overcurrent limit
    inline bool is_any_overcurrent_hard() const {
        return is_overcurrent_hard_norm(out_norm_.a) ||  // Check channel 1
               is_overcurrent_hard_norm(out_norm_.b) ||  // Check channel 2
               is_overcurrent_hard_norm(out_norm_.c);    // Check channel 3
    }
};  // End of CurrentSense structure definition
