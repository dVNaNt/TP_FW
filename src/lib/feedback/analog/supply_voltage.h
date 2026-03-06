#pragma once  // Ensure this header is included only once during compilation

// Supply voltage sensing with low-pass filtering
// - Input: uint16_t raw ADC value (0..65535) or up to 23-bit signed values
// - Output: int16_t normalized voltage (Q15 format, -32768..32767)
// - Output: int32_t voltage in millivolts
//
// The filter smooths ADC readings to reduce noise and provide stable voltage measurements.
// Uses precomputed scale factor for optimal resolution conversion between normalized and millivolt values.

#include "../../common.h"                          // Include common migration definitions and types
#include "../../math_integer/filters/ema.h"  // Include first-order filter for smoothing

// Forward declaration of ErrorManager to avoid circular dependency
struct ErrorManager;

// Function pointer type for comparator overvoltage threshold update callback
// Takes threshold value in ADC units (uint16_t) and returns nothing
typedef void (*DAC_SET_COMP_OV_Callback)(uint16_t threshold);

// Supply voltage structure with filtering and normalization
struct SupplyVoltage {  // Define the SupplyVoltage structure for voltage sensing
private:
    // ---------------------------------------------------------------------------
    // Private data members
    // ---------------------------------------------------------------------------
    ErrorManager& error_;  // Reference to error manager for reporting voltage faults
    
    // Callback function pointer for updating comparator threshold
    DAC_SET_COMP_OV_Callback comp_ov_callback_;  // Callback pointer for overvoltage comparator threshold updates

    FilterEMA filter_;  // Low-pass filter instance for smoothing ADC readings

    const int32_t max_voltage_mv_;  // Maximum voltage in millivolts for scaling (e.g., 24000 for 24V)

    int32_t scale_;  // Precomputed scale factor for norm<->mv conversion with minimal precision loss

    int32_t voltage_mv_;  // Voltage in millivolts

    int16_t voltage_norm_;  // Normalized voltage output in Q15 format (-32768..32767)

    int16_t overvoltage_soft_norm;   // Soft overvoltage limit in Q15 format
    int16_t undervoltage_soft_norm;  // Soft undervoltage limit in Q15 format

    bool is_updated;

    

    // Main processing function: Update filter and calculate normalized/millivolt outputs
    // raw_adc: Raw ADC reading (0..65535, where 65535 represents max_voltage_mv)
    void tick(uint16_t raw_adc) {   // Main processing function called each cycle with new ADC reading
        filter_.tick_lpf(raw_adc);  // Update low-pass filter with new ADC value

        int16_t filtered = filter_.get_output() >> 1;  // Get filtered ADC value (0..65535)

        // Convert to normalized Q15 format: shift right by 1 to map 0..65535 to 0..32767
        // Then cast to signed int16_t for Q15 representation
        voltage_norm_ = filtered;  // Normalize to Q15 format (0..32767)

        is_updated = true;
    }

    // Update OV_HARD DAC threshold value
    // Converts Q15 normalized threshold to ADC format by reversing the normalization process
    void update_ov_hard_dac_threshold(int16_t threshold) {
        // Call the callback function if it's set
        if (comp_ov_callback_ != nullptr) {
            // Convert Q15 threshold to ADC value
            // overvoltage_hard_norm is in Q15 format (0..32767)
            // Shift left by 1 to convert back to ADC range (0..65535)
            uint16_t dac_value = (uint16_t)threshold << 1;
            comp_ov_callback_(dac_value);  // Invoke callback with new threshold value
        }
    }

    friend struct Feedback;


    // Constructor: Initialize with filter alpha, maximum voltage, and error manager reference
    // alpha: Filter coefficient (0..255, where 0=infinite filtering, 255=no filtering)
    // max_voltage_mv: Maximum expected voltage in millivolts
    // error: Reference to error manager for reporting voltage faults
    SupplyVoltage(uint8_t alpha, int32_t max_voltage_mv, ErrorManager& error)  // Constructor: Create and initialize with filter settings and error manager
        : comp_ov_callback_(nullptr),                     // Initialize callback pointer to null
          filter_(0, alpha),                              // Initialize filter with zero input and specified alpha
          max_voltage_mv_(max_voltage_mv),                // Store maximum voltage for scaling
          scale_(0),                                      // Initialize scale factor (will be calculated)
          voltage_norm_(0),                               // Initialize normalized voltage to zero
          voltage_mv_(0),                                 // Initialize millivolt voltage to zero
          overvoltage_soft_norm(INT16_MAX),               // Initialize soft overvoltage limit to maximum
          undervoltage_soft_norm(INT16_MIN),              // Initialize soft undervoltage limit to minimum
          is_updated(false),                              // Initialize update flag to false
          error_(error) {
        // Calculate optimal scale factor for norm<->mv conversion
        // scale = (max_voltage_mv << shift) >> 15
        // We want maximum shift that doesn't overflow int32_t during multiplication
        // For 23-bit input: max_voltage_mv can be up to ~8,388,608 mV
        // Safe shift: 32768 * max_voltage_mv * (1 << shift) < INT32_MAX
        // For max_voltage_mv ~50000 (50V): shift = 10 is safe
        // scale = (max_voltage_mv << 10) >> 15 = max_voltage_mv >> 5
        scale_ = max_voltage_mv_ >> 5;  // Precompute scale with 10-bit shift for precision, divide by 32768 using >> 15
    }

    public:
    // Convert normalized Q15 value to millivolts with minimal precision loss
    // norm: Normalized voltage in Q15 format (-32768..32767)
    // Returns: Voltage in millivolts
    inline int32_t norm2mv(int16_t norm) const {  // Convert normalized value to millivolts
        // Formula: voltage_mv = (norm * scale) >> 10
        // This uses the precomputed scale factor for optimal precision
        // scale = (max_voltage_mv << 10) / 32768
        // Therefore: voltage_mv = (norm * max_voltage_mv << 10) / 32768 >> 10
        //                       = (norm * max_voltage_mv) / 32768
        return ((int32_t)norm * scale_) >> 10;  // Apply scale and shift back
    }

    // Convert millivolts to normalized Q15 value with minimal precision loss
    // mv: Voltage in millivolts
    // Returns: Normalized voltage in Q15 format (-32768..32767)
    int16_t mv2norm(int32_t mv) const {  // Convert millivolts to normalized value
        // Formula: norm = (mv << 15) / max_voltage_mv
        // To avoid overflow with 23-bit input, we use the scale factor:
        // norm = (mv << 10) / scale
        // Since scale = (max_voltage_mv << 10) / 32768
        // Therefore: norm = (mv << 10) * 32768 / (max_voltage_mv << 10)
        //                 = (mv * 32768) / max_voltage_mv
        

        
        if (scale_ == 0) {

            return 0;                             // Avoid division by zero
        }
        
        int32_t norm_calc = (mv << 10) / scale_;  // Calculate normalized value using scale

        return clamp_i32(norm_calc, INT16_MAX);  // Return clamped normalized value
    }

    // Convert millivolts to duty cycle (Q15 format)
    // target_mv: Target voltage in millivolts
    // Returns: Duty cycle in Q15 format (0..32767, where 32767 = 100%)
    inline int16_t mv2duty(int32_t target_mv) const {  // Convert millivolts to duty cycle
        if (voltage_mv_ <= 0)
            return 0;  // Avoid division by zero or neg duty
        if (voltage_mv_ <= target_mv)
            return INT16_MAX;                            // Clamp to maximum duty (100%)
        int32_t duty = (target_mv << 15) / voltage_mv_;  // Calculate duty: (target / supply) * 32768
        return (int16_t)duty;                            // Return duty cycle in Q15
    }

    // Convert normalized voltage to duty cycle (Q15 format)
    // target_norm: Target voltage in Q15 normalized format (0..32767)
    // Returns: Duty cycle in Q15 format (0..32767, where 32767 = 100%)
    static int16_t norm2duty(int16_t target_norm, int16_t supply_norm) {  // Convert normalized voltage to duty cycle
        if (supply_norm == 0)
            return 0;  // Avoid division by zero or negative duty
        if (supply_norm <= target_norm)
            return INT16_MAX;                                       // Clamp to maximum duty (100%)
        int32_t duty = ((int32_t)target_norm << 15) / supply_norm;  // Calculate duty: (target / supply) * 32768
        return (int16_t)duty;                                       // Return duty cycle in Q15 format
    }

    inline int16_t norm2duty(int16_t target_norm) const {  // Convert normalized voltage to duty cycle
        return norm2duty(target_norm, voltage_norm_);
    }

    // Get normalized voltage in Q15 format (0..32767 representing 0..max_voltage_mv)
    inline int16_t get_voltage_norm() const {  // Get normalized voltage output
        return voltage_norm_;                  // Return Q15 normalized voltage
    }

    // Get voltage in millivolts
    inline int32_t get_voltage_mv() {        // Get voltage in millivolts
        if (!is_updated)                           // Convert normalized value to millivolts using precomputed scale
            voltage_mv_ = norm2mv(voltage_norm_);  // Scale to millivolts with optimal precision
        return voltage_mv_;                        // Return voltage in millivolts
    }

    // Get maximum voltage configuration in millivolts
    inline int32_t get_max_voltage_mv() const {  // Get maximum voltage setting
        return max_voltage_mv_;                  // Return maximum voltage in millivolts
    }

    // Reset filter to a specific ADC value (useful for initialization)
    inline void reset(uint16_t raw_adc) {  // Reset filter to specific value
        filter_.reset(raw_adc);            // Reset filter internal state
        tick(raw_adc);                     // Update outputs with reset value
    }

    // Update filter alpha coefficient (0..255)
    inline void set_alpha(uint8_t alpha) {  // Set filter coefficient
        filter_.set_alpha(alpha);           // Update filter alpha value
    }

    // Set soft overvoltage limit in millivolts
    void set_overvoltage_soft_mv(int32_t v_lim) { overvoltage_soft_norm = mv2norm(v_lim); }

    // Set soft overvoltage limit in normalized Q15 format
    inline void set_overvoltage_soft_norm(int16_t v_lim) { overvoltage_soft_norm = v_lim; }

    // Set hard overvoltage limit in millivolts
    void set_overvoltage_hard_mv(int32_t v_lim) {
        int16_t norm = mv2norm(v_lim);
        update_ov_hard_dac_threshold(norm);  // Recalculate DAC threshold
    }

    // Set hard overvoltage limit in normalized Q15 format
    inline void set_overvoltage_hard_norm(int16_t v_lim) {
        update_ov_hard_dac_threshold(v_lim);  // Recalculate DAC threshold
    }

    bool is_overvoltage_soft() { return voltage_norm_ >= overvoltage_soft_norm; }

    bool is_undervoltage_soft() { return voltage_norm_ <= undervoltage_soft_norm; }

    // =========================================================================
    // COMPARATOR CALLBACK METHODS
    // =========================================================================

    // Set callback function for comparator threshold update
    // callback: Function pointer to be called when threshold is updated (or nullptr to clear)
    inline void set_overvoltage_hard_callback(DAC_SET_COMP_OV_Callback callback) {
        comp_ov_callback_ = callback;  // Store callback pointer
    }

    // Set soft undervoltage limit in millivolts
    void set_undervoltage_soft_mv(int32_t v_lim) { undervoltage_soft_norm = mv2norm(v_lim); }

    // Set soft undervoltage limit in normalized Q15 format
    inline void set_undervoltage_soft_norm(int16_t v_lim) { undervoltage_soft_norm = v_lim; }

    // Check if voltage exceeds soft overvoltage limit
    inline bool is_overvoltage_soft_norm(int16_t v_norm) const { return v_norm >= overvoltage_soft_norm; }

    // Check if voltage is below soft undervoltage limit
    inline bool is_undervoltage_soft_norm(int16_t v_norm) const { return v_norm <= undervoltage_soft_norm; }

};  // End of SupplyVoltage structure definition
