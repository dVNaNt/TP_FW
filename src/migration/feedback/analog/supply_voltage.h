#pragma once  // Ensure this header is included only once during compilation

// Supply voltage sensing with low-pass filtering
// - Input: uint16_t raw ADC value (0..65535) or up to 23-bit signed values
// - Output: int16_t normalized voltage (Q15 format, -32768..32767)
// - Output: int32_t voltage in millivolts
//
// The filter smooths ADC readings to reduce noise and provide stable voltage measurements.
// Uses precomputed scale factor for optimal resolution conversion between normalized and millivolt values.

#include "../../common.h"  // Include common migration definitions and types
#include "../../math_integer/filters/lpf.h"  // Include low-pass filter for smoothing

// Supply voltage structure with filtering and normalization
struct SupplyVoltage {  // Define the SupplyVoltage structure for voltage sensing
private:
    // ---------------------------------------------------------------------------
    // Private data members
    // ---------------------------------------------------------------------------
    
    FilterLPF filter_;  // Low-pass filter instance for smoothing ADC readings
    
    const int32_t max_voltage_mv_;  // Maximum voltage in millivolts for scaling (e.g., 24000 for 24V)
    
    int32_t scale_;  // Precomputed scale factor for norm<->mv conversion with minimal precision loss
    
    int16_t voltage_norm_;  // Normalized voltage output in Q15 format (-32768..32767)
    int32_t voltage_mv_;    // Voltage in millivolts

    // Main processing function: Update filter and calculate normalized/millivolt outputs
    // raw_adc: Raw ADC reading (0..65535, where 65535 represents max_voltage_mv)
    void tick(uint16_t raw_adc) {  // Main processing function called each cycle with new ADC reading
        filter_.tick(raw_adc);  // Update low-pass filter with new ADC value
        
        uint16_t filtered = filter_.get_output();  // Get filtered ADC value (0..65535)
        
        // Convert to normalized Q15 format: shift right by 1 to map 0..65535 to 0..32767
        // Then cast to signed int16_t for Q15 representation
        voltage_norm_ = (int16_t)(filtered >> 1);  // Normalize to Q15 format (0..32767)
        
        // Convert normalized value to millivolts using precomputed scale
        voltage_mv_ = norm2mv(voltage_norm_);  // Scale to millivolts with optimal precision
    }

    friend struct Feedback;
    
public:
    // Constructor: Initialize with filter alpha and maximum voltage
    // alpha: Filter coefficient (0..255, where 0=no filtering, 255=maximum filtering)
    // max_voltage_mv: Maximum expected voltage in millivolts
    SupplyVoltage(uint8_t alpha, int32_t max_voltage_mv)  // Constructor: Create and initialize with filter settings
        : filter_(0, alpha),  // Initialize filter with zero input and specified alpha
          max_voltage_mv_(max_voltage_mv),  // Store maximum voltage for scaling
          scale_(0),  // Initialize scale factor (will be calculated)
          voltage_norm_(0),  // Initialize normalized voltage to zero
          voltage_mv_(0)  // Initialize millivolt voltage to zero
    {
        // Calculate optimal scale factor for norm<->mv conversion
        // scale = (max_voltage_mv << shift) >> 15
        // We want maximum shift that doesn't overflow int32_t during multiplication
        // For 23-bit input: max_voltage_mv can be up to ~8,388,608 mV
        // Safe shift: 32768 * max_voltage_mv * (1 << shift) < INT32_MAX
        // For max_voltage_mv ~50000 (50V): shift = 10 is safe
        // scale = (max_voltage_mv << 10) >> 15 = max_voltage_mv >> 5
        scale_ = max_voltage_mv_ >> 5;  // Precompute scale with 10-bit shift for precision, divide by 32768 using >> 15
    }
    
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
        if (scale_ == 0) return 0;  // Avoid division by zero
        int32_t norm_calc = (mv << 10) / scale_;  // Calculate normalized value using scale
        
        // Clamp to int16_t range
        if (norm_calc > INT16_MAX) return INT16_MAX;  // Clamp to maximum positive value
        if (norm_calc < INT16_MIN) return INT16_MIN;  // Clamp to minimum negative value
        return (int16_t)norm_calc;  // Return clamped normalized value
    }
    
    // Get normalized voltage in Q15 format (0..32767 representing 0..max_voltage_mv)
    inline int16_t get_voltage_norm() const {  // Get normalized voltage output
        return voltage_norm_;  // Return Q15 normalized voltage
    }
    
    // Get voltage in millivolts
    inline int32_t get_voltage_mv() const {  // Get voltage in millivolts
        return voltage_mv_;  // Return voltage in millivolts
    }
    
    // Get maximum voltage configuration in millivolts
    inline int32_t get_max_voltage_mv() const {  // Get maximum voltage setting
        return max_voltage_mv_;  // Return maximum voltage in millivolts
    }
    
    // Reset filter to a specific ADC value (useful for initialization)
    inline void reset(uint16_t raw_adc) {  // Reset filter to specific value
        filter_.reset(raw_adc);  // Reset filter internal state
        tick(raw_adc);  // Update outputs with reset value
    }
    
    // Update filter alpha coefficient (0..255)
    inline void set_alpha(uint8_t alpha) {  // Set filter coefficient
        filter_.set_alpha(alpha);  // Update filter alpha value
    }

};  // End of SupplyVoltage structure definition

// Factory function to create SupplyVoltage instance
// alpha: Filter coefficient (0..255)
// max_voltage_mv: Maximum voltage in millivolts
inline SupplyVoltage supply_voltage_new(uint8_t alpha, int32_t max_voltage_mv) {  // Factory function for creating SupplyVoltage
    return SupplyVoltage(alpha, max_voltage_mv);  // Return new SupplyVoltage instance
}
