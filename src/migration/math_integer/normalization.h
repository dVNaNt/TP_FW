#pragma once

#include "../common.h"

/// Converts normalized value to milliamps, millivolts, etc.
/// 
/// # Arguments
/// * `value_norm` - The normalized value [i16]
/// * `full_scale` - The maximum full scale value in milliamps, millivolts, etc. (only positive range) [i32] 
/// 
/// # Returns
/// The value in milliamps, millivolts, etc. [i32]
static inline int32_t norm_to_value(int16_t value_norm, int32_t full_scale) {
    const uint8_t BIT_SHIFT = 9;  // Define bit shift for resolution enhancement
    
    // Calculate scaling factor to prevent overflow
    int32_t scale = full_scale >> (15 - BIT_SHIFT);
    
    // Scale and shift the normalized value to the target unit
    return ((int32_t)value_norm * scale) >> BIT_SHIFT;
}

/// Converts milliamps, millivolts, etc. to normalized value.
/// 
/// # Arguments
/// * `value` - The value in milliamps, millivolts, etc. [i32]
/// * `full_scale` - The maximum full scale value in milliamps, millivolts, etc. (only positive range) [i32]
/// 
/// # Returns
/// The normalized value [i16]
static inline int16_t value_to_norm(int32_t value, int32_t full_scale) {
    const uint8_t BIT_SHIFT = 9;  // Define bit shift for resolution enhancement
    
    // Calculate scaling factor to prevent overflow
    int32_t scale = full_scale >> (15 - BIT_SHIFT);
    
    // Shift and scale the value to normalized range
    return (int16_t)((value << BIT_SHIFT) / scale);
}


