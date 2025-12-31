#pragma once

#include "../common.h"

/// Calculate current in milliamps (mA) from voltage (mV) and resistance (mΩ).
/// 
/// # Arguments
/// * `voltage_mv` - The voltage in millivolts [i32]
/// * `resistance_mohm` - The resistance in milliohms [i32]
/// 
/// # Returns
/// The current in milliamps [i32]
static inline int32_t ohms_current(int32_t voltage_mv, int32_t resistance_mohm) {
    // I = V / R, ensuring we prevent division by zero
    if (resistance_mohm == 0) {
        return 0;
    }
    return (voltage_mv * 1000) / resistance_mohm;
}

/// Calculate voltage in millivolts (mV) from current (mA) and resistance (mΩ).
/// 
/// # Arguments
/// * `current_ma` - The current in milliamps [i32]
/// * `resistance_mohm` - The resistance in milliohms [i32]
/// 
/// # Returns
/// The voltage in millivolts [i32]
static inline int32_t ohms_voltage(int32_t current_ma, int32_t resistance_mohm) {
    // V = I * R
    return (current_ma * resistance_mohm) / 1000;
}

/// Calculate resistance in milliohms (mΩ) from voltage (mV) and current (mA).
/// 
/// # Arguments
/// * `voltage_mv` - The voltage in millivolts [i32]
/// * `current_ma` - The current in milliamps [i32]
/// 
/// # Returns
/// The resistance in milliohms [i32]
static inline int32_t ohms_resistance(int32_t voltage_mv, int32_t current_ma) {
    // R = V / I, ensuring we prevent division by zero
    if (current_ma == 0) {
        return 0;
    }
    return (voltage_mv * 1000) / current_ma;
}

/// Calculate power in milliwatts (mW) from voltage (mV) and current (mA).
/// 
/// # Arguments
/// * `voltage_mv` - The voltage in millivolts [i32]
/// * `current_ma` - The current in milliamps [i32]
/// 
/// # Returns
/// The power in milliwatts [i32]
static inline int32_t ohms_power(int32_t voltage_mv, int32_t current_ma) {
    // P = (V * I) / 1000
    return (voltage_mv * current_ma) / 1000;
}


