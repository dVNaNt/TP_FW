/**
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#pragma once

#include "../../common.h"

/// Frequency Generator
/// Generates sine and cosine waveforms at a target frequency by rotating a polar vector
/// Uses VecPol rotation and conversion to Vec2D for efficient waveform generation
/// Constraints: target frequency < 2000 Hz, system frequency < 65000 Hz
struct FrequencyGenerator {
private:
    Pol2D vector_pol_;           // Current polar vector (angle, amplitude)
    Vec2D out_;
    int16_t angle_increment_;    // Angle increment per tick (calculated from target frequency)
    uint16_t frequency_system_;  // System frequency in Hz (sampling rate, max 65000 Hz)
    uint16_t frequency_target_;  // Target output frequency in Hz (max 2000 Hz)
    
public:
    /// Constructor
    /// @param freq_system System frequency in Hz (sampling rate, e.g., 20000 Hz for 20 kHz PWM)
    FrequencyGenerator(uint16_t freq_system)
        : vector_pol_(0, INT16_MAX)
        , angle_increment_(0)
        , frequency_system_(freq_system)
        , frequency_target_(0),
        out_(0, 0)
    {
        // No calculation needed at initialization
    }
    
    /// Set system frequency and recalculate angle increment
    /// @param freq_system System frequency in Hz (max 65000 Hz)
    void set_frequency_system(uint16_t freq_system) {
        frequency_system_ = freq_system; // Update system frequency
        calculate_increment();           // Recalculate angle increment
    }
    
    /// Set target frequency and recalculate angle increment
    /// @param freq_target Target output frequency in Hz (max 2000 Hz)
    /// @return true if frequency is valid and set, false if out of range
    bool set_frequency_target(uint16_t freq_target) {
        if (freq_target > 2000) {       // Check frequency limit
            return false;                // Reject invalid frequency
        }
        frequency_target_ = freq_target; // Update target frequency
        calculate_increment();           // Recalculate angle increment
        return true;                     // Success
    }
    
    /// Set amplitude of the generated waveform
    /// @param amplitude Amplitude in i1.15 format (INT16_MAX = 1.0)
    void set_amplitude(int16_t amplitude) {
        vector_pol_.amplitude = amplitude; // Update amplitude directly
    }
    
    /// Get current amplitude
    /// @return Current amplitude in i1.15 format
    inline int16_t get_amplitude() const {
        return vector_pol_.amplitude; // Return current amplitude
    }
    
    /// Get current angle
    /// @return Current angle in normalized format (INT16_MIN to INT16_MAX = -π to π)
    inline int16_t get_angle() const {
        return vector_pol_.angle; // Return current angle
    }
    
    /// Get target frequency
    /// @return Target frequency in Hz
    inline uint16_t get_frequency_target() const {
        return frequency_target_; // Return target frequency
    }
    
    /// Get system frequency
    /// @return System frequency in Hz
    inline uint16_t get_frequency_system() const {
        return frequency_system_; // Return system frequency
    }
    
    /// Reset the generator to initial state (angle = 0)
    void reset() {
        vector_pol_.angle = 0; // Reset angle to zero
    }
    
    /// Tick function - updates the angle and returns the current Vec2D (sin, cos)
    /// Call this function at the system frequency rate to generate waveform
    /// @return Vec2D with updated sine and cosine values
    Vec2D tick() {
        // Rotate the polar vector by the calculated angle increment
        vector_pol_ = vector_pol_.rotate(angle_increment_);
        
        // Convert polar coordinates to Cartesian (Vec2D) and return
        out_ = vector_pol_.convert();

        return out_;
    }
    
    Vec2D get_output() {
        return out_;
    }
private:
    /// Calculate angle increment per tick based on target and system frequencies
    /// Simplified formula for freq_target < 2000 Hz and freq_system < 65000 Hz:
    /// angle_increment = (freq_target * 65536) / freq_system
    /// Uses 32-bit arithmetic (no 64-bit needed due to constraints)
    void calculate_increment() {
        if (frequency_system_ > 0) {     // Avoid division by zero
            // Calculate increment using 32-bit arithmetic
            // freq_target < 2000, so freq_target * 65536 < 131,072,000 (fits in int32_t)
            int32_t numerator = (int32_t)frequency_target_ << 16; // Multiply by 65536 (shift left 16 bits)
            angle_increment_ = (int16_t)(numerator / frequency_system_); // Divide by system frequency
        } else {
            angle_increment_ = 0;        // No increment if system frequency is invalid
        }
    }
};
