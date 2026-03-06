/**
 * @file pid_f.h
 * @brief Floating-point PI controller for Field-Oriented Control (FOC) current regulation
 * 
 * This module implements a frequency-independent PI controller optimized for FOC applications.
 * Uses floating-point arithmetic for high precision control with automatic anti-windup protection.
 * 
 * Key Features:
 * - Frequency-independent gains (kp, ki) for easy tuning across different PWM frequencies
 * - Automatic Q15 fixed-point to physical units conversion (mA, mV)
 * - Conditional integrator anti-windup to prevent saturation
 * - Configurable integrator limits for stability
 * - Optimized for STM32G4 with hardware FPU (single-precision floating-point)
 * 
 * Algorithm:
 * - Proportional term: P = kp * error
 * - Integral term: I += ki * error * dt (with anti-windup)
 * - Output: V = P + I (clamped to voltage limit)
 * 
 * Anti-windup Strategy:
 * - Integrator only updates when output is NOT saturated
 * - If output saturated AND error would increase saturation, integrator freezes
 * - This prevents integral wind-up during sustained errors
 * 
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#pragma once
#include <Arduino.h>  // Include Arduino core for compatibility
#include <math.h>     // Include math library for floating-point operations

// Floating-point PI controller class for FOC current regulation
class FocPI {
private:
    // Controller gains (frequency-independent)
    float kp_;              // Proportional gain coefficient [mV/mA]
    float ki_;              // Integral gain coefficient [mV/(mA*s)]
    
    // Scaling coefficients for Q15 fixed-point conversion
    float current_scale_;   // Milliamps per Q15 unit (18700.0 / 32767.0 ≈ 0.571 mA/LSB)
    float voltage_scale_;   // Millivolts per Q15 unit (69000.0 / 32767.0 ≈ 2.106 mV/LSB)
    
    // Integrator state
    float integrator_;      // Accumulated error in milliamps [mA]
    float integrator_max_;  // Maximum integrator value in millivolts [mV]
    
    // Protection limits
    float output_limit_;    // Current output voltage limit in millivolts [mV]
    int16_t output_;        // Last calculated output in Q15 format


public:
    /// Default constructor - initialize PI controller with zero gains
    FocPI() {
        kp_ = 0.0f;                                // Initialize proportional gain to zero
        ki_ = 0.0f;                                // Initialize integral gain to zero
        output_ = 0;                               // Initialize output to zero
        current_scale_ = 18700.0f / 32767.0f;      // Current scale: ~0.571 mA per Q15 LSB
        voltage_scale_ = 69000.0f / 32767.0f;      // Voltage scale: ~2.106 mV per Q15 LSB
        integrator_ = 0.0f;                        // Initialize integrator to zero
        integrator_max_ = 69000.0f;                // Default integrator limit = maximum voltage (69V)
    }
    
    /// Set controller gains (frequency-independent)
    /// @param kp Proportional gain [mV/mA] - determines immediate response to error
    /// @param ki Integral gain [mV/(mA*s)] - determines accumulated error correction rate
    void setGains(float kp, float ki) {
        kp_ = kp;  // Store proportional gain
        ki_ = ki;  // Store integral gain
    }
    
    /// Set maximum integrator value to prevent excessive wind-up
    /// @param limit_mV Maximum integrator value in millivolts [mV]
    void setIntegratorLimit(float limit_mV) {
        integrator_max_ = limit_mV;  // Store integrator limit
    }
    
    /// Reset integrator to zero (use when changing control modes or during transients)
    void reset() {
        integrator_ = 0.0f;  // Clear accumulated error
    }
    

    /// Main PI controller update function
    /// @param target_q15 Target current in Q15 format [-32767..32767]
    /// @param measurement_q15 Measured current in Q15 format [-32767..32767]
    /// @param limit_q15 Maximum output voltage in Q15 format [0..32767]
    /// @param dt_seconds Time since last call in seconds [s]
    /// @return Controller output voltage in Q15 format [-32767..32767]
    int16_t tick(int16_t target_q15, int16_t measurement_q15, int16_t limit_q15, float dt_seconds) {
        // Validate time step (protect against zero or negative dt)
        if (dt_seconds <= 0.0f) {
            dt_seconds = 0.0001f;  // Default to 10 kHz sampling rate (100 us period)
        }
        
        // Convert Q15 fixed-point values to physical units (milliamps and millivolts)
        float target_mA = (float)target_q15 * current_scale_;          // Target current [mA]
        float measurement_mA = (float)measurement_q15 * current_scale_; // Measured current [mA]
        float limit_mV = (float)limit_q15 * voltage_scale_;            // Voltage limit [mV]
        
        // Update current output limit
        output_limit_ = limit_mV;  // Store for anti-windup logic
        
        // Calculate current error
        float error_mA = target_mA - measurement_mA;  // Error in milliamps [mA]
        
        // Calculate proportional term: P = kp * error
        float p_term = kp_ * error_mA;  // Proportional contribution [mV]
        
        // Calculate integral step: I_step = ki * error * dt
        float integration_step = ki_ * error_mA * dt_seconds;  // Integral increment [mV]
        
        // Calculate preliminary output (proportional + accumulated integral)
        float output_mV = p_term + integrator_;  // Preliminary output [mV]
        
        // Anti-windup logic: Only update integrator if output is NOT saturated
        // OR if error would help reduce saturation
        bool saturated_positive = (output_mV >= output_limit_) && (integration_step > 0.0f);  // Positive saturation check
        bool saturated_negative = (output_mV <= -output_limit_) && (integration_step < 0.0f); // Negative saturation check
        
        // Update integrator only if not saturated or error helps desaturate
        if (!saturated_positive && !saturated_negative) {
            integrator_ += integration_step;  // Accumulate integral term
            
            // Clamp integrator to configured limits
            if (integrator_ > integrator_max_) {
                integrator_ = integrator_max_;  // Clamp to positive limit
            } else if (integrator_ < -integrator_max_) {
                integrator_ = -integrator_max_;  // Clamp to negative limit
            }
        }
        
        // Calculate final output: V = P + I
        output_mV = p_term + integrator_;  // Final output voltage [mV]
        
        // Clamp output to voltage limits
        if (output_mV > output_limit_) {
            output_mV = output_limit_;  // Clamp to positive limit
        } else if (output_mV < -output_limit_) {
            output_mV = -output_limit_;  // Clamp to negative limit
        }
        
        // Convert output back to Q15 fixed-point format
        int32_t output_q15 = (int32_t)(output_mV / voltage_scale_);  // Scale to Q15
        
        // Protect against overflow (clamp to Q15 range)
        if (output_q15 > 32767) {
            output_q15 = 32767;  // Clamp to maximum Q15 value
        } else if (output_q15 < -32767) {
            output_q15 = -32767;  // Clamp to minimum Q15 value
        }

        output_ = (int16_t)output_q15;  // Store output for debugging
        
        return (int16_t)output_q15;  // Return Q15 output voltage
    }
    
    /// Get current integrator value (for debugging and monitoring)
    /// @return Integrator value in millivolts [mV]
    float getIntegrator() const {
        return integrator_;  // Return accumulated integral term
    }
    
    /// Calculate current error from Q15 inputs (for debugging)
    /// @param target_q15 Target current in Q15 format
    /// @param measurement_q15 Measured current in Q15 format
    /// @return Current error in milliamps [mA]
    float getLastError(int16_t target_q15, int16_t measurement_q15) const {
        float target_mA = (float)target_q15 * current_scale_;          // Convert target to mA
        float measurement_mA = (float)measurement_q15 * current_scale_; // Convert measurement to mA
        return target_mA - measurement_mA;                             // Return error in mA
    }
};  // End of FocPI class definition
