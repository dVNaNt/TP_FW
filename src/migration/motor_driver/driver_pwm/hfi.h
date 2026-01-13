#pragma once

// High Frequency Injection (HFI) Signal Generator and Analyzer
// 
// This module generates high-frequency voltage injection signals (sine/cosine)
// and performs online DFT (Discrete Fourier Transform) analysis on the resulting
// current response to extract amplitude and phase information.
//
// Key Features:
// - Generates sine/cosine voltage signals at configurable frequency and amplitude
// - Performs online single-frequency DFT using Q15 fixed-point arithmetic
// - Measures current response in both D and Q axes independently (Vec2D)
// - Handles DC offset in current measurements via accumulator-based DFT
// - Detects amplitude of the injected frequency component in current
// - Accounts for potential time lag between voltage and current
// - Runs for a specified number of complete periods
//
// Algorithm Overview:
// 1. Voltage Generation: Uses FrequencyGenerator to produce sin/cos at target frequency
// 2. Current Analysis: Correlates current samples with reference sin/cos (DFT principle)
// 3. DC Rejection: DFT inherently rejects DC when integrating over complete periods
// 4. Amplitude Extraction: sqrt(real² + imag²) gives the amplitude at target frequency
//
// Licensed under the Apache License, Version 2.0
// Copyright 2024 Anton Khrustalev, creapunk.com

#include "../../common.h"
#include "frequency_generator.h"

/// HFI (High Frequency Injection) Signal Generator and Analyzer
/// Generates voltage signals and analyzes current response using online DFT
struct HFI {
private:
    // Frequency generator for voltage waveform synthesis
    FrequencyGenerator generator_;
    
    // DFT accumulators for D-axis current (real and imaginary components)
    // Real = correlation with cosine, Imaginary = correlation with sine
    int32_t dft_d_real_;   // Accumulator for D-axis real component (cos correlation)
    int32_t dft_d_imag_;   // Accumulator for D-axis imaginary component (sin correlation)
    
    // DFT accumulators for Q-axis current (real and imaginary components)
    int32_t dft_q_real_;   // Accumulator for Q-axis real component (cos correlation)
    int32_t dft_q_imag_;   // Accumulator for Q-axis imaginary component (sin correlation)
    
    // Sample counter for tracking progress within injection cycles
    uint32_t sample_count_;      // Current sample index within all periods
    uint32_t samples_per_period_; // Number of samples in one period
    uint32_t total_samples_;     // Total samples for all requested periods
    
    // Configuration parameters
    uint16_t periods_target_;    // Number of complete periods to run
    bool is_running_;            // Flag indicating if injection is active
    
    // Output results (computed after completion)
    int16_t amplitude_d_;        // Detected amplitude in D-axis current (Q15)
    int16_t amplitude_q_;        // Detected amplitude in Q-axis current (Q15)
    int16_t phase_d_;            // Detected phase in D-axis (normalized angle)
    int16_t phase_q_;            // Detected phase in Q-axis (normalized angle)

public:
    /// Constructor
    /// @param freq_system System/PWM frequency in Hz (sampling rate)
    HFI(uint16_t freq_system)
        : generator_(freq_system)          // Initialize frequency generator
        , dft_d_real_(0)                   // Clear D-axis real accumulator
        , dft_d_imag_(0)                   // Clear D-axis imaginary accumulator
        , dft_q_real_(0)                   // Clear Q-axis real accumulator
        , dft_q_imag_(0)                   // Clear Q-axis imaginary accumulator
        , sample_count_(0)                 // Reset sample counter
        , samples_per_period_(0)           // Will be calculated on start
        , total_samples_(0)                // Will be calculated on start
        , periods_target_(0)               // No periods requested yet
        , is_running_(false)               // Not running initially
        , amplitude_d_(0)                  // Clear D amplitude result
        , amplitude_q_(0)                  // Clear Q amplitude result
        , phase_d_(0)                      // Clear D phase result
        , phase_q_(0)                      // Clear Q phase result
    {
        // Constructor body empty - all initialization in initializer list
    }

    /// Start HFI injection and measurement
    /// Configures the generator and resets all accumulators for a new measurement cycle
    /// @param amplitude Voltage amplitude in Q15 format (INT16_MAX = 1.0 = full scale)
    /// @param frequency Target injection frequency in Hz (must be < 2000 Hz)
    /// @param periods Number of complete periods to run (integer, for DC rejection)
    /// @return true if started successfully, false if frequency out of range
    bool start(int16_t amplitude, uint16_t frequency, uint16_t periods) {
        // Validate and set target frequency in generator
        if (!generator_.set_frequency_target(frequency)) {
            return false;                  // Frequency out of valid range
        }
        
        // Set voltage amplitude for injection
        generator_.set_amplitude(amplitude);
        
        // Reset generator angle to start from zero phase
        generator_.reset();
        
        // Store number of periods to complete
        periods_target_ = periods;
        
        // Calculate samples per period: samples = freq_system / freq_target
        // This determines how many ticks make one complete sine wave
        uint16_t freq_sys = generator_.get_frequency_system();
        uint16_t freq_tgt = generator_.get_frequency_target();
        
        if (freq_tgt > 0) {
            samples_per_period_ = freq_sys / freq_tgt;  // Integer division
        } else {
            samples_per_period_ = 0;       // Invalid frequency
            return false;
        }
        
        // Calculate total samples for all periods
        total_samples_ = samples_per_period_ * periods_target_;
        
        // Reset all DFT accumulators to zero for fresh measurement
        // These accumulate the correlation between current and reference signals
        dft_d_real_ = 0;                   // D-axis cosine correlation
        dft_d_imag_ = 0;                   // D-axis sine correlation
        dft_q_real_ = 0;                   // Q-axis cosine correlation
        dft_q_imag_ = 0;                   // Q-axis sine correlation
        
        // Reset sample counter
        sample_count_ = 0;
        
        // NOTE: Do NOT clear previous results (amplitude_d_, amplitude_q_, phase_d_, phase_q_)
        // Keep them for SWD debugging between measurement cycles
        
        // Mark as running
        is_running_ = true;
        
        return true;                       // Successfully started
    }
    
    /// Generate voltage output for current tick
    /// Call this at the system frequency rate to get voltage setpoint
    /// @return Vec2D containing (sin, cos) voltage components in Q15 format
    Vec2D tick_voltage() {
        if (!is_running_) {
            return Vec2D(0, 0);             // Return zero if not running
        }
        
        // Generate next voltage sample using frequency generator
        // Returns Vec2D with sin and cos components scaled by amplitude
        return generator_.tick();
    }

    /// Process current measurement for DFT analysis
    /// Correlates current samples with reference sin/cos to extract frequency component
    /// DC offset is automatically rejected by integrating over complete periods
    /// @param current_dq Current measurement in D-Q axes (Vec2D in Q15 format)
    void tick_current(Vec2D current_dq) {
        if (!is_running_) {
            return;                         // Skip if not running
        }
        
        // Get current reference sin/cos from generator (same phase as voltage)
        // These are the correlation basis functions for DFT
        int16_t ref_sin, ref_cos;
        angle2sincos(generator_.get_angle(), &ref_sin, &ref_cos);
        
        // === Online DFT Algorithm ===
        // DFT at frequency f: X(f) = sum(x[n] * e^(-j*2*pi*f*n/N))
        // For single frequency: Real = sum(x[n] * cos(angle)), Imag = sum(x[n] * sin(angle))
        // We use the generator's current angle as the reference phase
        //
        // DC Rejection: When integrating over complete periods, DC component averages to zero
        // because sum(cos) = sum(sin) = 0 over complete cycles
        
        // === D-axis (flux) current correlation ===
        // Multiply D-axis current by reference cos and sin, accumulate
        // Using 64-bit accumulators to prevent overflow over many samples
        int32_t i_d = (int32_t)current_dq.flux;  // D-axis current (flux component of Vec2D)
        
        // Real part: correlation with cosine (in-phase component)
        dft_d_real_ += (int64_t)i_d * (int64_t)ref_cos;
        
        // Imaginary part: correlation with sine (quadrature component)
        dft_d_imag_ += (int64_t)i_d * (int64_t)ref_sin;
        
        // === Q-axis (torque) current correlation ===
        // Multiply Q-axis current by reference cos and sin, accumulate
        int32_t i_q = (int32_t)current_dq.torque;  // Q-axis current (torque component of Vec2D)
        
        // Real part: correlation with cosine (in-phase component)
        dft_q_real_ += (int64_t)i_q * (int64_t)ref_cos;
        
        // Imaginary part: correlation with sine (quadrature component)
        dft_q_imag_ += (int64_t)i_q * (int64_t)ref_sin;
        
        // Increment sample counter
        sample_count_++;
        
        // Check if all periods are complete
        if (sample_count_ >= total_samples_) {
            // Measurement complete - calculate final results
            compute_results();
            is_running_ = false;            // Stop running
        }
    }
    
    /// Check if HFI measurement cycle is complete
    /// @return true if all requested periods have been processed
    bool complete() const {
        return !is_running_ && (sample_count_ >= total_samples_) && (total_samples_ > 0);
    }
    
    /// Check if HFI is currently running
    /// @return true if injection is active
    bool is_running() const {
        return is_running_;
    }
    
    /// Get detected amplitude in D-axis current
    /// Valid only after complete() returns true
    /// @return Amplitude in Q15 format
    int16_t get_amplitude_d() const {
        return amplitude_d_;
    }
    
    /// Get detected amplitude in Q-axis current
    /// Valid only after complete() returns true
    /// @return Amplitude in Q15 format
    int16_t get_amplitude_q() const {
        return amplitude_q_;
    }
    
    /// Get detected phase in D-axis current (relative to voltage)
    /// Represents the time lag between voltage and current
    /// Valid only after complete() returns true
    /// @return Phase in normalized angle format (INT16_MIN to INT16_MAX = -π to π)
    int16_t get_phase_d() const {
        return phase_d_;
    }
    
    /// Get detected phase in Q-axis current (relative to voltage)
    /// Represents the time lag between voltage and current
    /// Valid only after complete() returns true
    /// @return Phase in normalized angle format (INT16_MIN to INT16_MAX = -π to π)
    int16_t get_phase_q() const {
        return phase_q_;
    }
    
    /// Get current sample count
    /// @return Number of samples processed so far
    uint32_t get_sample_count() const {
        return sample_count_;
    }
    
    /// Get total samples target
    /// @return Total number of samples for all periods
    uint32_t get_total_samples() const {
        return total_samples_;
    }

private:
    /// Compute final amplitude and phase results from DFT accumulators
    /// Called automatically when all periods are complete
    /// 
    /// Algorithm:
    /// 1. Normalize accumulators by number of samples (average)
    /// 2. Calculate amplitude: A = 2 * sqrt(real² + imag²) / N
    ///    Factor of 2 because DFT gives half amplitude for real signals
    /// 3. Calculate phase: phi = atan2(imag, real)
    ///    Phase represents delay between voltage and current
    void compute_results() {
        if (total_samples_ == 0) {
            return;                         // Avoid division by zero
        }
        
        // === Normalize DFT results ===
        // Divide by number of samples to get average correlation
        // Scale down from Q15*Q15 = Q30 to Q15 by shifting right 15 bits
        // Then divide by N (number of samples)
        // Combined: shift right 15, then divide by N
        // For amplitude, multiply by 2 (DFT of real signal gives half amplitude)
        
        // Calculate normalization factor
        // We need to: (accumulator >> 15) / N * 2 = (accumulator * 2) / (N << 15)
        // To avoid overflow, we do: (accumulator / N) >> 14 (includes *2 factor)
        
        int64_t n = (int64_t)total_samples_;
        
        // === D-axis amplitude and phase calculation ===
        // Normalize accumulators
        int32_t d_real_norm = (int32_t)((dft_d_real_ / n) >> 14);  // Normalized real, includes *2
        int32_t d_imag_norm = (int32_t)((dft_d_imag_ / n) >> 14);  // Normalized imag, includes *2
        
        // Calculate amplitude: sqrt(real² + imag²)
        // Use integer square root approximation
        amplitude_d_ = fast_magnitude(d_real_norm, d_imag_norm);
        
        
        // === Q-axis amplitude and phase calculation ===
        // Normalize accumulators
        int32_t q_real_norm = (int32_t)((dft_q_real_ / n) >> 14);  // Normalized real, includes *2
        int32_t q_imag_norm = (int32_t)((dft_q_imag_ / n) >> 14);  // Normalized imag, includes *2
        
        // Calculate amplitude: sqrt(real² + imag²)
        amplitude_q_ = fast_magnitude(q_real_norm, q_imag_norm);

    }
    
    /// Fast integer magnitude calculation: sqrt(x² + y²)
    /// Uses iterative approximation for embedded efficiency
    /// @param x Real component (Q15)
    /// @param y Imaginary component (Q15)
    /// @return Magnitude in Q15 format
    static int16_t fast_magnitude(int32_t x, int32_t y) {
        // Take absolute values for magnitude calculation
        uint32_t ax = (x >= 0) ? (uint32_t)x : (uint32_t)(-x);
        uint32_t ay = (y >= 0) ? (uint32_t)y : (uint32_t)(-y);
        
        // Alpha-max-beta-min approximation for fast magnitude
        // mag ≈ max(|x|,|y|) + 0.4 * min(|x|,|y|)
        // More accurate: mag ≈ 0.96 * max + 0.4 * min
        // We use: mag ≈ max + (min >> 2) + (min >> 3) ≈ max + 0.375 * min
        
        uint32_t max_val, min_val;
        if (ax > ay) {
            max_val = ax;
            min_val = ay;
        } else {
            max_val = ay;
            min_val = ax;
        }
        
        // Approximation: mag ≈ max + 0.375 * min = max + min/4 + min/8
        uint32_t mag = max_val + (min_val >> 2) + (min_val >> 3);
        
        // Clamp to Q15 range
        if (mag > INT16_MAX) {
            mag = INT16_MAX;
        }
        
        return (int16_t)mag;
    }
};
