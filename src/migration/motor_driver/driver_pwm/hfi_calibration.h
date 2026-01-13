#pragma once

// HFI Calibration Module: Amplitude Tuning and Inductance Identification
//
// This module implements:
// 1. HFI Amplitude Tuning (Step 4): Adjusts HFI voltage amplitude to achieve target current
// 2. Ld Identification & Zero Search (Step 5): Finds electrical zero and D-axis inductance
// 3. Lq Identification (Step 6): Measures Q-axis inductance
//
// Algorithm Overview:
// - Maintains DC bias (alignment current) on D-axis to hold rotor
// - Injects 1kHz HFI signal and measures current response via DFT
// - Adjusts amplitude until current reaches target (1/4 * min(I_EST, I_LIM))
// - Sweeps HFI angle to find electrical zero (max DFT response)
// - Calculates Ld and Lq from voltage/current relationship
//
// Licensed under the Apache License, Version 2.0
// Copyright 2024 Anton Khrustalev, creapunk.com

#include "../../common.h"
#include "../../feedback/feedback.h"
#include "../motor.h"
#include "hfi.h"
#include "sel_motor.h"

/// HFI Calibration States
enum HFICalibState : uint8_t {
    HFICal_Idle = 0,           // Not running
    HFICal_AmplitudeTuning,    // Step 4: Adjusting HFI amplitude
    HFICal_LdMeasure,          // Step 5: Measuring Ld at D-axis (rotor held by DC spring)
    HFICal_LqMeasure,          // Step 6: Measuring Lq at Q-axis (90° from D)
    HFICal_Pause,              // Pause between measurement cycles (200ms)
    HFICal_Complete,           // Calibration complete (not used in loop mode)
    HFICal_Error               // Error occurred
};

/// HFI Calibration Module
/// Performs amplitude tuning and inductance identification
struct HFICalibration {
private:
    // References to external modules
    Feedback& fb_;             // Feedback for current/voltage sensing
    Motor& motor_;             // Motor parameters storage
    MotorSelector& sel_;       // Motor selector for voltage output
    HFI& hfi_;                 // HFI signal generator/analyzer

    // Configuration constants
    static constexpr uint16_t HFI_FREQUENCY = 1000;      // HFI injection frequency: 1kHz
    static constexpr uint16_t HFI_PERIODS = 100;         // Periods per measurement cycle (increased for accuracy)
    static constexpr int32_t INITIAL_AMPLITUDE_MV = 200; // Initial HFI amplitude: 200mV
    static constexpr int16_t MAX_AMPLITUDE_MULT = 4;     // Max amplitude multiplier: 4x
    static constexpr uint16_t WARMUP_CYCLES = 18000;     // 1 second warmup at 18kHz

    // State machine
    HFICalibState state_;      // Current calibration state
    uint32_t cycle_counter_;   // General purpose cycle counter
    uint32_t pause_counter_;   // Counter for 200ms pause between measurements
    static constexpr uint32_t PAUSE_CYCLES = 3600;  // 200ms at 18kHz (18000 * 0.2)

    // Amplitude tuning variables (Step 4)
    int32_t hfi_amplitude_mv_; // Current HFI voltage amplitude in mV
    int32_t max_amplitude_mv_; // Maximum allowed amplitude (Vbus/4)
    int16_t target_current_;   // Target HFI current amplitude (Q15)
    int16_t tolerance_;        // Tolerance: target/8 (Q15)
    uint8_t amplitude_iterations_; // Iteration counter for amplitude tuning

    // Inductance measurement variables
    int32_t ld_uh_;            // D-axis inductance in microhenries
    int32_t lq_uh_;            // Q-axis inductance in microhenries
    int16_t electrical_zero_;  // Electrical zero angle (always 0 - rotor held by DC spring)

    // DC bias current for rotor alignment
    int16_t dc_bias_current_;  // DC current to hold rotor (Q15 normalized)
    int16_t dc_bias_angle_;    // DC bias angle (always 0 - D-axis alignment)

    // Output voltage command
    Vec2D voltage_out_;        // Combined DC + HFI voltage output

public:
    /// Constructor
    /// @param fb Reference to Feedback module
    /// @param motor Reference to Motor parameters
    /// @param sel Reference to MotorSelector
    /// @param hfi Reference to HFI module
    HFICalibration(Feedback& fb, Motor& motor, MotorSelector& sel, HFI& hfi)
        : fb_(fb)
        , motor_(motor)
        , sel_(sel)
        , hfi_(hfi)
        , state_(HFICal_Idle)
        , cycle_counter_(0)
        , pause_counter_(0)
        , hfi_amplitude_mv_(INITIAL_AMPLITUDE_MV)
        , max_amplitude_mv_(0)
        , target_current_(0)
        , tolerance_(0)
        , amplitude_iterations_(0)
        , ld_uh_(0)
        , lq_uh_(0)
        , electrical_zero_(0)
        , dc_bias_current_(0)
        , dc_bias_angle_(0)
        , voltage_out_(0, 0)
    {
    }

    /// Start HFI calibration sequence
    /// @param dc_current_ma DC bias current in mA for rotor alignment
    void start(int32_t dc_current_ma = 1000) {
        SerialUSB.println("[HFI] === STARTING CALIBRATION ===");
        
        // Calculate target HFI current: 1/8 * min(I_EST, I_LIM)
        int32_t i_limit = motor_.get_current_limit();                    // Motor current limit (mA)
        int32_t i_scale = fb_.current.norm2ma(INT16_MAX / 2);            // Sensor half-scale (mA)
        int32_t i_min = (i_limit < i_scale) ? i_limit : i_scale;         // min(I_EST, I_LIM)
        int32_t target_ma = i_min >> 3;                                  // Target = min/8

        // Convert target to normalized Q15 format
        target_current_ = fb_.current.ma2norm(target_ma);                // Target current (Q15)
        tolerance_ = target_current_ >> 3;                               // Tolerance = target/8

        // Calculate maximum HFI amplitude: Vbus/4
        max_amplitude_mv_ = fb_.supply.get_voltage_mv() >> 2;            // Max amplitude = Vbus/4

        // Set DC bias current for rotor alignment
        dc_bias_current_ = fb_.current.ma2norm(dc_current_ma);           // DC bias (Q15)
        dc_bias_angle_ = 0;                                              // Initial angle = 0

        // Log configuration
        SerialUSB.print("  Target Current: ");
        SerialUSB.print(target_ma);
        SerialUSB.println("mA");
        SerialUSB.print("  Max Voltage: ");
        SerialUSB.print(max_amplitude_mv_);
        SerialUSB.println("mV");
        SerialUSB.print("  DC Bias: ");
        SerialUSB.print(dc_current_ma);
        SerialUSB.println("mA");

        // Reset state variables
        hfi_amplitude_mv_ = INITIAL_AMPLITUDE_MV;                        // Start with 200mV
        amplitude_iterations_ = 0;                                       // Reset iteration counter
        electrical_zero_ = 0;                                            // Electrical zero at D-axis (0°)
        cycle_counter_ = 0;                                              // Reset cycle counter

        // Transition to amplitude tuning state
        SerialUSB.println("[HFI] Starting Amplitude Tuning");
        state_ = HFICal_AmplitudeTuning;                                 // Begin amplitude tuning

        // Start first HFI measurement
        start_hfi_measurement();                                         // Initialize HFI
    }

    /// Main tick function - call at PWM frequency
    /// @return Voltage output command (Vec2D in alpha-beta frame)
    Vec2D tick() {
        // Process based on current state
        switch (state_) {
            case HFICal_Idle:
                voltage_out_ = Vec2D(0, 0);                              // No output when idle
                break;

            case HFICal_AmplitudeTuning:
                tick_amplitude_tuning();                                 // Process amplitude tuning
                break;

            case HFICal_LdMeasure:
                tick_ld_measure();                                       // Process Ld measurement
                break;

            case HFICal_LqMeasure:
                tick_lq_measure();                                       // Process Lq measurement
                break;

            case HFICal_Pause:
                tick_pause();                                            // Process pause between cycles
                break;

            case HFICal_Complete:
            case HFICal_Error:
                voltage_out_ = Vec2D(0, 0);                              // No output when done
                break;
        }

        return voltage_out_;                                             // Return voltage command
    }

    /// Check if calibration is complete
    inline bool is_complete() const { return state_ == HFICal_Complete; }

    /// Check if error occurred
    inline bool is_error() const { return state_ == HFICal_Error; }

    /// Get current state
    inline HFICalibState get_state() const { return state_; }

    /// Get detected electrical zero angle
    inline int16_t get_electrical_zero() const { return electrical_zero_; }

    /// Get measured D-axis inductance in microhenries
    inline int32_t get_ld_uh() const { return ld_uh_; }

    /// Get measured Q-axis inductance in microhenries
    inline int32_t get_lq_uh() const { return lq_uh_; }

    /// Get calibrated HFI amplitude in mV
    inline int32_t get_hfi_amplitude_mv() const { return hfi_amplitude_mv_; }

    /// Get current sweep angle (for debugging)
    inline int16_t get_sweep_angle() const { return 0; }  // No sweep - always at D-axis

    /// Get best amplitude found during sweep (for debugging)
    inline int16_t get_best_amplitude() const { return hfi_.get_amplitude_d(); }  // Current amplitude

    /// Get target current for amplitude tuning (for debugging)
    inline int16_t get_target_current() const { return target_current_; }

    /// Get last measured HFI current amplitude from DFT
    inline int16_t get_measured_amplitude() const { return hfi_.get_amplitude_d(); }

private:
    /// Start a new HFI measurement cycle
    void start_hfi_measurement() {
        int16_t amplitude_duty = fb_.supply.mv2duty(hfi_amplitude_mv_);  // Convert mV to duty
        hfi_.start(amplitude_duty, HFI_FREQUENCY, HFI_PERIODS);          // Start HFI injection
    }

    /// Calculate DC bias voltage from current and resistance
    /// @return DC voltage in duty cycle (Q15)
    int16_t calculate_dc_voltage() {
        int32_t r_mohm = motor_.get_total_resistance();                  // Motor resistance (mOhm)
        int32_t i_ma = fb_.current.norm2ma(dc_bias_current_);            // DC current (mA)
        int32_t v_mv = (i_ma * r_mohm) / 1000;                           // V = I * R (mV)
        return fb_.supply.mv2duty(v_mv);                                 // Convert to duty
    }


    /// Step 4: Amplitude Tuning - adjust HFI voltage to achieve target current
    /// Uses PULSATING injection (not rotating) - only Alpha axis oscillates
    void tick_amplitude_tuning() {
        // 1. Get ONLY the scalar oscillation signal (Cosine part)
        // We ignore .sin because we want pulsating injection, not rotating
        Vec2D hfi_voltage = hfi_.tick_voltage();                         // Get HFI voltage
        int16_t injection_scalar = hfi_voltage.cos;                      // Use only cosine (pulsating)

        // 2. Get current measurements for DFT processing
        sel_.tick_current(fb_.current.norm());                           // Update current selector
        Vec2D current_ab = sel_.get_currents();                          // Get alpha-beta currents
        hfi_.tick_current(current_ab);                                   // Feed to HFI DFT

        // 3. Calculate DC bias voltage for rotor alignment
        int16_t dc_duty = calculate_dc_voltage();                        // DC voltage duty

        // 4. Output Voltage: DC Bias + Pulsating Injection on D-axis ONLY
        // CRITICAL: Pulsating injection - only Alpha oscillates, Beta = 0
        voltage_out_.alpha = dc_duty + injection_scalar;                 // DC + Oscillation on D-axis
        voltage_out_.beta = 0;                                           // Strictly ZERO on Q-axis

        // Check if HFI measurement cycle is complete
        if (hfi_.complete()) {
            // Get measured current amplitude in D-axis
            int16_t measured_amplitude = hfi_.get_amplitude_d();         // D-axis current amplitude

            // Log amplitude tuning iteration
            SerialUSB.print("[HFI AmpTune] Iter=");
            SerialUSB.print(amplitude_iterations_);
            SerialUSB.print(" V_mv=");
            SerialUSB.print(hfi_amplitude_mv_);
            SerialUSB.print(" I_meas=");
            SerialUSB.print(fb_.current.norm2ma(measured_amplitude));
            SerialUSB.print("mA I_tgt=");
            SerialUSB.print(fb_.current.norm2ma(target_current_));
            SerialUSB.println("mA");

            // Calculate error from target
            int16_t error = target_current_ - measured_amplitude;        // Error = target - measured
            int16_t abs_error = (error >= 0) ? error : -error;           // Absolute error

            // Check if within tolerance
            if (abs_error <= tolerance_) {
                // Amplitude tuning complete, proceed to Ld measurement
                SerialUSB.println("[HFI AmpTune] COMPLETE - Starting Ld Measurement");
                state_ = HFICal_LdMeasure;                               // Next state: Ld measurement
                start_hfi_measurement();                                 // Start Ld measurement (100 periods)
                return;
            }

            // Adjust amplitude based on error
            if (measured_amplitude < target_current_) {
                // Current too low, increase voltage amplitude
                // Scale factor: target/measured (clamped to 4x max)
                int32_t scale = 1;
                if (measured_amplitude > 0) {
                    scale = ((int32_t)target_current_ << 8) / measured_amplitude; // Q8 scale
                    if (scale > (MAX_AMPLITUDE_MULT << 8)) {
                        scale = MAX_AMPLITUDE_MULT << 8;                 // Clamp to 4x
                    }
                    if (scale < 256) scale = 256;                        // Minimum 1x
                } else {
                    scale = MAX_AMPLITUDE_MULT << 8;                     // Max if zero current
                }

                // Apply scale to amplitude
                hfi_amplitude_mv_ = (hfi_amplitude_mv_ * scale) >> 8;    // Scale amplitude
            } else {
                // Current too high, decrease voltage amplitude
                int32_t scale = ((int32_t)target_current_ << 8) / measured_amplitude;
                if (scale < 64) scale = 64;                              // Minimum 0.25x
                hfi_amplitude_mv_ = (hfi_amplitude_mv_ * scale) >> 8;    // Scale down
            }

            // Clamp amplitude to maximum (Vbus/4)
            if (hfi_amplitude_mv_ > max_amplitude_mv_) {
                hfi_amplitude_mv_ = max_amplitude_mv_;                   // Clamp to max
                SerialUSB.println("[HFI AmpTune] WARNING: Hit Vbus/4 limit");
            }

            // Check iteration limit
            amplitude_iterations_++;
            if (amplitude_iterations_ > 20) {
                // Too many iterations, proceed anyway with current amplitude
                SerialUSB.println("[HFI AmpTune] TIMEOUT - Proceeding to Ld Measurement");
                state_ = HFICal_LdMeasure;                               // Move to Ld measurement
            }

            // Start next measurement with adjusted amplitude
            start_hfi_measurement();                                     // Restart HFI
        }
    }

    /// Step 5: Ld Measurement - measure D-axis inductance at current position (held by DC spring)
    /// No angle sweep - rotor is already at minimum (electrical zero) due to DC bias
    /// Uses PULSATING injection (not rotating) - only Alpha axis oscillates
    void tick_ld_measure() {
        // 1. Get ONLY the scalar oscillation signal (Cosine part)
        // We ignore .sin because we want pulsating injection, not rotating
        Vec2D hfi_voltage = hfi_.tick_voltage();                         // Get HFI voltage
        int16_t injection_scalar = hfi_voltage.cos;                      // Use only cosine (pulsating)

        // 2. Feed Current to DFT
        // Since we inject into D (Alpha), we expect response in D
        // The DFT correlates with the internal cos/sin reference
        sel_.tick_current(fb_.current.norm());                           // Update current selector
        Vec2D current_ab = sel_.get_currents();                          // Get alpha-beta currents
        hfi_.tick_current(current_ab);                                   // Feed to HFI DFT (D-axis aligned)

        // 3. Output Voltage: DC Bias + Pulsating Injection on D-axis ONLY
        int16_t dc_duty = calculate_dc_voltage();                        // DC voltage duty

        // CRITICAL: Pulsating injection - only Alpha oscillates, Beta = 0
        voltage_out_.alpha = dc_duty + injection_scalar;                 // DC + Oscillation on D-axis
        voltage_out_.beta = 0;                                           // Strictly ZERO on Q-axis

        // 4. Check if HFI measurement cycle is complete (100 periods)
        if (hfi_.complete()) {
            // Get measured current amplitude in D-axis
            int16_t amplitude_d = hfi_.get_amplitude_d();                // D-axis current amplitude

            // Calculate Ld from measurement
            // L = V / (2 * pi * f * I)
            // K = 1000000 / (2 * 3.14159 * f) = 159155 / f
            // For 1000Hz -> K = 159.155
            int32_t v_mv = hfi_amplitude_mv_;                            // HFI voltage (mV)
            int32_t i_ma = fb_.current.norm2ma(amplitude_d);             // Current (mA)
            if (i_ma > 0) {
                // Ld = V * K / I, where K = 159155 / HFI_FREQUENCY
                int32_t k_const = 159155 / HFI_FREQUENCY;                // Calculate K constant
                ld_uh_ = (v_mv * k_const) / i_ma;                        // Ld in uH
            }

            // Log Ld measurement results
            // SerialUSB.println("[HFI Ld] COMPLETE");
            // SerialUSB.print("  D-axis Amplitude: ");
            // SerialUSB.print(i_ma);
            // SerialUSB.println("mA");
            SerialUSB.print("  Ld: ");
            SerialUSB.print(ld_uh_);
            SerialUSB.println("uH");

            // Store Ld in motor parameters
            motor_.set_phase_inductance(0, ld_uh_);                      // Store as phase 0

            // Proceed to Lq measurement
            // SerialUSB.println("[HFI] Starting Lq Measurement");
            state_ = HFICal_LqMeasure;                                   // Next state
            start_hfi_measurement();                                     // Start Lq measurement (100 periods)
        }
    }

    /// Step 6: Lq Measurement - measure Q-axis inductance with pulsating injection on Beta
    /// DC bias holds rotor on D-axis, pulsating injection on Q-axis (Beta)
    void tick_lq_measure() {
        // 1. Get ONLY the scalar oscillation signal (Cosine part)
        Vec2D hfi_voltage = hfi_.tick_voltage();                         // Get HFI voltage
        int16_t injection_scalar = hfi_voltage.cos;                      // Use only cosine (pulsating)

        // 2. Feed Current to DFT
        // CRITICAL: Since we are injecting into Q (Beta), the response will be in Beta (Torque current)
        // The HFI class normally correlates D-axis current with Cosine
        // We need to swap the current components so the HFI analyzer "sees" the Q-axis response
        // as if it were the main signal
        sel_.tick_current(fb_.current.norm());                           // Update currents
        Vec2D current_ab = sel_.get_currents();                          // Get alpha-beta currents
        
        // Swap current components for DFT analysis
        Vec2D current_swapped(0, 0);                                     // Initialize swapped vector
        current_swapped.flux = current_ab.torque;                        // Feed Q-current into HFI D-input
        current_swapped.torque = current_ab.flux;                        // Feed D-current to HFI Q-input
        
        hfi_.tick_current(current_swapped);                              // Feed swapped currents to DFT

        // 3. Output Voltage: DC Bias on D-axis, Pulsating Injection on Q-axis
        int16_t dc_duty = calculate_dc_voltage();                        // DC voltage duty

        // CRITICAL: DC bias holds rotor, pulsating injection on Q-axis only
        voltage_out_.alpha = dc_duty;                                    // DC Bias holds the rotor on D-axis
        voltage_out_.beta = injection_scalar;                            // Pulsating voltage on Q-axis

        // 4. Check if measurement complete (100 periods)
        if (hfi_.complete()) {
            // Get Q-axis current amplitude (we swapped inputs, so result is in D)
            int16_t amplitude_q = hfi_.get_amplitude_d();                // Use D-axis of swapped frame

            // Calculate Lq from measurement
            // L = V / (2 * pi * f * I)
            // K = 1000000 / (2 * 3.14159 * f) = 159155 / f
            int32_t v_mv = hfi_amplitude_mv_;                            // HFI voltage (mV)
            int32_t i_ma = fb_.current.norm2ma(amplitude_q);             // Current (mA)
            if (i_ma > 0) {
                // Lq = V * K / I, where K = 159155 / HFI_FREQUENCY
                int32_t k_const = 159155 / HFI_FREQUENCY;                // Calculate K constant
                lq_uh_ = (v_mv * k_const) / i_ma;                        // Lq in uH
            }

            // // Log Lq measurement results
            // SerialUSB.println("[HFI Lq] COMPLETE");
            // SerialUSB.print("  Q-axis Amplitude: ");
            // SerialUSB.print(i_ma);
            // SerialUSB.println("mA");
            SerialUSB.print("  Lq: ");
            SerialUSB.print(lq_uh_);
            SerialUSB.println("uH");

            // Store Lq in motor parameters
            motor_.set_phase_inductance(1, lq_uh_);                      // Store as phase 1

            // Start pause before next measurement cycle
            // SerialUSB.println("[HFI] === CYCLE COMPLETE - Starting 200ms Pause ===");
            state_ = HFICal_Pause;                                       // Go to pause state
            pause_counter_ = 0;                                          // Reset pause counter
            voltage_out_ = Vec2D(0, 0);                                  // Zero output during pause
        }
    }

    /// Pause state - wait 200ms between measurement cycles
    void tick_pause() {
        // Keep DC bias to hold rotor during pause
        int16_t dc_duty = calculate_dc_voltage();                        // DC voltage duty
        voltage_out_.alpha = dc_duty;                                    // DC bias on D-axis
        voltage_out_.beta = 0;                                           // Zero on Q-axis

        // Increment pause counter
        pause_counter_++;

        // Check if pause complete (200ms = 3600 cycles at 18kHz)
        if (pause_counter_ >= PAUSE_CYCLES) {
            // Pause complete, restart measurement cycle
            // SerialUSB.println("[HFI] === STARTING NEW MEASUREMENT CYCLE ===");
            state_ = HFICal_LdMeasure;                                   // Go back to Ld measurement
            start_hfi_measurement();                                     // Start new Ld measurement
        }
    }
};
