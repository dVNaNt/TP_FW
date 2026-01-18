#pragma once

// Motor Electric Test Module - Phase Calibration Algorithm
//
// This module implements automated motor parameter identification:
// 1. HFI-based inductance measurement for each phase pattern
// 2. Resistance measurement using PI controller with calculated Kp from inductance
// 3. Motor type detection based on measured parameters
//
// Algorithm Overview:
// - For each phase pattern (0, 1, 2):
//   a) Inject HFI sine wave to measure inductance (peak-to-peak current method)
//   b) Wait 1s
//   c) Use measured inductance to calculate PI controller Kp
//   d) Apply DC current to measure resistance
//   e) Wait 1s
// - All calculations in DQ coordinates (Angle 0)
//
// Licensed under the Apache License, Version 2.0
// Copyright 2024 Anton Khrustalev, creapunk.com

#include "../../common.h"
#include "../../math_integer/filters/filter1st.h"
#include "../motor.h"
#include "../types.h"
#include "sel_motor.h"
#include "../../math_integer/helpers/frequency_generator.h"

// Forward declarations
struct Feedback;
struct MotorSelector;

// =============================================================================
// CONSTANTS (Timing @ 18kHz)
// =============================================================================
constexpr uint16_t TEST_HFI_PERIODS_CHECK = 10;     // HFI periods for amplitude check (fast)
constexpr uint16_t TEST_HFI_PERIODS_MEAS  = 50;     // HFI periods for final measurement
constexpr uint16_t TEST_HFI_FREQUENCY     = 1000;   // HFI frequency: 1kHz
constexpr int32_t  TEST_HFI_START_MV      = 25;     // Starting HFI voltage: 100mV
constexpr uint16_t TEST_SETTLE_CYCLES     = 900;    // 50ms settle time
constexpr uint16_t TEST_MEASURE_SAMPLES   = 256;    // Samples for averaging
constexpr uint16_t TEST_RAMP_MAX_CYCLES   = 3600;   // 200ms max ramp time
constexpr uint16_t TEST_STABLE_COUNT      = 360;    // 20ms stability requirement
constexpr uint32_t TEST_WAIT_CYCLES       = 18000;  // 1s delay (18kHz)

// Current limits
constexpr int16_t TEST_OVERCURRENT        = INT16_MAX / 2;  // 50% full scale
constexpr int16_t TEST_CURRENT_EPSILON    = 50;             // Current tolerance

// PI controller constants  
constexpr int32_t TEST_R_EST_MOHM         = 50;     // Min expected resistance for Ki: 50mOhm
constexpr uint8_t TEST_LPF_ALPHA          = 230;    // LPF filter alpha

/// Test state machine states
enum TestState : uint8_t {
    Test_Idle = 0,           // Not running
    Test_HFI_Inject,         // HFI injection for inductance measurement
    Test_HFI_Measure,        // Measuring HFI response
    Test_HFI_Adjust,         // Adjusting HFI amplitude
    Test_Wait_Post_HFI,      // 1s Delay after HFI
    Test_Resistance_Ramp,    // Ramping current for resistance measurement
    Test_Resistance_Settle,  // Settling before resistance measurement
    Test_Resistance_Measure, // Measuring resistance
    Test_Wait_Post_DC,       // 1s Delay after DC (Resistance)
    Test_Complete,           // Test complete
    Test_Error               // Error occurred
};

/// DC Phase Check - Motor Parameter Identification
/// Measures inductance and resistance for each phase pattern
class DCPhaseCheck {
private:
    // References to external modules
    Feedback& fb_;           // Feedback for current/voltage sensing
    Motor& motor_;           // Motor parameters storage
    MotorSelector& sel_;     // Motor selector for voltage output

    // Frequency generator for HFI
    FrequencyGenerator hfi_gen_;  // HFI signal generator

    // DFT accumulators for HFI analysis
    int64_t dft_d_real_;     // D-axis real component accumulator
    int64_t dft_d_imag_;     // D-axis imaginary component accumulator
    int16_t hfi_current_min_;  // Minimum current during HFI
    int16_t hfi_current_max_;  // Maximum current during HFI

    // Measurement accumulators
    int32_t accum_v_;        // Voltage accumulator for averaging
    int32_t accum_i_;        // Current accumulator for averaging
    int32_t res_v1_;         // First point voltage (mV)
    int32_t res_i1_;         // First point current (mA)

    // Counters
    uint32_t cnt_;           // General purpose counter
    uint32_t hfi_sample_cnt_;  // HFI sample counter
    uint32_t hfi_total_samples_;  // Total HFI samples needed
    uint16_t stable_cnt_;    // Stability counter
    uint16_t samp_cnt_;      // Sample counter for averaging

    // HFI parameters
    int32_t hfi_amplitude_mv_;  // Current HFI amplitude in mV
    int32_t hfi_max_mv_;        // Max HFI amplitude = Vbus * 3/4
    int32_t measured_inductance_uh_;  // Measured inductance in uH
    bool hfi_final_measurement_;  // Flag: true = final 50-period measurement

    // PI controller state
    int32_t pi_integral_;    // PI integral accumulator
    int32_t pi_kp_;          // Calculated Kp from inductance
    int32_t pi_ki_;          // Calculated Ki from min resistance estimate
    int16_t target_current_; // Target current for resistance measurement
    int16_t target_hfi_pp_;  // Target HFI peak-to-peak current (1/4 max)

    // Voltage/duty output
    int16_t voltage_mv_;     // Current output voltage in mV
    int16_t duty_d_;         // D-axis duty cycle

    // LPF for current filtering
    Filter1st lpf_;          // Low-pass filter for current

    // State machine
    TestState state_;        // Current test state
    uint8_t pattern_idx_;    // Current pattern index (0-2)
    uint8_t hfi_iterations_; // HFI amplitude adjustment iterations

    // Flags
    bool is_second_point_;   // Flag for second resistance measurement point

    // --- Private Helper Methods ---

    /// Apply voltage to D-axis (DQ coordinates, Angle 0)
    /// @param duty_d D-axis duty cycle
    inline void apply_voltage_dq(int16_t duty_d) {
        duty_d_ = duty_d;  // Store for reference
        // Apply to selector: D-axis only (Q=0 for DC test), Angle 0 assumed by vector logic
        sel_.tick_voltage(Vec2D(duty_d, 0));  // D=cos, Q=sin
    }

    /// Set Hi-Z state (all phases floating)
    inline void set_hiz() {
        duty_d_ = 0;  // Clear duty
        sel_.tick_voltage(Vec2D(0, 0));  // Zero voltage
    }

    /// Convert mV to duty cycle
    /// @param mv Voltage in millivolts
    /// @return Duty cycle in Q15 format
    inline int16_t mv2duty(int32_t mv) const {
        return fb_.supply.mv2duty(mv);  // Use supply voltage converter
    }

    /// Prepare for new phase pattern test
    void prepare_pattern() {
        // Direct mapping: 0=CBA, 1=ACB, 2=ABC
        // Sequence logic simplified as requested
        
        // Reset state variables
        state_ = Test_HFI_Inject; 
        cnt_ = 0;  // Reset counter
        stable_cnt_ = 0;  // Reset stability counter
        samp_cnt_ = 0;  // Reset sample counter

        // Reset HFI variables
        hfi_amplitude_mv_ = TEST_HFI_START_MV;  // Start with 100mV
        hfi_max_mv_ = (fb_.supply.get_voltage_mv() * 3) >> 2;  // Max = Vbus * 3/4
        hfi_iterations_ = 0;  // Reset iteration counter
        hfi_sample_cnt_ = 0;  // Reset HFI sample counter
        hfi_final_measurement_ = false;  // Start with amplitude check (10 periods)
        dft_d_real_ = 0;  // Clear DFT accumulators
        dft_d_imag_ = 0;
        hfi_current_min_ = INT16_MAX;  // Reset min/max
        hfi_current_max_ = INT16_MIN;
        measured_inductance_uh_ = 0;  // Clear inductance

        // Reset PI controller - MUST reset for each pattern
        pi_integral_ = 0;  // Clear integral accumulator
        pi_kp_ = 100;  // Default Kp (will be calculated from inductance)
        pi_ki_ = 5;    // Default Ki (will be calculated from min R estimate)

        // Reset measurement variables
        voltage_mv_ = 0;  // Clear voltage
        duty_d_ = 0;  // Clear duty
        is_second_point_ = false;  // Start with first point
        accum_v_ = 0;  // Clear accumulators
        accum_i_ = 0;
        res_v1_ = 0;  // Clear first point results
        res_i1_ = 0;

        // Reset filter
        lpf_.reset(0);  // Reset LPF to zero

        // Calculate target current (75% of limit, max 50% of sensor scale)
        int32_t motor_75 = (motor_.get_current_limit() * 3) >> 2;  // 75% of motor limit
        int32_t scale_50 = fb_.current.norm2ma(INT16_MAX / 2);  // 50% of sensor scale
        int32_t target_ma = (motor_75 < scale_50) ? motor_75 : scale_50;  // Use smaller
        target_current_ = fb_.current.ma2norm(target_ma);  // Convert to normalized

        // Calculate target HFI peak-to-peak current: 1/4 of max current
        int32_t max_current = motor_.get_current_limit();  // Max current in mA
        int32_t hfi_target_ma = max_current >> 2;  // 1/4 of max current
        target_hfi_pp_ = fb_.current.ma2norm(hfi_target_ma);  // Convert to normalized

        // Configure motor selector
        sel_.set_motor_type(MotorType_DC);  // DC mode for testing
        // Use pattern_idx_ directly as per optimization request
        sel_.set_phase_pattern(static_cast<PhasePattern>(pattern_idx_));

        // Log pattern start
        SerialUSB.print("[TEST] Pattern ");
        SerialUSB.println(pattern_idx_);
    }

    /// Start HFI measurement cycle
    /// @param final_measurement If true, use 50 periods; if false, use 10 periods for amplitude check
    void start_hfi_measurement() {
        // Configure frequency generator
        hfi_gen_.set_frequency_target(TEST_HFI_FREQUENCY);  // Set HFI frequency
        int16_t amplitude_duty = mv2duty(hfi_amplitude_mv_);  // Convert to duty
        hfi_gen_.set_amplitude(amplitude_duty);  // Set amplitude
        hfi_gen_.reset();  // Reset phase to zero

        // Reset DFT accumulators
        dft_d_real_ = 0;
        dft_d_imag_ = 0;
        hfi_sample_cnt_ = 0;
        hfi_current_min_ = INT16_MAX;
        hfi_current_max_ = INT16_MIN;

        // Calculate HFI samples based on mode
        uint16_t freq_sys = 18000;  // System frequency
        uint16_t samples_per_period = freq_sys / TEST_HFI_FREQUENCY;  // Samples per period
        uint16_t periods = hfi_final_measurement_ ? TEST_HFI_PERIODS_MEAS : TEST_HFI_PERIODS_CHECK;
        hfi_total_samples_ = samples_per_period * periods;  // Total samples

        // Transition to HFI inject state
        state_ = Test_HFI_Inject;
    }

    /// Calculate inductance from HFI measurement
    /// L = V / (2 * pi * f * I_pp / 2) = V / (pi * f * I_pp)
    /// @return Inductance in microhenries
    int32_t calculate_inductance() {
        // Calculate peak-to-peak current
        int32_t i_pp_norm = hfi_current_max_ - hfi_current_min_;  // Peak-to-peak in Q15
        int32_t i_pp_ma = fb_.current.norm2ma(i_pp_norm);  // Convert to mA

        if (i_pp_ma <= 0) {
            return 0;  // Invalid measurement
        }

        // L = V / (pi * f * I_pp)
        // For 1kHz: L_uH = V_mV * 318 / I_pp_mA (318 = 1000000 / (pi * 1000))
        int32_t l_uh = (hfi_amplitude_mv_ * 318) / i_pp_ma;

        return l_uh;
    }

    /// Calculate PI controller Kp from inductance
    /// Kp proportional to inductance for current loop bandwidth
    int32_t calculate_kp_from_inductance(int32_t inductance_uh) {
        // Kp = L * 2 * pi * f_bw
        // Simplified: Kp = L_uH * 3 / 1000 (approx for 500Hz bandwidth)
        int32_t kp = (inductance_uh * 3) / 1000;

        // Only apply minimum limit, no maximum - let it scale with inductance
        if (kp < 1) kp = 1;  // Minimum Kp to avoid zero

        return kp;
    }

    /// Calculate PI controller Ki from minimum expected resistance
    /// Ki = R_min / L (ensures stability with lowest expected resistance)
    int32_t calculate_ki_from_inductance(int32_t inductance_uh) {
        // Ki = R_min / L
        if (inductance_uh <= 0) return 5;  // Default if invalid
        
        int32_t ki = (TEST_R_EST_MOHM * 1000) / inductance_uh;
        
        // Clamp to reasonable range
        if (ki < 1) ki = 1;    // Minimum Ki
        if (ki > 100) ki = 100;  // Maximum Ki
        
        return ki;
    }

    /// PI controller tick for current regulation
    /// @param error Current error (target - measured)
    /// @param limit Voltage limit in mV
    /// @return Voltage output in mV (always positive)
    int32_t pi_tick(int16_t error, int32_t limit) {
        // Proportional term
        int32_t p = ((int32_t)error * pi_kp_) >> 7;  // Scale Kp

        // Integral term with anti-windup (use calculated Ki)
        pi_integral_ += ((int32_t)error * pi_ki_) >> 7;

        // Clamp integral to positive range only (voltage magnitude)
        if (pi_integral_ > limit) pi_integral_ = limit;
        if (pi_integral_ < 0) pi_integral_ = 0;  // No negative integral

        // Output
        int32_t output = p + pi_integral_;

        // Clamp output to positive range (voltage magnitude)
        if (output > limit) output = limit;
        if (output < 0) output = 0;  // No negative voltage

        return output;
    }

public:
    /// Constructor
    DCPhaseCheck(Feedback& fb, Motor& motor, MotorSelector& sel)
        : fb_(fb)
        , motor_(motor)
        , sel_(sel)
        , hfi_gen_(18000)  // Initialize with 18kHz system frequency
        , lpf_(0, TEST_LPF_ALPHA)
        , state_(Test_Idle)
        , pattern_idx_(0)
    {
    }

    /// Start test sequence
    void start(bool repeat = false) {
        SerialUSB.println("[TEST] === STARTING MOTOR ELECTRIC TEST ===");
        pattern_idx_ = 0;  // Start with first pattern
        prepare_pattern();  // Prepare first pattern
    }

    /// Main tick function - call at PWM frequency (18kHz)
    void tick() {
        // Fast exit if idle or complete
        if (state_ == Test_Idle || state_ == Test_Complete) {
            set_hiz();  // Ensure Hi-Z
            return;
        }

        // Read and filter current (D-axis in DQ coordinates)
        sel_.tick_current(fb_.current.norm());  // Update current selector
        Vec2D current_dq = sel_.get_currents();  // Get DQ currents
        int16_t current_d = current_dq.d;  // D-axis current (flux)
        
        // Filter D-axis current directly (no manual polarity flips)
        int16_t filt = lpf_.tick_lpf(current_d);

        // Safety check - overcurrent protection (check absolute value)
        int16_t filt_abs = (filt >= 0) ? filt : -filt;  // Absolute value
        if (filt_abs > TEST_OVERCURRENT) {
            SerialUSB.println("[TEST] ERROR: Overcurrent detected!");
            motor_.set_phase_resistance(pattern_idx_, 0);  // Mark as short
            motor_.set_status(DriverStatus_Error);  // Set error status
            state_ = Test_Error;  // Transition to error state
            set_hiz();  // Hi-Z for safety
            return;
        }

        // State machine
        switch (state_) {
            case Test_HFI_Inject:
                tick_hfi_inject(current_d); 
                break;

            case Test_HFI_Adjust:
                tick_hfi_adjust();
                break;

            case Test_Wait_Post_HFI:
                tick_wait_post_hfi();
                break;

            case Test_Resistance_Ramp:
                tick_resistance_ramp(filt);
                break;

            case Test_Resistance_Settle:
                tick_resistance_settle();
                break;

            case Test_Resistance_Measure:
                tick_resistance_measure(filt);
                break;
            
            case Test_Wait_Post_DC:
                tick_wait_post_dc();
                break;

            default:
                set_hiz();  // Safety default
                break;
        }
    }

    /// Check if test is complete
    inline bool is_complete() const { return state_ == Test_Complete; }

    /// Check if error occurred
    inline bool is_error() const { return state_ == Test_Error; }

private:

    /// HFI injection state - inject sine wave and measure response
    /// @param current_d Current reading in D axis
    void tick_hfi_inject(int16_t current_d) {
        // Generate HFI voltage
        Vec2D hfi_voltage = hfi_gen_.tick();  // Get sin/cos from generator
        int16_t injection = hfi_voltage.cos;  // Use cosine for D-axis injection

        // Apply to D-axis (Angle 0)
        apply_voltage_dq(injection);

        // Track min/max current for peak-to-peak calculation
        if (current_d < hfi_current_min_) hfi_current_min_ = current_d;
        if (current_d > hfi_current_max_) hfi_current_max_ = current_d;

        // Get reference sin/cos for DFT
        int16_t ref_sin, ref_cos;
        angle2sincos(hfi_gen_.get_angle(), &ref_sin, &ref_cos);

        // Accumulate DFT (correlation with reference)
        dft_d_real_ += (int64_t)current_d * (int64_t)ref_cos;
        dft_d_imag_ += (int64_t)current_d * (int64_t)ref_sin;

        // Check for completion
        if (++hfi_sample_cnt_ >= hfi_total_samples_) {
            // Calculate inductance from peak-to-peak current
            measured_inductance_uh_ = calculate_inductance();

            SerialUSB.print("[TEST] HFI: V=");
            SerialUSB.print(hfi_amplitude_mv_);
            SerialUSB.print("mV, I_pp=");
            SerialUSB.print(hfi_current_max_ - hfi_current_min_);
            SerialUSB.print(", L=");
            SerialUSB.print(measured_inductance_uh_);
            SerialUSB.print("uH");
            SerialUSB.println(hfi_final_measurement_ ? " [FINAL]" : " [CHECK]");

            // If this was final measurement, proceed to Wait (then Resistance)
            if (hfi_final_measurement_) {
                // Store inductance using direct pattern index
                motor_.set_phase_inductance(pattern_idx_, measured_inductance_uh_);

                // Calculate Kp and Ki from inductance
                pi_kp_ = calculate_kp_from_inductance(measured_inductance_uh_);
                pi_ki_ = calculate_ki_from_inductance(measured_inductance_uh_);
                
                SerialUSB.print("[TEST] Calculated Kp=");
                SerialUSB.print(pi_kp_);
                SerialUSB.print(", Ki=");
                SerialUSB.println(pi_ki_);

                // Prepare for next step
                pi_integral_ = 0; 
                voltage_mv_ = 0;
                is_second_point_ = false;
                cnt_ = 0;
                
                // ADDED: Delay 1s after HFI
                state_ = Test_Wait_Post_HFI;
                return;
            }

            // Amplitude check mode - adjust if needed
            // Target: 1/4 of max current (target_hfi_pp_)
            int32_t i_pp = hfi_current_max_ - hfi_current_min_;

            if (i_pp > (target_hfi_pp_ << 1)) {
                // Current too high (>2x target) - reduce amplitude
                hfi_amplitude_mv_ = (hfi_amplitude_mv_ * target_hfi_pp_) / i_pp;
                if (hfi_amplitude_mv_ < 50) hfi_amplitude_mv_ = 50;  // Minimum 50mV
                state_ = Test_HFI_Adjust;
            } else if (i_pp < (target_hfi_pp_ >> 1) && hfi_amplitude_mv_ < hfi_max_mv_) {
                // Current too low (<0.5x target) - increase amplitude
                hfi_amplitude_mv_ = hfi_amplitude_mv_ * 2;
                if (hfi_amplitude_mv_ > hfi_max_mv_) hfi_amplitude_mv_ = hfi_max_mv_;
                state_ = Test_HFI_Adjust;
            } else {
                // Amplitude OK - do final measurement with 50 periods
                hfi_final_measurement_ = true;
                start_hfi_measurement();  // Restart with 50 periods
            }
        }
    }

    /// HFI amplitude adjustment state
    void tick_hfi_adjust() {
        // REMOVED: set_hiz() call for seamless transition

        if (++hfi_iterations_ > 10) {
            // Too many iterations - do final measurement anyway
            SerialUSB.println("[TEST] HFI adjustment timeout, doing final measurement");
            hfi_final_measurement_ = true;
            start_hfi_measurement();  // Do final measurement with current amplitude
        } else {
            // Restart HFI with new amplitude (still in check mode)
            start_hfi_measurement();
        }
    }

    /// Wait state after HFI measurement (1s delay)
    void tick_wait_post_hfi() {
        set_hiz(); // Relax motor

        if (++cnt_ >= TEST_WAIT_CYCLES) {
            // Delay complete, reset for Resistance Measurement
            cnt_ = 0;
            stable_cnt_ = 0;
            
            // Start Resistance measurement
            SerialUSB.println("[TEST] Starting resistance measurement");
            state_ = Test_Resistance_Ramp;
        }
    }

    /// Resistance ramp state - ramp current to target using PI controller
    /// @param filt Filtered D-axis current
    void tick_resistance_ramp(int16_t filt) {
        // Calculate voltage limit (50% of supply)
        int32_t limit_mv = fb_.supply.get_voltage_mv() >> 1;

        if (!is_second_point_) {
            // First point: ramp to target current using PI controller
            int16_t error = target_current_ - filt;  // Error = target - measured
            voltage_mv_ = pi_tick(error, limit_mv);  // PI controller output in mV

            // Apply voltage to D-axis (Angle 0)
            int16_t duty = mv2duty(voltage_mv_);  // Convert to duty
            apply_voltage_dq(duty); 

            // Check stability
            int16_t abs_error = (error >= 0) ? error : -error;
            if (abs_error < TEST_CURRENT_EPSILON) {
                if (++stable_cnt_ >= TEST_STABLE_COUNT) {
                    // Stable - proceed to settle
                    state_ = Test_Resistance_Settle;
                    cnt_ = 0;
                }
            } else {
                stable_cnt_ = 0;  // Reset stability counter
            }

            // Timeout check
            if (++cnt_ >= TEST_RAMP_MAX_CYCLES) {
                state_ = Test_Resistance_Settle;
                cnt_ = 0;
            }
        } else {
            // Second point: use half voltage from first point
            voltage_mv_ = res_v1_ >> 1;  // Half of first point voltage
            int16_t duty = mv2duty(voltage_mv_);  // Convert to duty
            apply_voltage_dq(duty);

            // Wait for stability
            if (++cnt_ >= TEST_STABLE_COUNT) {
                state_ = Test_Resistance_Settle;
                cnt_ = 0;
            }
        }
    }

    /// Resistance settle state - wait for transients to decay
    void tick_resistance_settle() {
        // Keep voltage constant
        int16_t duty = mv2duty(voltage_mv_);
        apply_voltage_dq(duty); 

        if (++cnt_ >= TEST_SETTLE_CYCLES) {
            // Proceed to measurement
            state_ = Test_Resistance_Measure;
            accum_v_ = 0;
            accum_i_ = 0;
            samp_cnt_ = 0;
        }
    }

    /// Resistance measure state - average voltage and current
    /// @param filt Filtered D-axis current
    void tick_resistance_measure(int16_t filt) {
        // Keep voltage constant
        int16_t duty = mv2duty(voltage_mv_);
        apply_voltage_dq(duty);

        // Accumulate samples
        accum_v_ += voltage_mv_;  // Voltage in mV
        accum_i_ += filt;         // Current

        if (++samp_cnt_ >= TEST_MEASURE_SAMPLES) {
            // Calculate averages
            int32_t avg_v = accum_v_ / TEST_MEASURE_SAMPLES;
            int32_t avg_i_norm = accum_i_ / TEST_MEASURE_SAMPLES;
            int32_t avg_i_ma = fb_.current.norm2ma(avg_i_norm);

            if (!is_second_point_) {
                // First point complete
                res_v1_ = avg_v;
                res_i1_ = avg_i_ma;

                SerialUSB.print("[TEST] Point 1: V=");
                SerialUSB.print(res_v1_);
                SerialUSB.print("mV, I=");
                SerialUSB.print(res_i1_);
                SerialUSB.println("mA");

                // Setup for second point - reset PI integral
                is_second_point_ = true;
                pi_integral_ = 0;  // Reset integral for second point
                cnt_ = 0;
                stable_cnt_ = 0;
                state_ = Test_Resistance_Ramp;
            } else {
                // Second point complete - calculate resistance
                int32_t v2 = avg_v;
                int32_t i2 = avg_i_ma;

                SerialUSB.print("[TEST] Point 2: V=");
                SerialUSB.print(v2);
                SerialUSB.print("mV, I=");
                SerialUSB.print(i2);
                SerialUSB.println("mA");

                // Calculate resistance: R = dV / dI
                int32_t dv = res_v1_ - v2;
                int32_t di = res_i1_ - i2;

                if (dv < 0) dv = -dv;  // Absolute value
                if (di < 0) di = -di;

                int32_t resistance_mohm;
                if (di < 10) {
                    // Current too low - open circuit
                    resistance_mohm = INT32_MAX;
                    SerialUSB.println("[TEST] Open circuit detected");
                } else {
                    // R = dV * 1000 / dI (mOhm)
                    resistance_mohm = (dv * 1000) / di;
                    SerialUSB.print("[TEST] Resistance: ");
                    SerialUSB.print(resistance_mohm);
                    SerialUSB.println(" mOhm");
                }

                // Store resistance directly using pattern index
                motor_.set_phase_resistance(pattern_idx_, resistance_mohm);

                // Transition to Wait Post DC
                state_ = Test_Wait_Post_DC;
                cnt_ = 0;
                set_hiz();
                lpf_.reset(0);
            }
        }
    }

    /// Wait state after Resistance measurement (1s delay)
    void tick_wait_post_dc() {
        set_hiz(); // Keep Hi-Z

        if (++cnt_ >= TEST_WAIT_CYCLES) {
             // Check if more patterns to test
            if (++pattern_idx_ >= 3) {
                // All patterns complete
                SerialUSB.println("[TEST] === TEST COMPLETE ===");
                sel_.set_motor_type(MotorType_Invalid);  // Reset selector
                motor_.detect_type();  // Detect motor type from measurements
                motor_.print(SerialUSB);  // Print results
                state_ = Test_Complete;
            } else {
                // Prepare next pattern
                prepare_pattern();
            }
        }
    }
};
