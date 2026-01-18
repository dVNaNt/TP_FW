#pragma once

// Calibration Module: HFI Inductance + Resistance Measurement
//
// 1. HFI_Inductance: Measures phase inductance using HFI injection
// 2. ResistanceMeasure: Measures resistance using PI controller with 2-point method
//
// PI Controller Tuning:
// - Kp = L_uH / 2 (from measured inductance)
// - Ki = 50mOhm * 1000 / L_uH (from minimum expected resistance)
//
// Resistance Measurement:
// - Point 1: 3/4 of target current
// - Point 2: 3/8 of target current
// - R = dV / dI (two-point method)
//
// Licensed under the Apache License, Version 2.0
// Copyright 2024 Anton Khrustalev, creapunk.com

#include "../../common.h"
#include "../../feedback/feedback.h"
#include "../../math_integer/controllers/pid.h"
#include "../../math_integer/helpers/frequency_generator.h"

// =============================================================================
// HFI INDUCTANCE CONSTANTS
// =============================================================================
constexpr uint16_t HFI_L_FREQUENCY       = 1000;    // HFI frequency: 1kHz
constexpr uint16_t HFI_L_PERIODS_CHECK   = 3;       // Periods for amplitude check
constexpr uint16_t HFI_L_PERIODS_FINAL   = 25;      // Periods for final measurement
constexpr uint8_t  HFI_L_MAX_ITERATIONS  = 5;       // Max amplitude adjustment iterations
constexpr int32_t  HFI_L_START_MV        = 50;      // Starting amplitude: always 50mV
constexpr int32_t  HFI_L_CONST           = 318310;  // 1000000 / pi

// =============================================================================
// RESISTANCE MEASUREMENT CONSTANTS
// =============================================================================
constexpr int32_t  RES_MIN_MOHM          = 50;      // Min expected resistance for Ki: 50mOhm
constexpr uint16_t RES_SETTLE_CYCLES     = 900;     // 50ms settle time @ 18kHz
constexpr uint16_t RES_MEASURE_SAMPLES   = 256;     // Samples for averaging (oversampling)
constexpr uint16_t RES_STABLE_COUNT      = 360;     // 20ms stability requirement
constexpr uint16_t RES_RAMP_MAX_CYCLES   = 3600;    // 200ms max ramp time
constexpr int16_t  RES_CURRENT_EPSILON   = 100;     // Current tolerance (Q15)
constexpr uint8_t  RES_LPF_ALPHA         = 230;     // LPF filter alpha for current

/// HFI Inductance measurement states
enum HFI_L_State : uint8_t {
    HFI_L_Idle = 0,
    HFI_L_Trial,
    HFI_L_Adjust,
    HFI_L_Final,
    HFI_L_Finished,
    HFI_L_Error
};

/// Resistance measurement states
enum Res_State : uint8_t {
    Res_Idle = 0,
    Res_Ramp1,           // Ramping to 3/4 target current
    Res_Settle1,         // Settling before first measurement
    Res_Measure1,        // Measuring first point
    Res_Ramp2,           // Ramping to 3/8 target current (fixed voltage)
    Res_Settle2,         // Settling before second measurement
    Res_Measure2,        // Measuring second point
    Res_Finished,
    Res_Error
};

// =============================================================================
// HFI INDUCTANCE MEASUREMENT
// =============================================================================
struct HFI_Inductance {
private:
    Feedback& fb_;
    FrequencyGenerator gen_;
    
    int32_t amplitude_mv_;
    int32_t amplitude_max_mv_;
    int32_t target_current_ma_;
    int32_t inductance_uh_;
    uint32_t sample_cnt_;
    uint32_t total_samples_;
    uint16_t samples_per_period_;
    int16_t current_min_;
    int16_t current_max_;
    uint8_t state_;
    uint8_t iteration_cnt_;

    void start_cycle(uint16_t periods) {
        gen_.set_amplitude(fb_.supply.mv2duty(amplitude_mv_));           // Set amplitude
        gen_.reset();                                                    // Reset phase
        sample_cnt_ = 0;                                                 // Reset counter
        total_samples_ = (uint32_t)samples_per_period_ * periods;        // Total samples
        current_min_ = INT16_MAX;                                        // Reset min
        current_max_ = INT16_MIN;                                        // Reset max
    }

    int32_t calculate_inductance(int32_t v_mv, int32_t i_pp_ma) const {
        if (i_pp_ma <= 0) return 0;                                      // Invalid
        return (v_mv * (HFI_L_CONST / HFI_L_FREQUENCY)) / i_pp_ma;        // L = V*K/I
    }

    void process_phase_complete() {
        int32_t i_pp_ma = fb_.current.norm2ma(current_max_ - current_min_);
        
        if (state_ == HFI_L_Trial) {
            process_adjustment(i_pp_ma);                                 // Check amplitude
        } else {
            inductance_uh_ = calculate_inductance(amplitude_mv_, i_pp_ma);
            SerialUSB.print("L=");
            SerialUSB.print(inductance_uh_);
            SerialUSB.println("uH");
            state_ = HFI_L_Finished;
        }
    }

    void process_adjustment(int32_t i_pp_ma) {
        // Target reached?
        if (i_pp_ma >= target_current_ma_) {
            state_ = HFI_L_Final;
            start_cycle(HFI_L_PERIODS_FINAL);
            return;
        }
        // Max voltage?
        if (amplitude_mv_ >= amplitude_max_mv_) {
            state_ = HFI_L_Final;
            start_cycle(HFI_L_PERIODS_FINAL);
            return;
        }
        // Max iterations?
        if (++iteration_cnt_ > HFI_L_MAX_ITERATIONS) {
            state_ = HFI_L_Error;
            return;
        }
        // Calculate scale Q8
        int32_t k = (i_pp_ma > 0) ? (target_current_ma_ << 8) / i_pp_ma : 1280;
        if (k > 1280) k = 1280;                                          // Max 5x
        if (k < 256) k = 256;                                            // Min 1x
        // Close enough?
        if (k < 307) {
            state_ = HFI_L_Final;
            start_cycle(HFI_L_PERIODS_FINAL);
            return;
        }
        // Apply scale
        amplitude_mv_ = (amplitude_mv_ * k) >> 8;
        if (amplitude_mv_ > amplitude_max_mv_) amplitude_mv_ = amplitude_max_mv_;
        
        state_ = HFI_L_Trial;
        start_cycle(HFI_L_PERIODS_CHECK);
    }

public:
    HFI_Inductance(Feedback& fb, uint16_t freq_sys)
        : fb_(fb), gen_(freq_sys)
        , amplitude_mv_(HFI_L_START_MV), amplitude_max_mv_(0)
        , target_current_ma_(0), inductance_uh_(0)
        , sample_cnt_(0), total_samples_(0)
        , samples_per_period_(freq_sys / HFI_L_FREQUENCY)
        , current_min_(INT16_MAX), current_max_(INT16_MIN)
        , state_(HFI_L_Idle), iteration_cnt_(0)
    {
        gen_.set_frequency_target(HFI_L_FREQUENCY);
    }

    void start(int32_t target_ma, int32_t v_max_mv) {
        target_current_ma_ = target_ma;
        amplitude_max_mv_ = v_max_mv;
        amplitude_mv_ = HFI_L_START_MV;
        iteration_cnt_ = 0;
        inductance_uh_ = 0;
        state_ = HFI_L_Trial;
        start_cycle(HFI_L_PERIODS_CHECK);
    }

    int16_t tick(int16_t current_d) {
        if (state_ == HFI_L_Idle || state_ >= HFI_L_Finished) return 0;
        
        // Track min/max
        if (current_d < current_min_) current_min_ = current_d;
        if (current_d > current_max_) current_max_ = current_d;
        
        // Check completion
        if (++sample_cnt_ >= total_samples_) {
            process_phase_complete();
        }
        
        return gen_.tick().cos;
    }

    bool is_finished() const { return state_ >= HFI_L_Finished; }
    bool is_error() const { return state_ == HFI_L_Error; }
    bool is_running() const { return state_ > HFI_L_Idle && state_ < HFI_L_Finished; }
    int32_t get_inductance_uh() const { return inductance_uh_; }
    HFI_L_State get_state() const { return (HFI_L_State)state_; }
    int32_t get_amplitude_mv() const { return amplitude_mv_; }
};

// =============================================================================
// RESISTANCE MEASUREMENT WITH PI CONTROLLER
// =============================================================================
struct ResistanceMeasure {
private:
    Feedback& fb_;                   // Feedback reference for conversions
    Filter1st lpf_;                  // LPF for current filtering
    
    // PI controller state
    int32_t pi_integral_;            // PI integral accumulator
    int32_t pi_kp_;                  // Calculated Kp from inductance
    int32_t pi_ki_;                  // Calculated Ki from min resistance
    
    // Voltage/current tracking
    int32_t voltage_mv_;             // Current output voltage (mV)
    int32_t voltage_fixed_mv_;       // Fixed voltage after stabilization
    int64_t accum_v_;                // Voltage accumulator (64-bit for oversampling)
    int64_t accum_i_;                // Current accumulator (64-bit for oversampling)
    int32_t res_v1_;                 // Point 1 voltage (mV)
    int32_t res_i1_;                 // Point 1 current (mA)
    int32_t resistance_mohm_;        // Result: resistance (mOhm)
    
    int16_t target_current_;         // Target current for point 1 (Q15 norm)
    int16_t current_limit_;          // Max allowed current (Q15 norm)
    
    uint16_t cnt_;                   // General counter
    uint16_t stable_cnt_;            // Stability counter
    uint16_t samp_cnt_;              // Sample counter
    uint8_t state_;                  // State machine state

    /// PI controller tick for current regulation with voltage limit
    int32_t pi_tick(int16_t error, int32_t limit) {
        int32_t p = ((int32_t)error * pi_kp_) >> 7;                      // Proportional term
        pi_integral_ += ((int32_t)error * pi_ki_) >> 7;                  // Integral term
        if (pi_integral_ > limit) pi_integral_ = limit;                  // Anti-windup high
        if (pi_integral_ < 0) pi_integral_ = 0;                          // Anti-windup low
        int32_t out = p + pi_integral_;                                  // Sum P + I
        if (out > limit) out = limit;                                    // Clamp high
        if (out < 0) out = 0;                                            // Clamp low
        return out;                                                      // Return voltage mV
    }

public:
    ResistanceMeasure(Feedback& fb)
        : fb_(fb)
        , lpf_(0, RES_LPF_ALPHA)
        , pi_integral_(0), pi_kp_(100), pi_ki_(10)
        , voltage_mv_(0), voltage_fixed_mv_(0)
        , accum_v_(0), accum_i_(0)
        , res_v1_(0), res_i1_(0), resistance_mohm_(0)
        , target_current_(0), current_limit_(0)
        , cnt_(0), stable_cnt_(0), samp_cnt_(0)
        , state_(Res_Idle)
    {}

    /// Start resistance measurement
    /// @param target_current Target current for point 1 (3/4 motor limit, Q15 norm)
    /// @param inductance_uh Measured inductance for PI tuning
    void start(int16_t target_current, int32_t inductance_uh) {
        target_current_ = target_current;                                // Point 1: 3/4 motor limit
        current_limit_ = target_current + (target_current >> 2);         // Limit: target + 25% margin
        
        // Kp = L_uH / 2 (proportional to inductance)
        pi_kp_ = inductance_uh >> 1;                                     // Divide by 2
        if (pi_kp_ < 10) pi_kp_ = 10;                                    // Min Kp
        if (pi_kp_ > 1000) pi_kp_ = 1000;                                // Max Kp
        
        // Ki = R_min * 1000 / L_uH (ensures stability)
        pi_ki_ = (inductance_uh > 0) ? (RES_MIN_MOHM * 1000) / inductance_uh : 10;
        if (pi_ki_ < 1) pi_ki_ = 1;                                      // Min Ki
        if (pi_ki_ > 100) pi_ki_ = 100;                                  // Max Ki
        
        SerialUSB.print("R:Kp=");                                        // Log Kp
        SerialUSB.print(pi_kp_);
        SerialUSB.print(" Ki=");                                         // Log Ki
        SerialUSB.print(pi_ki_);
        SerialUSB.print(" lim=");                                        // Log current limit
        SerialUSB.println(fb_.current.norm2ma(current_limit_));
        
        pi_integral_ = 0;                                                // Reset integral
        voltage_mv_ = 0;                                                 // Reset voltage
        voltage_fixed_mv_ = 0;                                           // Reset fixed voltage
        accum_v_ = 0;                                                    // Reset V accumulator
        accum_i_ = 0;                                                    // Reset I accumulator
        res_v1_ = 0;                                                     // Reset point 1 V
        res_i1_ = 0;                                                     // Reset point 1 I
        resistance_mohm_ = 0;                                            // Reset result
        cnt_ = 0;                                                        // Reset counter
        stable_cnt_ = 0;                                                 // Reset stability
        samp_cnt_ = 0;                                                   // Reset samples
        lpf_.reset(0);                                                   // Reset filter
        state_ = Res_Ramp1;                                              // Start ramp to point 1
    }

    /// Main tick function - call at PWM frequency
    /// @param current_d D-axis current (Q15 normalized)
    /// @return Duty cycle for D-axis voltage
    int16_t tick(int16_t current_d) {
        if (state_ == Res_Idle || state_ >= Res_Finished) return 0;      // Exit if not running
        
        int16_t filt = lpf_.tick_lpf(current_d);                             // Filter current
        int32_t v_limit = fb_.supply.get_voltage_mv() >> 1;              // 50% Vbus limit
        
        // Overcurrent protection - reduce voltage if over limit
        if (filt > current_limit_) {                                     // Check overcurrent
            voltage_mv_ = (voltage_mv_ * 3) >> 2;                        // Reduce to 75%
            pi_integral_ = (pi_integral_ * 3) >> 2;                      // Reduce integral
            SerialUSB.println("R:OC!");                                  // Log overcurrent
        }
        
        switch (state_) {
            case Res_Ramp1: {
                // Ramp to target current using PI controller
                int16_t error = target_current_ - filt;                  // Current error
                voltage_mv_ = pi_tick(error, v_limit);                   // PI output
                
                // Check stability (within epsilon for stable_count cycles)
                int16_t abs_err = (error >= 0) ? error : -error;         // Absolute error
                if (abs_err < RES_CURRENT_EPSILON) {                     // Within tolerance
                    if (++stable_cnt_ >= RES_STABLE_COUNT) {             // Stable long enough
                        voltage_fixed_mv_ = voltage_mv_;                 // Lock voltage
                        SerialUSB.print("R:Stab V=");                    // Log stabilization
                        SerialUSB.println(voltage_fixed_mv_);
                        state_ = Res_Settle1;                            // Go to settle
                        cnt_ = 0;                                        // Reset counter
                    }
                } else {
                    stable_cnt_ = 0;                                     // Reset stability
                }
                // Timeout protection
                if (++cnt_ >= RES_RAMP_MAX_CYCLES) {                     // Max ramp time
                    voltage_fixed_mv_ = voltage_mv_;                     // Use current voltage
                    SerialUSB.println("R:Timeout");                      // Log timeout
                    state_ = Res_Settle1;                                // Go to settle
                    cnt_ = 0;                                            // Reset counter
                }
                break;
            }
            
            case Res_Settle1:
                // Wait for transients to decay at fixed voltage
                voltage_mv_ = voltage_fixed_mv_;                         // Hold voltage
                if (++cnt_ >= RES_SETTLE_CYCLES) {                       // Settle complete
                    state_ = Res_Measure1;                               // Go to measure
                    accum_v_ = 0;                                        // Reset accumulators
                    accum_i_ = 0;
                    samp_cnt_ = 0;                                       // Reset sample count
                }
                break;
                
            case Res_Measure1:
                // Measure point 1: average voltage and current
                voltage_mv_ = voltage_fixed_mv_;                         // Hold voltage
                accum_v_ += voltage_mv_;                                 // Accumulate voltage
                accum_i_ += filt;                                        // Accumulate current
                if (++samp_cnt_ >= RES_MEASURE_SAMPLES) {                // All samples taken
                    res_v1_ = (int32_t)(accum_v_ / RES_MEASURE_SAMPLES); // Average V (mV)
                    res_i1_ = fb_.current.norm2ma((int32_t)(accum_i_ / RES_MEASURE_SAMPLES)); // Average I (mA)
                    SerialUSB.print("R:P1 V=");                          // Log point 1
                    SerialUSB.print(res_v1_);
                    SerialUSB.print("mV I=");
                    SerialUSB.print(res_i1_);
                    SerialUSB.println("mA");
                    voltage_fixed_mv_ = res_v1_ >> 1;                    // Point 2: V1/2
                    state_ = Res_Ramp2;                                  // Go to ramp 2
                    cnt_ = 0;                                            // Reset counter
                }
                break;
                
            case Res_Ramp2:
                // Apply V1/2 and wait for current to stabilize
                voltage_mv_ = voltage_fixed_mv_;                         // Apply V1/2
                if (++cnt_ >= RES_STABLE_COUNT) {                        // Wait for stability
                    state_ = Res_Settle2;                                // Go to settle
                    cnt_ = 0;                                            // Reset counter
                }
                break;
                
            case Res_Settle2:
                // Wait for transients at V1/2
                voltage_mv_ = voltage_fixed_mv_;                         // Hold V1/2
                if (++cnt_ >= RES_SETTLE_CYCLES) {                       // Settle complete
                    state_ = Res_Measure2;                               // Go to measure
                    accum_v_ = 0;                                        // Reset accumulators
                    accum_i_ = 0;
                    samp_cnt_ = 0;                                       // Reset sample count
                }
                break;
                
            case Res_Measure2:
                // Measure point 2 and calculate resistance
                voltage_mv_ = voltage_fixed_mv_;                         // Hold V1/2
                accum_v_ += voltage_mv_;                                 // Accumulate voltage
                accum_i_ += filt;                                        // Accumulate current
                if (++samp_cnt_ >= RES_MEASURE_SAMPLES) {                // All samples taken
                    int32_t v2 = (int32_t)(accum_v_ / RES_MEASURE_SAMPLES); // Average V2 (mV)
                    int32_t i2 = fb_.current.norm2ma((int32_t)(accum_i_ / RES_MEASURE_SAMPLES)); // Average I2 (mA)
                    SerialUSB.print("R:P2 V=");                          // Log point 2
                    SerialUSB.print(v2);
                    SerialUSB.print("mV I=");
                    SerialUSB.print(i2);
                    SerialUSB.println("mA");
                    
                    // Calculate R = dV / dI using two-point method
                    int32_t dv = res_v1_ - v2;                           // Voltage difference
                    int32_t di = res_i1_ - i2;                           // Current difference
                    if (dv < 0) dv = -dv;                                // Absolute dV
                    if (di < 0) di = -di;                                // Absolute dI
                    
                    if (di < 10) {                                       // Current too low
                        resistance_mohm_ = INT32_MAX;                    // Mark as open
                        SerialUSB.println("R:Open");                     // Log open circuit
                    } else {
                        resistance_mohm_ = (dv * 1000) / di;             // R = dV*1000/dI (mOhm)
                        SerialUSB.print("R=");                           // Log result
                        SerialUSB.print(resistance_mohm_);
                        SerialUSB.println("mOhm");
                    }
                    lpf_.reset(0);                                       // Reset filter
                    state_ = Res_Finished;                               // Done
                }
                break;
                
            default:
                return 0;                                                // Safety default
        }
        
        return fb_.supply.mv2duty(voltage_mv_);                          // Convert mV to duty
    }

    bool is_finished() const { return state_ >= Res_Finished; }          // Check if done
    bool is_error() const { return state_ == Res_Error; }                // Check for error
    bool is_running() const { return state_ > Res_Idle && state_ < Res_Finished; } // Check if running
    int32_t get_resistance_mohm() const { return resistance_mohm_; }     // Get result
    Res_State get_state() const { return (Res_State)state_; }            // Get state
};
