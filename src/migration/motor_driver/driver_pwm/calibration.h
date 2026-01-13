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
#include "frequency_generator.h"

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
constexpr uint16_t RES_MEASURE_SAMPLES   = 256;     // Samples for averaging
constexpr uint16_t RES_STABLE_COUNT      = 180;     // 10ms stability requirement
constexpr uint16_t RES_RAMP_MAX_CYCLES   = 3600;    // 200ms max ramp time
constexpr int16_t  RES_CURRENT_EPSILON   = 100;     // Current tolerance (Q15)

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
    Feedback& fb_;
    PID_Int pid_;
    
    int32_t voltage_mv_;             // Current output voltage (mV)
    int32_t voltage_fixed_mv_;       // Fixed voltage for point 2
    int32_t accum_v_;                // Voltage accumulator
    int32_t accum_i_;                // Current accumulator
    int32_t res_v1_;                 // Point 1 voltage (mV)
    int32_t res_i1_;                 // Point 1 current (mA)
    int32_t resistance_mohm_;        // Result: resistance (mOhm)
    
    int16_t target_current_;         // Target current (Q15)
    int16_t target_current_1_;       // Point 1: 3/4 of target
    int16_t target_current_2_;       // Point 2: 3/8 of target
    
    uint16_t cnt_;                   // General counter
    uint16_t stable_cnt_;            // Stability counter
    uint16_t samp_cnt_;              // Sample counter
    uint8_t state_;

public:
    ResistanceMeasure(Feedback& fb)
        : fb_(fb)
        , pid_(pid_int_new(100, 10, 0, 0))  // Default PI gains
        , voltage_mv_(0), voltage_fixed_mv_(0)
        , accum_v_(0), accum_i_(0)
        , res_v1_(0), res_i1_(0), resistance_mohm_(0)
        , target_current_(0), target_current_1_(0), target_current_2_(0)
        , cnt_(0), stable_cnt_(0), samp_cnt_(0)
        , state_(Res_Idle)
    {}

    /// Start resistance measurement
    /// @param target_current Target current in Q15 format
    /// @param inductance_uh Measured inductance for PI tuning
    void start(int16_t target_current, int32_t inductance_uh) {
        target_current_ = target_current;
        target_current_1_ = (target_current * 3) >> 2;                   // 3/4 of target
        target_current_2_ = (target_current * 3) >> 3;                   // 3/8 of target
        
        // Calculate PI gains from inductance
        // Kp = L_uH / 2 (scaled for PID_Int: percentage * 100)
        int32_t kp = inductance_uh >> 1;                                 // L/2
        if (kp < 10) kp = 10;                                            // Min Kp
        if (kp > 1000) kp = 1000;                                        // Max Kp
        
        // Ki = R_min * 1000 / L_uH (scaled)
        int32_t ki = (inductance_uh > 0) 
            ? (RES_MIN_MOHM * 1000) / inductance_uh 
            : 10;
        if (ki < 1) ki = 1;                                              // Min Ki
        if (ki > 100) ki = 100;                                          // Max Ki
        
        pid_ = pid_int_new(kp, ki, 0, 0);                                // Create PI controller
        
        SerialUSB.print("PI: Kp=");
        SerialUSB.print(kp);
        SerialUSB.print(" Ki=");
        SerialUSB.println(ki);
        
        // Reset state
        voltage_mv_ = 0;
        voltage_fixed_mv_ = 0;
        accum_v_ = 0;
        accum_i_ = 0;
        res_v1_ = 0;
        res_i1_ = 0;
        resistance_mohm_ = 0;
        cnt_ = 0;
        stable_cnt_ = 0;
        samp_cnt_ = 0;
        state_ = Res_Ramp1;
    }

    /// Main tick function
    /// @param current_d D-axis current (Q15)
    /// @return Voltage duty cycle (Q15)
    int16_t tick(int16_t current_d) {
        if (state_ == Res_Idle || state_ >= Res_Finished) return 0;
        
        int32_t v_limit_mv = fb_.supply.get_voltage_mv() >> 1;           // 50% of Vbus
        
        switch (state_) {
            case Res_Ramp1: {
                // Ramp to 3/4 target using PI
                int16_t error = target_current_1_ - current_d;
                pid_int_tick(&pid_, error, 0, INT16_MAX);
                voltage_mv_ = fb_.supply.norm2mv(pid_int_output(&pid_));
                if (voltage_mv_ > v_limit_mv) voltage_mv_ = v_limit_mv;
                
                // Check stability
                int16_t abs_err = (error >= 0) ? error : -error;
                if (abs_err < RES_CURRENT_EPSILON) {
                    if (++stable_cnt_ >= RES_STABLE_COUNT) {
                        voltage_fixed_mv_ = voltage_mv_;                 // Lock voltage
                        state_ = Res_Settle1;
                        cnt_ = 0;
                    }
                } else {
                    stable_cnt_ = 0;
                }
                // Timeout
                if (++cnt_ >= RES_RAMP_MAX_CYCLES) {
                    voltage_fixed_mv_ = voltage_mv_;
                    state_ = Res_Settle1;
                    cnt_ = 0;
                }
                break;
            }
            
            case Res_Settle1:
                voltage_mv_ = voltage_fixed_mv_;                         // Hold voltage
                if (++cnt_ >= RES_SETTLE_CYCLES) {
                    state_ = Res_Measure1;
                    accum_v_ = 0;
                    accum_i_ = 0;
                    samp_cnt_ = 0;
                }
                break;
                
            case Res_Measure1:
                voltage_mv_ = voltage_fixed_mv_;                         // Hold voltage
                accum_v_ += voltage_mv_;
                accum_i_ += current_d;
                if (++samp_cnt_ >= RES_MEASURE_SAMPLES) {
                    res_v1_ = accum_v_ / RES_MEASURE_SAMPLES;
                    res_i1_ = fb_.current.norm2ma(accum_i_ / RES_MEASURE_SAMPLES);
                    
                    SerialUSB.print("P1: V=");
                    SerialUSB.print(res_v1_);
                    SerialUSB.print("mV I=");
                    SerialUSB.print(res_i1_);
                    SerialUSB.println("mA");
                    
                    // Point 2: use half voltage
                    voltage_fixed_mv_ = res_v1_ >> 1;
                    state_ = Res_Ramp2;
                    cnt_ = 0;
                }
                break;
                
            case Res_Ramp2:
                voltage_mv_ = voltage_fixed_mv_;                         // Fixed voltage
                if (++cnt_ >= RES_STABLE_COUNT) {
                    state_ = Res_Settle2;
                    cnt_ = 0;
                }
                break;
                
            case Res_Settle2:
                voltage_mv_ = voltage_fixed_mv_;
                if (++cnt_ >= RES_SETTLE_CYCLES) {
                    state_ = Res_Measure2;
                    accum_v_ = 0;
                    accum_i_ = 0;
                    samp_cnt_ = 0;
                }
                break;
                
            case Res_Measure2:
                voltage_mv_ = voltage_fixed_mv_;
                accum_v_ += voltage_mv_;
                accum_i_ += current_d;
                if (++samp_cnt_ >= RES_MEASURE_SAMPLES) {
                    int32_t v2 = accum_v_ / RES_MEASURE_SAMPLES;
                    int32_t i2 = fb_.current.norm2ma(accum_i_ / RES_MEASURE_SAMPLES);
                    
                    SerialUSB.print("P2: V=");
                    SerialUSB.print(v2);
                    SerialUSB.print("mV I=");
                    SerialUSB.print(i2);
                    SerialUSB.println("mA");
                    
                    // Calculate R = dV / dI
                    int32_t dv = res_v1_ - v2;
                    int32_t di = res_i1_ - i2;
                    if (dv < 0) dv = -dv;
                    if (di < 0) di = -di;
                    
                    if (di < 10) {
                        resistance_mohm_ = INT32_MAX;                    // Open circuit
                    } else {
                        resistance_mohm_ = (dv * 1000) / di;             // R = dV*1000/dI
                    }
                    
                    SerialUSB.print("R=");
                    SerialUSB.print(resistance_mohm_);
                    SerialUSB.println("mOhm");
                    
                    state_ = Res_Finished;
                }
                break;
                
            default:
                return 0;
        }
        
        return fb_.supply.mv2duty(voltage_mv_);                          // Convert to duty
    }

    bool is_finished() const { return state_ >= Res_Finished; }
    bool is_error() const { return state_ == Res_Error; }
    bool is_running() const { return state_ > Res_Idle && state_ < Res_Finished; }
    int32_t get_resistance_mohm() const { return resistance_mohm_; }
    Res_State get_state() const { return (Res_State)state_; }
};
