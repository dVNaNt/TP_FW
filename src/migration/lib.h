#pragma once

// Unified migrated control path (3-phase PWM only). Old variants removed.
// Integrated with TunePulseHeart for complete motor control system.

#include "common.h"
#include "error.h"              // Include error management system
#include "feedback/feedback.h"  // Include unified feedback module
#include "feedback/feedback.h"
#include "math_integer/filters/filter1st.h"
#include "motor_driver/calibration/angle_calibrator.h"
#include "motor_driver/driver_pwm/driver_pwm.h"
#include "motor_driver/motor.h"
#include "motor_driver/types.h"

// Forward declaration
struct TunePulseHeart;

#ifndef PWM_TIMER_FREQ_HZ
#define PWM_TIMER_FREQ_HZ 20000
#endif

#ifndef PWM_TIMER_PERIOD
#define PWM_TIMER_PERIOD 4722
#endif

// ============================================================================
// TunePulseHeart - Complete motor control system
// ============================================================================
// Core control object that glues migrated motor-control logic with
// hardware-facing callbacks. Interrupts push fresh ADC/encoder data into
// the internal InputsDump, while the tick() method (called from loop)
// performs heavy math only when a complete, ready dataset is available.

struct TunePulseHeart {
public:
    // Debug/telemetry: raw ADC value of current channel 2 (ADC2_IN4 -> PA7).
    // Marked volatile so it is always written (useful for MCUView / debugger watches).
    volatile uint16_t i_ch2;

    // ========================================================================
    // Clock and timing
    // ========================================================================
    Frequency clock_;

    // ========================================================================
    // Unified feedback system (inputs, supply, current, position)
    // ========================================================================
    Feedback feedback;

    // ========================================================================
    // Error management system (public member)
    // ========================================================================
    ErrorManager error;

public:
    // ========================================================================
    // Motor parameters (owned here, passed by reference to driver)
    // ========================================================================
    Motor motor_params;

    // ========================================================================
    // Motor control components
    // ========================================================================
    DriverPWM driver_;
    AngleCalibrator angle_calibrator_;
    Filter1st filter_;

    // ========================================================================
    // Control state
    // ========================================================================
    DriverStatus driver_status_;
    uint16_t angle_el_;
    int16_t amplitude_;
    int16_t direction_;
    int16_t speed_;
    int32_t ticker_;
    size_t sup_check_;
    int32_t speed_incr_;

    // ========================================================================
    // PWM output
    // ========================================================================
    Vec3D pwm_norm_;
    int16_t pwm_duty_[3];
    uint16_t pwm_resolution_;

    // ========================================================================
    // User settings
    // ========================================================================
    int32_t current_target_ma_;

    // ========================================================================
    // Internal state
    // ========================================================================
    uint32_t calibration_cycles_;
    uint32_t last_usb_ms_;
    int16_t last_supply_norm_;

public:
    // Constructor
    TunePulseHeart(uint16_t freq_hz,
                   uint16_t pwm_res,
                   int32_t max_sup_mv, int32_t iA_fs, int32_t iB_fs, int32_t iC_fs )
        : i_ch2(0)
        , clock_(170000000, freq_hz)
        , feedback(max_sup_mv, iA_fs, iB_fs, iC_fs)  // Initialize unified feedback with all modules enabled
        , motor_params()  // Initialize Motor with default constructor
        , driver_(feedback, motor_params)  // Pass feedback and motor reference
        , angle_calibrator_(angle_calibrator_new(freq_hz))
        , filter_(0, 0)
        , driver_status_(DriverStatus_Ready)
        , angle_el_(0)
        , amplitude_(0)
        , direction_(0)
        , speed_(0)
        , ticker_(0)
        , sup_check_(100)
        , speed_incr_(10)
        , pwm_resolution_(pwm_res)
        , current_target_ma_(300)
        , calibration_cycles_(0)
        , last_usb_ms_(0)
        , last_supply_norm_(0)
    {
        // Set motor type and connection using setters
        // Set phase resistances for all phases (same value for initial config)

        feedback.enable_current();
        feedback.enable_encoder();
        feedback.enable_supply();
        
        reset_buffers();
    }
    // ========================================================================
    // Public API - Hardware callbacks
    // ========================================================================

    void set_current_target_ma(int32_t value_ma) { current_target_ma_ = clamp_i32(value_ma, INT16_MAX); }

    void on_encoder_raw(uint16_t angle_raw) {
        feedback.inputs.set_angle_raw(angle_raw);  // Use Feedback method without underscore
    }

    void on_adc_values(const uint16_t currents[3], uint16_t supply_adc, uint16_t temp_adc) {
        feedback.inputs.set_current_adc(currents);   // Use Feedback method without underscore
        feedback.inputs.set_supply_adc(supply_adc);  // Use Feedback method without underscore
        feedback.inputs.set_temper_adc(temp_adc);    // Use Feedback method without underscore
    }

    void tick() {
        // Update error manager (auto-clear expired errors)
        error.tick();

        // Update all feedback sensors (supply, current, position)
        feedback.tick();  // Process all feedback data

        if (!feedback.is_updated()) {
            return;
        } else {
        }

        // Motor controller tick logic
        amplitude_ = (int16_t)current_target_ma_;  // ma

        switch (driver_status_) {
            case DriverStatus_Ready: {
                ticker_ = (int32_t)(ticker_ + speed_incr_);
                angle_el_ = (uint16_t)(ticker_ / 10);
                break;
            }
            case DriverStatus_Error:
                amplitude_ = 0;
                break;
            case DriverStatus_Calibrating:
            default: {
                if (sup_check_ > 0) {
                    sup_check_ -= 1;
                    if (sup_check_ == 0) {
                        if (feedback.supply.get_voltage_mv() < 8000) {  // Use Feedback accessor without underscore
                            TUNEPULSE_LOG_WARN("SUPPLY is not enough: %dmV while at least 8000mV is needed",
                                               feedback.supply.get_voltage_mv());
                        } else {
                            TUNEPULSE_LOG_INFO("SUPPLY is OK: %dmV", feedback.supply.get_voltage_mv());
                        }
                    }
                }

                angle_el_ = feedback.position.get_value();  // Use Feedback accessor without underscore
                break;
            }
        }
        angle_el_ = 0;

        // If fatal error is active, disable power stage (set all channels to INT16_MIN)
        if (error.has_fatal()) {
            pwm_norm_[0] = INT16_MIN;
            pwm_norm_[1] = INT16_MIN;
            pwm_norm_[2] = INT16_MIN;
            update_pwm_duty();
            return;  // Skip normal control when fatal error is active
        }

        // Use FOC control with phase currents
        driver_.tick_control_foc((int16_t)angle_el_, amplitude_, feedback);  // Pass feedback reference instead of separate data
        pwm_norm_ = driver_.get_output();
        update_pwm_duty();
    }

    const int16_t* pwm_duty_values() const { return pwm_duty_; }

    // ========================================================================
    // Clock interface
    // ========================================================================

    const Frequency& get_clock() const { return clock_; }

    Frequency& get_clock() { return clock_; }

    // ========================================================================
    // Motor test interface
    // ========================================================================

    DriverStatus get_driver_status() const { return driver_.get_status(); }

    // ========================================================================
    // Motor control interface
    // ========================================================================

private:
    // ========================================================================
    // Private methods
    // ========================================================================

    void reset_buffers() {
        i_ch2 = 0;
        for (size_t i = 0; i < 3; ++i) {
            pwm_norm_[i] = 0;
            pwm_duty_[i] = 0;
        }
    }

    void update_pwm_duty() {
        const int32_t period = pwm_resolution_;
        for (size_t i = 0; i < 3; ++i) {
            int32_t value = pwm_norm_[i];
            if (value < 0) {
                pwm_duty_[i] = INT16_MIN;
                continue;
            }
            // Same scaling as Rust duty2period: (duty * period) >> 15
            int32_t duty = (value * period) >> 15;
            if (duty < 0)
                duty = 0;
            if (duty > period)
                duty = period;
            pwm_duty_[i] = (int16_t)duty;
        }
    }
};
