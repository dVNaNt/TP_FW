#pragma once

// Unified migrated control path (3-phase PWM only). Old variants removed.
// Integrated with TunePulseHeart for complete motor control system.

#include "common.h"
#include "feedback/feedback.h"
#include "math_integer/helpers/frequency_generator.h"
#include "motor_driver/calibration/angle_calibrator.h"
#include "motor_driver/driver_pwm/driver_pwm.h"
#include "motor_driver/driver_pwm/hfi_sense.h"
#include "motor_driver/motor.h"
#include "motor_driver/types.h"

// Forward declarations for comparator DAC callbacks
extern void DAC_Set_COMP1_OC_Threshold(uint16_t threshold);  // Phase B overcurrent
extern void DAC_Set_COMP2_OC_Threshold(uint16_t threshold);  // Phase A overcurrent
extern void DAC_Set_COMP3_OV_Threshold(uint16_t threshold);  // Overvoltage
extern void DAC_Set_COMP4_OC_Threshold(uint16_t threshold);  // Phase C overcurrent

// Forward declaration
struct TunePulseHeart;

// LED control function pointer type (platform-independent)
typedef void (*LED_Control_Callback)(bool state);

#ifndef PWM_TIMER_FREQ_HZ
#define PWM_TIMER_FREQ_HZ 20000
#endif

#ifndef PWM_TIMER_PERIOD
#define PWM_TIMER_PERIOD 4250
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
    // Motor parameters (owned here, passed by reference to driver)
    // ========================================================================
    Motor motor_params;

    // ========================================================================
    // Motor control components
    // ========================================================================
    DriverPWM driver;
    AngleCalibrator angle_calibrator_;
    FilterEMA filter_;

    // ========================================================================
    // HFI sensorless control (activated after motor calibration)
    // ========================================================================
    // Note: HFI components moved to DriverPWM for better encapsulation
    // Use driver.init_hfi_sdft() to enable HFI sensorless control

    // ========================================================================
    // Control state
    // ========================================================================
    MotorStatus driverstatus_;
    uint16_t angle_el_;
    int16_t amplitude_;
    int16_t direction_;
    int16_t speed_;
    int32_t ticker_ = 0;
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
    // LED control callbacks (platform-independent)
    // ========================================================================
    LED_Control_Callback led_callbacks_[3] = {nullptr, nullptr,
                                              nullptr};  // Array of LED control callbacks: [0]=Red, [1]=Green, [2]=Blue

    // ========================================================================
    // Protection recovery timer (200ms timeout after ALL hard faults cleared)
    // ========================================================================
    uint32_t protection_recovery_timestamp_;                          // Timestamp when ALL hard faults cleared (millis)
    bool protection_recovery_active_;                                 // Recovery timer active flag
    static constexpr uint32_t PROTECTION_RECOVERY_TIMEOUT_MS = 200U;  // 200ms recovery timeout

    // ========================================================================
    // Internal state
    // ========================================================================
    uint32_t calibration_cycles_;
    uint32_t last_usb_ms_;
    int16_t last_supply_norm_;
    bool has_hard_fault = false;

public:
    // Constructor
    TunePulseHeart(uint16_t freq_hz, uint16_t pwm_res, int32_t max_sup_mv, int32_t iA_fs, int32_t iB_fs, int32_t iC_fs)
        : i_ch2(0),
          clock_(170000000, freq_hz),
          feedback(max_sup_mv, iA_fs, iB_fs, iC_fs)  // Initialize unified feedback with all modules enabled
          ,
          motor_params()  // Initialize Motor with default constructor
          ,
          driver(feedback, motor_params)  // Pass feedback and motor reference
          ,
          angle_calibrator_(angle_calibrator_new(freq_hz)),
          filter_(0, 0),
          driverstatus_(MotorStatus::NotCalibrated),
          angle_el_(0),
          amplitude_(0),
          direction_(0),
          speed_(10),
          sup_check_(100),
          speed_incr_(0),
          pwm_resolution_(pwm_res),
          current_target_ma_(1000),
          protection_recovery_timestamp_(0),
          protection_recovery_active_(false),
          calibration_cycles_(0),
          last_usb_ms_(0),
          last_supply_norm_(0) {
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

    // DAC callback setters for hardware protection thresholds
    // These methods allow external code to register DAC update callbacks
    // that are called when protection thresholds change

    /// Set overcurrent hard protection callback for specific phase
    /// @param channel Current sense channel (CH1, CH2, or CH3)
    /// @param callback Function pointer to DAC update callback (receives threshold in ADC units)
    void set_overcurrent_hard_callback(Channel channel, void (*callback)(uint16_t)) {
        feedback.current.set_overcurrent_hard_callback(channel, callback);
    }

    /// Set overvoltage hard protection callback
    /// @param callback Function pointer to DAC update callback (receives threshold in ADC units)
    void set_overvoltage_hard_callback(void (*callback)(uint16_t)) { feedback.supply.set_overvoltage_hard_callback(callback); }

    /// Set overcurrent hard protection limit in milliamps for specific phase
    /// This will trigger the registered DAC callback to update comparator threshold
    /// @param channel Current sense channel (CH1, CH2, or CH3)
    /// @param limit_ma Overcurrent limit in milliamps
    void set_overcurrent_hard_ma(Channel channel, int32_t limit_ma) {
        feedback.current.set_overcurrent_hard_ma(channel, limit_ma);
    }

    /// Set overvoltage hard protection limit in millivolts
    /// This will trigger the registered DAC callback to update comparator threshold
    /// @param limit_mv Overvoltage limit in millivolts
    void set_overvoltage_hard_mv(int32_t limit_mv) { feedback.supply.set_overvoltage_hard_mv(limit_mv); }

    // ========================================================================
    // LED control methods (platform-independent)
    // ========================================================================

    /// Set LED control callback for specific LED channel
    /// @param led_index LED index (0=Red, 1=Green, 2=Blue)
    /// @param callback Function pointer to LED control callback (nullptr to clear)
    void set_led_callback(uint8_t led_index, LED_Control_Callback callback) {
        if (led_index < 3) {                       // Validate index range
            led_callbacks_[led_index] = callback;  // Store callback pointer
        }
    }

    /// Control LED state (calls registered callback if available)
    /// @param led_index LED index (0=Red, 1=Green, 2=Blue)
    /// @param state LED state (true=ON, false=OFF)
    void led_control(uint8_t led_index, bool state) {
        if (led_index < 3 && led_callbacks_[led_index] != nullptr) {  // Check index and callback validity
            led_callbacks_[led_index](state);                         // Call LED control callback
        }
    }

    /// Get pointer to LED callbacks array for direct access
    /// @return Pointer to LED callbacks array [R, G, B]
    LED_Control_Callback* get_led_callbacks() {
        return led_callbacks_;  // Return pointer to callbacks array
    }

    void set_current_target_ma(int32_t value_ma) { current_target_ma_ = clamp_i32(value_ma, INT16_MAX); }

    void on_encoder_raw(uint16_t angle_raw) {
        feedback.inputs.set_angle_raw(angle_raw);  // Use Feedback method without underscore
    }

    void on_adc_values(const uint16_t currents[3], uint16_t supply_adc, uint16_t temp_adc) {
        feedback.inputs.set_current_adc(currents);   // Use Feedback method without underscore
        feedback.inputs.set_supply_adc(supply_adc);  // Use Feedback method without underscore
        feedback.inputs.set_temper_adc(temp_adc);    // Use Feedback method without underscore
    }
    uint32_t ramp_ticker_ = 0;  // новый независимый счётчик для рампы
    int32_t sensor_allign_ticks = 1000;
    int16_t encoder_init_pos = 0;
    int16_t  encoder_step_size = -50;
    int16_t phase_shift = 8196;
    void tick() {
        bool feedback_is_updated = feedback.is_updated();
        if (feedback_is_updated) {
            // Update all feedback sensors (supply, current, position)
            feedback.tick();  // Process all feedback data
        }

        // ========================================================================
        // Protection recovery timer logic (200ms timeout after ALL hard faults cleared)
        // ========================================================================
        // Check if ALL hard faults are cleared (comparator outputs went low)
        bool prev_has_hard_fault = has_hard_fault;
        has_hard_fault = feedback.error.has_fatal();  // Check if any fatal error is active

        // Detect new fault (transition from no fault to fault)
        if (has_hard_fault && !prev_has_hard_fault) {
            // New fault detected - log to serial
            if (feedback.error.get_fatal_dc_ovlo()) {
                TP_LOG("COMPARATOR TRIGGERED: Overvoltage protection (COMP3)");
                // Overvoltage error - turn on red LED
                led_control(0, true);   // Red LED ON for overvoltage
                led_control(1, false);  // Green LED ON for overcurrent
                pwm_norm_[0] = 0;
                pwm_norm_[1] = 0;
                pwm_norm_[2] = 0;
                update_pwm_duty();
                return;  // Skip normal control when fatal error is active
            }
            if (feedback.error.get_fatal_oc()) {
                TP_LOG("COMPARATOR TRIGGERED: Overcurrent protection (COMP1/2/4)");
                // Overcurrent error - turn on green LED
                led_control(0, false);  // Red LED ON for overvoltage
                led_control(1, true);   // Green LED ON for overcurrent
                pwm_norm_[0] = INT16_MIN;
                pwm_norm_[1] = INT16_MIN;
                pwm_norm_[2] = INT16_MIN;
                update_pwm_duty();
                return;  // Skip normal control when fatal error is active
            }
        } else if (has_hard_fault) {
            // New fault detected - log to serial
            if (feedback.error.get_fatal_dc_ovlo()) {
                // Overvoltage error - turn on red LED
                led_control(0, true);   // Red LED ON for overvoltage
                led_control(1, false);  // Green LED ON for overcurrent
                pwm_norm_[0] = 0;
                pwm_norm_[1] = 0;
                pwm_norm_[2] = 0;
                update_pwm_duty();
                return;  // Skip normal control when fatal error is active
            }
            if (feedback.error.get_fatal_oc()) {
                feedback.error.reset_fatal_oc();
                // Overcurrent error - turn on green LED
                led_control(0, false);  // Red LED ON for overvoltage
                led_control(1, true);   // Green LED ON for overcurrent
                pwm_norm_[0] = INT16_MIN;
                pwm_norm_[1] = INT16_MIN;
                pwm_norm_[2] = INT16_MIN;
                update_pwm_duty();
                return;  // Skip normal control when fatal error is active
            }
        } else if (!has_hard_fault && prev_has_hard_fault) {
            // ALL hard faults just cleared - start recovery timer
            protection_recovery_timestamp_ = millis();  // Record timestamp
            TP_LOG("Protection recovery timer started (200ms)");
            return;

        } else if (!has_hard_fault) {
            // Recovery timer is active - check timeout
            uint32_t elapsed = millis() - protection_recovery_timestamp_;  // Calculate elapsed time
            if (elapsed >= PROTECTION_RECOVERY_TIMEOUT_MS) {
                // Timeout expired - clear all fatal errors permanently
                // TUNEPULSE_LOG_INFO("Protection recovery complete - system restored");

                // Turn off all LEDs after recovery timeout (platform-independent)
                led_control(0, false);  // Turn off red LED
                led_control(1, false);  // Turn off green LED
                led_control(2, false);  // Turn off blue LED
            } else {
                return;
            }
        }

        if (!feedback_is_updated) {
            return;
        }

        // ========================================================================
        // LED indication for protection errors (platform-independent)
        // ========================================================================

        // Motor controller tick logic
        amplitude_ = (int16_t)current_target_ma_;  // ma

        switch (driverstatus_) {
            case MotorStatus::Success: {
                int16_t encoder_relative_pos = feedback.position.get_angle() - encoder_init_pos;
                int32_t encoder_el_pos = encoder_relative_pos * encoder_step_size;  // Calculate electrical position from encoder
                angle_el_ = (int16_t)encoder_el_pos + phase_shift;
                break;
            }
            case MotorStatus::CalibError:
                amplitude_ = 0;
                break;
            case MotorStatus::NotCalibrated:
                if (driver.motor.get_status() == MotorStatus::Success) {
                    amplitude_ = 2000;
                    ticker_++;
                    if (ticker_ == sensor_allign_ticks) {
                        encoder_init_pos = feedback.position.get_angle();
                        if (driver.motor.get_motor_type() == MotorType_STEP)  angle_el_ += 16384;
                        else if (driver.motor.get_motor_type() == MotorType_BLDC) angle_el_ += 24576;
                    } else if (ticker_ == sensor_allign_ticks * 2) {
                        // encoder_step_size == encoder_init_pos - feedback.position.get_angle();
                        driverstatus_ = MotorStatus::Success;
                    }
                }
                break;

            default: {
                pwm_norm_[0] = INT16_MIN;
                pwm_norm_[1] = INT16_MIN;
                pwm_norm_[2] = INT16_MIN;
                update_pwm_duty();
                return;  // Skip normal control when fatal error is active
            }
        }


        // Call FOC with HFI parameters for sensorless position estimation
        driver.tick_control_foc((int16_t)angle_el_, Vec2D(0, amplitude_), feedback);

        pwm_norm_ = driver.get_output();

        update_pwm_duty();
    }

    const int16_t* pwm_duty_values() const { return pwm_duty_; }

    // ========================================================================
    // Clock interface
    // ========================================================================

    const Frequency& get_clock() const { return clock_; }

    Frequency& get_clock() { return clock_; }


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

            if (value == INT16_MAX) {
                pwm_duty_[i] = INT16_MAX;
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
