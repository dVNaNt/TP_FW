#pragma once

// Unified migrated control path (3-phase PWM only). Old variants removed.
// Integrated with TunePulseHeart for complete motor control system.

#include "common.h"
#include "feedback/feedback.h"
#include "foc_vector_synthesis.h"  // Position-velocity cascaded controller
#include "math/filters/ema_encoder.h"
#include "math/helpers/frequency_generator.h"
#include "math/motion/motion_executor.h"  // Real-time motion trajectory execution
#include "math/motion/motion_planer.h"    // Motion planning and task management
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
#define PWM_TIMER_PERIOD 170000000 / 40000
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
    AngleCalibrator angle_calibrator;
    // FilterAngleHybridEMA filter_;

    // ========================================================================
    // Motion planning and position control
    // ========================================================================
    MotionExecutor<32> motion_executor;  // Real-time trajectory executor (16 segment buffer)
    MotionPlanner<32> motion_planner;    // High-level motion task manager
    // PositionController position_controller = PositionController(0.01f, 0.0f, 0.0f, 0.1f, 0.00005f, 5000000);  //
    // Cascaded position-velocity controller
    MotorVectorSynthesis position_controller = MotorVectorSynthesis(
    /*wc_max*/         200.0f,   // рад/с
    /*wc_min*/          30.0f,   // рад/с  
    /*wo*/             1000.0f,   // рад/с (наблюдатель)
    /*b0*/               5.0f,  // подобрать под мотор
    /*dt*/               0.00005f, // 20kHz
    /*max_vel*/          16.6f,   // об/с
    /*min_curr_ma*/    100.0f,   // мА
    /*speed_foc*/        3.0f,   // об/с — порог FOC
    /*load_foc_ma*/    300.0f,   // мА — порог нагрузки
    /*pole_pairs*/       50.0f,    
    /*blend_rate*/       5.0f,   // Гц
    /*hysteresis*/       0.15f   // 15%
);

    // ========================================================================
    // Control state
    // ========================================================================
    MotorStatus motor_status;
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
    bool use_anti_cogging = true;       // <-- ИНТЕГРАЦИЯ: Флаг для включения/выключения анти-коггинга
    bool use_position_control = false;  // Enable position control mode (disables manual speed control)

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
        : clock_(170000000, freq_hz),
          feedback(max_sup_mv, iA_fs, iB_fs, iC_fs),  // Initialize unified feedback with all modules enabled
          motor_params(),                             // Initialize Motor with default constructor
          driver(feedback, motor_params),             // Pass feedback and motor reference
          angle_calibrator(freq_hz),

          motion_executor(freq_hz),          // Initialize motion executor with control frequency
          motion_planner(&motion_executor),  // Initialize motion planner with executor reference
          motor_status(MotorStatus::NotCalibrated),
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
        // filter_.configure_zones(8, 50, 2, 1);
        // filter_.configure_velocity_scaler(0.00005f, pole_pairs, 1.0f);
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

    inline void on_encoder_raw(uint16_t angle_raw) {
        feedback.inputs.set_angle_raw(angle_raw);  // Use Feedback method without underscore
    }

    inline void on_adc_values(const uint16_t currents[3], uint16_t supply_adc, uint16_t temp_adc) {
        feedback.inputs.set_current_adc(currents);   // Use Feedback method without underscore
        feedback.inputs.set_supply_adc(supply_adc);  // Use Feedback method without underscore
        feedback.inputs.set_temper_adc(temp_adc);    // Use Feedback method without underscore
    }
    bool mode = false;
    bool previous_mode = false;
    float bandwith = 500.0;
    bool is_pos_linear_used = false;
    int16_t d_current = 0;
    int16_t invert_ladrc = -1;
    float bw_ladrc_min = 50.0f;
    float bw_ladrc_max = 250.0f;
    int32_t POS_ERR = 0;
    bool is_pos_ctrl_used = true;
    int16_t pole_pairs = 50;
    bool is_filt_used = false;
    bool is_synt_used = true;
    bool khz10 = true;
    void tick() {
        bool feedback_is_updated = feedback.tick();
        if (fault_handler())
            return;

        if (!feedback_is_updated) {
            return;
        }

        // ========================================================================
        // Motor controller tick logic
        // ========================================================================
        

        switch (motor_status) {
            case MotorStatus::Success: {
                int32_t real_raw_pos = feedback.encoder.get_position();

                if (!angle_calibrator.is_ready()) {
                    feedback.position.tick(real_raw_pos);
                    angle_el_ = angle_calibrator.tick(real_raw_pos, (int16_t)driver.motor.get_motor_type());
                    if (angle_calibrator.is_ready()) {
                        TP_LOG("Angle Calibration Done. Starting Normal/Cogging operation");
                        current_target_ma_ = 800;
                        amplitude_ = feedback.current.ma2norm(current_target_ma_);
                    }
                    break;
                }

                uint16_t corrected_angle =
                    is_pos_linear_used ? angle_calibrator.get_position_correction(real_raw_pos) : real_raw_pos;

                feedback.position.tick(corrected_angle);

                // Вычисляем электрический угол один раз, так как он нужен в обеих ветках ниже
                angle_el_ = (uint16_t)((int16_t)(angle_calibrator.get_el_angle_correction(real_raw_pos)) * el_sign);
                // Motion planning and position control integration

                // Execute motion trajectory and get current state (position, velocity, acceleration)
                MotionState motion_state = motion_executor.tick();

                // Get current position and velocity from feedback in Q16.16 format
                int32_t current_pos_q16 = feedback.position.get_filtered_position();
                float current_vel = feedback.position.get_filtered_speed_float();
                float current_acc = feedback.position.get_acceleration_float();

                // if (khz10) {
                    // Position controller outputs current command based on trajectory tracking error
                    FOCCommand position_current_ma =
                        position_controller.tick(motion_state.pos,                      // Target position from motion planner
                                                 feedback.position.get_raw_position(),  // Current position from encoder
                                                 motion_state.vel,
                                                 current_vel,  // Current velocity from estimator
                                                 motion_state.acc, 0,
                                                 current_target_ma_,  // Maximum current limit
                                                 angle_el_);

                    POS_ERR = feedback.position.get_raw_position() - motion_state.pos;
                    if (previous_mode != mode) {
                        previous_mode = mode; 
                        // position_controller.set_mode(mode);
                    }
                    position_controller.set_bandwidth(bw_ladrc_max, bw_ladrc_min);

                    // Use position controller output as amplitude command
                    if (is_pos_ctrl_used) {
                        amplitude_ = feedback.current.ma2norm(position_current_ma.u_q) * invert_ladrc;
                        d_current = feedback.current.ma2norm(position_current_ma.u_d) * invert_ladrc;
                    } else {
                        amplitude_ = feedback.current.ma2norm(current_target_ma_);
                    }
                // }
                // khz10 = !khz10;
                // if (is_filt_used)
                //     angle_el_ = filter_.tick(angle_el_, current_vel);
                if (is_synt_used)
                    angle_el_ = position_current_ma.elec_angle;
                break;
            }
            case MotorStatus::CalibError:
                amplitude_ = 0;
                break;
            case MotorStatus::NotCalibrated:
                if (driver.motor.get_status() == MotorStatus::Success) {
                    TP_LOG("Going to MotorStatus::ElectricCalibrationDone from MotorStatus::NotCalibrated");
                    motor_status = MotorStatus::Success;
                    current_target_ma_ = 300;
                    amplitude_ = feedback.current.ma2norm(current_target_ma_);
                }
                break;
            default: {
                pwm_norm_[0] = pwm_norm_[1] = pwm_norm_[2] = INT16_MIN;
                update_pwm_duty();
                return;
            }
        }
        driver.tick_control_foc((int16_t)angle_el_, Vec2D(d_current, amplitude_), feedback);
        pwm_norm_ = driver.get_output();
        update_pwm_duty();
    }

    const int16_t* pwm_duty_values() const { return pwm_duty_; }

    // ========================================================================
    // Clock interface
    // ========================================================================

    const Frequency& get_clock() const { return clock_; }

    Frequency& get_clock() { return clock_; }
    int32_t is_cal_used = 1;
    int32_t el_sign = -1;

private:
    inline bool fault_handler() {
        // ========================================================================
        // Protection recovery timer logic
        // ========================================================================
        bool prev_has_hard_fault = has_hard_fault;
        has_hard_fault = feedback.error.has_fatal();

        // 1. Обработка активной ошибки (сгруппировано для исключения дублирования)
        if (has_hard_fault) {
            bool is_new = !prev_has_hard_fault;

            // Локальная inline-лямбда для устранения копипасты управления ШИМ и LED
            auto halt_drive = [&](int16_t pwm_val, bool r, bool g, bool b) {
                led_control(0, r);
                led_control(1, g);
                led_control(2, b);
                pwm_norm_[0] = pwm_norm_[1] = pwm_norm_[2] = pwm_val;
                update_pwm_duty();
            };

            if (feedback.error.get_fatal_dc_ovlo()) {
                if (is_new)
                    TP_LOG("COMPARATOR TRIGGERED: Overvoltage protection (COMP3)");
                halt_drive(0, false, false, true);
                return true;
            }
            if (feedback.error.get_fatal_oc()) {
                if (is_new)
                    TP_LOG("COMPARATOR TRIGGERED: Overcurrent protection (COMP1/2/4)");
                else
                    feedback.error.reset_fatal_oc();
                halt_drive(INT16_MIN, false, true, false);
                return true;
            }
            if (feedback.error.get_fatal_driver_error()) {
                if (is_new)
                    TP_LOG("COMPARATOR TRIGGERED: Driver fault");
                else
                    feedback.error.reset_fatal_driver();
                halt_drive(INT16_MIN, true, false, false);
                return true;
            }
        }
        // 2. Ошибки только что исчезли — запускаем таймер
        else if (prev_has_hard_fault) {
            protection_recovery_timestamp_ = millis();
            TP_LOG("Protection recovery timer started (200ms)");
            return true;
        }
        // 3. Быстрый путь нормальной работы
        else {
            // Если таймер не истек, выходим без лишних вычислений
            if (millis() - protection_recovery_timestamp_ < PROTECTION_RECOVERY_TIMEOUT_MS) {
                return true;
            }
            // Таймер истек — гасим диоды
            led_control(0, false);
            led_control(1, false);
            led_control(2, false);
        }
        return false;
    }
    // ========================================================================
    // Private methods
    // ========================================================================

    void reset_buffers() {
        for (size_t i = 0; i < 3; ++i) {
            pwm_norm_[i] = 0;
            pwm_duty_[i] = 0;
        }
    }

    void update_pwm_duty() {
        // -period = hiz, 0 = brake, +period = 100%
        const int32_t period = pwm_resolution_;

        for (size_t i = 0; i < 3; ++i) {
            int32_t val = pwm_norm_[i];

            // Арифметический сдвиг: (val * period) / 32768
            // Если val == 32767, сдвиг дает (period - 1). Логическое сравнение добавляет недостающую 1.
            pwm_duty_[i] = (int16_t)(((val * period) >> 15) + (val == INT16_MAX));
        }
    }
};
