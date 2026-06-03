#pragma once

// Unified migrated control path (3-phase PWM only). Old variants removed.
// Integrated with TunePulseHeart for complete motor control system.

#include "common.h"
#include "feedback/feedback.h"
#include "foc_vector_synthesis.h"  // Position-velocity cascaded controller
#include "math/filters/ema_encoder.h"
#include "math/helpers/frequency_generator.h"
#include "math/motion/motion_executor.h"  // Real-time motion trajectory execution
#include "math/motion/motion_governor.h"
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

struct TunePulseHeart {
public:
    // ========================================================================
    // Настройки энкодера для трекинга ошибки в Open-loop
    // ========================================================================
    uint16_t freq_hz_;  // Сохраненная частота контура управления

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

    // // ========================================================================
    // // Motion planning and position control
    // // ========================================================================
    // MotionExecutor<32> motion_executor;  // Real-time trajectory executor (16 segment buffer)
    // MotionPlanner<32> motion_planner;    // High-level motion task manager

    MotorVectorSynthesis position_controller = MotorVectorSynthesis();
    MotionGovernor<128> motion_governor = MotionGovernor<128>(20000);
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
    LED_Control_Callback led_callbacks_[3] = {nullptr, nullptr, nullptr};

    // ========================================================================
    // Protection recovery timer (200ms timeout after ALL hard faults cleared)
    // ========================================================================
    uint32_t protection_recovery_timestamp_;
    bool protection_recovery_active_;
    static constexpr uint32_t PROTECTION_RECOVERY_TIMEOUT_MS = 200U;

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
        : freq_hz_(freq_hz),
          clock_(170000000, freq_hz),
          feedback(max_sup_mv, iA_fs, iB_fs, iC_fs),  // Initialize unified feedback with all modules enabled
          motor_params(),                             // Initialize Motor with default constructor
          driver(feedback, motor_params),             // Pass feedback and motor reference
          angle_calibrator(freq_hz),

          //   motion_executor(freq_hz),          // Initialize motion executor with control frequency
          //   motion_planner(&motion_executor),  // Initialize motion planner with executor reference
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

        // Настройка параметров по умолчанию
        motion_governor.set_time_scale(1024);  // 1.0x скорость
        motion_governor.set_observer_bandwidth_limits(50.0f, 250.0f, 5.0f);
        // filter_.configure_zones(8, 50, 2, 1);
        // filter_.configure_velocity_scaler(0.00005f, pole_pairs, 1.0f);
    }

    void set_overcurrent_hard_callback(Channel channel, void (*callback)(uint16_t)) {
        feedback.current.set_overcurrent_hard_callback(channel, callback);
    }

    void set_overvoltage_hard_callback(void (*callback)(uint16_t)) { feedback.supply.set_overvoltage_hard_callback(callback); }

    void set_overcurrent_hard_ma(Channel channel, int32_t limit_ma) {
        feedback.current.set_overcurrent_hard_ma(channel, limit_ma);
    }

    void set_overvoltage_hard_mv(int32_t limit_mv) { feedback.supply.set_overvoltage_hard_mv(limit_mv); }

    void set_led_callback(uint8_t led_index, LED_Control_Callback callback) {
        if (led_index < 3) {
            led_callbacks_[led_index] = callback;
        }
    }

    void led_control(uint8_t led_index, bool state) {
        if (led_index < 3 && led_callbacks_[led_index] != nullptr) {
            led_callbacks_[led_index](state);
        }
    }

    LED_Control_Callback* get_led_callbacks() { return led_callbacks_; }

    void set_current_target_ma(int32_t value_ma) { current_target_ma_ = clamp_i32(value_ma, INT16_MAX); }

    inline void on_encoder_raw(uint32_t angle_raw) { feedback.inputs.set_angle_raw(angle_raw); }

    inline void on_adc_values(uint32_t clock, const uint16_t currents[3], uint16_t supply_adc, uint16_t temp_adc) {
        feedback.inputs.set_clock(clock);
        feedback.inputs.set_current_adc(currents);
        feedback.inputs.set_supply_adc(supply_adc);
        feedback.inputs.set_temper_adc(temp_adc);
    }

    int16_t d_current = 0;
    int32_t POS_ERR = 0;
    bool is_pos_ctrl_used = true;
    int16_t pole_pairs = 50;
    int32_t el_sign = -1;
    int32_t err_sign = -1;
    int32_t speed = 10;
    int32_t exec_pos = 0;
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
                uint32_t real_raw_ang = feedback.encoder.get_angle();
                int32_t real_raw_pos = feedback.encoder.get_position();

                if (!angle_calibrator.is_ready()) {
                    angle_el_ = angle_calibrator.tick(real_raw_pos, (int16_t)driver.motor.get_motor_type());
                    if (angle_calibrator.is_ready()) {
                        TP_LOG("Angle Calibration Done. Starting Normal/Cogging operation");
                        current_target_ma_ = 500;
                        amplitude_ = feedback.current.ma2norm(current_target_ma_);
                        feedback.clock.reset_motion(0);

                        motion_governor.inject_dummy_scurve(0);
                    }
                    break;
                } else {
                    // Реальный электрический угол (нужен для closed-loop и для sync)
                    angle_el_ = (uint16_t)((int16_t)(angle_calibrator.get_el_angle_correction(real_raw_ang)) * el_sign);
                    // 1. Берем текущее виртуальное время
                    // 1. Вычисляем текущее виртуальное время
                    uint32_t motion_time_us = feedback.clock.get_motion_us();

                    // 2. Получаем сырую физическую позицию (с учетом знака энкодера)
                    int64_t real_pos = feedback.encoder.get_position_64() * err_sign;

                    // 3. Вызываем губернатора.
                    // applied_torque_ff - это I_q (ток/момент), который был применен на предыдущем шаге
                    GovernorOutput gov = motion_governor.tick(motion_time_us, real_pos, (float)amplitude_);
                     // 4. Записываем текущий масштаб обратно в таймер
                    feedback.clock.set_motion_scale(motion_governor.get_recommended_time_scale());

                    // // 4. Записываем текущий масштаб обратно в таймер
                    // // 4. Debug: sweep scale от -100 до +100 и обратно
                    // static int32_t debug_scale = 100;
                    // static int32_t debug_scale_dir = -1;

                    // debug_scale += debug_scale_dir;
                    // if (debug_scale >= 100) {
                    //     debug_scale = 100;
                    //     debug_scale_dir = -1;
                    // }
                    // if (debug_scale <= -100) {
                    //     debug_scale = -100;
                    //     debug_scale_dir = 1;
                    // }

                    // feedback.clock.set_motion_scale(debug_scale);
                    // 5. Вычисляем ошибку позиции (POS_ERR) для отладки
                    POS_ERR = real_pos - gov.target.pos_q32_32;
                    exec_pos = (int32_t)(gov.target.pos_q32_32 >> 16);

                    // 6. Обновляем ПИД-регулятор (Cascade FOC Controller)
                    FOCCommand position_current_ma = position_controller.tick(
                        real_pos,               // Текущее реальное положение вала
                        gov.target.pos_q32_32,  // Целевое положение (Target Pos)
                        gov.target.vel,         // Целевая скорость (Feed-forward Vel) - УЖЕ МАСШТАБИРОВАНА И С ПРАВИЛЬНЫМ ЗНАКОМ!
                        current_target_ma_,     // Лимит тока / Максимальный крутящий момент
                        feedback.supply.get_voltage_mv(),  // Напряжение для Anti-Windup
                        driver.is_saturated                // Статус насыщения ШИМ
                    );

                    // FOCCommand position_current_ma =
                    //     position_controller.tick(real_raw_pos, 0, 0, current_target_ma_, feedback.supply.get_voltage_mv());
                    // FOCCommand position_current_ma = position_controller.tick(real_raw_pos, motion_state.pos, motion_state.vel,
                    //                                                           current_target_ma_,
                    //                                                           feedback.supply.get_voltage_mv());

                    if (is_pos_ctrl_used) {
                        amplitude_ = feedback.current.ma2norm(position_current_ma.i_q);
                        d_current = feedback.current.ma2norm(position_current_ma.i_d);
                    } else {
                        amplitude_ = feedback.current.ma2norm(current_target_ma_);
                    }
                }

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

    const Frequency& get_clock() const { return clock_; }
    Frequency& get_clock() { return clock_; }

private:
    inline bool fault_handler() {
        bool prev_has_hard_fault = has_hard_fault;
        has_hard_fault = feedback.error.has_fatal();

        if (has_hard_fault) {
            bool is_new = !prev_has_hard_fault;

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
        } else if (prev_has_hard_fault) {
            protection_recovery_timestamp_ = feedback.clock.get_us();
            TP_LOG("Protection recovery timer started (200ms)");
            return true;
        } else {
            if (feedback.clock.get_us() - protection_recovery_timestamp_ < PROTECTION_RECOVERY_TIMEOUT_MS) {
                return true;
            }
            led_control(0, false);
            led_control(1, false);
            led_control(2, false);
        }
        return false;
    }

    void reset_buffers() {
        for (size_t i = 0; i < 3; ++i) {
            pwm_norm_[i] = 0;
            pwm_duty_[i] = 0;
        }
    }

    void update_pwm_duty() {
        const int32_t period = pwm_resolution_;
        for (size_t i = 0; i < 3; ++i) {
            int32_t val = pwm_norm_[i];
            pwm_duty_[i] = (int16_t)(((val * period) >> 15) + (val == INT16_MAX));
        }
    }
};