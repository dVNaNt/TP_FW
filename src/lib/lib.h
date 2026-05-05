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

struct OpenLoopController {
public:
    // Параметры
    int16_t pole_pairs = 50;  // Количество пар полюсов
    int32_t target_rpm = 60;  // Целевая скорость, об/мин (знак = направление)
    int32_t iq_ma = 800;      // Фиксированный ток Iq в mA (момент)
    int32_t id_ma = 0;        // Ток Id (обычно 0)

private:
    int32_t control_freq_hz_;  // Частота вызова tick() в Гц
    uint32_t angle_accum_;     // Накопленный угол (переполнение = 1 эл. оборот)
    uint16_t el_angle_;        // Текущий электрический угол [0..65535]

public:
    explicit OpenLoopController(int32_t control_freq_hz) : control_freq_hz_(control_freq_hz), angle_accum_(0), el_angle_(0) {}

    // Вызывать каждый тик управления.
    // Возвращает текущий электрический угол для передачи в driver.tick_control_foc().
    uint16_t tick() {
        // Инкремент за один тик:
        // 1 электрический оборот = 2^32 в накопителе.
        int64_t delta = (int64_t)target_rpm * pole_pairs;
        delta = (delta << 32) / ((int64_t)60 * control_freq_hz_);

        angle_accum_ += (uint32_t)delta;
        el_angle_ = (uint16_t)(angle_accum_ >> 16);
        return el_angle_;
    }

    uint16_t get_el_angle() const { return el_angle_; }

    void reset() {
        angle_accum_ = 0;
        el_angle_ = 0;
    }

    // Синхронизировать фазу с реальным энкодером при переключении режимов
    void sync_angle(uint16_t real_el_angle) {
        el_angle_ = real_el_angle;
        angle_accum_ = (uint32_t)real_el_angle << 16;
    }
};

struct ElecAnglePLL {
    // ── Параметры ──────────────────────────────────────────────────────────
    float dt;
    float fc_min = 40.0f;    // Гц, сильная фильтрация в покое (убирает шум)
    float fc_max = 2000.0f;  // Гц, полное пропускание сигнала без фильтрации

    // Настройка влияния скорости на полосу (Hz полосы на 1 Hz эл. скорости)
    float vel_to_fc = 1.5f;

    // Лимит ошибки 10 градусов (в Q16 это: 65536 * 10 / 360 = 1820.4)
    float max_err_ticks = 1820.0f;

    // ── Состояние ─────────────────────────────────────────────────────────
    float theta_est = 0.0f;  // Оценка угла (float для точности интегрирования)
    float vel_est = 0.0f;    // Оценка эл. скорости (тиков/сек)
    bool is_initialized = false;

private:
    static inline float clampf(float x, float lo, float hi) { return (x < lo) ? lo : ((x > hi) ? hi : x); }

public:
    ElecAnglePLL(float dt = 0.00005f) : dt(dt) {}

    void reset(uint16_t initial_angle) {
        theta_est = (float)initial_angle;
        vel_est = 0.0f;
        is_initialized = true;
    }

    uint16_t tick(uint16_t raw_elec_angle) {
        if (!is_initialized) {
            reset(raw_elec_angle);
            return raw_elec_angle;
        }

        // 1. Штатное переполнение для получения кратчайшей ошибки [-32768...32767]
        uint16_t est_u16 = (uint16_t)theta_est;
        int16_t err_ticks = (int16_t)(raw_elec_angle - est_u16);
        float err_f = (float)err_ticks;
        float err_abs = fabsf(err_f);

        // 2. Оценка текущей эл. скорости в Герцах (1 оборот = 65536 тиков)
        float vel_hz = fabsf(vel_est) * (1.0f / 65536.0f);

        // 3. Формирование динамической полосы пропускания (Fc)
        // Базовая полоса от скорости
        float fc = fc_min + vel_hz * vel_to_fc;

        // Взрывообразная реакция при приближении к 10 градусам ошибки
        float err_ratio = err_abs / max_err_ticks;
        if (err_ratio > 0.1f) {
            // Возводим в 4-ю степень.
            // При 5° (ratio 0.5) добавка будет слабой (~0.06 от максимума).
            // При 10° (ratio 1.0) добавка 100% пробивает фильтр в максимум.
            float err_boost = err_ratio * err_ratio;
            err_boost *= err_boost;
            fc += (fc_max - fc_min) * err_boost;
        }

        fc = clampf(fc, fc_min, fc_max);

        // 4. Коэффициенты PLL (Критическое демпфирование zeta = 1.0)
        float wc = 2.0f * (float)M_PI * fc;
        float kp = 2.0f * wc;
        float ki = wc * wc;

        // 5. Интегрирование Type-2 Loop
        vel_est += ki * err_f * dt;
        theta_est += (vel_est + kp * err_f) * dt;

        // 6. Нормализация аккумулятора в пределах [0.0, 65536.0)
        // Использование float до 65536 безопасно, точность мантиссы ~0.003 тика
        if (theta_est >= 65536.0f) {
            theta_est -= 65536.0f;
        } else if (theta_est < 0.0f) {
            theta_est += 65536.0f;
        }

        return (uint16_t)theta_est;
    }

    // Полезный бонус: PLL сам вычисляет идеально гладкую эл. скорость
    float get_velocity_hz() const { return vel_est * (1.0f / 65536.0f); }
};

// ============================================================================
// TunePulseHeart - Complete motor control system
// ============================================================================

struct TunePulseHeart {
public:
    OpenLoopController open_loop_controller;  // Open-loop FOC без обратной связи
    bool use_open_loop = false;

    // ========================================================================
    // Настройки энкодера для трекинга ошибки в Open-loop
    // ========================================================================
    int32_t encoder_cpr = 16384;  // Укажите CPR вашего энкодера (напр. 16384 для 14-bit)
    int64_t virtual_pos_ = 0;     // Интегратор идеальной механической позиции (в тиках)
    uint16_t freq_hz_;            // Сохраненная частота контура управления

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

    MotorVectorSynthesis position_controller = MotorVectorSynthesis();

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
    ElecAnglePLL angle_pll = ElecAnglePLL(0.00005f);
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
        : open_loop_controller(freq_hz),
          freq_hz_(freq_hz),
          clock_(170000000, freq_hz),
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
    bool prev_use_open_loop_ = true;  // для sync при переключении режимов

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

    inline void on_adc_values(const uint16_t currents[3], uint16_t supply_adc, uint16_t temp_adc) {
        feedback.inputs.set_current_adc(currents);
        feedback.inputs.set_supply_adc(supply_adc);
        feedback.inputs.set_temper_adc(temp_adc);
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
    int32_t is_cal_used = 1;
    int32_t el_sign = -1;
    int16_t prev_el_angle = 0;

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
                int32_t real_raw_ang = feedback.encoder.get_angle();
                int32_t real_raw_pos = feedback.encoder.get_position();

                if (!angle_calibrator.is_ready()) {
                    angle_el_ = angle_calibrator.tick(real_raw_pos, (int16_t)driver.motor.get_motor_type());
                    if (angle_calibrator.is_ready()) {
                        TP_LOG("Angle Calibration Done. Starting Normal/Cogging operation");
                        current_target_ma_ = 1500;
                        amplitude_ = feedback.current.ma2norm(current_target_ma_);
                        // Синхронизируем open-loop фазу с реальным углом сразу после калибровки
                        open_loop_controller.sync_angle(angle_el_);
                        virtual_pos_ = real_raw_pos;  // Инициализируем виртуальную позицию
                    }
                    break;
                }

                // Реальный электрический угол (нужен для closed-loop и для sync)
                uint16_t real_el = (uint16_t)((int16_t)(angle_calibrator.get_el_angle_correction(real_raw_ang)) * el_sign);
                // uint16_t real_el_filt = angle_pll.tick(real_el);
                // ================================================================
                // РЕЖИМ ВЫБОРА: open-loop vs position_controller
                // ================================================================
                if (use_open_loop) {
                    // --- OPEN-LOOP: вращаем виртуальный вектор тока ---
                    if (!prev_use_open_loop_) {
                        open_loop_controller.sync_angle(real_el);
                        virtual_pos_ = real_raw_pos;  // Синхронизация при возврате в open-loop
                    }
                    angle_el_ = open_loop_controller.tick();

                    int16_t diff = prev_el_angle - angle_el_;
                    prev_el_angle = angle_el_;

                    // Умножаем на el_sign, чтобы направление роста позиции совпадало с энкодером
                    virtual_pos_ += diff;

                    // Обновляем ошибку положения (Реальная позиция минус Идеальная позиция)
                    POS_ERR = real_raw_pos - (int32_t)(virtual_pos_ / 50);

                    amplitude_ = feedback.current.ma2norm(open_loop_controller.iq_ma);
                    d_current = feedback.current.ma2norm(open_loop_controller.id_ma);

                } else {
                    // --- CLOSED-LOOP: position_controller (прежний путь) ---
                    angle_el_ = real_el;

                    MotionState motion_state = motion_executor.tick();

                    FOCCommand position_current_ma = position_controller.tick(motion_state.pos, real_raw_pos, motion_state.vel,
                                                                              motion_state.acc, 0, current_target_ma_, real_el, feedback.supply.get_voltage_mv());

                    POS_ERR = real_raw_pos - motion_state.pos;

                    if (previous_mode != mode) {
                        previous_mode = mode;
                    }

                    if (is_pos_ctrl_used) {
                        amplitude_ = feedback.current.ma2norm(position_current_ma.u_q) * invert_ladrc;
                        d_current = feedback.current.ma2norm(position_current_ma.u_d) * invert_ladrc;
                    } else {
                        amplitude_ = feedback.current.ma2norm(current_target_ma_);
                    }
                }

                prev_use_open_loop_ = use_open_loop;  // запомним для следующего тика
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
            protection_recovery_timestamp_ = millis();
            TP_LOG("Protection recovery timer started (200ms)");
            return true;
        } else {
            if (millis() - protection_recovery_timestamp_ < PROTECTION_RECOVERY_TIMEOUT_MS) {
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