/**
 * @file motor_calibration.h
 * @brief Automatic electrical parameter identification routines.
 *
 * Implements the MotorStatus state machine stages: winding-resistance
 * measurement (DC injection), high-frequency-injection inductance measurement
 * on the D/Q axes, and pole-pair/direction detection during slow rotation.
 */

// 3-phase PWM driver with FOC (Field Oriented Control) current regulation

#include <cmath>  // For sqrtf and fabsf
#include "../../common.h"
#include "../../drivers/pwm_driver_phy/pwm_driver.h"
#include "../../math/helpers/buzzer.h"
#include "../../math/helpers/frequency_generator.h"  // Frequency generator for HFI
#include "../../math/helpers/stability_check.h"      // Stability check for current monitoring
#include "../../math/helpers/timeout_timer.h"        // Timeout timer for countdown operations
#include "../motor.h"                                // Motor parameters structure
#include "foc-current.h"                             // FOC current controller with feedforward
#include "hfi_sense.h"                               // HFI sense for inductance measurement

// Phase patterns:
// ABC = 0, CBA = -16384, ACB = 32767

static constexpr uint16_t CAL_OVERSAMPLING_POW2 = 64;  // 64 samples
static constexpr uint8_t CAL_OVERSAMPLING_SHIFT = 6;   // log2(64) for shift optimization

// ~0.1 % of maximum driver output power. Otherwise we can bound ADC resolution easily and cant control driver
static constexpr float CAL_UNDERPOWER_RATIO = 0.001f;

// Старый шаг составлял 2 Q15-единицы за тик.
// Теперь это безразмерная доля Vbus, из которой на каждом тике получается шаг в Вольтах.
static constexpr float CAL_VOLTAGE_RAMP_STEP_RATIO = 0.0005;

static constexpr float CAL_VOLTAGE_LIMIT_RATIO = 0.5f;

// Сos(45°) для коррекции амплитуды тестового вектора.
static constexpr float COS_45 = 0.70710678118f;

// Минимальный ток для безопасного деления при расчёте R и Z.
static constexpr float MIN_MEASUREMENT_CURRENT_AMPS = 0.001f;

static constexpr int16_t CAL_ANGLE_45D = 8192;    // +45 deg
static constexpr int16_t CAL_ANGLE_90D = 16384;   // +90 deg
static constexpr int16_t CAL_ANGLE_135D = 24576;  // -135 deg
static constexpr int16_t CAL_ANGLE_180D = 32767;  // 180deg
static constexpr int16_t WARMUP_TIMEOUT = 4500;
static constexpr int16_t SETTLE_TIMEOUT = 900;
static constexpr int16_t ROTATION_TIMEOUT = 1800;
static constexpr uint32_t CALIB_PAUSE_CYCLES = 1800;  // 100ms pause @ 18kHz

// Увеличено время измерения индуктивности для большей точности (500 периодов на 1кГц)
static constexpr uint32_t INDUCTANCE_MEASURE_CYCLES_F1 = 9000;  // 0.5 sec at 18kHz
static constexpr uint32_t INDUCTANCE_MEASURE_CYCLES_F2 = 9000;  // 0.5 sec at 18kHz

// Частоты для двойной валидации индуктивности
static constexpr uint16_t HFI_FREQ_1 = 1000;  // 1000 Hz
static constexpr uint16_t HFI_FREQ_2 = 1500;  // 1500 Hz

// Motor calibration structure with all calibration-related variables and states
struct MotorCalibration {
private:
    // Connection calibration ramp variables
    float voltage_volts_;               // Current output voltage (Volts)
    float voltage_increment_volts_;     // Voltage increment per tick (Volts)
    float voltage_limit_volts_;         // Maximum voltage limit (Volts)
    float overcurrent_threshold_amps_;  // Overcurrent detection threshold (Amps)

    // Inductance measurement variables
    FrequencyGenerator hfi_gen_;
    float voltage_d_out_volts_;
    float voltage_q_out_volts_;
    float holding_voltage_volts_;
    float hfi_amplitude_volts_;
    float target_current_amps_;
    float current_max_amps_;
    float current_min_amps_;
    float hfi_accum_amps_;
    uint16_t hfi_periods_;

    // Результаты измерений на двух частотах
    float Ld_f1_, Ld_f2_;
    float Lq_f1_, Lq_f2_;

    // Slow rotation phase variables
    int16_t slow_rotation_angle_target_;     // Target angle for slow rotation (Q15)
    float slow_rotation_power_limit_watts_;  // Power limit during rotation (Watts)

    // Measurement reference values - accumulation for averaging
    float measure_current_a_sum_amps_;  // Phase A current accumulator for 100 samples
    float measure_current_b_sum_amps_;  // Phase B current accumulator for 100 samples

    // Inductance measurement accumulators
    int32_t inductance_a_sum_;  // Phase A inductance accumulator for HFI measurement
    int32_t inductance_b_sum_;  // Phase B inductance accumulator for HFI measurement

    // --- НОВЫЕ ПЕРЕМЕННЫЕ ДЛЯ КОРРЕКЦИИ СОПРОТИВЛЕНИЯ ---
    bool is_resistance_correction_;
    float first_ia_amps_;
    float first_ib_amps_;
    float uncorrected_total_resistance_ohms_;
    // ----------------------------------------------------

    // Calibration-specific timers and checkers
    timeout_timer timeout_;
    StabilityCheck stability_check_;
    PhyDriverPWM& drv;

    // Flash-restored profile snapshot (captured before identification overwrites
    // the values) - used for the post-calibration "same motor" decision.
    bool cfg_valid_ = false;    ///< A parameter profile was restored from system_dump.
    MotorType cfg_type_;        ///< Stored motor type.
    PhasePattern cfg_pattern_;  ///< Stored phase pattern.
    float cfg_resistance_ohm_;  ///< Stored winding resistance [Ohm].

    float max_power_watts_;
    float soft_power_limit_watts_;

public:
    // Constructor - initialize all calibration variables
    MotorCalibration(PhyDriverPWM& drv)
        : hfi_gen_(20000),  // Default 20kHz sys freq
          voltage_volts_(0.0f),
          voltage_increment_volts_(0.0f),
          voltage_limit_volts_(0.0f),
          overcurrent_threshold_amps_(0.0f),
          voltage_d_out_volts_(0.0f),
          voltage_q_out_volts_(0.0f),
          holding_voltage_volts_(0.0f),
          hfi_amplitude_volts_(0.0f),
          target_current_amps_(0.0f),
          current_max_amps_(-INFINITY),
          current_min_amps_(INFINITY),
          hfi_accum_amps_(0.0f),
          hfi_periods_(0),
          Ld_f1_(0.0f),
          Ld_f2_(0.0f),
          Lq_f1_(0.0f),
          Lq_f2_(0.0f),
          slow_rotation_angle_target_(0),
          slow_rotation_power_limit_watts_(0.0f),
          measure_current_a_sum_amps_(0.0f),
          measure_current_b_sum_amps_(0.0f),
          inductance_a_sum_(0),
          inductance_b_sum_(0),
          is_resistance_correction_(false),       // Инициализация
          first_ia_amps_(0.0f),                   // Инициализация
          first_ib_amps_(0.0f),                   // Инициализация
          uncorrected_total_resistance_ohms_(0),  // Инициализация
          timeout_(CALIB_PAUSE_CYCLES),
          stability_check_(0.05, 240),
          drv(drv),
          max_power_watts_(50.0f),
          soft_power_limit_watts_(0.75f) {}

    // Getters for calibration variables
    float get_voltage() const { return voltage_volts_; }
    float get_voltage_d() const { return voltage_d_out_volts_; }
    float get_voltage_q() const { return voltage_q_out_volts_; }
    float get_voltage_increment() const { return voltage_increment_volts_; }
    float get_voltage_limit() const { return voltage_limit_volts_; }
    int16_t get_slow_rotation_angle_target() const { return slow_rotation_angle_target_; }
    float get_slow_rotation_power_limit() const { return slow_rotation_power_limit_watts_; }

    float get_measure_current_a() const { return measure_current_a_sum_amps_ / static_cast<float>(CAL_OVERSAMPLING_POW2); }

    float get_measure_current_b() const { return measure_current_b_sum_amps_ / static_cast<float>(CAL_OVERSAMPLING_POW2); }

    // Setters for calibration variables
    void set_voltage(float voltage_volts) {
        voltage_volts_ = voltage_volts;

        if (voltage_volts_ > voltage_limit_volts_) {
            voltage_volts_ = voltage_limit_volts_;
        }

        if (voltage_volts_ < 0.0f) {
            voltage_volts_ = 0.0f;
        }
    }

    void accumulate_measure_currents(float current_a_amps, float current_b_amps) {
        measure_current_a_sum_amps_ += current_a_amps;
        measure_current_b_sum_amps_ += current_b_amps;
    }

    void reset_measure_accumulators() {
        measure_current_a_sum_amps_ = 0.0f;
        measure_current_b_sum_amps_ = 0.0f;
    }

    inline void update_voltage_range(float supply_voltage_volts) {
        voltage_limit_volts_ = supply_voltage_volts * CAL_VOLTAGE_LIMIT_RATIO;

        voltage_increment_volts_ = supply_voltage_volts * CAL_VOLTAGE_RAMP_STEP_RATIO;

        if (voltage_volts_ > voltage_limit_volts_) {
            voltage_volts_ = voltage_limit_volts_;
        }
    }

    inline void increase_voltage() {
        voltage_volts_ += voltage_increment_volts_;

        if (voltage_volts_ > voltage_limit_volts_) {
            voltage_volts_ = voltage_limit_volts_;
        }
    }

    inline void decrease_voltage() {
        voltage_volts_ -= voltage_increment_volts_;

        if (voltage_volts_ < 0.0f) {
            voltage_volts_ = 0.0f;
        }
    }

    void reset_voltage() {
        voltage_volts_ = 0.0f;
        voltage_d_out_volts_ = 0.0f;
        voltage_q_out_volts_ = 0.0f;
    }

    // ========================================================================
    // CALIBRATION LOGIC
    // ========================================================================
    void start_calibration(Motor& motor, MotorStatus& status_tracker, int16_t& angle_ref, float motor_ohmic_power_watts) {
        status_tracker = MotorStatus::SoftWarmup;

        // Snapshot the flash-restored profile (if any) BEFORE identification
        // overwrites these fields - needed later for the "same motor" check.
        cfg_valid_ = motor.is_loaded();
        cfg_type_ = motor.get_motor_type();
        cfg_pattern_ = motor.get_phase_pattern();
        cfg_resistance_ohm_ = motor.get_resistance();

        reset_voltage();
        stability_check_.reset();
        is_resistance_correction_ = false;  // Сбрасываем флаг

        soft_power_limit_watts_ = motor_ohmic_power_watts * 0.75f;

        // #REPLACE with just simple storing value + value threshold check

        drv.motor_sel_.set_motor_type(MotorType_STEP);
        drv.motor_sel_.set_phase_pattern(PhasePattern_ABC);

        angle_ref = CAL_ANGLE_45D;
        timeout_.set_timeout(WARMUP_TIMEOUT);
    }

    void tick_calibration(float supply_voltage_volts,
                          float power_watts,
                          float current_a_amps,
                          float current_b_amps,
                          float current_d_amps,
                          float current_q_amps,
                          MotorStatus& status_tracker,
                          Motor& motor,
                          DriverStatus& setup_status,
                          int16_t& angle_ref) {
        update_voltage_range(supply_voltage_volts);

        switch (status_tracker) {
            case MotorStatus::SoftWarmup: {
                voltage_d_out_volts_ = voltage_volts_;
                voltage_q_out_volts_ = 0.0f;

                // [OPT] measured_current нужен только здесь — вычисляем только здесь
                const float measured_current_amps = fabsf(current_a_amps + current_b_amps);

                // #REPLACE with just simple storing value + value threshold check
                const bool power_lim = power_watts >= soft_power_limit_watts_;

                const bool voltage_lim = voltage_volts_ >= voltage_limit_volts_;

                if ((stability_check_.tick(measured_current_amps) && power_watts > (max_power_watts_ * CAL_UNDERPOWER_RATIO)) ||
                    timeout_.tick() == 0) {
                    target_current_amps_ = measured_current_amps;

                    timeout_.set_timeout(SETTLE_TIMEOUT);
                    status_tracker = MotorStatus::Settle;
                } else {
                    if (power_lim) {
                        decrease_voltage();
                    } else {
                        increase_voltage();
                    }
                }
                break;
            }

            case MotorStatus::Settle: {
                voltage_d_out_volts_ = voltage_volts_;
                voltage_q_out_volts_ = 0.0f;

                if (timeout_.tick() == 0) {
                    status_tracker = MotorStatus::ResistanceMeasure;
                    timeout_.set_timeout(CAL_OVERSAMPLING_POW2);
                }
                break;
            }

            case MotorStatus::ResistanceMeasure: {
                voltage_d_out_volts_ = voltage_volts_;
                voltage_q_out_volts_ = 0.0f;

                // ИЗМЕНЕНИЕ: Добавлен аргумент power для вычисления корректировки
                // #REPLACE norm value should be removed and replaced with float power
                process_resistance_measure(current_a_amps, current_b_amps, power_watts, supply_voltage_volts, status_tracker,
                                           motor, setup_status, angle_ref);
                break;
            }

            case MotorStatus::SlowRotation: {
                // ... (Оригинальный код не менялся)
                voltage_d_out_volts_ = voltage_volts_;
                voltage_q_out_volts_ = 0.0f;

                if (power_watts >= max_power_watts_) {
                    set_error(status_tracker, motor, setup_status);
                    break;
                }

                if (timeout_.tick() == 0) {
                    angle_ref = slow_rotation_angle_target_;

                    timeout_.set_timeout(SETTLE_TIMEOUT);
                    status_tracker = MotorStatus::Settle;
                } else {
                    int16_t delta = slow_rotation_angle_target_ - angle_ref;

                    angle_ref += delta / static_cast<int32_t>(timeout_.get_timeout() + 1);
                }
                break;
            }

            // ... Остальные состояния HFI без изменений ...
            case MotorStatus::InductanceMeasureWarmup: {
                voltage_d_out_volts_ = holding_voltage_volts_;
                voltage_q_out_volts_ = 0.0f;

                if (timeout_.tick() == 0) {
                    hfi_gen_.set_frequency_target(HFI_FREQ_1);

                    setup_hfi_state(MotorStatus::InductanceMeasureD, status_tracker, motor, INDUCTANCE_MEASURE_CYCLES_F1);
                }
                break;
            }

            case MotorStatus::InductanceMeasureD: {
                process_hfi_axis(current_d_amps, true, false, supply_voltage_volts, status_tracker, motor, setup_status,
                                 MotorStatus::InductanceMeasureD2, INDUCTANCE_MEASURE_CYCLES_F2, &Ld_f1_, HFI_FREQ_1);
                break;
            }

            case MotorStatus::InductanceMeasureD2: {
                hfi_gen_.set_frequency_target(HFI_FREQ_2);

                process_hfi_axis(current_d_amps, true, false, supply_voltage_volts, status_tracker, motor, setup_status,
                                 MotorStatus::InductanceMeasureQ, INDUCTANCE_MEASURE_CYCLES_F1, &Ld_f2_, HFI_FREQ_2);
                break;
            }

            case MotorStatus::InductanceMeasureQ: {
                hfi_gen_.set_frequency_target(HFI_FREQ_1);

                // Изменено false, true -> false, false
                process_hfi_axis(current_q_amps, false, false, supply_voltage_volts, status_tracker, motor, setup_status,
                                 MotorStatus::InductanceMeasureQ2, INDUCTANCE_MEASURE_CYCLES_F2, &Lq_f1_, HFI_FREQ_1);
                break;
            }

            case MotorStatus::InductanceMeasureQ2: {
                hfi_gen_.set_frequency_target(HFI_FREQ_2);

                process_hfi_axis(current_q_amps, false, true, supply_voltage_volts, status_tracker, motor, setup_status,
                                 MotorStatus::Success, 0, &Lq_f2_, HFI_FREQ_2);
                break;
            }

            default:
                break;
        }
    }

private:
    void setup_hfi_state(MotorStatus target_state, MotorStatus& status_tracker, Motor& motor, uint32_t timeout_cycles) {
        status_tracker = target_state;
        timeout_.set_timeout(timeout_cycles);
        current_max_amps_ = -INFINITY;
        current_min_amps_ = INFINITY;
        hfi_accum_amps_ = 0.0f;
        hfi_periods_ = 0;
        hfi_gen_.reset();
    }

    void process_hfi_axis(float measured_current_amps,
                          bool is_d_axis,
                          bool is_final_step,
                          float supply_voltage_volts,
                          MotorStatus& status_tracker,
                          Motor& motor,
                          DriverStatus& setup_status,
                          MotorStatus next_state,
                          uint32_t next_timeout,
                          float* result_L,
                          uint16_t current_freq) {
        int16_t prev_angle = hfi_gen_.get_angle();
        Vec2D hfi_out = hfi_gen_.tick();
        int16_t curr_angle = hfi_gen_.get_angle();

        const float hfi_wave = static_cast<float>(hfi_out.cos) / static_cast<float>(INT16_MAX);

        const float hfi_voltage_volts = hfi_amplitude_volts_ * hfi_wave;

        if (is_d_axis) {
            voltage_d_out_volts_ = holding_voltage_volts_ + hfi_voltage_volts;

            voltage_q_out_volts_ = 0.0f;
        } else {
            voltage_d_out_volts_ = holding_voltage_volts_;
            voltage_q_out_volts_ = hfi_voltage_volts;
        }

        if (voltage_d_out_volts_ > voltage_limit_volts_) {
            voltage_d_out_volts_ = voltage_limit_volts_;
        }

        if (voltage_d_out_volts_ < -voltage_limit_volts_) {
            voltage_d_out_volts_ = -voltage_limit_volts_;
        }

        if (!is_d_axis && (holding_voltage_volts_ + hfi_amplitude_volts_) > voltage_limit_volts_) {
            hfi_amplitude_volts_ = voltage_limit_volts_ - holding_voltage_volts_;
        }

        if (hfi_amplitude_volts_ < 0.0f) {
            hfi_amplitude_volts_ = 0.0f;
        }

        if (measured_current_amps > current_max_amps_) {
            current_max_amps_ = measured_current_amps;
        }

        if (measured_current_amps < current_min_amps_) {
            current_min_amps_ = measured_current_amps;
        }

        if (prev_angle > 0 && curr_angle <= 0) {
            float ripple_amps = current_max_amps_ - current_min_amps_;

            if (hfi_periods_ > 0) {
                hfi_accum_amps_ += ripple_amps;  // Skip first partial period
            }

            hfi_periods_++;

            // PID-like adaptation for amplitude
            float target_ripple_min_amps = target_current_amps_ * 0.20f;

            // [OPT] 25% от target_current_ = деление на 4 = правый сдвиг на 2 бита
            float target_ripple_max_amps = target_current_amps_ * 0.25f;

            if (ripple_amps < target_ripple_min_amps) {
                hfi_amplitude_volts_ += voltage_increment_volts_;  // Slowed down adaptation for stability during long tests
            } else if (ripple_amps > target_ripple_max_amps) {
                hfi_amplitude_volts_ -= voltage_increment_volts_;
            }

            if ((holding_voltage_volts_ + hfi_amplitude_volts_) > voltage_limit_volts_) {
                hfi_amplitude_volts_ = voltage_limit_volts_ - holding_voltage_volts_;
            }

            if (hfi_amplitude_volts_ < 0.0f) {
                hfi_amplitude_volts_ = 0.0f;
            }

            current_max_amps_ = -INFINITY;
            current_min_amps_ = INFINITY;
        }

        if (timeout_.tick() == 0) {
            float avg_ripple_amps = (hfi_periods_ > 1) ? (hfi_accum_amps_ / static_cast<float>(hfi_periods_ - 1)) : 0.0f;

            // Calculate accurate Inductance taking Ohmic Resistance into account
            // [OPT] Кэшируем norm2mv(hfi_amplitude_) — вызывается дважды: для v_peak и для лога
            float v_peak_volts = hfi_amplitude_volts_;

            float i_peak_amps = avg_ripple_amps * 0.5f;
            float omega_rad_per_sec = 2.0f * 3.14159265f * static_cast<float>(current_freq);

            // Motor API пока возвращает миллиОмы; внутри класса сразу работаем только в Омах.
            float resistance_ohms = motor.get_total_resistance();  // in Ohms

            // [OPT] Z вычисляется заранее: устраняем повторное деление v_peak/i_peak в логе
            //       и защищаем от деления на ~0 (i_peak <= 0.001f был вне блока if ниже — это был баг)
            float impedance_ohms = (i_peak_amps > MIN_MEASUREMENT_CURRENT_AMPS) ? (v_peak_volts / i_peak_amps) : 0.0f;

            if (i_peak_amps > MIN_MEASUREMENT_CURRENT_AMPS) {
                float X_L_sq = (impedance_ohms * impedance_ohms) - (resistance_ohms * resistance_ohms);

                if (X_L_sq > 0.0f)
                    *result_L = (sqrtf(X_L_sq) / omega_rad_per_sec) * 1e6f;  // uH
                else
                    *result_L = 0.0f;  // R dominated Z entirely (shouldn't happen at 1kHz+)

            } else {
                *result_L = 0.0f;
            }

            // [OPT] Используем кэшированный amp_mv вместо повторного вызова norm2mv

            // [OPT] Используем кэшированный Z вместо повторного деления v_peak / i_peak

            if (is_final_step) {
                // Final validation logic
                float diff_d = (Ld_f1_ > 0.0f) ? fabsf(Ld_f1_ - Ld_f2_) / Ld_f1_ : 0.0f;

                float diff_q = (Lq_f1_ > 0.0f) ? fabsf(Lq_f1_ - Lq_f2_) / Lq_f1_ : 0.0f;

                // Average the 1kHz and 1.5kHz results
                float final_Ld = (Ld_f1_ + Ld_f2_) * 0.5f;
                float final_Lq = (Lq_f1_ + Lq_f2_) * 0.5f;

                // Store the identified D/Q inductances into the motor profile
                // (measurements are computed in microhenries, stored in H).
                motor.set_inductance_d(final_Ld * 1e-6f);
                motor.set_inductance_q(final_Lq * 1e-6f);
                finish_calibration_success(status_tracker, motor, setup_status);
            } else {
                setup_hfi_state(next_state, status_tracker, motor, next_timeout);
            }
        }
    }

    void start_slow_rotation(int16_t target_angle, float power_limit_watts) {
        slow_rotation_angle_target_ = target_angle;
        slow_rotation_power_limit_watts_ = power_limit_watts;
    }

    void set_error(MotorStatus& status_tracker, Motor& motor, DriverStatus& setup_status) {
        setup_status.set_driver_status(OperationStatus::Error);
        status_tracker = MotorStatus::CalibError;
        motor.set_status(MotorStatus::CalibError);
        reset_voltage();
    }

    void start_inductance_measurement(float supply_voltage_volts,
                                      MotorStatus& status_tracker,
                                      Motor& motor,
                                      DriverStatus& setup_status,
                                      int16_t& angle_ref) {
        motor.set_motor_type(drv.motor_sel_.get_motor_type());
        motor.set_phase_pattern(drv.motor_sel_.get_phase_pattern());
        setup_status.motor_type_not_verified = 0;
        setup_status.voltage_unsafe = supply_voltage_volts <= 0.0f;

        holding_voltage_volts_ = voltage_volts_ * 0.5f;
        hfi_amplitude_volts_ = voltage_increment_volts_ * 50.0f;

        if ((holding_voltage_volts_ + hfi_amplitude_volts_) > voltage_limit_volts_) {
            hfi_amplitude_volts_ = voltage_limit_volts_ - holding_voltage_volts_;
        }

        if (hfi_amplitude_volts_ < 0.0f) {
            hfi_amplitude_volts_ = 0.0f;
        }

        angle_ref = 0;  // Align to D-axis

        status_tracker = MotorStatus::InductanceMeasureWarmup;
        timeout_.set_timeout(SETTLE_TIMEOUT);
    }

    void finish_calibration_success(MotorStatus& status_tracker, Motor& motor, DriverStatus& setup_status) {
        status_tracker = MotorStatus::Success;
        motor.set_status(MotorStatus::Success);

        // Same-motor identification: compare the freshly identified set with
        // the profile restored from system_dump. Matching connection/type and
        // resistance within 15% => the same motor: keep the stored
        // configuration (Motor Identified). Otherwise overwrite it by
        // appending a new record - the loader always picks the newest one.
        bool same = cfg_valid_ && (motor.get_motor_type() == cfg_type_) && (motor.get_phase_pattern() == cfg_pattern_);
        if (same) {
            float r0 = cfg_resistance_ohm_;
            float r1 = motor.get_resistance();
            same = (fabsf(r0) > 1e-6f) && (fabsf(r1 - r0) <= 0.15f * fabsf(r0));
        }

        if (!same)
            motor.begin_save();  // Non-blocking, serviced 8 words per tick

        reset_voltage();
    }

    void trigger_resistance_correction(MotorStatus& status_tracker, Motor& motor, float ia_amps, float ib_amps) {
        // Сохраняем токи первого прохода (в mA) для доминантной фазы
        first_ia_amps_ = ia_amps;
        first_ib_amps_ = ib_amps;

        // Motor API пока возвращает миллиОмы; внутри класса сразу переводим в Омы.
        uncorrected_total_resistance_ohms_ = motor.get_total_resistance();

        is_resistance_correction_ = true;

        voltage_volts_ *= 0.5f;

        reset_measure_accumulators();
        status_tracker = MotorStatus::Settle;
        timeout_.set_timeout(SETTLE_TIMEOUT);
    }
    // -----------------------------------------------------------------

    void process_resistance_measure(float current_a_amps,
                                    float current_b_amps,
                                    float power_watts,
                                    float supply_voltage_volts,
                                    MotorStatus& status_tracker,
                                    Motor& motor,
                                    DriverStatus& setup_status,
                                    int16_t& angle_ref) {
        // First cycle: check if the counter is initialized (initially equals CAL_OVERSAMPLING_POW2, i.e. 64)
        if (timeout_.get_timeout() == CAL_OVERSAMPLING_POW2)
            // Reset all current accumulators to start clean averaging
            reset_measure_accumulators();

        // Accumulate current samples for phases A and B into accumulators for subsequent averaging
        accumulate_measure_currents(current_a_amps, current_b_amps);

        // Decrement the timeout counter (samples) by 1. When it reaches 0, all 64 measurements are collected
        if (timeout_.tick() == 0) {
            // Сos(45°) в Q16 = round(cos(π/4) * 65536)
            float voltage_corrected_volts = voltage_volts_ * COS_45;  // Voltage amplitude fix

            // Get the averaged phase A and B currents over the entire measurement period
            float ia_amps = get_measure_current_a();
            float ib_amps = get_measure_current_b();

            float applied_voltage_volts = voltage_corrected_volts;

            // Compute corrected voltage in microvolts (normalized → mV, then multiply by 1000)
            if (is_resistance_correction_) {
                const float abs_first_ia = fabsf(first_ia_amps_);
                const float abs_first_ib = fabsf(first_ib_amps_);

                // Выбираем доминантную фазу (лучший SNR)
                bool use_phase_a = abs_first_ia >= abs_first_ib;

                float abs_first_i_amps = use_phase_a ? abs_first_ia : abs_first_ib;

                float second_i_amps = use_phase_a ? ia_amps : ib_amps;

                float abs_second_i_amps = fabsf(second_i_amps);

                // delta_i должна быть > 0, так как I1 > I2 при V1 > V2
                float delta_i_amps = abs_first_i_amps - abs_second_i_amps;

                float corrected_total_resistance_ohms = uncorrected_total_resistance_ohms_;

                // Формула: R_corrected = R_uncorrected * I1 / (2 * delta_i)
                // Напряжение полностью сокращается при точном V2 = V1/2
                if (delta_i_amps > 0.0f && abs_first_i_amps > MIN_MEASUREMENT_CURRENT_AMPS) {
                    corrected_total_resistance_ohms = corrected_total_resistance_ohms * abs_first_i_amps / (2.0f * delta_i_amps);
                }

                // Граница старого Motor API: передаём миллиОмы.
                motor.set_total_resistance(corrected_total_resistance_ohms);

                is_resistance_correction_ = false;

                start_inductance_measurement(supply_voltage_volts, status_tracker, motor, setup_status, angle_ref);
                return;
            }

            if (power_watts >
                (max_power_watts_ * CAL_UNDERPOWER_RATIO)) {  // We have sufficient power, current calculations are reliable
                // Convert averaged normalized currents to milliamps
                float resistance_a_ohms =
                    (fabsf(ia_amps) > MIN_MEASUREMENT_CURRENT_AMPS) ? (applied_voltage_volts / ia_amps) : INFINITY;

                float resistance_b_ohms =
                    (fabsf(ib_amps) > MIN_MEASUREMENT_CURRENT_AMPS) ? (applied_voltage_volts / ib_amps) : INFINITY;

                // If angle equals 16384 (~+90 electrical degrees) — always the first pass for any motor
                if (angle_ref == CAL_ANGLE_45D) {
                    // Set the timeout counter to 1800 cycles (time allocated for slow rotation)
                    timeout_.set_timeout(ROTATION_TIMEOUT);

                    // Stepper or BLDC motor, first pass: phases should be balanced (within ~15% of average)
                    if (10.0f * fabsf(ib_amps - ia_amps) < (ia_amps + ib_amps)) {
                        // Store temporary resistance as arithmetic mean of phases A and B
                        float average_resistance_ohms = (resistance_a_ohms + resistance_b_ohms) * 0.5f;

                        // Граница старого Motor API: передаём миллиОмы.
                        motor.set_total_resistance(average_resistance_ohms);

                        // Slowly rotate the vector by 30° for verification. Target angle 16384 (current + 45° el. (BLDC: +30°))
                        start_slow_rotation(CAL_ANGLE_90D, power_watts);

                        status_tracker = MotorStatus::SlowRotation;
                        return;
                    } else {  // Possibly stepper but in wrong phase pattern, fixing phase commutation
                        set_voltage(voltage_corrected_volts);  // Fix voltage level to seamless transfer - mult power by 2

                        // Slowly rotate the vector backwards to angle -24576, rotation by 45°
                        start_slow_rotation(-CAL_ANGLE_135D, power_watts);

                        status_tracker = MotorStatus::SlowRotation;

                        if ((ia_amps * 0.25f) >= ib_amps) {
                            drv.motor_sel_.set_phase_pattern(PhasePattern_CBA);

                            // Set target angle to -16384 (corresponds to CBA connection)
                            angle_ref = -CAL_ANGLE_90D;
                            return;
                        } else if ((ib_amps * 0.25f) >= ia_amps) {
                            drv.motor_sel_.set_phase_pattern(PhasePattern_ACB);

                            // Set target angle to -32768 (corresponds to ACB connection)
                            angle_ref = CAL_ANGLE_180D;
                            return;
                        }
                    }
                }  // Second pass for BLDC or Stepper in ABC connection
                else if (angle_ref == CAL_ANGLE_90D) {
                    // BLDC motor, second pass: check if phases are balanced (within ~15% of average)
                    // Expected: current iA — negative and dominant; current iB — positive and equal to half of iA (iA = iB + iC,
                    // where iB ~= iC)
                    if (14.0f * fabsf((2.0f * ia_amps) + ib_amps) < ((-ia_amps * 2.0f) + ib_amps)) {
                        // Calculate average resistance accounting for all three phases
                        float previous_resistance_ohms = motor.get_total_resistance();

                        float average_resistance_ohms = (previous_resistance_ohms + resistance_a_ohms) / 3.0f;

                        // Write averaged resistance into the motor profile
                        motor.set_total_resistance(average_resistance_ohms);

                        // Set motor type in the selector as BLDC
                        drv.motor_sel_.set_motor_type(MotorType_BLDC);

                        trigger_resistance_correction(status_tracker, motor, ia_amps, ib_amps);
                        return;
                    }

                    // Stepper motor, second pass: check if iA is significantly 16x greater than iB
                    // (Expected: iA — negative dominant, iB close to 0 (any sign). Resistance was already calculated in first
                    // pass)
                    else if ((ib_amps * 0.0625f) > ia_amps) {
                        trigger_resistance_correction(status_tracker, motor, ia_amps, ib_amps);
                        return;
                    }
                }

                // Check if angle is around -24576 (-135 degrees). This is the second pass for Stepper with CBA or ACB connection
                else if (angle_ref == -CAL_ANGLE_135D) {
                    // Check if phases B and C are balanced (difference within ~15% of average)
                    if (10.0f * fabsf(ib_amps - ia_amps) < (-ia_amps - ib_amps)) {
                        // Stepper motor: take average resistance of two phases with negative sign (due to vector angle)
                        float average_resistance_ohms = -(resistance_a_ohms + resistance_b_ohms) * 0.5f;

                        // Write averaged resistance into the motor profile
                        motor.set_total_resistance(average_resistance_ohms);

                        trigger_resistance_correction(status_tracker, motor, ia_amps, ib_amps);
                        return;
                    }
                }
            }

            drv.motor_sel_.set_motor_type(MotorType_Invalid);
            set_error(status_tracker, motor, setup_status);
        }
    }
};
