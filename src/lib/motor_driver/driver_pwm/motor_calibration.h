// 3-phase PWM driver with FOC (Field Oriented Control) current regulation

#include <cmath>  // For sqrtf and fabsf
#include "../../common.h"
#include "../../feedback/feedback.h"
#include "../../math/helpers/buzzer.h"
#include "../../math/helpers/frequency_generator.h"  // Frequency generator for HFI
#include "../../math/helpers/stability_check.h"      // Stability check for current monitoring
#include "../../math/helpers/timeout_timer.h"        // Timeout timer for countdown operations
#include "../motor.h"                                // Motor parameters structure
#include "foc-current.h"                             // FOC current controller with feedforward
#include "hfi_sense.h"                               // HFI sense for inductance measurement
#include "motor_selector.h"

// Phase patterns:
// ABC = 0, CBA = -16384, ACB = 32767

// Forward declaration
struct Feedback;

static constexpr uint16_t CAL_OVERSAMPLING_POW2 = 64;  // 64 samples
static constexpr uint8_t CAL_OVERSAMPLING_SHIFT = 6;   // log2(64) for shift optimization

// ~0.1 % of maximum driver output power. Otherwise we can bound ADC resolution easily and cant control driver
static constexpr uint32_t CAL_UNDERPOWER_LIM_NORM = 256 * 256 * 16;

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
    int16_t voltage_;                // Current output voltage (Q15 duty)
    int16_t voltage_increment_;      // Voltage increment per tick (Q15)
    int16_t voltage_limit_;          // Maximum voltage limit (Q15)
    int16_t overcurrent_threshold_;  // Overcurrent detection threshold (Q15)

    // Inductance measurement variables
    FrequencyGenerator hfi_gen_;
    int16_t voltage_d_out_;
    int16_t voltage_q_out_;
    int16_t holding_voltage_;
    int16_t hfi_amplitude_;
    int16_t target_current_;
    int32_t current_max_;
    int32_t current_min_;
    int32_t hfi_accum_;
    uint16_t hfi_periods_;

    // Результаты измерений на двух частотах
    float Ld_f1_, Ld_f2_;
    float Lq_f1_, Lq_f2_;

    // Slow rotation phase variables
    int16_t slow_rotation_angle_target_;  // Target angle for slow rotation (Q15)
    uint16_t slow_rotation_power_limit_;  // Power limit during rotation (normalized)

    // Measurement reference values - accumulation for averaging
    int32_t measure_current_a_sum_;  // Phase A current accumulator for 100 samples
    int32_t measure_current_b_sum_;  // Phase B current accumulator for 100 samples

    // Inductance measurement accumulators
    int32_t inductance_a_sum_;  // Phase A inductance accumulator for HFI measurement
    int32_t inductance_b_sum_;  // Phase B inductance accumulator for HFI measurement

    // --- НОВЫЕ ПЕРЕМЕННЫЕ ДЛЯ КОРРЕКЦИИ СОПРОТИВЛЕНИЯ ---
    bool is_resistance_correction_;
    int32_t first_v_corrected_uv_;
    int32_t first_ia_ma_;
    int32_t first_ib_ma_;
    int32_t uncorrected_total_resistance_;
    // ----------------------------------------------------

    // Calibration-specific timers and checkers
    timeout_timer timeout_;
    StabilityCheck stability_check_;

public:
    // Constructor - initialize all calibration variables
    MotorCalibration()
        : hfi_gen_(20000),  // Default 20kHz sys freq
          voltage_(0),
          voltage_d_out_(0),
          voltage_q_out_(0),
          holding_voltage_(0),
          hfi_amplitude_(100),
          target_current_(0),
          current_max_(INT32_MIN),
          current_min_(INT32_MAX),
          hfi_accum_(0),
          hfi_periods_(0),
          Ld_f1_(0),
          Ld_f2_(0),
          Lq_f1_(0),
          Lq_f2_(0),
          voltage_increment_(2),
          voltage_limit_(INT16_MAX / 2),
          overcurrent_threshold_(INT16_MAX / 2),
          slow_rotation_angle_target_(0),
          slow_rotation_power_limit_(0),
          measure_current_a_sum_(0),
          measure_current_b_sum_(0),
          inductance_a_sum_(0),
          inductance_b_sum_(0),
          is_resistance_correction_(false),  // Инициализация
          first_v_corrected_uv_(0),          // Инициализация
          first_ia_ma_(0),                   // Инициализация
          first_ib_ma_(0),                   // Инициализация
          uncorrected_total_resistance_(0),  // Инициализация
          timeout_(CALIB_PAUSE_CYCLES),
          stability_check_(50, 240) {}

    // Getters for calibration variables
    int16_t get_voltage() const { return voltage_; }
    int16_t get_voltage_d() const { return voltage_d_out_; }
    int16_t get_voltage_q() const { return voltage_q_out_; }
    int16_t get_voltage_increment() const { return voltage_increment_; }
    int16_t get_voltage_limit() const { return voltage_limit_; }
    int16_t get_slow_rotation_angle_target() const { return slow_rotation_angle_target_; }
    int32_t get_slow_rotation_power_limit() const { return slow_rotation_power_limit_; }

    int16_t get_measure_current_a() const { return (int16_t)(measure_current_a_sum_ >> CAL_OVERSAMPLING_SHIFT); }
    int16_t get_measure_current_b() const { return (int16_t)(measure_current_b_sum_ >> CAL_OVERSAMPLING_SHIFT); }

    // Setters for calibration variables
    void set_voltage(int16_t voltage) { voltage_ = voltage; }

    void accumulate_measure_currents(int16_t current_a, int16_t current_b) {
        measure_current_a_sum_ += current_a;
        measure_current_b_sum_ += current_b;
    }

    void reset_measure_accumulators() {
        measure_current_a_sum_ = 0;
        measure_current_b_sum_ = 0;
    }

    inline void increase_voltage() { voltage_ += voltage_increment_; }

    inline void decrease_voltage() {
        voltage_ -= voltage_increment_;
        if (voltage_ < 0) {
            voltage_ = 0;
        }
    }

    void reset_voltage() {
        voltage_ = 0;
        voltage_d_out_ = 0;
        voltage_q_out_ = 0;
    }

    // ========================================================================
    // CALIBRATION LOGIC
    // ========================================================================
    int32_t max_power = 1000;
    void start_calibration(Motor& motor,
                           MotorSelector& motor_selector,
                           MotorStatus& status_tracker,
                           int16_t& angle_ref,
                           Feedback& feedback,
                           int32_t motor_ohmic_power) {
        status_tracker = MotorStatus::SoftWarmup;

        reset_voltage();
        stability_check_.reset();
        is_resistance_correction_ = false;  // Сбрасываем флаг
        max_power = motor_ohmic_power;
        feedback.power.set_overpower_soft_mw((motor_ohmic_power * 3) / 4);
        feedback.supply.set_overvoltage_soft_norm(feedback.supply.get_voltage_norm() / 2);

        motor_selector.set_motor_type(MotorType_STEP);
        motor_selector.set_phase_pattern(PhasePattern_ABC);

        angle_ref = CAL_ANGLE_45D;
        timeout_.set_timeout(WARMUP_TIMEOUT);

        TP_LOG("[CALIB] Motor calibration started");
    }

    void tick_calibration(int32_t power_norm,
                          int32_t power,
                          Vec2D current_ab,
                          Vec2D current_dq_raw,
                          MotorStatus& status_tracker,
                          Motor& motor,
                          MotorSelector& motor_selector,
                          DriverStatus& setup_status,
                          int16_t& angle_ref,
                          Feedback& feedback) {
        switch (status_tracker) {
            case MotorStatus::SoftWarmup: {
                voltage_d_out_ = voltage_;
                voltage_q_out_ = 0;

                // [OPT] measured_current нужен только здесь — вычисляем только здесь
                int16_t measured_current = abs(current_ab.a + current_ab.b);

                bool power_lim = feedback.power.is_overpower_soft_norm(power_norm);
                bool voltg_lim = feedback.supply.is_overvoltage_soft_norm(voltage_);

                if ((stability_check_.tick(measured_current) && CAL_UNDERPOWER_LIM_NORM < power_norm) || timeout_.tick() == 0) {
                    target_current_ = measured_current;
                    TP_LOG("[CALIB] Warmup complete - current stable");
                    TP_LOG_VAL("  Final voltage: ", voltage_);
                    timeout_.set_timeout(SETTLE_TIMEOUT);
                    status_tracker = MotorStatus::Settle;
                } else {
                    if (power_lim || voltg_lim)
                        decrease_voltage();
                    else
                        increase_voltage();
                }
                break;
            }

            case MotorStatus::Settle: {
                voltage_d_out_ = voltage_;
                voltage_q_out_ = 0;

                if (timeout_.get_timeout() == SETTLE_TIMEOUT) {
                    TP_LOG("[CALIB] Settling before measurement (900 ticks)");
                }
                if (timeout_.tick() == 0) {
                    TP_LOG("[CALIB] Settle complete - starting measurement");
                    status_tracker = MotorStatus::ResistanceMeasure;
                    timeout_.set_timeout(CAL_OVERSAMPLING_POW2);
                }
                break;
            }

            case MotorStatus::ResistanceMeasure: {
                voltage_d_out_ = voltage_;
                voltage_q_out_ = 0;

                // ИЗМЕНЕНИЕ: Добавлен аргумент power для вычисления корректировки
                process_resistance_measure(current_ab, power_norm, voltage_, status_tracker, motor, motor_selector, setup_status,
                                           angle_ref, feedback);
                break;
            }

            case MotorStatus::SlowRotation: {
                // ... (Оригинальный код не менялся)
                voltage_d_out_ = voltage_;
                voltage_q_out_ = 0;

                if (power >= max_power) {
                    TP_LOG("[CALIB] ERROR: OP duting rotation");
                    set_error(status_tracker, motor, setup_status);
                    break;
                }

                if (timeout_.tick() == 0) {
                    angle_ref = slow_rotation_angle_target_;
                    timeout_.set_timeout(SETTLE_TIMEOUT);
                    status_tracker = MotorStatus::Settle;
                } else {
                    int16_t delta = slow_rotation_angle_target_ - angle_ref;
                    angle_ref += delta / (int32_t)(timeout_.get_timeout() + 1);
                }
                break;
            }

            // ... Остальные состояния HFI без изменений ...
            case MotorStatus::InductanceMeasureWarmup: {
                voltage_d_out_ = holding_voltage_;
                voltage_q_out_ = 0;

                if (timeout_.tick() == 0) {
                    TP_LOG("[CALIB] Warmup complete - starting D-axis HFI @ Freq 1");
                    hfi_gen_.set_frequency_target(HFI_FREQ_1);
                    setup_hfi_state(MotorStatus::InductanceMeasureD, status_tracker, motor, INDUCTANCE_MEASURE_CYCLES_F1);
                }
                break;
            }

            case MotorStatus::InductanceMeasureD: {
                process_hfi_axis(current_dq_raw.flux, true, false, status_tracker, motor, setup_status, feedback,
                                 "D-axis F1 complete, switching to D-axis F2", MotorStatus::InductanceMeasureD2,
                                 INDUCTANCE_MEASURE_CYCLES_F2, &Ld_f1_, HFI_FREQ_1);
                break;
            }

            case MotorStatus::InductanceMeasureD2: {
                hfi_gen_.set_frequency_target(HFI_FREQ_2);
                process_hfi_axis(current_dq_raw.flux, true, false, status_tracker, motor, setup_status, feedback,
                                 "D-axis F2 complete, switching to Q-axis F1", MotorStatus::InductanceMeasureQ,
                                 INDUCTANCE_MEASURE_CYCLES_F1, &Ld_f2_, HFI_FREQ_2);
                break;
            }

            case MotorStatus::InductanceMeasureQ: {
                hfi_gen_.set_frequency_target(HFI_FREQ_1);
                // Изменено false, true -> false, false
                process_hfi_axis(current_dq_raw.torque, false, false, status_tracker, motor, setup_status, feedback,
                                 "Q-axis F1 complete, switching to Q-axis F2", MotorStatus::InductanceMeasureQ2,
                                 INDUCTANCE_MEASURE_CYCLES_F2, &Lq_f1_, HFI_FREQ_1);
                break;
            }

            case MotorStatus::InductanceMeasureQ2: {
                hfi_gen_.set_frequency_target(HFI_FREQ_2);
                process_hfi_axis(current_dq_raw.torque, false, true, status_tracker, motor, setup_status, feedback,
                                 "Q-axis F2 complete, finalizing inductance", MotorStatus::Success, 0, &Lq_f2_, HFI_FREQ_2);
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
        current_max_ = INT32_MIN;
        current_min_ = INT32_MAX;
        hfi_accum_ = 0;
        hfi_periods_ = 0;
        hfi_gen_.reset();
    }
    void process_hfi_axis(int16_t measured_current,
                          bool is_d_axis,
                          bool is_final_step,
                          MotorStatus& status_tracker,
                          Motor& motor,
                          DriverStatus& setup_status,
                          Feedback& feedback,
                          const char* log_msg,
                          MotorStatus next_state,
                          uint32_t next_timeout,
                          float* result_L,
                          uint16_t current_freq) {
        int16_t prev_angle = hfi_gen_.get_angle();
        Vec2D hfi_out = hfi_gen_.tick();
        int16_t curr_angle = hfi_gen_.get_angle();

        int32_t hfi_v = ((int32_t)hfi_amplitude_ * hfi_out.cos) >> 15;

        if (is_d_axis) {
            voltage_d_out_ = holding_voltage_ + hfi_v;
            voltage_q_out_ = 0;
        } else {
            voltage_d_out_ = holding_voltage_;
            voltage_q_out_ = hfi_v;
        }

        int16_t max_v = feedback.supply.get_voltage_norm() / 2;
        if (voltage_d_out_ > max_v)
            voltage_d_out_ = max_v;
        if (voltage_d_out_ < -max_v)
            voltage_d_out_ = -max_v;

        if (!is_d_axis && holding_voltage_ + hfi_amplitude_ > max_v) {
            hfi_amplitude_ = max_v - holding_voltage_;
        }

        if (measured_current > current_max_)
            current_max_ = measured_current;
        if (measured_current < current_min_)
            current_min_ = measured_current;

        if (prev_angle > 0 && curr_angle <= 0) {
            int32_t ripple = current_max_ - current_min_;
            if (hfi_periods_ > 0)
                hfi_accum_ += ripple;  // Skip first partial period
            hfi_periods_++;

            // PID-like adaptation for amplitude
            int32_t target_ripple_min = (target_current_ * 20) / 100;
            // [OPT] 25% от target_current_ = деление на 4 = правый сдвиг на 2 бита
            int32_t target_ripple_max = target_current_ >> 2;

            if (ripple < target_ripple_min)
                hfi_amplitude_ += 10;  // Slowed down adaptation for stability during long tests
            else if (ripple > target_ripple_max)
                hfi_amplitude_ -= 10;

            if (holding_voltage_ + hfi_amplitude_ > max_v)
                hfi_amplitude_ = max_v - holding_voltage_;
            if (hfi_amplitude_ < 0)
                hfi_amplitude_ = 0;

            current_max_ = INT32_MIN;
            current_min_ = INT32_MAX;
        }

        if (timeout_.tick() == 0) {
            int32_t avg_ripple = (hfi_periods_ > 1) ? (hfi_accum_ / (hfi_periods_ - 1)) : 0;

            // Calculate accurate Inductance taking Ohmic Resistance into account
            // [OPT] Кэшируем norm2mv(hfi_amplitude_) — вызывается дважды: для v_peak и для лога
            int32_t amp_mv = feedback.supply.norm2mv(hfi_amplitude_);
            float v_peak = (float)amp_mv / 1000.0f;
            float i_peak = (float)feedback.current.norm2ma(avg_ripple / 2) / 1000.0f;
            float omega = 2.0f * 3.14159265f * (float)current_freq;
            float R = (float)motor.get_total_resistance() / 1000.0f;  // in Ohms

            // [OPT] Z вычисляется заранее: устраняем повторное деление v_peak/i_peak в логе
            //       и защищаем от деления на ~0 (i_peak <= 0.001f был вне блока if ниже — это был баг)
            float Z = (i_peak > 0.001f) ? (v_peak / i_peak) : 0.0f;

            if (i_peak > 0.001f) {
                float X_L_sq = (Z * Z) - (R * R);

                if (X_L_sq > 0.0f) {
                    *result_L = (sqrtf(X_L_sq) / omega) * 1e6f;  // uH
                } else {
                    *result_L = 0.0f;  // R dominated Z entirely (shouldn't happen at 1kHz+)
                    TP_LOG("[CALIB] WARNING: Impedance dominated by resistance.");
                }
            } else {
                *result_L = 0.0f;
            }

            TP_LOG(log_msg);
            // [OPT] Используем кэшированный amp_mv вместо повторного вызова norm2mv
            TP_LOG_VALS("  Freq (Hz) / Amp (mV): ", current_freq, amp_mv);
            // [OPT] Используем кэшированный Z вместо повторного деления v_peak / i_peak
            TP_LOG_VALS("  I_peak (mA) / Z (Ohm): ", (int32_t)(i_peak * 1000), (int32_t)Z);
            TP_LOG_VAL("  Calculated L (uH): ", (int32_t)(*result_L));

            if (is_final_step) {
                // Final validation logic
                float diff_d = fabsf(Ld_f1_ - Ld_f2_) / Ld_f1_;
                float diff_q = fabsf(Lq_f1_ - Lq_f2_) / Lq_f1_;

                TP_LOG("[CALIB] --- FINAL INDUCTANCE RESULTS ---");
                TP_LOG_VALS("  Ld @ 1kHz/1.5kHz (uH): ", (int32_t)Ld_f1_, (int32_t)Ld_f2_);
                TP_LOG_VALS("  Lq @ 1kHz/1.5kHz (uH): ", (int32_t)Lq_f1_, (int32_t)Lq_f2_);

                if (diff_d > 0.3f || diff_q > 0.3f) {
                    TP_LOG("[CALIB] WARNING: >30% variance between frequencies. Possible core saturation.");
                }

                // Average the 1kHz and 1.5kHz results
                float final_Ld = (Ld_f1_ + Ld_f2_) / 2.0f;
                float final_Lq = (Lq_f1_ + Lq_f2_) / 2.0f;

                // motor.set_inductance_d((uint32_t)final_Ld);
                // motor.set_inductance_q((uint32_t)final_Lq);
                // motor.set_effective_inductance((uint32_t)((final_Ld + final_Lq) / 2.0f));

                finish_calibration_success(status_tracker, motor, setup_status);
            } else {
                setup_hfi_state(next_state, status_tracker, motor, next_timeout);
            }
        }
    }

    void start_slow_rotation(int16_t target_angle, int32_t power_limit) {
        slow_rotation_angle_target_ = target_angle;
        slow_rotation_power_limit_ = power_limit;
    }

    void set_error(MotorStatus& status_tracker, Motor& motor, DriverStatus& setup_status) {
        TP_LOG("[CALIB] ERROR Motor R L detection");
        setup_status.set_driver_status(OperationStatus::Error);
        status_tracker = MotorStatus::CalibError;
        motor.set_status(MotorStatus::CalibError);
        reset_voltage();
    }

    void start_inductance_measurement(MotorStatus& status_tracker,
                                      Motor& motor,
                                      MotorSelector& motor_selector,
                                      DriverStatus& setup_status,
                                      Feedback& feedback,
                                      int16_t& angle_ref) {
        motor.set_motor_type(motor_selector.get_motor_type());
        motor.set_phase_pattern(motor_selector.get_phase_pattern());
        setup_status.motor_type_not_verified = 0;
        int16_t voltage_norm = feedback.supply.get_voltage_norm();
        setup_status.voltage_unsafe = (voltage_norm <= 0 || voltage_norm >= INT16_MAX);

        holding_voltage_ = voltage_ / 2;
        hfi_amplitude_ = 100;

        angle_ref = 0;  // Align to D-axis

        status_tracker = MotorStatus::InductanceMeasureWarmup;
        timeout_.set_timeout(SETTLE_TIMEOUT);
    }

    void finish_calibration_success(MotorStatus& status_tracker, Motor& motor, DriverStatus& setup_status) {
        status_tracker = MotorStatus::Success;
        motor.set_status(MotorStatus::Success);
        reset_voltage();
    }

    void trigger_resistance_correction(MotorStatus& status_tracker, Motor& motor, Feedback& feedback, int32_t ia, int32_t ib) {
        // Сохраняем токи первого прохода (в mA) для доминантной фазы
        first_ia_ma_ = feedback.current.norm2ma(ia);
        first_ib_ma_ = feedback.current.norm2ma(ib);
        uncorrected_total_resistance_ = motor.get_total_resistance();
        is_resistance_correction_ = true;

        voltage_ >>= 1;

        reset_measure_accumulators();
        status_tracker = MotorStatus::Settle;
        timeout_.set_timeout(SETTLE_TIMEOUT);
    }
    // -----------------------------------------------------------------

    void process_resistance_measure(Vec2D current_ab,
                                    int32_t power_norm,
                                    int32_t voltage,
                                    MotorStatus& status_tracker,
                                    Motor& motor,
                                    MotorSelector& motor_selector,
                                    DriverStatus& setup_status,
                                    int16_t& angle_ref,
                                    Feedback& feedback) {
        // First cycle: check if the counter is initialized (initially equals CAL_OVERSAMPLING_POW2, i.e. 64)
        if (timeout_.get_timeout() == CAL_OVERSAMPLING_POW2) {
            // Reset all current accumulators to start clean averaging
            reset_measure_accumulators();
            // Log the start of the resistance measurement process and the expected number of samples (64)
            TP_LOG("[CALIB] Starting resistance measurement (64 samples)");
        }

        // Accumulate current samples for phases A and B into accumulators for subsequent averaging
        accumulate_measure_currents(current_ab.a, current_ab.b);

        // Decrement the timeout counter (samples) by 1. When it reaches 0, all 64 measurements are collected
        if (timeout_.tick() == 0) {
            // Сos(45°) в Q16 = round(cos(π/4) * 65536)
            static constexpr int32_t COS45_Q16 = 46341;
            int32_t voltage_corrected = (voltage * COS45_Q16) >> 16;  // Voltage amplitude fix
            // Get the averaged phase A and B currents over the entire measurement period
            int32_t ia = get_measure_current_a();
            int32_t ib = get_measure_current_b();

            // [OPT] Cache norm2mv result — used twice: for v_uv and for TP_LOG_VAL
            int32_t v_mv = feedback.supply.norm2mv((uint32_t)voltage_corrected);
            // Compute corrected voltage in microvolts (normalized → mV, then multiply by 1000)
            int32_t v_uv = v_mv * 1000;

            // If the resistance correction flag is set (second measurement pass at half voltage
            // to eliminate dead-time error via differential method)
            if (is_resistance_correction_) {
                const int32_t abs_first_ia = abs(first_ia_ma_);
                const int32_t abs_first_ib = abs(first_ib_ma_);

                // Выбираем доминантную фазу (лучший SNR)
                bool use_phase_a = (abs_first_ia >= abs_first_ib);

                int32_t abs_first_i = use_phase_a ? abs_first_ia : abs_first_ib;
                int32_t second_i_ma = feedback.current.norm2ma(use_phase_a ? ia : ib);
                int32_t abs_second_i = abs(second_i_ma);

                // delta_i должна быть > 0, так как I1 > I2 при V1 > V2
                int32_t delta_i = abs_first_i - abs_second_i;

                int32_t corrected_total_r = uncorrected_total_resistance_;

                // Формула: R_corrected = R_uncorrected * I1 / (2 * delta_i)
                // Напряжение полностью сокращается при точном V2 = V1/2
                if (delta_i > 0 && abs_first_i > 0) {
                    // int64_t умножение — предотвращаем переполнение при больших R * I
                    corrected_total_r = (int32_t)((int64_t)uncorrected_total_resistance_ * abs_first_i / (2 * delta_i));
                }

                motor.set_total_resistance(corrected_total_r);
                is_resistance_correction_ = false;

                start_inductance_measurement(status_tracker, motor, motor_selector, setup_status, feedback, angle_ref);
                return;
            }

            if (CAL_UNDERPOWER_LIM_NORM < power_norm) {  // We have sufficient power, current calculations are reliable
                // Convert averaged normalized currents to milliamps
                int32_t ia_ma = feedback.current.norm2ma(ia);
                int32_t ib_ma = feedback.current.norm2ma(ib);

                // Calculate phase A and B resistance in milliohms via Ohm's law (R = V_uv / I_ma)
                int32_t resistance_a = (ia_ma != 0) ? v_uv / ia_ma : INT32_MAX;
                int32_t resistance_b = (ib_ma != 0) ? v_uv / ib_ma : INT32_MAX;

                // If angle equals 16384 (~+90 electrical degrees) — always the first pass for any motor
                if (angle_ref == CAL_ANGLE_45D) {
                    // Set the timeout counter to 1800 cycles (time allocated for slow rotation)
                    timeout_.set_timeout(ROTATION_TIMEOUT);
                    // Stepper or BLDC motor, first pass: phases should be balanced (within ~15% of average)
                    if (10 * abs(ib - ia) < (ia + ib)) {
                        // Store temporary resistance as arithmetic mean of phases A and B
                        motor.set_total_resistance((resistance_a + resistance_b) / 2);
                        // Slowly rotate the vector by 30° for verification. Target angle 16384 (current + 45° el. (BLDC: +30°))
                        start_slow_rotation(CAL_ANGLE_90D, power_norm);
                        status_tracker = MotorStatus::SlowRotation;
                        return;
                    } else {  // Possibly stepper but in wrong phase pattern, fixing phase commutation
                        set_voltage((int16_t)voltage_corrected);  // Fix voltage level to seamless transfer - mult power by 2
                        // Slowly rotate the vector backwards to angle -24576, rotation by 45°
                        start_slow_rotation(-CAL_ANGLE_135D, power_norm);
                        status_tracker = MotorStatus::SlowRotation;

                        if ((ia >> 2) >= ib) {
                            motor_selector.set_phase_pattern(PhasePattern_CBA);
                            // Set target angle to -16384 (corresponds to CBA connection)
                            angle_ref = -CAL_ANGLE_90D;
                            return;
                        } else if ((ib >> 2) >= ia) {
                            motor_selector.set_phase_pattern(PhasePattern_ACB);
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
                    if (14 * abs((2 * ia) + ib) < ((-ia * 2) + ib)) {
                        // Calculate average resistance accounting for all three phases
                        int32_t avg_r = (motor.get_total_resistance() + resistance_a) / 3;
                        // Write averaged resistance into the motor profile
                        motor.set_total_resistance(avg_r);
                        // Set motor type in the selector as BLDC
                        motor_selector.set_motor_type(MotorType_BLDC);
                        // Log successful BLDC motor detection
                        TP_LOG("[CALIB] Motor type detected: BLDC");

                        trigger_resistance_correction(status_tracker, motor, feedback, ia, ib);
                        return;
                    }
                    // Stepper motor, second pass: check if iA is significantly 16x greater than iB
                    // (Expected: iA — negative dominant, iB close to 0 (any sign). Resistance was already calculated in first
                    // pass)
                    else if ((ib >> 4) > ia) {
                        // Log successful stepper motor detection
                        TP_LOG("[CALIB] Motor type detected: STEPPER");
                        trigger_resistance_correction(status_tracker, motor, feedback, ia, ib);
                        return;
                    }
                }
                // Check if angle is around -24576 (-135 degrees). This is the second pass for Stepper with CBA or ACB connection
                else if (angle_ref == -CAL_ANGLE_135D) {
                    // Check if phases B and C are balanced (difference within ~15% of average)
                    if (10 * abs(ib - ia) < (-ia - ib)) {
                        // Stepper motor: take average resistance of two phases with negative sign (due to vector angle)
                        int32_t avg_r = -(resistance_a + resistance_b) / 2;
                        // Write averaged resistance into the motor profile
                        motor.set_total_resistance(avg_r);
                        // Log stepper motor detection with balanced phases
                        TP_LOG("[CALIB] Motor type detected: STEPPER");

                        trigger_resistance_correction(status_tracker, motor, feedback, ia, ib);
                        return;
                    }
                }
            }
            // If none of checks success - motor error
            motor_selector.set_motor_type(MotorType_Invalid);
            set_error(status_tracker, motor, setup_status);
        }
    }
};