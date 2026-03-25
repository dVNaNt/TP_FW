/**
 * @file foc-current.h
 * @brief FOC Current Controller with Feedforward Compensation
 * 
 * This module implements a high-performance Field-Oriented Control (FOC) current controller
 * for PMSM and stepper motors. The controller combines model-based feedforward compensation
 * with PI feedback to achieve fast, accurate current tracking with minimal steady-state error.
 * 
 * Key Features:
 * - Model-based feedforward compensation for resistive, inductive, and back-EMF terms
 * - Cross-coupling compensation between d-q axes
 * - Derivative feedforward for improved transient response
 * - Circular voltage limiting to preserve voltage vector angle
 * - Conditional integrator anti-windup to prevent saturation
 * - Automatic gain tuning based on motor parameters and desired bandwidth
 * 
 * Control Algorithm:
 * The controller implements the following voltage equations in the d-q reference frame:
 * 
 * D-axis: Vd = R*Id + L*dId/dt - ω*L*Iq + PI_correction
 * Q-axis: Vq = R*Iq + L*dIq/dt + ω*L*Id + ω*Ke + PI_correction
 * 
 * Where:
 * - R: Phase resistance [Ω]
 * - L: Phase inductance [H]
 * - Ke: Back-EMF constant [V/(rad/s)]
 * - ω: Electrical angular velocity [rad/s]
 * - Id, Iq: D-axis and Q-axis currents [A]
 * 
 * The feedforward terms compensate for known motor dynamics, while the PI controller
 * corrects for model errors, parameter variations, and disturbances.
 * 
 * Usage Example:
 * ```cpp
 * // Initialize controller with motor parameters
 * FocCurrentController foc(2.4f, 0.0037f, 0.042f, 1.5f, 50.0f, 1200.0f);
 * 
 * // Execute control loop at 20 kHz
 * auto voltage = foc.tick(id_target, iq_target, id_measured, iq_measured,
 *                         bus_voltage, 0.95f, theta_el, omega_el);
 * ```
 * 
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#pragma once
#include <algorithm>  // For std::sqrt in voltage limiting
#include <cmath>      // For floating-point math operations

struct FocCurrentController {
    // ============================================================
    //                 Output Structure
    // ============================================================
    struct VoltageDQ {
        // Output voltage in d-q rotating reference frame
        // These values are then transformed to alpha-beta stationary frame
        // and used for Space Vector Modulation (SVM) or PWM generation
        float u_d = 0.0f;  // D-axis voltage [V]
        float u_q = 0.0f;  // Q-axis voltage [V]
    };

    // ============================================================
    //                 Controller Timing Constants
    // ============================================================
    // Controller is called at fixed 20 kHz frequency (50 μs period)
    static constexpr float kDt = 1.0f / 20000.0f;  // Sampling period: 50 μs
    static constexpr float kInvDt = 20000.0f;      // Inverse sampling period: 1/dt

    // ============================================================
    //                 Motor Model Parameters
    // ============================================================
    float R = 0.0f;      // Phase resistance [Ω]
    float L = 0.0f;      // Phase inductance [H]
    float Ke_el = 0.0f;  // Electrical back-EMF constant [V/(rad_el/s)]
    float K_ff = 0.0f;

    // ============================================================
    //                 PI Controller Gains
    // ============================================================
    // Unified gains for both d and q axes (typically identical for PMSM)
    float kp = 0.0f;  // Proportional gain [V/A]
    float ki = 0.0f;  // Integral gain [V/(A·s)]

    // ============================================================
    //                 PI Controller State
    // ============================================================
    float integrator_d = 0.0f;  // D-axis integral accumulator [V]
    float integrator_q = 0.0f;  // Q-axis integral accumulator [V]

    // ============================================================
    //                 Previous Step Memory
    // ============================================================
    // Stores previous target currents for derivative estimation (di/dt)
    float prev_id_target = 0.0f;  // Previous D-axis target current [A]
    float prev_iq_target = 0.0f;  // Previous Q-axis target current [A]

    // ============================================================
    //                 Initialization
    // ============================================================
    
    // Default constructor - initialize with zero parameters
    // Used when controller is declared but not yet configured
    FocCurrentController() {
        R = 0.0f;      // Zero resistance
        L = 0.0f;      // Zero inductance
        Ke_el = 0.0f;  // Zero back-EMF constant
        kp = 0.0f;     // Zero proportional gain
        ki = 0.0f;     // Zero integral gain
        reset();       // Clear integrator state
    }
    
    // Parameterized constructor - initialize with motor parameters
    // Automatically calculates PI gains based on desired current loop bandwidth
    //
    // @param phase_resistance_ohm Phase resistance [Ω]
    // @param phase_inductance_h Phase inductance [H]
    // @param max_torque_nm Maximum torque at rated current [N·m]
    // @param max_current_a Rated current corresponding to max torque [A]
    // @param pole_pairs Number of motor pole pairs (electrical cycles per mechanical revolution)
    // @param current_loop_bw_hz Desired current loop bandwidth [Hz]
    //
    // Example:
    // FocCurrentController(2.4f, 3.7e-3f, 0.042f, 1.5f, 50.0f, 1200.0f);
    FocCurrentController(float phase_resistance_ohm,
              float phase_inductance_h,
              float max_torque_nm,
              float max_current_a,
              float pole_pairs,
              float current_loop_bw_hz) {
        R = phase_resistance_ohm;
        L = phase_inductance_h;

        // Для модели удобно получить Ke через Kt.
        // В СИ Kt ~= Ke_mech, далее переводим в электрические рад/с.
        const float kt_mech = max_torque_nm / max_current_a;  // Нм/А = В*с/рад_мех
        Ke_el = kt_mech / pole_pairs;                         // В / (рад_эл/с)

        // Базовая настройка PI по объекту L/R.
        const float omega_c = 2.0f * 3.14159265358979f * current_loop_bw_hz;

        // kp_d = L * omega_c;
        // ki_d = R * omega_c;
        // kp_q = L * omega_c;
        // ki_q = R * omega_c;
        kp = L * omega_c;
        ki = R * omega_c;

        reset();
    }

    // ============================================================
    //                 Сброс состояния
    // ============================================================
    void reset() {
        integrator_d = 0.0f;
        integrator_q = 0.0f;
        prev_id_target = 0.0f;
        prev_iq_target = 0.0f;
    }

    // ============================================================
    //                 Основной шаг регулятора
    // ============================================================
    //
    // ВХОДНЫЕ ПАРАМЕТРЫ:
    //
    // id_target_a
    //   Целевой ток по оси d, А.
    //   Это ЗАДАНИЕ. Обычно:
    //   - 0 в обычном режиме,
    //   - отрицательный при field weakening,
    //   - может иметь оффсет для специальных режимов.
    //
    // iq_target_a
    //   Целевой ток по оси q, А.
    //   Это ЗАДАНИЕ. Обычно именно он задаёт момент.
    //
    // id_measured_a
    //   Реально измеренный ток по оси d, А.
    //   Это ИЗМЕРЕНИЕ после преобразования Clarke/Park.
    //
    // iq_measured_a
    //   Реально измеренный ток по оси q, А.
    //   Это ИЗМЕРЕНИЕ после преобразования Clarke/Park.
    //
    // bus_voltage_v
    //   Напряжение питания силовой части, В.
    //   Это напряжение шины, от которого зависит максимально доступный вектор.
    //
    // voltage_limit_fraction
    //   Допустимая доля от полного напряжения, 0...1.
    //   Нужна для запаса под deadtime, ADC window, ошибки модели и т.п.
    //   Пример: 0.90f или 0.95f.
    //
    // theta_el_rad
    //   Электрический угол ротора, рад.
    //   Нужен для обратного Park: dq -> alpha-beta.
    //
    // omega_el_rad_s
    //   Электрическая скорость ротора, рад_эл/с.
    //   Используется в:
    //   - BEMF компенсации,
    //   - перекрёстных связях w*L*i.
    //
    // ВОЗВРАТ:
    //   Напряжение в alpha-beta координатах, В.
    //
    VoltageDQ tick(float id_target_a,
                   float iq_target_a,
                   float id_measured_a,
                   float iq_measured_a,
                   float bus_voltage_v,
                   float voltage_limit_fraction,
                   float theta_el_rad,
                   float omega_el_rad_s) {
        // --------------------------------------------------------
        // 1) Вычисляем максимально допустимый модуль вектора напряжения
        // --------------------------------------------------------
        //
        // Здесь предполагается двухфазная alpha-beta модель, где 32 В —
        // это уже реально доступное фазное напряжение/предел вектора,
        // скорректированный под вашу систему.
        //
        // Если у вас limit_fraction = 0.95, то фактический предел:
        // Umax = bus_voltage_v * 0.95
        //
        const float u_max = bus_voltage_v * voltage_limit_fraction;  // [В]
        const float u_max_sq = u_max * u_max;

        // --------------------------------------------------------
        // 2) Оцениваем производные задания тока
        // --------------------------------------------------------
        //
        // Это feedforward член L * di/dt.
        // Важно: берём производную именно ЗАДАНИЯ, а не измерения,
        // чтобы не усиливать шум АЦП.
        //
        const float did_target_dt = (id_target_a - prev_id_target) * kInvDt;  // [А/с]
        const float diq_target_dt = (iq_target_a - prev_iq_target) * kInvDt;  // [А/с]

        prev_id_target = id_target_a;
        prev_iq_target = iq_target_a;

        // --------------------------------------------------------
        // 3) Feedforward модель по оси d
        // --------------------------------------------------------
        //
        // Уравнение d-оси:
        // ud = R*id + L*did/dt - w*L*iq
        //
        // Что берём:
        // - R*id_target     -> желаемое активное падение
        // - L*did_target_dt -> нужное напряжение для быстрой смены тока
        // - -w*L*iq_measured-> перекрёстная связь, лучше брать от измерения
        //
        const float ud_ff_resistive = R * id_target_a;                  // [В]
        const float ud_ff_dynamic = L * did_target_dt;                  // [В]
        const float ud_ff_cross = -omega_el_rad_s * L * iq_measured_a;  // [В]

        const float ud_ff = ud_ff_resistive + ud_ff_dynamic + ud_ff_cross;

        // --------------------------------------------------------
        // 4) Feedforward модель по оси q
        // --------------------------------------------------------
        //
        // Уравнение q-оси:
        // uq = R*iq + L*diq/dt + w*L*id + w*Ke
        //
        // Последний член w*Ke - это BEMF.
        //
        const float uq_ff_resistive = R * iq_target_a;                 // [В]
        const float uq_ff_dynamic = L * diq_target_dt;                 // [В]
        const float uq_ff_cross = omega_el_rad_s * L * id_measured_a;  // [В]
        const float uq_ff_bemf = omega_el_rad_s * Ke_el;               // [В]

        const float uq_ff = uq_ff_resistive + uq_ff_dynamic + uq_ff_cross + uq_ff_bemf;

        // --------------------------------------------------------
        // 5) Ошибки тока для PI
        // --------------------------------------------------------
        //
        // Это уже чистая ошибка слежения:
        // задание минус измерение.
        //
        const float err_d = id_target_a - id_measured_a;  // [А]
        const float err_q = iq_target_a - iq_measured_a;  // [А]

        // --------------------------------------------------------
        // 6) PI коррекция
        // --------------------------------------------------------
        //
        // PI компенсирует то, что не покрыла модель:
        // - ошибка R/L/Ke,
        // - шум,
        // - deadtime,
        // - насыщение,
        // - паразитные нелинейности.
        //
        const float ud_pi = kp * err_d + integrator_d;  // [В]
        const float uq_pi = kp * err_q + integrator_q;  // [В]

        // Полное напряжение в dq до лимитера.
        float ud = K_ff * ud_ff + ud_pi;
        float uq = K_ff * uq_ff + uq_pi;

        // --------------------------------------------------------
        // 7) Круговой лимит по напряжению
        // --------------------------------------------------------
        //
        // Ограничиваем не по осям отдельно, а по модулю вектора,
        // чтобы сохранить угол напряжения.
        //
        const float u_sq = ud * ud + uq * uq;

        bool saturated = false;
        if (u_sq > u_max_sq) {
            saturated = true;
            const float scale = u_max / std::sqrt(u_sq);
            ud *= scale;
            uq *= scale;
        }

        // --------------------------------------------------------
        // 8) Anti-windup
        // --------------------------------------------------------
        //
        // Простейший и надёжный вариант:
        // интегрируем только если не упёрлись в предел по напряжению.
        //
        if (!saturated) {
            integrator_d += ki * err_d * kDt;
            integrator_q += ki * err_q * kDt;
        }

        VoltageDQ out;
        out.u_d = ud;  // [В]
        out.u_q = uq;  // [В]

        return out;
    }
};
