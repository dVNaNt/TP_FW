/**
 * @file foc-current.h
 * @brief FOC Current Controller with Feedforward Compensation and Dynamic Bandwidth
 *
 * This module implements a high-performance Field-Oriented Control (FOC) current controller
 * for PMSM and stepper motors. The controller combines model-based feedforward compensation
 * with PI feedback to achieve fast, accurate current tracking with minimal steady-state error.
 *
 * Key Features:
 * - Model-based feedforward compensation for resistive, inductive, and back-EMF terms
 * - Cross-coupling compensation between d-q axes
 * - Dynamic Bandwidth (Gain Scheduling): Fc scales with electrical velocity to reduce zero-speed noise
 * - Circular voltage limiting to preserve voltage vector angle
 * - Conditional integrator anti-windup to prevent saturation
 *
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#pragma once
#include <cmath>      // For floating-point math operations
#include "../../common.h"

struct FocCurrentController {
    // ============================================================
    //                 Output Structure
    // ============================================================
    struct VoltageDQ {
        float u_d = 0.0f;  // D-axis voltage [V]
        float u_q = 0.0f;  // Q-axis voltage [V]
    };

    // ============================================================
    //                 Controller Timing Constants
    // ============================================================
    static constexpr float kDt = 1.0f / 20000.0f;  // Sampling period: 50 μs
    static constexpr float kInvDt = 20000.0f;      // Inverse sampling period: 1/dt

    // ============================================================
    //                 Motor Model Parameters
    // ============================================================
    float R = 0.0f;      // Phase resistance [Ω]
    float L = 0.0f;      // Phase inductance [H]
    float Ke_el = 0.0f;  // Electrical back-EMF constant [V/(rad_el/s)]
    
    // Коэффициент участия Feedforward (0.0 = выключен, 1.0 = 100% включен)
    float K_ff = 1.0f;   

    // ============================================================
    //                 Dynamic Bandwidth (Gain Scheduling)
    // ============================================================
    float bw_min_hz = 0.0f;              
    float bw_max_hz = 0.0f;             
    float bw_slope_hz_per_rads = 0.0f; 

    // ============================================================
    //                 PI Controller Gains
    // ============================================================
    float kp = 0.0f;    // Proportional gain [V/A] (Dynamically updated)
    float ki = 0.0f;    // Integral gain [V/(A·s)] (Dynamically updated)
    float k_aw = 0.0f;  // Anti-windup discharge coefficient (ki/kp) (Constant)

    // ============================================================
    //                 PI Controller State
    // ============================================================
    float integrator_d = 0.0f;  // D-axis integral accumulator [V]
    float integrator_q = 0.0f;  // Q-axis integral accumulator [V]

    // ============================================================
    //                 Previous Step Memory
    // ============================================================
    float prev_id_target = 0.0f;  // Previous D-axis target current [A]
    float prev_iq_target = 0.0f;  // Previous Q-axis target current [A]

private:
    static inline float clampf(float x, float lo, float hi) {
        return (x < lo) ? lo : ((x > hi) ? hi : x);
    }

public:
    // ============================================================
    //                 Initialization
    // ============================================================

    // Default constructor
    FocCurrentController() {
        reset();
    }

    // Parameterized constructor with default values for dynamic bandwidth
    FocCurrentController(float phase_resistance_ohm,
                         float phase_inductance_h,
                         float max_torque_nm,
                         float max_current_a,
                         float pole_pairs,
                         float bw_min = 150.0f,
                         float bw_max = 1500.0f,
                         float bw_slope = 1.59155f, // N = 10 (10 / 2pi)
                         float k_feedforward = 1.0f) {
        
        R = phase_resistance_ohm;
        L = phase_inductance_h;

        // Расчет константы противо-ЭДС
        const float kt_mech = max_torque_nm / max_current_a;
        Ke_el = kt_mech / pole_pairs;

        bw_min_hz = bw_min;
        bw_max_hz = bw_max;
        bw_slope_hz_per_rads = bw_slope;
        K_ff = k_feedforward;

        // Предрасчет anti-windup коэффициента.
        // Отношение ki/kp математически не зависит от текущей ширины полосы (Wc).
        k_aw = (L > 0.0f) ? ((R * 0.3f) / L) : 0.0f;

        reset();
    }

    void reset() {
        integrator_d = 0.0f;
        integrator_q = 0.0f;
        prev_id_target = 0.0f;
        prev_iq_target = 0.0f;
    }

    // ============================================================
    //                 Основной шаг регулятора
    // ============================================================
    VoltageDQ tick(int32_t id_target_ma,
                   int32_t iq_target_ma,
                   int32_t id_measured_ma,
                   int32_t iq_measured_ma,
                   int32_t bus_voltage_mv,
                   float voltage_limit_fraction,
                   float theta_el_rad,
                   float omega_el_rad_s) {
        
        (void)theta_el_rad; // Подавление warning

        const float id_target_a = (float)id_target_ma * 0.001f;
        const float iq_target_a = (float)iq_target_ma * 0.001f;
        const float id_measured_a = (float)id_measured_ma * 0.001f;
        const float iq_measured_a = (float)iq_measured_ma * 0.001f;
        const float bus_voltage_v = (float)bus_voltage_mv * 0.001f;

        // --------------------------------------------------------
        // 1) Динамический расчет полосы пропускания (Gain Scheduling)
        // --------------------------------------------------------
        float current_bw = bw_min_hz + bw_slope_hz_per_rads * fabsf(omega_el_rad_s);
        current_bw = clampf(current_bw, bw_min_hz, bw_max_hz);

        const float omega_c = 2.0f * 3.14159265358979f * current_bw;
        
        // Обновляем коэффициенты PI для текущего такта
        kp = L * omega_c ;
        ki = R * omega_c * 0.3f;

        // --------------------------------------------------------
        // 2) Вычисляем максимально допустимый модуль вектора напряжения
        // --------------------------------------------------------
        const float u_max = bus_voltage_v * voltage_limit_fraction;
        const float u_max_sq = u_max * u_max;

        // --------------------------------------------------------
        // 3) Оцениваем производные задания тока
        // --------------------------------------------------------
        const float did_target_dt = (id_target_a - prev_id_target) * kInvDt;
        const float diq_target_dt = (iq_target_a - prev_iq_target) * kInvDt;

        prev_id_target = id_target_a;
        prev_iq_target = iq_target_a;

        // --------------------------------------------------------
        // 4) Feedforward модель по оси d
        // --------------------------------------------------------
        const float ud_ff_resistive = R * id_target_a;
        const float ud_ff_dynamic = L * did_target_dt;
        const float ud_ff_cross = -omega_el_rad_s * L * iq_target_a;

        const float ud_ff_other = ud_ff_cross;

        // --------------------------------------------------------
        // 5) Feedforward модель по оси q
        // --------------------------------------------------------
        const float uq_ff_resistive = R * iq_target_a;
        const float uq_ff_dynamic = L * diq_target_dt;
        const float uq_ff_cross = omega_el_rad_s * L * id_target_a;
        const float uq_ff_bemf = omega_el_rad_s * Ke_el;

        const float uq_ff_other = uq_ff_cross + uq_ff_bemf;

        // --------------------------------------------------------
        // 6) Ошибки тока для PI
        // --------------------------------------------------------
        const float err_d = id_target_a - id_measured_a;
        const float err_q = iq_target_a - iq_measured_a;

        // --------------------------------------------------------
        // 7) PI коррекция и суммирование с FF
        // --------------------------------------------------------
        const float ud_pi = kp * err_d + integrator_d;
        const float uq_pi = kp * err_q + integrator_q;

        const float ud_unlimited = ud_ff_resistive + ud_ff_dynamic + (K_ff * ud_ff_other) + ud_pi;
        const float uq_unlimited = uq_ff_resistive + uq_ff_dynamic + (K_ff * uq_ff_other) + uq_pi;

        float ud_out = ud_unlimited;
        float uq_out = uq_unlimited;

        // --------------------------------------------------------
        // 8) Круговой лимит по напряжению
        // --------------------------------------------------------
        const float u_sq = ud_unlimited * ud_unlimited + uq_unlimited * uq_unlimited;

        if (u_sq > u_max_sq) {
            const float scale = u_max / sqrtf(u_sq + 1e-9f);
            ud_out *= scale;
            uq_out *= scale;
        }

        // --------------------------------------------------------
        // 9) Anti-windup (Back-calculation)
        // --------------------------------------------------------
        const float ud_err_aw = ud_out - ud_unlimited;
        const float uq_err_aw = uq_out - uq_unlimited;

        integrator_d += (ki * err_d + k_aw * ud_err_aw) * kDt;
        integrator_q += (ki * err_q + k_aw * uq_err_aw) * kDt;

        VoltageDQ out;
        out.u_d = ud_out;
        out.u_q = uq_out;

        return out;
    }
};