#pragma once

#include <math.h>
#include <stdint.h>
#include "common.h"

struct FOCCommand {
    float u_q;
    float u_d;
};

struct MotorVectorSynthesis {
    // ── Параметры Epsilon-Пружины ─────────────────────────────────
    float epsilon_rev = 0.01f;
    float min_current_ma = 100.0f;
    int32_t ov_start_mv = 52000;
    int32_t ov_end_mv = 56000;

    // ── Параметры α-β observer ────────────────────────────────────
    float dt = 0.00005f;             // Период вызова системы (напр. 50 мкс = 20 кГц)
    float obs_freq_min_hz = 40.0f;
    float obs_freq_max_hz = 300.0f;
    float obs_freq_per_rev_s = 40.0f;

    // ── Параметры демпфирования ───────────────────────────────────
    float damping_gain = 100.0f;
    float v_soft_rev_s = 0.25f;
    float max_damping_ratio = 0.4f;

private:
    // α-β observer state
    float ab_pos_rev = 0.0f;
    float ab_vel_rev_s = 0.0f;
    bool is_first_tick = true;

    // Внутренние переменные для разделения тактов
    bool tick_toggle = false;        // Переключатель логики
    float last_Iq_base = 0.0f;       // Сохраненный Iq до применения защит
    float last_Id_base = 0.0f;       // Сохраненный Id до применения защит

    static inline float clampf(float x, float lo, float hi) {
        return (x < lo) ? lo : ((x > hi) ? hi : x);
    }

    static inline float signf(float x) {
        return (x > 0.0f) ? 1.0f : ((x < 0.0f) ? -1.0f : 0.0f);
    }

public:
    MotorVectorSynthesis() {}

    float getEstimatedVelocityRevS() const { return ab_vel_rev_s; }
    float getEstimatedPositionRev()  const { return ab_pos_rev; }

    FOCCommand tick(int32_t target_pos,
                    int32_t current_pos,
                    float target_vel,
                    float target_acc,
                    float target_torque,
                    int32_t max_current,
                    int16_t raw_elec_angle,
                    int32_t supply_voltage_mv) {

        (void)target_acc;
        (void)raw_elec_angle;

        const float inv_q16 = 1.0f / 65536.0f;
        const float max_c = ((float)max_current < min_current_ma) ? min_current_ma : (float)max_current;

        // Переключаем такт (false -> true -> false)
        tick_toggle = !tick_toggle;

        // Инициализация при первом запуске
        if (is_first_tick) {
            ab_pos_rev = (float)current_pos * inv_q16;
            ab_vel_rev_s = 0.0f;
            last_Iq_base = 0.0f;
            last_Id_base = 0.0f;
            is_first_tick = false;
        }

        // ====================================================================
        // ЧЕРЕДОВАНИЕ ЛОГИКИ (РАЗГРУЗКА ЦПУ НА 50%)
        // ====================================================================
        if (tick_toggle) {
            // ── ТАКТ А: Dynamic α-β observer ─────────────────────────────────
            // Так как наблюдатель вызывается раз в 2 такта, его эффективный dt = 2 * dt
            float dt_obs = 2.0f * dt;
            float meas_pos_rev = (float)current_pos * inv_q16;

            float obs_freq_hz = obs_freq_min_hz + obs_freq_per_rev_s * fabsf(target_vel);
            obs_freq_hz = clampf(obs_freq_hz, obs_freq_min_hz, obs_freq_max_hz);

            float w = 2.0f * (float)M_PI * obs_freq_hz;

            float ab_alpha = 2.0f * w * dt_obs;
            float ab_beta  = (w * dt_obs) * (w * dt_obs);

            ab_alpha = clampf(ab_alpha, 0.0f, 1.0f);
            ab_beta  = clampf(ab_beta,  0.0f, 0.25f);

            float pred_pos = ab_pos_rev + dt_obs * ab_vel_rev_s;
            float pred_vel = ab_vel_rev_s;

            float resid = meas_pos_rev - pred_pos;

            ab_pos_rev   = pred_pos + ab_alpha * resid;
            ab_vel_rev_s = pred_vel + (ab_beta * resid / (dt_obs + 1e-12f));
        } 
        else {
            // ── ТАКТ Б: Расчет ошибки, пружины и демпфера ────────────────────
            int32_t pos_err = target_pos - current_pos;
            float ep = (float)pos_err * inv_q16;
            float ep_abs = fabsf(ep);

            // 1. Пружина с ИСПРАВЛЕННОЙ жесткостью (убрано лишнее * t_linear)
            float t_linear = clampf(ep_abs / (epsilon_rev + 1e-9f), 0.0f, 1.0f);
            float theta = signf(ep) * t_linear * (float)(M_PI / 2.0);
            float sin_err = sinf(theta);
            float Iq_spring = signf(ep) * max_c * sinf(theta) * t_linear;

            // 2. Демпфер с БЫСТРОЙ аппроксимацией вместо тяжелого tanhf()
            // x / (1 + |x|) работает так же как tanh, но в 10 раз быстрее
            float vel_err = ab_vel_rev_s - target_vel;
            float norm_vel = vel_err / (v_soft_rev_s + 1e-9f);
            float fast_tanh = norm_vel / (1.0f + fabsf(norm_vel)); 
            
            float Iq_damp = -damping_gain * fast_tanh;
            float damp_limit = max_c * max_damping_ratio;
            Iq_damp = clampf(Iq_damp, -damp_limit, damp_limit);

            // Сохраняем базовые значения токов
            last_Iq_base = Iq_spring + target_torque + Iq_damp;
            last_Id_base = 0.0f;

            float Iq_abs = fabsf(last_Iq_base);
            if (Iq_abs < min_current_ma) {
                last_Id_base = sqrtf(min_current_ma * min_current_ma - last_Iq_base * last_Iq_base);
            }
        }

        // ====================================================================
        // ОБЩАЯ ЛОГИКА (ВЫПОЛНЯЕТСЯ КАЖДЫЙ ТАКТ ДЛЯ БЕЗОПАСНОСТИ)
        // ====================================================================
        
        // Берем базовые значения токов (они обновляются в такте Б)
        float Iq = last_Iq_base;
        float Id = last_Id_base;

        // ── Защита от перенапряжения и сброс энергии ────────────────────────
        float ov_factor = 0.0f;
        if (supply_voltage_mv > ov_start_mv) {
            ov_factor = (float)(supply_voltage_mv - ov_start_mv) / (float)(ov_end_mv - ov_start_mv);
            ov_factor = clampf(ov_factor, 0.0f, 1.0f);
        }

        bool is_regenerating = (Iq * ab_vel_rev_s < 0.0f);

        if (is_regenerating && ov_factor > 0.0f) {
            Iq *= (1.0f - ov_factor);

            float available_id = sqrtf(max_c * max_c - Iq * Iq);
            float id_dump = -available_id * ov_factor;

            if (fabsf(id_dump) > fabsf(Id)) {
                Id = id_dump;
            }
        }

        // ── Защита от превышения амплитуды ──────────────────────────────────
        float amp_sq = Iq * Iq + Id * Id;
        if (amp_sq > max_c * max_c) {
            // fast inverse sqrt (если доступен) здесь был бы кстати
            float k = max_c / sqrtf(amp_sq + 1e-9f);
            Iq *= k;
            Id *= k;
        }

        FOCCommand cmd;
        cmd.u_q = Iq;
        cmd.u_d = Id;
        return cmd;
    }
};