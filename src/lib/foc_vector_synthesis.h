#pragma once

#include <stdint.h>
#include <cmath>  // для fabsf, sinf, sqrtf и т.д.
#include "common.h"

struct FOCCommand {
    float i_q;
    float i_d;
};

struct MotorVectorSynthesis {
    // ── Параметры Epsilon-Пружины ─────────────────────────────────
    float epsilon_rev = 0.0075f;
    float min_current_ma = 100.0f;

    // ── Защита от перенапряжения (Regen Dump) ─────────────────────
    int32_t ov_start_mv = 50000;
    int32_t ov_end_mv = 55000;
    float max_regen_brake = 4000.0f;  // Максимальный ток Id в mA для сброса мощности (приоритет над max_current)

    // ── Параметры ABY observer с переменной полосой ───────────────
    float dt = 0.00005f;  // 20 kHz
    float obs_fc_min_hz = 50.0f;
    float obs_fc_max_hz = 300.0f;
    float obs_fc_per_rev_s = 150.0f;   // +120 Гц на 1 rev/s target_vel
    float obs_fc_slew_hz_s = 2000.0f;  // ограничение скорости изменения Fc
    float b0_ff_rev_s2_per_ma = 0.0f;  // feedforward accel per mA, может быть 0
    float obs_acc_limit_rev_s2 = 200000.0f;

    // ── Параметры демпфирования ───────────────────────────────────
    float damping_gain = 0.25f;  // Vel controller limit
    float v_soft_rev_s = 0.25f;  // Target speed epsilon (% of target speed)

private:
    static inline float clampf(float x, float lo, float hi) { return (x < lo) ? lo : ((x > hi) ? hi : x); }
    static inline float signf(float x) { return (x > 0.0f) ? 1.0f : ((x < 0.0f) ? -1.0f : 0.0f); }

    // ABY state
    float obs_pos_rev = 0.0f;
    float obs_vel_rev_s = 0.0f;
    float obs_acc_rev_s2 = 0.0f;
    float obs_fc_hz = 50.0f;

    // Последний реально примененный ток для FF в предикторе
    float last_applied_iq_ma = 0.0f;

    bool is_first_tick = true;

public:
    MotorVectorSynthesis() {}

    float getEstimatedPositionRev() const { return obs_pos_rev; }
    float getEstimatedVelocityRevS() const { return obs_vel_rev_s; }
    float getEstimatedAccelerationRevS2() const { return obs_acc_rev_s2; }
    float getObserverFcHz() const { return obs_fc_hz; }

    FOCCommand tick(int32_t current_pos,
                    int32_t target_pos,
                    float target_vel,
                    int32_t max_current,
                    int32_t supply_voltage_mv,
                    bool is_volt_saturated) {
        const float inv_q16 = 1.0f / 65536.0f;
        const float max_c = ((float)max_current < min_current_ma) ? min_current_ma : (float)max_current;
        const float meas_pos_rev = (float)current_pos * inv_q16;

        // ====================================================================
        // 0. Инициализация
        // ====================================================================
        if (is_first_tick) {
            obs_pos_rev = meas_pos_rev;
            obs_vel_rev_s = 0.0f;
            obs_acc_rev_s2 = 0.0f;
            obs_fc_hz = obs_fc_min_hz;
            last_applied_iq_ma = 0.0f;
            is_first_tick = false;
        }

        // ====================================================================
        // 1. Dynamic ABY observer (Кинематическое ядро)
        // ====================================================================
        float fc_target_hz = obs_fc_min_hz + obs_fc_per_rev_s * fabsf(target_vel);
        fc_target_hz = clampf(fc_target_hz, obs_fc_min_hz, obs_fc_max_hz);

        float max_df = obs_fc_slew_hz_s * dt;
        float df = clampf(fc_target_hz - obs_fc_hz, -max_df, max_df);
        obs_fc_hz += df;

        const float w = 2.0f * (float)M_PI * obs_fc_hz;
        const float l1 = 3.0f * w;
        const float l2 = 3.0f * w * w;
        const float l3 = w * w * w;

        const float acc_ff = b0_ff_rev_s2_per_ma * last_applied_iq_ma;
        const float err = meas_pos_rev - obs_pos_rev;

        obs_pos_rev += dt * (obs_vel_rev_s + l1 * err);
        obs_vel_rev_s += dt * (obs_acc_rev_s2 + acc_ff + l2 * err);
        obs_acc_rev_s2 += dt * (l3 * err);

        obs_acc_rev_s2 = clampf(obs_acc_rev_s2, -obs_acc_limit_rev_s2, obs_acc_limit_rev_s2);

        // ====================================================================
        // 2. Управление: Пружина + Демпфер
        // ====================================================================
        float ep = (float)(target_pos - current_pos) * inv_q16;

        float t_linear = clampf(ep / (epsilon_rev + 1e-9f), -1.0f, 1.0f);
        // Approx smooth function of sign(x) * sin(x)^2
        float sin_pow2_fast = t_linear * fabs(t_linear) * (3.0f - 2.0f * t_linear);

        float Iq_spring = max_c * sin_pow2_fast;

        float vel_err = obs_vel_rev_s - target_vel;

        float norm_vel = vel_err / (v_soft_rev_s + 0.05f);
        float fast_tanh = norm_vel / (1.0f + fabsf(norm_vel));

        float Iq_damp = -max_c * damping_gain * fast_tanh * fast_tanh * signf(fast_tanh);

        // Предварительно вычисляем запрошенный момент
        float Iq_unlimited = Iq_spring + Iq_damp;
        float Iq = Iq_unlimited;

        // ====================================================================
        // 2.5 CASCADE ANTI-WINDUP (Clamping) по флагу от FocCurrentController
        // ====================================================================
        if (is_volt_saturated) {
            // Если напряжение уже на пределе, мы не можем увеличивать модуль требуемого тока.
            // Clamping разрешает уменьшать ток (чтобы затормозить или скорректировать перелет),
            // но запрещает наращивать его в сторону еще большего насыщения.
            if ((Iq_unlimited > 0.0f && Iq_unlimited > last_applied_iq_ma) ||
                (Iq_unlimited < 0.0f && Iq_unlimited < last_applied_iq_ma)) {
                // Если мы пытаемся продавить больший ток, чем на прошлом шаге, обрезаем его
                Iq = last_applied_iq_ma;
            }
        }

        float Id = 0.0f;
        float Iq_abs = fabsf(Iq);
        if (Iq_abs < min_current_ma) {
            Id = sqrtf(min_current_ma * min_current_ma - Iq_abs * Iq_abs);
        }

        // ====================================================================
        // 3. Защита от перенапряжения (Гладкий Regen Dump)
        // ====================================================================
        float ov_factor = 0.0f;
        if (supply_voltage_mv > ov_start_mv) {
            ov_factor = (float)(supply_voltage_mv - ov_start_mv) / (float)(ov_end_mv - ov_start_mv);
            ov_factor = clampf(ov_factor, 0.0f, 1.0f);
        }

        if (ov_factor > 0.0f) {
            // 1. Плавно уменьшаем доступный лимит для момента (Iq)
            float current_max_iq = max_c * (1.0f - ov_factor);

            // Ограничиваем запрошенный момент новым уменьшенным лимитом
            Iq = signf(Iq) * clampf(fabsf(Iq), 0.0f, current_max_iq);

            // 2. Вычисляем динамический лимит общего тока
            float current_max_total = max_c + (max_regen_brake - max_c) * ov_factor;

            // 3. Вычисляем доступное место для Id
            float available_id_sq = current_max_total * current_max_total - Iq * Iq;
            float available_id = (available_id_sq > 0.0f) ? sqrtf(available_id_sq) : 0.0f;

            // 4. Дамп-ток
            float target_id_dump = -max_regen_brake * ov_factor;
            if (fabsf(target_id_dump) > available_id) {
                target_id_dump = signf(target_id_dump) * available_id;
            }

            if (fabsf(target_id_dump) > fabsf(Id)) {
                Id = target_id_dump;
            }
        } else {
            // ====================================================================
            // 4. Ограничение амплитуды кругового вектора (Нормальный режим)
            // ====================================================================
            float amp_sq = Iq * Iq + Id * Id;
            if (amp_sq > max_c * max_c) {
                float k = max_c / sqrtf(amp_sq + 1e-9f);
                Iq *= k;
                Id *= k;
            }
        }

        // Сохраняем реально примененный Iq для FF наблюдателя и для Clamping на следующем шаге
        last_applied_iq_ma = Iq;

        FOCCommand cmd;
        cmd.i_q = Iq;
        cmd.i_d = Id;
        return cmd;
    }
};