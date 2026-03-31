#pragma once

#include "common.h"
#include <math.h>

struct PositionControllerLADRC {
public:
    float wc;        // Controller bandwidth [рад/с]
    float wo;        // Observer bandwidth [рад/с] (фиксирован, 3-5 * wc)
    float b0;        // Plant gain [об/с² / мА]
    float kd_extra;  // Дополнительный демпфер (0 = выкл, 0.5..2.0 = активный гаситель)
    float dt;

    // LESO states
    float z1 = 0.0f; // Оценка позиции
    float z2 = 0.0f; // Оценка скорости
    float z3 = 0.0f; // Оценка суммарного возмущения
    float u_prev = 0.0f; // Последний выданный и ограниченный ток

    // Float32 position management
    float y_local = 0.0f; // Текущая физическая позиция (в оборотах)
    float r_local = 0.0f; // Текущая целевая позиция (от внешнего планировщика)
    float r_local_filtered = 0.0f; // Внутренняя (сглаженная) целевая позиция для предотвращения рывков

    int32_t prev_current_pos = 0;
    int32_t prev_target_pos = 0;
    bool is_first_tick = true;

    // Максимальная ошибка для ПД-регулятора [об]. 
    // Если реальная ошибка превышает это значение, система "тянет" как пружина постоянной силы.
    float max_tracking_error = 0.5f; 

    // Precomputed observer gains
    float beta1, beta2, beta3;
    // Precomputed controller gains
    float kp_ctrl, kd_ctrl;

    int32_t max_velocity_q16;

    PositionControllerLADRC(float wc, float wo, float b0, float dt, int32_t max_vel_q16, float kd_extra = 0.0f)
        : wc(wc), wo(wo), b0(b0), dt(dt), max_velocity_q16(max_vel_q16), kd_extra(kd_extra) {
        _recompute_gains();
    }

    int32_t tick(int32_t target_pos,
                 int32_t current_pos,
                 float target_vel,
                 float current_vel,
                 float targer_acc,
                 float current_acc,
                 float target_torque,
                 int32_t max_current,
                 int8_t mode) {
        
        const float inv_q16 = 1.0f / 65536.0f;

        // ---------------------------------------------------------
        // 1. ИНИЦИАЛИЗАЦИЯ (Bumpless Transfer)
        // ---------------------------------------------------------
        if (is_first_tick) {
            prev_current_pos = current_pos;
            
            // Bumpless Transfer: При включении мы "замораживаем" начальную 
            // целевую точку там, где мы находимся физически.
            prev_target_pos = current_pos; 

            y_local = 0.0f;
            r_local = 0.0f;
            r_local_filtered = 0.0f;
            
            // Инициализация наблюдателя реальным положением дел
            z1 = 0.0f; 
            z2 = current_vel;
            
            // Инициализация z3: оцениваем текущее возмущение (например гравитацию),
            // чтобы первый выданный ток не вызвал падения или рывка.
            z3 = (target_torque - u_prev) * b0; 

            is_first_tick = false;
        }

        // ---------------------------------------------------------
        // 2. ОБНОВЛЕНИЕ КООРДИНАТ И БОРЬБА С FLOAT-ПЕРЕПОЛНЕНИЕМ
        // ---------------------------------------------------------
        y_local += (float)(current_pos - prev_current_pos) * inv_q16;
        r_local += (float)(target_pos - prev_target_pos) * inv_q16;
        
        prev_current_pos = current_pos;
        prev_target_pos = target_pos;

        // Нормализация координат (Float32 mantissa protection)
        // Сдвигаем ВСЕ пространственные переменные синхронно, чтобы не нарушить математику
        if (fabsf(y_local) > 8.0f) {
            float shift = y_local;
            y_local -= shift;
            r_local -= shift;
            r_local_filtered -= shift;
            z1 -= shift;
        }

        // ---------------------------------------------------------
        // 3. ПРОФИЛИРОВАНИЕ СКОРОСТИ ВОССТАНОВЛЕНИЯ
        // Решает проблему рывков при выходе из stall или при скачке target_pos.
        // ---------------------------------------------------------
        float r_dot_internal = target_vel * inv_q16; 
        
        if (max_velocity_q16 > 0) {
            float max_v = (float)max_velocity_q16 * inv_q16;
            
            // Разница между тем, куда мы хотим (r_local), и тем, 
            // куда смотрит сейчас наш контроллер (r_local_filtered)
            float step_error = r_local - r_local_filtered;
            
            // Вычисляем, какую скорость нужно развить, чтобы догнать цель
            // Жесткость этого "догоняния" можно взять равной wc.
            float recovery_vel = wc * step_error;
            
            // Ограничиваем скорость сближения
            if (recovery_vel > max_v) recovery_vel = max_v;
            else if (recovery_vel < -max_v) recovery_vel = -max_v;
            
            // Плавно подтягиваем внутренний reference к целевому
            r_local_filtered += recovery_vel * dt;
            
            // Внутренний r_dot для контроллера
            r_dot_internal = recovery_vel; 
        } else {
            // Если ограничения нет, внутренний таргет мгновенно равен внешнему
            r_local_filtered = r_local;
        }

        const float r_ddot = targer_acc * inv_q16;

        // ---------------------------------------------------------
        // 4. LESO (Наблюдатель расширенного состояния) с Anti-Windup
        // ---------------------------------------------------------
        float e = z1 - y_local; // Ошибка оценки позиции

        // Вычисление сырой производной z3
        float z3_dot = -beta3 * e;
        
        // Лимитер скорости изменения z3 (Rate Limiter).
        // Чрезвычайно важен для систем с шумным энкодером на низких скоростях.
        // Разрешаем z3 меняться со скоростью изменения полного момента за 2 мс.
        float max_z3_dot = ((float)max_current * b0) / 0.002f; 
        if (z3_dot > max_z3_dot) z3_dot = max_z3_dot;
        else if (z3_dot < -max_z3_dot) z3_dot = -max_z3_dot;

        // Интегрирование LESO (Полунеявный Эйлер)
        z3 += z3_dot * dt;
        
        // AWC-LESO: Наблюдатель использует u_prev (реально выданный, ОБРЕЗАННЫЙ ток), 
        // а не "желаемый" ток. Это гарантирует, что LESO "знает" о насыщении инвертора,
        // предотвращая расхождение состояний z при упоре в препятствие.
        z2 += (z3 - beta2 * e + b0 * u_prev) * dt; 
        z1 += (z2 - beta1 * e) * dt;

        // Физический Anti-windup на состояние z3
        // Возмущение в системе физически не может быть больше, чем максимальная сила мотора.
        float max_dist = (float)max_current * b0;
        if (z3 > max_dist) z3 = max_dist;
        else if (z3 < -max_dist) z3 = -max_dist;

        // ---------------------------------------------------------
        // 5. ЗАКОН УПРАВЛЕНИЯ (PD Controller)
        // ---------------------------------------------------------
        // Используем внутренний, сглаженный reference
        float ep = r_local_filtered - z1; 
        
        // Ограничитель рывка (Tracking Error Limiter):
        // Если мотор механически заблокирован (stall), ep будет расти бесконечно.
        // Ограничивая ep, мы превращаем мотор в "пружину с постоянной силой",
        // которая не копит ветер в D-звене.
        if      (ep >  max_tracking_error) ep =  max_tracking_error;
        else if (ep < -max_tracking_error) ep = -max_tracking_error;

        // Вычисляем ошибку скорости на базе внутреннего r_dot
        float ev = r_dot_internal - z2; 

        // Итоговое номинальное управление U0
        float u0 = kp_ctrl * ep 
                 + kd_ctrl * ev 
                 + r_ddot
                 - kd_extra * z2;  // Активное демпфирование механических резонансов
        
        // Добавление возмущения z3 и feedforward по току
        float u_unsat = (u0 - z3) / b0 + target_torque;

        // ---------------------------------------------------------
        // 6. НАСЫЩЕНИЕ (Output Clamping)
        // ---------------------------------------------------------
        float max_c = (float)max_current;
        float u_sat = u_unsat;
        
        if (u_sat > max_c) u_sat = max_c;
        else if (u_sat < -max_c) u_sat = -max_c;

        // Критично важно для AWC-LESO: сохраняем обрезанный ток для следующего такта!
        u_prev = u_sat;
        
        return (int32_t)u_sat;
    }

    inline void reset(float current_actual_vel = 0.0f) {
        z1 = 0.0f;
        z2 = current_actual_vel;
        z3 = 0.0f;
        u_prev = 0.0f;
        y_local = 0.0f;
        r_local = 0.0f;
        r_local_filtered = 0.0f;
        is_first_tick = true;
    }

    inline void set_bandwidth(float new_wc, float new_wo) {
        wc = new_wc;
        wo = new_wo;
        _recompute_gains();
    }

    inline float get_disturbance() const { return z3; }
    inline float get_velocity_est() const { return z2; }

private:
    inline void _recompute_gains() {
        // Коэффициенты наблюдателя (Размещение полюсов по методу Gao)
        beta1 = 3.0f * wo;
        beta2 = 3.0f * wo * wo;
        beta3 = wo * wo * wo;
        
        // Коэффициенты контроллера
        kp_ctrl = wc * wc;
        kd_ctrl = 2.0f * wc;
    }
};