// #pragma once

// #include "common.h"
// #include <math.h>

// // Константы для конвертации угла
// constexpr float INT16_TO_RAD = (float)(M_PI / 32768.0);
// constexpr float RAD_TO_INT16 = (float)(32768.0 / M_PI);

// struct FOCCommand {
//     float u_q;               // Номинальный Q-ток
//     float u_d;               // Номинальный D-ток 
 
// };

// struct MotorVectorSynthesis {
// public:
//     // Динамический контроллер
//     float wc_max;        // Максимальная полоса контроллера [рад/с] (на высоких скоростях)
//     float wc_min;        // Минимальная полоса контроллера [рад/с] (в статике)
//     float wo;            // Observer bandwidth [рад/с] (фиксирован)
//     float b0;            // Plant gain [об/с² / мА]
//     float kd_extra;      // Дополнительный демпфер
//     float dt;

//     // LESO states
//     float z1 = 0.0f; // Оценка позиции
//     float z2 = 0.0f; // Оценка скорости
//     float z3 = 0.0f; // Оценка суммарного возмущения
//     float u_prev = 0.0f; 

//     // Интегратор ошибки для минимизации tracking error
//     float err_integral = 0.0f;
//     float ki_ctrl;       // Коэффициент интегратора

//     // Float32 position management
//     float y_local = 0.0f; 
//     float r_local = 0.0f; 
//     float r_local_filtered = 0.0f; 

//     int32_t prev_current_pos = 0;
//     int32_t prev_target_pos = 0;
//     bool is_first_tick = true;

//     float max_tracking_error = 0.5f; 

//     // Precomputed observer gains
//     float beta1, beta2, beta3;

//     // Лимиты и пороги
//     float max_velocity;          
//     float min_current_ma;        
//     float speed_threshold_foc;   

//     MotorVectorSynthesis(float wc_max, float wc_min, float wo, float b0, float dt, 
//                          float max_vel, float min_curr, float speed_foc, float kd_extra = 0.0f)
//         : wc_max(wc_max), wc_min(wc_min), wo(wo), b0(b0), dt(dt), 
//           max_velocity(max_vel), min_current_ma(min_curr), speed_threshold_foc(speed_foc), kd_extra(kd_extra) {
        
//         // Настройка интегратора: обычно ki = wc^3 или подбирается вручную.
//         // Здесь берем небольшое значение, чтобы плавно дотягивать ошибку в статике.
//         ki_ctrl = wc_min * 5.0f; 
//         _recompute_observer_gains();
//     }

//     FOCCommand tick(int32_t target_pos,
//                     int32_t current_pos,
//                     float target_vel,
//                     float current_vel,
//                     float target_acc,
//                     float target_torque,
//                     int32_t max_current,
//                     int16_t real_elec_angle, 
//                     int8_t mode) {
        
//         const float inv_q16 = 1.0f / 65536.0f;

//         // ---------------------------------------------------------
//         // 1. ИНИЦИАЛИЗАЦИЯ (Bumpless Transfer)
//         // ---------------------------------------------------------
//         if (is_first_tick) {
//             prev_current_pos = current_pos;
//             prev_target_pos = current_pos; 

//             y_local = 0.0f;
//             r_local = 0.0f;
//             r_local_filtered = 0.0f;
            
//             z1 = 0.0f; 
//             z2 = current_vel;
//             z3 = (target_torque - u_prev) * b0; 
//             err_integral = 0.0f;

//             is_first_tick = false;
//         }

//         // ---------------------------------------------------------
//         // 2. ОБНОВЛЕНИЕ КООРДИНАТ И НОРМАЛИЗАЦИЯ FLOAT
//         // ---------------------------------------------------------
//         y_local += (float)(current_pos - prev_current_pos) * inv_q16;
//         r_local += (float)(target_pos - prev_target_pos) * inv_q16;
        
//         prev_current_pos = current_pos;
//         prev_target_pos = target_pos;

//         // Защита от потери точности float32
//         if (fabsf(y_local) > 8.0f) {
//             float shift = y_local;
//             y_local -= shift;
//             r_local -= shift;
//             r_local_filtered -= shift;
//             z1 -= shift;
//         }

//         // ---------------------------------------------------------
//         // 3. ДИНАМИЧЕСКИЙ WC ПО СКОРОСТИ
//         // ---------------------------------------------------------
//         float abs_speed = fabsf(z2);
//         // Линейная интерполяция wc в зависимости от скорости. 
//         // При скорости 0 об/с -> wc_min. При speed_threshold_foc и выше -> wc_max.
//         float speed_factor = abs_speed / speed_threshold_foc;
//         if (speed_factor > 1.0f) speed_factor = 1.0f;
        
//         float current_wc = wc_min + (wc_max - wc_min) * speed_factor;
//         float kp_ctrl = current_wc * current_wc;
//         float kd_ctrl = 2.0f * current_wc;

//         // ---------------------------------------------------------
//         // 4. ПРОФИЛИРОВАНИЕ СКОРОСТИ КОМПЕНСАЦИИ
//         // ---------------------------------------------------------
//         float r_dot_internal = target_vel * inv_q16; 
        
//         if (max_velocity > 0.0f) {
//             float step_error = r_local - r_local_filtered;
//             float target_recovery_vel = current_wc * step_error; // Используем текущий wc
            
//             if (target_recovery_vel > max_velocity) target_recovery_vel = max_velocity;
//             else if (target_recovery_vel < -max_velocity) target_recovery_vel = -max_velocity;

//             r_local_filtered += target_recovery_vel * dt;
//             r_dot_internal = target_recovery_vel; 
//         } else {
//             r_local_filtered = r_local;
//         }

//         const float r_ddot = target_acc;

//         // ---------------------------------------------------------
//         // 5. LESO (Наблюдатель расширенного состояния) 
//         // ---------------------------------------------------------
//         float e = z1 - y_local; 
        
//         // Лимитируем скорость изменения возмущения для стабильности
//         float z3_dot = -beta3 * e;
//         float max_z3_dot = ((float)max_current * b0) / 0.002f; 
//         if (z3_dot > max_z3_dot) z3_dot = max_z3_dot;
//         else if (z3_dot < -max_z3_dot) z3_dot = -max_z3_dot;

//         // Интегрирование Эйлера
//         z1 += (z2 - beta1 * e) * dt;
//         z2 += (z3 - beta2 * e + b0 * u_prev) * dt; 
//         z3 += z3_dot * dt;

//         // Ограничение абсолютного значения z3 (Windup protection для наблюдателя)
//         float max_dist = (float)max_current * b0;
//         if (z3 > max_dist) z3 = max_dist;
//         else if (z3 < -max_dist) z3 = -max_dist;

//         // ---------------------------------------------------------
//         // 6. ЗАКОН УПРАВЛЕНИЯ (LADRC PD + I + Anti-Windup)
//         // ---------------------------------------------------------
//         float ep = r_local_filtered - z1; 
        
//         if      (ep >  max_tracking_error) ep =  max_tracking_error;
//         else if (ep < -max_tracking_error) ep = -max_tracking_error;

//         float ev = r_dot_internal - z2; 

//         // Интегратор ошибки с Anti-windup (интегрируем только если ошибка мала)
//         // Это полностью убирает статическую ошибку положения.
//         if (fabsf(ep) < (max_tracking_error * 0.5f)) {
//             err_integral += ep * dt;
//             // Лимит интегратора (например, эквивалент 20% от макс тока)
//             float max_integral = ((float)max_current * 0.2f) * b0 / ki_ctrl;
//             if (err_integral > max_integral) err_integral = max_integral;
//             else if (err_integral < -max_integral) err_integral = -max_integral;
//         } else {
//             // Мягкий сброс интегратора при больших отклонениях
//             err_integral *= 0.99f; 
//         }

//         float u0 = kp_ctrl * ep 
//                  + kd_ctrl * ev 
//                  + ki_ctrl * err_integral
//                  + r_ddot
//                  - kd_extra * z2;  
        
//         float u_unsat = (u0 - z3) / b0 + target_torque;

//         float max_c = (float)max_current;
//         float u_sat = u_unsat;
//         if (u_sat > max_c) u_sat = max_c;
//         else if (u_sat < -max_c) u_sat = -max_c;

//         u_prev = u_sat;

//         // ---------------------------------------------------------
//         // 7. МАГНИТНАЯ ПРУЖИНА & FOC BLENDING (БЕЗ СИНТЕТИЧЕСКОГО УГЛА)
//         // ---------------------------------------------------------
//         float blend_margin = speed_threshold_foc * 0.5f; 
//         if (blend_margin < 0.001f) blend_margin = 0.001f;

//         float blend_factor = (abs_speed - (speed_threshold_foc - blend_margin)) / (2.0f * blend_margin);
//         if (blend_factor > 1.0f) blend_factor = 1.0f;
//         else if (blend_factor < 0.0f) blend_factor = 0.0f;

//         // Минимальный ток для намагничивания и жесткости на низких скоростях
//         float I_min_eff = min_current_ma * (1.0f - blend_factor);
        
//         FOCCommand cmd;
        
//         // ВАЖНО: Q-ток трогать нельзя, берем его строго из LADRC без dead-zone inverse.
//         cmd.u_q = u_sat; 
        
//         // Подаем "пружину" (ток удержания) в D-ось. Это сохранит магнитный поток 
//         // и повысит устойчивость двигателя в статике без релейной нестабильности.
//         cmd.u_d = I_min_eff; 
        
//         return cmd;
//     }

//     inline void reset(float current_actual_vel = 0.0f) {
//         z1 = 0.0f;
//         z2 = current_actual_vel;
//         z3 = 0.0f;
//         u_prev = 0.0f;
//         err_integral = 0.0f;
//         y_local = 0.0f;
//         r_local = 0.0f;
//         r_local_filtered = 0.0f;
//         is_first_tick = true;
//     }

//     inline void set_bandwidth(float new_wc_max, float new_wc_min) {
//         wc_max = new_wc_max;
//         wc_min = new_wc_min;
//     }

//     inline float get_disturbance() const { return z3; }
//     inline float get_velocity_est() const { return z2; }

// private:
//     inline void _recompute_observer_gains() {
//         // Наблюдатель (LESO) имеет фиксированную полосу wo
//         beta1 = 3.0f * wo;
//         beta2 = 3.0f * wo * wo;
//         beta3 = wo * wo * wo;
//     }
// };