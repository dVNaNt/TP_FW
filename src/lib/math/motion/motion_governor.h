/**
 * @file motion_governor.h
 * @brief Time-scaling governor for the motion pipeline.
 *
 * Converts a desired feed-rate/speed override into the integer `time_scale`
 * consumed by MotionExecutor::tick(). Positive values play segments forward,
 * negative values reverse them (only allowed when the active segment is
 * reversible), zero freezes trajectory time. The governor implements smooth
 * ramping between scale values so that speed changes do not produce torque
 * steps at the position loop input.
 */

// #pragma once

// #include "../../common.h"
// #include <stdint.h>
// #include <math.h>

// #include "observer.h"         
// #include "motion_executor.h" 


// // ============================================================================
// // MOTION GOVERNOR CLASS
// // ============================================================================

// template <size_t QueueSize>
// class MotionGovernor {
// private:
//     LinearObserver observer;
//     // TrajectoryExecutor<QueueSize> executor;  // TEMPORARILY DISABLED: TrajectoryExecutor not defined

//     // Параметры динамической полосы пропускания (Bandwidth Scheduling)
//     float min_observer_fc;
//     float max_observer_fc;
//     float fc_transition_vel; // Скорость (в об/с), при которой полоса достигает максимума

//     int32_t current_time_scale_q10; // Q10: 1024 = 1.0x (Нормальное время)

//     /// @brief Динамически рассчитывает полосу пропускания LESO на основе целевой скорости
//     inline float schedule_observer_bandwidth(float target_vel) {
//         float abs_vel = fabsf(target_vel);
        
//         // На низких скоростях энкодер шумит сильнее всего из-за квантования.
//         // Поэтому полоса сужается до min_observer_fc. 
//         if (abs_vel >= fc_transition_vel) {
//             return max_observer_fc;
//         }

//         // Линейная интерполяция между min и max в зависимости от скорости
//         float progress = abs_vel / fc_transition_vel;
//         return min_observer_fc + (max_observer_fc - min_observer_fc) * progress;
//     }

// public:
//     MotionGovernor(uint32_t freq_hz, float inertia_b0 = 0.0f) 
//         : observer(freq_hz, inertia_b0), 
//           // executor(),  // TEMPORARILY DISABLED
//           min_observer_fc(50.0f),       // 50 Hz по умолчанию (стоянка)
//           max_observer_fc(250.0f),      // 250 Hz по умолчанию (быстрое вращение)
//           fc_transition_vel(5.0f),      // Переход на максимальную полосу при 5 об/с
//           current_time_scale_q10(1024)  // Время течет нормально (1.0x)
//     {
//     }

//     // --- Настройка параметров ---

//     /// @brief Принудительно задает масштаб течения времени (используется для Override)
//     /// @param scale_q10 1024 = 1.0x, -1024 = Реверс, 0 = Пауза, 512 = 0.5x
//     inline void set_time_scale(int32_t scale_q10) {
//         current_time_scale_q10 = scale_q10;
//     }

//     /// @brief Возвращает текущий (рекомендуемый) масштаб времени для системного таймера
//     inline int32_t get_recommended_time_scale() const {
//         // В будущем здесь можно добавить логику автономного торможения траектории,
//         // если Actual Pos слишком сильно отстает от Target Pos.
//         // Пока мы просто возвращаем заданный извне масштаб.
//         return current_time_scale_q10;
//     }

//     /// @brief Настройка лимитов динамического изменения полосы фильтра
//     inline void set_observer_bandwidth_limits(float min_fc, float max_fc, float transition_vel) {
//         min_observer_fc = min_fc;
//         max_observer_fc = max_fc;
//         fc_transition_vel = transition_vel;
//     }
    
//     // --- Доступ к внутренним модулям ---

//     inline LinearObserver& get_observer() { return observer; }
//     // inline TrajectoryExecutor<QueueSize>& get_executor() { return executor; }  // TEMPORARILY DISABLED

//     // --- Управление очередью траектории ---

//     inline bool enqueue_command(const MotionCommand& cmd) {
//         // return executor.enqueue_command(cmd);  // TEMPORARILY DISABLED
//         return false;
//     }

//     inline void clear_queue() {
//         // executor.clear_queue();  // TEMPORARILY DISABLED
//     }

//     /// @brief Возвращает true, если прямо сейчас выполняется полином перемещения
//     inline bool is_trajectory_active() const {
//         // return executor.is_active(); // Требует наличия геттера is_active() в executor  // TEMPORARILY DISABLED
//         return false;
//     }

//     /// @brief Вспомогательный метод для загрузки плавного профиля на 20 оборотов
//     void inject_dummy_scurve() {
//         // TEMPORARILY DISABLED
//         // MotionCommand cmd;
//         // cmd.displacement = 20 * 65536; // 20 оборотов (Q16.16)
//         // cmd.velocity = 5.0f;           // 5 об/с
//         // cmd.acceleration = 3.0f;       // 3 об/с^2
//         // cmd.jerk = 1.0f;               // 1 об/с^3
//         // cmd.torque = 0.0f;
//         // executor.enqueue_command(cmd);
//     }

//     // --- Основной цикл вычислений ---

//     /// @brief Главный метод Губернатора. Вызывать каждый аппаратный тик контура.
//     /// @param motion_time_us Виртуальное время из системного таймера
//     /// @param real_pos_q32_32 Сырое значение энкодера (накопленное, Q32.32)
//     /// @param applied_torque_ff Момент (ток Iq), примененный на ПРОШЛОМ такте
//     /// @return Структура с Целевыми и Реальными значениями
//     inline __attribute__((always_inline)) ObserverState tick(  // TEMPORARILY CHANGED RETURN TYPE FROM GovernorOutput
//         uint32_t motion_time_us, 
//         int64_t real_pos_q32_32, 
//         float applied_torque_ff) 
//     {
//         // TEMPORARILY SIMPLIFIED - just return observer state
//         // 2. Вычисляем безопасную полосу пропускания LESO по ЦЕЛЕВОЙ скорости
//         float active_fc = schedule_observer_bandwidth(0.0f);  // Use 0 velocity for now

//         // 3. Оценка реальной физики (Actual) через LESO
//         ObserverState obs_state = observer.tick(real_pos_q32_32, applied_torque_ff, active_fc);

//         return obs_state;  // Return observer state directly
//     }

// public:
//         /// @brief Генерирует идеальную 4-фазную S-Curve для короткого перемещения.
//     /// @param current_motion_us Текущее виртуальное время из таймера для бесшовного старта
//     void inject_dummy_scurve(uint32_t current_motion_us) {
//         // TEMPORARILY DISABLED - TrajectoryExecutor not available
//         // Original code commented out
//     }
// };