#pragma once

#include "../../common.h"
#include <stdint.h>
#include <math.h>

#include "observer.h"         
#include "motion_executor.h" 


// ============================================================================
// MOTION GOVERNOR CLASS
// ============================================================================

template <size_t QueueSize>
class MotionGovernor {
private:
    LinearObserver observer;
    TrajectoryExecutor<QueueSize> executor;

    // Параметры динамической полосы пропускания (Bandwidth Scheduling)
    float min_observer_fc;
    float max_observer_fc;
    float fc_transition_vel; // Скорость (в об/с), при которой полоса достигает максимума

    int32_t current_time_scale_q10; // Q10: 1024 = 1.0x (Нормальное время)

    /// @brief Динамически рассчитывает полосу пропускания LESO на основе целевой скорости
    inline float schedule_observer_bandwidth(float target_vel) {
        float abs_vel = fabsf(target_vel);
        
        // На низких скоростях энкодер шумит сильнее всего из-за квантования.
        // Поэтому полоса сужается до min_observer_fc. 
        if (abs_vel >= fc_transition_vel) {
            return max_observer_fc;
        }

        // Линейная интерполяция между min и max в зависимости от скорости
        float progress = abs_vel / fc_transition_vel;
        return min_observer_fc + (max_observer_fc - min_observer_fc) * progress;
    }

public:
    MotionGovernor(uint32_t freq_hz, float inertia_b0 = 0.0f) 
        : observer(freq_hz, inertia_b0), 
          executor(),
          min_observer_fc(50.0f),       // 50 Hz по умолчанию (стоянка)
          max_observer_fc(250.0f),      // 250 Hz по умолчанию (быстрое вращение)
          fc_transition_vel(5.0f),      // Переход на максимальную полосу при 5 об/с
          current_time_scale_q10(1024)  // Время течет нормально (1.0x)
    {
    }

    // --- Настройка параметров ---

    /// @brief Принудительно задает масштаб течения времени (используется для Override)
    /// @param scale_q10 1024 = 1.0x, -1024 = Реверс, 0 = Пауза, 512 = 0.5x
    inline void set_time_scale(int32_t scale_q10) {
        current_time_scale_q10 = scale_q10;
    }

    /// @brief Возвращает текущий (рекомендуемый) масштаб времени для системного таймера
    inline int32_t get_recommended_time_scale() const {
        // В будущем здесь можно добавить логику автономного торможения траектории,
        // если Actual Pos слишком сильно отстает от Target Pos.
        // Пока мы просто возвращаем заданный извне масштаб.
        return current_time_scale_q10;
    }

    /// @brief Настройка лимитов динамического изменения полосы фильтра
    inline void set_observer_bandwidth_limits(float min_fc, float max_fc, float transition_vel) {
        min_observer_fc = min_fc;
        max_observer_fc = max_fc;
        fc_transition_vel = transition_vel;
    }
    
    // --- Доступ к внутренним модулям ---

    inline LinearObserver& get_observer() { return observer; }
    inline TrajectoryExecutor<QueueSize>& get_executor() { return executor; }

    // --- Управление очередью траектории ---

    inline bool enqueue_command(const MotionCommand& cmd) {
        return executor.enqueue_command(cmd);
    }

    inline void clear_queue() {
        executor.clear_queue();
    }

    /// @brief Возвращает true, если прямо сейчас выполняется полином перемещения
    inline bool is_trajectory_active() const {
        return executor.is_active(); // Требует наличия геттера is_active() в executor
    }

    /// @brief Вспомогательный метод для загрузки плавного профиля на 20 оборотов
    void inject_dummy_scurve() {
        MotionCommand cmd;
        cmd.displacement = 20 * 65536; // 20 оборотов (Q16.16)
        cmd.velocity = 5.0f;           // 5 об/с
        cmd.acceleration = 3.0f;       // 3 об/с^2
        cmd.jerk = 1.0f;               // 1 об/с^3
        cmd.torque = 0.0f;
        executor.enqueue_command(cmd);
    }

    // --- Основной цикл вычислений ---

    /// @brief Главный метод Губернатора. Вызывать каждый аппаратный тик контура.
    /// @param motion_time_us Виртуальное время из системного таймера
    /// @param real_pos_q32_32 Сырое значение энкодера (накопленное, Q32.32)
    /// @param applied_torque_ff Момент (ток Iq), примененный на ПРОШЛОМ такте
    /// @return Структура с Целевыми и Реальными значениями
    inline __attribute__((always_inline)) GovernorOutput tick(
        uint32_t motion_time_us, 
        int64_t real_pos_q32_32, 
        float applied_torque_ff) 
    {
        // 1. Расчет идеальной траектории (Target)
        TrajectoryState target_state = executor.tick(motion_time_us, real_pos_q32_32, current_time_scale_q10);
        
        // 2. Вычисляем безопасную полосу пропускания LESO по ЦЕЛЕВОЙ скорости
        // Целевая скорость идеальна и предсказывает движение до того, как мотор физически стронется
        float active_fc = schedule_observer_bandwidth(target_state.vel);

        // 3. Оценка реальной физики (Actual) через LESO
        ObserverState obs_state = observer.tick(real_pos_q32_32, applied_torque_ff, active_fc);

        // 4. Упаковка результатов
        GovernorOutput out;

        // --- Запланированные (Целевые) параметры ---
        out.target.pos_q32_32 = target_state.pos_q32_32;
        out.target.vel = target_state.vel;
        out.target.acc = target_state.acc;
        out.target.jerk = target_state.jerk;
        out.target.torque = target_state.torque; // Feed-forward

        // --- Оцененные (Реальные) параметры ---
        out.actual.pos_q32_32 = obs_state.pos_q32;
        out.actual.vel = obs_state.vel;
        out.actual.acc = obs_state.acc;
        out.actual.jerk = 0.0f; // LESO 3-го порядка не оценивает реальный рывок
        out.actual.torque = obs_state.dist; // Динамическое возмущение (Load Disturbance / Трение)

        return out;
    }

public:
        /// @brief Генерирует идеальную 4-фазную S-Curve для короткого перемещения.
    /// @param current_motion_us Текущее виртуальное время из таймера для бесшовного старта
    void inject_dummy_scurve(uint32_t current_motion_us) {
        // --- Физические параметры ---
        // Distance = 20, J = 1. Amax=3 и Vmax=5 недостижимы на этой дистанции.
        // Математика 4-фазного рывка: Distance = 2 * J * t_j^3
        // t_j = cbrt(Distance / (2 * J)) = cbrt(10) = 2.15443469 сек.
        
        float tj = cbrtf(10.0f); // Длительность одной фазы рывка
        uint32_t dt_us = (uint32_t)(tj * 1000000.0f); // Длительность в микросекундах
        uint32_t t_now = current_motion_us; 

        MotionCommand cmd;
        cmd.torque = 500.0f; // Передаем базовый Feed-forward момент (torque)
        
        // ====================================================================
        // Фаза 1: Наращивание ускорения (J > 0)
        // ====================================================================
        cmd.start_time_us = t_now;
        t_now += dt_us;
        cmd.end_time_us = t_now;
        
        cmd.jerk = 1.0f;
        cmd.acceleration = 0.0f;
        cmd.velocity = 0.0f;
        
        // Математика: 1/6 * J * t^3 = 10 / 6 = 1.666666... оборотов
        cmd.displacement = (int32_t)((1.0f / 6.0f * tj * tj * tj) * 65536.0f); // 109226 в Q16.16
        executor.enqueue_command(cmd);

        // ====================================================================
        // Фаза 2: Сброс ускорения до нуля (J < 0). Разгон до пиковой скорости.
        // ====================================================================
        cmd.start_time_us = t_now;
        t_now += dt_us;
        cmd.end_time_us = t_now;
        
        cmd.jerk = -1.0f;
        cmd.acceleration = tj;            // Пиковое ускорение: ~2.15 об/с^2
        cmd.velocity = 0.5f * tj * tj;    // Скорость в начале сегмента: ~2.32 об/с
        
        // Математика: 5/6 * t^3 = 50 / 6 = 8.333333... оборотов
        cmd.displacement = (int32_t)((tj * tj * tj - 1.0f / 6.0f * tj * tj * tj) * 65536.0f); // 546133 в Q16.16
        executor.enqueue_command(cmd);

        // ====================================================================
        // Фаза 3: Наращивание торможения (J < 0)
        // ====================================================================
        cmd.start_time_us = t_now;
        t_now += dt_us;
        cmd.end_time_us = t_now;
        
        cmd.jerk = -1.0f;
        cmd.acceleration = 0.0f;
        cmd.velocity = tj * tj;           // Пиковая скорость в центре профиля: ~4.64 об/с
        
        cmd.displacement = 546133;        // Полностью симметрично Фазе 2
        executor.enqueue_command(cmd);

        // ====================================================================
        // Фаза 4: Сброс торможения до полной остановки (J > 0)
        // ====================================================================
        cmd.start_time_us = t_now;
        t_now += dt_us;
        cmd.end_time_us = t_now;
        
        cmd.jerk = 1.0f;
        cmd.acceleration = -tj;           // Пиковое торможение: ~ -2.15 об/с^2
        cmd.velocity = 0.5f * tj * tj;    // Скорость: ~2.32 об/с
        
        // АБСОЛЮТНАЯ ТОЧНОСТЬ Q16.16:
        // Чтобы избежать микроскопической ошибки округления Float на стыках, 
        // последнему сегменту отдаем жестко вычисленный остаток до ровно 20.0 оборотов.
        // 20 оборотов = 20 * 65536 = 1310720.
        cmd.displacement = 1310720 - (109226 + 546133 + 546133); // 109228
        
        executor.enqueue_command(cmd);
    }
};