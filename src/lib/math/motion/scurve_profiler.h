/**
 * @file scurve_profiler.h
 * @brief 7-segment S-Curve motion profile generator with Q16 fixed-point outputs.
 */

#pragma once

#include <cmath>
#include <climits>
#include <algorithm>
#include <cstring> // Для безопасного std::memcpy
#include "motion_command.h"

/**
 * @class SCurveProfiler
 * @brief Time-optimal, jerk-limited kinematic trajectory generator.
 * @details Optimized for embedded systems (MCU). Uses purely 32-bit float 
 * kinematics and Delta-DDA to eliminate `double` precision overhead while 
 * maintaining perfect integer displacement matching.
 */
class SCurveProfiler {
public:
    // ============================================================
    // Public Constants & Types
    // ============================================================

    static constexpr float kQ16Scale = 65536.0f;
    static constexpr float kInvQ16Scale = 1.0f / 65536.0f;
    static constexpr int kMaxSegments = 7;

    // ============================================================
    // Public API
    // ============================================================

    [[nodiscard]] static inline int32_t toQ16(float x) noexcept {
        return static_cast<int32_t>(x * kQ16Scale);
    }

    /**
     * @brief Generates S-Curve profile and directly pushes to the provided callback.
     * @tparam PushCb Callable type, e.g. a lambda capturing `buf_.write` or `executor.push`
     */
    template <typename PushCb>
    static int generate(
        PushCb&& push_cb,
        int32_t displacement,
        uint32_t time_us,
        float max_vel,
        float max_acc,
        float max_jrk, 
        uint16_t torque) noexcept
    {
        if (displacement == 0) [[unlikely]] return 0;
        if (max_vel <= 0.0f || max_acc <= 0.0f || max_jrk <= 0.0f) [[unlikely]] return 0;

        // Локальный буфер на стеке для генерации сегментов перед отправкой
        // Позволяет безопасно сделать фиксацию последнего сегмента (компенсацию потерь).
        MotionCommand out[kMaxSegments];

        const float sg = (displacement < 0) ? -1.0f : 1.0f;
        const float D = static_cast<float>(displacement < 0 ? -displacement : displacement) * kInvQ16Scale;

        const float inv_A = 1.0f / max_acc;
        const float inv_J = 1.0f / max_jrk;

        const float t_j = max_acc * inv_J;
        const float t_j_sq = t_j * t_j;
        const float dv_ramp = max_acc * t_j;

        // --- Лямбда для расчета половины профиля ускорения ---
        auto accel_half = [&](float v_half) noexcept -> HalfProfile {
            if (v_half <= 0.0f) return {0.0f, 0.0f};

            if (v_half >= dv_ramp) {
                const float tc = (v_half - dv_ramp) * inv_A;
                const float d = max_acc * t_j_sq + 1.5f * max_acc * t_j * tc + 0.5f * max_acc * tc * tc;
                return {2.0f * t_j + tc, d};
            } else {
                const float tj2 = sqrtf(v_half * inv_J);
                return {2.0f * tj2, v_half * tj2};
            }
        };

        const HalfProfile full_ah = accel_half(max_vel);
        float v_peak, t_acc_half, d_acc_half;

        if (D >= 2.0f * full_ah.d) {
            v_peak = max_vel;
            t_acc_half = full_ah.t;
            d_acc_half = full_ah.d;
        } else {
            const float d_lim = 2.0f * max_acc * t_j_sq;

            if (D >= d_lim) {
                const float disc = t_j_sq + 4.0f * D * inv_A;
                const float tc = 0.5f * (-3.0f * t_j + sqrtf(disc)); 
                v_peak = dv_ramp + max_acc * fmaxf(0.0f, tc);
            } else {
                const float cbrt_val = fast_cbrtf(D * sqrtf(max_jrk) * 0.5f);
                v_peak = fminf(cbrt_val * cbrt_val, max_vel);
            }

            const HalfProfile ah = accel_half(v_peak);
            t_acc_half = ah.t;
            d_acc_half = ah.d;
        }

        const float d_cruise = fmaxf(0.0f, D - 2.0f * d_acc_half);
        const float t_cruise = (v_peak > 0.0f) ? (d_cruise / v_peak) : 0.0f;

        // --- Масштабирование времени (используем только float) ---
        float scale_t = 1.0f;
        float scale_v = 1.0f;

        if (time_us > 0U) {
            const float t_min = 2.0f * t_acc_half + t_cruise;
            const float t_req = static_cast<float>(time_us) * 1.0e-6f;

            if (t_req > t_min && t_min > 0.0f) {
                scale_t = t_req / t_min;
                scale_v = t_min / t_req;
            }
        }

        const float kEps = 1.0e-5f; // Реалистичный эпсилон для f32
        const bool has_flat_acc = (v_peak >= dv_ramp) && ((v_peak - dv_ramp) * inv_A > kEps);
        const bool has_cruise = (t_cruise > kEps);

        int n = 0;

        // --- Delta-DDA Аккумуляторы (Без double и без абсолютных координат) ---
        float rem_disp_q16 = 0.0f;
        float rem_time_us = 0.0f;
        
        int32_t total_emitted_disp = 0;
        int64_t total_emitted_time = 0;

        // Лямбда генерации и разбиения сегментов
        auto emit = [&](float delta_d, float delta_t, float v_end) noexcept {
            // Вычисляем точный шаг в float, прибавляя остаток от предыдущего шага
            const float step_disp_f = delta_d * sg * kQ16Scale + rem_disp_q16;
            const float step_time_f = delta_t * scale_t * 1000000.0f + rem_time_us;

            // Квантуем в целые числа
            int32_t disp_step = static_cast<int32_t>(step_disp_f);
            int64_t time_step = static_cast<int64_t>(step_time_f);

            // Сохраняем дробные остатки для следующего вызова
            rem_disp_q16 = step_disp_f - static_cast<float>(disp_step);
            rem_time_us = step_time_f - static_cast<float>(time_step);

            if (time_step <= 0) [[unlikely]] return;
            if (n >= kMaxSegments) [[unlikely]] return;

            // Учитываем то, что фактически передаем в буфер
            total_emitted_disp += disp_step;
            total_emitted_time += time_step;

            const float vel = sg * v_end * scale_v;
            constexpr int64_t kMaxDurUs = 0x7FFFFFFFL; 

            // Безопасное разбиение длинных сегментов чисто целочисленной математикой
            while (time_step > kMaxDurUs) [[unlikely]] {
                if (n >= kMaxSegments - 1) break; 

                // Вычисляем пропорцию через int64_t без float-деления
                const int32_t disp_slice = static_cast<int32_t>(
                    (static_cast<int64_t>(disp_step) * kMaxDurUs) / time_step
                );

                out[n++] = {static_cast<uint32_t>(kMaxDurUs), disp_slice, vel};

                time_step -= kMaxDurUs;
                disp_step -= disp_slice;
            }

            if (n < kMaxSegments) {
                out[n++] = {static_cast<uint32_t>(time_step), disp_step, vel};
            }
        };

        // --- Расчет геометрии ---
        float d1 = 0, d2 = 0, d3 = 0;
        float t1 = 0, t2 = 0, t3 = 0;
        float v1 = 0, v2 = 0;

        if (has_flat_acc) {
            t1 = t_j;
            v1 = 0.5f * max_jrk * t_j_sq;
            d1 = max_jrk * t_j_sq * t_j * (1.0f / 6.0f);

            t2 = (v_peak - dv_ramp) * inv_A;
            v2 = v1 + max_acc * t2;
            d2 = t2 * (v1 + 0.5f * max_acc * t2);

            t3 = t_j;
            d3 = d_acc_half - d1 - d2;
        } else if (v_peak > 0.0f) {
            t1 = t_acc_half * 0.5f;
            v1 = v_peak * 0.5f;
            d1 = max_jrk * t1 * t1 * t1 * (1.0f / 6.0f);
            t3 = t1;
            d3 = d_acc_half - d1;
        }

        // --- Генерация сегментов ---
        emit(d1, t1, v1);

        if (has_flat_acc) emit(d2, t2, v2);

        if (has_cruise) {
            emit(d3, t3, v_peak);
            emit(d_cruise, t_cruise, v_peak);
            emit(d3, t3, v_peak - v1);
        } else {
            emit(d3 * 2.0f, t3 * 2.0f, v_peak - v1);
        }

        if (has_flat_acc) emit(d2, t2, v1);

        emit(d1, t1, 0.0f);

        // --- Идеальная подгонка конечных условий ---
        if (n > 0) {
            out[n - 1].end_velocity = 0.0f;

            // Компенсируем потерянные дробные микро-шаги float прямо в последний сегмент
            out[n - 1].displacement += (displacement - total_emitted_disp);

            // Идеально вписываемся в запрашиваемое время
            if (time_us > 0U) {
                const int64_t dur_fix = static_cast<int64_t>(time_us) - total_emitted_time;
                const int64_t new_dur = static_cast<int64_t>(out[n - 1].duration) + dur_fix;

                if (new_dur > 0 && new_dur <= static_cast<int64_t>(UINT32_MAX >> 1)) {
                    out[n - 1].duration = static_cast<uint32_t>(new_dur);
                }
            }
        }

        // --- Отправка готовых сегментов через Callback ---
        for (int i = 0; i < n; ++i) {
            MotionCommandTM tm;
            tm.motion = out[i];
            tm.torque = torque;
            tm.mode = MotionMode::HERMITE;
            push_cb(tm);
        }

        return n;
    }

private:
    struct HalfProfile {
        float t; 
        float d; 
    };

    /**
     * @brief Быстрый кубический корень без UB (Undefined Behavior).
     */
    [[nodiscard]] static inline float fast_cbrtf(float x) noexcept {
        if (x == 0.0f) return 0.0f;
        const float ax = (x < 0.0f) ? -x : x;

        uint32_t i;
        std::memcpy(&i, &ax, sizeof(i)); // Безопасный bit-cast (для C++11 и выше)
        i = i / 3u + 0x2A517D3Cu;
        
        float r;
        std::memcpy(&r, &i, sizeof(r));

        r = (2.0f * r + ax / (r * r)) * 0.33333333f;
        r = (2.0f * r + ax / (r * r)) * 0.33333333f;

        return (x < 0.0f) ? -r : r;
    }
};