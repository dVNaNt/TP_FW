#pragma once
#include "../common.h"

struct SystemTimer {
private:
    uint64_t time_us;
    uint32_t prev_ticks;
    uint32_t tick_remainder;
    uint32_t ticks_per_us;

    int64_t motion_time_q10_us;

    // volatile вместо atomic: на ARM 32-бит read/write и так атомарен аппаратно.
    // volatile гарантирует что компилятор ВСЕГДА читает из памяти, а не из регистра.
    // std::atomic мог генерировать LDREX/STREX и создавать временные копии в регистрах.
    volatile int32_t motion_scale;

    // Debug
    int32_t debug_real_time_us_32;
    int32_t debug_motion_time_us_32;

    friend struct Feedback;

    inline __attribute__((always_inline)) void tick(uint32_t current_ticks) {
        // 1. Физическое время (всегда положительное)
        uint32_t delta_ticks = current_ticks - prev_ticks;
        prev_ticks = current_ticks;

        uint32_t total_ticks = delta_ticks + tick_remainder;
        uint32_t delta_us = total_ticks / ticks_per_us;
        tick_remainder = total_ticks - (delta_us * ticks_per_us);
        time_us += delta_us;

        debug_real_time_us_32 = (int32_t)(time_us & 0xFFFFFFFF);

        // 2. Виртуальное время
        int32_t scale = motion_scale;
        if (scale == 0)
            return;

        // Всегда беззнаковое умножение (delta_us * |scale|)
        // Знак применяется отдельно через сложение или вычитание
        uint32_t abs_scale = (scale < 0) ? (uint32_t)(-scale) : (uint32_t)(scale);
        uint64_t increment = (uint64_t)delta_us * abs_scale;  // Гарантированно положительное

        if (scale > 0) {
            motion_time_q10_us += (int64_t)increment;
        } else {
            motion_time_q10_us -= (int64_t)increment;
        }

        debug_motion_time_us_32 = (int32_t)(motion_time_q10_us >> 10);
    }

public:
    SystemTimer(uint32_t fc_hz) {
        ticks_per_us = fc_hz / 1000000;
        time_us = 0;
        prev_ticks = 0;
        tick_remainder = 0;
        motion_time_q10_us = 0;
        motion_scale = 1024;
        debug_real_time_us_32 = 0;
        debug_motion_time_us_32 = 0;
    }

    inline void reset_sys(uint32_t current_ticks) {
        time_us = 0;
        prev_ticks = current_ticks;
        tick_remainder = 0;
        debug_real_time_us_32 = 0;
    }

    inline void reset_motion(uint32_t target_motion_us = 0) {
        motion_time_q10_us = (int64_t)target_motion_us << 10;
        debug_motion_time_us_32 = (int32_t)target_motion_us;
    }

    inline void set_motion_scale(int32_t scale_q10) { motion_scale = scale_q10; }

    inline uint64_t get_us() const { return time_us; }
    inline uint32_t get_motion_us() const { return (uint32_t)(motion_time_q10_us >> 10); }
    inline int32_t get_motion_scale() const { return motion_scale; }

    struct DebugTimeState {
        int32_t real_time;
        int32_t motion_time;
    };

    inline DebugTimeState get_debug_times() const { return {debug_real_time_us_32, debug_motion_time_us_32}; }
};