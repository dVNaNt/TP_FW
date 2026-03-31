#pragma once  // Ensures this header file is included only once during compilation

#include "../../common.h"  // Includes common project definitions and hardware dependencies

/// @brief Represents the instantaneous kinematic and dynamic state at a specific tick
struct MotionState {
    int32_t pos;  // Position (e.g., Q16.16 format for turns/angle)
    float vel;    // Velocity (RPS)
    float acc;    // Acceleration (RPS/s)
    float trq;    // Torque/Current (Amperes)

    MotionState() : pos(0), vel(0.0f), acc(0.0f), trq(0.0f) {}

    MotionState(int32_t p, float v, float a, float q = 0.0f) : pos(p), vel(v), acc(a), trq(q) {}
};

/// @brief Represents a mathematical piece of a curve (spline or S-curve phase)
struct MotionSegment {
    uint32_t t;    // Segment duration in microseconds
    uint8_t mode;  // Control mode bitmask (DO_POS/VEL/ACC/TRQ)

    int32_t pos;  // Starting position (Q16.16 where 16 bits of full rotations + 16 bits of angle)
    float vel;    // Starting velocity (RPS)
    float acc;    // Starting acceleration (RPS/s)
    float jrk;    // Constant jerk applied during this segment (RPS/s^3)
    float trq;    // Torque/Current limit (Amperes)

    MotionSegment() : t(0), mode(0), pos(0), vel(0.0f), acc(0.0f), jrk(0.0f), trq(0.0f) {}

    MotionSegment(uint32_t time, uint8_t m, int32_t p, float v, float a, float j, float q)
        : t(time), mode(m), pos(p), vel(v), acc(a), jrk(j), trq(q) {}
};

/// @brief Evaluates MotionSegments in real-time, intended for high-frequency ISR
/// @tparam N The capacity of the FIFO segment buffer
template <size_t N>
class MotionExecutor {
private:
    BufferFIFO<MotionSegment, N> buffer;

    MotionSegment current_segment;
    MotionState inst_state;

    float dt;
    float freq;
    float us_to_ticks; // Предрассчитанный коэффициент для перевода мкс в такты

    float half_acc0;
    float half_jrk;
    float sixth_jrk;

    uint32_t time_ticks;
    uint32_t duration_ticks;

    int32_t base_pos;
    float base_vel;
    float base_acc;

public:
    explicit MotionExecutor(uint32_t freq_hz)
        : time_ticks(0),
          duration_ticks(0),
          dt(1.0f / static_cast<float>(freq_hz)),
          freq(static_cast<float>(freq_hz)),
          us_to_ticks(static_cast<float>(freq_hz) / 1000000.0f), // Инициализация конвертера
          base_pos(0),
          base_vel(0),
          base_acc(0) {}

    inline __attribute__((always_inline)) MotionState tick() {
        if (time_ticks >= duration_ticks) {
            if (buffer.read(current_segment)) {
                // Log new motion segment task to serial output (before processing to show actual buffer content)
                // TP_LOG_VALS("New segment:", current_segment.t, current_segment.mode, current_segment.pos, current_segment.vel, current_segment.acc, current_segment.jrk, current_segment.trq);

                base_pos = current_segment.pos;
                base_vel = current_segment.vel;
                base_acc = current_segment.acc;

                // Быстрый перевод микросекунд в такты через FPU (без деления и uint64).
                // Добавляем 0.5f для корректного математического округления до ближайшего целого.
                // При t < 16.7 секунд потери точности каста во float нет (до 24 бит).
                duration_ticks = static_cast<uint32_t>(static_cast<float>(current_segment.t) * us_to_ticks + 0.5f);
                time_ticks = 0;

                half_acc0 = current_segment.acc * 0.5f;
                half_jrk  = current_segment.jrk * 0.5f;
                sixth_jrk = current_segment.jrk * (1.0f / 6.0f);
            } else {
                duration_ticks = 0;
                time_ticks = 0;
                base_vel = 0.0f;
                base_acc = 0.0f;
                current_segment.jrk = 0.0f;
                half_acc0 = 0.0f;
                half_jrk = 0.0f;
                sixth_jrk = 0.0f;
                base_pos = inst_state.pos; 
            }
        }

        time_ticks++;
        float t = static_cast<float>(time_ticks) * dt;

        float j0 = current_segment.jrk;

        float delta_p = t * (base_vel + t * (half_acc0 + t * sixth_jrk));

        inst_state.pos = base_pos + static_cast<int32_t>(delta_p * 65536.0f);

        inst_state.vel = base_vel + t * (base_acc + t * half_jrk);
        inst_state.acc = base_acc + j0 * t;

        return inst_state;
    }

    bool add_segment(const MotionSegment& segment) {
        if (segment.t > 0) {
            return buffer.write(segment);
        }
        return false;
    }

    void set_freq(uint32_t freq_hz) {
        freq = static_cast<float>(freq_hz);
        dt = 1.0f / freq;
        us_to_ticks = freq / 1000000.0f; // Обязательно обновляем коэффициент при смене частоты
    }

    MotionState get_current_state() const { return inst_state; }
};
