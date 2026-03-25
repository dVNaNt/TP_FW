#pragma once

#include "../common.h"

/// Type-2 Position Tracking Observer (3rd-order PLL)
///
/// Tracks absolute position (Q16.16), velocity, and acceleration.
/// Fixed bandwidth for mathematical stability. Relies on 3rd-order
/// zero-lag tracking properties to maintain low delay at high speeds
/// and low noise at rest.
struct Position {
private:
    // -------------------------------------------------------------------------
    // State: position
    // -------------------------------------------------------------------------
    int32_t position_;

    // -------------------------------------------------------------------------
    // State: M/T method (raw speed, low-speed accuracy)
    // -------------------------------------------------------------------------
    uint32_t ticks_since_change_;
    float raw_speed_rev_s_;

    // -------------------------------------------------------------------------
    // State: Type-2 PLL (3rd-order observer)
    // -------------------------------------------------------------------------
    int32_t pll_pos_q16_;
    int32_t pll_vel_rev_s_q16_ = 0;
    float pll_pos_frac_rev_;
    float pll_vel_rev_s_;
    float pll_acc_rev_s2_;

    // -------------------------------------------------------------------------
    // Control & Timing Variables
    // -------------------------------------------------------------------------
    float current_bw_hz_;
    float l1_, l2_, l3_;
    float dt_;



public:
    // =========================================================================
    // Constructor
    // =========================================================================
    Position(float bandwidth_hz, int16_t freq)
        : position_(0),
          ticks_since_change_(0),
          raw_speed_rev_s_(0.0f),
          pll_pos_q16_(0),
          pll_pos_frac_rev_(0.0f),
          pll_vel_rev_s_(0.0f),
          pll_acc_rev_s2_(0.0f) {
        dt_ = 1.0f / (float)freq;

        set_bandwidth(bandwidth_hz);
    }

    // =========================================================================
    // Main Update Function
    // =========================================================================
    void tick(uint16_t input_pos) {
        set_bandwidth(current_bw_hz_);

        uint16_t prev_angle = (uint16_t)position_;
        int16_t dif = (int16_t)(input_pos - prev_angle);

        // 2. Аккумуляция точной абсолютной позиции Q16.16
        position_ += (int32_t)dif;

        // 3. Упрощенный M/T метод (сырая скорость для диагностики)
        ticks_since_change_++;
        if (dif != 0) {
            raw_speed_rev_s_ = (float)dif * (1.0f / 65536.0f) / ((float)ticks_since_change_ * dt_);
            ticks_since_change_ = 0;
        } else {
            // Ограничение сырой скорости при остановке (затухание)
            float limit = (1.0f / 65536.0f) / ((float)ticks_since_change_ * dt_);
            if (raw_speed_rev_s_ > limit)
                raw_speed_rev_s_ = limit;
            else if (raw_speed_rev_s_ < -limit)
                raw_speed_rev_s_ = -limit;
        }

        // 4. Ошибка слежения (Tracking error)
        int32_t error_int_q16 = position_ - pll_pos_q16_;
        float error_rev = (float)error_int_q16 * (1.0f / 65536.0f) - pll_pos_frac_rev_;

        // 5. Интегрирование Type-2 Observer (полунеявный метод Эйлера для макс. стабильности)
        pll_acc_rev_s2_ += l3_ * error_rev * dt_;
        pll_vel_rev_s_ += (l2_ * error_rev + pll_acc_rev_s2_) * dt_;

        float est_vel = pll_vel_rev_s_ + l1_ * error_rev;

        // 6. Обновление дробной части позиции
        pll_pos_frac_rev_ += est_vel * dt_;
        pll_vel_rev_s_q16_ = pll_vel_rev_s_ * 65536.0;

        // 7. Сброс накопленной фракции в целый Q16.16
        int32_t inc_q16 = (int32_t)(pll_pos_frac_rev_ * 65536.0f);
        if (inc_q16 != 0) {
            pll_pos_q16_ += inc_q16;
            pll_pos_frac_rev_ -= (float)inc_q16 * (1.0f / 65536.0f);
        }
    }

    // =========================================================================
    // Control
    // =========================================================================
    inline void set_bandwidth(float bandwidth_hz) {
        current_bw_hz_ = bandwidth_hz;
        float w = 2.0f * 3.14159265f * bandwidth_hz;
        l1_ = 3.0f * w;
        l2_ = 3.0f * w * w;
        l3_ = w * w * w;
    }

    inline void reset() {
        position_ = 0;
        ticks_since_change_ = 0;
        raw_speed_rev_s_ = 0.0f;
        pll_pos_q16_ = 0;
        pll_pos_frac_rev_ = 0.0f;
        pll_vel_rev_s_ = 0.0f;
        pll_acc_rev_s2_ = 0.0f;
    }

    // =========================================================================
    // Getters
    // =========================================================================
    inline uint16_t get_raw_angle() const { return (uint16_t)position_; }
    inline uint16_t get_filtered_angle() const { return (uint16_t)pll_pos_q16_; }
    inline int32_t get_filtered_position() const { return pll_pos_q16_; }
    inline float get_filtered_position_rev() const { return (float)pll_pos_q16_ * (1.0f / 65536.0f) + pll_pos_frac_rev_; }
    inline int16_t get_raw_rotations() const { return (int16_t)(position_ >> 16); }
    inline int32_t get_raw_position() const { return position_; }

    inline int32_t get_speed_q16() const { return (int32_t)(raw_speed_rev_s_ * 65536.0f); }
    inline int32_t get_filtered_speed_q16() const { return (int32_t)(pll_vel_rev_s_ * 65536.0f); }
    inline float get_filtered_speed_float() const { return pll_vel_rev_s_; }

    inline float get_acceleration_float() const { return pll_acc_rev_s2_; }
    inline int32_t get_acceleration_q16() const { return (int32_t)(pll_acc_rev_s2_ * 65536.0f); }

    inline float get_current_bandwidth() const { return current_bw_hz_; }
};
