/**
 * @file pll.h
 * @brief Type-II (PI) PLL for a quantized, noisy, wrapping int16 angle
 *        (65536 LSB / turn). For Fs >> Fc, coarse quantization and
 *        occasional corrupt samples.
 *
 *   e      = wrap(angle - theta)          // float PD, no re-quantization
 *   |e|>gate -> e = 0 (coast); 1 ms run -> snap to input, drop velocity
 *   integ  = clamp(integ + Ki*dt*e)       // anti-windup
 *   omega  = Kp*e + integ
 *   theta += omega*dt;  theta -= lrintf(theta)
 *
 *   wn = 2*pi*Fc, Kp = sqrt(2)*wn, Ki = wn^2   (zeta = 1/sqrt(2))
 *   discrete stability: wn*dt < 1.035 -> Fc < 0.164*Fs (clamped in set_fc)
 *
 * 15-bit encoder / 50 pole pairs / 20 kHz: 1 count = 100 LSB electrical;
 * +-2-count noise dithers the detector and is averaged by the loop into
 * sub-LSB resolution. Schedule Fc: down at low speed, up at high accel.
 *
 * Licensed under the Apache License, Version 2.0
 * Copyright 2026, creapunk
 */

#pragma once

#include <cstdint>
#include <cmath>

struct FilterPLL {
private:
    // ---- tuning ----
    float    dt_;       // sample period [s]
    float    kp_;       // proportional gain Kp            [1/s]
    float    ki_dt_;    // integral gain Ki*dt             [1/s]
    float    imax_;     // integrator clamp, numeric guard [turn/s]
    float    gate_;     // glitch gate on |e|, keep (0, 0.5) [turn]
    uint16_t relock_;   // glitch run length -> re-lock    [ticks]

    // ---- state ----
    float    theta_;    // angle estimate [turn], kept in [-0.5, 0.5]
    float    omega_;    // velocity estimate [turn/s]
    float    integral_; // PI integrator [turn/s]
    uint16_t glitch_;   // consecutive gated samples

    static float clampf(float x, float lim) {
        return x > lim ? lim : (x < -lim ? -lim : x);
    }

public:
    FilterPLL(float fs, float fc = 100.0f, float gate = 0.25f)
        : dt_(1.0f), kp_(0.0f), ki_dt_(0.0f), imax_(0.0f), gate_(gate)
        , relock_(1), theta_(0.0f), omega_(0.0f), integral_(0.0f), glitch_(0) {
        if (!(fs > 0.0f)) fs = 1.0f;
        dt_     = 1.0f / fs;
        imax_   = 0.45f / dt_;               // |omega*dt| stays bounded
        relock_ = (uint16_t)(1e-3f / dt_);   // 1 ms of garbage -> re-lock
        if (relock_ < 1u) relock_ = 1u;
        set_fc(fc);
    }

    /** Retune bandwidth (gain scheduling). Loop state is kept.
     *  Fc clamped to the discrete stability limit wn*dt < 1.035. */
    void set_fc(float fc) {
        const float fc_max = 0.164f / dt_;
        if (!(fc >= 0.0f)) fc = 0.0f;
        if (fc > fc_max)   fc = fc_max;
        const float wn = 6.283185307179586f * fc;
        kp_    = 1.4142135623730951f * wn;   // [1/s] — no dt here (zeta fix)
        ki_dt_ = wn * wn * dt_;
    }

    /** Process one sample. int16 wrap-around of the input is expected. */
    int16_t tick(int16_t angle) {
        const float a = (float)angle * (1.0f / 65536.0f);
        float e = a - theta_;
        e -= (float)lrintf(e);               // wrap to [-0.5, 0.5]; lrintf is one vcvtr, rintf is a libm call (no VRINT on M4F)

        // Glitch gate: huge residual = corrupt sample -> coast on the
        // prediction. If it persists, snap to the input and drop velocity.
        if (e > gate_ || e < -gate_) {
            if (++glitch_ >= relock_) {
                theta_    = a;
                integral_ = 0.0f;
                glitch_   = 1;               // stays "unlocked" while garbage persists
            }
            e = 0.0f;
        } else {
            glitch_ = 0;
        }

        // PI, integral first: no extra sample of delay in the loop.
        integral_ = clampf(integral_ + ki_dt_ * e, imax_);
        omega_    = kp_ * e + integral_;

        // NCO: single lrintf() wrap is correct for any omega*dt.
        theta_ += omega_ * dt_;
        theta_ -= (float)lrintf(theta_);     // same round-to-nearest-even as rintf, inlined

        return get_angle();
    }

    /// Filtered angle, 65536 LSB per turn (round-to-nearest, no bias)
    int16_t get_angle() const {
        return (int16_t)(int32_t)lrintf(theta_ * 65536.0f);
    }

    /// Filtered angle in turns [-0.5, 0.5) — for Park: rad = theta * 2*pi
    float get_theta() const { return theta_; }

    /// Velocity estimate [turn/s]
    float get_omega() const { return omega_; }

    /// False while a glitch run is in progress
    bool is_locked() const { return glitch_ == 0; }

    /// Clear state; pass the measured angle for an instant lock at startup
    void reset(int16_t angle = 0) {
        theta_    = (float)angle * (1.0f / 65536.0f);
        omega_    = 0.0f;
        integral_ = 0.0f;
        glitch_   = 0;
    }
};