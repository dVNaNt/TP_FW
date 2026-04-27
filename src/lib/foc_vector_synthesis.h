#pragma once

#include <math.h>
#include <stdint.h>
#include "common.h" // Убедитесь, что M_PI определен здесь, если его нет в math.h

constexpr float INT16_TO_RAD = (float)(M_PI / 32768.0f);
constexpr float RAD_TO_INT16 = (float)(32768.0f / M_PI);

struct FOCCommand {
    float   u_q;
    float   u_d;
    int16_t elec_angle;
    bool    is_angle_ctrl;
};

enum class MotorMode : uint8_t {
    ANGLE_CONTROL = 0,
    FOC           = 1,
    BLENDING      = 2,
};

// ─────────────────────────────────────────────────────────────────────────────
// Adaptive Type-2 PLL for electrical angle tracking.
// Zero steady-state phase error at constant speed.
// ─────────────────────────────────────────────────────────────────────────────
struct AngleTrackingPLL {
    float wn_min, wn_max, wn_urgent, zeta, dt;
    float urgency_lo, urgency_hi, max_elec_hz;

    float   theta_       = 0.0f;
    float   omega_       = 0.0f;
    int16_t out_         = 0;
    bool    initialized_ = false;

private:
    static inline int16_t _diff(int16_t a, int16_t b) {
        int32_t d = (int32_t)a - (int32_t)b;
        if (d >  32767) d -= 65536;
        if (d < -32768) d += 65536;
        return (int16_t)d;
    }

    static inline float _smoothstep(float x) {
        if (x <= 0.0f) return 0.0f;
        if (x >= 1.0f) return 1.0f;
        return x * x * (3.0f - 2.0f * x);
    }

public:
    AngleTrackingPLL(float bw_min_hz = 15.0f, float bw_max_hz = 200.0f, float bw_urgent_hz = 500.0f,
                     float dt = 0.0001f, float max_elec_hz = 1200.0f, float zeta = 0.707f,
                     float urgency_lo_counts = 4096.0f,   // π/8
                     float urgency_hi_counts = 12288.0f)  // 3π/8
        : wn_min(2.0f * (float)M_PI * bw_min_hz)
        , wn_max(2.0f * (float)M_PI * bw_max_hz)
        , wn_urgent(2.0f * (float)M_PI * bw_urgent_hz)
        , zeta(zeta), dt(dt)
        , urgency_lo(urgency_lo_counts), urgency_hi(urgency_hi_counts)
        , max_elec_hz(max_elec_hz)
    {}

    int16_t tick(int16_t raw, float activity, float vel_ff_counts_sec = 0.0f) {
        if (!initialized_) {
            theta_       = (float)raw;
            omega_       = vel_ff_counts_sec; 
            out_         = raw;
            initialized_ = true;
            return raw;
        }

        const int16_t theta_i = (int16_t)(int32_t)theta_;
        const float   e       = (float)_diff(raw, theta_i);
        const float   abs_e   = (e < 0.0f) ? -e : e;

        // Anti-stall urgency boost
        const float urgency = _smoothstep((abs_e - urgency_lo) / (urgency_hi - urgency_lo + 1e-6f));
        const float wn = wn_min + (wn_max - wn_min) * activity + (wn_urgent - wn_max) * urgency;

        const float kp = 2.0f * zeta * wn;
        const float ki = wn * wn;

        omega_ += ki * e * dt;

        const float max_omega = 65536.0f * max_elec_hz;
        if (omega_ >  max_omega) omega_ =  max_omega;
        if (omega_ < -max_omega) omega_ = -max_omega;

        theta_ += (kp * e + omega_) * dt;

        if (theta_ >  32767.0f) theta_ -= 65536.0f;
        if (theta_ < -32768.0f) theta_ += 65536.0f;

        out_ = (int16_t)(int32_t)theta_;
        return out_;
    }

    void reset(int16_t initial_angle = 0, float initial_omega_counts_per_sec = 0.0f) {
        theta_       = (float)initial_angle;
        omega_       = initial_omega_counts_per_sec;
        out_         = initial_angle;
        initialized_ = false;
    }

    inline float freq_hz() const { return omega_ / 65536.0f; }
};

// ─────────────────────────────────────────────────────────────────────────────
// Main Synthesis Controller
// ─────────────────────────────────────────────────────────────────────────────
struct MotorVectorSynthesis {

    // ── LADRC parameters ──────────────────────────────────────────
    float wc_max, wc_min, wo, b0, kd_extra, dt;

    // ── LESO state ────────────────────────────────────────────────
    float z1 = 0.0f;      
    float z2 = 0.0f;      
    float z3 = 0.0f;      
    float u_prev = 0.0f;  

    // ── Disturbance Memory (formerly err_integral) ────────────────
    float disturbance_mem = 0.0f;
    float ki_ctrl;

    // ── Local coordinate frame ────────────────────────────────────
    float   y_local = 0.0f, r_local = 0.0f;
    int32_t prev_current_pos = 0, prev_target_pos = 0;
    bool    is_first_tick = true;

    // ── Motor parameters ──────────────────────────────────────────
    float   max_tracking_error = 2.0f;
    uint8_t motor_pole_pairs;
    float   beta1, beta2, beta3;
    float   max_velocity;
    float   min_current_ma;
    float   holding_current_ma;
    float   speed_threshold_foc;
    float   load_threshold_ma;
    float   mode_hysteresis;

    // ── Mode and blend ────────────────────────────────────────────
    MotorMode current_mode = MotorMode::ANGLE_CONTROL;
    float     blend_factor = 0.0f;
    float     blend_rate;
    float     blend_rate_fast;

    // ── Load estimation ───────────────────────────────────────────
    float estimated_load_smooth = 0.0f;
    float load_smooth_alpha;

    // ── Slew-rate (0 = disabled) ──────────────────────────────────
    float slew_rate_frac = 0.0f;
    
    // ── Angle PLL ─────────────────────────────────────────────────
    AngleTrackingPLL angle_pll_;

private:
    float activity_index_ = 0.0f;
    float activity_lpf_   = 0.0f;  
    float filter_activity_lpf_ = 0.0f;

    static inline float _smoothstep(float x) {
        if (x <= 0.0f) return 0.0f;
        if (x >= 1.0f) return 1.0f;
        return x * x * (3.0f - 2.0f * x);
    }

    inline void _recompute_observer_gains() {
        beta1 = 3.0f * wo;
        beta2 = 3.0f * wo * wo;
        beta3 = wo * wo * wo;
    }

public:
    MotorVectorSynthesis(
        float   wc_max,
        float   wc_min,
        float   wo,
        float   b0,
        float   dt,
        float   max_vel,
        float   min_curr_ma,
        float   speed_foc,
        float   load_foc_ma,
        float   pole_pairs      = 50.0f,
        float   blend_rate_hz   = 5.0f,
        float   hysteresis      = 0.15f,
        float   holding_curr_ma = 0.0f,
        float   kd_extra        = 0.0f)
        : wc_max(wc_max), wc_min(wc_min), wo(wo), b0(b0), kd_extra(kd_extra), dt(dt)
        , max_velocity(max_vel), min_current_ma(min_curr_ma)
        , speed_threshold_foc(speed_foc), load_threshold_ma(load_foc_ma)
        , motor_pole_pairs((uint8_t)pole_pairs)
        , blend_rate(blend_rate_hz)
        , mode_hysteresis(hysteresis)
        , angle_pll_(15.0f, 200.0f, 500.0f, dt) // Default PLL bandwidths
    {
        holding_current_ma = (holding_curr_ma > 0.0f) ? holding_curr_ma : load_foc_ma;
        ki_ctrl = wc_min * 5.0f;
        load_smooth_alpha = dt / (0.005f + dt);
        
        // Safety clamp for fast blend rate to avoid torque jumps
        float max_blend_fast = blend_rate_hz * 20.0f;
        blend_rate_fast = (max_blend_fast > 100.0f) ? 100.0f : max_blend_fast;
        
        _recompute_observer_gains();
    }

    FOCCommand tick(int32_t  target_pos,
                    int32_t  current_pos,
                    float    target_vel,
                    float    current_vel,
                    float    target_acc,
                    float    target_torque,
                    int32_t  max_current,
                    int16_t  raw_elec_angle)
    {
        const float inv_q16 = 1.0f / 65536.0f;
        const float max_c   = (float)max_current;

        if (is_first_tick) {
            prev_current_pos      = current_pos;
            prev_target_pos       = target_pos;
            y_local               = 0.0f;
            r_local               = (float)(target_pos - current_pos) * inv_q16;
            z1                    = 0.0f;
            z2                    = current_vel;
            z3                    = 0.0f;
            u_prev                = 0.0f;
            disturbance_mem       = 0.0f;
            blend_factor          = 0.0f;
            estimated_load_smooth = 0.0f;
            activity_index_       = 0.0f;
            activity_lpf_         = 0.0f;
            filter_activity_lpf_  = 0.0f;
            angle_pll_.reset(raw_elec_angle, current_vel * (float)motor_pole_pairs * 65536.0f);
            is_first_tick         = false;
        }

        y_local += (float)(current_pos - prev_current_pos) * inv_q16;
        r_local += (float)(target_pos  - prev_target_pos)  * inv_q16;
        prev_current_pos = current_pos;
        prev_target_pos  = target_pos;

        if (fabsf(r_local) > 16.0f) {
            float sh = floorf(r_local);
            r_local -= sh;
            y_local -= sh;
            z1      -= sh;
        }

        float max_dist = max_c * b0;
        float e_obs    = z1 - y_local;

        const float obs_core = 8.0f / 65536.0f;
        if (e_obs > obs_core) e_obs -= obs_core;
        else if (e_obs < -obs_core) e_obs += obs_core;
        else e_obs = 0.0f;

        z1 += (z2     - beta1 * e_obs) * dt;
        z2 += (z3     - beta2 * e_obs + b0 * u_prev) * dt;
        z3 += (-beta3 * e_obs) * dt;

        if (z3 >  max_dist) z3 =  max_dist;
        if (z3 < -max_dist) z3 = -max_dist;

        float ep_raw    = r_local - z1;
        float abs_speed = fabsf(z2);
        float abs_ep    = fabsf(ep_raw);

        const float sp_foc = speed_threshold_foc;
        float speed_activity = _smoothstep(abs_speed / (sp_foc * 0.5f));
        
        const float pos_noise_floor = 8.0f;
        float ep_counts = abs_ep * 65536.0f;
        float error_activity = 0.0f;
        if (ep_counts > pos_noise_floor) {
            error_activity = _smoothstep((ep_counts - pos_noise_floor) / 16.0f);
        }

        float raw_activity = (speed_activity > error_activity) ? speed_activity : error_activity;
        float act_alpha = (raw_activity > activity_lpf_)
                        ? dt / (0.002f + dt)
                        : dt / (0.10f  + dt);
        activity_lpf_ += act_alpha * (raw_activity - activity_lpf_);
        activity_index_ = activity_lpf_;

        float filt_alpha = (speed_activity > filter_activity_lpf_)
                         ? dt / (0.002f + dt)
                         : dt / (0.05f  + dt);
        filter_activity_lpf_ += filt_alpha * (speed_activity - filter_activity_lpf_);

        // ── Execute Type-2 Angle PLL ──────────────────────────────────────────
        float vel_ff_counts_sec = z2 * (float)motor_pole_pairs * 65536.0f;
        int16_t filtered_angle = angle_pll_.tick(raw_elec_angle, filter_activity_lpf_, vel_ff_counts_sec);

        float load_ma = fabsf(z3) / b0;
        estimated_load_smooth += load_smooth_alpha * (load_ma - estimated_load_smooth);

        float sp_start   = sp_foc * 0.25f;
        float tb_spd_raw = (abs_speed - sp_start) / (sp_foc - sp_start + 1e-6f);
        float tb_spd     = _smoothstep(tb_spd_raw);

        float ld_start   = load_threshold_ma * 0.3f;
        float tb_ld_raw  = (estimated_load_smooth - ld_start) / (load_threshold_ma - ld_start + 1e-6f);
        float tb_load    = _smoothstep(tb_ld_raw);

        float target_blend = (tb_spd > tb_load) ? tb_spd : tb_load;
        if (error_activity > target_blend) target_blend = error_activity;

        float active_rate = (target_blend > blend_factor) 
                            ? blend_rate + (blend_rate_fast - blend_rate) * activity_index_
                            : blend_rate;

        float br = active_rate * dt;
        if (blend_factor < target_blend) {
            blend_factor += br;
            if (blend_factor > target_blend) blend_factor = target_blend;
        } else {
            blend_factor -= br;
            if (blend_factor < target_blend) blend_factor = target_blend;
        }
        
          if (blend_factor > 1.0f) blend_factor = 1.0f;
        if (blend_factor < 0.0f) blend_factor = 0.0f;

        if      (blend_factor >= 1.0f - 1e-4f) current_mode = MotorMode::FOC;
        else if (blend_factor <= 1e-4f)        current_mode = MotorMode::ANGLE_CONTROL;
        else                                   current_mode = MotorMode::BLENDING;

        float kp = wc_max * wc_max;
        float kd = 2.0f * wc_max;
        float ep = r_local - z1;

        // 1. ИСПРАВЛЕНИЕ: Ограничение ошибки положения (Anti-Windup)
        // Используем забытый параметр max_tracking_error
        if (ep > max_tracking_error) ep = max_tracking_error;
        if (ep < -max_tracking_error) ep = -max_tracking_error;

        // 2. Зона нечувствительности (Deadband)
        if (activity_index_ < 0.1f) {
            const float ep_db = 4.0f / 65536.0f;
            if (ep > ep_db) ep -= ep_db;
            else if (ep < -ep_db) ep += ep_db;
            else ep = 0.0f;
        }

        float vel_est = target_vel * (1.0f - blend_factor) + z2 * blend_factor;
        float ev = target_vel - vel_est; 
        
        // 3. ИСПРАВЛЕНИЕ: Защита производной (опционально, но рекомендуется при больших инерциях)
        // Ограничиваем влияние ошибки скорости, чтобы избежать резких бросков тока
        const float max_ev = max_velocity * 1.5f;
        if (ev > max_ev) ev = max_ev;
        if (ev < -max_ev) ev = -max_ev;

        float u0 = kp * ep + kd * ev + target_acc - kd_extra * vel_est;

        const float eps_spring  = 0.25f / 65536.0f;
        const float eps_pos_max = 2.0f  / 65536.0f;

        // ── Disturbance Memory Tracking (formerly err_integral) ───────────────
        if (abs_ep < eps_spring && abs_speed < sp_foc * 0.10f) {
            disturbance_mem += ep * dt;
            float max_i = (max_c * 0.5f) * b0 / (ki_ctrl + 1e-6f);
            if (disturbance_mem >  max_i) disturbance_mem =  max_i;
            if (disturbance_mem < -max_i) disturbance_mem = -max_i;
        } else if (abs_ep > eps_pos_max || abs_speed > sp_foc * 0.5f) {
            disturbance_mem *= 0.98f;
        }

        float u_spring_raw = (u0 - ki_ctrl * disturbance_mem) / b0 + target_torque;
        float u_foc_raw    = (u0 - z3) / b0 + target_torque;

        if (blend_factor > 0.01f) {
            float tracking_target = z3 / (ki_ctrl + 1e-6f);
            float tracking_rate   = blend_factor * wo * 0.1f * dt;
            if (tracking_rate > 1.0f) tracking_rate = 1.0f;
            disturbance_mem += tracking_rate * (tracking_target - disturbance_mem);
        }

        float u_blended = u_spring_raw * (1.0f - blend_factor) + u_foc_raw * blend_factor;

        if (u_blended >  max_c) u_blended =  max_c;
        if (u_blended < -max_c) u_blended = -max_c;

        float u_target = u_blended;

        if (activity_index_ < 0.3f) {
            float smooth_alpha = activity_index_ * dt / (0.005f + dt)
                               + (1.0f - activity_index_) * dt / (0.020f + dt);
            u_target = u_prev + smooth_alpha * (u_blended - u_prev);
        }

        if (slew_rate_frac > 0.0f) {
            float max_du = max_c * slew_rate_frac;
            float du     = u_target - u_prev;
            if (du >  max_du) du =  max_du;
            if (du < -max_du) du = -max_du;
            u_prev += du;
        } else {
            u_prev = u_target;
        }

        float Iq = u_prev;
        float static_id = holding_current_ma * 0.1f;
        float active_id = min_current_ma;

        float id_blend = (blend_factor > activity_index_) ? blend_factor : activity_index_;
        float Id = static_id + id_blend * (active_id - static_id);
        
        if (active_id >= static_id) {
            if (Id < static_id) Id = static_id;
        } else {
            if (Id < active_id) Id = active_id;
        }

        float amp = sqrtf(Iq * Iq + Id * Id);
        if (amp > max_c) {
            float inv_a = max_c / amp;
            Iq *= inv_a;
            Id *= inv_a;
        }

        FOCCommand cmd;
        cmd.u_q           = Iq;
        cmd.u_d           = Id;
        cmd.elec_angle    = filtered_angle;
        cmd.is_angle_ctrl = false;
        return cmd;
    }

    void reset(float current_actual_vel = 0.0f) {
        z1 = z3 = u_prev = disturbance_mem = 0.0f;
        y_local = r_local = 0.0f;
        blend_factor          = 0.0f;
        estimated_load_smooth = 0.0f;
        activity_index_       = 0.0f;
        activity_lpf_         = 0.0f;
        filter_activity_lpf_  = 0.0f;
        current_mode          = MotorMode::ANGLE_CONTROL;
        z2                    = current_actual_vel;
        is_first_tick         = true;
        // PLL will be reset automatically on next tick via is_first_tick
    }

    void set_bandwidth(float new_wc_max, float new_wc_min) {
        wc_max  = new_wc_max;
        wc_min  = new_wc_min;
        ki_ctrl = new_wc_min * 5.0f;
        _recompute_observer_gains();
    }

    inline float     get_disturbance()  const { return z3; }
    inline float     get_velocity_est() const { return z2; }
    inline MotorMode get_mode()         const { return current_mode; }
    inline float     get_blend_factor() const { return blend_factor; }
    inline float     get_load_est_ma()  const { return fabsf(z3) / b0; }
    
    // Updated getters for the new PLL architecture
    // inline int16_t   get_filtered_angle() const { return angle_pll_.output(); }
    inline float     get_angle_pll_freq() const { return angle_pll_.freq_hz(); }
};