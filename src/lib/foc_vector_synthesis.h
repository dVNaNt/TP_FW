#pragma once

#include <math.h>
#include "common.h"

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

struct MotorVectorSynthesis {

    // ── LADRC parameters ──────────────────────────────────────────
    float wc_max, wc_min, wo, b0, kd_extra, dt;

    // ── LESO state ────────────────────────────────────────────────
    float z1 = 0.0f;       
    float z2 = 0.0f;       
    float z3 = 0.0f;       
    float u_prev = 0.0f;   

    // ── Single integrator with tracking ───────────────────────────
    float err_integral = 0.0f;
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

    float u_spring_track = 0.0f;   
    float u_foc_track    = 0.0f;   

private:
    // ── Angle filter state ────────────────────────────────────────
    int32_t  af_state_  = 0;
    int16_t  af_output_ = 0;
    uint8_t  af_alpha_min_ = 1;
    uint8_t  af_alpha_dyn_ = 1;

    uint16_t af_err_z1_  = 0, af_err_z2_  = 0, af_err_slope_q8_ = 0;
    uint16_t af_vel_z1_  = 0, af_vel_z2_  = 0, af_vel_slope_q8_ = 0;
    float    af_vel_mult_ = 0.0f;

    uint16_t af_err_deadband_ = 0;
    uint16_t af_vel_deadband_ = 0;

    float activity_index_ = 0.0f;
    float activity_lpf_   = 0.0f;  
    
    // [FIX 11] Изолированная активность для фильтра угла (только по скорости)
    float filter_activity_lpf_ = 0.0f;

    static inline int16_t _angle_diff(int16_t a, int16_t b)
    {
        int32_t d = (int32_t)a - (int32_t)b;
        if (d >  32767) d -= 65536;
        if (d < -32768) d += 65536;
        return (int16_t)d;
    }

    static inline float _smoothstep(float x)
    {
        if (x <= 0.0f) return 0.0f;
        if (x >= 1.0f) return 1.0f;
        return x * x * (3.0f - 2.0f * x);
    }

    int16_t _af_tick(int16_t raw, float activity)
    {
        int16_t dev = _angle_diff(raw, af_output_);

        int16_t noise_core = (int16_t)(af_err_deadband_ / 2);
        if (dev > noise_core) {
            dev -= noise_core;
        } else if (dev < -noise_core) {
            dev += noise_core;
        } else {
            dev = 0;
        }

        uint16_t adev = (dev < 0) ? (uint16_t)(-dev) : (uint16_t)dev;
        float    av   = (z2 < 0.0f) ? -z2 : z2;
        uint16_t dvel = (uint16_t)(av * af_vel_mult_);

        if (activity < 0.05f && adev == 0) {
            af_alpha_dyn_ = 0;
            af_state_ = (int32_t)af_output_ << 16;
            return af_output_;
        }

        uint16_t ae = 0;
        if (adev > af_err_z1_)
            ae = (adev < af_err_z2_)
                ? (uint16_t)(((uint32_t)(adev - af_err_z1_) * af_err_slope_q8_) >> 8)
                : 255u;

        uint16_t av2 = 0;
        if (dvel > af_vel_z1_)
            av2 = (dvel < af_vel_z2_)
                ? (uint16_t)(((uint32_t)(dvel - af_vel_z1_) * af_vel_slope_q8_) >> 8)
                : 255u;

        uint32_t alpha_dynamic = (uint32_t)ae + (uint32_t)av2;
        uint32_t alpha_scaled  = (uint32_t)(activity * (float)alpha_dynamic);
        uint32_t asum          = (uint32_t)af_alpha_min_ + alpha_scaled;
        uint8_t  adyn          = (asum > 255u) ? 255u : (uint8_t)asum;

        af_alpha_dyn_ = adyn;
        af_state_ += ((int32_t)dev << 8) * (int32_t)adyn;
        af_output_ = (int16_t)(af_state_ >> 16);

        if (adyn > 20) {
            float delay_ticks = (256.0f - (float)adyn) / ((float)adyn + 1e-6f);
            if (delay_ticks > 8.0f) delay_ticks = 8.0f;
            float comp_angle = z2 * dt * (float)motor_pole_pairs * 65536.0f * delay_ticks * 0.5f;
            int32_t comp_i = (int32_t)comp_angle;
            
            if (comp_i >  2000) comp_i =  2000;
            if (comp_i < -2000) comp_i = -2000;
            
            int32_t predicted = (int32_t)af_output_ + comp_i;
            if (predicted >  32767) predicted -= 65536;
            if (predicted < -32768) predicted += 65536;
            return (int16_t)predicted;
        }

        return af_output_;
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
        : wc_max(wc_max), wc_min(wc_min), wo(wo), b0(b0), kd_extra(kd_extra), dt(dt),
          max_velocity(max_vel), min_current_ma(min_curr_ma),
          speed_threshold_foc(speed_foc), load_threshold_ma(load_foc_ma),
          motor_pole_pairs((uint8_t)pole_pairs),
          blend_rate(blend_rate_hz),
          blend_rate_fast(blend_rate_hz * 50.0f),
          mode_hysteresis(hysteresis)
    {
        holding_current_ma = (holding_curr_ma > min_curr_ma)
                             ? holding_curr_ma : load_foc_ma;
        ki_ctrl           = wc_min * 5.0f;
        load_smooth_alpha = dt / (0.005f + dt);
        _recompute_observer_gains();
    }

    void configure_angle_filter(uint16_t noise_mech_q15, uint8_t safe_margin,
                                uint8_t alpha_min, float mech_vel_to_rps)
    {
        af_alpha_min_ = alpha_min;
        uint32_t noise_e = (uint32_t)noise_mech_q15 * motor_pole_pairs;
        uint32_t eps     = noise_e * safe_margin;

        af_err_z1_ = (uint16_t)(2u * eps);
        af_err_z2_ = (uint16_t)(4u * eps);
        {
            uint32_t d = (uint32_t)af_err_z2_ - (uint32_t)af_err_z1_;
            af_err_slope_q8_ = (d > 0)
                ? (uint16_t)(((255UL - alpha_min) << 8) / d) : 0;
        }

        af_vel_z1_ = (uint16_t)(noise_e / 4u);
        af_vel_z2_ = (uint16_t)(noise_e);
        {
            uint32_t d = (uint32_t)af_vel_z2_ - (uint32_t)af_vel_z1_;
            af_vel_slope_q8_ = (d > 0)
                ? (uint16_t)(((255UL - alpha_min) << 8) / d) : 0;
        }

        af_err_deadband_ = (uint16_t)(eps);
        af_vel_deadband_ = (uint16_t)(noise_e / 8u);
        af_vel_mult_     = mech_vel_to_rps * dt * (float)motor_pole_pairs * 65536.0f;
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
            err_integral          = 0.0f;
            blend_factor          = 0.0f;
            estimated_load_smooth = 0.0f;
            u_spring_track        = 0.0f;
            u_foc_track           = 0.0f;
            activity_index_       = 0.0f;
            activity_lpf_         = 0.0f;
            filter_activity_lpf_  = 0.0f;
            af_output_            = raw_elec_angle;
            af_state_             = (int32_t)raw_elec_angle << 16;
            is_first_tick         = false;
        }

        y_local += (float)(current_pos - prev_current_pos) * inv_q16;
        r_local += (float)(target_pos  - prev_target_pos)  * inv_q16;
        prev_current_pos = current_pos;
        prev_target_pos  = target_pos;

        if (fabsf(y_local) > 16.0f) {
            float sh = floorf(y_local);
            y_local -= sh;
            r_local -= sh;
            z1      -= sh;
        }

        // ══════════════════════════════════════════════════════════
        //  3. LESO
        // ══════════════════════════════════════════════════════════
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

        // Control activity учитывает ошибку (чтобы давить моментом), 
        // Filter activity учитывает только скорость (чтобы не пропускать шум)
        float raw_activity = (speed_activity > error_activity) ? speed_activity : error_activity;
        float act_alpha = (raw_activity > activity_lpf_)
                        ? dt / (0.002f + dt)
                        : dt / (0.10f  + dt);
        activity_lpf_ += act_alpha * (raw_activity - activity_lpf_);
        activity_index_ = activity_lpf_;

        // [FIX 11] Фильтр угла получает активность только от физической скорости
        float filt_alpha = (speed_activity > filter_activity_lpf_) 
                         ? dt / (0.002f + dt) 
                         : dt / (0.05f  + dt);
        filter_activity_lpf_ += filt_alpha * (speed_activity - filter_activity_lpf_);

        int16_t filtered_angle = _af_tick(raw_elec_angle, filter_activity_lpf_);

        float load_ma = fabsf(z3) / b0;
        estimated_load_smooth += load_smooth_alpha * (load_ma - estimated_load_smooth);

        float sp_start   = sp_foc * 0.25f;
        float tb_spd_raw = (abs_speed - sp_start) / (sp_foc - sp_start + 1e-6f);
        float tb_spd     = _smoothstep(tb_spd_raw);

        float ld_start   = load_threshold_ma * 0.3f;
        float tb_ld_raw  = (estimated_load_smooth - ld_start)
                         / (load_threshold_ma - ld_start + 1e-6f);
        float tb_load    = _smoothstep(tb_ld_raw);

        float target_blend = (tb_spd > tb_load) ? tb_spd : tb_load;

        if (error_activity > target_blend) {
            target_blend = error_activity;
        }

        float active_rate;
        if (target_blend > blend_factor) {
            active_rate = blend_rate + (blend_rate_fast - blend_rate) * activity_index_;
        } else {
            active_rate = blend_rate;
        }

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

        // [FIX 10] Убрано * inv_q16. target_vel и z2 оба находятся в RPS.
        float ev = target_vel - z2;

        if (activity_index_ < 0.1f) {
            const float ep_db = 4.0f / 65536.0f; 
            if (ep > ep_db) {
                ep -= ep_db;
            } else if (ep < -ep_db) {
                ep += ep_db;
            } else {
                ep = 0.0f;
            }
        }

        // ── Spring mode output: PD + integrator ───────────────────
        float u0_spring = kp * ep + kd * ev + target_acc - kd_extra * z2;

        const float eps_spring  = 0.25f / 65536.0f;
        const float eps_pos_max = 2.0f  / 65536.0f;

        if (abs_ep < eps_spring && abs_speed < sp_foc * 0.10f) {
            err_integral += ep * dt; 
            float max_i = (max_c * 0.5f) * b0 / (ki_ctrl + 1e-6f);
            if (err_integral >  max_i) err_integral =  max_i;
            if (err_integral < -max_i) err_integral = -max_i;
        } else if (abs_ep > eps_pos_max || abs_speed > sp_foc * 0.5f) {
            err_integral *= 0.98f;
        }

        float u_spring_raw = (u0_spring - ki_ctrl * err_integral) / b0 + target_torque;

        // ── FOC mode output: PD + z3 disturbance rejection ────────
        float u0_foc = kp * ep + kd * ev + target_acc - kd_extra * z2;
        float u_foc_raw = (u0_foc - z3) / b0 + target_torque;

        if (blend_factor > 0.01f) {
            float tracking_target = z3 / (ki_ctrl + 1e-6f);
            float tracking_rate   = blend_factor * wo * 0.1f * dt;
            if (tracking_rate > 1.0f) tracking_rate = 1.0f;
            err_integral += tracking_rate * (tracking_target - err_integral);
        }

        float u_blended = u_spring_raw * (1.0f - blend_factor)
                        + u_foc_raw    *         blend_factor;

        if (u_blended >  max_c) u_blended =  max_c;
        if (u_blended < -max_c) u_blended = -max_c;

        if (slew_rate_frac > 0.0f) {
            float max_du = max_c * slew_rate_frac;
            float du     = u_blended - u_prev;
            if (du >  max_du) du =  max_du;
            if (du < -max_du) du = -max_du;
            u_prev += du;
        } else {
            u_prev = u_blended;
        }

        float Iq = u_prev;

        float static_id = holding_current_ma * 0.1f;
        float active_id = min_current_ma;

        float id_blend = (blend_factor > activity_index_) ? blend_factor : activity_index_;
        float Id = static_id + id_blend * (active_id - static_id);
        if (Id < static_id) Id = static_id;

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

    void reset(float current_actual_vel = 0.0f)
    {
        z1 = z3 = u_prev = err_integral = 0.0f;
        y_local = r_local = 0.0f;
        blend_factor          = 0.0f;
        estimated_load_smooth = 0.0f;
        u_spring_track        = 0.0f;
        u_foc_track           = 0.0f;
        activity_index_       = 0.0f;
        activity_lpf_         = 0.0f;
        filter_activity_lpf_  = 0.0f;
        current_mode          = MotorMode::ANGLE_CONTROL;
        z2                    = current_actual_vel;
        is_first_tick         = true;
    }

    void set_bandwidth(float new_wc_max, float new_wc_min) {
        wc_max  = new_wc_max;
        wc_min  = new_wc_min;
        ki_ctrl = new_wc_min * 5.0f;
        _recompute_observer_gains();
    }

    inline float     get_disturbance()  const { return z3; }
    inline float     get_velocity_est()   const { return z2; }
    inline MotorMode get_mode()           const { return current_mode; }
    inline float     get_blend_factor()   const { return blend_factor; }
    inline float     get_load_est_ma()    const { return fabsf(z3) / b0; }
    inline int16_t   get_filtered_angle() const { return af_output_; }
    inline uint8_t   get_angle_alpha()    const { return af_alpha_dyn_; }
};