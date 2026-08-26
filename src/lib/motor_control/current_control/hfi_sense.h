/**
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#pragma once  // Ensure this header is included only once during compilation

#include "../../common.h"                          // Include common migration definitions and types

// Maximum SDFT window size (buffer length)
// Determined by ratio: F_sampling / F_carrier
#define HFI_MAX_WINDOW 16

// ============================================================================
// Helper Functions - Q15 Fixed-Point Math
// ============================================================================

static inline int16_t hfi_mul_q15(int16_t a, int16_t b) {
    return (int16_t)(((int32_t)a * (int32_t)b) >> 15);
}

// ============================================================================
// HFI Phase-Locked Loop (PLL)
// ============================================================================
struct HfiPll {
    friend struct HfiSdft;

private:
    float omega_est_;
    float theta_est_;
    float kp_;
    float ki_;
    float integral_err_;

public:
    void init(float kp_f, float ki_f) {
        kp_ = kp_f;
        ki_ = ki_f;
        omega_est_ = 0.0f;
        theta_est_ = 0.0f;
        integral_err_ = 0.0f;
    }

    void update(int16_t error) {
        float error_f = (float)error / 32768.0f;

        // PI controller
        float p_term = kp_ * error_f;
        integral_err_ += ki_ * error_f;

        omega_est_ = p_term + integral_err_;
        theta_est_ += omega_est_;

        // Wrap angle to -PI..PI for consistency
        if (theta_est_ > (float)M_PI)
            theta_est_ -= 2.0f * (float)M_PI;
        else if (theta_est_ < -(float)M_PI)
            theta_est_ += 2.0f * (float)M_PI;
    }

    inline int16_t get_angle() const { return (int16_t)(theta_est_ * (32768.0f / (float)M_PI)); }

    inline int32_t get_velocity() const { return (int32_t)(omega_est_ * 65536.0f); }

    void reset() {
        omega_est_ = 0.0f;
        theta_est_ = 0.0f;
        integral_err_ = 0.0f;
    }
};

// ============================================================================
// HFI Sliding DFT - Main Algorithm Structure
// ============================================================================
struct HfiSdft {
private:
    // Configuration
    int16_t window_size_;
    int16_t v_h_amplitude_;

    // State
    int16_t iq_buffer_[HFI_MAX_WINDOW];
    uint8_t buffer_index_;
    float sdft_real_;
    float sdft_imag_;

    // Constants
    float rotation_cos_;
    float rotation_sin_;
    float delay_comp_cos_;
    float delay_comp_sin_;
    float phase_correction_;

    // Sign detection
    int8_t sign_filter_;

    // PLL
    HfiPll pll_;

private:
    float corr_cos_;  // Cosine of the correction angle
    float corr_sin_;  // Sine of the correction angle

    // Private members:
float sign_product_filtered_;
int8_t polarity_;

public:
    // Debug variables
    int16_t debug_iq_input;
    int16_t debug_iq_old;
    int16_t debug_iq_delta;

    int32_t debug_sdft_temp_real;
    int32_t debug_sdft_temp_imag;

    int32_t debug_sdft_real;
    int32_t debug_sdft_imag;

    int32_t debug_magnitude_sq;
    int16_t debug_raw_amplitude;
    int16_t debug_amplitude;

    int16_t debug_theta_0;
    int16_t debug_recon_angle;
    int16_t debug_i_gamma;

    int16_t debug_cos_delayed;
    int32_t debug_sign_product;
    int8_t debug_polarity;
    int8_t debug_sign_filter;

    int16_t debug_hfi_error;
    int32_t debug_pll_p_term;
    int32_t debug_pll_i_term;
    int32_t debug_pll_omega;
    int32_t debug_pll_theta;

    // ========================================================================
    // Initialization
    // ========================================================================
    void init(int16_t v_h_norm, float f_h, float sampling_freq, float kp_pll = 2.0f, float ki_pll = 50.0f) {
        window_size_ = (int16_t)(sampling_freq / f_h);
        if (window_size_ > HFI_MAX_WINDOW)
            window_size_ = HFI_MAX_WINDOW;

        v_h_amplitude_ = v_h_norm;

        sign_product_filtered_ = 0.0f;
polarity_ = 1;

        float theta_step = 2.0f * (float)M_PI / (float)window_size_;
        rotation_cos_ = cosf(theta_step);
        rotation_sin_ = sinf(theta_step);

        float delay_angle = (2.0f * (float)M_PI * f_h) / sampling_freq;
        delay_comp_cos_ = cosf(delay_angle);
        delay_comp_sin_ = sinf(delay_angle);

        float corr_angle_rad = ((float)(window_size_ - 1) / (float)window_size_) * 2.0f * (float)M_PI;
        while (corr_angle_rad > (float)M_PI) corr_angle_rad -= 2.0f * (float)M_PI;
        phase_correction_ = corr_angle_rad;

        // TODO: add this:
        corr_cos_ = cosf(phase_correction_);
        corr_sin_ = sinf(phase_correction_);

        buffer_index_ = 0;
        sdft_real_ = 0.0f;
        sdft_imag_ = 0.0f;
        sign_filter_ = 0;
        debug_theta_0 = 3;

        for (int i = 0; i < HFI_MAX_WINDOW; i++) iq_buffer_[i] = 0;

        pll_.init(kp_pll, ki_pll);
    }

    // ========================================================================
    // Main Update Method
    // ========================================================================
    void update(int16_t iq_measured, int16_t gen_sin, int16_t gen_cos) {
        // --- Stage 1: Sliding DFT (Comb Filter) ---
        debug_iq_input = iq_measured;
        debug_iq_old = iq_buffer_[buffer_index_];

        int16_t old_sample = iq_buffer_[buffer_index_];
        iq_buffer_[buffer_index_] = iq_measured;

        buffer_index_++;
        if (buffer_index_ >= window_size_)
            buffer_index_ = 0;

        int16_t delta_i16 = iq_measured - old_sample;
        debug_iq_delta = delta_i16;

        float delta = (float)delta_i16 / 32768.0f;

        float tmp_real = sdft_real_ + delta;
        float tmp_imag = sdft_imag_;

        debug_sdft_temp_real = (int32_t)(tmp_real * 32768.0f);
        debug_sdft_temp_imag = (int32_t)(tmp_imag * 32768.0f);

        // --- Stage 2: Complex Rotation ---
        sdft_real_ = tmp_real * rotation_cos_ - tmp_imag * rotation_sin_;
        sdft_imag_ = tmp_real * rotation_sin_ + tmp_imag * rotation_cos_;

        debug_sdft_real = (int32_t)(sdft_real_ * 32768.0f);
        debug_sdft_imag = (int32_t)(sdft_imag_ * 32768.0f);

        // --- Stage 3: Amplitude Extraction ---
        float mag_sq = sdft_real_ * sdft_real_ + sdft_imag_ * sdft_imag_;
        debug_magnitude_sq = (int32_t)(mag_sq * 32768.0f * 32768.0f);

        float raw_amplitude = sqrtf(mag_sq);
        debug_raw_amplitude = (int16_t)(raw_amplitude * 32768.0f);

        float amplitude_f = raw_amplitude * (2.0f / (float)window_size_);
        int16_t amplitude = (int16_t)(amplitude_f * 32768.0f);
        debug_amplitude = amplitude;

        // --- Stage 4: Reconstruction (OPTIMIZED) ---
        // Compute i_gamma_f directly through vector rotation
        // i_gamma = Real * cos(phi) - Imag * sin(phi)
        // Note: the amplitude must be scaled by the factor (2.0 / window_size),
        // which is already accounted for in amplitude_f, but sdft_real/imag are raw.
        // Therefore apply the (2.0 / window_size) scale here, or use raw data and scale the result.

        float scale_factor = (2.0f / (float)window_size_);
        float i_gamma_raw = sdft_real_ * corr_cos_ - sdft_imag_ * corr_sin_;
        float i_gamma_f = i_gamma_raw * scale_factor;

        debug_i_gamma = (int16_t)(i_gamma_f * 32768.0f);


        // --- Stage 5: Delay Compensation & Sign Detection ---
        float gen_cos_f = (float)gen_cos / 32768.0f;
        float gen_sin_f = (float)gen_sin / 32768.0f;

        // float cos_delayed_f = gen_cos_f * delay_comp_cos_ + gen_sin_f * delay_comp_sin_;
        // debug_cos_delayed = (int16_t)(cos_delayed_f * 32768.0f);

        float sign_product_f = i_gamma_f * gen_cos_f;
        debug_sign_product = (int32_t)(sign_product_f * 32768.0f * 32768.0f);

        // *** FIXED LOGIC MATCHING ORIGINAL SOURCE ***
        static int8_t p = 1;
        if (sign_product_f >= 0.0f) {
            sign_filter_++;                       // hhfi->s++
            if (sign_filter_ >= debug_theta_0) {  // if (hhfi->s >= 1)
                sign_filter_ = debug_theta_0;     //   hhfi->s = 1
                p = 1;                            //   p = 1
            }

        } else {
            sign_filter_--;                        // hhfi->s--
            if (sign_filter_ <= -debug_theta_0) {  // if (hhfi->s <= -1)
                sign_filter_ = -debug_theta_0;     //   hhfi->s = -1
                p = -1;                            //   p = -1
            }
        }

        debug_polarity = p;
        debug_sign_filter = (int8_t)sign_filter_;

        // --- Stage 6: PLL Update ---
        int16_t hfi_error = (p > 0) ? -amplitude : amplitude;
        debug_hfi_error = hfi_error;

        pll_.update(hfi_error);

        debug_pll_omega = pll_.get_velocity();
        debug_pll_theta = (int32_t)(pll_.theta_est_ * (32768.0f / (float)M_PI));
    }

    // ========================================================================
    // Methods
    // ========================================================================
    inline int16_t get_injection_voltage(int16_t gen_cos) const { return hfi_mul_q15(v_h_amplitude_, gen_cos); }

    inline int16_t get_estimated_angle() const { return pll_.get_angle(); }

    inline int32_t get_estimated_velocity() const { return pll_.get_velocity(); }

    void reset() {
        buffer_index_ = 0;
        sdft_real_ = 0.0f;
        sdft_imag_ = 0.0f;
        sign_filter_ = 0.0f;
        for (int i = 0; i < HFI_MAX_WINDOW; i++) iq_buffer_[i] = 0;
        pll_.reset();
    }

    inline int16_t get_injection_amplitude() const { return v_h_amplitude_; }
    inline void set_injection_amplitude(int16_t v_h_norm) { v_h_amplitude_ = v_h_norm; }
};

// ============================================================================
// HFI Sense (Legacy)
// ============================================================================
struct HFISense {
private:
    FilterEMA input_hpf_;
    FilterEMA demod_lpf_;
    int16_t demod_output_;
    uint16_t carrier_freq_hz_;
    uint16_t system_freq_hz_;

public:
    HFISense(uint16_t carrier_freq_hz, uint16_t system_freq_hz)
        : input_hpf_(), demod_lpf_(), demod_output_(0), carrier_freq_hz_(carrier_freq_hz), system_freq_hz_(system_freq_hz) {
        input_hpf_.set_cutoff(carrier_freq_hz_ / 10, system_freq_hz_);
        demod_lpf_.set_cutoff(carrier_freq_hz_ / 6, system_freq_hz_);
    }

    void tick(int16_t current, int16_t ref_sin) {
        int16_t current_ac = input_hpf_.tick_hpf(current);
        int32_t mixed = ((int32_t)current_ac * (int32_t)ref_sin) >> 15;
        demod_lpf_.tick_lpf((int16_t)clamp_i32(mixed, INT16_MAX));
        demod_output_ = demod_lpf_.get_output();
    }

    inline int16_t get_output() const { return demod_output_; }

    void reset() {
        input_hpf_.reset(0);
        demod_lpf_.reset(0);
        demod_output_ = 0;
    }
};
