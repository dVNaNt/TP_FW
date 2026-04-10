// #pragma once
// #include "../../common.h"

// // Hybrid Adaptive Angle Filter for FOC Motor Control
// //
// // Licensed under the Apache License, Version 2.0
// // Copyright 2026 Anton Khrustalev, creapunk.com

// struct FilterAngleHybridEMA {
// private:
//     int32_t  state_;        // Internal state Q16.16
//     int16_t  output_;       // Current filtered output (Q1.15)
//     uint8_t  alpha_min_;    // Base filtering strength in pure static mode

//     // --- Phase Error Factors ---
//     uint16_t err_z1_;       // Error zone 1 threshold (2 * epsilon)
//     uint16_t err_z2_;       // Error zone 2 threshold (4 * epsilon)
//     uint16_t err_slope_q8_; // Slope for error alpha interpolation

//     // --- Velocity Factors ---
//     uint16_t vel_z1_;       // Velocity zone 1 (1/4 of electrical noise per tick)
//     uint16_t vel_z2_;       // Velocity zone 2 (1x electrical noise per tick)
//     uint16_t vel_slope_q8_; // Slope for velocity alpha interpolation
    
//     // --- Velocity scaling ---
//     float vel_mult_float_;  // Multiplier to convert float mech_vel to delta_elec_q15

//     uint8_t  alpha_dyn_;    // Last computed dynamic alpha (for telemetry)

// public:
//     FilterAngleHybridEMA() 
//         : state_(0), output_(0), alpha_min_(5),
//           err_z1_(0), err_z2_(0), err_slope_q8_(0),
//           vel_z1_(0), vel_z2_(0), vel_slope_q8_(0),
//           vel_mult_float_(0.0f), alpha_dyn_(5) {}

//     /// Configure filter constraints. Runs only on init or parameter change.
//     /// @param noise_mech_q15 Mechanical noise of the encoder in Q1.15 (LSB)
//     /// @param pole_pairs     Motor pole pairs multiplier
//     /// @param safe_margin    Multiplier for epsilon (e.g., 2)
//     /// @param alpha_min      Heavy filtering coefficient for static hold
//     void configure_zones(uint16_t noise_mech_q15, uint8_t pole_pairs, 
//                          uint8_t safe_margin, uint8_t alpha_min) 
//     {
//         alpha_min_ = alpha_min;

//         // 1. Calculate base electrical noise in Q15 LSBs
//         uint32_t noise_elec_q15 = (uint32_t)noise_mech_q15 * pole_pairs;
        
//         // 2. Calculate Error-based parameters
//         uint32_t epsilon = noise_elec_q15 * safe_margin;
//         err_z1_ = (uint16_t)(2 * epsilon);
//         err_z2_ = (uint16_t)(4 * epsilon);
        
//         uint32_t err_diff = err_z2_ - err_z1_;
//         err_slope_q8_ = (err_diff > 0) ? (uint16_t)(((255UL - alpha_min_) << 8) / err_diff) : 0;

//         // 3. Calculate Velocity-based parameters
//         vel_z1_ = (uint16_t)(noise_elec_q15 / 4);
//         vel_z2_ = (uint16_t)(noise_elec_q15);

//         uint32_t vel_diff = vel_z2_ - vel_z1_;
//         vel_slope_q8_ = (vel_diff > 0) ? (uint16_t)(((255UL - alpha_min_) << 8) / vel_diff) : 0;
//     }

//     /// Configure the scaler for float mechanical velocity.
//     /// @param dt_sec           Control loop period in seconds (e.g., 0.00005 for 20kHz)
//     /// @param pole_pairs       Motor pole pairs
//     /// @param mech_vel_to_rps  Multiplier to convert your mech_vel variable to Revs Per Second (Hz)
//     ///                         (If mech_vel is already in Revs/sec (Hz), pass 1.0f)
//     void configure_velocity_scaler(float dt_sec, uint8_t pole_pairs, float mech_vel_to_rps) {
//         // formula: delta_elec_q15 = mech_vel_rps * dt_sec * pole_pairs * 65536.0f
//         vel_mult_float_ = mech_vel_to_rps * dt_sec * (float)pole_pairs * 65536.0f;
//     }

//     /// Process one sample
//     /// @param angle_q15     Raw electrical angle from LUT in Q1.15
//     /// @param mech_velocity Real mechanical speed (float)
//     /// @return Filtered electrical angle
//     int16_t tick(int16_t angle_q15, float mech_velocity) {
//         // --- 1. Error Factor Calculation ---
//         // ИСправлено: вычисляем разницу строго в int16_t для корректного wraparound (перехода 360->0)
//         int16_t dev_signed = angle_q15 - output_;
//         uint16_t abs_dev = (dev_signed < 0) ? (uint16_t)(-dev_signed) : (uint16_t)(dev_signed);
        
//         uint16_t alpha_err = 0;
//         if (abs_dev > err_z1_) {
//             if (abs_dev < err_z2_) {
//                 alpha_err = (uint16_t)(((uint32_t)(abs_dev - err_z1_) * err_slope_q8_) >> 8);
//             } else {
//                 alpha_err = 255;
//             }
//         }

//         // --- 2. Velocity Factor Calculation ---
//         // ИСправлено: скорость float, вычисляем изменение угла за такт
//         float abs_vel = (mech_velocity < 0.0f) ? -mech_velocity : mech_velocity;
//         uint16_t delta_elec_q15 = (uint16_t)(abs_vel * vel_mult_float_);

//         uint16_t alpha_vel = 0;
//         if (delta_elec_q15 > vel_z1_) {
//             if (delta_elec_q15 < vel_z2_) {
//                 alpha_vel = (uint16_t)(((uint32_t)(delta_elec_q15 - vel_z1_) * vel_slope_q8_) >> 8);
//             } else {
//                 alpha_vel = 255;
//             }
//         }

//         // --- 3. Combine Factors (Sum Strategy) ---
//         uint16_t alpha_sum = alpha_min_ + alpha_err + alpha_vel;
//         uint8_t alpha_dyn = (alpha_sum > 255) ? 255 : (uint8_t)alpha_sum;
//         alpha_dyn_ = alpha_dyn;

//         // --- 4. Apply EMA Filter ---
//         // ИСправлено: математика фильтрации через разницу (diff), а не абсолютные значения
//         // ВАЖНО: diff должен быть sign-extended из int16_t, чтобы учитывать переход через ноль
//         int32_t diff_32 = (int32_t)dev_signed; // sign-extension (сохраняет знак разницы)
        
//         // Обновляем состояние: state = state + alpha * diff
//         // diff_32 << 8 эквивалентно сдвигу (diff * alpha) / 256
//         state_ += (diff_32 << 8) * (int32_t)alpha_dyn;
        
//         // Извлекаем output из Q16.16 (отбрасываем младшие 16 бит)
//         output_ = (int16_t)(state_ >> 16);

//         return output_;
//     }

//     /// Reset internal state
//     void reset(int16_t value) {
//         output_ = value;
//         state_  = (int32_t)value << 16;
//     }

//     inline int16_t get_output()    const { return output_; }
//     inline uint8_t get_alpha_dyn() const { return alpha_dyn_; }
// };