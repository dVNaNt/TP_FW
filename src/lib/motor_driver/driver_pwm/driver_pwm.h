/**
 * @file driver_pwm.h
 * @brief High-level PWM motor driver object.
 *
 * Aggregates the physical driver (PhyDriverPWM), the FOC current controller
 * (foc-current.h), the motor description (Motor) and the calibration state
 * machines into the single application-facing driver used by TunePulseHeart.
 */

#pragma once
// 3-phase PWM driver with FOC (Field Oriented Control) current regulation

#include <cmath>  // For sqrtf and fabsf
#include "../../common.h"
#include "../../drivers/pwm_driver_phy/pwm_driver.h"
#include "../../math/helpers/frequency_generator.h"  // Frequency generator for HFI
#include "../../math/helpers/stability_check.h"      // Stability check for current monitoring
#include "../../math/helpers/timeout_timer.h"        // Timeout timer for countdown operations
#include "../motor.h"                                // Motor parameters structure
#include "foc-current.h"                             // FOC current controller with feedforward
#include "hfi_sense.h"                               // HFI sense for inductance measurement
#include "motor_calibration.h"

static constexpr float test_pwr = 5.0;
struct DriverPWM {
public:
    Motor& motor;
    DriverStatus setup_status;
    MotorStatus motor_calib_status;
    FilterMED3 med3_id_;
    FilterMED3 med3_iq_;
    FilterEMA_f32 lpf_id_;
    FilterEMA_f32 lpf_iq_;
    FilterAdaptiveAlphaEMA_f32 lpf_adaptive_alpha_ = FilterAdaptiveAlphaEMA_f32(20000, 500, 3500);
    FocCurrentController foc_current_;
    static constexpr uint32_t CALIBRATION_ZERO_CYCLES = 50;
    MotorCalibration motor_calib;
    int16_t angle_ = 0;
    int16_t angle_prev_ = 0;
    timeout_timer driver_timeout;
    float power = 0;
    Vec2D_f32 current_dq = {0, 0};
    Vec2D_f32 current_ab = {0, 0};
    Vec2D_f32 voltage_ab_ = {0, 0};
    Vec2D_f32 voltage_dq_ = {0, 0};
    bool is_saturated = false;
    PhyDriverPWM& drv;

    DriverPWM(Motor& motor_ref, PhyDriverPWM& drv)
        : motor(motor_ref),
          setup_status(),
          motor_calib_status(MotorStatus::NotCalibrated),
          driver_timeout(CALIB_PAUSE_CYCLES),
          drv(drv),
          motor_calib(drv) {}

    MotorStatus get_motor_status() const { return motor.get_status(); }
    OperationStatus get_driver_status() const { return setup_status.get_driver_status(); }

    int16_t get_angle() const { return angle_; }

    void start_motor_calibration() {
        // if (setup_status.get_driver_status() != OperationStatus::Ready)
        //     return;
        motor_calib.start_calibration(motor, motor_calib_status, angle_, test_pwr);
    }

    MotorStatus get_motor_calibration_status() const { return motor_calib_status; }

private:
    void init_foc_controller() {
        float R_ohm = motor.get_total_resistance();
        float L_h = motor.get_effective_inductance();
        const float torque_nm = 0.042f;
        const float rated_current_a = 1.5f;
        float pole_pairs = (drv.motor_sel_.get_motor_type() == MotorType_BLDC) ? 7.0f : 50.0f;
        // foc_current_ = FocCurrentController(R_ohm, L_h, torque_nm, rated_current_a, pole_pairs);
        foc_current_ = FocCurrentController(drv.motor_sel_.get_motor_type(), R_ohm, L_h, torque_nm, rated_current_a, pole_pairs);
        setup_status.set_driver_status(OperationStatus::Ready);
    }

public:
    Vec2D_f32 tick_control_foc(int16_t angle, float target_current) {
        // Taking control over angle to perform electric test
        if (motor.get_status() == MotorStatus::Success)
            angle_ = angle;

        int16_t angular_speed = angle_ - angle_prev_;
        angle_prev_ = angle_;

        // Prepearing angles for future rotations
        Pol2D angle_polar = Pol2D(angle_, INT16_MAX);
        Vec2D angle_sincos = angle_polar.convert();
        Vec2D angle_sincos_inv(angle_sincos.cos, -angle_sincos.sin);

        current_ab = drv.get_current_ab_amps();

        Vec2D_f32 current_dq_raw = current_ab.rotate(angle_sincos_inv);
        current_dq = current_dq_raw;

        uint8_t alpha = lpf_adaptive_alpha_.tick(abs(angular_speed));
        lpf_id_.set_alpha(alpha);
        lpf_iq_.set_alpha(alpha);

        int16_t filtered_id = med3_id_.tick(current_dq_raw.flux);
        int16_t filtered_iq = med3_iq_.tick(current_dq_raw.torque);

        (int16_t)lpf_id_.tick_lpf(filtered_id);
        (int16_t)lpf_iq_.tick_lpf(filtered_iq);

        Vec2D_f32 current_dq_filt = Vec2D_f32(filtered_id, filtered_iq);

        Vec2D_f32 voltage_dq(0, 0);

        float power_d = filtered_id * voltage_dq_.d;
        float power_q = filtered_iq * voltage_dq_.q;

        power = abs(power_d) + abs(power_q);

        switch (setup_status.get_driver_status()) {
            case OperationStatus::Ready: {
                if (motor_calib_status == MotorStatus::Success) {
                    constexpr float OMEGA_COEF = 3.14159f * 20000.0f / 32768.0f;
                    constexpr float ANGLE_COEF = 3.14159f / 32768.0f;

                    float omega_e_rad_s = (float)angular_speed * OMEGA_COEF;
                    float voltage_limit_fraction = 0.50f;
                    float theta_el_rad = (float)angle_ * ANGLE_COEF;

                    voltage_dq = foc_current_.tick(target_current, current_dq_raw.flux, current_dq_raw.torque,
                                                   drv.get_supply_volts(), voltage_limit_fraction, omega_e_rad_s);

                    is_saturated = foc_current_.is_saturated;

                    break;
                }

                Vec2D_f32 current_ab_filt = current_dq_filt.rotate(angle_sincos);
                MotorStatus prev_status = motor_calib_status;


                motor_calib.tick_calibration(drv.get_supply_volts(), power, current_ab_filt.a, current_ab_filt.b, current_dq_raw.flux, current_dq_raw.torque,
                                             motor_calib_status, motor, setup_status, angle_);

                if (prev_status != MotorStatus::Success && motor_calib_status == MotorStatus::Success) {
                    init_foc_controller();
                } else if (prev_status != MotorStatus::CalibError && motor_calib_status == MotorStatus::CalibError) {
                    voltage_ab_ = Vec2D_f32(0.0, 0.0);
                }

                voltage_dq = Vec2D_f32(motor_calib.get_voltage_d(), motor_calib.get_voltage_q());
                break;
            }

            case OperationStatus::DriverCalibration:

                setup_status.current_not_calibrated = 0;
                setup_status.current_sensors_faulty = 0;
                setup_status.set_driver_status(OperationStatus::Ready);
                start_motor_calibration();

                break;

            default:
                return Vec2D_f32(NAN, NAN);
        }

        voltage_dq_ = voltage_dq;
        return voltage_dq.rotate(angle_sincos);
    }
};