#pragma once
// 3-phase PWM driver with FOC (Field Oriented Control) current regulation

#include <cmath>  // For sqrtf and fabsf
#include "../../common.h"
#include "../../feedback/feedback.h"
#include "../../math/helpers/buzzer.h"
#include "../../math/helpers/frequency_generator.h"  // Frequency generator for HFI
#include "../../math/helpers/stability_check.h"      // Stability check for current monitoring
#include "../../math/helpers/timeout_timer.h"        // Timeout timer for countdown operations
#include "../motor.h"                                // Motor parameters structure
#include "foc-current.h"                             // FOC current controller with feedforward
#include "hfi_sense.h"                               // HFI sense for inductance measurement
#include "motor_calibration.h"
#include "motor_selector.h"





static constexpr int32_t test_pwr = 5000;
struct DriverPWM {
public:
    Motor& motor;
    DriverStatus setup_status;
    MotorStatus motor_calib_status;
    MotorSelector motor_selector;
    FilterMED3 med3_id_;
    FilterMED3 med3_iq_;
    FilterEMA lpf_id_;
    FilterEMA lpf_iq_;
    FilterAdaptiveAlphaEMA lpf_adaptive_alpha_ = FilterAdaptiveAlphaEMA(20000, 500, 3500);
    FocCurrentController foc_current_;
    static constexpr uint32_t CALIBRATION_ZERO_CYCLES = 50;
    MotorCalibration motor_calib;
    int16_t angle_ = 0;
    int16_t angle_prev_ = 0;
    timeout_timer driver_timeout;
    Vec3D output = {INT16_MIN, INT16_MIN, INT16_MIN};
    int32_t power = 0;
    Vec2D current_dq = {0, 0};
    Vec2D voltage_ab_ = {0, 0};
    Vec2D voltage_dq_ = {0, 0};
    Feedback& feedback;
    

    DriverPWM(Feedback& parent, Motor& motor_ref)
        : motor(motor_ref),
          setup_status(),
          motor_calib_status(MotorStatus::NotCalibrated),
          motor_selector(MotorType_STEP, PhasePattern_ABC, 995u, 256),
          driver_timeout(CALIB_PAUSE_CYCLES),
          feedback(parent) {
        parent.current.set_calibration_samples(CAL_OVERSAMPLING_POW2);
    }

    void set_current_epsilon(int16_t epsilon) { motor_selector.set_current_epsilon(epsilon); }
    bool is_current_calibrated() const { return setup_status.current_not_calibrated == 0; }
    MotorStatus get_motor_status() const { return motor.get_status(); }
    OperationStatus get_driver_status() const { return setup_status.get_driver_status(); }

    void start_current_offset_calibration() {
        motor.set_status(MotorStatus::NotCalibrated);
        driver_timeout.set_timeout(CALIBRATION_ZERO_CYCLES);
        feedback.current.calibrate();
        setup_status.current_not_calibrated = 1;
        setup_status.set_driver_status(OperationStatus::DriverCalibration);
    }

    void set_current_calibration_samples(int32_t n) { feedback.current.set_calibration_samples(n); }
    int16_t get_angle() const { return angle_; }

    void start_motor_calibration() {
        if (setup_status.get_driver_status() != OperationStatus::Ready)
            return;
        motor_calib.start_calibration(motor, motor_selector, motor_calib_status, angle_, feedback, test_pwr);
    }

    MotorStatus get_motor_calibration_status() const { return motor_calib_status; }
    Vec3D get_output() const { return output; }

private:
    void init_foc_controller() {
        float R_ohm = (float)motor.get_total_resistance() * 0.001f;
        float L_h = (float)motor.get_effective_inductance() * 1e-6f;
        const float torque_nm = 0.042f;
        const float rated_current_a = 1.5f;
        float pole_pairs = (motor_selector.get_motor_type() == MotorType_BLDC) ? 7.0f : 50.0f;
        foc_current_ = FocCurrentController(R_ohm, L_h, torque_nm, rated_current_a, pole_pairs);
        setup_status.set_driver_status(OperationStatus::Ready);
    }

public:
    void tick_control_foc(int16_t angle, Vec2D target_current, Feedback& feedback) {
        if (motor.get_status() == MotorStatus::Success)
            angle_ = angle;

        int16_t angular_speed = angle_ - angle_prev_;
        angle_prev_ = angle_;

        Pol2D angle_polar = Pol2D(angle_, INT16_MAX);
        Vec2D angle_sincos = angle_polar.convert();

        motor_selector.tick_current(feedback.current.norm());
        Vec2D current_ab = motor_selector.get_currents();

        Vec2D angle_sincos_inv(angle_sincos.cos, -angle_sincos.sin);
        Vec2D current_dq_raw = current_ab.rotate(angle_sincos_inv);
        current_dq = current_dq_raw;

        uint8_t alpha = lpf_adaptive_alpha_.tick(abs(angular_speed));
        lpf_id_.set_alpha(alpha);
        lpf_iq_.set_alpha(alpha);

        // int16_t filtered_id = (int16_t)lpf_id_.tick_lpf(med3_id_.tick(current_dq_raw.flux));
        // int16_t filtered_iq = (int16_t)lpf_iq_.tick_lpf(med3_iq_.tick(current_dq_raw.torque));
        int16_t filtered_id = med3_id_.tick(current_dq_raw.flux);
        int16_t filtered_iq = med3_iq_.tick(current_dq_raw.torque);

        (int16_t)lpf_id_.tick_lpf(filtered_id);
        (int16_t)lpf_iq_.tick_lpf(filtered_iq);

        Vec2D current_dq_filt = Vec2D(filtered_id, filtered_iq);

        Vec2D voltage_dq(0, 0);

        int32_t power_d = feedback.power.power_norm(filtered_id, voltage_dq_.d);
        int32_t power_q = feedback.power.power_norm(filtered_iq, voltage_dq_.q);

        int32_t power_norm = abs(power_d) + abs(power_q);
        power = feedback.power.norm2mw(power_norm);

        switch (setup_status.get_driver_status()) {
            case OperationStatus::Ready: {
                if (motor_calib_status == MotorStatus::Success) {
                    constexpr float OMEGA_COEF = 3.14159f * 20000.0f / 32768.0f;
                    constexpr float ANGLE_COEF = 3.14159f / 32768.0f;

                    float omega_e_rad_s = (float)angular_speed * OMEGA_COEF;
                    float voltage_limit_fraction = 0.50f;
                    float theta_el_rad = (float)angle_ * ANGLE_COEF;

                    FocCurrentController::VoltageDQ voltage_dq_result = foc_current_.tick(
                        feedback.current.norm2ma(target_current.d), feedback.current.norm2ma(target_current.q),
                        feedback.current.norm2ma(current_dq_filt.flux), feedback.current.norm2ma(current_dq_filt.torque),
                        feedback.supply.norm2mv(feedback.supply.get_voltage_norm()), voltage_limit_fraction, theta_el_rad, omega_e_rad_s);

                    int32_t Vd_mv = (int32_t)(voltage_dq_result.u_d * 1000.0f);
                    int32_t Vq_mv = (int32_t)(voltage_dq_result.u_q * 1000.0f);
                    int16_t Vd_norm = feedback.supply.mv2norm(Vd_mv);
                    int16_t Vq_norm = feedback.supply.mv2norm(Vq_mv);

                    voltage_dq = Vec2D(Vd_norm, Vq_norm);
                    break;
                }

                Vec2D current_ab_filt = current_dq_filt.rotate(angle_sincos);
                MotorStatus prev_status = motor_calib_status;

                motor_calib.tick_calibration(power_norm, power, current_ab_filt, current_dq_raw, motor_calib_status, motor,
                                             motor_selector, setup_status, angle_, feedback);

                if (prev_status != MotorStatus::Success && motor_calib_status == MotorStatus::Success) {
                    init_foc_controller();
                } else if (prev_status != MotorStatus::CalibError && motor_calib_status == MotorStatus::CalibError) {
                    voltage_ab_ = Vec2D(0, 0);
                }

                voltage_dq = Vec2D(motor_calib.get_voltage_d(), motor_calib.get_voltage_q());
                break;
            }

            case OperationStatus::DriverCalibration:
                if (driver_timeout.tick() > 0) {
                    output.ch1 = output.ch2 = output.ch3 = 0;
                    return;
                } else if (driver_timeout.get_timeout() == 0) {
                    feedback.current.calibrate();
                    driver_timeout.tick();
                    output.ch1 = output.ch2 = output.ch3 = INT16_MAX;
                    return;
                } else if (!feedback.current.is_calibrated()) {
                    return;
                } else {
                    setup_status.current_not_calibrated = 0;
                    setup_status.current_sensors_faulty = 0;
                    setup_status.set_driver_status(OperationStatus::Ready);
                    start_motor_calibration();
                }
                break;

            default:
                output.ch1 = output.ch2 = output.ch3 = INT16_MIN;
                return;
        }

        voltage_dq_ = voltage_dq;
        Vec2D voltage_ab = voltage_dq.rotate(angle_sincos);

        voltage_ab.sin = feedback.supply.norm2duty(voltage_ab.sin);
        voltage_ab.cos = feedback.supply.norm2duty(voltage_ab.cos);

        voltage_ab_ = voltage_ab;
        motor_selector.tick_voltage(voltage_ab);
        Vec3D voltages = motor_selector.get_voltages();

        if (feedback.error.get_fatal_dc_ovlo()) {
            output[0] = output[1] = output[2] = 0;
        } else if (feedback.error.get_fatal_oc()) {
            output[0] = output[1] = output[2] = INT16_MIN;
        } else {
            output[0] = voltages[0];
            output[1] = voltages[1];
            output[2] = voltages[2];
        }
    }
};