#pragma once

// 3-phase PWM driver with FOC (Field Oriented Control) current regulation

#include "../../common.h"
#include "../../feedback/feedback.h"
#include "../../math/helpers/frequency_generator.h"  // Frequency generator for HFI
#include "../../math/helpers/speaker.h"              // Motor speaker for audio feedback
#include "../../math/helpers/stability_check.h"      // Stability check for current monitoring
#include "../../math/helpers/timeout_timer.h"        // Timeout timer for countdown operations
#include "../motor.h"                                        // Motor parameters structure
#include "hfi_sense.h"                                       // HFI sense for inductance measurement
#include "sel_motor.h"
#include "foc-current.h"                                     // FOC current controller with feedforward

volatile int16_t dead_time_duty = 1310;
volatile int16_t dead_time_duty_sincos = 0;
volatile int16_t deadzone = 1;
volatile int16_t deadzone_sincos = 1;

// Motor test power should be between 10% to 40% of total motor power
static constexpr int32_t test_pwr = 3000;

// Forward declaration
struct Feedback;

static constexpr uint16_t CAL_OVERSAMPLING_POW2 = 64;  // 64 samples

// ~0.1 % of maximum driver output power. Otherwise we can bound ADC resolution easily and cant control driver
static constexpr uint32_t CAL_UNDERPOWER_LIM_NORM = 256 * 256 * 16;

static constexpr int16_t CAL_ANGLE_45D = 8192;  // -45 deg
static constexpr int16_t WARMUP_TIMEOUT = 4500;
static constexpr int16_t SETTLE_TIMEOUT = 900;
static constexpr int16_t ROTATION_TIMEOUT = 1800;


// Motor calibration structure with all calibration-related variables and states
struct MotorCalibration {
private:
    // Connection calibration ramp variables
    int16_t voltage_;                // Current output voltage (Q15 duty)
    int16_t voltage_increment_;      // Voltage increment per tick (Q15)
    int16_t voltage_limit_;          // Maximum voltage limit (Q15)
    int16_t overcurrent_threshold_;  // Overcurrent detection threshold (Q15)

    // Slow rotation phase variables
    int16_t slow_rotation_angle_target_;  // Target angle for slow rotation (Q15)
    uint16_t slow_rotation_power_limit_;  // Power limit during rotation (normalized)

    // Measurement reference values - accumulation for averaging
    int32_t measure_current_a_sum_;  // Phase A current accumulator for 100 samples
    int32_t measure_current_b_sum_;  // Phase B current accumulator for 100 samples

    // Inductance measurement accumulators
    int32_t inductance_a_sum_;  // Phase A inductance accumulator for HFI measurement
    int32_t inductance_b_sum_;  // Phase B inductance accumulator for HFI measurement

public:
    // Constructor - initialize all calibration variables
    MotorCalibration()
        : voltage_(0),
          voltage_increment_(2),
          voltage_limit_(INT16_MAX / 2),
          overcurrent_threshold_(INT16_MAX / 2),
          slow_rotation_angle_target_(0),
          slow_rotation_power_limit_(0),
          measure_current_a_sum_(0),
          measure_current_b_sum_(0),
          inductance_a_sum_(0),
          inductance_b_sum_(0) {}

    // Start slow rotation phase
    void start_slow_rotation(int16_t target_angle, int32_t power_limit) {
        slow_rotation_angle_target_ = target_angle;  // Set target angle directly
        slow_rotation_power_limit_ = power_limit;    // Save power limit
    }

    // Getters for calibration variables
    int16_t get_voltage() const { return voltage_; }
    int16_t get_voltage_increment() const { return voltage_increment_; }
    int16_t get_voltage_limit() const { return voltage_limit_; }
    int16_t get_slow_rotation_angle_target() const { return slow_rotation_angle_target_; }
    int32_t get_slow_rotation_power_limit() const { return slow_rotation_power_limit_; }
    int16_t get_measure_current_a() const {
        return (int16_t)(measure_current_a_sum_ / CAL_OVERSAMPLING_POW2);
    }  // Return averaged value
    int16_t get_measure_current_b() const {
        return (int16_t)(measure_current_b_sum_ / CAL_OVERSAMPLING_POW2);
    }  // Return averaged value

    // Setters for calibration variables
    void set_voltage(int16_t voltage) { voltage_ = voltage; }

    void accumulate_measure_currents(int16_t current_a, int16_t current_b) {
        measure_current_a_sum_ += current_a;  // Accumulate phase A
        measure_current_b_sum_ += current_b;  // Accumulate phase B
    }
    void reset_measure_accumulators() {
        measure_current_a_sum_ = 0;  // Reset phase A accumulator
        measure_current_b_sum_ = 0;  // Reset phase B accumulator
    }

    // Inductance measurement accumulation methods
    void accumulate_inductance(int16_t hfi_a, int16_t hfi_b) {
        inductance_a_sum_ += abs(hfi_a);  // Accumulate absolute HFI response for phase A
        inductance_b_sum_ += abs(hfi_b);  // Accumulate absolute HFI response for phase B
    }
    void reset_inductance_accumulators() {
        inductance_a_sum_ = 0;  // Reset phase A inductance accumulator
        inductance_b_sum_ = 0;  // Reset phase B inductance accumulator
    }
    int32_t get_inductance_a() const {
        return inductance_a_sum_ / 1800;  // Return averaged inductance A (1800 samples)
    }
    int32_t get_inductance_b() const {
        return inductance_b_sum_ / 1800;  // Return averaged inductance B (1800 samples)
    }

    // Voltage control methods
    inline void increase_voltage() {
        voltage_ += voltage_increment_;  // Increase by increment
    }

    inline void decrease_voltage() {
        voltage_ -= voltage_increment_;  // Decrease by increment
        if (voltage_ < 0) {              // Clamp to zero
            voltage_ = 0;
        }
    }

    void reset_voltage() { voltage_ = 0; }  // Reset voltage to zero
};


struct DriverPWM {
public:
    // Motor configuration reference (owned by TunePulseHeart)
    Motor& motor;

    // Setup status structure for driver readiness tracking
    // Optimized for single-instruction zero-check: status_byte == 0 means fully ready
    DriverStatus setup_status;

    // Motor calibration status tracking (separate from driver status)
    MotorStatus motor_calib_status;

    // Constructor: takes Motor reference from parent (TunePulseHeart owns Motor)
    DriverPWM(Feedback& parent, Motor& motor_ref)
        : motor(motor_ref),
          feedback(parent),
          motor_selector(parent, MotorType_STEP, PhasePattern_ABC, 995u, 256),
          stability_check(50, 240),                       // Initialize stability checker: epsilon=50, window=180 cycles
          timeout(CALIB_PAUSE_CYCLES),                    // Initialize timeout timer to 100ms @ 18kHz
          hfi_generator_(20000),                          // Initialize HFI generator at 18 kHz system frequency
          motor_calib_status(MotorStatus::NotCalibrated)  // Initialize motor calibration status
    {
        // Set calibration samples to 100 for feedback current sensor
        parent.current.set_calibration_samples(CAL_OVERSAMPLING_POW2);  // Configure calibration samples
    }

    void tick_control_foc(int16_t angle, Vec2D target_current, Feedback& feedback) {
        tick_control_foc_(angle, target_current, feedback);
    }


    // Set current balance tolerance
    void set_current_epsilon(int16_t epsilon) { motor_selector.set_current_epsilon(epsilon); }

    // Check if current sensors are calibrated in feedback
    bool is_current_calibrated() const {
        return setup_status.current_not_calibrated == 0;  // Check bitfield flag (0 = calibrated)
    }

    // Get current motor calibration status
    MotorStatus get_motor_status() const { return motor.get_status(); }

    OperationStatus get_driver_status() const { return setup_status.get_driver_status(); }

    // Manually start current calibration in feedback
    void start_current_offset_calibration() {
        motor.set_status(MotorStatus::NotCalibrated);                        // Update motor status
        timeout.set_timeout(CALIBRATION_ZERO_CYCLES);                        // Set timeout for zero cycles
        feedback.current.calibrate();                                        // Start calibration through friend access
        setup_status.current_not_calibrated = 1;                             // Set flag (calibration in progress)
        setup_status.set_driver_status(OperationStatus::DriverCalibration);  // Set driver status to DriverCalibration
    }

    // Configure number of calibration samples in feedback
    void set_current_calibration_samples(int32_t n) {
        feedback.current.set_calibration_samples(n);  // Configure samples through friend access
    }

    /// Get current rotor angle (Q15 format, -32768 to 32767 = -π to π)
    int16_t get_angle() const { return angle_; }

    /// Start motor calibration sequence
    /// This method initiates the motor calibration process which includes:
    /// 1. Soft warmup - gradually increase voltage to check motor connection
    /// 2. Resistance measurement - measure phase resistances
    /// 3. Motor type detection - identify motor type (BLDC, Stepper, DC)
    /// 4. Phase pattern detection - determine correct phase wiring
    /// Prerequisites: Current sensors must be calibrated first (setup_status.current_not_calibrated == 0)
    void start_motor_calibration() {
        // Check if driver is ready for motor calibration
        if (setup_status.get_driver_status() != OperationStatus::Ready) {
            return;  // Driver not ready - current sensors must be calibrated first
        }

        // Initialize motor calibration state
        motor_calib_status = MotorStatus::SoftWarmup;  // Start with soft warmup phase
        motor.set_status(MotorStatus::SoftWarmup);     // Update motor status

        // Reset calibration variables
        motor_calib.reset_voltage();  // Reset voltage to zero
        stability_check.reset();      // Reset stability checker

        // Configure soft limits for calibration
        feedback.power.set_overpower_soft_mw(test_pwr);                                     // Set soft power limit
        feedback.supply.set_overvoltage_soft_norm(feedback.supply.get_voltage_norm() / 2);  // Set soft voltage limit

        // Configure motor selector for initial calibration attempt (Stepper, ABC pattern)
        motor_selector.set_motor_type(MotorType_STEP);       // Start with stepper motor assumption
        motor_selector.set_phase_pattern(PhasePattern_ABC);  // Start with ABC phase pattern

        // Set initial angle for calibration (-45 degrees)
        angle_ = CAL_ANGLE_45D;

        // Set warmup timeout
        timeout.set_timeout(WARMUP_TIMEOUT);

        // Log calibration start
        TP_LOG("[CALIB] Motor calibration started");
    }

    // Get motor calibration status
    MotorStatus get_motor_calibration_status() const {
        return motor_calib_status;  // Return current motor calibration status
    }

    Vec3D get_output() const { return output; }

private:
    // Reference to parent Feedback
    Feedback& feedback;

    MotorSelector motor_selector;  // Unified motor selector (voltage + current + phase)
    int16_t i_deadband = 10;
    int16_t i_deadband_sincos = 10;

    ///////////////////////////////////////////////////////////////////////////////
    // CURRENT DIGITAL SIGNAL PROCESSING          /////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////
    // Median filter to cut random ADC deviations caused by noise. MED3 have small delay so its fine
    FilterMED3 med3_id_;
    FilterMED3 med3_iq_;

    // LPF filter to make signal smoother. May cause phase delay
    FilterEMA lpf_id_;
    FilterEMA lpf_iq_;


    // Adaptive low-pass filter for current measurement from 100hz to 3500hz spin frequency with 20khz base rate
    // Main goal is to adjust Fc to reduce phase delay on high frequency signals. At low speeds it will ensure high filtration to
    // reduce audiable noise caused by current regulators
    FilterAdaptiveAlphaEMA lpf_adaptive_alpha_ = FilterAdaptiveAlphaEMA(20000, 100, 3500);

    
    // FOC current controller with feedforward compensation
    FocCurrentController foc_current_;

    // Used to charge mosfet driver before going high on all channels
    static constexpr uint32_t CALIBRATION_ZERO_CYCLES = 50;  // Cycles with zero PWM

    // Motor calibration structure with all calibration-related variables
    MotorCalibration motor_calib;

    static constexpr uint32_t CALIB_PAUSE_CYCLES = 1800;         // 100ms pause @ 18kHz
    static constexpr uint32_t INDUCTANCE_MEASURE_CYCLES = 1800;  // 100 periods @ 1kHz (18kHz / 1kHz * 100)

    int16_t angle_ = 0;              // Current rotor angle (Q15, -32768 to 32767 = -π to π)
    int16_t angle_prev_ = 0;         // Previous rotor angle for velocity calculation (Q15)
    StabilityCheck stability_check;  // Stability checker for current monitoring
    timeout_timer timeout;           // Timeout timer for general purpose countdown operations

    // HFI components for inductance measurement and sensorless control
    FrequencyGenerator hfi_generator_;  // 1 kHz frequency generator for HFI injection
    HfiSdft hfi_sdft_;                  // HFI Sliding DFT position estimator for sensorless control
    bool hfi_enabled_ = false;          // Flag to enable/disable HFI sensorless control

    Vec3D output = {INT16_MIN, INT16_MIN, INT16_MIN};  // HiZ default

    int32_t power = 0;

    Vec2D current_dq = {0, 0};

    // Debug/monitoring
    Vec2D voltage_ab_ = {0, 0};  // Last voltage command
    Vec2D voltage_dq_ = {0, 0};  // Last voltage command

    // TEMPORARY/TODO
    int8_t compensation_current = 0;
    int16_t test_state = 0;
    int16_t test_comp_val = 0;

    // ========================================================================
    // HFI Test Variables (Public for monitoring)
    // ========================================================================
    int16_t hfi_estimated_angle = 0;      // HFI estimated electrical angle (0-65535 = 0-360°)
    int32_t hfi_estimated_velocity = 0;   // HFI estimated angular velocity (internal units)
    int16_t hfi_injection_voltage = 0;    // Current HFI injection voltage (Q15)
    int16_t hfi_measured_iq = 0;          // Measured Q-axis current for HFI (Q15)
    Vec2D hfi_generator_signal = {0, 0};  // HFI generator sin/cos output (Q15)
    bool hfi_active = false;              // Flag indicating HFI is running

    inline void tick_control_foc_(int16_t angle, Vec2D target_current, Feedback& feedback) {
        // Ensuring that we have full internal control over angle until driver is ready
        if (motor.get_status() == MotorStatus::Success) {
            angle_ = angle;  // Override to zero
        }

        // Some helper math
        int16_t angular_speed = angle_ - angle_prev_;  // Calculate angle change (wraps automatically)
        angle_prev_ = angle_;                          // Store current angle for next iteration

        // Step 1: Convert encoder angle to rotation vector (sin/cos)
        Pol2D angle_polar = Pol2D(angle_, INT16_MAX);  // Use class member
        Vec2D angle_sincos = angle_polar.convert();

        // ========================================================================
        // I. Processing current
        // ========================================================================

        // Step 1: Get current ADC values from feedback data
        const int16_t* currents_abc = feedback.current.norm();  // Access current ADC array from feedback

        // Step 2: Get calibrated normalized currents from feedback and convert to AB using motor selector
        const int16_t* normalized_currents = feedback.current.norm();  // Use feedback current sensor
        motor_selector.tick_current(normalized_currents);
        Vec2D current_ab = motor_selector.get_currents();

        // Step 3: Park transform - rotate AB currents by -encoder angle to get DQ
        // Inverse rotation: negate sin component for reverse rotation (cos, -sin)
        Vec2D angle_sincos_inv(angle_sincos.cos, -angle_sincos.sin);
        Vec2D current_dq_raw = current_ab.rotate(angle_sincos_inv);
        current_dq = current_dq_raw;

        // Step 4: Dynamic scale of filtration to make currents more stable at low speeds
        uint8_t alpha = lpf_adaptive_alpha_.tick(abs(angular_speed));
        lpf_id_.set_alpha(alpha);
        lpf_iq_.set_alpha(alpha);

        // Step 5: Apply LPF filtering to measured DQ currents
        int16_t filtered_id = (int16_t)lpf_id_.tick_lpf(med3_id_.tick(current_dq_raw.flux));
        int16_t filtered_iq = (int16_t)lpf_iq_.tick_lpf(med3_iq_.tick(current_dq_raw.torque));
        Vec2D current_dq_filt = Vec2D(filtered_id, filtered_iq);

        // ========================================================================
        // II. Voltage preparation
        // ========================================================================

        int16_t supply_voltage_norm = feedback.supply.get_voltage_norm();
        Vec2D voltage_dq(0, 0);

        Vec2D voltage_ab(0, 0);

        // ========================================================================
        // III. Processing power control
        // ========================================================================

        int32_t power_d = feedback.power.power_norm(filtered_id, voltage_dq_.d);
        int32_t power_q = feedback.power.power_norm(filtered_iq, voltage_dq_.q);

        int32_t power_norm = abs(power_d) + abs(power_q);
        power = feedback.power.norm2mw(power_norm);

        switch (setup_status.get_driver_status()) {
            case OperationStatus::Ready: {
                if (motor_calib_status == MotorStatus::Success) {  // Check if motor calibration is complete

                    // ========================================================================
                    // IV. Processing main FOC loop execution with FOC Current Controller
                    // ========================================================================

                    // System timing parameters
                    const float TS = 1.0f / 20000.0f;  // Sampling period: 50 us (20 kHz PWM frequency)

                    // Voltage limit (60% of supply voltage for safety margin)
                    int16_t v_limit = ((int32_t)feedback.supply.get_voltage_norm() * 3) / 5;  // Q15 normalized voltage limit

                    // ================================================================
                    // Calculate Electrical Angular Velocity (ωe)
                    // ================================================================
                    // ωe = Δθ / Δt, where Δθ is change in electrical angle
                    // Angle is in Q15 format: -32768 to 32767 = -π to π radians
                    // Angular velocity in rad/s = (angular_speed / 32768) * π / TS

                    // Convert angle delta to electrical angular velocity in rad/s
                    // ωe [rad/s] = (angular_speed / 32768) * π / TS
                    // Simplified: ωe = angular_speed * (π / (32768 * TS))
                    // For TS = 50us: ωe = angular_speed * (3.14159 / (32768 * 0.00005)) = angular_speed * 1.917
                    float omega_e_rad_s =
                        (float)angular_speed * (3.14159f / (32768.0f * TS));  // Electrical angular velocity [rad/s]

                    // ================================================================
                    // Convert Target and Measured Currents to Amperes
                    // ================================================================
                    // Current is in Q15 format, convert to physical units (A)
                    
                    float Id_target_a = (float)feedback.current.norm2ma(target_current.d) / 1000.0f;  // D-axis target current [A]
                    float Iq_target_a = (float)feedback.current.norm2ma(target_current.q) / 1000.0f;  // Q-axis target current [A]
                    
                    float Id_measured_a = (float)feedback.current.norm2ma(current_dq_filt.flux) / 1000.0f;  // D-axis measured current [A]
                    float Iq_measured_a = (float)feedback.current.norm2ma(current_dq_filt.torque) / 1000.0f;  // Q-axis measured current [A]

                    // ================================================================
                    // FOC Current Controller Execution
                    // ================================================================
                    // Call FOC current controller with feedforward compensation
                    // Controller automatically handles:
                    // - Resistive voltage drop (R * I)
                    // - Inductive voltage drop (ω * L * I)
                    // - Back-EMF compensation (ω * Ke)
                    // - Cross-coupling terms (d-q interaction)
                    // - PI feedback correction
                    // - Anti-windup protection
                    
                    float bus_voltage_v = (float)feedback.supply.norm2mv(feedback.supply.get_voltage_norm()) / 1000.0f;  // Bus voltage [V]
                    float voltage_limit_fraction = 0.50f;  // 60% voltage limit for safety margin
                    float theta_el_rad = (float)angle_ * (3.14159f / 32768.0f);  // Convert Q15 angle to radians
                    
                    // Execute FOC current controller
                    FocCurrentController::VoltageDQ voltage_dq_result = foc_current_.tick(
                        Id_target_a,           // Target D-axis current [A]
                        Iq_target_a,           // Target Q-axis current [A]
                        Id_measured_a,         // Measured D-axis current [A]
                        Iq_measured_a,         // Measured Q-axis current [A]
                        bus_voltage_v,         // Bus voltage [V]
                        voltage_limit_fraction,// Voltage limit fraction (0.6 = 60%)
                        theta_el_rad,          // Electrical angle [rad]
                        omega_e_rad_s          // Electrical angular velocity [rad/s]
                    );
                    
                    // ================================================================
                    // Convert FOC Controller Output to Q15 Normalized Format
                    // ================================================================
                    // Convert from volts to Q15 normalized format using feedback supply voltage
                    
                    int32_t Vd_mv = (int32_t)(voltage_dq_result.u_d * 1000.0f);  // Convert to millivolts
                    int32_t Vq_mv = (int32_t)(voltage_dq_result.u_q * 1000.0f);  // Convert to millivolts
                    
                    int16_t Vd_norm = feedback.supply.mv2norm(Vd_mv);  // Convert to Q15
                    int16_t Vq_norm = feedback.supply.mv2norm(Vq_mv);  // Convert to Q15
                    
                    // Store final voltage commands
                    voltage_dq = Vec2D(Vd_norm, Vq_norm);
                    break;  // Fast exit from switch
                }

                // Motor calibration in progress - unified motor calibration switch
                // Handles all calibration sub-states
                Vec2D current_ab_filt = current_dq_filt.rotate(angle_sincos);
                const int32_t curnt_a = current_ab_filt.a;
                const int32_t curnt_b = current_ab_filt.b;

                // Correction for sqrt(2)
                int32_t voltage_corrected = (motor_calib.get_voltage() * 46341) >> 16;

                int16_t measured_current = abs(curnt_a + curnt_b);

                switch (motor_calib_status) {
                    // CHECK WARMUP SUB-STATE ////////////////////////////////////////////////
                    case MotorStatus::SoftWarmup: {
                        // Check for overpower using soft limit
                        bool power_lim = feedback.power.is_overpower_soft_norm(power_norm);
                        bool voltg_lim = feedback.supply.is_overvoltage_soft_norm(motor_calib.get_voltage());
                        // bool curnt_lim =  feedback.current.is_overcurrent_soft_norm(measured_current);
                        bool curnt_lim = false;

                        // If current stable or timeout - transition to settle before measure1
                        if ((stability_check.tick(measured_current) and CAL_UNDERPOWER_LIM_NORM < power_norm) or
                            timeout.tick() == 0) {
                            TP_LOG("[CALIB] Warmup complete - current stable");
                            TP_LOG_VAL("  Final voltage: ", motor_calib.get_voltage());
                            timeout.set_timeout(900);                  // Set settle timeout to 900 ticks
                            motor_calib_status = MotorStatus::Settle;  // Transition to settle phase
                            motor.set_status(MotorStatus::Settle);
                        } else {
                            // Check for overcurrent - reduce voltage by increment. If any limit hit, system will bounce around
                            // lim voltage
                            if (power_lim or curnt_lim or voltg_lim) {  // Overpower detected
                                motor_calib.decrease_voltage();         // Reduce by increment
                            } else {
                                // Normal ramp - increase voltage by increment
                                motor_calib.increase_voltage();  // Increase by increment
                            }
                        }
                        break;
                    }

                    // SETTLE SUB-STATE (wait 900 ticks before measurement) ////////////////
                    case MotorStatus::Settle: {
                        // Log settle start on first cycle
                        if (timeout.get_timeout() == 900) {  // First cycle
                            TP_LOG("[CALIB] Settling before measurement (900 ticks)");
                        }

                        // Wait for settle timeout to complete
                        if (timeout.tick() == 0) {  // Settle period complete
                            TP_LOG("[CALIB] Settle complete - starting measurement");
                            motor_calib_status = MotorStatus::ResistanceMeasure;  // Transition to resistance measurement
                            motor.set_status(MotorStatus::ResistanceMeasure);
                            timeout.set_timeout(CAL_OVERSAMPLING_POW2);  // Set measurement cycles
                        }
                        // During settle phase, maintain current voltage and angle
                        break;
                    }

                    case MotorStatus::ResistanceMeasure: {
                        resistance_measure(current_ab, power_norm, voltage_corrected);
                    } break;

                    // SLOW ROTATION SUB-STATE (motor type detection) ///////////////////////
                    case MotorStatus::SlowRotation: {
                        // Check power limit: if power exceeds limit by 25%, throw error
                        if (power >= (test_pwr * 5) / 2) {
                            TP_LOG("[CALIB] ERROR: OP duting rotation");
                            set_calibration_error();  // Power exceeded threshold
                            break;
                        }

                        // Decrement counter and check if rotation complete
                        if (timeout.tick() == 0) {                                  // Counter reached 0 - rotation done
                            angle_ = motor_calib.get_slow_rotation_angle_target();  // Set exact target angle
                            timeout.set_timeout(900);                               // Set settle timeout to 900 ticks
                            motor_calib_status = MotorStatus::Settle;               // Transition to settle phase
                            motor.set_status(MotorStatus::Settle);
                        } else {
                            // Calculate shortest path delta with int16_t overflow (automatic wrap-around)
                            int16_t delta = motor_calib.get_slow_rotation_angle_target() - angle_;  // Shortest path via overflow
                            angle_ += delta / (int16_t)(timeout.get_timeout() + 1);  // Interpolate along shortest path
                        }
                        break;
                    }

                    // INDUCTANCE MEASUREMENT SUB-STATE (HFI-based inductance detection) ////
                    case MotorStatus::InductanceMeasure: {
                        // // Log start of inductance measurement on first cycle
                        // if (timeout.get_timeout() == INDUCTANCE_MEASURE_CYCLES) {  // First cycle
                        //     TP_LOG("[CALIB] Starting inductance measurement (1800 samples @ 1kHz HFI)");
                        //     TP_LOG_VAL("  HFI voltage amplitude: ", motor_calib.get_voltage());
                        //     TP_LOG_VAL("  Angle (D-axis): ", angle_);
                        // }

                        // // Generate 1 kHz HFI signal for D-axis injection
                        // Vec2D hfi_signal = hfi_generator_.tick();  // Get sine and cosine from generator

                        // // Apply HFI voltage to D-axis (cosine component for voltage injection)
                        // int16_t hfi_voltage = (motor_calib.get_voltage() * hfi_signal.cos * 25) >> 15;  // Scale by cosine
                        // (Q15) voltage_dq = Vec2D(hfi_voltage + motor_calib.get_voltage(), 0);  // Inject HFI voltage into
                        // D-axis only

                        // // Get current measurements from motor selector
                        // const int32_t curnt_a = motor_selector.get_currents().alpha;  // Phase A current
                        // const int32_t curnt_b = motor_selector.get_currents().beta;   // Phase B current

                        // // Demodulate HFI response using sine component (90° phase shift for inductive response)
                        // hfi_sense_a_.tick((int16_t)curnt_a, hfi_signal.cos);  // Demodulate phase A with sine reference
                        // hfi_sense_b_.tick((int16_t)curnt_b, hfi_signal.cos);  // Demodulate phase B with sine reference

                        // // Accumulate demodulated HFI responses for averaging
                        // int16_t hfi_response_a = hfi_sense_a_.get_output();                  // Get demodulated signal A
                        // int16_t hfi_response_b = hfi_sense_b_.get_output();                  // Get demodulated signal B
                        // // motor_calib.accumulate_inductance(hfi_response_a, hfi_response_b);  // Accumulate for averaging

                        // // Check if measurement complete (1800 cycles = 100 periods @ 1kHz)
                        // if (timeout.tick() == 0) {  // Counter reached 0 - measurement done
                        //     // Calculate inductance from averaged HFI response
                        //     // L = V / (2 * pi * f * I_hfi), where f = 1kHz
                        //     // Simplified: L_norm = HFI_response (proportional to inductance)
                        //     int32_t inductance_a = motor_calib.get_inductance_a();  // Get averaged inductance A
                        //     int32_t inductance_b = motor_calib.get_inductance_b();  // Get averaged inductance B

                        //     // Log inductance measurement results
                        //     TP_LOG("[CALIB] Inductance measurement complete:");
                        //     TP_LOG_VALS("  HFI response A/B (normalized): ", inductance_a, inductance_b);

                        //     // Store inductance values in motor structure (in microhenries, scaled from normalized)
                        //     // Conversion: L_uH = (V_norm * 1000) / (2 * pi * 1000 * I_norm) = V_norm / (6.28 * I_norm)
                        //     // Simplified proportional storage for now (actual calibration factor TBD)
                        //     motor.set_phase_inductance(0, inductance_a);  // Store phase A inductance
                        //     motor.set_phase_inductance(1, inductance_b);  // Store phase B inductance

                        //     TP_LOG_VALS("  Stored inductance A/B (uH): ", inductance_a, inductance_b);
                        //     TP_LOG("[CALIB] Motor calibration complete - transitioning to Ready");

                        //     // // Inductance measurement complete - transition to Ready state
                        //     // switch_status(DriverStatus_Ready);  // Calibration complete
                        //     // motor_calib.reset_voltage();       // Reset voltage to zero
                        //     // voltage_ab = Vec2D(0, 0);           // Zero output voltage
                        // }
                        break;
                    }

                    default:
                        // Unknown calibration state - zero output
                        voltage_ab = Vec2D(0, 0);
                        break;
                }

                // Set voltage_dq for all calibration states except inductance measurement
                // (inductance measurement sets voltage_dq internally with HFI modulation)
                if (motor_calib_status != MotorStatus::InductanceMeasure) {
                    voltage_dq = Vec2D(motor_calib.get_voltage(), 0);  // D axis injection
                }
                break;
            }
                ///////////////////////////////////////////////////////////////////////////////
            // CURRENT SENSOR OFFSET CALIBRATION SEQUENCE /////////////////////////////////
            ///////////////////////////////////////////////////////////////////////////////
            case OperationStatus::DriverCalibration:
                // During calibration: first 50 cycles with zero PWM to ensure charge of bootstrap circuit
                if (timeout.tick() > 0) {
                    // Phase 1: Zero PWM for 50 cycles
                    output.ch1 = 0;
                    output.ch2 = 0;
                    output.ch3 = 0;
                    return;
                }  // When charging complete - making all outputs high to ensure 0 current flow through low side sense
                else if (timeout.get_timeout() == 0) {
                    // Start calibration for all channels using feedback current sensor
                    // Set calibration samples to 100 for feedback current sensor
                    feedback.current.calibrate();  // Start calibration through friend access
                    timeout.tick();
                    // Phase 2: Max PWM during current sensor sampling
                    output.ch1 = INT16_MAX;
                    output.ch2 = INT16_MAX;
                    output.ch3 = INT16_MAX;
                    return;
                }  // If calibration is not complete skip this turn
                else if (!feedback.current.is_calibrated()) {
                    return;
                }  // If calibration complete switching to next step
                else {
                    // Calibration complete, update setup_status and transition to motor calibration phase
                    setup_status.current_not_calibrated = 0;  // Clear flag (current sensors calibrated)

                    // Check if current sensors are functional by verifying calibration values
                    bool sensors_ok = true;  // Assume sensors are functional
                    // TODO: Add actual sensor health check logic here
                    setup_status.current_sensors_faulty = sensors_ok ? 0 : 1;  // Update sensor health flag

                    // Transition to motor calibration
                    setup_status.set_driver_status(OperationStatus::Ready);
                    start_motor_calibration();  // Start motor calibration sequence
                }
                break;

            default:
                output.ch1 = INT16_MIN;
                output.ch2 = INT16_MIN;
                output.ch3 = INT16_MIN;
                return;
        }

        // Step 7: Inverse Park transform - rotate DQ voltages back to AB
        voltage_dq_ = voltage_dq;

        voltage_ab = voltage_dq.rotate(angle_sincos);

        Vec2D current_ab_filt = current_dq_filt.rotate(angle_sincos);

        voltage_ab.sin = feedback.supply.norm2duty(voltage_ab.sin);
        voltage_ab.cos = feedback.supply.norm2duty(voltage_ab.cos);

        // if (compensation_current == 1) {
        //     int16_t dt_correction_sincos =
        //         deadtime_compensate_sqrt(current_ab_filt.cos + current_ab_filt.sin, voltage_ab.cos + voltage_ab.sin,
        //                                  deadzone_sincos, i_deadband_sincos, dead_time_duty);
        //     int16_t dt_correction_sin =
        //         deadtime_compensate_sqrt(current_ab_filt.sin, voltage_ab.sin, deadzone, i_deadband, dead_time_duty) +
        //         dt_correction_sincos;
        //     int16_t dt_correction_cos =
        //         deadtime_compensate_sqrt(current_ab_filt.cos, voltage_ab.cos, deadzone, i_deadband, dead_time_duty) +
        //         dt_correction_sincos;

        //     voltage_ab.sin += dt_correction_sin;
        //     voltage_ab.cos += dt_correction_cos;
        // }

        voltage_ab_ = voltage_ab;

        // Convert AB voltages to ABC phase voltages with phase remapping
        motor_selector.tick_voltage(voltage_ab, feedback.supply.get_voltage_norm());
        const int16_t* voltages = motor_selector.get_voltages();

        // Check for overcurrent or overvoltage errors and override PWM values
        // OV has higher priority than OC
        if (feedback.error.get_fatal_dc_ovlo()) {
            // Overvoltage: Force regenerative braking (low-side ON, high-side OFF)
            // Set all phases to 0 (negative duty = low-side active)
            output[0] = 0;  // Phase A: regenerative braking
            output[1] = 0;  // Phase B: regenerative braking
            output[2] = 0;  // Phase C: regenerative braking
        } else if (feedback.error.get_fatal_oc()) {
            // Overcurrent: Force all outputs to Hi-Z (all FETs OFF)
            // Set all phases to INT16_MIN (inactive state)
            output[0] = INT16_MIN;  // Phase A: Hi-Z
            output[1] = INT16_MIN;  // Phase B: Hi-Z
            output[2] = INT16_MIN;  // Phase C: Hi-Z
        } else {
            // No error: Use normal PWM values from motor selector
            output[0] = voltages[0];  // Phase A: normal operation
            output[1] = voltages[1];  // Phase B: normal operation
            output[2] = voltages[2];  // Phase C: normal operation
        }
    }

    /**
     * @param current       Текущий ток (int16_t)
     * @param threshold     Пороговое значение тока (int16_t, например 100)
     * @param compensation  Полная величина компенсации (int16_t)
     * @return              Скорректированная компенсация
     */
    int16_t deadtime_compensate_sqrt(int16_t current, int16_t voltage, int16_t threshold, int16_t i_dband, int16_t compensation) {
        if (threshold <= 0 || compensation == 0) {
            return 0;
        }
        int32_t cur = current;

        int8_t current_sign = (cur >= 0) ? 1 : -1;
        int8_t voltage_sign = (voltage >= 0) ? 1 : -1;

        int32_t abs_current = cur * current_sign;

        if (abs_current >= threshold) {
            return current_sign * compensation;
        } else if (abs_current <= i_dband) {
            return 0;
        } else {
            // Переводим в float для расчета корня
            float ratio = (float)(abs_current - i_dband) / (float)(threshold - i_dband);
            // sqrtf() компилируется напрямую в инструкцию VSQRT.F32
            // при правильных настройках FPU (-mfloat-abi=hard -mfpu=fpv4-sp-d16)
            float f = sqrtf(ratio);
            // Масштабируем и возвращаем в целые числа
            int16_t scaled_comp = (int16_t)((float)compensation * f);
            return current_sign * scaled_comp;
        }
    }

    void set_calibration_error() {
        setup_status.set_driver_status(OperationStatus::Error);  // Set driver status to Error
        motor_calib_status = MotorStatus::CalibError;            // Set calibration error
        motor.set_status(MotorStatus::CalibError);               // Update motor status
        motor_calib.reset_voltage();
        voltage_ab_ = Vec2D(0, 0);  // Reset local debug var
    }

    // OPTIMIZATION: Helper for success transition - now transitions to inductance measurement
    void set_calibration_success() {
        motor.set_motor_type(motor_selector.get_motor_type());        // Save detected motor type
        motor.set_phase_pattern(motor_selector.get_phase_pattern());  // Save phase pattern

        // Update setup_status flags after successful motor calibration
        setup_status.motor_type_not_verified = 0;  // Clear flag (motor type verified)

        // Check voltage safety (example: voltage should be within safe range)
        int16_t voltage_norm = feedback.supply.get_voltage_norm();
        bool voltage_safe = (voltage_norm > 0) && (voltage_norm < INT16_MAX);  // Basic safety check
        setup_status.voltage_unsafe = voltage_safe ? 0 : 1;                    // Update voltage safety flag

        // Mark motor calibration as complete (Success = 0)
        motor_calib_status = MotorStatus::Success;  // Calibration complete
        motor.set_status(MotorStatus::Success);     // Update motor status

        // ================================================================
        // Initialize FOC Current Controller with Motor Parameters
        // ================================================================
        // Motor parameters (SI units):
        // - Phase resistance: 2.4 Ω (measured during calibration)
        // - Phase inductance: 3.7 mH = 0.0037 H
        // - Torque at 1.5A: 4.2 kg·cm = 0.042 N·m
        // - Stepper motor: 50 pole pairs (100 poles, 200 steps/rev)
        
        // Convert measured resistance from milliohms to ohms
        float R_ohm = (float)motor.get_total_resistance() / 1000.0f;  // [Ω]
        
        // Convert inductance from microhenries to henries
        float L_h = (float)motor.get_effective_inductance() / 1000000.0f;  // [H]
        
        // Calculate motor parameters for FOC controller
        // Torque constant: Kt = Torque / Current = 0.042 N·m / 1.5 A = 0.028 N·m/A
        const float torque_nm = 0.042f;      // Torque at rated current [N·m]
        const float rated_current_a = 1.5f;  // Rated current [A]
        
        // Pole pairs for stepper motor (50 pole pairs = 100 poles)
        const float pole_pairs = 50.0f;
        
        // Desired current loop bandwidth (Hz) - aggressive tuning for fast response
        const float current_loop_bw_hz = 1500.0f;  // 1.2 kHz bandwidth
        
        // Initialize FOC current controller with motor parameters
        foc_current_ = FocCurrentController(
            R_ohm,                // Phase resistance [Ω]
            L_h,                  // Phase inductance [H]
            torque_nm,            // Max torque [N·m]
            rated_current_a,      // Current at max torque [A]
            pole_pairs,           // Number of pole pairs
            current_loop_bw_hz    // Current loop bandwidth [Hz]
        );
        
        TP_LOG("[FOC] Current controller initialized:");
        TP_LOG_VAL("  R (Ω): ", (int32_t)(R_ohm * 1000.0f));  // Log in milliohms
        TP_LOG_VAL("  L (μH): ", motor.get_effective_inductance());
        TP_LOG_VAL("  Kt (mN·m/A): ", (int32_t)(torque_nm / rated_current_a * 1000.0f));
        TP_LOG_VAL("  Pole pairs: ", (int32_t)pole_pairs);
        TP_LOG_VAL("  BW (Hz): ", (int32_t)current_loop_bw_hz);

        // Set driver status to Ready (bits 0-3 = 0)
        setup_status.set_driver_status(OperationStatus::Ready);  // Driver now ready
    }

    inline void resistance_measure(Vec2D current_ab, int32_t power_norm, int32_t voltage_corrected) {
        // First cycle: reset accumulators (counter == 100 initially)
        if (timeout.get_timeout() == CAL_OVERSAMPLING_POW2) {                // First cycle
            motor_calib.reset_measure_accumulators();                        // Reset all accumulators
            TP_LOG("[CALIB] Starting resistance measurement (64 samples)");  // Log measurement start
        }

        // Accumulate current samples for averaging
        motor_calib.accumulate_measure_currents(current_ab.a,
                                                current_ab.b);  // Add to accumulators
                                                                // Decrement counter and check if measurement complete
        if (timeout.tick() == 0) {                              // Counter reached 0 - 100 cycles done
            int32_t ia = motor_calib.get_measure_current_a();
            int32_t ib = motor_calib.get_measure_current_b();
            // Too low power, possibly no connection OR not enough power
            if (CAL_UNDERPOWER_LIM_NORM >= power_norm) {
                // No connection or DC motor branch
                if (motor_selector.get_motor_type() == MotorType_DC) {
                    // we set ABC in else branch, so its easy way to check second entry
                    TP_LOG("[CALIB] Error:");
                    TP_LOG_VALS("  Current A/B (mA): ", feedback.current.norm2ma(ia), feedback.current.norm2ma(ib));
                    TP_LOG_VAL("  Voltage (mV): ", feedback.supply.norm2mv((uint32_t)voltage_corrected));
                    TP_LOG_VAL("  Angle: ", angle_);
                    set_calibration_error();
                } else if (motor.get_total_resistance() != 0) {
                    motor_selector.set_motor_type(MotorType_DC);
                    if (motor_selector.get_phase_pattern() == PhasePattern_ACB)
                        motor_selector.set_phase_pattern(PhasePattern_CBA);
                    else
                        motor_selector.set_phase_pattern(PhasePattern_ACB);
                    TP_LOG_VAL(" DC motor: ", motor_selector.get_phase_pattern());
                    set_calibration_success();
                } else {
                    // retry because it may be DC motor
                    motor_selector.set_motor_type(MotorType_DC);
                    motor_calib.reset_voltage();  // Reset voltage
                    stability_check.reset();
                    motor_calib_status = MotorStatus::SoftWarmup;  // Restart warmup
                    motor.set_status(MotorStatus::SoftWarmup);
                    timeout.set_timeout(WARMUP_TIMEOUT - 1);
                    TP_LOG("[CALIB] Warmup again");
                    // we are far away from target power, something close to zero current
                    // probably DC motor in different connection
                }
            } else {  // we have enough power

                int32_t v_uv = feedback.supply.norm2mv((uint32_t)voltage_corrected) * 1000;

                int32_t resistance_a = v_uv / feedback.current.norm2ma(ia);  // Resistance in milliohms

                int32_t resistance_b = v_uv / feedback.current.norm2ma(ib);  // Resistance in milliohms

                // Log measured currents and resistances
                TP_LOG("[CALIB] Resistance measurement complete:");
                TP_LOG_VALS("  Current A/B (mA): ", feedback.current.norm2ma(ia), feedback.current.norm2ma(ib));
                TP_LOG_VALS("  Resistance A/B (mOhm): ", resistance_a, resistance_b);
                TP_LOG_VAL("  Voltage (mV): ", feedback.supply.norm2mv((uint32_t)voltage_corrected));
                TP_LOG_VAL("  Angle: ", angle_);

                if (angle_ == 16384) {
                    if (14 * abs((2 * ia) + ib) < ((-ia * 2) + ib)) {
                        // BLDC motor second pass: (phases equeal within ~15% from average)
                        // if current a = sum of b and c, where ib ~= ic
                        // Expected: iA - negative and dominant
                        // iB - positive and equal to half of iA amplitude
                        int32_t avg_r = (motor.get_total_resistance() + resistance_a) / 3;
                        motor.set_total_resistance(avg_r);

                        motor_selector.set_motor_type(MotorType_BLDC);
                        TP_LOG("[CALIB] Motor type detected: BLDC");
                        TP_LOG_VAL("  Average resistance (mOhm): ", avg_r);
                        set_calibration_success();
                    }

                    else if ((ib >> 4) > ia) {
                        // Stepper motor second pass:
                        // Check if iA much greater than iB
                        // Expected: iA - negative dominant, iB close to 0 (any sign)
                        // Resistance already calculated
                        TP_LOG("[CALIB] Motor type detected: STEPPER");
                        TP_LOG_VAL("  Total resistance (mOhm): ", motor.get_total_resistance());
                        set_calibration_success();
                    } else {
                        TP_LOG("[CALIB] ERROR: Motor type detection failed at angle 16384");
                        set_calibration_error();
                    }
                }  // Check if angle is around +225° (on second path for Stepper or DC motors)
                else if (angle_ == -24576) {
                    // Check if b and c are balanced (~15% from average)
                    if (10 * abs(ib - ia) < (-ia - ib)) {
                        // Stepper motor

                        int32_t avg_r = -(resistance_a + resistance_b) / 2;
                        motor.set_total_resistance(avg_r);
                        TP_LOG("[CALIB] Motor type detected: STEPPER (balanced phases)");
                        TP_LOG_VAL("  Average resistance (mOhm): ", avg_r);
                        set_calibration_success();
                    } else {
                        TP_LOG("[CALIB] ERROR: Motor type detection failed at angle -24576");
                        set_calibration_error();
                    }
                } else if (angle_ == CAL_ANGLE_45D) {
                    timeout.set_timeout(1800);  // Set counter to cycles
                    if (10 * abs(ib - ia) < (ia + ib)) {
                        // Stepper motor or BLDC first pass where phases are balanced (~15% from average)
                        motor.set_total_resistance((resistance_a + resistance_b) / 2);

                        TP_LOG("[CALIB] Phases balanced - preparing rotation for motor type verification");
                        TP_LOG_VAL("  Temporary resistance (mOhm): ", motor.get_total_resistance());

                        // Rotate angle by 30° slowly to verify
                        motor_calib.start_slow_rotation(16384, power);   // Target: current + 30°
                        motor_calib_status = MotorStatus::SlowRotation;  // Set slow rotation status
                        motor.set_status(MotorStatus::SlowRotation);
                        // set_calibration_success(); // TODO: REMOVE
                    }  // Check if angle is around 90° (angle woks on second pass for BLDC and stepper in correct
                       // phase)
                    else {
                        motor_calib.set_voltage((int16_t)voltage_corrected);
                        motor_calib.start_slow_rotation(-24576, power);
                        motor_calib_status = MotorStatus::SlowRotation;  // Set slow rotation status
                        motor.set_status(MotorStatus::SlowRotation);
                        if (motor_selector.get_motor_type() == MotorType_DC) {
                            // DC motor branch - connection ABC
                            motor.set_total_resistance(-resistance_a);
                            TP_LOG("[CALIB] Motor type detected: DC (ABC connection)");
                            TP_LOG_VAL("  Resistance (mOhm): ", -resistance_a);
                            set_calibration_success();
                        } else if ((ia >> 2) >= ib) {
                            // motor type - Stepper or DC
                            // If next time we hit underpower (shouldnt happen for Stepper - then we go to DC
                            // motor)
                            motor.set_total_resistance(resistance_a);
                            TP_LOG("[CALIB] Phase B dominant - setting CBA pattern and rotating");
                            angle_ = PhasePattern_CBA;
                            motor_selector.set_phase_pattern(PhasePattern_CBA);

                        } else if ((ib >> 2) >= ia) {
                            TP_LOG("[CALIB] Phase C dominant - setting ACB pattern and rotating");
                            // If next time we hit underpower (shouldnt happen for Stepper - then we go to DC
                            // motor)
                            motor.set_total_resistance(resistance_b);
                            motor_selector.set_phase_pattern(PhasePattern_ACB);
                            angle_ = PhasePattern_ACB;
                            // Phase C dominant: motor type - Stepper or DC
                            // Set counter to cycles

                        } else {
                            TP_LOG("[CALIB] ERROR: Excessive deviation - unknown motor type - wrong 1st loop");
                            set_calibration_error();  // motor looks so strange
                        }
                    }
                } else {
                    TP_LOG("[CALIB] ERROR: Excessive deviation - unknown motor type1");
                    set_calibration_error();  // motor looks so strange
                }
            }
        }
    }
};
