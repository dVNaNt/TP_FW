#pragma once

// 3-phase PWM driver with FOC (Field Oriented Control) current regulation

#include "../../common.h"
#include "../../feedback/feedback.h"
#include "../../math_integer/helpers/frequency_generator.h"  // Frequency generator for HFI
#include "../../math_integer/helpers/speaker.h"              // Motor speaker for audio feedback
#include "../../math_integer/helpers/stability_check.h"      // Stability check for current monitoring
#include "../../math_integer/helpers/timeout_timer.h"        // Timeout timer for countdown operations
#include "../motor.h"                                        // Motor parameters structure
#include "hfi_sense.h"                                       // HFI sense for inductance measurement
#include "sel_motor.h"

// Motor test power should be between 10% to 40% of total motor power
static constexpr int32_t test_pwr = 3000;

// Calibration logging macro (uses SerialUSB if available)
#ifdef ARDUINO
#include <Arduino.h>
#define CALIB_LOG(msg) SerialUSB.println(msg)
#define CALIB_LOG_VAL(msg, val) \
    do {                        \
        SerialUSB.print(msg);   \
        SerialUSB.println(val); \
    } while (0)
#define CALIB_LOG_VAL2(msg, val1, val2) \
    do {                                \
        SerialUSB.print(msg);           \
        SerialUSB.print(val1);          \
        SerialUSB.print(", ");          \
        SerialUSB.println(val2);        \
    } while (0)
#else
#define CALIB_LOG(msg)
#define CALIB_LOG_VAL(msg, val)
#define CALIB_LOG_VAL2(msg, val1, val2)
#endif

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
public:
    // Calibration status enum - accessible as MotorCalibration::Status
    enum Status : uint8_t {
        Status_Idle,              // Not calibrating
        Status_CheckWarmup,       // Checking motor warmup and connection
        Status_Settle,            // Settling phase before measurement (900 ticks)
        Status_SlowRotation,      // Slow rotation for motor type detection
        Status_Complete,          // Calibration complete
        Status_Error,             // Calibration error
        Status_Measure,           // Measuring resistance
        Status_InductanceMeasure  // Measuring inductance using HFI
    };

private:
    Status status_;  // Current calibration status

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
        : status_(Status_Idle),
          voltage_(0),
          voltage_increment_(2),
          voltage_limit_(INT16_MAX / 2),
          overcurrent_threshold_(INT16_MAX / 2),
          slow_rotation_angle_target_(0),
          slow_rotation_power_limit_(0),
          measure_current_a_sum_(0),
          measure_current_b_sum_(0),
          inductance_a_sum_(0),
          inductance_b_sum_(0) {}

    // Get current calibration status
    Status get_status() const { return status_; }

    // Set calibration status
    void set_status(Status new_status) { status_ = new_status; }

    // Start slow rotation phase
    void start_slow_rotation(int16_t target_angle, int32_t power_limit) {
        slow_rotation_angle_target_ = target_angle;  // Set target angle directly
        slow_rotation_power_limit_ = power_limit;    // Save power limit
        set_status(Status_SlowRotation);             // Transition to rotation
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
    Motor& motor_;

private:
    // Reference to parent Feedback
    Feedback& feedback_;

    MotorSelector motor_selector_;  // Unified motor selector (voltage + current + phase)

    // PI controllers for d and q axis current control
    PID_Int pid_d_;
    PID_Int pid_q_;

    // LPF filters for d and q axis current measurements
    Filter1st lpf_id_;
    Filter1st lpf_iq_;

    // Used to charge mosfet driver before going high on all channels
    static constexpr uint32_t CALIBRATION_ZERO_CYCLES = 50;  // Cycles with zero PWM

    // Motor calibration structure with all calibration-related variables
    MotorCalibration motor_calib_;

    // Motor speaker for audio feedback after calibration
    MotorSpeaker speaker_;

    static constexpr uint32_t CALIB_PAUSE_CYCLES = 1800;         // 100ms pause @ 18kHz
    static constexpr uint32_t INDUCTANCE_MEASURE_CYCLES = 1800;  // 100 periods @ 1kHz (18kHz / 1kHz * 100)

    int16_t angle_;                   // Current rotor angle (Q15, -32768 to 32767 = -π to π)
    StabilityCheck stability_check_;  // Stability checker for current monitoring
    timeout_timer timeout;            // Timeout timer for general purpose countdown operations

    // HFI components for inductance measurement
    FrequencyGenerator hfi_generator_;  // 1 kHz frequency generator for HFI injection
    HFISense hfi_sense_a_;              // HFI demodulator for phase A
    HFISense hfi_sense_b_;              // HFI demodulator for phase B

    Vec3D output;
    int32_t power;

    Vec2D current_dq;

    // Debug/monitoring
    Vec2D voltage_ab_;  // Last voltage command
    Vec2D voltage_dq_;  // Last voltage command

public:
    // Constructor: takes Motor reference from parent (TunePulseHeart owns Motor)
    DriverPWM(Feedback& parent, Motor& motor_ref)
        : motor_(motor_ref),
          power(0),
          feedback_(parent),
          motor_selector_(parent, MotorType_STEP, PhasePattern_ABC, 995u, 256),

          pid_d_(0, 0, 0, 0)  // Default PI gains: kp=10%, ki=5%, kd=0, kff=0
          ,
          pid_q_(0, 0, 0, 0),
          lpf_id_(0, 220)  // LPF for id with alpha=64 (25% filtering)
          ,
          lpf_iq_(0, 220)  // LPF for iq with alpha=64 (25% filtering)
          ,
          motor_calib_(),        // Initialize motor calibration structure
          speaker_(18000, 3276)  // Speaker: 20kHz system, ~10% amplitude
          ,
          angle_(0)  // Initialize angle to zero
          ,
          stability_check_(50, 240)  // Initialize stability checker: epsilon=50, window=180 cycles
          ,
          timeout(CALIB_PAUSE_CYCLES)  // Initialize timeout timer to 100ms @ 18kHz
          ,
          hfi_generator_(18000)  // Initialize HFI generator at 18 kHz system frequency
          ,
          hfi_sense_a_(1000, 18000)  // Initialize HFI sense A: 1 kHz carrier, 18 kHz system
          ,
          hfi_sense_b_(1000, 18000)  // Initialize HFI sense B: 1 kHz carrier, 18 kHz system
          ,
          voltage_ab_{0, 0},
          voltage_dq_{0, 0},
          current_dq{0, 0},
          output{INT16_MIN, INT16_MIN, INT16_MIN} {
        // Set calibration samples to 100 for feedback current sensor
        parent.current.set_calibration_samples(CAL_OVERSAMPLING_POW2);  // Configure calibration samples
    }

    // Convenience overload: takes raw id/iq and encoder angle with Feedback reference
    void tick_control_foc(int16_t angle, int16_t target_current, const Feedback& feedback) {
        // Store angle in class member for global access
        // Save input angle

        // Ensuring that we have full internal control over angle until driver is ready
        if (motor_.get_status() == DriverStatus_Ready) {
            angle_ = angle;  // Override to zero
        }
        // Get current ADC values from feedback data
        const int16_t* currents_abc = feedback.current.norm();  // Access current ADC array from feedback
        int16_t supply_voltage_norm = feedback.supply.get_voltage_norm();

        // Step 1: Convert encoder angle to rotation vector (sin/cos)
        Pol2D angle_polar = Pol2D(angle_, INT16_MAX);  // Use class member
        Vec2D angle_sincos = angle_polar.convert();

        // Step 2: Get calibrated normalized currents from feedback and convert to AB using motor selector
        const int16_t* normalized_currents = feedback_.current.norm();  // Use feedback current sensor
        motor_selector_.tick_current(normalized_currents);
        Vec2D current_ab = motor_selector_.get_currents();

        // Step 3: Park transform - rotate AB currents by -encoder angle to get DQ
        // Inverse rotation: negate sin component for reverse rotation (cos, -sin)
        Vec2D angle_sincos_inv(angle_sincos.cos, -angle_sincos.sin);
        Vec2D current_dq_raw = current_ab.rotate(angle_sincos_inv);
        current_dq = current_dq_raw;

        // Step 3.5: Apply LPF filtering to measured DQ currents
        int16_t filtered_id = (int16_t)lpf_id_.tick_lpf((uint16_t)(current_dq_raw.flux));
        int16_t filtered_iq = (int16_t)lpf_iq_.tick_lpf((uint16_t)(current_dq_raw.torque));
        Vec2D current_dq_filt = Vec2D(filtered_id, filtered_iq);

        Vec2D voltage_dq(0, 0);

        Vec2D voltage_ab(0, 0);

        int32_t power_d = feedback_.power.power_norm(filtered_id, voltage_dq_.d);
        int32_t power_q = feedback_.power.power_norm(filtered_iq, voltage_dq_.q);

        int32_t power_norm = abs(power_d) + abs(power_q);
        power = feedback_.power.norm2mw(power_norm);

        switch (motor_.get_status()) {
            ///////////////////////////////////////////////////////////////////////////////
            // CURRENT SENSOR OFFSET CALIBRATION SEQUENCE /////////////////////////////////
            ///////////////////////////////////////////////////////////////////////////////
            case DriverStatus_CalibrateCurrent:
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
                    feedback_.current.calibrate();  // Start calibration through friend access
                    timeout.tick();
                    // Phase 2: Max PWM during current sensor sampling
                    output.ch1 = INT16_MAX;
                    output.ch2 = INT16_MAX;
                    output.ch3 = INT16_MAX;
                    return;
                }  // If calibration is not complete skip this turn
                else if (!feedback_.current.is_calibrated()) {
                    return;
                }  // If calibration complete switching to next step
                else {
                    // Calibration complete, transition to motor calibration phase
                    switch_status(DriverStatus_CalibratingMotor);                   // Transition to motor calibration
                    motor_calib_.set_status(MotorCalibration::Status_CheckWarmup);  // Set calibration status
                    feedback_.current.set_overcurrent_hard_ma(2000);
                    timeout.set_timeout(WARMUP_TIMEOUT);
                    voltage_ab = Vec2D(0, 0);
                }
                break;

            ///////////////////////////////////////////////////////////////////////////////
            // MOTOR CALIBRATION PHASE (unified for all motor calibration states) ////
            ///////////////////////////////////////////////////////////////////////////////
            case DriverStatus_CalibratingMotor: {
                // Unified motor calibration switch - handles all calibration sub-states
                const int32_t curnt_a = motor_selector_.get_filtered_a();
                const int32_t curnt_b = motor_selector_.get_filtered_b();

                int32_t voltage_corrected = (motor_calib_.get_voltage() * 46341) >> 16;

                int16_t measured_current = abs(curnt_a + curnt_b);

                switch (motor_calib_.get_status()) {
                    // CHECK WARMUP SUB-STATE ////////////////////////////////////////////////
                    case MotorCalibration::Status_CheckWarmup: {
                        // Get measured current (use D-axis current from DQ transform)
                        if (timeout.get_timeout() == WARMUP_TIMEOUT) {
                            CALIB_LOG("[CALIB] Starting motor warmup and connection check");
                            // Setup stability checker for warmup phase
                            stability_check_.reset();
                            // Configure soft limits for calibration
                            feedback_.power.set_overpower_soft_mw(test_pwr);  // Set soft power limit to 2W
                            feedback_.supply.set_overvoltage_soft_norm(feedback_.supply.get_voltage_norm() / 2);
                            // feedback_.current.set_overcurrent_soft_norm(INT16_MAX / 4);
                            motor_selector_.set_motor_type(MotorType_STEP);
                            motor_selector_.set_phase_pattern(PhasePattern_ABC);
                            motor_calib_.reset_voltage();
                            angle_ = CAL_ANGLE_45D;
                            timeout.tick();
                            break;
                        }

                        // Check for overpower using soft limit
                        bool power_lim = feedback_.power.is_overpower_soft_norm(power_norm);
                        bool voltg_lim = feedback_.supply.is_overvoltage_soft_norm(motor_calib_.get_voltage());
                        // bool curnt_lim = feedback_.current.is_overcurrent_soft_norm(measured_current);
                        bool curnt_lim = false;

                        // If current stable or timeout - transition to settle before measure1
                        if ((stability_check_.tick(measured_current) and CAL_UNDERPOWER_LIM_NORM < power_norm) or
                            timeout.tick() == 0) {
                            CALIB_LOG("[CALIB] Warmup complete - current stable");
                            CALIB_LOG_VAL("  Final voltage: ", motor_calib_.get_voltage());
                            timeout.set_timeout(900);                                  // Set settle timeout to 900 ticks
                            motor_calib_.set_status(MotorCalibration::Status_Settle);  // Transition to settle phase
                        } else {
                            // Check for overcurrent - reduce voltage by increment. If any limit hit, system will bounce around
                            // lim voltage
                            if (power_lim or curnt_lim or voltg_lim) {  // Overpower detected
                                motor_calib_.decrease_voltage();        // Reduce by increment
                            } else {
                                // Normal ramp - increase voltage by increment
                                motor_calib_.increase_voltage();  // Increase by increment
                            }
                        }
                        break;
                    }

                    // SETTLE SUB-STATE (wait 900 ticks before measurement) ////////////////
                    case MotorCalibration::Status_Settle: {
                        // Log settle start on first cycle
                        if (timeout.get_timeout() == 900) {  // First cycle
                            CALIB_LOG("[CALIB] Settling before measurement (900 ticks)");
                        }

                        // Wait for settle timeout to complete
                        if (timeout.tick() == 0) {  // Settle period complete
                            CALIB_LOG("[CALIB] Settle complete - starting measurement");
                            motor_calib_.set_status(MotorCalibration::Status_Measure);
                            timeout.set_timeout(CAL_OVERSAMPLING_POW2);  // Set measurement cycles
                        }
                        // During settle phase, maintain current voltage and angle
                        break;
                    }

                    case MotorCalibration::Status_Measure: {
                        // First cycle: reset accumulators (counter == 100 initially)
                        if (timeout.get_timeout() == CAL_OVERSAMPLING_POW2) {                   // First cycle
                            motor_calib_.reset_measure_accumulators();                          // Reset all accumulators
                            CALIB_LOG("[CALIB] Starting resistance measurement (64 samples)");  // Log measurement start
                        }

                        // Accumulate current samples for averaging
                        motor_calib_.accumulate_measure_currents(
                            current_ab.a, current_ab.b);  // Add to accumulators
                                                          // Decrement counter and check if measurement complete
                        if (timeout.tick() == 0) {        // Counter reached 0 - 100 cycles done
                            int32_t ia = motor_calib_.get_measure_current_a();
                            int32_t ib = motor_calib_.get_measure_current_b();
                            // Too low power, possibly no connection OR not enough power
                            if (CAL_UNDERPOWER_LIM_NORM >= power_norm) {
                                // No connection or DC motor branch
                                if (motor_selector_.get_motor_type() == MotorType_DC) {
                                    // we set ABC in else branch, so its easy way to check second entry
                                    CALIB_LOG("[CALIB] Error:");
                                    CALIB_LOG_VAL2("  Current A/B (mA): ", feedback_.current.norm2ma(ia),
                                                   feedback_.current.norm2ma(ib));
                                    CALIB_LOG_VAL("  Voltage (mV): ", feedback_.supply.norm2mv((uint32_t)voltage_corrected));
                                    CALIB_LOG_VAL("  Angle: ", angle_);
                                    set_calibration_error();
                                } else if (motor_.get_total_resistance() != 0) {
                                    motor_selector_.set_motor_type(MotorType_DC);
                                    if (motor_selector_.get_phase_pattern() == PhasePattern_ACB)
                                        motor_selector_.set_phase_pattern(PhasePattern_CBA);
                                    else
                                        motor_selector_.set_phase_pattern(PhasePattern_ACB);
                                    CALIB_LOG_VAL(" DC motor: ", motor_selector_.get_phase_pattern());
                                    set_calibration_success();
                                } else {
                                    // retry because it may be DC motor
                                    motor_selector_.set_motor_type(MotorType_DC);
                                    motor_calib_.reset_voltage();  // Reset voltage
                                    stability_check_.reset();
                                    motor_calib_.set_status(MotorCalibration::Status_CheckWarmup);  // Set calibration status
                                    timeout.set_timeout(WARMUP_TIMEOUT - 1);
                                    CALIB_LOG("[CALIB] Warmup again");
                                    // we are far away from target power, something close to zero current
                                    // probably DC motor in different connection
                                }
                            } else {  // we have enough power

                                int32_t v_uv = feedback_.supply.norm2mv((uint32_t)voltage_corrected) * 1000;

                                int32_t resistance_a = v_uv / feedback_.current.norm2ma(ia);  // Resistance in milliohms

                                int32_t resistance_b = v_uv / feedback_.current.norm2ma(ib);  // Resistance in milliohms

                                // Log measured currents and resistances
                                CALIB_LOG("[CALIB] Resistance measurement complete:");
                                CALIB_LOG_VAL2("  Current A/B (mA): ", feedback_.current.norm2ma(ia),
                                               feedback_.current.norm2ma(ib));
                                CALIB_LOG_VAL2("  Resistance A/B (mOhm): ", resistance_a, resistance_b);
                                CALIB_LOG_VAL("  Voltage (mV): ", feedback_.supply.norm2mv((uint32_t)voltage_corrected));
                                CALIB_LOG_VAL("  Angle: ", angle_);

                                if (angle_ == 16384) {
                                    if (14 * abs((2 * ib) + ia) < ((ib * 2) - ia)) {
                                        // BLDC motor second pass: (phases equeal within ~15% from average)
                                        // if current a = sum of b and c, where ib ~= ic
                                        // Expected: iA - negative and dominant
                                        // iB - positive and equal to half of iA amplitude
                                        int32_t avg_r = (motor_.get_total_resistance() - resistance_a) / 3;
                                        motor_.set_total_resistance(avg_r);

                                        motor_selector_.set_motor_type(MotorType_BLDC);
                                        CALIB_LOG("[CALIB] Motor type detected: BLDC");
                                        CALIB_LOG_VAL("  Average resistance (mOhm): ", avg_r);
                                        set_calibration_success();
                                    }

                                    else if ((ia >> 4) < ib) {
                                        // Stepper motor second pass:
                                        // Check if iA much greater than iB
                                        // Expected: iA - negative dominant, iB close to 0 (any sign)
                                        // Resistance already calculated
                                        CALIB_LOG("[CALIB] Motor type detected: STEPPER");
                                        CALIB_LOG_VAL("  Total resistance (mOhm): ", motor_.get_total_resistance());
                                        set_calibration_success();
                                    } else {
                                        CALIB_LOG("[CALIB] ERROR: Motor type detection failed at angle 16384");
                                        set_calibration_error();
                                    }
                                }  // Check if angle is around +225° (on second path for Stepper or DC motors)
                                else if (angle_ == -24576) {
                                    // Check if b and c are balanced (~15% from average)
                                    if (14 * abs(ib - ia) < (ia + ib)) {
                                        // Stepper motor

                                        int32_t avg_r = (resistance_a + resistance_b) / 2;
                                        motor_.set_total_resistance(avg_r);
                                        CALIB_LOG("[CALIB] Motor type detected: STEPPER (balanced phases)");
                                        CALIB_LOG_VAL("  Average resistance (mOhm): ", avg_r);
                                        set_calibration_success();
                                    } else {
                                        CALIB_LOG("[CALIB] ERROR: Motor type detection failed at angle -24576");
                                        set_calibration_error();
                                    }
                                } else if (angle_ == CAL_ANGLE_45D) {
                                    timeout.set_timeout(1800);  // Set counter to cycles
                                    if (14 * abs(ib - ia) < (-ia - ib)) {
                                        // Stepper motor or BLDC first pass where phases are balanced (~15% from average)
                                        motor_.set_total_resistance(-(resistance_a + resistance_b) / 2);

                                        CALIB_LOG("[CALIB] Phases balanced - preparing rotation for motor type verification");
                                        CALIB_LOG_VAL("  Temporary resistance (mOhm): ", motor_.get_total_resistance());

                                        // Rotate angle by 30° slowly to verify
                                        motor_calib_.start_slow_rotation(16384, power);  // Target: current + 30°
                                        // set_calibration_success(); // TODO: REMOVE
                                    }  // Check if angle is around 90° (angle woks on second pass for BLDC and stepper in correct
                                       // phase)
                                    else {
                                        motor_calib_.set_voltage((int16_t)voltage_corrected);
                                        motor_calib_.start_slow_rotation(-24576, power);
                                        if (motor_selector_.get_motor_type() == MotorType_DC) {
                                            // DC motor branch - connection ABC
                                            motor_.set_total_resistance(resistance_a);
                                            CALIB_LOG("[CALIB] Motor type detected: DC (ABC connection)");
                                            CALIB_LOG_VAL("  Resistance (mOhm): ", resistance_a);
                                            set_calibration_success();
                                        } else if ((-ia >> 2) >= -ib) {
                                            // motor type - Stepper or DC
                                            // If next time we hit underpower (shouldnt happen for Stepper - then we go to DC
                                            // motor)
                                            motor_.set_total_resistance(-resistance_a);
                                            CALIB_LOG("[CALIB] Phase B dominant - setting CBA pattern and rotating");
                                            angle_ = PhasePattern_CBA;
                                            motor_selector_.set_phase_pattern(PhasePattern_CBA);

                                        } else if ((-ib >> 2) >= -ia) {
                                            CALIB_LOG("[CALIB] Phase C dominant - setting ACB pattern and rotating");
                                            // If next time we hit underpower (shouldnt happen for Stepper - then we go to DC
                                            // motor)
                                            motor_.set_total_resistance(-resistance_b);
                                            motor_selector_.set_phase_pattern(PhasePattern_ACB);
                                            angle_ = PhasePattern_ACB;
                                            // Phase C dominant: motor type - Stepper or DC
                                            // Set counter to cycles

                                        } else {
                                            CALIB_LOG("[CALIB] ERROR: Excessive deviation - unknown motor type");
                                            switch_status(DriverStatus_Error);  // motor looks so strange
                                        }
                                    }
                                } else {
                                    CALIB_LOG("[CALIB] ERROR: Excessive deviation - unknown motor type1");
                                    switch_status(DriverStatus_Error);  // motor looks so strange
                                }
                            }
                        }
                    } break;

                    // SLOW ROTATION SUB-STATE (motor type detection) ///////////////////////
                    case MotorCalibration::Status_SlowRotation: {
                        // Check power limit: if power exceeds limit by 25%, throw error
                        if (power >= (test_pwr * 5) / 2) {
                            CALIB_LOG("[CALIB] ERROR: OP duting rotation");
                            set_calibration_error();  // Power exceeded threshold
                            break;
                        }

                        // Decrement counter and check if rotation complete
                        if (timeout.tick() == 0) {                                     // Counter reached 0 - rotation done
                            angle_ = motor_calib_.get_slow_rotation_angle_target();    // Set exact target angle
                            timeout.set_timeout(900);                                  // Set settle timeout to 900 ticks
                            motor_calib_.set_status(MotorCalibration::Status_Settle);  // Transition to settle phase
                        } else {
                            // Calculate shortest path delta with int16_t overflow (automatic wrap-around)
                            int16_t delta = motor_calib_.get_slow_rotation_angle_target() - angle_;  // Shortest path via overflow
                            angle_ += delta / (int16_t)(timeout.get_timeout() + 1);  // Interpolate along shortest path
                        }
                        break;
                    }

                    // INDUCTANCE MEASUREMENT SUB-STATE (HFI-based inductance detection) ////
                    case MotorCalibration::Status_InductanceMeasure: {
                        // Log start of inductance measurement on first cycle
                        if (timeout.get_timeout() == INDUCTANCE_MEASURE_CYCLES) {  // First cycle
                            CALIB_LOG("[CALIB] Starting inductance measurement (1800 samples @ 1kHz HFI)");
                            CALIB_LOG_VAL("  HFI voltage amplitude: ", motor_calib_.get_voltage());
                            CALIB_LOG_VAL("  Angle (D-axis): ", angle_);
                        }

                        // Generate 1 kHz HFI signal for D-axis injection
                        Vec2D hfi_signal = hfi_generator_.tick();  // Get sine and cosine from generator

                        // Apply HFI voltage to D-axis (cosine component for voltage injection)
                        int16_t hfi_voltage = (motor_calib_.get_voltage() * hfi_signal.cos * 25) >> 15;  // Scale by cosine (Q15)
                        voltage_dq = Vec2D(hfi_voltage + motor_calib_.get_voltage(), 0);  // Inject HFI voltage into D-axis only

                        // Get current measurements from motor selector
                        const int32_t curnt_a = motor_selector_.get_currents().alpha;  // Phase A current
                        const int32_t curnt_b = motor_selector_.get_currents().beta;   // Phase B current

                        // Demodulate HFI response using sine component (90° phase shift for inductive response)
                        hfi_sense_a_.tick((int16_t)curnt_a, hfi_signal.cos);  // Demodulate phase A with sine reference
                        hfi_sense_b_.tick((int16_t)curnt_b, hfi_signal.cos);  // Demodulate phase B with sine reference

                        // Accumulate demodulated HFI responses for averaging
                        int16_t hfi_response_a = hfi_sense_a_.get_output();                  // Get demodulated signal A
                        int16_t hfi_response_b = hfi_sense_b_.get_output();                  // Get demodulated signal B
                        // motor_calib_.accumulate_inductance(hfi_response_a, hfi_response_b);  // Accumulate for averaging

                        // Check if measurement complete (1800 cycles = 100 periods @ 1kHz)
                        if (timeout.tick() == 0) {  // Counter reached 0 - measurement done
                            // Calculate inductance from averaged HFI response
                            // L = V / (2 * pi * f * I_hfi), where f = 1kHz
                            // Simplified: L_norm = HFI_response (proportional to inductance)
                            int32_t inductance_a = motor_calib_.get_inductance_a();  // Get averaged inductance A
                            int32_t inductance_b = motor_calib_.get_inductance_b();  // Get averaged inductance B

                            // Log inductance measurement results
                            CALIB_LOG("[CALIB] Inductance measurement complete:");
                            CALIB_LOG_VAL2("  HFI response A/B (normalized): ", inductance_a, inductance_b);

                            // Store inductance values in motor structure (in microhenries, scaled from normalized)
                            // Conversion: L_uH = (V_norm * 1000) / (2 * pi * 1000 * I_norm) = V_norm / (6.28 * I_norm)
                            // Simplified proportional storage for now (actual calibration factor TBD)
                            motor_.set_phase_inductance(0, inductance_a);  // Store phase A inductance
                            motor_.set_phase_inductance(1, inductance_b);  // Store phase B inductance

                            CALIB_LOG_VAL2("  Stored inductance A/B (uH): ", inductance_a, inductance_b);
                            CALIB_LOG("[CALIB] Motor calibration complete - transitioning to Ready");

                            // // Inductance measurement complete - transition to Ready state
                            // switch_status(DriverStatus_Ready);  // Calibration complete
                            // motor_calib_.reset_voltage();       // Reset voltage to zero
                            // voltage_ab = Vec2D(0, 0);           // Zero output voltage
                        }
                        break;
                    }

                    default:
                        // Unknown calibration state - zero output
                        voltage_ab = Vec2D(0, 0);
                        break;
                }

                // Set voltage_dq for all calibration states except inductance measurement
                // (inductance measurement sets voltage_dq internally with HFI modulation)
                if (motor_calib_.get_status() != MotorCalibration::Status_InductanceMeasure) {
                    voltage_dq = Vec2D(motor_calib_.get_voltage(), 0);  // D axis injection
                }
                break;
            }

            case DriverStatus_Ready:
                // FOC current control with calibrated current values from feedback
                // foc_current_control(Vec2D(0, target_current), angle, feedback_.current.norm(), voltage_ab);  // Use
                // feedback current
                break;

            case DriverStatus_PlayingMelody:
            case DriverStatus_Error:
                // Output motor data to serial on error

                // Restart calibration after error
                switch_status(DriverStatus_Error);  // Power cant be reached
                // start_inductance_calibration();
                voltage_ab = Vec2D(0, 0);
                break;

            case DriverStatus_Calibrating:
            default:
                voltage_ab = Vec2D(0, 0);
                break;
        }

        //         // Step 4: Calculate current errors
        // int16_t error_d = voltage_dq.flux - current_dq_filt.flux;      // d-axis (flux) error
        // int16_t error_q = voltage_dq.torque - current_dq_filt.torque;  // q-axis (torque) error

        // // Step 5: PI controllers for d and q axis (feedforward = 0, limit = INT16_MAX)
        // pid_d_.tick(error_d, 0, INT16_MAX / 2);
        // pid_q_.tick(error_q, 0, INT16_MAX / 2);

        // // Step 6: Get voltage commands from PI controllers
        // Vec2D voltage_dq(pid_d_.output(), pid_q_.output());

        // Step 7: Inverse Park transform - rotate DQ voltages back to AB
        voltage_dq_ = voltage_dq;

        // voltage_ab = voltage_dq.rotate(angle_sincos);
        voltage_ab = voltage_dq.rotate(angle_sincos);

        // converting to duty
        voltage_ab.sin = feedback_.supply.norm2duty(voltage_ab.sin);
        voltage_ab.cos = feedback_.supply.norm2duty(voltage_ab.cos);
        voltage_ab_ = voltage_ab;

        // Convert AB voltages to ABC phase voltages with phase remapping
        motor_selector_.tick_voltage(voltage_ab, feedback_.supply.get_voltage_norm());
        const int16_t* voltages = motor_selector_.get_voltages();

        output[0] = voltages[0];
        output[1] = voltages[1];
        output[2] = voltages[2];
    }

    Vec3D get_output() const { return output; }

    void set_calibration_error() {
        switch_status(DriverStatus_Error);
        motor_calib_.reset_voltage();
        voltage_ab_ = Vec2D(0, 0);  // Reset local debug var
    }

    // OPTIMIZATION: Helper for success transition - now transitions to inductance measurement
    void set_calibration_success() {
        motor_.set_motor_type(motor_selector_.get_motor_type());        // Save detected motor type
        motor_.set_phase_pattern(motor_selector_.get_phase_pattern());  // Save phase pattern

        // // Transition to inductance measurement phase
        // motor_calib_.set_status(MotorCalibration::Status_InductanceMeasure);  // Start inductance measurement
        // timeout.set_timeout(INDUCTANCE_MEASURE_CYCLES);                       // Set timeout to 1800 cycles (100 periods @ 1kHz)
        // motor_calib_.reset_inductance_accumulators();                         // Reset inductance accumulators

        // // Configure HFI generator for 1 kHz injection
        // hfi_generator_.set_frequency_target(1000);    // Set 1 kHz carrier frequency
        // hfi_generator_.set_amplitude(motor_calib_.get_voltage());  // Set amplitude to 25% of maximum
        // hfi_generator_.reset();                       // Reset generator phase to zero

        // // Reset HFI sense filters
        // hfi_sense_a_.reset();  // Reset phase A HFI demodulator
        // hfi_sense_b_.reset();  // Reset phase B HFI demodulator

        // angle_ = 0;  // Set angle to 0° for D-axis injection
    }

    const int16_t* get_voltages() const { return motor_selector_.get_voltages(); }

    const int16_t* get_control() const { return motor_selector_.get_voltages(); }

    // Set maximum duty cycle limit
    void set_duty(uint16_t duty_permille) { motor_selector_.set_duty(duty_permille); }

    // Set current balance tolerance
    void set_current_epsilon(int16_t epsilon) { motor_selector_.set_current_epsilon(epsilon); }

    // Get last voltage command (for debugging)
    Vec2D get_voltage_ab() const { return voltage_ab_; }

    // Get current AB values (for debugging)
    Vec2D get_current_ab() const { return motor_selector_.get_currents(); }

    // Get normalized current values (for debugging) from feedback
    const int16_t* get_current_normalized() const {
        return feedback_.current.norm();  // Return normalized currents from feedback
    }

    // Check if current sensors are calibrated in feedback
    bool is_current_calibrated() const {
        return feedback_.current.is_calibrated();  // Check calibration status from feedback
    }

    // Get current driver status from motor structure
    DriverStatus get_status() const { return motor_.get_status(); }

    // Manually start current calibration in feedback
    void start_current_calibration() {
        switch_status(DriverStatus_CalibrateCurrent);  // Update motor status
        timeout.set_timeout(CALIBRATION_ZERO_CYCLES);
        feedback_.current.calibrate();  // Start calibration through friend access
    }

    // Configure number of calibration samples in feedback
    void set_current_calibration_samples(int32_t n) {
        feedback_.current.set_calibration_samples(n);  // Configure samples through friend access
    }

    /// Get current rotor angle (Q15 format, -32768 to 32767 = -π to π)
    int16_t get_angle() const { return angle_; }

private:
    /// Switch driver status and save previous state (inline for performance)
    /// @param new_status New driver status to transition to
    inline void switch_status(DriverStatus new_status) {
        motor_.set_status(new_status);  // Set new status
    }
};
