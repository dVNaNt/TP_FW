#pragma once

// 3-phase PWM driver with FOC (Field Oriented Control) current regulation

#include "../../common.h"
#include "../../feedback/analog/current_sense.h"
#include "../../feedback/feedback.h"  // Include unified feedback module
#include "../../math_integer/controllers/pid.h"
#include "../../math_integer/filters/filter1st.h"
#include "../../math_integer/helpers/stability_check.h"  // Stability check for current monitoring
#include "../../math_integer/normalization.h"
#include "../../math_integer/trigonometry.h"
#include "../motor.h"  // Motor parameters structure
#include "../types.h"
// #include "motor_electric_test.h"  // COMMENTED: Replaced by calibration.h
#include "hfi.h"  // High Frequency Injection for angle estimation
#include "sel_motor.h"
#include "speaker.h"  // Motor speaker for audio feedback
// #include "hfi_calibration.h"  // COMMENTED: Replaced by calibration.h
#include "calibration.h"    // Phase inductance calibration using HFI

// Forward declaration
struct Feedback;

struct DriverPWM {
public:
    // Motor configuration reference (owned by TunePulseHeart)
    Motor& motor_;

private:
    // Reference to parent Feedback
    Feedback& feedback_;

    MotorSelector motor_selector_;  // Unified motor selector (voltage + current + phase)
    ControlMode control_mode_;

    // PI controllers for d and q axis current control
    PID_Int pid_d_;
    PID_Int pid_q_;

    // LPF filters for d and q axis current measurements
    Filter1st lpf_id_;
    Filter1st lpf_iq_;

    uint32_t calibration_counter_;  // Counter for calibration cycles

    // Used to charge mosfet driver before going high on all channels
    static constexpr uint32_t CALIBRATION_ZERO_CYCLES = 50;  // Cycles with zero PWM

    // COMMENTED: Motor parameter identification - replaced by Calibrator
    // DCPhaseCheck motor_test_;

    // Motor speaker for audio feedback after calibration
    MotorSpeaker speaker_;

    // HFI (High Frequency Injection) for sensorless angle estimation
    HFI hfi_;  // HFI signal generator and analyzer
    // COMMENTED: HFI calibration - replaced by HFI_Inductance
    // HFICalibration hfi_cal_;         // HFI amplitude tuning and inductance identification

    // Phase inductance measurement using HFI
    HFI_Inductance hfi_inductance_;   // HFI inductance measurement module
    ResistanceMeasure res_measure_;   // Resistance measurement module
    uint8_t calib_phase_idx_;         // Current phase being calibrated (0-2)
    uint32_t calib_pause_cnt_;        // Pause counter between phases
    bool calib_measuring_r_;          // true = measuring R, false = measuring L
    static constexpr uint32_t CALIB_PAUSE_CYCLES = 1800;  // 100ms pause @ 18kHz

    // Motor connection calibration ramp variables
    int16_t conn_calib_voltage_;                        // Current output voltage (Q15 duty)
    int16_t conn_calib_increment_;                      // Voltage increment per tick (Q15)
    int16_t conn_calib_voltage_limit_;                  // Maximum voltage limit (Q15)
    int16_t conn_calib_overcurrent_threshold_;          // Overcurrent detection threshold (Q15)
    uint32_t conn_calib_timeout_;                       // Timeout counter (cycles)
    uint32_t conn_calib_timeout_limit_;                 // Timeout limit (cycles)
    bool conn_calib_timeout_flag_;                      // Timeout occurred flag

    int16_t hfi_angle_estimate_;                        // Estimated rotor angle from HFI (output)
    uint32_t hfi_warmup_counter_;                       // Counter for 1 second warmup with DC current
    StabilityCheck stability_check_;                    // Stability checker for current monitoring
    static constexpr uint32_t HFI_WARMUP_CYCLES = 1;    // 1 second at 18kHz PWM
    static constexpr uint16_t HFI_FREQUENCY = 1000;     // HFI injection frequency in Hz
    static constexpr uint16_t HFI_PERIODS = 10;         // Number of periods per measurement
    static constexpr int32_t HFI_AMPLITUDE_MV = 15000;  // HFI voltage amplitude in mV (5V)

    Vec3D output;
    int32_t power;

    // Debug/monitoring
    Vec2D voltage_ab_;  // Last voltage command


public:
    // Constructor: takes Motor reference from parent (TunePulseHeart owns Motor)
    DriverPWM(Feedback& parent, Motor& motor_ref)
        : motor_(motor_ref),
        power(0),
          feedback_(parent),
          motor_selector_(parent, MotorType_STEP, PhasePattern_CBA, 995u, 256),
          control_mode_(ControlMode_VoltageAB),
          pid_d_(0, 0, 0, 0)  // Default PI gains: kp=10%, ki=5%, kd=0, kff=0
          ,
          pid_q_(0, 0, 0, 0),
          lpf_id_(0, 220)  // LPF for id with alpha=64 (25% filtering)
          ,
          lpf_iq_(0, 220)  // LPF for iq with alpha=64 (25% filtering)
          ,
          calibration_counter_(0)
          // , motor_test_(parent, motor_, motor_selector_)  // COMMENTED: Replaced by hfi_inductance_
          ,
          speaker_(18000, 3276)  // Speaker: 20kHz system, ~10% amplitude
          ,
          hfi_(18000)  // HFI: 18kHz system frequency
          // , hfi_cal_(parent, motor_, motor_selector_, hfi_)  // COMMENTED: Replaced by hfi_inductance_
          ,
          hfi_inductance_(parent, 18000)  // HFI inductance: Feedback ref + 18kHz
          ,
          res_measure_(parent)  // Resistance measurement: Feedback ref
          ,
          calib_phase_idx_(0)  // Start with phase 0
          ,
          calib_pause_cnt_(0)  // Reset pause counter
          ,
          calib_measuring_r_(false)  // Start with inductance measurement
          ,
          conn_calib_voltage_(0)  // Initialize ramp voltage
          ,
          conn_calib_increment_(2)  // Default increment (user will set)
          ,
          conn_calib_voltage_limit_(INT16_MAX / 2)  // Default limit (user will set)
          ,
          conn_calib_overcurrent_threshold_(INT16_MAX / 2)  // Default threshold (user will set)
          ,
          conn_calib_timeout_(0)  // Initialize timeout counter
          ,
          conn_calib_timeout_limit_(18000)  // Default 1 second @ 18kHz (user will set)
          ,
          conn_calib_timeout_flag_(false)  // No timeout initially
          ,
          hfi_angle_estimate_(0)  // Initialize angle estimate to zero
          ,
          hfi_warmup_counter_(0)  // Initialize warmup counter
          ,
          stability_check_(50, 180)  // Initialize stability checker: epsilon=50, window=180 cycles
          ,
          voltage_ab_{0, 0},
          output{INT16_MIN, INT16_MIN, INT16_MIN} {
        // Set calibration samples to 100 for feedback current sensor
        parent.current.set_calibration_samples(100);  // Configure calibration samples
    }



    // Convenience overload: takes raw id/iq and encoder angle with Feedback reference
    void tick_control_foc(int16_t angle, int16_t target_current, const Feedback& feedback) {
        // Ensuring that we have full internal control over angle until driver is ready
        if (motor_.get_status() != DriverStatus_Ready) {
            angle = 0;
        }
        // Get current ADC values from feedback data
        const int16_t* currents_abc = feedback.current.norm();  // Access current ADC array from feedback
        int16_t supply_voltage_norm = feedback.supply.get_voltage_norm();

        // Step 1: Convert encoder angle to rotation vector (sin/cos)
        Pol2D angle_polar = Pol2D(angle, INT16_MAX);
        Vec2D angle_sincos = angle_polar.convert();

        // Step 2: Get calibrated normalized currents from feedback and convert to AB using motor selector
        const int16_t* normalized_currents = feedback_.current.norm();  // Use feedback current sensor
        motor_selector_.tick_current(normalized_currents);
        Vec2D current_ab = motor_selector_.get_currents();

        // Step 3: Park transform - rotate AB currents by -encoder angle to get DQ
        // Inverse rotation: negate sin component for reverse rotation
        Vec2D angle_sincos_inv(-angle_sincos.sin, angle_sincos.cos);
        Vec2D current_dq_raw = current_ab.rotate(angle_sincos_inv);

        // Step 3.5: Apply LPF filtering to measured DQ currents
        int16_t filtered_id = (int16_t)lpf_id_.tick_lpf((uint16_t)(current_dq_raw.flux));
        int16_t filtered_iq = (int16_t)lpf_iq_.tick_lpf((uint16_t)(current_dq_raw.torque));
        Vec2D current_dq_filt = Vec2D(filtered_id, filtered_iq);

        Vec2D target_dq(0, 0); 

        Vec2D voltage_ab(0, 0);

        

        switch (motor_.get_status()) {
            ///////////////////////////////////////////////////////////////////////////////
            // CURRENT SENSOR OFFSET CALIBRATION SEQUENCE /////////////////////////////////
            ///////////////////////////////////////////////////////////////////////////////
            case DriverStatus_CalibrateCurrent:
                // During calibration: first 50 cycles with zero PWM to ensure charge of bootstrap circuit
                if (calibration_counter_ < CALIBRATION_ZERO_CYCLES) {
                    // Phase 1: Zero PWM for 50 cycles
                    output.ch1 = 0;
                    output.ch2 = 0;
                    output.ch3 = 0;
                    calibration_counter_++;
                } // When charging complete - making all outputs high to ensure 0 current flow through low side sense
                else if (calibration_counter_ == CALIBRATION_ZERO_CYCLES) {
                    // Start calibration for all channels using feedback current sensor
                    // Set calibration samples to 100 for feedback current sensor
                    feedback_.current.calibrate();  // Start calibration through friend access
                    calibration_counter_++;
                    // Phase 2: Max PWM during current sensor sampling
                    output.ch1 = INT16_MAX;
                    output.ch2 = INT16_MAX;
                    output.ch3 = INT16_MAX;
                    return;
                }  // If calibration is not complete skip this turn
                else if (!feedback_.current.is_calibrated()) {
                    return;
                } // If calibration complete switching to next step
                else {
                    // Calibration complete, transition to Inductance Calibration
                    motor_.set_status(DriverStatus_MotorConnectionCalibration);
                    

                    // DC motor type to handle current control
                    motor_selector_.set_motor_type(MotorType_Calibration);
                    motor_selector_.set_phase_pattern(PhasePattern_CBA);
                    // Start inductance calibration using HFI
                    // start_inductance_calibration();


                    stability_check_.setup(50, 100);

                    feedback_.power.set_overpower_soft_mw(2000);  // Set soft power limit to 5W
                    feedback_.supply.set_overvoltage_soft_norm(feedback_.supply.get_voltage_norm()/2);
                    feedback_.current.set_overcurrent_soft_norm(INT16_MAX/2);

                    voltage_ab = Vec2D(0, 0);
                }
                break;

            ///////////////////////////////////////////////////////////////////////////////
            // Motor calibration /////////////////////////////////
            ///////////////////////////////////////////////////////////////////////////////
            case DriverStatus_MotorConnectionCalibration: {
                // Get measured current (use D-axis current from DQ transform)
                const int16_t curnt_a_abs = abs(motor_selector_.get_filtered_a());
                const int16_t curnt_b_abs = abs(motor_selector_.get_filtered_b());    
                const int16_t curnt_c_abs = abs(motor_selector_.get_filtered_c());   
                int32_t measured_current = (curnt_a_abs + curnt_b_abs + curnt_c_abs) / 2; // Ensure we have valid current independent from anything
                angle = 0;
                // Check for overpower using soft limit
                bool power_lim = feedback_.power.is_overpower_soft_norm(conn_calib_voltage_, measured_current);
                bool voltg_lim = feedback_.supply.is_overvoltage_soft_norm(conn_calib_voltage_);
                bool curnt_lim = feedback_.current.is_overcurrent_soft_norm(measured_current);

                power = feedback_.power.norm2mw(measured_current * conn_calib_voltage_);

                // Increment timeout counter
                conn_calib_timeout_++;                                   // Count cycles
                
                // Check timeout
                if (conn_calib_timeout_ >= conn_calib_timeout_limit_) {  // Timeout reached
                    conn_calib_timeout_flag_ = true;                     // Set timeout flag
                    conn_calib_voltage_ = 0;                             // Reset voltage
                    voltage_ab = Vec2D(0, 0);                            // Zero output
                    // User handles timeout externally
                    break;
                }

                // Check for overcurrent - reduce voltage by increment
                if (power_lim or curnt_lim or voltg_lim) {   // Overpower detected
                    conn_calib_voltage_ -= conn_calib_increment_;        // Reduce by increment
                    if (conn_calib_voltage_ < 0) {                       // Clamp to zero
                        conn_calib_voltage_ = 0;
                    }
                } else {
                    // Normal ramp - increase voltage by increment
                    conn_calib_voltage_ += conn_calib_increment_;        // Increase by increment
                    if (conn_calib_voltage_ > conn_calib_voltage_limit_) { // Clamp to limit
                        conn_calib_voltage_ = conn_calib_voltage_limit_;
                    }
                }

                // If current stabilized
                if (stability_check_.tick(measured_current)) {
                    SerialUSB.println("STABLE");
                    stability_check_.setup(50, 100);
                }

                // Output voltage on D-axis
                target_dq = Vec2D(0, conn_calib_voltage_);              // D-axis voltage only
                
                // motor_selector_.tick_voltage(voltage_ab);
                // const int16_t* voltages = motor_selector_.get_voltages();
                // output[0] = voltages[0];
                // output[1] = voltages[1];
                // output[2] = voltages[2];
                // return;
            }

            case DriverStatus_Ready:
                // FOC current control with calibrated current values from feedback
                // foc_current_control(Vec2D(0, target_current), angle, feedback_.current.norm(), voltage_ab);  // Use feedback
                // current
                break;

            case DriverStatus_PlayingMelody: 
            case DriverStatus_HFI_Warmup: 
            case DriverStatus_HFI_AmplitudeTuning:
            case DriverStatus_HFI_LdMeasure:
            case DriverStatus_HFI_LqMeasure:
            case DriverStatus_HFI_Running:
            case DriverStatus_Error:
                // Output motor data to serial on error
                motor_.print(SerialUSB);

                // Restart calibration after error
                motor_.set_status(DriverStatus_MotorConnectionCalibration);
                start_inductance_calibration();
                voltage_ab = Vec2D(0, 0);
                break;

            case DriverStatus_Calibrating:
            default:
                voltage_ab = Vec2D(0, 0);
                break;
        }

        //         // Step 4: Calculate current errors
        // int16_t error_d = target_dq.flux - current_dq_filt.flux;      // d-axis (flux) error
        // int16_t error_q = target_dq.torque - current_dq_filt.torque;  // q-axis (torque) error

        // // Step 5: PI controllers for d and q axis (feedforward = 0, limit = INT16_MAX)
        // pid_d_.tick(error_d, 0, INT16_MAX / 2);
        // pid_q_.tick(error_q, 0, INT16_MAX / 2);

        // // Step 6: Get voltage commands from PI controllers
        // Vec2D voltage_dq(pid_d_.output(), pid_q_.output());

        // Step 7: Inverse Park transform - rotate DQ voltages back to AB

        target_dq.sin = feedback_.supply.norm2duty(target_dq.sin);
        target_dq.cos = feedback_.supply.norm2duty(target_dq.cos);

        // voltage_ab = voltage_dq.rotate(angle_sincos);
        voltage_ab = target_dq.rotate(angle_sincos);
        voltage_ab_ = voltage_ab;

        // Convert AB voltages to ABC phase voltages with phase remapping
        motor_selector_.tick_voltage(voltage_ab);
        const int16_t* voltages = motor_selector_.get_voltages();

        output[0] = voltages[0];
        output[1] = voltages[1];
        output[2] = voltages[2];
    }

    Vec3D get_output() const { return output; }

    bool change_motor_mode(MotorType motor_type_param) {
        motor_selector_.set_motor_type((uint32_t)motor_type_param);
        motor_.set_motor_type(motor_type_param);  // Update motor structure
        return true;
    }

    bool change_phase_mode(PhasePattern connection) {
        motor_selector_.set_phase_pattern(connection);
        motor_.set_phase_pattern(connection);  // Update motor structure
        return true;
    }

    bool change_control_mode(ControlMode mode) {
        control_mode_ = mode;
        return true;
    }

    const int16_t* get_voltages() const { return motor_selector_.get_voltages(); }

    const int16_t* get_control() const { return motor_selector_.get_voltages(); }

    // Configure PI controller gains for current control
    void set_current_pi_gains(int32_t kp, int32_t ki) {
        pid_d_.setKp(kp);  // Update proportional gain
        pid_d_.setKi(ki);  // Update integral gain
        pid_d_.reset();    // Reset controller state
        pid_q_.setKp(kp);  // Update proportional gain
        pid_q_.setKi(ki);  // Update integral gain
        pid_q_.reset();    // Reset controller state
    }

    // Set maximum duty cycle limit
    void set_duty(uint16_t duty_permille) { motor_selector_.set_duty(duty_permille); }

    // Set current balance tolerance
    void set_current_epsilon(int16_t epsilon) { motor_selector_.set_current_epsilon(epsilon); }

    // Configure LPF filter alpha for current measurements (0-255, where 0=infinite filtering, 255=no filtering)
    void set_current_lpf_alpha(uint8_t alpha) {
        lpf_id_.set_alpha(alpha);
        lpf_iq_.set_alpha(alpha);
    }

    // Get current DQ values for debugging/monitoring (returns Vec2D)
    Vec2D get_current_dq(const int16_t currents_abc[3], const Pol2D& encoder_pos) {
        // Convert encoder angle to rotation vector
        Vec2D encoder_rotation = encoder_pos.convert();

        // Convert ABC currents to AB using motor selector
        motor_selector_.tick_current(currents_abc);
        Vec2D current_ab = motor_selector_.get_currents();

        // Park transform to get DQ (inverse rotation)
        Vec2D encoder_rotation_inv(-encoder_rotation.sin, encoder_rotation.cos);
        return current_ab.rotate(encoder_rotation_inv);
    }

    // Convenience overload: takes raw encoder angle and outputs to pointers
    void get_current_dq(const int16_t currents_abc[3], int16_t encoder_angle, int16_t* out_id, int16_t* out_iq) {
        Pol2D encoder_pos(encoder_angle, INT16_MAX);
        Vec2D current_dq = get_current_dq(currents_abc, encoder_pos);
        *out_id = current_dq.flux;    // D-axis (flux) current
        *out_iq = current_dq.torque;  // Q-axis (torque) current
    }

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
        motor_.set_status(DriverStatus_CalibrateCurrent);  // Update motor status
        calibration_counter_ = 0;
        feedback_.current.calibrate();  // Start calibration through friend access
    }

    // Configure number of calibration samples in feedback
    void set_current_calibration_samples(int32_t n) {
        feedback_.current.set_calibration_samples(n);  // Configure samples through friend access
    }

    // Get HFI estimated rotor angle (valid when DriverStatus_HFI_Running)
    // Returns angle in normalized format (INT16_MIN to INT16_MAX = -π to π)
    int16_t get_hfi_angle() const {
        return hfi_angle_estimate_;  // Return latest HFI angle estimate
    }

    // Get HFI D-axis amplitude (for debugging/tuning)
    int16_t get_hfi_amplitude_d() const {
        return hfi_.get_amplitude_d();  // Return D-axis current amplitude at HFI frequency
    }

    // Get HFI Q-axis amplitude (for debugging/tuning)
    int16_t get_hfi_amplitude_q() const {
        return hfi_.get_amplitude_q();  // Return Q-axis current amplitude at HFI frequency
    }

    // Check if HFI is currently running
    bool is_hfi_running() const {
        return hfi_.is_running();  // Return HFI running status
    }


    // === HFI Inductance measurement getters ===

    // Get HFI inductance measurement state
    HFI_L_State get_hfi_inductance_state() const {
        return hfi_inductance_.get_state();  // Return measurement state
    }

    // Check if inductance calibration is running
    bool is_calibrating() const {
        return hfi_inductance_.is_running();  // Return measurement running status
    }

    // Get measured inductance for current phase
    int32_t get_current_inductance_uh() const {
        return hfi_inductance_.get_inductance_uh();  // Return inductance in uH
    }

    // Get current HFI amplitude during calibration in millivolts
    int32_t get_hfi_inductance_amplitude_mv() const {
        return hfi_inductance_.get_amplitude_mv();  // Return HFI amplitude in mV
    }

    // Get current phase being calibrated
    uint8_t get_calibration_phase() const {
        return calib_phase_idx_;  // Return phase index
    }

    // =========================================================================
    // CURRENT STABILITY CHECK
    // =========================================================================
    
    /// Check if current is stable using StabilityCheck class
    /// @param current Measured current (Q15)
    /// @return true if current stable for configured window cycles
    bool is_current_stable(int16_t current) {
        return stability_check_.tick(current);                           // Update and check stability
    }

    /// Configure stability check parameters
    /// @param epsilon Max allowed peak-to-peak noise (Q15)
    /// @param window Required consecutive stable cycles
    void setup_stability_check(int16_t epsilon, uint16_t window) {
        stability_check_.setup(epsilon, window);                         // Configure parameters
    }

    /// Reset stability checker state
    void reset_stability_check() { 
        stability_check_.reset();                                        // Reset internal state
    }

    /// Check if current is currently stable (read-only)
    bool is_stable() const { 
        return stability_check_.is_stable();                             // Check stability status
    }

    /// Get auto-detected reference value from stability checker
    /// @return Center of noise floor (valid when is_stable() returns true)
    int16_t get_detected_reference() const {
        return stability_check_.get_detected_reference();                // Get detected reference
    }

    // =========================================================================
    // MOTOR CONNECTION CALIBRATION RAMP CONFIGURATION
    // =========================================================================
    
    /// Setup connection calibration ramp parameters
    /// @param increment Voltage increment per tick (Q15)
    /// @param voltage_limit Maximum voltage limit (Q15)
    /// @param overcurrent_threshold Overcurrent detection threshold (Q15)
    /// @param timeout_cycles Timeout limit in cycles
    void setup_conn_calib(int16_t increment, int16_t voltage_limit, 
                          int16_t overcurrent_threshold, uint32_t timeout_cycles) {
        conn_calib_increment_ = increment;                               // Set increment
        conn_calib_voltage_limit_ = voltage_limit;                       // Set voltage limit
        conn_calib_overcurrent_threshold_ = overcurrent_threshold;       // Set overcurrent threshold
        conn_calib_timeout_limit_ = timeout_cycles;                      // Set timeout limit
        reset_conn_calib();                                              // Reset state
    }

    /// Reset connection calibration state
    void reset_conn_calib() {
        conn_calib_voltage_ = 0;                                         // Reset voltage
        conn_calib_timeout_ = 0;                                         // Reset timeout counter
        conn_calib_timeout_flag_ = false;                                // Clear timeout flag
    }

    /// Check if connection calibration timed out
    bool is_conn_calib_timeout() const { return conn_calib_timeout_flag_; }

    /// Get current connection calibration voltage (Q15)
    int16_t get_conn_calib_voltage() const { return conn_calib_voltage_; }

    /// Get connection calibration timeout counter
    uint32_t get_conn_calib_timeout_count() const { return conn_calib_timeout_; }

private:
    /// Start inductance calibration sequence
    void start_inductance_calibration() {
        calib_phase_idx_ = 0;           // Start with phase 0
        calib_pause_cnt_ = 0;           // No initial pause
        calib_measuring_r_ = false;     // Start with inductance
        start_phase_measurement(0);     // Start first phase
        SerialUSB.println("[CAL] === START L+R CALIBRATION ===");
    }

    /// Start resistance measurement for current phase
    void start_resistance_measurement() {
        // Get measured inductance for PI tuning
        int32_t inductance_uh = motor_.get_phase_inductance(calib_phase_idx_);
        
        // Calculate target current: 3/4 of motor limit
        int32_t max_ma = motor_.get_current_limit();
        int32_t target_ma = (max_ma * 3) >> 2;                           // 3/4 of max
        int16_t target_norm = feedback_.current.ma2norm(target_ma);
        
        // Start resistance measurement with PI tuned from inductance
        res_measure_.start(target_norm, inductance_uh);
        
        SerialUSB.print("[CAL] Ph");
        SerialUSB.print(calib_phase_idx_);
        SerialUSB.print(" R: tgt=");
        SerialUSB.print(target_ma);
        SerialUSB.print("mA L=");
        SerialUSB.print(inductance_uh);
        SerialUSB.println("uH");
    }

    /// Start measurement for specific phase
    /// @param phase Phase index (0-2)
    void start_phase_measurement(uint8_t phase) {
        // Configure motor selector for DC mode with current phase pattern
        motor_selector_.set_motor_type(MotorType_DC);                    // Set DC mode
        motor_selector_.set_phase_pattern(static_cast<PhasePattern>(phase));  // Set phase

        // Calculate target current: 1/4 of motor current limit
        int32_t max_current_ma = motor_.get_current_limit();             // Get motor limit (mA)
        int32_t target_ma = max_current_ma >> 2;                         // 1/4 of max current

        // Calculate max amplitude: 1/2 of supply voltage in mV
        int32_t voltage_max_mv = feedback_.supply.get_voltage_mv() >> 1; // Half of Vbus

        // Start HFI inductance measurement (uses Feedback internally)
        hfi_inductance_.start(target_ma, voltage_max_mv);                // Start measurement

        SerialUSB.print("[CAL] Phase ");                                 // Log phase start
        SerialUSB.print(phase);                                          // Log phase index
        SerialUSB.print(" target=");                                     // Log target label
        SerialUSB.print(target_ma);                                      // Log target value
        SerialUSB.print("mA, V_max=");                                   // Log voltage label
        SerialUSB.print(voltage_max_mv);                                 // Log voltage value
        SerialUSB.println("mV");                                         // Log units
    }
};
