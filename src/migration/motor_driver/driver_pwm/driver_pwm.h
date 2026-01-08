#pragma once

// 3-phase PWM driver with FOC (Field Oriented Control) current regulation

#include "../../feedback/feedback.h"  // Include unified feedback module
#include "../../feedback/analog/current_sense.h"
#include "../../common.h"
#include "../../math_integer/controllers/pid.h"
#include "../../math_integer/filters/lpf.h"
#include "../../math_integer/normalization.h"
#include "../../math_integer/trigonometry.h"
#include "../types.h"
#include "motor_electric_test.h"
#include "sel_motor.h"
#include "speaker.h"  // Motor speaker for audio feedback
#include "hfi.h"      // High Frequency Injection for angle estimation

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
    FilterLPF lpf_id_;
    FilterLPF lpf_iq_;
    
    uint32_t calibration_counter_;  // Counter for calibration cycles

    // Used to charge mosfet driver before going high on all channels
    static constexpr uint32_t CALIBRATION_ZERO_CYCLES = 50;  // Cycles with zero PWM
    
    // Motor parameter identification
    DCPhaseCheck motor_test_;
    
    // Motor speaker for audio feedback after calibration
    MotorSpeaker speaker_;
    
    // HFI (High Frequency Injection) for sensorless angle estimation
    HFI hfi_;                        // HFI signal generator and analyzer
    int16_t hfi_angle_estimate_;     // Estimated rotor angle from HFI (output)
    uint32_t hfi_warmup_counter_;    // Counter for 1 second warmup with DC current
    static constexpr uint32_t HFI_WARMUP_CYCLES = 1;  // 1 second at 18kHz PWM
    static constexpr uint16_t HFI_FREQUENCY = 1000;       // HFI injection frequency in Hz
    static constexpr uint16_t HFI_PERIODS = 10;           // Number of periods per measurement
    static constexpr int32_t HFI_AMPLITUDE_MV = 15000;     // HFI voltage amplitude in mV (5V)
    
    // Debug/monitoring
    Vec2D voltage_ab_;  // Last voltage command


    // FOC current control: converts target id/iq and measured currents to voltage commands
    void foc_current_control(const Vec2D& target_dq, int16_t angle, 
                            const int16_t currents_abc[3], Vec2D& out_voltage_ab) {
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
        Vec2D current_dq = current_ab.rotate(angle_sincos_inv);
        
        // Step 3.5: Apply LPF filtering to measured DQ currents
        int16_t filtered_id = (int16_t)lpf_id_.tick((uint16_t)(current_dq.sin));
        int16_t filtered_iq = (int16_t)lpf_iq_.tick((uint16_t)(current_dq.cos));
        current_dq = Vec2D(filtered_id, filtered_iq);
        
        // Step 4: Calculate current errors
        int16_t error_d = target_dq.sin - current_dq.sin;  // d-axis error
        int16_t error_q = target_dq.cos - current_dq.cos;  // q-axis error
        
        // Step 5: PI controllers for d and q axis (feedforward = 0, limit = INT16_MAX)
        pid_int_tick(&pid_d_, error_d, 0, INT16_MAX/2);
        pid_int_tick(&pid_q_, error_q, 0, INT16_MAX/2);
        
        // Step 6: Get voltage commands from PI controllers
        Vec2D voltage_dq(pid_int_output(&pid_d_), pid_int_output(&pid_q_));
        
        // Step 7: Inverse Park transform - rotate DQ voltages back to AB
        out_voltage_ab = voltage_dq.rotate(angle_sincos);
        
        // Store for debugging
        voltage_ab_ = out_voltage_ab;
    }

public:
    // Constructor: takes Motor reference from parent (TunePulseHeart owns Motor)
    DriverPWM(Feedback& parent, Motor& motor_ref) 
        : motor_(motor_ref)
        , feedback_(parent)
        , motor_selector_(parent, MotorType_STEP, ABC, 995u, 256)
        , control_mode_(ControlMode_VoltageAB)
        , pid_d_(pid_int_new(10, 0, 0, 0))  // Default PI gains: kp=10%, ki=5%, kd=0, kff=0
        , pid_q_(pid_int_new(10, 0, 0, 0))
        , lpf_id_(0, 64)  // LPF for id with alpha=64 (25% filtering)
        , lpf_iq_(0, 64)  // LPF for iq with alpha=64 (25% filtering)
        , calibration_counter_(0)
        , motor_test_(parent, motor_, motor_selector_)  // Pass references to motor_test
        , speaker_(18000, 3276)  // Speaker: 20kHz system, ~10% amplitude
        , hfi_(18000)            // HFI: 18kHz system frequency
        , hfi_angle_estimate_(0) // Initialize angle estimate to zero
        , hfi_warmup_counter_(0) // Initialize warmup counter
        , voltage_ab_{0, 0}
    {
        // Set calibration samples to 100 for feedback current sensor
        parent.current.set_calibration_samples(100);  // Configure calibration samples
        parent.current.calibrate();  // Start calibration for all channels
    }

    // Constructor with control mode and PI gains
    DriverPWM(Feedback& parent, Motor& motor_ref, ControlMode control_mode_param, 
              int32_t kp = 1000, int32_t ki = 500) 
        : motor_(motor_ref)
        , feedback_(parent)
        , motor_selector_(parent,
                         (uint32_t)motor_ref.get_motor_type(), 
                         motor_ref.get_phase_pattern(),
                         995u, 256)
        , control_mode_(control_mode_param)
        , pid_d_(pid_int_new(kp, ki, 0, 0))  // PI controller with configurable gains
        , pid_q_(pid_int_new(kp, ki, 0, 0))
        , lpf_id_(0, 128)  // LPF for id with alpha=64 (25% filtering)
        , lpf_iq_(0, 128)  // LPF for iq with alpha=64 (25% filtering)
        , calibration_counter_(0)
        , motor_test_(parent, motor_, motor_selector_)  // Pass references to motor_test
        , speaker_(20000, 3276)  // Speaker: 20kHz system, ~10% amplitude
        , hfi_(18000)            // HFI: 18kHz system frequency
        , hfi_angle_estimate_(0) // Initialize angle estimate to zero
        , hfi_warmup_counter_(0) // Initialize warmup counter
        , voltage_ab_{0, 0}
    {
        // Set calibration samples to 100 for feedback current sensor
        parent.current.set_calibration_samples(100);  // Configure calibration samples
    }

    // Convenience overload: takes raw id/iq and encoder angle with Feedback reference
    void tick_control_foc(int16_t angle, int16_t target_current, 
                         const Feedback& feedback,
                         int16_t out[3]) {
       
        // Get current ADC values from feedback data
        const int16_t* currents_abc = feedback.current.norm();  // Access current ADC array from feedback
        int16_t supply_voltage_norm = feedback.supply.get_voltage_norm();
        
        Vec2D voltage_ab(0, 0);
        
        switch (motor_.get_status()) {
            case DriverStatus_CalibrateCurrent:
                // During calibration: first 50 cycles with zero PWM, then max PWM for sampling
                if (calibration_counter_ < CALIBRATION_ZERO_CYCLES) {
                    // Phase 1: Zero PWM for 50 cycles
                    out[0] = 0;
                    out[1] = 0;
                    out[2] = 0;
                    calibration_counter_++;
                    return;
                } else if (calibration_counter_ == CALIBRATION_ZERO_CYCLES) {       
                    // Start calibration for all channels using feedback current sensor
                    feedback_.current.calibrate();  // Start calibration through friend access
                    calibration_counter_++;
                } else if (!feedback_.current.is_calibrated()) {  // Check calibration status
                    // Phase 2: Max PWM during current sensor sampling
                    out[0] = INT16_MAX;
                    out[1] = INT16_MAX;
                    out[2] = INT16_MAX;
                    return;
                } else {
                    // Calibration complete, transition to Motor Test
                    motor_.set_status(DriverStatus_MotorConnectionCalibration);
                    // Start motor test with repeat mode enabled (uses motor_.get_current_limit())
                    motor_test_.start(true);
                    voltage_ab = Vec2D(0, 0);
                }
                break;
                
            case DriverStatus_MotorConnectionCalibration: {
                // Motor parameter identification using DC mode
                int16_t test_pwm[3];
                
                // Check if test cycle just completed
                static bool was_cycle_complete = false;
                bool is_cycle_complete_now = motor_test_.is_complete();
                
                
                was_cycle_complete = is_cycle_complete_now;
                
                // Tick motor test with feedback current sensor normalized values
                const int16_t* norm_currents = feedback_.current.norm();  // Get normalized currents from feedback
                motor_test_.tick();  // Pass normalized currents to motor test
                
                // Check if error occurred during test
                if (motor_test_.is_error()) {
                    // Output motor data to serial on error
                    motor_.print(SerialUSB);
                    
                    // Restart motor test after error
                    motor_.set_status(DriverStatus_MotorConnectionCalibration);
                    motor_test_.start(true);
                    
                    // Set outputs to zero
                    out[0] = 0;
                    out[1] = 0;
                    out[2] = 0;
                    return;
                }
                
                // Check if test just completed (first time only, not in repeat mode)
                static bool was_complete = false;
                bool is_complete_now = motor_test_.is_complete();
                
                if (is_complete_now && !was_complete) {
                    // Test just completed, determine motor type and phase pattern
                    uint32_t detected_motor_type = motor_.get_motor_type();
                    PhasePattern detected_pattern = motor_.get_phase_pattern();
                    
                    motor_selector_.set_motor_type(motor_.get_motor_type());
                    motor_selector_.set_phase_pattern(motor_.get_phase_pattern());  // Default ABC for BLDC
                    
                    // Output motor data to serial after calibration cycle
                    motor_.print(SerialUSB);
                    
                    // === Calculate Ki from motor resistance ===
                    // Ki = 1 / (R * tau) where tau is the current loop time constant
                    // For fast response: Ki ≈ R / L, but we use simplified formula
                    // Ki = 10000 / R_mohm (empirical formula for good response)
                    // This gives Ki in percentage units (0-10000%)
                    int32_t resistance_mohm = motor_.get_total_resistance();
                    int32_t ki_calculated = 0;
                    if (resistance_mohm > 0 && resistance_mohm < 100000) {
                        // Ki = 500000 / R_mohm, clamped to valid range
                        // Higher resistance = lower Ki for stability
                        ki_calculated = 500000 / resistance_mohm;
                        if (ki_calculated > 5000) ki_calculated = 5000;  // Clamp max Ki
                        if (ki_calculated < 10) ki_calculated = 10;      // Clamp min Ki
                    } else {
                        ki_calculated = 100;  // Default Ki if resistance invalid
                    }
                    
                    // Set PI gains with calculated Ki (Kp = 1000 default)
                    pid_d_ = pid_int_new(1000, ki_calculated, 0, 0);
                    pid_q_ = pid_int_new(1000, ki_calculated, 0, 0);
                    
                    // Transition to HFI warmup instead of melody
                    motor_.set_status(DriverStatus_HFI_Warmup);
                    hfi_warmup_counter_ = 0;  // Reset warmup counter
                }
                
                was_complete = is_complete_now;
                    const int16_t* voltages = motor_selector_.get_voltages();
                
                out[0] = voltages[0];
                out[1] = voltages[1];
                out[2] = voltages[2];
                
                return;  // Skip normal voltage conversion
            }
                
            case DriverStatus_Ready:
                // FOC current control with calibrated current values from feedback
                foc_current_control(Vec2D(0, target_current), angle, feedback_.current.norm(), voltage_ab);  // Use feedback current
                break;
            
            case DriverStatus_PlayingMelody: {
                // Playing startup melody through motor coils
                Vec2D melody_output = speaker_.tick();  // Get audio waveform
                
                // Check if melody finished
                if (speaker_.is_finished()) {
                    // Transition to Ready mode after melody
                    motor_.set_status(DriverStatus_Ready);
                    voltage_ab = Vec2D(0, 0);
                } else {
                    // Output melody waveform to motor (inject into alpha-beta)
                    voltage_ab = melody_output;
                }
                break;
            }
            
            case DriverStatus_HFI_Warmup: {
                // HFI warmup phase: apply 1A DC current to D-axis for 1 second
                // This saturates the rotor magnetically for better HFI response
                
                // Convert 1000mA to normalized Q15 format for D-axis current target
                int16_t target_id_norm = feedback_.current.ma2norm(1000);  // 1A = 1000mA
                
                // Apply FOC current control with D-axis current only (Q-axis = 0)
                // Use angle = 0 since we're just warming up, not tracking position
                foc_current_control(Vec2D(target_id_norm, 0), 0, feedback_.current.norm(), voltage_ab);
                
                // Increment warmup counter
                hfi_warmup_counter_++;
                
                // Check if warmup period (1 second) is complete
                if (hfi_warmup_counter_ >= HFI_WARMUP_CYCLES) {
                    // Transition to HFI running mode
                    motor_.set_status(DriverStatus_HFI_Running);
                    
                    // Calculate HFI amplitude in duty cycle (Q15) from voltage
                    int16_t hfi_amplitude_duty = feedback_.supply.mv2duty(HFI_AMPLITUDE_MV);
                    
                    // Start HFI injection: 50 periods, 1000Hz, 5V amplitude
                    hfi_.start(hfi_amplitude_duty, HFI_FREQUENCY, HFI_PERIODS);
                }
                break;
            }
            
            case DriverStatus_HFI_Running: {
                // HFI running: inject high-frequency signal and measure current response
                // Combines DC current (1A in D-axis) with HFI voltage injection
                
                // === Step 1: Generate HFI voltage injection ===
                Vec2D hfi_voltage = hfi_.tick_voltage();  // Get HFI sin/cos voltage
                
                // === Step 2: Get current measurements and process through HFI DFT ===
                // Convert ABC currents to DQ frame for HFI analysis
                const int16_t* norm_currents = feedback_.current.norm();
                motor_selector_.tick_current(norm_currents);
                Vec2D current_ab = motor_selector_.get_currents();
                
                // For HFI, we analyze current in stationary frame (alpha-beta = D-Q at angle 0)
                // Pass current to HFI for DFT processing
                hfi_.tick_current(current_ab);
                
                // === Step 3: Calculate base voltage for 1A D-axis current ===
                // Use simple V = I * R calculation for DC component
                int32_t resistance_mohm = motor_.get_total_resistance();
                int32_t base_voltage_mv = (1000 * resistance_mohm) / 1000;  // 1A * R = V (in mV)
                int16_t base_duty = feedback_.supply.mv2duty(base_voltage_mv);
                
                // === Step 4: Combine base DC voltage with HFI injection ===
                // Base voltage goes to D-axis (cos), HFI adds oscillation
                voltage_ab.cos = base_duty + hfi_voltage.cos;  // D-axis: DC + HFI
                voltage_ab.sin = 0;              // Q-axis: HFI only
                
                // === Step 5: Check if HFI measurement cycle is complete ===
                if (hfi_.complete()) {
                    // Extract angle estimate from HFI phase measurements
                    // The phase difference between D and Q axis responses indicates rotor position
                    // For salient pole motors: angle ≈ (phase_d - phase_q) / 2
                    int16_t phase_d = hfi_.get_phase_d();
                    int16_t phase_q = hfi_.get_phase_q();
                    
                    // Calculate angle estimate from phase difference
                    // Divide by 2 because HFI sees 2x electrical angle due to saliency
                    hfi_angle_estimate_ = (phase_d - phase_q) >> 1;
                    
                    // Restart HFI for continuous angle tracking
                    int16_t hfi_amplitude_duty = feedback_.supply.mv2duty(HFI_AMPLITUDE_MV);
                    hfi_.start(hfi_amplitude_duty, HFI_FREQUENCY, HFI_PERIODS);
                }
                break;
            }
                
            case DriverStatus_Error:
                // Output motor data to serial on error
                motor_.print(SerialUSB);
                
                // Restart motor test after error
                motor_.set_status(DriverStatus_MotorConnectionCalibration);
                motor_test_.start(true);
                voltage_ab = Vec2D(0, 0);
                break;
                
            case DriverStatus_Calibrating:
            default:
                voltage_ab = Vec2D(0, 0);
                break;
        }

        // Convert AB voltages to ABC phase voltages with phase remapping
        motor_selector_.tick_voltage(voltage_ab);
        const int16_t* voltages = motor_selector_.get_voltages();
        
        out[0] = voltages[0];
        out[1] = voltages[1];
        out[2] = voltages[2];

    }

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

    const int16_t* get_voltages() const {
        return motor_selector_.get_voltages();
    }

    const int16_t* get_control() const {
        return motor_selector_.get_voltages();
    }
    
    // Configure PI controller gains for current control
    void set_current_pi_gains(int32_t kp, int32_t ki) {
        pid_d_ = pid_int_new(kp, ki, 0, 0);
        pid_q_ = pid_int_new(kp, ki, 0, 0);
    }
    
    // Set maximum duty cycle limit
    void set_duty(uint16_t duty_permille) {
        motor_selector_.set_duty(duty_permille);
    }
    
    // Set current balance tolerance
    void set_current_epsilon(int16_t epsilon) {
        motor_selector_.set_current_epsilon(epsilon);
    }
    
    // Configure LPF filter alpha for current measurements (0-255, where 0=no filtering, 255=max filtering)
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
    void get_current_dq(const int16_t currents_abc[3], int16_t encoder_angle, 
                       int16_t* out_id, int16_t* out_iq) {
        Pol2D encoder_pos(encoder_angle, INT16_MAX);
        Vec2D current_dq = get_current_dq(currents_abc, encoder_pos);
        *out_id = current_dq.sin;
        *out_iq = current_dq.cos;
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
};
