#pragma once

// =============================================================================
// DC Phase Check - Motor Resistance Measurement via Adaptive PI Current Control
// =============================================================================
//
// PURPOSE:
// Measures phase-to-phase resistance for all 3 phase combinations (ABC, BCA, CAB)
// using adaptive PI controller with slew rate limiting for safe operation.
// Detects motor type (BLDC/Stepper/DC) based on resistance patterns.
//
// ALGORITHM OVERVIEW:
// ─────────────────────────────────────────────────────────────────────────────
//
// 1. PI ADAPTIVE CONTROL STAGE (max 75ms):
//    - PI controller regulates VOLTAGE (not duty directly)
//    - Slew rate limited for minimum inductance (100µH) safety
//    - PI coefficients recalculated every ~500µs based on R/L estimates
//    - Initial coefficients calculated for worst-case: L=100µH, R=50mΩ
//    - Duty calculated from target voltage / actual supply voltage
//    - Exit early if |i - i_ref| and |Δi/Δt| are small for N consecutive cycles
//
// 2. SETTLE STAGE (50ms):
//    - Maintain constant voltage found during PI stage
//    - Continue monitoring current for stability
//    - Accumulate V/I samples for resistance calculation
//    - OVERCURRENT CHECK: If current exceeds threshold → R=0 (short circuit)
//
// 3. MEASUREMENT & RESISTANCE CALCULATION:
//    - Average accumulated V and I samples
//    - Calculate R = V_avg / I_avg in milliohms
//    - Two-point method: repeat with doubled voltage for accuracy
//
// 4. PAUSE STAGE (HiZ discharge, 50ms):
//    - Set all phases to High-Z (floating)
//    - Allow motor inductance to discharge safely
//
// 5. REPEAT FOR ALL 3 PATTERNS:
//    - Pattern 0 (ABC): Phase A+, Phase B-, Phase C floating
//    - Pattern 1 (BCA): Phase B+, Phase C-, Phase A floating
//    - Pattern 2 (CAB): Phase C+, Phase A-, Phase B floating
//
// PI COEFFICIENT CALCULATION:
// ─────────────────────────────────────────────────────────────────────────────
// - Time constant τ = L/R (motor electrical time constant)
// - Target closed-loop bandwidth: τ_cl < 100ms (user requirement)
// - Kp = L / (τ_cl * Vdc) - proportional gain
// - Ki = R / (τ_cl * Vdc) - integral gain
// - Slew rate limit: dV/dt_max = Vdc * R / L_min (for L_min = 100µH)
//
// MOTOR TYPE DETECTION:
// ─────────────────────────────────────────────────────────────────────────────
// - BLDC: All 3 resistances valid and within 25% deviation
// - Stepper: Two equal resistances within 25%, third = sum of two (±20%)
// - DC: Only one valid resistance, pattern with max R = connected phases
// - Invalid: No valid resistance measurements
//
// =============================================================================

#include "../../common.h"       // Common types and utilities
#include "../types.h"           // Motor types and enums
#include "../motor.h"           // Motor configuration structure
#include "../../math_integer/filters/lpf.h"  // Low-pass filter

struct Feedback;                // Forward declaration for feedback system
struct MotorSelector;           // Forward declaration for motor selector
#include "sel_motor.h"          // Motor selector implementation

// =============================================================================
// TIMING CONSTANTS (cycles @ 18kHz PWM, ~55.5µs per cycle)
// =============================================================================
constexpr uint16_t PI_MAX_CYCLES = 1700;       // 75ms max PI convergence time
constexpr uint16_t PI_SETTLE_CYCLES = 1700;     // 50ms settle with constant voltage
constexpr uint16_t PI_PAUSE_CYCLES = 900;      // 50ms HiZ discharge between patterns
constexpr uint16_t PI_REPEAT_WAIT_CYCLES = 900;// 50ms wait between repeat cycles
constexpr uint16_t PI_MEASURE_CYCLES = 50;     // 2.5ms measurement accumulation
constexpr uint16_t PI_RECALC_CYCLES = 20;       // ~500µs between PI coefficient updates

// =============================================================================
// MOTOR PARAMETER BOUNDS (for initial PI calculation)
// =============================================================================
constexpr int32_t L_MIN_UH = 100;              // Minimum inductance: 100µH
constexpr int32_t L_MAX_UH = 300000;           // Maximum inductance: 300mH
constexpr int32_t R_MIN_MOHM = 50;             // Minimum resistance: 50mΩ
constexpr int32_t R_MAX_MOHM = 100000;         // Maximum resistance: 100Ω
constexpr int32_t TAU_CL_US = 100000;          // Target closed-loop time constant: 100ms

// =============================================================================
// PI CONTROLLER CONSTANTS
// =============================================================================
constexpr int32_t PI_KP_SCALE = 1000;          // Kp scaling factor for fixed-point
constexpr int32_t PI_KI_SCALE = 1000;          // Ki scaling factor for fixed-point
constexpr int32_t PI_INTEGRAL_MAX = 100000;    // Max integral accumulator (mV)

// =============================================================================
// SLEW RATE CONSTANTS (for L_min = 100µH safety)
// =============================================================================
// dV/dt_max = Vdc * R / L_min, but we limit to safe value
// At 24V, 50mΩ, 100µH: dV/dt = 24 * 0.05 / 0.0001 = 12000 V/s = 12 mV/µs
// Per PWM cycle (55.5µs): ~666mV max change
// We use conservative 200mV/cycle for safety margin
constexpr int16_t SLEW_RATE_MV_PER_CYCLE = 25;// Max voltage change per PWM cycle (mV)

// =============================================================================
// CONVERGENCE CRITERIA
// =============================================================================
constexpr int16_t PI_CURRENT_TOL_Q15 = 820;    // Current tolerance: ~2.5% of full scale
constexpr int16_t PI_DERIV_TOL_Q15 = 164;      // Derivative tolerance: ~0.5% per tick
constexpr uint8_t PI_STABLE_COUNT = 10;        // Consecutive stable cycles required

// =============================================================================
// VOLTAGE AND CURRENT LIMITS
// =============================================================================
constexpr int16_t PI_MIN_VOLTAGE_MV = 500;     // Minimum applied voltage (mV)
constexpr int16_t PI_MAX_DUTY_Q15 = 24576;     // Max duty: 75% of supply (3/4)

// =============================================================================
// RESISTANCE VALIDATION BOUNDS (mOhm)
// =============================================================================
constexpr int32_t PI_MIN_R_MOHM = 50;          // Minimum valid: 50 mOhm
constexpr int32_t PI_MAX_R_MOHM = 100000;      // Maximum valid: 100 Ohm

// =============================================================================
// LPF CONFIGURATION
// =============================================================================
constexpr uint8_t PI_LPF_ALPHA = 220;          // LPF alpha for current filtering

// =============================================================================
// STATE MACHINE STATES
// =============================================================================
enum PIState : uint8_t {
    PI_Idle = 0,        // Inactive state
    PI_Control,         // PI adaptive control phase
    PI_Settle,          // Constant voltage settle phase
    PI_Measure,         // Measurement accumulation phase
    PI_Pause,           // HiZ discharge between patterns
    PI_Complete,        // Test finished
    PI_RepeatWait,      // Wait before next iteration
    PI_Error            // Error state with HiZ
};

// =============================================================================
// DC PHASE CHECK STRUCTURE - ADAPTIVE PI IMPLEMENTATION
// =============================================================================
struct DCPhaseCheck {
private:
    // =========================================================================
    // REFERENCES TO EXTERNAL SYSTEMS
    // =========================================================================
    Feedback& fb_;                // Reference to feedback system for sensors
    Motor& motor_;                // Reference to motor configuration storage
    MotorSelector& sel_;          // Reference to motor selector for PWM output
    
    // =========================================================================
    // STATE MACHINE VARIABLES
    // =========================================================================
    PIState state_;               // Current test state
    uint16_t cnt_;                // Cycle counter for timing
    uint16_t recalc_cnt_;         // Counter for PI coefficient recalculation
    uint8_t pat_;                 // Current phase pattern index (0-2)
    
    // =========================================================================
    // CURRENT FILTERING
    // =========================================================================
    FilterLPF lpf_;               // Low-pass filter for current smoothing
    
    // =========================================================================
    // VOLTAGE CONTROL (in millivolts, not duty!)
    // =========================================================================
    int32_t voltage_target_mv_;   // Target voltage from PI controller (mV)
    int32_t voltage_output_mv_;   // Actual output voltage after slew limiting (mV)
    int16_t duty_current_;        // Current duty cycle output (Q15 format)
    int16_t voltage_mv_settled_;  // Voltage found during PI (mV) for settle
    
    // =========================================================================
    // PI CONTROLLER STATE
    // =========================================================================
    int32_t pi_kp_;               // Proportional gain (scaled by PI_KP_SCALE)
    int32_t pi_ki_;               // Integral gain (scaled by PI_KI_SCALE)
    int32_t pi_integral_;         // Integral accumulator (mV)
    int16_t pi_prev_error_;       // Previous error for derivative check
    
    // =========================================================================
    // ESTIMATED MOTOR PARAMETERS (updated during control)
    // =========================================================================
    int32_t est_r_mohm_;          // Estimated resistance (mΩ)
    int32_t est_l_uh_;            // Estimated inductance (µH)
    
    // =========================================================================
    // CONVERGENCE TRACKING
    // =========================================================================
    int16_t prev_filtered_;       // Previous filtered current for derivative
    uint8_t stable_cnt_;          // Counter for consecutive stable cycles
    
    // =========================================================================
    // TWO-POINT MEASUREMENT DATA
    // =========================================================================
    int32_t v1_mv_;               // First measurement voltage (mV)
    int32_t i1_ma_;               // First measurement current (mA)
    int32_t v2_mv_;               // Second measurement voltage (mV)
    int32_t i2_ma_;               // Second measurement current (mA)
    
    // =========================================================================
    // MEASUREMENT ACCUMULATOR
    // =========================================================================
    int32_t voltage_acc_;         // Voltage accumulator for averaging (mV sum)
    int32_t current_acc_;         // Current accumulator for averaging (Q15 sum)
    uint16_t sample_cnt_;         // Sample counter for measurement
    
    // =========================================================================
    // RESISTANCE RESULTS [mOhm] FOR EACH PATTERN
    // =========================================================================
    int32_t r_[3];                // Measured resistances for ABC, BCA, CAB
    
    // =========================================================================
    // FLAGS (bit-packed for memory efficiency)
    // =========================================================================
    uint8_t second_cycle_ : 1;    // True if measuring second point
    uint8_t repeat_mode_ : 1;     // True if continuous repeat enabled
    uint8_t cycle_complete_ : 1;  // True when one full cycle done
    
    // =========================================================================
    // OVERCURRENT THRESHOLD (Q15 normalized, 1/2 of full scale)
    // =========================================================================
    static constexpr int16_t OVERCURRENT_THRESHOLD = INT16_MAX / 2;
    
    // =========================================================================
    // STORED FIRST POINT CURRENT FOR SECOND POINT CALCULATION
    // =========================================================================
    int32_t first_point_ma_;                  // Measured current at first point (mA)

    // =========================================================================
    // HELPER: Get target current for first measurement point (I_lim/4)
    // =========================================================================
    inline int32_t get_first_point_current() const {
        return motor_.get_current_limit() / 4;  // Quarter of configured current limit
    }
    
    // =========================================================================
    // HELPER: Get target current for second measurement point (2x first)
    // =========================================================================
    inline int32_t get_second_point_current() const {
        // Second point = 2x the actual measured first point current
        // This guarantees second > first regardless of PI accuracy
        int32_t target = first_point_ma_ * 2;
        const int32_t limit = motor_.get_current_limit();
        if (target > limit) target = limit;     // Clamp to limit
        if (target < first_point_ma_ + 50) target = first_point_ma_ + 50; // Min 50mA difference
        return target;
    }

    // =========================================================================
    // HELPER: Check if resistance value is within valid bounds
    // =========================================================================
    inline bool valid_r(int32_t r) const {
        return (r >= PI_MIN_R_MOHM) && (r <= PI_MAX_R_MOHM);
    }
    
    // =========================================================================
    // HELPER: Get absolute value of current (Q15 input)
    // =========================================================================
    inline int16_t abs_current(int16_t norm) const {
        return (norm < 0) ? -norm : norm;       // Return absolute value
    }
    
    // =========================================================================
    // HELPER: Convert normalized current (Q15) to milliamps
    // =========================================================================
    inline int32_t norm_to_ma(int16_t norm) const {
        const int32_t fs = fb_.current.get_full_scale_max_ma();
        return ((int32_t)norm * fs) >> 15;      // Scale Q15 to milliamps
    }
    
    // =========================================================================
    // HELPER: Convert milliamps to normalized current (Q15)
    // =========================================================================
    inline int16_t ma_to_norm(int32_t ma) const {
        const int32_t fs = fb_.current.get_full_scale_max_ma();
        if (fs <= 0) return 0;                  // Avoid division by zero
        int32_t norm = (ma << 15) / fs;         // Scale milliamps to Q15
        if (norm > INT16_MAX) return INT16_MAX; // Clamp to max
        if (norm < -INT16_MAX) return -INT16_MAX;
        return (int16_t)norm;
    }
    
    // =========================================================================
    // HELPER: Calculate duty cycle from voltage with supply compensation
    // Input: target voltage in mV, Output: duty in Q15
    // =========================================================================
    inline int16_t voltage_to_duty(int32_t target_mv) const {
        const int32_t supply_mv = fb_.supply.get_voltage_mv();
        if (supply_mv <= 0) return 0;           // Avoid division by zero
        int32_t duty = (target_mv << 15) / supply_mv;
        if (duty > PI_MAX_DUTY_Q15) duty = PI_MAX_DUTY_Q15; // Clamp to 75%
        if (duty < 0) duty = 0;                 // No negative duty
        return (int16_t)duty;
    }
    
    // =========================================================================
    // HELPER: Apply PWM pattern using motor selector
    // =========================================================================
    void apply_pattern() {
        Vec2D voltage(duty_current_, 0);        // DC mode uses sin component only
        sel_.tick_voltage(voltage);             // Apply voltage through selector
    }
    
    // =========================================================================
    // HELPER: Set all phases to High-Z (floating)
    // =========================================================================
    inline void set_hiz() {
        sel_.set_motor_type(MotorType_Invalid); // Invalid type outputs HiZ
        sel_.tick_voltage(Vec2D(0, 0));         // Trigger voltage update
    }
    
    // =========================================================================
    // HELPER: Get current measurement from selector
    // =========================================================================
    inline int16_t get_current() {
        const int16_t* currents = fb_.current.norm();
        sel_.tick_current(currents);            // Process through selector
        return sel_.get_currents().sin;         // Return DC mode current
    }
    
    // =========================================================================
    // HELPER: Setup motor type and pattern for DC measurement
    // =========================================================================
    inline void setup_dc_mode() {
        sel_.set_motor_type(MotorType_DC);      // Set DC mode
        sel_.set_phase_pattern(static_cast<PhasePattern>(pat_));
    }
    
    // =========================================================================
    // HELPER: Check for overcurrent condition
    // =========================================================================
    inline bool check_overcurrent(int16_t current_norm) const {
        return abs_current(current_norm) > OVERCURRENT_THRESHOLD;
    }
    
    // =========================================================================
    // HELPER: Handle overcurrent - set R=0 and transition to pause
    // =========================================================================
    void handle_overcurrent() {
        r_[pat_] = 0;                           // Mark as short circuit
        motor_.set_phase_resistance(pat_, 0);   // Store in motor config
        state_ = PI_Pause;                      // Transition to pause
        cnt_ = 0;                               // Reset counter
        set_hiz();                              // Disable output
    }

    // =========================================================================
    // PI COEFFICIENT CALCULATION - Based on estimated R and L
    // Calculates Kp and Ki for target closed-loop time constant
    // Formula: Kp = L / (τ_cl * Vdc), Ki = R / (τ_cl * Vdc)
    // =========================================================================
    void calc_pi_coefficients() {
        const int32_t supply_mv = fb_.supply.get_voltage_mv();
        if (supply_mv <= 0) return;             // Avoid division by zero
        
        // Use estimated or default parameters
        int32_t r_mohm = est_r_mohm_;           // Resistance in mΩ
        int32_t l_uh = est_l_uh_;               // Inductance in µH
        
        // Clamp to valid ranges
        if (r_mohm < R_MIN_MOHM) r_mohm = R_MIN_MOHM;
        if (r_mohm > R_MAX_MOHM) r_mohm = R_MAX_MOHM;
        if (l_uh < L_MIN_UH) l_uh = L_MIN_UH;
        if (l_uh > L_MAX_UH) l_uh = L_MAX_UH;
        
        // Kp = L / (τ_cl * Vdc)
        // L in µH, τ_cl in µs, Vdc in mV
        // Kp = (L_uh * 1000) / (TAU_CL_US * supply_mv / 1000)
        // Kp = (L_uh * 1000000) / (TAU_CL_US * supply_mv)
        // Scale by PI_KP_SCALE for fixed-point
        int64_t kp_num = (int64_t)l_uh * 1000000LL * PI_KP_SCALE;
        int64_t kp_den = (int64_t)TAU_CL_US * supply_mv;
        if (kp_den > 0) {
            pi_kp_ = (int32_t)(kp_num / kp_den);
        }
        
        // Ki = R / (τ_cl * Vdc)
        // R in mΩ, τ_cl in µs, Vdc in mV
        // Ki = (R_mohm * 1000) / (TAU_CL_US * supply_mv / 1000)
        // Ki = (R_mohm * 1000000) / (TAU_CL_US * supply_mv)
        // Scale by PI_KI_SCALE for fixed-point
        int64_t ki_num = (int64_t)r_mohm * 1000000LL * PI_KI_SCALE;
        int64_t ki_den = (int64_t)TAU_CL_US * supply_mv;
        if (ki_den > 0) {
            pi_ki_ = (int32_t)(ki_num / ki_den);
        }
        
        // Ensure minimum gains for stability
        if (pi_kp_ < 1) pi_kp_ = 1;
        if (pi_ki_ < 1) pi_ki_ = 1;
    }
    
    // =========================================================================
    // PI INITIALIZATION - Reset controller state for new measurement
    // =========================================================================
    void pi_init() {
        // Set initial estimates to worst-case (fastest response)
        est_r_mohm_ = R_MIN_MOHM;               // Start with minimum R
        est_l_uh_ = L_MIN_UH;                   // Start with minimum L
        
        // Calculate initial PI coefficients
        calc_pi_coefficients();
        
        // Reset controller state
        pi_integral_ = 0;                       // Clear integral accumulator
        pi_prev_error_ = 0;                     // Clear previous error
        voltage_target_mv_ = 0;                 // Clear target voltage
        voltage_output_mv_ = 0;                 // Clear output voltage
        stable_cnt_ = 0;                        // Reset stability counter
        prev_filtered_ = 0;                     // Reset derivative tracking
        recalc_cnt_ = 0;                        // Reset recalc counter
    }
    
    // =========================================================================
    // UPDATE R/L ESTIMATES - Based on current V/I measurements
    // Called periodically during control to refine PI coefficients
    // =========================================================================
    void update_estimates(int16_t current_norm, int16_t prev_current_norm) {
        // Need non-zero current for estimation
        const int32_t i_ma = norm_to_ma(abs_current(current_norm));
        if (i_ma < 10) return;                  // Skip if current too low
        
        // Estimate R from steady-state: R = V / I
        // V in mV, I in mA -> R in mΩ: R = V * 1000 / I
        if (voltage_output_mv_ > 0) {
            int32_t r_est = (voltage_output_mv_ * 1000) / i_ma;
            // Low-pass filter the estimate
            est_r_mohm_ = est_r_mohm_ + ((r_est - est_r_mohm_) >> 3);
        }
        
        // Estimate L from transient: L = V * dt / di
        // di = change in current over one PWM cycle
        const int32_t di_ma = norm_to_ma(abs_current(current_norm) - abs_current(prev_current_norm));
        if (di_ma != 0) {
            // dt = 55.5µs (one PWM cycle at 18kHz)
            // L = V * 55.5 / di (in µH if V in mV, di in mA)
            int32_t l_est = (voltage_output_mv_ * 56) / di_ma;
            if (l_est < 0) l_est = -l_est;      // Absolute value
            if (l_est >= L_MIN_UH && l_est <= L_MAX_UH) {
                // Low-pass filter the estimate
                est_l_uh_ = est_l_uh_ + ((l_est - est_l_uh_) >> 4);
            }
        }
    }

    // =========================================================================
    // PI CONTROLLER TICK - Calculate voltage command with slew rate limiting
    // Input: current error (target - measured) in Q15
    // Output: updates voltage_target_mv_ and voltage_output_mv_
    // =========================================================================
    void pi_tick(int16_t error_norm) {
        // Convert error to mA for PI calculation
        const int32_t error_ma = norm_to_ma(error_norm);
        
        // Proportional term: P = Kp * error
        int32_t p_term = (pi_kp_ * error_ma) / PI_KP_SCALE;
        
        // Integral term: I += Ki * error, with anti-windup
        pi_integral_ += (pi_ki_ * error_ma) / PI_KI_SCALE;
        
        // Anti-windup: clamp integral to prevent excessive accumulation
        if (pi_integral_ > PI_INTEGRAL_MAX) pi_integral_ = PI_INTEGRAL_MAX;
        if (pi_integral_ < -PI_INTEGRAL_MAX) pi_integral_ = -PI_INTEGRAL_MAX;
        
        // Calculate target voltage: V = P + I
        voltage_target_mv_ = p_term + pi_integral_;
        
        // Clamp target to valid range
        const int32_t max_v = (fb_.supply.get_voltage_mv() * 3) / 4; // 75% of supply
        if (voltage_target_mv_ > max_v) voltage_target_mv_ = max_v;
        if (voltage_target_mv_ < PI_MIN_VOLTAGE_MV) voltage_target_mv_ = PI_MIN_VOLTAGE_MV;
        
        // Apply slew rate limiting for safe operation with low inductance
        int32_t delta = voltage_target_mv_ - voltage_output_mv_;
        if (delta > SLEW_RATE_MV_PER_CYCLE) {
            voltage_output_mv_ += SLEW_RATE_MV_PER_CYCLE;
        } else if (delta < -SLEW_RATE_MV_PER_CYCLE) {
            voltage_output_mv_ -= SLEW_RATE_MV_PER_CYCLE;
        } else {
            voltage_output_mv_ = voltage_target_mv_;
        }
        
        // Ensure output stays in valid range
        if (voltage_output_mv_ < 0) voltage_output_mv_ = 0;
        if (voltage_output_mv_ > max_v) voltage_output_mv_ = max_v;
        
        // Store error for convergence check
        pi_prev_error_ = error_norm;
    }
    
    // =========================================================================
    // CHECK CONVERGENCE - Determine if current is stable at target
    // Returns true if stable for required number of consecutive cycles
    // =========================================================================
    bool check_convergence(int16_t filtered, int16_t target) {
        // Calculate current error
        int16_t error = filtered - target;
        if (error < 0) error = -error;          // Absolute error
        
        // Calculate derivative (rate of change)
        int16_t deriv = filtered - prev_filtered_;
        if (deriv < 0) deriv = -deriv;          // Absolute derivative
        prev_filtered_ = filtered;              // Store for next tick
        
        // Check if within tolerance
        if (error < PI_CURRENT_TOL_Q15 && deriv < PI_DERIV_TOL_Q15) {
            ++stable_cnt_;                      // Increment stable counter
            if (stable_cnt_ >= PI_STABLE_COUNT) {
                return true;                    // Converged!
            }
        } else {
            stable_cnt_ = 0;                    // Reset counter on instability
        }
        
        return false;
    }

    // =========================================================================
    // CALCULATE RESISTANCE - From accumulated V/I measurements
    // Uses absolute values to handle any measurement order
    // =========================================================================
    void calc_resistance() {
        // Use absolute difference for both voltage and current
        int32_t dv = v2_mv_ - v1_mv_;           // Voltage difference (mV)
        int32_t di = i2_ma_ - i1_ma_;           // Current difference (mA)
        
        // Take absolute values
        if (dv < 0) dv = -dv;                   // |dV|
        if (di < 0) di = -di;                   // |dI|
        
        if (di == 0) {
            r_[pat_] = INT32_MAX;               // Open circuit (no current change)
        } else {
            // R = |dV|/|dI| in milliohms: (mV * 1000) / mA = mOhm
            r_[pat_] = (dv * 1000) / di;        // Always positive result
        }
        motor_.set_phase_resistance(pat_, r_[pat_]); // Store result
    }

    // =========================================================================
    // ADVANCE TO NEXT PATTERN OR COMPLETE TEST
    // =========================================================================
    void next_pattern() {
        ++pat_;                                 // Increment pattern index
        if (pat_ >= 3) {
            detect_type();                      // Determine motor type
            print_results();                    // Print calibration results
            if (repeat_mode_) {
                cycle_complete_ = 1;            // Mark cycle complete
                state_ = PI_RepeatWait;         // Wait before next cycle
                cnt_ = 0;
            } else {
                state_ = PI_Complete;           // Test finished
            }
        } else {
            start_pattern();                    // Start next pattern
        }
    }
    
    // =========================================================================
    // INITIALIZE MEASUREMENT FOR CURRENT PATTERN
    // =========================================================================
    void start_pattern() {
        state_ = PI_Control;                    // Start with PI control
        cnt_ = 0;                               // Reset cycle counter
        duty_current_ = 0;                      // Start from zero duty
        second_cycle_ = 0;                      // First measurement point
        voltage_mv_settled_ = 0;                // Clear settled voltage
        first_point_ma_ = 0;                    // Clear first point current
        
        // Initialize PI controller
        pi_init();
        
        // Reset LPF with alpha for filtering
        lpf_.reset(0);
        lpf_.set_alpha(PI_LPF_ALPHA);
        
        // Clear measurement accumulators
        voltage_acc_ = 0;
        current_acc_ = 0;
        sample_cnt_ = 0;
    }

    // =========================================================================
    // MOTOR TYPE DETECTION - Based on resistance patterns
    // =========================================================================
    void detect_type() {
        const int32_t r0 = r_[0], r1 = r_[1], r2 = r_[2];
        
        // Check validity of each measurement (valid = within bounds, not 0 or INT32_MAX)
        const uint8_t v0 = valid_r(r0);         // Pattern 0 valid
        const uint8_t v1 = valid_r(r1);         // Pattern 1 valid
        const uint8_t v2 = valid_r(r2);         // Pattern 2 valid
        const uint8_t vc = v0 + v1 + v2;        // Total valid count
        
        // Check for short circuits (R=0) - any short = invalid motor
        const bool has_short = (r0 == 0) || (r1 == 0) || (r2 == 0);
        if (has_short) {
            motor_.set_motor_type(MotorType_Invalid);  // Short circuit detected
            motor_.set_phase_pattern(ABC);
            motor_.set_deviation_percent(-1);
            return;
        }
        
        // No valid measurements at all = invalid
        if (vc==0 or vc==2) {
            motor_.set_motor_type(MotorType_Invalid);
            motor_.set_phase_pattern(ABC);
            motor_.set_deviation_percent(-1);
            return;
        }
        
        // Find max and min resistance among valid measurements
        int32_t rmax = 0, rmin = INT32_MAX;
        uint8_t maxi = 0;
        if (v0) { rmax = r0; rmin = r0; maxi = 0; }
        if (v1) { 
            if (r1 > rmax) { rmax = r1; maxi = 1; }
            if (r1 < rmin) rmin = r1;
        }
        if (v2) { 
            if (r2 > rmax) { rmax = r2; maxi = 2; }
            if (r2 < rmin) rmin = r2;
        }
        
        // Calculate deviation percentage and store it
        const int16_t dev = (rmax > 0) ? (int16_t)(((rmax - rmin) * 100) / rmax) : 100;
        motor_.set_deviation_percent(dev);      // Store deviation in motor config
        
        // DC motor pattern: max resistance indicates connected phases
        PhasePattern pp = static_cast<PhasePattern>(maxi);
        
        // BLDC: all 3 valid with deviation <= 25%
        if (vc == 3 && dev <= 25) {
            motor_.set_motor_type(MotorType_BLDC);
            motor_.set_phase_pattern(ABC);
            return;
        }
        
        // Stepper: two phases equal, third = sum of two (±20%)
        if (vc == 3) {
            const int32_t th = (rmax * 20) / 100;
            
            // Check ABC + BCA ≈ CAB
            int32_t s01 = r0 + r1;
            int32_t d01 = (r0 > r1) ? (r0 - r1) : (r1 - r0);
            int32_t d2s = (r2 > s01) ? (r2 - s01) : (s01 - r2);
            if (d01 <= th && d2s <= th) {
                motor_.set_motor_type(MotorType_STEP);
                motor_.set_phase_pattern(CAB);
                return;
            }
            
            // Check BCA + CAB ≈ ABC
            int32_t s12 = r1 + r2;
            int32_t d12 = (r1 > r2) ? (r1 - r2) : (r2 - r1);
            int32_t d0s = (r0 > s12) ? (r0 - s12) : (s12 - r0);
            if (d12 <= th && d0s <= th) {
                motor_.set_motor_type(MotorType_STEP);
                motor_.set_phase_pattern(ABC);
                return;
            }
            
            // Check CAB + ABC ≈ BCA
            int32_t s20 = r2 + r0;
            int32_t d20 = (r2 > r0) ? (r2 - r0) : (r0 - r2);
            int32_t d1s = (r1 > s20) ? (r1 - s20) : (s20 - r1);
            if (d20 <= th && d1s <= th) {
                motor_.set_motor_type(MotorType_STEP);
                motor_.set_phase_pattern(BCA);
                return;
            }
        }
        
        // DC motor: at least 1 valid measurement (open phases are OK for DC)
        motor_.set_motor_type(MotorType_DC);
        motor_.set_phase_pattern(pp);
    }

    // =========================================================================
    // PRINT CALIBRATION RESULTS TO SERIAL
    // =========================================================================
    void print_results() {
        SerialUSB.println("\n=== Motor Calibration ===");
        
        SerialUSB.print("Type: ");
        switch (motor_.get_motor_type()) {
            case MotorType_BLDC: SerialUSB.println("BLDC"); break;
            case MotorType_STEP: SerialUSB.println("Stepper"); break;
            case MotorType_DC:   SerialUSB.println("DC"); break;
            default:             SerialUSB.println("Invalid"); break;
        }
        
        SerialUSB.print("Pattern: ");
        switch (motor_.get_phase_pattern()) {
            case ABC: SerialUSB.println("ABC"); break;
            case BCA: SerialUSB.println("BCA"); break;
            case CAB: SerialUSB.println("CAB"); break;
        }
        
        // Print resistances
        SerialUSB.println("R [mOhm]:");
        const char* names[3] = {"ABC", "BCA", "CAB"};
        int32_t sum = 0;
        uint8_t valid_cnt = 0;
        for (uint8_t i = 0; i < 3; i++) {
            int32_t r = r_[i];
            SerialUSB.print("  ");
            SerialUSB.print(names[i]);
            SerialUSB.print(": ");
            if (r == INT32_MAX) {
                SerialUSB.println("OPEN");
            } else if (r == 0) {
                SerialUSB.println("SHORT");
            } else {
                SerialUSB.println(r);
                sum += r;
                ++valid_cnt;
            }
        }
        
        // Calculate and print average
        if (valid_cnt > 0) {
            int32_t avg = sum / valid_cnt;
            motor_.set_average_resistance(avg);
            SerialUSB.print("Avg: ");
            SerialUSB.print(avg);
            SerialUSB.println(" mOhm");
        }
        
        // Print deviation
        int16_t dev = motor_.get_deviation_percent();
        if (dev >= 0) {
            SerialUSB.print("Dev: ");
            SerialUSB.print(dev);
            SerialUSB.println("%");
        }
        
        SerialUSB.println("=========================\n");
    }

public:
    // =========================================================================
    // CONSTRUCTOR - Initialize with references to external systems
    // =========================================================================
    DCPhaseCheck(Feedback& fb, Motor& motor, MotorSelector& sel)
        : fb_(fb)                               // Store feedback reference
        , motor_(motor)                         // Store motor reference
        , sel_(sel)                             // Store selector reference
        , state_(PI_Idle)                       // Start in idle state
        , cnt_(0)                               // Reset counter
        , recalc_cnt_(0)                        // Reset recalc counter
        , pat_(0)                               // Start with pattern 0
        , lpf_(0, PI_LPF_ALPHA)                 // Initialize LPF
        , voltage_target_mv_(0)                 // Zero target voltage
        , voltage_output_mv_(0)                 // Zero output voltage
        , duty_current_(0)                      // Zero duty
        , voltage_mv_settled_(0)                // Zero settled voltage
        , pi_kp_(1)                             // Initial Kp
        , pi_ki_(1)                             // Initial Ki
        , pi_integral_(0)                       // Clear integral
        , pi_prev_error_(0)                     // Clear previous error
        , est_r_mohm_(R_MIN_MOHM)               // Initial R estimate
        , est_l_uh_(L_MIN_UH)                   // Initial L estimate
        , prev_filtered_(0)                     // No previous filtered
        , stable_cnt_(0)                        // Reset stable counter
        , first_point_ma_(0)                    // Clear first point current
        , v1_mv_(0), i1_ma_(0)                  // Clear first measurement
        , v2_mv_(0), i2_ma_(0)                  // Clear second measurement
        , voltage_acc_(0)                       // Clear voltage accumulator
        , current_acc_(0)                       // Clear current accumulator
        , sample_cnt_(0)                        // Clear sample counter
        , r_{0, 0, 0}                           // Clear resistances
        , second_cycle_(0)                      // First cycle
        , repeat_mode_(0)                       // Single shot mode
        , cycle_complete_(0)                    // Cycle not complete
    {}

    // =========================================================================
    // START - Begin DC phase check test
    // repeat: if true, continuously repeat measurements
    // =========================================================================
    void start(bool repeat = false) {
        repeat_mode_ = repeat ? 1 : 0;          // Store repeat mode
        cycle_complete_ = 0;                    // Reset cycle complete flag
        pat_ = 0;                               // Start with first pattern
        r_[0] = r_[1] = r_[2] = 0;              // Clear all resistances
        start_pattern();                        // Begin first pattern
    }
    
    // =========================================================================
    // STOP - Halt test and set HiZ
    // =========================================================================
    void stop() {
        state_ = PI_Idle;                       // Return to idle
        set_hiz();                              // Disable output
    }

    // =========================================================================
    // MAIN TICK - Call once per PWM cycle (18kHz)
    // =========================================================================
    void tick() {
        // Handle inactive states
        if (state_ == PI_Idle || state_ == PI_Complete || state_ == PI_Error) {
            set_hiz();                          // Ensure HiZ in inactive states
            return;
        }
        
        // Setup DC mode and pattern before reading current
        setup_dc_mode();
        
        // Get current measurement
        const int16_t current_norm = get_current();
        
        // State machine dispatch
        switch (state_) {
            case PI_Control:
                tick_pi_control(current_norm);
                break;
            case PI_Settle:
                tick_settle(current_norm);
                break;
            case PI_Measure:
                tick_measure(current_norm);
                break;
            case PI_Pause:
                tick_pause();
                break;
            case PI_RepeatWait:
                tick_repeat_wait();
                break;
            default:
                break;
        }
    }
    
    // =========================================================================
    // TICK OVERLOAD - For backward compatibility (ignores currents array)
    // =========================================================================
    void tick(const int16_t* /*currents*/) {
        tick();                                 // Call parameterless version
    }

private:
    // =========================================================================
    // PI CONTROL STAGE - Adaptive current control with slew rate limiting
    // =========================================================================
    void tick_pi_control(int16_t current_norm) {
        // Check for overcurrent
        if (check_overcurrent(current_norm)) {
            handle_overcurrent();
            return;
        }
        
        // Store previous current for L estimation
        const int16_t prev_current = (int16_t)(lpf_.get_output() - 32768);
        
        // Update LPF with current measurement (offset to unsigned for filter)
        lpf_.tick((uint16_t)(current_norm + 32768));
        
        // Get filtered current (convert back to signed Q15)
        const int16_t filtered = (int16_t)(lpf_.get_output() - 32768);
        
        // Get target current based on measurement point
        const int32_t target_ma = second_cycle_ ? get_second_point_current() : get_first_point_current();
        const int16_t target_norm = ma_to_norm(target_ma);
        
        // Calculate error: target - measured (positive error = need more current)
        const int16_t error = target_norm - abs_current(filtered);
        
        // Run PI controller to get voltage command
        pi_tick(error);
        
        // Convert voltage to duty cycle using actual supply voltage
        duty_current_ = voltage_to_duty(voltage_output_mv_);
        
        // Apply voltage
        apply_pattern();
        
        // Periodically update R/L estimates and recalculate PI coefficients
        ++recalc_cnt_;
        if (recalc_cnt_ >= PI_RECALC_CYCLES) {
            recalc_cnt_ = 0;
            update_estimates(filtered, prev_current);
            calc_pi_coefficients();
        }
        
        // Check convergence
        if (check_convergence(abs_current(filtered), target_norm)) {
            // Current stabilized - record voltage and transition to settle
            voltage_mv_settled_ = (int16_t)voltage_output_mv_;
            state_ = PI_Settle;
            cnt_ = 0;
            voltage_acc_ = 0;
            current_acc_ = 0;
            sample_cnt_ = 0;
            return;
        }
        
        ++cnt_;
        
        // Check timeout
        if (cnt_ >= PI_MAX_CYCLES) {
            // Timeout - use current voltage and proceed to settle
            voltage_mv_settled_ = (int16_t)voltage_output_mv_;
            state_ = PI_Settle;
            cnt_ = 0;
            voltage_acc_ = 0;
            current_acc_ = 0;
            sample_cnt_ = 0;
        }
    }

    // =========================================================================
    // SETTLE STAGE - Constant voltage, accumulate V/I samples
    // =========================================================================
    void tick_settle(int16_t current_norm) {
        // Check for overcurrent
        if (check_overcurrent(current_norm)) {
            handle_overcurrent();
            return;
        }
        
        // Update LPF
        lpf_.tick((uint16_t)(current_norm + 32768));
        
        // Maintain constant voltage (with supply compensation)
        duty_current_ = voltage_to_duty(voltage_mv_settled_);
        apply_pattern();
        
        // Accumulate samples for averaging (after initial settling)
        if (cnt_ >= PI_SETTLE_CYCLES - PI_MEASURE_CYCLES) {
            voltage_acc_ += voltage_mv_settled_;
            current_acc_ += abs_current(current_norm);
            ++sample_cnt_;
        }
        
        ++cnt_;
        
        // Check if settle complete
        if (cnt_ >= PI_SETTLE_CYCLES) {
            state_ = PI_Measure;
            cnt_ = 0;
        }
    }
    
    // =========================================================================
    // MEASURE STAGE - Calculate average and store measurement point
    // =========================================================================
    void tick_measure(int16_t current_norm) {
        // Check for overcurrent
        if (check_overcurrent(current_norm)) {
            handle_overcurrent();
            return;
        }
        
        // Maintain voltage
        duty_current_ = voltage_to_duty(voltage_mv_settled_);
        apply_pattern();
        
        // Continue accumulating
        voltage_acc_ += voltage_mv_settled_;
        current_acc_ += abs_current(current_norm);
        ++sample_cnt_;
        ++cnt_;
        
        // Check if measurement complete
        if (cnt_ >= PI_MEASURE_CYCLES) {
            // Calculate averages
            const int32_t avg_v_mv = voltage_acc_ / sample_cnt_;
            const int16_t avg_i_norm = (int16_t)(current_acc_ / sample_cnt_);
            const int32_t avg_i_ma = norm_to_ma(avg_i_norm);
            
            if (!second_cycle_) {
                // First measurement point
                v1_mv_ = avg_v_mv;
                i1_ma_ = avg_i_ma;
                first_point_ma_ = avg_i_ma;     // Store for second point target calculation
                
                // For second point: target current is 2x measured first point
                // PI controller will find appropriate voltage automatically
                // Reset for second measurement with higher target current
                second_cycle_ = 1;
                pi_init();                      // Reinitialize PI for new target
                state_ = PI_Control;            // Return to PI control
                cnt_ = 0;
                voltage_acc_ = 0;
                current_acc_ = 0;
                sample_cnt_ = 0;
            } else {
                // Second measurement point
                v2_mv_ = avg_v_mv;
                i2_ma_ = avg_i_ma;
                
                // Check for open circuit (current too low)
                if (avg_i_ma < 50) {            // Less than 50mA = open circuit
                    r_[pat_] = INT32_MAX;       // Open circuit
                    motor_.set_phase_resistance(pat_, INT32_MAX);
                } else {
                    calc_resistance();          // Calculate R from two points
                }
                
                // Move to pause
                state_ = PI_Pause;
                cnt_ = 0;
                set_hiz();
            }
        }
    }
    
    // =========================================================================
    // PAUSE STAGE - HiZ discharge between patterns
    // =========================================================================
    void tick_pause() {
        set_hiz();                              // Ensure HiZ
        ++cnt_;
        
        if (cnt_ >= PI_PAUSE_CYCLES) {
            next_pattern();                     // Advance to next pattern
        }
    }
    
    // =========================================================================
    // REPEAT WAIT STAGE - Delay between measurement cycles
    // =========================================================================
    void tick_repeat_wait() {
        set_hiz();                              // Ensure HiZ
        ++cnt_;
        
        if (cnt_ >= PI_REPEAT_WAIT_CYCLES) {
            cycle_complete_ = 0;                // Clear flag for new cycle
            pat_ = 0;                           // Reset to first pattern
            r_[0] = r_[1] = r_[2] = 0;          // Clear resistances
            start_pattern();                    // Start new cycle
        }
    }

public:
    // =========================================================================
    // STATUS QUERIES
    // =========================================================================
    
    // Check if test is complete (single shot mode)
    inline bool is_complete() const {
        return state_ == PI_Complete;
    }
    
    // Check if one measurement cycle is complete (repeat mode)
    inline bool is_cycle_complete() const {
        return cycle_complete_ != 0;
    }
    
    // Check if test is running
    inline bool is_running() const {
        return state_ != PI_Idle && state_ != PI_Complete && state_ != PI_Error;
    }
    
    // Check if in error state
    inline bool is_error() const {
        return state_ == PI_Error;
    }
    
    // Get current state (cast to uint8_t for compatibility)
    inline uint8_t get_state() const {
        return static_cast<uint8_t>(state_);
    }
    
    // Get current pattern index (0-2)
    inline uint8_t get_pattern() const {
        return pat_;
    }
    
    // Get resistance for specific pattern (mOhm)
    inline int32_t get_resistance(uint8_t pattern) const {
        return (pattern < 3) ? r_[pattern] : 0;
    }
    
    // Get all resistances
    inline const int32_t* get_resistances() const {
        return r_;
    }
    
    // Get current duty cycle (Q15)
    inline int16_t get_duty() const {
        return duty_current_;
    }
    
    // Get settled voltage (mV)
    inline int16_t get_target_voltage() const {
        return voltage_mv_settled_;
    }
    
    // Get filtered current from LPF (Q15, signed)
    inline int16_t get_filtered_current() const {
        return (int16_t)(lpf_.get_output() - 32768);
    }
    
    // Get current output voltage (mV)
    inline int32_t get_output_voltage() const {
        return voltage_output_mv_;
    }
    
    // Get estimated resistance (mΩ)
    inline int32_t get_est_resistance() const {
        return est_r_mohm_;
    }
    
    // Get estimated inductance (µH)
    inline int32_t get_est_inductance() const {
        return est_l_uh_;
    }
    
    // Get PI proportional gain
    inline int32_t get_pi_kp() const {
        return pi_kp_;
    }
    
    // Get PI integral gain
    inline int32_t get_pi_ki() const {
        return pi_ki_;
    }
};

// Backward compatibility alias
using MotorTest = DCPhaseCheck;

// Backward compatibility state enum alias
using DCState = PIState;
constexpr PIState DC_Idle = PI_Idle;
constexpr PIState DC_Ramp = PI_Control;         // Map old Ramp to new Control
constexpr PIState DC_Settle = PI_Settle;
constexpr PIState DC_Measure = PI_Measure;
constexpr PIState DC_Pause = PI_Pause;
constexpr PIState DC_Complete = PI_Complete;
constexpr PIState DC_RepeatWait = PI_RepeatWait;
constexpr PIState DC_Error = PI_Error;

// Legacy RLS state aliases for backward compatibility
using RLSState = PIState;
constexpr PIState RLS_Idle = PI_Idle;
constexpr PIState RLS_Control = PI_Control;
constexpr PIState RLS_Settle = PI_Settle;
constexpr PIState RLS_Measure = PI_Measure;
constexpr PIState RLS_Pause = PI_Pause;
constexpr PIState RLS_Complete = PI_Complete;
constexpr PIState RLS_RepeatWait = PI_RepeatWait;
constexpr PIState RLS_Error = PI_Error;
