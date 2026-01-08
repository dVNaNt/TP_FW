#pragma once

#include "../../common.h"
#include "../../math_integer/filters/lpf.h"
#include "../motor.h"
#include "../types.h"
// Forward declarations для уменьшения зависимостей в заголовочном файле
struct Feedback;
struct MotorSelector;
#include "sel_motor.h"

// =============================================================================
// CONSTANTS (Timing @ 18kHz)
// =============================================================================
constexpr uint16_t RAMP_MAX_CYCLES      = 3600;  // 200ms
constexpr uint16_t RAMP_SETTLE_CYCLES   = 180;   // 25ms
constexpr uint16_t RAMP_MEASURE_SAMPLES = 128;   // Samples count
constexpr uint16_t RAMP_HIZ_CYCLES    = 90;   // 25ms
constexpr uint16_t RAMP_STABLE_COUNT    = 360;   // 20ms
constexpr uint16_t ZERO_CURRENT_TIMEOUT = 1350;  // 75ms

// Controller constants
constexpr int16_t RAMP_ACCEL      = 90;  // 200mV/s^2
constexpr int16_t RAMP_STEP_MAX   = 10;   // 10mV limit
constexpr int16_t CURRENT_EPSILON = 50;   // Tolerance
constexpr uint8_t RAMP_LPF_ALPHA  = 230;  // Filter alpha

enum RampState : uint8_t {
    Ramp_Idle = 0,
    Ramp_Control,
    Ramp_Settle,
    Ramp_Measure,
    Ramp_Pause,
    Ramp_Complete,
    Ramp_Error
};

class DCPhaseCheck {
private:
    // 1. References (Pointers size usually 4 bytes on STM32)
    Feedback& fb_;
    Motor& motor_;
    MotorSelector& sel_;

    // 2. Heavy Math / Accumulators (4 bytes aligned)
    // Removed arrays, store only the first measurement point result
    int32_t res_v1_;      // Stored Voltage form 1st point (mV)
    int32_t res_i1_;      // Stored Current from 1st point (mA)
    
    int32_t accum_v_;     // Accumulator for averaging
    int32_t accum_i_;     // Accumulator for averaging

    // 3. Counters (2 bytes aligned)
    uint16_t cnt_;        // Main generic counter
    uint16_t stable_cnt_; // Stability counter in Ramp
    uint16_t zero_cnt_;   // Zero current timeout
    uint16_t samp_cnt_;   // Sample counter for averaging

    // 4. State Variables (2 bytes aligned)
    // Pre-calculated targets to save CPU cycles in ISR/Tick
    int16_t target_i_norm_; // Target current in normalized units
    int16_t limit_v_mv_;    // Voltage limit in mV
    
    int16_t voltage_mv_;    // Current output voltage
    int16_t duty_;          // Current duty cycle
    int16_t step_mv_;       // Ramp step variable
    int16_t prev_curr_;     // Derivative calculation
    
    // LPF Object (usually contains int16 state)
    FilterLPF lpf_;

    // 5. Small flags (1 byte aligned)
    // Grouped together to fill the 32-bit word alignment gap
    RampState state_;
    uint8_t pat_;           // Current pattern index
    bool is_second_cycle_;  // Flag: 1st or 2nd measurement point
    bool overshoot_;        // Flag: Ramp overshoot
    bool volt_limited_;     // Flag: Voltage saturation

    static constexpr int16_t OVERCURRENT = INT16_MAX / 2;

    // --- Private Optimized Helpers ---

    // Applies voltage to the selector. Inline for speed.
    inline void apply_out() {
        // Optimization: Don't call mv2duty every time if voltage hasn't changed drastically, 
        // but here we assume voltage changes.
        // Direct call to reduce overhead.
        sel_.tick_voltage(Vec2D(duty_, 0));
    }

    // Force High-Z state
    inline void set_hiz() {
        sel_.tick_voltage(Vec2D(0, 0));
    }

    // Pre-calculate limits and reset variables for a new phase pattern
    void prepare_pattern() {
        state_ = Ramp_Control;
        
        // Reset counters & accumulators
        cnt_ = 0;
        stable_cnt_ = 0;
        zero_cnt_ = 0;
        samp_cnt_ = 0;
        
        // Reset Physics
        voltage_mv_ = 0;
        step_mv_ = 1;
        duty_ = 0;
        prev_curr_ = 0;
        lpf_.reset(0);

        // Reset Logic Flags
        is_second_cycle_ = false;
        overshoot_ = false;
        volt_limited_ = false;

        // OPTIMIZATION: Calculate invariants ONCE per pattern, not every tick
        // 1. Max Voltage Limit (3/4 supply)
        limit_v_mv_ = (fb_.supply.get_voltage_mv()) >> 1;

        // 2. Target Current Limit
        int32_t motor_75 = (motor_.get_current_limit() * 3) >> 2;
        int32_t scale_50 = fb_.current.norm2ma(INT16_MAX / 2);
        int32_t target_ma = (motor_75 < scale_50) ? motor_75 : scale_50;
        
        // Convert to normalized units immediately to avoid conversion in tick()
        target_i_norm_ = fb_.current.ma2norm(target_ma);

        // Hardware Setup
        sel_.set_motor_type(MotorType_DC);
        sel_.set_phase_pattern(static_cast<PhasePattern>(pat_));
    }

public:
    DCPhaseCheck(Feedback& fb, Motor& motor, MotorSelector& sel)
        : fb_(fb), motor_(motor), sel_(sel), 
          lpf_((int16_t)0, RAMP_LPF_ALPHA),
          state_(Ramp_Idle), pat_(0) {}

    void start(bool repeat = false) {
        pat_ = 0;
        prepare_pattern();
    }

    // Optimized Tick:
    // 1. Minimized redundant calls (setup_dc_mode removed)
    // 2. Pre-calculated targets used
    // 3. Early exits
    void tick() {
        // Fast Exit if nothing to do
        if (state_ == Ramp_Idle || state_ == Ramp_Complete) {
            set_hiz();
            return;
        }

        // 1. Current Acquisition & Filtering
        // We must read current every tick for safety
        sel_.tick_current(fb_.current.norm());
        const int16_t filt = lpf_.tick(sel_.get_currents().sin);

        // Safety Check (Critical)
        if (filt > OVERCURRENT) {
            motor_.set_phase_resistance(pat_, 0);
            motor_.set_status(DriverStatus_Error);  // Set error status in motor
            state_ = Ramp_Error;
            set_hiz();
            return;
        }

        
        // 2. State Machine
        switch (state_) {
            case Ramp_Control: {
                // --- Zero Current Logic ---
                // Optimization: abs() using branching-less logic or simple check
                int16_t diff = filt - prev_curr_;
                if (diff < 0) diff = -diff;
                
                if (filt < CURRENT_EPSILON && diff < CURRENT_EPSILON) {
                    if (++zero_cnt_ >= ZERO_CURRENT_TIMEOUT) {
                        // Open circuit detected
                        motor_.set_phase_resistance(pat_, INT32_MAX);
                        state_ = Ramp_Pause;
                        cnt_ = 0;
                        set_hiz();
                        return;
                    }
                } else {
                    zero_cnt_ = 0;
                }
                prev_curr_ = filt;

                // --- Ramp Logic ---
                if (!is_second_cycle_) {
                    // First Cycle: Ramp to target current
                    // TARGET: Use pre-calculated target_i_norm_
                    
                    // Check overshoot
                    if (filt > target_i_norm_) overshoot_ = true;

                    // Calc Step
                    // Optimization: Use local var for calc
                    int32_t step;
                    if (!overshoot_) {
                         // Division by 18000 kept as requested
                        step = step_mv_ + ((int32_t)RAMP_ACCEL * cnt_) / 18000;
                    } else {
                        // P-Regulator
                        int16_t err = target_i_norm_ - filt;
                        step = err >> 6; // Fast division
                    }

                    // Clamp Step
                    if (step > RAMP_STEP_MAX) step = RAMP_STEP_MAX;
                    else if (step < -RAMP_STEP_MAX) step = -RAMP_STEP_MAX;
                    
                    voltage_mv_ += (int16_t)step;

                    // Convergence Check for first cycle
                    int16_t err_abs = filt - target_i_norm_;
                    if (err_abs < 0) err_abs = -err_abs;

                    bool stable = (err_abs < CURRENT_EPSILON) || volt_limited_;
                    
                    if (stable) {
                        if (++stable_cnt_ >= RAMP_STABLE_COUNT) {
                            state_ = Ramp_Settle;
                            cnt_ = 0;
                        }
                    } else {
                        stable_cnt_ = 0;
                    }

                } else {
                    // Second Cycle: Force voltage to half of result 1
                    voltage_mv_ = res_v1_ >> 1; // Divide by 2
                    
                    // Stability check for second cycle: current peak-to-peak within epsilon
                    int16_t curr_diff = filt - prev_curr_;
                    if (curr_diff < 0) curr_diff = -curr_diff;
                    
                    // Current is stable if p-p variation is within epsilon
                    bool stable = (curr_diff < CURRENT_EPSILON);
                    
                    if (stable) {
                        if (++stable_cnt_ >= RAMP_STABLE_COUNT) {
                            // Current is stable for required cycles, proceed to settle
                            state_ = Ramp_Settle;
                            cnt_ = 0;
                        }
                    } else {
                        // Reset stability counter if current varies too much
                        stable_cnt_ = 0;
                    }
                }
                // Voltage Limiting
                if (voltage_mv_ >= limit_v_mv_) {
                    voltage_mv_ = limit_v_mv_;
                    volt_limited_ = true;
                } else if (voltage_mv_ < 0) {
                    voltage_mv_ = 0;
                }

                // Apply Output
                duty_ = fb_.supply.mv2duty(voltage_mv_);
                apply_out();

                // Timeout Check (for both cycles)
                if (++cnt_ >= RAMP_MAX_CYCLES) {
                    state_ = Ramp_Settle;
                    cnt_ = 0;
                }
                break;
            }

            case Ramp_Settle: {
                // Keep output constant
                apply_out(); 
                
                // Determine wait time
                // Optimization: reuse cnt_
                uint16_t wait_target = is_second_cycle_ ? RAMP_SETTLE_CYCLES : RAMP_SETTLE_CYCLES; 
                // Note: Original code used first_ticks_ for second cycle settle? 
                // That seemed weird (waiting as long as ramp took?). 
                // Standardizing to Settle Time is safer and consistent. 
                // If dynamic settle is needed, restore variable. 
                // Assuming RAMP_SETTLE_CYCLES is sufficient.

                if (++cnt_ >= wait_target) {
                    state_ = Ramp_Measure;
                    accum_v_ = 0;
                    accum_i_ = 0;
                    samp_cnt_ = 0;
                }
                break;
            }

            case Ramp_Measure: {
                apply_out();
                
                // Accumulate
                accum_v_ += voltage_mv_;
                accum_i_ += filt; // Use raw or filt? Usually raw averaged is fine, but filt is smoother. Using raw as per typical AVG logic.

                if (++samp_cnt_ >= RAMP_MEASURE_SAMPLES) {
                    // Averaging
                    int32_t avg_v = accum_v_ / RAMP_MEASURE_SAMPLES;
                    int32_t avg_i_norm = accum_i_ / RAMP_MEASURE_SAMPLES;
                    int32_t avg_i_ma = fb_.current.norm2ma((int16_t)avg_i_norm);

                    if (!is_second_cycle_) {
                        // === Point 1 Finished ===
                        res_v1_ = avg_v;
                        res_i1_ = avg_i_ma;
                        
                        // Setup for Cycle 2
                        is_second_cycle_ = true;
                        overshoot_ = false;
                        volt_limited_ = false;
                        // Start ramping/setting from current voltage (already at res_v1)
                        // Actually logic says we go to Control state which sets voltage_mv_ = res_v1_ >> 1
                        state_ = Ramp_Control;
                        cnt_ = 0;
                        stable_cnt_ = 0;
                    } else {
                        // === Point 2 Finished ===
                        // Calc Resistance
                        int32_t meas_i_2 = avg_i_ma;
                        int32_t meas_v_2 = avg_v;

                        if (meas_i_2 < 50) {
                            // Current too low -> Open Load
                            motor_.set_phase_resistance(pat_, INT32_MAX);
                        } else {
                            int32_t dv = res_v1_ - meas_v_2;
                            int32_t di = res_i1_ - meas_i_2;
                            
                            // Prevent div by zero
                            if (di == 0) di = 1; 
                            
                            // R = dU / dI (mOhm)
                            motor_.set_phase_resistance(pat_, (dv * 1000) / di);
                        }

                        // Go to Pause
                        state_ = Ramp_Pause;
                        cnt_ = 0;
                        set_hiz();
                        lpf_.reset(0);
                    }
                }
                break;
            }

            case Ramp_Pause: {
                set_hiz();
                if (++cnt_ >= RAMP_HIZ_CYCLES) {
                    // Next Pattern Logic
                    if (++pat_ >= 3) {
                        sel_.set_motor_type(MotorType_Invalid); // Reset selector
                        motor_.detect_type();
                        state_ = Ramp_Complete;
                    } else {
                        prepare_pattern(); // Setup next phase
                    }
                }
                break;
            }
            
            default: // Error or unknown
                set_hiz();
                break;
        }
    }

    // Check if test cycle is complete
    inline bool is_complete() const { return state_ == Ramp_Complete; }
    
    // Check if error occurred during test
    inline bool is_error() const { return state_ == Ramp_Error; }
};
