#pragma once

#include "../../common.h"
#include "../types.h"
#include "sel_current.h"
#include "sel_voltage.h"

// Forward declaration
struct Feedback;

// ============================================================================
// MotorSelector - unified structure combining current, voltage, and phase selection
// ============================================================================
// Combines ABC2AB (current), AB2ABC (voltage), and integrated phase remapping
// into a single interface for motor control with minimal overhead.
//
// Input:
//   - vec2d voltages (alpha-beta)
//   - three current channels (ABC)
//   - motor type
//   - phase pattern (commutation type)
//   - control mode (voltage/current)
//
// Output:
//   - ABC voltages (after phase remapping)
//   - vec2d currents (alpha-beta)
//   - error flags
//   - access to current and voltage values
//
// Two main methods:
//   - tick_voltage: processes voltage path (AB -> ABC with phase remapping)
//   - tick_current: processes current path (ABC -> AB)
//
// Phase remapping is integrated directly for zero-overhead abstraction
// ============================================================================

struct MotorSelector {
private:
    // Reference to parent Feedback
    Feedback& feedback_;
    
    // Sub-modules
    ABC2AB current_converter_;  // Current: ABC -> AB
    AB2ABC voltage_converter_;  // Voltage: AB -> ABC

    // Configuration
    uint32_t motor_type_;         // Motor type: 1=DC, 4=Stepper, 6=BLDC
    PhasePattern phase_pattern_;  // Phase connection pattern

    // Integrated phase remapping indices (precomputed for zero overhead)
    uint8_t idx0_;  // Index for output channel 0
    uint8_t idx1_;  // Index for output channel 1
    uint8_t idx2_;  // Index for output channel 2

    // Output cache for voltage path (only final remapped result)
    Vec3D abc_voltages_remapped_;  // ABC voltages after phase remapping

    // Intermediate current values (after duty masking)
    Vec3D abc_currents_masked_;  // Physical currents after high-duty masking

    // Error flags
    uint8_t err_voltage_;  // Error flag for voltage path
    uint8_t err_current_;  // Error flag for current path

public:
    /// Constructor: initializes motor selector with motor type and phase pattern
    /// @param parent Reference to parent Feedback instance
    /// @param motor_type Motor type (1=DC, 4=Stepper, 6=BLDC)
    /// @param phase_pattern Phase connection pattern (ABC, ACB, CBA)
    /// @param control_mode Control mode (VoltageAB or CurrentAB)
    /// @param duty_permille Maximum duty cycle in permille (0-995 = 0.0%-99.5%)
    /// @param current_epsilon Balance tolerance for current in Q15 format
    MotorSelector(Feedback& parent,
                  uint32_t motor_type = MotorType_STEP,
                  PhasePattern phase_pattern = PhasePattern_ABC,
                  uint16_t duty_permille = 995u,
                  int16_t current_epsilon = 256)
        : feedback_(parent),
          current_converter_(parent, current_epsilon),
          voltage_converter_(parent, duty_permille),
          motor_type_(motor_type),
          phase_pattern_(phase_pattern),
          idx0_(0),
          idx1_(1),
          idx2_(2),
          abc_voltages_remapped_(0, 0, 0),
          err_voltage_(0u),
          err_current_(0u) {
        set_phase_pattern(phase_pattern);
    }

    /// Set motor type for both current and voltage converters
    /// @param motor_type Motor type (1=DC, 4=Stepper, 6=BLDC)
    inline void set_motor_type(uint32_t motor_type) {
        motor_type_ = motor_type;
    }
    
    /// Get current motor type
    /// @return Motor type (1=DC, 4=Stepper, 6=BLDC)
    inline uint32_t get_motor_type() const {
        return motor_type_;
    }

    /// Set phase connection pattern
    /// @param phase_pattern Phase pattern (ABC, ACB, CBA)
    void set_phase_pattern(PhasePattern phase_pattern) {
        phase_pattern_ = phase_pattern;
        // Precomputed permutations:
        // ABC -> [0,1,2]
        // BCA -> [1,2,0]
        // CAB -> [2,0,1]
        switch (phase_pattern) {
            case PhasePattern_CBA:
                idx0_ = 2;
                idx1_ = 1;
                idx2_ = 0;
                break;
            case PhasePattern_ACB:
                idx0_ = 0;
                idx1_ = 2;
                idx2_ = 1;
                break;
            case PhasePattern_ABC:
            default:
                idx0_ = 0;
                idx1_ = 1;
                idx2_ = 2;
                break;
        }
    }

    /// Set maximum duty cycle limit for voltage converter
    /// @param duty_permille Duty cycle in permille (0-995 = 0.0%-99.5%)
    inline void set_duty(uint16_t duty_permille) { voltage_converter_.set_duty(duty_permille); }

    /// Set balance tolerance for current converter
    /// @param epsilon Balance tolerance in Q15 format
    inline void set_current_epsilon(int16_t epsilon) { current_converter_.set_epsilon(epsilon); }

    /// Process voltage path: AB voltages -> ABC voltages with phase remapping
    /// @param voltage Vec2D voltage (sin=alpha, cos=beta) in Q15 format
    const int16_t* tick_voltage(Vec2D voltage) {
        // Convert AB to ABC using voltage converter
        voltage_converter_.tick(voltage, motor_type_);
        err_voltage_ = voltage_converter_.is_err();

        // Apply inline phase remapping (logical ABC -> physical channels)
        const int16_t* abc = voltage_converter_.get_phase_duty();
        abc_voltages_remapped_.a = abc[idx0_];
        abc_voltages_remapped_.b = abc[idx1_];
        abc_voltages_remapped_.c = abc[idx2_];

        return abc_voltages_remapped_.arr;
    }

    /// Process current path: ABC currents -> AB currents
    /// Physical channels are remapped to logical ABC, then converted to AB
    /// @param physical Physical channel currents [Ch0, Ch1, Ch2] in Q15 format (INT16_MIN = sensor missing)
    Vec2D tick_current(const int16_t physical[3]) {
        // Apply high duty cycle masking: if PWM duty > 30720, invalidate corresponding current sensor
        // Apply inline inverse phase remapping (physical channels -> logical ABC)
        constexpr int16_t DUTY_THRESHOLD = 30720;  // ~93.8% duty cycle
        Vec3D logical;
        logical[idx0_] = (abc_voltages_remapped_.a > DUTY_THRESHOLD) ? INT16_MIN : physical[0];
        logical[idx1_] = (abc_voltages_remapped_.b > DUTY_THRESHOLD) ? INT16_MIN : physical[1];
        logical[idx2_] = (abc_voltages_remapped_.c > DUTY_THRESHOLD) ? INT16_MIN : physical[2];

        // Convert logical ABC to AB using current converter
        current_converter_.tick(logical, motor_type_);
        err_current_ = current_converter_.is_err();
        return current_converter_.get_ab();
    }

    /// Check if voltage path error occurred
    /// @return 1 if error, 0 if no error
    uint8_t is_voltage_err() const { return err_voltage_; }

    /// Check if current path error occurred
    /// @return 1 if error, 0 if no error
    uint8_t is_current_err() const { return err_current_; }

    /// Check if any error occurred
    /// @return 1 if any error, 0 if no errors
    uint8_t is_err() const { return err_voltage_ | err_current_; }

    /// Get pointer to remapped ABC voltages (final output for PWM)
    /// @return Pointer to 3-element array [A, B, C] in Q15 format
    const int16_t* get_voltages() const { return abc_voltages_remapped_.arr; }

    /// Get AB currents as Vec2D (alpha-beta)
    /// @return Vec2D with sin=alpha, cos=beta in Q15 format
    inline Vec2D get_currents() const { return current_converter_.get_ab(); }

    /// Get filtered phase A current value (transit method)
    /// @return Filtered phase A current in Q15 format
    inline int16_t get_filtered_a() const { return current_converter_.get_filtered_a(); }

    /// Get filtered phase B current value (transit method)
    /// @return Filtered phase B current in Q15 format
    inline int16_t get_filtered_b() const { return current_converter_.get_filtered_b(); }

    /// Get filtered phase C current value (transit method)
    /// @return Filtered phase C current in Q15 format
    inline int16_t get_filtered_c() const { return current_converter_.get_filtered_c(); }
};
