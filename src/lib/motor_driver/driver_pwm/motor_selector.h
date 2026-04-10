#pragma once

#include "../../common.h"

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
    // --- State variables for Current processing (formerly ABC2AB) ---
    FilterEMA lpf_;                           // LPF filter for phase balance
    static constexpr uint8_t LPF_ALPHA = 42;  // Filter coefficient (0-255)

    int16_t epsilon_abs_;  // Absolute balance tolerance in Q15 format
    Vec3D abc_voltages_;   // ABC voltages after phase remapping

    Vec2D ab_currents_;  // Output Vec2D: sin=alpha, cos=beta in Q15 format

    MotorType motor_type_;  // Motor type: 1=DC, 4=Stepper, 6=BLDC
    // Integrated phase remapping indices (precomputed for zero overhead)
    uint8_t idx0_;  // Index for output channel 0
    uint8_t idx1_;  // Index for output channel 1
    uint8_t idx2_;  // Index for output channel 2

    PhasePattern phase_pattern_;  // Phase connection pattern

    // Error flags
    uint8_t err_voltage_;  // Error flag for voltage path
    uint8_t err_current_;  // Error flag for current path

    // ============================================================================
    // Internal method: Voltage AB to ABC converter with SVPWM (formerly AB2ABC::tick)
    // ============================================================================
    Vec3D ab2abc_tick(Vec2D input, uint32_t motor_type, uint8_t& error) {
        int32_t voltg_a = input.alpha;
        int32_t voltg_b = input.beta;
        int32_t voltg_c;  // Temporary 32-bit voltages for calculations
        error = false;
        const int32_t neg_voltg_a_div2 = -voltg_a >> 1;  // precalculated value for math

        if (motor_type == MotorType_BLDC) {  // Handle BLDC motor type (3-phase with Clarke Transform)
            // sqrt(3)/2 in Q16.16 fixed-point format (floor(0.8660254037844386 * 65536) = 56755)
            constexpr static int32_t VOLTAGE_SQRT3DIV2_Q16_16 = 56755;
            // Calculate beta * sqrt(3)/2 using Q16.16 fixed-point multiplication
            int32_t beta_sqrt3_div2 = (VOLTAGE_SQRT3DIV2_Q16_16 * voltg_b) >> 16;

            // Phase A = V_alpha by default
            voltg_b = neg_voltg_a_div2 + beta_sqrt3_div2;  // Phase B = -V_alpha/2 + V_beta*sqrt(3)/2
            voltg_c = neg_voltg_a_div2 - beta_sqrt3_div2;  // Phase C = -V_alpha/2 - V_beta*sqrt(3)/2
        } else if (motor_type == MotorType_STEP) {         // Handle Stepper motor type (direct 2-phase mapping)
            // Phase A = alpha input (coil A) by default
            // Phase B = beta input (coil B) by default
            voltg_c = 0;                          // Phase C: unused for stepper
        } else if (motor_type == MotorType_DC) {  // Handle DC motor type (differential drive on two channels)
            // Check for disabled state (INT16_MIN) or brake (0)
            voltg_c = INT16_MIN;  // Phase C: unused for DC motor
            if (voltg_a == INT16_MIN || voltg_a == 0) {
                voltg_b = voltg_a;  // Set all outputs to reference
            } else {
                constexpr static int32_t MIDPOINT = (int32_t)(INT16_MAX / 2);  // Calculate PWM midpoint
                voltg_a = (int16_t)(MIDPOINT - neg_voltg_a_div2);              // Phase A: midpoint + half duty (forward)
                voltg_b = (int16_t)(MIDPOINT + neg_voltg_a_div2);              // Phase B: midpoint - half duty (reverse)
            }
            return Vec3D((int16_t)(voltg_a), (int16_t)(voltg_b), (int16_t)(voltg_c));
        } else {  // Handle unexpected invalid motor type
            error = true;
            return Vec3D(INT16_MIN, INT16_MIN, INT16_MIN);  // Store phase C result
        }

        // Find minimum and maximum voltage among all three phases
        int32_t min_ab, max_ab;
        if (voltg_a < voltg_b) {
            min_ab = voltg_a;
            max_ab = voltg_b;
        } else {
            min_ab = voltg_b;
            max_ab = voltg_a;
        }
        int32_t voltg_min = (min_ab < voltg_c) ? min_ab : voltg_c;
        int32_t voltg_max = (max_ab > voltg_c) ? max_ab : voltg_c;

        int32_t voltg_offset;                              // Offset for SVPWM centering
        int32_t voltg_full_scale = voltg_max - voltg_min;  // Calculate peak-to-peak range

        // Check if voltage range exceeds available supply
        if (voltg_full_scale < INT16_MAX) {
            // Voltage fits - use SVPWM centering for optimal utilization
            voltg_offset = (INT16_MAX - voltg_max - voltg_min) >> 1;  // Center the voltage vector
            voltg_a += voltg_offset;                                  // Apply offset to phase A
            voltg_b += voltg_offset;                                  // Apply offset to phase B
            voltg_c += voltg_offset;                                  // Apply offset to phase C
        } else {
            // Scale down voltages to fit within max_output
            uint32_t scale = ((uint32_t)INT16_MAX << 15) / (uint32_t)voltg_full_scale;
            voltg_a = ((voltg_a - voltg_min) * scale) >> 15;
            voltg_b = ((voltg_b - voltg_min) * scale) >> 15;
            voltg_c = ((voltg_c - voltg_min) * scale) >> 15;  // Offset to shift into positive range
        }

        // Convert 32-bit results back to 16-bit outputs
        return Vec3D((int16_t)(voltg_a), (int16_t)(voltg_b), (int16_t)(voltg_c));  // Store phase C result
    }

    // ============================================================================
    // Internal method: ABC to AB current converter with sensor recombination (formerly ABC2AB::tick)
    // ============================================================================
       // ============================================================================
    // Internal method: ABC to AB current converter with sensor recombination (formerly ABC2AB::tick)
    // ============================================================================
    void abc2ab_tick(Vec3D abc, MotorType mt) {
        const bool valid_a = (abc.a != INT16_MIN);
        const bool valid_b = (abc.b != INT16_MIN);
        const bool valid_c = (abc.c != INT16_MIN);

        const int32_t present_cnt = (int32_t)valid_a + (int32_t)valid_b + (int32_t)valid_c;

        // Validate motor type is supported or no current sense or we have only C current for DC or we have only 1 current for
        // BLDC/STEP (short exit)
        if (!(valid_a || valid_b) or (mt != MotorType_DC and present_cnt == 1)) {
            ab_currents_ = Vec2D(INT16_MIN, INT16_MIN);
            err_current_ = true;
            return;  // Exit early on invalid motor type
        }

        // making invalid currents as zero to simplify balance math
        int32_t ia = (abc.a != INT16_MIN) ? abc.a : 0;
        int32_t ib = (abc.b != INT16_MIN) ? abc.b : 0;
        int32_t ic = (abc.c != INT16_MIN) ? abc.c : 0;

        const int32_t sum = clamp_i32(ia + ib + ic, INT16_MAX);
        if (present_cnt < 3) {  // speedup logic if all 3 sensors present
            // Recombine missing phase using Kirchhoff's Current Law (ia + ib + ic = 0)
            if (!valid_a)
                ia = -sum;  // Phase A missing: ia = -(ib + ic)
            else if (!valid_b)
                ib = -sum;  // Phase B missing: ib = -(ia + ic)
            else if (!valid_c)
                ic = -sum;  // Phase C missing: ic = -(ia + ib)
        }

        int16_t alpha;
        int16_t beta;

        // Handle DC motor: calculate current from available sensors
        if (mt == MotorType_BLDC) {
            // Perform Clarke Transform to convert ABC to alpha-beta
            constexpr static int32_t INV_SQRT3_Q15 = 18919;  // 1/sqrt(3) in Q15 format (floor(0.577350269 * 2^15) = 18918.6)
            alpha = ia;
            // Calculate beta: I_beta = (I_B - I_C) *  1/sqrt(3)
            beta = clamp_i32(((ib - ic) * INV_SQRT3_Q15) >> 15, INT16_MAX);  // Shift right to convert to Q15
        } else if (mt == MotorType_DC) {
            alpha = (ib - ia) >> 1;  // if we have only one sensor it will be recongured so less math branches
            beta = 0;                // Beta = 0 for DC motor
        } else if (mt == MotorType_STEP) {
            alpha = ia;
            beta = ib;
        } else {
            ab_currents_ = Vec2D(INT16_MIN, INT16_MIN);
            err_current_ = true;
            return;  // Exit early on invalid motor type
        }

        ab_currents_ = {alpha, beta};
        err_current_ = false;  // Clear error flag

        if (present_cnt + (mt == MotorType_DC) >= 3) {  // Handle motor phase balance
            // Calibration mode uses stepper-style current sensing (direct alpha-beta mapping)
            // Filter updated only when enough sensors for motor balance (2 for DC and 3 for rest). Using real sum of sensors
            const int16_t bal = lpf_.tick_lpf(sum);
            if (bal > epsilon_abs_ or bal < -epsilon_abs_)  // Check if within tolerance
                err_current_ = true;                        // Clear error flag if balanced
        }
    }

public:
    /// Constructor: initializes motor selector with motor type and phase pattern
    /// @param motor_type Motor type (1=DC, 4=Stepper, 6=BLDC)
    /// @param phase_pattern Phase connection pattern (ABC, ACB, CBA)
    /// @param control_mode Control mode (VoltageAB or CurrentAB)
    /// @param duty_permille Maximum duty cycle in permille (0-995 = 0.0%-99.5%)
    /// @param current_epsilon Balance tolerance for current in Q15 format
    MotorSelector(MotorType motor_type = MotorType_STEP,
                  PhasePattern phase_pattern = PhasePattern_ABC,
                  uint16_t duty_permille = 995u,
                  int16_t current_epsilon = 256)
        : motor_type_(motor_type),
          phase_pattern_(phase_pattern),
          idx0_(0),
          idx1_(1),
          idx2_(2),
          abc_voltages_(0, 0, 0),
          err_voltage_(0u),
          err_current_(0u),
          ab_currents_{0, 0},
          lpf_(0, LPF_ALPHA),
          epsilon_abs_(0) {
        set_phase_pattern(phase_pattern);
        set_current_epsilon(current_epsilon);
    }

    /// Set motor type for both current and voltage converters
    /// @param motor_type Motor type (1=DC, 4=Stepper, 6=BLDC)
    inline void set_motor_type(MotorType motor_type) { motor_type_ = motor_type; }

    /// Get current motor type
    /// @return Motor type (1=DC, 4=Stepper, 6=BLDC)
    inline MotorType get_motor_type() const { return motor_type_; }

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

    inline PhasePattern get_phase_pattern() { return phase_pattern_; }

    /// Set balance tolerance for current converter
    /// @param epsilon Balance tolerance in Q15 format
    inline void set_current_epsilon(int16_t epsilon) { epsilon_abs_ = abs((int32_t)epsilon); }

    /// Process voltage path: AB voltages -> ABC voltages with phase remapping
    /// @param voltage Vec2D voltage (sin=alpha, cos=beta) in Q15 format
    const Vec3D tick_voltage(Vec2D voltage) {
        // Convert AB to ABC using internal voltage converter
        Vec3D abc = ab2abc_tick(voltage, motor_type_, err_voltage_);

        // Apply inline phase remapping (logical ABC -> physical channels)
        abc_voltages_.a = abc[idx0_];
        abc_voltages_.b = abc[idx1_];
        abc_voltages_.c = abc[idx2_];

        return abc_voltages_;
    }

    /// Process current path: ABC currents -> AB currents
    /// Physical channels are remapped to logical ABC, then converted to AB
    /// @param physical Physical channel currents [Ch0, Ch1, Ch2] in Q15 format (INT16_MIN = sensor missing)
    Vec2D tick_current(const int16_t physical[3]) {
        // Apply high duty cycle masking: if PWM duty > 30720, invalidate corresponding current sensor
        // Apply inline inverse phase remapping (physical channels -> logical ABC)
        constexpr int16_t DUTY_THRESHOLD = 30720;  // ~93.8% duty cycle
        Vec3D logical;
        logical[idx0_] = (abc_voltages_.a > DUTY_THRESHOLD) ? INT16_MIN : physical[0];
        logical[idx1_] = (abc_voltages_.b > DUTY_THRESHOLD) ? INT16_MIN : physical[1];
        logical[idx2_] = (abc_voltages_.c > DUTY_THRESHOLD) ? INT16_MIN : physical[2];

        // Convert logical ABC to AB using internal current converter
        abc2ab_tick(logical, motor_type_);

        return ab_currents_;
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
    inline Vec3D get_voltages() const { return abc_voltages_; }

    /// Get AB currents as Vec2D (alpha-beta)
    /// @return Vec2D with sin=alpha, cos=beta in Q15 format
    inline Vec2D get_currents() const { return ab_currents_; }
};