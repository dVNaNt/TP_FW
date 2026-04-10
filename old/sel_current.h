#pragma once

#include "../../common.h"

// Forward declaration
struct Feedback;

// Fast integer-only current module (ABC -> AB)
// - Header-only C++ implementation
// - Validated struct wrapper with error flags
// - Supports missing sensors via INT16_MIN sentinel + recombination

// ============================================================================
// ABC to AB current converter with sensor recombination
// - INT16_MIN => sensor missing (will be recombined)
// - BLDC: validates ia + ib + ic == 0
// - STEP: validates ic == ia + ib
// - Supports 2 or 3 sensors (recombines missing phase)
// - Error if < 2 sensors or invalid motor type
// ============================================================================

struct ABC2AB {
private:
    // Output cache
    Vec2D ab_out_;  // Output Vec2D: sin=alpha, cos=beta in Q15 format

    // LPF filters for phase currents (used for balance validation)
    FilterEMA lpf_;  // LPF filter for phase balance


    // Configuration
    int32_t epsilon_abs_;  // Absolute balance tolerance in Q15 format

    // State
    bool err_;  // Error flag: 1=error occurred, 0=no error

    // LPF filter alpha constant for all filters
    static constexpr uint8_t LPF_ALPHA = 42;  // Filter coefficient (0-255)

public:
    /// Constructor: initializes current converter with motor type and balance tolerance
    /// @param parent Reference to parent Feedback instance
    /// @param epsilon Balance tolerance in Q15 format. If |balance| > epsilon => error flag set
    ABC2AB(int16_t epsilon = 0)
        : 
          epsilon_abs_(0)  // Will be calculated in set_epsilon()
          ,
          lpf_(0, LPF_ALPHA)  // Initialize LPF for phase A
          ,
          ab_out_{0, 0}  // Initialize output Vec2D to zeros
          ,
          err_(false) {        // Clear error flag
        set_epsilon(epsilon);  // Calculate absolute epsilon value
    }

    /// Set balance tolerance for validation
    /// @param epsilon Balance tolerance in Q15 format (can be negative, will use absolute value)
    void set_epsilon(int16_t epsilon) {
        epsilon_abs_ = abs((int32_t)epsilon);  // Store absolute value of epsilon
    }

    /// Main calculation: convert ABC currents to AB with sensor recombination
    /// @param abc ABC currents array [A, B, C] in Q15 format (INT16_MIN = sensor missing)
    /// @param motor_type Motor type (1=DC, 4=Stepper, 6=BLDC)
    void tick(Vec3D abc, MotorType mt) {
        const bool valid_a = (abc.a != INT16_MIN);
        const bool valid_b = (abc.b != INT16_MIN);
        const bool valid_c = (abc.c != INT16_MIN);

        const int32_t present_cnt = (int32_t)valid_a + (int32_t)valid_b + (int32_t)valid_c;

        // Validate motor type is supported or no current sense or we have only C current for DC or we have only 1 current for
        // BLDC/STEP (short exit)
        if (!(valid_a || valid_b) or (mt != MotorType_DC and present_cnt == 1)) {
            ab_out_ = Vec2D(INT16_MIN, INT16_MIN);
            err_ = true;
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
        switch (mt) {
            case MotorType_DC:
                alpha = (ib - ia) >> 1;  // if we have only one sensor it will be recongured so less math branches
                beta = 0;                // Beta = 0 for DC motor
                break;
            case MotorType_STEP:
                alpha = ia;
                beta = ib;
                break;
            case MotorType_BLDC:
                // Perform Clarke Transform to convert ABC to alpha-beta
                constexpr static int32_t INV_SQRT3_Q15 = 18919;  // 1/sqrt(3) in Q15 format (floor(0.577350269 * 2^15) = 18918.6)
                alpha = ia;
                // Calculate beta: I_beta = (I_B - I_C) *  1/sqrt(3)
                beta = clamp_i32(((ib - ic) * INV_SQRT3_Q15) >> 15, INT16_MAX);  // Shift right to convert to Q15
                break;
            default:
                ab_out_ = Vec2D(INT16_MIN, INT16_MIN);
                err_ = true;
                return;  // Exit early on invalid motor type
        }
        ab_out_ = {alpha, beta};
        err_ = false;  // Clear error flag

        if (present_cnt + (mt == MotorType_DC) >= 3) {  // Handle motor phase balance
            // Calibration mode uses stepper-style current sensing (direct alpha-beta mapping)
            // Filter updated only when enough sensors for motor balance (2 for DC and 3 for rest). Using real sum of sensors
            const int16_t bal = lpf_.tick_lpf(sum);
            if (bal > epsilon_abs_ or bal < -epsilon_abs_)  // Check if within tolerance
                err_ = true;                                // Clear error flag if balanced
        }
    }

    /// Check if an error occurred during last tick()
    /// @return 1 if error, 0 if no error
    inline uint8_t is_err() const { return err_; }

    /// Get AB output as Vec2D (alpha-beta currents)
    /// @return Vec2D with sin=alpha, cos=beta in Q15 format
    inline Vec2D get_ab() const { return ab_out_; }
};
