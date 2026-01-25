#pragma once

#include "../../common.h"
#include "../../math_integer/filters/filter1st.h"
#include "../types.h"

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
    Filter1st lpf_a_;  // LPF filter for phase A current
    Filter1st lpf_b_;  // LPF filter for phase B current
    Filter1st lpf_c_;  // LPF filter for phase C current

    // Reference to parent Feedback
    Feedback& feedback_;

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
    ABC2AB(Feedback& parent, int16_t epsilon = 0)
        : feedback_(parent),
          epsilon_abs_(0)  // Will be calculated in set_epsilon()
          ,
          lpf_a_(0, LPF_ALPHA)  // Initialize LPF for phase A
          ,
          lpf_b_(0, LPF_ALPHA)  // Initialize LPF for phase B
          ,
          lpf_c_(0, LPF_ALPHA)  // Initialize LPF for phase C
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

    // /// Main calculation: convert ABC currents to AB with sensor recombination
    // /// @param abc ABC currents array [A, B, C] in Q15 format (INT16_MIN = sensor missing)
    // /// @param motor_type Motor type (1=DC, 4=Stepper, 6=BLDC)
    // void tick(Vec3D abc, uint32_t motor_type) {
    //     const int16_t ia = abc[0];
    //     const int16_t ib = abc[1];
    //     const int16_t ic = abc[2];
    //     err_ = 1u;      // Start with error flag set (pessimistic)
    //     static const Vec2D err_out = Vec2D(INT16_MIN, INT16_MIN);

    //     const uint32_t mt = motor_type;  // Cache motor type for efficiency
    //     // Validate motor type is supported
    //     if (mt != MotorType_DC || mt != MotorType_BLDC || mt != MotorType_STEP) {
    //         ab_out_ = err_out;
    //         return;  // Exit early on invalid motor type
    //     }

    //     // Build bitmask of present sensors (INT16_MIN = missing)
    //     uint32_t present = 0u;  // Initialize presence bitmask
    //     if (abc.a != INT16_MIN)
    //         present |= 1u;  // Bit 0: phase A present
    //     if (abc.b != INT16_MIN)
    //         present |= 2u;  // Bit 1: phase B present
    //     if (abc.c != INT16_MIN)
    //         present |= 4u;  // Bit 2: phase C present

    //     // Count number of present sensors
    //     const uint32_t present_cnt =
    //         (uint32_t)((present & 1u) != 0u) +
    //         (uint32_t)((present & 2u) != 0u) +
    //         (uint32_t)((present & 4u) != 0u);

    //     // Handle DC motor: calculate current from available sensors
    //     if (mt == MotorType_DC) {
    //         int16_t val;
    //         if (present_cnt == 0u) {  // No sensors available
    //             ab_out_ = err_out;
    //             return;               // Exit with error flag set
    //         }

    //         // If both sensors present, calculate (a - b) / 2 and validate difference
    //         if ((present & 3u) == 3u) {  // Both A and B present
    //             const int32_t diff = -((int32_t)abc.a - (int32_t)abc.b);  // Calculate difference
    //             const int32_t sum = (int32_t)abc.a + (int32_t)abc.b ;   // Calculate sum for validation
    //             val = diff / 2;                 // (a - b) / 2 (cant make overflow)
    //             // Check if sensors differ too much: |a - b| > epsilon
    //             // if (abs_i32(sum) > epsilon_abs_) {
    //             //     return;  // Exit with error flag set
    //             // }
    //         } else {
    //             // Single sensor: use it directly (other assumed zero)
    //             val = (present & 1u) ? abc.a : - abc.b;     // Or B, or C
    //         }

    //         ab_out_.alpha = val;  // Alpha = calculated current
    //         ab_out_.beta = 0;     // Beta = 0 for DC motor
    //         err_ = 0u;          // Clear error flag (DC motor successful)
    //         return;             // Exit after DC motor handling
    //     }

    //     // BLDC and STEP motors require at least 2 sensors for recombination
    //     if (present_cnt < 2u) {  // Less than 2 sensors available
    //         ab_out_ = err_out;
    //         return;              // Exit with error flag set
    //     }

    //     int32_t a = (int32_t)ia;  // Convert phase A to 32-bit
    //     int32_t b = (int32_t)ib;  // Convert phase B to 32-bit
    //     int32_t c = (int32_t)ic;  // Convert phase C to 32-bit

    //     // Recombine missing phase using Kirchhoff's Current Law (ia + ib + ic = 0)
    //     if (!(present & 1u)) {          // Phase A missing
    //         a = -((int32_t)ib + (int32_t)ic);  // ia = -(ib + ic)
    //         a = sat_i32_to_i16(a);
    //     } else if (!(present & 2u)) {   // Phase B missing
    //         b = -((int32_t)ia + (int32_t)ic);  // ib = -(ia + ic)
    //         b = sat_i32_to_i16(b);
    //     } else if (!(present & 4u)) {   // Phase C missing
    //         c = -((int32_t)ia + (int32_t)ib);  // ic = -(ia + ib)
    //         c = sat_i32_to_i16(c);
    //     }

    //     // Update LPF filters with recombined values (always tick all 3 filters)
    //     lpf_a_.tick_lpf(a);  // Filter phase A current
    //     lpf_b_.tick_lpf(b);  // Filter phase B current
    //     lpf_c_.tick_lpf(c);  // Filter phase C current

    //     // Handle BLDC motor: balance constraint ia + ib + ic == 0
    //     if (mt == MotorType_BLDC) {
    //         // Perform Clarke Transform to convert ABC to alpha-beta
    //         clarke_transform(a, b, c, &ab_out_.alpha, &ab_out_.beta);
    //     }
    //     // Handle Stepper motor and Calibration mode: balance constraint ic == ia + ib
    //     // Calibration mode uses stepper-style current sensing (direct alpha-beta mapping)
    //     else if (mt == MotorType_STEP) {
    //         // For stepper: coils A and B map directly to alpha and beta
    //         ab_out_.a = a;  // Alpha = coil A current
    //         ab_out_.b = b;  // Beta = coil B current
    //     }

    //     // Validate balance using filtered values if all 3 sensors are present
    //     if (present_cnt == 3u) {  // All sensors available
    //         // Use filtered values for balance check: ic == ia + ib
    //         const int32_t bal = (int32_t)lpf_c_.get_output() +
    //                             (int32_t)lpf_a_.get_output() + (int32_t)lpf_b_.get_output();
    //         if (abs_i32(bal) <= epsilon_abs_) {  // Check if within tolerance
    //             err_ = 0u;                       // Clear error flag if balanced
    //         }
    //         // else: current leakage detected
    //     } else {        // Only 2 sensors (recombined 3rd)
    //         err_ = 0u;  // Clear error flag (recombination successful)
    //     }

    //     // Should never reach here due to motor type validation, but keep safe
    // }

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
        if (!(mt == MotorType_DC || mt == MotorType_BLDC || mt == MotorType_STEP) or !(valid_a || valid_b) or
            (mt != MotorType_DC and present_cnt == 1)) {
            ab_out_ = Vec2D(INT16_MIN, INT16_MIN);
            err_ = true;
            return;  // Exit early on invalid motor type
        }
 
        // making invalid currents as zero to simplify balance math
        int32_t ia = (abc.a != INT16_MIN) ? abc.a : 0;
        int32_t ib = (abc.b != INT16_MIN) ? abc.b : 0;
        int32_t ic = (abc.c != INT16_MIN) ? abc.c : 0;

        if (present_cnt < 3) { // speedup logic if all 3 sensors present
            const int32_t sum = clamp_i32(ia + ib + ic, INT16_MAX);
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
                alpha = (ib - ia) / 2;  // if we have only one sensor it will be recongured so less math branches
                beta = 0;               // Beta = 0 for DC motor
                break;
            case MotorType_STEP:
                alpha = ia;
                beta = ib;
                break;
            default:  // MotorType_BLDC
                // Perform Clarke Transform to convert ABC to alpha-beta
                constexpr static int32_t INV_SQRT3_Q15 = 18919;  // 1/sqrt(3) in Q15 format (floor(0.577350269 * 2^15) = 18918.6)
                alpha = ia;
                // Calculate beta: I_beta = (I_B - I_C) *  1/sqrt(3)
                beta = clamp_i32(((ib - ic) * INV_SQRT3_Q15) >> 15, INT16_MAX);  // Shift right to convert to Q15
                break;
        }
        ab_out_ = {alpha, beta};
        err_ = false;  // Clear error flag

        // Update LPF filters with recombined values (always tick all 3 filters)
        lpf_a_.tick_lpf(ia);  // Filter phase A current
        lpf_b_.tick_lpf(ib);  // Filter phase B current
        lpf_c_.tick_lpf(ic);  // Filter phase C current

        if (present_cnt + (mt == MotorType_DC) >= 3) {
            // Handle motor phase balance
            // Calibration mode uses stepper-style current sensing (direct alpha-beta mapping)
            const int32_t bal = (int32_t)lpf_c_.get_output() + (int32_t)lpf_a_.get_output() + (int32_t)lpf_b_.get_output();
            if (abs(bal) > epsilon_abs_) {  // Check if within tolerance
                err_ = true;                // Clear error flag if balanced
            }
        }
    }

    /// Check if an error occurred during last tick()
    /// @return 1 if error, 0 if no error
    inline uint8_t is_err() const { return err_; }

    /// Get AB output as Vec2D (alpha-beta currents)
    /// @return Vec2D with sin=alpha, cos=beta in Q15 format
    inline Vec2D get_ab() const { return ab_out_; }

    /// Get filtered phase A current value
    /// @return Filtered phase A current in Q15 format
    inline int16_t get_filtered_a() const { return lpf_a_.get_output(); }

    /// Get filtered phase B current value
    /// @return Filtered phase B current in Q15 format
    inline int16_t get_filtered_b() const { return lpf_b_.get_output(); }

    /// Get filtered phase C current value
    /// @return Filtered phase C current in Q15 format
    inline int16_t get_filtered_c() const { return lpf_c_.get_output(); }
};
