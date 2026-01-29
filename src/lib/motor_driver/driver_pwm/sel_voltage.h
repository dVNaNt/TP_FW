#pragma once

#include "../../common.h"
#include "../types.h"
#include "../../feedback/analog/supply_voltage.h"

// Forward declaration
struct Feedback;

// Fast integer-only voltage module (AB -> ABC)
// - Header-only C++ implementation
// - SVPWM centering + optional scaling if range exceeds available duty
// - Validated struct wrapper with error flags and duty limit


// ============================================================================
// Voltage AB to ABC converter with SVPWM
// ============================================================================

struct AB2ABC  {
private:
    // Reference to parent Feedback
    Feedback& feedback_;
    
    // Configuration
    int16_t max_duty_q15_;  // Maximum duty cycle in Q15 format [0..INT16_MAX]

    // Output cache
    Vec3D abc_duty_;  // Output: [A, B, C] PWM duty cycles in Q15 format

    

    // State
    uint8_t err_;  // Error flag: 1=error occurred, 0=no error
    
public:
    /// Constructor: initializes converter with duty cycle limit
    /// @param parent Reference to parent Feedback instance
    /// @param duty_permille Maximum duty cycle in permille (0-995 = 0.0%-99.5%)
    AB2ABC (Feedback& parent, uint16_t duty_permille = 995u)
        : feedback_(parent),
          max_duty_q15_(0)  // Will be calculated in set_duty()
          ,
          abc_duty_(0, 0, 0)  // Initialize output to zeros
          ,
          err_(0u) {  // Clear error flag
        set_duty(duty_permille);  // Calculate and set max duty in Q15 format
    }

    /// Set maximum duty cycle limit
    /// @param duty_permille Duty cycle in permille (0-995 = 0.0%-99.5%)
    void set_duty(uint16_t duty_permille) {
        if (duty_permille > 1000u) {  // Clamp to maximum safe value
            duty_permille = 1000u;    // Limit to 99.5% duty cycle
        }
        // Convert permille to Q15 format: (INT16_MAX * duty_permille) / 1000
        max_duty_q15_ = static_cast<int16_t>((static_cast<int32_t>(INT16_MAX) * static_cast<int32_t>(duty_permille)) / 1000);
    }

    /// Main calculation: convert AB voltages to ABC with SVPWM
    /// @param input Vec2D with sin=alpha, cos=beta in Q15 format [-32768, 32767]
    /// @param motor_type Motor type (1=DC, 4=Stepper, 6=BLDC)
    void tick(Vec2D input, uint32_t motor_type) {
        int16_t voltg_alpha = input.alpha;  // Alpha component of voltage
        int16_t voltg_beta = input.beta;    // Beta component of voltage
        err_ = 1u;  // Set error flag at start of calculation
        
        // Validate motor type is one of the supported types
        // Validate that alpha²+beta² is within normalized range (≤ 32767² * 2)
        if (!is_valid_motor_type(motor_type) or !check_alpha_beta_norm(voltg_alpha, voltg_beta)) {
            set_error_state();  // Set error and output zeros
            return;             // Exit early on invalid motor type
        }

        // Perform AB to ABC conversion with SVPWM algorithm
        calculate_abc(voltg_alpha, voltg_beta, motor_type, static_cast<int32_t>(max_duty_q15_), &abc_duty_.a, &abc_duty_.b,
                      &abc_duty_.c);

        // Verify that no output channel exceeds the configured duty limit
        if (!(abc_duty_.a > max_duty_q15_ || abc_duty_.b > max_duty_q15_ || abc_duty_.c > max_duty_q15_)) {
            err_ = 0u;  // Set no error flag if everything fine
        }
    }

    /// Check if an error occurred during last tick()
    /// @return 1 if error, 0 if no error
    uint8_t is_err() const { return err_; }

    /// Get pointer to ABC output array
    /// @return Pointer to 3-element array [A, B, C] in Q15 format
    const int16_t* get_phase_duty() const { return abc_duty_.arr; }

private:

    // sqrt(3)/2 in Q16.16 fixed-point format (floor(0.8660254037844386 * 65536) = 56755)
    constexpr static int32_t VOLTAGE_SQRT3DIV2_Q16_16 = 56755;



    /// Internal helper: calculates ABC from AB with configurable max duty
    /// Implements SVPWM (Space Vector PWM) with automatic scaling and centering
    /// @param voltg_alpha Alpha component in Q15 format
    /// @param voltg_beta Beta component in Q15 format
    /// @param motor_type Motor type (1=DC, 4=Stepper, 6=BLDC)
    /// @param max_output Maximum allowed output value
    /// @param out_a Output pointer for phase A duty cycle
    /// @param out_b Output pointer for phase B duty cycle
    /// @param out_c Output pointer for phase C duty cycle
    inline void calculate_abc(int16_t voltg_alpha,
                       int16_t voltg_beta,
                       uint32_t motor_type,
                       int32_t max_output,
                       int16_t* out_a,
                       int16_t* out_b,
                       int16_t* out_c) {
        int32_t voltg_a, voltg_b, voltg_c;  // Temporary 32-bit voltages for calculations

        // Handle DC motor type (differential drive on two channels)
        if (motor_type == MotorType_DC) {
            int16_t voltg_ref = voltg_alpha;  // Only alpha component used for DC
            // Check for disabled state (INT16_MIN) or brake (0)
            if (voltg_alpha == INT16_MIN || voltg_alpha == 0) {
                *out_a = *out_b = *out_c = voltg_alpha;  // Set all outputs to reference
                return;                                // Exit early
            }

            int16_t duty = voltg_alpha >> 1;                                   // Calculate duty as half of reference
            const int16_t MIDPOINT = static_cast<int16_t>(max_output >> 1);  // Calculate PWM midpoint
            *out_a = static_cast<int16_t>(MIDPOINT + duty);                  // Phase A: midpoint + duty (forward)
            *out_b = static_cast<int16_t>(MIDPOINT - duty);                  // Phase B: midpoint - duty (reverse)
            *out_c = INT16_MIN;                                              // Phase C: unused for DC motor
            return;                                                          // Exit after DC motor handling
        }
        // Handle BLDC motor type (3-phase with Clarke Transform)
        // Handle Calibration mode: voltage commutation like BLDC
        else if (motor_type == MotorType_BLDC) {
            int32_t alpha32 = static_cast<int32_t>(voltg_alpha);  // Convert alpha to 32-bit for calculations
            int32_t beta32 = static_cast<int32_t>(voltg_beta);    // Convert beta to 32-bit for calculations
            // Calculate beta * sqrt(3)/2 using Q16.16 fixed-point multiplication
            int32_t beta_sqrt3_div2 = (VOLTAGE_SQRT3DIV2_Q16_16 * beta32) >> 16;

            voltg_a = alpha32;                            // Phase A = V_alpha
            voltg_b = -(alpha32 >> 1) + beta_sqrt3_div2;  // Phase B = -V_alpha/2 + V_beta*sqrt(3)/2
            voltg_c = -(alpha32 >> 1) - beta_sqrt3_div2;  // Phase C = -V_alpha/2 - V_beta*sqrt(3)/2
        }
        // Handle Stepper motor type (direct 2-phase mapping)
        else if (motor_type == 4U) {
            voltg_a = (int32_t)voltg_alpha;  // Phase A = alpha input (coil A)
            voltg_b = (int32_t)voltg_beta;   // Phase B = beta input (coil B)
            voltg_c = 0;                     // Phase C: unused for stepper
        }
        // Handle invalid motor type
        else {
            *out_a = *out_b = *out_c = INT16_MIN;  // Output Hi-Z for safety
            return;                                // Exit early on invalid type
        }

        // Find minimum voltage among all three phases
        int32_t voltg_min = voltg_a < voltg_b ? voltg_a : voltg_b;  // Compare A and B
        voltg_min = voltg_min < voltg_c ? voltg_min : voltg_c;      // Compare result with C

        // Find maximum voltage among all three phases
        int32_t voltg_max = voltg_a > voltg_b ? voltg_a : voltg_b;  // Compare A and B
        voltg_max = voltg_max > voltg_c ? voltg_max : voltg_c;      // Compare result with C

        int32_t voltg_offset;                              // Offset for SVPWM centering
        int32_t voltg_full_scale = voltg_max - voltg_min;  // Calculate peak-to-peak range

        // Check if voltage range exceeds available supply
        if (voltg_full_scale > max_output) {
            // Scale down voltages to fit within max_output
            int32_t voltg_scale = (max_output << 15) / voltg_full_scale;  // Calculate Q15 scaling factor
            voltg_a = (voltg_a * voltg_scale) >> 15;                      // Scale phase A
            voltg_b = (voltg_b * voltg_scale) >> 15;                      // Scale phase B
            voltg_c = (voltg_c * voltg_scale) >> 15;                      // Scale phase C
            int32_t voltg_min_scaled = (voltg_min * voltg_scale) >> 15;   // Scale minimum value
            voltg_offset = -voltg_min_scaled;                             // Offset to shift into positive range
        } else {
            // Voltage fits - use SVPWM centering for optimal utilization
            voltg_offset = (max_output - voltg_max - voltg_min) >> 1;  // Center the voltage vector
        }

        // Apply offset if non-zero voltage is requested (not brake mode)
        if (voltg_full_scale != 0) {
            voltg_a += voltg_offset;  // Apply offset to phase A
            voltg_b += voltg_offset;  // Apply offset to phase B
            voltg_c += voltg_offset;  // Apply offset to phase C
        }

        // Convert 32-bit results back to 16-bit outputs
        *out_a = static_cast<int16_t>(voltg_a);  // Store phase A result
        *out_b = static_cast<int16_t>(voltg_b);  // Store phase B result
        *out_c = static_cast<int16_t>(voltg_c);  // Store phase C result
    }


    /// Set error state and set all outputs to Hi-Z (INT16_MIN)
    void set_error_state() {
        err_ = 1u;                 // Set error flag
        abc_duty_.a = INT16_MIN;    // Set phase A to Hi-Z
        abc_duty_.b = INT16_MIN;    // Set phase B to Hi-Z
        abc_duty_.c = INT16_MIN;    // Set phase C to Hi-Z
    }

    /// Check if motor type is one of the supported types
    /// @param mt Motor type to validate
    /// @return true if valid (DC, BLDC, Stepper, or Calibration), false otherwise
    static bool is_valid_motor_type(uint32_t mt) {
        return (mt == MotorType_DC || mt == MotorType_BLDC || mt == MotorType_STEP);
    }
    
    /// Check if alpha and beta values are within normalized range
    /// Validates that alpha²+beta² ≤ 32767² * 2 (normalized magnitude check)
    /// @param alpha_val Alpha component in Q15 format
    /// @param beta_val Beta component in Q15 format
    /// @return true if within valid range, false otherwise
    static bool check_alpha_beta_norm(int16_t alpha_val, int16_t beta_val) {
        // Calculate alpha² + beta² using 32-bit arithmetic to avoid overflow
        int32_t alpha_sq = (int32_t)alpha_val * alpha_val;  // alpha² in Q30 format
        int32_t beta_sq = (int32_t)beta_val * beta_val;     // beta² in Q30 format
        int32_t magnitude_sq = alpha_sq + beta_sq;          // Total magnitude squared
        
        // Maximum valid magnitude: 32767² * 2 = 2147352578
        constexpr int32_t MAX_MAGNITUDE_SQ = 2147352578;
        
        return magnitude_sq <= MAX_MAGNITUDE_SQ;  // Return true if within valid range
    }
};

