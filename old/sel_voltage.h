#pragma once

#include "../../common.h"
#include "../../feedback/analog/supply_voltage.h"
#include "../types.h"

// Forward declaration
struct Feedback;

// Fast integer-only voltage module (AB -> ABC)
// - Header-only C++ implementation
// - SVPWM centering + optional scaling if range exceeds available duty
// - Validated struct wrapper with error flags and duty limit

// ============================================================================
// Voltage AB to ABC converter with SVPWM
// ============================================================================

struct AB2ABC {
public:
    /// Main calculation: convert AB voltages to ABC with SVPWM
    /// @param input Vec2D with sin=alpha, cos=beta in Q15 format [-32768, 32767]
    /// @param motor_type Motor type (1=DC, 4=Stepper, 6=BLDC)
    Vec3D tick(Vec2D input, uint32_t motor_type, uint8_t& error) {
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
};
