#pragma once

#include "../common.h"
#include "types.h"

// Motor calibration status enum - accessible as SetupStatus::MotorCalibrationStatus
enum class MotorStatus : uint8_t {
    Success = 0,            // Not calibrating
    SoftWarmup = 1,         // Checking motor warmup and connection
    Settle = 2,             // Settling phase before measurement (900 ticks)
    SlowRotation = 3,       // Slow rotation for motor type detection
    ResistanceMeasure = 4,  // Measuring resistance
    InductanceMeasure = 5,  // Measuring inductance using HFI
    CalibError = 6,         // Calibration error
    NotCalibrated = 0xFF
};

// Motor and driver system parameters structure
// Contains electrical and mechanical constants for motor control
struct Motor {
private:
    // Motor type and configuration
    MotorType motor_type_;            // Type of motor (DC, STEP, BLDC, Invalid)
    PhasePattern phase_pattern_;      // Phase connection pattern (ABC, BCA, CAB)
    MotorStatus status_;             // Current driver status

    // Motor mechanical parameters
    int32_t pole_count_;  // Number of pole pairs
    int16_t direction_;   // Motor rotation direction (+1/-1)

    // Motor angles
    int32_t electric_angle_;  // Electric angle in internal units
    int32_t mechanic_angle_;  // Mechanical angle in internal units

    // Phase resistance [mOhm]
    int32_t total_resistance_mohm_;     // Total motor resistance in milliohms (calculated from phases)

    

    // Phase inductance [uH]
    int32_t phase_inductance_uh_[3];  // Phase inductance in microhenries
    
    // Default inductance value [uH] - used when inductance is not measured
    int32_t default_inductance_uh_;  // Default inductance in microhenries (3000 uH = 3 mH)

    // Operating limits
    int32_t current_limit_ma_;  // Current limit for calibration in milliamps
    int32_t max_voltage_mv_;    // Maximum voltage in millivolts

public:
int32_t phase_resistance_mohm[3] = {0,0,0};
    // Constructor with default values
    Motor()
        : motor_type_(MotorType_Invalid),
          phase_pattern_(PhasePattern_ABC),
          status_(MotorStatus::NotCalibrated),
          pole_count_(7),
          direction_(1),
          electric_angle_(0),
          mechanic_angle_(0),
          current_limit_ma_(600),
          max_voltage_mv_(24000),
          total_resistance_mohm_(0),
          default_inductance_uh_(3000) {  // Default 3 mH inductance
        // Initialize resistance and inductance arrays to zero
        for (uint8_t i = 0; i < 3; i++) {
            phase_inductance_uh_[i] = 0;
        }
    }

    // === Setters ===

    // Set motor type
    inline void set_motor_type(MotorType type) { motor_type_ = type; }
    inline void set_motor_type(int8_t type) { motor_type_ = (MotorType)type; }

    // Set phase pattern
    inline void set_phase_pattern(PhasePattern pattern) { phase_pattern_ = pattern; }

    // Set driver status
    inline void set_status(MotorStatus status) { status_ = status; }

    // Set pole count
    inline void set_pole_count(int32_t poles) { pole_count_ = poles; }

    // Set direction
    inline void set_direction(int16_t dir) { direction_ = dir; }

    // Set electric angle
    inline void set_electric_angle(int32_t angle) { electric_angle_ = angle; }

    // Set mechanical angle
    inline void set_mechanic_angle(int32_t angle) { mechanic_angle_ = angle; }


    // Set phase inductance for specific phase (0-2)
    void set_phase_inductance(uint8_t phase_idx, int32_t inductance_uh) {
        if (phase_idx >= 3)
            return;                                       // Validate index
        phase_inductance_uh_[phase_idx] = inductance_uh;  // Store inductance
    }

    // Set current limit for calibration
    inline void set_current_limit(int32_t current_ma) { current_limit_ma_ = current_ma; }

    // Set maximum voltage
    inline void set_max_voltage(int32_t voltage_mv) { max_voltage_mv_ = voltage_mv; }

    // === Getters ===

    // Get motor type
    inline MotorType get_motor_type() const { return motor_type_; }

    // Get phase pattern
    inline PhasePattern get_phase_pattern() const { return phase_pattern_; }

    // Get driver status
    inline MotorStatus get_status() const { return status_; }

    // Get pole count
    inline int32_t get_pole_count() const { return pole_count_; }

    // Get direction
    inline int16_t get_direction() const { return direction_; }

    // Get electric angle
    inline int32_t get_electric_angle() const { return electric_angle_; }

    // Get mechanical angle
    inline int32_t get_mechanic_angle() const { return mechanic_angle_; }


    // Get phase inductance for specific phase
    inline int32_t get_phase_inductance(uint8_t phase_idx) const { return (phase_idx < 3) ? phase_inductance_uh_[phase_idx] : 0; }
    
    // Get effective inductance (measured or default)
    // Returns measured inductance if available, otherwise returns default value
    inline int32_t get_effective_inductance() const {
        // Check if any phase has measured inductance
        if (phase_inductance_uh_[0] > 0) {
            return phase_inductance_uh_[0];  // Return measured value
        }
        return default_inductance_uh_;  // Return default 3 mH
    }
    
    // Set default inductance value
    inline void set_default_inductance(int32_t inductance_uh) { default_inductance_uh_ = inductance_uh; }

    // Get current limit for calibration
    inline int32_t get_current_limit() const { return current_limit_ma_; }

    // Get maximum voltage
    inline int32_t get_max_voltage() const { return max_voltage_mv_; }

    // Get total motor resistance (calculated from phases based on motor type)
    inline int32_t get_total_resistance() const { return total_resistance_mohm_; }

    inline void set_total_resistance(int32_t res) {
        total_resistance_mohm_ = res;
    }

    // // Uses statistical analysis (min/max/sum) instead of combinatorial checks
    // // to reduce Flash memory usage significantly on STM32G4.
    // void detect_type() {
    //     // 1. Load phase resistances into a local array for indexed access
    //     // This allows using loops instead of hardcoded variables, reducing instruction count.
    //     const int32_t r[3] = {phase_resistance_mohm_[0], phase_resistance_mohm_[1], phase_resistance_mohm_[2]};

    //     // 2. Analyze the readings in a single pass
    //     // We track validity, short circuits, min/max values, and the index of the maximum value.
    //     int32_t r_min = INT32_MAX;
    //     int32_t r_max = -1;
    //     int32_t r_sum_valid = 0;  // Sum of valid resistances
    //     int8_t valid_count = 0;
    //     int8_t max_index = 0;       // Index (0,1,2) of the largest resistance
    //     int8_t last_valid_idx = 0;  // Helper for DC detection
    //     bool short_detected = false;

    //     // Iterate through all 3 phases
    //     for (int i = 0; i < 3; i++) {
    //         const int32_t val = r[i];

    //         // Check for immediate short circuit (0 Ohm)
    //         if (val == 0) {
    //             short_detected = true;
    //         }

    //         // Validate resistance range (50 mOhm to 100 Ohm)
    //         if (val >= 50 && val <= 100000) {
    //             valid_count++;
    //             last_valid_idx = i;  // Store index for DC motor pattern logic
    //             r_sum_valid += val;  // Accumulate sum for Stepper check later

    //             // Update Min/Max statistics
    //             if (val < r_min)
    //                 r_min = val;
    //             if (val > r_max) {
    //                 r_max = val;
    //                 max_index = i;
    //             }
    //         }
    //     }

    //     // 3. Check for invalid states (Short circuits or ambiguous open phases)
    //     // Conditions: Any short detected OR invalid count is 0 (all open) OR 2 (ambiguous).
    //     if (short_detected || valid_count == 0 || valid_count == 2) {
    //         motor_type_ = MotorType_Invalid;
    //         phase_pattern_ = PhasePattern_CBA;  // Default safe pattern (CBA for BLDC)
    //         total_resistance_mohm_ = 0;  // Invalid motor - no resistance calculation
    //         return;
    //     }

    //     // 4. DC Motor Detection
    //     // Condition: Exactly one phase pair has valid resistance (others are open).
    //     if (valid_count == 1) {
    //         // motor_type_ = MotorType_DC;
    //         motor_type_ = MotorType_Invalid; // TODO: Fix
    //         // Map the single valid index to the pattern (0->ABC, 1->BCA, 2->CAB)
    //         phase_pattern_ = static_cast<PhasePattern>(last_valid_idx);
    //         // DC motor: total resistance equals the single valid phase resistance
    //         total_resistance_mohm_ = r[last_valid_idx];
    //         return;
    //     }

    //     // --- At this point, valid_count is guaranteed to be 3 ---

    //     // 5. BLDC Motor Detection
    //     // Logic: All phases are roughly equal. Deviation between Min and Max is small.
    //     // Calculate deviation percentage: ((Max - Min) / Max) * 100
    //     // Using integer math; multiply by 100 first to preserve precision.
    //     const int32_t deviation = (r_max > 0) ? ((r_max - r_min) * 100) / r_max : 100;

    //     if (deviation <= 25) {  // Threshold: 25%
    //         motor_type_ = MotorType_BLDC;
    //         phase_pattern_ = PhasePattern_CBA;  // Default BLDC pattern (CBA)
    //         // BLDC: R = (AB + BC + AC) / 6 = (r[0] + r[1] + r[2]) / 6
    //         // Each phase measurement is line-to-line, so total = sum / 6
    //         total_resistance_mohm_ = r_sum_valid / 6;
    //         return;
    //     }

    //     // 6. Stepper Motor Detection2
    //     // Logic: A Stepper (measured as 3-phase) typically shows two smaller coils and one larger sum.
    //     // Pattern: Two phases are roughly equal (Min ≈ Mid), and the third is their sum (Max ≈ Min + Mid).

    //     // Calculate the 'Middle' resistance value derived from the sum
    //     // Sum = Min + Mid + Max  =>  Mid = Sum - Min - Max
    //     const int32_t r_mid = r_sum_valid - r_min - r_max;

    //     // Threshold for stepper symmetry checks (20% of the maximum value)
    //     const int32_t threshold = (r_max * 20) / 100;

    //     // Check 1: Is the largest value roughly equal to the sum of the other two?
    //     // |(Min + Mid) - Max| <= Threshold
    //     const bool sum_match = std::abs((r_min + r_mid) - r_max) <= threshold;

    //     // Check 2: Are the two smaller values roughly equal to each other?
    //     // |Mid - Min| <= Threshold
    //     const bool symmetry_match = (r_mid - r_min) <= threshold;

    //     if (sum_match && symmetry_match) {
    //         motor_type_ = MotorType_STEP;
    //         // The phase pattern is determined by WHICH phase had the maximum resistance.
    //         // Index of Max: 0->ABC, 1->BCA, 2->CAB
    //         phase_pattern_ = static_cast<PhasePattern>(max_index);
    //         // Stepper: R = (A + B) / 2, where A and B are the two smaller coil resistances
    //         // r_min and r_mid are the two coil resistances
    //         total_resistance_mohm_ = (r_min + r_mid) >> 1;
    //         return;
    //     }

    //     // 7. Fallback if no known pattern matches
    //     motor_type_ = MotorType_Invalid;
    //     total_resistance_mohm_ = 0;  // Invalid motor - no resistance calculation
    // }


    // Print parameters to serial
    template <typename SerialType>
    void print(SerialType& serial) const {
        serial.println("=== Motor Parameters ===");

        // Motor type
        serial.print("Type: ");
        switch (motor_type_) {
            case MotorType_BLDC:
                serial.println("BLDC");
                break;
            case MotorType_STEP:
                serial.println("Stepper");
                break;
            case MotorType_DC:
                serial.println("DC");
                break;
            default:
                serial.println("Invalid");
                break;
        }

        // Phase pattern
        serial.print("Pattern: ");
        switch (phase_pattern_) {
            case PhasePattern_ABC:
                serial.println("ABC");
                break;
            case PhasePattern_ACB:
                serial.println("ACB");
                break;
            case PhasePattern_CBA:
                serial.println("CBA");
                break;
        }

        // Driver status
        serial.print("Status: ");
        switch (status_) {
            case MotorStatus::Success:
                serial.println("Success");
                break;
            case MotorStatus::SoftWarmup:
                serial.println("SoftWarmup");
                break;
            case MotorStatus::Settle:
                serial.println("Settle");
                break;
            case MotorStatus::SlowRotation:
                serial.println("SlowRotation");
                break;
            case MotorStatus::ResistanceMeasure:
                serial.println("ResistanceMeasure");
                break;
            case MotorStatus::InductanceMeasure:
                serial.println("InductanceMeasure");
                break;
            case MotorStatus::CalibError:
                serial.println("CalibError");
                break;
            case MotorStatus::NotCalibrated:
                serial.println("NotCalibrated");
                break;
            default:
                serial.println("Unknown");
                break;
        }

        // Pole count and direction
        serial.print("Poles: ");
        serial.print(pole_count_);
        serial.print(", Dir: ");
        serial.println(direction_);

        // Angles
        serial.print("E_angle: ");
        serial.print(electric_angle_);
        serial.print(", M_angle: ");
        serial.println(mechanic_angle_);

        // Phase resistances
        serial.print("Resistances [mOhm]:");
        serial.println(total_resistance_mohm_);

        // Operating limits
        serial.print("I_lim: ");
        serial.print(current_limit_ma_);
        serial.print(" mA, V_max: ");
        serial.print(max_voltage_mv_);
        serial.println(" mV");

        serial.println("========================");
    }
};
