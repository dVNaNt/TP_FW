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
    ElectricCalibrationDone = 7,
    InductanceMeasureWarmup = 8,
    InductanceMeasureD  = 9,
    InductanceMeasureQ = 10,
        InductanceMeasureD2  = 11,
    InductanceMeasureQ2 = 12,
    // MechanicalCalibration = 8,
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

};
