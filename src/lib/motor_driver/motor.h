#pragma once

/**
 * @file motor.h
 * @brief Motor description parameters and calibration lifecycle state.
 *
 * Motor holds the electrical/mechanical identity of the connected machine
 * (type, pole pairs, resistance/inductance, limits) plus the MotorStatus
 * state machine value describing where the automatic calibration sequence
 * currently is.
 */

#include "../common.h"
#include "types.h"

/**
 * Motor calibration status enum - accessible as SetupStatus::MotorCalibrationStatus.
 *
 * The automatic calibration sequence walks through these states:
 * SoftWarmup -> Settle -> SlowRotation -> ResistanceMeasure ->
 * InductanceMeasure(Warmup/D/Q/D2/Q2) -> ElectricCalibrationDone -> Success.
 */
enum class MotorStatus : uint8_t {
    Success = 0,            ///< Calibration finished / not calibrating.
    SoftWarmup = 1,         ///< Checking motor warmup and connection.
    Settle = 2,             ///< Settling phase before measurement (~900 ticks).
    SlowRotation = 3,       ///< Slow rotation for motor type detection.
    ResistanceMeasure = 4,  ///< Measuring winding resistance.
    InductanceMeasure = 5,  ///< Measuring inductance using HFI (high-frequency injection).
    CalibError = 6,         ///< Calibration error (abort).
    ElectricCalibrationDone = 7,   ///< Electrical identification finished.
    InductanceMeasureWarmup = 8,   ///< HFI warm-up stage.
    InductanceMeasureD  = 9,       ///< HFI measurement on the D axis (first pass).
    InductanceMeasureQ = 10,       ///< HFI measurement on the Q axis (first pass).
    InductanceMeasureD2  = 11,     ///< HFI measurement on the D axis (second pass).
    InductanceMeasureQ2 = 12,      ///< HFI measurement on the Q axis (second pass).
    // MechanicalCalibration = 8,  ///< (historical, unused)
    NotCalibrated = 127     ///< Initial state: no calibration has been run.
};

/**
 * Motor and driver system parameter container.
 *
 * Holds the electrical and mechanical constants needed for motor control:
 * type/pattern identity, pole count, per-phase R/L values and operating
 * limits. Mostly a passive data holder with inline accessors.
 */
struct Motor {
private:
    // Motor type and configuration
    MotorType motor_type_;        ///< Type of motor (STEP / BLDC / Invalid).
    PhasePattern phase_pattern_;  ///< Phase connection pattern (ABC, ACB, CBA).
    MotorStatus status_;          ///< Current calibration status.

    // Motor mechanical parameters
    int32_t pole_count_;  ///< Number of POLE PAIRS (default 7 for a CLN17-class gimbal motor).
    int16_t direction_;   ///< Motor rotation direction (+1/-1).

    // Motor angles
    int32_t electric_angle_;  ///< Electric angle in internal fixed-point units.
    int32_t mechanic_angle_;  ///< Mechanical angle in internal fixed-point units.

    /// Total (line-to-line equivalent) motor resistance in milliohms.
    float total_resistance_mohm_;

    /// Per-phase inductance in HENRIES as stored by default ctor path (see note below).
    float phase_inductance_[3];

    /**
     * Default inductance used when no measurement exists.
     * @note UNIT MISMATCH: the constructor stores 0.003 with comments claiming
     *       "microhenries" elsewhere, but 0.003 only equals 3 mH if the unit
     *       is HENRIES. See CODE_ISSUES.md.
     */
    float default_inductance_;

    // Operating limits
    float current_limit_ma_;  ///< Current limit for calibration in milliamperes.
    float max_voltage_mv_;    ///< Maximum voltage in millivolts.

public:
    /// Per-phase resistance measurements in milliohms (filled by calibration).
    int32_t phase_resistance_mohm[3] = {0,0,0};
    /// Constructor with default values (7 pole pairs, +1 direction, 600 mA
    /// calibration current limit, 24 V max voltage, 3 mH default inductance).
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
          default_inductance_(0.003) {  // Default 3 mH inductance
        // Initialize resistance and inductance arrays to zero
        for (uint8_t i = 0; i < 3; i++) {
            phase_inductance_[i] = 0;
        }
    }

    // === Setters ===

    /// Set motor type (enum overload).
    inline void set_motor_type(MotorType type) { motor_type_ = type; }
    /// Set motor type (raw numeric overload, cast to MotorType).
    inline void set_motor_type(int8_t type) { motor_type_ = (MotorType)type; }

    /// Set phase connection pattern.
    inline void set_phase_pattern(PhasePattern pattern) { phase_pattern_ = pattern; }

    /// Set calibration status.
    inline void set_status(MotorStatus status) { status_ = status; }

    /// Set number of pole pairs.
    inline void set_pole_count(int32_t poles) { pole_count_ = poles; }

    /// Set rotation direction (+1/-1).
    inline void set_direction(int16_t dir) { direction_ = dir; }

    /// Set electric angle (internal units).
    inline void set_electric_angle(int32_t angle) { electric_angle_ = angle; }

    /// Set mechanical angle (internal units).
    inline void set_mechanic_angle(int32_t angle) { mechanic_angle_ = angle; }


    /// Set measured inductance for a specific phase (index 0-2; out-of-range ignored).
    void set_phase_inductance(uint8_t phase_idx, float inductance_uh) {
        if (phase_idx >= 3)
            return;                                       // Validate index
        phase_inductance_[phase_idx] = inductance_uh;  // Store inductance
    }

    // Set current limit for calibration
    inline void set_current_limit(float current_ma) { current_limit_ma_ = current_ma; }

    // Set maximum voltage
    inline void set_max_voltage(float voltage_mv) { max_voltage_mv_ = voltage_mv; }

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
    inline float get_phase_inductance(uint8_t phase_idx) const { return (phase_idx < 3) ? phase_inductance_[phase_idx] : 0; }
    
    /**
     * Get effective inductance (measured or default).
     *
     * Returns the phase-A measured inductance when available (> 0),
     * otherwise falls back to default_inductance_. Only phase A is checked;
     * measurements present solely on phases B/C are ignored.
     */
    inline float get_effective_inductance() const {
        if (phase_inductance_[0] > 0) {
            return phase_inductance_[0];  // Return measured value
        }
        return default_inductance_;  // Return default (nominally 3 mH)
    }
    
    /// Override the fallback inductance value (same unit convention as storage).
    inline void set_default_inductance(float inductance_uh) { default_inductance_ = inductance_uh; }

    // Get current limit for calibration
    inline float get_current_limit() const { return current_limit_ma_; }

    // Get maximum voltage
    inline float get_max_voltage() const { return max_voltage_mv_; }

    // Get total motor resistance (calculated from phases based on motor type)
    inline float get_total_resistance() const { return total_resistance_mohm_; }

    inline void set_total_resistance(float res) {
        total_resistance_mohm_ = res;
    }

};
