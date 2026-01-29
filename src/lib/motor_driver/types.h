#pragma once

#include "../common.h"


/// MotorType enumeration with predefined motor types
typedef enum : int8_t {
    MotorType_Invalid = 0,
    MotorType_DC = 1,
    MotorType_STEP = 4,
    MotorType_BLDC = 6,
} MotorType;

/// PhasePattern enumeration for 3-phase remap (output ordering)
/// PhasePattern_CBA is default for BLDC motors (positive calibration polarity)
/// Calibration sequence: CBA(0) -> ACB(1) -> ABC(2)
typedef enum : int16_t {
    PhasePattern_ABC = 0, // Negative calibration polarity
    PhasePattern_CBA = -16384, // Default BLDC pattern, positive calibration polarity
    PhasePattern_ACB = 32767, // Negative calibration polarity
} PhasePattern;


/// Represents the motor's overall calibration status.
typedef enum {
    /// Motor is currently undergoing calibration.
    DriverStatus_Calibrating,
    /// Current sensors are being calibrated.
    DriverStatus_CalibrateCurrent,
    /// Motor connection and type calibration (includes warmup, measure, rotation phases).
    DriverStatus_CalibratingMotor,
    /// Playing startup melody through motor coils.
    DriverStatus_PlayingMelody,
    /// HFI warmup phase: applying DC current to D-axis for 1 second.
    DriverStatus_HFI_Warmup,
    /// HFI running: injecting high-frequency signal and measuring response.
    DriverStatus_HFI_Running,
    /// HFI amplitude tuning: adjusting voltage to achieve target current.
    DriverStatus_HFI_AmplitudeTuning,
    /// HFI Ld measurement: measuring D-axis inductance.
    DriverStatus_HFI_LdMeasure,
    /// HFI Lq measurement: measuring Q-axis inductance.
    DriverStatus_HFI_LqMeasure,
    /// Motor calibration completed successfully and ready for normal operation.
    DriverStatus_Ready,
    /// An error occurred during calibration or normal operation.
    DriverStatus_Error
} DriverStatus;
