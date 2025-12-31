#pragma once

#include "../common.h"


/// MotorType enumeration with predefined motor types
typedef enum {
    MotorType_Invalid = 0,
    MotorType_DC = 1,
    MotorType_STEP = 4,
    MotorType_BLDC = 6
} MotorType;

/// PhasePattern enumeration for 3-phase remap (output ordering)
/// 4 just to ensure easier calibration algo
typedef enum {
    ABC = 0, 
    BCA = 1,
    CAB = 2,
} PhasePattern;

/// ControlMode enumeration
typedef enum {
    /// As Sin and Cos in alpha-beta coordinate system
    ControlMode_VoltageAB,
    /// As angle of current + DC current amplitude
    ControlMode_CurrentAB
} ControlMode;

/// Represents the motor's overall calibration status.
typedef enum {
    /// Motor is currently undergoing calibration.
    DriverStatus_Calibrating,
    /// Current sensors are being calibrated.
    DriverStatus_CalibrateCurrent,
    /// Motor connection and parameters are being identified.
    DriverStatus_MotorConnectionCalibration,
    /// Motor calibration completed successfully and ready for normal operation.
    DriverStatus_Ready,
    /// An error occurred during calibration or normal operation.
    DriverStatus_Error
} DriverStatus;
