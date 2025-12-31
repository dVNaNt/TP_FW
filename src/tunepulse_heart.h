#ifndef TUNEPULSE_HEART_H
#define TUNEPULSE_HEART_H

#include <Arduino.h>
#include "setup/timer_pwm_setup.h"

// Rename migration types to avoid collisions with legacy block classes
#define SupplyVoltage MigrationSupplyVoltage
#define Motor MigrationMotor
#define MotorType MigrationMotorType
#define PhasePattern MigrationPhasePattern
#define ControlMode MigrationControlMode
#define DriverStatus MigrationDriverStatus

#include "migration/feedback/inputs_dump.h"
#include "migration/lib.h"
#include "migration/common.h"
#include "migration/feedback/analog/current_sense.h"
#include "migration/motor_driver/driver_pwm/sel_current.h"

#undef SupplyVoltage
#undef Motor
#undef MotorType
#undef PhasePattern
#undef ControlMode
#undef DriverStatus

// TunePulseHeart is now defined in migration/lib.h
// This header provides type aliases and global instance declaration

using MigMotorType = MigrationMotorType;
using MigPhasePattern = MigrationPhasePattern;

extern TunePulseHeart TUNE_HEART;

#endif  // TUNEPULSE_HEART_H

