#ifndef TUNEPULSE_HEART_H
#define TUNEPULSE_HEART_H

#include <Arduino.h>

#include "lib/feedback/inputs_dump.h"
#include "lib/lib.h"
#include "lib/common.h"
#include "lib/feedback/analog/current_sense.h"
#include "lib/motor_driver/driver_pwm/sel_current.h"

#undef SupplyVoltage
#undef Motor
#undef MotorType
#undef PhasePattern
#undef ControlMode
#undef DriverStatus


extern TunePulseHeart TUNE_HEART;

#endif  // TUNEPULSE_HEART_H

