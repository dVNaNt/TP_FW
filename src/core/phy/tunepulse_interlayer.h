#ifndef INTERLAYER_HPP
#define INTERLAYER_HPP

/**
 * @file tunepulse_interlayer.h
 * @brief Convenience umbrella header grouping the hardware "interlayer"
 *        drivers used by the TunePulse stack.
 *
 * Pulls in, with a single include:
 * - flash_eeprom.h            : EEPROM-emulation middleware (calibration
 *                               table persistence).
 * - pwm_driver/driver_pwm.h : physical PWM/ADC driver layer (DriverPWM,
 *                               current sensing, supply voltage monitoring).
 * - encoder_phy/encoder.h     : magnetic encoder interface.
 *
 * This header defines no code of its own; it exists so that application code
 * can depend on one stable include instead of the individual driver paths.
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#include "flash_eeprom.h"
#include "pwm_driver/driver_pwm.h"
#include "encoder_phy/encoder.h"

#endif