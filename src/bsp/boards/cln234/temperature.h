/**
 * @file temperature.h
 * @brief External NTC thermistor (TEMP = PC4 / ADC2_IN5) support for CLN234.
 *
 * Board glue for the generic Temperature driver
 * (core/phy/pwm_driver/temperature.h): provides the raw -> degC
 * conversion callback and the ISR feed helper. No lib changes: everything
 * below is CLN234-target-only code.
 *
 * ## Divider topology
 *
 * The sensor is an NTC 100k (R25 = 100 kOhm) wired to GND (LOW side) with a
 * 40.2 kOhm pull-up to VDDA (upper leg). The ADC measures
 *
/**
 * @file temperature.h
 * @brief External NTC thermistor (TEMP = PC4 / ADC2_IN5) support for CLN234.
 *
 * Board glue for the generic Temperature driver
 * (core/phy/pwm_driver/temperature.h): provides the raw -> degC
 * conversion callback and the ISR feed helper. No lib changes: everything
 * below is CLN234-target-only code.
 *
 * ## Divider topology
 *
 * The sensor is an NTC 100k (R25 = 100 kOhm) wired to GND (LOW side) with a
 * 40.2 kOhm pull-up to VDDA (upper leg). The ADC measures
 *
 *     code = 65535 * R_ntc / (R_pull + R_ntc)     (VDDA cancels out)
 *
 * so a HOTTER thermistor produces a SMALLER code - the raw domain is
 * anti-monotonic with temperature, while Temperature requires "more raw =
 * hotter" (binary-searched thresholds + `raw >= trip` hysteresis).
 * Temperature_NTC_Feed() inverts the word ONCE at ingestion
 * (`internal = 0xFFFF - raw`), which guarantees the monotonic domain for
 * the whole pipeline: 0 = coldest, 65535 = hottest.
 *
 * The conversion callback runs the inverse path: it undoes the feed
 * inversion and hands the TRUE uint16 left-aligned divider word to the
 * hardware NTC template (NTC_GetCelsius<R25, B25, Pullup>), which applies
 * its OWN low-side code inversion internally, recovers the resistance and
 * evaluates the beta model (B25/50 = 4250 K, exact at 25 degC). A result
 * outside the safe window (-60..+150 degC) means the sensor is shorted or
 * not connected -> the template returns NaN.
 *
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#ifndef TEMPERATURE_CLN234_H
#define TEMPERATURE_CLN234_H

#include "bsp/chips/temperature/ntc.h"            // Target-agnostic NTC template (NTC_GetCelsius)
#include "core/phy/pwm_driver/temperature.h"  // Generic Temperature driver (callback API)

/**
 * @brief Raw -> degC conversion callback for the generic Temperature driver.
 *
 * Signature-compatible with Temperature_Convert_Callback (uint32_t). The
 * argument is the FILTERED raw word in the INVERTED monotonic domain
 * produced by Temperature_NTC_Feed() (0 = coldest, 65535 = hottest). The
 * feed inversion is undone here and the TRUE uint16 left-aligned divider
 * word is forwarded to the hardware NTC template - which applies its OWN
 * low-side inversion internally and returns NaN outside the safe window
 * (shorted / not connected sensor); the NaN never reaches the integer
 * tick() path, only the lazy telemetry conversion.
 *
 * @param raw_adc Filtered raw value, 16-bit left-aligned, monotonic domain
 * @return Temperature in degrees Celsius, NaN on short / not connected
 */
inline float Temperature_NTC_To_Celsius(uint32_t raw_adc) {
    return NTC_GetCelsius<100000, 4250, 40200>((uint16_t)(0xFFFFU - raw_adc));  // Undo feed inversion -> TRUE word; R25 = 100k, B25 = 4250 K, pull-up = 40.2k (low side)
}

/**
 * @brief ISR feed helper: publish a fresh NTC ADC word into a Temperature.
 *
 * Accepts the TRUE left-aligned ADC word straight from the DMA buffer (same
 * convention as Temperature::set_temperature_irq). The low-side NTC divider
 * is anti-monotonic with temperature, so the word is inverted once here into
 * the internal monotonic domain required by the driver's binary-searched
 * thresholds and hysteresis comparator (0 = coldest, 65535 = hottest). Safe
 * to call from any interrupt context; a single relaxed store.
 *
 * @param temperature Generic Temperature instance to feed
 * @param raw_adc Raw uint16 left-aligned ADC value of the NTC channel
 */
inline void Temperature_NTC_Feed(Temperature& temperature, uint16_t raw_adc) {
    temperature.set_temperature_irq((uint16_t)(0xFFFFU - raw_adc));  // Invert: hot -> large code (monotonic driver domain)
}

#endif  // TEMPERATURE_CLN234_H
