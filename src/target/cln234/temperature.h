/**
 * @file temperature.h
 * @brief External NTC thermistor (TEMP = PC4 / ADC2_IN5) support for CLN234.
 *
 * Board glue for the generic Temperature driver
 * (lib/drivers/pwm_driver/temperature.h): provides the raw -> degC
 * conversion callback and the ISR feed helper. No lib changes: everything
 * below is CLN234-target-only code.
 *
 * ## Divider topology
 *
 * The sensor is an NTC 100k (R25 = 100 kOhm) wired to GND (LOW side) with a
 * 40.2 kOhm pull-up to VDDA (upper leg). The ADC measures
 *
 *     code = 4095 * R_ntc / (R_pull + R_ntc)     (VDDA cancels out)
 *
 * so a HOTTER thermistor produces a SMALLER code - the raw domain is
 * anti-monotonic with temperature, while Temperature requires "more raw =
 * hotter" (binary-searched thresholds + `raw >= trip` hysteresis).
 * Temperature_NTC_Feed() inverts the word ONCE at ingestion
 * (`internal = 0xFFFF - raw`; the low nibble of a left-aligned word is 0,
 * hence internal>>4 == 4095 - code), which guarantees the monotonic domain
 * for the whole pipeline: 0 = coldest, 65535 = hottest.
 *
 * ## Piecewise beta model
 *
 * The NTC is specified by two-point beta values measured between 25 degC and
 * an upper anchor. Each beta is exact at 25 degC and at its anchor, so the
 * most accurate cheap model is piecewise: pick the beta whose interval
 * contains the result:
 *
 *     R(T) = R25 * exp( B (1/T - 1/T25) )        (forward)
 *     T    = 1 / (1/T25 + ln(R/R25) / B)         (inversion used here)
 *
 * The beta table is walked upward: start with B25/50; while the computed
 * temperature exceeds the current anchor, recompute with the next beta
 * (a larger beta lowers the result, so the walk converges in one pass).
 * One logf and at most 4 divisions per conversion. Degenerate divider states
 * are clamped: a shorted NTC reports +150 degC (guarantees an OT trip), an
 * open/disconnected NTC reports -60 degC.
 *
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#ifndef TEMPERATURE_CLN234_H
#define TEMPERATURE_CLN234_H

#include "../../lib/drivers/pwm_driver/temperature.h"  // Generic Temperature driver (callback API)

// ============================ NTC parameters ================================

static constexpr float kNtcPullupOhm = 40200.0f;  ///< Upper leg resistor [Ohm]
static constexpr float kNtcR25Ohm = 100000.0f;    ///< NTC resistance at 25 degC [Ohm]
static constexpr float kNtcT25Kelvin = 298.15f;   ///< 25 degC reference point [K]
static constexpr float kNtcKelvin0 = 273.15f;     ///< Kelvin -> Celsius offset
static constexpr float kNtcClampHotC = 150.0f;    ///< Report for a shorted sensor [degC]
static constexpr float kNtcClampColdC = -60.0f;   ///< Report for an open sensor [degC]

/**
 * @brief One piecewise-beta table entry.
 *
 * @p beta is the two-point B25/Tmax value: exact at 25 degC and at
 * @p t_max_c; used for all temperatures inside its interval
 * (previous anchor .. t_max_c]. The last entry extends to the top of the
 * measurable range.
 */
struct NtcBetaPoint {
    float t_max_c;  ///< Upper anchor of the interval [degC]
    float beta;     ///< Two-point beta B25/Tmax for this interval
};

/// Piecewise beta table (see the file header for the model description).
static constexpr NtcBetaPoint kNtcBetas[4] = {
    {50.0f, 4250.0f},   // B25/50  - exact at 25 and 50 degC
    {80.0f, 4303.0f},   // B25/80  - exact at 25 and 80 degC
    {85.0f, 4311.0f},   // B25/85  - exact at 25 and 85 degC
    {200.0f, 4334.0f},  // B25/100 - used above 85 degC
};

/**
 * @brief Raw -> degC conversion callback for the generic Temperature driver.
 *
 * Signature-compatible with Temperature_Convert_Callback. The argument is
 * the FILTERED raw word in the INVERTED monotonic domain produced by
 * Temperature_NTC_Feed() (0 = coldest, 65535 = hottest).
 *
 * @param raw_adc Filtered raw value, 16-bit left-aligned, monotonic domain
 * @return Temperature in degrees Celsius (clamped on sensor faults)
 */
inline float Temperature_NTC_To_Celsius(uint32_t raw_adc) {
    const uint32_t code_inv = (raw_adc >> 4) & 0x0FFFU;  // Internal 12-bit code (4095 = coldest)
    if (code_inv >= 4094U)
        return kNtcClampHotC;   // ADC reads ~0 V: NTC shorted to GND -> hotter than any limit
    if (code_inv <= 1U)
        return kNtcClampColdC;  // ADC reads full scale: NTC open/disconnected -> colder than any range

    // Recover the thermistor resistance from the divider equation:
    //   code = 4095 * R_ntc / (R_pull + R_ntc), code = 4095 - code_inv
    //   -> R_ntc = R_pull * (4095 - code_inv) / code_inv
    const float r_ntc = kNtcPullupOhm * (float)(4095U - code_inv) / (float)code_inv;

    // Piecewise beta inversion: one log, at most one division per beta.
    // ln(R/R25) is negative when hot (R < R25) and positive when cold.
    const float ln_ratio = logf(r_ntc / kNtcR25Ohm);  // ln(R_ntc / R25), evaluated once
    const float inv_t25 = 1.0f / kNtcT25Kelvin;       // Precomputed 1/T25
    float t_celsius = kNtcClampColdC;                 // Fall-back result (never used: 4 entries)
    for (uint8_t i = 0; i < 4; ++i) {
        t_celsius = 1.0f / (inv_t25 + (ln_ratio / kNtcBetas[i].beta)) - kNtcKelvin0;  // Model inversion
        if (t_celsius < kNtcBetas[i].t_max_c)
            break;  // Result fits this beta's validity interval -> done
    }
    return t_celsius;
}

/**
 * @brief ISR feed helper: publish a fresh NTC ADC word into a Temperature.
 *
 * Accepts the TRUE left-aligned ADC word straight from the DMA buffer (same
 * convention as Temperature::set_temperature_irq). The low-side NTC divider
 * is anti-monotonic with temperature, so the word is inverted once here into
 * the internal monotonic domain used by the filter, the thresholds and the
 * conversion callback. Safe to call from any interrupt context; a single
 * relaxed store.
 *
 * @param temperature Generic Temperature instance to feed
 * @param raw_adc Raw 16-bit left-aligned ADC value of the NTC channel
 */
inline void Temperature_NTC_Feed(Temperature& temperature, uint32_t raw_adc) {
    temperature.set_temperature_irq(0xFFFFU - (raw_adc & 0x0000FFFFU));  // Invert: hot -> large code
}

#endif  // TEMPERATURE_CLN234_H
