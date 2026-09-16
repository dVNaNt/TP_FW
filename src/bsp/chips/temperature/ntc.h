/**
 * @file ntc.h
 * @brief Compile-time-configured NTC thermistor converter:
 *        uint16_t LEFT-ALIGNED ADC word in -> float degC out.
 *
 * ## Contract (the same for every file in hardware/temperature/)
 *
 *   - INPUT : uint16_t, LEFT-ALIGNED ADC data. ALL 16 BITS are used: the
 *     full-scale code is 65535 (a 12-bit ADC left-aligned word simply has
 *     its low nibble at zero, but the math never discards it).
 *   - OUTPUT: float, temperature in degrees Celsius - a pure physics
 *     conversion, no filtering or protection applied, always unchanged.
 *
 * ## Divider topologies and internal code inversion
 *
 * Which reference leg is present (Pullup / Pulldown) fixes the sensor
 * position AND the code polarity. The getters normalize the code ONCE
 * internally into the monotonic domain (hotter = LARGER code), so every
 * scenario after normalization shares one code path:
 *
 * \verbatim
 *   LOW SIDE  (Pullup given):          HIGH SIDE (Pulldown given):
 *
 *       VREF  <--- Pullup                 VREF
 *        |                                 |
 *      [Rpull]                           [NTC]   sensor to VREF
 *        |--- ADC tap (code)               |--- ADC tap (code)
 *      [NTC]   sensor to GND             [Rpull]
 *        |                                 |
 *       GND                               GND
 *
 *   hot sensor -> SMALLER code         hot sensor -> LARGER code
 *   inverted internally:               used as-is:
 *     mono = 65535 - code                mono = code
 * \endverbatim
 *
 * In the normalized domain both topologies share the same formula:
 *
 *   R_sensor = R_ref * (65535 - mono) / mono
 *
 * ## Safe range -> NaN (short / not connected)
 *
 * @p Tmin_x10 / @p Tmax_x10 define the CUSTOM safe temperature window
 * (tenths of degC). A result outside the window means the divider is in a
 * degenerate state - SHORTED sensor (T above the window) or NOT CONNECTED /
 * open sensor (T below the window) - and the getters return NaN instead of
 * a fake value.
 *
 * Binary-search compatibility: the generic Temperature driver
 * (core/phy/pwm_driver/temperature.h) inverts degC limits with
 * celsius2raw(), a binary search using the predicate `T(mid) < target`.
 * NaN compares as "not less", so probes inside the NaN regions terminate
 * the search at the nearest boundary of the valid band - no infinite loop,
 * no misinversion. The integer hysteresis comparator in tick() never sees
 * the callback at all. Configure the OT limits INSIDE the safe window.
 *
 * ## Configuration = template arguments (presence/absence selects scenario)
 *
 * Every option is an INTEGER compile-time constant (C++17-compatible NTTP;
 * float NTTPs would need C++20). An ABSENT option (left at its zero default)
 * removes its whole scenario from the binary via `if constexpr` - the
 * generated code contains only the math the configuration actually needs:
 * zero flash / zero RAM overhead.
 *
 *   R25        [Ohm]      - NTC resistance at 25 degC (mandatory)
 *   B25        [K]        - beta number B25/x (mandatory)
 *   Pullup     [Ohm]      - reference leg to VREF, sensor to GND ("low side")
 *   Pulldown   [Ohm]      - reference leg to GND, sensor to VREF ("high side")
 *   Tmin_x10   [degC x10] - safe window lower bound (default -600 = -60.0)
 *   Tmax_x10   [degC x10] - safe window upper bound (default 1500 = +150.0)
 *
 * ## Physics
 *
 *   NTC:      R(T) = R25 * exp(B * (1/T - 1/T25))        (forward beta model)
 *             T    = 1 / (1/T25 + ln(R/R25) / B)         (inversion used)
 *
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#ifndef HARDWARE_TEMPERATURE_NTC_H
#define HARDWARE_TEMPERATURE_NTC_H

#include <math.h>    // logf, NAN
#include <stdint.h>  // Fixed-width integer types

// ---------------------------------------------------------------- constants
inline constexpr float NTC_T25_K = 298.15f;  // Reference temperature: 25 degC [K]

/**
 * @brief NTC thermistor -> degrees Celsius.
 *
 * Handles BOTH divider polarities internally: a low-side sensor (Pullup
 * present) produces an anti-monotonic code (hotter = smaller), which is
 * inverted ONCE here; a high-side sensor (Pulldown present) is already
 * monotonic and is used as-is. Callers feed the raw ADC word straight from
 * the DMA buffer - no pre-inversion is ever required.
 *
 * Result outside the safe window [Tmin_x10, Tmax_x10] (shorted or not
 * connected sensor) yields NaN - see the file header for the binary-search
 * compatibility notes.
 *
 * @param R25      NTC resistance at 25 degC [Ohm] (mandatory)
 * @param B25      Beta number B25/x [K] (mandatory)
 * @param Pullup   Reference leg to VREF, sensor to GND [Ohm] (0 = not present)
 * @param Pulldown Reference leg to GND, sensor to VREF [Ohm] (0 = not present)
 * @param Tmin_x10 Safe window lower bound [degC x10] (default -600 = -60.0)
 * @param Tmax_x10 Safe window upper bound [degC x10] (default 1500 = +150.0)
 * @param raw_adc  uint16_t LEFT-ALIGNED ADC word of the divider tap (all 16 bits used)
 * @return Temperature [degC], or NaN when shorted / not connected
 */
template <int32_t R25, int32_t B25, int32_t Pullup = 0, int32_t Pulldown = 0,
          int32_t Tmin_x10 = -600, int32_t Tmax_x10 = 1500>
inline float NTC_GetCelsius(uint16_t raw_adc) {
    static_assert(R25 > 0 && B25 > 0, "R25 and B25 are mandatory");                          // Sensor identity
    static_assert((Pullup > 0) != (Pulldown > 0), "Provide exactly one of Pullup/Pulldown"); // One divider topology
    static_assert(Tmin_x10 < Tmax_x10, "Safe window is empty");                              // Sane window

    // Internal code inversion (see the diagram in the file header):
    // normalize once into the monotonic domain "hotter = LARGER code".
    // The FULL 16-bit word is the code - nothing is discarded.
    const uint16_t mono = (Pullup > 0)                    // Normalized code
                              ? (uint16_t)(65535U - raw_adc)  // Low side: hotter = smaller -> invert
                              : raw_adc;                      // High side: hotter = larger -> as-is
    if (mono == 0U)                                    // Guard the single zero divisor (open sensor rail)
        return NAN;                                    // Infinite resistance -> outside any window -> NaN

    // Recover the sensor resistance (unified formula, Vref cancels out)
    const float r_ntc = (float)(Pullup + Pulldown) * (float)(65535U - mono) / (float)mono;  // Sensor resistance [Ohm]

    // Beta-model inversion: one log, two divisions
    const float t_c = 1.0f / (1.0f / NTC_T25_K + logf(r_ntc / (float)R25) / (float)B25) - 273.15f;

    // Custom safe window: outside it the divider is degenerate -
    // above = shorted sensor, below = not connected sensor -> NaN
    if (t_c < (float)Tmin_x10 * 0.1f || t_c > (float)Tmax_x10 * 0.1f)
        return NAN;

    return t_c;                                        // Valid temperature [degC]
}
#endif  // HARDWARE_TEMPERATURE_NTC_H
