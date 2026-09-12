/**
 * @file ptc.h
 * @brief Compile-time-configured linear PTC thermistor converter:
 *        uint16_t LEFT-ALIGNED ADC word in -> float degC out.
 *
 * Companion of ntc.h for the linear PTC family (silicon PTCs of the
 * KTY8x class and similar). The SAME contract, the SAME divider diagrams,
 * the SAME internal code inversion, the SAME safe-window/NaN rule and the
 * SAME optional template arguments apply - only the R(T) model differs:
 * linear instead of beta.
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
 *      [Rpull]                           [PTC]   sensor to VREF
 *        |--- ADC tap (code)               |--- ADC tap (code)
 *      [PTC]   sensor to GND             [Rpull]
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
 * a fake value. Binary-search compatibility with the generic Temperature
 * driver: see ntc.h (NaN compares as "not less", the celsius2raw search
 * converges at the boundary of the valid band; tick() is integer-only).
 *
 * ## Configuration = template arguments (see ntc.h for the full table)
 *
 *   R25        [Ohm]      - sensor resistance at 25 degC (mandatory)
 *   K_mOhm     [mOhm/K]   - resistance slope k, x1000 (mandatory; KTY84-130
 *                           = 4670 for 4.67 Ohm/K)
 *   Pullup     [Ohm]      - reference leg to VREF, sensor to GND (0 = absent)
 *   Pulldown   [Ohm]      - reference leg to GND, sensor to VREF (0 = absent)
 *   Tmin_x10   [degC x10] - safe window lower bound (default -600 = -60.0)
 *   Tmax_x10   [degC x10] - safe window upper bound (default 1500 = +150.0)
 *
 * ## Physics
 *
 *   PTC:      R(T) = R25 + k * (T - 25 degC)             (linear silicon PTC)
 *             T    = 25 degC + (R - R25) / k             (inversion used)
 *
 * Standard sensor values (KTY84-130 class): R25 = 980 Ohm, R100 = 1330 Ohm
 * -> slope k = (1330 - 980) / 75 = 4.67 Ohm/K = 4670 mOhm/K.
 *
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#ifndef HARDWARE_TEMPERATURE_PTC_H
#define HARDWARE_TEMPERATURE_PTC_H

#include <math.h>    // NAN
#include <stdint.h>  // Fixed-width integer types

// ---------------------------------------------------------------- constants
inline constexpr float PTC_T25_C = 25.0f;  // Reference temperature [degC]

/**
 * @brief Linear PTC thermistor -> degrees Celsius.
 *
 * Handles BOTH divider polarities internally: a low-side sensor (Pullup
 * present) produces an anti-monotonic code (hotter = smaller), which is
 * inverted ONCE here; a high-side sensor (Pulldown present) is already
 * monotonic and is used as-is. Callers feed the raw ADC word straight from
 * the DMA buffer - no pre-inversion is ever required.
 *
 * Result outside the safe window [Tmin_x10, Tmax_x10] (shorted or not
 * connected sensor) yields NaN - see ntc.h for the binary-search
 * compatibility notes.
 *
 * @param R25      Sensor resistance at 25 degC [Ohm] (mandatory)
 * @param K_mOhm   Resistance slope k [mOhm/K], x1000 (mandatory)
 * @param Pullup   Reference leg to VREF, sensor to GND [Ohm] (0 = not present)
 * @param Pulldown Reference leg to GND, sensor to VREF [Ohm] (0 = not present)
 * @param Tmin_x10 Safe window lower bound [degC x10] (default -600 = -60.0)
 * @param Tmax_x10 Safe window upper bound [degC x10] (default 1500 = +150.0)
 * @param raw_adc  uint16_t LEFT-ALIGNED ADC word of the divider tap (all 16 bits used)
 * @return Temperature [degC], or NaN when shorted / not connected
 */
template <int32_t R25, int32_t K_mOhm, int32_t Pullup = 0, int32_t Pulldown = 0,
          int32_t Tmin_x10 = -600, int32_t Tmax_x10 = 1500>
inline float PTC_GetCelsius(uint16_t raw_adc) {
    static_assert(R25 > 0 && K_mOhm > 0, "R25 and K_mOhm are mandatory");                          // Sensor identity
    static_assert((Pullup > 0) != (Pulldown > 0), "Provide exactly one of Pullup/Pulldown");       // One divider topology
    static_assert(Tmin_x10 < Tmax_x10, "Safe window is empty");                                    // Sane window

    // Internal code inversion (see the diagram in the file header):
    // normalize once into the monotonic domain "hotter = LARGER code".
    // The FULL 16-bit word is the code - nothing is discarded.
    const uint16_t mono = (Pullup > 0)                    // Normalized code
                              ? (uint16_t)(65535U - raw_adc)  // Low side: hotter = smaller -> invert
                              : raw_adc;                      // High side: hotter = larger -> as-is
    if (mono == 0U)                                    // Guard the single zero divisor (open sensor rail)
        return NAN;                                    // Infinite resistance -> outside any window -> NaN

    // Recover the sensor resistance (unified formula, Vref cancels out)
    const float r_ptc = (float)(Pullup + Pulldown) * (float)(65535U - mono) / (float)mono;  // Sensor resistance [Ohm]

    // Linear-model inversion: T = t25 + (R - R25) / k (one division)
    const float t_c = PTC_T25_C + (r_ptc - (float)R25) * 1000.0f / (float)K_mOhm;

    // Custom safe window: outside it the divider is degenerate -
    // above = shorted sensor, below = not connected sensor -> NaN
    if (t_c < (float)Tmin_x10 * 0.1f || t_c > (float)Tmax_x10 * 0.1f)
        return NAN;

    return t_c;                                        // Valid temperature [degC]
}
#endif  // HARDWARE_TEMPERATURE_PTC_H
