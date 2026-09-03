/**
 * @file selftest.h
 * @brief Detailed board bring-up self-test for the CLN234 (TARGET_CLN234).
 *
 * Purpose: the full tunepulse_init() bring-up is a long sequential chain
 * with several blocking waits. On a first-revision board any step can
 * fail or hang with NO observable output - exactly the "USB enumerates
 * but nothing happens" symptom.
 *
 * This module replaces the normal boot with a strictly non-blocking,
 * heavily logged sequence:
 *
 * - EVERY step prints a "[Txx]" line with the result (OK/FAIL + raw
 *   register values) BEFORE the next step runs.
 * - No step is allowed to wait forever: every wait is bounded by
 *   MAIN_TEST_TIMEOUT_MS and reported as "TIMEOUT".
 * - The WS2812 chain runs a continuous color test: only pure dim
 *   RED / GREEN / BLUE, the two LEDs always show DIFFERENT colors and
 *   the colors rotate all the time, so both LEDs and all three color
 *   channels are verified without a console attached.
 * - Calibration / FOC start are NOT part of this test, but the bridge IS
 *   actively driven: [T06] starts complementary PWM at 20 kHz / 50% duty
 *   and it is never disabled afterwards (per bring-up checklist). DO NOT
 *   connect a motor during the self-test.
 * - The interrupt-driven control path (ADC ISR / TIM1 CC4) is armed but
 *   reported as counters in the heartbeat line instead of driving FOC.
 *
 * Steps: T01 BOOT, T02 CLOCK, T03 LED, T04 GPIO (DRV_EN=HIGH,
 * DRV_ERR pull-up input + EXTI), T05 ADC, T06 PWM (complementary
 * 20 kHz @ 50% duty, stays ON for the whole test), T07 SPI, T08
 * MT6835, T09 ENC samples, then a [T11] heartbeat every 2 s.
 * NOTE: the DRV8353 registers are never accessed (no writes, no reads);
 * the gate driver runs on its hardware defaults.
 *
 * Usage: set USE_BOARD_SELFTEST=1 in platformio.ini build_flags.
 *
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#ifndef SELFTEST_CLN234_H
#define SELFTEST_CLN234_H

#include <Arduino.h>
#include "target.h"
#include "clock_setup.h"

// ==============================================================================
// CONFIG
// ==============================================================================
constexpr uint32_t MAIN_TEST_TIMEOUT_MS = 3000;  // Step-wait timeout
constexpr uint32_t MAIN_TEST_LED_COLOR_MS = 400; // LED color-test rotation period
constexpr uint32_t MAIN_TEST_PERIODIC_MS = 2000; // Periodic report interval

// ==============================================================================
// SEQUENCER STATE
// ==============================================================================
namespace MAIN_TEST {

enum Step : uint8_t {
    ST_BOOT = 0,
    ST_CLOCK,
    ST_LED,
    ST_GPIO,
    ST_ADC,
    ST_PWM,
    ST_SPI_EN,
    ST_SPI_RAW,
    ST_DRV,
    ST_ENC,
    ST_RUN,       // Post-bring-up: periodic monitoring
    ST_DONE,
    ST_COUNT
};

inline volatile uint8_t  step = ST_BOOT;         // Current step
inline uint32_t          step_t0 = 0;            // Step start timestamp
inline bool              step_started = false;   // Step entry flag
inline uint32_t          enc_last = 0;           // Last encoder angle snapshot
inline uint32_t          enc_last_tick = 0;      // Time of the last angle change
inline uint32_t          enc_changes = 0;        // Angle-change counter
inline uint32_t          last_periodic = 0;      // Last periodic report time
inline uint32_t          led_seq_t0 = 0;         // LED color-test phase timer
inline uint32_t          t_enc = 0;              // Last encoder poll time
inline uint16_t          adc_raw[4] = {0};       // Raw ADC snapshot for periodic log

// ==============================================================================
// HELPERS
// ==============================================================================

/**
 * @brief Non-blocking LED color test: the palette holds only pure
 *        primary colors (dim RED / dim GREEN / dim BLUE). Every
 *        MAIN_TEST_LED_COLOR_MS the rotation advances one step; LED1
 *        shows palette[i] and LED2 shows palette[i + 1], so the two
 *        LEDs always display DIFFERENT colors and the colors keep
 *        changing all the time (continuous R->G->B rotation).
 *
 * Brightness is kept low (~15-20% of full scale): the WS2812B at full
 * current is blinding on a bench and needlessly heats the board.
 */
inline void led_test() {
    // Dim primary palette (R, G, B): green is perceptually brightest,
    // so it gets the lowest level to keep all three colors balanced
    static const uint8_t palette[3][3] = {
        {48, 0, 0},   // Dim red
        {0, 25, 0},   // Dim green
        {0, 0, 48}    // Dim blue
    };

    if ((millis() - led_seq_t0) < MAIN_TEST_LED_COLOR_MS) {
        return;  // Keep the current colors until the rotation period elapses
    }
    led_seq_t0 = millis();  // Start the next rotation step

    // Persistent rotation counter: colors advance on every period
    static uint8_t phase = 0;
    phase = (phase + 1U) % 3U;

    // LED1 takes palette[phase], LED2 the next palette entry -> the two
    // LEDs always show different primary colors
    const uint8_t c1 = phase;                  // LED1 color index
    const uint8_t c2 = (phase + 1U) % 3U;      // LED2 color index (always != c1)
    LED_SetRGB2(palette[c1][0], palette[c1][1], palette[c1][2],
                palette[c2][0], palette[c2][1], palette[c2][2]);
}

/** @brief Run one state-machine iteration; call from loop(). */
inline void poll() {
    const uint32_t now = millis();

    // Encoder change tracking (always, for the periodic report)
    const uint32_t angle = Encoder_GetAngle();
    if (angle != enc_last) {
        enc_last = angle;
        enc_last_tick = now;
        enc_changes++;
    }

    // Re-arm the encoder burst read periodically: in the normal firmware
    // this is done from the TIM1 CC4 ISR every PWM cycle, but the
    // self-test has no control interrupt running.
    if ((now - t_enc) >= 10U) {
        t_enc = now;
        Encoder_StartTransfer();  // Bounded: skipped while a transfer is in flight
    }

    // LED color test: two LEDs always different colors, changing all the time
    led_test();

    // Periodic heartbeat once bring-up finished
    if ((step >= ST_RUN) && ((now - last_periodic) >= MAIN_TEST_PERIODIC_MS)) {
        last_periodic = now;
        SerialUSB.print("[T11] t=");
        SerialUSB.print(now / 1000U);
        SerialUSB.print("s clk=");
        SerialUSB.print(SystemCoreClock / 1000000U);
        SerialUSB.print("MHz st=0x");
        SerialUSB.print(SystemClock_Status, HEX);
        SerialUSB.print(" led_tc=");
        SerialUSB.print(ws2812_tc_count);
        SerialUSB.print(" drv_en=");
        SerialUSB.print(LL_GPIO_IsOutputPinSet(GPIOB, LL_GPIO_PIN_12));
        SerialUSB.print(" drv_err=");
        SerialUSB.print(LL_GPIO_IsInputPinSet(GPIOB, LL_GPIO_PIN_1));
        SerialUSB.print(" poll=");
        SerialUSB.print(SPI1_ENCODER_POLLING);
        SerialUSB.print(" act=");
        SerialUSB.print(SPI1_TRANSFER_ACTIVE);
        SerialUSB.print(" ang=0x");
        SerialUSB.print((uint32_t)angle, HEX);
        SerialUSB.print(" enc_chg=");
        SerialUSB.print(enc_changes);
        SerialUSB.print(" age=");
        SerialUSB.print(now - enc_last_tick);
        SerialUSB.print("ms adc=");
        SerialUSB.print(adc_raw[0]);  // I_A
        SerialUSB.print(",");
        SerialUSB.print(adc_raw[1]);  // I_B
        SerialUSB.print(",");
        SerialUSB.print(adc_raw[2]);  // I_C
        SerialUSB.print(",");
        SerialUSB.print(adc_raw[3]);  // VBUS
        SerialUSB.print(" ms=");
        SerialUSB.println(millis());
    }
}

}  // namespace MAIN_TEST
#endif  // SELFTEST_CLN234_H