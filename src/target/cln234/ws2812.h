#ifndef WS2812_CLN234_H
#define WS2812_CLN234_H

#include <Arduino.h>
#include "stm32g4xx.h"

// ==============================================================================
// TIMING CONSTANTS (nominal values; real tick counts are derived at runtime
// from SystemCoreClock so the driver works at 170 MHz PLL clock as well as on
// the 16 MHz HSI when the clock bring-up step was skipped)
// ==============================================================================
constexpr uint32_t WS2812_T0H_NS    = 300;   // "0" high time (spec 220-380 ns)
constexpr uint32_t WS2812_T1H_NS    = 650;   // "1" high time (spec 580-1000 ns)
constexpr uint32_t WS2812_BIT_NS    = 1300;  // Total bit period (spec >= 1250 ns)
constexpr uint8_t  WS2812_LED_COUNT = 2;     // Two WS2812 LEDs daisy-chained on PC6
constexpr uint32_t WS2812_RESET_US  = 300;   // Latch: line idle LOW > 280 us per spec

// ==============================================================================
// STATE
// ==============================================================================
inline volatile uint8_t  ws2812_rgb[WS2812_LED_COUNT][3] = {{0, 0, 0}, {0, 0, 0}};  // Shadow colors (RGB order, per LED)
inline volatile uint32_t ws2812_tc_count = 0;             // Completed frames (diagnostics)
inline volatile uint32_t ws2812_frame_end_us = 0;         // micros() stamp of the last frame end (reset-time guard)
inline uint32_t ws2812_tck_t0h = 0;                       // "0" high time in CPU cycles
inline uint32_t ws2812_tck_t1h = 0;                       // "1" high time in CPU cycles
inline uint32_t ws2812_tck_bit = 0;                       // Full bit period in CPU cycles


// ==============================================================================
// INITIALIZATION
// ==============================================================================

/**
 * @brief Initialize the WS2812 output: PC6 as a PLAIN GPIO open-drain output.
 *
 * Deliberately dumb hardware setup: no timer, no DMA, no interrupts.
 * - LOW  : the on-chip N-MOS pulls the line down (WS2812 reset / "0" low).
 * - HIGH : the pin is released and the EXTERNAL pull-up to 5 V provides the
 *          high level required by the WS2812 logic (the MCU runs at 3.3 V).
 * After init the line idles LOW, so both LEDs are dark (reset state).
 *
 * The DWT cycle counter is also enabled here; it is the time base used to
 * bit-bang the WS2812 waveform.
 */
inline void LED_Init() {
    // ---- GPIO: PC6 = plain output, open-drain, very high speed ----
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;      // Clock GPIOC

    GPIOC->MODER &= ~(3U << (6 * 2));
    GPIOC->MODER |=  (1U << (6 * 2));         // General-purpose output mode
    GPIOC->OTYPER |= GPIO_OTYPER_OT6;         // OPEN-DRAIN: high level comes from external pull-up
    GPIOC->OSPEEDR |= (3U << (6 * 2));        // Very high speed for sharp edges
    GPIOC->PUPDR &= ~(3U << (6 * 2));         // No internal pull (pull-up is external)

    GPIOC->BSRR = GPIO_BSRR_BR6;              // Drive LOW: idle / reset state (LEDs dark)

    // ---- DWT cycle counter: bit-level time base ----
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;  // Enable DWT access
    DWT->CYCCNT = 0;                          // Reset the cycle counter
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;      // Start counting CPU cycles

    // ---- Convert the nominal nanosecond timings into CPU cycles ----
    // Ceiling division keeps every generated pulse at or above the nominal
    // value; at 170 MHz: T0H=51, T1H=111, bit=221 cycles (all inside the
    // datasheet windows with margin).
    const uint64_t hz = (uint64_t)SystemCoreClock;  // CPU frequency in Hz
    ws2812_tck_t0h = (uint32_t)((hz * WS2812_T0H_NS + 999999999ULL) / 1000000000ULL);  // Cycles for T0H
    ws2812_tck_t1h = (uint32_t)((hz * WS2812_T1H_NS + 999999999ULL) / 1000000000ULL);  // Cycles for T1H
    ws2812_tck_bit = (uint32_t)((hz * WS2812_BIT_NS + 999999999ULL) / 1000000000ULL);  // Cycles per bit
}

// ==============================================================================
// FRAME TRANSMISSION (bit-banged)
// ==============================================================================

/**
 * @brief Blocking transmission of one frame (both LEDs) by direct GPIO
 *        manipulation, timed on the DWT cycle counter.
 *
 * Algorithm:
 * 1. Enforce the WS2812B reset time: the line must have been LOW for
 *    >= WS2812_RESET_US since the previous frame ended (bounded spin,
 *    at most one reset period).
 * 2. Pack both pixels into a 48-bit shift register (GRB, MSB first,
 *    24 bits per LED).
 * 3. Disable interrupts (a full frame takes only ~62 us) and emit each
 *    bit: release the pin (external pull-up drives HIGH) for T0H/T1H,
 *    then pull it LOW until the end of the bit period. The DWT cycle
 *    counter provides cycle-accurate timing; the 32-bit counter wraps
 *    every ~25 s, which unsigned subtraction handles transparently.
 * 4. Re-enable interrupts, stamp the frame-end time, count the frame.
 *
 * Complexity: O(1) per frame (48 bits), ~1 ms worst case including the
 * reset-time wait. There is no state machine and nothing to arm, so
 * there is nothing that can silently break - this is intentionally the
 * least clever implementation possible.
 */
inline void LED_Show() {
    // Reset-time guard: after the previous frame the line must stay LOW
    // for >= WS2812_RESET_US before new bits are latched by the LEDs.
    // Bounded spin: at most one reset period (~300 us).
    while (((uint32_t)(micros() - ws2812_frame_end_us)) < WS2812_RESET_US) {
    }

    // Pack both pixels, 24 GRB bits each: G[7:0], R[7:0], B[7:0], MSB first
    uint64_t frame = 0;                                   // 48-bit shift register (2 x 24 bits)
    for (uint8_t led = 0; led < WS2812_LED_COUNT; led++) {
        const uint8_t grb[3] = {ws2812_rgb[led][1], ws2812_rgb[led][0], ws2812_rgb[led][2]};  // RGB -> GRB order
        for (uint8_t byte = 0; byte < 3; byte++) {        // Three bytes per LED
            for (uint8_t mask = 0x80U; mask != 0U; mask >>= 1) {  // MSB first
                frame = (frame << 1) | ((grb[byte] & mask) != 0U);  // Append one bit
            }
        }
    }

    noInterrupts();                            // Atomic frame: ~62 us with IRQs off
    const uint32_t t0h  = ws2812_tck_t0h;      // Cache timings (single-cycle reads)
    const uint32_t t1h  = ws2812_tck_t1h;
    const uint32_t tbit = ws2812_tck_bit;

    for (uint8_t i = 48; i != 0U; i--) {       // 48 bits, MSB first
        const bool bit = ((frame >> 47) & 1ULL) != 0ULL;  // Current bit (MSB of the register)
        frame <<= 1;                           // Shift the stream left
        const uint32_t t0 = DWT->CYCCNT;       // Bit period start stamp
        GPIOC->BSRR = GPIO_BSRR_BS6;           // Release the pin: external pull-up drives HIGH
        if (bit) {                             // "1": long high pulse
            while ((DWT->CYCCNT - t0) < t1h) {
            }
        } else {                               // "0": short high pulse
            while ((DWT->CYCCNT - t0) < t0h) {
            }
        }
        GPIOC->BSRR = GPIO_BSRR_BR6;           // Pull LOW for the rest of the bit period
        while ((DWT->CYCCNT - t0) < tbit) {
        }
    }
    interrupts();                              // Frame done, restore the interrupt context

    ws2812_frame_end_us = micros();            // Reset-time guard reference for the next frame
    ws2812_tc_count++;                         // Diagnostics: completed frame counter
}

/**
 * @brief Set the shadow color of BOTH LEDs (RGB order, 0-255 each) and
 *        send the frame.
 */
inline void LED_SetRGB(uint8_t r, uint8_t g, uint8_t b) {
    ws2812_rgb[0][0] = r;  // LED1: Red
    ws2812_rgb[0][1] = g;  // LED1: Green
    ws2812_rgb[0][2] = b;  // LED1: Blue
    ws2812_rgb[1][0] = r;  // LED2: Red
    ws2812_rgb[1][1] = g;  // LED2: Green
    ws2812_rgb[1][2] = b;  // LED2: Blue
    LED_Show();            // Push the new color to the LEDs
}

/**
 * @brief Set the shadow colors of the two LEDs independently (RGB order,
 *        0-255 each) and send the frame.
 */
inline void LED_SetRGB2(uint8_t r1, uint8_t g1, uint8_t b1,
                        uint8_t r2, uint8_t g2, uint8_t b2) {
    ws2812_rgb[0][0] = r1;  // LED1: Red
    ws2812_rgb[0][1] = g1;  // LED1: Green
    ws2812_rgb[0][2] = b1;  // LED1: Blue
    ws2812_rgb[1][0] = r2;  // LED2: Red
    ws2812_rgb[1][1] = g2;  // LED2: Green
    ws2812_rgb[1][2] = b2;  // LED2: Blue
    LED_Show();             // Push the new colors to the LEDs
}

/** @brief Turn both LEDs off. */
inline void LED_Off() {
    LED_SetRGB(0, 0, 0);
}

#endif  // WS2812_CLN234_H

