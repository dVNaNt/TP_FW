/**
 * @file ws2812.h
 * @brief Interrupt-safe zero-CPU WS2812 driver for CLN234 (TIM8_CH1 + DMA).
 *
 * Drives two daisy-chained WS2812B LEDs on PC6 (TIM8_CH1, AF4). The pin is
 * an ALTERNATE-FUNCTION OPEN-DRAIN output: LOW is produced by the on-chip
 * N-MOS, HIGH comes from the external pull-up to 5 V (the MCU runs at
 * 3.3 V). Hardware behavior verified on the board during bring-up:
 *
 * - PWM MODE 1 (OC1M = 110), CC1P = 0 + open-drain: OCREF = 1 while
 *   CNT < CCR, and OCREF = 1 means the open-drain transistor is RELEASED
 *   (line HIGH). Therefore the line is HIGH for exactly CCR ticks starting
 *   at each update event, then LOW - precisely the WS2812 bit format.
 *   Idle: CCR = 0 -> OCREF always 0 -> transistor always ON -> line LOW.
 *   (PWM mode 2 is the complement and produces an always-HIGH idle with
 *   inverted low-first bits - it breaks every WS2812 latch.)
 *
 * IRQ-friendly data path (designed for LED_Update() called at up to 20 kHz):
 * 1. LED_SetRGB/LED_SetRGB2/LED_Off only write the shadow colors, set the
 *    pending flag and invalidate the packed buffer - a couple of cycles,
 *    callable from any context.
 * 2. LED_Update() is a flat state machine with no spin loops:
 *    - hot path (nothing pending): one flag read + return, ~5 cycles;
 *    - frame in flight: one CNDTR read + return, ~10 cycles;
 *    - completion: timestamp the moment (the line is now LOW);
 *    - cooling: pack the fresh colors once into the buffer (~100 cycles);
 *    - reset elapsed (>= 300 us since the timestamp): reload the DMA
 *      (~30 cycles) and the frame goes out.
 *    No interrupt disabling, no busy waits, worst case < 1 us.
 * 3. The WS2812B reset latch (line LOW >= 280 us) is enforced by a
 *    timestamp taken when the transfer counter reaches 0 - that is the
 *    moment the line goes LOW. The timestamp is always taken AFTER the
 *    actual low-start (poll latency), so waiting 300 us from it never
 *    violates the spec. The buffer carries just ONE trailing idle entry
 *    (CCR = 0) to park the line LOW after the last bit - 98 bytes of RAM
 *    instead of a 562-byte idle tail.
 * 4. Stream self-start: after a frame the shadow CCR holds 0, so the
 *    compare event at CNT == 0 pulls the first word from the new buffer
 *    on the next update event - no priming, no UIF polling, no races
 *    (in both possible DMA-latency outcomes the stream stays in order).
 *    One DMA request per compare event then streams the whole frame: DMA
 *    writes the next compare value into the CCR1 preload register
 *    (OC1PE = 1), latched exactly at the next update event, so every bit
 *    occupies exactly one full 1.3 us period.
 *
 * G4 classic DMA specifics (verified on hardware):
 * - The EN bit does NOT auto-clear at transfer complete (CNDTR reaches 0
 *   and TCIF is set while EN stays 1). Every re-arm performs
 *   disable -> reload -> enable; with an exhausted counter the disable
 *   completes in a couple of bus cycles.
 * - Requests arriving while EN = 1 / CNDTR = 0 are safely ignored
 *   (verified: the channel sat in that state for seconds without damage).
 *
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#ifndef WS2812_CLN234_H
#define WS2812_CLN234_H

#include <Arduino.h>              // micros() and basic Arduino types

#include "stm32g4xx.h"            // CMSIS device header (TIM8, DMA1, GPIOC)
#include "stm32g4xx_ll_bus.h"     // LL bus clock enable helpers
#include "stm32g4xx_ll_dma.h"     // LL DMA configuration helpers
#include "stm32g4xx_ll_dmamux.h"  // LL DMAMUX request routing helpers

// ==============================================================================
// TIMING CONSTANTS (nominal nanosecond values; tick counts are derived at
// runtime from SystemCoreClock so the driver also works on the 16 MHz HSI)
// ==============================================================================
constexpr uint32_t WS2812_T0H_NS    = 300;     // "0" high time (spec 220-380 ns)
constexpr uint32_t WS2812_T1H_NS    = 650;     // "1" high time (spec 580-1000 ns)
constexpr uint32_t WS2812_BIT_NS    = 1300;    // Total bit period (spec >= 1250 ns)
constexpr uint8_t  WS2812_LED_COUNT = 2;       // Two WS2812B LEDs daisy-chained on PC6
constexpr uint32_t WS2812_RESET_US  = 300;     // Latch: line idle LOW > 280 us per spec
constexpr uint32_t WS2812_IDLE_CCR  = 0x0000;  // PWM mode 1: CCR = 0 -> OCREF low -> line LOW

constexpr uint32_t WS2812_FRAME_BITS   = WS2812_LED_COUNT * 24U;          // Payload bits per frame (48)
constexpr uint32_t WS2812_BUF_SIZE     = WS2812_FRAME_BITS + 1U;          // 48 bits + 1 trailing "line LOW" (98 bytes of RAM)

// ==============================================================================
// STATE (all in SRAM; the DMA buffer IS the waveform)
// ==============================================================================
inline volatile uint8_t ws2812_rgb[WS2812_LED_COUNT][3] = {{0, 0, 0}, {0, 0, 0}};  // Shadow colors (RGB order, per LED)
inline volatile uint8_t ws2812_pending = 0;               // New colors waiting for the next frame
inline volatile uint8_t ws2812_packed = 0;                // Buffer holds the current pending colors
inline volatile uint32_t ws2812_low_us = 0;               // Stamp of the last observed line-LOW moment (0 = none)
inline volatile uint32_t ws2812_tc_count = 0;             // Completed frames (diagnostics)
inline uint16_t ws2812_dma_buf[WS2812_BUF_SIZE];          // CCR1 compare values: bits + parking entry (RAM)
inline uint32_t ws2812_tck_t0h = 0;                       // "0" high time in timer ticks (CCR value)
inline uint32_t ws2812_tck_t1h = 0;                       // "1" high time in timer ticks (CCR value)
inline uint32_t ws2812_tck_bit = 0;                       // Full bit period in timer ticks (ARR + 1)

/**
 * @brief Convert a nominal nanosecond duration into timer ticks (rounded).
 * @param ns Duration in nanoseconds.
 * @return Nearest number of timer ticks at the current timer clock.
 */
inline uint32_t ws2812_ns_to_ticks(uint32_t ns) {
    // (f_MHz * ns + 500) / 1000 == round(f_tick * ns / 1e9) without 64-bit math
    return ((SystemCoreClock / 1000000U) * ns + 500U) / 1000U;
}

// ==============================================================================
// INITIALIZATION
// ==============================================================================

/**
 * @brief Initialize the WS2812 output: PC6 = TIM8_CH1 (AF4) open-drain +
 *        TIM8 PWM mode 1 + DMA1_Channel2.
 *
 * Clocks: GPIOC on AHB2 (170 MHz), TIM8 on APB2 (170 MHz, prescaler DIV1),
 * DMA1/DMAMUX1 on AHB1 (170 MHz) - all enabled here explicitly.
 *
 * - GPIO : PC6 alternate function 4 (TIM8_CH1, verified against the
 *          variant PeripheralPins.c), open-drain, very high speed.
 * - TIM8 : PSC=0, ARR = bit_ticks-1 (1.3 us period), CH1 in PWM mode 1
 *          with CCR preload. Mode 1 + open-drain: line HIGH for CCR ticks
 *          after each update event, LOW for the rest. CCR1 = 0 -> line LOW
 *          (reset/idle state, both LEDs dark).
 * - DMA1_Channel2 (free on this board: CH1 = encoder SPI, CH5 = ADC):
 *          memory->peripheral, 16-bit, incrementing, request = TIM8_CH1
 *          (one request per compare event = one per bit period). No IRQs.
 *
 * After init the line idles LOW and the stream self-starts: the compare
 * event at CNT == 0 (shadow CCR = 0) pulls the first word on the next
 * update event once LED_Update() reloads the transfer counter.
 */
inline void LED_Init() {
    // ---- Derived tick counts (also used by the DMA buffer packer) ----
    ws2812_tck_t0h = ws2812_ns_to_ticks(WS2812_T0H_NS);  // "0" pulse length in ticks
    ws2812_tck_t1h = ws2812_ns_to_ticks(WS2812_T1H_NS);  // "1" pulse length in ticks
    ws2812_tck_bit = ws2812_ns_to_ticks(WS2812_BIT_NS);  // Full bit period in ticks

    // ---- GPIO: PC6 = TIM8_CH1 (AF4), open-drain, very high speed ----
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;      // Clock GPIOC
    GPIOC->MODER &= ~(3U << (6 * 2));         // Clear PC6 mode
    GPIOC->MODER |= (2U << (6 * 2));          // Alternate function mode
    GPIOC->OTYPER |= GPIO_OTYPER_OT6;         // OPEN-DRAIN: logic 1 = released (pull-up)
    GPIOC->OSPEEDR |= (3U << (6 * 2));        // Very high speed for sharp edges
    GPIOC->PUPDR &= ~(3U << (6 * 2));         // No internal pull (pull-up is external)
    GPIOC->AFR[0] &= ~(0xFU << (6 * 4));      // Clear PC6 alternate function
    GPIOC->AFR[0] |= (4U << (6 * 4));         // AF4 = TIM8_CH1

    // ---- DMA1_Channel2: CCR1 waveform stream (request = TIM8_CH1) ----
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMAMUX1EN;  // Clock DMA1 + DMAMUX
    LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_2,
                                    LL_DMA_DIRECTION_MEMORY_TO_PERIPH);  // RAM -> CCR1
    LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PRIORITY_HIGH);  // Above average bus load
    LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MODE_NORMAL);  // One-shot: stops after the frame
    LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PERIPH_NOINCREMENT);  // Fixed CCR1 address
    LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MEMORY_INCREMENT);    // Walk the bit buffer
    LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PDATAALIGN_HALFWORD);    // CCR1 is 16-bit
    LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MDATAALIGN_HALFWORD);    // Buffer entries are 16-bit
    LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_2, LL_DMAMUX_REQ_TIM8_CH1);     // One request per compare event
    LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_2, (uint32_t)&TIM8->CCR1);      // Destination: compare register
    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_2);                               // Idle until LED_Update()
    LL_DMA_ClearFlag_GI2(DMA1);                                                  // Clear stale completion flags

    // ---- TIM8: 1.3 us period, CH1 = PWM mode 1 + preload, DMA on compare ----
    RCC->APB2ENR |= RCC_APB2ENR_TIM8EN;       // Clock TIM8 (APB2 = 170 MHz, x1)
    TIM8->CR1 = 0;                            // Stop the counter while configuring
    TIM8->PSC = 0;                            // No prescaler: tick = 1 / 170 MHz
    TIM8->ARR = ws2812_tck_bit - 1U;          // Bit period = ARR + 1 ticks (1.3 us)
    // PWM MODE 1 (OC1M = 110) + CCR1 preload enable (OC1PE): DMA writes are
    // latched into the shadow register exactly at the update event, so every
    // bit occupies exactly one full timer period.
    TIM8->CCMR1 = TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1PE;
    TIM8->CCER = TIM_CCER_CC1E;               // Enable CH1 output, active-high polarity
    TIM8->BDTR = TIM_BDTR_MOE;                // Advanced timer: master output enable required
    TIM8->DIER = TIM_DIER_CC1DE;              // DMA request on CH1 compare event (no interrupts)
    TIM8->CCR1 = WS2812_IDLE_CCR;             // CCR = 0: OCREF low -> line LOW (reset state)
    TIM8->EGR = TIM_EGR_UG;                   // Force an update: load shadow registers now
    TIM8->CR1 = TIM_CR1_CEN;                  // Start the counter (runs forever)
}

// ==============================================================================
// RUNTIME
// ==============================================================================

/**
 * @brief Pack the shadow colors into the CCR1 DMA buffer.
 *
 * Layout: 48 WS2812 bits (MSB-first, WS2812 GRB order: G[7:0], R[7:0],
 * B[7:0] per LED) followed by ONE idle entry (CCR = 0) that parks the line
 * LOW after the last bit. The reset latch between frames is enforced by
 * the timestamp in LED_Update(), not by buffer padding.
 *
 * Cost: ~100 cycles per call - runs at most twice per frame cycle
 * (early pack during the cooling window + optional refresh if the colors
 * changed since).
 */
inline void ws2812_pack() {
    const uint16_t ccr_lut[2] = {                             // Bit value -> CCR pulse length
        (uint16_t)ws2812_tck_t0h, (uint16_t)ws2812_tck_t1h};  // "0" and "1" high times
    uint32_t idx = 0;                                         // Buffer write cursor
    for (uint8_t led = 0; led < WS2812_LED_COUNT; led++) {    // Two pixels per frame
        const uint8_t grb[3] = {ws2812_rgb[led][1], ws2812_rgb[led][0],
                                ws2812_rgb[led][2]};          // RGB -> GRB byte order
        for (uint8_t byte = 0; byte < 3; byte++) {            // Three bytes per LED
            for (uint8_t mask = 0x80U; mask != 0U; mask >>= 1) {  // MSB first
                ws2812_dma_buf[idx++] = ccr_lut[(grb[byte] & mask) != 0U];  // One CCR entry per bit
            }
        }
    }
    ws2812_dma_buf[WS2812_FRAME_BITS] = (uint16_t)WS2812_IDLE_CCR;  // Parking entry: line LOW
}

/**
 * @brief IRQ-safe driver tick: timestamp completions, pre-pack during the
 *        cooling window and start a frame once the reset latch has elapsed.
 *        Call at any rate (e.g. 20 kHz control IRQ).
 *
 * Flat state machine, no spin loops, no interrupt disabling:
 * - hot path (nothing pending): one flag read + return, ~5 cycles;
 * - frame in flight: one CNDTR read + return, ~10 cycles;
 * - completion: one micros() stamp (~10 cycles), the line is now LOW;
 * - cooling: pack the fresh colors once (~100 cycles), then no-ops;
 * - reset elapsed: pack-if-stale + DMA reload (~130 cycles), frame goes out.
 *
 * The reset latch uses the completion timestamp: it is always taken AFTER
 * the actual line-LOW moment (poll latency), so a 300 us wait from it
 * never violates the WS2812B spec.
 *
 * The stream self-starts: with the shadow CCR at 0 (parking value from
 * the previous frame) the compare event at CNT == 0 pulls the first word
 * on the next update event, then one request per compare event streams
 * the whole frame.
 *
 * @return true  - a frame was armed;
 * @return false - nothing to do, cooling, or the previous frame is running.
 */
inline bool LED_Update() {
    if (ws2812_pending == 0U) {                 // Hot path: nothing to send
        return false;                           // ~5 cycles
    }
    if (DMA1_Channel2->CNDTR != 0U) {           // Frame still on the wire?
        return false;                           // Retry on a later tick (~10 cycles)
    }
    const uint32_t now = micros();              // One time read per tick
    if (ws2812_low_us == 0U) {                  // First observation of the idle DMA?
        ws2812_low_us = now;                    // Stamp: the line just went LOW
    }
    if ((now - ws2812_low_us) >= WS2812_RESET_US) {   // Reset latch satisfied?
        if (ws2812_packed == 0U) {              // Colors changed after the early pack
            ws2812_pack();                      // Refresh the buffer (~100 cycles)
            ws2812_packed = 1;
        }
        ws2812_pending = 0;                     // Consume the request

        // Re-arm: EN is still set after the previous frame (G4 classic DMA
        // never auto-clears it), so force-disable and wait for the release -
        // with an exhausted counter this completes in a couple of bus cycles.
        LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_2);
        while (LL_DMA_IsEnabledChannel(DMA1, LL_DMA_CHANNEL_2)) {
        }
        LL_DMA_ClearFlag_GI2(DMA1);                                     // Clear GIF/TCIF/HTIF/TEIF
        LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_2,
                                 (uint32_t)ws2812_dma_buf);             // Stream buf[0..48]
        LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_2, WS2812_BUF_SIZE);  // 49 transfers
        LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_2);                   // Arm the compare-event stream
        ws2812_low_us = 0;                      // Re-stamp at the next completion
        ws2812_packed = 0;                      // Buffer consumed
        ws2812_tc_count++;                      // Diagnostics: frame counter
        return true;                            // Frame is on the wire
    }
    if (ws2812_packed == 0U) {                  // Cooling window: spread the load
        ws2812_pack();                          // Prep the buffer off the hot arm path
        ws2812_packed = 1;
    }
    return false;                               // Still cooling - retry on a later tick
}

/**
 * @brief Set the shadow color of BOTH LEDs (RGB order, 0-255 each). The
 *        colors go out on the next LED_Update() tick (see the notes about
 *        torn frames in case of a preemption during packing).
 */
inline void LED_SetRGB(uint8_t r, uint8_t g, uint8_t b) {
    ws2812_rgb[0][0] = r;  // LED1: Red
    ws2812_rgb[0][1] = g;  // LED1: Green
    ws2812_rgb[0][2] = b;  // LED1: Blue
    ws2812_rgb[1][0] = r;  // LED2: Red
    ws2812_rgb[1][1] = g;  // LED2: Green
    ws2812_rgb[1][2] = b;  // LED2: Blue
    ws2812_pending = 1;    // Request a frame (non-blocking, IRQ-safe)
    ws2812_packed = 0;     // The packed buffer is now stale
}

/**
 * @brief Set the shadow colors of the two LEDs independently (RGB order,
 *        0-255 each). The colors go out on the next LED_Update() tick.
 */
inline void LED_SetRGB2(uint8_t r1, uint8_t g1, uint8_t b1, uint8_t r2,
                        uint8_t g2, uint8_t b2) {
    ws2812_rgb[0][0] = r1;  // LED1: Red
    ws2812_rgb[0][1] = g1;  // LED1: Green
    ws2812_rgb[0][2] = b1;  // LED1: Blue
    ws2812_rgb[1][0] = r2;  // LED2: Red
    ws2812_rgb[1][1] = g2;  // LED2: Green
    ws2812_rgb[1][2] = b2;  // LED2: Blue
    ws2812_pending = 1;     // Request a frame (non-blocking, IRQ-safe)
    ws2812_packed = 0;      // The packed buffer is now stale
}

/** @brief Turn both LEDs off (applied on the next LED_Update() tick). */
inline void LED_Off() {
    LED_SetRGB(0, 0, 0);
}

#endif  // WS2812_CLN234_H