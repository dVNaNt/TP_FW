#ifndef RUN_CLN234_H
#define RUN_CLN234_H

/**
 * @file run.h
 * @brief CLN234 application layer: run_setup() / run_loop().
 *
 * Full motor-stack application:
 *
 * - run_setup(): arms the SW1 hold-to-bootloader button, waits out the
 *   power-rail/USB stabilization window (polling SW1 meanwhile), prints the
 *   console banner, brings up the full motor stack (tunepulse_init(),
 *   including both blocking calibrations) and the CAN-FD link.
 * - run_loop(): pumps the console <-> CAN-FD chunk bridge (same as the
 *   CLN17 v3 application) and prints live MT6835 telemetry every 500 ms.
 *   The telemetry reads ONLY the registers published by the encoder DMA ISR
 *   (angle / status / CRC), so it never steals the SPI1 bus from the
 *   live FOC loop.
 *
 * The SPI1 bus is owned exclusively by the MT6835 encoder (DMA angle
 * polling). The DRV8353 SPI primitives are kept in spi_setup.h but are
 * NOT used for now: hardware OC/UVLO/OTSD protection is delegated to the
 * gate driver itself and reported through nFAULT (PB1), and the gate
 * driver runs on its hardware-default configuration (DRV_EN = HIGH).
 * Fast per-phase overcurrent and bus overvoltage shutdown are handled by
 * the MCU comparators (comp_setup.h), wired exactly like on CLN17 v3.
 *
 * main.cpp only prepares the USB console and calls these two entry points.
 * The full board integration (TP object, ISR glue, callbacks, motor-stack
 * bring-up) lives in tunepulse.h next to this file.
 *
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#include <Arduino.h>

#include "spi_setup.h"  // MT6835 encoder API (angle/status/CRC accessors)
#include "tunepulse.h"  // Board layer: tunepulse_init(), BOOT button, TP

/** Period of the live MT6835 telemetry line [ms]. */
static constexpr uint32_t TELEMETRY_PERIOD_MS = 500;

/** Cumulative MT6835 CRC-8 validated frame count (link-health indicator). */
static uint32_t crc_ok_count = 0;

/** Cumulative MT6835 CRC-8 failure count (bus-noise indicator). */
static uint32_t crc_errs = 0;

/** Last telemetry dispatch timestamp [ms]. */
static uint32_t telemetry_t0 = 0;

/** CAN identifier used for peer-to-peer console text. */
static constexpr uint32_t MSG_ID = 0x123;

/** Serial line assembly limit (console -> chunk staging). */
static constexpr uint16_t TEXT_LINE_MAX = 120;

/** Serial line assembly buffer (console -> chunk staging). */
static char line_buf[TEXT_LINE_MAX];

/** Characters currently stored in the line buffer. */
static uint16_t line_len = 0;

/**
 * @brief One-time initialization: bootloader button, motor stack, CAN-FD.
 */
inline void run_setup() {
    SerialUSB.begin();  // USB CDC console (baud ignored by CDC); main.cpp no
                        // longer does this - see the note on setup() there.
    // The USB stack installs its interrupts at priority 1, above the control
    // pass; a background service must not preempt the loop (see CLN17 V3).
    NVIC_SetPriority(USB_HP_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 4, 0));
    NVIC_SetPriority(USB_LP_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 4, 0));
    BOOT::init_button();  // SW1 (PC14) -> 3 s hold enters the USB DFU bootloader

    // Power-rail / USB stack stabilization; SW1 is polled meanwhile, so
    // holding it during power-up counts towards the 3 s bootloader trigger.
    const uint32_t t0 = millis();
    while ((millis() - t0) < 2000U) {
        BOOT::poll();  // may never return (enters USB DFU bootloader)
        delay(BOOT::POLL_INTERVAL_MS);
    }

    SerialUSB.println("=== CLN234 motor stack bring-up ===");

    // Full motor-stack bring-up: peripherals, protection limits, callbacks,
    // current-offset calibration and motor angle calibration (blocking until
    // both complete, SW1 still polled for the bootloader escape hatch).
    tunepulse_init();

    // CAN bring-up: fixed link configuration (FD-BRS, 1 Mbps / 5 Mbps, NORMAL).
    // The peripheral itself was NOT initialized inside tunepulse_init().
    if (!CAN_BUS.begin()) {
        SerialUSB.println("CAN init FAILED");
    }

    telemetry_t0 = millis();  // Start the telemetry pacer
}
/**
 * @brief Main loop: console <-> CAN chunk bridge + live MT6835 telemetry.
 */
inline void run_loop() {
    BOOT::poll();  // hold SW1 for 3 s to enter the USB bootloader

    // --- Console -> CAN: send complete lines as whole messages --------------
    while (SerialUSB.available() > 0) {          // Drain pending console bytes
        const int b = SerialUSB.read();          // Fetch one byte
        if (b < 0) {
            break;                               // Buffer drained
        }
        const char c = static_cast<char>(b);     // Interpret as character
        if ((c == '\n') || (c == '\r')) {        // Line complete
            if (line_len > 0) {
                line_buf[line_len++] = '\n';     // Line terminator byte
                line_buf[line_len] = '\0';       // Terminate for logging only
                CAN_BUS.write(MSG_ID, line_buf, line_len);  // All chunks queued
                line_len = 0;                    // Restart the line buffer
            }
        } else if (line_len < (TEXT_LINE_MAX - 1U)) {
            line_buf[line_len++] = c;            // Append in-range character
        } else {
            line_len = 0;                        // Overflow: drop the line
        }
    }

    // --- CAN -> console: echo received chunks until a newline byte ---------
    CAN_BUS.update();                        // Pump TX frame ring -> HW FIFO
    uint8_t chunk[CANDriver::CHUNK_SIZE];    // Received chunk storage
    bool line_done = false;                  // Newline encountered flag
    while (CAN_BUS.is_rx_available()) {      // Drain the whole RX queue
        uint32_t cid = 0;                    // Chunk identifier (unused)
        if (!CAN_BUS.read(cid, chunk)) {
            break;                           // Queue drained
        }
        for (uint8_t i = 0; i < CANDriver::CHUNK_SIZE; i++) {  // Echo bytes
            const uint8_t c = chunk[i];      // One payload byte
            if (c == '\n') {                 // Line terminator
                line_done = true;            // End of the console line
                break;                       // Newline inside this chunk
            }
            if (c != 0U) {                   // Skip zero padding
                SerialUSB.write(c);          // Echo one byte
            }
        }
        if (line_done) {
            SerialUSB.println();             // Terminate the console line
            line_done = false;               // Continue with the next line
        }
        (void)cid;                           // ID unused in this demo
    }
}

#endif  // RUN_CLN234_H