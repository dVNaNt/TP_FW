#ifndef RUN_CLN17V3_H
#define RUN_CLN17V3_H

/**
 * @file run.h
 * @brief CLN17 v3 application layer: run_setup() / run_loop().
 *
 * main.cpp only prepares the USB console and calls these two entry points -
 * everything board-specific lives here (and in tunepulse.h, which owns the
 * peripheral bring-up, the protection limits and the board callbacks):
 *
 * - run_setup(): arms the SW1 hold-to-bootloader button, waits out the
 *   power-rail/USB stabilization window (polling SW1 meanwhile), prints the
 *   console banner, runs the full motor-stack bring-up (tunepulse_init(),
 *   including both blocking calibrations) and brings the CAN-FD link up.
 * - run_loop(): pumps the console <-> CAN-FD chunk bridge: complete serial
 *   lines are queued to CAN as 8-byte chunks, received chunks are echoed
 *   back to the console line-by-line.
 *
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#include <Arduino.h>

#include "tunepulse.h"  // Board bring-up: BOOT button, tunepulse_init(), TP
#include "can_setup.h"  // CAN_BUS chunk API (also pulled in via tunepulse.h)

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
    BOOT::init_button();  // SW1 (PC6) -> 3 s hold enters the USB DFU bootloader

    // Power-rail / USB stack stabilization; SW1 is polled meanwhile, so
    // holding it during power-up counts towards the 3 s bootloader trigger.
    const uint32_t t0 = millis();
    while ((millis() - t0) < 2000U) {
        BOOT::poll();  // may never return (enters USB DFU bootloader)
        delay(BOOT::POLL_INTERVAL_MS);
    }

    SerialUSB.println("=== TunePulse CAN chunk bridge (1 Mbps / 5 Mbps FD) ===");

    tunepulse_init();  // Peripherals + protection limits + both calibrations

    // CAN bring-up: fixed link configuration (FD-BRS, 1 Mbps / 5 Mbps, NORMAL).
    // The peripheral itself was NOT initialized inside tunepulse_init().
    if (!CAN_BUS.begin()) {
        SerialUSB.println("CAN init FAILED");
    }
}

/**
 * @brief Main loop: console text -> write() -> CAN, received chunks -> console.
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

#endif  // RUN_CLN17V3_H