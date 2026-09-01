/**
 * @file main.cpp
 * @brief Firmware entry point for the TunePulse project.
 *
 * Demo application: console text is written over CAN as 8-byte command
 * chunks in a single write() call (packed into CAN-FD frames); received
 * chunks are echoed to the console until a newline byte arrives. Both
 * boards run the identical firmware.
 *
 * All CAN handling lives in can_setup.h (board layer) and can_control.h
 * (chunk/command layer); main.cpp only glues SerialUSB to CAN_CTRL.
 *
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#include <Arduino.h>

#include "target/cln17_v3/can_setup.h"  // Merged CAN driver + chunk API

#include "tunepulse.h"    // Board bring-up helpers (BOOT button polling)

/** CAN identifier used for peer-to-peer console text. */
static constexpr uint32_t MSG_ID = 0x123;

/** Serial line assembly limit (console -> chunk staging). */
static constexpr uint16_t TEXT_LINE_MAX = 120;

/** Serial line assembly buffer (console -> chunk staging). */
static char line_buf[TEXT_LINE_MAX];

/** Characters currently stored in the line buffer. */
static uint16_t line_len = 0;

/**
 * @brief One-time initialization: USB console, then the CAN-FD interface.
 */
void setup() {
    BOOT::init_button();

    // Power-rail / USB stack stabilization; SW1 is polled meanwhile, so
    // holding it during power-up counts towards the 3 s bootloader trigger.
    const uint32_t t0 = millis();
    while ((millis() - t0) < 2000U) {
        BOOT::poll();  // may never return (enters USB DFU bootloader)
        delay(BOOT::POLL_INTERVAL_MS);
    }

    SerialUSB.begin();
    SerialUSB.println("=== TunePulse CAN chunk bridge (1 Mbps / 5 Mbps FD) ===");
    tunepulse_init();
    if (!CAN_BUS.begin()) {
        SerialUSB.println("CAN init FAILED");
    }
}

/**
 * @brief Main loop: console text -> write() -> CAN, received chunks -> console.
 */
void loop() {
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