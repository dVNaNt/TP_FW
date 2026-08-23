/**
 * @file main.cpp
 * @brief Firmware entry point for the TunePulse project.
 *
 * This file contains the Arduino-style program entry points:
 * - setup(): performs one-time hardware and subsystem initialization.
 * - loop():  the main infinite application loop.
 *
 * @note The actual motor-control stack is initialized by tunepulse_init()
 *       declared in "tunepulse.h". It is currently commented out because
 *       this build is used as a CAN communication test suite rather than
 *       a full motor-control firmware image.
 */

#include <Arduino.h>

#include "tunepulse.h"

/**
 * @brief One-time hardware initialization, called once by the Arduino core
 *        before the main loop starts.
 *
 * Execution flow:
 * 1. Waits 2000 ms to allow power rails, the USB stack and external
 *    peripherals to stabilize before any communication is attempted.
 * 2. Starts the native USB serial port (SerialUSB) used as the debug/log
 *    console.
 * 3. Prints a banner identifying this binary as the TunePulse CAN test
 *    suite. The full motor-driver initialization (tunepulse_init()) is
 *    intentionally disabled in this test configuration.
 */
void setup() {
    BOOT::init_button();

    // Original 2000 ms power-rail / USB stabilization delay; the button is
    // polled meanwhile, so holding SW1 already during power-up counts
    // towards the 3 s bootloader trigger.
    const uint32_t stabilize_start_ms = millis();
    while ((millis() - stabilize_start_ms) < 2000U) {
        BOOT::poll();  // may never return (enters USB DFU bootloader)
        delay(BOOT::POLL_INTERVAL_MS);
    }

    SerialUSB.begin();
    tunepulse_init();
}

/**
 * @brief Main application loop, called repeatedly by the Arduino core
 *        after setup() completes.
 *
 * Currently empty except for the SW1 (PC6) hold-to-bootloader poll: keeping
 * the button pressed for more than 3 seconds at any time de-initializes all
 * peripherals and jumps into the built-in USB DFU system bootloader.
 */
void loop() {
    BOOT::poll();  // hold SW1 for 3 s to enter the USB bootloader
}