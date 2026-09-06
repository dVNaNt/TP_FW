/**
 * @file main.cpp
 * @brief Firmware entry point for the TunePulse project (target-agnostic).
 *
 * main() only prepares the USB CDC console and delegates everything else to
 * the board layer of the ACTIVE target:
 *
 * - run.h       : run_setup() / run_loop() - the application body of the
 *                 active target. Resolved through the per-environment include
 *                 path (-Isrc/target/<board> set in platformio.ini).
 * - tunepulse.h : peripheral bring-up, protection limits and board callbacks
 *                 of the active target (lives next to run.h).
 *
 * Supported targets (platformio.ini environments):
 * - cln17_v3_release : CLN17 V3 (STM32G431CB) - FOC stack + CAN chunk bridge.
 * - cln234_release   : CLN234 (STM32G474CE) - full FOC stack; the SPI1 bus
 *                      serves the MT6835 encoder only (DRV8353 SPI
 *                      primitives kept but unused for now).
 *
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#include <Arduino.h>

#include "run.h"  // Target-specific run_setup()/run_loop() (-Isrc/target/<board>)

/**
 * @brief Arduino entry point: USB console up, then the board application.
 */
void setup() {
    SerialUSB.begin();  // USB CDC console (the baud rate is ignored by CDC)
    run_setup();        // Board bring-up (target/cln17_v3/run.h or cln234/run.h)
}

/**
 * @brief Arduino main loop: board application body.
 */
void loop() {
    run_loop();  // Board application body (target-specific)
}