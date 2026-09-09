/**
 * @file main.cpp
 * @brief Firmware entry point for the TunePulse project (board-agnostic).
 *
 * main() knows nothing about any board. It includes bsp/board.h, which declares
 * two functions, and the active board's board.cpp defines them. The board is
 * chosen entirely by platformio.ini: -Isrc/bsp/boards/<board> resolves the
 * header, and the per-environment source filter compiles exactly one board.cpp.
 *
 * Keeping this file free of the board's peripheral headers is what allows the
 * board to own its interrupt vectors and globals in a translation unit of its
 * own. Everything that must exist exactly once in the image lives there.
 *
 * USB bring-up belongs to the board, not here: CLN17 v3 enumerates a COMPOSITE
 * device (CDC console plus an HID control pipe) and performs USBD_Init() itself,
 * so it replaces SerialUSB.begin() rather than following it. Each board opens
 * its console the way its own hardware needs.
 *
 * Supported boards (platformio.ini environments):
 * - cln17_v3_release : CLN17 v3 (STM32G431CB) - FOC stack plus the TunePulse
 *                      CAN Protocol over FDCAN1 and a composite USB HID pipe.
 * - cln234_release   : CLN234 (STM32G474CE) - full FOC stack; the SPI1 bus is
 *                      shared between the MT6835 encoder and the DRV8353 gate
 *                      driver.
 *
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#include <Arduino.h>

#include "bsp/board.h"  // board_setup() / board_loop(), defined by the active board

/**
 * @brief Arduino entry point: hand straight over to the board.
 */
void setup() {
    board_setup();  // Clocks, peripherals, calibration, comms (bsp/boards/<board>/board.cpp)
}

/**
 * @brief Arduino main loop: the board application body.
 */
void loop() {
    board_loop();  // Service comms and diagnostics; control runs from interrupts
}
