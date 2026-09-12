/**
 * @file board.h
 * @brief The contract between main() and whichever board is being built.
 *
 * This is deliberately the smallest header in the tree. main.cpp includes only
 * this, never a board's peripheral headers, which is what keeps the board's
 * interrupt vectors, mutable globals and drive object confined to a single
 * translation unit (bsp/boards/<board>/board.cpp).
 *
 * The active board is selected by the include path (-Isrc/bsp/boards/<board>)
 * and the source filter, both set per environment in platformio.ini. Nothing
 * here names a board, so main.cpp is genuinely board-agnostic.
 *
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#pragma once

/**
 * @brief Bring the board up: clocks, peripherals, protection limits,
 *        calibration and, where the board has one, the communication stack.
 *
 * Called once from setup(). Blocking waits inside are bounded, so a drive that
 * cannot calibrate still reaches the console rather than hanging.
 */
void board_setup();

/**
 * @brief The board application body, called from the Arduino main loop.
 *
 * Runs in THREAD mode. Control is interrupt-driven and does not depend on this
 * being called at any particular rate.
 */
void board_loop();
