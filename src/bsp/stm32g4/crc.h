/**
 * @file crc.h
 * @brief Hardware backend of core/math/crc.h on the STM32G4 CRC unit.
 *
 * Identical on every part in the family: the block has no board-dependent
 * wiring, so nothing here is parameterized. Compiled in only when TP_CRC_HW
 * is defined (platformio.ini); without it the lib layer shifts in software
 * and this header is never included.
 *
 * The G4 CRC unit is a programmable MSB-first shift register: any polynomial
 * of 7, 8, 16 or 32 bits (POL), any initial value (INIT), optional input and
 * output bit reversal (unused here) and no final XOR. Fed byte by byte (an
 * 8-bit write to DR shifts in exactly that byte) it computes precisely what
 * Crc::shift_byte() computes, which is what lets Crc::Stream switch backends
 * without changing a single stored CRC. A 32-bit write would shift the word
 * in big-endian order instead, so the byte access is deliberate.
 *
 * It is ONE peripheral with no ownership tracking: begin() reprograms it and
 * resets the register, so a stream that spans several control ticks (the
 * calibration table load reads one flash block per tick) must be the only
 * user until value() has been read. Every one-shot Crc::crc8/16/32 call stays
 * in software for that reason.
 *
 * Direct register access, no HAL: enabling the clock and three register
 * writes are the whole set-up, and the byte feed has to be a single store to
 * be worth anything next to the software loop.
 *
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#pragma once

#include "stm32g4xx.h"              // CMSIS: CRC, RCC register maps
#include "core/math/crc.h"   // The declarations this header defines (TP_CRC_HW set)

#ifdef TP_CRC_HW

/**
 * @brief Program the unit for one CRC and reset its register to the initial value.
 *
 * The AHB clock is enabled on every call: it is one read-modify-write, and it
 * removes any ordering requirement between this backend and the board's
 * clock set-up (the first stream may open inside the control ISR before any
 * thread-mode initialisation touched the block).
 *
 * @param width Register width in bits: 8, 16 or 32 (7 is not exposed).
 * @param poly  Polynomial, right-aligned.
 * @param init  Initial register value, right-aligned.
 */
inline void tp_crc_hw_begin(uint8_t width, uint32_t poly, uint32_t init) {
    RCC->AHB1ENR |= RCC_AHB1ENR_CRCEN;             // Clock the block (idempotent)
    (void)RCC->AHB1ENR;                            // Read back: the enable takes effect before the next access
    uint32_t polysize = 0u;                        // POLYSIZE 00 = 32 bits
    if (width == 16u) {
        polysize = CRC_CR_POLYSIZE_0;              // 01 = 16 bits
    } else if (width == 8u) {
        polysize = CRC_CR_POLYSIZE_1;              // 10 = 8 bits
    }
    CRC->POL = poly;                               // Polynomial (must be written before RESET)
    CRC->INIT = init;                              // Value loaded into DR by RESET
    CRC->CR = polysize | CRC_CR_RESET;             // No reversal, load INIT, select the width
}

/**
 * @brief Shift bytes into the unit in memory order.
 * @param data Bytes.
 * @param len  Byte count.
 */
inline void tp_crc_hw_feed(const uint8_t* data, size_t len) {
    volatile uint8_t* dr = (volatile uint8_t*)&CRC->DR;  // 8-bit access: one byte per write
    for (size_t i = 0; i < len; ++i) {
        *dr = data[i];                             // Each store advances the register by eight bits
    }
}

/**
 * @brief Current remainder, right-aligned to the programmed width.
 *
 * DR reads back the register; for the 8- and 16-bit polynomials the unit
 * keeps only the low bits meaningful, and the mask makes that explicit.
 */
inline uint32_t tp_crc_hw_value() {
    const uint32_t polysize = CRC->CR & CRC_CR_POLYSIZE_Msk;  // Width the unit was programmed for
    const uint32_t dr = CRC->DR;                               // Remainder so far
    if (polysize == CRC_CR_POLYSIZE_1) {
        return dr & 0xFFu;                         // 8-bit register
    }
    if (polysize == CRC_CR_POLYSIZE_0) {
        return dr & 0xFFFFu;                       // 16-bit register
    }
    return dr;                                     // 32-bit register
}

#endif  // TP_CRC_HW
