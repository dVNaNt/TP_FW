/**
 * @file crc.h
 * @brief Cyclic redundancy checks: CRC-8 / CRC-16 / CRC-32 with a software
 *        implementation and an optional hardware backend behind one switch.
 *
 * One algorithm family for the whole firmware, so an encoder frame, a stored
 * record and a serial packet are checked the same way: MSB-first (non
 * reflected) shift-register CRC with a programmable polynomial and initial
 * value, no input or output reversal and no final XOR. Those are exactly the
 * parameters of the STM32 CRC unit, which is what lets the hardware backend
 * replace the software one bit-for-bit:
 *
 * | Name (width)      | Polynomial   | Init         | Used for                       |
 * |-------------------|--------------|--------------|--------------------------------|
 * | CRC-8 (0x07)      | x^8+x^2+x+1  | 0x00         | MT6835 angle burst frame       |
 * | CRC-16/CCITT-FALSE| 0x1021       | 0xFFFF       | short records, serial packets  |
 * | CRC-32/MPEG-2     | 0x04C11DB7   | 0xFFFFFFFF   | calibration table record       |
 *
 * Two ways to use it:
 *
 *  - One-shot: Crc::crc8() / crc16() / crc32() over a byte buffer. Always the
 *    software implementation and constexpr, so a fixed frame can be checked at
 *    compile time and the functions are safe from any context (a 20 kHz
 *    interrupt included - three bytes are 24 shift steps).
 *  - Streaming: Crc::Stream, for data that arrives a few words at a time
 *    (a flash record read one block per control tick). This is where the
 *    hardware backend plugs in: with TP_CRC_HW defined the stream feeds the
 *    MCU's CRC unit through three functions the BSP provides
 *    (tp_crc_hw_begin / feed / value, see bsp/stm32g4/crc.h), otherwise it
 *    shifts in software. The unit is ONE shared peripheral with no locking:
 *    only one Stream may be open at a time, and a stream that spans several
 *    ticks must not be interleaved with another user. Every one-shot call
 *    stays in software for that reason, so it can never trample a stream.
 *
 * Byte order: a multi-byte word is fed as its little-endian bytes (memory
 * order), which is also how the hardware unit sees a word written byte by
 * byte. Streams and the one-shots agree on this, so a CRC written by one can
 * be verified by the other.
 *
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#ifndef TP_MATH_CRC_H
#define TP_MATH_CRC_H

#include <stddef.h>  // size_t
#include <stdint.h>  // Fixed-width integer types

// Backend selection. Software is the default; a board that defines TP_CRC_HW
// (platformio.ini) provides the three hook functions below and the streaming
// context runs on its CRC peripheral instead. Declared inline here and
// defined inline in the BSP (bsp/stm32g4/crc.h): the firmware is one
// translation unit per board, so the definition follows in the same unit and
// the lib layer names no peripheral.
#ifndef TP_CRC_HW
#define TP_CRC_SOFTWARE 1  ///< Crc::Stream shifts in software.
#else
/// Program the unit for a width (8 / 16 / 32), polynomial and initial value, and reset it.
inline void tp_crc_hw_begin(uint8_t width, uint32_t poly, uint32_t init);
/// Shift in bytes in memory order.
inline void tp_crc_hw_feed(const uint8_t* data, size_t len);
/// Current remainder, masked to the programmed width.
inline uint32_t tp_crc_hw_value();
#endif

namespace Crc {

// ============================================================================
// Standard parameters
// ============================================================================

constexpr uint8_t  POLY8 = 0x07u;          ///< CRC-8: x^8 + x^2 + x + 1 (MT6835, SMBus PEC).
constexpr uint8_t  INIT8 = 0x00u;          ///< CRC-8 initial value.
constexpr uint16_t POLY16 = 0x1021u;       ///< CRC-16/CCITT-FALSE: x^16 + x^12 + x^5 + 1.
constexpr uint16_t INIT16 = 0xFFFFu;       ///< CRC-16/CCITT-FALSE initial value.
constexpr uint32_t POLY32 = 0x04C11DB7u;   ///< CRC-32 (IEEE polynomial, MPEG-2 parameters).
constexpr uint32_t INIT32 = 0xFFFFFFFFu;   ///< CRC-32/MPEG-2 initial value.

// ============================================================================
// Software shift register (constexpr, any context)
// ============================================================================

/**
 * @brief Shift one byte into an MSB-first CRC register of any width up to 32 bits.
 *
 * The register is kept left-aligned in a 32-bit word (the top @p width bits),
 * so one loop serves every width: the byte is XORed into the top bits and
 * eight conditional reductions by the left-aligned polynomial follow.
 *
 * @param crc   Register, left-aligned (bit 31 is the CRC's most significant bit).
 * @param byte  Next data byte.
 * @param poly  Polynomial, left-aligned the same way.
 * @return Updated left-aligned register.
 */
constexpr uint32_t shift_byte(uint32_t crc, uint8_t byte, uint32_t poly) {
    crc ^= (uint32_t)byte << 24;                       // Feed the byte into the top of the register
    for (int i = 0; i < 8; ++i) {                      // One reduction step per data bit
        crc = (crc & 0x80000000u) ? ((crc << 1) ^ poly) : (crc << 1);
    }
    return crc;
}

/**
 * @brief One-shot CRC-8 over a byte buffer.
 * @param data Bytes in memory order.
 * @param len  Byte count.
 * @param poly Polynomial (default POLY8).
 * @param init Initial register value (default INIT8).
 */
constexpr uint8_t crc8(const uint8_t* data, size_t len, uint8_t poly = POLY8, uint8_t init = INIT8) {
    uint32_t crc = (uint32_t)init << 24;               // Left-align the 8-bit register
    const uint32_t p = (uint32_t)poly << 24;           // Left-aligned polynomial
    for (size_t i = 0; i < len; ++i) {
        crc = shift_byte(crc, data[i], p);
    }
    return (uint8_t)(crc >> 24);                       // Back to 8 bits
}

/**
 * @brief One-shot CRC-16 over a byte buffer.
 * @param data Bytes in memory order.
 * @param len  Byte count.
 * @param poly Polynomial (default POLY16).
 * @param init Initial register value (default INIT16).
 */
constexpr uint16_t crc16(const uint8_t* data, size_t len, uint16_t poly = POLY16, uint16_t init = INIT16) {
    uint32_t crc = (uint32_t)init << 16;               // Left-align the 16-bit register
    const uint32_t p = (uint32_t)poly << 16;           // Left-aligned polynomial
    for (size_t i = 0; i < len; ++i) {
        crc = shift_byte(crc, data[i], p);
    }
    return (uint16_t)(crc >> 16);                      // Back to 16 bits
}

/**
 * @brief One-shot CRC-32 over a byte buffer.
 * @param data Bytes in memory order.
 * @param len  Byte count.
 * @param poly Polynomial (default POLY32).
 * @param init Initial register value (default INIT32).
 */
constexpr uint32_t crc32(const uint8_t* data, size_t len, uint32_t poly = POLY32, uint32_t init = INIT32) {
    uint32_t crc = init;                               // A 32-bit register is already left-aligned
    for (size_t i = 0; i < len; ++i) {
        crc = shift_byte(crc, data[i], poly);
    }
    return crc;
}

// Known answers ("123456789" check values of the named algorithms) prove the
// register logic at compile time, so a broken edit cannot reach a board.
namespace check {
constexpr uint8_t kVector[9] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
static_assert(crc8(kVector, 9) == 0xF4u, "CRC-8 check value");
static_assert(crc16(kVector, 9) == 0x29B1u, "CRC-16/CCITT-FALSE check value");
static_assert(crc32(kVector, 9) == 0x0376E6E7u, "CRC-32/MPEG-2 check value");
}  // namespace check

// ============================================================================
// Streaming context (software, or the hardware unit when TP_CRC_HW is set)
// ============================================================================

/**
 * @brief A CRC accumulated over data that arrives in pieces.
 *
 * begin() programs width, polynomial and initial value; feed() shifts in
 * bytes (or one little-endian word); value() is the remainder so far and may
 * be read at any point. Only one Stream may be open at a time when the
 * hardware backend is in use (see the file header).
 */
class Stream {
public:
    /**
     * @brief Start (or restart) a CRC.
     * @param width Register width in bits: 8, 16 or 32.
     * @param poly  Polynomial, right-aligned (0x07, 0x1021, 0x04C11DB7).
     * @param init  Initial register value, right-aligned.
     */
    void begin(uint8_t width, uint32_t poly, uint32_t init) {
        width_ = width;
#ifdef TP_CRC_SOFTWARE
        const uint8_t shift = (uint8_t)(32u - width);  // Left-align both the polynomial and the register
        poly_ = poly << shift;
        crc_ = init << shift;
#else
        tp_crc_hw_begin(width, poly, init);            // The unit holds the register from here on
#endif
    }

    /**
     * @brief Shift in bytes in memory order.
     * @param data Bytes.
     * @param len  Byte count.
     */
    void feed(const uint8_t* data, size_t len) {
#ifdef TP_CRC_SOFTWARE
        for (size_t i = 0; i < len; ++i) {
            crc_ = shift_byte(crc_, data[i], poly_);
        }
#else
        tp_crc_hw_feed(data, len);
#endif
    }

    /**
     * @brief Shift in one 32-bit word as its four little-endian bytes.
     * @param w Word (a flash record word, for instance).
     */
    void feed_word(uint32_t w) {
        const uint8_t b[4] = {(uint8_t)w, (uint8_t)(w >> 8), (uint8_t)(w >> 16), (uint8_t)(w >> 24)};
        feed(b, 4);
    }

    /// Remainder so far, right-aligned to the width given to begin().
    uint32_t value() const {
#ifdef TP_CRC_SOFTWARE
        return crc_ >> (32u - width_);                 // Back to the natural width
#else
        return tp_crc_hw_value();
#endif
    }

private:
    uint8_t width_ = 32;                               ///< Register width in bits.
#ifdef TP_CRC_SOFTWARE
    uint32_t poly_ = POLY32;                           ///< Left-aligned polynomial.
    uint32_t crc_ = INIT32;                            ///< Left-aligned register.
#endif
};

}  // namespace Crc

#endif  // TP_MATH_CRC_H
