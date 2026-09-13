/**
 * @file mt6835.h
 * @brief Compile-time register map and SPI command builders for the MagnTek
 *        MT6835 21-bit absolute magnetic encoder.
 *
 * Zero-RAM design: everything is constexpr - the CRC-8 can even be checked
 * at compile time. Only the constants actually referenced by the firmware
 * occupy flash.
 *
 * SPI protocol (mode 3: CPOL=1, CPHA=1, MSB first):
 *   24-bit frame = [3:0] OP | [15:4] ADDR(12 bit) | [23:16] DATA
 *   Read : OP_READ,  the register content is shifted out during the third
 *          byte (the master clocks a dummy byte).
 *   Write: OP_WRITE, the sensor ACKs with 0x55.
 *   Burst: OP_ANGLE + start address, the sensor streams consecutive
 *          register bytes until CS goes high.
 *
 * This header is target-agnostic (no MCU dependencies).
 *
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */
#ifndef HARDWARE_MT6835_H
#define HARDWARE_MT6835_H

#include <stdint.h>

namespace MT6835 {

// ------------------------------------------------------------------ opcodes
/** 4-bit command opcodes. */
enum Op : uint8_t {
    OP_READ  = 0x3,  ///< Read a register (0b0011)
    OP_WRITE = 0x6,  ///< Write a register, ACK = 0x55 (0b0110)
    OP_ZERO  = 0x5,  ///< Latch current angle as zero position (0b0101)
    OP_PROG  = 0xC,  ///< Burn OTP/EEPROM, takes ~6 s (0b1100)
    OP_ANGLE = 0xA,  ///< Angle burst read starting at REG_ANGLE1 (0b1010)
};

// ------------------------------------------------------------------ addresses
/** Register addresses (12-bit). */
enum Reg : uint16_t {
    REG_USERID     = 0x001,   ///< User ID
    REG_ANGLE1     = 0x003,   ///< ANGLE[20:13]
    REG_ANGLE2     = 0x004,   ///< ANGLE[12:5]
    REG_ANGLE3     = 0x005,   ///< ANGLE[4:0] + STATUS[2:0]
    REG_ANGLE4     = 0x006,   ///< Angle frame CRC-8
    REG_ABZ_RES1   = 0x007,   ///< ABZ resolution low
    REG_ABZ_RES2   = 0x008,   ///< ABZ resolution high / AB swap
    REG_ZERO1      = 0x009,   ///< Zero position [11:4]
    REG_ZERO2      = 0x00A,   ///< Zero position [3:0] + Z pulse options
    REG_OPTS1      = 0x00B,   ///< UVW resolution / offset / mux
    REG_OPTS2      = 0x00C,   ///< PWM mode / NLC enable
    REG_OPTS3      = 0x00D,   ///< Hysteresis / rotation direction
    REG_OPTS4      = 0x00E,   ///< Autocal frequency / GPIO drive
    REG_OPTS5      = 0x011,   ///< Bandwidth
    REG_NLC_BASE   = 0x013,   ///< Non-linear correction table (192 bytes)
    REG_CAL_STATUS = 0x113,   ///< Self-calibration status
};

/** Angle counts per revolution: 2^21. */
inline constexpr uint32_t CPR = 2097152UL;

/** Self-calibration finished flag (REG_CAL_STATUS bit 7). */
inline constexpr uint8_t CAL_DONE = 0x80U;

/** STATUS[2:0] bits (ANGLE3 low bits). */
namespace ST {
    constexpr uint8_t OVERSPEED  = 0x1;  ///< Angle change faster than the tracking rate
    constexpr uint8_t WEAK_FIELD = 0x2;  ///< Magnetic field strength below nominal
    constexpr uint8_t UNDERVOLT  = 0x4;  ///< Internal supply undervoltage
}

// ------------------------------------------------------------------ frames
/**
 * Build the 24-bit command word: OP(4) | ADDR(12) | DATA(8).
 * Extract the three wire bytes with BYTE0()/BYTE1()/BYTE2() (MSB first).
 */
inline constexpr uint32_t Cmd(uint8_t op, uint16_t addr, uint8_t data = 0) {
    return ((uint32_t)(op & 0xFU) << 20) | ((uint32_t)(addr & 0xFFFU) << 8) | data;
}

/** First (MSB) wire byte of the 24-bit frame. */
inline constexpr uint8_t BYTE0(uint32_t cmd) { return (uint8_t)(cmd >> 16); }

/** Second wire byte of the 24-bit frame. */
inline constexpr uint8_t BYTE1(uint32_t cmd) { return (uint8_t)(cmd >> 8); }

/** Third (LSB) wire byte of the 24-bit frame. */
inline constexpr uint8_t BYTE2(uint32_t cmd) { return (uint8_t)(cmd); }

/** Ready-made 24-bit read command for one register. */
inline constexpr uint32_t CmdRead(uint16_t addr) { return Cmd(OP_READ, addr); }

/** Ready-made 24-bit write command (verify the ACK 0x55 on the wire). */
inline constexpr uint32_t CmdWrite(uint16_t addr, uint8_t data) { return Cmd(OP_WRITE, addr, data); }

/** Ready-made angle burst command (data starts streaming from REG_ANGLE1). */
inline constexpr uint32_t CmdAngleBurst() { return Cmd(OP_ANGLE, REG_ANGLE1); }

// ------------------------------------------------------------------ decode
/**
 * Assemble the 21-bit raw angle from the three burst payload bytes.
 *
 *   b0 = ANGLE[20:13], b1 = ANGLE[12:5], b2[7:3] = ANGLE[4:0].
 *
 * Result range: 0..2097151 (2^21 - 1). The three fields are packed WITHOUT
 * overlap: b0 -> bits [20:13], b1 -> bits [12:5], b2 >> 3 -> bits [4:0].
 *
 * @param b0 First payload byte  (ANGLE[20:13])
 * @param b1 Second payload byte (ANGLE[12:5])
 * @param b2 Third payload byte  (ANGLE[4:0] << 3 | STATUS[2:0])
 * @return 21-bit absolute angle (0..2097151)
 */
inline constexpr uint32_t Angle(uint8_t b0, uint8_t b1, uint8_t b2) {
    return ((uint32_t)b0 << 13) | ((uint32_t)b1 << 5) | (uint32_t)(b2 >> 3) << 11;
}

/** Extract STATUS[2:0] from the third burst payload byte. */
inline constexpr uint8_t Status(uint8_t b2) { return (uint8_t)(b2 & 0x7U); }

/**
 * CRC-8 of one angle burst frame, as the sensor computes it for REG_ANGLE4.
 *
 * Polynomial x^8 + x^2 + x + 1 (0x07), initial value 0x00, no reflection and
 * no final XOR, over the 24 payload bits ANGLE[20:0] + STATUS[2:0] in wire
 * order (b0, b1, b2). Bit-serial on purpose: three bytes are 24 shift steps,
 * cheap enough for a 20 kHz interrupt and free of any lookup table.
 *
 * @param b0 First payload byte  (ANGLE[20:13])
 * @param b1 Second payload byte (ANGLE[12:5])
 * @param b2 Third payload byte  (ANGLE[4:0] << 3 | STATUS[2:0])
 * @return CRC-8 to compare with the fourth burst byte (REG_ANGLE4)
 */
inline constexpr uint8_t Crc8(uint8_t b0, uint8_t b1, uint8_t b2) {
    uint32_t data = ((uint32_t)b0 << 16) | ((uint32_t)b1 << 8) | b2;  // 24 payload bits, MSB first
    uint8_t crc = 0;                                                    // Initial value 0x00
    for (int i = 23; i >= 0; --i) {                                     // One shift per payload bit
        const uint8_t bit = (uint8_t)(((data >> i) & 1U) ^ (crc >> 7));  // Feed the next bit into the register
        crc = (uint8_t)(crc << 1);                                      // Shift the register
        if (bit) {
            crc ^= 0x07U;                                               // Reduce by the polynomial
        }
    }
    return crc;
}
static_assert(Crc8(0x00, 0x00, 0x00) == 0x00, "CRC-8 of an all-zero frame is zero");



}  // namespace MT6835

#endif  // HARDWARE_MT6835_H