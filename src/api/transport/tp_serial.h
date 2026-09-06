#pragma once

/**
 * @file tp_serial.h
 * @brief Binds the TunePulse CAN Protocol engine to the USB CDC console.
 *
 * The protocol engine names no peripheral: it moves frames through four
 * function pointers. tp_can.h is the CAN binding; this is the serial one. It
 * exists so the entire protocol - dispatch, registers, telemetry, motion,
 * calibration, persistence - can be exercised over the USB cable that is
 * already attached, with no CAN adapter and no second node on a bus.
 *
 * ## Framing
 *
 * A CAN bus delivers framed messages; USB CDC delivers a byte stream, so the
 * identifier and the payload need a frame around them:
 *
 *     A5 5A | id_lo id_hi | d0 d1 d2 d3 d4 d5 d6 d7 | xor
 *
 * Thirteen bytes, fixed length, XOR checksum over the ten payload bytes. Fixed
 * length means resynchronization is trivial: hunt for the sync pair, and let a
 * bad checksum throw the frame away.
 *
 * ## Why binary frames and log text can share one stream
 *
 * The sync byte 0xA5 has bit 7 set and the log output is ASCII, so a log line
 * can never contain it. The host scans for the sync pair and passes everything
 * else through as text - which means the diagnostic console and the protocol
 * coexist on one USB endpoint with no mode switch and no second cable.
 *
 * ## Non-blocking, always
 *
 * USBSerial::write() spins when the host stops draining its endpoint. Called
 * from loop() that would stall the protocol service and, on a bus, take the
 * node offline. Every transmit here checks free room first and DROPS the frame
 * instead of blocking - the same rule the TP_LOG macros follow in the ISR.
 *
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#include <Arduino.h>

#include "../tp_chunks.h"
#include "../tp_wire.h"

/** First byte of the frame delimiter (bit 7 set: never appears in ASCII logs). */
#define TP_SERIAL_SYNC0 0xA5u

/** Second byte of the frame delimiter. */
#define TP_SERIAL_SYNC1 0x5Au

/** Total bytes on the wire per framed protocol frame. */
#define TP_SERIAL_FRAME_LEN 13u

/**
 * @class TpSerialTransport
 * @brief Length-framed protocol transport over the USB CDC console.
 */
class TpSerialTransport {
public:
    /**
     * @brief Feed the receive state machine and hand back a complete frame.
     *
     * Drains at most one frame per call so a flood on the console cannot
     * monopolize loop().
     *
     * @param[out] id  Identifier the frame was addressed to.
     * @param[out] f   Frame payload.
     * @param[out] len Payload length; always 8 on this transport.
     * @return true when a complete, checksum-valid frame was assembled.
     */
    bool read(uint16_t& id, TpFrame& f, uint8_t& len) {
        while (SerialUSB.available() > 0) {                  // Consume whatever has arrived
            const int b = SerialUSB.read();                  // One byte from the CDC endpoint
            if (b < 0) {
                break;                                       // Nothing left after all
            }
            const uint8_t byte = (uint8_t)b;                 // Byte under examination

            if (state_ == 0u) {
                state_ = (byte == TP_SERIAL_SYNC0) ? 1u : 0u;  // Hunt for the first sync byte
                continue;
            }
            if (state_ == 1u) {
                // A second 0xA5 keeps us waiting for 0x5A rather than
                // restarting, so a run of sync bytes still frames correctly.
                state_ = (byte == TP_SERIAL_SYNC1) ? 2u : ((byte == TP_SERIAL_SYNC0) ? 1u : 0u);
                continue;
            }

            buf_[state_ - 2u] = byte;                        // Collect identifier and payload
            state_++;
            if (state_ < TP_SERIAL_FRAME_LEN) {
                continue;                                    // Frame still incomplete
            }

            state_ = 0u;                                     // Frame complete either way
            uint8_t sum = 0;                                 // XOR over identifier + payload
            for (uint32_t i = 0; i < 10u; ++i) {
                sum ^= buf_[i];
            }
            if (sum != buf_[10]) {
                errors_++;                                   // Corrupt or a false sync: drop it
                continue;
            }
            id = (uint16_t)(((uint16_t)buf_[1] << 8) | buf_[0]) & 0x7FFu;  // Little-endian identifier
            for (uint32_t i = 0; i < 8u; ++i) {
                f.b8[i] = buf_[2u + i];                      // Payload straight into the frame
            }
            len = 8u;                                        // This transport is always 8 bytes
            rx_count_++;
            return true;
        }
        return false;                                        // Nothing complete yet
    }

    /**
     * @brief Transmit one framed protocol frame.
     * @param id Identifier to label the frame with.
     * @param f  Frame payload.
     * @return true when the frame was written, false when it was dropped.
     */
    bool write(uint16_t id, const TpFrame& f) {
        // Drop rather than block: a host that stops reading must never be able
        // to stall the protocol service loop.
        if (SerialUSB.availableForWrite() < (int)(TP_SERIAL_FRAME_LEN + 8u)) {
            dropped_++;
            return false;
        }
        uint8_t out[TP_SERIAL_FRAME_LEN];                    // Complete framed record
        out[0] = TP_SERIAL_SYNC0;                            // Delimiter
        out[1] = TP_SERIAL_SYNC1;                            // Delimiter
        out[2] = (uint8_t)(id & 0xFFu);                      // Identifier, little-endian
        out[3] = (uint8_t)((id >> 8) & 0x07u);
        uint8_t sum = out[2] ^ out[3];                       // Checksum accumulator
        for (uint32_t i = 0; i < 8u; ++i) {
            out[4u + i] = f.b8[i];                           // Payload
            sum ^= f.b8[i];
        }
        out[12] = sum;                                       // XOR checksum
        SerialUSB.write(out, TP_SERIAL_FRAME_LEN);
        tx_count_++;
        return true;
    }

    /** @brief Frames received and accepted. */
    uint32_t rx_count() const { return rx_count_; }

    /** @brief Frames transmitted. */
    uint32_t tx_count() const { return tx_count_; }

    /** @brief Frames dropped because the console was not being drained. */
    uint32_t dropped() const { return dropped_; }

    /** @brief Frames discarded on a checksum mismatch. */
    uint32_t errors() const { return errors_; }

private:
    uint8_t  buf_[11] = {0};   ///< Identifier, payload and checksum being assembled.
    uint8_t  state_ = 0;       ///< 0/1 hunting for sync, 2..12 collecting.
    uint32_t rx_count_ = 0;    ///< Accepted frames.
    uint32_t tx_count_ = 0;    ///< Transmitted frames.
    uint32_t dropped_ = 0;     ///< Frames dropped on back-pressure.
    uint32_t errors_ = 0;      ///< Frames failing their checksum.
};

/** @brief Serial transport instance shared by the protocol binding. */
inline TpSerialTransport TP_SERIAL;
