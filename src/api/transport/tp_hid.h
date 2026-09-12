#pragma once

/**
 * @file tp_hid.h
 * @brief Binds the TunePulse CAN Protocol engine to the vendor HID endpoints.
 *
 * The protocol engine names no peripheral: it moves frames through four
 * function pointers. tp_can.h is the CAN binding and tp_serial.h the USB CDC
 * one; this is the USB HID binding, and it is the one meant for control.
 *
 * ## Why this transport exists next to tp_serial.h
 *
 * Both run over the same USB cable, but a HID interrupt endpoint is a message
 * pipe where CDC is a byte stream, and that difference is the whole point:
 *
 *   - **Framing is free.** A report arrives whole or not at all, so there is no
 *     sync pattern to hunt for, no checksum, and no resynchronization after a
 *     glitch. tp_serial.h spends thirteen bytes and a state machine per frame
 *     to emulate what the hardware does here for nothing.
 *   - **Flow control is free.** The host polls the IN endpoint once per USB
 *     frame and the device NAKs when it has nothing to say. On CDC the host
 *     cannot see the device's buffer at all, which is why the serial host tool
 *     has to hand the port four frames at a time with a sleep between batches:
 *     any faster silently overruns the board, any slower starves the trajectory
 *     queue mid-move.
 *   - **Latency is bounded.** One millisecond, set by the endpoint descriptor,
 *     rather than whatever the host's serial driver decides to buffer.
 *   - **Six frames per report.** A 64-byte report carries six protocol frames,
 *     so a full trajectory burst reaches the drive in a handful of USB frames
 *     instead of one frame per transfer.
 *
 * ## Report layout
 *
 *     byte 0      record count n, 0..6
 *     byte 1+     n records of ten bytes:
 *                   [0]    identifier, low eight bits
 *                   [1]    bits 0..2 identifier bits 8..10, bits 4..7 payload length
 *                   [2..9] payload, always eight bytes, only `length` meaningful
 *     remainder   zero padding to 64 bytes
 *
 * Identifiers are eleven bits, exactly as on the bus, so the same identifier
 * map and the same engine serve every transport with no translation.
 *
 * ## Threading
 *
 * Reports arrive in USB interrupt context and are pushed onto a lock-free
 * single-producer / single-consumer ring; the engine drains that ring from
 * thread mode. Transmission is entirely thread mode: frames queue into a second
 * ring and tp_hid_service() packs them into reports. Nothing here can block, so
 * a host that stops reading costs dropped frames, never a stalled control loop.
 *
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#include <Arduino.h>

#include "../../bsp/boards/cln17_v3/usb_hid_setup.h"  // Composite device and the report endpoints
#include "../tp_chunks.h"                         // TpFrame
#include "../tp_wire.h"                           // Identifier map

/** Bytes per record on the wire: two of identifier and length, eight of payload. */
#define TP_HID_REC_LEN 10u

/** Records that fit in one report once the count byte is accounted for. */
#define TP_HID_REC_PER_REPORT ((TP_HID_EP_SIZE - 1u) / TP_HID_REC_LEN)

/**
 * @brief Frames buffered per direction. Power of two so the wrap is a mask.
 *
 * Thirty-two is five reports' worth in each direction: deep enough to absorb a
 * trajectory burst arriving faster than the engine drains it, shallow enough
 * that the two rings together cost well under a kilobyte of RAM.
 */
#define TP_HID_RING 32u

/**
 * @class TpHidTransport
 * @brief Protocol transport over the vendor HID interrupt endpoints.
 */
class TpHidTransport {
public:
    /**
     * @brief Push one received record onto the receive ring.
     *
     * Called from USB interrupt context, once per record in an arriving
     * report. Copies ten bytes and advances a head index - deliberately the
     * cheapest possible ISR path, because the engine that interprets the
     * record runs in thread mode.
     *
     * @param rec Ten-byte record.
     */
    void push_rx(const uint8_t* rec) {
        const uint8_t next = (uint8_t)((rx_head_ + 1u) & (TP_HID_RING - 1u));  // Slot after this one
        if (next == rx_tail_) {
            overruns_++;                              // Ring full: the engine is behind
            return;
        }
        memcpy(rx_ring_[rx_head_], rec, TP_HID_REC_LEN);
        rx_head_ = next;                              // Publish the record to the consumer
    }

    /**
     * @brief Hand the engine one received frame.
     *
     * @param[out] id  Identifier the frame was addressed to.
     * @param[out] f   Frame payload.
     * @param[out] len Payload length as the host sent it (0..8).
     * @return true when a frame was delivered.
     */
    bool read(uint16_t& id, TpFrame& f, uint8_t& len) {
        if (rx_tail_ == rx_head_) {
            return false;                             // Nothing queued
        }
        const uint8_t* rec = rx_ring_[rx_tail_];      // Oldest record
        id = (uint16_t)(((uint16_t)(rec[1] & 0x07u) << 8) | rec[0]);  // Eleven-bit identifier
        len = (uint8_t)(rec[1] >> 4);                 // Payload length the host declared
        if (len > 8u) {
            len = 8u;                                 // Corrupt length: clamp to a frame
        }
        for (uint32_t i = 0; i < 8u; ++i) {
            f.b8[i] = rec[2u + i];                    // Payload straight into the frame
        }
        rx_tail_ = (uint8_t)((rx_tail_ + 1u) & (TP_HID_RING - 1u));
        rx_count_++;
        return true;
    }

    /**
     * @brief Queue one frame for the host.
     *
     * Only queues - the report is assembled and handed to the endpoint by
     * tp_hid_service(). Batching that way is what lets six frames share one
     * USB transfer instead of one transfer each.
     *
     * @param id Identifier to label the frame with.
     * @param f  Frame payload.
     * @return true when the frame was queued, false when the ring was full.
     */
    bool write(uint16_t id, const TpFrame& f) {
        const uint8_t next = (uint8_t)((tx_head_ + 1u) & (TP_HID_RING - 1u));  // Slot after this one
        if (next == tx_tail_) {
            dropped_++;                               // Host is not collecting: drop, never block
            return false;
        }
        uint8_t* rec = tx_ring_[tx_head_];            // Slot being filled
        rec[0] = (uint8_t)(id & 0xFFu);               // Identifier, low eight bits
        rec[1] = (uint8_t)(((id >> 8) & 0x07u) | (8u << 4));  // High bits plus a full payload
        for (uint32_t i = 0; i < 8u; ++i) {
            rec[2u + i] = f.b8[i];                    // Payload
        }
        tx_head_ = next;
        return true;
    }

    /**
     * @brief Pack queued frames into reports and hand them to the endpoint.
     *
     * Call from loop() as often as possible. Sends at most one report per call:
     * the endpoint takes one report per USB frame anyway, so looping here would
     * only spin on a busy flag.
     */
    void service() {
        if (tx_tail_ == tx_head_) {
            return;                                   // Nothing to send
        }
        if (!tp_usb_hid_ready()) {
            return;                                   // Not configured, or one still in flight
        }

        uint8_t report[TP_HID_EP_SIZE];               // Report under construction
        memset(report, 0, sizeof(report));            // Pad the tail with zeros
        uint8_t n = 0;                                // Records packed so far
        while ((n < TP_HID_REC_PER_REPORT) && (tx_tail_ != tx_head_)) {
            memcpy(&report[1u + (n * TP_HID_REC_LEN)], tx_ring_[tx_tail_], TP_HID_REC_LEN);
            tx_tail_ = (uint8_t)((tx_tail_ + 1u) & (TP_HID_RING - 1u));
            n++;
        }
        report[0] = n;                                // Tell the host how many are meaningful

        if (tp_usb_hid_send(report)) {
            tx_count_ += n;
        } else {
            // The endpoint refused after all. The records are already off the
            // ring, so they are lost - counted rather than silently forgotten.
            dropped_ += n;
        }
    }

    /** @brief Frames delivered to the engine. */
    uint32_t rx_count() const { return rx_count_; }

    /** @brief Frames handed to the endpoint. */
    uint32_t tx_count() const { return tx_count_; }

    /** @brief Frames dropped because the host was not collecting. */
    uint32_t dropped() const { return dropped_; }

    /** @brief Records lost because the engine was behind. */
    uint32_t overruns() const { return overruns_; }

private:
    uint8_t rx_ring_[TP_HID_RING][TP_HID_REC_LEN];  ///< Records received, oldest at rx_tail_.
    uint8_t tx_ring_[TP_HID_RING][TP_HID_REC_LEN];  ///< Records to send, oldest at tx_tail_.

    volatile uint8_t rx_head_ = 0;  ///< Written by the USB interrupt only.
    uint8_t rx_tail_ = 0;           ///< Written by thread mode only.
    uint8_t tx_head_ = 0;           ///< Written by thread mode only.
    uint8_t tx_tail_ = 0;           ///< Written by thread mode only.

    uint32_t rx_count_ = 0;         ///< Frames delivered to the engine.
    uint32_t tx_count_ = 0;         ///< Frames handed to the endpoint.
    uint32_t dropped_ = 0;          ///< Frames dropped on a full transmit ring.
    uint32_t overruns_ = 0;         ///< Records lost on a full receive ring.
};

/** @brief HID transport instance shared by the protocol binding. */
inline TpHidTransport TP_HID;

/**
 * @brief Unpack an arriving report onto the receive ring.
 *
 * Installed as the endpoint's receive hook, so this runs in USB interrupt
 * context once per report.
 *
 * @param report The 64 bytes the host sent.
 */
inline void tp_hid_on_report(const uint8_t* report) {
    uint8_t n = report[0];                            // Records the host says are present
    if (n > TP_HID_REC_PER_REPORT) {
        n = TP_HID_REC_PER_REPORT;                    // Corrupt count: take what fits
    }
    for (uint8_t i = 0; i < n; ++i) {
        TP_HID.push_rx(&report[1u + (i * TP_HID_REC_LEN)]);
    }
}

/**
 * @brief Bring up the composite USB device and attach the HID transport.
 * @return true when the device started.
 */
inline bool tp_hid_begin() {
    tp_usb_hid_set_rx_hook(tp_hid_on_report);         // Route reports onto the ring
    return tp_usb_begin();                            // CDC console plus HID control pipe
}

/** @brief Pack and send whatever the engine queued. Call from loop(). */
inline void tp_hid_service() {
    TP_HID.service();
}
