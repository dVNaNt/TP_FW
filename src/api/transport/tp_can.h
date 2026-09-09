#pragma once

/**
 * @file tp_can.h
 * @brief Binds the TunePulse CAN Protocol engine to the FDCAN1 driver.
 *
 * The engine is transport-agnostic: it moves frames through four function
 * pointers and never names a peripheral. This file is the CAN implementation of
 * those four, plus the bring-up and service entry points the application calls.
 *
 * It also multiplexes the CAN link with the two USB transports - the framed
 * serial transport in tp_serial.h and the HID transport in tp_hid.h - so the
 * same protocol answers on the USB cable as well as the bus. That is what makes
 * the whole protocol usable on a bench with no CAN adapter at all, and what
 * gives a host a low-latency control link; see the routing rules at
 * tp_mux_write().
 *
 * It also owns the FlashEEPROM instance bound to the node-configuration
 * partition. That partition is deliberately not one of the two owned by
 * TunePulseHeart: the calibration partition belongs to the angle table and the
 * user partition belongs to the motor profile, whose save engine erases the
 * whole partition when it fills up. A node address stored there would be
 * destroyed by an unrelated profile save.
 *
 * Usage:
 * @code
 *   tunepulse_init();      // Bring the drive up first
 *   tp_can_begin();        // Then the link and the protocol
 *   for (;;) {
 *       tp_can_service();  // From loop(), as often as possible
 *   }
 * @endcode
 *
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#include "bsp/stm32g4/bootloader.h"  // USB DFU entry for the reboot opcode
#include "bsp/stm32g4/fdcan.h"        // CAN_BUS: the FDCAN1 driver
#include "bsp/stm32g4/nvm_flash.h"        // Configuration partition callbacks
#include "../tp_engine.h"                           // The protocol engine
#include "tp_hid.h"                                 // Protocol transport over the vendor HID endpoints
#include "tp_serial.h"                              // Framed protocol transport over USB CDC

/** @brief The protocol engine instance for this drive. */
inline TpProtocol TP_API;

/** @brief Persistence backend for the node-configuration partition. */
inline FlashEEPROM tp_config_flash;

// ============================================================================
// Transport callbacks
// ============================================================================

/**
 * @brief Fetch one pending frame from the CAN driver.
 * @param[out] id  Full 11-bit identifier.
 * @param[out] f   Frame payload (always 8 bytes, short frames zero-padded).
 * @param[out] len Real payload length as transmitted (1..8).
 * @return true when a frame was delivered.
 */
inline bool tp_can_read(uint16_t& id, TpFrame& f, uint8_t& len) {
    uint32_t raw_id = 0;                             // Driver reports a 32-bit identifier
    uint8_t raw_len = 0;                             // Real DLC of the received frame
    if (!CAN_BUS.read(raw_id, f.b8, raw_len)) {
        return false;                                // Receive ring empty
    }
    id = (uint16_t)(raw_id & 0x7FFu);                // Standard identifiers only
    len = raw_len;                                   // Hand the real length to the engine
    return true;
}

/**
 * @brief Queue one 8-byte protocol frame for transmission.
 * @param id Identifier to transmit with.
 * @param f  Frame payload.
 * @return true when the frame was queued.
 */
inline bool tp_can_write(uint16_t id, const TpFrame& f) {
    return CAN_BUS.send(id, f.b8);                   // Single-chunk fast path, no padding
}

/**
 * @brief Queue an identifier-only frame for the discovery arbitration.
 *
 * The payload is empty on purpose: the answer IS the identifier, and CAN
 * arbitration resolves the lowest one for free.
 *
 * Anything still queued is aborted first. A node that lost the previous
 * arbitration round would otherwise re-send its stale reply into the next
 * round, answering a question the master never asked.
 *
 * @param id Identifier to transmit.
 * @return true when the frame was queued.
 */
inline bool tp_can_write_id(uint16_t id) {
    CAN_BUS.abort_pending_tx();                      // Drop any stale arbitration reply
    return CAN_BUS.send_empty(id);
}

// ============================================================================
// Transport multiplexer
// ============================================================================
//
// The engine drives ONE transport interface, but a drive usefully answers on
// three: the CAN bus it is deployed on, and the two functions of the USB cable
// that is already attached on a bench - the CDC console and the HID control
// pipe. Multiplexing them costs one routing rule:
//
//   - a RESPONSE goes back to whichever transport delivered the request, so a
//     USB host and a bus master never receive each other's replies;
//   - everything unsolicited - telemetry, emergency frames - goes to ALL of
//     them, so watching one link never hides what the others are being told.
//
// A discovery reply is the exception. On CAN its whole mechanism is bit-level
// arbitration between competing nodes; the USB transports have no equivalent,
// so they answer with a plain frame instead - see tp_mux_write_id().

/** Transport a frame arrived on, so its answer goes back the same way. */
enum TpMuxSource : uint8_t {
    TP_MUX_CAN = 0,  ///< The CAN bus.
    TP_MUX_SERIAL,   ///< The framed transport on the USB CDC console.
    TP_MUX_HID       ///< The vendor HID interrupt endpoints.
};

/** Transport the frame currently being answered arrived on. */
inline TpMuxSource tp_mux_source = TP_MUX_CAN;

/**
 * @brief Fetch a pending frame from whichever transport has one.
 *
 * HID is polled first because it is the control link: its frames are the ones
 * with a deadline, and a trajectory burst queued behind a console flood would
 * arrive late. Serial comes next so a bench host stays responsive while the bus
 * is busy. All three are drained across successive calls either way.
 */
inline bool tp_mux_read(uint16_t& id, TpFrame& f, uint8_t& len) {
    if (TP_HID.read(id, f, len)) {
        tp_mux_source = TP_MUX_HID;                  // Answer this one over HID
        return true;
    }
    if (TP_SERIAL.read(id, f, len)) {
        tp_mux_source = TP_MUX_SERIAL;               // Answer this one over serial
        return true;
    }
    if (tp_can_read(id, f, len)) {
        tp_mux_source = TP_MUX_CAN;                  // Answer this one over CAN
        return true;
    }
    return false;
}

/**
 * @brief Transmit a frame on the transport its identifier implies.
 * @param id Identifier the engine wants to send on.
 * @param f  Frame payload.
 * @return true when at least one transport accepted the frame.
 */
inline bool tp_mux_write(uint16_t id, const TpFrame& f) {
    const bool is_response = (id >= TP_ID_RESPONSE_BASE) && (id < TP_ID_DISCOVERY_BASE);
    if (is_response) {
        // Directed reply: back to whoever asked.
        switch (tp_mux_source) {
        case TP_MUX_HID:
            return TP_HID.write(id, f);
        case TP_MUX_SERIAL:
            return TP_SERIAL.write(id, f);
        default:
            return tp_can_write(id, f);
        }
    }
    // Unsolicited: telemetry and emergencies reach every observer.
    const bool a = tp_can_write(id, f);
    const bool b = TP_SERIAL.write(id, f);
    const bool c = TP_HID.write(id, f);
    return a || b || c;
}

/**
 * @brief Emit a discovery reply on the transport that asked.
 *
 * On CAN the reply is a data-less frame whose identifier IS the answer, and
 * competing nodes are resolved by bit-level arbitration. The USB transports are
 * point-to-point: there is nobody to arbitrate against, so the same identifier
 * is simply sent with an empty payload and the single reply is the answer.
 *
 * Routing this through the multiplexer is what makes an UNADDRESSED node
 * reachable over USB at all. With the reply hard-wired to CAN, a node with no
 * address could never be discovered on a bench - it ignores everything but
 * network management, and its answer to network management went out on a bus
 * that was not connected.
 */
inline bool tp_mux_write_id(uint16_t id) {
    if (tp_mux_source == TP_MUX_CAN) {
        return tp_can_write_id(id);
    }
    TpFrame empty;                                   // Identifier-only: payload is unused
    empty.u64 = 0;
    return (tp_mux_source == TP_MUX_HID) ? TP_HID.write(id, empty) : TP_SERIAL.write(id, empty);
}

/**
 * @brief Rebind the hardware acceptance filters to a node address.
 * @param node_id     Address to accept traffic for (0 = network management only).
 * @param promiscuous true to additionally accept every identifier.
 */
inline void tp_can_bind_filters(uint8_t node_id, bool promiscuous) {
    CAN_BUS.set_node_filters(node_id, promiscuous);
}

/**
 * @brief Reboot the device.
 * @param mode 0 = warm reset, 1 = jump into the USB DFU bootloader.
 */
inline void tp_can_reboot(uint8_t mode) {
    if (mode == 1u) {
        BOOT::jump_to_system_bootloader();           // Never returns
    }
    NVIC_SystemReset();                              // Warm reset, never returns
}

// ============================================================================
// Bring-up and service
// ============================================================================

/**
 * @brief Bring up the configuration partition, the link and the protocol.
 *
 * Order matters: the stored configuration is read BEFORE the link starts, so
 * the peripheral can be brought up at the stored bit rates and in the stored
 * frame format instead of at the defaults followed by a visible reconfiguration
 * glitch on the bus.
 *
 * @param tp Drive to expose over the protocol.
 * @return true when the link came up.
 */
inline bool tp_can_begin(TunePulseHeart& tp) {
    // Configuration partition: its own region, its own callbacks, so a runaway
    // access can never reach the calibration table or the motor profile.
    tp_config_flash.set_max_size(FlashG4_LL::config_section_size());
    tp_config_flash.set_callback()
        .erase_cb(&FlashG4_LL::config_erase)
        .read_cb(&FlashG4_LL::config_read)
        .write_cb(&FlashG4_LL::config_write);

    TP_API.attach(&tp, tp_mux_read, tp_mux_write, tp_mux_write_id, nullptr);
    TP_API.bind_platform(&tp_config_flash, tp_can_reboot);
    TP_API.begin();  // Identity, stored configuration, hardware limits

    // Start the link using whatever the stored configuration asked for.
    const TpNodeConfig& cfg = TP_API.config();
    const bool fd = (cfg.flags & TP_CFG_CLASSIC_LINK) == 0u;
#ifdef TP_BENCH_LOOPBACK
    // Bench mode: the controller receives everything it transmits and needs no
    // second node to acknowledge it. A single node on a real bus never gets an
    // acknowledgement, so its transmit error counter climbs until it goes
    // bus-off - which is why the whole protocol stack is untestable with one
    // board unless the controller is put in loopback.
    const bool loopback = true;
#else
    const bool loopback = false;
#endif
    const bool up = CAN_BUS.configure_link((uint32_t)cfg.arb_kbps * 1000u,
                                           (uint32_t)cfg.data_kbps * 1000u, fd, loopback);
    if (!up) {
        TP_LOG("TPCP: CAN link bring-up FAILED");
        return false;
    }

    // Filters can only be installed once the peripheral exists, which is why
    // the engine was attached with a null filter callback above: it is bound
    // here, after the link is up, and applied immediately.
    TP_API.attach(&tp, tp_mux_read, tp_mux_write, tp_mux_write_id, tp_can_bind_filters);
    tp_can_bind_filters(cfg.node_id, (cfg.flags & TP_CFG_PROMISCUOUS) != 0u);

    TP_LOG_VALS("TPCP: link up", (unsigned long)((uint32_t)cfg.arb_kbps * 1000u),
                fd ? "FD" : "classic", (unsigned)cfg.node_id);
    return true;
}

/**
 * @brief Send one request to this node's OWN address (loopback self-test).
 *
 * With the controller in internal loopback the frame comes straight back
 * through the acceptance filters, is dispatched by the engine and answered on
 * the response identifier. A rising TpProtocol::handled() therefore proves the
 * whole chain end to end - bit timing, filters, the receive ring, dispatch,
 * chunk packing and the transmit ring - with nothing but the board itself.
 *
 * @param opcode Status opcode to request (defaults to Device Info).
 * @return true when the request was queued.
 */
inline bool tp_can_selftest_ping(uint8_t opcode = TP_OP_DEVICE_INFO) {
    const uint8_t node = TP_API.config().node_id;    // Our own address
    if (node == TP_NODE_UNASSIGNED) {
        return false;                                // Unaddressed: the filters accept nothing
    }
    TpFrame f;                                       // Request under construction
    f.u64 = 0;                                       // Read requests carry a zero payload
    f.b8[0] = opcode;                                // What to ask for
    return CAN_BUS.send((uint16_t)(TP_ID_REQUEST_BASE + node), f.b8);
}

/**
 * @brief Service the link and the protocol. Call from loop(), as often as possible.
 *
 * Everything here runs in thread mode and is bounded, so it can never delay the
 * control ISR: the protocol only ever hands the control loop work through the
 * DriveControl mailbox.
 */
inline void tp_can_service() {
    CAN_BUS.update();  // Bus-off recovery + pump the transmit ring
    tp_hid_service();  // Pack whatever the engine queued into a HID report

    // Publish the link health so the COMM_STATUS chunk and the warning byte
    // report what the peripheral actually sees rather than a software guess.
    const CANDriver::BusStatus bs = CAN_BUS.get_bus_status();
    const uint8_t flags = (uint8_t)((bs.error_passive ? 0x01u : 0u) |
                                    (bs.bus_off ? 0x02u : 0u) |
                                    (bs.warning ? 0x04u : 0u));
    const uint32_t dropped = CAN_BUS.dropped;
    TP_API.set_bus_counters(bs.tec, bs.rec, flags,
                            (uint16_t)((dropped > 0xFFFFu) ? 0xFFFFu : dropped));

    TP_API.poll();     // Receive, execute, answer, emit telemetry, persist
}
