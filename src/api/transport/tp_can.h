#pragma once

/**
 * @file tp_can.h
 * @brief Binds the TunePulse CAN Protocol engine to the FDCAN1 driver.
 *
 * The engine is transport-agnostic: it moves frames through three function
 * pointers and never names a peripheral. This file is the CAN implementation of
 * those three, plus the bring-up and service entry points the application calls.
 *
 * It also multiplexes the CAN link with the two USB transports - the framed
 * serial transport in tp_serial.h and the HID transport in tp_hid.h - so the
 * same protocol answers on the USB cable as well as the bus. That is what makes
 * the whole protocol usable on a bench with no CAN adapter at all, and what
 * gives a host a low-latency control link; see the routing rules at
 * tp_mux_write().
 *
 * On top of the multiplexer sits a USB-to-CAN gateway: once a host is heard on
 * the USB cable, the frames it sends for the bus are relayed onto CAN and the
 * answers other nodes put on CAN are relayed back to it, so one USB cable
 * reaches every drive on the bus - see the gateway section below.
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

#include "bsp/stm32g4/bootloader.h"  // USB DFU entry for the reboot action
#include "bsp/stm32g4/fdcan.h"       // CAN_BUS: the FDCAN1 driver
#include "bsp/stm32g4/nvm_flash.h"   // Configuration partition callbacks
#include "../tp_engine.h"            // The protocol engine
#include "tp_hid.h"                  // Protocol transport over the vendor HID endpoints
#include "tp_serial.h"               // Framed protocol transport over USB CDC

/** @brief The protocol engine instance for this drive. */
inline TpProtocol TP_API;

/** @brief Persistence backend for the node-configuration partition. */
inline FlashEEPROM tp_config_flash;

/**
 * @brief True once a USB host has been heard and the bus is being relayed.
 *
 * Declared here rather than in the gateway section because the filter binding
 * below consults it: an armed gateway keeps the master filter slots open
 * whatever the stored configuration says. Set by tp_gateway_arm(), never cleared.
 */
inline bool tp_gateway_armed = false;

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
 * @brief Rebind the hardware acceptance filters to a node address.
 *
 * An armed gateway needs every node -> master frame, not just this node's
 * own: the frames it relays to the USB host are other nodes' answers. The
 * master filter slots therefore stay open while the gateway is armed,
 * whatever the stored configuration flag says.
 *
 * @param node_id     Address to accept traffic for (255 = unassigned).
 * @param promiscuous true to additionally accept every node -> master class.
 */
inline void tp_can_bind_filters(uint8_t node_id, bool promiscuous) {
    CAN_BUS.set_node_filters(node_id, promiscuous || tp_gateway_armed);
}

// ============================================================================
// USB-to-CAN gateway
// ============================================================================
//
// A host on the USB cable talks to ONE drive: the multiplexer below answers it
// from this node's engine, and nothing the host sends ever reaches the bus. A
// second drive hanging off this node's CAN connector is therefore invisible to
// it. The gateway removes that limit by relaying between the USB transports
// and CAN, one rule per direction:
//
//   - DOWN: every command frame the host sends for the bus - the broadcast
//     and frames for OTHER nodes - is also put on the bus. A frame addressed
//     to this node alone stays local: the bus never needs it and nothing there
//     would answer it.
//   - UP: every node -> master frame received from the bus - emergencies,
//     responses, discovery replies, PulseBeats and telemetry - is copied to
//     both USB transports and withheld from the engine. Another node sent it;
//     it is not this node's to act on.
//
// The two directions never meet. What goes down is always a command frame and
// what comes up is always a node -> master frame, so nothing can circulate,
// and a response to the host still leaves on the transport that asked because
// tp_mux_write() is untouched.
//
// The gateway arms itself on the first protocol frame that arrives over USB
// and stays armed until reset. A deployed node never sees such a frame, so the
// gateway costs it nothing; a bench host gets the whole bus the moment it
// starts talking, with no address and no register write required first - the
// discovery sweep that finds the unaddressed bus nodes is itself the frame
// that arms it. Arming opens the master filter slots, because the answers this
// node must relay are addressed to nobody it would otherwise hear.
//
// Discovery works through the gateway because every reply the host collects
// is a plain "this subtree is populated": the bus nodes merge their identical
// replies on the wire, this node adds its own when unaddressed, and the host
// sees one answer either way.
//
// Frames go down with DLC 8 - the CAN driver's fast path, and what the host
// transports send anyway - and are dropped, counted, when the transmit ring
// is full: a burst the host streams at HID speed can outrun a classic 1 Mbit
// bus, and the host, not this node, is the one that must pace it.
//
// A controller in internal loopback (TP_BENCH_LOOPBACK) receives its own
// frames, so relaying there would only hand every frame to the engine twice:
// the gateway never arms in that build.

/** Frames the host sent that were copied onto the bus. */
inline uint32_t tp_gateway_down = 0;

/** Frames the bus delivered that were copied to the host. */
inline uint32_t tp_gateway_up = 0;

/** Frames that could not be relayed because the receiving ring was full. */
inline uint32_t tp_gateway_lost = 0;

/**
 * Bus frames one tp_mux_read() call may relay up before giving up its turn.
 * Bounds the work hidden inside a single engine read, so a telemetry storm
 * from the bus cannot keep the engine's own poll budget from ever completing.
 */
constexpr uint8_t TP_GATEWAY_UP_BUDGET = 8;

/**
 * @brief Arm the gateway on the first frame heard from a USB host.
 *
 * Idempotent and cheap once armed. Arming rebinds the acceptance filters so
 * the master slots open; the node address and the stored flag are passed
 * through unchanged, tp_can_bind_filters() adds the gateway's own requirement.
 */
inline void tp_gateway_arm() {
#ifndef TP_BENCH_LOOPBACK
    if (!tp_gateway_armed) {
        tp_gateway_armed = true;                     // Latched until reset
        const TpNodeConfig& cfg = TP_API.config();   // Address and flags to keep
        tp_can_bind_filters(cfg.node_id, (cfg.flags & TP_CFG_PROMISCUOUS) != 0u);
        TP_LOG("TPCP: gateway armed, relaying USB <-> CAN");
    }
#endif
}

/**
 * @brief Relay one frame the host sent onto the bus, unless it is ours alone.
 *
 * Called for every frame a USB transport delivers, before the engine sees it,
 * so the local engine and the bus receive the same frame in the same order.
 *
 * @param id Identifier the host sent the frame with.
 * @param f  Frame payload.
 */
inline void tp_gateway_relay_down(uint16_t id, const TpFrame& f) {
    tp_gateway_arm();                                // A host is talking: start relaying
    if (!tp_gateway_armed) {
        return;                                      // Loopback build: nothing to relay to
    }
    if (tp_id_class(id) != TP_ID_COMMAND_BASE) {
        return;                                      // Node->master classes are not the host's to send
    }
    // Directed to us alone: nothing on the bus would answer. The broadcast and
    // frames for other nodes are exactly what the host would put on a bus of
    // its own.
    if (tp_id_node(id) == TP_API.config().node_id) {
        return;
    }
    if (tp_can_write(id, f)) {
        tp_gateway_down++;
    } else {
        tp_gateway_lost++;                           // Transmit ring full: the host outran the bus
    }
}

/**
 * @brief Relay one frame the bus delivered to the host, if it is an answer.
 *
 * Only node->master classes are relayed; a command frame from a real bus
 * master is left to the engine, whose own address check decides its fate.
 *
 * @param id Identifier the frame arrived with.
 * @param f  Frame payload, zero-padded by the driver when short.
 * @return true when the frame was another node's and has been relayed, so the
 *         engine must not see it; false when it is the engine's to dispatch.
 */
inline bool tp_gateway_relay_up(uint16_t id, const TpFrame& f) {
    if (!tp_gateway_armed) {
        return false;                                // Nobody to relay to
    }
    if (!tp_is_node_to_master(id)) {
        return false;                                // Command frame: the engine's filter decides
    }
    const bool a = TP_HID.write(id, f);
    const bool b = TP_SERIAL.write(id, f);
    if (a || b) {
        tp_gateway_up++;
    } else {
        tp_gateway_lost++;                           // Neither host was collecting
    }
    return true;
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
//   - everything unsolicited - PulseBeats, telemetry, emergency frames - goes
//     to ALL of them, so watching one link never hides what the others are
//     being told.

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
 *
 * The gateway hooks in here: a USB frame is copied to the bus on its way to
 * the engine, and a bus frame that is another node's answer is copied to the
 * USB transports and skipped, bounded by TP_GATEWAY_UP_BUDGET per call.
 */
inline bool tp_mux_read(uint16_t& id, TpFrame& f, uint8_t& len) {
    if (TP_HID.read(id, f, len)) {
        tp_mux_source = TP_MUX_HID;                  // Answer this one over HID
        tp_gateway_relay_down(id, f);                // The bus sees it too
        return true;
    }
    if (TP_SERIAL.read(id, f, len)) {
        tp_mux_source = TP_MUX_SERIAL;               // Answer this one over serial
        tp_gateway_relay_down(id, f);                // The bus sees it too
        return true;
    }
    for (uint8_t n = 0; n < TP_GATEWAY_UP_BUDGET; ++n) {
        if (!tp_can_read(id, f, len)) {
            return false;                            // Bus ring empty
        }
        if (!tp_gateway_relay_up(id, f)) {
            tp_mux_source = TP_MUX_CAN;              // Ours: answer this one over CAN
            return true;
        }
        // Relayed to the host and withheld from the engine: look at the next one.
    }
    return false;                                    // Budget spent on relays; the rest waits a turn
}

/**
 * @brief Transmit a frame on the transport its identifier implies.
 * @param id Identifier the engine wants to send on.
 * @param f  Frame payload.
 * @return true when at least one transport accepted the frame.
 */
inline bool tp_mux_write(uint16_t id, const TpFrame& f) {
    if (tp_id_class(id) == TP_ID_RESPONSE_BASE) {
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
    // Unsolicited: PulseBeats, telemetry and emergencies reach every observer.
    const bool a = tp_can_write(id, f);
    const bool b = TP_SERIAL.write(id, f);
    const bool c = TP_HID.write(id, f);
    return a || b || c;
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

    TP_API.attach(&tp, tp_mux_read, tp_mux_write, nullptr);
    TP_API.bind_platform(&tp_config_flash, tp_can_reboot);
    TP_API.begin();  // Identity, stored configuration, hardware limits

    // Start the link using whatever the stored configuration asked for.
    const TpNodeConfig& cfg = TP_API.config();
    const bool fd = cfg.bus_type != TP_BUS_CLASSIC;
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
    TP_API.attach(&tp, tp_mux_read, tp_mux_write, tp_can_bind_filters);
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
 * register packing and the transmit ring - with nothing but the board itself.
 *
 * @param reg Read register to request (defaults to Device Info).
 * @return true when the request was queued.
 */
inline bool tp_can_selftest_ping(uint8_t reg = TP_REG_DEVICE_INFO) {
    const uint8_t node = TP_API.config().node_id;    // Our own address
    if (node == TP_NODE_UNASSIGNED) {
        return false;                                // Unaddressed: nothing but discovery is accepted
    }
    TpFrame f;                                       // Request under construction
    f.u64 = 0;                                       // Read requests carry a zero payload
    f.b8[0] = reg;                                   // What to ask for
    return CAN_BUS.send((uint16_t)(TP_ID_COMMAND_BASE + node), f.b8);
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

    // Publish the link health so the communication diagnostics and the warning
    // byte report what the peripheral actually sees rather than a software guess.
    const CANDriver::BusStatus bs = CAN_BUS.get_bus_status();
    const uint8_t flags = (uint8_t)((bs.error_passive ? 0x01u : 0u) |
                                    (bs.bus_off ? 0x02u : 0u) |
                                    (bs.warning ? 0x04u : 0u) |
                                    (tp_gateway_armed ? 0x08u : 0u));
    const uint32_t dropped = CAN_BUS.dropped;
    TP_API.set_bus_counters(bs.tec, bs.rec, flags,
                            (uint16_t)((dropped > 0xFFFFu) ? 0xFFFFu : dropped));

    TP_API.poll();     // Receive, execute, answer, emit telemetry, persist
}
