#pragma once

/**
 * @file tp_ids.h
 * @brief TPCP v1 - protocol identity, the 11-bit identifier map and its predicates.
 *
 * The identifier is the protocol's routing layer: every frame's meaning starts
 * with which channel base it falls into, and the node address is simply an
 * offset from that base. Eight channels are carved out of the 11-bit space,
 * ordered so that CAN's own bit-level arbitration resolves them by priority -
 * emergency lowest-numbered (wins every contest), bulk transfer highest.
 *
 * Keeping the map and the "is this frame mine" predicates in one file means the
 * offsets and the tests that apply them can never drift apart.
 *
 * The build-time identity macros (TP_HW_VERSION, TP_FW_VERSION,
 * TP_DEVICE_CODE) are #ifndef-guarded so a board environment can override them
 * with -D; this header must therefore be reached before their first use.
 *
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#include <stdint.h>  // Fixed-width integer types (host tools include this file standalone)

// ============================================================================
// Protocol identity
// ============================================================================

/** Protocol version reported by DEVICE_INFO / the PROTOCOL_VERSION register. */
#define TP_PROTOCOL_VERSION 0x0100u  // Major 1, minor 0 (high byte . low byte)

/** Payload size of every core protocol frame, in bytes (one classic CAN frame). */
#define TP_FRAME_SIZE 8u

/**
 * Board identity reported by DEVICE_INFO and the identity registers.
 * Override any of them from the build configuration (-D TP_DEVICE_CODE=...)
 * when the same firmware is built for a different board revision.
 */
#ifndef TP_HW_VERSION
#define TP_HW_VERSION 0x30u  // CLN17 v3.0
#endif
#ifndef TP_FW_VERSION
#define TP_FW_VERSION 0x0100u  // Firmware 1.0
#endif
#ifndef TP_DEVICE_CODE
#define TP_DEVICE_CODE 0x434C1703u  // "CL" + 0x17 (CLN17) + revision 3
#endif

// ============================================================================
// Identifier map
// ============================================================================

/** Base identifier of the emergency channel (node -> bus, highest priority). */
#define TP_ID_EMCY_BASE 0x000u

/** Base identifier of the network-management block (master -> bus). */
#define TP_ID_NMT_BASE 0x100u

/** Base identifier of the real-time setpoint stream (master -> node). */
#define TP_ID_SETPOINT_BASE 0x200u

/** Base identifier of the request / command channel (master -> node). */
#define TP_ID_REQUEST_BASE 0x300u

/** Base identifier of the response channel (node -> master). */
#define TP_ID_RESPONSE_BASE 0x400u

/** Base identifier of the discovery arbitration replies (node -> master). */
#define TP_ID_DISCOVERY_BASE 0x500u

/** Base identifier of heartbeat and cyclic telemetry (node -> bus). */
#define TP_ID_TELEMETRY_BASE 0x600u

/** First identifier of the block reserved for bulk transfer / firmware update. */
#define TP_ID_BULK_BASE 0x700u

/** Node address that every drive answers in addition to its own address. */
#define TP_NODE_BROADCAST 0xFFu

/** Node address meaning "no address assigned yet" (discovery pending). */
#define TP_NODE_UNASSIGNED 0x00u

/** Highest individually addressable node number. */
#define TP_NODE_MAX 0xFEu

/** Number of bytes in a device UID (56-bit, one full frame minus the node byte). */
#define TP_UID_BYTES 7u

/**
 * @brief Network-management identifiers inside the 0x100 block.
 *
 * All of them are broadcast frames sent by the master; every drive listens to
 * the whole block regardless of whether it already owns a node address.
 */
enum TpNmtId : uint16_t {
    TP_NMT_DISCOVER_BEGIN  = TP_ID_NMT_BASE + 0x0u,  ///< Arm the UID search on every unaddressed node.
    TP_NMT_DISCOVER_PREFIX = TP_ID_NMT_BASE + 0x1u,  ///< byte 0 = prefix length, bytes 1-7 = prefix.
    TP_NMT_DISCOVER_ASSIGN = TP_ID_NMT_BASE + 0x2u,  ///< bytes 0-6 = full UID, byte 7 = address to assign.
    TP_NMT_DISCOVER_END    = TP_ID_NMT_BASE + 0x3u,  ///< Leave discovery mode, resume normal filtering.
    TP_NMT_GLOBAL_CONTROL  = TP_ID_NMT_BASE + 0x4u,  ///< byte 0 = TpGlobalCmd, bytes 4-7 = key when required.
    TP_NMT_SYNC            = TP_ID_NMT_BASE + 0x5u,  ///< Latch setpoints and emit sync-triggered telemetry.
    TP_NMT_TIME            = TP_ID_NMT_BASE + 0x6u   ///< bytes 0-3 = master wall clock [us], informational.
};

/**
 * The discovery prefix length travels in BYTE 0, not in the frame's DLC.
 *
 * The original draft encoded it as the DLC, which cannot survive a CAN-FD link
 * (payload sizes above 8 are quantized to 12/16/20/24/32/48/64, so a DLC is not
 * a byte count) and forces every receiver to plumb the DLC through to the
 * application. Carrying the length in the payload keeps the rule that EVERY
 * protocol frame is 8 bytes, on both classic CAN and CAN-FD.
 */
#define TP_DISCOVER_PREFIX_MAX 6u  // A 7-byte UID is resolved in seven steps: prefixes 0..6

/** Sub-commands carried in byte 0 of a TP_NMT_GLOBAL_CONTROL broadcast. */
enum TpGlobalCmd : uint8_t {
    TP_GLOBAL_STOP        = 0x00u,  ///< Emergency stop: every node enters its configured safe state.
    TP_GLOBAL_DISABLE     = 0x01u,  ///< Disable the output stage on every node.
    TP_GLOBAL_ENABLE      = 0x02u,  ///< Enable the output stage on every node.
    TP_GLOBAL_FAULT_RESET = 0x03u,  ///< Clear latched faults on every node.
    TP_GLOBAL_HEARTBEAT   = 0x04u,  ///< Ask every node to emit one heartbeat immediately.
    TP_GLOBAL_REBOOT      = 0x05u   ///< Reboot every node (byte 1..4 must carry TP_REBOOT_KEY).
};

// ============================================================================
// Identifier helpers
// ============================================================================

/**
 * @brief True when an 11-bit identifier addresses this node's request channel.
 * @param id      Received standard identifier.
 * @param node_id This node's assigned address.
 */
static inline bool tp_is_request_for(uint16_t id, uint8_t node_id) {
    return (id == (uint16_t)(TP_ID_REQUEST_BASE + node_id)) ||           // Directed request
           (id == (uint16_t)(TP_ID_REQUEST_BASE + TP_NODE_BROADCAST));   // Broadcast request
}

/**
 * @brief True when an 11-bit identifier addresses this node's setpoint stream.
 * @param id      Received standard identifier.
 * @param node_id This node's assigned address.
 */
static inline bool tp_is_setpoint_for(uint16_t id, uint8_t node_id) {
    return (id == (uint16_t)(TP_ID_SETPOINT_BASE + node_id)) ||          // Directed setpoint
           (id == (uint16_t)(TP_ID_SETPOINT_BASE + TP_NODE_BROADCAST));  // Broadcast setpoint
}

/**
 * @brief True when an 11-bit identifier belongs to the network-management block.
 * @param id Received standard identifier.
 */
static inline bool tp_is_nmt(uint16_t id) {
    return (id >= TP_ID_NMT_BASE) && (id < (TP_ID_NMT_BASE + 0x10u));    // 0x100..0x10F
}
