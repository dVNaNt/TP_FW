#pragma once

/**
 * @file tp_ids.h
 * @brief TPCP v1 - protocol identity, the 11-bit identifier map and its predicates.
 *
 * The identifier is the protocol's routing layer: every frame's meaning starts
 * with which channel base it falls into, and the node address is simply an
 * offset from that base. Four channels are carved out of the 11-bit space,
 * ordered so that CAN's own bit-level arbitration resolves them by priority:
 * a tripping drive (0x1xx) outranks every command (0x3xx), a command outranks
 * every answer (0x4xx), and status telemetry (0x5xx) can never delay either.
 *
 * Node addresses: 0 is the broadcast address (0x300 reaches every node at
 * once), 1..254 are assignable, 255 is the address of a node that has not been
 * given one yet. An unassigned node listens on 0x3FF, answers on 0x4FF, and
 * accepts nothing but the two discovery registers until it is addressed.
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

/** Protocol version reported by DEVICE_INFO (major in the high byte, minor in the low byte). */
#define TP_PROTOCOL_VERSION 0x0100u

/** Payload size of every protocol frame, in bytes (one classic CAN frame). */
#define TP_FRAME_SIZE 8u

/**
 * Board identity reported by DEVICE_INFO.
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
#define TP_ID_EMCY_BASE 0x100u

/** Base identifier of the command channel (master -> node: registers, setpoints, flashing). */
#define TP_ID_COMMAND_BASE 0x300u

/** Base identifier of the response channel (node -> master; discovery replies too). */
#define TP_ID_RESPONSE_BASE 0x400u

/** Base identifier of the PulseBeat and cyclic telemetry channel (node -> bus). */
#define TP_ID_TELEMETRY_BASE 0x500u

/** Mask isolating the channel class of an identifier (0x100, 0x300, 0x400, 0x500). */
#define TP_ID_CLASS_MASK 0x700u

/** Node address that every drive answers in addition to its own address. */
#define TP_NODE_BROADCAST 0x00u

/** Node address of a drive that has not been assigned one yet (discovery pending). */
#define TP_NODE_UNASSIGNED 0xFFu

/** Lowest individually assignable node address. */
#define TP_NODE_MIN 0x01u

/** Highest individually assignable node address. */
#define TP_NODE_MAX 0xFEu

/** Bits in a device HashUID (the identity discovery searches on). */
#define TP_UID_BITS 48u

/** Bytes in a device HashUID (six bytes, little-endian on the wire). */
#define TP_UID_BYTES 6u

/** Mask of the 48 significant bits of a HashUID held in a 64-bit word. */
#define TP_UID_MASK 0x0000FFFFFFFFFFFFull

// ============================================================================
// Identifier helpers
// ============================================================================

/**
 * @brief Channel class of an identifier (TP_ID_*_BASE), ignoring the node byte.
 * @param id Received standard identifier.
 */
static inline uint16_t tp_id_class(uint16_t id) {
    return (uint16_t)(id & TP_ID_CLASS_MASK);  // Top three bits of the 11-bit identifier
}

/**
 * @brief Node byte of an identifier.
 * @param id Received standard identifier.
 */
static inline uint8_t tp_id_node(uint16_t id) {
    return (uint8_t)(id & 0xFFu);  // Low eight bits are the node address
}

/**
 * @brief True when an 11-bit identifier addresses this node's command channel.
 *
 * A node listens on its own address and on the broadcast address. An
 * unassigned node (255) therefore listens on 0x3FF and 0x300.
 *
 * @param id      Received standard identifier.
 * @param node_id This node's assigned address.
 */
static inline bool tp_is_command_for(uint16_t id, uint8_t node_id) {
    return (id == (uint16_t)(TP_ID_COMMAND_BASE + node_id)) ||           // Directed command
           (id == (uint16_t)(TP_ID_COMMAND_BASE + TP_NODE_BROADCAST));   // Broadcast command
}

/**
 * @brief True when an identifier is a broadcast command (0x300).
 * @param id Received standard identifier.
 */
static inline bool tp_is_broadcast(uint16_t id) {
    return id == (uint16_t)(TP_ID_COMMAND_BASE + TP_NODE_BROADCAST);
}

/**
 * @brief True when an identifier belongs to a node -> master or node -> bus class.
 *
 * Used by the USB-to-CAN gateway to decide which bus frames are answers worth
 * relaying to a host: emergencies, responses and telemetry.
 *
 * @param id Received standard identifier.
 */
static inline bool tp_is_node_to_master(uint16_t id) {
    const uint16_t cls = tp_id_class(id);  // Channel class
    return (cls == TP_ID_EMCY_BASE) || (cls == TP_ID_RESPONSE_BASE) || (cls == TP_ID_TELEMETRY_BASE);
}
