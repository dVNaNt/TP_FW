#pragma once

/**
 * @file tp_wire.h
 * @brief TunePulse CAN Protocol (TPCP) v1 - wire-level definitions and codecs.
 *
 * This header is the aggregate of everything that appears ON THE BUS. The
 * definitions themselves live in wire/, split by concern:
 *
 * | Header              | Contents                                     |
 * |:--------------------|:---------------------------------------------|
 * | `wire/tp_ids.h`     | Protocol identity, identifier map, predicates |
 * | `wire/tp_opcodes.h` | The opcode space and its guard keys           |
 * | `wire/tp_enums.h`   | Error, state, warning and selector enums      |
 * | `wire/tp_codecs.h`  | Fixed-point and mini-float codecs             |
 *
 * Include THIS file, not the parts: the four are ordered here and a host tool
 * should not have to know that ordering. The whole set is deliberately free of
 * any hardware or firmware dependency, so a host-side tool can include this one
 * file and be bit-compatible with the drive without pulling in the
 * motor-control stack. Nothing reachable from here may ever include ../lib/ -
 * that is what keeps `pio test -e native_tpcp` compiling.
 *
 * ## Framing
 * Every core frame is EXACTLY 8 bytes. That is the entire payload of a classic
 * CAN 2.0A frame, so the protocol runs unchanged on a classic bus; on a CAN-FD
 * link the same 8-byte frames are sent as FD frames (optionally with BRS) and
 * the 9..64 byte payload sizes stay reserved for future bulk transfer.
 *
 * Byte 0 of every frame is the OPCODE. Bytes 1..7 are the opcode-specific
 * payload ("Respond value" in the original draft). Multi-byte integers are
 * little-endian - the native order of the Cortex-M4 - so packing is a store
 * and unpacking is a load. Sub-byte fields are allocated from the LSB upward.
 *
 * ## Identifier map (11-bit standard identifiers only)
 * Lower identifier == higher bus priority. The map is ordered so that a drive
 * reporting a fault always wins arbitration, network management comes next,
 * commands outrank responses, and status telemetry can never delay a command.
 *
 * | Range         | Direction     | Purpose                                  |
 * |:--------------|:--------------|:-----------------------------------------|
 * | 0x000 + node  | node -> bus   | Emergency / fault report (highest prio)  |
 * | 0x100 - 0x10F | master -> bus | Network management, sync, discovery      |
 * | 0x200 + node  | master -> node| Real-time setpoint stream (no response)  |
 * | 0x300 + node  | master -> node| Request / command channel                |
 * | 0x400 + node  | node -> master| Response channel                         |
 * | 0x500 \| byte | node -> master| Discovery byte arbitration reply         |
 * | 0x600 + node  | node -> bus   | Heartbeat and cyclic telemetry           |
 * | 0x700 - 0x7FF | reserved      | Bulk transfer / firmware update (v1.1)   |
 *
 * Node addresses: 0 = unassigned (the drive answers network management only),
 * 1..254 = assigned, 255 = broadcast (0x2FF / 0x3FF address every node at once).
 *
 * The 0x100 / 0x300 / 0x400 assignments are inherited verbatim from the
 * original can_api.md draft so existing notes stay valid; the discovery reply
 * was moved off 0x400 (where it collided with the response channel of an
 * already-addressed node) onto its own 0x500 block.
 *
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#include "wire/tp_ids.h"      // Protocol identity, 11-bit identifier map, id predicates
#include "wire/tp_opcodes.h"  // Byte 0 of every frame: the opcode space + guard keys
#include "wire/tp_enums.h"    // Result / status enumerations carried as payload bytes
#include "wire/tp_codecs.h"   // Compact fixed-point and mini-float codecs
