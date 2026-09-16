#pragma once

/**
 * @file tp_wire.h
 * @brief TunePulse CAN Protocol (TPCP) v1 - wire-level definitions and codecs.
 *
 * This header is the aggregate of everything that appears ON THE BUS. The
 * definitions themselves live in wire/, split by concern:
 *
 * | Header              | Contents                                         |
 * |:--------------------|:-------------------------------------------------|
 * | `wire/tp_ids.h`     | Protocol identity, identifier map, predicates    |
 * | `wire/tp_cmds.h`    | The CMD byte: groups, register addresses, keys   |
 * | `wire/tp_enums.h`   | Error, state, warning and selector enumerations  |
 * | `wire/tp_codecs.h`  | Fixed-point and mini-float codecs                |
 *
 * Include THIS file, not the parts: the four are ordered here and a host tool
 * should not have to know that ordering. The whole set is deliberately free of
 * any hardware or firmware dependency, so a host-side tool can include this one
 * file and be bit-compatible with the drive without pulling in the
 * motor-control stack. Nothing reachable from here may ever include ../core/ -
 * that is what keeps `pio test -e native_tpcp` compiling.
 *
 * ## Framing
 * Every frame is EXACTLY 8 bytes. That is the entire payload of a classic
 * CAN 2.0A frame, so the protocol runs unchanged on a classic bus; on a CAN-FD
 * link the same 8-byte frames are sent as FD frames.
 *
 * Byte 0 of every frame is the CMD: a register address with the group bits
 * folded in (see wire/tp_cmds.h). Bytes 1..7 are the register's payload.
 * Multi-byte integers are little-endian - the native order of the Cortex-M4 -
 * so packing is a store and unpacking is a load. Sub-byte fields are allocated
 * from the LSB upward. The bit-packed burst frame is the one exception: it is
 * a 64-bit bit field laid out MSB-first.
 *
 * ## Identifier map (11-bit standard identifiers only)
 * Lower identifier == higher bus priority.
 *
 * | Range         | Direction      | Purpose                                     |
 * |:--------------|:---------------|:--------------------------------------------|
 * | 0x100 + node  | node -> bus    | Emergency / fault report (highest priority) |
 * | 0x300 + node  | master -> node | Commands: registers, setpoints, flashing    |
 * | 0x400 + node  | node -> master | Responses, discovery replies                |
 * | 0x500 + node  | node -> bus    | PulseBeat and cyclic telemetry              |
 *
 * Node addresses: 0 = broadcast (0x300 reaches every node), 1..254 = assigned,
 * 255 = unassigned (the node listens on 0x3FF and answers on 0x4FF until it is
 * given an address by the discovery registers).
 *
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#include "wire/tp_ids.h"     // Protocol identity, 11-bit identifier map, id predicates
#include "wire/tp_cmds.h"    // Byte 0 of every frame: groups, register addresses, guard keys
#include "wire/tp_enums.h"   // Result / status enumerations carried as payload bytes
#include "wire/tp_codecs.h"  // Compact fixed-point and mini-float codecs
