#pragma once

/**
 * @file tp_registers.h
 * @brief TPCP v1 - the typed register map and the config-to-hardware path.
 *
 * This header is the aggregate of the register layer. The parts live in
 * registers/:
 *
 * | Header                          | Contents                             |
 * |:--------------------------------|:-------------------------------------|
 * | `registers/tp_register_desc.h`  | Descriptor model, context, value helpers |
 * | `registers/tp_register_ids.h`   | The TpRegId identifier space         |
 * | `registers/tp_register_table.h` | TP_REGISTERS + lookup + normalization |
 * | `registers/tp_config_apply.h`   | tp_apply_config: shadow -> hardware  |
 *
 *
 * Fixed status chunks answer "how is the drive doing"; the register map answers
 * "what is the drive configured to do". Every tunable quantity in the firmware
 * gets a 16-bit identifier, a type, access flags and a pair of accessors, so a
 * host tool can read and write anything without a dedicated opcode per field
 * and without a compiled-in copy of the firmware's data structures.
 *
 * ## Identifier grouping
 *
 * | Range         | Group                                                    |
 * |:--------------|:---------------------------------------------------------|
 * | 0x0000-0x00FF | Identity, node address, link and communication settings   |
 * | 0x0100-0x01FF | Motor identity and identified electrical parameters       |
 * | 0x0200-0x02FF | Protection limits (current, voltage, thermal, power)      |
 * | 0x0300-0x03FF | Control loop: mode, spring/damper, observer bandwidth     |
 * | 0x0400-0x04FF | Motion envelope and trajectory playback                   |
 * | 0x0500-0x05FF | Read-only live telemetry mirrors                          |
 *
 * ## Values on the wire
 *
 * Every register value travels as a 32-bit word. TpRegType says how to read it:
 * integers are zero/sign extended, floats are the raw IEEE-754 bit pattern, and
 * Q16.16 values are the native fixed-point encoding used by the encoder. The
 * type is echoed in every reply, so a generic tool can decode a register it has
 * never seen before.
 *
 * ## Why a configuration shadow exists
 *
 * Several protection limits are write-only in the driver layer: the proxies
 * program a comparator threshold and keep nothing readable. TpNodeConfig is
 * therefore the authoritative copy of every configured value; registers read
 * and write the shadow, and tp_apply_config() pushes it into the hardware. That
 * also makes save/load trivially correct - the thing that gets persisted is
 * exactly the thing the registers report.
 *
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#include "registers/tp_register_desc.h"   // TpRegDesc, TpRegContext, TpRegType/Flag
#include "registers/tp_register_ids.h"    // TpRegId identifier space
#include "registers/tp_register_table.h"  // The sorted map, tp_find_register, normalize
#include "registers/tp_config_apply.h"    // tp_apply_config
