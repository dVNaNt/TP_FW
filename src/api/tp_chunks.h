#pragma once

/**
 * @file tp_chunks.h
 * @brief TunePulse CAN Protocol (TPCP) v1 - the 8-byte frame payload layouts.
 *
 * This header is the aggregate of the frame layouts plus the union that ties
 * them together. The layouts themselves live in chunks/, split by CMD group
 * so each file matches one section of CAN_PROTOCOL.md:
 *
 * | Header                       | CMD range          | Contents                     |
 * |:-----------------------------|:-------------------|:-----------------------------|
 * | `chunks/tp_chunk_config.h`   | 0x00 - 0x3F        | Read/write registers         |
 * | `chunks/tp_chunk_status.h`   | 0x40 - 0x7F        | Read-only registers, replies |
 * | `chunks/tp_chunk_motion.h`   | 0xC0 - 0xFF        | Motion tasks, save, MIDI, burst |
 * | `chunks/tp_chunk_system.h`   | -                  | union TpFrame                |
 *
 * Include THIS file, not the parts. A host tool needs exactly this file and
 * tp_wire.h to be bit-compatible with the drive.
 *
 * Layout rules (identical for every view):
 * - Byte 0 is the CMD (the read form in every reply).
 * - Multi-byte integers are little-endian. The structs are packed, so a field
 *   lands exactly where the specification puts it whatever its alignment.
 * - Sub-byte fields are allocated from the LSB upward and reached through
 *   accessor macros, never through compiler bitfields.
 *
 * The ONE exception to the little-endian rule is the bit-packed burst frame
 * (see TpBurst): it is a 64-bit bit field, not a struct of integers, and is
 * laid out MSB-first so that its group bits land at the top of byte 0.
 *
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#include "chunks/tp_chunk_system.h"  // Every layout plus union TpFrame
