#pragma once

/**
 * @file tp_registers.h
 * @brief TPCP v1 - the register map and the config-to-hardware path.
 *
 * This header is the aggregate of the register layer. The parts live in
 * registers/:
 *
 * | Header                         | Contents                                   |
 * |:-------------------------------|:-------------------------------------------|
 * | `registers/tp_register_map.h`  | TP_REGISTERS: access rules, sub-space sizes |
 * | `registers/tp_config_apply.h`  | tp_apply_config: shadow -> hardware        |
 *
 * A register is one 8-byte frame whose layout is fixed by its CMD (see
 * tp_chunks.h). The map here says what may be done with each CMD; the engine
 * does the packing and the applying, because those need the whole drive.
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

#include "registers/tp_register_map.h"   // TP_REGISTERS, tp_find_register, TpRegFlag
#include "registers/tp_config_apply.h"   // tp_apply_config, tp_watchdog_ticks
