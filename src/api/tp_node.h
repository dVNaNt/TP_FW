#pragma once

/**
 * @file tp_node.h
 * @brief TPCP v1 - node identity, persistent configuration and UID discovery.
 *
 * This header is the aggregate of the node layer. The parts live in node/:
 *
 * | Header                      | Contents                                  |
 * |:----------------------------|:------------------------------------------|
 * | `node/tp_node_config.h`     | The persisted record + build-time defaults |
 * | `node/tp_node_identity.h`   | The 56-bit UID folded from the die id      |
 * | `node/tp_node_store.h`      | Flash-backed load / save engine            |
 * | `node/tp_discovery.h`       | The byte-by-byte UID search participant    |
 *
 * Only tp_node_store.h touches hardware; the other three are pure data and
 * logic, so the split keeps the hardware dependency in one visible place.
 *
 *
 * Three things a drive on a shared bus must own before it can be talked to at
 * all, and all three live here:
 *
 * 1. **Identity** - a 56-bit UID derived from the MCU's factory-programmed
 *    unique device identifier. It never changes, is unique per silicon die and
 *    is what the discovery protocol arbitrates on.
 * 2. **Configuration** - the node address plus everything else the master can
 *    persist (link bit rates, protection limits, safe-state policy, watchdog,
 *    telemetry slots), stored in its own flash partition with a magic header,
 *    a version and a CRC32.
 * 3. **Discovery** - the state machine that answers the master's byte-by-byte
 *    UID search so an unaddressed drive can be found and given an address on a
 *    bus it shares with any number of identical drives.
 *
 * ## Why the configuration gets its own flash partition
 *
 * The motor-profile engine appends a record per save into the user partition
 * and erases the WHOLE partition when it runs out of slots (about 60 saves).
 * A node address living there would be silently destroyed by an unrelated
 * profile save, and a drive that forgets its bus address is a field failure.
 * NVM_CONFIG_SECTION_SIZE gives it a partition nothing else writes.
 *
 * ## Why the record is written from thread mode
 *
 * Flash programming stalls the single-bank CPU - tens of milliseconds for an
 * erase - and that stall freezes the control ISR with whatever voltage vector
 * is latched on the bridge. The store therefore only ever programs while
 * tp_flash_programming_window is true, i.e. while the last control pass
 * commanded zero current. Enforcing that is the caller's job (see
 * TpNodeStore::service).
 *
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#include "node/tp_node_config.h"    // TpNodeConfig, TpTelemetrySlot, record constants
#include "node/tp_node_identity.h"  // TpNodeIdentity: the 56-bit UID
#include "node/tp_node_store.h"     // TpNodeStore: flash-backed persistence
#include "node/tp_discovery.h"      // TpDiscovery: UID search state machine
