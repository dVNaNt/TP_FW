#pragma once

/**
 * @file tp_node.h
 * @brief TPCP v1 - node identity, persistent configuration and HashUID discovery.
 *
 * This header is the aggregate of the node layer. The parts live in node/:
 *
 * | Header                      | Contents                                    |
 * |:----------------------------|:--------------------------------------------|
 * | `node/tp_node_config.h`     | The persisted record + build-time defaults   |
 * | `node/tp_node_identity.h`   | The 48-bit HashUID folded from the die id    |
 * | `node/tp_node_store.h`      | Flash-backed load / save engine              |
 * | `node/tp_discovery.h`       | The binary HashUID search participant        |
 *
 * Only tp_node_store.h touches hardware; the other three are pure data and
 * logic, so the split keeps the hardware dependency in one visible place.
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
#include "node/tp_node_identity.h"  // TpNodeIdentity: the 48-bit HashUID
#include "node/tp_node_store.h"     // TpNodeStore: flash-backed persistence
#include "node/tp_discovery.h"      // TpDiscovery: HashUID search participant
