#pragma once

/**
 * @file tp_discovery.h
 * @brief TPCP v1 - the binary HashUID search a node takes part in.
 *
 * The master finds unaddressed nodes by walking the 48-bit HashUID space as a
 * binary tree: it broadcasts a prefix of `depth` bits on 0x300, and every
 * unaddressed node whose HashUID starts with that prefix answers on 0x4FF.
 *
 * Every matching node answers with the SAME frame - the prefix it matched
 * and the incremented depth - so when several nodes answer at once their
 * frames are bit-identical and CAN merges them into one. Had the reply
 * carried the next bit of each node's own HashUID, two nodes disagreeing on
 * it would collide in the data field, which CAN does not arbitrate: both
 * frames are destroyed and the master learns nothing. Echoing the prefix
 * turns a reply into a pure "this subtree is populated" and lets the master
 * descend by probing prefix|0 and prefix|1 at the next depth. A probe at depth
 * 48 is answered by exactly one node, whose full HashUID the master then owns.
 *
 * An addressed node ignores broadcast searches - a running system is never
 * disturbed by a sweep - but answers a search sent to its OWN address, which
 * is how a master checks two drives have not been given the same number.
 *
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#include "../tp_wire.h"        // TP_UID_BITS, TP_NODE_UNASSIGNED
#include "tp_node_identity.h"  // TpNodeIdentity, whose HashUID is being searched

// ============================================================================
// HashUID search participant
// ============================================================================

/**
 * @class TpDiscovery
 * @brief Answers the master's binary HashUID search.
 */
class TpDiscovery {
public:
    /**
     * @brief Decide whether to answer one search frame and build the reply.
     *
     * @param id           Node identity (source of the HashUID).
     * @param node_id      This node's current address.
     * @param directed     true when the frame was addressed to this node alone.
     * @param depth        Prefix length in bits the master sent.
     * @param prefix       Prefix bits the master sent.
     * @param[out] reply_depth  Depth to answer with (depth + 1, capped at 48).
     * @param[out] reply_prefix Prefix bits to answer with (the matched prefix).
     * @return true when this node must reply.
     */
    static bool on_search(const TpNodeIdentity& id, uint8_t node_id, bool directed, uint8_t depth,
                          uint64_t prefix, uint8_t& reply_depth, uint64_t& reply_prefix) {
        if ((node_id != TP_NODE_UNASSIGNED) && !directed) {
            return false;                             // Addressed nodes ignore broadcast searches
        }
        if (depth > TP_UID_BITS) {
            return false;                             // Malformed: deeper than a HashUID
        }
        if (!id.prefix_matches(prefix, depth)) {
            return false;                             // Not in the master's subtree
        }
        reply_depth = (uint8_t)((depth < TP_UID_BITS) ? (depth + 1u) : TP_UID_BITS);
        reply_prefix = id.raw() & TpNodeIdentity::prefix_mask(depth);  // Echo what was matched
        return true;
    }

    /**
     * @brief Interpret one assignment frame (0x11 write).
     *
     * @param id        Node identity.
     * @param uid       HashUID carried by the frame (0 = wildcard).
     * @param addr      Address carried by the frame.
     * @param broadcast true when the frame reached every node.
     * @param[out] assign Address to adopt when the frame applies to this node.
     * @return true when this node must adopt @p assign.
     */
    static bool on_assign(const TpNodeIdentity& id, uint64_t uid, uint8_t addr, bool broadcast, uint8_t& assign) {
        if (uid == id.raw()) {
            assign = addr;                            // Directed at this very drive
            return true;
        }
        if (uid == 0ull) {
            // Wildcard: only a reset to the unassigned address is permitted,
            // whether broadcast (reset the whole bus) or directed (reset every
            // node that answers to this address - the collision case).
            if (addr == TP_NODE_UNASSIGNED) {
                assign = TP_NODE_UNASSIGNED;
                return true;
            }
            (void)broadcast;                          // A wildcard assignment is never allowed
        }
        return false;
    }
};
