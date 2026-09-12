#pragma once

/**
 * @file tp_discovery.h
 * @brief TPCP v1 - the byte-by-byte UID search participant.
 *
 * The master isolates one node at a time by walking the UID tree: for step i
 * it broadcasts the prefix resolved so far, and every matching node answers
 * with a DATA-LESS frame whose identifier is 0x500 | UID[i]. CAN arbitrates
 * bit by bit and a dominant bit wins, so the frame that survives is the one
 * carrying the LOWEST byte - the bus itself performs the minimum search at
 * zero software cost. Seven steps yield a full UID and an address is assigned.
 *
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#include "../tp_wire.h"     // TP_ID_DISCOVERY_BASE, TP_UID_BYTES
#include "tp_node_identity.h"  // TpNodeIdentity, whose UID is being searched
// ============================================================================
// UID discovery engine
// ============================================================================

/**
 * @class TpDiscovery
 * @brief Answers the master's byte-by-byte UID search.
 *
 * The master isolates one node at a time by walking the UID tree. For search
 * step i it broadcasts the prefix it has already resolved; every node whose
 * UID starts with that prefix replies with a DATA-LESS frame whose identifier
 * is 0x500 | UID[i]. Because a CAN identifier is arbitrated bit by bit and a
 * dominant bit wins, the frame that survives arbitration is automatically the
 * one carrying the LOWEST byte value - the bus itself performs the minimum
 * search, at zero cost in software. The master records that byte and moves to
 * step i+1. After seven steps it holds a complete UID and assigns an address.
 *
 * A node that loses arbitration must not let the controller re-send its stale
 * reply into the next round, where it would answer a question that was never
 * asked. Two independent defences cover that: every new prefix request aborts
 * whatever is still pending in the transmitter, and a node only ever replies
 * once per request. Disabling automatic retransmission (DAR) on the link makes
 * the first defence unnecessary, but the protocol does not require it.
 */
class TpDiscovery {
public:
    /** @brief Enter discovery mode and reset the search state. */
    void begin() {
        active_ = true;      // Answer prefix requests from now on
        isolated_ = false;   // Not yet singled out by the master
    }

    /** @brief Leave discovery mode. */
    void end() {
        active_ = false;     // Ignore prefix requests again
        isolated_ = false;   // Clear any isolation
    }

    /** @brief true while the node participates in a UID search. */
    bool is_active() const { return active_; }

    /**
     * @brief Decide whether to answer one prefix request.
     *
     * A node that has already been isolated in this search round stays quiet so
     * the master can find the remaining devices without it winning every
     * arbitration again.
     *
     * @param id         Node identity (source of the UID).
     * @param prefix     Prefix bytes received from the master.
     * @param len        Prefix length in bytes (0 .. TP_UID_BYTES-1).
     * @param[out] reply Identifier to transmit with DLC 0 when the answer is yes.
     * @return true when this node must reply.
     */
    bool on_prefix(const TpNodeIdentity& id, const uint8_t* prefix, uint8_t len, uint16_t& reply) {
        if (!active_ || isolated_) {
            return false;                             // Not searching, or already found
        }
        if (len >= TP_UID_BYTES) {
            return false;                             // Nothing left to disclose
        }
        if (!id.prefix_matches(prefix, len)) {
            return false;                             // This node is not in the master's subtree
        }
        reply = (uint16_t)(TP_ID_DISCOVERY_BASE | id.bytes()[len]);  // Next UID byte rides the identifier
        return true;
    }

    /**
     * @brief Test whether an assignment frame addresses this node.
     *
     * The frame carries the full 7-byte UID plus the address to assign, so an
     * assignment can never land on the wrong drive even if two UIDs share
     * their first six bytes.
     *
     * @param id      Node identity.
     * @param payload The 8 frame bytes: UID[0..6] followed by the node address.
     * @param[out] assigned Address to adopt when the UID matches.
     * @return true when this node is the addressee.
     */
    bool on_assign(const TpNodeIdentity& id, const uint8_t* payload, uint8_t& assigned) {
        if (!id.prefix_matches(payload, TP_UID_BYTES)) {
            return false;                             // Meant for a different drive
        }
        assigned = payload[TP_UID_BYTES];             // Byte 7 carries the address
        isolated_ = true;                             // Stay quiet for the rest of the search
        return true;
    }

private:
    bool active_ = false;    ///< Discovery mode entered.
    bool isolated_ = false;  ///< Already isolated in this search round.
};
