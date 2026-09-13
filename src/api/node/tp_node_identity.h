#pragma once

/**
 * @file tp_node_identity.h
 * @brief TPCP v1 - the immutable 48-bit HashUID this node is discovered by.
 *
 * The STM32 carries a 96-bit factory identifier. The discovery frames have
 * room for 48 bits beside the address / depth byte, so the 96 are folded down
 * with an FNV-1a mix - deterministic across reboots and reflashes, and
 * avalanching, so two dies differing only in wafer coordinate still land far
 * apart. With 2^48 codes a bus of a few hundred drives has a collision
 * probability far below any other failure mode on the board.
 *
 * Kept apart from the store because it is not persisted: the HashUID is
 * recomputed from silicon on every boot and can never be configured.
 *
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#include "../tp_wire.h"  // TP_UID_BITS, TP_UID_BYTES, TP_UID_MASK

// ============================================================================
// Node identity
// ============================================================================

/**
 * @class TpNodeIdentity
 * @brief The immutable 48-bit HashUID this node is discovered by.
 */
class TpNodeIdentity {
public:
    /** @brief Compute the HashUID once; subsequent calls return the cached value. */
    void init() {
        if (ready_) {
            return;                                   // Already folded
        }
#ifdef UID_BASE
        const volatile uint32_t* uid = (const volatile uint32_t*)UID_BASE;  // Factory identifier
        const uint32_t w0 = uid[0];                   // Lot number / wafer coordinates
        const uint32_t w1 = uid[1];                   // Lot number continued
        const uint32_t w2 = uid[2];                   // Lot number continued
#else
        // Host builds and boards without a factory identifier: a fixed value
        // keeps the protocol testable, but such a node must be given its
        // address explicitly because discovery cannot tell two of them apart.
        const uint32_t w0 = 0x54554E45u;              // "TUNE"
        const uint32_t w1 = 0x50554C53u;              // "PULS"
        const uint32_t w2 = 0x45000001u;              // "E" + instance
#endif
        // FNV-1a over the twelve identifier bytes: one multiply and one xor per
        // byte, no table, and every input bit reaches every output bit.
        uint64_t h = 0xCBF29CE484222325ull;           // FNV offset basis
        h = fold(h, w0);                              // Mix word 0
        h = fold(h, w1);                              // Mix word 1
        h = fold(h, w2);                              // Mix word 2
        // Fold the top 16 bits back in rather than discarding them, so no
        // input information is thrown away when the hash is truncated.
        h = (h ^ (h >> TP_UID_BITS)) & TP_UID_MASK;
        // A HashUID of all zeros is the wildcard of the assignment register,
        // so map that one degenerate value onto a fixed non-zero code.
        if (h == 0ull) {
            h = TP_UID_MASK;
        }
        raw_ = h;                                     // Cache the packed form
        ready_ = true;
    }

    /** @brief The 48-bit HashUID packed into a 64-bit word (top 16 bits zero). */
    uint64_t raw() const { return raw_; }

    /**
     * @brief Test whether this node's HashUID starts with a given bit prefix.
     *
     * Bits are counted from bit 0 of the little-endian value: "the first
     * depth bits" are the depth least significant bits of the word.
     *
     * @param prefix Prefix bits (only the low depth bits are compared).
     * @param depth  Prefix length in bits (0 matches every node, 48 exactly one).
     * @return true when the first depth bits match.
     */
    bool prefix_matches(uint64_t prefix, uint8_t depth) const {
        if (depth > TP_UID_BITS) {
            return false;                             // Longer than a HashUID: never matches
        }
        const uint64_t mask = prefix_mask(depth);     // Bits under comparison
        return ((raw_ ^ prefix) & mask) == 0ull;
    }

    /** @brief Mask selecting the first depth bits of a HashUID. */
    static uint64_t prefix_mask(uint8_t depth) {
        return (depth >= TP_UID_BITS) ? TP_UID_MASK : ((1ull << depth) - 1ull);
    }

private:
    /**
     * @brief Mix one 32-bit word into an FNV-1a accumulator, byte by byte.
     * @param h Current accumulator.
     * @param w Word to absorb.
     * @return Updated accumulator.
     */
    static uint64_t fold(uint64_t h, uint32_t w) {
        for (uint32_t i = 0; i < 4u; ++i) {
            h ^= (uint64_t)((w >> (8u * i)) & 0xFFu);  // Absorb one byte
            h *= 0x100000001B3ull;                     // FNV prime
        }
        return h;
    }

    uint64_t raw_ = 0;      ///< Packed 48-bit identity.
    bool     ready_ = false; ///< init() has run.
};
