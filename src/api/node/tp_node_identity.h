#pragma once

/**
 * @file tp_node_identity.h
 * @brief TPCP v1 - the immutable 56-bit UID this node is discovered by.
 *
 * The STM32 carries a 96-bit factory identifier. The discovery frame has room
 * for 56 bits beside the node address, so the 96 are folded down with an
 * FNV-1a mix - deterministic across reboots and reflashes, and avalanching, so
 * two dies differing only in wafer coordinate still land far apart.
 *
 * Kept apart from the store because it is not persisted: the UID is recomputed
 * from silicon on every boot and can never be configured.
 *
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#include "../tp_wire.h"  // TP_UID_BYTES and the identifier map
// ============================================================================
// Node identity
// ============================================================================

/**
 * @class TpNodeIdentity
 * @brief The immutable 56-bit UID this node is discovered by.
 *
 * The STM32 carries a 96-bit factory identifier (lot number, wafer number, die
 * X/Y). The protocol frame has room for 56 bits next to the node address, so
 * the 96 bits are folded into 56 with an FNV-1a mix. The fold is deterministic
 * (the same die always produces the same UID across reboots and reflashes) and
 * spreads every input bit across the output, so two dies that differ only in
 * their wafer coordinate still land far apart. With 2^56 codes, a bus of a few
 * hundred drives has a collision probability far below any other failure mode
 * on the board.
 */
class TpNodeIdentity {
public:
    /** @brief Compute the UID once; subsequent calls return the cached value. */
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
        // Fold the top 8 bits back in rather than discarding them, so no input
        // information is thrown away when the hash is truncated to 56 bits.
        h = (h ^ (h >> 56)) & 0x00FFFFFFFFFFFFFFull;
        // A UID of all zeros would tie in every arbitration round, so map that
        // one degenerate value onto a fixed non-zero code.
        if (h == 0ull) {
            h = 0x00FFFFFFFFFFFFFFull;
        }
        for (uint32_t i = 0; i < TP_UID_BYTES; ++i) {
            uid_[i] = (uint8_t)(h >> (8u * i));       // Little-endian byte order
        }
        raw_ = h;                                     // Cache the packed form
        ready_ = true;
    }

    /** @brief The 56-bit UID as seven bytes, least significant byte first. */
    const uint8_t* bytes() const { return uid_; }

    /** @brief The 56-bit UID packed into a 64-bit word (top byte is zero). */
    uint64_t raw() const { return raw_; }

    /** @brief Low 32 bits of the UID, used as the guard on address changes. */
    uint32_t guard() const { return (uint32_t)raw_; }

    /**
     * @brief Test whether this node's UID starts with a given prefix.
     * @param prefix Prefix bytes, least significant byte first.
     * @param len    Prefix length in bytes (0 matches every node).
     * @return true when the first len bytes match.
     */
    bool prefix_matches(const uint8_t* prefix, uint8_t len) const {
        if (len > TP_UID_BYTES) {
            return false;                             // Longer than a UID: never matches
        }
        for (uint8_t i = 0; i < len; ++i) {
            if (uid_[i] != prefix[i]) {
                return false;                         // First mismatch decides
            }
        }
        return true;
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

    uint8_t  uid_[TP_UID_BYTES] = {0};  ///< Byte view handed to the discovery engine.
    uint64_t raw_ = 0;                  ///< Packed 56-bit view.
    bool     ready_ = false;            ///< init() has run.
};
