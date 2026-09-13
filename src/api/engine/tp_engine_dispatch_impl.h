#pragma once

/**
 * @file tp_engine_dispatch_impl.h
 * @brief Frame routing: identifier, then CMD group, then register.
 *
 * dispatch() accepts only the command channel (0x300 + node and the 0x300
 * broadcast), reloads the watchdog, and - for an unaddressed node - lets
 * nothing but the two discovery registers through. handle_frame() then fans
 * out by CMD group: burst, write-only command, read, or register write.
 *
 * Two orderings here are load-bearing. A burst frame is tested for FIRST,
 * because its CMD byte is the top of the stream's time / counter field. And
 * the discovery registers are handled before the generic register path,
 * because an unaddressed node must answer them while refusing everything
 * else, and because their replies leave on the address the frame changed.
 *
 * respond() and nack() live here too: they are the reply mechanism the whole
 * dispatch tree shares, nack() is what counts a rejection, and answers() is
 * where the Bus Settings response policy is enforced.
 *
 * Part of TpProtocol, declared in tp_engine.h.
 *
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#include "../tp_engine.h"  // class TpProtocol - the declarations these bodies define

inline void TpProtocol::dispatch(uint16_t id, const TpFrame& f, uint8_t len) {
    (void)len;                                      // Every protocol frame is 8 bytes; short frames arrive zero-padded
    if (!tp_is_command_for(id, config_.node_id)) {
        return;                                     // Another node's traffic (promiscuous filter) or an answer
    }
    const bool broadcast = tp_is_broadcast(id);     // 0x300 reaches every node
    feed_watchdog();                                // The master is alive
    if (handled_ != 0xFFFFu) {
        handled_++;                                 // Saturating: a wrapped counter hides a storm
    }
    if (handle_discovery(f, broadcast)) {
        return;                                     // Discovery registers answer their own way
    }
    if (config_.node_id == TP_NODE_UNASSIGNED) {
        return;                                     // No address: nothing but discovery is accepted
    }
    handle_frame(f, broadcast);
}

inline bool TpProtocol::handle_discovery(const TpFrame& f, bool broadcast) {
    const uint8_t cmd = f.b8[0];                    // CMD as received
    const uint8_t reg = tp_cmd_register(cmd);       // Address with the write bit stripped
    if (reg == TP_REG_DISCOVERY_SEARCH) {
        // A search is a question whichever group bit it carries: the answer
        // is the read form, and an addressed node only takes part when the
        // master asked it directly (an address-collision check).
        uint8_t reply_depth = 0;                    // Depth to answer with
        uint64_t reply_prefix = 0;                  // Prefix to answer with
        const uint64_t prefix = tp_unpack_uid(f.search.prefix);  // Prefix the master sent
        if (TpDiscovery::on_search(identity_, config_.node_id, !broadcast, f.search.depth, prefix,
                                   reply_depth, reply_prefix)) {
            TpFrame tx;                             // Reply under construction
            tx.u64 = 0;
            tx.search.cmd = TP_REG_DISCOVERY_SEARCH;
            tx.search.depth = reply_depth;
            tp_pack_uid(tx.search.prefix, reply_prefix);
            respond(tx);                            // On 0x4FF while unassigned
        }
        return true;
    }
    if (reg != TP_REG_DISCOVERY_ASSIGN) {
        return false;                               // Not a discovery register
    }
    const uint64_t uid = tp_unpack_uid(f.discovery.uid);  // HashUID the master named
    if (tp_cmd_is_read(cmd)) {
        // A read pings: a matching HashUID (or the wildcard 0 on a directed
        // request, which asks "who are you") is answered with this node's
        // address and identity. An unassigned node answers a wildcard too -
        // that is the one-node fast path of a discovery sweep.
        const bool mine = (uid == identity_.raw());  // Named this drive
        const bool who = (uid == 0ull);              // Asked whoever hears it
        if (!mine && !who) {
            return true;                            // Meant for a different drive
        }
        TpFrame tx;                                 // Reply under construction
        tx.u64 = 0;
        tx.discovery.cmd = TP_REG_DISCOVERY_ASSIGN;
        tx.discovery.addr = config_.node_id;
        tp_pack_uid(tx.discovery.uid, identity_.raw());
        respond(tx);
        return true;
    }
    // A write assigns, resets or re-addresses.
    uint8_t assign = 0;                             // Address the master wants us to take
    if (!TpDiscovery::on_assign(identity_, uid, f.discovery.addr, broadcast, assign)) {
        return true;                                // Meant for a different drive, or a bad wildcard
    }
    if ((assign != TP_NODE_UNASSIGNED) && ((assign < TP_NODE_MIN) || (assign > TP_NODE_MAX))) {
        TpFrame tx;                                 // Reject: the broadcast address is not assignable
        nack(tx, cmd, TP_ERR_BAD_PARAM, assign);
        if (!broadcast) {
            respond(tx);
        }
        return true;
    }
    adopt_node_id(assign, true);                    // Take it, refilter, persist
    if (!broadcast && (assign != TP_NODE_UNASSIGNED)) {
        TpFrame tx;                                 // Confirmation on the NEW address
        tx.u64 = 0;
        tx.discovery.cmd = TP_REG_DISCOVERY_ASSIGN;
        tx.discovery.addr = config_.node_id;
        tp_pack_uid(tx.discovery.uid, identity_.raw());
        respond(tx);
    }
    return true;
}

inline void TpProtocol::handle_frame(const TpFrame& f, bool broadcast) {
    const uint8_t cmd = f.b8[0];       // CMD byte
    TpFrame tx;                        // Reply under construction
    tx.u64 = 0;                        // Start from a clean frame

    // ---- Burst stream: bits 7..5 set, no register address at all -------
    if (tp_cmd_is_burst(cmd)) {
        const TpError e = handle_burst(f);
        if (e != TP_ERR_NONE) {
            nack(tx, cmd, e, 0u);
        } else {
            ack(tx, cmd, 0u);
        }
        if (!broadcast && answers(false, e == TP_ERR_NONE)) {
            respond(tx);
        }
        return;
    }

    // ---- Write-only commands ---------------------------------------
    if (tp_cmd_is_write_only(cmd)) {
        TpError e = TP_ERR_UNKNOWN_CMD;             // Result of the command
        bool setpoint = false;                      // Streaming frame under the response policy
        switch (cmd) {
            case TP_CMD_MOTION_TASK_1:
            case TP_CMD_MOTION_TASK_2:
            case TP_CMD_MOTION_TASK_3:
            case TP_CMD_MOTION_TASK_4:
                e = handle_motion_task(f);
                setpoint = true;
                break;
            case TP_CMD_SAVE_CONFIG:
                e = handle_save(f);
                break;
            case TP_CMD_MIDI:
                e = handle_midi(f);
                break;
            default:
                break;                              // Unknown write-only register
        }
        if (e != TP_ERR_NONE) {
            nack(tx, cmd, e, f.b8[1]);
        } else {
            ack(tx, cmd, f.b8[1]);
        }
        // A broadcast write is executed by every node but answered by none:
        // 250 simultaneous acknowledgements would be a burst, not information.
        if (!broadcast && answers(false, setpoint && (e == TP_ERR_NONE))) {
            respond(tx);
        }
        return;
    }

    // ---- Reads: registers, batch and range requests -------------------
    if (tp_cmd_is_read(cmd)) {
        if (cmd == TP_REG_BATCH_READ) {
            const TpError e = handle_batch_read(f);  // Replies flow from service_batch()
            if ((e != TP_ERR_NONE) && !broadcast) {
                nack(tx, cmd, e, 0u);
                respond(tx);
            }
            return;
        }
        if (cmd == TP_REG_RANGE_READ) {
            const TpError e = handle_range_read(f);  // Replies flow from service_batch()
            if ((e != TP_ERR_NONE) && !broadcast) {
                nack(tx, cmd, e, f.b8[1]);
                respond(tx);
            }
            return;
        }
        const TpRegDesc* d = tp_find_register(cmd); // Look the register up
        if (d == nullptr) {
            nack(tx, cmd, TP_ERR_UNKNOWN_CMD, 0u);
        } else {
            const uint8_t sub = ((d->flags & TP_RF_SUB) != 0u) ? f.b8[1] : 0u;  // Record selector
            const TpError e = read_register(cmd, sub, f, tx);
            if (e != TP_ERR_NONE) {
                nack(tx, cmd, e, sub);
            }
        }
        // Even a broadcast read gets an answer: every node replies on its
        // own identifier, which is exactly how a master inventories a bus.
        if (answers(true, false)) {
            respond(tx);
        }
        return;
    }

    // ---- Register writes ------------------------------------------------
    const TpError e = write_register(f, tx);        // Apply, then read back into tx
    if (e != TP_ERR_NONE) {
        nack(tx, cmd, e, f.b8[1]);
    }
    if (!broadcast && answers(false, false)) {
        respond(tx);
    }
}

inline bool TpProtocol::answers(bool is_read, bool is_setpoint) const {
    switch (config_.comm_mode) {
        case TP_COMM_SILENT:
            return false;                            // Never a word
        case TP_COMM_EVERY:
            return true;                             // Everything is acknowledged
        case TP_COMM_ON_REQUEST:
        default:
            // Reads and register writes are requests and get their answer; an
            // accepted setpoint is streaming traffic and stays silent, a
            // rejected one is still worth a NACK.
            return is_read || !is_setpoint;
    }
}

inline void TpProtocol::respond(const TpFrame& f) {
    if ((write_ != nullptr) && (config_.comm_mode != TP_COMM_SILENT)) {
        write_((uint16_t)(TP_ID_RESPONSE_BASE + config_.node_id), f);
    }
}

inline void TpProtocol::nack(TpFrame& f, uint8_t cmd, TpError err, uint8_t detail) {
    f.u64 = 0;                     // Reserved bytes transmit as zero
    f.ack.cmd = TP_CMD_NACK;       // Generic failure reply
    f.ack.echo = cmd;              // What was refused
    f.ack.error = (uint8_t)err;    // Why it was refused
    f.ack.detail = detail;         // Context: sub id, segment id, fault mask
    if (rejected_ != 0xFFFFu) {
        rejected_++;               // Saturating: a wrapped counter hides a storm
    }
}

// ============================================================================
// Batch and range reads
// ============================================================================

inline TpError TpProtocol::handle_batch_read(const TpFrame& f) {
    uint8_t prev = 0xFFu;                            // Previous explicit register (0xFF = none)
    bool range_open = false;                         // A filler followed an explicit register
    bool any = false;                                // At least one readable register named
    for (uint32_t i = 0; i < 7u; ++i) {
        const uint8_t r = f.batch.regs[i];           // Entry under consideration
        if (r == TP_REG_BATCH_READ) {
            range_open = (prev != 0xFFu);            // One filler between two registers spans them
            continue;
        }
        if (r > 0x7Fu) {
            return TP_ERR_BAD_PARAM;                 // Not a read register
        }
        uint8_t first = r;                           // First register to mark
        if (range_open && (r > prev)) {
            first = (uint8_t)(prev + 1u);            // Fill the span after the previous one
        }
        range_open = false;
        for (uint32_t k = first; k <= r; ++k) {
            const TpRegDesc* d = tp_find_register((uint8_t)k);  // Only real registers are queued
            if ((d != nullptr) && ((d->flags & TP_RF_READ) != 0u)) {
                batch_bits_[k >> 3] |= (uint8_t)(1u << (k & 7u));
                any = true;
            }
        }
        prev = r;
    }
    if (!any) {
        return TP_ERR_BAD_PARAM;                     // Nothing readable was named
    }
    batch_pending_ = true;
    return TP_ERR_NONE;
}

inline TpError TpProtocol::handle_range_read(const TpFrame& f) {
    const TpRegDesc* d = tp_find_register(f.range.reg);  // Register to walk
    if ((d == nullptr) || ((d->flags & TP_RF_READ) == 0u) || (f.range.reg > 0x7Fu)) {
        return TP_ERR_UNKNOWN_CMD;                   // Not a readable register
    }
    uint32_t start = tp_unpack_u24(f.range.start);   // First record
    uint32_t end = tp_unpack_u24(f.range.end);       // Last record, inclusive
    if (end < start) {
        return TP_ERR_BAD_PARAM;                     // Empty range
    }
    if ((d->flags & TP_RF_SUB) != 0u) {
        if (start >= d->subs) {
            return TP_ERR_UNKNOWN_SUB;               // Beyond the sub-space
        }
        if (end >= d->subs) {
            end = d->subs - 1u;                      // Clamp to the last record
        }
    } else if (f.range.reg == TP_REG_DIRECT) {
        start &= ~3u;                                // Word aligned
        end &= ~3u;
    } else if (f.range.reg != TP_REG_CAL_TABLE) {
        start = 0u;                                  // A plain register has one record
        end = 0u;
    }
    if ((end - start) > 4095u) {
        return TP_ERR_OUT_OF_RANGE;                  // Bound a runaway walk
    }
    range_active_ = true;                            // service_batch() emits from here on
    range_cmd_ = f.range.reg;
    range_cursor_ = start;
    range_end_ = end;
    return TP_ERR_NONE;
}

inline void TpProtocol::service_batch() {
    if (config_.node_id == TP_NODE_UNASSIGNED) {
        return;                                      // A node must stay silent without an address
    }
    uint32_t budget = REPLY_BUDGET;                  // Replies this call may emit

    // ---- Range walk first: it was asked for as one unit ---------------
    while (range_active_ && (budget > 0u)) {
        TpFrame req;                                 // Synthesized request naming the record
        req.u64 = 0;
        req.b8[0] = range_cmd_;
        uint8_t sub = 0;                             // Record selector for sub-spaces
        if (range_cmd_ == TP_REG_DIRECT) {
            tp_pack_u24(req.direct.addr, range_cursor_);  // Address under the cursor
        } else if (range_cmd_ == TP_REG_CAL_TABLE) {
            req.cal_table.sample = (uint16_t)range_cursor_;  // Sample under the cursor
        } else {
            sub = (uint8_t)range_cursor_;            // Sub identifier under the cursor
            req.b8[1] = sub;
        }
        TpFrame tx;                                  // Reply under construction
        tx.u64 = 0;
        const TpError e = read_register(range_cmd_, sub, req, tx);
        if (e != TP_ERR_NONE) {
            nack(tx, range_cmd_, e, sub);            // Say which record could not be read
        }
        respond(tx);
        budget--;
        if (range_cursor_ >= range_end_) {
            range_active_ = false;                   // Walk complete
        } else {
            range_cursor_ += (range_cmd_ == TP_REG_DIRECT) ? 4u : 1u;
        }
    }

    // ---- Batch bitmap ---------------------------------------------------
    while (batch_pending_ && (budget > 0u)) {
        uint32_t k = 0;                              // Lowest pending register
        while ((k < 0x80u) && ((batch_bits_[k >> 3] & (1u << (k & 7u))) == 0u)) {
            k++;
        }
        if (k >= 0x80u) {
            batch_pending_ = false;                  // Bitmap drained
            break;
        }
        const TpRegDesc* d = tp_find_register((uint8_t)k);  // Register to emit
        const uint8_t subs = ((d != nullptr) && ((d->flags & TP_RF_SUB) != 0u)) ? d->subs : 1u;
        // A sub-space register is emitted record by record.
        TpFrame req;                                 // Synthesized request
        req.u64 = 0;
        req.b8[0] = (uint8_t)k;
        req.b8[1] = batch_sub_;
        TpFrame tx;                                  // Reply under construction
        tx.u64 = 0;
        const TpError e = read_register((uint8_t)k, batch_sub_, req, tx);
        if (e != TP_ERR_NONE) {
            nack(tx, (uint8_t)k, e, batch_sub_);
        }
        respond(tx);
        budget--;
        batch_sub_++;
        if (batch_sub_ >= subs) {
            batch_sub_ = 0u;                         // Next register starts at record 0
            batch_bits_[k >> 3] &= (uint8_t)~(1u << (k & 7u));  // This one is done
        }
    }
}
