#pragma once

/**
 * @file tp_engine_registers_impl.h
 * @brief Register read, write and introspection.
 *
 * A register access is a binary search into TP_REGISTERS followed by a call
 * through the descriptor accessor pair. The write path additionally honours
 * TP_RF_DISABLED (refused while the output stage is energized), re-applies the
 * configuration to hardware when the descriptor asks for it, and replies with
 * the value that was actually stored rather than the one requested.
 *
 * Part of TpProtocol, declared in tp_engine.h. Including that header pulls this
 * one in at the end; including this one first pulls tp_engine.h, whose own
 * include of this file is then skipped by #pragma once - so the class is always
 * complete before these definitions are parsed, whichever order is used.
 *
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#include "../tp_engine.h"  // class TpProtocol - the declarations these bodies define

inline TpRegContext TpProtocol::reg_context() {
    TpRegContext c;         // Context handed to the accessors
    c.tp = tp_;             // Live firmware state
    c.cfg = &config_;       // Configuration shadow
    c.id = &identity_;      // Device identity
    c.dirty = false;        // No persisted register touched yet
    c.hw_reapply = false;   // No hardware reprogramming needed yet
    return c;
}

inline void TpProtocol::handle_reg_read(const TpFrame& f, TpFrame& tx) {
    const TpRegDesc* d = tp_find_register(f.reg.reg);        // Look the register up
    if (d == nullptr) {
        nack(tx, TP_OP_REG_READ, TP_ERR_UNKNOWN_REG, (uint8_t)(f.reg.reg & 0xFFu));
        return;
    }
    if (((d->flags & TP_RF_READ) == 0u) || (d->get == nullptr)) {
        nack(tx, TP_OP_REG_READ, TP_ERR_WRITE_ONLY, (uint8_t)(f.reg.reg & 0xFFu));
        return;
    }
    TpRegContext c = reg_context();                          // Accessor context
    uint32_t value = 0;                                      // Read-back value
    if (!d->get(c, value)) {
        nack(tx, TP_OP_REG_READ, TP_ERR_WRONG_STATE, (uint8_t)(f.reg.reg & 0xFFu));
        return;
    }
    tx.reg.cmd = TP_OP_REG_READ;                             // Echo the opcode
    tx.reg.info = (uint8_t)d->type;                          // Let a generic tool decode it
    tx.reg.reg = f.reg.reg;                                  // Echo the identifier
    tx.reg.value = value;                                    // The value itself
}

inline void TpProtocol::handle_reg_write(const TpFrame& f, TpFrame& tx) {
    const TpRegDesc* d = tp_find_register(f.reg.reg);        // Look the register up
    if (d == nullptr) {
        nack(tx, TP_OP_REG_WRITE, TP_ERR_UNKNOWN_REG, (uint8_t)(f.reg.reg & 0xFFu));
        return;
    }
    if (((d->flags & TP_RF_WRITE) == 0u) || (d->set == nullptr)) {
        nack(tx, TP_OP_REG_WRITE, TP_ERR_READ_ONLY, (uint8_t)(f.reg.reg & 0xFFu));
        return;
    }
    if (((d->flags & TP_RF_DISABLED) != 0u) && is_enabled()) {
        // Retuning a protection threshold or the machine's identity under
        // load moves a trip point beneath a live current: refuse it.
        nack(tx, TP_OP_REG_WRITE, TP_ERR_WRONG_STATE, (uint8_t)(f.reg.reg & 0xFFu));
        return;
    }
    TpRegContext c = reg_context();                          // Accessor context
    if (!d->set(c, tp_reg_normalize(d->type, f.reg.value))) {
        nack(tx, TP_OP_REG_WRITE, TP_ERR_OUT_OF_RANGE, (uint8_t)(f.reg.reg & 0xFFu));
        return;
    }
    if (c.hw_reapply && (tp_ != nullptr)) {
        tp_apply_config(*tp_, config_);                      // Push the change into hardware
    }
    // Answer with the value the drive holds at the moment of the reply, not
    // with the value that was sent, so a clamped or rounded write is
    // visible immediately. Registers that route through the control
    // mailbox (targets, mode, enable) take effect on the next control pass,
    // so their reply can still show the previous value - read the register
    // again to confirm one of those.
    uint32_t applied = f.reg.value;                          // Fallback if there is no getter
    if (d->get != nullptr) {
        (void)d->get(c, applied);
    }
    tx.reg.cmd = TP_OP_REG_WRITE;                            // Echo the opcode
    tx.reg.info = (uint8_t)d->type;                          // Type of the value
    tx.reg.reg = f.reg.reg;                                  // Echo the identifier
    tx.reg.value = applied;                                  // What is in force now
}

inline void TpProtocol::handle_reg_info(const TpFrame& f, TpFrame& tx) {
    const TpRegDesc* d = tp_find_register(f.reg.reg);        // Look the register up
    if (d == nullptr) {
        nack(tx, TP_OP_REG_INFO, TP_ERR_UNKNOWN_REG, (uint8_t)(f.reg.reg & 0xFFu));
        return;
    }
    tx.reg.cmd = TP_OP_REG_INFO;                             // Echo the opcode
    tx.reg.info = (uint8_t)d->type;                          // Payload interpretation
    tx.reg.reg = d->id;                                      // Echo the identifier
    // Byte 4 carries the access flags, byte 5 the group (the identifier's
    // high byte), bytes 6-7 the total register count so a tool can tell
    // when it has walked the whole map.
    tx.reg.value = (uint32_t)d->flags | ((uint32_t)(d->id >> 8) << 8) |
                   ((uint32_t)TP_REGISTER_COUNT << 16);
}
