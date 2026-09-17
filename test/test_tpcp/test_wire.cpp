/**
 * @file test_wire.cpp
 * @brief Host-side conformance tests for the TunePulse CAN Protocol wire format.
 *
 * Covers the parts of the protocol that are pure data and therefore testable
 * without any hardware: the fixed-point codecs, the 8-byte frame layouts, the
 * CMD group classification, the bit-packed burst frame, the identifier
 * helpers, the register map ordering and the discovery search logic. A drive
 * and a host tool disagreeing about any of these is a silent, hard-to-debug
 * failure on a live bus, which is exactly the class of bug worth pinning down
 * at build time.
 *
 * Run with:  pio test -e native_tpcp
 *
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#include <unity.h>

#include <cmath>
#include <cstdint>

#include "../../src/api/tp_chunks.h"
#include "../../src/api/registers/tp_register_map.h"
#include "../../src/api/node/tp_node_identity.h"
#include "../../src/api/node/tp_discovery.h"
#include "../../src/core/math/crc.h"
#include "../../src/core/math/gearbox.h"

/** Unity per-test setup hook (nothing to prepare). */
void setUp(void) {}

/** Unity per-test teardown hook (nothing to release). */
void tearDown(void) {}

// ============================================================================
// Codecs
// ============================================================================

/** bfloat16 keeps full float range at roughly 0.4 % relative accuracy, and NaN is detectable. */
void test_bfloat16_roundtrip(void) {
    const float samples[] = {0.0f, 1.0f, -1.0f, 24.0f, 0.0016f, 51.0f, 1e-6f, 1e6f};
    for (unsigned i = 0; i < sizeof(samples) / sizeof(samples[0]); ++i) {
        const float v = samples[i];                                  // Value under test
        const float r = tp_unpack_bf16(tp_pack_bf16(v));             // Round trip
        const float err = (v == 0.0f) ? std::fabs(r) : std::fabs((r - v) / v);
        TEST_ASSERT_TRUE_MESSAGE(err < 0.01f, "bfloat16 relative error above 1 %");
        TEST_ASSERT_TRUE(tp_bf16_is_finite(tp_pack_bf16(v)));
    }
    TEST_ASSERT_FALSE(tp_bf16_is_finite(TP_BF16_NAN));
    TEST_ASSERT_FALSE(tp_bf16_is_finite(tp_pack_bf16(INFINITY)));
}

/** float24 keeps full float range at roughly 0.008 % accuracy; negatives must stay finite. */
void test_float24_roundtrip(void) {
    const float samples[] = {0.0f, 1.0f, -1.0f, -12.5f, 3000.0f, -0.001f, 1e-6f};
    for (unsigned i = 0; i < sizeof(samples) / sizeof(samples[0]); ++i) {
        const float v = samples[i];                                  // Value under test
        const float r = tp_unpack_f24(tp_pack_f24(v));               // Round trip
        TEST_ASSERT_TRUE_MESSAGE(std::isfinite(r), "float24 decoded to a non-finite value");
        const float err = (v == 0.0f) ? std::fabs(r) : std::fabs((r - v) / v);
        TEST_ASSERT_TRUE_MESSAGE(err < 1e-4f, "float24 relative error above 0.01 %");
        TEST_ASSERT_TRUE(tp_f24_is_finite(tp_pack_f24(v)));
    }
    TEST_ASSERT_FALSE(tp_f24_is_finite(tp_pack_f24(NAN)));
}

/** The 48-bit and 56-bit fields must survive both extremes of their ranges. */
void test_wide_fields_roundtrip(void) {
    const int64_t s48[] = {0, 65536, -65536, 140737488355327LL, -140737488355328LL};
    for (unsigned i = 0; i < sizeof(s48) / sizeof(s48[0]); ++i) {
        uint8_t bytes[6];                                            // Wire representation
        tp_pack_i48(bytes, s48[i]);
        TEST_ASSERT_EQUAL_INT64(s48[i], tp_unpack_i48(bytes));
    }
    const int64_t s56[] = {0, 1, -1, 36028797018963967LL, -36028797018963968LL};
    for (unsigned i = 0; i < sizeof(s56) / sizeof(s56[0]); ++i) {
        uint8_t bytes[7];                                            // Wire representation
        tp_pack_i56(bytes, s56[i]);
        TEST_ASSERT_EQUAL_INT64(s56[i], tp_unpack_i56(bytes));
    }
    const int32_t s24[] = {0, 1, -1, 8388607, -8388608};
    for (unsigned i = 0; i < sizeof(s24) / sizeof(s24[0]); ++i) {
        uint8_t bytes[3];                                            // Wire representation
        tp_pack_i24(bytes, s24[i]);
        TEST_ASSERT_EQUAL_INT32(s24[i], tp_unpack_i24(bytes));
    }
}

/** A native q1.15.16 position widens to q1.23.24 and back without loss; wider values saturate. */
void test_position_conversions(void) {
    TEST_ASSERT_EQUAL_INT64(65536LL << 8, tp_q16_to_q24(65536));
    TEST_ASSERT_EQUAL_INT32(-65536, tp_q24_to_q16(tp_q16_to_q24(-65536)));
    TEST_ASSERT_EQUAL_INT32(2147483647, tp_q24_to_q16((int64_t)1 << 47));   // Saturates, never wraps
    TEST_ASSERT_EQUAL_INT32(-2147483647 - 1, tp_q24_to_q16(-((int64_t)1 << 47)));
    TEST_ASSERT_EQUAL_INT64(65536LL << 16, tp_q16_to_q32(65536));
    TEST_ASSERT_EQUAL_INT32(65536, tp_q32_to_q16(tp_q16_to_q32(65536)));
}

/** The Q10 time scale must survive the trip through its 12-bit wire form. */
void test_timescale_roundtrip(void) {
    const int32_t samples[] = {1024, 0, -1024, 512, 2048};
    for (unsigned i = 0; i < sizeof(samples) / sizeof(samples[0]); ++i) {
        TEST_ASSERT_EQUAL_INT32(samples[i], tp_unpack_timescale(tp_pack_timescale(samples[i])));
    }
}

/** The single-byte codecs must scale correctly and saturate rather than wrap. */
void test_byte_codecs(void) {
    TEST_ASSERT_EQUAL_UINT8(0, tp_pack_temp_u8(NAN));                // No data
    TEST_ASSERT_EQUAL_UINT8(25, tp_pack_temp_u8(25.2f));             // Nearest degree
    TEST_ASSERT_EQUAL_UINT8(255, tp_pack_temp_u8(300.0f));           // Over range
    TEST_ASSERT_EQUAL_UINT8(1, tp_pack_temp_u8(-40.0f));             // Never aliases "no data"
    TEST_ASSERT_EQUAL_UINT8(0x80, tp_pack_usage_i8(1.5f));           // Overload marker
    TEST_ASSERT_EQUAL_INT8(-127, (int8_t)tp_pack_usage_i8(-2.0f));   // Regen saturates
    TEST_ASSERT_EQUAL_INT8(63, (int8_t)tp_pack_usage_i8(0.5f));      // Half the limit
    TEST_ASSERT_EQUAL_UINT8(16, tp_pack_q1_1_4(1.0f));               // q1.1.4: 16 == 1.0
    TEST_ASSERT_EQUAL_UINT8(31, tp_pack_q1_1_4(9.0f));               // Saturates at +1.9375
    TEST_ASSERT_EQUAL_UINT8(0x20, tp_pack_q1_1_4(-2.0f));            // -32 in six bits
}

/** Baud codes map onto the standard rates and back. */
void test_baud_codes(void) {
    TEST_ASSERT_EQUAL_UINT16(500, tp_baud_kbps(TP_BAUD_500K));
    TEST_ASSERT_EQUAL_UINT8(TP_BAUD_1M, tp_baud_code(1000));
    TEST_ASSERT_EQUAL_UINT8(TP_BAUD_KEEP, tp_baud_code(333));        // Not a standard rate
}

// ============================================================================
// CMD groups
// ============================================================================

/** The three group bits classify every CMD byte exactly once. */
void test_cmd_groups(void) {
    for (unsigned c = 0; c < 256u; ++c) {
        const uint8_t cmd = (uint8_t)c;                              // CMD under test
        const int n = (tp_cmd_is_rw_read(cmd) ? 1 : 0) + (tp_cmd_is_read_only(cmd) ? 1 : 0) +
                      (tp_cmd_is_rw_write(cmd) ? 1 : 0) + (tp_cmd_is_write_only(cmd) ? 1 : 0) +
                      (tp_cmd_is_burst(cmd) ? 1 : 0);
        TEST_ASSERT_EQUAL_INT_MESSAGE(1, n, "a CMD byte fell into zero or several groups");
    }
    TEST_ASSERT_TRUE(tp_cmd_is_rw_read(0x21));
    TEST_ASSERT_TRUE(tp_cmd_is_rw_write(0x21 | TP_CMD_WRITE));
    TEST_ASSERT_EQUAL_UINT8(0x21, tp_cmd_register(0xA1));
    TEST_ASSERT_TRUE(tp_cmd_is_read_only(TP_REG_DRIVER_STATUS));
    TEST_ASSERT_TRUE(tp_cmd_is_write_only(TP_CMD_MOTION_TASK_1));
    TEST_ASSERT_TRUE(tp_cmd_is_write_only(TP_CMD_MIDI));
    TEST_ASSERT_TRUE(tp_cmd_is_burst(0xE0));
    TEST_ASSERT_TRUE(tp_cmd_is_burst(0xFF));
    TEST_ASSERT_TRUE(tp_cmd_is_pulsebeat(0x7A));
    TEST_ASSERT_FALSE(tp_cmd_is_pulsebeat(0x6F));
    TEST_ASSERT_TRUE(tp_cmd_is_read(TP_CMD_PULSEBEAT));
}

/** The register map is sorted (binary search) and every listed CMD is a read-form address. */
void test_register_map(void) {
    for (unsigned i = 1; i < TP_REGISTER_COUNT; ++i) {
        TEST_ASSERT_TRUE_MESSAGE(TP_REGISTERS[i - 1].cmd < TP_REGISTERS[i].cmd, "register map not sorted");
    }
    for (unsigned i = 0; i < TP_REGISTER_COUNT; ++i) {
        TEST_ASSERT_TRUE(tp_cmd_is_read(TP_REGISTERS[i].cmd));
        TEST_ASSERT_EQUAL_PTR(&TP_REGISTERS[i], tp_find_register(TP_REGISTERS[i].cmd));
        TEST_ASSERT_EQUAL_PTR(&TP_REGISTERS[i], tp_find_register((uint8_t)(TP_REGISTERS[i].cmd | TP_CMD_WRITE)));
    }
    TEST_ASSERT_NULL(tp_find_register(0x3E));                        // Unallocated address
    TEST_ASSERT_EQUAL_UINT8(TP_CMD_PULSEBEAT, tp_find_register(0x7C)->cmd);  // Any beat counter
    TEST_ASSERT_EQUAL_UINT8(TP_SUB_AXIS_COUNT, tp_find_register(TP_REG_AXIS)->subs);
}

// ============================================================================
// Frame layouts
// ============================================================================

/** Every typed view must occupy exactly one classic CAN payload. */
void test_frame_is_eight_bytes(void) {
    TEST_ASSERT_EQUAL_UINT32(8u, (uint32_t)sizeof(TpFrame));
    TEST_ASSERT_EQUAL_UINT32(8u, (uint32_t)TP_FRAME_SIZE);
}

/** Packed fields land exactly where the specification puts them. */
void test_register_layouts(void) {
    TpFrame f;
    f.u64 = 0;
    f.supply_prot.cmd = TP_REG_SUPPLY_PROTECTION | TP_CMD_WRITE;
    f.supply_prot.hysteresis = 0x3F80;                               // 1.0 V at bytes 1-2
    f.supply_prot.recovery = 7;                                      // Byte 3
    f.supply_prot.ov = 0x424C;                                       // 51 V at bytes 4-5
    TEST_ASSERT_EQUAL_UINT8(0xA6, f.b8[0]);
    TEST_ASSERT_EQUAL_UINT8(0x80, f.b8[1]);                          // Little-endian low byte first
    TEST_ASSERT_EQUAL_UINT8(0x3F, f.b8[2]);
    TEST_ASSERT_EQUAL_UINT8(7, f.b8[3]);
    TEST_ASSERT_EQUAL_UINT8(0x4C, f.b8[4]);

    f.u64 = 0;
    f.motor1.conn_dir_poles = (uint16_t)(1u | (0x3u << 2) | (50u << 4));  // ABC, CW, 50 pole pairs
    TEST_ASSERT_EQUAL_UINT8(0x2D, f.b8[2]);                          // 0b0010'1101
    TEST_ASSERT_EQUAL_UINT8(0x03, f.b8[3]);

    f.u64 = 0;
    f.motion1.queue_ts = tp_beat_queue_ts(15, 1024);                 // Depth 15, 1.0x
    TEST_ASSERT_EQUAL_UINT8(0x0F, f.b8[6]);
    TEST_ASSERT_EQUAL_UINT8(0x10, f.b8[7]);                          // q1.3.8: 256 == 1.0x
}

/** Motion Task 1 packs its flags from the LSB upward as listed. */
void test_motion_task_flags(void) {
    const uint16_t fl = tp_mt1_flags(TP_MT_PVT, true, TP_MT_POS_TARGET, TP_MT_TORQUE_LIMIT,
                                     TP_MT_TIME_DURATION, false, true, false, false);
    TEST_ASSERT_EQUAL_UINT8(TP_MT_PVT, TP_MT1_MODE(fl));
    TEST_ASSERT_TRUE(TP_MT1_IS_ABS(fl));
    TEST_ASSERT_EQUAL_UINT8(TP_MT_POS_TARGET, TP_MT1_POS_MODE(fl));
    TEST_ASSERT_EQUAL_UINT8(TP_MT_TORQUE_LIMIT, TP_MT1_TORQ_MODE(fl));
    TEST_ASSERT_EQUAL_UINT8(TP_MT_TIME_DURATION, TP_MT1_TIME_MODE(fl));
    TEST_ASSERT_FALSE(TP_MT1_IO_TRIG(fl));
    TEST_ASSERT_TRUE(TP_MT1_IS_VEL(fl));
    TEST_ASSERT_FALSE(TP_MT1_IS_ACC(fl));
    TEST_ASSERT_EQUAL_UINT16(0x1000u | 0x200u | 0x80u | 0x20u | 0x10u | 0x6u, fl);
}

/** The PulseBeat groups collapse the fault and warning bytes as documented. */
void test_pulsebeat_groups(void) {
    TEST_ASSERT_EQUAL_UINT8(TP_FAULT_GROUP_NONE, tp_fault_group(0));
    TEST_ASSERT_EQUAL_UINT8(TP_FAULT_GROUP_OVERCURRENT, tp_fault_group(0x02));
    TEST_ASSERT_EQUAL_UINT8(TP_FAULT_GROUP_PHASE, tp_fault_group(0x40));
    TEST_ASSERT_EQUAL_UINT8(TP_FAULT_GROUP_MULTIPLE, tp_fault_group(0x03));
    TEST_ASSERT_EQUAL_UINT8(0, tp_warn_group(0));
    TEST_ASSERT_EQUAL_UINT8(8, tp_warn_group(0x81));                 // Highest bit wins
    TpFrame f;
    f.u64 = 0;
    f.beat.warn_mode_err = tp_beat_warn_mode_err(3, TP_MODE_MOTION, 1.0f);
    TEST_ASSERT_EQUAL_UINT16(3u | (4u << 4) | (16u << 10), f.beat.warn_mode_err);
}

// ============================================================================
// Burst frame
// ============================================================================

/** Every field of a PVT burst frame survives the round trip, MSB first. */
void test_burst_pvt_roundtrip(void) {
    uint8_t b[8];                                                    // Wire representation
    tp_burst_pack_pvt(b, 0x1FFFFFu, -12.5f, 65536 * 3);
    TEST_ASSERT_TRUE(tp_burst_is(b));
    TEST_ASSERT_EQUAL_UINT8(0xFF, b[0]);                             // 111 + top five time bits
    TEST_ASSERT_EQUAL_UINT32(0x1FFFFFu, tp_burst_time21(b));
    TEST_ASSERT_EQUAL_FLOAT(-12.5f, tp_unpack_bf16(tp_burst_vel_raw(b)));
    TEST_ASSERT_EQUAL_INT32(65536 * 3, tp_burst_pos24(b));
    tp_burst_pack_pvt(b, 5u, 1.0f, -65536);
    TEST_ASSERT_EQUAL_UINT32(5u, tp_burst_time21(b));
    TEST_ASSERT_EQUAL_INT32(-65536, tp_burst_pos24(b));               // Sign-extended from bit 23
}

/** Every field of a PVcTF burst frame survives the round trip. */
void test_burst_pvctf_roundtrip(void) {
    uint8_t b[8];                                                    // Wire representation
    tp_burst_pack_pvctf(b, 0x15u, 0.25f, 2.0f, 0x7FFFFF);
    TEST_ASSERT_TRUE(tp_burst_is(b));
    TEST_ASSERT_EQUAL_UINT8(0x15u, tp_burst_frame(b));
    TEST_ASSERT_EQUAL_FLOAT(0.25f, tp_unpack_bf16(tp_burst_force_raw(b)));
    TEST_ASSERT_EQUAL_FLOAT(2.0f, tp_unpack_bf16(tp_burst_vel_raw(b)));
    TEST_ASSERT_EQUAL_INT32(0x7FFFFF, tp_burst_pos24(b));
}

/** The masked position resolves to the absolute value nearest the reference. */
void test_burst_unmask(void) {
    const int32_t ref = 1000 * 65536;                                // 1000 revolutions
    const int32_t target = ref + 3 * 65536;                          // Three turns further
    uint8_t b[8];                                                    // Wire representation
    tp_burst_pack_pvt(b, 0u, 0.0f, target);
    TEST_ASSERT_EQUAL_INT32(target, tp_burst_unmask(tp_burst_pos24(b), ref));
    tp_burst_pack_pvt(b, 0u, 0.0f, ref - 7 * 65536);                 // Seven turns back
    TEST_ASSERT_EQUAL_INT32(ref - 7 * 65536, tp_burst_unmask(tp_burst_pos24(b), ref));
    tp_burst_pack_pvt(b, 0u, 0.0f, -2 * 65536);                      // Across zero
    TEST_ASSERT_EQUAL_INT32(-2 * 65536, tp_burst_unmask(tp_burst_pos24(b), 65536));
}

/** No register address may ever be mistaken for a burst frame. */
void test_burst_never_aliases_a_register(void) {
    for (unsigned c = 0; c < 0xE0u; ++c) {
        const uint8_t frame[8] = {(uint8_t)c, 0, 0, 0, 0, 0, 0, 0};
        TEST_ASSERT_FALSE_MESSAGE(tp_burst_is(frame), "a register CMD was decoded as a burst");
    }
}

// ============================================================================
// Identifier map
// ============================================================================

/** A node answers its own command channel and the broadcast, nothing else. */
void test_identifier_matching(void) {
    TEST_ASSERT_TRUE(tp_is_command_for(0x307, 7));
    TEST_ASSERT_TRUE(tp_is_command_for(0x300, 7));                    // Broadcast
    TEST_ASSERT_FALSE(tp_is_command_for(0x308, 7));                   // A neighbour's address
    TEST_ASSERT_FALSE(tp_is_command_for(0x407, 7));                   // The response channel
    TEST_ASSERT_TRUE(tp_is_command_for(0x3FF, TP_NODE_UNASSIGNED));   // Unassigned node listens on 0x3FF
    TEST_ASSERT_TRUE(tp_is_broadcast(0x300));
    TEST_ASSERT_FALSE(tp_is_broadcast(0x3FF));
    TEST_ASSERT_EQUAL_UINT16(TP_ID_RESPONSE_BASE, tp_id_class(0x4FF));
    TEST_ASSERT_EQUAL_UINT8(0x12, tp_id_node(0x512));
    TEST_ASSERT_TRUE(tp_is_node_to_master(0x107));
    TEST_ASSERT_TRUE(tp_is_node_to_master(0x407));
    TEST_ASSERT_TRUE(tp_is_node_to_master(0x507));
    TEST_ASSERT_FALSE(tp_is_node_to_master(0x307));
}

/** The channel bases must not overlap and must all fit eleven bits. */
void test_channel_bases_are_distinct(void) {
    TEST_ASSERT_TRUE(TP_ID_EMCY_BASE < TP_ID_COMMAND_BASE);
    TEST_ASSERT_TRUE(TP_ID_COMMAND_BASE < TP_ID_RESPONSE_BASE);
    TEST_ASSERT_TRUE(TP_ID_RESPONSE_BASE < TP_ID_TELEMETRY_BASE);
    TEST_ASSERT_TRUE((TP_ID_TELEMETRY_BASE + 0xFFu) <= 0x7FFu);       // Everything fits 11 bits
}

// ============================================================================
// Discovery
// ============================================================================

/** The search answers only matching prefixes, echoes the matched prefix, and deepens by one. */
void test_discovery_search(void) {
    TpNodeIdentity id;                                               // Host identity (fixed fold)
    id.init();
    const uint64_t uid = id.raw();                                   // The HashUID under search
    TEST_ASSERT_EQUAL_UINT64(0ull, uid >> TP_UID_BITS);              // Only 48 bits populated
    TEST_ASSERT_NOT_EQUAL(0ull, uid);                                // Never the wildcard

    uint8_t depth = 0;                                               // Reply depth
    uint64_t prefix = 0;                                             // Reply prefix
    // Depth 0 matches every node and answers depth 1 with an empty prefix.
    TEST_ASSERT_TRUE(TpDiscovery::on_search(id, TP_NODE_UNASSIGNED, false, 0, 0xFFFFFFFFFFFFull, depth, prefix));
    TEST_ASSERT_EQUAL_UINT8(1, depth);
    TEST_ASSERT_EQUAL_UINT64(0ull, prefix);
    // A matching 17-bit prefix answers depth 18 and echoes exactly those 17 bits.
    const uint64_t p17 = uid & 0x1FFFFull;                           // First 17 bits of the identity
    TEST_ASSERT_TRUE(TpDiscovery::on_search(id, TP_NODE_UNASSIGNED, false, 17, p17 | (1ull << 40), depth, prefix));
    TEST_ASSERT_EQUAL_UINT8(18, depth);
    TEST_ASSERT_EQUAL_UINT64(p17, prefix);
    // A prefix that differs in one of its bits is not this node's subtree.
    TEST_ASSERT_FALSE(TpDiscovery::on_search(id, TP_NODE_UNASSIGNED, false, 17, p17 ^ 1ull, depth, prefix));
    // The full identity answers depth 48 with the whole HashUID.
    TEST_ASSERT_TRUE(TpDiscovery::on_search(id, TP_NODE_UNASSIGNED, false, 48, uid, depth, prefix));
    TEST_ASSERT_EQUAL_UINT8(48, depth);
    TEST_ASSERT_EQUAL_UINT64(uid, prefix);
    // Deeper than a HashUID is malformed.
    TEST_ASSERT_FALSE(TpDiscovery::on_search(id, TP_NODE_UNASSIGNED, false, 49, uid, depth, prefix));
    // An addressed node ignores a broadcast search but answers a directed one.
    TEST_ASSERT_FALSE(TpDiscovery::on_search(id, 7, false, 0, 0, depth, prefix));
    TEST_ASSERT_TRUE(TpDiscovery::on_search(id, 7, true, 0, 0, depth, prefix));
}

/** Assignment applies to the named HashUID; the wildcard only resets. */
void test_discovery_assign(void) {
    TpNodeIdentity id;                                               // Host identity (fixed fold)
    id.init();
    uint8_t assign = 0;                                              // Address to adopt
    TEST_ASSERT_TRUE(TpDiscovery::on_assign(id, id.raw(), 7, false, assign));
    TEST_ASSERT_EQUAL_UINT8(7, assign);
    TEST_ASSERT_FALSE(TpDiscovery::on_assign(id, id.raw() ^ 1ull, 7, false, assign));   // Another drive
    TEST_ASSERT_TRUE(TpDiscovery::on_assign(id, 0ull, TP_NODE_UNASSIGNED, true, assign));  // Reset all
    TEST_ASSERT_EQUAL_UINT8(TP_NODE_UNASSIGNED, assign);
    TEST_ASSERT_FALSE(TpDiscovery::on_assign(id, 0ull, 7, true, assign));               // Wildcard assign refused
}

// ============================================================================
// Gearbox
// ============================================================================

/**
 * The gearbox sits on the API boundary: the wire speaks output-shaft units,
 * the drive motor-shaft units. Q32.32 positions must cross the integer ratio
 * exactly (modulo the low fractional bits of the internal pre-shift), the
 * derivatives ride the precomputed float multipliers, and a degenerate
 * divisor must remap to the identity instead of dividing by zero.
 */
void test_gearbox_conversion(void) {
    Gearbox gb;                                                      // 3:1 reducer under test
    gb.set_ratio(3, 1);
    TEST_ASSERT_EQUAL_INT64(6LL << 32, gb.pos_int2ext(2LL << 32));        // 2 motor rev -> 6 output rev
    TEST_ASSERT_EQUAL_INT64(2LL << 32, gb.pos_ext2int(6LL << 32));       // And back
    TEST_ASSERT_EQUAL_INT64(-(6LL << 32), gb.pos_int2ext(-(2LL << 32)));  // The sign survives

    Gearbox half;                                                    // 1:2 overdrive
    half.set_ratio(1, 2);
    TEST_ASSERT_EQUAL_FLOAT(0.5f, half.vel_int2ext(1.0f));           // Wire-side velocity
    TEST_ASSERT_EQUAL_FLOAT(4.0f, half.vel_ext2int(2.0f));           // Commanded velocity
    TEST_ASSERT_EQUAL_FLOAT(4.0f, half.acc_ext2int(2.0f));           // Acceleration shares the linear gain
    TEST_ASSERT_EQUAL_FLOAT(8.0f, half.jerk_ext2int(2.0f));          // Jerk too

    Gearbox degen;                                                   // Degenerate denominator
    degen.set_ratio(5, 0);
    TEST_ASSERT_EQUAL_INT64(10LL << 32, degen.pos_int2ext(2LL << 32));    // Remapped to 1, never a trap
}

/**
 * The shared CRC module: the three named algorithms against their published
 * check values, the streaming context against the one-shot (any split of the
 * data, restart between uses), and the little-endian word feed - the byte
 * order every stored record relies on. The firmware's static_asserts cover
 * the check values too; this keeps them on the host where a failure prints.
 */
void test_crc_module(void) {
    const uint8_t v[9] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
    TEST_ASSERT_EQUAL_HEX8(0xF4u, Crc::crc8(v, 9));             // CRC-8 (poly 0x07)
    TEST_ASSERT_EQUAL_HEX16(0x29B1u, Crc::crc16(v, 9));         // CRC-16/CCITT-FALSE
    TEST_ASSERT_EQUAL_HEX32(0x0376E6E7u, Crc::crc32(v, 9));     // CRC-32/MPEG-2
    TEST_ASSERT_EQUAL_HEX8(0x00u, Crc::crc8(v, 0));             // Empty input: the initial value
    TEST_ASSERT_EQUAL_HEX32(0xFFFFFFFFu, Crc::crc32(v, 0));

    Crc::Stream st;                                             // Streaming == one-shot, any split
    for (uint8_t width = 8; width <= 32; width = (uint8_t)(width * 2)) {
        for (size_t cut = 0; cut <= 9; ++cut) {
            uint32_t expect = 0;
            if (width == 8) {
                st.begin(8, Crc::POLY8, Crc::INIT8);
                expect = Crc::crc8(v, 9);
            } else if (width == 16) {
                st.begin(16, Crc::POLY16, Crc::INIT16);
                expect = Crc::crc16(v, 9);
            } else {
                st.begin(32, Crc::POLY32, Crc::INIT32);
                expect = Crc::crc32(v, 9);
            }
            st.feed(v, cut);
            st.feed(v + cut, 9 - cut);
            TEST_ASSERT_EQUAL_HEX32(expect, st.value());
        }
    }

    const uint8_t bytes[8] = {0x01, 0x00, 0x3E, 0xFA, 0x28, 0x00, 0x00, 0x00};  // Two words, memory order
    st.begin(32, Crc::POLY32, Crc::INIT32);
    st.feed_word(0xFA3E0001u);                                  // Record marker + version, little-endian
    st.feed_word(40u);                                          // Sample count
    TEST_ASSERT_EQUAL_HEX32(Crc::crc32(bytes, 8), st.value()); // A word is its four little-endian bytes
}

/** Test entry point. */
int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_bfloat16_roundtrip);
    RUN_TEST(test_float24_roundtrip);
    RUN_TEST(test_wide_fields_roundtrip);
    RUN_TEST(test_position_conversions);
    RUN_TEST(test_timescale_roundtrip);
    RUN_TEST(test_byte_codecs);
    RUN_TEST(test_baud_codes);
    RUN_TEST(test_cmd_groups);
    RUN_TEST(test_register_map);
    RUN_TEST(test_frame_is_eight_bytes);
    RUN_TEST(test_register_layouts);
    RUN_TEST(test_motion_task_flags);
    RUN_TEST(test_pulsebeat_groups);
    RUN_TEST(test_burst_pvt_roundtrip);
    RUN_TEST(test_burst_pvctf_roundtrip);
    RUN_TEST(test_burst_unmask);
    RUN_TEST(test_burst_never_aliases_a_register);
    RUN_TEST(test_identifier_matching);
    RUN_TEST(test_channel_bases_are_distinct);
    RUN_TEST(test_discovery_search);
    RUN_TEST(test_discovery_assign);
    RUN_TEST(test_crc_module);
    return UNITY_END();
}
