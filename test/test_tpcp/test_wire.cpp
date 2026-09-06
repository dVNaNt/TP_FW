/**
 * @file test_wire.cpp
 * @brief Host-side conformance tests for the TunePulse CAN Protocol wire format.
 *
 * Covers the parts of the protocol that are pure data and therefore testable
 * without any hardware: the fixed-point codecs, the 8-byte frame layouts, the
 * bit-packed motion burst frame and the identifier helpers. A drive and a host
 * tool disagreeing about any of these is a silent, hard-to-debug failure on a
 * live bus, which is exactly the class of bug worth pinning down at build time.
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

/** Unity per-test setup hook (nothing to prepare). */
void setUp(void) {}

/** Unity per-test teardown hook (nothing to release). */
void tearDown(void) {}

// ============================================================================
// Fixed-point codecs
// ============================================================================

/** bfloat16 keeps full float range at roughly 0.4 % relative accuracy. */
void test_bfloat16_roundtrip(void) {
    const float samples[] = {0.0f, 1.0f, -1.0f, 24.0f, 0.0016f, 51.0f, 1e-6f, 1e6f};
    for (unsigned i = 0; i < sizeof(samples) / sizeof(samples[0]); ++i) {
        const float v = samples[i];                                  // Value under test
        const float r = tp_unpack_bf16(tp_pack_bf16(v));             // Round trip
        const float err = (v == 0.0f) ? std::fabs(r) : std::fabs((r - v) / v);
        TEST_ASSERT_TRUE_MESSAGE(err < 0.01f, "bfloat16 relative error above 1 %");
    }
}

/**
 * float24 keeps full float range at roughly 0.008 % accuracy.
 *
 * Negative values are the interesting case: the draft decoder sign-extended the
 * payload downward, which overwrote the exponent and turned every negative
 * velocity into a huge number, an infinity or a NaN.
 */
void test_float24_roundtrip(void) {
    const float samples[] = {0.0f, 1.0f, -1.0f, -12.5f, 3000.0f, -0.001f, 1e-6f};
    for (unsigned i = 0; i < sizeof(samples) / sizeof(samples[0]); ++i) {
        const float v = samples[i];                                  // Value under test
        const float r = tp_unpack_f24(tp_pack_f24(v));               // Round trip
        TEST_ASSERT_TRUE_MESSAGE(std::isfinite(r), "float24 decoded to a non-finite value");
        const float err = (v == 0.0f) ? std::fabs(r) : std::fabs((r - v) / v);
        TEST_ASSERT_TRUE_MESSAGE(err < 1e-4f, "float24 relative error above 0.01 %");
    }
}

/** The 48-bit position field must survive both extremes of its range. */
void test_position_48bit_roundtrip(void) {
    const int64_t samples[] = {0, 65536, -65536, 140737488355327LL, -140737488355328LL};
    for (unsigned i = 0; i < sizeof(samples) / sizeof(samples[0]); ++i) {
        uint8_t bytes[6];                                            // Wire representation
        tp_pack_i48(bytes, samples[i]);
        TEST_ASSERT_EQUAL_INT64(samples[i], tp_unpack_i48(bytes));
    }
}

/** The 24-bit following-error field must sign-extend correctly. */
void test_error_24bit_roundtrip(void) {
    const int32_t samples[] = {0, 1, -1, 8388607, -8388608};
    for (unsigned i = 0; i < sizeof(samples) / sizeof(samples[0]); ++i) {
        uint8_t bytes[3];                                            // Wire representation
        tp_pack_i24(bytes, samples[i]);
        TEST_ASSERT_EQUAL_INT32(samples[i], tp_unpack_i24(bytes));
    }
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
    TEST_ASSERT_EQUAL_UINT8(96, tp_pack_volts_q2(24.0f));            // 24 V at 0.25 V per LSB
    TEST_ASSERT_EQUAL_UINT8(255, tp_pack_volts_q2(99.0f));           // Saturates, never wraps
    TEST_ASSERT_EQUAL_UINT8(0, tp_pack_volts_q2(-5.0f));             // Clamps a sensor glitch
    TEST_ASSERT_EQUAL_INT8(-40, (int8_t)tp_pack_temp_c(-40.0f));     // Signed degrees Celsius
    TEST_ASSERT_EQUAL_INT8(64, (int8_t)tp_pack_q1_1_6(1.0f));        // Q1.1.6: 64 == 1.0
    TEST_ASSERT_EQUAL_INT8(127, (int8_t)tp_pack_q1_1_6(9.0f));       // Saturates at +1.98
}

// ============================================================================
// Bit-packed motion burst frame
// ============================================================================

/** Every field of a burst frame must survive the round trip at both extremes. */
void test_motion_burst_roundtrip(void) {
    struct Case {
        int32_t  disp;   // Q16.16 revolutions
        float    vel;    // Revolutions per second
        uint16_t tail;   // Duration [100 us] or normalized force
        bool     const_t;
    };
    const Case cases[] = {
        {65536, 10.0f, 1000u, false},       // +1 revolution over 100 ms
        {-65536, -10.0f, 1000u, false},     // Same, reversed
        {0x1FFFFF, 1.0f, 0xFFFFu, true},    // Largest positive displacement
        {-0x200000, -1.0f, 0u, true},       // Largest negative displacement
        {1, 0.0f, 1u, false}                // Smallest representable step
    };
    for (unsigned i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i) {
        uint8_t frame[8];                                            // Wire representation
        tp_burst_pack(frame, cases[i].const_t, cases[i].disp, cases[i].vel, cases[i].tail);
        TEST_ASSERT_TRUE(tp_burst_is(frame));
        TEST_ASSERT_EQUAL_UINT8(cases[i].const_t ? TP_BURST_CONST_T : 0u,
                                (uint8_t)(frame[0] & TP_BURST_CONST_T));
        TEST_ASSERT_EQUAL_INT32(cases[i].disp, tp_burst_disp(frame));
        TEST_ASSERT_EQUAL_UINT16(cases[i].tail, tp_burst_tail(frame));
        const float v = tp_burst_vel(frame);                         // Decoded velocity
        const float err = (cases[i].vel == 0.0f) ? std::fabs(v)
                                                 : std::fabs((v - cases[i].vel) / cases[i].vel);
        TEST_ASSERT_TRUE_MESSAGE(err < 1e-4f, "burst velocity relative error too large");
    }
}

/**
 * No opcode may ever be mistaken for a burst frame.
 *
 * The whole two-format scheme on one channel rests on this: opcodes occupy
 * 0x00-0x7F and a burst frame always sets bit 7 of byte 0.
 */
void test_burst_never_aliases_an_opcode(void) {
    for (unsigned op = 0; op <= 0x7Fu; ++op) {
        const uint8_t frame[8] = {(uint8_t)op, 0, 0, 0, 0, 0, 0, 0};
        TEST_ASSERT_FALSE_MESSAGE(tp_burst_is(frame), "an opcode was decoded as a motion burst");
    }
}

// ============================================================================
// Frame layouts
// ============================================================================

/** Every typed view must occupy exactly one classic CAN payload. */
void test_frame_is_eight_bytes(void) {
    TEST_ASSERT_EQUAL_UINT32(8u, (uint32_t)sizeof(TpFrame));
    TEST_ASSERT_EQUAL_UINT32(8u, (uint32_t)TP_FRAME_SIZE);
}

/** Register access places its fields where the specification says it does. */
void test_register_frame_layout(void) {
    TpFrame f;
    f.u64 = 0;
    f.reg.cmd = TP_OP_REG_READ;
    f.reg.info = 0x06;
    f.reg.reg = 0x0405;
    f.reg.value = 0xDEADBEEFu;
    TEST_ASSERT_EQUAL_UINT8(TP_OP_REG_READ, f.b8[0]);
    TEST_ASSERT_EQUAL_UINT8(0x06, f.b8[1]);
    TEST_ASSERT_EQUAL_UINT8(0x05, f.b8[2]);   // Register identifier, little-endian
    TEST_ASSERT_EQUAL_UINT8(0x04, f.b8[3]);
    TEST_ASSERT_EQUAL_UINT8(0xEF, f.b8[4]);   // Value, little-endian
    TEST_ASSERT_EQUAL_UINT8(0xDE, f.b8[7]);
}

/** The heartbeat packs two nibble pairs that a master decodes by hand. */
void test_heartbeat_layout(void) {
    TpFrame f;
    f.u64 = 0;
    f.heartbeat.cmd = TP_OP_HEARTBEAT;
    f.heartbeat.state = TP_STATE_RUNNING;   // 4
    f.heartbeat.mode = TP_MODE_POSITION;    // 3
    f.heartbeat.seq = 5;
    f.heartbeat.queue = 9;
    TEST_ASSERT_EQUAL_UINT8(TP_OP_HEARTBEAT, f.b8[0]);
    TEST_ASSERT_EQUAL_UINT8(0x34, f.b8[1]);  // mode in the high nibble, state in the low one
    TEST_ASSERT_EQUAL_UINT8(0x95, f.b8[7]);  // queue in the high nibble, seq in the low one
}

/** Motion Status 3 packs a 4-bit queue depth and a 12-bit time scale. */
void test_motion_status3_layout(void) {
    TpFrame f;
    f.u64 = 0;
    f.motion_status3.queue = 15;
    f.motion_status3.time_scale = 0x100;    // Q1.3.8: 256 == 1.0x
    TEST_ASSERT_EQUAL_UINT8(0x0F, f.b8[6]);
    TEST_ASSERT_EQUAL_UINT8(0x10, f.b8[7]);
}

// ============================================================================
// Identifier map
// ============================================================================

/** A node must answer its own address and the broadcast address, nothing else. */
void test_identifier_matching(void) {
    TEST_ASSERT_TRUE(tp_is_request_for(0x307, 7));
    TEST_ASSERT_TRUE(tp_is_request_for(0x3FF, 7));      // Broadcast request
    TEST_ASSERT_FALSE(tp_is_request_for(0x308, 7));     // A neighbour's address
    TEST_ASSERT_FALSE(tp_is_request_for(0x207, 7));     // The setpoint channel
    TEST_ASSERT_TRUE(tp_is_setpoint_for(0x207, 7));
    TEST_ASSERT_TRUE(tp_is_setpoint_for(0x2FF, 7));     // Broadcast setpoint
    TEST_ASSERT_FALSE(tp_is_setpoint_for(0x307, 7));    // The request channel
}

/** The network-management block is exactly sixteen identifiers wide. */
void test_nmt_range(void) {
    TEST_ASSERT_TRUE(tp_is_nmt(0x100));
    TEST_ASSERT_TRUE(tp_is_nmt(0x10F));
    TEST_ASSERT_FALSE(tp_is_nmt(0x0FF));
    TEST_ASSERT_FALSE(tp_is_nmt(0x110));
}

/** The channel bases must not overlap, or two message classes would alias. */
void test_channel_bases_are_distinct(void) {
    TEST_ASSERT_TRUE(TP_ID_EMCY_BASE < TP_ID_NMT_BASE);
    TEST_ASSERT_TRUE(TP_ID_NMT_BASE < TP_ID_SETPOINT_BASE);
    TEST_ASSERT_TRUE(TP_ID_SETPOINT_BASE < TP_ID_REQUEST_BASE);
    TEST_ASSERT_TRUE(TP_ID_REQUEST_BASE < TP_ID_RESPONSE_BASE);
    TEST_ASSERT_TRUE(TP_ID_RESPONSE_BASE < TP_ID_DISCOVERY_BASE);
    TEST_ASSERT_TRUE(TP_ID_DISCOVERY_BASE < TP_ID_TELEMETRY_BASE);
    TEST_ASSERT_TRUE(TP_ID_TELEMETRY_BASE < TP_ID_BULK_BASE);
    TEST_ASSERT_TRUE((TP_ID_BULK_BASE + 0xFFu) <= 0x7FFu);  // Everything fits 11 bits
}

/** Test entry point. */
int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_bfloat16_roundtrip);
    RUN_TEST(test_float24_roundtrip);
    RUN_TEST(test_position_48bit_roundtrip);
    RUN_TEST(test_error_24bit_roundtrip);
    RUN_TEST(test_timescale_roundtrip);
    RUN_TEST(test_byte_codecs);
    RUN_TEST(test_motion_burst_roundtrip);
    RUN_TEST(test_burst_never_aliases_an_opcode);
    RUN_TEST(test_frame_is_eight_bytes);
    RUN_TEST(test_register_frame_layout);
    RUN_TEST(test_heartbeat_layout);
    RUN_TEST(test_motion_status3_layout);
    RUN_TEST(test_identifier_matching);
    RUN_TEST(test_nmt_range);
    RUN_TEST(test_channel_bases_are_distinct);
    return UNITY_END();
}
