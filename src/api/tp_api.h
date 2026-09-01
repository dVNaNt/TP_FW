#pragma once

/**
 * @file tp_api.h
 * @brief Compact 8-byte "chunk" command/status API for the TunePulse driver.
 *
 * The API exchanges fixed-size 8-byte chunks (exactly one classic CAN frame,
 * well within any CAN-FD payload), so the very same pack/unpack code works
 * over CAN 2.0A, CAN-FD, USB CDC or any other byte-oriented transport.
 *
 * Design goals:
 * - Zero-copy serialization: every chunk is a union of a raw 64-bit word,
 *   a byte array and a packed bitfield struct view. Packing a response is
 *   a handful of stores, unpacking a request is a handful of loads.
 * - Transport independence: the API never touches hardware itself. It holds
 *   two caller-supplied function pointers - a READ callback (fetches one
 *   pending chunk from the interface) and a WRITE callback (pushes one chunk
 *   to the interface). Different transports can be mixed freely.
 * - lib reuse: bfloat16 conversion is reused from lib/motor_control/motor.h,
 *   live telemetry is pulled directly from TunePulseHeart subsystems.
 *   Everything not yet implemented in the driver is exposed as a stub field
 *   that the application layer may fill manually.
 *
 * Chunk layout conventions:
 * - Byte 0 of every chunk is always the command ID.
 * - All multi-byte integers are little-endian (native for Cortex-M4).
 * - Bitfields inside a byte/halfword allocate from the LSB upward.
 * - Layouts are fixed by static_asserts and documented in src/api/API.md.
 *
 * NOTE on the command map: the draft "Motor properties" chunk was assigned
 * 0x06, which collides with "Motion Status 1". Here Motor properties uses
 * 0x09 (see API_CMD_MOTOR_PROPERTIES); the Heartbeat CMD is a 4-bit value
 * (0x7) living in the low nibble of byte 0 and is only valid as a
 * driver-initiated broadcast, never as a master request.
 *
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#include "../lib/common.h"  // Standard integer types and shared helpers
#include "../lib/lib.h"     // TunePulseHeart: live data source (lib layer)

// ============================================================================
// Command identifiers
// ============================================================================

/** 8-bit command IDs carried in byte 0 of every request/response chunk. */
enum ApiCmd : uint8_t {
    API_CMD_DEVICE_INFO      = 0x01,  ///< Device Info (R): HW/FW version, device code.
    API_CMD_DEVICE_UID       = 0x02,  ///< Device UID (R): 56-bit unique identifier.
    API_CMD_DEVICE_STATUS    = 0x03,  ///< Device Status (R): uptime + fault bits.
    API_CMD_MOTOR_STATUS     = 0x04,  ///< Motor Status (RW): type/mode/phase/limits.
    API_CMD_ELECTRIC_STATUS  = 0x05,  ///< Electric Status (R): power/supply/current.
    API_CMD_MOTION_STATUS_1  = 0x06,  ///< Motion Status 1 (R): mode + position.
    API_CMD_MOTION_STATUS_2  = 0x07,  ///< Motion Status 2 (R): velocity/accel/torque.
    API_CMD_MOTION_STATUS_3  = 0x08,  ///< Motion Status 3 (R): errors/queue/time scale.
    API_CMD_MOTOR_PROPERTIES = 0x09,  ///< Motor Properties (RW). Was 0x06 in the
                                      ///< draft (collision) - remapped to 0x09.
    API_CMD_HEARTBEAT        = 0x07   ///< 4-bit CMD (low nibble), heartbeat only.
};

// ============================================================================
// Chunk view structures (all exactly 8 bytes)
// ============================================================================

/** @brief 0x01 Device Info (R). HW version, FW version, 32-bit device code. */
struct ChunkDeviceInfo {
    uint8_t  cmd;      ///< Byte 0: command ID, always API_CMD_DEVICE_INFO.
    uint8_t  hw_ver;   ///< Byte 1: hardware version tag.
    uint16_t fw_ver;   ///< Bytes 2-3: firmware version (LE).
    uint32_t device;   ///< Bytes 4-7: device code (LE).
};
static_assert(sizeof(ChunkDeviceInfo) == 8, "ChunkDeviceInfo must fill one 8-byte chunk");

/** @brief 0x02 Device UID (R): 56-bit unique identifier as 7 raw bytes. */
struct ChunkDeviceUID {
    uint8_t cmd;     ///< Byte 0: command ID, always API_CMD_DEVICE_UID.
    uint8_t uid[7];  ///< Bytes 1-7: 56-bit device UID, LSB-first.
};
static_assert(sizeof(ChunkDeviceUID) == 8, "ChunkDeviceUID must fill one 8-byte chunk");

/** @brief 0x03 Device Status (R): 24-bit uptime seconds + 32-bit fault word. */
struct ChunkDeviceStatus {
    uint8_t  cmd;        ///< Byte 0: command ID, always API_CMD_DEVICE_STATUS.
    uint8_t  uptime[3];  ///< Bytes 1-3: uptime in seconds, uint24 LE (194 days).
    uint32_t fault;      ///< Bytes 4-7: error bit field (LE).
};
static_assert(sizeof(ChunkDeviceStatus) == 8, "ChunkDeviceStatus must fill one 8-byte chunk");

/**
 * @brief 0x04 Motor Status (RW): packed mode/type/phase + poles/temp/direction.
 *
 * Byte 1 bit layout (LSB first): mode[3:0], motor_type[5:4], phase_conn[7:6].
 * Bytes 4-7: bit0 = direction (0: CW, 1: CCW), bits 1-31 reserved.
 */
struct ChunkMotorStatus {
    uint8_t cmd;  ///< Byte 0: command ID, always API_CMD_MOTOR_STATUS.
    union {
        struct {
            uint8_t mode : 4;        ///< Bits 0-3: motor control mode (TBD).
            uint8_t motor_type : 2;  ///< Bits 4-5: 0 none, 1 STEPPER, 2 BLDC, 3 ERROR.
            uint8_t phase_conn : 2;  ///< Bits 6-7: 0 NC, 1 ABC, 2 CAB, 3 ACB.
        };
        uint8_t flags1;              ///< Whole-byte view of byte 1.
    };
    uint8_t pole_pairs;  ///< Byte 2: number of pole pairs (0-255).
    uint8_t temp;        ///< Byte 3: temperature, degC; 255 = no info.
    union {
        struct {
            uint32_t direction : 1;   ///< Bit 32: 0 = CW, 1 = CCW.
            uint32_t reserved4 : 31;  ///< Bits 33-63: reserved, write as zero.
        };
        uint32_t flags4;              ///< Whole-word view of bytes 4-7.
    };
};
static_assert(sizeof(ChunkMotorStatus) == 8, "ChunkMotorStatus must fill one 8-byte chunk");

/** @brief 0x05 Electric Status (R): efficiency + bf16 power/supply/current. */
struct ChunkElectricStatus {
    uint8_t  cmd;         ///< Byte 0: command ID, always API_CMD_ELECTRIC_STATUS.
    uint8_t  efficiency;  ///< Byte 1: 0..255 mapped to 0..100 %.
    uint16_t power;       ///< Bytes 2-3: instant motor power [W], bfloat16.
    uint16_t supply;      ///< Bytes 4-5: driver supply voltage [V], bfloat16.
    uint16_t current;     ///< Bytes 6-7: instant peak current [A], bfloat16.
};
static_assert(sizeof(ChunkElectricStatus) == 8, "ChunkElectricStatus must fill one 8-byte chunk");

/** @brief 0x06 Motion Status 1 (R): execution mode + 48-bit absolute position. */
struct ChunkMotionStatus1 {
    uint8_t cmd;          ///< Byte 0: command ID, always API_CMD_MOTION_STATUS_1.
    uint8_t motion_mode;  ///< Byte 1: type of the executing command (TBD enum).
    uint8_t position[6];  ///< Bytes 2-7: real absolute position, Q1.31.16 LE
                          ///< (48-bit signed, 16 fractional bits = rot.angle).
};
static_assert(sizeof(ChunkMotionStatus1) == 8, "ChunkMotionStatus1 must fill one 8-byte chunk");

/** @brief 0x07 Motion Status 2 (R): velocity, acceleration, torque. */
struct ChunkMotionStatus2 {
    uint8_t  cmd;          ///< Byte 0: command ID, always API_CMD_MOTION_STATUS_2.
    uint8_t  velocity[3];  ///< Bytes 1-3: instant velocity [RPS], float24 (1.8.15).
    uint16_t accel;        ///< Bytes 4-5: acceleration [RPS/s], bfloat16.
    uint16_t torque;       ///< Bytes 6-7: raw torque value.
};
static_assert(sizeof(ChunkMotionStatus2) == 8, "ChunkMotionStatus2 must fill one 8-byte chunk");

/**
 * @brief 0x08 Motion Status 3 (R): following errors, queue depth, time scale.
 *
 * Bytes 6-7 bit layout (LSB first): queue[3:0], time_scale[15:4].
 * TimeScale is Q1.3.8 (12-bit signed, 256 = 1.0x flow of time).
 */
struct ChunkMotionStatus3 {
    uint8_t  cmd;         ///< Byte 0: command ID, always API_CMD_MOTION_STATUS_3.
    uint8_t  pos_err[3];  ///< Bytes 1-3: position error [rot.angle], Q1.7.16 LE.
    uint16_t vel_err;     ///< Bytes 4-5: velocity error [RPS], bfloat16.
    union {
        struct {
            uint16_t queue : 4;        ///< Bits 48-51: remaining motion commands.
            uint16_t time_scale : 12;  ///< Bits 52-63: Q1.3.8 time multiplier.
        };
        uint16_t queue_ts;             ///< Whole-word view of bytes 6-7.
    };
};
static_assert(sizeof(ChunkMotionStatus3) == 8, "ChunkMotionStatus3 must fill one 8-byte chunk");

/** @brief 0x09 Motor Properties (RW): thermal limit + R/L/torque constants. */
struct ChunkMotorProperties {
    uint8_t  cmd;         ///< Byte 0: command ID, always API_CMD_MOTOR_PROPERTIES.
    uint8_t  temp_lim;    ///< Byte 1: maximum temperature, degC.
    uint16_t resistance;  ///< Bytes 2-3: winding resistance [Ohm], bfloat16.
    uint16_t inductance;  ///< Bytes 4-5: winding inductance [H], bfloat16.
    uint16_t torque;      ///< Bytes 6-7: motor torque constant [N*m], bfloat16.
};
static_assert(sizeof(ChunkMotorProperties) == 8, "ChunkMotorProperties must fill one 8-byte chunk");

/**
 * @brief Heartbeat chunk (driver-initiated feedback, 4-bit CMD = 0x7).
 *
 * Full 64-bit layout, LSB first:
 * cmd:4 | beats:4 | cmdq:4 | time_scale:12 | status:4 | fault:4 | warn:4 |
 * mode:6 | error:6 | load:8 | io:8
 *
 * - time_scale : Q1.3.8 (256 = 1.0x), motion governor requested multiplier.
 * - error      : Q1.1.4 simplified following error (-2.0 .. +1.9375).
 * - load       : int8 motor load; -128 marks motor overload.
 */
struct ChunkHeartbeat {
    uint64_t cmd : 4;          ///< Bits 0-3: 4-bit command ID, always 0x7.
    uint64_t beats : 4;        ///< Bits 4-7: incrementing heartbeat counter.
    uint64_t cmdq : 4;         ///< Bits 8-11: remaining motion commands (dir dep.).
    uint64_t time_scale : 12;  ///< Bits 12-23: Q1.3.8 time multiplier (signed).
    uint64_t status : 4;       ///< Bits 24-27: driver status group (TBD).
    uint64_t fault : 4;        ///< Bits 28-31: fault group (TBD).
    uint64_t warn : 4;         ///< Bits 32-35: warning group (TBD).
    uint64_t mode : 6;         ///< Bits 36-41: current execution mode (TBD).
    uint64_t error : 6;        ///< Bits 42-47: Q1.1.4 following error (signed).
    uint64_t load : 8;         ///< Bits 48-55: int8 motor load, -128 = overload.
    uint64_t io : 8;           ///< Bits 56-63: endstop/probe/homing IO state.
};
static_assert(sizeof(ChunkHeartbeat) == 8, "ChunkHeartbeat must fill one 8-byte chunk");

/**
 * @brief Universal 8-byte chunk container.
 *
 * A single union gives byte-wise transport access and zero-copy typed views
 * over the very same 64 bits. No memcpy, no serialization loops: filling a
 * typed view IS the packing, reading a typed view IS the unpacking.
 */
union ApiChunk {
    uint8_t              b8[8];            ///< Raw byte access (transport interface).
    uint64_t             u64;              ///< Whole-chunk word (fast compare/clear).
    ChunkDeviceInfo      device_info;      ///< View: 0x01 Device Info.
    ChunkDeviceUID       device_uid;       ///< View: 0x02 Device UID.
    ChunkDeviceStatus    device_status;    ///< View: 0x03 Device Status.
    ChunkMotorStatus     motor_status;     ///< View: 0x04 Motor Status.
    ChunkElectricStatus  electric_status;  ///< View: 0x05 Electric Status.
    ChunkMotionStatus1   motion_status1;   ///< View: 0x06 Motion Status 1.
    ChunkMotionStatus2   motion_status2;   ///< View: 0x07 Motion Status 2.
    ChunkMotionStatus3   motion_status3;   ///< View: 0x08 Motion Status 3.
    ChunkMotorProperties motor_props;      ///< View: 0x09 Motor Properties.
    ChunkHeartbeat       heartbeat;        ///< View: heartbeat (4-bit CMD).
};
static_assert(sizeof(ApiChunk) == 8, "ApiChunk must be exactly 8 bytes (CAN 2.0 frame)");

// ============================================================================
// Fixed-size codec helpers (pack/unpack, branch-minimal)
// ============================================================================

/**
 * @brief Pack a float into bfloat16 (1.8.7).
 *
 * Delegates to the lib implementation (motor.h) so the API and the motor
 * record storage always agree on the wire encoding of parameters.
 */
inline uint16_t api_pack_bf16(float value) {
    return float_to_bfloat16(value);  // Truncating f32 -> bf16 conversion from lib
}

/**
 * @brief Unpack a bfloat16 word into a float.
 *
 * Delegates to the lib implementation (motor.h); a single 16-bit shift.
 */
inline float api_unpack_bf16(uint16_t raw) {
    return bfloat16_to_float(raw);  // bf16 -> f32 widening from lib
}

/**
 * @brief Pack a float into float24 (1.8.15).
 *
 * float32 (1.8.23) and float24 (1.8.15) share the sign and exponent fields,
 * so the conversion is a pure truncation of the 8 lowest mantissa bits -
 * two shifts, no branching and no FPU work. Relative error <= 0.008 %.
 *
 * @param value IEEE-754 single-precision input.
 * @return 24-bit float24 payload (right-aligned in a 32-bit container).
 */
inline uint32_t api_pack_f24(float value) {
    uint32_t bits;
    __builtin_memcpy(&bits, &value, 4);  // Bit-cast float -> uint32
    return (bits >> 8) & 0x00FFFFFFu;    // Drop 8 lowest mantissa bits
}

/**
 * @brief Unpack a float24 (1.8.15) word into a float.
 *
 * Shifts the payload back into place and replicates the sign bit into the
 * upper byte when set (manual sign extension of the exponent field).
 *
 * @param raw 24-bit float24 payload (right-aligned).
 * @return Reconstructed single-precision value.
 */
inline float api_unpack_f24(uint32_t raw) {
    uint32_t bits = ((raw & 0x00800000u) != 0u)         // Sign bit set?
                        ? ((raw << 8) | 0xFF000000u)    // Extend sign into bits 31-24
                        : (raw << 8);                   // Positive: zero-extend
    float value;
    __builtin_memcpy(&value, &bits, 4);  // Bit-cast uint32 -> float
    return value;
}

/**
 * @brief Pack a Q16.16 signed value into a little-endian 48-bit field.
 *
 * The API position format Q1.31.16 is exactly an int64 Q16.16 truncated to
 * its 48 lowest bits (range +/-32768 revolutions, 1/65536 resolution).
 *
 * @param dst     Destination byte array (at least 6 bytes).
 * @param q16_16  Signed value with 16 fractional bits.
 */
inline void api_pack_i48(uint8_t* dst, int64_t q16_16) {
    dst[0] = (uint8_t)((uint64_t)q16_16);        // Bits 0-7
    dst[1] = (uint8_t)((uint64_t)q16_16 >> 8);   // Bits 8-15
    dst[2] = (uint8_t)((uint64_t)q16_16 >> 16);  // Bits 16-23
    dst[3] = (uint8_t)((uint64_t)q16_16 >> 24);  // Bits 24-31
    dst[4] = (uint8_t)((uint64_t)q16_16 >> 32);  // Bits 32-39
    dst[5] = (uint8_t)((uint64_t)q16_16 >> 40);  // Bits 40-47
}

/**
 * @brief Unpack a little-endian 48-bit signed field into int64 (Q16.16).
 *
 * Sign extension is done with two shifts (no branch): shift left by 16 to
 * put bit 47 at position 63, then arithmetic right shift back.
 *
 * @param src Source byte array (at least 6 bytes).
 * @return Signed value with 16 fractional bits.
 */
inline int64_t api_unpack_i48(const uint8_t* src) {
    uint64_t bits = (uint64_t)src[0]           // Bits 0-7
                  | ((uint64_t)src[1] << 8)    // Bits 8-15
                  | ((uint64_t)src[2] << 16)   // Bits 16-23
                  | ((uint64_t)src[3] << 24)   // Bits 24-31
                  | ((uint64_t)src[4] << 32)   // Bits 32-39
                  | ((uint64_t)src[5] << 40);  // Bits 40-47
    return (int64_t)(bits << 16) >> 16;        // Sign-extend from bit 47
}

/**
 * @brief Pack a Q16.16 signed value into a little-endian 24-bit field.
 *
 * Used for the Q1.7.16 position error: range +/-128 rotations with 1/65536
 * angular resolution.
 *
 * @param dst     Destination byte array (at least 3 bytes).
 * @param q16_16  Signed value with 16 fractional bits.
 */
inline void api_pack_i24(uint8_t* dst, int64_t q16_16) {
    int32_t v = (int32_t)q16_16;          // Truncate to 32 bits first
    dst[0] = (uint8_t)((uint32_t)v);      // Bits 0-7
    dst[1] = (uint8_t)((uint32_t)v >> 8); // Bits 8-15
    dst[2] = (uint8_t)((uint32_t)v >> 16);  // Bits 16-23
}

/**
 * @brief Unpack a little-endian 24-bit signed field into int64 (Q16.16).
 *
 * @param src Source byte array (at least 3 bytes).
 * @return Signed value with 16 fractional bits.
 */
inline int64_t api_unpack_i24(const uint8_t* src) {
    uint32_t bits = (uint32_t)src[0]         // Bits 0-7
                  | ((uint32_t)src[1] << 8)  // Bits 8-15
                  | ((uint32_t)src[2] << 16);  // Bits 16-23
    return (int32_t)(bits << 8) >> 8;        // Sign-extend from bit 23
}

/**
 * @brief Convert a governor time scale from Q10 (1024 = 1.0x) to Q1.3.8.
 *
 * Pure integer path: Q10 -> Q8 is a right shift by 2, then a symmetric
 * saturation into the 12-bit signed field (range -8.0x .. +7.996x).
 *
 * @param scale_q10 Signed time scale with 10 fractional bits.
 * @return 12-bit Q1.3.8 payload (right-aligned, two's complement).
 */
inline uint16_t api_pack_timescale(int32_t scale_q10) {
    int32_t q8 = scale_q10 >> 2;                // 1024 -> 256 per unit (Q10 -> Q8)
    if (q8 > 2047)  q8 = 2047;                  // Saturate high (+7.996x)
    if (q8 < -2048) q8 = -2048;                 // Saturate low  (-8.0x)
    return (uint16_t)((uint32_t)q8 & 0x0FFFu);  // Mask into 12 bits
}

/**
 * @brief Unpack a Q1.3.8 time scale into a float multiplier (256 = 1.0x).
 *
 * @param raw 12-bit signed payload (right-aligned).
 * @return Time scale as a float (e.g. 0.5 for half-speed playback).
 */
inline float api_unpack_timescale(uint16_t raw) {
    int16_t q8 = (int16_t)((uint16_t)(raw << 4) >> 4);  // Sign-extend 12 bits
    return (float)q8 * (1.0f / 256.0f);                // Q8 -> float
}

/**
 * @brief Pack a following-error ratio into the Q1.1.4 heartbeat field.
 *
 * Q1.1.4 is a 6-bit signed value: 1 sign bit + 1 integer bit + 4 fractional
 * bits, covering -2.0 .. +1.9375. The input is scaled by 16, rounded away
 * from zero and saturated - three compares total on the hot path.
 *
 * @param err_ratio Following error as a normalized ratio (e.g. err/limit).
 * @return 6-bit payload (right-aligned, two's complement).
 */
inline uint8_t api_pack_follow_error(float err_ratio) {
    int32_t q = (int32_t)(err_ratio * 16.0f + ((err_ratio >= 0.0f) ? 0.5f : -0.5f));  // Scale + round
    if (q > 31)  q = 31;                     // Saturate high (+1.9375)
    if (q < -32) q = -32;                    // Saturate low  (-2.0)
    return (uint8_t)((uint32_t)q & 0x3Fu);   // Mask to 6 bits
}

// ============================================================================
// Hardware transport callbacks
// ============================================================================

/**
 * @brief Hardware WRITE callback: transmits one 8-byte chunk.
 *
 * Implemented by the board/communication layer (CAN-FD, CAN 2.0A, USB CDC,
 * ...). Must not block; if the interface queue is full the chunk is dropped.
 *
 * @param chunk Fully packed chunk to transmit.
 */
typedef void (*ApiWriteCb)(const ApiChunk& chunk);

/**
 * @brief Hardware READ callback: fetches one pending chunk, non-blocking.
 *
 * Implemented by the board/communication layer. Should return false when no
 * complete chunk is available; the API then simply stops polling.
 *
 * @param[out] chunk Storage for the received chunk.
 * @return true if a chunk was fetched, false if the interface is empty.
 */
typedef bool (*ApiReadCb)(ApiChunk& chunk);

// ============================================================================
// TunePulseAPI - chunk dispatcher bound to TunePulseHeart
// ============================================================================

/**
 * @class TunePulseAPI
 * @brief Request/response engine for the 8-byte chunk protocol.
 *
 * Responsibilities:
 * - Pull pending request chunks through the hardware READ callback and
 *   answer each of them through the hardware WRITE callback (poll()).
 * - Pack responses from live lib state (Motor profile, temperature, power,
 *   supply voltage, driver status) where such data exists in the firmware.
 * - Apply RW chunks (Motor Status, Motor Properties) back into the lib
 *   objects using the existing setters.
 * - Build the driver-initiated Heartbeat chunk (send_heartbeat()).
 *
 * Fields of chunks that have no backing data in the current firmware
 * (uptime, UID, efficiency, motion feedback) are exposed as public members
 * so the application layer can feed them until the corresponding subsystems
 * are wired (all such places are marked STUB in the code and API.md).
 */
class TunePulseAPI {
public:
    // ------------------------------------------------------------------
    // Public configuration / stub data sources (fill from the app layer)
    // ------------------------------------------------------------------

    // Device identity (no provisioning source exists in FW yet - STUB).
    uint8_t  hw_ver = 0;       ///< Hardware version reported by 0x01.
    uint16_t fw_ver = 1;       ///< Firmware version reported by 0x01.
    uint32_t device_code = 0;  ///< 32-bit device code reported by 0x01.
    uint8_t  uid[7] = {0};     ///< 56-bit UID reported by 0x02 (STUB).
    uint32_t uptime_s = 0;     ///< Uptime seconds for 0x03; increment in the app loop (STUB).

    /// Application-filled motion feedback for 0x06/0x07/0x08 (STUB: the
    /// motion executor is not wired into the control path yet).
    struct MotionFeedback {
        uint8_t  motion_mode = 0;       ///< Type of the executing command.
        int64_t  position_q16 = 0;      ///< Real absolute position, Q16.16 rot.
        int64_t  pos_err_q16 = 0;       ///< Position error, Q16.16 rot.angle.
        float    velocity_rps = 0.0f;   ///< Instant velocity [RPS].
        float    accel_rps2 = 0.0f;     ///< Instant acceleration [RPS/s].
        uint16_t torque_raw = 0;        ///< Raw torque value.
        float    vel_err_rps = 0.0f;    ///< Velocity error [RPS].
        uint8_t  queue = 0;             ///< Remaining motion commands (0-15).
        int32_t  timescale_q10 = 1024;  ///< Governor time scale, 1024 = 1.0x.
    } motion;

    // ------------------------------------------------------------------
    // Wiring
    // ------------------------------------------------------------------

    /**
     * @brief Install the two hardware transport callbacks.
     * @param read_cb  Non-blocking chunk fetch callback (may be nullptr).
     * @param write_cb Chunk transmit callback (may be nullptr).
     */
    void attach(ApiReadCb read_cb, ApiWriteCb write_cb) {
        on_read_ = read_cb;    // Store RX callback (polling source)
        on_write_ = write_cb;  // Store TX callback (response sink)
    }

    /**
     * @brief Bind the live-data source (the global TunePulseHeart object).
     * @param tp Pointer to the application heart; may be nullptr to run
     *           purely on stub data (useful for host-side unit tests).
     */
    void bind(TunePulseHeart* tp) {
        tp_ = tp;  // Remember the telemetry source
    }

    // ------------------------------------------------------------------
    // Main service entry points
    // ------------------------------------------------------------------

    /**
     * @brief Poll the READ callback and service every pending request.
     *
     * Call this from a non-realtime context (main loop / comms task).
     * Each fetched chunk is dispatched by handle(); responses go straight
     * into the WRITE callback.
     */
    void poll() {
        if (on_read_ == nullptr) return;  // No RX wired: nothing to do
        ApiChunk rx;                      // Scratch storage for RX
        while (on_read_(rx)) {            // Drain the interface queue
            handle(rx);                   // Dispatch this request
        }
    }

    /**
     * @brief Dispatch one request chunk and transmit the response.
     *
     * The switch runs on byte 0 only; every branch fills a typed view of a
     * stack-local chunk, so the whole handler compiles to a handful of
     * stores plus one indirect call to the WRITE callback.
     *
     * @param req Received chunk (byte 0 selects the handler).
     */
    void handle(const ApiChunk& req) {
        ApiChunk tx;                   // Response chunk (zero-copy)
        tx.u64 = 0;                    // Start from a clean 64-bit word

        switch (req.b8[0]) {           // Dispatch on the command byte
            case API_CMD_DEVICE_INFO:
                pack_device_info(tx);      // 0x01: static identity
                break;
            case API_CMD_DEVICE_UID:
                pack_device_uid(tx);       // 0x02: 56-bit UID
                break;
            case API_CMD_DEVICE_STATUS:
                pack_device_status(tx);    // 0x03: uptime + fault word
                break;
            case API_CMD_MOTOR_STATUS:
                apply_motor_status(req);   // RW: apply incoming setup first
                pack_motor_status(tx);     //     then answer with real state
                break;
            case API_CMD_ELECTRIC_STATUS:
                pack_electric_status(tx);  // 0x05: power/supply/current
                break;
            case API_CMD_MOTION_STATUS_1:
                pack_motion_status1(tx);   // 0x06: mode + position
                break;
            case API_CMD_MOTION_STATUS_2:
                pack_motion_status2(tx);   // 0x07: velocity/accel/torque
                break;
            case API_CMD_MOTION_STATUS_3:
                pack_motion_status3(tx);   // 0x08: errors/queue/timescale
                break;
            case API_CMD_MOTOR_PROPERTIES:
                apply_motor_properties(req);  // RW: apply incoming R/L/etc.
                pack_motor_properties(tx);    //     then answer with real state
                break;
            default:
                return;                    // Unknown CMD: stay silent
        }

        if (on_write_ != nullptr) {        // Only transmit when wired
            on_write_(tx);                 // Push the response chunk out
        }
    }

    /**
     * @brief Build and transmit one Heartbeat chunk (driver-initiated).
     *
     * The heartbeat is not request/response: the firmware emits it on its
     * own schedule. Only the incrementing beat counter and the governor
     * time scale are derived here; status/fault/warn groups remain TBD
     * placeholders until the error taxonomy is finalized (STUB).
     *
     * @param beats         Incrementing 4-bit heartbeat counter.
     * @param cmdq          Remaining motion commands (direction dependent).
     * @param timescale_q10 Governor time scale, Q10 (1024 = 1.0x).
     * @param status        Driver status group (TBD).
     * @param fault         Fault group (TBD).
     * @param warn          Warning group (TBD).
     * @param mode          Current execution mode (TBD).
     * @param err_ratio     Following error ratio for the Q1.1.4 field.
     * @param load          int8 motor load, -128 = overload.
     * @param io            Endstop/probe/homing IO state byte.
     * @return true when the chunk was handed to the WRITE callback.
     */
    bool send_heartbeat(uint8_t beats, uint8_t cmdq, int32_t timescale_q10,
                        uint8_t status = 0, uint8_t fault = 0, uint8_t warn = 0,
                        uint8_t mode = 0, float err_ratio = 0.0f,
                        int8_t load = 0, uint8_t io = 0) {
        if (on_write_ == nullptr) return false;  // No TX wired: drop silently

        ApiChunk tx;                             // Scratch chunk on the stack
        tx.u64 = 0;                              // Start from a clean word
        tx.heartbeat.cmd = API_CMD_HEARTBEAT;    // 4-bit command tag (0x7)
        tx.heartbeat.beats = beats & 0x0Fu;      // Rolling beat counter
        tx.heartbeat.cmdq = cmdq & 0x0Fu;        // Motion queue depth
        tx.heartbeat.time_scale = api_pack_timescale(timescale_q10);  // Q1.3.8
        tx.heartbeat.status = status & 0x0Fu;    // Status group (TBD)
        tx.heartbeat.fault = fault & 0x0Fu;      // Fault group (TBD)
        tx.heartbeat.warn = warn & 0x0Fu;        // Warning group (TBD)
        tx.heartbeat.mode = mode & 0x3Fu;        // Execution mode (TBD)
        tx.heartbeat.error = api_pack_follow_error(err_ratio);  // Q1.1.4
        tx.heartbeat.load = (uint8_t)load;       // Motor load byte
        tx.heartbeat.io = io;                    // IO state byte

        on_write_(tx);                           // Fire the hardware callback
        return true;
    }

private:
    // ------------------------------------------------------------------
    // Chunk packers (0x01 - 0x09) - fill typed views from lib / stub state
    // ------------------------------------------------------------------

    /**
     * @brief Convert a Celsius temperature into the API uint8 representation.
     *
     * Negative, NaN and above-range values collapse to 255 ("no info");
     * valid values are rounded and clamped into 0..254.
     *
     * @param celsius Filtered temperature in degrees Celsius.
     * @return Encoded byte (255 = no information).
     */
    static uint8_t temp_to_u8(float celsius) {
        if (isnan(celsius) || celsius < 0.0f || celsius >= 254.5f) {
            return 255;                          // Out-of-range: report "no info"
        }
        return (uint8_t)(celsius + 0.5f);        // Round to nearest degree
    }

    /**
     * @brief Pack 0x01 Device Info from the static identity fields.
     * @param tx Response chunk to fill.
     */
    void pack_device_info(ApiChunk& tx) {
        tx.device_info.cmd = API_CMD_DEVICE_INFO;  // Command tag
        tx.device_info.hw_ver = hw_ver;            // Hardware version byte
        tx.device_info.fw_ver = fw_ver;            // Firmware version halfword
        tx.device_info.device = device_code;       // 32-bit device code
    }

    /**
     * @brief Pack 0x02 Device UID (STUB: UID provisioning not in FW yet).
     * @param tx Response chunk to fill.
     */
    void pack_device_uid(ApiChunk& tx) {
        tx.device_uid.cmd = API_CMD_DEVICE_UID;    // Command tag
        for (int i = 0; i < 7; ++i) {              // Copy the 56-bit UID
            tx.device_uid.uid[i] = uid[i];         // LSB-first byte order
        }
    }

    /**
     * @brief Pack 0x03 Device Status (uptime STUB; fault from lib status).
     *
     * The 32-bit fault word currently carries the lib MotorStatus lifecycle
     * code in its low byte; the detailed error-bit taxonomy is not defined
     * in the firmware yet (STUB).
     *
     * @param tx Response chunk to fill.
     */
    void pack_device_status(ApiChunk& tx) {
        uint32_t fault = 0;                                    // Default: no faults
        if (tp_ != nullptr) {                                  // Live source available?
            fault = (uint32_t)(uint8_t)tp_->motor_status;      // Low byte: MotorStatus
        }
        tx.device_status.cmd = API_CMD_DEVICE_STATUS;          // Command tag
        tx.device_status.uptime[0] = (uint8_t)(uptime_s);      // Bits 0-7
        tx.device_status.uptime[1] = (uint8_t)(uptime_s >> 8); // Bits 8-15
        tx.device_status.uptime[2] = (uint8_t)(uptime_s >> 16);  // Bits 16-23
        tx.device_status.fault = fault;                        // Error bits
    }

    /**
     * @brief Pack 0x04 Motor Status from the live Motor profile.
     *
     * The 4-bit "mode" field carries the OperationStatus lifecycle value
     * (Ready/Calibration/Disabled/...); the dedicated control-mode enum is
     * still TBD (STUB).
     *
     * @param tx Response chunk to fill.
     */
    void pack_motor_status(ApiChunk& tx) {
        tx.motor_status.cmd = API_CMD_MOTOR_STATUS;  // Command tag
        tx.motor_status.flags1 = 0;                  // Clear the packed byte
        tx.motor_status.flags4 = 0;                  // Clear direction + reserved

        if (tp_ != nullptr) {                        // Live source available?
            const Motor& m = tp_->current_driver.motor;  // Motor profile alias

            // Map the 4-bit mode field from the driver operation status.
            tx.motor_status.mode =
                (uint8_t)tp_->current_driver.get_driver_status() & 0x0Fu;

            // Map internal MotorType (STEP=4, BLDC=6) onto protocol codes.
            switch (m.get_motor_type()) {
                case MotorType_STEP: tx.motor_status.motor_type = 1; break;  // STEPPER
                case MotorType_BLDC: tx.motor_status.motor_type = 2; break;  // BLDC
                default:             tx.motor_status.motor_type = 0; break;  // Not defined
            }

            // Map PhasePattern onto protocol codes (CAB == firmware CBA).
            switch (m.get_phase_pattern()) {
                case PhasePattern_ABC: tx.motor_status.phase_conn = 1; break;  // ABC
                case PhasePattern_CBA: tx.motor_status.phase_conn = 2; break;  // CAB
                case PhasePattern_ACB: tx.motor_status.phase_conn = 3; break;  // ACB
                default:               tx.motor_status.phase_conn = 0; break;  // Not connected
            }

            // Pole pairs (clamped into uint8 range).
            int32_t poles = m.get_pole_pairs();
            tx.motor_status.pole_pairs =
                (uint8_t)((poles > 255) ? 255 : ((poles < 0) ? 0 : poles));

            // Temperature: live degC from the PHY, 255 = no info.
            tx.motor_status.temp = temp_to_u8(tp_->driver_phy.temperature.get_temperature_c());

            // Rotation direction: lib stores +1/-1; protocol wants a bit.
            tx.motor_status.direction = (m.get_direction() < 0) ? 1u : 0u;
        } else {
            tx.motor_status.temp = 255;              // No source: "no info"
        }
    }

    /**
     * @brief Apply an incoming 0x04 chunk to the lib Motor profile.
     *
     * Write convention: any field encoded as non-zero in the request is
     * applied; zero-valued fields keep their current state (a pure read
     * request therefore has all payload bytes zero).
     *
     * @param req Received chunk (CMD 0x04).
     */
    void apply_motor_status(const ApiChunk& req) {
        if (tp_ == nullptr) return;                  // No live source bound
        Motor& m = tp_->current_driver.motor;        // Motor profile alias

        if (req.motor_status.motor_type != 0) {      // Type requested
            // Protocol code -> internal MotorType enum.
            switch (req.motor_status.motor_type) {
                case 1:  m.set_motor_type(MotorType_STEP);    break;  // STEPPER
                case 2:  m.set_motor_type(MotorType_BLDC);    break;  // BLDC
                default: m.set_motor_type(MotorType_Invalid); break;  // Reset
            }
        }
        if (req.motor_status.phase_conn != 0) {      // Phase order requested
            // Protocol code -> internal PhasePattern enum.
            switch (req.motor_status.phase_conn) {
                case 1:  m.set_phase_pattern(PhasePattern_ABC); break;  // ABC
                case 2:  m.set_phase_pattern(PhasePattern_CBA); break;  // CAB
                case 3:  m.set_phase_pattern(PhasePattern_ACB); break;  // ACB
                default: break;                      // 0 never reaches here
            }
        }
        if (req.motor_status.pole_pairs != 0) {      // Pole count requested
            m.set_pole_pairs((int32_t)req.motor_status.pole_pairs);
        }
        if (req.motor_status.flags4 != 0) {          // Direction bit requested
            m.set_direction((req.motor_status.direction != 0) ? (int16_t)-1   // CCW
                                                              : (int16_t)+1); // CW
        }
        // mode[3:0]: control mode taxonomy is TBD - ignored for now (STUB).
    }

    /**
     * @brief Pack 0x05 Electric Status from the live PHY telemetry.
     *
     * Efficiency has no implementation in the firmware yet - it stays 0
     * until a mechanical-power estimate exists (STUB).
     *
     * @param tx Response chunk to fill.
     */
    void pack_electric_status(ApiChunk& tx) {
        tx.electric_status.cmd = API_CMD_ELECTRIC_STATUS;  // Command tag
        tx.electric_status.efficiency = 0;                 // STUB: not measured yet

        if (tp_ != nullptr) {                              // Live source available?
            tx.electric_status.power =                     // Instant power [W]
                api_pack_bf16(tp_->driver_phy.power.get_power_w());
            tx.electric_status.supply =                    // Bus voltage [V]
                api_pack_bf16(tp_->driver_phy.supply.get_voltage());
            tx.electric_status.current =                   // Input current [A]
                api_pack_bf16(tp_->driver_phy.power.get_supply_current_usage_a());
        } else {
            tx.electric_status.power = 0;                  // No source: zeros
            tx.electric_status.supply = 0;
            tx.electric_status.current = 0;
        }
    }

    /**
     * @brief Pack 0x06 Motion Status 1 (STUB: motion pipeline not wired).
     * @param tx Response chunk to fill.
     */
    void pack_motion_status1(ApiChunk& tx) {
        tx.motion_status1.cmd = API_CMD_MOTION_STATUS_1;     // Command tag
        tx.motion_status1.motion_mode = motion.motion_mode;  // Executing cmd type
        api_pack_i48(tx.motion_status1.position, motion.position_q16);  // Q1.31.16
    }

    /**
     * @brief Pack 0x07 Motion Status 2 (STUB: motion pipeline not wired).
     * @param tx Response chunk to fill.
     */
    void pack_motion_status2(ApiChunk& tx) {
        tx.motion_status2.cmd = API_CMD_MOTION_STATUS_2;     // Command tag
        uint32_t v24 = api_pack_f24(motion.velocity_rps);    // Velocity -> float24
        tx.motion_status2.velocity[0] = (uint8_t)(v24);      // Bits 0-7
        tx.motion_status2.velocity[1] = (uint8_t)(v24 >> 8); // Bits 8-15
        tx.motion_status2.velocity[2] = (uint8_t)(v24 >> 16);  // Bits 16-23
        tx.motion_status2.accel = api_pack_bf16(motion.accel_rps2);  // bf16 accel
        tx.motion_status2.torque = motion.torque_raw;        // Raw torque value
    }

    /**
     * @brief Pack 0x08 Motion Status 3 (STUB: motion pipeline not wired).
     * @param tx Response chunk to fill.
     */
    void pack_motion_status3(ApiChunk& tx) {
        tx.motion_status3.cmd = API_CMD_MOTION_STATUS_3;     // Command tag
        api_pack_i24(tx.motion_status3.pos_err, motion.pos_err_q16);    // Q1.7.16
        tx.motion_status3.vel_err = api_pack_bf16(motion.vel_err_rps);  // bf16 vel err
        tx.motion_status3.queue = motion.queue & 0x0Fu;      // Remaining commands
        tx.motion_status3.time_scale = api_pack_timescale(motion.timescale_q10);  // Q1.3.8
    }

    /**
     * @brief Pack 0x09 Motor Properties from the live Motor profile.
     * @param tx Response chunk to fill.
     */
    void pack_motor_properties(ApiChunk& tx) {
        tx.motor_props.cmd = API_CMD_MOTOR_PROPERTIES;   // Command tag
        tx.motor_props.temp_lim = 0;                     // Cleared for the no-source case
        tx.motor_props.resistance = 0;                   // Cleared for the no-source case
        tx.motor_props.inductance = 0;                   // Cleared for the no-source case
        tx.motor_props.torque = 0;                       // Cleared for the no-source case

        if (tp_ != nullptr) {                            // Live source available?
            const Motor& m = tp_->current_driver.motor;  // Motor profile alias
            tx.motor_props.temp_lim = m.get_max_temp();  // Thermal limit [degC]
            tx.motor_props.resistance = api_pack_bf16(m.get_resistance());           // R [Ohm]
            tx.motor_props.inductance = api_pack_bf16(m.get_effective_inductance()); // L [H]
            tx.motor_props.torque = api_pack_bf16(m.get_max_torque());               // Kt [N*m]
        }
    }

    /**
     * @brief Apply an incoming 0x09 chunk to the lib Motor profile.
     *
     * Write convention: a bf16 field whose raw word is zero (0.0f) is
     * treated as "no change" so a pure read request never mutates the
     * motor profile; temp_lim == 0 likewise means "keep current limit".
     *
     * @param req Received chunk (CMD 0x09).
     */
    void apply_motor_properties(const ApiChunk& req) {
        if (tp_ == nullptr) return;                      // No live source bound
        Motor& m = tp_->current_driver.motor;            // Motor profile alias

        if (req.motor_props.temp_lim != 0) {             // Thermal limit requested
            m.set_max_temp(req.motor_props.temp_lim);
        }
        if (req.motor_props.resistance != 0) {           // Resistance requested
            m.set_resistance(api_unpack_bf16(req.motor_props.resistance));
        }
        if (req.motor_props.inductance != 0) {           // Inductance requested
            float l = api_unpack_bf16(req.motor_props.inductance);
            m.set_inductance_d(l);                       // Store into the D axis
            m.set_inductance_q(l);                       // Assume Ld == Lq (isotropic)
        }
        if (req.motor_props.torque != 0) {               // Torque constant requested
            m.set_max_torque(api_unpack_bf16(req.motor_props.torque));
        }
    }

    // ------------------------------------------------------------------
    // State
    // ------------------------------------------------------------------

    ApiReadCb  on_read_  = nullptr;   ///< Hardware RX callback (chunk fetch).
    ApiWriteCb on_write_ = nullptr;   ///< Hardware TX callback (chunk push).
    TunePulseHeart* tp_ = nullptr;    ///< Live telemetry source (lib layer).
};