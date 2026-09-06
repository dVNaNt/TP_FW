#pragma once

/**
 * @file tp_wire.h
 * @brief TunePulse CAN Protocol (TPCP) v1 - wire-level definitions and codecs.
 *
 * This header is the single source of truth for everything that appears ON THE
 * BUS: the 11-bit identifier map, the opcode space, the error codes, the drive
 * state / warning enumerations and the fixed-point codecs used by every frame
 * payload. It is deliberately free of any hardware or firmware dependency so
 * that a host-side tool can include this one file and be bit-compatible with
 * the drive without pulling in the motor-control stack.
 *
 * ## Framing
 * Every core frame is EXACTLY 8 bytes. That is the entire payload of a classic
 * CAN 2.0A frame, so the protocol runs unchanged on a classic bus; on a CAN-FD
 * link the same 8-byte frames are sent as FD frames (optionally with BRS) and
 * the 9..64 byte payload sizes stay reserved for future bulk transfer.
 *
 * Byte 0 of every frame is the OPCODE. Bytes 1..7 are the opcode-specific
 * payload ("Respond value" in the original draft). Multi-byte integers are
 * little-endian - the native order of the Cortex-M4 - so packing is a store
 * and unpacking is a load. Sub-byte fields are allocated from the LSB upward.
 *
 * ## Identifier map (11-bit standard identifiers only)
 * Lower identifier == higher bus priority. The map is ordered so that a drive
 * reporting a fault always wins arbitration, network management comes next,
 * commands outrank responses, and status telemetry can never delay a command.
 *
 * | Range         | Direction     | Purpose                                  |
 * |:--------------|:--------------|:-----------------------------------------|
 * | 0x000 + node  | node -> bus   | Emergency / fault report (highest prio)  |
 * | 0x100 - 0x10F | master -> bus | Network management, sync, discovery      |
 * | 0x200 + node  | master -> node| Real-time setpoint stream (no response)  |
 * | 0x300 + node  | master -> node| Request / command channel                |
 * | 0x400 + node  | node -> master| Response channel                         |
 * | 0x500 \| byte | node -> master| Discovery byte arbitration reply         |
 * | 0x600 + node  | node -> bus   | Heartbeat and cyclic telemetry           |
 * | 0x700 - 0x7FF | reserved      | Bulk transfer / firmware update (v1.1)   |
 *
 * Node addresses: 0 = unassigned (the drive answers network management only),
 * 1..254 = assigned, 255 = broadcast (0x2FF / 0x3FF address every node at once).
 *
 * The 0x100 / 0x300 / 0x400 assignments are inherited verbatim from the
 * original can_api.md draft so existing notes stay valid; the discovery reply
 * was moved off 0x400 (where it collided with the response channel of an
 * already-addressed node) onto its own 0x500 block.
 *
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#include <stdint.h>  // Fixed-width integer types (host tools include this file standalone)

// ============================================================================
// Protocol identity
// ============================================================================

/** Protocol version reported by DEVICE_INFO / the PROTOCOL_VERSION register. */
#define TP_PROTOCOL_VERSION 0x0100u  // Major 1, minor 0 (high byte . low byte)

/** Payload size of every core protocol frame, in bytes (one classic CAN frame). */
#define TP_FRAME_SIZE 8u

/**
 * Board identity reported by DEVICE_INFO and the identity registers.
 * Override any of them from the build configuration (-D TP_DEVICE_CODE=...)
 * when the same firmware is built for a different board revision.
 */
#ifndef TP_HW_VERSION
#define TP_HW_VERSION 0x30u  // CLN17 v3.0
#endif
#ifndef TP_FW_VERSION
#define TP_FW_VERSION 0x0100u  // Firmware 1.0
#endif
#ifndef TP_DEVICE_CODE
#define TP_DEVICE_CODE 0x434C1703u  // "CL" + 0x17 (CLN17) + revision 3
#endif

// ============================================================================
// Identifier map
// ============================================================================

/** Base identifier of the emergency channel (node -> bus, highest priority). */
#define TP_ID_EMCY_BASE 0x000u

/** Base identifier of the network-management block (master -> bus). */
#define TP_ID_NMT_BASE 0x100u

/** Base identifier of the real-time setpoint stream (master -> node). */
#define TP_ID_SETPOINT_BASE 0x200u

/** Base identifier of the request / command channel (master -> node). */
#define TP_ID_REQUEST_BASE 0x300u

/** Base identifier of the response channel (node -> master). */
#define TP_ID_RESPONSE_BASE 0x400u

/** Base identifier of the discovery arbitration replies (node -> master). */
#define TP_ID_DISCOVERY_BASE 0x500u

/** Base identifier of heartbeat and cyclic telemetry (node -> bus). */
#define TP_ID_TELEMETRY_BASE 0x600u

/** First identifier of the block reserved for bulk transfer / firmware update. */
#define TP_ID_BULK_BASE 0x700u

/** Node address that every drive answers in addition to its own address. */
#define TP_NODE_BROADCAST 0xFFu

/** Node address meaning "no address assigned yet" (discovery pending). */
#define TP_NODE_UNASSIGNED 0x00u

/** Highest individually addressable node number. */
#define TP_NODE_MAX 0xFEu

/** Number of bytes in a device UID (56-bit, one full frame minus the node byte). */
#define TP_UID_BYTES 7u

/**
 * @brief Network-management identifiers inside the 0x100 block.
 *
 * All of them are broadcast frames sent by the master; every drive listens to
 * the whole block regardless of whether it already owns a node address.
 */
enum TpNmtId : uint16_t {
    TP_NMT_DISCOVER_BEGIN  = TP_ID_NMT_BASE + 0x0u,  ///< Arm the UID search on every unaddressed node.
    TP_NMT_DISCOVER_PREFIX = TP_ID_NMT_BASE + 0x1u,  ///< byte 0 = prefix length, bytes 1-7 = prefix.
    TP_NMT_DISCOVER_ASSIGN = TP_ID_NMT_BASE + 0x2u,  ///< bytes 0-6 = full UID, byte 7 = address to assign.
    TP_NMT_DISCOVER_END    = TP_ID_NMT_BASE + 0x3u,  ///< Leave discovery mode, resume normal filtering.
    TP_NMT_GLOBAL_CONTROL  = TP_ID_NMT_BASE + 0x4u,  ///< byte 0 = TpGlobalCmd, bytes 4-7 = key when required.
    TP_NMT_SYNC            = TP_ID_NMT_BASE + 0x5u,  ///< Latch setpoints and emit sync-triggered telemetry.
    TP_NMT_TIME            = TP_ID_NMT_BASE + 0x6u   ///< bytes 0-3 = master wall clock [us], informational.
};

/**
 * The discovery prefix length travels in BYTE 0, not in the frame's DLC.
 *
 * The original draft encoded it as the DLC, which cannot survive a CAN-FD link
 * (payload sizes above 8 are quantized to 12/16/20/24/32/48/64, so a DLC is not
 * a byte count) and forces every receiver to plumb the DLC through to the
 * application. Carrying the length in the payload keeps the rule that EVERY
 * protocol frame is 8 bytes, on both classic CAN and CAN-FD.
 */
#define TP_DISCOVER_PREFIX_MAX 6u  // A 7-byte UID is resolved in seven steps: prefixes 0..6

/** Sub-commands carried in byte 0 of a TP_NMT_GLOBAL_CONTROL broadcast. */
enum TpGlobalCmd : uint8_t {
    TP_GLOBAL_STOP        = 0x00u,  ///< Emergency stop: every node enters its configured safe state.
    TP_GLOBAL_DISABLE     = 0x01u,  ///< Disable the output stage on every node.
    TP_GLOBAL_ENABLE      = 0x02u,  ///< Enable the output stage on every node.
    TP_GLOBAL_FAULT_RESET = 0x03u,  ///< Clear latched faults on every node.
    TP_GLOBAL_HEARTBEAT   = 0x04u,  ///< Ask every node to emit one heartbeat immediately.
    TP_GLOBAL_REBOOT      = 0x05u   ///< Reboot every node (byte 1..4 must carry TP_REBOOT_KEY).
};

// ============================================================================
// Opcode space (byte 0 of every frame)
// ============================================================================

/**
 * @brief Protocol opcodes.
 *
 * Groups: 0x00-0x0F status reads (also valid as cyclic-telemetry chunk ids),
 * 0x10-0x1F real-time control and motion, 0x20-0x2F parameter registers and
 * persistence, 0x30-0x3F calibration and system control, 0x40-0x6F reserved
 * for bulk transfer, 0x7E/0x7F generic acknowledge / reject.
 *
 * A read is requested by sending the opcode with a zero payload; the drive
 * answers with the same opcode and the populated payload. Opcodes marked (W)
 * carry their arguments in the request payload and are answered with ACK (or
 * with their own status opcode where noted).
 */
enum TpOpcode : uint8_t {
    // ---- 0x00-0x0F: status reads / telemetry chunks -----------------------
    TP_OP_NOP                = 0x00u,  ///< No operation; a zero-filled frame is always ignored.
    TP_OP_DEVICE_INFO        = 0x01u,  ///< (R) hardware version, firmware version, device code.
    TP_OP_DEVICE_UID         = 0x02u,  ///< (R) 56-bit device UID.
    TP_OP_DEVICE_STATUS      = 0x03u,  ///< (R) uptime seconds + 32-bit fault/warning word.
    TP_OP_MOTOR_STATUS       = 0x04u,  ///< (R) control mode, motor type, phase order, poles, temperature.
    TP_OP_ELECTRIC_STATUS    = 0x05u,  ///< (R) efficiency, power, supply voltage, phase current.
    TP_OP_MOTION_STATUS_1    = 0x06u,  ///< (R) motion mode + 48-bit absolute position.
    TP_OP_MOTION_STATUS_2    = 0x07u,  ///< (R) velocity, acceleration, torque.
    TP_OP_MOTION_STATUS_3    = 0x08u,  ///< (R) position error, velocity error, queue depth, time scale.
    TP_OP_MOTOR_PROPERTIES   = 0x09u,  ///< (RW) temperature limit, resistance, inductance, torque constant.
    TP_OP_ENCODER_STATUS     = 0x0Au,  ///< (R) raw encoder angle, revolution count, encoder health.
    TP_OP_DRIVER_STATUS      = 0x0Bu,  ///< (R) driver lifecycle state, fault byte, warning byte, control mode.
    TP_OP_CALIBRATION_STATUS = 0x0Cu,  ///< (R) calibration stage, progress and result flags.
    TP_OP_COMM_STATUS        = 0x0Du,  ///< (R) CAN and protocol counters (tx, rx, dropped, errors).
    TP_OP_POWER_STATUS       = 0x0Eu,  ///< (R) bus current, mechanical power estimate, thermal headroom.
    TP_OP_HEARTBEAT          = 0x0Fu,  ///< (R) compact liveness chunk; emitted cyclically on its own.

    // ---- 0x10-0x1F: real-time control and motion --------------------------
    TP_OP_CONTROL_MODE       = 0x10u,  ///< (W) select control mode and enable/disable the output stage.
    TP_OP_TARGET_POSITION    = 0x11u,  ///< (W) absolute or relative position target, Q1.31.16.
    TP_OP_TARGET_VELOCITY    = 0x12u,  ///< (W) velocity target plus acceleration limit.
    TP_OP_TARGET_TORQUE      = 0x13u,  ///< (W) q-axis current target plus current limit.
    TP_OP_MOTION_LIMITS      = 0x14u,  ///< (W) absolute velocity / acceleration / jerk envelope.
    TP_OP_STOP               = 0x15u,  ///< (W) controlled stop; byte 1 selects the stop style.
    TP_OP_ANCHOR             = 0x16u,  ///< (W) redefine the current position without moving.
    // A PVT / PVcTF segment is never sent as an opcode frame: it needs all 64
    // bits (see TpMotionBurst) and travels as the bit-packed burst frame
    // instead. These two identifiers exist so the acknowledgement can name
    // which kind of segment was queued; sending them as a request opcode is
    // rejected with TP_ERR_UNKNOWN_OPCODE.
    TP_OP_MOTION_PVT         = 0x18u,  ///< (ACK label) a PVT segment was queued.
    TP_OP_MOTION_PVCTF       = 0x19u,  ///< (ACK label) a PVcTF segment was queued.
    TP_OP_MOTION_SCURVE      = 0x1Au,  ///< (W) queue a jerk-limited point-to-point move.
    TP_OP_MOTION_QUEUE_CTL   = 0x1Bu,  ///< (W) clear / pause / resume the queue, set the time scale.
    TP_OP_MOTION_QUEUE_STAT  = 0x1Cu,  ///< (R) queue depth, executor state, time scale.

    // ---- 0x20-0x2F: parameter registers and persistence -------------------
    TP_OP_REG_READ           = 0x20u,  ///< (R) read one register by 16-bit identifier.
    TP_OP_REG_WRITE          = 0x21u,  ///< (W) write one register; answered with the value actually applied.
    TP_OP_REG_INFO           = 0x22u,  ///< (R) register introspection: type, access flags, group.
    TP_OP_PARAM_SAVE         = 0x28u,  ///< (W) persist the configuration into NVM.
    TP_OP_PARAM_LOAD         = 0x29u,  ///< (W) reload the configuration from NVM.
    TP_OP_PARAM_DEFAULTS     = 0x2Au,  ///< (W) restore compiled-in defaults (does not persist by itself).
    TP_OP_PARAM_STATUS       = 0x2Bu,  ///< (R) state of the persistence engine.

    // ---- 0x30-0x3F: calibration and system control ------------------------
    TP_OP_CALIB_START        = 0x30u,  ///< (W) start a calibration routine (see TpCalibKind).
    TP_OP_CALIB_ABORT        = 0x31u,  ///< (W) abort the running calibration.
    TP_OP_CALIB_RESULT       = 0x32u,  ///< (R) identified resistance / inductance / offset.
    TP_OP_SYS_ENABLE         = 0x38u,  ///< (W) enable or disable the output stage.
    TP_OP_SYS_FAULT_RESET    = 0x39u,  ///< (W) clear latched faults.
    TP_OP_SYS_REBOOT         = 0x3Au,  ///< (W) reboot, optionally into the USB DFU bootloader.
    TP_OP_SYS_NODE_ID        = 0x3Bu,  ///< (RW) read or change this node's address.
    TP_OP_SYS_TELEMETRY      = 0x3Cu,  ///< (RW) configure one cyclic telemetry slot.
    TP_OP_SYS_WATCHDOG       = 0x3Du,  ///< (RW) configure the master-liveness watchdog.

    // ---- 0x7E-0x7F: generic replies ---------------------------------------
    TP_OP_ACK                = 0x7Eu,  ///< Generic success reply: echoes the opcode that was executed.
    TP_OP_NACK               = 0x7Fu   ///< Generic reject: echoes the opcode plus a TpError code.
};

/** Key required in bytes 1..4 of TP_OP_SYS_REBOOT to make a reboot take effect. */
#define TP_REBOOT_KEY 0x544F4F42u  // ASCII "BOOT" little-endian: guards against a stray frame

/** Key required in bytes 1..4 of TP_OP_PARAM_DEFAULTS to wipe the stored configuration. */
#define TP_DEFAULTS_KEY 0x54455344u  // ASCII "DSET" little-endian

// ============================================================================
// Result and status enumerations
// ============================================================================

/** Error codes carried in byte 2 of a TP_OP_NACK reply. */
enum TpError : uint8_t {
    TP_ERR_NONE           = 0x00u,  ///< No error (never transmitted in a NACK).
    TP_ERR_UNKNOWN_OPCODE = 0x01u,  ///< Byte 0 is not a defined opcode.
    TP_ERR_BAD_LENGTH     = 0x02u,  ///< Frame shorter than the opcode requires.
    TP_ERR_BAD_PARAM      = 0x03u,  ///< A payload field holds an undefined value.
    TP_ERR_READ_ONLY      = 0x04u,  ///< Write attempted on a read-only register.
    TP_ERR_WRITE_ONLY     = 0x05u,  ///< Read attempted on a write-only register.
    TP_ERR_UNKNOWN_REG    = 0x06u,  ///< Register identifier is not in the map.
    TP_ERR_OUT_OF_RANGE   = 0x07u,  ///< Value rejected by the register range check.
    TP_ERR_WRONG_STATE    = 0x08u,  ///< Not permitted in the current drive state.
    TP_ERR_BUSY           = 0x09u,  ///< Subsystem busy (flash write, calibration running).
    TP_ERR_QUEUE_FULL     = 0x0Au,  ///< Motion queue cannot accept another segment.
    TP_ERR_NOT_IMPLEMENTED= 0x0Bu,  ///< Opcode reserved but not implemented in this build.
    TP_ERR_ACCESS_DENIED  = 0x0Cu,  ///< Guard key missing or the drive must be disabled first.
    TP_ERR_FAULT_ACTIVE   = 0x0Du,  ///< Refused because a latched fault is pending.
    TP_ERR_NVM            = 0x0Eu,  ///< Persistent storage read/write failure.
    TP_ERR_NOT_CALIBRATED = 0x0Fu   ///< Refused because the drive is not calibrated yet.
};

/** High-level drive lifecycle state reported in the heartbeat and 0x0B. */
enum TpDriveState : uint8_t {
    TP_STATE_BOOT        = 0x0u,  ///< Powering up, peripherals not ready.
    TP_STATE_CALIBRATING = 0x1u,  ///< A calibration routine is running.
    TP_STATE_IDLE        = 0x2u,  ///< Calibrated, output stage disabled.
    TP_STATE_READY       = 0x3u,  ///< Output stage enabled, holding position.
    TP_STATE_RUNNING     = 0x4u,  ///< Executing a motion command or tracking a setpoint.
    TP_STATE_FAULT       = 0x5u,  ///< A latched fault forced the safe state.
    TP_STATE_CONFIG      = 0x6u,  ///< Discovery / configuration mode, motion refused.
    TP_STATE_REBOOTING   = 0x7u   ///< Reboot or DFU jump pending.
};

/**
 * @brief Warning bits reported in the heartbeat and in DEVICE_STATUS.
 *
 * Warnings never stop the drive by themselves - they describe conditions the
 * master should react to before they turn into faults.
 */
enum TpWarn : uint8_t {
    TP_WARN_NOT_CALIBRATED = 1u << 0,  ///< Angle calibration missing or invalid.
    TP_WARN_FLASH_BUSY     = 1u << 1,  ///< An NVM transfer is in progress.
    TP_WARN_WATCHDOG       = 1u << 2,  ///< Master liveness watchdog expired.
    TP_WARN_QUEUE_LOW      = 1u << 3,  ///< Motion queue about to starve (one segment left).
    TP_WARN_QUEUE_DROP     = 1u << 4,  ///< A motion segment was dropped (queue was full).
    TP_WARN_COMM           = 1u << 5,  ///< Frames were lost (RX ring overflow / bus errors).
    TP_WARN_TEMPERATURE    = 1u << 6,  ///< Temperature above 80 % of the trip threshold.
    TP_WARN_SUPPLY         = 1u << 7   ///< Supply voltage outside the nominal band.
};

/** Stop styles accepted in byte 1 of TP_OP_STOP. */
enum TpStopMode : uint8_t {
    TP_STOP_HOLD      = 0x0u,  ///< Freeze the position target at the measured position.
    TP_STOP_FREEWHEEL = 0x1u,  ///< Release the bridge into high impedance.
    TP_STOP_BRAKE     = 0x2u,  ///< Short the phases through the low-side switches.
    TP_STOP_QUICK     = 0x3u   ///< Clear the queue and decelerate to a hold.
};

/** Calibration routines selectable through TP_OP_CALIB_START. */
enum TpCalibKind : uint8_t {
    TP_CALIB_CURRENT_OFFSET = 0x0u,  ///< Re-run the current-sense offset calibration.
    TP_CALIB_ELECTRICAL     = 0x1u,  ///< Identify resistance and inductance (HFI).
    TP_CALIB_ANGLE          = 0x2u,  ///< Sweep and store the encoder angle table.
    TP_CALIB_FULL           = 0x3u   ///< Electrical identification followed by the angle sweep.
};

/** Flag bits accepted in the flags byte of TP_OP_CALIB_START. */
enum TpCalibFlag : uint8_t {
    TP_CALIB_FLAG_PERSIST = 0x01u,  ///< Reserved: persist the result when the run succeeds.
    TP_CALIB_FLAG_FORCE   = 0x02u   ///< Skip the stored-table fast path and sweep for real.
};

/** Behaviour selected when the drive is disabled or the watchdog expires. */
enum TpSafeState : uint8_t {
    TP_SAFE_FREEWHEEL = 0x0u,  ///< High impedance: the rotor coasts.
    TP_SAFE_BRAKE     = 0x1u,  ///< All low-side switches on: dynamic braking.
    TP_SAFE_HOLD      = 0x2u   ///< Keep the position loop closed on the current position.
};

/** Control modes selectable through TP_OP_CONTROL_MODE and register 0x0300. */
enum TpControlMode : uint8_t {
    TP_MODE_DISABLED = 0x0u,  ///< Output stage off; the drive holds its safe state.
    TP_MODE_TORQUE   = 0x1u,  ///< Direct q-axis current command.
    TP_MODE_VELOCITY = 0x2u,  ///< Velocity command; the position target ramps at that rate.
    TP_MODE_POSITION = 0x3u,  ///< Absolute position command.
    TP_MODE_MOTION   = 0x4u,  ///< Trajectory queue executed by the motion executor.
    TP_MODE_TONE     = 0x5u   ///< Coils driven as a speaker at TP_REG_TONE_FREQ.
};

/** Flag bits accepted in the flags byte of a cyclic telemetry slot. */
enum TpTelemetryFlag : uint8_t {
    TP_TLM_ENABLE   = 1u << 0,  ///< Slot active.
    TP_TLM_ON_SYNC  = 1u << 1,  ///< Emit on every SYNC frame instead of on a timer.
    TP_TLM_ON_EVENT = 1u << 2   ///< Emit whenever the underlying value changes (state chunks only).
};

// ============================================================================
// Fixed-point codecs
// ============================================================================
//
// The protocol uses four compact float formats plus two fixed-point integer
// formats. All of them are pure bit manipulation - no FPU work, no branches on
// the hot path - so a 20 kHz control loop can pack telemetry without cost.
//
//  bfloat16 (1.8.7)  : the top 16 bits of an IEEE-754 float. Full float range,
//                      ~0.4 % relative error. Used for physical parameters.
//  float24  (1.8.15) : the top 24 bits of an IEEE-754 float. Full range,
//                      ~0.008 % relative error. Used for velocity.
//  Q1.31.16 (48 bit) : signed position in revolutions with 1/65536 resolution,
//                      +/-32768 revolutions. The native encoder format widened.
//  Q1.7.16  (24 bit) : signed following error, +/-128 revolutions.
//  Q1.3.8   (12 bit) : signed time scale, 256 = 1.0x, range -8.0 .. +7.996.
//  Q1.1.6   (8 bit)  : signed normalized error, 64 = 1.0, range -2.0 .. +1.98.
// ============================================================================

/**
 * @brief Pack an IEEE-754 float into bfloat16 (1 sign / 8 exponent / 7 mantissa).
 *
 * Truncating conversion: the low 16 bits of the representation are discarded.
 * This is the same encoding the motor parameter records use in flash, so a
 * value read over CAN is bit-identical to the value stored in NVM.
 *
 * @param value Single-precision input.
 * @return 16-bit bfloat16 payload.
 */
static inline uint16_t tp_pack_bf16(float value) {
    uint32_t bits;                          // Raw IEEE-754 representation
    __builtin_memcpy(&bits, &value, 4);     // Bit-cast float -> uint32 (no aliasing UB)
    return (uint16_t)(bits >> 16);          // Keep sign, exponent and 7 mantissa bits
}

/**
 * @brief Unpack a bfloat16 payload back into a float.
 * @param raw 16-bit bfloat16 payload.
 * @return Reconstructed single-precision value.
 */
static inline float tp_unpack_bf16(uint16_t raw) {
    uint32_t bits = ((uint32_t)raw) << 16;  // Restore the discarded mantissa as zeros
    float value;                            // Reconstructed float
    __builtin_memcpy(&value, &bits, 4);     // Bit-cast uint32 -> float
    return value;
}

/**
 * @brief Pack an IEEE-754 float into float24 (1 sign / 8 exponent / 15 mantissa).
 *
 * float32 and float24 share the sign and exponent fields, so the conversion is
 * a pure truncation of the 8 lowest mantissa bits: two shifts, no branching.
 *
 * @param value Single-precision input.
 * @return 24-bit payload, right-aligned in a 32-bit container.
 */
static inline uint32_t tp_pack_f24(float value) {
    uint32_t bits;                          // Raw IEEE-754 representation
    __builtin_memcpy(&bits, &value, 4);     // Bit-cast float -> uint32
    return (bits >> 8) & 0x00FFFFFFu;       // Drop the 8 lowest mantissa bits
}

/**
 * @brief Unpack a float24 payload back into a float.
 *
 * A pure left shift: float24 is the TOP 24 bits of an IEEE-754 float, so
 * restoring it only has to put those bits back and leave the discarded low
 * mantissa bits as zero.
 *
 * There is deliberately no sign extension here. Sign extension belongs to
 * two's-complement integers; IEEE-754 is sign-magnitude, and its sign bit
 * already lands on bit 31 after the shift. Replicating it downwards - which
 * the original draft codec did - overwrites bits 30..24, i.e. the entire
 * exponent, turning every negative value into a huge number, an infinity or a
 * NaN. That defect is what this comment exists to keep from coming back.
 *
 * @param raw 24-bit payload, right-aligned.
 * @return Reconstructed single-precision value.
 */
static inline float tp_unpack_f24(uint32_t raw) {
    const uint32_t bits = (raw & 0x00FFFFFFu) << 8;  // Restore sign, exponent and 15 mantissa bits
    float value;                                     // Reconstructed float
    __builtin_memcpy(&value, &bits, 4);              // Bit-cast uint32 -> float
    return value;
}

/**
 * @brief Store a signed Q16.16 value as a little-endian 48-bit field.
 * @param dst    Destination buffer, at least 6 bytes.
 * @param q16_16 Signed value with 16 fractional bits (revolutions).
 */
static inline void tp_pack_i48(uint8_t* dst, int64_t q16_16) {
    uint64_t u = (uint64_t)q16_16;          // Two's-complement bit pattern
    dst[0] = (uint8_t)(u);                  // Bits 0-7
    dst[1] = (uint8_t)(u >> 8);             // Bits 8-15
    dst[2] = (uint8_t)(u >> 16);            // Bits 16-23
    dst[3] = (uint8_t)(u >> 24);            // Bits 24-31
    dst[4] = (uint8_t)(u >> 32);            // Bits 32-39
    dst[5] = (uint8_t)(u >> 40);            // Bits 40-47
}

/**
 * @brief Load a little-endian 48-bit field into a signed Q16.16 value.
 *
 * Sign extension is branchless: shift bit 47 up to bit 63, then shift back
 * arithmetically.
 *
 * @param src Source buffer, at least 6 bytes.
 * @return Signed value with 16 fractional bits.
 */
static inline int64_t tp_unpack_i48(const uint8_t* src) {
    uint64_t bits = (uint64_t)src[0]              // Bits 0-7
                  | ((uint64_t)src[1] << 8)       // Bits 8-15
                  | ((uint64_t)src[2] << 16)      // Bits 16-23
                  | ((uint64_t)src[3] << 24)      // Bits 24-31
                  | ((uint64_t)src[4] << 32)      // Bits 32-39
                  | ((uint64_t)src[5] << 40);     // Bits 40-47
    return (int64_t)(bits << 16) >> 16;           // Sign-extend from bit 47
}

/**
 * @brief Store the low 24 bits of a signed value as a little-endian field.
 * @param dst Destination buffer, at least 3 bytes.
 * @param v   Signed value (Q16.16 following error, or a raw 24-bit quantity).
 */
static inline void tp_pack_i24(uint8_t* dst, int32_t v) {
    uint32_t u = (uint32_t)v;               // Two's-complement bit pattern
    dst[0] = (uint8_t)(u);                  // Bits 0-7
    dst[1] = (uint8_t)(u >> 8);             // Bits 8-15
    dst[2] = (uint8_t)(u >> 16);            // Bits 16-23
}

/**
 * @brief Load a little-endian 24-bit field as a sign-extended 32-bit value.
 * @param src Source buffer, at least 3 bytes.
 * @return Sign-extended signed value.
 */
static inline int32_t tp_unpack_i24(const uint8_t* src) {
    uint32_t bits = (uint32_t)src[0]              // Bits 0-7
                  | ((uint32_t)src[1] << 8)       // Bits 8-15
                  | ((uint32_t)src[2] << 16);     // Bits 16-23
    return (int32_t)(bits << 8) >> 8;             // Sign-extend from bit 23
}

/**
 * @brief Convert a Q10 time scale (1024 = 1.0x) into the 12-bit Q1.3.8 field.
 *
 * Integer only: Q10 -> Q8 is a shift by two, followed by a symmetric
 * saturation into the 12-bit signed range.
 *
 * @param scale_q10 Signed time scale with 10 fractional bits.
 * @return 12-bit payload, right-aligned, two's complement.
 */
static inline uint16_t tp_pack_timescale(int32_t scale_q10) {
    int32_t q8 = scale_q10 >> 2;            // 1024 per unit -> 256 per unit
    if (q8 > 2047) {
        q8 = 2047;                          // Saturate at +7.996x
    }
    if (q8 < -2048) {
        q8 = -2048;                         // Saturate at -8.0x
    }
    return (uint16_t)((uint32_t)q8 & 0x0FFFu);  // Mask into the 12-bit field
}

/**
 * @brief Convert a 12-bit Q1.3.8 time-scale field back into Q10.
 * @param raw 12-bit payload, right-aligned.
 * @return Signed time scale with 10 fractional bits (1024 = 1.0x).
 */
static inline int32_t tp_unpack_timescale(uint16_t raw) {
    int16_t q8 = (int16_t)((uint16_t)(raw << 4)) >> 4;  // Sign-extend the 12-bit field
    return (int32_t)q8 << 2;                            // Q8 -> Q10
}

/**
 * @brief Pack a normalized ratio into the 8-bit Q1.1.6 error field.
 *
 * Q1.1.6 covers -2.0 .. +1.984 with 1/64 resolution: enough to show the master
 * how close the drive is to its following-error limit in a single byte.
 *
 * @param ratio Normalized value (typically error divided by its limit).
 * @return 8-bit two's-complement payload.
 */
static inline uint8_t tp_pack_q1_1_6(float ratio) {
    int32_t q = (int32_t)(ratio * 64.0f + (ratio >= 0.0f ? 0.5f : -0.5f));  // Scale and round away from zero
    if (q > 127) {
        q = 127;                            // Saturate at +1.984
    }
    if (q < -128) {
        q = -128;                           // Saturate at -2.0
    }
    return (uint8_t)((int8_t)q);            // Two's-complement byte
}

/**
 * @brief Encode a supply voltage as an unsigned byte with 0.25 V resolution.
 * @param volts Supply voltage in volts (0 .. 63.75 V).
 * @return Encoded byte; saturates instead of wrapping.
 */
static inline uint8_t tp_pack_volts_q2(float volts) {
    int32_t q = (int32_t)(volts * 4.0f + 0.5f);  // Convert to quarter-volt steps, rounded
    if (q < 0) {
        q = 0;                              // Clamp negatives (sensor glitch) to zero
    }
    if (q > 255) {
        q = 255;                            // Saturate at 63.75 V
    }
    return (uint8_t)q;
}

/**
 * @brief Encode a temperature as a signed byte in whole degrees Celsius.
 * @param celsius Temperature in degrees Celsius.
 * @return Encoded byte; INT8_MIN (-128) is reserved for "no data".
 */
static inline uint8_t tp_pack_temp_c(float celsius) {
    int32_t q = (int32_t)(celsius + (celsius >= 0.0f ? 0.5f : -0.5f));  // Round to the nearest degree
    if (q > 127) {
        q = 127;                            // Saturate at +127 degC
    }
    if (q < -127) {
        q = -127;                           // Keep -128 free as the "no data" marker
    }
    return (uint8_t)((int8_t)q);
}

// ============================================================================
// Identifier helpers
// ============================================================================

/**
 * @brief True when an 11-bit identifier addresses this node's request channel.
 * @param id      Received standard identifier.
 * @param node_id This node's assigned address.
 */
static inline bool tp_is_request_for(uint16_t id, uint8_t node_id) {
    return (id == (uint16_t)(TP_ID_REQUEST_BASE + node_id)) ||           // Directed request
           (id == (uint16_t)(TP_ID_REQUEST_BASE + TP_NODE_BROADCAST));   // Broadcast request
}

/**
 * @brief True when an 11-bit identifier addresses this node's setpoint stream.
 * @param id      Received standard identifier.
 * @param node_id This node's assigned address.
 */
static inline bool tp_is_setpoint_for(uint16_t id, uint8_t node_id) {
    return (id == (uint16_t)(TP_ID_SETPOINT_BASE + node_id)) ||          // Directed setpoint
           (id == (uint16_t)(TP_ID_SETPOINT_BASE + TP_NODE_BROADCAST));  // Broadcast setpoint
}

/**
 * @brief True when an 11-bit identifier belongs to the network-management block.
 * @param id Received standard identifier.
 */
static inline bool tp_is_nmt(uint16_t id) {
    return (id >= TP_ID_NMT_BASE) && (id < (TP_ID_NMT_BASE + 0x10u));    // 0x100..0x10F
}
