#pragma once

/**
 * @file tp_enums.h
 * @brief TPCP v1 - the result and status enumerations exchanged as payload bytes.
 *
 * These are the small vocabularies the protocol speaks back to a master: why a
 * frame was rejected (TpError), where the drive is in its lifecycle
 * (TpDriveState), what is degraded but not yet fatal (TpWarn), and the mode /
 * stop / calibration / safe-state selectors a master writes.
 *
 * They are grouped here, apart from the opcodes, because they are payload
 * values rather than frame identity: an opcode says what a frame is, these say
 * what it reports. Several are mirrored into the packed status word (see
 * TpDeviceStatus and the TP_STATUS_*_SHIFT macros), so their numeric values are
 * wire-visible and must not be reordered.
 *
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#include <stdint.h>  // Fixed-width integer types

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

