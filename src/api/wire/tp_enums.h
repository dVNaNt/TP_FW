#pragma once

/**
 * @file tp_enums.h
 * @brief TPCP v1 - the result and status enumerations exchanged as payload bytes.
 *
 * These are the small vocabularies the protocol speaks back to a master: why a
 * frame was rejected (TpError), where the drive is in its lifecycle
 * (TpDriveState), what is degraded but not yet fatal (TpWarn), and the mode /
 * motion-task / policy selectors a master writes.
 *
 * They are grouped here, apart from the CMD byte, because they are payload
 * values rather than frame identity: a CMD says what a frame is, these say
 * what it reports. Several are mirrored into packed status words, so their
 * numeric values are wire-visible and must not be reordered.
 *
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#include <stdint.h>  // Fixed-width integer types

// ============================================================================
// Result and status enumerations
// ============================================================================

/** Error codes carried in byte 2 of a TP_CMD_NACK reply. */
enum TpError : uint8_t {
    TP_ERR_NONE            = 0x00u,  ///< No error (never transmitted in a NACK).
    TP_ERR_UNKNOWN_CMD     = 0x01u,  ///< Byte 0 is not a defined register or command.
    TP_ERR_BAD_LENGTH      = 0x02u,  ///< Frame shorter than the command requires.
    TP_ERR_BAD_PARAM       = 0x03u,  ///< A payload field holds an undefined value.
    TP_ERR_READ_ONLY       = 0x04u,  ///< Write attempted on a read-only register.
    TP_ERR_WRITE_ONLY      = 0x05u,  ///< Read attempted on a write-only register.
    TP_ERR_UNKNOWN_SUB     = 0x06u,  ///< Sub identifier is not defined for this register.
    TP_ERR_OUT_OF_RANGE    = 0x07u,  ///< Value rejected by the register range check.
    TP_ERR_WRONG_STATE     = 0x08u,  ///< Not permitted in the current drive state.
    TP_ERR_BUSY            = 0x09u,  ///< Subsystem busy (flash write, calibration running, mailbox full).
    TP_ERR_QUEUE_FULL      = 0x0Au,  ///< Motion queue cannot accept another segment.
    TP_ERR_NOT_IMPLEMENTED = 0x0Bu,  ///< Defined by the protocol but not implemented in this build.
    TP_ERR_ACCESS_DENIED   = 0x0Cu,  ///< Guard key missing or the drive must be disabled first.
    TP_ERR_FAULT_ACTIVE    = 0x0Du,  ///< Refused because a latched fault is pending.
    TP_ERR_NVM             = 0x0Eu,  ///< Persistent storage read/write failure.
    TP_ERR_NOT_CALIBRATED  = 0x0Fu,  ///< Refused because the drive is not calibrated yet.
    TP_ERR_SEGMENT         = 0x10u   ///< Motion task part does not match the segment being assembled.
};

/** High-level drive lifecycle state (4 bits in the status words and the PulseBeat). */
enum TpDriveState : uint8_t {
    TP_STATE_BOOT        = 0x0u,  ///< Powering up, current-sense offsets unknown.
    TP_STATE_CALIBRATING = 0x1u,  ///< A calibration routine is running.
    TP_STATE_IDLE        = 0x2u,  ///< Calibrated, output stage disabled.
    TP_STATE_READY       = 0x3u,  ///< Output stage enabled, holding position.
    TP_STATE_RUNNING     = 0x4u,  ///< Executing a motion command or tracking a setpoint.
    TP_STATE_FAULT       = 0x5u,  ///< A latched fault forced the safe state.
    TP_STATE_UNADDRESSED = 0x6u,  ///< No node address: discovery pending.
    TP_STATE_REBOOTING   = 0x7u   ///< Reboot or DFU jump pending.
};

/**
 * @brief Warning bits reported in the status words.
 *
 * Warnings never stop the drive by themselves - they describe conditions the
 * master should react to before they turn into faults. The PulseBeat carries
 * the highest set bit as a 4-bit group (1 + bit index, 0 = none).
 */
enum TpWarn : uint8_t {
    TP_WARN_NOT_CALIBRATED = 1u << 0,  ///< Angle calibration missing or invalid.
    TP_WARN_FLASH_BUSY     = 1u << 1,  ///< An NVM transfer is in progress.
    TP_WARN_WATCHDOG       = 1u << 2,  ///< Master liveness watchdog expired.
    TP_WARN_QUEUE_LOW      = 1u << 3,  ///< Motion queue about to starve (one segment left).
    TP_WARN_QUEUE_DROP     = 1u << 4,  ///< A motion segment was dropped (queue full or incomplete).
    TP_WARN_COMM           = 1u << 5,  ///< Frames were lost (RX ring overflow / bus errors).
    TP_WARN_TEMPERATURE    = 1u << 6,  ///< Driver temperature above the warning threshold.
    TP_WARN_SUPPLY         = 1u << 7   ///< Supply voltage outside the nominal band.
};

/** Fault groups reported in the PulseBeat (4 bits): 1 + DriverPWM fault bit index. */
enum TpFaultGroup : uint8_t {
    TP_FAULT_GROUP_NONE         = 0x0u,  ///< No fault latched.
    TP_FAULT_GROUP_OVERVOLTAGE  = 0x1u,  ///< Supply above the OV trip.
    TP_FAULT_GROUP_OVERCURRENT  = 0x2u,  ///< Phase current above the OC trip.
    TP_FAULT_GROUP_OVERHEAT     = 0x3u,  ///< Driver temperature above the OT trip.
    TP_FAULT_GROUP_EXTERNAL     = 0x4u,  ///< External fault input (gate driver nFAULT).
    TP_FAULT_GROUP_UNDERVOLTAGE = 0x5u,  ///< Supply below the UV trip.
    TP_FAULT_GROUP_OVERPOWER    = 0x6u,  ///< Power / bus current above the OP trip.
    TP_FAULT_GROUP_PHASE        = 0x7u,  ///< Phase-stage error.
    TP_FAULT_GROUP_MULTIPLE     = 0xFu   ///< More than one fault latched at once.
};

/** Behaviour selected when the drive is disabled or the watchdog expires. */
enum TpSafeState : uint8_t {
    TP_SAFE_FREEWHEEL = 0x0u,  ///< High impedance: the rotor coasts.
    TP_SAFE_BRAKE     = 0x1u,  ///< All low-side switches on: dynamic braking.
    TP_SAFE_HOLD      = 0x2u   ///< Keep the position loop closed on the current position.
};

/** Control modes (the Mode nibble of Motor Properties 1 and of the status words). */
enum TpControlMode : uint8_t {
    TP_MODE_DISABLED = 0x0u,  ///< Output stage off; the drive holds its safe state.
    TP_MODE_TORQUE   = 0x1u,  ///< Direct q-axis current command.
    TP_MODE_VELOCITY = 0x2u,  ///< Velocity command; the position target ramps at that rate.
    TP_MODE_POSITION = 0x3u,  ///< Absolute position command.
    TP_MODE_MOTION   = 0x4u,  ///< Trajectory queue executed by the motion executor.
    TP_MODE_TONE     = 0x5u   ///< Coils driven as a speaker (MIDI control).
};

/** Motion task modes (the 4-bit Mode field of Motion Task 1). */
enum TpMotionTaskMode : uint8_t {
    TP_MT_HOLD         = 0x0u,  ///< Keep position, optionally for a duration.
    TP_MT_FREERUN      = 0x1u,  ///< Constant velocity / torque until a trigger fires.
    TP_MT_FREERUN_LERP = 0x2u,  ///< Velocity / torque interpolated to the end value.
    TP_MT_FREEWHEEL    = 0x3u,  ///< Release the bridge: zero-load rotation.
    TP_MT_BRAKE        = 0x4u,  ///< Short the phases: braking.
    TP_MT_SCURVE       = 0x5u,  ///< Jerk-limited point-to-point move.
    TP_MT_PVT          = 0x6u,  ///< Position / velocity / time interpolation segment.
    TP_MT_ANCHOR       = 0x7u,  ///< Redefine the current position without moving.
    TP_MT_LIMIT        = 0x8u,  ///< Adopt the velocity / acceleration / jerk limits carried.
    TP_MT_HOMING       = 0x9u,  ///< Homing sequence (not implemented).
    TP_MT_STEP_DIR     = 0xAu,  ///< Step / direction input mode (not implemented).
    TP_MT_RESET        = 0xFu   ///< Clear the queue and hold.
};

/** Position field usage of a motion task (PosMode, 2 bits). */
enum TpMtPosMode : uint8_t {
    TP_MT_POS_UNUSED  = 0x0u,  ///< No position part.
    TP_MT_POS_TARGET  = 0x1u,  ///< Position is the segment target.
    TP_MT_POS_TRIGGER = 0x2u   ///< Position ends the segment when reached.
};

/** Torque field usage of a motion task (TorqMode, 2 bits). */
enum TpMtTorqueMode : uint8_t {
    TP_MT_TORQUE_UNUSED  = 0x0u,  ///< No torque part.
    TP_MT_TORQUE_LIMIT   = 0x1u,  ///< Torque caps the segment.
    TP_MT_TORQUE_TRIGGER = 0x2u   ///< Reaching the torque ends the segment.
};

/** Time field usage of a motion task (TimeMode, 2 bits). */
enum TpMtTimeMode : uint8_t {
    TP_MT_TIME_UNUSED   = 0x0u,  ///< No time part.
    TP_MT_TIME_DURATION = 0x1u,  ///< Time is the segment duration.
    TP_MT_TIME_TRIGGER  = 0x2u   ///< Time elapsing ends the segment.
};

/** Response policy selected in Bus Settings (Comm field). */
enum TpCommMode : uint8_t {
    TP_COMM_SILENT     = 0x0u,  ///< Never transmit: execute commands, answer nothing.
    TP_COMM_ON_REQUEST = 0x1u,  ///< Answer reads and register writes; setpoints stay silent unless rejected.
    TP_COMM_EVERY      = 0x2u   ///< Answer every frame, motion tasks and burst segments included.
};

/** Bus type selected in Bus Settings. */
enum TpBusType : uint8_t {
    TP_BUS_CLASSIC = 0x0u,  ///< Classic CAN 2.0A.
    TP_BUS_FD      = 0x1u,  ///< CAN-FD with bit-rate switching.
    TP_BUS_FD_NOBRS = 0x2u  ///< CAN-FD frames at the arbitration rate.
};

/** Bit-rate codes of the Baud fields of Bus Settings (0 keeps the current rate). */
enum TpBaudCode : uint8_t {
    TP_BAUD_KEEP  = 0x0u,  ///< Leave the rate unchanged.
    TP_BAUD_125K  = 0x1u,  ///< 125 kbit/s.
    TP_BAUD_250K  = 0x2u,  ///< 250 kbit/s.
    TP_BAUD_500K  = 0x3u,  ///< 500 kbit/s.
    TP_BAUD_1M    = 0x4u,  ///< 1 Mbit/s.
    TP_BAUD_2M    = 0x5u,  ///< 2 Mbit/s (data phase only).
    TP_BAUD_4M    = 0x6u,  ///< 4 Mbit/s (data phase only).
    TP_BAUD_5M    = 0x7u,  ///< 5 Mbit/s (data phase only).
    TP_BAUD_8M    = 0x8u   ///< 8 Mbit/s (data phase only).
};

/**
 * @brief Bit rate in kbit/s named by a TpBaudCode (0 for TP_BAUD_KEEP / unknown).
 * @param code Baud code from a Bus Settings frame.
 */
static inline uint16_t tp_baud_kbps(uint8_t code) {
    switch (code) {
        case TP_BAUD_125K: return 125u;
        case TP_BAUD_250K: return 250u;
        case TP_BAUD_500K: return 500u;
        case TP_BAUD_1M:   return 1000u;
        case TP_BAUD_2M:   return 2000u;
        case TP_BAUD_4M:   return 4000u;
        case TP_BAUD_5M:   return 5000u;
        case TP_BAUD_8M:   return 8000u;
        default:           return 0u;    // TP_BAUD_KEEP or undefined
    }
}

/**
 * @brief TpBaudCode naming a bit rate in kbit/s (TP_BAUD_KEEP when no code matches).
 * @param kbps Bit rate [kbit/s].
 */
static inline uint8_t tp_baud_code(uint16_t kbps) {
    for (uint8_t code = TP_BAUD_125K; code <= TP_BAUD_8M; ++code) {
        if (tp_baud_kbps(code) == kbps) {
            return code;                 // Exact match
        }
    }
    return TP_BAUD_KEEP;                 // Not a standard rate
}

/** Device functions an IO pin can be mapped to (IO Control Map, byte 2). */
enum TpIoFunction : uint8_t {
    TP_IO_STATUS      = 0x00u,  ///< Plain status IO: read or write the pin directly.
    TP_IO_HARD_STOP   = 0x01u,  ///< Immediate power-off input.
    TP_IO_HOME        = 0x02u,  ///< Absolute zero position switch.
    TP_IO_LIMIT_CW    = 0x03u,  ///< CW travel limit switch.
    TP_IO_WARN_CW     = 0x04u,  ///< CW travel warning switch.
    TP_IO_LIMIT_CCW   = 0x05u,  ///< CCW travel limit switch.
    TP_IO_WARN_CCW    = 0x06u,  ///< CCW travel warning switch.
    TP_IO_GEARBOX_IDX = 0x07u,  ///< Gearbox output index pulse.
    TP_IO_BRAKE       = 0x08u,  ///< Brake output.
    TP_IO_PROBE_1     = 0x10u,  ///< General purpose probe 1.
    TP_IO_PROBE_8     = 0x17u   ///< General purpose probe 8 (probes are 0x10 .. 0x17).
};

/** Number of mappable device functions (0x00 .. 0x08 plus eight probes). */
#define TP_IO_FUNCTIONS 17u

/**
 * @brief Slot index of a device function in the IO map, or 0xFF when undefined.
 * @param function TpIoFunction value.
 */
static inline uint8_t tp_io_function_slot(uint8_t function) {
    if (function <= TP_IO_BRAKE) {
        return function;                                   // 0x00 .. 0x08 -> slots 0 .. 8
    }
    if ((function >= TP_IO_PROBE_1) && (function <= TP_IO_PROBE_8)) {
        return (uint8_t)(9u + (function - TP_IO_PROBE_1));  // Probes -> slots 9 .. 16
    }
    return 0xFFu;                                          // Not a defined function
}

/**
 * @brief Device function of an IO map slot (inverse of tp_io_function_slot()).
 * @param slot Slot index 0 .. TP_IO_FUNCTIONS-1.
 */
static inline uint8_t tp_io_slot_function(uint8_t slot) {
    return (slot <= TP_IO_BRAKE) ? slot : (uint8_t)(TP_IO_PROBE_1 + (slot - 9u));
}

/** Waveforms of the MIDI control command (0xD0, low nibble of byte 1). */
enum TpMidiWaveform : uint8_t {
    TP_WAVE_SINE     = 0x0u,  ///< Sine (the polar-rotation oscillator's native output).
    TP_WAVE_SQUARE   = 0x1u,  ///< Square: bang-bang sign of the phase.
    TP_WAVE_TRIANGLE = 0x2u,  ///< Triangle: piecewise-linear, sine-aligned.
    TP_WAVE_SAWTOOTH = 0x3u   ///< Sawtooth: full-range ramp.
};

/** Envelope shapes of the MIDI control command (0xD0 Mode nibble, bits 0-1). */
enum TpMidiEnvelope : uint8_t {
    TP_MIDI_ENV_LINEAR     = 0x0u,  ///< Linear attack / decay shares of the duration.
    TP_MIDI_ENV_EXPONENTIAL= 0x1u,  ///< Curved (exponential) attack and decay, piano-like.
    TP_MIDI_ENV_PERCUSSIVE = 0x2u,  ///< Instant attack, exponential decay over the whole note.
    TP_MIDI_ENV_ORGAN      = 0x3u   ///< Flat envelope: full volume until the note ends.
};

/** Note effects of the MIDI control command (0xD0 Mode nibble, bits 2-3). */
enum TpMidiEffect : uint8_t {
    TP_MIDI_FX_NONE    = 0x0u,  ///< Plain note.
    TP_MIDI_FX_TREMOLO = 0x1u,  ///< Amplitude modulation, 8 Hz, 25 % depth.
    TP_MIDI_FX_VIBRATO = 0x2u,  ///< Pitch modulation, 6 Hz, +/-2 %.
    TP_MIDI_FX_TRILL   = 0x3u   ///< Alternation with the whole tone above, 8 Hz.
};

/** Flag bits of a cyclic telemetry slot. */
enum TpTelemetryFlag : uint8_t {
    TP_TLM_ENABLE  = 1u << 0,  ///< Slot active.
    TP_TLM_ON_SYNC = 1u << 1   ///< Emit on every Bus Settings time sync instead of on a timer.
};

/** Calibration table selectors of register 0x2F (3-bit Table field). */
enum TpCalTable : uint8_t {
    TP_TABLE_ENCODER   = 0x0u,  ///< Encoder angle correction table.
    TP_TABLE_POLES     = 0x1u,  ///< Pole map (not present).
    TP_TABLE_COGGING   = 0x2u,  ///< Cogging map (not present).
    TP_TABLE_CURRENT   = 0x3u,  ///< Current -> torque (not present).
    TP_TABLE_SPEED     = 0x4u,  ///< Speed -> torque (not present).
    TP_TABLE_OUTPUT    = 0x5u   ///< Output position map (not present).
};
