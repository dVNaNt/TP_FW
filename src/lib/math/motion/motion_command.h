/**
 * @file motion_command.h
 * @brief Data structures describing queued motion segments.
 *
 * Defines MotionCommandTM (one trajectory segment: mode bitfield, duration,
 * displacement, end velocity, torque limit, trigger configuration) plus the
 * supporting enums MotionMode, TorqueTrigger and IO trigger masks. Commands
 * are produced by the application layer and consumed by MotionExecutor.
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#pragma once

#include "../../common.h"

/**
 * @struct MotionCommand
 * @brief Base mathematical structure of a trajectory segment.
 */
struct MotionCommand {
    uint32_t duration = 0;     // Segment duration in us (0 = infinite or instantaneous, mode-dependent)
    int32_t displacement = 0;  // Displacement (in ticks/steps)
    float end_velocity = 0.0f; // Segment end velocity (RPS)
};

/**
 * @struct MotionExecutorOutput
 * @brief Output telemetry of the trajectory generator for every clock tick.
 */
struct MotionExecutorOutput {
    int64_t position_q32; /**< Interpolated absolute position Q32.32 */
    float velocity_rps;   /**< Instantaneous target velocity (RPS) */
    float torque;         /**< Normalized torque limit (0.0f ... 1.0f) */
};

/**
 * @brief Basic motion modes enumeration (2 bits)
 */
enum class MotionMode : uint8_t {
    FREERUN = 0b00,       // Open loop: constant speed (displacement acts as a position limit)
    FREERUN_LERP = 0b01,  // Open loop: linear velocity interpolation (LERP)
    ANCHOR = 0b10,        // INSTANTANEOUS mathematical anchor shift (executes in 0 ticks)
    HERMITE = 0b11        // Closed loop: Hermite spline trajectory (becomes Hold when disp=0)
};

/**
 * @brief Torque/current trigger operating modes (2 bits)
 */
enum class TorqueTrigger : uint8_t {
    DISABLED = 0b00,     // Trigger disabled
    LOWER_THAN = 0b01,   // Fires when torque drops below the threshold
    HIGHER_THAN = 0b10,  // Fires when torque exceeds the threshold
    RESERVED = 0b11      // Reserved
};

/**
 * @brief Packed 16-bit register of motion mode and triggers.
 * Optimized for atomic network transfer and copying.
 */
union MotionConfig {
    struct {
        uint16_t mode : 2;      // Bits 0-1: Motion mode (MotionMode)
        uint16_t torque : 2;    // Bits 2-3: Torque trigger mode (TorqueTrigger)
        uint16_t pos_mask : 6;  // Bits 4-9: Positive-direction IO mask
        uint16_t neg_mask : 6;  // Bits 10-15: Negative-direction IO mask (inverted)
    } fields;

    uint16_t raw;

    // Default constructor: the neg mask is initialized to 0x3F (all ones), 
    // meaning no active-low trigger is configured.
    constexpr MotionConfig(MotionMode m = MotionMode::FREERUN,
                           TorqueTrigger t = TorqueTrigger::DISABLED,
                           uint8_t pos = 0,
                           uint8_t neg = 0x3F)
        : fields{
              static_cast<uint16_t>(m), 
              static_cast<uint16_t>(t),
              static_cast<uint16_t>(pos & 0x3F),
              static_cast<uint16_t>(neg & 0x3F)
          } {}

    constexpr explicit MotionConfig(uint16_t raw_val) : raw(raw_val) {}

    inline MotionMode getMode() const { return static_cast<MotionMode>(fields.mode); }
    inline TorqueTrigger getTorqueTrigger() const { return static_cast<TorqueTrigger>(fields.torque); }
    inline uint8_t getPosMask() const { return fields.pos_mask; }
    inline uint8_t getNegMask() const { return fields.neg_mask; }
};

/**
 * @struct MotionCommandTM
 * @brief Full command packet (Telemetry + Mode) pushed into the buffer
 */
struct MotionCommandTM {
public:
    MotionCommand motion;
    uint16_t torque = UINT16_MAX; /**< Torque limit OR trigger threshold (when trigger is active) */
    MotionConfig mode = MotionMode::ANCHOR; 
};