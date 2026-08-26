/**
 * @file types.h
 * @brief Shared type definitions for the PWM driver stack.
 *
 * Holds the small common vocabulary used by more than one sub-module so the
 * definitions live in exactly one place:
 * - @c CompThresholdCallback : comparator DAC-threshold update callback,
 *   shared by the overcurrent (CurrentSense) and over/undervoltage
 *   (SupplyVoltage) protection paths.
 * - @c kDefaultRecoveryCycles : default length of the fault recovery timers
 *   expressed in control-loop ticks (10000 ticks = 500 ms at the nominal
 *   20 kHz control rate).
 *
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#pragma once

#include <stdint.h>

/// Callback that programs a hardware comparator (DAC) threshold.
/// @param threshold 16-bit threshold value for the analog comparator
typedef void (*CompThresholdCallback)(uint16_t);

/// Default fault recovery timer length in control-loop ticks.
/// At the nominal 20 kHz control rate this equals 500 ms.
static constexpr int32_t kDefaultRecoveryCycles = 10000;
