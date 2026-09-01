/**
 * @file target.h
 * @brief Board-configuration umbrella for the CLN17 v3 hardware.
 *
 * Collects MCU clock constants, PWM timer frequency/period, ADC full-scale
 * definitions (SYS_PARAMS) and pulls in all peripheral setup headers. The
 * macros SupplyVoltage/Motor/MotorType/PhasePattern/ControlMode/DriverStatus
 * defined here are #undef'd by tunepulse.h after inclusion.
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#pragma once

#define TARGET_CLN17V3

#include "Arduino.h"

// Bring in the encoder selection (ENCODER_TYPE) first: the board pinout below
// depends on the compile-time selected encoder type (MT6835 vs TLE5012B).
#include "spi_setup.h"

namespace PINOUT {
// Motor Control (PWM & Sensing)
constexpr PinName PWM1H = PA_8;   // TIM1_CH1 - High-side Gate 1
constexpr PinName PWM2H = PA_9;   // TIM1_CH2 - High-side Gate 2
constexpr PinName PWM3H = PA_10;  // TIM1_CH3 - High-side Gate 3
constexpr PinName PWM1L = PB_13;  // TIM1_CH1N - Low-side Gate 1
constexpr PinName PWM2L = PB_14;  // TIM1_CH2N - Low-side Gate 2
constexpr PinName PWM3L = PB_15;  // TIM1_CH3N - Low-side Gate 3

// Gate-driver enable / service pins: the assignment depends on the encoder
// type selected in spi_setup.h (different board revisions).
#if (ENCODER_TYPE == ENCODER_MT6835)
constexpr PinName DRV_EN = PB_12;     // Driver Enable (MT6835 board revision)
constexpr PinName DRV_LOW = PB_11;    // Permanently driven LOW (MT6835 board revision)
#else
constexpr PinName DRV_EN = PB_11;     // Driver Enable (TLE5012B board revision)
constexpr PinName I_CAL = PB_12;      // Current Calibration/Offset (TLE5012B board revision)
#endif

constexpr PinName DRV_FAULT = PB_10;  // Driver Fault Input (active low)

// Current sensing (ADC)
constexpr PinName I_A = PA_3;   // ADC1_IN4 - Current Sense Phase A, COMP2 + DAC1.2
constexpr PinName I_B = PA_1;   // ADC2_IN2 - Current Sense Phase B, COMP1 + DAC1.1
constexpr PinName I_C = PB_0;   // ADC1_IN15 - Current Sense Phase C, COMP4 + DAC3.2
// Note: I_CAL (PB_12) is defined in the encoder-type conditional above; the
// current-calibration pin only exists on the TLE5012B board revision.

// Voltage sensing
constexpr PinName VSENS = PA_0;  // ADC2_IN1 - Bus Voltage Sense, COMP3 + DAC3.1
constexpr PinName PWR_V = PA_2;  // Power Voltage Sense


constexpr PinName CAN_RX = PB_8;   // CAN Receive (also BOOT0)
constexpr PinName CAN_TX = PB_9;   // CAN Transmit
constexpr PinName CAN_SD = PC_14;  // CAN Shutdown/Silent

constexpr PinName EXP_TX = PC_10;  // Expansion UART TX (USART3)
constexpr PinName EXP_RX = PC_11;  // Expansion UART RX (USART3)

// Encoder SPI
constexpr PinName ENC_SCK = PA_5;   // Encoder SPI Clock
constexpr PinName ENC_MISO = PA_6;  // Encoder SPI MISO
constexpr PinName ENC_MOSI = PA_7;  // Encoder SPI MOSI
constexpr PinName ENC_CS = PA_4;    // Encoder Chip Select

// Control SPI
constexpr PinName CTRL_SCK = PB_3;   // Control ERR pin | SPI3 CLK  | USART2 TX | TIM3 ABZ Z |
constexpr PinName CTRL_MISO = PB_4;  // Control STP pin | SPI3 MISO | USART2 RX | TIM3 ABZ B |
constexpr PinName CTRL_MOSI = PB_5;  // Control DIR pin | SPI3 MOSI | USART2 CK | TIM3 ABZ A | 
constexpr PinName CTRL_CS = PA_15;   // Control ENB pin | SPI3 NSS  | I2C1 SCL  | ---------- |
constexpr PinName CTRL_SDA = PB_7;   // Control DIR pin | --------- | I2C1 SDA  | ---------- |
constexpr PinName CTRL_5V = PC_15;   // Control Interface Power Supply

// User Interface & Misc
constexpr PinName LED_RED = PB_1;   // Red LED
constexpr PinName LED_BLU = PB_2;   // Blue LED
constexpr PinName LED_GRN = PC_4;   // Green LED
constexpr PinName SW1 = PC_6;       // Switch/Button

// Debug interface
constexpr PinName IO_SWDIO = PA_13;  // SWD Data
constexpr PinName IO_SWCLK = PA_14;  // SWD Clock
}  // namespace PINOUT

namespace SYS_PARAMS {
    constexpr float MAX_SUPPLY_FS = 102.3;
    constexpr float IA_MAX_CURRENT_FS = 11.0;
    constexpr float IB_MAX_CURRENT_FS = 11.0;
    constexpr float IC_MAX_CURRENT_FS = 16.5;
    constexpr int32_t ADC_DELAY_TICKS_CONST = 120;
    constexpr int32_t DEADTIME_TICKS_CONST = 20;
}

