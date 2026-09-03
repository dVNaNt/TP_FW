/**
 * @file target.h
 * @brief Board-configuration umbrella for the CLN234 hardware.
 *
 * Collects MCU clock constants, PWM timer frequency/period, ADC full-scale
 * definitions (SYS_PARAMS) and pulls in all peripheral setup headers.
 *
 * Board summary:
 * - MCU            : STM32G474CEU6 (512 KiB flash, 128 KiB SRAM, LQFP48)
 * - Gate driver    : DRV8353 (SPI), enabled by DRV_EN, fault on DRV_ERR
 * - Encoder        : MT6835 (21-bit absolute), SPI mode 3
 * - Shared SPI bus : DRV8353 and MT6835 both sit on SPI1 with separate
 *                    chip selects (DRV_CS on PC15, ENC_CS on PA4). The bus
 *                    starts in DRV8353 mode (16-bit frames, SPI mode 1),
 *                    then is reconfigured for the MT6835 (8-bit, mode 3)
 *                    once the driver configuration is done.
 * - Status LED     : WS2812 on PC6 (TIM8_CH1 PWM, open-drain output)
 * - CAN            : FDCAN1 on PB8/PB9, transceiver standby on PC13
 *
 * The macros SupplyVoltage/Motor/MotorType/PhasePattern/ControlMode/
 * DriverStatus defined here are #undef'd by tunepulse.h after inclusion.
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#pragma once

#define TARGET_CLN234

#include "Arduino.h"

// Bring in the shared-SPI bus setup: the DRV8353/MT6835 bus arbitration
// primitives are needed by the driver and encoder layers below.
#include "spi_setup.h"

namespace PINOUT {
// Motor Control (PWM & Sensing)
constexpr PinName PWM1H = PA_8;   // TIM1_CH1 - High-side Gate 1
constexpr PinName PWM2H = PA_9;   // TIM1_CH2 - High-side Gate 2
constexpr PinName PWM3H = PA_10;  // TIM1_CH3 - High-side Gate 3
constexpr PinName PWM1L = PB_13;  // TIM1_CH1N - Low-side Gate 1
constexpr PinName PWM2L = PB_14;  // TIM1_CH2N - Low-side Gate 2
constexpr PinName PWM3L = PB_15;  // TIM1_CH3N - Low-side Gate 3

// Gate-driver control pins
constexpr PinName DRV_EN = PB_12;     // Driver Enable (active high)
constexpr PinName DRV_ERR = PB_1;     // Driver nFAULT input (active low)

// Current sensing (ADC, fed by the DRV8353 integrated shunt amplifiers)
constexpr PinName I_A = PA_3;   // ADC1_IN4  - Current Sense Phase A
constexpr PinName I_B = PA_1;   // ADC2_IN2  - Current Sense Phase B
constexpr PinName I_C = PB_0;   // ADC1_IN15 - Current Sense Phase C

// Voltage sensing
constexpr PinName VSENS = PA_0;  // ADC2_IN1 - Bus Voltage Sense
constexpr PinName ENC_CAL = PA_2; // Encoder calibration output (analog, unused for now)
constexpr PinName TEMP = PC_4;   // ADC2_IN5 - External board temperature sensor

// CAN bus
constexpr PinName CAN_RX = PB_8;   // CAN Receive (also BOOT0)
constexpr PinName CAN_TX = PB_9;   // CAN Transmit
constexpr PinName CAN_SD = PC_13;  // CAN Transceiver Standby/Shutdown

// Expansion UART (USART3)
constexpr PinName EXP_TX = PC_10;  // Expansion UART TX
constexpr PinName EXP_RX = PC_11;  // Expansion UART RX

// Shared encoder / gate-driver SPI bus (SPI1)
constexpr PinName ENC_SCK = PA_5;   // SPI1 SCK (AF5)
constexpr PinName ENC_MISO = PA_6;  // SPI1 MISO (AF5)
constexpr PinName ENC_MOSI = PA_7;  // SPI1 MOSI (AF5)
constexpr PinName ENC_CS = PA_4;    // MT6835 Chip Select (GPIO)
constexpr PinName DRV_CS = PC_15;   // DRV8353 Chip Select (GPIO)

// Control connector A (SPI3 / step-dir / ABZ)
constexpr PinName CTRL_A_SCK = PB_3;    // CTRL.A_SCK | SPI3 CLK | USART2 TX | TIM3 Z
constexpr PinName CTRL_A_MISO = PB_4;   // CTRL.A_MISO | SPI3 MISO | USART2 RX | TIM3 B
constexpr PinName CTRL_A_MOSI = PB_5;   // CTRL.A_MOSI | SPI3 MOSI | USART2 CK | TIM3 A
constexpr PinName CTRL_A_CS = PB_11;    // CTRL.A_CS (GPIO)

// Control connector B (I2C1 / LPUART1)
constexpr PinName CTRL_B_SCL = PA_15;   // CTRL.B_SCL | I2C1 SCL
constexpr PinName CTRL_B_A = PB_6;      // CTRL.B_A | I2C1 SDA (alt function)
constexpr PinName CTRL_B_SDA = PB_7;    // CTRL.B_SDA/B | I2C1 SDA
constexpr PinName CTRL_B_TX = PC_10;    // CTRL.B_TX | LPUART1_TX
constexpr PinName CTRL_B_RX = PC_11;    // CTRL.B_RX | LPUART1_RX

// User interface & misc
constexpr PinName LED = PC_6;        // WS2812 status LED (TIM8_CH1, open-drain)
constexpr PinName SW1 = PC_14;       // Switch/button (active low)

// Debug interface
constexpr PinName IO_SWDIO = PA_13;  // SWD Data
constexpr PinName IO_SWCLK = PA_14;  // SWD Clock
}  // namespace PINOUT

namespace SYS_PARAMS {
    // NOTE: the current-sense full scales below are inherited from the
    // CLN17 v3 design as a bring-up default. They MUST be re-derived for
    // the CLN234 shunt resistors and the DRV8353 CSA gain (see drv8353.h,
    // CSA_CTRL.CSA_GAIN) once the board BOM is fixed.
    constexpr float MAX_SUPPLY_FS = 102.3;       // Bus voltage ADC full scale [V]
    constexpr float IA_MAX_CURRENT_FS = 11.0;    // Phase A current full scale [A]
    constexpr float IB_MAX_CURRENT_FS = 11.0;    // Phase B current full scale [A]
    constexpr float IC_MAX_CURRENT_FS = 16.5;    // Phase C current full scale [A]
    constexpr int32_t ADC_DELAY_TICKS_CONST = 120;  // ADC trigger offset after PWM center
    constexpr int32_t DEADTIME_TICKS_CONST = 20;    // TIM1 dead-time in timer ticks
}
