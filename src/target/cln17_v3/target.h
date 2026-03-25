#pragma once

#define TARGET_CLN17V3

#include "Arduino.h"

namespace PINOUT {
// Motor Control (PWM & Sensing)
constexpr PinName PWM1H = PA_8;   // TIM1_CH1 - High-side Gate 1
constexpr PinName PWM2H = PA_9;   // TIM1_CH2 - High-side Gate 2
constexpr PinName PWM3H = PA_10;  // TIM1_CH3 - High-side Gate 3
constexpr PinName PWM1L = PB_13;  // TIM1_CH1N - Low-side Gate 1
constexpr PinName PWM2L = PB_14;  // TIM1_CH2N - Low-side Gate 2
constexpr PinName PWM3L = PB_15;  // TIM1_CH3N - Low-side Gate 3

constexpr PinName DRV_EN = PB_11;     // Driver Enable
constexpr PinName DRV_FAULT = PB_10;  // Driver Fault Input (active low)

// Current sensing (ADC)
constexpr PinName I_A = PA_3;   // ADC1_IN4 - Current Sense Phase A, COMP2 + DAC1.2
constexpr PinName I_B = PA_1;   // ADC2_IN2 - Current Sense Phase B, COMP1 + DAC1.1
constexpr PinName I_C = PB_0;   // ADC1_IN15 - Current Sense Phase C, COMP4 + DAC3.2
constexpr PinName I_CAL = PB_12;  // Current Calibration/Offset

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
constexpr PinName CTRL_SCK = PB_3;   // Control SPI Clock / ERR 
constexpr PinName CTRL_MISO = PB_4;  // Control SPI MISO / STP
constexpr PinName CTRL_MOSI = PB_5;  // Control SPI MOSI / DIR
constexpr PinName CTRL_CS = PA_15;   // Control SPI Chip Select / EN
constexpr PinName CTRL_5V = PC_15;   // Control Interface Power/Signal

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
    constexpr int32_t MAX_SUPPLY_FS = 102300;
    constexpr int32_t IA_MAX_CURRENT_FS = 11000;
    constexpr int32_t IB_MAX_CURRENT_FS = 11000;
    constexpr int32_t IC_MAX_CURRENT_FS = 16500;
    constexpr int32_t ADC_DELAY_TICKS_CONST = 120;
    constexpr int32_t DEADTIME_TICKS_CONST = 20;
}

