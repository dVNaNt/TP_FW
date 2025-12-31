#pragma once

#define TARGET_RADIX

#include "Arduino.h"

namespace PINOUT {
// Reset and clock
constexpr PinName IO_RESET = PG_10;      // NRST
constexpr PinName OSC_IN = PF_0;         // HSE in
constexpr PinName OSC_OUT = PF_1;        // HSE out

// PWM outputs
constexpr PinName PWM_CH1 = PB_11;
constexpr PinName PWM_CH2 = PA_0;
constexpr PinName PWM_CH3 = PA_1;
constexpr PinName PWM_CH4 = PA_2;

// Channel enables
constexpr PinName EN_CH1 = PB_10;
constexpr PinName EN_CH2 = PB_2;
constexpr PinName EN_CH3 = PB_1;
constexpr PinName EN_CH4 = PA_6;

constexpr PinName EN_BST = PC_6;

// Current sensing
constexpr PinName I_CH1 = PA_3; // ADC1 IN4
constexpr PinName I_CH2 = PA_7; // ADC2 IN4
constexpr PinName I_CH3 = PB_13;
constexpr PinName I_CH4 = PB_0; // ADC1 IN15

// Analog monitoring
constexpr PinName TEMP = PA_4;
constexpr PinName IVREF = PA_5;
constexpr PinName VSENS = PB_14;

// Encoder ABZ signals
constexpr PinName IO_ABZ_A = PA_8;
constexpr PinName IO_ABZ_B = PA_9;
constexpr PinName IO_ABZ_Z = PA_10;
constexpr PinName IO_ABZ_Z_ALT = PC_4;  // Alternate index signal location

// USB
constexpr PinName USB_N = PA_11;
constexpr PinName USB_P = PA_12;

// Debug interface
constexpr PinName IO_SWDIO = PA_13;
constexpr PinName IO_SWCLK = PA_14;

// I2C
constexpr PinName IO_IIC_SCL = PA_15;
constexpr PinName IO_IIC_SDA = PB_7;

// SPI
constexpr PinName IO_SPI_SCK = PB_3;
constexpr PinName IO_SPI_MISO = PB_4;
constexpr PinName IO_SPI_MOSI = PB_5;

// USART
constexpr PinName IO_USRT_CK = PB_12;
constexpr PinName IO_USRT_TX = PC_10;
constexpr PinName IO_USRT_RX = PC_11;

// CAN
constexpr PinName CAN_SD = PB_6;
constexpr PinName CAN_RX = PB_8;
constexpr PinName CAN_TX = PB_9;

// LEDs
constexpr PinName LED_GRN = PC_13;
constexpr PinName LED_BLU = PC_14;
constexpr PinName LED_RED = PC_15;
}  // namespace PINOUT

