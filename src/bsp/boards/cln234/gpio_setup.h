/**
 * @file gpio_setup.h
 * @brief LL-driver GPIO initialization for CLN234.
 *
 * Configures ADC/analog pins (VSENS, I_A, I_B, I_C, TEMP, ENC_CAL),
 * TIM1 PWM pins, DRV_EN / DRV_ERR, CAN stand-by, SW1, SWD and the
 * control-connector pins; provides IO_Init().
 *
 * NOTE: SPI1 pins are configured by SPI1_GPIO_Init() (spi_setup.h) and
 * the WS2812 pin by LED_Init() (ws2812.h).
 *
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#ifndef GPIO_SETUP_CLN234_H
#define GPIO_SETUP_CLN234_H

#include "stm32g4xx_ll_bus.h"   // STM32G4 bus peripheral library
#include "stm32g4xx_ll_gpio.h"  // STM32G4 GPIO peripheral library
#include "stm32g4xx_ll_rcc.h"   // STM32G4 RCC peripheral library
#include "target.h"             // CLN234 pinout definitions

void IO_Init() {
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};  // GPIO initialization structure

    // Enable peripheral clocks for all GPIO ports
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);

    //////////////////////////////// ADC PINS /////////////////////////////////

    // Configure ADC pins as analog inputs
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;  // Analog mode for ADC
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;      // No pull-up/pull-down

    // PA0 (VSENS), PA1 (I_B), PA3 (I_A)
    GPIO_InitStruct.Pin = LL_GPIO_PIN_0 | LL_GPIO_PIN_1 | LL_GPIO_PIN_3;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // PB0 (I_C)
    GPIO_InitStruct.Pin = LL_GPIO_PIN_0;
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // PC4 (TEMP - external board temperature sensor)
    GPIO_InitStruct.Pin = LL_GPIO_PIN_4;
    LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /////////////////////////////////// TIM1 ///////////////////////////////////

    // Configure TIM1 PWM pins (complementary mode)
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;        // Alternate function mode
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL; // Push-pull output
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;      // High speed for PWM
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;               // No pull resistors
    GPIO_InitStruct.Alternate = LL_GPIO_AF_6;             // AF6 = TIM1

    // PA8 (PWM1H - TIM1_CH1), PA9 (PWM2H - TIM1_CH2), PA10 (PWM3H - TIM1_CH3)
    GPIO_InitStruct.Pin = LL_GPIO_PIN_8 | LL_GPIO_PIN_9 | LL_GPIO_PIN_10;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // PB13 (PWM1L - TIM1_CH1N), PB14 (PWM2L - TIM1_CH2N) - AF6
    GPIO_InitStruct.Alternate = LL_GPIO_AF_6;             // AF6 = TIM1
    GPIO_InitStruct.Pin = LL_GPIO_PIN_13 | LL_GPIO_PIN_14;
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // PB15 (PWM3L - TIM1_CH3N) - AF4 (different from PB13/PB14!)
    GPIO_InitStruct.Alternate = LL_GPIO_AF_4;             // AF4 = TIM1_CH3N for PB15
    GPIO_InitStruct.Pin = LL_GPIO_PIN_15;
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /////////////////////////// CONTROL / MISC GPIO ///////////////////////////

    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;           // Push-pull outputs
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_MEDIUM;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = 0;

    // PB12 (DRV_EN): gate-driver enable, LOW until the driver is configured
    GPIO_InitStruct.Pin = LL_GPIO_PIN_12;
    LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_12);  // Default disabled
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // PA2 (ENC_CAL): encoder calibration line, held LOW permanently
    GPIO_InitStruct.Pin = LL_GPIO_PIN_2;
    LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_2);   // ENC_CAL = LOW
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // PC13 (CAN_SD): CAN transceiver stand-by control
    GPIO_InitStruct.Pin = LL_GPIO_PIN_13;
    LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    // PB1 (DRV_ERR): gate-driver fault input, pull-up (active low)
    GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
    GPIO_InitStruct.Pin = LL_GPIO_PIN_1;
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // SW1 (PC14) - Button input with pull-up
    GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
    GPIO_InitStruct.Pin = LL_GPIO_PIN_14;
    LL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

#endif  // GPIO_SETUP_CLN234_H