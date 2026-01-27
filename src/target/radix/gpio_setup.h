#ifndef GPIO_SETUP_H
#define GPIO_SETUP_H

#include "stm32g4xx_ll_bus.h"   // Include STM32G4 bus peripheral library
#include "stm32g4xx_ll_gpio.h"  // Include STM32G4 GPIO peripheral library
#include "stm32g4xx_ll_rcc.h"   // Include STM32G4 RCC peripheral library

void IO_Init() {
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};  // GPIO initialization structure

    // Enable the peripheral clock for GPIOA and GPIOB
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);

    //////////////////////////////// ADC PINS /////////////////////////////////

    // Configure ADC1 GPIOs Mode
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;  // Set pins to analog mode
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;      // Disable pull-up / pull-down

    // Assign selcted mode to GPIOs
    GPIO_InitStruct.Pin = LL_GPIO_PIN_3 | LL_GPIO_PIN_7;  // PA3 (I_CH1 ADC1_IN4), PA7 (I_CH2 ADC2_IN4)
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);                // Initialize GPIOA

    GPIO_InitStruct.Pin = LL_GPIO_PIN_0 | LL_GPIO_PIN_14;  // PB0 (I_CH4 ADC1_IN15), PB14 (VSENS ADC1_IN5)
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);                 // Initialize GPIOB

    /////////////////////////////////// TIM2 //////////////////////////////////

    // Configure TIM2 GPIOs Mode
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_1;  // TIM2

    // Assign selcted mode to GPIOs (3-phase only: PA1 is intentionally NOT PWM)
    GPIO_InitStruct.Pin = LL_GPIO_PIN_0 | LL_GPIO_PIN_2;  // PA0, PA2
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);                // Initialize GPIOA

    GPIO_InitStruct.Pin = LL_GPIO_PIN_11;  // PB11
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);  // Initialize GPIOB

    /////////////////////////////////// SPI1 //////////////////////////////////

    // PB3   ------> SPI3_SCK (AF6)
    // PB4   ------> SPI3_MISO (AF6)
    // PB5   ------> SPI3_MOSI (AF6)
    // PB7   ------> SPI3_CS  (GPIO) (I2C1-SDA pin)

    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_6;

    GPIO_InitStruct.Pin = LL_GPIO_PIN_3;
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LL_GPIO_PIN_4;
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LL_GPIO_PIN_5;
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_MEDIUM;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = 0;

    GPIO_InitStruct.Pin = LL_GPIO_PIN_7;  // CS
    LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_7);
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /////////////////////////// GENERAL PURPOSE GPIO //////////////////////////

    // Assign GPIO default state
    LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_6);  // EN_BST default LOW before enable

    // Configure GPIOs Mode
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_MEDIUM;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
    GPIO_InitStruct.Alternate = 0;

    // Assign selcted mode to GPIOs
    // EN_BST output (ACTIVE HIGH)
    GPIO_InitStruct.Pin = LL_GPIO_PIN_6;  // PC6
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_6);
    LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    // EN_CH pins as open-drain outputs (default LOW)
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = 0;
    GPIO_InitStruct.Pin = LL_GPIO_PIN_10;  // PB10 (EN_CH1)
    LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_10);
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = LL_GPIO_PIN_2;  // PB2 (EN_CH2)
    LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_2);
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = LL_GPIO_PIN_1;  // PB1 (EN_CH3)
    LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_1);
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = LL_GPIO_PIN_6;  // PA6 (EN_CH4)
    LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_6);
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // LED PINS (active HIGH -> ON when set)
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
    GPIO_InitStruct.Pin = LL_GPIO_PIN_13 | LL_GPIO_PIN_14 | LL_GPIO_PIN_15;  // PC13, PC14, PC15
    LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_13 | LL_GPIO_PIN_14 | LL_GPIO_PIN_15);
    LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_14);
    LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    // DAC pin (PA5) to analog mode for IVREF
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Pin = LL_GPIO_PIN_5;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

#endif  // GPIO_SETUP_H