#ifndef GPIO_SETUP_CLN17V3_H
#define GPIO_SETUP_CLN17V3_H

#include "stm32g4xx_ll_bus.h"   // STM32G4 bus peripheral library
#include "stm32g4xx_ll_gpio.h"  // STM32G4 GPIO peripheral library
#include "stm32g4xx_ll_rcc.h"   // STM32G4 RCC peripheral library
#include "target.h"             // CLN17V3 pinout definitions

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

    // PA0 (VSENS - ADC2_IN1), PA1 (I_B - ADC1_IN2/ADC2_IN2), PA3 (I_A - ADC1_IN4)
    GPIO_InitStruct.Pin = LL_GPIO_PIN_0 | LL_GPIO_PIN_1 | LL_GPIO_PIN_3;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // PB0 (I_C - ADC1_IN15)
    GPIO_InitStruct.Pin = LL_GPIO_PIN_0;
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /////////////////////////////////// TIM1 //////////////////////////////////

    // Configure TIM1 PWM pins (complementary mode)
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;       // Alternate function mode
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL; // Push-pull output
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;     // High speed for PWM
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;              // No pull resistors
    GPIO_InitStruct.Alternate = LL_GPIO_AF_6;            // AF6 = TIM1

    // PA8 (PWM1H - TIM1_CH1), PA9 (PWM2H - TIM1_CH2), PA10 (PWM3H - TIM1_CH3)
    GPIO_InitStruct.Pin = LL_GPIO_PIN_8 | LL_GPIO_PIN_9 | LL_GPIO_PIN_10;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // PB13 (PWM1L - TIM1_CH1N), PB14 (PWM2L - TIM1_CH2N) - AF6
    GPIO_InitStruct.Alternate = LL_GPIO_AF_6;            // AF6 = TIM1
    GPIO_InitStruct.Pin = LL_GPIO_PIN_13 | LL_GPIO_PIN_14;
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    // PB15 (PWM3L - TIM1_CH3N) - AF4 (different from PB13/PB14!)
    GPIO_InitStruct.Alternate = LL_GPIO_AF_4;            // AF4 = TIM1_CH3N for PB15
    GPIO_InitStruct.Pin = LL_GPIO_PIN_15;
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /////////////////////////////////// SPI1 (Encoder) ////////////////////////

    // Configure SPI1 pins for encoder interface
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;       // Alternate function mode
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;     // High speed for SPI
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL; // Push-pull output
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;              // No pull resistors
    GPIO_InitStruct.Alternate = LL_GPIO_AF_5;            // AF5 = SPI1

    // PA5 (ENC_SCK), PA6 (ENC_MISO), PA7 (ENC_MOSI)
    GPIO_InitStruct.Pin = LL_GPIO_PIN_5 | LL_GPIO_PIN_6 | LL_GPIO_PIN_7;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // PA4 (ENC_CS) - GPIO output for chip select
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = 0;
    GPIO_InitStruct.Pin = LL_GPIO_PIN_4;
    LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_4);  // CS idle high
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /////////////////////////// GENERAL PURPOSE GPIO //////////////////////////

    // DRV_EN (PB11) - Driver enable output (active high)
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_MEDIUM;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Pin = LL_GPIO_PIN_11;
    LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_11);  // Default disabled
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // DRV_FAULT (PB10) - Driver fault input with pull-up (active low)
    GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
    GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // LED pins (active low - common anode configuration)
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    
    // PB1 (LED_RED), PB2 (LED_BLU)
    GPIO_InitStruct.Pin = LL_GPIO_PIN_1 | LL_GPIO_PIN_2;
    LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_1 | LL_GPIO_PIN_2);  // LEDs off (high)
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // PC4 (LED_GRN)
    GPIO_InitStruct.Pin = LL_GPIO_PIN_4;
    LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_4);  // LED off (high)
    LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    // SW1 (PC6) - Button input with pull-up
    GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
    GPIO_InitStruct.Pin = LL_GPIO_PIN_6;
    LL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

#endif  // GPIO_SETUP_CLN17V3_H
