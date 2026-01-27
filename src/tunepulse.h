#ifndef TUNEPULSE_H
#define TUNEPULSE_H

#include "setup/interrupt_setup.h"

#if defined(TARGET_RADIX)
    #include "target/radix/dac_setup.h"
    #include "target/radix/comp_setup.h"

    inline void tunepulse_init() {
        IO_Init();                    // Initialize GPIO pins
        DAC_Init();                   // Set IVREF to 50% (DAC1)
        DMA_Init();                   // Initialize DMA for ADC
        ADC_Init();                   // Initialize ADC channels
        PWM_Init();                   // Initialize TIM2 PWM
        COMP_Protection_Init();       // Initialize overcurrent protection (DAC3 + COMP2/COMP4)
        SPI3_Init();                  // Initialize SPI for encoder
        SPI3_Start();                 // Start SPI communication
        NVIC_Init();                  // Configure interrupt priorities

        digitalWriteFast(PB_2, HIGH); // Enable channel 2
        digitalWriteFast(PA_4, HIGH); // Enable temperature sensor
    }
#elif defined(TARGET_CLN17V3)
    #include "target/cln17_v3/comp_setup.h"

    inline void tunepulse_init() {
        IO_Init();                    // Initialize GPIO pins
        DMA_Init();                   // Initialize DMA for ADC
        ADC_Init();                   // Initialize ADC channels
        PWM_Init();                   // Initialize TIM1 PWM (complementary mode)
        COMP_Protection_Init();       // Initialize overcurrent protection (DAC1/DAC3 + COMP1/COMP2/COMP4)
        SPI1_Init();                  // Initialize SPI1 for encoder
        SPI1_Start();                 // Start SPI communication
        NVIC_Init();                  // Configure interrupt priorities

        // Enable DRV8323H driver
        LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_11);  // DRV_EN = 1
    }
#else
    #error "No target defined! Define TARGET_RADIX or TARGET_CLN17V3"
#endif

#endif  // TUNEPULSE_H