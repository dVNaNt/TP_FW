#ifndef TUNEPULSE_H
#define TUNEPULSE_H

#include "setup\interrupt_setup.h"
#include "setup/dac_setup.h"
#include "setup/comp_setup.h"

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

#endif  // TUNEPULSE_H