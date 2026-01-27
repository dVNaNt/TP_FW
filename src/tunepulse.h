#ifndef TUNEPULSE_H
#define TUNEPULSE_H

#include "setup/interrupt_setup.h"

#if defined(TARGET_RADIX)
    #include "target/radix/dac_setup.h"
    #include "target/radix/comp_setup.h"

    inline void tunepulse_init() {
        SystemClock_Config();         // Configure system clock to 170 MHz
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
        SerialUSB.println("0. SystemClock_Config...");
        SerialUSB.flush();
        delay(100);
        SystemClock_Config();         // Configure system clock to 170 MHz
        SerialUSB.println("   [OK] Clock configured");
        SerialUSB.flush();
        delay(100);
        
        SerialUSB.println("1. IO_Init...");
        SerialUSB.flush();
        delay(100);
        IO_Init();                    // Initialize GPIO pins
        SerialUSB.println("   [OK] GPIO initialized");
        SerialUSB.flush();
        delay(100);
        
        SerialUSB.println("2. DMA_Init...");
        SerialUSB.flush();
        delay(100);
        DMA_Init();                   // Initialize DMA for ADC
        SerialUSB.println("   [OK] DMA initialized");
        SerialUSB.flush();
        delay(100);
        
        SerialUSB.println("3. ADC_Init...");
        SerialUSB.flush();
        delay(100);
        ADC_Init();                   // Initialize ADC channels
        SerialUSB.println("   [OK] ADC initialized");
        SerialUSB.flush();
        delay(100);
        
        SerialUSB.println("4. PWM_Init...");
        SerialUSB.flush();
        delay(100);
        PWM_Init();                   // Initialize TIM1 PWM (complementary mode)
        SerialUSB.println("   [OK] PWM initialized");
        SerialUSB.flush();
        delay(100);
        
        SerialUSB.println("5. COMP_Protection_Init...");
        SerialUSB.flush();
        delay(100);
        COMP_Protection_Init();       // Initialize overcurrent protection (DAC1/DAC3 + COMP1/COMP2/COMP4)
        SerialUSB.println("   [OK] Comparator initialized");
        SerialUSB.flush();
        delay(100);
        
        SerialUSB.println("6. SPI1_Init...");
        SerialUSB.flush();
        delay(100);
        SPI1_Init();                  // Initialize SPI1 for encoder
        SerialUSB.println("   [OK] SPI1 initialized");
        SerialUSB.flush();
        delay(100);
        
        SerialUSB.println("7. SPI1_Start...");
        SerialUSB.flush();
        delay(100);
        SPI1_Start();                 // Start SPI communication
        SerialUSB.println("   [OK] SPI1 started");
        SerialUSB.flush();
        delay(100);
        
        SerialUSB.println("8. NVIC_Init...");
        SerialUSB.flush();
        delay(100);
        NVIC_Init();                  // Configure interrupt priorities
        SerialUSB.println("   [OK] NVIC configured");
        SerialUSB.flush();
        delay(100);

        SerialUSB.println("9. Enable DRV8323H...");
        SerialUSB.flush();
        delay(100);
        LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_11);  // DRV_EN = 1
        SerialUSB.println("   [OK] Driver enabled");
        SerialUSB.flush();
        delay(100);
    }
#else
    #error "No target defined! Define TARGET_RADIX or TARGET_CLN17V3"
#endif

#endif  // TUNEPULSE_H