/**
 * @file comp_dac.h
 * @brief Comparator-based overcurrent and overvoltage protection for CLN17 V3
 *
 * This module implements hardware-based protection using STM32G4 comparators and DACs:
 * - COMP1: Overcurrent protection for Phase B (I_B on PA1 vs DAC1_CH1)
 * - COMP2: Overcurrent protection for Phase A (I_A on PA3 vs DAC1_CH2)
 * - COMP3: Overvoltage protection (VSENS on PA0 vs DAC3_CH1)
 * - COMP4: Overcurrent protection for Phase C (I_C on PB0 vs DAC3_CH2)
 *
 * PROTECTION STRATEGY:
 *
 * Overcurrent (OC) Protection:
 * - When comparator triggers: Set all MOSFETs to Hi-Z (disable all outputs)
 * - PWM duty cycles forced to INT16_MIN (inactive state)
 * - Error flag set in feedback.error manager
 *
 * Overvoltage (OV) Protection:
 * - When comparator triggers: Enable regenerative braking
 * - Low-side MOSFETs ON, high-side MOSFETs OFF (short motor phases to ground)
 * - PWM duty cycles forced to 0 (low-side active)
 * - Error flag set in feedback.error manager
 *
 * Priority Handling:
 * - Overvoltage has higher priority than overcurrent
 * - If both OV and OC occur: Low-side ON, high-side OFF (regenerative braking)
 * - This safely dissipates energy while protecting against overcurrent
 *
 * Interrupt Architecture:
 * - Overcurrent (COMP1/2/4): Rising edge only (Fault detection, sets error flag)
 * - Overvoltage (COMP3): Both edges (Rising sets fault, Falling clears fault)
 * - PB10 (DRV_FAULT): Both edges (Falling sets fault, Rising clears fault)
 *
 * Recovery Timer (managed in lib.h):
 * - Timer starts when ALL hard faults are cleared (comparator outputs go low)
 * - 200ms timeout before permanent error clearing
 * - If fault repeats during recovery: Timer resets and starts again
 * - Timer logic is in TunePulseHeart::tick() for centralized error management
 *
 * Integration with PWM Driver:
 * - TIM1_Set_PWM_Values() checks feedback.error flags before updating PWM
 * - If error present: PWM values determined by error state, not software
 * - Ensures hardware protection overrides software control
 *
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#ifndef BSP_STM32G4_COMP_DAC_H
#define BSP_STM32G4_COMP_DAC_H

#include <Arduino.h>
#include "core/dev_log.h"   // TP_LOG_VAL: threshold echo exists only in DEV_LOG_ENABLE builds
#include "bsp/stm32g4/chip.h"
#include "stm32g4xx_hal.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_comp.h"
#include "stm32g4xx_ll_dac.h"
#include "stm32g4xx_ll_exti.h"
#include "stm32g4xx_ll_gpio.h"
#include "stm32g4xx_ll_system.h"

// Forward declaration for immediate PWM protection in ISR
extern void TIM1_Set_PWM_Values(const int16_t pwmValues[3]);

// Forward declarations
void COMP_Set_OC_Protection_State(void);
void COMP_Set_OV_Protection_State(void);
void COMP_Clear_OC_Protection_State(void);
void COMP_Clear_OV_Protection_State(void);

static inline void MX_DAC1_Init(void) {         // Begin function to initialize DAC1 peripheral
    RCC->AHB2ENR |= RCC_AHB2ENR_DAC1EN;         // Enable DAC1 clock on the AHB2 bus via RCC register
    (void)(RCC->AHB2ENR & RCC_AHB2ENR_DAC1EN);  // Dummy read to ensure clock is fully propagated before access

    // Configure both channels: Mode 3 (internal connect, buffer OFF)
    DAC1->MCR = (3UL << DAC_MCR_MODE1_Pos) | (3UL << DAC_MCR_MODE2_Pos);

    DAC1->CR |= DAC_CR_EN1 | DAC_CR_EN2;  // Enable DAC1 Channel 1 and Channel 2 simultaneously

    // Wait until both hardware ready flags are set
    while ((DAC1->SR & (DAC_SR_DAC1RDY | DAC_SR_DAC2RDY)) != (DAC_SR_DAC1RDY | DAC_SR_DAC2RDY))
        __NOP();  // Prevent compiler optimization of the loop while waiting for hardware stabilization

    // OPTIMIZATION: A conservative startup threshold avoids false trips of the current 
    // comparators at power-up, before the control code programs the real threshold.
    DAC1->DHR12R1 = 256U;  
    DAC1->DHR12R2 = 256U;  
}  // End of MX_DAC1_Init function

static inline void MX_DAC3_Init(void) {         // Begin function to initialize DAC3 peripheral
    RCC->AHB2ENR |= RCC_AHB2ENR_DAC3EN;         // Enable DAC3 clock on the AHB2 bus via RCC register
    (void)(RCC->AHB2ENR & RCC_AHB2ENR_DAC3EN);  // Dummy read to guarantee bus clock synchronization

    // Set both channels to Mode 3: buffer disabled, internal peripheral connection
    DAC3->MCR = (3UL << DAC_MCR_MODE1_Pos) | (3UL << DAC_MCR_MODE2_Pos);
    DAC3->CR |= DAC_CR_EN1 | DAC_CR_EN2;  // Turn on both DAC3 channels at the same time

    // Poll status register for wake-up completion
    while ((DAC3->SR & (DAC_SR_DAC1RDY | DAC_SR_DAC2RDY)) != (DAC_SR_DAC1RDY | DAC_SR_DAC2RDY))
        __NOP();  // Execute no-operation instruction to save power during short hardware spinlock

    DAC3->DHR12R1 = 1024U;  // Apply default startup threshold for overvoltage protection to Channel 1
    DAC3->DHR12R2 = 256U;  // OPTIMIZATION: Safe OC startup threshold
}  // End of MX_DAC3_Init function

void DAC_Set_COMP2_OC_Threshold(uint16_t threshold) {  // Function to dynamically update Phase A overcurrent threshold
    DAC1->DHR12R2 =
        (threshold >> 4) & 0x0FFF;  // Right shift by 4, mask safely to 12 bits, and write directly to DAC1 CH2 register
    TP_LOG_VAL("Set COMP2: ", threshold);                // Console echo of the new threshold (DEV_LOG_ENABLE builds only)

}  // End of DAC_Set_COMP2_OC_Threshold function

void DAC_Set_COMP1_OC_Threshold(uint16_t threshold) {  // Function to dynamically update Phase B overcurrent threshold
    DAC1->DHR12R1 =
        (threshold >> 4) & 0x0FFF;  // Right shift by 4, mask safely to 12 bits, and write directly to DAC1 CH1 register
    TP_LOG_VAL("Set COMP1: ", threshold);                // Console echo of the new threshold (DEV_LOG_ENABLE builds only)
    }  // End of DAC_Set_COMP1_OC_Threshold function

void DAC_Set_COMP4_OC_Threshold(uint16_t threshold) {  // Function to dynamically update Phase C overcurrent threshold
    DAC3->DHR12R2 =
        (threshold >> 4) & 0x0FFF;  // Right shift by 4, mask safely to 12 bits, and write directly to DAC3 CH2 register
    TP_LOG_VAL("Set COMP4: ", threshold);                // Console echo of the new threshold (DEV_LOG_ENABLE builds only)
}  // End of DAC_Set_COMP4_OC_Threshold function

void DAC_Set_COMP3_OV_Threshold(uint16_t threshold) {   // Function to dynamically update overvoltage threshold without blocking code
    if ((DAC3->CR & DAC_CR_EN1) != 0U) {            // Safely verify that DAC3 Channel 1 is actually enabled to prevent hardfaults
        DAC3->DHR12R1 = (threshold >> 4) & 0x0FFF;  // Perform 16-to-12-bit downscaling safely and update threshold register
    }  // End of hardware enable check condition
    TP_LOG_VAL("Set COMP3 OV: ", threshold);                // Console echo of the new threshold (DEV_LOG_ENABLE builds only)
}  // End of DAC_Set_COMP3_OV_Threshold function

static inline void MX_COMPS_INIT(void) {          // Begin function to initialize all four analog comparators simultaneously
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;         // Enable peripheral clock for SYSCFG bus which contains the comparators
    (void)(RCC->APB2ENR & RCC_APB2ENR_SYSCFGEN);  // Perform a dummy read to ensure AHB/APB bridge clock synchronization

    COMP1->CSR = 0UL;  // Force disable COMP1 and clear all previous register states to ensure hardware safety
    COMP2->CSR = 0UL;  // Force disable COMP2 and clear all previous register states to ensure hardware safety
    COMP3->CSR = 0UL;  // Force disable COMP3 and clear all previous register states to ensure hardware safety
    COMP4->CSR = 0UL;  // Force disable COMP4 and clear all previous register states to ensure hardware safety

    // Apply Phase B overcurrent config: PA1 input, DAC1_CH1 ref, inverted
    COMP1->CSR = LL_COMP_INPUT_PLUS_IO1 | LL_COMP_INPUT_MINUS_DAC1_CH1 | LL_COMP_HYSTERESIS_MEDIUM | LL_COMP_OUTPUTPOL_INVERTED |
                 LL_COMP_BLANKINGSRC_NONE;
    // Apply Phase A overcurrent config: PA3 input, DAC1_CH2 ref, inverted
    COMP2->CSR = LL_COMP_INPUT_PLUS_IO2 | LL_COMP_INPUT_MINUS_DAC1_CH2 | LL_COMP_HYSTERESIS_MEDIUM | LL_COMP_OUTPUTPOL_INVERTED |
                 LL_COMP_BLANKINGSRC_NONE;
    // Apply Overvoltage config: PA0 input, DAC3_CH1 ref, non-inverted
    COMP3->CSR = LL_COMP_INPUT_PLUS_IO1 | LL_COMP_INPUT_MINUS_DAC3_CH1 | LL_COMP_HYSTERESIS_MEDIUM |
                 LL_COMP_OUTPUTPOL_NONINVERTED | LL_COMP_BLANKINGSRC_NONE;
    // Apply Phase C overcurrent config: PB0 input, DAC3_CH2 ref, inverted
    COMP4->CSR = LL_COMP_INPUT_PLUS_IO1 | LL_COMP_INPUT_MINUS_DAC3_CH2 | LL_COMP_HYSTERESIS_MEDIUM | LL_COMP_OUTPUTPOL_INVERTED |
                 LL_COMP_BLANKINGSRC_NONE;

    COMP1->CSR |= COMP_CSR_EN;  // Safely set the enable bit for COMP1 only after the configuration is fully applied
    COMP2->CSR |= COMP_CSR_EN;  // Safely set the enable bit for COMP2 only after the configuration is fully applied
    COMP3->CSR |= COMP_CSR_EN;  // Safely set the enable bit for COMP3 only after the configuration is fully applied
    COMP4->CSR |= COMP_CSR_EN;  // Safely set the enable bit for COMP4 only after the configuration is fully applied

    volatile uint32_t wait = 1000;  // Initialize a volatile delay variable to wait for analog comparator startup time
    while (wait--) {                // Loop continuously until the countdown reaches zero
        __NOP();  // Execute hardware no-operation to prevent compiler from optimizing out the 10 microsecond delay
    }  // End of analog stabilization delay loop
}  // End of MX_COMPS_INIT function

/**
 * @brief Initialize GPIO PB10 (DRV_FAULT) with pullup for external fault detection
 * PB10 is connected to driver fault output (active low)
 * When driver detects overcurrent, it pulls PB10 low triggering interrupt
 */
#if BOARD_HAS_DRV_FAULT_EXTI
static inline void MX_DRV_FAULT_GPIO_Init(void) {
    // Enable SYSCFG clock (required for EXTI configuration)
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);

    // Enable GPIOB clock
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);

    // Configure PB10 as input with pullup
    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_10, LL_GPIO_MODE_INPUT);        // Set as input
    LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_10, LL_GPIO_PULL_UP);           // Enable pullup resistor
    LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_10, LL_GPIO_SPEED_FREQ_HIGH);  // High speed for fast response

    // Configure EXTI line 10 for PB10
    LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTB, LL_SYSCFG_EXTI_LINE10);  // Connect EXTI10 to GPIOB

    // FIXED: Both-edge interrupts so the fault flag can also be cleared
    LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_10);           
    LL_EXTI_EnableRisingTrig_0_31(LL_EXTI_LINE_10);   // Rising edge trigger (clear fault)
    LL_EXTI_EnableFallingTrig_0_31(LL_EXTI_LINE_10);  // Falling edge trigger (set fault)

    // Clear any pending flags
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_10);

    // Enable NVIC interrupt for EXTI10
    NVIC_SetPriority(EXTI15_10_IRQn, 0);  // Highest-1 priority for safety
    NVIC_EnableIRQ(EXTI15_10_IRQn);
}
#endif  // BOARD_HAS_DRV_FAULT_EXTI

/**
 * @brief Configure EXTI interrupts for all comparators
 * OC Comparators: Rising edge only (Fault detection)
 * OV Comparator: Both edges (Fault detection and clearing)
 */
static inline void COMP_EXTI_Init(void) {
    // COMP1 is connected to EXTI21
    // COMP2 is connected to EXTI22
    // COMP3 is connected to EXTI29
    // COMP4 is connected to EXTI30

    // Enable EXTI line 21 (COMP1) - Rising edge only
    LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_21);            
    LL_EXTI_EnableRisingTrig_0_31(LL_EXTI_LINE_21);    
    LL_EXTI_DisableFallingTrig_0_31(LL_EXTI_LINE_21);  

    // Enable EXTI line 22 (COMP2) - Rising edge only
    LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_22);            
    LL_EXTI_EnableRisingTrig_0_31(LL_EXTI_LINE_22);    
    LL_EXTI_DisableFallingTrig_0_31(LL_EXTI_LINE_22);  

    // Enable EXTI line 30 (COMP4) - Rising edge only
    LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_30);            
    LL_EXTI_EnableRisingTrig_0_31(LL_EXTI_LINE_30);    
    LL_EXTI_DisableFallingTrig_0_31(LL_EXTI_LINE_30);  

    // Enable EXTI line 29 (COMP3) - Both edges
    LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_29);           
    LL_EXTI_EnableRisingTrig_0_31(LL_EXTI_LINE_29);   
    LL_EXTI_EnableFallingTrig_0_31(LL_EXTI_LINE_29);  

    // Clear any pending flags
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_21 | LL_EXTI_LINE_22 | LL_EXTI_LINE_29 | LL_EXTI_LINE_30);

    // Enable NVIC interrupts
#if BOARD_COMP_IRQ_ARMED
    NVIC_SetPriority(COMP1_2_3_IRQn, 0);  // Highest priority for safety
    NVIC_EnableIRQ(COMP1_2_3_IRQn);

    NVIC_SetPriority(chip::kComp4Irq, 0);  // Highest priority for safety
    NVIC_EnableIRQ(chip::kComp4Irq);
#endif  // BOARD_COMP_IRQ_ARMED
}

/**
 * @brief Set overcurrent protection state (called from ISR on rising edge)
 * Disables all PWM outputs (Hi-Z state) immediately and sets error flag
 */
inline void COMP_Set_OC_Protection_State(void) {
    // OPTIMIZATION: Arrays declared static const so the ISR spends no time on stack stores
    static const int16_t pwm_hiz[3] = {INT16_MIN, INT16_MIN, INT16_MIN};  // All phases Hi-Z
    TIM1_Set_PWM_Values(pwm_hiz);                                         // Apply immediately in ISR
}

/**
 * @brief Set overvoltage protection state (called from ISR on rising edge)
 * Enables regenerative braking (low-side ON, high-side OFF) immediately and sets error flag
 */
inline void COMP_Set_OV_Protection_State(void) {
    // OPTIMIZATION: Arrays declared static const so the ISR spends no time on stack stores
    static const int16_t pwm_regen[3] = {0, 0, 0};  // All phases regenerative braking
    TIM1_Set_PWM_Values(pwm_regen);                 // Apply immediately in ISR
}

/**
 * @brief COMP1/2/3 interrupt handler (shared IRQ)
 * Handles COMP1 (Phase B OC), COMP2 (Phase A OC), COMP3 (OV)
 */
// COMP1_2_3_IRQHandler: interrupt vector, defined in the board translation unit
// of whichever board is being built, so that exactly one object file owns it.

/**
 * @brief COMP4 interrupt handler
 * Handles COMP4 (Phase C overcurrent)
 */
// COMP4_IRQHandler: interrupt vector, defined in the board translation unit
// of whichever board is being built, so that exactly one object file owns it.

/**
 * @brief EXTI15_10 interrupt handler
 * Handles PB10 (DRV_FAULT) external interrupt on both edges
 * Falling edge: Fault signal LOW -> Set error
 * Rising edge: Fault signal HIGH -> Clear error
 */
// EXTI15_10_IRQHandler: interrupt vector, defined in the board translation unit
// of whichever board is being built, so that exactly one object file owns it.

/**
 * @brief Initialize complete protection system
 * Initializes DACs, comparators, GPIO fault input, and EXTI interrupts for overcurrent and overvoltage protection
 */
inline void COMP_Protection_Init(void) {
    MX_DAC1_Init();  // Initialize DAC1 for COMP1/COMP2 thresholds
    MX_DAC3_Init();  // Initialize DAC3 for COMP3/COMP4 thresholds
    MX_COMPS_INIT();
#if BOARD_HAS_DRV_FAULT_EXTI
    MX_DRV_FAULT_GPIO_Init();  // PB10 (DRV_FAULT) input with pull-up and EXTI
#endif
    COMP_EXTI_Init();          // Enable comparator interrupts
}

#endif  // BSP_STM32G4_COMP_DAC_H