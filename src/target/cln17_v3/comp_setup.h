/**
 * @file comp_setup.h
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
 * - Rising edge: Fault detection, sets error flag in feedback.error
 * - Falling edge: Fault cleared, clears error flag in feedback.error
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

#ifndef COMP_SETUP_CLN17V3_H
#define COMP_SETUP_CLN17V3_H

#include <Arduino.h>
#include "stm32g4xx_hal.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_comp.h"
#include "stm32g4xx_ll_dac.h"
#include "stm32g4xx_ll_exti.h"
#include "stm32g4xx_ll_gpio.h"
#include "stm32g4xx_ll_system.h"

// Forward declaration of TunePulseHeart for error flag access
struct TunePulseHeart;
extern TunePulseHeart* volatile g_heart_ptr;  // Global volatile pointer for ISR access

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

    DAC1->DHR12R1 = 0U;  // Set initial mid-scale/default value to Channel 1 data register
    DAC1->DHR12R2 = 0U;  // Set initial mid-scale/default value to Channel 2 data register
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
    DAC3->DHR12R2 = 0U;     // Apply default startup threshold for overcurrent to Channel 2
}  // End of MX_DAC3_Init function

void DAC_Set_COMP2_OC_Threshold(uint16_t threshold) {  // Function to dynamically update Phase A overcurrent threshold
    DAC1->DHR12R2 =
        (threshold >> 4) & 0x0FFF;  // Right shift by 4, mask safely to 12 bits, and write directly to DAC1 CH2 register
}  // End of DAC_Set_COMP2_OC_Threshold function

void DAC_Set_COMP1_OC_Threshold(uint16_t threshold) {  // Function to dynamically update Phase B overcurrent threshold
    DAC1->DHR12R1 =
        (threshold >> 4) & 0x0FFF;  // Right shift by 4, mask safely to 12 bits, and write directly to DAC1 CH1 register
}  // End of DAC_Set_COMP1_OC_Threshold function

void DAC_Set_COMP4_OC_Threshold(uint16_t threshold) {  // Function to dynamically update Phase C overcurrent threshold
    DAC3->DHR12R2 =
        (threshold >> 4) & 0x0FFF;  // Right shift by 4, mask safely to 12 bits, and write directly to DAC3 CH2 register
}  // End of DAC_Set_COMP4_OC_Threshold function

void DAC_Set_COMP3_OV_Threshold(
    uint16_t threshold) {                           // Function to dynamically update overvoltage threshold without blocking code
    if ((DAC3->CR & DAC_CR_EN1) != 0U) {            // Safely verify that DAC3 Channel 1 is actually enabled to prevent hardfaults
        DAC3->DHR12R1 = (threshold >> 4) & 0x0FFF;  // Perform 16-to-12-bit downscaling safely and update threshold register
    }  // End of hardware enable check condition
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

    // Enable EXTI line 10 with falling edge trigger (active low fault signal)
    LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_10);           // Unmask interrupt
    LL_EXTI_EnableFallingTrig_0_31(LL_EXTI_LINE_10);  // Falling edge trigger (fault when LOW)
    LL_EXTI_DisableRisingTrig_0_31(LL_EXTI_LINE_10);  // Disable rising edge

    // Clear any pending flags
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_10);

    // Enable NVIC interrupt for EXTI10
    NVIC_SetPriority(EXTI15_10_IRQn, 0);  // Highest-1 priority for safety
    NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/**
 * @brief Configure EXTI interrupts for all comparators
 * Rising edge: Fault detection (comparator output goes high)
 * Falling edge: Fault cleared (comparator output goes low)
 */
static inline void COMP_EXTI_Init(void) {
    // COMP1 is connected to EXTI21
    // COMP2 is connected to EXTI22
    // COMP3 is connected to EXTI29
    // COMP4 is connected to EXTI30

    // Enable EXTI line 21 (COMP1) - both edges for fault detection and clearing
    LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_21);            // Unmask interrupt
    LL_EXTI_EnableRisingTrig_0_31(LL_EXTI_LINE_21);    // Rising edge trigger (fault)
    LL_EXTI_DisableFallingTrig_0_31(LL_EXTI_LINE_21);  // Falling edge trigger (clear)

    // Enable EXTI line 22 (COMP2) - both edges
    LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_22);            // Unmask interrupt
    LL_EXTI_EnableRisingTrig_0_31(LL_EXTI_LINE_22);    // Rising edge trigger (fault)
    LL_EXTI_DisableFallingTrig_0_31(LL_EXTI_LINE_22);  // Falling edge trigger (clear)

    // Enable EXTI line 30 (COMP4) - both edges
    LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_30);            // Unmask interrupt
    LL_EXTI_EnableRisingTrig_0_31(LL_EXTI_LINE_30);    // Rising edge trigger (fault)
    LL_EXTI_DisableFallingTrig_0_31(LL_EXTI_LINE_30);  // Falling edge trigger (clear)

    // Enable EXTI line 29 (COMP3) - both edges
    LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_29);           // Unmask interrupt
    LL_EXTI_EnableRisingTrig_0_31(LL_EXTI_LINE_29);   // Rising edge trigger (fault)
    LL_EXTI_EnableFallingTrig_0_31(LL_EXTI_LINE_29);  // Falling edge trigger (clear)

    // Clear any pending flags
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_21 | LL_EXTI_LINE_22 | LL_EXTI_LINE_29 | LL_EXTI_LINE_30);

    // Enable NVIC interrupts
    // COMP1/2/3 share COMP1_2_3_IRQn
    // COMP4 uses COMP4_IRQn
    NVIC_SetPriority(COMP1_2_3_IRQn, 0);  // Highest priority for safety
    NVIC_EnableIRQ(COMP1_2_3_IRQn);

    NVIC_SetPriority(COMP4_IRQn, 0);  // Highest priority for safety
    NVIC_EnableIRQ(COMP4_IRQn);
}

/**
 * @brief Set overcurrent protection state (called from ISR on rising edge)
 * Disables all PWM outputs (Hi-Z state) immediately and sets error flag
 */
inline void COMP_Set_OC_Protection_State(void) {
    // IMMEDIATE PWM protection: Set all phases to Hi-Z (INT16_MIN)
    // This provides instant hardware protection before software can react
    const int16_t pwm_hiz[3] = {INT16_MIN, INT16_MIN, INT16_MIN};  // All phases Hi-Z
    TIM1_Set_PWM_Values(pwm_hiz);                                  // Apply immediately in ISR
}

/**
 * @brief Set overvoltage protection state (called from ISR on rising edge)
 * Enables regenerative braking (low-side ON, high-side OFF) immediately and sets error flag
 */
inline void COMP_Set_OV_Protection_State(void) {
    // IMMEDIATE PWM protection: Set all phases to regenerative braking (PWM = 0)
    // This provides instant hardware protection before software can react
    const int16_t pwm_regen[3] = {0, 0, 0};  // All phases regenerative braking
    TIM1_Set_PWM_Values(pwm_regen);          // Apply immediately in ISR
}

/**
 * @brief COMP1/2/3 interrupt handler (shared IRQ)
 * Handles COMP1 (Phase B OC), COMP2 (Phase A OC), COMP3 (OV)
 */
extern "C" void COMP1_2_3_IRQHandler(void) {
    // Check COMP1 (Phase B overcurrent) - EXTI line 21
    if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_21)) {
        LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_21);  // Clear pending flag
                                                  // Set fatal overcurrent error flag
        if (g_heart_ptr != nullptr) {
            g_heart_ptr->feedback.error.set_fatal_phase2_oc();  // Set fatal overcurrent error
        }
    }

    // Check COMP2 (Phase A overcurrent) - EXTI line 22
    if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_22)) {
        LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_22);  // Clear pending flag
        if (g_heart_ptr != nullptr) {
            g_heart_ptr->feedback.error.set_fatal_phase1_oc();  // Set fatal overcurrent error
        }
    }

    // Check COMP3 (Overvoltage) - EXTI line 29
    if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_29)) {
        LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_29);  // Clear pending flag

        // Check if rising edge (fault) or falling edge (clear)
        if (LL_COMP_ReadOutputLevel(COMP3) == LL_COMP_OUTPUT_LEVEL_HIGH) {
            COMP_Set_OV_Protection_State();  // Rising edge: set overvoltage protection
            if (g_heart_ptr != nullptr)
                g_heart_ptr->feedback.error.set_fatal_dc_ovlo();  // Set fatal overvoltage error
        } else {
            if (g_heart_ptr != nullptr)
                g_heart_ptr->feedback.error.reset_fatal_dc_ovlo();  // Clear fatal overvoltage error
        }
    }
}

/**
 * @brief COMP4 interrupt handler
 * Handles COMP4 (Phase C overcurrent)
 */
extern "C" void COMP4_IRQHandler(void) {
    // Check COMP4 (Phase C overcurrent) - EXTI line 30
    if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_30)) {
        LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_30);  // Clear pending flag
        if (g_heart_ptr != nullptr) {
            g_heart_ptr->feedback.error.set_fatal_phase3_oc();  // Set fatal overcurrent error
        }
    }
}

/**
 * @brief EXTI15_10 interrupt handler
 * Handles PB10 (DRV_FAULT) external interrupt on falling edge
 * Called when driver fault signal goes LOW (active low fault detection)
 */
extern "C" void EXTI15_10_IRQHandler(void) {
    // Check EXTI line 10 (PB10 - DRV_FAULT)
    if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_10)) {
        LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_10);  // Clear pending flag

        // Set fatal overcurrent error flag
        if (g_heart_ptr != nullptr) {
            g_heart_ptr->feedback.error.set_fatal_driver();  // Set fatal overcurrent error
        }
    }
}

/**
 * @brief Initialize complete protection system
 * Initializes DACs, comparators, GPIO fault input, and EXTI interrupts for overcurrent and overvoltage protection
 */
inline void COMP_Protection_Init(void) {
    MX_DAC1_Init();  // Initialize DAC1 for COMP1/COMP2 thresholds
    MX_DAC3_Init();  // Initialize DAC3 for COMP3/COMP4 thresholds
    MX_COMPS_INIT();
    MX_DRV_FAULT_GPIO_Init();  // Initialize PB10 (DRV_FAULT) GPIO with pullup and EXTI
    COMP_EXTI_Init();          // Enable comparator interrupts
}

#endif  // COMP_SETUP_CLN17V3_H
