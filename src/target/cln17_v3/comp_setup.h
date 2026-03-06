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

/**
 * @brief Initialize DAC1 for overcurrent comparator thresholds
 * DAC1_CH1: COMP1 threshold (Phase B overcurrent on PA1)
 * DAC1_CH2: COMP2 threshold (Phase A overcurrent on PA3)
 */
static inline void MX_DAC1_Init(void) {
    // Enable DAC1 clock using HAL macro
    __HAL_RCC_DAC1_CLK_ENABLE();

    // Wait for clock to stabilize
    volatile uint32_t wait = 1000;
    while (wait--) {
        __NOP();
    }

    // Configure DAC1 Channel 1 (COMP1 threshold for Phase B)
    LL_DAC_SetMode(DAC1, LL_DAC_CHANNEL_1, LL_DAC_MODE_NORMAL_OPERATION);
    LL_DAC_SetOutputBuffer(DAC1, LL_DAC_CHANNEL_1, LL_DAC_OUTPUT_BUFFER_DISABLE);        // Internal connection only
    LL_DAC_SetOutputConnection(DAC1, LL_DAC_CHANNEL_1, LL_DAC_OUTPUT_CONNECT_INTERNAL);  // Connect to COMP
    LL_DAC_Enable(DAC1, LL_DAC_CHANNEL_1);                                               // Enable channel 1

    // Wait for DAC channel to stabilize
    wait = 1000;
    while (wait--) {
        __NOP();
    }

    // Configure DAC1 Channel 2 (COMP2 threshold for Phase A)
    LL_DAC_SetMode(DAC1, LL_DAC_CHANNEL_2, LL_DAC_MODE_NORMAL_OPERATION);
    LL_DAC_SetOutputBuffer(DAC1, LL_DAC_CHANNEL_2, LL_DAC_OUTPUT_BUFFER_DISABLE);        // Internal connection only
    LL_DAC_SetOutputConnection(DAC1, LL_DAC_CHANNEL_2, LL_DAC_OUTPUT_CONNECT_INTERNAL);  // Connect to COMP
    LL_DAC_Enable(DAC1, LL_DAC_CHANNEL_2);                                               // Enable channel 2

    // Wait for DAC channel to stabilize
    wait = 1000;
    while (wait--) {
        __NOP();
    }

    // Set default threshold values (will be updated by current sense callbacks)
    LL_DAC_ConvertData12RightAligned(DAC1, LL_DAC_CHANNEL_1, 0);  // Mid-scale default
    LL_DAC_ConvertData12RightAligned(DAC1, LL_DAC_CHANNEL_2, 0);  // Mid-scale default

    // Wait for DAC to settle
    wait = 1000;
    while (wait--) {
        __NOP();
    }
}

/**
 * @brief Initialize DAC3 for overvoltage and overcurrent comparator thresholds
 * DAC3_CH1: COMP3 threshold (Overvoltage on PA0)
 * DAC3_CH2: COMP4 threshold (Phase C overcurrent on PB0)
 */
static inline void MX_DAC3_Init(void) {
    // Enable DAC3 clock using HAL macro
    __HAL_RCC_DAC3_CLK_ENABLE();

    // Wait for clock to stabilize
    volatile uint32_t wait = 1000;
    while (wait--) {
        __NOP();
    }

    // Configure DAC3 Channel 1 (COMP3 threshold for overvoltage)
    LL_DAC_SetMode(DAC3, LL_DAC_CHANNEL_1, LL_DAC_MODE_NORMAL_OPERATION);
    LL_DAC_SetOutputBuffer(DAC3, LL_DAC_CHANNEL_1, LL_DAC_OUTPUT_BUFFER_DISABLE);        // Internal connection only
    LL_DAC_SetOutputConnection(DAC3, LL_DAC_CHANNEL_1, LL_DAC_OUTPUT_CONNECT_INTERNAL);  // Connect to COMP
    LL_DAC_Enable(DAC3, LL_DAC_CHANNEL_1);                                               // Enable channel 1

    // Wait for DAC channel to stabilize
    wait = 1000;
    while (wait--) {
        __NOP();
    }

    // Configure DAC3 Channel 2 (COMP4 threshold for Phase C overcurrent)
    LL_DAC_SetMode(DAC3, LL_DAC_CHANNEL_2, LL_DAC_MODE_NORMAL_OPERATION);
    LL_DAC_SetOutputBuffer(DAC3, LL_DAC_CHANNEL_2, LL_DAC_OUTPUT_BUFFER_DISABLE);        // Internal connection only
    LL_DAC_SetOutputConnection(DAC3, LL_DAC_CHANNEL_2, LL_DAC_OUTPUT_CONNECT_INTERNAL);  // Connect to COMP
    LL_DAC_Enable(DAC3, LL_DAC_CHANNEL_2);                                               // Enable channel 2

    // Wait for DAC channel to stabilize
    wait = 1000;
    while (wait--) {
        __NOP();
    }

    // Set default threshold values (will be updated by callbacks)
    LL_DAC_ConvertData12RightAligned(DAC3, LL_DAC_CHANNEL_1, 1024);  // Mid-scale default
    LL_DAC_ConvertData12RightAligned(DAC3, LL_DAC_CHANNEL_2, 0);     // Mid-scale default

    // Wait for DAC to settle
    wait = 1000;
    while (wait--) {
        __NOP();
    }
}

/**
 * @brief DAC callback for Phase A overcurrent threshold (COMP2)
 * Called by CurrentSense when overcurrent limit is updated
 * @param threshold ADC threshold value (0-4095, 12-bit)
 */
void DAC_Set_COMP2_OC_Threshold(uint16_t threshold) {
    LL_DAC_ConvertData12RightAligned(DAC1, LL_DAC_CHANNEL_2, threshold >> 4);  // Update DAC1_CH2 for COMP2
}

/**
 * @brief DAC callback for Phase B overcurrent threshold (COMP1)
 * Called by CurrentSense when overcurrent limit is updated
 * @param threshold ADC threshold value (0-4095, 12-bit)
 */
void DAC_Set_COMP1_OC_Threshold(uint16_t threshold) {
    LL_DAC_ConvertData12RightAligned(DAC1, LL_DAC_CHANNEL_1, threshold >> 4);  // Update DAC1_CH1 for COMP1
}

/**
 * @brief DAC callback for Phase C overcurrent threshold (COMP4)
 * Called by CurrentSense when overcurrent limit is updated
 * @param threshold ADC threshold value (0-4095, 12-bit)
 */
void DAC_Set_COMP4_OC_Threshold(uint16_t threshold) {
    LL_DAC_ConvertData12RightAligned(DAC3, LL_DAC_CHANNEL_2, threshold >> 4);  // Update DAC3_CH2 for COMP4
}

/**
 * @brief DAC callback for overvoltage threshold (COMP3)
 * Called by SupplyVoltage when overvoltage limit is updated
 * @param threshold ADC threshold value (0-65535, 16-bit scaled to 12-bit)
 */
void DAC_Set_COMP3_OV_Threshold(uint16_t threshold) {
    SerialUSB.print("   [DEBUG] DAC_Set_COMP3_OV_Threshold called with threshold=");
    SerialUSB.println(threshold);
    SerialUSB.flush();

    // Convert 16-bit ADC value to 12-bit DAC value (right shift by 4)
    uint16_t dac_value = threshold >> 4;  // Scale from 16-bit to 12-bit

    SerialUSB.print("   [DEBUG] dac_value (12-bit)=");
    SerialUSB.println(dac_value);
    SerialUSB.flush();

    // Check if DAC3 is enabled before writing
    if (!LL_DAC_IsEnabled(DAC3, LL_DAC_CHANNEL_1)) {
        SerialUSB.println("   [ERROR] DAC3_CH1 is not enabled!");
        SerialUSB.flush();
        return;
    }

    SerialUSB.println("   [DEBUG] DAC3_CH1 is enabled, writing value...");
    SerialUSB.flush();

    // Use software trigger to update DAC value safely
    LL_DAC_ConvertData12RightAligned(DAC3, LL_DAC_CHANNEL_1, dac_value);  // Update DAC3_CH1 for COMP3

    // Wait for DAC to settle
    volatile uint32_t wait = 100;
    while (wait--) {
        __NOP();
    }

    SerialUSB.println("   [DEBUG] DAC3_CH1 updated successfully");
    SerialUSB.flush();
}

/**
 * @brief Initialize COMP1 for Phase B overcurrent detection
 * INP: PA1 (I_B current sense)
 * INM: DAC1_CH1 (threshold reference)
 */
static inline void MX_COMP1_Init(void) {
    // Enable SYSCFG clock (contains COMP peripheral)
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);

    // Disable COMP1 before configuration
    LL_COMP_Disable(COMP1);

    // Set input plus to PA1 (IO1 for COMP1)
    LL_COMP_SetInputPlus(COMP1, LL_COMP_INPUT_PLUS_IO1);

    // Set input minus to DAC1_CH1
    LL_COMP_SetInputMinus(COMP1, LL_COMP_INPUT_MINUS_DAC1_CH1);

    // Set hysteresis for noise immunity
    LL_COMP_SetInputHysteresis(COMP1, LL_COMP_HYSTERESIS_MEDIUM);

    // Output polarity: non-inverted (high when INP > INM)
    LL_COMP_SetOutputPolarity(COMP1, LL_COMP_OUTPUTPOL_INVERTED);

    // Blanking source: none
    LL_COMP_SetOutputBlankingSource(COMP1, LL_COMP_BLANKINGSRC_NONE);

    // Enable COMP1
    LL_COMP_Enable(COMP1);

    // Wait for COMP1 startup time (~10us)
    volatile uint32_t wait = 200;
    while (wait--) {
        __NOP();
    }
}

/**
 * @brief Initialize COMP2 for Phase A overcurrent detection
 * INP: PA3 (I_A current sense)
 * INM: DAC1_CH2 (threshold reference)
 */
static inline void MX_COMP2_Init(void) {
    // Disable COMP2 before configuration
    LL_COMP_Disable(COMP2);

    // Set input plus to PA3 (IO2 for COMP2)
    LL_COMP_SetInputPlus(COMP2, LL_COMP_INPUT_PLUS_IO2);

    // Set input minus to DAC1_CH2
    LL_COMP_SetInputMinus(COMP2, LL_COMP_INPUT_MINUS_DAC1_CH2);

    // Set hysteresis for noise immunity
    LL_COMP_SetInputHysteresis(COMP2, LL_COMP_HYSTERESIS_MEDIUM);

    // Output polarity: non-inverted (high when INP > INM)
    LL_COMP_SetOutputPolarity(COMP2, LL_COMP_OUTPUTPOL_INVERTED);

    // Blanking source: none
    LL_COMP_SetOutputBlankingSource(COMP2, LL_COMP_BLANKINGSRC_NONE);

    // Enable COMP2
    LL_COMP_Enable(COMP2);

    // Wait for COMP2 startup time (~10us)
    volatile uint32_t wait = 200;
    while (wait--) {
        __NOP();
    }
}

/**
 * @brief Initialize COMP3 for overvoltage detection
 * INP: PA0 (VSENS voltage sense)
 * INM: DAC3_CH1 (threshold reference)
 */
static inline void MX_COMP3_Init(void) {
    // Disable COMP3 before configuration
    LL_COMP_Disable(COMP3);

    // Set input plus to PA0 (IO1 for COMP3)
    LL_COMP_SetInputPlus(COMP3, LL_COMP_INPUT_PLUS_IO1);

    // Set input minus to DAC3_CH1
    LL_COMP_SetInputMinus(COMP3, LL_COMP_INPUT_MINUS_DAC3_CH1);

    // Set hysteresis for noise immunity
    LL_COMP_SetInputHysteresis(COMP3, LL_COMP_HYSTERESIS_MEDIUM);

    // Output polarity: non-inverted (high when INP > INM)
    LL_COMP_SetOutputPolarity(COMP3, LL_COMP_OUTPUTPOL_NONINVERTED);

    // Blanking source: none
    LL_COMP_SetOutputBlankingSource(COMP3, LL_COMP_BLANKINGSRC_NONE);

    // Enable COMP3
    LL_COMP_Enable(COMP3);

    // Wait for COMP3 startup time (~10us)
    volatile uint32_t wait = 200;
    while (wait--) {
        __NOP();
    }
}

/**
 * @brief Initialize COMP4 for Phase C overcurrent detection
 * INP: PB0 (I_C current sense)
 * INM: DAC3_CH2 (threshold reference)
 */
static inline void MX_COMP4_Init(void) {
    // Disable COMP4 before configuration
    LL_COMP_Disable(COMP4);

    // Set input plus to PB0 (IO1 for COMP4)
    LL_COMP_SetInputPlus(COMP4, LL_COMP_INPUT_PLUS_IO1);

    // Set input minus to DAC3_CH2
    LL_COMP_SetInputMinus(COMP4, LL_COMP_INPUT_MINUS_DAC3_CH2);

    // Set hysteresis for noise immunity
    LL_COMP_SetInputHysteresis(COMP4, LL_COMP_HYSTERESIS_MEDIUM);

    // Output polarity: non-inverted (high when INP > INM)
    LL_COMP_SetOutputPolarity(COMP4, LL_COMP_OUTPUTPOL_INVERTED);

    // Blanking source: none
    LL_COMP_SetOutputBlankingSource(COMP4, LL_COMP_BLANKINGSRC_NONE);

    // Enable COMP4
    LL_COMP_Enable(COMP4);

    // Wait for COMP4 startup time (~10us)
    volatile uint32_t wait = 200;
    while (wait--) {
        __NOP();
    }
}

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

    // Enable EXTI line 29 (COMP3) - both edges
    LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_29);           // Unmask interrupt
    LL_EXTI_EnableRisingTrig_0_31(LL_EXTI_LINE_29);   // Rising edge trigger (fault)
    LL_EXTI_EnableFallingTrig_0_31(LL_EXTI_LINE_29);  // Falling edge trigger (clear)

    // Enable EXTI line 30 (COMP4) - both edges
    LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_30);            // Unmask interrupt
    LL_EXTI_EnableRisingTrig_0_31(LL_EXTI_LINE_30);    // Rising edge trigger (fault)
    LL_EXTI_DisableFallingTrig_0_31(LL_EXTI_LINE_30);  // Falling edge trigger (clear)

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
            g_heart_ptr->feedback.error.set_fatal_oc();  // Set fatal overcurrent error
        }
    }
}

/**
 * @brief Initialize complete protection system
 * Initializes DACs, comparators, GPIO fault input, and EXTI interrupts for overcurrent and overvoltage protection
 */
inline void COMP_Protection_Init(void) {
    MX_DAC1_Init();            // Initialize DAC1 for COMP1/COMP2 thresholds
    MX_DAC3_Init();            // Initialize DAC3 for COMP3/COMP4 thresholds
    MX_COMP1_Init();           // Initialize COMP1 (Phase B overcurrent)
    MX_COMP2_Init();           // Initialize COMP2 (Phase A overcurrent)
    MX_COMP3_Init();           // Initialize COMP3 (Overvoltage)
    MX_COMP4_Init();           // Initialize COMP4 (Phase C overcurrent)
    MX_DRV_FAULT_GPIO_Init();  // Initialize PB10 (DRV_FAULT) GPIO with pullup and EXTI
    COMP_EXTI_Init();          // Enable comparator interrupts
}

#endif  // COMP_SETUP_CLN17V3_H
