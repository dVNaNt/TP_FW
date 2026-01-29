#ifndef COMP_SETUP_CLN17V3_H
#define COMP_SETUP_CLN17V3_H

#include <Arduino.h>
#include "stm32g4xx_hal.h"
#include "stm32g4xx_ll_comp.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_gpio.h"

// Global DAC handles for callback access
static DAC_HandleTypeDef hdac1;  // DAC1 handle for COMP1 and COMP2
static DAC_HandleTypeDef hdac3;  // DAC3 handle for COMP3 and COMP4

// Overcurrent threshold DAC value
constexpr uint32_t OVERCURRENT_DAC_VALUE = 0;

// Overvoltage threshold DAC value (default to max voltage)
constexpr uint32_t OVERVOLTAGE_DAC_VALUE = 4095;

// Fault lockout duration in milliseconds
constexpr uint32_t FAULT_LOCKOUT_MS = 1000U;

// Fault source flags (bitmask)
constexpr uint8_t FAULT_NONE = 0x00;   // No fault
constexpr uint8_t FAULT_COMP1 = 0x01;  // COMP1 triggered (PA1 - I_B)
constexpr uint8_t FAULT_COMP2 = 0x02;  // COMP2 triggered (PA3 - I_A)
constexpr uint8_t FAULT_COMP3 = 0x04;  // COMP3 triggered (PA0 - VSENS)
constexpr uint8_t FAULT_COMP4 = 0x08;  // COMP4 triggered (PB0 - I_C)

// Fault state variables
volatile uint8_t g_fault_source = FAULT_NONE;  // Which comparator triggered
volatile uint32_t g_fault_timestamp = 0;       // When fault occurred
volatile bool g_fault_active = false;          // Fault lockout active
volatile bool g_fault_needs_log = false;       // Flag to trigger logging from main loop
volatile uint8_t g_fault_log_source = FAULT_NONE;  // Fault source to log

// Forward declarations
void PWM_Emergency_Disable(uint8_t fault_source);
void COMP_Update_Fault_State(void);
void COMP_Update_DAC_Channel_A(uint16_t threshold);  // DAC update callback for Phase A
void COMP_Update_DAC_Channel_B(uint16_t threshold);  // DAC update callback for Phase B
void COMP_Update_DAC_Channel_C(uint16_t threshold);  // DAC update callback for Phase C
void COMP_Update_DAC_Voltage(uint16_t threshold);    // DAC update callback for Voltage

// =========================================================================
// DAC UPDATE CALLBACKS FOR CURRENT SENSE
// =========================================================================

/**
 * @brief Update DAC1 Channel 2 threshold for COMP2 (Phase A - I_A on PA3)
 * @param threshold: New threshold value in ADC units (0-4095 for 12-bit DAC)
 */
void COMP_Update_DAC_Channel_A(uint16_t threshold) {
    // Convert 16-bit ADC value to 12-bit DAC value (right shift by 4 bits)
    uint16_t dac_value = threshold >> 4;  // Scale from 16-bit (0-65535) to 12-bit (0-4095)
    
    // Update DAC1 Channel 2 for COMP2 (Phase A)
    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, dac_value);
    
    // Debug output to SerialUSB
    SerialUSB.print("COMP2 (Phase A) threshold updated: ");
    SerialUSB.print(threshold);
    SerialUSB.print(" ADC units -> ");
    SerialUSB.print(dac_value);
    SerialUSB.println(" DAC units");
}

/**
 * @brief Update DAC1 Channel 1 threshold for COMP1 (Phase B - I_B on PA1)
 * @param threshold: New threshold value in ADC units (0-4095 for 12-bit DAC)
 */
void COMP_Update_DAC_Channel_B(uint16_t threshold) {
    // Convert 16-bit ADC value to 12-bit DAC value (right shift by 4 bits)
    uint16_t dac_value = threshold >> 4;  // Scale from 16-bit (0-65535) to 12-bit (0-4095)
    
    // Update DAC1 Channel 1 for COMP1 (Phase B)
    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dac_value);
    
    // Debug output to SerialUSB
    SerialUSB.print("COMP1 (Phase B) threshold updated: ");
    SerialUSB.print(threshold);
    SerialUSB.print(" ADC units -> ");
    SerialUSB.print(dac_value);
    SerialUSB.println(" DAC units");
}

/**
 * @brief Update DAC3 Channel 2 threshold for COMP4 (Phase C - I_C on PB0)
 * @param threshold: New threshold value in ADC units (0-4095 for 12-bit DAC)
 */
void COMP_Update_DAC_Channel_C(uint16_t threshold) {
    // Convert 16-bit ADC value to 12-bit DAC value (right shift by 4 bits)
    uint16_t dac_value = threshold >> 4;  // Scale from 16-bit (0-65535) to 12-bit (0-4095)
    
    // Update DAC3 Channel 2 for COMP4 (Phase C)
    HAL_DAC_SetValue(&hdac3, DAC_CHANNEL_2, DAC_ALIGN_12B_R, dac_value);
    
    // Debug output to SerialUSB
    SerialUSB.print("COMP4 (Phase C) threshold updated: ");
    SerialUSB.print(threshold);
    SerialUSB.print(" ADC units -> ");
    SerialUSB.print(dac_value);
    SerialUSB.println(" DAC units");
}

/**
 * @brief Update DAC3 Channel 1 threshold for COMP3 (Voltage - VSENS on PA0)
 * @param threshold: New threshold value in ADC units (0-4095 for 12-bit DAC)
 */
void COMP_Update_DAC_Voltage(uint16_t threshold) {
    // Convert 16-bit ADC value to 12-bit DAC value (right shift by 4 bits)
    uint16_t dac_value = threshold >> 4;  // Scale from 16-bit (0-65535) to 12-bit (0-4095)
    
    // Update DAC3 Channel 1 for COMP3 (Voltage)
    HAL_DAC_SetValue(&hdac3, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dac_value);
    
    // Debug output to SerialUSB
    SerialUSB.print("COMP3 (Voltage) threshold updated: ");
    SerialUSB.print(threshold);
    SerialUSB.print(" ADC units -> ");
    SerialUSB.print(dac_value);
    SerialUSB.println(" DAC units");
}

/**
 * @brief Initialize DAC1 for comparator reference voltages
 * DAC1_OUT1 (internal) for COMP1, DAC1_OUT2 (internal) for COMP2
 */
static inline void MX_DAC1_Init(void) {
    __HAL_RCC_DAC1_CLK_ENABLE();  // Enable DAC1 clock
    
    hdac1.Instance = DAC1;  // Use global handle
    
    if (HAL_DAC_Init(&hdac1) != HAL_OK) {
        Error_Handler();
    }
    
    // Configure DAC1 Channel 1 (for COMP1)
    DAC_ChannelConfTypeDef sConfig = {0};
    sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
    sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_DISABLE;
    sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_INTERNAL;
    sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
    
    if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }
    
    // Configure DAC1 Channel 2 (for COMP2)
    if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK) {
        Error_Handler();
    }
    
    // Start both DAC channels
    if (HAL_DAC_Start(&hdac1, DAC_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_DAC_Start(&hdac1, DAC_CHANNEL_2) != HAL_OK) {
        Error_Handler();
    }
    
    // Set threshold values
    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, OVERCURRENT_DAC_VALUE);
    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, OVERCURRENT_DAC_VALUE);
}

/**
 * @brief Initialize DAC3 for COMP3 and COMP4 references
 * DAC3_OUT1 (internal) for COMP3, DAC3_OUT2 (internal) for COMP4
 */
static inline void MX_DAC3_Init(void) {
    __HAL_RCC_DAC3_CLK_ENABLE();  // Enable DAC3 clock
    
    hdac3.Instance = DAC3;  // Use global handle
    
    if (HAL_DAC_Init(&hdac3) != HAL_OK) {
        Error_Handler();
    }
    
    // Configure DAC3 Channel 1 (for COMP3 - Voltage)
    DAC_ChannelConfTypeDef sConfig = {0};
    sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
    sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_DISABLE;
    sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_INTERNAL;
    sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
    
    if (HAL_DAC_ConfigChannel(&hdac3, &sConfig, DAC_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }
    
    // Configure DAC3 Channel 2 (for COMP4 - Phase C)
    if (HAL_DAC_ConfigChannel(&hdac3, &sConfig, DAC_CHANNEL_2) != HAL_OK) {
        Error_Handler();
    }
    
    // Start both DAC3 channels
    if (HAL_DAC_Start(&hdac3, DAC_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_DAC_Start(&hdac3, DAC_CHANNEL_2) != HAL_OK) {
        Error_Handler();
    }
    
    // Set threshold values
    HAL_DAC_SetValue(&hdac3, DAC_CHANNEL_1, DAC_ALIGN_12B_R, OVERVOLTAGE_DAC_VALUE);  // Voltage threshold
    HAL_DAC_SetValue(&hdac3, DAC_CHANNEL_2, DAC_ALIGN_12B_R, OVERCURRENT_DAC_VALUE);  // Current threshold
}

/**
 * @brief Initialize COMP1 for overcurrent detection on PA1 (I_B)
 * INP: PA1, INM: DAC1_OUT1
 */
static inline void MX_COMP1_Init(void) {
    __HAL_RCC_SYSCFG_CLK_ENABLE();  // Enable COMP clock
    
    LL_COMP_Disable(COMP1);  // Disable before configuration
    
    LL_COMP_SetInputPlus(COMP1, LL_COMP_INPUT_PLUS_IO1);  // PA1
    LL_COMP_SetInputMinus(COMP1, LL_COMP_INPUT_MINUS_DAC1_CH1);  // DAC1_OUT1
    LL_COMP_SetInputHysteresis(COMP1, LL_COMP_HYSTERESIS_MEDIUM);  // Noise immunity
    LL_COMP_SetOutputPolarity(COMP1, LL_COMP_OUTPUTPOL_INVERTED);  // High when INP > INM
    LL_COMP_SetOutputBlankingSource(COMP1, LL_COMP_BLANKINGSRC_NONE);  // No blanking
    
    LL_COMP_Enable(COMP1);  // Enable COMP1
    
    // Wait for startup
    volatile uint32_t wait = 200;
    while (wait--) { __NOP(); }
}

/**
 * @brief Initialize COMP2 for overcurrent detection on PA3 (I_A)
 * INP: PA3, INM: DAC1_OUT2
 */
static inline void MX_COMP2_Init(void) {
    LL_COMP_Disable(COMP2);  // Disable before configuration
    
    LL_COMP_SetInputPlus(COMP2, LL_COMP_INPUT_PLUS_IO2);  // PA3
    LL_COMP_SetInputMinus(COMP2, LL_COMP_INPUT_MINUS_DAC1_CH2);  // DAC1_OUT2
    LL_COMP_SetInputHysteresis(COMP2, LL_COMP_HYSTERESIS_MEDIUM);  // Noise immunity
    LL_COMP_SetOutputPolarity(COMP2, LL_COMP_OUTPUTPOL_INVERTED);  // High when INP > INM
    LL_COMP_SetOutputBlankingSource(COMP2, LL_COMP_BLANKINGSRC_NONE);  // No blanking
    
    LL_COMP_Enable(COMP2);  // Enable COMP2
    
    // Wait for startup
    volatile uint32_t wait = 200;
    while (wait--) { __NOP(); }
}

/**
 * @brief Initialize COMP3 for overvoltage detection on PA0 (VSENS)
 * INP: PA0, INM: DAC3_OUT1
 */
static inline void MX_COMP3_Init(void) {
    LL_COMP_Disable(COMP3);  // Disable before configuration
    
    LL_COMP_SetInputPlus(COMP3, LL_COMP_INPUT_PLUS_IO1);  // PA0
    LL_COMP_SetInputMinus(COMP3, LL_COMP_INPUT_MINUS_DAC3_CH1);  // DAC3_OUT1
    LL_COMP_SetInputHysteresis(COMP3, LL_COMP_HYSTERESIS_MEDIUM);  // Noise immunity
    LL_COMP_SetOutputPolarity(COMP3, LL_COMP_OUTPUTPOL_INVERTED);  // High when INP > INM
    LL_COMP_SetOutputBlankingSource(COMP3, LL_COMP_BLANKINGSRC_NONE);  // No blanking
    
    LL_COMP_Enable(COMP3);  // Enable COMP3
    
    // Wait for startup
    volatile uint32_t wait = 200;
    while (wait--) { __NOP(); }
}

/**
 * @brief Initialize COMP4 for overcurrent detection on PB0 (I_C)
 * INP: PB0, INM: DAC3_OUT2
 */
static inline void MX_COMP4_Init(void) {
    LL_COMP_Disable(COMP4);  // Disable before configuration
    
    LL_COMP_SetInputPlus(COMP4, LL_COMP_INPUT_PLUS_IO1);  // PB0
    LL_COMP_SetInputMinus(COMP4, LL_COMP_INPUT_MINUS_DAC3_CH2);  // DAC3_OUT2
    LL_COMP_SetInputHysteresis(COMP4, LL_COMP_HYSTERESIS_MEDIUM);  // Noise immunity
    LL_COMP_SetOutputPolarity(COMP4, LL_COMP_OUTPUTPOL_INVERTED);  // High when INP > INM
    LL_COMP_SetOutputBlankingSource(COMP4, LL_COMP_BLANKINGSRC_NONE);  // No blanking
    
    LL_COMP_Enable(COMP4);  // Enable COMP4
    
    // Wait for startup
    volatile uint32_t wait = 200;
    while (wait--) { __NOP(); }
}

/**
 * @brief Configure EXTI interrupts for COMP1, COMP2, COMP3, COMP4
 */
static inline void COMP_EXTI_Init(void) {
    // COMP1 -> EXTI21, COMP2 -> EXTI22, COMP3 -> EXTI29, COMP4 -> EXTI30
    
    EXTI->IMR1 |= EXTI_IMR1_IM21;    // Unmask COMP1 interrupt
    EXTI->RTSR1 |= EXTI_RTSR1_RT21;  // Rising edge trigger
    EXTI->FTSR1 &= ~EXTI_FTSR1_FT21; // Disable falling edge
    
    EXTI->IMR1 |= EXTI_IMR1_IM22;    // Unmask COMP2 interrupt
    EXTI->RTSR1 |= EXTI_RTSR1_RT22;  // Rising edge trigger
    EXTI->FTSR1 &= ~EXTI_FTSR1_FT22; // Disable falling edge
    
    EXTI->IMR1 |= EXTI_IMR1_IM29;    // Unmask COMP3 interrupt
    EXTI->RTSR1 |= EXTI_RTSR1_RT29;  // Rising edge trigger
    EXTI->FTSR1 &= ~EXTI_FTSR1_FT29; // Disable falling edge
    
    EXTI->IMR1 |= EXTI_IMR1_IM30;    // Unmask COMP4 interrupt
    EXTI->RTSR1 |= EXTI_RTSR1_RT30;  // Rising edge trigger
    EXTI->FTSR1 &= ~EXTI_FTSR1_FT30; // Disable falling edge
    
    // Clear pending flags
    EXTI->PR1 = EXTI_PR1_PIF21 | EXTI_PR1_PIF22 | EXTI_PR1_PIF29 | EXTI_PR1_PIF30;
    
    // Enable NVIC interrupts (highest priority)
    NVIC_SetPriority(COMP1_2_3_IRQn, 0);
    NVIC_EnableIRQ(COMP1_2_3_IRQn);
    
    NVIC_SetPriority(COMP4_IRQn, 0);
    NVIC_EnableIRQ(COMP4_IRQn);
}

/**
 * @brief Turn off all LEDs (open-drain, HIGH = OFF)
 */
inline void LED_All_Off(void) {
    LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_4);  // Green OFF
    LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_2);  // Blue OFF
    LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_1);  // Red OFF
}

/**
 * @brief Set red LED only (phase A fault)
 */
inline void LED_Set_Red(void) {
    LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_4);   // Green OFF
    LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_2);   // Blue OFF
    LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_1); // Red ON (LOW)
}

/**
 * @brief Set green LED only (phase B fault)
 */
inline void LED_Set_Green(void) {
    LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_2);   // Blue OFF
    LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_1);   // Red OFF
    LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_4); // Green ON (LOW)
}

/**
 * @brief Set blue LED only (phase C fault)
 */
inline void LED_Set_Blue(void) {
    LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_4);   // Green OFF
    LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_1);   // Red OFF
    LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_2); // Blue ON (LOW)
}

/**
 * @brief Set all LEDs on (voltage fault)
 */
inline void LED_Set_All(void) {
    LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_4); // Green ON (LOW)
    LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_2); // Blue ON (LOW)
    LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_1); // Red ON (LOW)
}

/**
 * @brief Emergency disable PWM with LED indication
 * Called from ISR - must be fast, no SerialUSB calls
 */
inline void PWM_Emergency_Disable(uint8_t fault_source) {
    g_fault_source = fault_source;  // Record fault source
    g_fault_timestamp = millis();   // Record timestamp
    g_fault_active = true;          // Set fault active flag
    
    // Set flag for deferred logging from main loop
    g_fault_needs_log = true;       // Request logging
    g_fault_log_source = fault_source;  // Store fault source for logging
    
    // // Disable DRV8323H driver
    // LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_11);  // DRV_EN = 0
    
    // Set PWM to safe state (all channels off)
    TIM1->CCR1 = 0;  // Channel 1 = 0%
    TIM1->CCR2 = 0;  // Channel 2 = 0%
    TIM1->CCR3 = 0;  // Channel 3 = 0%

    TIM1->CCER &= ~(0x5 << (0 * 4));
    TIM1->CCER &= ~(0x5 << (1 * 4));
    TIM1->CCER &= ~(0x5 << (2 * 4));
    
    // Set LED based on fault source
    if (fault_source & FAULT_COMP3) {
        LED_Set_All();    // Voltage -> All LEDs
    } else if (fault_source & FAULT_COMP2) {
        LED_Set_Red();    // Phase A -> Red LED
    } else if (fault_source & FAULT_COMP1) {
        LED_Set_Green();  // Phase B -> Green LED
    } else if (fault_source & FAULT_COMP4) {
        LED_Set_Blue();   // Phase C -> Blue LED
    }
}

/**
 * @brief Log fault information - call from main loop
 * Rate limited to once per 100ms
 */
inline void COMP_Log_Fault(void) {
    // Rate limiting: only log once per 100ms
    static uint32_t last_log_time = 0;  // Last time we logged fault information
    
    if (!g_fault_needs_log) {  // Check if logging is needed
        return;  // Exit if no logging requested
    }
    
    uint32_t current_time = millis();  // Get current timestamp
    
    // Check if 100ms elapsed since last log
    if (current_time - last_log_time >= 100) {
        last_log_time = current_time;  // Update last log timestamp
        g_fault_needs_log = false;     // Clear logging flag
        
        uint8_t fault_source = g_fault_log_source;  // Get fault source to log
        
        SerialUSB.println("\n=== OVERCURRENT FAULT DETECTED ===");
        
        // Report which comparators triggered
        SerialUSB.print("Triggered comparators: ");
        bool first = true;  // Flag to handle comma separation
        
        if (fault_source & FAULT_COMP3) {  // Check if COMP3 (Voltage) triggered
            if (!first) SerialUSB.print(", ");
            SerialUSB.print("COMP3 (Voltage)");
            first = false;
        }
        
        if (fault_source & FAULT_COMP2) {  // Check if COMP2 (Phase A) triggered
            if (!first) SerialUSB.print(", ");
            SerialUSB.print("COMP2 (Phase A)");
            first = false;
        }
        
        if (fault_source & FAULT_COMP1) {  // Check if COMP1 (Phase B) triggered
            if (!first) SerialUSB.print(", ");
            SerialUSB.print("COMP1 (Phase B)");
            first = false;
        }
        
        if (fault_source & FAULT_COMP4) {  // Check if COMP4 (Phase C) triggered
            if (!first) SerialUSB.print(", ");
            SerialUSB.print("COMP4 (Phase C)");
            first = false;
        }
        SerialUSB.println();
        
        // Report DAC threshold values (12-bit values)
        uint32_t dac1_ch1 = HAL_DAC_GetValue(&hdac1, DAC_CHANNEL_1);  // Get DAC1 Channel 1 value (COMP1, Phase B)
        uint32_t dac1_ch2 = HAL_DAC_GetValue(&hdac1, DAC_CHANNEL_2);  // Get DAC1 Channel 2 value (COMP2, Phase A)
        uint32_t dac3_ch1 = HAL_DAC_GetValue(&hdac3, DAC_CHANNEL_1);  // Get DAC3 Channel 1 value (COMP3, Voltage)
        uint32_t dac3_ch2 = HAL_DAC_GetValue(&hdac3, DAC_CHANNEL_2);  // Get DAC3 Channel 2 value (COMP4, Phase C)
        
        SerialUSB.println("DAC Threshold Values (12-bit):");
        SerialUSB.print("  Voltage (DAC3_CH1): ");
        SerialUSB.print(dac3_ch1);
        SerialUSB.print(" (");
        SerialUSB.print((dac3_ch1 * 3300) / 4095);  // Convert to millivolts (assuming 3.3V reference)
        SerialUSB.println(" mV)");
        
        SerialUSB.print("  Phase A (DAC1_CH2): ");
        SerialUSB.print(dac1_ch2);
        SerialUSB.print(" (");
        SerialUSB.print((dac1_ch2 * 3300) / 4095);  // Convert to millivolts
        SerialUSB.println(" mV)");
        
        SerialUSB.print("  Phase B (DAC1_CH1): ");
        SerialUSB.print(dac1_ch1);
        SerialUSB.print(" (");
        SerialUSB.print((dac1_ch1 * 3300) / 4095);  // Convert to millivolts
        SerialUSB.println(" mV)");
        
        SerialUSB.print("  Phase C (DAC3_CH2): ");
        SerialUSB.print(dac3_ch2);
        SerialUSB.print(" (");
        SerialUSB.print((dac3_ch2 * 3300) / 4095);  // Convert to millivolts
        SerialUSB.println(" mV)");
        
        SerialUSB.println("PWM disabled, driver shutdown");
        SerialUSB.println("==================================\n");
        SerialUSB.flush();  // Ensure all data is sent
    }
}

/**
 * @brief Update fault state - call from main loop
 */
inline void COMP_Update_Fault_State(void) {
    // Handle deferred fault logging
    COMP_Log_Fault();  // Log fault if needed (rate limited)
    
    if (g_fault_active) {
        uint32_t elapsed = millis() - g_fault_timestamp;
        if (elapsed >= FAULT_LOCKOUT_MS) {
            g_fault_active = false;  // Clear fault after timeout
            g_fault_source = FAULT_NONE;
            LED_All_Off();  // Turn off fault LED
        }
    }
}

/**
 * @brief Check if fault lockout is active
 */
inline bool COMP_Is_Fault_Active(void) {
    return g_fault_active;
}

/**
 * @brief Get current fault source
 */
inline uint8_t COMP_Get_Fault_Source(void) {
    return g_fault_source;
}

/**
 * @brief Force clear fault
 */
inline void COMP_Clear_Fault(void) {
    g_fault_active = false;
    g_fault_source = FAULT_NONE;
    LED_All_Off();
}

volatile int32_t COMP_VAL = 0;

// COMP1/2/3 interrupt handler
extern "C" void COMP1_2_3_IRQHandler(void) {
    uint8_t fault = FAULT_NONE;
    
    if (EXTI->PR1 & EXTI_PR1_PIF21) {  // COMP1 (PA1 - I_B)
        EXTI->PR1 = EXTI_PR1_PIF21;
        fault |= FAULT_COMP1;
        // SerialUSB.println("COMP1 ON");
    }
    
    if (EXTI->PR1 & EXTI_PR1_PIF22) {  // COMP2 (PA3 - I_A)
        EXTI->PR1 = EXTI_PR1_PIF22;
        fault |= FAULT_COMP2;
        // SerialUSB.println("COMP2 ON");
    }
    
    if (EXTI->PR1 & EXTI_PR1_PIF29) {  // COMP3 (PA0 - VSENS)
        EXTI->PR1 = EXTI_PR1_PIF29;
        fault |= FAULT_COMP3;
        // SerialUSB.println("COMP3 ON");
    }
    
    if (fault != FAULT_NONE) {
        PWM_Emergency_Disable(fault);
    }
}

// COMP4 interrupt handler
extern "C" void COMP4_IRQHandler(void) {
    if (EXTI->PR1 & EXTI_PR1_PIF30) {  // COMP4 (PB0 - I_C)
        EXTI->PR1 = EXTI_PR1_PIF30;
        // SerialUSB.println("COMP3 ON");
        PWM_Emergency_Disable(FAULT_COMP4);
    }
}

/**
 * @brief Initialize complete overcurrent and overvoltage protection system
 */
inline void COMP_Protection_Init(void) {
    MX_DAC1_Init();    // Initialize DAC1 (COMP1, COMP2)
    MX_DAC3_Init();    // Initialize DAC3 (COMP3, COMP4)
    MX_COMP1_Init();   // Initialize COMP1 on PA1 (Phase B current)
    MX_COMP2_Init();   // Initialize COMP2 on PA3 (Phase A current)
    MX_COMP3_Init();   // Initialize COMP3 on PA0 (Bus voltage)
    MX_COMP4_Init();   // Initialize COMP4 on PB0 (Phase C current)
    COMP_EXTI_Init();  // Enable interrupts
}

#endif  // COMP_SETUP_CLN17V3_H
