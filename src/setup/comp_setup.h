#ifndef COMP_SETUP_H
#define COMP_SETUP_H

#include <Arduino.h>
#include "stm32g4xx_hal.h"
#include "stm32g4xx_ll_comp.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_gpio.h"

// Overcurrent threshold: 2048 + 512 = 2560 (DAC3_OUT2 value)
// This corresponds to ~2.06V at 3.3V reference
constexpr uint32_t OVERCURRENT_DAC_VALUE = 3500U;

// Fault lockout duration in milliseconds
constexpr uint32_t FAULT_LOCKOUT_MS = 1000U;

// Fault source flags (bitmask)
constexpr uint8_t FAULT_NONE = 0x00;   // No fault
constexpr uint8_t FAULT_COMP2 = 0x01;  // COMP2 triggered (PA3)
constexpr uint8_t FAULT_COMP4 = 0x02;  // COMP4 triggered (PB0)

// Fault state variables
volatile uint8_t g_fault_source = FAULT_NONE;  // Which comparator triggered
volatile uint32_t g_fault_timestamp = 0;       // When fault occurred (millis)
volatile bool g_fault_active = false;          // Fault lockout active

// Forward declarations
void PWM_Emergency_Disable(uint8_t fault_source);
void COMP_Update_Fault_State(void);

/**
 * @brief Initialize DAC3 Channel 2 for comparator reference voltage
 * DAC3 is internal-only (no external pin), used as reference for COMP2 and COMP4
 */
static inline void MX_DAC3_Init(void) {
    // Enable DAC3 clock
    __HAL_RCC_DAC3_CLK_ENABLE();
    
    // DAC3 initialization - internal DAC, no external output
    DAC_HandleTypeDef hdac3;
    hdac3.Instance = DAC3;
    
    if (HAL_DAC_Init(&hdac3) != HAL_OK) {
        Error_Handler();
    }
    
    // Configure DAC3 Channel 2
    DAC_ChannelConfTypeDef sConfig = {0};
    sConfig.DAC_Trigger = DAC_TRIGGER_NONE;           // Software trigger (immediate)
    sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_DISABLE; // Internal connection only
    sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_INTERNAL; // Connect to COMP
    sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
    
    if (HAL_DAC_ConfigChannel(&hdac3, &sConfig, DAC_CHANNEL_2) != HAL_OK) {
        Error_Handler();
    }
    
    // Start DAC3 Channel 2
    if (HAL_DAC_Start(&hdac3, DAC_CHANNEL_2) != HAL_OK) {
        Error_Handler();
    }
    
    // Set threshold value: 2048 + 512 = 2560
    HAL_DAC_SetValue(&hdac3, DAC_CHANNEL_2, DAC_ALIGN_12B_R, OVERCURRENT_DAC_VALUE);
}

/**
 * @brief Initialize COMP2 for overcurrent detection on PA3 (I_CH1)
 * INP: PA3 (current sense channel 1)
 * INM: DAC3_OUT2 (threshold reference)
 */
static inline void MX_COMP2_Init(void) {
    // Enable COMP clock (SYSCFG contains COMP)
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    
    // PA3 already configured as analog in gpio_setup.h
    
    // Configure COMP2 using LL driver for precise control
    // COMP2_INP: PA3 (INP_IO1)
    // COMP2_INM: DAC3_OUT2
    
    // Disable COMP2 before configuration
    LL_COMP_Disable(COMP2);
    
    // Set input plus to PA3 (IO1)
    LL_COMP_SetInputPlus(COMP2, LL_COMP_INPUT_PLUS_IO1);
    
    // Set input minus to DAC3 channel 2
    LL_COMP_SetInputMinus(COMP2, LL_COMP_INPUT_MINUS_DAC3_CH2);
    
    // Set hysteresis for noise immunity
    LL_COMP_SetInputHysteresis(COMP2, LL_COMP_HYSTERESIS_MEDIUM);
    
    // Output polarity: non-inverted (high when INP > INM)
    LL_COMP_SetOutputPolarity(COMP2, LL_COMP_OUTPUTPOL_NONINVERTED);
    
    // Blanking source: none
    LL_COMP_SetOutputBlankingSource(COMP2, LL_COMP_BLANKINGSRC_NONE);
    
    // Enable COMP2
    LL_COMP_Enable(COMP2);
    
    // Wait for COMP2 startup time (~10us)
    volatile uint32_t wait = 200;
    while (wait--) { __NOP(); }
}

/**
 * @brief Initialize COMP4 for overcurrent detection on PB0 (I_CH4)
 * INP: PB0 (current sense channel 4)
 * INM: DAC3_OUT2 (threshold reference)
 */
static inline void MX_COMP4_Init(void) {
    // PB0 already configured as analog in gpio_setup.h
    
    // Disable COMP4 before configuration
    LL_COMP_Disable(COMP4);
    
    // Set input plus to PB0 (IO1)
    LL_COMP_SetInputPlus(COMP4, LL_COMP_INPUT_PLUS_IO1);
    
    // Set input minus to DAC3 channel 2
    LL_COMP_SetInputMinus(COMP4, LL_COMP_INPUT_MINUS_DAC3_CH2);
    
    // Set hysteresis for noise immunity
    LL_COMP_SetInputHysteresis(COMP4, LL_COMP_HYSTERESIS_MEDIUM);
    
    // Output polarity: non-inverted (high when INP > INM)
    LL_COMP_SetOutputPolarity(COMP4, LL_COMP_OUTPUTPOL_NONINVERTED);
    
    // Blanking source: none
    LL_COMP_SetOutputBlankingSource(COMP4, LL_COMP_BLANKINGSRC_NONE);
    
    // Enable COMP4
    LL_COMP_Enable(COMP4);
    
    // Wait for COMP4 startup time (~10us)
    volatile uint32_t wait = 200;
    while (wait--) { __NOP(); }
}

/**
 * @brief Configure EXTI interrupts for COMP2 and COMP4
 * Rising edge triggers overcurrent protection
 */
static inline void COMP_EXTI_Init(void) {
    // COMP2 is connected to EXTI22
    // COMP4 is connected to EXTI30
    
    // Enable EXTI line 22 (COMP2) - rising edge
    EXTI->IMR1 |= EXTI_IMR1_IM22;    // Unmask interrupt
    EXTI->RTSR1 |= EXTI_RTSR1_RT22;  // Rising edge trigger
    EXTI->FTSR1 &= ~EXTI_FTSR1_FT22; // Disable falling edge
    
    // Enable EXTI line 30 (COMP4) - rising edge
    EXTI->IMR1 |= EXTI_IMR1_IM30;    // Unmask interrupt
    EXTI->RTSR1 |= EXTI_RTSR1_RT30;  // Rising edge trigger
    EXTI->FTSR1 &= ~EXTI_FTSR1_FT30; // Disable falling edge
    
    // Clear any pending flags
    EXTI->PR1 = EXTI_PR1_PIF22 | EXTI_PR1_PIF30;
    
    // Enable NVIC interrupts
    // COMP2 uses COMP1_2_3_IRQn (shared)
    // COMP4 uses COMP4_IRQn
    NVIC_SetPriority(COMP1_2_3_IRQn, 0);  // Highest priority
    NVIC_EnableIRQ(COMP1_2_3_IRQn);
    
    NVIC_SetPriority(COMP4_IRQn, 0);      // Highest priority
    NVIC_EnableIRQ(COMP4_IRQn);
}

/**
 * @brief Turn off all LEDs (open-drain, common +, LOW = ON)
 */
inline void LED_All_Off(void) {
    LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_13);  // Green OFF (HIGH)
    LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_14);  // Blue OFF (HIGH)
    LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_15);  // Red OFF (HIGH)
}

/**
 * @brief Set green LED only (COMP2 fault indicator)
 */
inline void LED_Set_Green(void) {
    LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_14);   // Blue OFF
    LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_15);   // Red OFF
    LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_13); // Green ON (LOW)
}

/**
 * @brief Set blue LED only (COMP4 fault indicator)
 */
inline void LED_Set_Blue(void) {
    LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_13);   // Green OFF
    LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_15);   // Red OFF
    LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_14); // Blue ON (LOW)
}

/**
 * @brief Emergency disable all PWM outputs (Hi-Z state) with LED indication
 * @param fault_source Which comparator triggered (FAULT_COMP2 or FAULT_COMP4)
 */
inline void PWM_Emergency_Disable(uint8_t fault_source) {
    // Record fault source and timestamp
    g_fault_source = fault_source;
    g_fault_timestamp = millis();
    g_fault_active = true;
    
    // // Disable all EN_CH pins (set to LOW for Hi-Z)
    // LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_10); // EN_CH1 = 0
    // LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_2);  // EN_CH2 = 0
    // LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_1);  // EN_CH3 = 0
    // LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_6);  // EN_CH4 = 0
    
    // // Set PWM to 100% duty (outputs stay low when EN is disabled)
    // TIM2->CCR1 = TIM2->ARR;  // Channel 1 = 100%
    // TIM2->CCR3 = TIM2->ARR;  // Channel 3 = 100%
    // TIM2->CCR4 = TIM2->ARR;  // Channel 4 = 100%
    
    // Set LED based on fault source (only one LED at a time)
    if (fault_source == FAULT_COMP2) {
        LED_Set_Green();  // COMP2 (PA3) -> Green LED
    } else if (fault_source == FAULT_COMP4) {
        LED_Set_Blue();   // COMP4 (PB0) -> Blue LED
    }
}

/**
 * @brief Update fault state - call from main loop to handle timeout
 * Clears fault after FAULT_LOCKOUT_MS milliseconds
 */
inline void COMP_Update_Fault_State(void) {
    if (g_fault_active) {
        uint32_t elapsed = millis() - g_fault_timestamp;
        if (elapsed >= FAULT_LOCKOUT_MS) {
            // Lockout period expired - clear fault
            g_fault_active = false;
            g_fault_source = FAULT_NONE;
            LED_All_Off();  // Turn off fault LED
        }
    }
}

/**
 * @brief Check if fault lockout is active
 * @return true if fault occurred and lockout period not expired
 */
inline bool COMP_Is_Fault_Active(void) {
    return g_fault_active;
}

/**
 * @brief Get current fault source
 * @return FAULT_NONE, FAULT_COMP2, or FAULT_COMP4
 */
inline uint8_t COMP_Get_Fault_Source(void) {
    return g_fault_source;
}

/**
 * @brief Force clear fault (manual reset)
 */
inline void COMP_Clear_Fault(void) {
    g_fault_active = false;
    g_fault_source = FAULT_NONE;
    LED_All_Off();
}

/**
 * @brief Check current comparator output states
 * @return Bitmask: bit0 = COMP2, bit1 = COMP4
 */
inline uint8_t COMP_Get_Output_State(void) {
    uint8_t state = 0;
    if (LL_COMP_ReadOutputLevel(COMP2) == LL_COMP_OUTPUT_LEVEL_HIGH) {
        state |= 0x01;
    }
    if (LL_COMP_ReadOutputLevel(COMP4) == LL_COMP_OUTPUT_LEVEL_HIGH) {
        state |= 0x02;
    }
    return state;
}

// COMP1/2/3 interrupt handler (COMP2 uses this)
extern "C" void COMP1_2_3_IRQHandler(void) {
    // Check if COMP2 triggered (EXTI line 22)
    if (EXTI->PR1 & EXTI_PR1_PIF22) {
        EXTI->PR1 = EXTI_PR1_PIF22;       // Clear pending flag
        PWM_Emergency_Disable(FAULT_COMP2); // Disable PWM, green LED
    }
}

// COMP4 interrupt handler
extern "C" void COMP4_IRQHandler(void) {
    // Check if COMP4 triggered (EXTI line 30)
    if (EXTI->PR1 & EXTI_PR1_PIF30) {
        EXTI->PR1 = EXTI_PR1_PIF30;       // Clear pending flag
        PWM_Emergency_Disable(FAULT_COMP4); // Disable PWM, blue LED
    }
}

/**
 * @brief Initialize complete overcurrent protection system
 * DAC3_OUT2 as threshold, COMP2 (PA3) and COMP4 (PB0) as detectors
 */
inline void COMP_Protection_Init(void) {
    MX_DAC3_Init();    // Initialize DAC3 with threshold value
    MX_COMP2_Init();   // Initialize COMP2 on PA3
    MX_COMP4_Init();   // Initialize COMP4 on PB0
    COMP_EXTI_Init();  // Enable interrupts
}

#endif  // COMP_SETUP_H
