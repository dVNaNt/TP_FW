#ifndef COMP_SETUP_CLN17V3_H
#define COMP_SETUP_CLN17V3_H

#include <Arduino.h>
#include "stm32g4xx_hal.h"
#include "stm32g4xx_ll_comp.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_gpio.h"

// Overcurrent threshold DAC value
constexpr uint32_t OVERCURRENT_DAC_VALUE = 3700U;

// Fault lockout duration in milliseconds
constexpr uint32_t FAULT_LOCKOUT_MS = 1000U;

// Fault source flags (bitmask)
constexpr uint8_t FAULT_NONE = 0x00;   // No fault
constexpr uint8_t FAULT_COMP1 = 0x01;  // COMP1 triggered (PA1 - I_B)
constexpr uint8_t FAULT_COMP2 = 0x02;  // COMP2 triggered (PA3 - I_A)
constexpr uint8_t FAULT_COMP4 = 0x04;  // COMP4 triggered (PB0 - I_C)

// Fault state variables
volatile uint8_t g_fault_source = FAULT_NONE;  // Which comparator triggered
volatile uint32_t g_fault_timestamp = 0;       // When fault occurred
volatile bool g_fault_active = false;          // Fault lockout active

// Forward declarations
void PWM_Emergency_Disable(uint8_t fault_source);
void COMP_Update_Fault_State(void);

/**
 * @brief Initialize DAC1 for comparator reference voltages
 * DAC1_OUT1 (internal) for COMP1, DAC1_OUT2 (internal) for COMP2
 */
static inline void MX_DAC1_Init(void) {
    __HAL_RCC_DAC1_CLK_ENABLE();  // Enable DAC1 clock
    
    DAC_HandleTypeDef hdac1;
    hdac1.Instance = DAC1;
    
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
 * @brief Initialize DAC3 Channel 2 for COMP4 reference
 */
static inline void MX_DAC3_Init(void) {
    __HAL_RCC_DAC3_CLK_ENABLE();  // Enable DAC3 clock
    
    DAC_HandleTypeDef hdac3;
    hdac3.Instance = DAC3;
    
    if (HAL_DAC_Init(&hdac3) != HAL_OK) {
        Error_Handler();
    }
    
    // Configure DAC3 Channel 2 (for COMP4)
    DAC_ChannelConfTypeDef sConfig = {0};
    sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
    sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_DISABLE;
    sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_INTERNAL;
    sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
    
    if (HAL_DAC_ConfigChannel(&hdac3, &sConfig, DAC_CHANNEL_2) != HAL_OK) {
        Error_Handler();
    }
    
    // Start DAC3 Channel 2
    if (HAL_DAC_Start(&hdac3, DAC_CHANNEL_2) != HAL_OK) {
        Error_Handler();
    }
    
    // Set threshold value
    HAL_DAC_SetValue(&hdac3, DAC_CHANNEL_2, DAC_ALIGN_12B_R, OVERCURRENT_DAC_VALUE);
}

/**
 * @brief Initialize COMP1 for overcurrent detection on PA1 (I_B)
 * INP: PA1, INM: DAC1_OUT1
 */
static inline void MX_COMP1_Init(void) {
    __HAL_RCC_SYSCFG_CLK_ENABLE();  // Enable COMP clock
    
    LL_COMP_Disable(COMP1);  // Disable before configuration
    
    LL_COMP_SetInputPlus(COMP1, LL_COMP_INPUT_PLUS_IO2);  // PA1
    LL_COMP_SetInputMinus(COMP1, LL_COMP_INPUT_MINUS_DAC1_CH1);  // DAC1_OUT1
    LL_COMP_SetInputHysteresis(COMP1, LL_COMP_HYSTERESIS_MEDIUM);  // Noise immunity
    LL_COMP_SetOutputPolarity(COMP1, LL_COMP_OUTPUTPOL_NONINVERTED);  // High when INP > INM
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
    
    LL_COMP_SetInputPlus(COMP2, LL_COMP_INPUT_PLUS_IO1);  // PA3
    LL_COMP_SetInputMinus(COMP2, LL_COMP_INPUT_MINUS_DAC1_CH2);  // DAC1_OUT2
    LL_COMP_SetInputHysteresis(COMP2, LL_COMP_HYSTERESIS_MEDIUM);  // Noise immunity
    LL_COMP_SetOutputPolarity(COMP2, LL_COMP_OUTPUTPOL_NONINVERTED);  // High when INP > INM
    LL_COMP_SetOutputBlankingSource(COMP2, LL_COMP_BLANKINGSRC_NONE);  // No blanking
    
    LL_COMP_Enable(COMP2);  // Enable COMP2
    
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
    LL_COMP_SetOutputPolarity(COMP4, LL_COMP_OUTPUTPOL_NONINVERTED);  // High when INP > INM
    LL_COMP_SetOutputBlankingSource(COMP4, LL_COMP_BLANKINGSRC_NONE);  // No blanking
    
    LL_COMP_Enable(COMP4);  // Enable COMP4
    
    // Wait for startup
    volatile uint32_t wait = 200;
    while (wait--) { __NOP(); }
}

/**
 * @brief Configure EXTI interrupts for COMP1, COMP2, COMP4
 */
static inline void COMP_EXTI_Init(void) {
    // COMP1 -> EXTI21, COMP2 -> EXTI22, COMP4 -> EXTI30
    
    EXTI->IMR1 |= EXTI_IMR1_IM21;    // Unmask COMP1 interrupt
    EXTI->RTSR1 |= EXTI_RTSR1_RT21;  // Rising edge trigger
    EXTI->FTSR1 &= ~EXTI_FTSR1_FT21; // Disable falling edge
    
    EXTI->IMR1 |= EXTI_IMR1_IM22;    // Unmask COMP2 interrupt
    EXTI->RTSR1 |= EXTI_RTSR1_RT22;  // Rising edge trigger
    EXTI->FTSR1 &= ~EXTI_FTSR1_FT22; // Disable falling edge
    
    EXTI->IMR1 |= EXTI_IMR1_IM30;    // Unmask COMP4 interrupt
    EXTI->RTSR1 |= EXTI_RTSR1_RT30;  // Rising edge trigger
    EXTI->FTSR1 &= ~EXTI_FTSR1_FT30; // Disable falling edge
    
    // Clear pending flags
    EXTI->PR1 = EXTI_PR1_PIF21 | EXTI_PR1_PIF22 | EXTI_PR1_PIF30;
    
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
 * @brief Emergency disable PWM with LED indication
 */
inline void PWM_Emergency_Disable(uint8_t fault_source) {
    g_fault_source = fault_source;  // Record fault source
    g_fault_timestamp = millis();   // Record timestamp
    g_fault_active = true;          // Set fault active flag
    
    // TODO: Log fault to error manager when integrated
    // For now, just disable driver and PWM
    
    // Disable DRV8323H driver
    LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_11);  // DRV_EN = 0
    
    // Set PWM to safe state (all channels off)
    TIM1->CCR1 = 0;  // Channel 1 = 0%
    TIM1->CCR2 = 0;  // Channel 2 = 0%
    TIM1->CCR3 = 0;  // Channel 3 = 0%
    
    // Set LED based on fault source
    if (fault_source & FAULT_COMP2) {
        LED_Set_Red();    // Phase A -> Red LED
    } else if (fault_source & FAULT_COMP1) {
        LED_Set_Green();  // Phase B -> Green LED
    } else if (fault_source & FAULT_COMP4) {
        LED_Set_Blue();   // Phase C -> Blue LED
    }
}

/**
 * @brief Update fault state - call from main loop
 */
inline void COMP_Update_Fault_State(void) {
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

// COMP1/2/3 interrupt handler
extern "C" void COMP1_2_3_IRQHandler(void) {
    uint8_t fault = FAULT_NONE;
    
    if (EXTI->PR1 & EXTI_PR1_PIF21) {  // COMP1 (PA1 - I_B)
        EXTI->PR1 = EXTI_PR1_PIF21;
        fault |= FAULT_COMP1;
    }
    
    if (EXTI->PR1 & EXTI_PR1_PIF22) {  // COMP2 (PA3 - I_A)
        EXTI->PR1 = EXTI_PR1_PIF22;
        fault |= FAULT_COMP2;
    }
    
    if (fault != FAULT_NONE) {
        // PWM_Emergency_Disable(fault);
    }
}

// COMP4 interrupt handler
extern "C" void COMP4_IRQHandler(void) {
    if (EXTI->PR1 & EXTI_PR1_PIF30) {  // COMP4 (PB0 - I_C)
        EXTI->PR1 = EXTI_PR1_PIF30;
        // PWM_Emergency_Disable(FAULT_COMP4);
    }
}

/**
 * @brief Initialize complete overcurrent protection system
 */
inline void COMP_Protection_Init(void) {
    MX_DAC1_Init();    // Initialize DAC1 (COMP1, COMP2)
    MX_DAC3_Init();    // Initialize DAC3 (COMP4)
    MX_COMP1_Init();   // Initialize COMP1 on PA1
    MX_COMP2_Init();   // Initialize COMP2 on PA3
    MX_COMP4_Init();   // Initialize COMP4 on PB0
    COMP_EXTI_Init();  // Enable interrupts
}

#endif  // COMP_SETUP_CLN17V3_H
