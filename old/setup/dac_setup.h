#ifndef DAC_SETUP_H
#define DAC_SETUP_H

#include <Arduino.h>
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_dac.h"

// Overcurrent threshold value for comparators (12-bit DAC)
// 3700 corresponds to ~3.0V at 3.3V reference
constexpr uint32_t OVERCURRENT_DAC_VALUE = 3700U;

// Overvoltage threshold value for COMP3 (12-bit DAC)
// 2048 corresponds to ~1.65V at 3.3V reference
constexpr uint32_t OVERVOLTAGE_DAC_VALUE = 2048U;

// DAC handles for all three DAC instances
static DAC_HandleTypeDef hdac1; // DAC1: CH1 (PA_1/I_B), CH2 (PA_3/I_A)
static DAC_HandleTypeDef hdac3; // DAC3: CH1 (PA_0/VSENS), CH2 (PB_0/I_C)

/**
 * @brief Initialize DAC1 for current sense comparator thresholds
 * DAC1_CH1 (internal) -> COMP1 INM (I_B threshold on PA_1)
 * DAC1_CH2 (internal) -> COMP2 INM (I_A threshold on PA_3)
 */
static inline void MX_DAC1_Init(void) {
    // Enable DAC1 clock
    __HAL_RCC_DAC1_CLK_ENABLE();
    
    // Initialize DAC1 instance
    hdac1.Instance = DAC1;
    if (HAL_DAC_Init(&hdac1) != HAL_OK) {
        Error_Handler();
    }
    
    // Configure DAC1 Channel 1 (for COMP1 - I_B on PA_1)
    DAC_ChannelConfTypeDef sConfig = {0};
    sConfig.DAC_Trigger = DAC_TRIGGER_NONE;                      // Software trigger (immediate update)
    sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_DISABLE;         // Internal connection only
    sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_INTERNAL; // Connect to comparator
    sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;             // Use factory calibration
    
    if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }
    
    // Configure DAC1 Channel 2 (for COMP2 - I_A on PA_3)
    if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK) {
        Error_Handler();
    }
    
    // Start both DAC1 channels
    if (HAL_DAC_Start(&hdac1, DAC_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_DAC_Start(&hdac1, DAC_CHANNEL_2) != HAL_OK) {
        Error_Handler();
    }
    
    // Set overcurrent threshold values for both channels
    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, OVERCURRENT_DAC_VALUE);
    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, OVERCURRENT_DAC_VALUE);
}

/**
 * @brief Initialize DAC3 for current sense and voltage sense comparator thresholds
 * DAC3_CH1 (internal) -> COMP3 INM (VSENS threshold on PA_0)
 * DAC3_CH2 (internal) -> COMP4 INM (I_C threshold on PB_0)
 */
static inline void MX_DAC3_Init(void) {
    // Enable DAC3 clock
    __HAL_RCC_DAC3_CLK_ENABLE();
    
    // Initialize DAC3 instance
    hdac3.Instance = DAC3;
    if (HAL_DAC_Init(&hdac3) != HAL_OK) {
        Error_Handler();
    }
    
    // Configure DAC3 Channel 1 (for COMP3 - VSENS on PA_0)
    DAC_ChannelConfTypeDef sConfig = {0};
    sConfig.DAC_Trigger = DAC_TRIGGER_NONE;                      // Software trigger (immediate update)
    sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_DISABLE;         // Internal connection only
    sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_INTERNAL; // Connect to comparator
    sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;             // Use factory calibration
    
    if (HAL_DAC_ConfigChannel(&hdac3, &sConfig, DAC_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }
    
    // Configure DAC3 Channel 2 (for COMP4 - I_C on PB_0)
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
    HAL_DAC_SetValue(&hdac3, DAC_CHANNEL_1, DAC_ALIGN_12B_R, OVERVOLTAGE_DAC_VALUE);  // VSENS threshold
    HAL_DAC_SetValue(&hdac3, DAC_CHANNEL_2, DAC_ALIGN_12B_R, OVERCURRENT_DAC_VALUE);  // I_C threshold
}

/**
 * @brief Initialize all DAC channels for comparator thresholds
 * Call this function during system initialization
 */
static inline void DAC_Init(void) {
    MX_DAC1_Init(); // Initialize DAC1 (CH1 for I_B, CH2 for I_A)
    MX_DAC3_Init(); // Initialize DAC3 (CH1 for VSENS, CH2 for I_C)
}

/**
 * @brief Update overcurrent threshold for all current sense comparators
 * @param dac_value 12-bit DAC value (0-4095)
 */
static inline void DAC_Set_Overcurrent_Threshold(uint32_t dac_value) {
    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dac_value); // I_B (COMP1)
    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, dac_value); // I_A (COMP2)
    HAL_DAC_SetValue(&hdac3, DAC_CHANNEL_2, DAC_ALIGN_12B_R, dac_value); // I_C (COMP4)
}

/**
 * @brief Update overvoltage threshold for voltage sense comparator
 * @param dac_value 12-bit DAC value (0-4095)
 */
static inline void DAC_Set_Overvoltage_Threshold(uint32_t dac_value) {
    HAL_DAC_SetValue(&hdac3, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dac_value); // VSENS (COMP3)
}

#endif  // DAC_SETUP_H

