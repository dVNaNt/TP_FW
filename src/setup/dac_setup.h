#ifndef DAC_SETUP_H
#define DAC_SETUP_H

#include <Arduino.h>
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_dac.h"

// Simple DAC helper to drive IVREF at 50% on DAC1_CH2 (PA5)
static DAC_HandleTypeDef hdac1;

static inline void MX_DAC1_Init(void) {
    __HAL_RCC_DAC1_CLK_ENABLE();

    hdac1.Instance = DAC1;
    if (HAL_DAC_Init(&hdac1) != HAL_OK) {
        Error_Handler();
    }

    DAC_ChannelConfTypeDef sConfig = {0};
    sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
    sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
    if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK) {
        Error_Handler();
    }
}

static inline void DAC_Set_IVREF_50(void) {
    // 12-bit right aligned -> 50% of full-scale
    const uint32_t mid = 2048U;
    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, mid);
}

static inline void DAC_Init(void) {
    MX_DAC1_Init();
    HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);
    DAC_Set_IVREF_50();
}

#endif  // DAC_SETUP_H

