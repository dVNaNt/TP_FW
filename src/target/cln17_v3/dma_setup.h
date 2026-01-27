#ifndef DMA_SETUP_CLN17V3_H
#define DMA_SETUP_CLN17V3_H

#include <Arduino.h>
#include "stm32g4xx_hal.h"
#include "stm32g4xx_ll_dma.h"

extern void ADC_READY_Callback();  // Forward declaration

// DMA buffers for ADC1 and ADC2
volatile uint16_t dma_buffer_adc1[3] = {0};  // I_A, I_B, I_C
volatile uint16_t dma_buffer_adc2[2] = {0};  // VSENS, I_B

DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;

/**
 * @brief Initialize DMA for ADC1
 */
void MX_DMA_ADC1_Init(void) {
    __HAL_RCC_DMAMUX1_CLK_ENABLE();  // Enable DMAMUX clock
    __HAL_RCC_DMA1_CLK_ENABLE();     // Enable DMA1 clock

    // Configure DMA for ADC1 (DMA1 Channel 5)
    hdma_adc1.Instance = DMA1_Channel5;
    hdma_adc1.Init.Request = DMA_REQUEST_ADC1;
    hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc1.Init.Mode = DMA_CIRCULAR;
    hdma_adc1.Init.Priority = DMA_PRIORITY_HIGH;
    
    if (HAL_DMA_Init(&hdma_adc1) != HAL_OK) {
        Error_Handler();
    }

    // Enable DMA interrupt
    HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

    // Start DMA transfer
    HAL_DMA_Start_IT(&hdma_adc1, 
                     (uint32_t)&ADC1->DR, 
                     (uint32_t)dma_buffer_adc1, 
                     3);  // 3 channels
}

/**
 * @brief Initialize DMA for ADC2
 */
void MX_DMA_ADC2_Init(void) {
    // Configure DMA for ADC2 (DMA1 Channel 6)
    hdma_adc2.Instance = DMA1_Channel6;
    hdma_adc2.Init.Request = DMA_REQUEST_ADC2;
    hdma_adc2.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc2.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc2.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc2.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc2.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc2.Init.Mode = DMA_CIRCULAR;
    hdma_adc2.Init.Priority = DMA_PRIORITY_HIGH;
    
    if (HAL_DMA_Init(&hdma_adc2) != HAL_OK) {
        Error_Handler();
    }

    // Enable DMA interrupt
    HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);

    // Start DMA transfer
    HAL_DMA_Start_IT(&hdma_adc2, 
                     (uint32_t)&ADC2->DR, 
                     (uint32_t)dma_buffer_adc2, 
                     2);  // 2 channels
}

/**
 * @brief DMA1 Channel 5 IRQ handler (ADC1)
 */
extern "C" void DMA1_Channel5_IRQHandler(void) {
    HAL_DMA_IRQHandler(&hdma_adc1);
    ADC_READY_Callback();  // Call ADC ready callback
}

/**
 * @brief DMA1 Channel 6 IRQ handler (ADC2)
 */
extern "C" void DMA1_Channel6_IRQHandler(void) {
    HAL_DMA_IRQHandler(&hdma_adc2);
}

/**
 * @brief Initialize all DMA channels
 */
void DMA_Init(void) {
    MX_DMA_ADC1_Init();  // Initialize DMA for ADC1
    MX_DMA_ADC2_Init();  // Initialize DMA for ADC2
}

#endif  // DMA_SETUP_CLN17V3_H
