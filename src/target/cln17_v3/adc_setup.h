#ifndef ADC_SETUP_CLN17V3_H
#define ADC_SETUP_CLN17V3_H

#include <Arduino.h>
#include "stm32g4xx_ll_adc.h"   // STM32G4 ADC peripheral library
#include "stm32g4xx_ll_bus.h"   // STM32G4 bus peripheral library
#include "stm32g4xx_ll_gpio.h"  // STM32G4 GPIO peripheral library
#include "stm32g4xx_ll_rcc.h"   // STM32G4 RCC peripheral library
#include "dma_setup.h"          // DMA buffer definitions

// Function to initialize ADC1 with DMA
void MX_ADC1_Init() {
    LL_ADC_InitTypeDef ADC_InitStruct = {0};              // ADC initialization structure
    LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {0};      // ADC regular group init
    LL_ADC_CommonInitTypeDef ADC_CommonInitStruct = {0};  // ADC common init

    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};  // Peripheral clock init

    // Configure ADC12 clock source
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC12;
    PeriphClkInit.Adc12ClockSelection = RCC_ADC12CLKSOURCE_SYSCLK;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
        Error_Handler();  // Handle clock configuration error
    }

    // Enable ADC12 peripheral clock
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_ADC12);

    // Configure ADC resolution and data alignment
    ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_12B;      // 12-bit resolution
    ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_LEFT;  // Left-aligned (normalization)
    ADC_InitStruct.LowPowerMode = LL_ADC_LP_MODE_NONE;      // No low power mode
    LL_ADC_Init(ADC1, &ADC_InitStruct);

    // Configure ADC1 regular group (3 channels: I_A, I_B, I_C)
    ADC_REG_InitStruct.SequencerLength = LL_ADC_REG_SEQ_SCAN_ENABLE_3RANKS;  // 3 channels
    ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE;             // Software trigger
    ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;    // No discontinuous
    ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_SINGLE;              // Single conversion
    ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_LIMITED;        // DMA limited mode
    ADC_REG_InitStruct.Overrun = LL_ADC_REG_OVR_DATA_PRESERVED;              // Preserve on overrun
    LL_ADC_REG_Init(ADC1, &ADC_REG_InitStruct);

    LL_ADC_SetGainCompensation(ADC1, 0);                // No gain compensation
    LL_ADC_SetOverSamplingScope(ADC1, LL_ADC_OVS_DISABLE);  // No oversampling

    // Configure dual mode: simultaneous regular conversions
    ADC_CommonInitStruct.CommonClock = LL_ADC_CLOCK_SYNC_PCLK_DIV4;  // Clock divider
    ADC_CommonInitStruct.Multimode = LL_ADC_MULTI_DUAL_REG_SIMULT;   // Dual simultaneous mode
    LL_ADC_CommonInit(__LL_ADC_COMMON_INSTANCE(ADC1), &ADC_CommonInitStruct);

    // Disable deep power down and enable internal regulator
    LL_ADC_DisableDeepPowerDown(ADC1);
    LL_ADC_EnableInternalRegulator(ADC1);
    
    // Wait for regulator stabilization
    uint32_t wait_loop_index = ((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US * (SystemCoreClock / (100000 * 2))) / 10);
    while (wait_loop_index != 0) {
        wait_loop_index--;
    }

    // Configure ADC1 channels
    // Rank 1: PA3 (I_A) -> ADC1_IN4
    LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_4);
    LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_4, LL_ADC_SAMPLINGTIME_2CYCLES_5);
    LL_ADC_SetChannelSingleDiff(ADC1, LL_ADC_CHANNEL_4, LL_ADC_SINGLE_ENDED);

    // Rank 2: PA1 (I_B) -> ADC1_IN2
    LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_2, LL_ADC_CHANNEL_2);
    LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_2, LL_ADC_SAMPLINGTIME_2CYCLES_5);
    LL_ADC_SetChannelSingleDiff(ADC1, LL_ADC_CHANNEL_2, LL_ADC_SINGLE_ENDED);

    // Rank 3: PB0 (I_C) -> ADC1_IN15
    LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_3, LL_ADC_CHANNEL_15);
    LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_15, LL_ADC_SAMPLINGTIME_2CYCLES_5);
    LL_ADC_SetChannelSingleDiff(ADC1, LL_ADC_CHANNEL_15, LL_ADC_SINGLE_ENDED);
}

// Function to start ADC1
void MX_ADC1_Start() {
    // Calibrate ADC1
    LL_ADC_StartCalibration(ADC1, LL_ADC_SINGLE_ENDED);
    while (LL_ADC_IsCalibrationOnGoing(ADC1));  // Wait for calibration
    
    // Enable ADC1
    LL_ADC_Enable(ADC1);
    while (LL_ADC_IsActiveFlag_ADRDY(ADC1) == 0);  // Wait until ready
    
    // Clear end of conversion flag
    if (LL_ADC_IsActiveFlag_EOC(ADC1))
        LL_ADC_ClearFlag_EOC(ADC1);
}

// Function to start DMA conversion for ADC1
void ADC1_StartDMAConversion() {
    LL_ADC_REG_StartConversion(ADC1);  // Start ADC1 conversion
}

// Function to initialize ADC2 with DMA (PA0 VSENS -> ADC2_IN1, PA1 I_B -> ADC2_IN2)
void MX_ADC2_Init() {
    LL_ADC_InitTypeDef ADC_InitStruct = {0};
    LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {0};

    // Enable ADC12 peripheral clock
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_ADC12);

    // Configure ADC2 resolution and data alignment
    ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_12B;
    ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_LEFT;
    ADC_InitStruct.LowPowerMode = LL_ADC_LP_MODE_NONE;
    LL_ADC_Init(ADC2, &ADC_InitStruct);

    // Configure ADC2 regular group (2 channels: VSENS, I_B)
    ADC_REG_InitStruct.SequencerLength = LL_ADC_REG_SEQ_SCAN_ENABLE_2RANKS;  // 2 channels
    ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE;
    ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
    ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_SINGLE;
    ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_LIMITED;
    ADC_REG_InitStruct.Overrun = LL_ADC_REG_OVR_DATA_PRESERVED;
    LL_ADC_REG_Init(ADC2, &ADC_REG_InitStruct);

    LL_ADC_SetGainCompensation(ADC2, 0);
    LL_ADC_SetOverSamplingScope(ADC2, LL_ADC_OVS_DISABLE);

    // Disable deep power down and enable internal regulator
    LL_ADC_DisableDeepPowerDown(ADC2);
    LL_ADC_EnableInternalRegulator(ADC2);
    
    // Wait for regulator stabilization
    uint32_t wait_loop_index = ((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US * (SystemCoreClock / (100000 * 2))) / 10);
    while (wait_loop_index != 0) {
        wait_loop_index--;
    }

    // Configure ADC2 channels
    // Rank 1: PA0 (VSENS) -> ADC2_IN1
    LL_ADC_REG_SetSequencerRanks(ADC2, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_1);
    LL_ADC_SetChannelSamplingTime(ADC2, LL_ADC_CHANNEL_1, LL_ADC_SAMPLINGTIME_2CYCLES_5);
    LL_ADC_SetChannelSingleDiff(ADC2, LL_ADC_CHANNEL_1, LL_ADC_SINGLE_ENDED);

    // Rank 2: PA1 (I_B) -> ADC2_IN2
    LL_ADC_REG_SetSequencerRanks(ADC2, LL_ADC_REG_RANK_2, LL_ADC_CHANNEL_2);
    LL_ADC_SetChannelSamplingTime(ADC2, LL_ADC_CHANNEL_2, LL_ADC_SAMPLINGTIME_2CYCLES_5);
    LL_ADC_SetChannelSingleDiff(ADC2, LL_ADC_CHANNEL_2, LL_ADC_SINGLE_ENDED);
}

// Function to start DMA conversion for ADC2
void ADC2_StartDMAConversion() {
    LL_ADC_REG_StartConversion(ADC2);  // Start ADC2 conversion
}

// Initialize both ADC1 and ADC2
void ADC_Init() {
    MX_ADC1_Init();  // Initialize ADC1
    MX_ADC2_Init();  // Initialize ADC2
    MX_ADC1_Start(); // Start ADC1
    
    // Start ADC2
    LL_ADC_StartCalibration(ADC2, LL_ADC_SINGLE_ENDED);
    while (LL_ADC_IsCalibrationOnGoing(ADC2));
    LL_ADC_Enable(ADC2);
    while (LL_ADC_IsActiveFlag_ADRDY(ADC2) == 0);
    if (LL_ADC_IsActiveFlag_EOC(ADC2))
        LL_ADC_ClearFlag_EOC(ADC2);
}

// Get ADC values from DMA buffers
void ADC_get_values(uint16_t& current_a,
                    uint16_t& current_b,
                    uint16_t& current_c,
                    uint16_t& voltage_supply,
                    uint16_t& voltage_vref,
                    uint16_t& voltage_temp) {
    current_a = dma_buffer_adc1[0];       // I_A (PA3 - ADC1_IN4)
    current_b = dma_buffer_adc1[1];       // I_B (PA1 - ADC1_IN2)
    current_c = dma_buffer_adc1[2];       // I_C (PB0 - ADC1_IN15)
    voltage_supply = dma_buffer_adc2[0];  // VSENS (PA0 - ADC2_IN1)
    voltage_vref = 0;                     // Not used
    voltage_temp = 0;                     // Not used
}

#endif  // ADC_SETUP_CLN17V3_H
