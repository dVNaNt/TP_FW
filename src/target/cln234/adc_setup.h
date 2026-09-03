/**
 * @file adc_setup.h
 * @brief Dual-ADC (regular simultaneous) sampling backend for CLN234.
 *
 * Rank layout (DMA1_Channel5 reads ADC12_COMMON->CDR, 3 words):
 *   Word 0 : ADC1 rank1 = I_A (PA3/ADC1_IN4)  | ADC2 rank1 = I_B (PA1/ADC2_IN2)
 *   Word 1 : ADC1 rank2 = I_C (PB0/ADC1_IN15) | ADC2 rank2 = VSENS (PA0/ADC2_IN1)
 *   Word 2 : ADC1 rank3 = internal temp (ch16)| ADC2 rank3 = TEMP (PC4/ADC2_IN5)
 *
 * ENC_CAL (PA2/ADC2_IN3) is left in analog mode for future use.
 *
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#ifndef ADC_DMA_SETUP_CLN234_H
#define ADC_DMA_SETUP_CLN234_H

#include <Arduino.h>          // Include standard Arduino framework types
#include "stm32g4xx_hal.h"    // Include HAL required for DMA and NVIC setup
#include "stm32g4xx_ll_adc.h" // Include LL ADC (Kept for compatibility with your existing DMA callback)
#include "stm32g4xx_ll_bus.h" // Include LL Bus definitions
#include "stm32g4xx_ll_gpio.h" // Include LL GPIO definitions
#include "stm32g4xx_ll_rcc.h" // Include LL RCC definitions
#include "stm32g4xx_ll_dma.h" // Include LL DMA definitions

extern void ADC_READY_Callback(); // Forward declaration for the FOC execution callback

// OPTIMIZATION: 4-byte memory alignment to prevent Cortex-M4 cache and bus fetch delays
// Three words: one per rank pair of the dual regular simultaneous sequence.
// Word 3 = internal temperature (ADC1) | external TEMP sensor (ADC2, PC4).
alignas(4) inline volatile uint32_t dma_buffer_adc12[3] = {0}; // Triple-rank dual ADC DMA buffer

inline DMA_HandleTypeDef hdma_adc1; // Global HAL handle for DMA structure

/**
 * @brief Initialize DMA for ADC12 (Dual Mode Master)
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */
inline void MX_DMA_ADC1_Init(void) { // Initialize the DMA mechanism
    __HAL_RCC_DMAMUX1_CLK_ENABLE(); // Enable the hardware clock for the DMA Multiplexer
    __HAL_RCC_DMA1_CLK_ENABLE(); // Enable the hardware clock for DMA1

    hdma_adc1.Instance = DMA1_Channel5; // Assign DMA instance to Channel 5
    hdma_adc1.Init.Request = DMA_REQUEST_ADC1; // Route the hardware request trigger from ADC1
    hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY; // Set transfer direction from ADC to RAM
    hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE; // Keep peripheral address fixed at the Common Data Register
    hdma_adc1.Init.MemInc = DMA_MINC_ENABLE; // Increment memory address to fill the array elements

    // OPTIMIZATION: Read exactly a 32-bit word in one go (CDR contains Master+Slave combined)
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD; // Read 32 bits from peripheral
    hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_WORD; // Write 32 bits to memory
    hdma_adc1.Init.Mode = DMA_CIRCULAR; // Automatically wrap around to the start of the buffer
    hdma_adc1.Init.Priority = DMA_PRIORITY_VERY_HIGH; // Highest bus priority to prevent FOC starvation

    if (HAL_DMA_Init(&hdma_adc1) != HAL_OK) { // Execute HAL initialization
        Error_Handler(); // Trigger global error state if DMA init fails
    } // End DMA init check

    HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 1, 0); // Set very high interrupt priority for FOC latency
    HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn); // Enable DMA Channel 5 interrupt in the NVIC

    // START DMA: Reading continuously from the COMMON Data Register (CDR)
    HAL_DMA_Start(&hdma_adc1, (uint32_t)&ADC12_COMMON->CDR, (uint32_t)dma_buffer_adc12, 3); // Map CDR to array (one word per rank pair)

    __HAL_DMA_ENABLE_IT(&hdma_adc1, DMA_IT_TC | DMA_IT_TE); // Enable Transfer Complete and Transfer Error interrupts
} // End of MX_DMA_ADC1_Init function

/**
 * @brief DMA1 Channel 5 IRQ handler
 */
extern "C" void DMA1_Channel5_IRQHandler(void) { // Interrupt vector for DMA Channel 5
    if (LL_DMA_IsActiveFlag_TC5(DMA1)) { // Check if the Transfer Complete flag is triggered
        // OPTIMIZATION: Clear global flag (TC, TE, HT) in exactly 1 hardware clock cycle
        LL_DMA_ClearFlag_GI5(DMA1); // Apply single-cycle flag clearance
        ADC_READY_Callback(); // Trigger the main FOC mathematical loop
    } // End of flag check
} // End of DMA IRQ Handler

/**
 * @brief Fast direct-register initialization for ADC1 and ADC2
 * Completely replaces all bloated LL_ADC initialization structures for maximum speed and memory compactness.
 */
inline void MX_ADCs_Init() { // Unified bare-metal initialization function for both ADCs
    // 1. Clock Configuration
    RCC->CCIPR = (RCC->CCIPR & ~RCC_CCIPR_ADC12SEL) | (2U << RCC_CCIPR_ADC12SEL_Pos); // Route SYSCLK directly to ADC12
    RCC->AHB2ENR |= RCC_AHB2ENR_ADC12EN; // Enable hardware clock bus for the ADC12 peripheral

    // 2. Disable Deep Power Down and Enable Regulators
    ADC1->CR &= ~ADC_CR_DEEPPWD; // Exit deep power down mode for ADC1
    ADC2->CR &= ~ADC_CR_DEEPPWD; // Exit deep power down mode for ADC2
    ADC1->CR |= ADC_CR_ADVREGEN; // Enable internal voltage regulator for ADC1
    ADC2->CR |= ADC_CR_ADVREGEN; // Enable internal voltage regulator for ADC2

    // 3. Delay for regulator stabilization (STM32G4 requires ~20us minimum)
    // 170 MHz system clock = 170 cycles per microsecond. A simple while loop takes ~3 cycles per iteration.
    volatile uint32_t wait_loop_index = (170UL * 20UL) / 3UL; // Calculate 20us wait dynamic threshold
    while (wait_loop_index != 0) { wait_loop_index--; } // Block CPU execution until regulator is completely stable

    // 4. ADC12 Common Register Configuration (Dual Mode Setup)
    // CKMODE = 3 (Synchronous PCLK/4), MDMA = 2 (12/10-bit mode for DMA), DUAL = 6 (Regular Simultaneous Mode)
    // VSENSESEL = 1: enable the internal temperature sensor conversion path
    // (the internal sensor is wired to ADC1 channel 16 only).
    ADC12_COMMON->CCR = (3U << ADC_CCR_CKMODE_Pos) | (2U << ADC_CCR_MDMA_Pos) | (6U << ADC_CCR_DUAL_Pos) |
                        ADC_CCR_VSENSESEL; // Write common setup

    // 5. ADC1 (Master) Configuration
    // ALIGN = 1 (Left-aligned), DMACFG = 1 (Circular DMA), DMAEN = 1 (Enable DMA requests)
    ADC1->CFGR = ADC_CFGR_ALIGN | ADC_CFGR_DMACFG | ADC_CFGR_DMAEN; // Write CFGR register

    // Sequencer SQR1: L = 2 (Length = 3 ranks). SQ1 = 4 (PA3/I_A),
    // SQ2 = 15 (PB0/I_C), SQ3 = 16 (internal temperature sensor).
    // The thermal sample now travels through the SAME regular group/DMA
    // stream as the current channels: fetching it costs zero CPU cycles
    // in the ISR (plain DMA-buffer read instead of an injected-conversion
    // state machine).
    ADC1->SQR1 = (2U << ADC_SQR1_L_Pos) | (4U << ADC_SQR1_SQ1_Pos) | (15U << ADC_SQR1_SQ2_Pos) |
                 (16U << ADC_SQR1_SQ3_Pos); // Write Rank sequence

    // Sampling Time: SMP4 = 1 (6.5 cycles), SMP15 = 1 (6.5 cycles)
    ADC1->SMPR1 = (1U << ADC_SMPR1_SMP4_Pos); // Apply ultra-fast sampling time to Channel 4
    ADC1->SMPR2 = (1U << ADC_SMPR2_SMP15_Pos); // Apply ultra-fast sampling time to Channel 15

    // Internal temperature sensor sampling time (rank-3 channel only, does
    // not affect the current-sense channels): SMP16 = 6 -> 247.5 cycles
    // ~= 5.8 us at fADC = 42.5 MHz. This MEETS the datasheet t_S_temp >= 5 us
    // requirement and MUST NOT be shortened: the internal sensor has a high
    // output impedance, and an undersampled S/H capacitor keeps residual
    // charge from the preceding current-sense rank -- producing a large
    // CONSTANT positive temperature offset plus readings that track phase
    // currents. The software EMA filter cannot remove such a DC bias.
    // Cost: every regular sequence stretches by ~6 us inside the 50 us
    // control period (accepted trade-off for valid thermal data).
    ADC1->SMPR2 |= (6U << ADC_SMPR2_SMP16_Pos);

    // 6. ADC2 (Slave) Configuration
    // ALIGN = 1 (Left-aligned). NOTE: Slave does NOT generate DMA requests, so DMAEN is left as 0
    ADC2->CFGR = ADC_CFGR_ALIGN; // Write slave CFGR register

    // Sequencer SQR1: L = 2 (Length = 3 ranks). SQ1 = 2 (PA1/I_B),
    // SQ2 = 1 (PA0/VSENS), SQ3 = 5 (PC4/TEMP external sensor).
    // In regular simultaneous mode BOTH sequences must have equal length;
    // the external board temperature sensor rides in the upper half of
    // DMA word 3 (exposed via ADC_get_values(), unused for now).
    ADC2->SQR1 = (2U << ADC_SQR1_L_Pos) | (2U << ADC_SQR1_SQ1_Pos) | (1U << ADC_SQR1_SQ2_Pos) |
                 (5U << ADC_SQR1_SQ3_Pos); // Write Rank sequence

    // Sampling Time: SMP1 = 1 (6.5 cycles), SMP2 = 1 (6.5 cycles)
    // TEMP channel (SMP5) is slow-sampled like the internal sensor.
    ADC2->SMPR1 = (1U << ADC_SMPR1_SMP1_Pos) | (1U << ADC_SMPR1_SMP2_Pos) |
                  (6U << ADC_SMPR1_SMP5_Pos); // Apply sampling times to Ch1, Ch2 and Ch5
} // End of bare-metal MX_ADCs_Init function

/**
 * @brief Ultra-fast single-instruction trigger for Dual ADC conversion
 */
__attribute__((always_inline)) inline void ADC1_StartDMAConversion() { // Inline start command replacing LL
    ADC1->CR |= ADC_CR_ADSTART; // Start ADC1 conversion (ADC2 automatically triggers simultaneously in Dual Mode)
} // End of ADC1_StartDMAConversion function

/**
 * @brief Master initialization wrapper orchestrating ADC startup sequencing
 */
inline void ADC_Init() { // Main initialization wrapper
    MX_DMA_ADC1_Init(); // Initialize DMA first (Using untouched HAL/LL logic)
    MX_ADCs_Init(); // Initialize ADC bare-metal hardware configuration

    // Calibrate ADC1 (Master)
    ADC1->CR |= ADC_CR_ADCAL; // Trigger hardware calibration sequence for ADC1
    while (ADC1->CR & ADC_CR_ADCAL); // Block CPU execution until ADC1 hardware clears the calibration flag

    // Calibrate ADC2 (Slave)
    ADC2->CR |= ADC_CR_ADCAL; // Trigger hardware calibration sequence for ADC2
    while (ADC2->CR & ADC_CR_ADCAL); // Block CPU execution until ADC2 hardware clears the calibration flag

    // Clear Ready Flags
    ADC1->ISR = ADC_ISR_ADRDY; // Write 1 to clear the ADRDY flag before enabling
    ADC2->ISR = ADC_ISR_ADRDY; // Write 1 to clear the ADRDY flag before enabling

    // Enable ADCs
    ADC1->CR |= ADC_CR_ADEN; // Set Enable bit for ADC1
    ADC2->CR |= ADC_CR_ADEN; // Set Enable bit for ADC2

    // Wait for both ADCs to be fully ready
    while ((ADC1->ISR & ADC_ISR_ADRDY) == 0); // Block CPU until ADC1 is confirmed ready
    while ((ADC2->ISR & ADC_ISR_ADRDY) == 0); // Block CPU until ADC2 is confirmed ready

    // Clear End of Conversion flags before first use to ensure clean interrupts
    ADC1->ISR = ADC_ISR_EOC; // Write 1 to clear EOC flag
    ADC2->ISR = ADC_ISR_EOC; // Write 1 to clear EOC flag
} // End of ADC_Init function

/**
 * @brief Safely parses DMA buffer into human-readable variables
 */
__attribute__((always_inline)) inline void ADC_get_values(uint16_t& current_a, // Phase A reference
                           uint16_t& current_b, // Phase B reference
                           uint16_t& current_c, // Phase C reference
                           uint16_t& voltage_supply, // Bus voltage reference
                           uint16_t& voltage_temp) { // Internal MCU temperature reference

    // OPTIMIZATION: Fetch the volatile memory exactly once to prevent cache stalls and race conditions
    uint32_t reg0 = dma_buffer_adc12[0]; // Fetch Rank 1 dual data
    uint32_t reg1 = dma_buffer_adc12[1]; // Fetch Rank 2 dual data
    uint32_t reg2 = dma_buffer_adc12[2]; // Fetch Rank 3 dual data (internal temp | external TEMP)

    current_a      = (uint16_t)(reg0 & 0xFFFF);        // Extract lower 16 bits: ADC1 Rank 1 (I_A)
    current_b      = (uint16_t)((reg0 >> 16) & 0xFFFF);// Extract upper 16 bits: ADC2 Rank 1 (I_B)

    current_c      = (uint16_t)(reg1 & 0xFFFF);        // Extract lower 16 bits: ADC1 Rank 2 (I_C)
    voltage_supply = (uint16_t)((reg1 >> 16) & 0xFFFF);// Extract upper 16 bits: ADC2 Rank 2 (VSENS)

    voltage_temp = (uint16_t)(reg2 & 0xFFFF); // Internal temperature sensor: ADC1 rank 3, straight from the DMA buffer (zero CPU cost)
    // Note: the external TEMP (PC4) result is the upper half of reg2; it is
    // routed to a dedicated input of the board layer when needed.
} // End of ADC_get_values function

// ============================================================================
// INTERNAL TEMPERATURE SENSOR (regular rank 3, fetched by the shared DMA)
// ============================================================================

constexpr uint32_t ADC_VDDA_MV = 3300;  // Board analog reference voltage [mV] (TS_CALx are 3.0 V calibrated, macro compensates)

/**
 * @brief Convert a raw temperature-sensor word into degrees Celsius.
 *
 * Recalculation callback installed into Temperature by the board layer.
 * Uses the STM32G4 factory calibration values TS_CAL1 (30 degC) /
 * TS_CAL2 (110 degC) from system memory via the LL helper macro. All-integer
 * math internally (no float division), so it is safe to run from tick() in
 * interrupt context at the ~1 Hz sample rate.
 *
 * @param raw_adc Filtered raw value in left-aligned 16-bit format
 * @return Temperature in degrees Celsius
 */
inline float ADC_Temperature_To_Celsius(uint32_t raw_adc) {
    int32_t celsius_int =
        __LL_ADC_CALC_TEMPERATURE(ADC_VDDA_MV,                     // Actual Vref+ [mV]
                                  (uint32_t)(raw_adc >> 4),        // Left-aligned word -> 12-bit right-aligned code
                                  LL_ADC_RESOLUTION_12B);          // Native resolution of the calibration data
    return (float)celsius_int;
} // End of ADC_Temperature_To_Celsius function

#endif // ADC_DMA_SETUP_CLN234_H // Final close of the header include guard
