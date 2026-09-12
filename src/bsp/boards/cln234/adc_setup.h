/**
 * @file adc_setup.h
 * @brief Triple-ADC simultaneous sampling backend for CLN234.
 *
 * All three phase currents are converted in ONE trigger event. The
 * STM32G474CEU6 supports regular simultaneous mode only inside fixed pairs
 * (ADC1+ADC2 and ADC3+ADC4/5), so the packing scheme is:
 *
 *   ADC1 (master) + ADC2 (slave), dual regular simultaneous, DMA1_Ch5 reads
 *   ADC12_COMMON->CDR (2 packed words, one per rank pair):
 *     Word 0 : ADC1 rank1 = I_A (PA3/ADC1_IN4)   | ADC2 rank1 = I_B (PA1/ADC2_IN2)
 *     Word 1 : ADC1 rank2 = VSENS (PA0/ADC1_IN1) | ADC2 rank2 = TEMP (PC4/ADC2_IN5)
 *
 *   ADC3, single rank, same software trigger:
 *     I_C (PB0/ADC3_IN12), result fetched from ADC3->DR in the DMA
 *     transfer-complete callback (the single-rank conversion always
 *     completes before the 2-rank ADC12 sequence raises DMA TC).
 *
 * PIN MAPPING NOTE (verified against the CubeMX G474CEUx pin database):
 * PA3 exposes ADC1_IN4 ONLY - it has no ADC3 routing (ADC3_IN12 is
 * physically PB0). The pin/channel assignment above is therefore the only
 * feasible mapping that keeps every phase current on its own converter:
 *   PB0 (I_C) -> ADC3.12, PA3 (I_A) -> ADC1.4, PA1 (I_B) -> ADC2.2,
 *   PC4 (TEMP) -> ADC2.5, PA0 (VSENS) -> ADC1.1.
 * I_A and I_B are still sampled at the exact same instant (dual mode); I_C
 * starts on the same software trigger write sequence (< 100 ns skew).
 *
 * MASTER/SLAVE TIMING (bench-verified): in dual regular simultaneous mode
 * the pair-N DMA request is issued at the MASTER's EOC, while the slave
 * converts a couple of ADC clocks behind. With EQUAL sampling windows the
 * slave's rank-N data is not yet latched when the DMA reads the CDR: the
 * rank-2 slave half-word then carries the PREVIOUS rank value (TEMP slot
 * showed the I_B level in ~70% of sequences). CLN17 V3 gets away with equal
 * windows only because its rank-1 slave dummy echoes the same channel and
 * its rank-3 master window is long - the race is invisible there. FIX here:
 * the master samples 12.5 cycles vs the slave's 6.5, so the slave latches
 * ~6 ADC cycles (141 ns) BEFORE the master's EOC fires the request - no
 * race at any trigger rate, and the whole 2-rank sequence still takes only
 * 50 ADC cycles (~1.2 us).
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
// Two words: one per rank pair of the dual regular simultaneous sequence.
// Word 1 upper half = ADC2 rank 2 = TEMP (PC4) = the external NTC thermistor.
alignas(4) inline volatile uint32_t dma_buffer_adc12[2] = {0}; // Dual-rank ADC12 DMA buffer

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
    HAL_DMA_Start(&hdma_adc1, (uint32_t)&ADC12_COMMON->CDR, (uint32_t)dma_buffer_adc12, 2); // Map CDR to array (one word per rank pair)

    __HAL_DMA_ENABLE_IT(&hdma_adc1, DMA_IT_TC | DMA_IT_TE); // Enable Transfer Complete and Transfer Error interrupts
} // End of MX_DMA_ADC1_Init function

/**
 * @brief DMA1 Channel 5 IRQ handler
 */
// DMA1_Channel5_IRQHandler: interrupt vector, defined in the board translation unit
// (bsp/boards/cln234/board.cpp) so that exactly one object file owns it.

/**
 * @brief Fast direct-register initialization for ADC1, ADC2 and ADC3
 * Completely replaces all bloated LL_ADC initialization structures for maximum speed and memory compactness.
 */
inline void MX_ADCs_Init() { // Unified bare-metal initialization function for all three ADCs
    // 1. Clock Configuration
    RCC->CCIPR = (RCC->CCIPR & ~RCC_CCIPR_ADC12SEL) | (2U << RCC_CCIPR_ADC12SEL_Pos); // Route SYSCLK directly to ADC1/ADC2
    RCC->CCIPR = (RCC->CCIPR & ~RCC_CCIPR_ADC345SEL) | (2U << RCC_CCIPR_ADC345SEL_Pos); // Route SYSCLK directly to the ADC3/4/5 group
    RCC->AHB2ENR |= RCC_AHB2ENR_ADC12EN; // Enable hardware clock bus for the ADC1/ADC2 peripheral
    RCC->AHB2ENR |= RCC_AHB2ENR_ADC345EN; // Enable hardware clock bus for the ADC3/4/5 peripheral

    // 2. Disable Deep Power Down and Enable Regulators (all three converters)
    ADC1->CR &= ~ADC_CR_DEEPPWD; // Exit deep power down mode for ADC1
    ADC2->CR &= ~ADC_CR_DEEPPWD; // Exit deep power down mode for ADC2
    ADC3->CR &= ~ADC_CR_DEEPPWD; // Exit deep power down mode for ADC3
    ADC1->CR |= ADC_CR_ADVREGEN; // Enable internal voltage regulator for ADC1
    ADC2->CR |= ADC_CR_ADVREGEN; // Enable internal voltage regulator for ADC2
    ADC3->CR |= ADC_CR_ADVREGEN; // Enable internal voltage regulator for ADC3

    // 3. Delay for regulator stabilization (STM32G4 requires ~20us minimum)
    // 170 MHz system clock = 170 cycles per microsecond. A simple while loop takes ~3 cycles per iteration.
    volatile uint32_t wait_loop_index = (170UL * 20UL) / 3UL; // Calculate 20us wait dynamic threshold
    while (wait_loop_index != 0) { wait_loop_index--; } // Block CPU execution until regulator is completely stable

    // 4. ADC12 Common Register Configuration (Dual Mode Setup)
    // CKMODE = 3 (Synchronous PCLK/4), MDMA = 2 (12/10-bit mode for DMA), DUAL = 6 (Regular Simultaneous Mode)
    // NOTE: VSENSESEL is deliberately LEFT CLEAR: the CLN234 board temperature
    // comes from the external NTC (PC4/ADC2_IN5), the internal die sensor of
    // the old CLN17 V3 rank-3 slot is not sampled at all.
    ADC12_COMMON->CCR = (3U << ADC_CCR_CKMODE_Pos) | (2U << ADC_CCR_MDMA_Pos) | (6U << ADC_CCR_DUAL_Pos); // Write common setup

    // ADC3/4/5 group must be clocked synchronously as well, otherwise ADC3
    // would run on its default asynchronous kernel clock and drift against
    // the ADC12 pair. Independent (non-dual) mode: DUAL is left at 0.
    ADC345_COMMON->CCR = (3U << ADC_CCR_CKMODE_Pos); // Synchronous PCLK/4 for the ADC3 group

    // 5. ADC1 (Master) Configuration
    // ALIGN = 1 (Left-aligned), DMACFG = 1 (Circular DMA), DMAEN = 1 (Enable DMA requests)
    ADC1->CFGR = ADC_CFGR_ALIGN | ADC_CFGR_DMACFG | ADC_CFGR_DMAEN; // Write CFGR register

    // Sequencer SQR1: L = 1 (Length = 2 ranks). SQ1 = 4 (PA3/I_A),
    // SQ2 = 1 (PA0/VSENS).
    ADC1->SQR1 = (1U << ADC_SQR1_L_Pos) | (4U << ADC_SQR1_SQ1_Pos) |
                 (1U << ADC_SQR1_SQ2_Pos); // Write Rank sequence

    // Sampling Time: SMP4 = 2 (12.5 cycles) for I_A, SMP1 = 2 (12.5 cycles)
    // for VSENS. The MASTER must finish every rank AFTER the slave (6.5
    // cycles) so the slave's data is latched into CDR before the master's
    // EOC fires the DMA request - see the header note. 6 ADC cycles (~141
    // ns) of margin make the ordering deterministic at any trigger rate.
    ADC1->SMPR1 = (2U << ADC_SMPR1_SMP4_Pos) | (2U << ADC_SMPR1_SMP1_Pos); // Apply sampling time to Ch4 and Ch1

    // 6. ADC2 (Slave) Configuration
    // ALIGN = 1 (Left-aligned). NOTE: Slave does NOT generate DMA requests, so DMAEN is left as 0
    ADC2->CFGR = ADC_CFGR_ALIGN; // Write slave CFGR register

    // Sequencer SQR1: L = 1 (Length = 2 ranks). SQ1 = 2 (PA1/I_B),
    // SQ2 = 5 (PC4/TEMP external NTC sensor).
    // In regular simultaneous mode BOTH sequences must have equal length;
    // the board temperature sensor rides in the upper half of DMA word 1
    // (exposed via ADC_get_values()).
    ADC2->SQR1 = (1U << ADC_SQR1_L_Pos) | (2U << ADC_SQR1_SQ1_Pos) |
                 (5U << ADC_SQR1_SQ2_Pos); // Write Rank sequence

    // Sampling Time: SMP2 = 1 (6.5 cycles) for I_B, SMP5 = 1 (6.5 cycles)
    // for the TEMP channel - the SHORTEST window, on the SLAVE converter.
    // The slave always finishes its rank 6 ADC cycles before the master
    // (12.5 cycles), keeping the CDR slave half-word fresh (see header).
    ADC2->SMPR1 = (1U << ADC_SMPR1_SMP2_Pos) | (1U << ADC_SMPR1_SMP5_Pos); // Apply sampling times to Ch2 and Ch5

    // 7. ADC3 Configuration (standalone converter, I_C phase current)
    // ALIGN = 1 (Left-aligned). No DMA: the single-rank result is fetched
    // straight from ADC3->DR inside the DMA1_Channel5 transfer-complete
    // callback. The single conversion (6.5 + 12.5 = 19 ADC cycles ~ 0.45 us)
    // always finishes before the 2-rank ADC12 sequence (38 cycles ~ 0.9 us)
    // raises the DMA TC interrupt, so DR is guaranteed valid at read time.
    // EXTEN = 0 after reset: software trigger only (ADSTART), as requested.
    ADC3->CFGR = ADC_CFGR_ALIGN; // Write CFGR register

    // Sequencer SQR1: L = 0 (Length = 1 rank). SQ1 = 12 (PB0/I_C).
    ADC3->SQR1 = (0U << ADC_SQR1_L_Pos) | (12U << ADC_SQR1_SQ1_Pos); // Write Rank sequence

    // Sampling Time: SMP12 = 1 (6.5 cycles), identical to all other channels
    ADC3->SMPR2 = (1U << ADC_SMPR2_SMP12_Pos); // Apply ultra-fast sampling time to Ch12
} // End of bare-metal MX_ADCs_Init function

// NOTE: there is deliberately NO per-trigger DMA re-arm (ADC_DMA_Resync was
// removed). The circular stream is phase-locked to the sequences - exactly
// 2 word transfers per 2-word circular buffer - which is how the proven
// CLN17 V3 setup runs its dual pair without any re-arm at 20 kHz.

/**
 * @brief Ultra-fast trigger for the whole simultaneous 3-converter sample burst
 *
 * ADC3 (single rank, I_C) is started FIRST so its result is always settled
 * when the ADC12 dual sequence raises DMA TC; ADC1->ADSTART then starts the
 * dual regular simultaneous conversion (ADC2 follows ADC1 automatically in
 * Dual Mode). Both writes are back-to-back register stores: the sampling
 * windows of all three phase currents are aligned within ~100 ns.
 */
__attribute__((always_inline)) inline void ADC1_StartDMAConversion() { // Inline start command replacing LL
    ADC3->CR |= ADC_CR_ADSTART; // Start ADC3 single-rank conversion (I_C on PB0/ADC3_IN12)
    ADC1->CR |= ADC_CR_ADSTART; // Start ADC1 conversion (ADC2 automatically triggers simultaneously in Dual Mode)
} // End of ADC1_StartDMAConversion function

/**
 * @brief Master initialization wrapper orchestrating ADC startup sequencing
 */
inline void ADC_Init() { // Main initialization wrapper
    MX_DMA_ADC1_Init(); // Initialize DMA first (Using untouched HAL/LL logic)
    MX_ADCs_Init(); // Initialize ADC bare-metal hardware configuration

    // Calibrate ADC1 (Master of the dual pair)
    ADC1->CR |= ADC_CR_ADCAL; // Trigger hardware calibration sequence for ADC1
    while (ADC1->CR & ADC_CR_ADCAL); // Block CPU execution until ADC1 hardware clears the calibration flag

    // Calibrate ADC2 (Slave of the dual pair)
    ADC2->CR |= ADC_CR_ADCAL; // Trigger hardware calibration sequence for ADC2
    while (ADC2->CR & ADC_CR_ADCAL); // Block CPU execution until ADC2 hardware clears the calibration flag

    // Calibrate ADC3 (standalone I_C converter)
    ADC3->CR |= ADC_CR_ADCAL; // Trigger hardware calibration sequence for ADC3
    while (ADC3->CR & ADC_CR_ADCAL); // Block CPU execution until ADC3 hardware clears the calibration flag

    // Clear Ready Flags
    ADC1->ISR = ADC_ISR_ADRDY; // Write 1 to clear the ADRDY flag before enabling
    ADC2->ISR = ADC_ISR_ADRDY; // Write 1 to clear the ADRDY flag before enabling
    ADC3->ISR = ADC_ISR_ADRDY; // Write 1 to clear the ADRDY flag before enabling

    // Enable ADCs
    ADC1->CR |= ADC_CR_ADEN; // Set Enable bit for ADC1
    ADC2->CR |= ADC_CR_ADEN; // Set Enable bit for ADC2
    ADC3->CR |= ADC_CR_ADEN; // Set Enable bit for ADC3

    // Wait for all three ADCs to be fully ready
    while ((ADC1->ISR & ADC_ISR_ADRDY) == 0); // Block CPU until ADC1 is confirmed ready
    while ((ADC2->ISR & ADC_ISR_ADRDY) == 0); // Block CPU until ADC2 is confirmed ready
    while ((ADC3->ISR & ADC_ISR_ADRDY) == 0); // Block CPU until ADC3 is confirmed ready

    // Clear End of Conversion flags before first use to ensure clean interrupts
    ADC1->ISR = ADC_ISR_EOC; // Write 1 to clear EOC flag
    ADC2->ISR = ADC_ISR_EOC; // Write 1 to clear EOC flag
    ADC3->ISR = ADC_ISR_EOC; // Write 1 to clear EOC flag
} // End of ADC_Init function

/**
 * @brief Safely parses the DMA buffer + ADC3 data register into human-readable variables
 *
 * @param current_a      I_A raw word (PA3 / ADC1_IN4, rank 1, lower half of word 0)
 * @param current_b      I_B raw word (PA1 / ADC2_IN2, rank 1, upper half of word 0)
 * @param current_c      I_C raw word (PB0 / ADC3_IN12, straight from ADC3->DR)
 * @param voltage_supply VSENS raw word (PA0 / ADC1_IN1, rank 2, lower half of word 1)
 * @param voltage_temp   TEMP raw word (PC4 / ADC2_IN5 external NTC, upper half of word 1)
 */
__attribute__((always_inline)) inline void ADC_get_values(uint16_t& current_a, // Phase A reference
                           uint16_t& current_b, // Phase B reference
                           uint16_t& current_c, // Phase C reference
                           uint16_t& voltage_supply, // Bus voltage reference
                           uint16_t& voltage_temp) { // External board temperature reference

    // OPTIMIZATION: Fetch the volatile memory exactly once to prevent cache stalls and race conditions
    uint32_t reg0 = dma_buffer_adc12[0]; // Fetch Rank 1 dual data
    uint32_t reg1 = dma_buffer_adc12[1]; // Fetch Rank 2 dual data

    current_a      = (uint16_t)(reg0 & 0xFFFF);        // Extract lower 16 bits: ADC1 Rank 1 (I_A)
    current_b      = (uint16_t)((reg0 >> 16) & 0xFFFF);// Extract upper 16 bits: ADC2 Rank 1 (I_B)

    current_c      = (uint16_t)(ADC3->DR & 0xFFFF);    // Single-rank I_C result, straight from the data register (zero DMA cost)
    voltage_supply = (uint16_t)(reg1 & 0xFFFF);        // Extract lower 16 bits: ADC1 Rank 2 (VSENS)
    voltage_temp   = (uint16_t)((reg1 >> 16) & 0xFFFF);// Extract upper 16 bits: ADC2 Rank 2 (TEMP, external NTC)
} // End of ADC_get_values function

#endif // ADC_DMA_SETUP_CLN234_H // Final close of the header include guard
