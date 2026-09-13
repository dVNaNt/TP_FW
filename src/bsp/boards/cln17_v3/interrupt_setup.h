/**
 * @file interrupt_setup.h
 * @brief NVIC configuration and ISR glue for CLN17 v3.
 *
 * NVIC_Init() assigns priorities; the ISR handlers bridge hardware events
 * (ADC end-of-conversion, comparators, timers) into the global TP
 * object.
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#ifndef NVIC_CLN17V3_H  // Include guard to prevent multiple inclusions of this header file
#define NVIC_CLN17V3_H  // Define the include guard macro for the preprocessor

#include <Arduino.h>  // Include Arduino framework for basic standard types

#include "adc_setup.h"    // Include ADC subsystem initialization and control
#include "clock_setup.h"  // Include system clock configuration
#include "bsp/stm32g4/comp_dac.h"   // Include analog comparator subsystem setup
#include "gpio_setup.h"   // Include general purpose I/O configuration

#include "bsp/stm32g4/fdcan.h"        // Include CAN/CAN-FD communication setup
#include "spi_setup.h"        // Include SPI communication setup for the encoder
#include "bsp/stm32g4/tim1_pwm.h"  // Include PWM timer hardware definitions and functions

// OPTIMIZATION (P2.5): cache &TP.driver_phy once instead of walking
// the global object member chain on every sample. The address is fixed at
// link time; the first ISR call binds it, afterwards only a register load remains.
inline DriverPWM* heart_driver_phy() {
    static DriverPWM* phy = &TP.driver_phy;
    return phy;
}

inline void ADC_READY_Callback() {  // Define inline callback for ADC DMA completion event

    DriverPWM* phy = heart_driver_phy();  // Cached driver-physical-layer pointer

    uint16_t current_values[3];                              // Declare array to store raw ADC current readings for all 3 phases
    uint16_t voltage_supply, temperature_adc;  // Declare variables for auxiliary system measurements

    ADC_get_values(current_values[0], current_values[1], current_values[2], voltage_supply, 
                   temperature_adc);  // Fetch all ADC readings simultaneously (temperature rides in the same DMA group)

    constexpr uint16_t upper_th = uint16_t(
        (65535.0f * (3.3f - 0.3f)) / 3.3f);  // Calculate max valid ADC threshold at compile time (excluding non-linear zone)
    constexpr uint16_t lower_th = uint16_t((65535.0f * 0.3f) / 3.3f);  // Calculate min valid ADC threshold at compile time
    constexpr uint32_t valid_range =
        upper_th - lower_th;  // Calculate the allowed ADC range at compile time to avoid runtime math

    if ((uint32_t)(current_values[0] - lower_th) > valid_range)  // Ultra-fast single-branch bounds check for Phase A
        current_values[0] = UINT16_MAX;  // Mark Phase A reading as invalid if it falls outside the valid range

    if ((uint32_t)(current_values[1] - lower_th) > valid_range)  // Ultra-fast single-branch bounds check for Phase B
        current_values[1] = UINT16_MAX;  // Mark Phase B reading as invalid if it falls outside the valid range

    if ((uint32_t)(current_values[2] - lower_th) > valid_range)  // Ultra-fast single-branch bounds check for Phase C
        current_values[2] = UINT16_MAX;  // Mark Phase C reading as invalid if it falls outside the valid range
    
    phy->current.set_currents_irq(current_values);
    phy->supply.set_voltage_irq(voltage_supply);
    // Feed the thermal protection pipeline (raw 12-bit left-aligned). A zero
    // word means "no injected conversion completed yet" (the sensor reads
    // ~1.4 V, never 0), so it is skipped to keep the filter unprimed.
    phy->temperature.set_temperature_irq(temperature_adc);


}  // End of ADC_READY_Callback function

/**
 * Control-pass timing, in CPU cycles from the DWT cycle counter (enabled in
 * NVIC_Init()). The pass runs on the encoder frame's completion, so its start
 * is the SPI transfer time after the CH4 match, and it MUST end before the
 * next match: the duty it computes is latched there, and a pass that ends
 * after it is latched one period later. A pass end that wanders across that
 * boundary from one pass to the next makes the current-loop delay jitter
 * between one and two periods, which destabilised the loop on long profiled
 * moves before the encoder clock was raised (spi_setup.h). PASS_LATE counts
 * the passes that ended after the boundary; it must stay at zero. Read these
 * over the protocol's direct register access (0x00, SRAM window) by symbol
 * address when a build needs checking - no register carries them.
 */
inline volatile uint32_t PASS_T_CC4 = 0;        ///< CYCCNT at the TIM1 CH4 match (ISR entry).
inline volatile uint32_t PASS_OFFSET_LAST = 0;  ///< Cycles from the CH4 match to the pass start (the encoder frame).
inline volatile uint32_t PASS_CYCLES_LAST = 0;  ///< Duration of the last pass.
inline volatile uint32_t PASS_CYCLES_MAX = 0;   ///< Longest pass since boot.
inline volatile uint32_t PASS_LATE = 0;         ///< Passes that ended after the next CH4 match.

inline void POSITION_Callback() {  // Define inline callback for SPI encoder DMA completion
    const uint32_t t_cc4 = PASS_T_CC4;                 // Match this frame was started at
    const uint32_t t0 = DWT->CYCCNT;
    PASS_OFFSET_LAST = t0 - t_cc4;
    TP.encoder.set_raw_position(Encoder_GetAngle());  // Fetch raw mechanical angle from SPI and inject it into the FOC core
    TP.tick();
    const uint32_t t1 = DWT->CYCCNT;
    PASS_CYCLES_LAST = t1 - t0;
    if (PASS_CYCLES_LAST > PASS_CYCLES_MAX) {
        PASS_CYCLES_MAX = PASS_CYCLES_LAST;
    }
    if (PASS_T_CC4 != t_cc4) {
        PASS_LATE++;                                   // The next match fired while this pass ran
    }
}  // End of POSITION_Callback function

inline void NVIC_Init() {  // Define the initialization function for the Nested Vectored Interrupt Controller
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;  // Cycle counter for the pass timing (POSITION_Callback)
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    NVIC_SetPriority(SPI1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 6, 0));  // Set SPI1 interrupt to priority level 6
    NVIC_EnableIRQ(SPI1_IRQn);  // Enable the SPI1 interrupt in the NVIC hardware

    // TIM1 CH4 match: starts the encoder transfer and latches the next
    // duty vector. Priority 0, above the USB stack (1) and the ADC DMA (1):
    // at 3 it waited behind every USB transfer, which shifted the encoder
    // sample and delayed the duty push by the USB handler's duration.
    NVIC_SetPriority(TIM1_CC_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    NVIC_EnableIRQ(TIM1_CC_IRQn);                            // Enable the TIM1 Capture/Compare interrupt in the NVIC hardware
}  // End of NVIC_Init function

/**
 * @brief Ultra-fast TIM1 CH4 Match Callback (Replaces HAL_TIM_OC_DelayElapsedCallback)
 *
 * This function bypasses HAL entirely. It must be called directly from TIM1_CC_IRQHandler.
 */
inline void FOC_TimTrig_Callback() {  // Define custom inline callback for the timer trigger event
    PASS_T_CC4 = DWT->CYCCNT;         // Timestamp of the CH4 match (pass timing, see POSITION_Callback)
    // The ADC is NOT started here any more: TIM1 triggers it in hardware at
    // this same CH4 match (adc_setup.h), so the sampling instant no longer
    // depends on how long this interrupt took to be entered.
    Encoder_StartTransfer();  // Initiate non-blocking SPI DMA transfer to request the next encoder reading

    TP.driver_phy.tick_icr();  // Request new duty cycles from FOC core and write them to hardware registers
}  // End of FOC_TimTrig_Callback function

#endif  // NVIC_CLN17V3_H // Final close of the header include guard
