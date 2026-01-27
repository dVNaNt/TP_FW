#ifndef NVIC_RADIX_H
#define NVIC_RADIX_H

#include <Arduino.h>
#include "config.h"

#include "adc_setup.h"
#include "dma_setup.h"
#include "gpio_setup.h"
#include "spi_setup.h"
#include "clock_setup.h"
#include "timer_pwm_setup.h"
#include "system_setup.h"

/**
 * PWM Timer Execution Pattern (Center-Aligned Mode 3)
 * ====================================================
 * 
 * Timer cycle:     0 ────────► ARR ────────► 0
 *                  │    UP      │    DOWN    │
 *                  UF          OF           UF
 * 
 * PWM output (CCR=0 at start):
 *   - Pin is LOW when CNT >= CCR (most of the time when CCR is small)
 *   - Pin is HIGH when CNT < CCR
 * 
 * Event sequence per full cycle:
 * ┌─────────────────────────────────────────────────────────────────┐
 * │  CNT:  0 ──► CCR2 ──────────► ARR ──► CCR2 ──────────► 0        │
 * │             │                  │       │                │        │
 * │        ADC_START_Callback    PWM_Callback │           ENCODER_Callback
 * │        (ADC sample)       (update PWM) │           (read encoder)
 * │                                        │                         │
 * │                               (skip ADC on down)                 │
 * └─────────────────────────────────────────────────────────────────┘
 * 
 * POSITION_Callback is called from SPI DMA complete (when encoder data ready)
 */

extern void PWM_Callback();         // Called at overflow (CNT reaches ARR) - update PWM duty
extern void ENCODER_Callback();     // Called at underflow (CNT reaches 0) - trigger encoder read
extern void ADC_START_Callback();   // Called before overflow (CH2 match on UP) - sample ADC
extern void ADC_READY_Callback();   // Called from DMA complete
extern void POSITION_Callback();    // Called from SPI complete (encoder data ready)

void NVIC_Init() {
    /* DMA interrupt init */
    /* DMA1_Channel1_IRQn interrupt configuration */
    NVIC_SetPriority(DMA1_Channel1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), DMA_CH1_INT_PRIORITY, 0));
    NVIC_EnableIRQ(DMA1_Channel1_IRQn);

    /* SPI1 interrupt init */
    NVIC_SetPriority(SPI1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), SPI1_INT_PRIORITY, 0));
    NVIC_EnableIRQ(SPI1_IRQn);

    /* PWM TIM2 interrupt Init */
    HAL_NVIC_SetPriority(TIM2_IRQn, TIM2_INT_PRIORITY, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

/**
 * Timer Update callback (Center-Aligned Mode 3: Update on BOTH overflow and underflow)
 * 
 * Sequence:
 *   Start:      is_overflow = true (first event will be overflow after CNT goes 0→ARR)
 *   1st Update: is_overflow = true  → PWM_Callback() at overflow (CNT=ARR)
 *   2nd Update: is_overflow = false → ENCODER_Callback() at underflow (CNT=0)
 *   3rd Update: is_overflow = true  → PWM_Callback()
 *   4th Update: is_overflow = false → ENCODER_Callback()
 *   ...
 */
extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
    if (htim->Instance != TIM2) return;  // Only handle TIM2 (PWM timer)

    static bool is_overflow = true;  // Toggle flag, starts true (first event is OF)
    
    if (is_overflow) {
        PWM_Callback();       // Update PWM duty at overflow (CNT=ARR, all pins LOW)
    } else {
        ENCODER_Callback();   // Trigger encoder read at underflow (CNT=0)
    }
    
    is_overflow = !is_overflow;  // Toggle for next event
}

/**
 * Output Compare match callback (CH2 triggers when CNT == CCR2)
 * 
 * CH2 match occurs TWICE per full cycle (on UP and DOWN count)
 * ADC sampling happens on count UP, just before overflow when all PWM pins are LOW
 * 
 * Sequence:
 *   Start:      is_count_up = true (first match is on UP toward ARR)
 *   1st match:  is_count_up = true  → ADC_START_Callback() (sample ADC before OF)
 *   2nd match:  is_count_up = false → skip (count DOWN, after OF)
 *   3rd match:  is_count_up = true  → ADC_START_Callback()
 *   4th match:  is_count_up = false → skip
 *   ...
 */
extern "C" void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef* htim) {
    if (htim->Instance != TIM2) return;                    // Only handle TIM2
    if (htim->Channel != HAL_TIM_ACTIVE_CHANNEL_2) return; // Only handle CH2 (ADC trigger)

    static bool is_count_up = true;  // Toggle flag, starts true (first match on UP)
    
    if (is_count_up) {
        ADC_START_Callback();  // Sample ADC on count UP (before overflow, pins LOW)
    }
    
    is_count_up = !is_count_up;  // Toggle for next match
}

extern "C" void DMA1_Channel1_IRQHandler(void) {
    // Check whether DMA transfer complete caused the DMA interruption
    if (LL_DMA_IsActiveFlag_TC1(DMA1) == 1) {
        LL_DMA_ClearFlag_TC1(DMA1);  // Clear flag DMA transfer complete
        ADC_READY_Callback();
    }

    // Check whether DMA half transfer caused the DMA interruption
    if (LL_DMA_IsActiveFlag_HT1(DMA1) == 1)
        LL_DMA_ClearFlag_HT1(DMA1);  // Clear flag DMA half transfer
}

#endif  // NVIC_RADIX_H
