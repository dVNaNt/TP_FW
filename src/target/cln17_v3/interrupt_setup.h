#ifndef NVIC_CLN17V3_H
#define NVIC_CLN17V3_H

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
 * PWM Timer Execution Pattern (TIM1 Center-Aligned Mode 3)
 * =========================================================
 * 
 * Timer cycle:     0 ────────► ARR ────────► 0
 *                  │    UP      │    DOWN    │
 *                  UF          OF           UF
 * 
 * PWM output (complementary mode with dead time):
 *   - CHx  HIGH when CNT < CCRx (high-side on)
 *   - CHxN HIGH when CNT >= CCRx + deadtime (low-side on)
 * 
 * Event sequence per full cycle:
 * ┌─────────────────────────────────────────────────────────────────┐
 * │  CNT:  0 ──► CCR4 ──────────► ARR ──► CCR4 ──────────► 0        │
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
extern void ADC_START_Callback();   // Called before overflow (CH4 match on UP) - sample ADC
extern void ADC_READY_Callback();   // Called from DMA complete
extern void POSITION_Callback();    // Called from SPI complete (encoder data ready)

/**
 * @brief Initialize NVIC interrupts
 */
void NVIC_Init() {
    // DMA interrupt init (ADC1 RX - Channel 5)
    NVIC_SetPriority(DMA1_Channel5_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 3, 0));
    NVIC_EnableIRQ(DMA1_Channel5_IRQn);

    // DMA interrupt init (ADC2 RX - Channel 6)
    NVIC_SetPriority(DMA1_Channel6_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 3, 0));
    NVIC_EnableIRQ(DMA1_Channel6_IRQn);

    // SPI1 interrupt init
    NVIC_SetPriority(SPI1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0));
    NVIC_EnableIRQ(SPI1_IRQn);

    // TIM1 Update interrupt init
    HAL_NVIC_SetPriority(TIM1_UP_TIM16_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);

    // TIM1 Capture Compare interrupt init
    HAL_NVIC_SetPriority(TIM1_CC_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);
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
    if (htim->Instance != TIM1) return;  // Only handle TIM1 (PWM timer)

    static bool is_overflow = true;  // Toggle flag, starts true (first event is OF)
    
    if (is_overflow) {
        PWM_Callback();       // Update PWM duty at overflow (CNT=ARR)
    } else {
        ENCODER_Callback();   // Trigger encoder read at underflow (CNT=0)
    }
    
    is_overflow = !is_overflow;  // Toggle for next event
}

/**
 * Output Compare match callback (CH4 triggers when CNT == CCR4)
 * 
 * CH4 match occurs TWICE per full cycle (on UP and DOWN count)
 * ADC sampling happens on count UP, just before overflow
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
    if (htim->Instance != TIM1) return;                    // Only handle TIM1
    if (htim->Channel != HAL_TIM_ACTIVE_CHANNEL_4) return; // Only handle CH4 (ADC trigger)

    static bool is_count_up = true;  // Toggle flag, starts true (first match on UP)
    
    if (is_count_up) {
        ADC_START_Callback();  // Sample ADC on count UP (before overflow)
    }
    
    is_count_up = !is_count_up;  // Toggle for next match
}

#endif  // NVIC_CLN17V3_H
