#ifndef TIMER_PWM_CLN17V3_H
#define TIMER_PWM_CLN17V3_H

#include <Arduino.h>
#include "config.h"
#include "stm32g4xx_hal.h"
#include "stm32g4xx_ll_gpio.h"
#include "target.h"

// PWM timer parameters
constexpr uint16_t PWM_TIMER_PERIOD = 4722;      // Timer period (ARR)
constexpr uint16_t PWM_TIMER_FREQ_HZ = 18000;    // PWM frequency in Hz
constexpr int32_t TIMER_CLOCK_HZ = 170UL;        // System clock 170 MHz

// Dead time configuration (in timer ticks)
constexpr uint8_t PWM_DEADTIME_TICKS = 34;  // ~200ns at 170MHz

// ADC trigger delay in timer ticks
volatile int16_t ADC_DELAY_TICKS = 220;
volatile int16_t ADC_TRIGGER_POINT = 0;

TIM_HandleTypeDef htim1;

/**
 * @brief Initialize TIM1 for complementary PWM
 */
void MX_TIM1_Init(void) {
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

    // TIM1 base configuration
    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 0;  // No prescaler (170 MHz)
    htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED3;  // Center-aligned mode 3
    htim1.Init.Period = PWM_TIMER_PERIOD;  // ARR value
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;  // No repetition
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    
    if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
        Error_Handler();
    }

    // Clock source configuration
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
        Error_Handler();
    }

    // PWM mode initialization
    if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
        Error_Handler();
    }

    // Master configuration
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }

    // PWM channel configuration (CH1, CH2, CH3)
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;  // Initial duty cycle = 0%
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;

    // Configure all three PWM channels
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) {
        Error_Handler();
    }

    // Configure CH4 for ADC trigger
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = ADC_TRIGGER_POINT;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK) {
        Error_Handler();
    }

    // Break and dead time configuration
    sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
    sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime = PWM_DEADTIME_TICKS;  // Dead time
    sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTimeConfig.BreakFilter = 0;
    sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
    sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
    sBreakDeadTimeConfig.Break2Filter = 0;
    sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
    
    if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK) {
        Error_Handler();
    }
}

/**
 * @brief Start TIM1 PWM outputs
 */
void MX_TIM1_Start() {
    // Start complementary PWM on channels 1, 2, 3
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);  // Start CH1N
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);     // Start CH1
    
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);  // Start CH2N
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);     // Start CH2
    
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);  // Start CH3N
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);     // Start CH3
    
    // Start CH4 with interrupts for ADC trigger
    HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_4);

    // Clear pending Update flag
    __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);
    
    // Clear CH4 compare flag
    __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_CC4);

    // Start timer with Update interrupt
    HAL_TIM_Base_Start_IT(&htim1);
    
    // Enable main output
    __HAL_TIM_MOE_ENABLE(&htim1);
}

/**
 * @brief Set PWM duty cycle values for 3 phases
 * @param pwmValues Array of 3 duty cycle values (-PWM_TIMER_PERIOD to +PWM_TIMER_PERIOD)
 *                  Positive: normal PWM mode (high-side switching)
 *                  Negative: braking mode (low-side switching, high-side OFF)
 *                  INT16_MIN = disable phase (all outputs LOW)
 */
void TIM1_Set_PWM_Values(const int16_t pwmValues[3]) {
    // Process each phase
    for (uint8_t phase = 0; phase < 3; phase++) {
        uint32_t channel;  // TIM channel identifier
        
        // Map phase index to TIM channel
        if (phase == 0) channel = TIM_CHANNEL_1;       // Phase A
        else if (phase == 1) channel = TIM_CHANNEL_2;  // Phase B
        else channel = TIM_CHANNEL_3;                  // Phase C
        
        int16_t duty = pwmValues[phase];  // Get duty cycle value
        
        if (duty == INT16_MIN) {
            // Timer inactive mode: all FETs OFF (both high-side and low-side disabled)
            __HAL_TIM_SET_COMPARE(&htim1, channel, 0);  // Set compare to 0
            TIM1->CCER &= ~(0x5 << (phase * 4));        // Disable both CHx and CHxN outputs (all FETs OFF)
        } 
        else if (duty < 0) {
            // Braking mode: high-side OFF, low-side PWM with corrected duty
            int16_t abs_duty = -duty;  // Convert to positive value
            if (abs_duty > PWM_TIMER_PERIOD) abs_duty = PWM_TIMER_PERIOD;  // Clamp to max
            
            __HAL_TIM_SET_COMPARE(&htim1, channel, abs_duty);  // Set PWM duty cycle
            
            // Configure outputs: CHx=OFF (high-side), CHxN=ON (low-side)
            uint32_t ccer_mask = (0x5 << (phase * 4));  // Mask for CHxE and CHxNE bits
            TIM1->CCER &= ~ccer_mask;                   // Clear both enable bits
            TIM1->CCER |= (0x4 << (phase * 4));         // Enable only CHxN (low-side)
        } 
        else {
            // Normal PWM mode: complementary outputs active
            if (duty > PWM_TIMER_PERIOD) duty = PWM_TIMER_PERIOD;  // Clamp to max
            
            __HAL_TIM_SET_COMPARE(&htim1, channel, duty);  // Set PWM duty cycle
            
            // Enable both CHx (high-side) and CHxN (low-side) outputs
            TIM1->CCER |= (0x5 << (phase * 4));  // Set CHxE and CHxNE bits
        }
    }
    
    // Find maximum absolute PWM value for ADC trigger calculation
    int16_t max_pwm = 0;  // Initialize to zero
    for (uint8_t i = 0; i < 3; i++) {
        int16_t abs_val = (pwmValues[i] < 0) ? -pwmValues[i] : pwmValues[i];  // Get absolute value
        if (abs_val != INT16_MIN && abs_val > max_pwm) {  // Update max if valid and larger
            max_pwm = abs_val;
        }
    }
    
    if (max_pwm == 0) max_pwm = PWM_TIMER_PERIOD;  // Default to full period if all disabled
    
    // Calculate ADC trigger point: max PWM + delay
    int16_t adc_trigger = max_pwm + ADC_DELAY_TICKS;
    
    // Clamp to valid range [0, PWM_TIMER_PERIOD-1]
    if (adc_trigger < 0) adc_trigger = 0;
    if (adc_trigger > PWM_TIMER_PERIOD) adc_trigger = PWM_TIMER_PERIOD - 1;

    // Set ADC trigger on channel 4
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, (uint16_t)adc_trigger);
}

/**
 * @brief Update ADC trigger point when frequency changes
 */
void TIM1_Update_ADC_Trigger(uint16_t new_period, uint16_t new_freq_hz, int16_t adc_delay_ticks = ADC_DELAY_TICKS) {
    ADC_DELAY_TICKS = adc_delay_ticks;  // Update delay
    int16_t new_trigger_point = 0;
    
    // Update timer period
    __HAL_TIM_SET_AUTORELOAD(&htim1, new_period);
    
    // Update ADC trigger point on channel 4
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, new_trigger_point);
}

/**
 * @brief TIM1 IRQ handler
 */
extern "C" void TIM1_UP_TIM16_IRQHandler(void) {
    HAL_TIM_IRQHandler(&htim1);
}

/**
 * @brief TIM1 CC IRQ handler
 */
extern "C" void TIM1_CC_IRQHandler(void) {
    HAL_TIM_IRQHandler(&htim1);
}

/**
 * @brief TIM1 MSP initialization
 */
extern "C" void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base) {
    if (htim_base->Instance == TIM1) {
        __HAL_RCC_TIM1_CLK_ENABLE();  // Enable TIM1 clock
    }
}

/**
 * @brief Initialize PWM system
 */
void PWM_Init() {
    MX_TIM1_Init();  // Initialize TIM1
    MX_TIM1_Start(); // Start TIM1 PWM
}

#endif  // TIMER_PWM_CLN17V3_H
