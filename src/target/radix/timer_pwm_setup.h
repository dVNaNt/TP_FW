#ifndef TIMER_PWM_H
#define TIMER_PWM_H

#include <Arduino.h>

#include "stm32g4xx_hal.h"
#include "stm32g4xx_ll_gpio.h"

// PWM timer parameters (use guards to avoid redefinition from lib.h)
#ifndef PWM_TIMER_PERIOD
constexpr uint16_t PWM_TIMER_PERIOD = 4250;      // Timer period (ARR)
#endif

#ifndef PWM_TIMER_FREQ_HZ
constexpr uint16_t PWM_TIMER_FREQ_HZ = 20000;    // PWM frequency in Hz
#endif

// System bus frequency (170 MHz for STM32G431)
constexpr int32_t TIMER_CLOCK_HZ = 170UL;


// ADC trigger delay in timer ticks (positive value, default 220 ticks)
volatile int16_t ADC_DELAY_TICKS = 260;           // ADC delay in timer ticks
volatile int16_t ADC_TRIGGER_POINT = 0;           // ADC trigger point (calculated from max PWM + delay)

TIM_HandleTypeDef htim2;

void MX_TIM2_Init(void) {
    // Init timer
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 0;
    htim2.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED3;  // Update on both OF and UF
    htim2.Init.Period = PWM_TIMER_PERIOD;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
        Error_Handler();
    }

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) {
        Error_Handler();
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }

    // Настройка каналов ШИМ (CH1, CH3, CH4)
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK) {
        Error_Handler();
    }

    // Настройка CH2 для генерации прерываний ADC
    // CH2 используется для триггера ADC в момент минимальной скважности минус задержка
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = ADC_TRIGGER_POINT;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
        Error_Handler();
    }
}

void MX_TIM2_Start() {
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);  // Start PWM on channel 1
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);  // Start PWM on channel 3
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);  // Start PWM on channel 4
    
    // Start CH2 with interrupts for ADC trigger
    HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_2);

    // Clear any pending Update flag before enabling interrupt
    // This prevents false trigger at startup (HAL may set UG bit during init)
    __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);
    
    // Clear CH2 compare flag to ensure first ADC trigger is on count UP
    __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_CC2);

    // Now start timer with Update interrupt - first real event will be overflow (CNT=ARR)
    HAL_TIM_Base_Start_IT(&htim2);
}

// 3-phase PWM: CH1(PA0), CH3(PA2), CH4(PB11). CH2(PA1) - ADC trigger.
// Mapping: voltages[0]->PB11(TIM_CH4)->EN_CH4, voltages[1]->PA0(TIM_CH1)->EN_CH1, voltages[2]->PA2(TIM_CH3)->EN_CH2
void TIM2_Set_PWM_Values(const int16_t pwmValues[3]) {
    // voltages[0] -> TIM_CH4 (PB11) -> EN_CH1 (PB10)
    if (pwmValues[0] == INT16_MIN) {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, PWM_TIMER_PERIOD);  // 100% скважность
        LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_10);  // EN_CH1 = 0
    } else {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, (pwmValues[0] < 0) ? 0 : pwmValues[0]);
        LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_10);  // EN_CH1 = 1
    }
    
    // voltages[1] -> TIM_CH1 (PA0) -> EN_CH2 (PB2)
    if (pwmValues[1] == INT16_MIN) {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, PWM_TIMER_PERIOD);  // 100% скважность
        LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_2);  // EN_CH2 = 0
    } else {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (pwmValues[1] < 0) ? 0 : pwmValues[1]);
        LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_2);  // EN_CH2 = 1
    }
    
    // voltages[2] -> TIM_CH3 (PA2) -> EN_CH4 (PA6)
    if (pwmValues[2] == INT16_MIN) {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, PWM_TIMER_PERIOD);  // 100% скважность
        LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_6);  // EN_CH4 = 0
    } else {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, (pwmValues[2] < 0) ? 0 : pwmValues[2]);
        LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_6);  // EN_CH4 = 1
    }
    
    // Find maximum PWM value from three channels
    int16_t max_pwm = pwmValues[0];
    if (pwmValues[1] > max_pwm) max_pwm = pwmValues[1];
    if (pwmValues[2] > max_pwm) max_pwm = pwmValues[2];

    if (max_pwm == INT16_MIN) max_pwm = PWM_TIMER_PERIOD;
    
    // Calculate ADC trigger point: max PWM + delay (positive delay)
    int16_t adc_trigger = max_pwm + ADC_DELAY_TICKS;
    
    // Clamp to valid range
    if (adc_trigger < 0) adc_trigger = 0;
    if (adc_trigger > PWM_TIMER_PERIOD) adc_trigger = PWM_TIMER_PERIOD - 1;


    // Set ADC trigger on channel 2
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, (uint16_t)adc_trigger);
}

// Update ADC trigger point when frequency changes
void TIM2_Update_ADC_Trigger(uint16_t new_period, uint16_t new_freq_hz, int16_t adc_delay_ticks = ADC_DELAY_TICKS) {
    // Update ADC delay ticks
    ADC_DELAY_TICKS = adc_delay_ticks;
    int16_t new_trigger_point = 0;
    
    // Update timer period
    __HAL_TIM_SET_AUTORELOAD(&htim2, new_period);
    
    // Update ADC trigger point on channel 2
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, new_trigger_point);
}

extern "C" void TIM2_IRQHandler(void) {
    HAL_TIM_IRQHandler(&htim2);
}

extern "C" void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base) {
    if (htim_base->Instance == TIM2) {
        /* Peripheral clock enable */
        __HAL_RCC_TIM2_CLK_ENABLE();
    }
}

void PWM_Init() {
    MX_TIM2_Init();
    MX_TIM2_Start();
}

#endif  // TIMER_PWM_H