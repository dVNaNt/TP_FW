#ifndef CLOCK_SETUP_CLN17V3_H
#define CLOCK_SETUP_CLN17V3_H

#include "stm32g4xx_hal.h"

/**
 * @brief System Clock Configuration
 * System Clock source: PLL (HSE)
 * SYSCLK(Hz): 170000000
 * HCLK(Hz): 170000000
 * AHB Prescaler: 1
 * APB1 Prescaler: 1
 * APB2 Prescaler: 1
 * HSE Frequency(Hz): 12000000
 * HSI48: ON (for USB)
 * PLL_M: 3
 * PLL_N: 85
 * PLL_P: 2
 * PLL_Q: 2
 * PLL_R: 2
 */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    // Configure voltage scaling for 170 MHz operation
    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

    // Initialize HSE oscillator, HSI48 (for USB), and PLL
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48 | RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;           // Enable external 12 MHz crystal
    RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;       // Enable HSI48 for USB
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV3;  // 12 MHz / 3 = 4 MHz (PLL input)
    RCC_OscInitStruct.PLL.PLLN = 85;             // 4 MHz * 85 = 340 MHz (VCO)
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;  // 340 MHz / 2 = 170 MHz (PLLP output)
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;  // 340 MHz / 2 = 170 MHz (PLLQ output)
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;  // 340 MHz / 2 = 170 MHz (SYSCLK)
    
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    // Initialize CPU, AHB and APB buses clocks
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;   // 170 MHz
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;    // 170 MHz
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;    // 170 MHz

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
        Error_Handler();
    }
    
    // Update SystemCoreClock variable
    SystemCoreClockUpdate();
}

#endif  // CLOCK_SETUP_CLN17V3_H
