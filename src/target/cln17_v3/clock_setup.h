#ifndef CLOCK_SETUP_CLN17V3_H
#define CLOCK_SETUP_CLN17V3_H

#include <Arduino.h>
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
 * PLL_M: 3
 * PLL_N: 85
 * PLL_P: 2
 * PLL_Q: 2
 * PLL_R: 2
 */
extern "C" void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    // Configure voltage scaling
    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

    // --- Oscillators ---
    // Prefer HSE (12MHz), but don't brick the MCU if HSE is absent:
    // fall back to PLL from HSI (16MHz). Always enable HSI48 for USB.

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48 | RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV3;  // 12 MHz / 3 = 4 MHz
    RCC_OscInitStruct.PLL.PLLN = 85;             // 4 MHz * 85 = 340 MHz
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;  // 340 MHz / 2 = 170 MHz
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;  // 340 MHz / 2 = 170 MHz (SYSCLK)

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        // Fallback: HSI(16MHz) -> PLLM=4 gives 4MHz PLL input, same PLLN/PLLR for 170MHz
        RCC_OscInitStruct = {};
        RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48 | RCC_OSCILLATORTYPE_HSI;
        RCC_OscInitStruct.HSIState = RCC_HSI_ON;
        RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
        RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
        RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
        RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
        RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;  // 16 MHz / 4 = 4 MHz
        RCC_OscInitStruct.PLL.PLLN = 85;             // 4 MHz * 85 = 340 MHz
        RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;  // 340 MHz / 2 = 170 MHz
        RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
        RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;  // 340 MHz / 2 = 170 MHz (SYSCLK)

        if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
            Error_Handler();
        }
    }

    // Initialize CPU, AHB and APB buses clocks
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;   // 170 MHz
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;    // 170 MHz
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;    // 170 MHz

    // genericSTM32G431CB core uses FLASH_LATENCY_4 at 170MHz in BOOST mode.
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
        Error_Handler();
    }

    SystemCoreClockUpdate();
}

#endif  // CLOCK_SETUP_CLN17V3_H
