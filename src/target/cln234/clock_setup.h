/**
 * @file clock_setup.h
 * @brief System Clock Configuration for the CLN234 (STM32G474CEU6).
 *
 * The CLN234 board does not populate an external high-speed crystal (the
 * HSE pins are not routed; PC14/PC15 are used as SW1/DRV_CS, so there is
 * no LSE either). The clock tree therefore starts from the internal
 * HSI16 oscillator:
 *
 * - PLL source   : HSI16
 * - PLL input    : 16 MHz / 4 (PLLM) = 4 MHz
 * - VCO          : 4 MHz * 85 (PLLN) = 340 MHz
 * - SYSCLK (PLLR): 340 MHz / 2 = 170 MHz
 * - HSI48        : ON (USB clock source)
 *
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#ifndef CLOCK_SETUP_CLN234_H
#define CLOCK_SETUP_CLN234_H

#include "stm32g4xx_hal.h"

/**
 * @brief System Clock Configuration (HSI-based, 170 MHz) - non-fatal.
 *
 * Status is reported through SystemClock_Status instead of calling
 * Error_Handler(), so a bring-up failure is visible to the self-test
 * rather than silently hanging the board.
 *
 * Status bits:
 *   0x0 - configuration applied successfully
 *   0x1 - PWR voltage scaling failed
 *   0x2 - oscillator/PLL configuration failed
 *   0x4 - bus clock configuration failed
 *   0x8 - skipped: 170 MHz PLL-from-HSI already running (warm restart)
 */
inline uint32_t SystemClock_Status = 0xFFFFFFFFUL;

void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    SystemClock_Status = 0;  // Assume success until a step fails

    // Skip reconfiguration when the PLL is already the SYSCLK source at
    // 170 MHz (e.g. restart from a debugger without a reset).
    if ((SystemCoreClock == 170000000U) &&
        (READ_BIT(RCC->CFGR, RCC_CFGR_SWS) == RCC_SYSCLKSOURCE_PLLCLK) &&
        (READ_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLLSRC) == RCC_PLLSOURCE_HSI)) {
        SystemClock_Status = 0x8UL;  // Skipped: already configured
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;  // Enable the DWT block
        DWT->CYCCNT = 0;                                 // Zero the counter
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;             // Start the cycle counter
        SystemCoreClockUpdate();
        return;
    }

    // Configure voltage scaling for 170 MHz operation
    if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST) != HAL_OK) {
        SystemClock_Status |= 0x1UL;  // Voltage scaling failure (non-fatal)
    }

    // Initialize HSI16 (PLL source), HSI48 (USB) and PLL
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSI48;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;       // Enable the internal 16 MHz RC
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;   // Enable HSI48 for USB
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;   // 16 MHz / 4 = 4 MHz (PLL input)
    RCC_OscInitStruct.PLL.PLLN = 85;              // 4 MHz * 85 = 340 MHz (VCO)
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;   // 340 MHz / 2 = 170 MHz (PLLP output)
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;   // 340 MHz / 2 = 170 MHz (PLLQ output)
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;   // 340 MHz / 2 = 170 MHz (SYSCLK)

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        SystemClock_Status |= 0x2UL;  // Oscillator/PLL failure (non-fatal)
    }

    // Initialize CPU, AHB and APB buses clocks
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;  // 170 MHz
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;   // 170 MHz
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;   // 170 MHz

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
        SystemClock_Status |= 0x4UL;  // Bus clock failure (non-fatal)
    }

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;  // Enable the DWT block
    DWT->CYCCNT = 0;                                 // Zero the counter
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;             // Start the cycle counter

    // Update SystemCoreClock variable
    SystemCoreClockUpdate();
}

#endif  // CLOCK_SETUP_CLN234_H
