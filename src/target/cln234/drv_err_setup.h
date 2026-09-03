/**
 * @file drv_err_setup.h
 * @brief DRV8353 nFAULT (DRV_ERR) input handling for CLN234.
 *
 * The CLN234 delegates hardware overcurrent/overvoltage detection to the
 * DRV8353 smart gate driver (VDS_OCP / SEN_OCP / UVLO / OTSD), which
 * reports any fault on the open-drain nFAULT output wired to PB1
 * (DRV_ERR, active low, external pull-up). There are no MCU comparator
 * channels on this board, so unlike the CLN17 v3 no COMP/DAC setup is
 * needed.
 *
 * PB1 is routed through SYSCFG to EXTI line 1 with BOTH-edge triggering:
 * - Falling edge (pin LOW)  : set the external fault (safe state callback
 *                             registered by tunepulse.h forces Hi-Z)
 * - Rising edge (pin HIGH)  : clear the external fault
 *
 * The clear-on-release behaviour matches the comparator fault manager in
 * lib.h (FaultManager clears the error once the hardware condition is
 * gone and the recovery timer elapses).
 *
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#ifndef DRV_ERR_SETUP_CLN234_H
#define DRV_ERR_SETUP_CLN234_H

#include <Arduino.h>
#include "stm32g4xx_hal.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_exti.h"
#include "stm32g4xx_ll_gpio.h"
#include "stm32g4xx_ll_system.h"

/**
 * @brief Configure PB1 (DRV_ERR) as EXTI input with pull-up and enable
 *        the interrupt. Call once during bring-up, after IO_Init().
 */
inline void DRV_ERR_Init() {
    // Enable the SYSCFG clock (EXTI line routing) if not already on
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);

    // GPIO: digital input with pull-up (idle HIGH, fault = LOW)
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_1, LL_GPIO_MODE_INPUT);
    LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_1, LL_GPIO_PULL_UP);

    // Route PB1 -> EXTI line 1 (STM32G4 SYSCFG external interrupt selector)
    LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTB, LL_SYSCFG_EXTI_LINE1);
    LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_1);
    LL_EXTI_EnableRisingTrig_0_31(LL_EXTI_LINE_1);   // Fault released
    LL_EXTI_EnableFallingTrig_0_31(LL_EXTI_LINE_1);  // Fault asserted

    // Priority: below the real-time interrupts (TIM1_CC=3, DMA=5), above USB
    NVIC_SetPriority(EXTI1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 8, 0));
    NVIC_EnableIRQ(EXTI1_IRQn);
}

/**
 * @brief EXTI line 1 handler (PB1 = DRV_ERR, active low).
 *
 * The pin level is sampled directly: LOW = fault, HIGH = released.
 */
extern "C" void EXTI1_IRQHandler(void) {
    if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_1)) {
        LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_1);  // Clear pending flag

        // Active-low request: pin LOW = fault set, pin HIGH = clear
        const bool is_fault = (LL_GPIO_IsInputPinSet(GPIOB, LL_GPIO_PIN_1) == 0U);
        TP.driver_phy.set_fault().external(is_fault);
    }
}

#endif  // DRV_ERR_SETUP_CLN234_H