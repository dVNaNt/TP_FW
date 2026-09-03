/**
 * @file bootloader_jump.h
 * @brief Software entry into the built-in STM32G4 system-memory USB bootloader
 *        (ROM DFU) triggered by holding the SW1 push-button (PC14) for > 3 s.
 *
 * The CLN17 v3 board has no access to the BOOT0/NBOOT1 straps, so the jump
 * into the factory bootloader is purely software:
 *
 * 1. Every active peripheral is brought into a safe state:
 *    - TIM1 PWM outputs disconnected and MOE cleared (bridge = Hi-Z),
 *    - DRV_EN (PB11) driven LOW (gate driver disabled),
 *    - LEDs switched OFF.
 * 2. Interrupts masked, SysTick stopped, all NVIC channels disabled/purged.
 * 3. Clock tree returned to reset state (HSI 16 MHz, PLL/HSE/HSI48 off) via
 *    HAL_RCC_DeInit(), all peripherals reset via HAL_DeInit().
 * 4. All GPIO ports reconfigured to analog inputs with no pull resistors,
 *    except PA13/PA14 (SWDIO/SWCLK) which are kept so a debugger can still
 *    attach after a reboot.
 * 5. Stack pointer and return address are loaded from the vector table at
 *    0x1FFF0000 (STM32G474 system memory, see AN2606) and the bootloader
 *    image is entered. From there the USB FS device DFU interface is served
 *    on PA11/PA12 exactly like a hardware BOOT0 entry.
 *
 * Button handling is interrupt-driven: PC14 -> EXTI line 14 with BOTH-edge
 * triggering. The ISR only latches a millis() timestamp on the falling
 * edge (one subtraction = "time between fronts"), and poll() performs a
 * single comparison to decide when the 3 s hold has elapsed.
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#ifndef BOOTLOADER_JUMP_CLN234_H
#define BOOTLOADER_JUMP_CLN234_H

#include <Arduino.h>

#include "stm32g4xx_hal.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_exti.h"
#include "stm32g4xx_ll_gpio.h"
#include "stm32g4xx_ll_system.h"

#include "target.h"  // PINOUT definitions (SW1 = PC14)

namespace BOOT {

/// Base address of the STM32G474 system-memory ROM bootloader (AN2606).
constexpr uint32_t SYSTEM_MEMORY_BASE = 0x1FFF0000UL;

/// Continuous press time required before entering the bootloader.
constexpr uint32_t HOLD_THRESHOLD_MS = 3000UL;

/// Button pin: SW1 on PC14, configured as input with pull-up (active LOW).
constexpr uint32_t BUTTON_PORT_PIN = LL_GPIO_PIN_14;

/** Milliseconds between two consecutive polls of the hold-time check. */
constexpr uint32_t POLL_INTERVAL_MS = 10;

/** Contact-bounce filter window applied to EXTI edges. */
constexpr uint32_t DEBOUNCE_MS = 30UL;

// --- Shared state written by the EXTI ISR, read by poll() -----------------
volatile bool g_pressed = false;          // true between falling/rising edge
volatile uint32_t g_press_start_ms = 0;   // timestamp of the accepted falling
                                          // edge ("super fast" timekeeping:
                                          // one subtraction in the ISR)
volatile uint32_t g_last_edge_ms = 0;     // debounce reference

/**
 * @brief Configures SW1 (PC14) as interrupt-driven input with internal pull-up.
 *
 * PC14 is routed through SYSCFG to EXTI line 14 with BOTH-edge triggering
 * (falling = press, rising = release). The ISR itself only records a
 * millisecond timestamp, so the time between edges costs essentially one
 * subtraction. NVIC priority is set below the real-time control interrupts
 * (SPI1 = 6 / TIM1_CC = 3).
 */
inline void init_button() {
    // GPIO: plain digital input with pull-up (idle HIGH, pressed = LOW).
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);

    LL_GPIO_InitTypeDef btn = {0};
    btn.Pin = BUTTON_PORT_PIN;
    btn.Mode = LL_GPIO_MODE_INPUT;
    btn.Speed = LL_GPIO_SPEED_FREQ_LOW;
    btn.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    btn.Pull = LL_GPIO_PULL_UP;
    LL_GPIO_Init(GPIOC, &btn);

    // Route PC14 -> EXTI14, both edges, unmasked.
    LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTC, LL_SYSCFG_EXTI_LINE6);
    LL_EXTI_EnableRisingTrig_0_31(LL_EXTI_LINE_14);  // release
    LL_EXTI_EnableFallingTrig_0_31(LL_EXTI_LINE_14); // press
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_14);
    LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_14);

    // Below the real-time control interrupts (SPI1 = 6 / TIM1_CC = 3).
    NVIC_SetPriority(EXTI15_10_IRQn,
                     NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 7, 0));
    NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/**
 * @brief True while SW1 is physically held down (pin reads LOW).
 */
inline bool button_pressed() {
    return ((GPIOC->IDR & BUTTON_PORT_PIN) == 0U);
}

/**
 * @brief Brings the whole chip into a safe, power-on-like state and jumps
 *        into the system-memory USB DFU bootloader. Never returns.
 */
[[noreturn]] inline void jump_to_system_bootloader() {
    // ------------------------------------------------------------------
    // 1. Motor stage safety: force the 3-phase bridge into Hi-Z and
    //    disable the gate-driver supply before touching anything else.
    // ------------------------------------------------------------------
    TIM1->CCER = 0;                    // disconnect all compare outputs
    TIM1->BDTR &= ~TIM_BDTR_MOE;       // kill master output enable
    TIM1->CR1 &= ~TIM_CR1_CEN;         // stop the PWM counter
    LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_11);  // DRV_EN = 0

    // LEDs off (active-low common anode: high level = dark)
    LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_1 | LL_GPIO_PIN_2);
    LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_4);

    // ------------------------------------------------------------------
    // 2. Interrupt context teardown.
    // ------------------------------------------------------------------
    __disable_irq();

    SysTick->CTRL = 0;                 // stop the Arduino/HAL tick source
    SysTick->LOAD = 0;
    SysTick->VAL = 0;

    // Purge every NVIC channel: disable + clear any pending request so the
    // bootloader starts from a clean interrupt landscape.
    for (uint32_t i = 0; i < 8U; ++i) {
        NVIC->ICER[i] = 0xFFFFFFFFUL;
        NVIC->ICPR[i] = 0xFFFFFFFFUL;
    }

    // ------------------------------------------------------------------
    // 3. Clock tree back to reset defaults (SYSCLK = HSI 16 MHz, PLL/HSE/
    //    HSI48 off, bus dividers = 1, flash latency re-tuned automatically),
    //    then hardware reset of every peripheral via the RCC reset regs.
    // ------------------------------------------------------------------
    HAL_RCC_DeInit();
    HAL_DeInit();

    // ------------------------------------------------------------------
    // 4. GPIO to a neutral analog state (no drive levels, no pulls) so no
    //    external circuitry sees undefined voltages inside the bootloader.
    //    PA13/PA14 are intentionally left alone to keep SWD alive.
    // ------------------------------------------------------------------
    GPIO_TypeDef* const ports[] = {GPIOA, GPIOB, GPIOC};
    for (GPIO_TypeDef* port : ports) {
        port->MODER = 0xFFFFFFFFUL;    // everything analog...
        port->OSPEEDR = 0x00000000UL;
        port->OTYPER = 0x00000000UL;
        port->PUPDR = 0x00000000UL;    // ...and floating
        port->AFR[0] = 0x00000000UL;
        port->AFR[1] = 0x00000000UL;
    }
    // Restore SWD function on PA13 (SWDIO) / PA14 (SWCLK).
    LL_GPIO_InitTypeDef swd = {0};
    swd.Pin = LL_GPIO_PIN_13 | LL_GPIO_PIN_14;
    swd.Mode = LL_GPIO_MODE_ALTERNATE;
    swd.Pull = LL_GPIO_PULL_NO;
    swd.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    swd.Alternate = LL_GPIO_AF_0;      // AF0 = SWJ
    LL_GPIO_Init(GPIOA, &swd);

    // ------------------------------------------------------------------
    // 5. Enter the ROM bootloader: fetch initial SP and reset vector from
    //    the system-memory vector table and branch there.
    // ------------------------------------------------------------------
    const uint32_t boot_sp =
        *(volatile const uint32_t*)(SYSTEM_MEMORY_BASE);
    const uint32_t boot_pc =
        *(volatile const uint32_t*)(SYSTEM_MEMORY_BASE + 4U);

    SCB->VTOR = SYSTEM_MEMORY_BASE;    // bootloader manages its own vectors
    __set_CONTROL(0);                  // privileged, MSP (defensive; no RTOS)
    __DSB();
    __ISB();

    __set_MSP(boot_sp);                // switch to bootloader stack
    ((void (*)(void))boot_pc)();       // never comes back

    while (true) {
    }                                  // unreachable, silences warnings
}

/**
 * @brief Non-blocking hold-time check: call this frequently (setup()/loop()
 *        and long busy-wait loops). All heavy lifting happens in the EXTI
 *        ISR; here it is a single compare against the timestamp captured on
 *        the falling edge. If SW1 stays pressed for HOLD_THRESHOLD_MS the
 *        firmware enters the USB DFU bootloader.
 */
inline void poll() {
    if (g_pressed &&
        (millis() - g_press_start_ms) >= HOLD_THRESHOLD_MS) {
        jump_to_system_bootloader();   // [[noreturn]]
    }
}

}  // namespace BOOT

/**
 * @brief EXTI handler for PC14 (EXTI line 14, shared vector EXTI15_10).
 *
 * Deliberately minimal: clear flag, grab millis(), apply a small debounce
 * window and record the press timestamp. Time between the edges is measured
 * with one subtraction — no timers, no state machines in interrupt context.
 */
extern "C" void EXTI15_10_IRQHandler(void) {
    if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_14)) {
        LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_14);

        const uint32_t now = millis();

        // Contact-bounce filter: ignore edges closer than DEBOUNCE_MS.
        if ((now - BOOT::g_last_edge_ms) < BOOT::DEBOUNCE_MS) {
            return;
        }
        BOOT::g_last_edge_ms = now;

        // Latch the level right away: falling edge = press started.
        BOOT::g_pressed = ((GPIOC->IDR & BOOT::BUTTON_PORT_PIN) == 0U);
        if (BOOT::g_pressed) {
            BOOT::g_press_start_ms = now;
        }
    }
}

#endif  // BOOTLOADER_JUMP_CLN234_H