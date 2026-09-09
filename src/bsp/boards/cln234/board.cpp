/**
 * @file board.cpp
 * @brief The single translation unit of the CLN234 (STM32G474CE) board.
 *
 * Everything that must exist EXACTLY ONCE in the image lives here rather than in
 * a header: the interrupt vectors, the weak-override callbacks, and the global
 * drive object. Every other board file is a header of inline definitions that
 * this unit is the sole includer of.
 *
 * Why this file exists at all: the firmware used to be a single translation unit
 * rooted at main.cpp, which meant non-inline function bodies and mutable objects
 * could sit in headers without the linker ever noticing. Adding any second .cpp
 * would have produced duplicate symbols. Concentrating the unique definitions
 * here makes the ownership explicit and lets the peripheral drivers become a
 * shared, board-independent layer.
 *
 * Vectors are grouped by the interrupt priority they run at, because on this
 * drive the priority IS the contract: the fault comparators must be able to cut
 * the bridge underneath the control pass, and the control pass must not be able
 * to delay the ADC trigger that feeds it.
 *
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#include "run.h"  // Board application body; transitively pulls in every peripheral header

// ============================================================================
// Global drive object
// ============================================================================

/**
 * @brief The one motor-control application object for this board.
 *
 * Declared extern in tunepulse.h and defined here, so that the object has a
 * single owning translation unit. Constructed from the board constants in
 * target.h; see tunepulse.h for what each argument means.
 */
TunePulseHeart TP(PWM_TIMER_FREQ_HZ,
                  PWM_TIMER_PERIOD,
                  SYS_PARAMS::MAX_SUPPLY_FS,
                  SYS_PARAMS::IA_MAX_CURRENT_FS,
                  SYS_PARAMS::IB_MAX_CURRENT_FS,
                  SYS_PARAMS::IC_MAX_CURRENT_FS);

// ============================================================================
// Interrupt vectors
// ============================================================================

// ---------------------------------------------------------------------------
// From adc_setup.h
// ---------------------------------------------------------------------------

extern "C" void DMA1_Channel5_IRQHandler(void) { // Interrupt vector for DMA Channel 5
    if (LL_DMA_IsActiveFlag_TC5(DMA1)) { // Check if the Transfer Complete flag is triggered
        // OPTIMIZATION: Clear global flag (TC, TE, HT) in exactly 1 hardware clock cycle
        LL_DMA_ClearFlag_GI5(DMA1); // Apply single-cycle flag clearance
        ADC_READY_Callback(); // Trigger the main FOC mathematical loop
    } // End of flag check
} // End of DMA IRQ Handler


// ---------------------------------------------------------------------------
// From bsp/stm32g4/tim1_pwm.h
// ---------------------------------------------------------------------------

extern "C" void TIM1_CC_IRQHandler(void) {  // Define the hardware interrupt vector for Capture/Compare events
    if (TIM1->SR & TIM_SR_CC4IF) {          // Check purely if the Channel 4 (ADC trigger) interrupt flag is active
        TIM1->SR = ~TIM_SR_CC4IF;           // Clear the interrupt flag immediately by writing 0 to its bit
        FOC_TimTrig_Callback();             // Directly call our optimized callback, bypassing HAL completely
    }  // End of CC4IF check
}  // End of TIM1_CC_IRQHandler function


// ---------------------------------------------------------------------------
// From spi_setup.h
// ---------------------------------------------------------------------------

extern "C" void DMA1_Channel1_IRQHandler(void) {
    const uint32_t isr = DMA1->ISR;

    // Transfer error: finish the SPI transaction safely.
    if (isr & DMA_ISR_TEIF1) {
        DMA1->IFCR = DMA_IFCR_CTEIF1 | DMA_IFCR_CGIF1;

        DMA1_Channel1->CCR &= ~DMA_CCR_EN;
        DMA1_Channel2->CCR &= ~DMA_CCR_EN;

        while (SPI1->SR & SPI_SR_BSY) {
            __NOP();
        }

        SPI1_EncChipSelect(false);
        SPI1_TRANSFER_ACTIVE = false;
        return;
    }

    if (isr & DMA_ISR_TCIF1) {
        DMA1->IFCR = DMA_IFCR_CGIF1;

        // RX DMA completed. Always wait for the final SPI edge before raising CS.
        while (SPI1->SR & SPI_SR_BSY) {
            __NOP();
        }

        SPI1_EncChipSelect(false);
        SPI1_TRANSFER_ACTIVE = false;

        MT6835_ParseResponse();

        // Called after POSITION_ENCODER_MT_RAW has been updated.
        POSITION_Callback();
    }
}


// ---------------------------------------------------------------------------
// From comp_setup.h
// ---------------------------------------------------------------------------

extern "C" void COMP1_2_3_IRQHandler(void) {
    // Check COMP1 (Phase B overcurrent) - EXTI line 21
    if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_21)) {
        LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_21);  // Clear pending flag

        TP.driver_phy.set_fault().overcurrent(true);  // Set fatal overcurrent error
        SerialUSB.println("B");
    }

    // Check COMP2 (Phase A overcurrent) - EXTI line 22
    if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_22)) {
        LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_22);  // Clear pending flag

        TP.driver_phy.set_fault().overcurrent(true);  // Set fatal overcurrent error
        SerialUSB.println("A");
    }

    // Check COMP3 (Overvoltage) - EXTI line 29
    if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_29)) {
        LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_29);  // Clear pending flag

        // Check if rising edge (fault) or falling edge (clear)
        if (LL_COMP_ReadOutputLevel(COMP3) == LL_COMP_OUTPUT_LEVEL_HIGH) {
            TP.driver_phy.set_fault().overvoltage(true);  // Set fatal overvoltage error
        } else {
            TP.driver_phy.set_fault().overvoltage(false);  // Clear overvoltage error
        }
        SerialUSB.println("V");
    }
}

extern "C" void COMP4_5_6_IRQHandler(void) {
    // Check COMP4 (Phase C overcurrent) - EXTI line 30
    if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_30)) {
        LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_30);  // Clear pending flag

        TP.driver_phy.set_fault().overcurrent(true);  // Set fatal overcurrent error
        SerialUSB.println("C");
    }
}


// ---------------------------------------------------------------------------
// From can_setup.h
// ---------------------------------------------------------------------------

extern "C" void FDCAN1_IT0_IRQHandler(void) {
    HAL_FDCAN_IRQHandler(&CAN_BUS.hfdcan);
}

extern "C" void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef*, uint32_t its) {
    if ((its & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0U) {
        CAN_BUS.on_rx_interrupt();      // Unpack FIFO into the chunk ring
    }
}


// ---------------------------------------------------------------------------
// From bsp/stm32g4/nvm_flash.h
// ---------------------------------------------------------------------------

extern "C" __attribute__((naked)) void NMI_Handler(void) {
    __asm volatile(
        "tst     lr, #4          \n"
        "ite     eq              \n"
        "mrseq   r0, msp         \n"
        "mrsne   r0, psp         \n"
        "b       flashg4_ll_nmi_dispatch \n");
}


// ---------------------------------------------------------------------------
// From bsp/stm32g4/bootloader.h
// ---------------------------------------------------------------------------

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


// ---------------------------------------------------------------------------
// From drv_err_setup.h
// ---------------------------------------------------------------------------

extern "C" void EXTI1_IRQHandler(void) {
    if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_1)) {
        LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_1);  // Clear pending flag

        // Active-low request: pin LOW = fault set, pin HIGH = clear
        const bool is_fault = (LL_GPIO_IsInputPinSet(GPIOB, LL_GPIO_PIN_1) == 0U);
        TP.driver_phy.set_fault().external(is_fault);
    }
}

// ============================================================================
// Application entry points
// ============================================================================

/**
 * @brief Board bring-up, called once from main().
 *
 * Kept as a thin forwarder so that main.cpp depends only on bsp/board.h and
 * never on this board's peripheral headers - that decoupling is what allows a
 * second translation unit to exist at all.
 */
void board_setup() { run_setup(); }

/** @brief Board application body, called from the Arduino main loop. */
void board_loop() { run_loop(); }
