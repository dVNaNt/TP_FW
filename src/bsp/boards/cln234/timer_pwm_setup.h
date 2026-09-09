/**
 * @file timer_pwm_setup.h
 * @brief TIM1 complementary-PWM hardware backend for CLN234.
 *
 * PWM_Init() configures TIM1 (center-aligned, complementary outputs, dead
 * time); TIM1_Set_PWM_Values() writes the three compare registers;
 * PWM_Set_All_HiZ()/PWM_Set_All_Low() implement the fault safe states.
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#ifndef TIMER_PWM_CLN234_H  // Include guard to prevent multiple inclusions of this header file
#define TIMER_PWM_CLN234_H  // Define the include guard macro for the preprocessor

#include <Arduino.h>            // Include Arduino framework for basic standard types
#include "stm32g4xx.h"          // Include pure CMSIS register definitions for STM32G4
#include "stm32g4xx_hal_tim.h"  // Include HAL TIM purely to support the required IRQ handler structure
#include "target.h"             // Include hardware target-specific tuning parameters

#ifndef PWM_TIMER_PERIOD                     // Check if the PWM timer period is not defined at the project level
constexpr uint16_t PWM_TIMER_PERIOD = 4250;  // Set default timer period (ARR value) for the carrier frequency
#endif                                       // Close conditional preprocessor block for the period

#ifndef PWM_TIMER_FREQ_HZ                      // Check if the PWM frequency is not defined at the project level
constexpr uint16_t PWM_TIMER_FREQ_HZ = 20000;  // Set default PWM frequency to 20 kHz
#endif                                         // Close conditional preprocessor block for the frequency

constexpr int32_t TIMER_CLOCK_HZ = 170UL;  // Define the core system clock speed as 170 MHz for reference

constexpr uint8_t PWM_DEADTIME_TICKS = SYS_PARAMS::DEADTIME_TICKS_CONST;  // Extract initial deadtime from system parameters

volatile uint8_t PWM_DEADTIME_RUNTIME = PWM_DEADTIME_TICKS;  // Allow deadtime to be updated dynamically at runtime

volatile int16_t ADC_DELAY_TICKS = SYS_PARAMS::ADC_DELAY_TICKS_CONST;  // Set volatile ADC delay for FOC trigger offset
volatile int16_t ADC_TRIGGER_POINT = 0;                                // Initialize volatile ADC trigger point to zero

// OPTIMIZATION (P2.1): last written CCR4 delay / BDTR deadtime. Both values
// change rarely, so the per-period register rewrites are skipped until the
// requested value actually differs. Seeded with impossible values so the
// first cycle after boot always performs the write.
static int16_t adc_delay_written = -1;          // Cached clamped ADC delay currently present in CCR4
static uint8_t deadtime_written = 0xFF;         // Cached deadtime currently present in BDTR (0xFF never configured)

TIM_HandleTypeDef htim1;  // Declare the HAL structure globally to satisfy HAL_TIM_IRQHandler requirement

inline void MX_TIM1_Init(void) {         // Define an inline function for bare-metal register initialization
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;  // Enable the hardware clock routing specifically for TIM1 peripheral
    htim1.Instance = TIM1;               // Link the HAL handler to TIM1 instance so HAL_TIM_IRQHandler does not crash

    // FIX: TIM_CR1_CMS_1 corresponds to Center-Aligned Mode 2 (Interrupt flags set on counting up)
    TIM1->CR1 = TIM_CR1_CMS_1 | TIM_CR1_ARPE;  // Set Center-aligned mode 2 and enable Auto-Reload Preload
    TIM1->CR2 = 0;                 // Clear CR2 to keep default states (no master mode TRGO needed if CH4 is used directly)
    TIM1->PSC = 0;                 // Set prescaler to 0 to run the timer at the absolute maximum 170MHz frequency
    TIM1->ARR = PWM_TIMER_PERIOD;  // Write the carrier period directly to the Auto-Reload Register

    // Configure CCMR1 for CH1 and CH2: PWM Mode 1 (110 in OCxM) and enable preload (OCxPE)
    TIM1->CCMR1 =
        (6U << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE | (6U << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;  // Write CH1/CH2 config

    // Configure CCMR2 for CH3 and CH4: PWM Mode 1 (110 in OCxM) and enable preload (OCxPE)
    TIM1->CCMR2 =
        (6U << TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE | (6U << TIM_CCMR2_OC4M_Pos) | TIM_CCMR2_OC4PE;  // Write CH3/CH4 config

    TIM1->CCER = 0;                   // Initialize Capture/Compare Enable Register to 0 (all outputs safely disabled for now)
    TIM1->BDTR = PWM_DEADTIME_TICKS;  // Write initial deadtime directly to Break and Dead-Time Register
    TIM1->EGR = TIM_EGR_UG;           // Force an Update Generation event to shadow all registers immediately
}  // End of MX_TIM1_Init bare-metal function

inline void MX_TIM1_Start() {  // Define an inline function to activate timer outputs directly via registers
    TIM1->CCR4 = PWM_TIMER_PERIOD - ADC_DELAY_TICKS;  // Write initial ADC trigger timing to Capture/Compare 4

    TIM1->SR = ~(TIM_SR_UIF | TIM_SR_CC4IF);  // Clear Update Interrupt Flag and CH4 Interrupt Flag efficiently
    TIM1->DIER = TIM_DIER_CC4IE;              // Enable hardware interrupt specifically for CH4 (ADC trigger)

    TIM1->CCER = 0x1555U;        // Pre-enable all complimentary outputs: CC1E/NE, CC2E/NE, CC3E/NE, and CC4E
    TIM1->CR1 |= TIM_CR1_CEN;    // Set Counter Enable bit to start the primary TIM1 timer counting
    TIM1->BDTR |= TIM_BDTR_MOE;  // Set Main Output Enable bit to route timer signals to physical GPIO pins
}  // End of MX_TIM1_Start bare-metal function

/**
 * @brief Force all PWM outputs to High-Impedance (All FETs OFF)
 * Mimics the Over-Current protection state by setting CCR to ARR
 * and disabling High-side outputs (CCxE=0, CCxNE=1).
 */
__attribute__((always_inline, optimize("O3"))) inline void PWM_Set_All_HiZ() {
    TIM1->CCR1 = 0;  // Set CCR to 0 -> OCxREF is 0 -> Low-side ON 100%
    TIM1->CCR2 = 0;
    TIM1->CCR3 = 0;

    // Keep CC4E (0x1000) for ADC. Enable only CCxNE (0x0444), clear CCxE.
    TIM1->CCER = 0x1444U;
}

/**
 * @brief Force all PWM outputs to Low (All Low-sides ON, High-sides OFF)
 * Mimics the Over-Voltage protection state by setting CCR to 0
 * and disabling High-side outputs (CCxE=0, CCxNE=1).
 */
__attribute__((always_inline, optimize("O3"))) inline void PWM_Set_All_Low() {
    TIM1->CCR1 = PWM_TIMER_PERIOD;  // Set CCR to ARR -> OCxREF is 0 -> Low-side OFF 100%
    TIM1->CCR2 = PWM_TIMER_PERIOD;
    TIM1->CCR3 = PWM_TIMER_PERIOD;

    // Keep CC4E (0x1000) for ADC. Enable only CCxNE (0x0444), clear CCxE.
    TIM1->CCER = 0x1444U;
}

/**
 * @brief Compute compare value and CCER channel-mask bits for one phase.
 *
 * Signed-duty semantics (duty in timer ticks, -period..+period):
 * - duty > 0            : ccr = duty, both outputs on (0x05 = CCxE | CCxNE)
 *                         -> standard complementary PWM, high-side ratio = duty/period.
 * - duty == 0           : ccr = 0, low-side only (0x04 = CCxNE). Low-side gate is the
 *                         inverted OCxREF, and OCxREF is always 0 here -> low-side ON 100%
 *                         -> maximum regenerative braking.
 * - -1 .. -(period-1)   : ccr = |duty|, low-side only. OCxREF is high for ccr/period of
 *                         the carrier, so the low-side conducts (1 - |duty|/period):
 *                         near -1 almost always ON (braking), fading toward freewheel.
 * - duty <= -period     : pure freewheel — mask 0x00 disables BOTH FETs of the phase
 *                         (glitch-free; avoids the residual 1-tick pulse that a full-period
 *                         CCR would produce in center-aligned mode at the counter peak).
 * @param d    Signed duty in ticks
 * @param ccr  [out] Clamped compare value to write into CCRx
 * @return Channel-enable bits (CCxE/CCxNE pattern) for this phase's CCER slot
 */
__attribute__((always_inline)) static inline uint32_t pwm_phase_ccr(int32_t d, uint32_t& ccr) {
    // __builtin_abs compiles to a single-cycle, branchless ARM instruction (no pipeline stalls)
    uint32_t mag = __builtin_abs(d);                 // Magnitude of the requested duty cycle

    // Branchless-style clamp saturates any out-of-range request to the full period
    if (mag > (uint32_t)PWM_TIMER_PERIOD) mag = PWM_TIMER_PERIOD;
    ccr = mag;                                       // Compare value: |duty| (saturating)

    if (d > 0) return 0x05U;                         // Active drive: high-side + low-side complementary
    if (mag >= (uint32_t)PWM_TIMER_PERIOD) return 0x00U;  // Full negative: freewheel, all FETs OFF
    return 0x04U;                                    // Braking blend: low-side only, weight = 1 - |duty|/period
}


/**
 * @brief Ultra-fast 3-phase PWM update with signed-duty semantics.
 *
 * Duty range is -period..+period (in timer ticks) per phase:
 * - duty > 0        : normal complementary drive, high-side ON ratio = duty/period.
 * - duty == 0       : maximum regenerative braking (high-side OFF, low-side 100% ON).
 * - duty in -1..-period : high-side always OFF; low-side conducts
 *   (1 - |duty|/period) of the carrier -> smooth blend between braking and freewheel.
 *   Near -1 the low-side is almost always ON (strong braking), near -period almost never.
 * - duty <= -period : pure freewheel — both FETs of the phase are disabled (HiZ),
 *   guaranteed glitch-free (no residual carrier pulses).
 */
__attribute__((always_inline, optimize("O3"))) inline void TIM1_Set_PWM_Values(
    const int16_t pwmValues[3]) {  // Define ultra-fast PWM update function
    int32_t d0 = pwmValues[0];     // Load requested duty cycle for Phase A
    int32_t d1 = pwmValues[1];     // Load requested duty cycle for Phase B
    int32_t d2 = pwmValues[2];     // Load requested duty cycle for Phase C

    if (TP.driver_phy.get_fault()) {  // Check active fault state (single global object, no pointer indirection)
        return;
    }  // End of protection condition check

    uint32_t ccr0, ccr1, ccr2;                                        // Compare values computed per phase
    uint32_t m0 = pwm_phase_ccr(d0, ccr0);                            // Phase A: compare value + channel mask
    uint32_t m1 = pwm_phase_ccr(d1, ccr1);                            // Phase B: compare value + channel mask
    uint32_t m2 = pwm_phase_ccr(d2, ccr2);                            // Phase C: compare value + channel mask

    TIM1->CCR1 = ccr0;  // Write Phase A compare register
    TIM1->CCR2 = ccr1;  // Write Phase B compare register
    TIM1->CCR3 = ccr2;  // Write Phase C compare register

    // Fast mask assembly: CC4E is bit 12 (0x1000U), kept unconditionally for the ADC trigger.
    // Per-phase masks occupy 4-bit slots: xE bit0/xNE bit2 -> <<0, <<4, <<8.
    uint32_t ccer_mask = 0x1000U | m0 | (m1 << 4) | (m2 << 8);

    // Direct assignment is significantly faster than Read-Modify-Write (|=) because we know all exact bit states
    TIM1->CCER = ccer_mask;  // Overwrite CCER register completely in one atomic hardware cycle

    // OPTIMIZATION (P2.1): CCR4/BDTR only when the requested values change.
    int16_t delay = ADC_DELAY_TICKS;  // Fetch volatile ADC delay variable exactly once to prevent race conditions
    if (delay != adc_delay_written) {  // Write the trigger point only on an actual parameter change
        uint32_t safe_delay = (delay < 0) ? 0 : ((delay > PWM_TIMER_PERIOD) ? PWM_TIMER_PERIOD : delay);  // Branchless clamp
        TIM1->CCR4 = PWM_TIMER_PERIOD - safe_delay;  // Assign safe calculated delay directly to CCR4 register
        adc_delay_written = delay;                   // Remember what the hardware now holds
    }

    uint8_t deadtime = (uint8_t)(PWM_DEADTIME_RUNTIME & 0xFFU);  // Fetch volatile runtime deadtime exactly once
    if (deadtime != deadtime_written) {  // Rewrite BDTR only on an actual deadtime change
        // Direct assignment overwriting BDTR completely, preserving MOE and updating runtime deadtime instantly
        TIM1->BDTR = TIM_BDTR_MOE | deadtime;  // Update deadtime safely without Read-Modify-Write
        deadtime_written = deadtime;           // Remember what the hardware now holds
    }
}  // End of TIM1_Set_PWM_Values ultra-fast function

extern void FOC_TimTrig_Callback();                // Forward declare the ultra-fast callback located in the NVIC file
extern "C" FDCAN_HandleTypeDef* CAN_Get_Handle();  // Forward declare function to get CAN FDCAN handle

extern "C" void TIM1_CC_IRQHandler(void) {  // Define the hardware interrupt vector for Capture/Compare events
    if (TIM1->SR & TIM_SR_CC4IF) {          // Check purely if the Channel 4 (ADC trigger) interrupt flag is active
        TIM1->SR = ~TIM_SR_CC4IF;           // Clear the interrupt flag immediately by writing 0 to its bit
        FOC_TimTrig_Callback();             // Directly call our optimized callback, bypassing HAL completely
    }  // End of CC4IF check
}  // End of TIM1_CC_IRQHandler function

inline void PWM_Init() {  // Define inline wrapper to trigger full PWM subsystem initialization
    MX_TIM1_Init();       // Execute bare-metal timer configuration
    MX_TIM1_Start();      // Execute bare-metal output and interrupt startup sequence
}  // End of PWM_Init function

#endif  // TIMER_PWM_CLN234_H // Final close of the header include guard