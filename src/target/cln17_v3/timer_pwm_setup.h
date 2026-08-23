/**
 * @file timer_pwm_setup.h
 * @brief TIM1 complementary-PWM hardware backend.
 *
 * PWM_Init() configures TIM1 (center-aligned, complementary outputs, dead
 * time); TIM1_Set_PWM_Values() writes the three compare registers;
 * PWM_Set_All_HiZ()/PWM_Set_All_Low() implement the fault safe states.
 */

#ifndef TIMER_PWM_CLN17V3_H  // Include guard to prevent multiple inclusions of this header file
#define TIMER_PWM_CLN17V3_H  // Define the include guard macro for the preprocessor

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

extern TunePulseHeart* volatile g_heart_ptr;  // Expose the global control state pointer for protection checks

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

// Force maximum inline speed and O3 optimization exclusively for this ultra-fast mathematical loop
__attribute__((always_inline, optimize("O3"))) inline void TIM1_Set_PWM_Values(
    const int16_t pwmValues[3]) {  // Define ultra-fast PWM update function
    int32_t d0 = pwmValues[0];     // Load requested duty cycle for Phase A
    int32_t d1 = pwmValues[1];     // Load requested duty cycle for Phase B
    int32_t d2 = pwmValues[2];     // Load requested duty cycle for Phase C

    TunePulseHeart* heart = g_heart_ptr;      // Safely cache the volatile pointer to a local register for speed
    if (heart != nullptr) {                   // Check if the pointer has been properly initialized
        if (heart->driver_phy.get_fault()) {  // Check Over-Voltage state first (highest priority)
            return;
        }  // End of protection condition checks
    }  // End of pointer safety check

    // __builtin_abs compiles to a single-cycle, branchless ARM instruction (no pipeline stalls)
    uint32_t abs0 = __builtin_abs(d0);  // Get absolute value for Phase A duty cycle
    uint32_t abs1 = __builtin_abs(d1);  // Get absolute value for Phase B duty cycle
    uint32_t abs2 = __builtin_abs(d2);  // Get absolute value for Phase C duty cycle

    // Clamping using ternary operators is highly optimized by GCC into branchless IT (If-Then) blocks
    TIM1->CCR1 = (abs0 > PWM_TIMER_PERIOD) ? PWM_TIMER_PERIOD : abs0;  // Clamp and write Phase A register
    TIM1->CCR2 = (abs1 > PWM_TIMER_PERIOD) ? PWM_TIMER_PERIOD : abs1;  // Clamp and write Phase B register
    TIM1->CCR3 = (abs2 > PWM_TIMER_PERIOD) ? PWM_TIMER_PERIOD : abs2;  // Clamp and write Phase C register

    // Fast mask generation: CC4E is bit 12 (0x1000U). We unconditionally keep CH4 enabled.
    uint32_t ccer_mask = 0x1000U;          // Start with CH4 Output Compare enable bit set
    ccer_mask |= 0x04U | (d0 > 0);         // CH1: CC1NE is always 1 (0x04), CC1E is 1 only if duty is positive
    ccer_mask |= (0x04U | (d1 > 0)) << 4;  // CH2: Shifted by 4 bits for CC2NE and CC2E
    ccer_mask |= (0x04U | (d2 > 0)) << 8;  // CH3: Shifted by 8 bits for CC3NE and CC3E

    // Direct assignment is significantly faster than Read-Modify-Write (|=) because we know all exact bit states
    TIM1->CCER = ccer_mask;  // Overwrite CCER register completely in one atomic hardware cycle

    int16_t delay = ADC_DELAY_TICKS;  // Fetch volatile ADC delay variable exactly once to prevent race conditions
    uint32_t safe_delay = (delay < 0) ? 0 : ((delay > PWM_TIMER_PERIOD) ? PWM_TIMER_PERIOD : delay);  // Branchless clamp
    TIM1->CCR4 = PWM_TIMER_PERIOD - safe_delay;  // Assign safe calculated delay directly to CCR4 register

    // Direct assignment overwriting BDTR completely, preserving MOE and updating runtime deadtime instantly
    TIM1->BDTR = TIM_BDTR_MOE | (PWM_DEADTIME_RUNTIME & 0xFFU);  // Update deadtime safely without Read-Modify-Write
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

#endif  // TIMER_PWM_CLN17V3_H // Final close of the header include guard