#ifndef COMP_SETUP_H
#define COMP_SETUP_H

#include <Arduino.h>
#include "stm32g4xx_hal.h"
#include "stm32g4xx_ll_comp.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_gpio.h"
#include "stm32g4xx_ll_exti.h"

// Fault lockout duration in milliseconds
constexpr uint32_t FAULT_LOCKOUT_MS = 1000U;

// Fault source flags (bitmask)
constexpr uint8_t FAULT_NONE = 0x00;   // No fault
constexpr uint8_t FAULT_COMP1 = 0x01;  // COMP1 triggered (I_B on PA_1)
constexpr uint8_t FAULT_COMP2 = 0x02;  // COMP2 triggered (I_A on PA_3)
constexpr uint8_t FAULT_COMP3 = 0x04;  // COMP3 triggered (VSENS on PA_0)
constexpr uint8_t FAULT_COMP4 = 0x08;  // COMP4 triggered (I_C on PB_0)

// Fault state variables
volatile uint8_t g_fault_source = FAULT_NONE;  // Which comparator triggered (bitmask)
volatile uint32_t g_fault_timestamp = 0;       // When fault occurred (millis)
volatile bool g_fault_active = false;          // Fault lockout active

// Forward declaration of TunePulseHeart for error flag access
struct TunePulseHeart;
extern TunePulseHeart* g_heart_ptr;  // Global pointer to TunePulseHeart instance

// Forward declarations
void PWM_Emergency_Disable(uint8_t fault_source);
void COMP_Update_Fault_State(void);

/**
 * @brief Initialize COMP1 for overcurrent detection on PA_1 (I_B)
 * INP: PA_1 (current sense phase B)
 * INM: DAC1_CH1 (threshold reference)
 */
static inline void MX_COMP1_Init(void) {
    // Enable COMP clock (SYSCFG contains COMP)
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    
    // PA_1 already configured as analog in ADC setup
    
    // Disable COMP1 before configuration
    LL_COMP_Disable(COMP1);
    
    // Set input plus to PA_1 (IO2)
    LL_COMP_SetInputPlus(COMP1, LL_COMP_INPUT_PLUS_IO2);
    
    // Set input minus to DAC1 channel 1
    LL_COMP_SetInputMinus(COMP1, LL_COMP_INPUT_MINUS_DAC1_CH1);
    
    // Set hysteresis for noise immunity
    LL_COMP_SetInputHysteresis(COMP1, LL_COMP_HYSTERESIS_MEDIUM);
    
    // Output polarity: non-inverted (high when INP > INM)
    LL_COMP_SetOutputPolarity(COMP1, LL_COMP_OUTPUTPOL_NONINVERTED);
    
    // Blanking source: none
    LL_COMP_SetOutputBlankingSource(COMP1, LL_COMP_BLANKINGSRC_NONE);
    
    // Enable COMP1
    LL_COMP_Enable(COMP1);
    
    // Wait for COMP1 startup time (~10us)
    volatile uint32_t wait = 200;
    while (wait--) { __NOP(); }
}

/**
 * @brief Initialize COMP2 for overcurrent detection on PA_3 (I_A)
 * INP: PA_3 (current sense phase A)
 * INM: DAC1_CH2 (threshold reference)
 */
static inline void MX_COMP2_Init(void) {
    // PA_3 already configured as analog in ADC setup
    
    // Disable COMP2 before configuration
    LL_COMP_Disable(COMP2);
    
    // Set input plus to PA_3 (IO1)
    LL_COMP_SetInputPlus(COMP2, LL_COMP_INPUT_PLUS_IO1);
    
    // Set input minus to DAC1 channel 2
    LL_COMP_SetInputMinus(COMP2, LL_COMP_INPUT_MINUS_DAC1_CH2);
    
    // Set hysteresis for noise immunity
    LL_COMP_SetInputHysteresis(COMP2, LL_COMP_HYSTERESIS_MEDIUM);
    
    // Output polarity: non-inverted (high when INP > INM)
    LL_COMP_SetOutputPolarity(COMP2, LL_COMP_OUTPUTPOL_NONINVERTED);
    
    // Blanking source: none
    LL_COMP_SetOutputBlankingSource(COMP2, LL_COMP_BLANKINGSRC_NONE);
    
    // Enable COMP2
    LL_COMP_Enable(COMP2);
    
    // Wait for COMP2 startup time (~10us)
    volatile uint32_t wait = 200;
    while (wait--) { __NOP(); }
}

/**
 * @brief Initialize COMP3 for overvoltage detection on PA_0 (VSENS)
 * INP: PA_0 (bus voltage sense)
 * INM: DAC3_CH1 (threshold reference)
 */
static inline void MX_COMP3_Init(void) {
    // PA_0 already configured as analog in ADC setup
    
    // Disable COMP3 before configuration
    LL_COMP_Disable(COMP3);
    
    // Set input plus to PA_0 (IO1)
    LL_COMP_SetInputPlus(COMP3, LL_COMP_INPUT_PLUS_IO1);
    
    // Set input minus to DAC3 channel 1
    LL_COMP_SetInputMinus(COMP3, LL_COMP_INPUT_MINUS_DAC3_CH1);
    
    // Set hysteresis for noise immunity
    LL_COMP_SetInputHysteresis(COMP3, LL_COMP_HYSTERESIS_MEDIUM);
    
    // Output polarity: non-inverted (high when INP > INM)
    LL_COMP_SetOutputPolarity(COMP3, LL_COMP_OUTPUTPOL_NONINVERTED);
    
    // Blanking source: none
    LL_COMP_SetOutputBlankingSource(COMP3, LL_COMP_BLANKINGSRC_NONE);
    
    // Enable COMP3
    LL_COMP_Enable(COMP3);
    
    // Wait for COMP3 startup time (~10us)
    volatile uint32_t wait = 200;
    while (wait--) { __NOP(); }
}

/**
 * @brief Initialize COMP4 for overcurrent detection on PB_0 (I_C)
 * INP: PB_0 (current sense phase C)
 * INM: DAC3_CH2 (threshold reference)
 */
static inline void MX_COMP4_Init(void) {
    // PB_0 already configured as analog in ADC setup
    
    // Disable COMP4 before configuration
    LL_COMP_Disable(COMP4);
    
    // Set input plus to PB_0 (IO1)
    LL_COMP_SetInputPlus(COMP4, LL_COMP_INPUT_PLUS_IO1);
    
    // Set input minus to DAC3 channel 2
    LL_COMP_SetInputMinus(COMP4, LL_COMP_INPUT_MINUS_DAC3_CH2);
    
    // Set hysteresis for noise immunity
    LL_COMP_SetInputHysteresis(COMP4, LL_COMP_HYSTERESIS_MEDIUM);
    
    // Output polarity: non-inverted (high when INP > INM)
    LL_COMP_SetOutputPolarity(COMP4, LL_COMP_OUTPUTPOL_NONINVERTED);
    
    // Blanking source: none
    LL_COMP_SetOutputBlankingSource(COMP4, LL_COMP_BLANKINGSRC_NONE);
    
    // Enable COMP4
    LL_COMP_Enable(COMP4);
    
    // Wait for COMP4 startup time (~10us)
    volatile uint32_t wait = 200;
    while (wait--) { __NOP(); }
}

/**
 * @brief Configure EXTI interrupts for all comparators
 * Rising edge triggers protection (overcurrent/overvoltage)
 */
static inline void COMP_EXTI_Init(void) {
    // COMP1 is connected to EXTI21
    // COMP2 is connected to EXTI22
    // COMP3 is connected to EXTI29
    // COMP4 is connected to EXTI30
    
    // Enable EXTI line 21 (COMP1) - rising edge
    LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_21);           // Unmask interrupt
    LL_EXTI_EnableRisingTrig_0_31(LL_EXTI_LINE_21);   // Rising edge trigger
    LL_EXTI_DisableFallingTrig_0_31(LL_EXTI_LINE_21); // Disable falling edge
    
    // Enable EXTI line 22 (COMP2) - rising edge
    LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_22);           // Unmask interrupt
    LL_EXTI_EnableRisingTrig_0_31(LL_EXTI_LINE_22);   // Rising edge trigger
    LL_EXTI_DisableFallingTrig_0_31(LL_EXTI_LINE_22); // Disable falling edge
    
    // Enable EXTI line 29 (COMP3) - rising edge
    LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_29);           // Unmask interrupt
    LL_EXTI_EnableRisingTrig_0_31(LL_EXTI_LINE_29);   // Rising edge trigger
    LL_EXTI_DisableFallingTrig_0_31(LL_EXTI_LINE_29); // Disable falling edge
    
    // Enable EXTI line 30 (COMP4) - rising edge
    LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_30);           // Unmask interrupt
    LL_EXTI_EnableRisingTrig_0_31(LL_EXTI_LINE_30);   // Rising edge trigger
    LL_EXTI_DisableFallingTrig_0_31(LL_EXTI_LINE_30); // Disable falling edge
    
    // Clear any pending flags
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_21 | LL_EXTI_LINE_22 | LL_EXTI_LINE_29 | LL_EXTI_LINE_30);
    
    // Enable NVIC interrupts
    // COMP1/2/3 share COMP1_2_3_IRQn
    // COMP4 uses COMP4_IRQn
    NVIC_SetPriority(COMP1_2_3_IRQn, 0);  // Highest priority for safety
    NVIC_EnableIRQ(COMP1_2_3_IRQn);
    
    NVIC_SetPriority(COMP4_IRQn, 0);      // Highest priority for safety
    NVIC_EnableIRQ(COMP4_IRQn);
}

/**
 * @brief Turn off all LEDs (open-drain, common +, LOW = ON)
 */
inline void LED_All_Off(void) {
    LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_13);  // Green OFF (HIGH)
    LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_14);  // Blue OFF (HIGH)
    LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_15);  // Red OFF (HIGH)
}

/**
 * @brief Set green LED only (COMP1 fault indicator - I_B)
 */
inline void LED_Set_Green(void) {
    LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_14);   // Blue OFF
    LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_15);   // Red OFF
    LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_13); // Green ON (LOW)
}

/**
 * @brief Set blue LED only (COMP2 fault indicator - I_A)
 */
inline void LED_Set_Blue(void) {
    LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_13);   // Green OFF
    LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_15);   // Red OFF
    LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_14); // Blue ON (LOW)
}

/**
 * @brief Set red LED only (COMP3/COMP4 fault indicator - VSENS/I_C)
 */
inline void LED_Set_Red(void) {
    LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_13);   // Green OFF
    LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_14);   // Blue OFF
    LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_15); // Red ON (LOW)
}

/**
 * @brief Emergency disable all PWM outputs (Hi-Z state) with LED indication
 * @param fault_source Which comparator triggered (bitmask: FAULT_COMPx)
 */
inline void PWM_Emergency_Disable(uint8_t fault_source) {
    // Record fault source and timestamp (OR with existing faults)
    g_fault_source |= fault_source;
    g_fault_timestamp = millis();
    g_fault_active = true;
    
    // Set error flags in TunePulseHeart error manager
    if (g_heart_ptr != nullptr) {
        if (fault_source & FAULT_COMP1) {
            g_heart_ptr->error.set_fatal_phase2_oc();  // COMP1 (PA_1/I_B) -> Phase B overcurrent
        }
        if (fault_source & FAULT_COMP2) {
            g_heart_ptr->error.set_fatal_phase1_oc();  // COMP2 (PA_3/I_A) -> Phase A overcurrent
        }
        if (fault_source & FAULT_COMP3) {
            g_heart_ptr->error.set_fatal_overvoltage(); // COMP3 (PA_0/VSENS) -> Overvoltage
        }
        if (fault_source & FAULT_COMP4) {
            g_heart_ptr->error.set_fatal_phase3_oc();  // COMP4 (PB_0/I_C) -> Phase C overcurrent
        }
    }
    
    // Disable all EN_CH pins (set to LOW for Hi-Z)
    LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_10); // EN_CH1 = 0
    LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_2);  // EN_CH2 = 0
    LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_1);  // EN_CH3 = 0
    LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_6);  // EN_CH4 = 0
    
    // Set PWM to 100% duty (outputs stay low when EN is disabled)
    TIM2->CCR1 = TIM2->ARR;  // Channel 1 = 100%
    TIM2->CCR3 = TIM2->ARR;  // Channel 3 = 100%
    TIM2->CCR4 = TIM2->ARR;  // Channel 4 = 100%
    
    // Set LED based on fault source priority
    if (fault_source & (FAULT_COMP3 | FAULT_COMP4)) {
        LED_Set_Red();    // COMP3 (VSENS) or COMP4 (I_C) -> Red LED
    } else if (fault_source & FAULT_COMP2) {
        LED_Set_Blue();   // COMP2 (I_A) -> Blue LED
    } else if (fault_source & FAULT_COMP1) {
        LED_Set_Green();  // COMP1 (I_B) -> Green LED
    }
}

/**
 * @brief Update fault state - call from main loop to handle timeout
 * Clears fault after FAULT_LOCKOUT_MS milliseconds
 */
inline void COMP_Update_Fault_State(void) {
    if (g_fault_active) {
        uint32_t elapsed = millis() - g_fault_timestamp;
        if (elapsed >= FAULT_LOCKOUT_MS) {
            // Lockout period expired - clear fault
            g_fault_active = false;
            g_fault_source = FAULT_NONE;
            LED_All_Off();  // Turn off fault LED
        }
    }
}

/**
 * @brief Check if fault lockout is active
 * @return true if fault occurred and lockout period not expired
 */
inline bool COMP_Is_Fault_Active(void) {
    return g_fault_active;
}

/**
 * @brief Get current fault source
 * @return FAULT_NONE, FAULT_COMP2, or FAULT_COMP4
 */
inline uint8_t COMP_Get_Fault_Source(void) {
    return g_fault_source;
}

/**
 * @brief Force clear fault (manual reset)
 */
inline void COMP_Clear_Fault(void) {
    g_fault_active = false;
    g_fault_source = FAULT_NONE;
    LED_All_Off();
}

/**
 * @brief Check current comparator output states
 * @return Bitmask: bit0=COMP1, bit1=COMP2, bit2=COMP3, bit3=COMP4
 */
inline uint8_t COMP_Get_Output_State(void) {
    uint8_t state = 0;
    if (LL_COMP_ReadOutputLevel(COMP1) == LL_COMP_OUTPUT_LEVEL_HIGH) {
        state |= 0x01;  // COMP1 (I_B)
    }
    if (LL_COMP_ReadOutputLevel(COMP2) == LL_COMP_OUTPUT_LEVEL_HIGH) {
        state |= 0x02;  // COMP2 (I_A)
    }
    if (LL_COMP_ReadOutputLevel(COMP3) == LL_COMP_OUTPUT_LEVEL_HIGH) {
        state |= 0x04;  // COMP3 (VSENS)
    }
    if (LL_COMP_ReadOutputLevel(COMP4) == LL_COMP_OUTPUT_LEVEL_HIGH) {
        state |= 0x08;  // COMP4 (I_C)
    }
    return state;
}

// COMP1/2/3 interrupt handler (shared for COMP1, COMP2, COMP3)
extern "C" void COMP1_2_3_IRQHandler(void) {
    // Check if COMP1 triggered (EXTI line 21)
    if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_21)) {
        LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_21);      // Clear pending flag
        PWM_Emergency_Disable(FAULT_COMP1);           // Disable PWM, green LED
    }
    
    // Check if COMP2 triggered (EXTI line 22)
    if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_22)) {
        LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_22);      // Clear pending flag
        PWM_Emergency_Disable(FAULT_COMP2);           // Disable PWM, blue LED
    }
    
    // Check if COMP3 triggered (EXTI line 29)
    if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_29)) {
        LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_29);      // Clear pending flag
        PWM_Emergency_Disable(FAULT_COMP3);           // Disable PWM, red LED
    }
}

// COMP4 interrupt handler
extern "C" void COMP4_IRQHandler(void) {
    // Check if COMP4 triggered (EXTI line 30)
    if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_30)) {
        LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_30);      // Clear pending flag
        PWM_Emergency_Disable(FAULT_COMP4);           // Disable PWM, red LED
    }
}

/**
 * @brief Initialize complete protection system
 * All 4 comparators with DAC thresholds and interrupts
 */
inline void COMP_Protection_Init(void) {
    MX_COMP1_Init();   // Initialize COMP1 on PA_1 (I_B) with DAC1_CH1
    MX_COMP2_Init();   // Initialize COMP2 on PA_3 (I_A) with DAC1_CH2
    MX_COMP3_Init();   // Initialize COMP3 on PA_0 (VSENS) with DAC3_CH1
    MX_COMP4_Init();   // Initialize COMP4 on PB_0 (I_C) with DAC3_CH2
    COMP_EXTI_Init();  // Enable interrupts for all comparators
}

#endif  // COMP_SETUP_H
