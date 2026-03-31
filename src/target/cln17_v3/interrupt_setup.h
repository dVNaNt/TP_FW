#ifndef NVIC_CLN17V3_H  // Include guard to prevent multiple inclusions of this header file
#define NVIC_CLN17V3_H  // Define the include guard macro for the preprocessor

#include <Arduino.h>  // Include Arduino framework for basic standard types

#include "adc_setup.h"    // Include ADC subsystem initialization and control
#include "clock_setup.h"  // Include system clock configuration
#include "comp_setup.h"   // Include analog comparator subsystem setup
#include "gpio_setup.h"   // Include general purpose I/O configuration

#include "spi_setup.h"        // Include SPI communication setup for the encoder
#include "timer_pwm_setup.h"  // Include PWM timer hardware definitions and functions



struct TunePulseHeart;                        // Forward declaration of the main FOC controller structure
extern TunePulseHeart* volatile g_heart_ptr;  // Expose global volatile pointer to access FOC instance inside ISRs

inline void ADC_READY_Callback() {  // Define inline callback for ADC DMA completion event

    uint16_t current_values[3];                              // Declare array to store raw ADC current readings for all 3 phases
    uint16_t voltage_supply, voltage_vref, temperature_adc;  // Declare variables for auxiliary system measurements
    ADC_get_values(current_values[0], current_values[1], current_values[2], voltage_supply, voltage_vref,
                   temperature_adc);  // Fetch all ADC readings simultaneously

    constexpr uint16_t upper_th = uint16_t(
        (65535.0f * (3.3f - 0.3f)) / 3.3f);  // Calculate max valid ADC threshold at compile time (excluding non-linear zone)
    constexpr uint16_t lower_th = uint16_t((65535.0f * 0.3f) / 3.3f);  // Calculate min valid ADC threshold at compile time
    constexpr uint32_t valid_range =
        upper_th - lower_th;  // Calculate the allowed ADC range at compile time to avoid runtime math

    if ((uint32_t)(current_values[0] - lower_th) > valid_range)  // Ultra-fast single-branch bounds check for Phase A
        current_values[0] = UINT16_MAX;  // Mark Phase A reading as invalid if it falls outside the valid range

    if ((uint32_t)(current_values[1] - lower_th) > valid_range)  // Ultra-fast single-branch bounds check for Phase B
        current_values[1] = UINT16_MAX;  // Mark Phase B reading as invalid if it falls outside the valid range

    if ((uint32_t)(current_values[2] - lower_th) > valid_range)  // Ultra-fast single-branch bounds check for Phase C
        current_values[2] = UINT16_MAX;  // Mark Phase C reading as invalid if it falls outside the valid range

    TUNE_HEART.on_adc_values(current_values, voltage_supply,
                             temperature_adc);  // Pass the validated data into the FOC algorithm core

}  // End of ADC_READY_Callback function

inline void POSITION_Callback() {  // Define inline callback for SPI encoder DMA completion

    TUNE_HEART.on_encoder_raw(SPI1_GetAngle());  // Fetch raw mechanical angle from SPI and inject it into the FOC core
    TUNE_HEART.tick();

}  // End of POSITION_Callback function



inline void NVIC_Init() {  // Define the initialization function for the Nested Vectored Interrupt Controller
    NVIC_SetPriority(SPI1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 6, 0));  // Set SPI1 interrupt to priority level 6
    NVIC_EnableIRQ(SPI1_IRQn);  // Enable the SPI1 interrupt in the NVIC hardware

    NVIC_SetPriority(TIM1_CC_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 3,
                                                       0));  // Set TIM1 Capture/Compare interrupt to high priority level 3
    NVIC_EnableIRQ(TIM1_CC_IRQn);                            // Enable the TIM1 Capture/Compare interrupt in the NVIC hardware
}  // End of NVIC_Init function

/**
 * @brief Ultra-fast TIM1 CH4 Match Callback (Replaces HAL_TIM_OC_DelayElapsedCallback)
 *
 * This function bypasses HAL entirely. It must be called directly from TIM1_CC_IRQHandler.
 */
inline void FOC_TimTrig_Callback() {  // Define custom inline callback for the timer trigger event
    ADC1_StartDMAConversion();  // Trigger hardware dual ADC conversion (ADC1 and ADC2 start synchronously)

    SPI1_StartTransfer();  // Initiate non-blocking SPI DMA transfer to request the next encoder reading

    TIM1_Set_PWM_Values(
        TUNE_HEART.pwm_duty_values());  // Request new duty cycles from FOC core and write them to hardware registers
}  // End of FOC_TimTrig_Callback function

#endif  // NVIC_CLN17V3_H // Final close of the header include guard
