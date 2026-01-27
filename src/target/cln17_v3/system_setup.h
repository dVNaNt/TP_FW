#ifndef SYSTEM_SETUP_CLN17V3_H
#define SYSTEM_SETUP_CLN17V3_H

#include "tunepulse_heart.h"

#include "adc_setup.h"
#include "clock_setup.h"
#include "comp_setup.h"
#include "dma_setup.h"
#include "gpio_setup.h"
#include "spi_setup.h"
#include "timer_pwm_setup.h"

extern TunePulseHeart TUNE_HEART;

/**
 * @brief PWM callback - called on timer update
 */
inline void PWM_Callback() {
    TIM1_Set_PWM_Values(TUNE_HEART.pwm_duty_values());  // Update PWM duty cycles
}

/**
 * @brief Encoder callback - called on timer event
 */
inline void ENCODER_Callback() {
    SPI1_StartTransfer();  // Start SPI transfer to read encoder
}

/**
 * @brief ADC start callback - triggered by timer
 */
inline void ADC_START_Callback() {
    ADC1_StartDMAConversion();  // Start ADC1 (triggers ADC2 in dual mode)
}

/**
 * @brief ADC ready callback - called when DMA transfer complete
 */
inline void ADC_READY_Callback() {
    uint16_t current_a, current_b, current_c, voltage_supply, voltage_vref, temperature_adc;
    
    // Get ADC values from DMA buffers
    ADC_get_values(current_a, current_b, current_c, voltage_supply, voltage_vref, temperature_adc);

    // Pack current values into array
    uint16_t current_values[3] = {current_a, current_b, current_c};
    
    // Store phase B current for debugging
    TUNE_HEART.i_ch2 = current_b;
    
    // Call TunePulseHeart ADC callback
    TUNE_HEART.on_adc_values(current_values, voltage_supply, temperature_adc);
}

/**
 * @brief Position callback - called when encoder data ready
 */
inline void POSITION_Callback() {
    TUNE_HEART.on_encoder_raw(SPI1_GetAngle());  // Update encoder position
}

#endif  // SYSTEM_SETUP_CLN17V3_H
