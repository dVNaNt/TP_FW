#ifndef SYSTEM_SETUP_CLN17V3_H
#define SYSTEM_SETUP_CLN17V3_H

#include "adc_setup.h"
#include "clock_setup.h"
#include "comp_setup.h"
#include "dma_setup.h"
#include "gpio_setup.h"
#include "spi_setup.h"
#include "timer_pwm_setup.h"
#include "interrupt_setup.h"

/**
 * @brief PWM callback - called on timer update
 */
inline void PWM_Callback() {
    // TODO: Implement PWM callback for motor control
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
    // TODO: Process ADC values for current sensing and voltage monitoring
}

/**
 * @brief Position callback - called when encoder data ready
 */
inline void POSITION_Callback() {
    // TODO: Process encoder position data
}

#endif  // SYSTEM_SETUP_CLN17V3_H
