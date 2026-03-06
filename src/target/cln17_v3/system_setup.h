#ifndef SYSTEM_SETUP_CLN17V3_H
#define SYSTEM_SETUP_CLN17V3_H

#include "adc_setup.h"
#include "clock_setup.h"
#include "comp_setup.h"
#include "dma_setup.h"
#include "gpio_setup.h"
#include "interrupt_setup.h"
#include "spi_setup.h"
#include "timer_pwm_setup.h"

extern TunePulseHeart TUNE_HEART;

inline void PWM_Callback() {
    TIM1_Set_PWM_Values(TUNE_HEART.pwm_duty_values());  // Update PWM duty cycles
}

inline void ENCODER_Callback() {
    SPI1_StartTransfer();  // Start SPI transfer to read encoder
}

inline void ADC_START_Callback() {
    ADC1_StartDMAConversion();  // In dual mode, starting ADC1 automatically triggers ADC2
}

inline void ADC_READY_Callback() {
    uint16_t current_a, current_b, current_c, voltage_supply, voltage_vref, temperature_adc;
    ADC_get_values(current_a, current_b, current_c, voltage_supply, voltage_vref, temperature_adc);
    uint16_t current_values[3] = {current_a, current_b, current_c};

    constexpr uint16_t upper_th = uint16_t((65565.0f * (3.3f - 0.3f)) / 3.3f);  // Eliminating non-linear zone of current sensor
    constexpr uint16_t lower_th = uint16_t((65565.0f * 0.3f) / 3.3f);
    
    for (size_t i = 0; i < 3; i++)
        if (current_values[i] <= lower_th or current_values[i] >= upper_th)
            current_values[i] = UINT16_MAX;  // Marking current as invalid

    TUNE_HEART.i_ch2 = current_b;
    TUNE_HEART.on_adc_values(current_values, voltage_supply, temperature_adc);
}

inline void POSITION_Callback() {
    TUNE_HEART.on_encoder_raw(SPI1_GetAngle());  // Update encoder position
}

#endif  // SYSTEM_SETUP_CLN17V3_H
