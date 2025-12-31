#ifndef SYSTEM_SETUP_H
#define SYSTEM_SETUP_H

#include "tunepulse_heart.h"

#include "setup/adc_setup.h"
#include "setup/clock_setup.h"
#include "setup/comp_setup.h"
#include "setup/dma_setup.h"
#include "setup/gpio_setup.h"
#include "setup/spi_setup.h"
#include "setup/timer_pwm_setup.h"


extern TunePulseHeart TUNE_HEART;

inline void PWM_Callback() {
    TIM2_Set_PWM_Values(TUNE_HEART.pwm_duty_values());     
}

inline void ENCODER_Callback() {
    SPI3_StartTransfer();
}

inline void ADC_START_Callback() {
    // In dual mode, starting ADC1 automatically triggers ADC2
    ADC1_StartDMAConversion();
}

inline void ADC_READY_Callback() {
    uint16_t current_a, current_b, current_c, voltage_supply, voltage_vref, temperature_adc;
    ADC_get_values(current_a, current_b, current_c, voltage_supply, voltage_vref, temperature_adc);

    uint16_t current_values[3] = {current_a, current_b, current_c};
    TUNE_HEART.i_ch2 = current_b;
    TUNE_HEART.on_adc_values(current_values, voltage_supply, temperature_adc);
}

inline void POSITION_Callback() {
    TUNE_HEART.on_encoder_raw(SPI3_GetAngle());
}

#endif