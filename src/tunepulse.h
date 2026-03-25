#ifndef TUNEPULSE_H
#define TUNEPULSE_H

#include <Arduino.h>

#include "lib/common.h"
#include "lib/feedback/analog/current_sense.h"
#include "lib/feedback/inputs_dump.h"
#include "lib/lib.h"
#include "lib/motor_driver/driver_pwm/sel_current.h"

#include "target.h"

#undef SupplyVoltage
#undef Motor
#undef MotorType
#undef PhasePattern
#undef ControlMode
#undef DriverStatus

// Initialize TunePulseHeart with board-specific parameters
// freq_hz=20000, pwm_res=4250, max_supply, iA_fs, iB_fs, iC_fs
TunePulseHeart TUNE_HEART(PWM_TIMER_FREQ_HZ,
                          PWM_TIMER_PERIOD,
                          SYS_PARAMS::MAX_SUPPLY_FS,
                          SYS_PARAMS::IA_MAX_CURRENT_FS,
                          SYS_PARAMS::IB_MAX_CURRENT_FS,
                          SYS_PARAMS::IC_MAX_CURRENT_FS);

// Global pointer for comparator interrupt handlers to access error manager
TunePulseHeart* volatile g_heart_ptr = &TUNE_HEART;


// Include target-specific system setup based on build configuration
#if defined(TARGET_CLN17V3)
#include "target/cln17_v3/interrupt_setup.h"
#elif defined(TARGET_RADIX)
#include "target/radix/system_setup.h"
#else
#error "No target defined! Define TARGET_RADIX or TARGET_CLN17V3"
#endif

#if defined(TARGET_RADIX)
#include "target/radix/comp_setup.h"
#include "target/radix/dac_setup.h"
#include "target/radix/interrupt_setup.h"

inline void tunepulse_init() {
    SystemClock_Config();    // Configure system clock to 170 MHz
    IO_Init();               // Initialize GPIO pins
    DAC_Init();              // Set IVREF to 50% (DAC1)
    DMA_Init();              // Initialize DMA for ADC
    ADC_Init();              // Initialize ADC channels
    PWM_Init();              // Initialize TIM2 PWM
    COMP_Protection_Init();  // Initialize overcurrent protection (DAC3 + COMP2/COMP4)
    SPI3_Init();             // Initialize SPI for encoder
    SPI3_Start();            // Start SPI communication
    NVIC_Init();             // Configure interrupt priorities

    digitalWriteFast(PB_2, HIGH);  // Enable channel 2
    digitalWriteFast(PA_4, HIGH);  // Enable temperature sensor
}
#elif defined(TARGET_CLN17V3)
#include "target/cln17_v3/comp_setup.h"
#include "target/cln17_v3/interrupt_setup.h"
#include "target/cln17_v3/cordic_setup.h"  // Include CORDIC hardware accelerator setup

// LED control functions (active LOW - LOW = ON, HIGH = OFF)
void LED_Red_Control(bool state) {
    if (state) {
        LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_1);  // PB1 = LOW (LED ON)
    } else {
        LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_1);  // PB1 = HIGH (LED OFF)
    }
}

void LED_Green_Control(bool state) {
    if (state) {
        LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_4);  // PC4 = LOW (LED ON)
    } else {
        LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_4);  // PC4 = HIGH (LED OFF)
    }
}

void LED_Blue_Control(bool state) {
    if (state) {
        LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_2);  // PB2 = LOW (LED ON)
    } else {
        LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_2);  // PB2 = HIGH (LED OFF)
    }
}

inline void tunepulse_init() {
    SerialUSB.println("0. SystemClock_Config...");
    SerialUSB.flush();
    delay(100);
    SystemClock_Config();  // Configure system clock to 170 MHz
    SerialUSB.println("   [OK] Clock configured");
    SerialUSB.flush();
    delay(100);

    SerialUSB.println("0.1. CORDIC_Init...");
    SerialUSB.flush();
    delay(100);
    CORDIC_Init();  // Initialize CORDIC hardware accelerator for fast trigonometry
    SerialUSB.println("   [OK] CORDIC initialized");
    SerialUSB.flush();
    delay(100);

    SerialUSB.println("1. IO_Init...");
    SerialUSB.flush();
    delay(100);
    IO_Init();  // Initialize GPIO pins
    SerialUSB.println("   [OK] GPIO initialized");
    SerialUSB.flush();
    delay(100);

    // Register LED control callbacks in TunePulseHeart
    TUNE_HEART.set_led_callback(0, LED_Red_Control);    // Register red LED callback (index 0)
    TUNE_HEART.set_led_callback(1, LED_Green_Control);  // Register green LED callback (index 1)
    TUNE_HEART.set_led_callback(2, LED_Blue_Control);   // Register blue LED callback (index 2)

    // Turn off all LEDs initially
    LED_Red_Control(false);    // Red LED OFF
    LED_Green_Control(false);  // Green LED OFF
    LED_Blue_Control(false);   // Blue LED OFF

    SerialUSB.println("   [OK] LED callbacks registered");
    SerialUSB.flush();
    delay(100);


    SerialUSB.println("3. ADC_Init...");
    SerialUSB.flush();
    delay(100);
    ADC_Init();  // Initialize ADC channels
    SerialUSB.println("   [OK] ADC initialized");
    SerialUSB.flush();
    delay(100);

    SerialUSB.println("4. PWM_Init...");
    SerialUSB.flush();
    delay(100);
    PWM_Init();  // Initialize TIM1 PWM (complementary mode)
    SerialUSB.println("   [OK] PWM initialized");
    SerialUSB.flush();
    delay(100);

    SerialUSB.println("5. COMP_Protection_Init...");
    SerialUSB.flush();
    delay(100);
    COMP_Protection_Init();  // Initialize overcurrent protection (DAC1/DAC3 + COMP1/COMP2/COMP3/COMP4)
    SerialUSB.println("   [OK] Comparator initialized");
    SerialUSB.flush();
    delay(100);

    SerialUSB.println("6. SPI1_Init...");
    SerialUSB.flush();
    delay(100);
    SPI1_Init();  // Initialize SPI1 for encoder
    SerialUSB.println("   [OK] SPI1 initialized");
    SerialUSB.flush();
    delay(100);

    SerialUSB.println("7. SPI1_Start...");
    SerialUSB.flush();
    delay(100);
    SPI1_Start();  // Start SPI communication
    SerialUSB.println("   [OK] SPI1 started");
    SerialUSB.flush();
    delay(100);

    SerialUSB.println("8. NVIC_Init...");
    SerialUSB.flush();
    delay(100);
    NVIC_Init();  // Configure interrupt priorities
    SerialUSB.println("   [OK] NVIC configured");
    SerialUSB.flush();
    delay(100);

    SerialUSB.println("9. Enable DRV8323H...");
    SerialUSB.flush();
    delay(100);
    LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_11);  // DRV_EN = 1
    SerialUSB.println("   [OK] Driver enabled");
    SerialUSB.flush();
    delay(100);

    SerialUSB.println("10. Configure protection limits...");
    SerialUSB.flush();
    delay(100);

    // Register DAC callbacks for hardware protection (must be done before setting limits)
    SerialUSB.println("   Registering DAC callbacks...");
    SerialUSB.flush();
    TUNE_HEART.set_overcurrent_hard_callback(CH1, DAC_Set_COMP2_OC_Threshold);  // Phase A -> COMP2
    TUNE_HEART.set_overcurrent_hard_callback(CH2, DAC_Set_COMP1_OC_Threshold);  // Phase B -> COMP1
    TUNE_HEART.set_overcurrent_hard_callback(CH3, DAC_Set_COMP4_OC_Threshold);  // Phase C -> COMP4
    TUNE_HEART.set_overvoltage_hard_callback(DAC_Set_COMP3_OV_Threshold);       // Overvoltage -> COMP3
    SerialUSB.println("   [OK] DAC callbacks registered");
    SerialUSB.flush();

    delay(100);

    // Set overvoltage protection limit to 53V
    SerialUSB.println("   Setting overvoltage limit...");
    SerialUSB.flush();
    TUNE_HEART.set_overvoltage_hard_mv(55000);  // 53V overvoltage limit
    SerialUSB.println("   [OK] Overvoltage limit set");
    SerialUSB.flush();

    delay(100);

    SerialUSB.println("   [OK] Protection limits configured (OC: 5A/7.5A, OV: 53V)");
    SerialUSB.flush();
    delay(100);

    // Setup current sensing
    TUNE_HEART.driver.start_current_offset_calibration();
    SerialUSB.println("Start current calibration");
    // const int16_t empty_arr[3]= {2000,2000,2000}; 
    // TIM1_Set_PWM_Values(empty_arr);
    while (TUNE_HEART.driver.get_driver_status() != OperationStatus::Ready) {
        TUNE_HEART.tick();
    }

    
    TUNE_HEART.set_overcurrent_hard_ma(CH1, 4600);  // Phase A: 4.6A
    TUNE_HEART.set_overcurrent_hard_ma(CH2, 4600);  // Phase B: 4.6A
    TUNE_HEART.set_overcurrent_hard_ma(CH3, 6900);  // Phase C: 6.9A

    SerialUSB.println("Current sensing setup complete");

    // Calibrate motor
    TUNE_HEART.driver.start_motor_calibration();
    while (TUNE_HEART.driver.get_motor_status() != MotorStatus::Success) {
        TUNE_HEART.tick();
        if (TUNE_HEART.driver.get_motor_status() == MotorStatus::CalibError) {
            SerialUSB.println("Motor calibration Error");
            break;
        }
    }
}
#else
#error "No target defined! Define TARGET_RADIX or TARGET_CLN17V3"
#endif

#endif  // TUNEPULSE_H