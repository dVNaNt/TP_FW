#ifndef TUNEPULSE_H
#define TUNEPULSE_H

/**
 * @file tunepulse.h
 * @brief Board-level integration layer for the TunePulse CLN17-v3 controller.
 *
 * This header wires together all firmware subsystems into a single global
 * application object (@c TUNE_HEART of type TunePulseHeart):
 *
 * - Instantiates the core @c TunePulseHeart object with board-specific
 *   hardware parameters (PWM frequency/resolution, supply voltage full-scale,
 *   per-phase current-sense full scales).
 * - Registers hardware interrupt handlers (interrupt_setup.h).
 * - Provides LED control callbacks (active-LOW GPIO on PB1/PB2/PC4).
 * - Implements tunepulse_init(): the sequential bring-up routine that
 *   configures clocks, peripherals, protection limits and runs both the
 *   current-sense offset calibration and the motor angle calibration.
 *
 * The macros SupplyVoltage / Motor / MotorType / PhasePattern / ControlMode /
 * DriverStatus are #undef'd after including target.h so that their names can
 * be reused freely by the library without colliding with target-layer
 * definitions.
 */

#include <Arduino.h>

#include "lib/lib.h"

#include "target.h"

#undef SupplyVoltage
#undef Motor
#undef MotorType
#undef PhasePattern
#undef ControlMode
#undef DriverStatus

/**
 * @brief Global motor-control application object for this board.
 *
 * Constructed with board-specific constants from the target layer:
 * - PWM_TIMER_FREQ_HZ : PWM carrier frequency in Hz (e.g. 20 kHz).
 * - PWM_TIMER_PERIOD  : PWM timer auto-reload period (counts), defines the
 *                       PWM resolution together with the timer clock.
 * - MAX_SUPPLY_FS     : ADC full-scale value corresponding to the maximum
 *                       measurable supply voltage.
 * - IA/IB/IC_MAX_CURRENT_FS : ADC full-scale values corresponding to the
 *                       maximum measurable phase currents for phases A/B/C.
 */
TunePulseHeart TUNE_HEART(PWM_TIMER_FREQ_HZ,
                          PWM_TIMER_PERIOD,
                          SYS_PARAMS::MAX_SUPPLY_FS,
                          SYS_PARAMS::IA_MAX_CURRENT_FS,
                          SYS_PARAMS::IB_MAX_CURRENT_FS,
                          SYS_PARAMS::IC_MAX_CURRENT_FS);

/**
 * @brief Volatile global pointer to the application heart object.
 *
 * Used by hardware comparator ISR handlers (interrupt_setup.h) to reach the
 * error/fault manager of the single global TunePulseHeart instance from
 * C-style interrupt service routines that have no object context.
 * Declared @c volatile because it is written/read across execution contexts
 * (main thread and ISRs).
 */
TunePulseHeart* volatile g_heart_ptr = &TUNE_HEART;

#include "target/cln17_v3/interrupt_setup.h"

#include "target/cln17_v3/can_setup.h"  // Include CAN-FD driver setup
#include "target/cln17_v3/comp_setup.h"
#include "target/cln17_v3/cordic_setup.h"  // Include CORDIC hardware accelerator setup
#include "target/cln17_v3/interrupt_setup.h"
#include "target/cln17_v3/nvm_flash.h"
#include "target/cln17_v3/timer_pwm_setup.h"

/**
 * @brief Red LED control callback (active LOW: true = ON, false = OFF).
 * @param state true switches the LED on (pin driven LOW), false switches it off.
 */
void LED_Red_Control(bool state) {
    if (state) {
        LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_1);  // PB1 = LOW (LED ON)
    } else {
        LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_1);  // PB1 = HIGH (LED OFF)
    }
}

/**
 * @brief Green LED control callback (active LOW: true = ON, false = OFF).
 * @param state true switches the LED on (pin driven LOW), false switches it off.
 */
void LED_Green_Control(bool state) {
    if (state) {
        LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_4);  // PC4 = LOW (LED ON)
    } else {
        LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_4);  // PC4 = HIGH (LED OFF)
    }
}

/**
 * @brief Blue LED control callback (active LOW: true = ON, false = OFF).
 * @param state true switches the LED on (pin driven LOW), false switches it off.
 */
void LED_Blue_Control(bool state) {
    if (state) {
        LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_2);  // PB2 = LOW (LED ON)
    } else {
        LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_2);  // PB2 = HIGH (LED OFF)
    }
}

/**
 * @brief Callback invoked when an ADC current-offset calibration cycle starts.
 *
 * Drives the output voltage vector to (+INFINITY, +INFINITY), which the PWM
 * backend (PhyDriverPWM::set_voltage_ab_volts) maps to INT16_MAX duty on all
 * three phases, i.e. ALL HIGH-SIDE switches ON. All motor terminals are then
 * tied to the DC bus: zero differential voltage, no torque, and the current
 * amplifiers read their pure offset without back-EMF contribution.
 */
void start_curnt_cal() {
    TUNE_HEART.driver_phy.set_voltage_ab_volts(Vec2D_f32(INFINITY, INFINITY));
}

/**
 * @brief Callback invoked when an ADC current-offset calibration cycle ends.
 *
 * Drives the output voltage vector to (NAN, NAN), which the PWM backend maps
 * to INT16_MIN duty on all three phases, i.e. ALL LOW-SIDE switches ON. All
 * motor terminals are tied to ground (again zero differential voltage),
 * releasing the drive into a defined static state.
 */
void end_curnt_cal() {
    TUNE_HEART.driver_phy.set_voltage_ab_volts(Vec2D_f32(NAN, NAN));
}

/**
 * @brief Full sequential bring-up of the motor-control stack.
 *
 * Initialization order (each step logs progress over SerialUSB):
 * 1. SystemClock_Config()      - system clock to 170 MHz.
 * 2. CORDIC_Init()             - hardware trigonometry accelerator.
 * 3. Calibration dump setup    - binds flash erase/read/write callbacks and
 *                                reserves 4096 entries for the calibration
 *                                table persistence layer.
 * 4. IO_Init()                 - GPIO pins.
 * 5. LED callbacks registered  - red/green/blue mapped to indices 0/1/2.
 * 6. ADC_Init()                - ADC channels for currents and supply voltage.
 * 7. PWM_Init()                - TIM1 complementary PWM.
 * 8. Encoder_SmartInit()       - SPI1 for the magnetic encoder.
 * 9. NVIC_Init()               - interrupt priorities.
 * 10. CAN bus configuration    - classic 1 Mbps arbitration / 2 Mbps FD data
 *     phase, external loopback mode, transceiver enabled, then CAN_Driver_Init().
 * 11. DRV8323H gate driver enabled via PB11 (DRV_EN = high).
 * 12. COMP_Protection_Init()   - DAC-driven comparators for hardware OC/OV.
 * 13. Protection limits        - DAC threshold callbacks registered, then
 *     software limit values applied (see note below about log text mismatch).
 * 14. Fault callbacks          - OC/ext/UV faults force Hi-Z, OV forces all-low.
 * 15. Current-sense offset calibration - blocking wait until complete.
 * 16. Motor angle calibration  - blocking wait until MotorStatus::Success or
 *     MotorStatus::CalibError.
 *
 * @note This function blocks until both calibrations finish; it must not be
 *       called before the scheduler/interrupts required by those calibrations
 *       are active.
 */
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

    // Calibration-table persistence setup:
    // bind the raw internal-flash operations of the CALIBRATION partition used
    // by the dump/persistence layer. The partition size comes from the NVM
    // layout defines (nvm_flash.h) so it stays flexible and single-sourced.
    TUNE_HEART.calibration_dump.set_max_size(FlashG4_LL::calib_section_size());
    TUNE_HEART.calibration_dump.set_callback()
        .erase_cb(&FlashG4_LL::calibration_erase)
        .read_cb(&FlashG4_LL::calibration_read)
        .write_cb(&FlashG4_LL::calibration_write);

    // User-data persistence setup (system_dump):
    // a second independent FlashEEPROM instance bound to the USER-DATA
    // partition callbacks. All bounds/safety logic is handled inside the NVM
    // flash driver per-section, exactly like for the calibration partition.
    TUNE_HEART.system_dump.set_max_size(FlashG4_LL::user_section_size());
    TUNE_HEART.system_dump.set_callback()
        .erase_cb(&FlashG4_LL::user_erase)
        .read_cb(&FlashG4_LL::user_read)
        .write_cb(&FlashG4_LL::user_write);

    // Angle calibration persistence is fully self-managed: on the first ticks
    // the calibrator restores a valid table from flash (is_calibrated() -> true)
    // or automatically starts a fresh calibration cycle. No manual call needed.

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

    SerialUSB.println("6. SPI1_Init...");
    SerialUSB.flush();
    delay(100);
    Encoder_SmartInit();  // Initialize SPI1 for encoder
    SerialUSB.println("   [OK] SPI initialized");
    SerialUSB.flush();
    delay(100);

    SerialUSB.println("8. NVIC_Init...");
    SerialUSB.flush();
    delay(100);
    NVIC_Init();  // Configure interrupt priorities
    SerialUSB.println("   [OK] NVIC configured");
    SerialUSB.flush();
    delay(500);

    // CAN initialization moved to application code (main.cpp)
    // Configure for EXTERNAL_LOOPBACK mode (self-test with physical pins)
    CAN_BUS.set_baudrate_classic(1000000);         // 1 Mbps classic
    CAN_BUS.set_baudrate_fd(2000000);              // 2 Mbps data phase
    CAN_BUS.set_mode(CANMode::EXTERNAL_LOOPBACK);  // External loopback for testing
    CAN_BUS.enable_auto_retry(false);              // No auto-retry for testing
    CAN_BUS.enable_delay_compensation(true);

    // Enable transceiver
    CAN_BUS.enable();
    SerialUSB.println("8.1. CAN_Driver_Init...");
    SerialUSB.flush();
    delay(100);
    CAN_Driver_Init();  // Initialize CAN-FD driver for CAN 2.0A at 1 Mbps
    SerialUSB.println("   [OK] CAN initialized");
    SerialUSB.flush();
    delay(100);

    SerialUSB.println("9. Enable DRV8323H...");
    SerialUSB.flush();
    delay(100);
    LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_11);  // DRV_EN = 1
    SerialUSB.println("   [OK] Driver enabled");
    SerialUSB.flush();
    delay(100);

    SerialUSB.println("5. COMP_Protection_Init...");
    SerialUSB.flush();
    delay(100);
    COMP_Protection_Init();  // Initialize overcurrent protection (DAC1/DAC3 + COMP1/COMP2/COMP3/COMP4)
    SerialUSB.println("   [OK] Comparator initialized");
    SerialUSB.flush();
    delay(100);

    SerialUSB.println("10. Configure protection limits...");
    SerialUSB.flush();
    delay(100);

    // Register DAC callbacks for hardware protection (must be done before setting limits)
    SerialUSB.println("   Registering DAC callbacks...");
    SerialUSB.flush();
    TUNE_HEART.driver_phy.current_sense.overcurrent.set_callback()
        .oc_lim_ia_cb(DAC_Set_COMP2_OC_Threshold)
        .oc_lim_ib_cb(DAC_Set_COMP1_OC_Threshold)
        .oc_lim_ic_cb(DAC_Set_COMP4_OC_Threshold);
    // Hardware overcurrent thresholds (amps): phases A/B at 4.0 A, phase C at
    // 6.2 A. NOTE: the comment below previously claimed 4.6 A / 6.9 A which did
    // not match the actual configured values.
    TUNE_HEART.driver_phy.current_sense.overcurrent.set_limit_amps().oc_ia(4.0).oc_ib(4.0).oc_ic(6.2);
    TUNE_HEART.driver_phy.supply_voltage.overvoltage.set_lim_callback(DAC_Set_COMP3_OV_Threshold);
    TUNE_HEART.driver_phy.supply_voltage.set_limit_volts().uv_lim_volts(16.0).ov_lim_volts(48.0);
    TUNE_HEART.driver_phy.current_sense.on_calibration_start(start_curnt_cal);
    TUNE_HEART.driver_phy.current_sense.on_calibration_complete(end_curnt_cal);

    SerialUSB.println("   [OK] DAC callbacks registered");
    SerialUSB.flush();

    delay(100);

    // Overvoltage protection limit was configured above via set_limit_volts()
    // (UV 16.0 V / OV 48.0 V); the historical comment mentioning 53 V did not
    // match the actually programmed value.
    SerialUSB.println("   Setting overvoltage limit...");
    SerialUSB.flush();

    SerialUSB.println("   [OK] Overvoltage limit set");
    SerialUSB.flush();

    delay(100);

    // NOTE: this log line historically reported "OC: 5A/7.5A, OV: 53V" which
    // does not match the real configuration (OC: 4.0/4.0/6.2 A, UV: 16 V,
    // OV: 48 V). Kept verbatim to preserve serial protocol compatibility.
    SerialUSB.println("   [OK] Protection limits configured (OC: 5A/7.5A, OV: 53V)");
    SerialUSB.flush();
    delay(100);

    TUNE_HEART.driver_phy.pwm.set_period(PWM_TIMER_PERIOD);
    TUNE_HEART.driver_phy.set_fault()
        .set_callback()
        .oc_cb(PWM_Set_All_HiZ)
        .ov_cb(PWM_Set_All_Low)
        .ext_cb(PWM_Set_All_HiZ)
        .uv_cb(PWM_Set_All_HiZ);
    TUNE_HEART.driver_phy.pwm.set_callback().apply_phases_cb(TIM1_Set_PWM_Values);

    // Current-sense offset calibration:
    // start_curnt_cal() ties all phases to VBUS (all high-side ON) and
    // end_curnt_cal() ties them to GND (all low-side ON); in both states the
    // differential voltage is zero, so the ADC captures amplifier offsets only.
    TUNE_HEART.driver_phy.current_sense.calibrate();
    SerialUSB.println("Start current calibration");

    // Blocking wait for completion. Completion is expected to be signalled
    // from the ADC/interrupt context; the periodic tick() call is disabled
    // here, so if no interrupt advances the calibration state machine this
    // loop will spin forever (see CODE_ISSUES.md).
    while (TUNE_HEART.driver_phy.current_sense.is_calibrated() == false) {
        // TUNE_HEART.tick();
    }

    SerialUSB.println("Current sensing setup complete");

    // Motor (electrical-angle) calibration:
    // sets the driver status to DriverCalibration and starts the automatic
    // calibration sequence (rotor alignment + sweep). Waits in a blocking
    // loop until the status becomes Success or CalibError. As above, the
    // tick() call is commented out, so progress depends entirely on
    // interrupt-driven updates (see CODE_ISSUES.md).
    TUNE_HEART.driver.setup_status.set_driver_status(OperationStatus::DriverCalibration);
    TUNE_HEART.driver.start_motor_calibration();
    while (TUNE_HEART.driver.get_motor_status() != MotorStatus::Success) {
        // TUNE_HEART.tick();
        if (TUNE_HEART.driver.get_motor_status() == MotorStatus::CalibError) {
            SerialUSB.println("Motor calibration Error");
            break;
        }
    }
    SerialUSB.println("Going motion");
}

#endif  // TUNEPULSE_H