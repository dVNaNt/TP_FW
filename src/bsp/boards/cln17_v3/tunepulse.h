#ifndef TUNEPULSE_H
#define TUNEPULSE_H

/**
 * @file tunepulse.h
 * @brief Board-level integration layer for the TunePulse CLN17-v3 controller.
 *
 * - Instantiates the core @c TunePulseHeart object with board-specific
 *   hardware parameters (PWM frequency/resolution, supply voltage full-scale,
 *   per-phase current-sense full scales).
 * - Registers hardware interrupt handlers (interrupt_setup.h).
 * - Provides the LED apply callback for the onboard tricolor element
 *   (active-LOW GPIO on PB1/PB2/PC4).
 * - Implements tunepulse_init(): the sequential bring-up routine that
 *   configures clocks, peripherals, protection limits and runs both the
 *   current-sense offset calibration and the motor angle calibration.
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
 * DEFINED in board.cpp, the board's single translation unit, so that exactly
 * one object file owns it. Declared here because nearly every board header and
 * the interrupt glue refer to it.
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
extern TunePulseHeart TP;

#include "bsp/stm32g4/bootloader.h"  // SW1 hold-to-bootloader (USB DFU)
#include "interrupt_setup.h"  // ISR glue: ADC / encoder / comparators -> TP

#include "bsp/stm32g4/fdcan.h"        // CAN-FD driver + chunk API (CAN_BUS)
#include "bsp/stm32g4/comp_dac.h"       // Comparator (hardware OC/OV) protection
#include "bsp/stm32g4/cordic.h"     // CORDIC hardware trigonometry accelerator
#include "bsp/stm32g4/nvm_flash.h"        // Internal-flash NVM partitions (FlashG4_LL)
#include "bsp/stm32g4/tim1_pwm.h"  // TIM1 complementary PWM backend
#include "lib/dev_log.h"      // TP_LOG*: bring-up messages exist only in DEV_LOG_ENABLE builds

/**
 * @brief LED apply callback for the CLN17-v3 tricolor element.
 *
 * The library LED driver hands over TWO packed RGB888 words (0x00RRGGBB, one
 * per LED element). The CLN17-v3 board has a SINGLE tricolor element built
 * from discrete active-LOW pins (PB1 = red, PC4 = green, PB2 = blue), so the
 * first word is decoded channel-by-channel onto the pins and the second word
 * is ignored. The decoding format is simple: if a channel BYTE is non-zero,
 * that LED is ON (full brightness); a zero byte is OFF.
 *
 * The pins are written through the GPIO BSRR registers DIRECTLY (no HAL/LL
 * calls): one atomic store per port toggles any subset of pins, which is
 * both faster and interrupt-safe by construction.
 *
 * @param led1 Packed RGB888 color for LED element 1 (the onboard LED).
 * @param led2 Packed RGB888 color for LED element 2 (unused on this board).
 */
void LED_Apply(int32_t led1, int32_t led2) {
    (void)led2;  // Second element not populated on CLN17-v3

    // Channel bytes of the first packed word (non-zero = LED ON)
    const uint32_t r = (uint32_t)led1 & 0x0000FFU;  // Red byte   (bits [7:0])
    const uint32_t g = (uint32_t)led1 & 0x00FF00U;  // Green byte (bits [15:8])
    const uint32_t b = (uint32_t)led1 & 0xFF0000U;  // Blue byte  (bits [23:16])

    // Active-LOW pins: ON = pin LOW = BSRR reset (bit 16), OFF = pin HIGH =
    // BSRR set (bit 0). One atomic store per port.
    GPIOB->BSRR = (r ? GPIO_BSRR_BR1 : GPIO_BSRR_BS1)    // PB1 = RED (active LOW)
                | (b ? GPIO_BSRR_BR2 : GPIO_BSRR_BS2);   // PB2 = BLUE (active LOW)
    GPIOC->BSRR = (g ? GPIO_BSRR_BR4 : GPIO_BSRR_BS4);   // PC4 = GREEN (active LOW)
}

/**
 * @brief Callback invoked when an ADC current-offset calibration cycle starts.
 *
 * Drives the output voltage vector to (+INFINITY, +INFINITY), which the PWM
 * backend (MotorControl::set_voltage_ab_volts) maps to INT16_MAX duty on all
 * three phases, i.e. ALL HIGH-SIDE switches ON. All motor terminals are then
 * tied to the DC bus: zero differential voltage, no torque, and the current
 * amplifiers read their pure offset without back-EMF contribution.
 */
void start_curnt_cal() {
    TP.driver_phy.motor_ctrl.set_voltage_ab_volts(Vec2D_f32(INFINITY, INFINITY));
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
    TP.driver_phy.motor_ctrl.set_voltage_ab_volts(Vec2D_f32(NAN, NAN));
}

/**
 * Longest the bring-up will wait for the current-sense offset calibration [ms].
 *
 * The wait used to be unbounded. It is advanced only from interrupt context, so
 * anything that stops those interrupts - no motor, a dead ADC trigger, a
 * mis-set timer - hung the firmware here forever: no console command, no CAN,
 * no protocol, nothing to diagnose with. A bounded wait fails loudly and lets
 * the communication stack come up so the drive can be interrogated.
 */
#ifndef TP_CURRENT_CAL_TIMEOUT_MS
#define TP_CURRENT_CAL_TIMEOUT_MS 10000U
#endif

/** Longest the bring-up will wait for the motor angle calibration [ms]. */
#ifndef TP_MOTOR_CAL_TIMEOUT_MS
#define TP_MOTOR_CAL_TIMEOUT_MS 60000U
#endif

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
 * 5. LED apply callback registered - TP.led (single apply callback, two
 *    packed RGB888 words; see LED_Apply).
 * 6. ADC_Init()                - ADC channels for currents and supply voltage.
 * 7. PWM_Init()                - TIM1 complementary PWM.
 * 8. Encoder_Init()            - SPI1 for the magnetic encoder (type set by
 *                                ENCODER_TYPE in spi_setup.h).
 * 9. NVIC_Init()               - interrupt priorities.
 * 10. CAN bus configuration    - classic 1 Mbps arbitration / 2 Mbps FD data
 *     phase, external loopback mode, transceiver enabled, then CAN_Driver_Init().
 * 11. DRV8323H gate driver enabled via DRV_EN (PB12 on the MT6835 board
 *     revision, PB11 on the TLE5012B revision).
 * 12. COMP_Protection_Init()   - DAC-driven comparators for hardware OC/OV.
 * 13. Protection limits        - DAC threshold callbacks registered, then
 *     software limit values applied (see note below about log text mismatch).
 * 14. Fault callbacks          - OC/ext/UV faults force Hi-Z, OV forces all-low.
 * 15. Current-sense offset calibration - bounded wait (TP_CURRENT_CAL_TIMEOUT_MS).
 * 16. Motor angle calibration  - bounded wait until MotorStatus::Success,
 *     MotorStatus::CalibError or TP_MOTOR_CAL_TIMEOUT_MS.
 *
 * @note This function blocks while the calibrations run, so it must not be
 *       called before the interrupts that advance them are active. Both waits
 *       are BOUNDED: on timeout the drive continues de-energized rather than
 *       hanging, so the console and the CAN protocol still come up and can
 *       report what failed.
 */
inline void tunepulse_init() {
    delay(100);
    SystemClock_Config();  // Configure system clock to 170 MHz

    delay(100);

    CORDIC_Init();  // Initialize CORDIC hardware accelerator for fast trigonometry

    delay(100);

    // Calibration-table persistence setup:
    // bind the raw internal-flash operations of the CALIBRATION partition used
    // by the dump/persistence layer. The partition size comes from the NVM
    // layout defines (nvm_flash.h) so it stays flexible and single-sourced.
    TP.calibration_dump.set_max_size(FlashG4_LL::calib_section_size());
    TP.calibration_dump.set_callback()
        .erase_cb(&FlashG4_LL::calibration_erase)
        .read_cb(&FlashG4_LL::calibration_read)
        .write_cb(&FlashG4_LL::calibration_write);

    // User-data persistence setup (system_dump):
    // a second independent FlashEEPROM instance bound to the USER-DATA
    // partition callbacks. All bounds/safety logic is handled inside the NVM
    // flash driver per-section, exactly like for the calibration partition.
    TP.system_dump.set_max_size(FlashG4_LL::user_section_size());
    TP.system_dump.set_callback()
        .erase_cb(&FlashG4_LL::user_erase)
        .read_cb(&FlashG4_LL::user_read)
        .write_cb(&FlashG4_LL::user_write);

    // Motor-parameter persistence: bind the motor record store to the
    // USER-DATA partition and arm the boot-time restore scan. The scan runs
    // non-blocking (8 words / 32 bytes per system tick from TunePulseHeart::
    // tick()) and restores the NEWEST valid record, printing the restored
    // parameter set over serial when done.
    //
    // NOTE: begin_load(MOTOR_ID_ANY) is mandatory here. The no-argument
    // overload filters on the CURRENT cal_id_, which is still the constructor
    // default (0) at this point - before any record has been restored. Since
    // a recalibration appends its table record to the NEXT free flash chunk,
    // the freshly saved profile usually carries a NON-ZERO Cal-ID; filtering
    // the boot scan by 0 silently DROPPED it, so after the first re-edit the
    // power-cycle state (stale profile / full re-calibration at every boot)
    // diverged from the live post-calibration state.
    TP.motor_params.attach_flash(&TP.system_dump);
    TP.motor_params.begin_load(MOTOR_ID_ANY);

    delay(100);
    IO_Init();  // Initialize GPIO pins

    delay(100);

    // Register the single LED apply callback in TunePulseHeart: two packed
    // RGB888 words, one per LED element (LED2 unused on this board).
    TP.led.set_callback().apply_cb(&LED_Apply);

    // Turn the tricolor element off initially
    TP.led.off();  // All channels dark (applies immediately)

    delay(100);

    ADC_Init();  // Initialize ADC channels

    delay(100);

    PWM_Init();  // Initialize TIM1 PWM (complementary mode)

    delay(100);

    Encoder_Init();  // Initialize SPI1 for the encoder (ENCODER_TYPE in spi_setup.h)

    delay(100);

    NVIC_Init();  // Configure interrupt priorities

    delay(500);

    // CAN bring-up moved to the application: main.cpp calls CAN_BUS.begin()
    // with the fixed link configuration (FD-BRS, 1 Mbps / 5 Mbps, NORMAL).
    // Gate-driver enable: the DRV_EN pin depends on the board revision, which
    // follows the compile-time selected encoder type (see spi_setup.h).
#if (ENCODER_TYPE == ENCODER_MT6835)
    LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_12);  // DRV_EN = 1 (PB12, MT6835 board)
#else
    LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_11);  // DRV_EN = 1 (PB11, TLE5012B board)
#endif

    delay(100);

    COMP_Protection_Init();  // Initialize overcurrent protection (DAC1/DAC3 + COMP1/COMP2/COMP3/COMP4)

    delay(100);

    TP.driver_phy.current.overcurrent.set_callback()
        .oc_lim_ia_cb(DAC_Set_COMP2_OC_Threshold)
        .oc_lim_ib_cb(DAC_Set_COMP1_OC_Threshold)
        .oc_lim_ic_cb(DAC_Set_COMP4_OC_Threshold);
    // Hardware overcurrent thresholds (amps): phases A/B at 4.0 A, phase C at
    // 6.2 A.
    TP.driver_phy.current.overcurrent.set_limit_amps().oc_ia(4.0).oc_ib(4.0).oc_ic(6.2);
    TP.driver_phy.supply.overvoltage.set_lim_callback(DAC_Set_COMP3_OV_Threshold);
    TP.driver_phy.supply.set_limit_volts().uv_lim_volts(16.0).ov_lim_volts(51.0);

    // Thermal protection (Temperature, see temperature.h):
    // - OT (overtemperature) trip at 85 degC — forces FAULT_OVERHEAT (Hi-Z safe state)
    // - Hysteresis band 10 degC — the fault releases only after the filtered
    //   temperature falls back below 75 degC (no chatter around one threshold).
    // The integer EMA filter time constant is 4096 ticks (~205 ms at 20 kHz);
    // thresholds are precomputed into the raw ADC domain once, so tick() does
    // pure integer comparisons only.
    TP.driver_phy.temperature.set_limit_celsius().ot_lim_celsius(85.0f).ot_hyst_celsius(10.0f);
    TP.driver_phy.power.overpower.set_limit().op_lim_watts(60.0f).op_lim_amps(2.50f);
    // Raw -> degC recalculation callback (STM32G4 factory calibration, adc_setup.h):
    // Temperature converts the filtered internal-sensor reading into real
    // degrees Celsius and compares the OT limits against that value. Installing
    // the callback re-inverts the configured thresholds through the calibration
    // curve, so the call order relative to set_limit_celsius() does not matter.
    TP.driver_phy.temperature.set_do_conversion_cb(ADC_Temperature_To_Celsius);

    TP.driver_phy.current.on_calibration_start(start_curnt_cal);
    TP.driver_phy.current.on_calibration_complete(end_curnt_cal);

    TP.driver_phy.pwm.set_period(PWM_TIMER_PERIOD);
    TP.driver_phy.set_fault()
        .set_callback()
        .oc_cb(PWM_Set_All_HiZ)
        .ov_cb(PWM_Set_All_Low)
        .ext_cb(PWM_Set_All_HiZ)
        .uv_cb(PWM_Set_All_HiZ)
        .ot_cb(PWM_Set_All_HiZ)   // Software overtemperature: same Hi-Z safe state as hardware faults
        .op_cb(PWM_Set_All_HiZ).phase_cb(PWM_Set_All_HiZ)  // Software overpower: Hi-Z safe state on power/current limit violation
        .enc_cb(PWM_Set_All_HiZ);                           // Position feedback lost: Hi-Z rather than a loop chasing a phantom
    TP.driver_phy.pwm.set_callback().apply_phases_cb(TIM1_Set_PWM_Values);

    delay(100);

    // Current-sense offset calibration:
    // start_curnt_cal() ties all phases to VBUS (all high-side ON) and
    // end_curnt_cal() ties them to GND (all low-side ON); in both states the
    // differential voltage is zero, so the ADC captures amplifier offsets only.
    TP.driver_phy.current.calibrate();
    TP_LOG("Start current calibration");

    // Bounded wait for completion. Completion is signalled from the ADC
    // interrupt; if no interrupt ever advances the state machine the drive is
    // broken, and hanging here forever would hide that behind a dead board.
    // Timing out instead leaves the phases de-energized and lets setup()
    // finish, so the console and the CAN protocol come up and can report why.
    const uint32_t isense_t0 = millis();
    while (TP.driver_phy.current.is_calibrated() == false) {
        BOOT::poll();  // hold SW1 for 3 s to enter the USB bootloader
        if ((millis() - isense_t0) > TP_CURRENT_CAL_TIMEOUT_MS) {
            TP_LOG("Current sensing calibration TIMED OUT - continuing de-energized");
            break;
        }
    }

    if (TP.driver_phy.current.is_calibrated()) {
        TP_LOG("Current sensing setup complete");
    }

    // Motor (electrical-angle) calibration:
    // sets the driver status to DriverCalibration and starts the automatic
    // calibration sequence (rotor alignment + sweep). Waits in a blocking
    // loop until the status becomes Success or CalibError. As above, the
    // tick() call is commented out, so progress depends entirely on
    // interrupt-driven updates (see CODE_ISSUES.md).
    TP.current_driver.setup_status.set_driver_status(OperationStatus::DriverCalibration);
    TP.current_driver.start_motor_calibration();

    const uint32_t motor_t0 = millis();
    while (TP.current_driver.get_motor_status() != MotorStatus::Success) {
        BOOT::poll();  // hold SW1 for 3 s to enter the USB bootloader
        if (TP.current_driver.get_motor_status() == MotorStatus::CalibError) {
            TP_LOG("Motor calibration Error");
            break;
        }
        if ((millis() - motor_t0) > TP_MOTOR_CAL_TIMEOUT_MS) {
            // Same reasoning as the current-sense wait: a drive that cannot
            // identify its motor (none connected, open phase, dead encoder) is
            // far more useful reporting that over the protocol than hanging.
            TP_LOG("Motor calibration TIMED OUT - continuing de-energized");
            break;
        }
    }
}

#endif  // TUNEPULSE_H