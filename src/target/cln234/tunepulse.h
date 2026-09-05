#ifndef TUNEPULSE_CLN234_H
#define TUNEPULSE_CLN234_H

/**
 * @file tunepulse.h
 * @brief Board-level integration layer for the TunePulse CLN234 controller.
 *
 * This header wires together all firmware subsystems into a single global
 * application object (@c TP of type TunePulseHeart). It is the CLN234 twin of
 * target/cln17_v3/tunepulse.h: main.cpp includes only "run.h" (which lives
 * next to this file), and every target ships its own peripheral bring-up,
 * protection limits and board callbacks. CLN234 differences vs. CLN17 v3:
 *
 * - MCU                : STM32G474CEU6 (512 KiB flash, 128 KiB SRAM).
 * - Gate driver        : DRV8353 smart driver on the SHARED SPI1 bus (with
 *                        the MT6835 encoder, separate chip selects). Hardware
 *                        OC/UVLO/OTSD is delegated to the DRV8353, which
 *                        reports through nFAULT (PB1, EXTI1 -> external
 *                        fault). There are NO MCU comparators/DACs on this
 *                        board, so no COMP/DAC threshold setup is needed -
 *                        only software protection limits are installed.
 * - Status LED         : WS2812 chain (2 px) on PC6 (TIM8_CH1 + DMA)
 *                        instead of the discrete red/green/blue GPIO LEDs.
 * - Temperature        : external NTC 100k on PC4 (piecewise beta model)
 *                        instead of the factory-calibrated internal sensor.
 * - Current calibration: the DRV8353 integrated shunt amplifiers (CSA).
 *
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#include <Arduino.h>

#include "lib/lib.h"  // TunePulseHeart + the full motor-control stack

#include "target.h"   // CLN234 pinout + SYS_PARAMS (pulls spi_setup.h)

/**
 * @brief Global motor-control application object for this board.
 *
 * Constructed with board-specific constants from the target layer:
 * - PWM_TIMER_FREQ_HZ : PWM carrier frequency in Hz (default 20 kHz).
 * - PWM_TIMER_PERIOD  : PWM timer auto-reload period (counts), defines the
 *                       PWM resolution together with the timer clock.
 * - MAX_SUPPLY_FS     : ADC full-scale value corresponding to the maximum
 *                       measurable supply voltage.
 * - IA/IB/IC_MAX_CURRENT_FS : ADC full-scale values corresponding to the
 *                       maximum measurable phase currents for phases A/B/C.
 */
TunePulseHeart TP(PWM_TIMER_FREQ_HZ,
                  PWM_TIMER_PERIOD,
                  SYS_PARAMS::MAX_SUPPLY_FS,
                  SYS_PARAMS::IA_MAX_CURRENT_FS,
                  SYS_PARAMS::IB_MAX_CURRENT_FS,
                  SYS_PARAMS::IC_MAX_CURRENT_FS);

// Board bring-up headers. Order matters: the ISR glue and the fault hooks
// reference the TP object, so they are included after its construction.
#include "ws2812.h"           // WS2812 status LED (TIM8_CH1 + DMA)
#include "bootloader_jump.h"  // SW1 (PC14) hold-to-bootloader (USB DFU)
#include "interrupt_setup.h"  // ISR glue + NVIC (pulls clock/gpio/adc/spi/timer_pwm/drv_err/temperature)
#include "cordic_setup.h"     // CORDIC hardware trigonometry accelerator
#include "nvm_flash.h"        // Internal-flash NVM partitions (FlashG4_LL)
#include "can_setup.h"        // CAN-FD driver + chunk API (CAN_BUS)

/**
 * @brief Status-LED control callback for the WS2812 chain (PC6).
 *
 * The TunePulseHeart LED API is a boolean per-index callback; the CLN234 has
 * no discrete LEDs, so index 0 maps to the whole WS2812 chain: true = dim
 * RED on both pixels, false = dark. Index 0 is driven by TunePulseHeart::
 * tick() as the FAULT indicator (led_control(0, driver_phy.get_fault())),
 * matching the red fault LED of the CLN17 board. Colors are only written to
 * the shadow buffer - LED_Update() streams the frame out non-blocking (it
 * is pumped from the 20 kHz control ISR, see interrupt_setup.h).
 *
 * @param state true switches the status LED on (dim red = fault indicator).
 */
void LED_WS2812_Control(bool state) {
    if (state) {
        LED_SetRGB(60, 0, 0);  // Both LEDs: dim RED (fault indicator, index 0)
    } else {
        LED_Off();             // Both LEDs dark
    }
    LED_Update();              // Arm the non-blocking frame transfer
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
 * @brief Full sequential bring-up of the motor-control stack (CLN234).
 *
 * Initialization order (each step logs progress over SerialUSB):
 * 1. SystemClock_Config()      - HSI-based PLL to 170 MHz (no HSE on board).
 * 2. CORDIC_Init()             - hardware trigonometry accelerator.
 * 3. Calibration dump setup    - binds flash erase/read/write callbacks and
 *                                reserves the CALIBRATION partition size for
 *                                the calibration table persistence layer.
 * 4. User-data dump setup      - binds the USER partition and arms the boot
 *                                time motor-parameter restore scan (non-blocking).
 * 5. IO_Init()                 - GPIO pins (ADC / TIM1 / control / SW1).
 * 6. LED_Init()                - WS2812 chain (PC6, TIM8_CH1 + DMA) and the
 *                                boolean status callback registered at index 0
 *                                (driven by TunePulseHeart::tick() as the
 *                                FAULT indicator, matching the CLN17 red LED).
 * 7. ADC_Init()                - triple-ADC simultaneous backend (I_A/I_B dual
 *                                mode, I_C standalone, VSENS + NTC TEMP).
 * 8. PWM_Init()                - TIM1 complementary PWM (center-aligned).
 * 9. SPI1_Init() + Encoder_Init() - shared SPI1 bus in MT6835 DMA mode; the
 *                                angle polling loop starts here and, exactly
 *                                like on CLN17 v3, POSITION_Callback() drives
 *                                TP.tick() from the first fresh angle frame
 *                                (tick() self-gates on ADC data freshness,
 *                                and the NotCalibrated state holds the phases
 *                                shorted until the offset calibration ends).
 * 10. NVIC_Init()              - TIM1_CC priority; DMA priority was armed by
 *                                Encoder_Init(), EXTI by DRV_ERR_Init().
 * 11. DRV_ERR_Init()           - PB1 nFAULT (EXTI1, both edges): falling edge
 *                                sets the external fault, rising edge clears it.
 * 12. DRV_EN (PB12) = HIGH     - gate driver enabled (wake + CSA outputs).
 * 13. Protection limits        - software-only thresholds (hardware OC/UVLO/
 *                                OTSD is delegated to the DRV8353, there are
 *                                no MCU comparators/DACs on this board).
 * 14. Fault callbacks          - OC/ext/UV/OT/OP/phase faults force Hi-Z,
 *                                OV forces all-low (same policy as CLN17 v3).
 * 15. Current-sense offset calibration - blocking wait until complete
 *                                (offsets are captured by the DRV8353 CSA
 *                                through the ADC, Hi-Z/low-side states only).
 * 16. Motor angle calibration  - blocking wait until MotorStatus::Success or
 *                                MotorStatus::CalibError.
 *
 * @note This function blocks until both calibrations finish; it must not be
 *       called before the interrupts required by those calibrations (ADC DMA,
 *       TIM1_CC) are armed below.
 */
inline void tunepulse_init() {
    delay(100);
    SystemClock_Config();  // Configure system clock to 170 MHz (HSI PLL)
    // Non-fatal clock status: report the configuration result so a bring-up
    // failure is visible on the console instead of hanging silently.
    SerialUSB.print("System clock: 0x");
    SerialUSB.println(SystemClock_Status, HEX);

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

    LED_Init();  // WS2812 chain: TIM8_CH1 + DMA1_Channel3, line parked LOW

    // Register the status-LED callback in TunePulseHeart (index 0 = FAULT
    // indicator, driven from the system tick; the WS2812 frame itself is
    // streamed non-blocking from the 20 kHz control ISR).
    TP.set_led_callback(0, LED_WS2812_Control);

    LED_Off();     // Both WS2812 pixels dark initially (applied on first tick)
    LED_Update();  // Push the "all dark" frame out right away

    delay(100);

    ADC_Init();  // Initialize the triple-ADC simultaneous backend

    delay(100);

    PWM_Init();  // Initialize TIM1 PWM (complementary mode, center-aligned)

    delay(100);

    // Shared SPI1 bus + MT6835 DMA angle polling. SPI1_Init() is idempotent
    // and Encoder_Init() starts the DMA polling loop; from the first frame
    // POSITION_Callback() drives TP.tick() exactly like on CLN17 v3.
    SPI1_Init();
    Encoder_Init();

    delay(100);

    NVIC_Init();  // Configure TIM1_CC interrupt priorities

    delay(500);

    DRV_ERR_Init();  // PB1 nFAULT (EXTI1): falling edge = fault, rising = clear

    // Gate-driver enable: DRV_EN (PB12) was driven LOW by IO_Init(); take the
    // DRV8353 out of sleep so the CSA outputs drive I_A/I_B/I_C. This is a
    // GPIO enable line, NOT a register write - the gate driver keeps its
    // hardware defaults (read-only SPI policy, see run.h).
    LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_12);  // DRV_EN = 1

    delay(100);

    // Protection limits (software only on this board - the DRV8353 handles
    // the hardware VDS/SEN overcurrent, UVLO and OTSD internally and reports
    // through nFAULT / DRV_ERR_Init()):
    // - OC (software overcurrent) trip at 4.0 A on phases A/B and 6.2 A on
    //   phase C (full scales are provisional bring-up defaults, see target.h).
    // - Bus undervoltage 16.0 V / overvoltage 51.0 V.
    // - OT (overtemperature) trip at 85 degC with a 10 degC hysteresis band;
    //   the fault releases only after the filtered temperature falls back
    //   below 75 degC (no chatter around one threshold).
    // - Overpower: 60 W / 2.5 A (bring-up defaults, same policy as CLN17 v3).
    TP.driver_phy.current.overcurrent.set_limit_amps().oc_ia(4.0).oc_ib(4.0).oc_ic(4.0);
    TP.driver_phy.supply.set_limit_volts().uv_lim_volts(16.0).ov_lim_volts(51.0);
    TP.driver_phy.temperature.set_limit_celsius().ot_lim_celsius(85.0f).ot_hyst_celsius(10.0f);
    TP.driver_phy.power.overpower.set_limit().op_lim_watts(60.0f).op_lim_amps(2.50f);
    // Raw -> degC recalculation callback (external NTC 100k on PC4,
    // temperature.h piecewise-beta model): Temperature converts the filtered
    // internal-sensor reading into real degrees Celsius and compares the OT
    // limits against that value. Installing the callback re-inverts the
    // configured thresholds through the calibration curve, so the call order
    // relative to set_limit_celsius() does not matter.
    TP.driver_phy.temperature.set_do_conversion_cb(Temperature_NTC_To_Celsius);


    TP.driver_phy.current.on_calibration_start(start_curnt_cal);
    TP.driver_phy.current.on_calibration_complete(end_curnt_cal);

    TP.driver_phy.pwm.set_period(PWM_TIMER_PERIOD);
    TP.driver_phy.set_fault()
        .set_callback()
        .oc_cb(PWM_Set_All_HiZ)
        .ov_cb(PWM_Set_All_Low)
        .ext_cb(PWM_Set_All_HiZ)   // nFAULT (DRV8353): Hi-Z safe state
        .uv_cb(PWM_Set_All_HiZ)
        .ot_cb(PWM_Set_All_HiZ)    // Software overtemperature: Hi-Z safe state
        .op_cb(PWM_Set_All_HiZ).phase_cb(PWM_Set_All_HiZ);  // Software overpower: Hi-Z safe state
    TP.driver_phy.pwm.set_callback().apply_phases_cb(TIM1_Set_PWM_Values);

    delay(100);

    // Current-sense offset calibration:
    // start_curnt_cal() ties all phases to VBUS (all high-side ON) and
    // end_curnt_cal() ties them to GND (all low-side ON); in both states the
    // differential voltage is zero, so the ADC captures the DRV8353 CSA
    // offsets only.
    TP.driver_phy.current.calibrate();
    SerialUSB.println("Start current calibration");

    // Blocking wait for completion. Completion is expected to be signalled
    // from the ADC/interrupt context; the periodic tick() call is disabled
    // here, so if no interrupt advances the calibration state machine this
    // loop will spin forever.
    while (TP.driver_phy.current.is_calibrated() == false) {
        BOOT::poll();  // hold SW1 for 3 s to enter the USB bootloader
    }

    SerialUSB.println("Current sensing setup complete");

    // Motor (electrical-angle) calibration:
    // sets the driver status to DriverCalibration and starts the automatic
    // calibration sequence (rotor alignment + sweep). Waits in a blocking
    // loop until the status becomes Success or CalibError. As above, the
    // tick() call is disabled here, so progress depends entirely on
    // interrupt-driven updates.
    TP.current_driver.setup_status.set_driver_status(OperationStatus::DriverCalibration);
    TP.current_driver.start_motor_calibration();

    while (TP.current_driver.get_motor_status() != MotorStatus::Success) {
        BOOT::poll();  // hold SW1 for 3 s to enter the USB bootloader
        if (TP.current_driver.get_motor_status() == MotorStatus::CalibError) {
            SerialUSB.println("Motor calibration Error");
            break;
        }
    }

    // Boot complete: the encoder DMA interrupt has been driving
    // TP.encoder.set_raw_position() + TP.tick() (the whole FOC machinery)
    // on every PWM-period angle sample since Encoder_Init(), the same way
    // as on CLN17 v3.
    SerialUSB.println("TunePulse stack ready");
}

#endif  // TUNEPULSE_CLN234_H