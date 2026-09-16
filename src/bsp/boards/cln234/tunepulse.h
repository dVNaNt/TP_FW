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
 *                        fault). On top of that, the MCU COMPARATORS are
 *                        wired EXACTLY like on CLN17 v3 (same pins, same
 *                        DAC threshold parameters, see comp_setup.h):
 *                        COMP1 (I_B/PA1), COMP2 (I_A/PA3), COMP4 (I_C/PB0)
 *                        for per-phase hardware overcurrent and COMP3
 *                        (VSENS/PA0) for hardware overvoltage.
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

#include "core/lib.h"  // TunePulseHeart + the full motor-control stack

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
// DEFINED in board.cpp, the board's single translation unit, so that exactly
// one object file owns it. Declared here because nearly every board header and
// the interrupt glue refer to it.
extern TunePulseHeart TP;

// Board bring-up headers. Order matters: the ISR glue and the fault hooks
// reference the TP object, so they are included after its construction.
#include "ws2812.h"           // WS2812 status LED (TIM8_CH1 + DMA)
#include "bsp/stm32g4/bootloader.h"  // SW1 (PC14) hold-to-bootloader (USB DFU)
#include "interrupt_setup.h"  // ISR glue + NVIC (pulls clock/gpio/adc/spi/timer_pwm/drv_err/temperature)
#include "bsp/stm32g4/comp_dac.h"       // Comparator (hardware OC/OV) protection (same circuit as CLN17 v3)
#include "bsp/stm32g4/cordic.h"     // CORDIC hardware trigonometry accelerator
#include "bsp/stm32g4/crc.h"        // CRC unit backend of core/math/crc.h (active with TP_CRC_HW)
#include "bsp/stm32g4/nvm_flash.h"        // Internal-flash NVM partitions (FlashG4_LL)
#include "bsp/stm32g4/fdcan.h"        // CAN-FD driver + chunk API (CAN_BUS)

/**
 * @brief LED apply callback for the WS2812 chain (PC6).
 *
 * The library LED driver hands over TWO packed RGB888 words (0x00RRGGBB, one
 * per LED element), which map 1:1 onto the two daisy-chained WS2812B pixels.
 * The WS2812 driver accepts exactly this int32-pair format, so the words are
 * forwarded unchanged. Colors are only written to the WS2812 shadow buffer -
 * LED_Update() streams the frame out non-blocking (pumped from the 20 kHz
 * control ISR, see interrupt_setup.h).
 *
 * @param led1 Packed RGB888 color for LED element 1 (WS2812 pixel 1).
 * @param led2 Packed RGB888 color for LED element 2 (WS2812 pixel 2).
 */
void LED_Apply(int32_t led1, int32_t led2) {
    LED_SetRGB2(led1, led2);  // Forward the packed pair into the shadow buffer (IRQ-safe, ~cycles)
    LED_Update();             // Arm the non-blocking frame transfer
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
 * @brief Print a 16-bit word as 4 padded hex digits (0x0000..0xFFFF).
 *
 * SerialUSB::print(v, HEX) drops leading zeros, which makes raw SPI frame
 * logs hard to read; this helper always prints exactly 4 hex digits.
 * @param v Word to print.
 */
static void DRV_PrintHex16(uint16_t v) {
    if (v < 0x1000) SerialUSB.print('0');  // Pad thousands
    if (v < 0x0100) SerialUSB.print('0');  // Pad hundreds
    if (v < 0x0010) SerialUSB.print('0');  // Pad tens
    SerialUSB.print(v, HEX);               // Hex digits (no prefix)
}

/** Short register names indexed by DRV8353S::Reg for console dumps. */
static const char* const DRV_REG_NAMES[8] = {
    "FAULT1", "VGS", "CTRL", "GATE_HS", "GATE_LS", "OCP", "CSA", "CONFIG",
};

/**
 * @brief Gate-driver enable callback: DRV_EN (PB12) + DRV8353S configuration.
 *
 * Registered in TunePulseHeart (TP.driver_phy.get_enable()). On enable:
 * 1. DRV_EN (PB12) is held LOW for 1.1 s: per TI (SLVSDY6A) the DRV835x
 *    SPI registers keep their values across ordinary sleep periods and are
 *    only guaranteed to reset after the EN pin has been low for ~1 s. The
 *    forced reset gives the configuration pass a deterministic all-defaults
 *    starting state (leftover values from earlier sessions were observed
 *    live: OCP=0x0B6 / CSA=0x141 retained across reboots).
 * 2. DRV_EN is driven HIGH, taking the DRV8353S out of sleep (the SPI
 *    bus needs VM power, which the DRV_EN line gates). A 30 ms wake delay
 *    covers the nSLEEP wake-up time before the first SPI frame.
 * 3. The shared SPI1 bus is parked: the MT6835 DMA angle polling loop is
 *    paused and the in-flight frame is drained, then the bus is switched
 *    to the DRV8353 (16-bit frames, SPI mode 1 levels, GPIO bit-banged).
 * 4. PRE-WRITE DEFAULT DUMP: every register (0x00..0x07) is read TWICE and
 *    the RAW 16-bit frames are logged with the expected power-on values
 *    (DRV8353S::RESET). After the forced reset above every register must
 *    match; double reads expose SDO pipeline latency and the raw frames
 *    expose bit-slip / frame corruption on the bus.
 * 5. The configuration registers 0x02..0x06 are written. The register
 *    images start from the observed power-on values (DRV8353S::RESET) and
 *    change ONLY the requested fields - every other field stays at its
 *    default:
 *      - CTRL   : OCP_ACT=1 (fault shuts down all half-bridges),
 *                 OTW_REP=1 (overtemperature warning is report-only)
 *      - GATE_HS: IDRIVEP_HS=0100b, IDRIVEN_HS=0100b (LOCK[10:8] stays
 *                 011b = unlocked)
 *      - GATE_LS: CBC=0, IDRIVEP_LS=0110b, IDRIVEN_LS=0110b
 *      - OCP    : TRETRY=1 (50 us retry), OCP_DEG=01b, VDS_LVL=0100b
 *      - CSA    : SEN_LVL=00b (everything else default)
 *    Fault registers (0x00/0x01) are never written (a write clears latched
 *    faults) and CONFIG (0x07) keeps its power-on default. Each write is
 *    verified by TWO readback frames and retried up to 3 times; the raw
 *    SDO response of the write frame is logged for diagnostics.
 * 6. A diagnostic single-bit write scan on GATE_LS proves that every DATA
 *    bit position survives a write both ways, then restores the intended
 *    value.
 * 7. The shared bus is handed back to the MT6835 DMA polling loop.
 *
 * On disable: DRV_EN (PB12) is driven LOW only (hardware sleep), the SPI
 * bus is not touched.
 *
 * @param state Requested gate-driver state (true = enable + configure).
 */
void DRV_Enable_Callback(bool state) {
    if (state == false) {
        LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_12);  // DRV_EN = 0: DRV8353S sleep
        return;
    }

    // Forced register reset: DRV_EN (PB12) LOW for >1 s. The DRV835x keeps
    // SPI register values across short sleep periods; only a sustained EN
    // low guarantees the power-on defaults restored before the dump.
    LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_12);  // DRV_EN = 0
    delay(1100);                                    // >1 s: guaranteed register reset
    SerialUSB.println("DRV8353S registers reset (EN low 1.1 s)");

    LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_12);  // DRV_EN = 1: wake the gate driver
    delay(30);  // nSLEEP wake-up time + internal LDO/charge pump before the first SPI frame
    // Park the shared SPI1 bus: stop arming new MT6835 DMA transfers, wait
    // for the in-flight frame to complete, then switch to DRV8353 mode
    // (16-bit, mode 1, polled - DMA requests are detached by the mode switch).
    SPI1_ENCODER_POLLING = false;
    while (SPI1_TRANSFER_ACTIVE) {}  // Drain the last encoder frame
    SPI_Bus_ConfigureDRV();

    // ---- Step 1: pre-write dump of the power-on defaults (read-only) ----
    // Every register is read twice: rd1 == rd2 rules out SDO pipeline
    // latency, matching DRV8353S::RESET[] (the factory defaults, restored
    // by the forced EN-low reset above) rules out bus corruption.
    SerialUSB.println("DRV8353S power-on defaults (raw frames):");
    bool def_ok = true;  // Overall defaults verdict
    for (uint8_t a = DRV8353S::REG_FAULT1; a <= DRV8353S::REG_CONFIG; a++) {
        const uint16_t r1 = DRV8353_ReadReg(a);  // First read frame
        const uint16_t r2 = DRV8353_ReadReg(a);  // Second read frame
        const bool ok = (DRV8353S::Data(r2) == DRV8353S::RESET[a]);
        def_ok = def_ok && ok;
        SerialUSB.print("  ");
        SerialUSB.print(DRV_REG_NAMES[a]);
        SerialUSB.print(": rd1=0x");
        DRV_PrintHex16(r1);
        SerialUSB.print(" rd2=0x");
        DRV_PrintHex16(r2);
        SerialUSB.print(" exp=0x");
        DRV_PrintHex16(DRV8353S::RESET[a]);
        SerialUSB.println(ok ? " [OK]" : " [MISMATCH]");
    }
    SerialUSB.println(def_ok ? "DRV8353S defaults verified"
                             : "DRV8353S defaults MISMATCH (check SPI bus / device variant)");

    // Register images: power-on defaults with only the requested fields
    // modified (all computed at compile time where possible).
    const uint16_t cfg_ctrl =  // 0x02 DRIVER_CTRL
        DRV8353S::Put(DRV8353S::Put(DRV8353S::RESET[DRV8353S::REG_CTRL],
                                    DRV8353S::F::OCP_ACT, 1),
                      DRV8353S::F::OTW_REP, 1);
    const uint16_t cfg_gate_hs =  // 0x03 GATE_DRIVE_HS
        DRV8353S::Put(DRV8353S::Put(DRV8353S::RESET[DRV8353S::REG_GATE_HS],
                                    DRV8353S::F::IDRIVEP_HS, 0x4),
                      DRV8353S::F::IDRIVEN_HS, 0x4);
    const uint16_t cfg_gate_ls =  // 0x04 GATE_DRIVE_LS
        DRV8353S::Put(DRV8353S::Put(DRV8353S::Put(
                                        DRV8353S::RESET[DRV8353S::REG_GATE_LS],
                                        DRV8353S::F::CBC, 0),
                                   DRV8353S::F::IDRIVEP_LS, 0x6),
                      DRV8353S::F::IDRIVEN_LS, 0x6);
    const uint16_t cfg_ocp =  // 0x05 OCP_CTRL
        DRV8353S::Put(DRV8353S::Put(DRV8353S::Put(
                                        DRV8353S::RESET[DRV8353S::REG_OCP],
                                        DRV8353S::F::TRETRY, 1),
                                   DRV8353S::F::DEGLITCH, 1),
                      DRV8353S::F::VDS_LVL, 0x4);
    const uint16_t cfg_csa =  // 0x06 CSA_CTRL
        DRV8353S::Put(DRV8353S::RESET[DRV8353S::REG_CSA],
                      DRV8353S::F::SEN_LVL, 0);

    // ---- Step 2: program the configuration registers (write + readback) ----
    // Write + readback verification table (address, expected data).
    struct DrvReg {
        uint8_t addr;      // Register address (0x00-0x07)
        uint16_t val;      // Expected 11-bit DATA payload
    };
    const DrvReg cfg[5] = {
        {DRV8353S::REG_CTRL, cfg_ctrl},
        {DRV8353S::REG_GATE_HS, cfg_gate_hs},
        {DRV8353S::REG_GATE_LS, cfg_gate_ls},
        {DRV8353S::REG_OCP, cfg_ocp},
        {DRV8353S::REG_CSA, cfg_csa},
    };

    SerialUSB.println("DRV8353S configuration:");
    bool ok_all = true;  // Overall configuration verdict
    for (const DrvReg& r : cfg) {
        bool reg_ok = false;  // Per-register verdict (after retries)
        for (uint8_t attempt = 1; attempt <= 3 && !reg_ok; attempt++) {
            const uint16_t wr_sdo = DRV8353_WriteReg(r.addr, r.val);  // Write + raw SDO response
            delayMicroseconds(50);                         // Settle time: let the commit propagate before readback
            const uint16_t rd1 = DRV8353_ReadReg(r.addr);  // First readback frame
            const uint16_t rd2 = DRV8353_ReadReg(r.addr);  // Second readback frame
            reg_ok = (DRV8353S::Data(rd2) == r.val);       // Verdict on the settled frame
            ok_all = ok_all && reg_ok;
            SerialUSB.print("  ");
            SerialUSB.print(DRV_REG_NAMES[r.addr]);
            SerialUSB.print(" try");
            SerialUSB.print(attempt);
            SerialUSB.print(": wr=0x");
            DRV_PrintHex16(r.val);
            SerialUSB.print(" sdo=0x");
            DRV_PrintHex16(wr_sdo);
            SerialUSB.print(" rd1=0x");
            DRV_PrintHex16(rd1);
            SerialUSB.print(" rd2=0x");
            DRV_PrintHex16(rd2);
            SerialUSB.println(reg_ok ? " [OK]" : " [MISMATCH]");
        }
    }
    SerialUSB.println(ok_all ? "DRV8353S configuration verified"
                             : "DRV8353S configuration FAILED");

    // ---- Step 3: DIAGNOSTIC single-bit write scan on GATE_LS ----
    // Purpose: map exactly which DATA bit positions survive a write when ALL
    // other bits are zero. Evidence so far: SPI reads are bit-perfect (the
    // defaults dump above) and the write-frame SDO echoes the CORRECT
    // register with its OLD content (header decode is fine), but the stored
    // value equals the written value ANDed with a fixed mask (lost bits
    // {2,4,6,7,9,10}, kept {1,5,8} across ALL registers). The scan decides
    // between:
    //   - fixed-position loss  -> single bits at {2,4,6,7,9,10} never stick
    //     (device write-path / input-buffer issue), and
    //   - neighbour-dependent loss -> all single bits stick (signal
    //     integrity: bit loss only with specific 0->1->0 patterns).
    // Safe on GATE_LS: the outputs are static at this point (no switching),
    // so IDRIVE/TDRIVE/CBC values cannot cause dynamic gate events. The
    // intended configuration value is restored afterwards.
    SerialUSB.println("DRV8353S GATE_LS write scan:");
    const uint16_t probes[13] = {
        0x3FF,                        // Power-on default rewrite (expect [OK])
        0x000,                        // All zeros (expect [OK])
        0x001, 0x002, 0x004, 0x008,   // Single bits 0..3
        0x010, 0x020, 0x040, 0x080,   // Single bits 4..7
        0x100, 0x200, 0x400,          // Single bits 8..10
    };
    for (uint8_t i = 0; i < 13; i++) {
        (void)DRV8353_WriteReg(DRV8353S::REG_GATE_LS, probes[i]);  // Probe write
        const uint16_t rd = DRV8353S::Data(DRV8353_ReadReg(DRV8353S::REG_GATE_LS));
        SerialUSB.print("  wr=0x");
        DRV_PrintHex16(probes[i]);
        SerialUSB.print(" rd=0x");
        DRV_PrintHex16(rd);
        SerialUSB.println(rd == probes[i] ? " [OK]" : " [LOST]");
    }
    (void)DRV8353_WriteReg(DRV8353S::REG_GATE_LS, cfg_gate_ls);  // Restore intended value
    SerialUSB.print("  restore rd=0x");
    DRV_PrintHex16(DRV8353S::Data(DRV8353_ReadReg(DRV8353S::REG_GATE_LS)));
    SerialUSB.println("");

    // Hand the shared bus back to the MT6835 DMA angle polling loop: switch
    // SPI1 to encoder mode (8-bit, mode 3, DMA requests enabled), resume the
    // polling loop and request a fresh frame immediately.
    SPI_Bus_ConfigureEncoder();
    SPI1_ENCODER_POLLING = true;
    Encoder_StartTransfer();
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
 * 12. DRV_Enable_Callback(true) - DRV_EN (PB12) = HIGH, the DRV8353S
 *                                configuration registers (0x02..0x06) are
 *                                programmed and read back for verification
 *                                (shared SPI1 bus parked/restored around
 *                                the transaction).
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

    // Register the single LED apply callback in TunePulseHeart: two packed
    // RGB888 words map 1:1 onto the two WS2812 pixels. The fault indicator
    // (LED1 red channel) is driven from the system tick; the frame itself is
    // streamed non-blocking from the 20 kHz control ISR.
    TP.led.set_callback().apply_cb(&LED_Apply);

    TP.led.off();  // Both WS2812 pixels dark initially (applied immediately)

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

    // Gate-driver enable through the TunePulseHeart callback: the registered
    // callback drives DRV_EN (PB12) HIGH, programs the DRV8353S configuration
    // registers (OCP_ACT/OTW_REP, IDRIVEP/N_HS, CBC + IDRIVEP/N_LS, TRETRY,
    // OCP_DEG, VDS_LVL, SEN_LVL) and reads every written register back for
    // verification. The callback also parks and restores the shared SPI1 bus
    // (MT6835 DMA angle polling) around the transaction.
    //
    // DEBUG: 3 s pause so the USB CDC host can re-enumerate after a reset
    // before the one-shot DRV8353 diagnostics are printed (prints made while
    // the port is closed are lost).
    delay(3000);
    TP.driver_phy.get_enable().setCallback(&DRV_Enable_Callback);
    TP.driver_phy.get_enable().set(true);  // DRV_EN = 1 + DRV8353S register config

    delay(100);

    // Hardware comparator protection (identical to CLN17 v3): DAC1/DAC3 +
    // COMP1/COMP2/COMP3/COMP4 with the same startup thresholds and EXTI edges.
    COMP_Protection_Init();

    delay(100);

    // Protection limits:
    // - Hardware OC thresholds: the DAC threshold callbacks below convert the
    //   amp limits into 12-bit DAC codes feeding COMP1/COMP2/COMP4 (per-phase
    //   hardware overcurrent shutdown). CLN234 channel pairing: the driver's
    //   CH1/CH2/CH3 correspond to pwm_ch1/pwm_ch2/pwm_ch3, and on this board
    //   pwm_ch1 carries I_B (PA1 -> COMP1/DAC1_CH1), pwm_ch2 carries I_A
    //   (PA3 -> COMP2/DAC1_CH2) and pwm_ch3 carries I_C (PB0 -> COMP4/DAC3_CH2).
    //   Each channel's threshold (derived from that channel's own zero-current
    //   offset) is therefore routed to the comparator watching the SAME pin
    //   its current comes from - see interrupt_setup.h for the matching ADC
    //   slot reorder.
    // - OC (software overcurrent) trip at 4.0 A on phases A/B and 6.2 A on
    //   phase C (full scales are provisional bring-up defaults, see target.h).
    // - Hardware OV threshold: DAC_Set_COMP3_OV_Threshold callback feeds
    //   COMP3 (bus overvoltage, regenerative braking safe state).
    // - Bus undervoltage 16.0 V / overvoltage 51.0 V (same as CLN17 v3).
    // - OT (overtemperature) trip at 85 degC with a 10 degC hysteresis band;
    //   the fault releases only after the filtered temperature falls back
    //   below 75 degC (no chatter around one threshold).
    // - Overpower: 60 W / 2.5 A (bring-up defaults, same policy as CLN17 v3).
    TP.driver_phy.current.overcurrent.set_callback()
        .oc_lim_ia_cb(DAC_Set_COMP1_OC_Threshold)  // CH1 = pwm_ch1 = I_B (PA1) -> COMP1 / DAC1_CH1
        .oc_lim_ib_cb(DAC_Set_COMP2_OC_Threshold)  // CH2 = pwm_ch2 = I_A (PA3) -> COMP2 / DAC1_CH2
        .oc_lim_ic_cb(DAC_Set_COMP4_OC_Threshold);  // CH3 = pwm_ch3 = I_C (PB0) -> COMP4 / DAC3_CH2
    TP.driver_phy.supply.overvoltage.set_lim_callback(DAC_Set_COMP3_OV_Threshold);
    TP.driver_phy.current.overcurrent.set_limit_amps().oc_ia(10.0).oc_ib(10.0).oc_ic(10.0);
    TP.driver_phy.supply.set_limit_volts().uv_lim_volts(10.0).ov_lim_volts(51.0);
    TP.driver_phy.temperature.set_limit_celsius().ot_lim_celsius(85.0f).ot_hyst_celsius(10.0f);
    TP.driver_phy.power.overpower.set_limit().op_lim_watts(600.0f).op_lim_amps(10.50f);
    TP.driver_phy.motor_ctrl.set_current_epsilon(5000);
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
    SerialUSB.println(TP.driver_phy.current.cal_val_.a);
    SerialUSB.println(TP.driver_phy.current.cal_val_.b);
    SerialUSB.println(TP.driver_phy.current.cal_val_.c);
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