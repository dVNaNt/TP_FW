/**
 * @file main.cpp
 * @brief Firmware entry point for the TunePulse project.
 *
 * Two build targets are selected by the -D TARGET_* compiler flag:
 * - TARGET_CLN234 : READ-ONLY SPI register dump over the shared SPI1 bus.
 *                   Every 2 s the bus is switched to DRV8353 mode (16-bit,
 *                   SPI mode 1) to print the gate-driver register map
 *                   0x00-0x07, then to MT6835 mode (8-bit, SPI mode 3) to
 *                   print the encoder register map + a live angle burst.
 *                   No register is ever written on either chip.
 * - otherwise     : CLN17 V3 CAN chunk bridge demo (console text over CAN-FD).
 *
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#include <Arduino.h>

#if defined(TARGET_CLN234)

#include "clock_setup.h"  // 170 MHz PLL-from-HSI clock bring-up
#include "spi_setup.h"    // Shared SPI1 bus: DRV8353 + MT6835 read-only access
#include "lib/dev_log.h"  // TP_LOG* macros (also pulled in by spi_setup.h)

/**
 * Motor-control hook required by spi_setup.h (called from the encoder DMA
 * ISR after each angle frame). This build has no control loop, so it is
 * intentionally empty.
 */
void POSITION_Callback() {}

/**
 * @brief Configure the gate-driver control pins for SPI access.
 *
 * - DRV_EN (PB12) = HIGH: takes the DRV8353 out of sleep so its SPI
 *   interface responds (and the CSA outputs drive I_A/I_B/I_C). This is a
 *   GPIO enable line, NOT a register write - the gate driver keeps its
 *   hardware defaults.
 * - DRV_ERR (PB1) = input with pull-up: the nFAULT state is reported in
 *   the periodic log.
 */
static void DRV_Pins_Init() {
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;      // Clock for GPIO port B

    // ---- PB12: DRV_EN, push-pull output, driven HIGH ----
    GPIOB->MODER &= ~(3U << (12 * 2));
    GPIOB->MODER |=  (1U << (12 * 2));        // PB12 = digital output
    GPIOB->OTYPER &= ~GPIO_OTYPER_OT12;       // Push-pull driver
    GPIOB->BSRR = GPIO_PIN_12;                // DRV_EN = 1: wake the DRV8353 SPI

    // ---- PB1: DRV_ERR / nFAULT, input with pull-up ----
    GPIOB->MODER &= ~(3U << (1 * 2));         // PB1 = digital input
    GPIOB->PUPDR &= ~(3U << (1 * 2));
    GPIOB->PUPDR |=  (1U << (1 * 2));         // Pull-up: idle-high nFAULT
}

/**
 * @brief Print the DRV8353 register map (0x00-0x07) over USB CDC.
 *
 * Switches the shared SPI1 bus into gate-driver mode (16-bit frames, SPI
 * mode 1, 5.3125 MHz) and performs one read frame per register. Every
 * frame carries R/W = 1 (bit 15), so the transaction is always a read -
 * the DRV8353 registers are never written.
 */
static void DRV8353_DumpRegisters() {
    SPI_Bus_ConfigureDRV();  // 16-bit frames, SPI mode 1 (CPOL=0, CPHA=1)
    SerialUSB.println("--- DRV8353 registers (SPI1 mode 1, 5.3125 MHz) ---");

    static const struct { uint8_t addr; const char* name; } regs[] = {
        {0x00, "FAULT"},     // Fault status register 1 (cleared by write only - none here)
        {0x01, "VGS"},       // Fault status register 2 (VGS gate-drive faults)
        {0x02, "DRV_CTRL"},  // Driver control
        {0x03, "GATE_HS"},   // Gate drive high-side settings
        {0x04, "GATE_LS"},   // Gate drive low-side settings
        {0x05, "OCP_CTRL"},  // Overcurrent protection control
        {0x06, "CSA_CTRL"},  // Current-shunt-amplifier control
        {0x07, "RESERVED"},  // Reserved address (reads 0x000)
    };

    for (uint8_t i = 0; i < (sizeof(regs) / sizeof(regs[0])); i++) {
        const uint16_t val = DRV8353_ReadReg(regs[i].addr);  // One 16-bit read frame
        SerialUSB.print("  0x0");                            // Fixed-width address column
        SerialUSB.print(regs[i].addr, HEX);
        SerialUSB.print(" ");
        SerialUSB.print(regs[i].name);
        SerialUSB.print(" : 0x");
        if (val < 0x100U) SerialUSB.print('0');              // Pad data to 3 hex digits
        SerialUSB.println(val, HEX);
    }
}

/**
 * @brief Print the MT6835 register map + one angle burst over USB CDC.
 *
 * Switches the shared SPI1 bus into encoder mode (8-bit frames, SPI mode 3,
 * 5.3125 MHz), reads every documented register with individual 24-bit
 * OP_READ frames (read-only), then issues the read-only OP_ANGLE burst to
 * sample the live angle / status / CRC.
 */
static void MT6835_DumpRegisters() {
    SPI_Bus_ConfigureEncoder();  // 8-bit frames, SPI mode 3 (CPOL=1, CPHA=1)
    SerialUSB.println("--- MT6835 registers (SPI1 mode 3, 5.3125 MHz) ---");

    static const struct { uint16_t reg; const char* name; } regs[] = {
        {0x001, "USERID"},             // User ID register
        {0x003, "ANGLE[20:13]"},       // Angle high byte
        {0x004, "ANGLE[12:5]"},        // Angle middle byte
        {0x005, "ANGLE[4:0]+STATUS"},  // Angle low bits + STATUS[2:0]
        {0x006, "ANGLE_CRC8"},         // CRC-8 over the angle frame
        {0x007, "ABZ_RES1"},           // ABZ resolution low byte
        {0x008, "ABZ_RES2"},           // ABZ resolution high byte
        {0x009, "ZERO[11:4]"},         // Zero position high byte
        {0x00A, "ZERO[3:0]/OPTS0"},    // Zero position low + Z pulse width
        {0x00B, "OPTS1"},              // UVW resolution / offset / mux options
        {0x00C, "OPTS2"},              // PWM mode / NLC enable options
        {0x00D, "OPTS3"},              // Hysteresis / rotation direction
        {0x00E, "OPTS4"},              // Autocal frequency / GPIO drive
        {0x011, "OPTS5"},              // Bandwidth option
        {0x113, "CAL_STATUS"},         // Self-calibration status
    };

    for (uint16_t i = 0; i < (sizeof(regs) / sizeof(regs[0])); i++) {
        const uint8_t val = MT6835_ReadReg(regs[i].reg);  // One 24-bit read frame
        SerialUSB.print("  0x");                          // Address column
        if (regs[i].reg < 0x100U) SerialUSB.print('0');   // Keep 3-digit addresses aligned
        SerialUSB.print(regs[i].reg, HEX);
        SerialUSB.print(" ");
        SerialUSB.print(regs[i].name);
        SerialUSB.print(" : 0x");
        if (val < 0x10U) SerialUSB.print('0');            // Pad data to two hex digits
        SerialUSB.println(val, HEX);
    }

    // Read-only angle burst: OP_ANGLE (0b1010) + start address 0x003, then
    // 4 dummy bytes whose received bytes carry the angle frame.
    SPI1_EncChipSelect(true);                     // Assert ENC_CS (PA4)
    (void)SPI1_Transfer8(0xA0);                   // OP_ANGLE opcode (read-only command)
    (void)SPI1_Transfer8(0x03);                   // Burst start address 0x003
    const uint8_t b0 = SPI1_Transfer8(0x00);      // ANGLE[20:13]
    const uint8_t b1 = SPI1_Transfer8(0x00);      // ANGLE[12:5]
    const uint8_t b2 = SPI1_Transfer8(0x00);      // ANGLE[4:0] + STATUS[2:0]
    const uint8_t b3 = SPI1_Transfer8(0x00);      // CRC-8 (poly 0x07)
    SPI1_EncChipSelect(false);                    // Release ENC_CS

    const uint32_t angle = ((uint32_t)b0 << 13) | ((uint32_t)b1 << 5) | (b2 >> 3);  // 21-bit raw angle
    SerialUSB.print("  angle burst: raw=0x");
    SerialUSB.print(angle, HEX);
    SerialUSB.print(" status=");
    SerialUSB.print(b2 & 0x07U, HEX);   // STATUS[2:0]: overspeed / weak field / undervoltage
    SerialUSB.print(" crc=0x");
    if (b3 < 0x10U) SerialUSB.print('0');
    SerialUSB.println(b3, HEX);
}

/**
 * @brief One-time init: clocks, USB console, gate-driver enable, SPI1 bus.
 */
void setup() {
    SystemClock_Config();  // 170 MHz from HSI (non-fatal on failure)

    SerialUSB.begin();  // USB CDC console
    delay(1500);        // Let the USB stack enumerate before printing
    SerialUSB.println("=== CLN234 SPI register dump (READ-ONLY, no writes) ===");
    SerialUSB.print("SystemCoreClock: ");
    SerialUSB.println(SystemCoreClock);

    DRV_Pins_Init();  // DRV_EN = 1 (wakes the DRV8353 SPI), nFAULT input pull-up
    SPI1_Init();      // SPI1 + ENC_CS/DRV_CS + DMA routing; bus boots in MT6835 mode
}

/**
 * @brief Main loop: dump both chips' registers every 2 s over USB CDC.
 *
 * Each cycle first switches SPI1 into DRV8353 mode (16-bit, mode 1) and
 * prints the gate-driver register map, then reconfigures the SAME bus for
 * the MT6835 (8-bit, mode 3) and prints the encoder register map plus a
 * live angle burst sample. Reconfiguration around every device switch is
 * the essential part of the shared-bus arbitration (mode 1 vs mode 3).
 */
void loop() {
    static uint32_t t_dump = 0;               // Dump pacer timestamp
    if ((millis() - t_dump) < 2000U) {
        return;                               // 2 s report interval
    }
    t_dump = millis();                        // Restart the pacer

    SerialUSB.print("[t=");
    SerialUSB.print(millis() / 1000U);
    SerialUSB.println("s]");

    DRV8353_DumpRegisters();  // Bus switches to 16-bit mode 1 inside
    MT6835_DumpRegisters();   // Bus switches back to 8-bit mode 3 inside

    // External status lines: nFAULT (PB1, active low) and the DRV_EN output
    // level (must read 1 for any DRV8353 SPI response at all).
    SerialUSB.print("  drv_en=");
    SerialUSB.print((GPIOB->ODR >> 12) & 1U);   // DRV_EN output latch state
    SerialUSB.print(" nfault=");
    SerialUSB.println((GPIOB->IDR >> 1) & 1U);  // Live nFAULT pin level
}

#else  // ==================== CLN17 V3 CAN bridge demo ====================

#include "target/cln17_v3/can_setup.h"  // Merged CAN driver + chunk API

#include "tunepulse.h"    // Board bring-up helpers (BOOT button polling)

/** CAN identifier used for peer-to-peer console text. */
static constexpr uint32_t MSG_ID = 0x123;

/** Serial line assembly limit (console -> chunk staging). */
static constexpr uint16_t TEXT_LINE_MAX = 120;

/** Serial line assembly buffer (console -> chunk staging). */
static char line_buf[TEXT_LINE_MAX];

/** Characters currently stored in the line buffer. */
static uint16_t line_len = 0;

/**
 * @brief One-time initialization: USB console, then the CAN-FD interface.
 */
void setup() {
    BOOT::init_button();

    // Power-rail / USB stack stabilization; SW1 is polled meanwhile, so
    // holding it during power-up counts towards the 3 s bootloader trigger.
    const uint32_t t0 = millis();
    while ((millis() - t0) < 2000U) {
        BOOT::poll();  // may never return (enters USB DFU bootloader)
        delay(BOOT::POLL_INTERVAL_MS);
    }

    SerialUSB.begin();
    SerialUSB.println("=== TunePulse CAN chunk bridge (1 Mbps / 5 Mbps FD) ===");
    tunepulse_init();
    if (!CAN_BUS.begin()) {
        SerialUSB.println("CAN init FAILED");
    }
}

/**
 * @brief Main loop: console text -> write() -> CAN, received chunks -> console.
 */
void loop() {
    BOOT::poll();  // hold SW1 for 3 s to enter the USB bootloader

    // --- Console -> CAN: send complete lines as whole messages --------------
    while (SerialUSB.available() > 0) {          // Drain pending console bytes
        const int b = SerialUSB.read();          // Fetch one byte
        if (b < 0) {
            break;                               // Buffer drained
        }
        const char c = static_cast<char>(b);     // Interpret as character
        if ((c == '\n') || (c == '\r')) {        // Line complete
            if (line_len > 0) {
                line_buf[line_len++] = '\n';     // Line terminator byte
                line_buf[line_len] = '\0';       // Terminate for logging only
                CAN_BUS.write(MSG_ID, line_buf, line_len);  // All chunks queued
                line_len = 0;                    // Restart the line buffer
            }
        } else if (line_len < (TEXT_LINE_MAX - 1U)) {
            line_buf[line_len++] = c;            // Append in-range character
        } else {
            line_len = 0;                        // Overflow: drop the line
        }
    }

    // --- CAN -> console: echo received chunks until a newline byte ---------
    CAN_BUS.update();                        // Pump TX frame ring -> HW FIFO
    uint8_t chunk[CANDriver::CHUNK_SIZE];    // Received chunk storage
    bool line_done = false;                  // Newline encountered flag
    while (CAN_BUS.is_rx_available()) {      // Drain the whole RX queue
        uint32_t cid = 0;                    // Chunk identifier (unused)
        if (!CAN_BUS.read(cid, chunk)) {
            break;                           // Queue drained
        }
        for (uint8_t i = 0; i < CANDriver::CHUNK_SIZE; i++) {  // Echo bytes
            const uint8_t c = chunk[i];      // One payload byte
            if (c == '\n') {                 // Line terminator
                line_done = true;            // End of the console line
                break;                       // Newline inside this chunk
            }
            if (c != 0U) {                   // Skip zero padding
                SerialUSB.write(c);          // Echo one byte
            }
        }
        if (line_done) {
            SerialUSB.println();             // Terminate the console line
            line_done = false;               // Continue with the next line
        }
        (void)cid;                           // ID unused in this demo
    }
}

#endif  // TARGET_CLN234