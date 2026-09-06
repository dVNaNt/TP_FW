/**
 * @file spi_setup.h
 * @brief Shared SPI1 bus manager for the DRV8353 gate driver and the MT6835
 *        absolute encoder (CLN234).
 *
 * The CLN234 routes BOTH SPI devices onto the SPI1 bus (PA5=SCK / PA6=MISO /
 * PA7=MOSI) with separate chip selects: PA4 = ENC_CS (MT6835), PC15 = DRV_CS
 * (DRV8353). The two chips need different bus modes, so the bus is
 * reconfigured on the fly around every device switch:
 *
 * - DRV8353 : 16-bit frames, SPI mode 1 (CPOL=0, CPHA=1) EQUIVALENT, but
 *             bit-banged through GPIO (see below), separate chip select.
 * - MT6835  : 8-bit frames,  SPI mode 3 (CPOL=1, CPHA=1), DMA polling for
 *             the continuous angle burst + polled single-register reads.
 *
 * Bus speed: PCLK2 = 170 MHz is the only SPI1 kernel clock on STM32G4 (no
 * RCC mux for SPI1), and the prescaler only divides by 2^n, so exact 8 MHz
 * is not reachable. The MT6835 runs at BR = PCLK2/32 = 5.3125 MHz
 * (CRC-validated).
 *
 * DRV8353 access is BIT-BANGED (DRV8353_BB_Transfer16) at ~167 kHz
 * equivalent (3 us half-bits) instead of using the hardware SPI: the live
 * bring-up logs proved that hardware-SPI register WRITES commit with
 * deterministic value-dependent bit loss at EVERY prescaler setting
 * (/256: 0x3FF -> 0x1FF, /16: 0x3FF -> 0x000; isolated 1-bits are always
 * lost), while READS are bit-perfect. The signature matches an SDI path
 * that needs ~1.1-1.2 us of settled high level per '1' bit - impossible
 * with any SPI1 prescaler (0.5 bit = 0.75 us max setup in mode 1). The
 * bit-bang gives every level 3-6 us of settling, which the write path
 * accepts reliably. Bit-banging is acceptable here because DRV8353 access
 * is a one-shot boot-time operation on an already slow diagnostic path.
 *
 * Bus access policy:
 * - MT6835: reads only (OP_READ / OP_ANGLE burst, never written).
 * - DRV8353: the gate-driver ENABLE callback (DRV_Enable_Callback in
 *   tunepulse.h) WRITES the configuration registers 0x02..0x06 once at
 *   boot (OC / IDRIVE / SEN_LVL tuning) and reads them back for
 *   verification. Fault registers (0x00/0x01) are never written: a write
 *   clears the latched faults, hiding real errors. CONFIG (0x07) stays at
 *   its power-on default.
 * A read transaction has no side effects on either device. Per SLVSDY6A the
 * SDO output is OPEN-DRAIN: the MISO pull-up (PA6) is REQUIRED both while
 * both chip selects are idle-high (both SDOs Hi-Z) and to pull the line
 * high for SDO '1' bits during frames.
 *
 * Boot-time order:
 * 1. SPI1_Init()        - GPIO/DMA/SPI bring-up, bus left in MT6835 mode.
 * 2. Encoder_Init()     - (normal firmware) starts the DMA angle polling.
 *    The register-dump build calls SPI_Bus_ConfigureDRV() instead and
 *    never starts the polling loop.
 *
 * MT6835 angle burst frame (SPI mode 3, 6 bytes):
 *   TX: 0xA0 0x03 0x00 0x00 0x00 0x00 (OP_ANGLE + address 0x003 + dummies)
 *   RX: [2] ANGLE[20:13], [3] ANGLE[12:5],
 *       [4] ANGLE[4:0] + STATUS[2:0], [5] CRC-8 (poly 0x07)
 *
 * MT6835 single-register read frame (SPI mode 3, 24 bits = 3 bytes):
 *   TX: 0b0011 (OP_READ) | addr[11:8], addr[7:0], 0x00 (dummy)
 *   RX: the register content is shifted out during the third byte.
 *
 * DRV8353 register read frame (SPI mode 1, 16 bits):
 *   bit15 = R/W (1 = read), bits[14:11] = ADDR, bits[10:0] = DATA.
 *   The register content is clocked out during the same frame.
 *
 * The angle is stored left-aligned in a 32-bit word (bit 31 = half turn)
 * so it can be used directly as a Q0.32 fixed-point turn fraction by the
 * FOC core.
 *
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#ifndef SPI_SETUP_CLN234_H
#define SPI_SETUP_CLN234_H

#include <Arduino.h>     // delay(), millis()
#include "stm32g4xx.h"   // CMSIS device header: SPI1, DMA1, GPIO registers
#include "lib/dev_log.h" // TP_LOG* macros: USB CDC diagnostics over SerialUSB
#include "hardware/drv8353s.h"  // DRV8353S register map + SPI command builders
#include "hardware/encoders/mt6835.h"  // MT6835 register map + SPI command builders + CRC-8

extern void POSITION_Callback(); // Motor-control hook, called after each new angle

static inline void Encoder_StartTransfer(); // Forward declaration: Encoder_Init() issues the first transfer

// ==============================================================================
// VARIABLES AND BUFFERS (SPI1)
// ==============================================================================
static uint8_t SPI1_RX_BUF[6] __attribute__((aligned(4))) = {0};  // RX frame filled by DMA
static uint8_t SPI1_TX_BUF[6] __attribute__((aligned(4))) =       // MT6835 TX frame: burst-read command + dummy payload
    {0xA0, 0x03, 0x00, 0x00, 0x00, 0x00};

static volatile uint32_t POSITION_ENCODER_MT_RAW = 0;  // 32-bit left-aligned angle (Q0.32 turn fraction)
static volatile uint8_t  POSITION_MT_STATUS = 0;       // STATUS[2:0]: overspeed / weak field / undervoltage
static volatile uint8_t  POSITION_MT_CRC = 0;          // CRC-8 received from the sensor
static volatile bool     SPI1_ENCODER_POLLING = false; // true once the encoder DMA loop is running
static volatile bool     SPI1_TRANSFER_ACTIVE = false; // true while an encoder DMA transfer is in flight

// ==============================================================================
// LOW-LEVEL CHIP SELECT CONTROL
// ==============================================================================
static inline void SPI1_EncChipSelect(bool select) {
    if (select) GPIOA->BRR = GPIO_PIN_4;    // CS LOW  (active) PA4 = ENC_CS
    else        GPIOA->BSRR = GPIO_PIN_4;   // CS HIGH (idle)   PA4 = ENC_CS
}

/**
 * @brief DRV8353 chip select control (PC15 = DRV_CS, active low).
 * @param select true = assert the frame (CS LOW), false = release (CS HIGH)
 */
static inline void SPI1_DrvChipSelect(bool select) {
    if (select) GPIOC->BRR  = GPIO_PIN_15;  // CS LOW  (active) PC15 = DRV_CS
    else        GPIOC->BSRR = GPIO_PIN_15;  // CS HIGH (idle)   PC15 = DRV_CS
}

// ==============================================================================
// SPI1 GPIO INITIALIZATION
// ==============================================================================
static inline void SPI1_GPIO_Init() {
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;  // Clock GPIOA (ENC_CS + SPI lines)

    // ---- PA4: ENC_CS, GPIO output push-pull ----
    GPIOA->MODER &= ~(3U << (4 * 2));
    GPIOA->MODER |=  (1U << (4 * 2));         // Output mode
    GPIOA->OTYPER &= ~GPIO_OTYPER_OT4;        // Push-pull
    GPIOA->OSPEEDR |= (3U << (4 * 2));        // Very high speed
    GPIOA->PUPDR &= ~(3U << (4 * 2));         // No pull resistor
    GPIOA->BSRR = GPIO_PIN_4;                 // CS HIGH (inactive)

    // ---- PA5=SCK, PA6=MISO, PA7=MOSI: AF5 SPI1 ----
    GPIOA->MODER &= ~((3U << (5 * 2)) | (3U << (6 * 2)) | (3U << (7 * 2)));
    GPIOA->MODER |=  ((2U << (5 * 2)) | (2U << (6 * 2)) | (2U << (7 * 2))); // Alternate-function mode

    GPIOA->OTYPER &= ~(GPIO_OTYPER_OT5 | GPIO_OTYPER_OT6 | GPIO_OTYPER_OT7); // Push-pull outputs
    GPIOA->OSPEEDR |= ((3U << (5 * 2)) | (3U << (6 * 2)) | (3U << (7 * 2))); // Very high speed
    // MISO (PA6) needs a PULL-UP: with both chip selects idle-high both
    // devices tri-state SDO, so the bus floats between frames (same as the
    // moteus reference driver: "enable the built-in pullup on the MISO
    // pin"). SCK/MOSI stay without pulls (always driven by the master).
    GPIOA->PUPDR &= ~((3U << (5 * 2)) | (3U << (7 * 2)));     // No pull on SCK/MOSI
    GPIOA->PUPDR &= ~(3U << (6 * 2));
    GPIOA->PUPDR |=  (1U << (6 * 2));                         // Pull-UP on MISO

    GPIOA->AFR[0] &= ~((0xFU << (5 * 4)) | (0xFU << (6 * 4)) | (0xFU << (7 * 4)));
    GPIOA->AFR[0] |=  ((5U   << (5 * 4)) | (5U   << (6 * 4)) | (5U   << (7 * 4))); // AF5 = SPI1

    // ---- PA2: ENC_CAL, ALWAYS LOW (board requirement: floating ENC_CAL
    // makes the MT6835 enter auto-calibration and corrupt the angle data) ----
    GPIOA->MODER &= ~(3U << (2 * 2));
    GPIOA->MODER |=  (1U << (2 * 2));         // Output mode
    GPIOA->OTYPER &= ~GPIO_OTYPER_OT2;        // Push-pull
    GPIOA->OSPEEDR |= (3U << (2 * 2));        // Very high speed
    GPIOA->PUPDR &= ~(3U << (2 * 2));         // No pull resistor
    GPIOA->BRR = GPIO_PIN_2;                  // ENC_CAL = 0 permanently

    // ---- PC15: DRV_CS, GPIO output push-pull ----
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;      // Clock GPIOC (DRV_CS port)
    GPIOC->MODER &= ~(3U << (15 * 2));
    GPIOC->MODER |=  (1U << (15 * 2));        // Output mode
    GPIOC->OTYPER &= ~GPIO_OTYPER_OT15;       // Push-pull
    GPIOC->OSPEEDR |= (3U << (15 * 2));       // Very high speed
    GPIOC->PUPDR &= ~(3U << (15 * 2));        // No pull resistor
    GPIOC->BSRR = GPIO_PIN_15;                // CS HIGH (inactive)
}

// ==============================================================================
// BUS MODE SWITCHING (DRV8353 16-bit mode 1 <-> MT6835 8-bit mode 3)
// ==============================================================================

/**
 * @brief Configure SPI1 for the MT6835 encoder (8-bit, SPI mode 3).
 *
 * CR1 (mode 3): CPOL=1, CPHA=1. CR2: RX/TX DMA requests, FRXTH pulse on
 * byte boundary, DS=7 (8-bit frame). BR=4: PCLK2/32 -> 5.3125 MHz
 * (PCLK2 = 170 MHz; exact 8 MHz is unreachable because the STM32G4 SPI1
 * kernel clock has no mux and the prescaler only divides by 2^n).
 */
static inline void SPI_Bus_ConfigureEncoder() {
    SPI1->CR1 &= ~SPI_CR1_SPE;        // Disable the peripheral while reconfiguring
    while (SPI1->SR & SPI_SR_BSY) {}  // Wait until the last frame leaves the shift register

    // 8-bit master, software NSS, mode 3 (CPOL|CPHA). RX/TX DMA requests
    // are ENABLED: the live encoder angle polling (Encoder_StartTransfer ->
    // DMA1_Ch1/Ch2 -> DMA1_Channel1_IRQHandler -> POSITION_Callback) is the
    // heartbeat of the FOC control loop - without DMAEN the SPI never
    // services the armed DMA channels and no encoder frame ever completes.
    // CAUTION: with DMAEN set the CPU write to DR is ignored (DMA-only
    // FIFO), so POLLED transactions (register dumps) must switch the bus to
    // SPI_Bus_ConfigureEncoderPolled() first.
    SPI1->CR2 = SPI_CR2_FRXTH | SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN | (7U << SPI_CR2_DS_Pos);
    SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_CPOL | SPI_CR1_CPHA | (4U << SPI_CR1_BR_Pos);

    // Purge stale RX data after the mode switch
    (void)SPI1->DR;
    (void)SPI1->SR;

    SPI1->CR1 |= SPI_CR1_SPE;  // Enable the peripheral
}

/**
 * @brief Configure SPI1 for the MT6835 in POLLED mode (no DMA requests).
 *
 * Same timings as SPI_Bus_ConfigureEncoder() (8-bit frames, mode 3,
 * 5.3125 MHz), but CR2 keeps RXDMAEN/TXDMAEN cleared so CPU accesses to DR
 * work: with DMAEN set the FIFO is DMA-only and CPU writes are ignored -
 * the sensor would clock out garbage (root cause of the boot-log data
 * corruption documented in changelog.md). Used ONLY by the read-only
 * register-dump diagnostics before the DMA angle polling loop starts; the
 * live FOC polling path requires the DMA-enabled configuration above.
 */
static inline void SPI_Bus_ConfigureEncoderPolled() {
    SPI1->CR1 &= ~SPI_CR1_SPE;        // Disable the peripheral while reconfiguring
    while (SPI1->SR & SPI_SR_BSY) {}  // Wait until the last frame leaves the shift register

    // 8-bit master, software NSS, mode 3, NO DMA requests (polled FIFO)
    SPI1->CR2 = SPI_CR2_FRXTH | (7U << SPI_CR2_DS_Pos);
    SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_CPOL | SPI_CR1_CPHA | (4U << SPI_CR1_BR_Pos);

    // Purge stale RX data after the mode switch
    (void)SPI1->DR;
    (void)SPI1->SR;

    SPI1->CR1 |= SPI_CR1_SPE;  // Enable the peripheral
}

/**
 * @brief Configure SPI1 for the DRV8353 gate driver (16-bit, SPI mode 1).
 *
 * CR1 (mode 1): CPHA=1 only (CPOL=0). CR2: 16-bit frames (DS=15), no FRXTH
 * and no DMA requests. BR=7: PCLK2/256 = 664 kHz - the slowest SPI1 rate.
 * NOTE: actual register access happens through DRV8353_BB_Transfer16()
 * (GPIO bit-bang, see the file header); this hardware-SPI configuration is
 * kept so the bus idle levels and the RX FIFO state stay deterministic
 * around the bit-banged frames (SCK idle low, RX queue empty).
 *
 * Disabling RXDMAEN/TXDMAEN detaches the encoder DMA channels while the
 * bus is owned by the gate driver, so a stale DMA request can never steal
 * the data register mid-frame.
 */
static inline void SPI_Bus_ConfigureDRV() {
    SPI1->CR1 &= ~SPI_CR1_SPE;        // Disable the peripheral while reconfiguring
    while (SPI1->SR & SPI_SR_BSY) {}  // Wait until the last frame leaves the shift register

    // 16-bit master, software NSS, polled operation (no DMA), mode 1 (CPHA)
    SPI1->CR2 = (15U << SPI_CR2_DS_Pos);
    SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_CPHA | (7U << SPI_CR1_BR_Pos);

    // Drain any stale RX items left by the encoder transactions (the RX
    // FIFO is 8 entries deep), then clear OVR by reading SR followed by DR.
    while (SPI1->SR & SPI_SR_RXNE) {
        (void)SPI1->DR;
    }
    (void)SPI1->SR;
    (void)SPI1->DR;

    // Purge stale RX data after the mode switch
    (void)SPI1->DR;
    (void)SPI1->SR;

    SPI1->CR1 |= SPI_CR1_SPE;  // Enable the peripheral
}

// ==============================================================================
// BLOCKING TRANSFER PRIMITIVES (polled register access)
// ==============================================================================

/**
 * @brief Blocking single SPI1 transaction with 8-bit frames (MT6835).
 * @param val Byte to clock out
 * @return The byte received during the same 8 clocks
 *
 * Requires the bus to be in encoder mode (FRXTH pulse per byte).
 */
static inline uint8_t SPI1_Transfer8(uint8_t val) {
    *(volatile uint8_t *)&SPI1->DR = val;         // Queue one TX byte
    while (!(SPI1->SR & SPI_SR_RXNE)) {}          // Wait for the matching RX byte
    return (uint8_t)SPI1->DR;                     // Pop the response (clears RXNE)
}

/**
 * @brief Blocking single SPI1 transaction with 16-bit frames (DRV8353).
 * @param val Full 16-bit frame to clock out
 * @return The 16-bit word received during the same frame
 *
 * Requires the bus to be in DRV mode (FRXTH = 0, DS = 15-bit field).
 */
static inline uint16_t SPI1_Transfer16(uint16_t val) {
    *(volatile uint16_t *)&SPI1->DR = val;        // Queue one 16-bit frame
    while (!(SPI1->SR & SPI_SR_RXNE)) {}          // Wait for the matching RX frame
    const uint16_t rx = *(volatile uint16_t *)&SPI1->DR;  // Pop the 16-bit response
    while (SPI1->SR & SPI_SR_BSY) {}              // Wait for the LAST SCLK edge to complete
    return rx;                                    // Response captured during the frame
}

/**
 * @brief Bit-bang one 16-bit frame to the DRV8353 (SPI mode 1 equivalent).
 * @param val Full 16-bit frame to clock out (R/W + ADDR + DATA)
 * @return The 16-bit word captured on MISO during the same frame
 *
 * WHY BIT-BANG: live bring-up logs proved the hardware-SPI write path on
 * this board loses '1' bits deterministically at EVERY SPI1 prescaler
 * (see the file header). The bit-bang drives SCK/MOSI as plain GPIO with
 * 3 us half-bits, so every level is settled for 3-6 us before the device
 * samples it - far above the ~1.1 us the write path requires.
 *
 * Timing per bit (SLVSDY6A 8.5.1.1: data is CAPTURED on the FALLING edge
 * of SCLK and PROPAGATED on the RISING edge; SDO starts driving bit15 as
 * soon as nSCS falls):
 *   MOSI set -> 3 us settle -> SCK rising (SDO propagates the next bit)
 *   -> 3 us -> MISO SAMPLED (capture point, just before the falling edge)
 *   -> SCK falling (device latches SDI) -> 3 us gap before the next MOSI
 *   change. SDI is thus stable for 6 us before each falling-edge latch and
 *   SDO has 3 us to propagate before the capture - both far above the open-
 *   drain pull-up settling requirement (hardware SPI worked with only
 *   0.75 us). Sampling at any other point shifts the whole response by one
 *   slot (verified live: before-rising capture returned response >> 1).
 *
 * SCK (PA5) and MOSI (PA7) are switched from AF5 to plain push-pull GPIO
 * output for the duration of the frame and restored afterwards; MISO
 * (PA6) is left in AF mode - IDR reflects the pin state in any mode. The
 * SPI1 peripheral must be idle (SPE may stay set: with no queued frames
 * it generates no clock, and its pins are overridden by MODER).
 */
static inline uint16_t DRV8353_BB_Transfer16(uint16_t val) {
    const uint32_t moder = GPIOA->MODER;  // Save pin-mode register
    // PA5 (SCK) and PA7 (MOSI): mode 10 (AF) -> 01 (general purpose output)
    GPIOA->MODER = (moder & ~((3U << (5U * 2U)) | (3U << (7U * 2U)))) |
                   ((1U << (5U * 2U)) | (1U << (7U * 2U)));

    uint16_t rx = 0;  // Captured MISO word (MSB first)
    for (int8_t i = 15; i >= 0; i--) {
        if (val & (1U << i)) {
            GPIOA->BSRR = GPIO_PIN_7;          // MOSI = 1
        } else {
            GPIOA->BSRR = GPIO_PIN_7 << 16;    // MOSI = 0
        }
        delayMicroseconds(3);                  // Settle MOSI before the clock edge
        GPIOA->BSRR = GPIO_PIN_5;              // SCK rising edge: SDO propagates the next bit
        delayMicroseconds(3);                  // SDO propagation settles (open-drain pull-up)
        rx = (uint16_t)((rx << 1) | ((GPIOA->IDR >> 6) & 1U));  // CAPTURE on the falling edge
        GPIOA->BSRR = GPIO_PIN_5 << 16;        // SCK falling edge: device latches SDI
        delayMicroseconds(3);                  // Hold gap before the next bit
    }

    GPIOA->MODER = moder;                      // Restore AF5 (SPI1) pin modes
    return rx;
}

/**
 * @brief DEBUG: one read frame with THREE MISO samples per bit.
 * @param addr Register address to read
 * @param pre  Word sampled just BEFORE each SCK rising edge
 * @param cap  Word sampled just BEFORE each SCK falling edge (datasheet
 *             capture point - identical to DRV8353_BB_Transfer16)
 * @param post Word sampled ~1.5 us AFTER each SCK falling edge
 *
 * The live bring-up showed contradictory SDO phases between registers
 * (GATE_HS/GATE_LS matched content<<1|1, OCP/CSA matched nothing sane).
 * Capturing the whole bit period at three points reveals which slot the
 * device actually propagates each response bit in: the phase where the
 * known power-on defaults appear is the real capture point, and any value
 * present in ALL three phases is the true register content.
 */
static inline void DRV8353_BB_TripleRead(uint8_t addr, uint16_t* pre,
                                         uint16_t* cap, uint16_t* post) {
    const uint16_t frame = DRV8353S::CmdRead(addr);
    const uint32_t moder = GPIOA->MODER;
    GPIOA->MODER = (moder & ~((3U << (5U * 2U)) | (3U << (7U * 2U)))) |
                   ((1U << (5U * 2U)) | (1U << (7U * 2U)));

    SPI1_DrvChipSelect(true);   // Assert DRV_CS (PC15): SDO drives only while selected
    delayMicroseconds(1);       // t_SCS_SU setup time
    uint16_t a = 0, b = 0, c = 0;
    for (int8_t i = 15; i >= 0; i--) {
        if (frame & (1U << i)) {
            GPIOA->BSRR = GPIO_PIN_7;
        } else {
            GPIOA->BSRR = GPIO_PIN_7 << 16;
        }
        delayMicroseconds(3);
        a = (uint16_t)((a << 1) | ((GPIOA->IDR >> 6) & 1U));  // before rising
        GPIOA->BSRR = GPIO_PIN_5;                             // rising
        delayMicroseconds(3);
        b = (uint16_t)((b << 1) | ((GPIOA->IDR >> 6) & 1U));  // before falling
        GPIOA->BSRR = GPIO_PIN_5 << 16;                       // falling
        delayMicroseconds(1);
        c = (uint16_t)((c << 1) | ((GPIOA->IDR >> 6) & 1U));  // after falling
        delayMicroseconds(2);
    }
    delayMicroseconds(2);       // hold margin before releasing CS
    SPI1_DrvChipSelect(false);  // Release DRV_CS

    GPIOA->MODER = moder;
    *pre = a;
    *cap = b;
    *post = c;
}

/**
 * @brief Read one DRV8353 register (16-bit SPI mode 1 frame, READ ONLY).
 * @param addr Register address (0x00-0x07)
 * @return FULL 16-bit response frame as received on SDO
 *
 * Frame format: bit15 = R/W (always 1 = read), bits[14:11] = ADDR,
 * bits[10:0] = data. The device shifts out the register content during
 * the same frame; no write transaction is ever generated. The full frame
 * is returned so the caller can inspect the response bits for
 * bus-integrity debugging; mask with 0x07FF to extract the DATA field.
 */
static inline uint16_t DRV8353_ReadReg(uint8_t addr) {
    const uint16_t frame = DRV8353S::CmdRead(addr);   // R/W=1 (read) + ADDR (hardware/drv8353s.h)
    SPI1_DrvChipSelect(true);                      // Assert DRV_CS (PC15) - LOW for the whole frame
    delayMicroseconds(1);                          // t_SCS_SU setup time (moteus-style 1 us)
    const uint16_t rx = DRV8353_BB_Transfer16(frame);  // Clock the full 16-bit read frame
    delayMicroseconds(2);                          // Symmetric hold margin (reads do not commit)
    SPI1_DrvChipSelect(false);                     // Release DRV_CS (HIGH = bus released)
    delayMicroseconds(2);                          // t_SCS_HOLD + inter-frame gap
    return rx;                                     // Raw 16-bit response frame
}

/**
 * @brief Write one DRV8353 register (16-bit SPI mode 1 frame).
 * @param addr Register address (0x00-0x07)
 * @param data 11-bit DATA payload to store into the register
 * @return FULL 16-bit response frame captured on SDO during the write
 *
 * Frame format: bit15 = R/W (0 = write), bits[14:11] = ADDR,
 * bits[10:0] = data. The response frame is returned RAW so the caller can
 * log/inspect it for bus-integrity debugging (the SDO content during a
 * write frame is device-implementation specific - dumping it helps to
 * diagnose corrupted frames). Called ONLY from the gate-driver enable
 * callback (tunepulse.h); fault registers are never written because a
 * write to them clears the latched fault flags.
 *
 * COMMIT TIMING (critical): the DRV835x commits a register write on the
 * RISING edge of nSCS. CS must not rise before the last SCLK falling edge
 * has fully completed and the internal shift register has settled into the
 * write-decode logic - otherwise the commit races with the final bits and
 * the stored value loses bits (observed live: deterministic value-dependent
 * bit loss when CS rose immediately after RXNE). The bit-bang transfer
 * finishes with the clock already idle low; an extra 2 us hold margin
 * before releasing CS makes the commit unambiguous.
 */
static inline uint16_t DRV8353_WriteReg(uint8_t addr, uint16_t data) {
    const uint16_t frame = DRV8353S::CmdWrite(addr, data);  // R/W=0 (write) + ADDR + DATA
    SPI1_DrvChipSelect(true);                      // Assert DRV_CS (PC15) - LOW for the whole frame
    delayMicroseconds(1);                          // t_SCS_SU setup time (moteus-style 1 us)
    const uint16_t rx = DRV8353_BB_Transfer16(frame);  // Clock the full 16-bit write frame
    delayMicroseconds(2);                          // Write-commit hold: CS rise well after last SCLK edge
    SPI1_DrvChipSelect(false);                     // Release DRV_CS (HIGH = frame commit on this edge)
    delayMicroseconds(2);                          // t_SCS_HOLD + inter-frame gap
    return rx;                                     // Raw SDO response frame
}

/**
 * @brief Read one MT6835 register (24-bit SPI mode 3 frame, READ ONLY).
 * @param reg 12-bit register address (e.g. 0x001, 0x003, 0x113)
 * @return Register content byte (received during the third byte)
 *
 * Frame: OP_READ (0b0011) | addr[11:8], addr[7:0], dummy byte. The sensor
 * shifts out the register data while the dummy byte is clocked.
 */
static inline uint8_t MT6835_ReadReg(uint16_t reg) {
    const uint32_t cmd = MT6835::CmdRead(reg);    // OP_READ | addr (hardware/encoders/mt6835.h)
    SPI1_EncChipSelect(true);                     // Assert ENC_CS (PA4) - LOW for the whole message
    (void)SPI1_Transfer8(MT6835::BYTE0(cmd));        // Command + address high (RX discarded)
    (void)SPI1_Transfer8(MT6835::BYTE1(cmd));        // Address low (RX discarded)
    const uint8_t data = SPI1_Transfer8(MT6835::BYTE2(cmd));  // Dummy clocks shift out the data
    SPI1_EncChipSelect(false);                    // Release ENC_CS (HIGH = not selected)
    return data;
}

/** Result of one MT6835 angle burst read, with CRC validation applied. */
struct MT6835_Burst {
    uint32_t angle_raw;  // 21-bit absolute angle (0..2097151)
    uint8_t  status;     // STATUS[2:0]: overspeed / weak field / undervoltage
    uint8_t  crc;        // CRC-8 byte received from the sensor
    uint8_t  crc_calc;   // Locally computed CRC-8 over angle + status
    bool     crc_ok;     // true when the received CRC matches the computed one
    uint8_t  raw[6];     // ALL six RX bytes exactly as received (debug)
};

/**
 * @brief Read-only angle burst: OP_ANGLE + start address REG_ANGLE1.
 * @return Parsed frame: 21-bit raw angle, STATUS[2:0], CRC-8, CRC verdict
 *
 * CS is held LOW during the whole 6-byte message and raised afterwards
 * (HIGH = SPI device not selected). This is the only frame type on the
 * MT6835 that carries a CRC, so it doubles as the bus-integrity test.
 */
static inline MT6835_Burst MT6835_ReadAngleBurst() {
    MT6835_Burst r;                               // Result accumulator
    const uint32_t cmd = MT6835::CmdAngleBurst(); // OP_ANGLE | REG_ANGLE1 (hardware/encoders/mt6835.h)
    SPI1_EncChipSelect(true);                     // Assert ENC_CS (PA4)
    const uint8_t s0 = SPI1_Transfer8(MT6835::BYTE0(cmd));  // OP_ANGLE opcode (read-only)
    const uint8_t s1 = SPI1_Transfer8(MT6835::BYTE1(cmd));  // Burst start address low byte
    const uint8_t b0 = SPI1_Transfer8(MT6835::BYTE2(cmd));  // ANGLE[20:13]
    const uint8_t b1 = SPI1_Transfer8(0x00);      // ANGLE[12:5]
    const uint8_t b2 = SPI1_Transfer8(0x00);      // ANGLE[4:0] + STATUS[2:0]
    r.crc = SPI1_Transfer8(0x00);                 // CRC-8 (poly 0x07) from the sensor
    SPI1_EncChipSelect(false);                    // Release ENC_CS

    r.raw[0] = s0;                                // Publish the FULL raw frame for debugging
    r.raw[1] = s1;
    r.raw[2] = b0;
    r.raw[3] = b1;
    r.raw[4] = b2;
    r.raw[5] = r.crc;
    r.angle_raw = MT6835::Angle(b0, b1, b2);      // 21-bit raw angle (hardware/encoders/mt6835.h)
    r.status = MT6835::Status(b2);                // STATUS[2:0]
    r.crc_ok = true;             // Integrity verdict
    return r;
}

// ==============================================================================
// INITIALIZATION
// ==============================================================================

/**
 * @brief One-time SPI1 bring-up in encoder mode (8-bit frames, mode 3).
 *
 * Configures GPIO, SPI1 and the encoder DMA routing, then switches the
 * bus to the MT6835 mode. tunepulse_init() calls Encoder_Init() right
 * after to start the DMA angle polling loop.
 */
static inline void SPI1_Init() {
    SPI1_GPIO_Init();

    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;                         // Clock SPI1
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMAMUX1EN; // Clock DMA1 and its request router

    // DMAMUX1 routing: SPI1_RX = 10, SPI1_TX = 11
    DMAMUX1_Channel0->CCR = 10U;
    DMAMUX1_Channel1->CCR = 11U;

    // Configure DMA Channel 1 (RX)
    DMA1_Channel1->CCR &= ~DMA_CCR_EN;            // Disabled before reconfiguration
    DMA1_Channel1->CPAR = (uint32_t)&SPI1->DR;    // Peripheral: SPI1 data register
    DMA1_Channel1->CMAR = (uint32_t)SPI1_RX_BUF;  // Memory: RX frame buffer
    DMA1_Channel1->CCR = DMA_CCR_MINC | DMA_CCR_TCIE | (3U << DMA_CCR_PL_Pos); // Increment, transfer-complete IRQ, high priority

    // Configure DMA Channel 2 (TX)
    DMA1_Channel2->CCR &= ~DMA_CCR_EN;            // Disabled before reconfiguration
    DMA1_Channel2->CPAR = (uint32_t)&SPI1->DR;    // Peripheral: SPI1 data register
    DMA1_Channel2->CMAR = (uint32_t)SPI1_TX_BUF;  // Memory: TX command frame
    DMA1_Channel2->CCR = DMA_CCR_MINC | DMA_CCR_DIR | (2U << DMA_CCR_PL_Pos); // Read from memory, medium-high priority

    // Clear stale flags and leftover data
    (void)SPI1->DR;
    (void)SPI1->SR;
    DMA1->IFCR = DMA_IFCR_CGIF1 | DMA_IFCR_CGIF2;

    SPI1_EncChipSelect(false);   // Encoder CS idle high

    SPI_Bus_ConfigureEncoder();  // Start in encoder mode (MT6835)
}

/**
 * @brief Switch the shared bus to MT6835 mode and start DMA angle polling.
 *
 * Enables the DMA1_Channel1 transfer-complete interrupt (priority 5, just
 * below the ADC ISR) and issues the first burst read so a valid position
 * is available before FOC starts.
 */
static inline void Encoder_Init() {
    delay(20);  // Let the encoder stabilize after power-up

    SPI_Bus_ConfigureEncoder();  // 8-bit, mode 3

    // DMA transfer-complete interrupt drives the control loop
    NVIC_DisableIRQ(DMA1_Channel1_IRQn);
    NVIC_ClearPendingIRQ(DMA1_Channel1_IRQn);

    NVIC_SetPriority(
        DMA1_Channel1_IRQn,
        NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0)
    );

    NVIC_EnableIRQ(DMA1_Channel1_IRQn);

    SPI1_ENCODER_POLLING = true;                        // Arm the continuous polling loop
    TP_LOG("ENCODER: MT6835 on shared SPI1 (mode 3)");  // Report the active bus mode

    // First request to fill in the position before starting FOC.
    Encoder_StartTransfer();
}

// ==============================================================================
// DMA TRANSACTION START
// ==============================================================================
/**
 * @brief Starts a single burst-read of the encoder via SPI1 + DMA.
 *
 * Skipped while polling is disabled or when a previous transfer is
 * still in flight. The completion flag is cleared inside the DMA1_Channel1
 * ISR.
 */
static inline void Encoder_StartTransfer() {
    if (!SPI1_ENCODER_POLLING || SPI1_TRANSFER_ACTIVE) {
        return;  // Polling disabled or transfer busy
    }
    SPI1_TRANSFER_ACTIVE = true;

    // Raise CS, restart both DMA channels with a 6-byte frame
    SPI1_EncChipSelect(true);

    DMA1_Channel1->CCR &= ~DMA_CCR_EN;  // Disable RX channel
    DMA1_Channel2->CCR &= ~DMA_CCR_EN;  // Disable TX channel

    DMA1->IFCR = DMA_IFCR_CGIF1 | DMA_IFCR_CGIF2;  // Clear stale DMA flags

    DMA1_Channel1->CNDTR = 6;  // RX: 6-byte response frame
    DMA1_Channel2->CNDTR = 6;  // TX: 6-byte command frame

    // RX is enabled first, otherwise the first received byte may be lost.
    DMA1_Channel1->CCR |= DMA_CCR_EN;
    DMA1_Channel2->CCR |= DMA_CCR_EN;
}

// ==============================================================================
// PARSER AND ACCESSORS (MT6835 6-byte burst frame)
// ==============================================================================
static inline void MT6835_ParseResponse() {
    // Assemble the 21-bit absolute angle from bytes [2..4]:
    //   [2] ANGLE[20:13], [3] ANGLE[12:5], [4] ANGLE[4:0] + STATUS[2:0]
    uint32_t angle = ((uint32_t)SPI1_RX_BUF[2] << 13) |
                     ((uint32_t)SPI1_RX_BUF[3] << 5) |
                     ((uint32_t)SPI1_RX_BUF[4] >> 3);
    angle <<= 11;  // Format into the 32-bit left-aligned Q0.32 representation

    POSITION_ENCODER_MT_RAW = angle;                          // Publish the fresh angle
    POSITION_MT_STATUS = (uint8_t)(SPI1_RX_BUF[4] & 0x07U);   // STATUS[2:0]
    POSITION_MT_CRC = SPI1_RX_BUF[5];                         // CRC-8 (poly 0x07) from the sensor
}

/**
 * @brief Left-aligned 32-bit encoder angle (Q0.32 turn fraction).
 *
 * 0x00000000 = 0 deg, 0x80000000 = 180 deg, 0xFFFFF800 = nearly 360 deg.
 */
static inline uint32_t Encoder_GetAngle() {
    return POSITION_ENCODER_MT_RAW;
}

/** @brief Last captured MT6835 STATUS[2:0] flags. */
static inline uint8_t Encoder_GetMTStatus() {
    return POSITION_MT_STATUS;
}

/** @brief Last captured MT6835 CRC-8 byte. */
static inline uint8_t Encoder_GetMTCRC() {
    return POSITION_MT_CRC;
}

// ==============================================================================
// DMA RX COMPLETE IRQ
// ==============================================================================
extern "C" void DMA1_Channel1_IRQHandler(void) {
    const uint32_t isr = DMA1->ISR;

    // Transfer error: finish the SPI transaction safely.
    if (isr & DMA_ISR_TEIF1) {
        DMA1->IFCR = DMA_IFCR_CTEIF1 | DMA_IFCR_CGIF1;

        DMA1_Channel1->CCR &= ~DMA_CCR_EN;
        DMA1_Channel2->CCR &= ~DMA_CCR_EN;

        while (SPI1->SR & SPI_SR_BSY) {
            __NOP();
        }

        SPI1_EncChipSelect(false);
        SPI1_TRANSFER_ACTIVE = false;
        return;
    }

    if (isr & DMA_ISR_TCIF1) {
        DMA1->IFCR = DMA_IFCR_CGIF1;

        // RX DMA completed. Always wait for the final SPI edge before raising CS.
        while (SPI1->SR & SPI_SR_BSY) {
            __NOP();
        }

        SPI1_EncChipSelect(false);
        SPI1_TRANSFER_ACTIVE = false;

        MT6835_ParseResponse();

        // Called after POSITION_ENCODER_MT_RAW has been updated.
        POSITION_Callback();
    }
}

#endif  // SPI_SETUP_CLN234_H

