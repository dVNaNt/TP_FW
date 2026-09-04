/**
 * @file spi_setup.h
 * @brief Shared SPI1 bus manager for the DRV8353 gate driver and the MT6835
 *        absolute encoder (CLN234) - READ-ONLY register access.
 *
 * The CLN234 routes BOTH SPI devices onto the SPI1 bus (PA5=SCK / PA6=MISO /
 * PA7=MOSI) with separate chip selects: PA4 = ENC_CS (MT6835), PC15 = DRV_CS
 * (DRV8353). The two chips need different bus modes, so the bus is
 * reconfigured on the fly around every device switch:
 *
 * - DRV8353 : 16-bit frames, SPI mode 1 (CPOL=0, CPHA=1), polled transfers.
 * - MT6835  : 8-bit frames,  SPI mode 3 (CPOL=1, CPHA=1), DMA polling for
 *             the continuous angle burst + polled single-register reads.
 *
 * Bus speed: PCLK2 = 170 MHz is the only SPI1 kernel clock on STM32G4 (no
 * RCC mux for SPI1), and the prescaler only divides by 2^n, so exact 8 MHz
 * is not reachable. Both devices are configured with BR = PCLK2/32 =
 * 5.3125 MHz, which is within the SPI specification of both chips
 * (DRV8353 max 10 MHz, MT6835 max 16 MHz).
 *
 * IMPORTANT - this build performs READS ONLY. No register is ever written:
 * - DRV8353 frames always set R/W = 1 (bit 15), so writing is impossible.
 * - MT6835 frames always use the OP_READ opcode (0b0011) or the read-only
 *   OP_ANGLE burst command (0b1010).
 * A read transaction has no side effects on either device (the DRV8353
 * fault registers are cleared only by writes; none are performed here).
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
    // Important: disable pull resistors on SPI lines in normal operation
    GPIOA->PUPDR &= ~((3U << (5 * 2)) | (3U << (6 * 2)) | (3U << (7 * 2)));

    GPIOA->AFR[0] &= ~((0xFU << (5 * 4)) | (0xFU << (6 * 4)) | (0xFU << (7 * 4)));
    GPIOA->AFR[0] |=  ((5U   << (5 * 4)) | (5U   << (6 * 4)) | (5U   << (7 * 4))); // AF5 = SPI1

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

    // 8-bit master, software NSS, DMA requests enabled, mode 3 (CPOL|CPHA)
    SPI1->CR2 = SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN | SPI_CR2_FRXTH | (7U << SPI_CR2_DS_Pos);
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
 * and no DMA requests - register reads are fully polled. BR=4: PCLK2/32 ->
 * 5.3125 MHz, within the DRV8353 SPI specification (max 10 MHz).
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
    SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_CPHA | (4U << SPI_CR1_BR_Pos);

    // Purge stale RX data after the mode switch
    (void)SPI1->DR;
    (void)SPI1->SR;

    SPI1->CR1 |= SPI_CR1_SPE;  // Enable the peripheral
}

// ==============================================================================
// BLOCKING READ PRIMITIVES (read-only register access)
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
    return *(volatile uint16_t *)&SPI1->DR;       // Pop the 16-bit response
}

/**
 * @brief Read one DRV8353 register (16-bit SPI mode 1 frame, READ ONLY).
 * @param addr Register address (0x00-0x07)
 * @return 11-bit DATA field of the response
 *
 * Frame format: bit15 = R/W (always 1 = read), bits[14:11] = ADDR,
 * bits[10:0] = data. The device shifts out the register content during
 * the same frame; no write transaction is ever generated.
 */
static inline uint16_t DRV8353_ReadReg(uint8_t addr) {
    const uint16_t frame = 0x8000U | ((uint16_t)addr << 11);  // R/W=1 (read) + ADDR + zero data
    SPI1_DrvChipSelect(true);                     // Assert DRV_CS (PC15)
    const uint16_t rx = SPI1_Transfer16(frame);   // Clock the full 16-bit read frame
    SPI1_DrvChipSelect(false);                    // Release DRV_CS
    return rx & 0x07FFU;                          // Strip the command echo, keep DATA[10:0]
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
    const uint8_t tx0 = (uint8_t)(0x30U | ((reg >> 8) & 0x0FU));  // OP_READ + addr[11:8]
    const uint8_t tx1 = (uint8_t)(reg & 0xFFU);                   // addr[7:0]
    SPI1_EncChipSelect(true);                     // Assert ENC_CS (PA4)
    (void)SPI1_Transfer8(tx0);                    // Command + address high (RX discarded)
    (void)SPI1_Transfer8(tx1);                    // Address low (RX discarded)
    const uint8_t data = SPI1_Transfer8(0x00);    // Dummy clocks shift out the data
    SPI1_EncChipSelect(false);                    // Release ENC_CS
    return data;
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

