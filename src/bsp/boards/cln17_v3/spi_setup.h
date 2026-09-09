/**
 * @file spi_setup.h
 * @brief SPI1 setup and DMA-driven polling for the magnetic encoder.
 *
 * The drive uses a single absolute magnetic encoder attached to the internal
 * SPI1 bus. The encoder model is selected at compile time via the
 * ENCODER_TYPE define (see "ENCODER SELECTION" below); there is no runtime
 * auto-detection.
 *
 * Encoder_Init() configures GPIO, SPI1 and DMA1; Encoder_StartTransfer()
 * launches a non-blocking burst read, and the DMA1_Channel1 ISR parses the
 * received packet and feeds the fresh angle into the FOC core through
 * POSITION_Callback(). The configured encoder type is reported over SerialUSB
 * via the TP_LOG macro (see dev_log.h).
 *
 * Hardware mapping:
 *   PA4 = CSN   (GPIO output, active LOW)
 *   PA5 = SCK   (AF5 SPI1)
 *   PA6 = MISO  (AF5 SPI1)
 *   PA7 = MOSI  (AF5 SPI1)
 *
 * Supported encoders:
 *   MT6835 (SPI mode 3: CPOL=1, CPHA=1), 6-byte burst read:
 *     TX: 0xA0 0x03 0x00 0x00 0x00 0x00 (read command + 4 dummy bytes)
 *     RX: [2] ANGLE[20:13], [3] ANGLE[12:5],
 *         [4] ANGLE[4:0] + STATUS[2:0], [5] CRC-8 (poly 0x07)
 *   TLE5012B (SPI mode 1: CPOL=0, CPHA=1), 4-byte angle read:
 *     TX: 0x80 0x20 0xFF 0xFF (read command + dummy data)
 *     RX: [2] ANGLE[14:7], [3] ANGLE[6:0]
 *
 * The angle is stored left-aligned in a 32-bit word (bit 31 = half turn) so
 * it can be used directly as a Q0.32 fixed-point turn fraction by the FOC
 * core.
 *
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#ifndef SMART_ENCODER_SETUP_H
#define SMART_ENCODER_SETUP_H

#include <Arduino.h>     // delay(), millis()
#include "stm32g4xx.h"   // CMSIS device header: SPI1, DMA1, GPIO registers
#include "lib/dev_log.h" // TP_LOG* macros: USB CDC diagnostics over SerialUSB

// ==============================================================================
// ENCODER SELECTION (set manually, no runtime auto-detection)
// ==============================================================================
// Uncomment exactly ONE of the two type options below.
#define ENCODER_MT6835   1 // 21-bit absolute encoder, SPI mode 3, 6-byte frames
#define ENCODER_TLE5012B 2 // 15-bit absolute encoder, SPI mode 1, 4-byte frames

#define ENCODER_TYPE ENCODER_MT6835 // <<< CHANGE HERE to switch the encoder

// Compile-time sanity check: an unsupported selection must fail the build.
#if (ENCODER_TYPE != ENCODER_MT6835) && (ENCODER_TYPE != ENCODER_TLE5012B)
#error "spi_setup.h: ENCODER_TYPE must be either ENCODER_MT6835 or ENCODER_TLE5012B"
#endif

extern void POSITION_Callback(); // Motor-control hook, called after each new angle

static inline void Encoder_StartTransfer(); // Forward declaration: Encoder_Init() issues the first transfer

// ==============================================================================
// VARIABLES AND BUFFERS (SPI1)
// ==============================================================================
static uint8_t SPI1_RX_BUF[6] __attribute__((aligned(4))) = {0}; // RX frame filled by DMA

#if (ENCODER_TYPE == ENCODER_MT6835)
static uint8_t SPI1_TX_BUF[6] __attribute__((aligned(4))) = // MT6835 TX frame: burst-read command + dummy payload
    {0xA0, 0x03, 0x00, 0x00, 0x00, 0x00};
static volatile uint32_t POSITION_ENCODER_MT_RAW = 0; // 32-bit left-aligned angle (Q0.32 turn fraction)
static volatile uint8_t  POSITION_MT_STATUS = 0;      // STATUS[2:0]: overspeed / weak field / undervoltage
static volatile uint8_t  POSITION_MT_CRC = 0;         // CRC-8 received from the sensor
#else
static uint8_t SPI1_TX_BUF[4] __attribute__((aligned(4))) = // TLE5012B TX frame: angle-read command
    {0x80, 0x20, 0xFF, 0xFF};
static volatile uint16_t POSITION_ENCODER_TLE = 0; // 15-bit left-aligned angle
#endif

// ==============================================================================
// LOW-LEVEL CHIP SELECT CONTROL
// ==============================================================================
static inline void SPI1_ChipSelect(bool select) {
    if (select) GPIOA->BRR = GPIO_PIN_4;       // CS LOW (Active) PA4
    else        GPIOA->BSRR = GPIO_PIN_4;      // CS HIGH (Idle) PA4
}

// ==============================================================================
// SPI1 GPIO INITIALIZATION
// ==============================================================================
static inline void SPI1_GPIO_Init() {
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;      // Clock GPIOA (CS + SPI lines)

    // ---- PA4: CSN, GPIO output push-pull ----
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
}

// ==============================================================================
// SPI1 INITIALIZATION
// ==============================================================================
static inline void SPI1_Init() {
    SPI1_GPIO_Init();

    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;                       // Clock SPI1
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

    // SPI master, software NSS, 8-bit frame with RXNE pulse on byte boundary
    // (FRXTH), DMA requests enabled. BR=5: PCLK2 / 64 -> ~2.66 MHz at 170 MHz.
    SPI1->CR2 = SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN | SPI_CR2_FRXTH | (7U << SPI_CR2_DS_Pos);

#if (ENCODER_TYPE == ENCODER_MT6835)
    // SPI mode 3 (CPOL=1, CPHA=1) as required by the MT6835
    SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_CPOL | SPI_CR1_CPHA | (5U << SPI_CR1_BR_Pos);
#else
    // SPI mode 1 (CPOL=0, CPHA=1) as required by the TLE5012B
    SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_CPHA | (5U << SPI_CR1_BR_Pos);
#endif

    // Clear stale flags and leftover data
    (void)SPI1->DR;
    (void)SPI1->SR;
    DMA1->IFCR = DMA_IFCR_CGIF1 | DMA_IFCR_CGIF2;

    SPI1_ChipSelect(false);   // Deassert CS before enabling SPI
    SPI1->CR1 |= SPI_CR1_SPE; // Enable the peripheral
}

// ==============================================================================
// INITIALIZATION (COMPILE-TIME SELECTED ENCODER)
// ==============================================================================
static inline void Encoder_Init() {
    delay(20); // Let the encoder stabilize after power-up

    SPI1_Init();

    // DMA transfer-complete interrupt drives the control loop
    NVIC_DisableIRQ(DMA1_Channel1_IRQn);
    NVIC_ClearPendingIRQ(DMA1_Channel1_IRQn);

    NVIC_SetPriority(
        DMA1_Channel1_IRQn,
        NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0)
    );

    NVIC_EnableIRQ(DMA1_Channel1_IRQn);

#if (ENCODER_TYPE == ENCODER_MT6835)
    TP_LOG("ENCODER: MT6835 on SPI1 (compile-time selected)"); // Report the configured encoder
#else
    TP_LOG("ENCODER: TLE5012B on SPI1 (compile-time selected)"); // Report the configured encoder
#endif

    // Optional: first request to fill in the position before starting FOC.
    Encoder_StartTransfer();
}

// ==============================================================================
// DMA TRANSACTION START
// ==============================================================================
/**
 * @brief Starts a single burst-read of the encoder via SPI1 + DMA.
 *
 * Frame length depends on the compile-time selected encoder:
 * 6 bytes (mode 3) for the MT6835, 4 bytes (mode 1) for the TLE5012B.
 *
 * Important: start the next transfer only after the previous one completes.
 * Normally called from the control timer; the DMA IRQ invokes POSITION_Callback().
 */
static inline void Encoder_StartTransfer() {
    SPI1_ChipSelect(true);                 // Assert CS to frame the transaction
    DMA1_Channel1->CCR &= ~DMA_CCR_EN;     // Stop RX channel
    DMA1_Channel2->CCR &= ~DMA_CCR_EN;     // Stop TX channel

    DMA1->IFCR = DMA_IFCR_CGIF1 | DMA_IFCR_CGIF2; // Clear stale completion flags

#if (ENCODER_TYPE == ENCODER_MT6835)
    DMA1_Channel1->CNDTR = 6;              // 6 bytes to receive
    DMA1_Channel2->CNDTR = 6;              // 6 bytes to transmit
#else
    DMA1_Channel1->CNDTR = 4;              // 4 bytes to receive
    DMA1_Channel2->CNDTR = 4;              // 4 bytes to transmit
#endif

    // RX is enabled first, otherwise the first received byte may be lost.
    DMA1_Channel1->CCR |= DMA_CCR_EN;
    DMA1_Channel2->CCR |= DMA_CCR_EN;
}

// ==============================================================================
// ANGLE GETTER (32-bit Left Aligned)
// ==============================================================================
// Returns the angle in left-aligned 32-bit format:
// 0x00000000 = 0 deg
// 0x80000000 = 180 deg
// 0xFFFFF800 = nearly 360 deg
static inline uint32_t Encoder_GetAngle() {
#if (ENCODER_TYPE == ENCODER_MT6835)
    return POSITION_ENCODER_MT_RAW;
#else
    return ((uint32_t)POSITION_ENCODER_TLE) << 16;
#endif
}

#if (ENCODER_TYPE == ENCODER_MT6835)
static inline uint8_t Encoder_GetMTStatus() {
    return POSITION_MT_STATUS;
}

static inline uint8_t Encoder_GetMTCRC() {
    return POSITION_MT_CRC;
}
#endif

// ==============================================================================
// DMA RX COMPLETE IRQ
// ==============================================================================
// DMA1_Channel1_IRQHandler: interrupt vector, defined in the board translation unit
// (bsp/boards/cln17_v3/board.cpp) so that exactly one object file owns it.

#endif // SMART_ENCODER_SETUP_H

