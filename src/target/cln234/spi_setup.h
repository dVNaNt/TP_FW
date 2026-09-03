/**
 * @file spi_setup.h
 * @brief SPI1 bus manager for the MT6835 absolute encoder (CLN234).
 *
 * The CLN234 routes the MT6835 absolute encoder onto the SPI1 bus
 * (PA5=SCK / PA6=MISO / PA7=MOSI) with the PA4 chip select.
 *
 * Bus mode: 8-bit frames, SPI mode 3 (CPOL=1, CPHA=1), DMA polling.
 *
 * The DRV8353 gate driver is NOT accessed over SPI at all (its registers
 * are never written nor read); it runs on its hardware defaults with the
 * DRV_EN (PB12) enable line and the nFAULT input (PB1) only.
 *
 * Boot-time order (enforced by tunepulse_init()):
 * 1. SPI1_Init()        - GPIO/DMA/SPI bring-up in encoder mode.
 * 2. Encoder_Init()     - DMA polling started.
 *
 * Encoder frame (SPI mode 3, 6 bytes):
 *   TX: 0xA0 0x03 0x00 0x00 0x00 0x00 (burst-read command + dummy bytes)
 *   RX: [2] ANGLE[20:13], [3] ANGLE[12:5],
 *       [4] ANGLE[4:0] + STATUS[2:0], [5] CRC-8 (poly 0x07)
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
}

// ==============================================================================
// BUS CONFIGURATION (MT6835 8-bit mode 3)
// ==============================================================================

/**
 * @brief Configure SPI1 for the MT6835 encoder (8-bit, SPI mode 3).
 *
 * CR1 (mode 3): CPOL=1, CPHA=1. CR2: RX/TX DMA requests, FRXTH pulse on
 * byte boundary, DS=7 (8-bit frame). BR=5: PCLK2/64 -> ~2.66 MHz.
 */
static inline void SPI_Bus_ConfigureEncoder() {
    SPI1->CR1 &= ~SPI_CR1_SPE;        // Disable the peripheral while reconfiguring
    while (SPI1->SR & SPI_SR_BSY) {}  // Wait until the last frame leaves the shift register

    // 8-bit master, software NSS, DMA requests enabled, mode 3 (CPOL|CPHA)
    SPI1->CR2 = SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN | SPI_CR2_FRXTH | (7U << SPI_CR2_DS_Pos);
    SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_CPOL | SPI_CR1_CPHA | (5U << SPI_CR1_BR_Pos);

    // Purge stale RX data after the mode switch
    (void)SPI1->DR;
    (void)SPI1->SR;

    SPI1->CR1 |= SPI_CR1_SPE;  // Enable the peripheral
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

