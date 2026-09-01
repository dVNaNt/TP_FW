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

#define ENCODER_TYPE ENCODER_TLE5012B // <<< CHANGE HERE to switch the encoder

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
extern "C" void DMA1_Channel1_IRQHandler(void) {
    const uint32_t isr = DMA1->ISR;

    // Transfer error: finish the SPI transaction safely.
    if (isr & DMA_ISR_TEIF1) {
        DMA1->IFCR = DMA_IFCR_CTEIF1 | DMA_IFCR_CGIF1; // Clear error and global flags

        DMA1_Channel1->CCR &= ~DMA_CCR_EN;             // Stop RX channel
        DMA1_Channel2->CCR &= ~DMA_CCR_EN;             // Stop TX channel

        while (SPI1->SR & SPI_SR_BSY) {                // Wait for the physical SPI completion
            __NOP();
        }

        SPI1_ChipSelect(false);                        // Release the bus
        return;
    }

    if (isr & DMA_ISR_TCIF1) {
        DMA1->IFCR = DMA_IFCR_CGIF1;                   // Acknowledge RX transfer-complete

        // OPTIMIZATION (P1.3): at ~2.66 MHz SPI the last bytes are still
        // shifting after DMA-complete. Sleep in WFE (woken by the
        // higher-priority ADC/TIM1 interrupts that fire every cycle)
        // instead of busy-spinning NOPs inside this priority-5 ISR.
        while (SPI1->SR & SPI_SR_BSY) {
            __WFE();
        }

        SPI1_ChipSelect(false); // Deassert CS to latch the packet

#if (ENCODER_TYPE == ENCODER_MT6835)
        // Assemble 21-bit angle {RX[2], RX[3], RX[4][7:3]} and capture status / CRC
        POSITION_ENCODER_MT_RAW =
            ((uint32_t)SPI1_RX_BUF[2] << 13) |
            ((uint32_t)SPI1_RX_BUF[3] << 5)  |
            ((uint32_t)SPI1_RX_BUF[4] >> 3);
        POSITION_ENCODER_MT_RAW <<= 11; // Left-align into 32-bit Q0.32 representation

        POSITION_MT_STATUS = (uint8_t)(SPI1_RX_BUF[4] & 0x07U); // Status bits: overspeed / weak field / undervoltage
        POSITION_MT_CRC    = SPI1_RX_BUF[5];                    // Received CRC-8 for diagnostics
#else
        // TLE5012B: 15-bit left-aligned angle from RX[2..3]
        POSITION_ENCODER_TLE = ((uint16_t)SPI1_RX_BUF[2] << 9) | ((uint16_t)SPI1_RX_BUF[3] << 1);
#endif

        // Called after the position variable has been updated.
        POSITION_Callback();
    }
}

#endif // SMART_ENCODER_SETUP_H



// /**
//  * @file spi_setup.h
//  * @brief SPI1 setup and DMA-driven polling for the magnetic encoder.
//  *
//  * Encoder_SmartInit() configures SPI1; the polling routine continuously reads
//  * absolute angle words from the sensor and feeds Encoder::set_raw_position().
//  * Copyright 2026, creapunk
//  * Licensed under the Apache License, Version 2.0
//  */

// #ifndef SMART_ENCODER_SETUP_H
// #define SMART_ENCODER_SETUP_H

// #include <Arduino.h>
// #include "stm32g4xx.h"

// extern void POSITION_Callback();

// // ==============================================================================
// // VARIABLES AND BUFFERS (SPI1 - INTERNAL TLE5012)
// // ==============================================================================
// static uint8_t SPI1_TX_BUF[4] __attribute__((aligned(4))) = {0x80, 0x20, 0xFF, 0xFF};
// static uint8_t SPI1_RX_BUF[4] __attribute__((aligned(4))) = {0};
// static volatile uint16_t POSITION_ENCODER_TLE = 0;

// // ==============================================================================
// // VARIABLES AND BUFFERS (SPI3 - EXTERNAL MT6835)
// // ==============================================================================
// static uint8_t SPI3_TX_BUF[6] __attribute__((aligned(4))) = {0xA0, 0x03, 0x00, 0x00, 0x00, 0x00};
// static uint8_t SPI3_RX_BUF[6] __attribute__((aligned(4))) = {0};

// static volatile uint32_t POSITION_ENCODER_MT_RAW = 0;
// static volatile uint8_t  POSITION_MT_STATUS = 0;
// static volatile uint8_t  POSITION_MT_CRC = 0;

// // Global active-encoder flag (true = MT6835, false = TLE5012)
// static volatile bool ENCODER_USE_EXTERNAL = false;

// // ==============================================================================
// // LOW-LEVEL CHIP SELECT CONTROL
// // ==============================================================================
// static inline void SPI1_ChipSelect(bool select) {
//     if (select) GPIOA->BRR = GPIO_PIN_4;       // CS LOW (Active)
//     else        GPIOA->BSRR = GPIO_PIN_4;      // CS HIGH (Idle)
// }

// static inline void SPI3_ChipSelect(bool select) {
//     if (select) GPIOA->BRR = GPIO_PIN_15;      // CS LOW (Active) PA15
//     else        GPIOA->BSRR = GPIO_PIN_15;     // CS HIGH (Idle) PA15
// }

// // ==============================================================================
// // SPI1 INITIALIZATION (TLE5012 - FALLBACK)
// // ==============================================================================
// static inline void SPI1_Init() {
//     RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
//     RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMAMUX1EN;

//     // DMAMUX1 routing: SPI1_RX = 10, SPI1_TX = 11
//     DMAMUX1_Channel0->CCR = 10U;
//     DMAMUX1_Channel1->CCR = 11U;

//     // Configure DMA Channel 1 (RX)
//     DMA1_Channel1->CCR &= ~DMA_CCR_EN;
//     DMA1_Channel1->CPAR = (uint32_t)&SPI1->DR;
//     DMA1_Channel1->CMAR = (uint32_t)SPI1_RX_BUF;
//     DMA1_Channel1->CCR = DMA_CCR_MINC | DMA_CCR_TCIE | (3U << DMA_CCR_PL_Pos); 

//     // Configure DMA Channel 2 (TX)
//     DMA1_Channel2->CCR &= ~DMA_CCR_EN;
//     DMA1_Channel2->CPAR = (uint32_t)&SPI1->DR;
//     DMA1_Channel2->CMAR = (uint32_t)SPI1_TX_BUF;
//     DMA1_Channel2->CCR = DMA_CCR_MINC | DMA_CCR_DIR | (2U << DMA_CCR_PL_Pos); 

//     // Configure SPI1
//     SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_CPHA | (5U << SPI_CR1_BR_Pos); 
//     SPI1->CR2 = SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN | SPI_CR2_FRXTH | (7U << SPI_CR2_DS_Pos); 
    
//     // Clear stale flags and leftover data
//     (void)SPI1->DR;
//     (void)SPI1->SR;
//     DMA1->IFCR = DMA_IFCR_CGIF1 | DMA_IFCR_CGIF2;
    
//     SPI1_ChipSelect(false); // Make sure CS is deasserted
//     SPI1->CR1 |= SPI_CR1_SPE;
// }

// // ==============================================================================
// // SPI3 INITIALIZATION (MT6835 - PRIMARY)
// // ==============================================================================
// static inline void SPI3_GPIO_Init() {
//     RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN;

//     // ---- PA15 as GPIO output for CS ----
//     GPIOA->MODER &= ~(3U << (15 * 2));
//     GPIOA->MODER |=  (1U << (15 * 2));       // Output mode
//     GPIOA->OTYPER &= ~GPIO_OTYPER_OT15;      // Push-pull
//     GPIOA->OSPEEDR |= (3U << (15 * 2));      // Very high speed
//     GPIOA->PUPDR &= ~(3U << (15 * 2));       // No pull resistor
//     GPIOA->BSRR = GPIO_PIN_15;               // CS HIGH (inactive)

//     // ---- PB3/PB4/PB5 -> AF6 SPI3 ----
//     GPIOB->MODER &= ~((3U << (3 * 2)) | (3U << (4 * 2)) | (3U << (5 * 2)));
//     GPIOB->MODER |=  ((2U << (3 * 2)) | (2U << (4 * 2)) | (2U << (5 * 2)));

//     GPIOB->OTYPER &= ~(GPIO_OTYPER_OT3 | GPIO_OTYPER_OT4 | GPIO_OTYPER_OT5);
//     GPIOB->OSPEEDR |= ((3U << (3 * 2)) | (3U << (4 * 2)) | (3U << (5 * 2)));
//     // Important: disable pull resistors on SPI lines in normal operation
//     GPIOB->PUPDR &= ~((3U << (3 * 2)) | (3U << (4 * 2)) | (3U << (5 * 2)));

//     GPIOB->AFR[0] &= ~((0xFU << (3 * 4)) | (0xFU << (4 * 4)) | (0xFU << (5 * 4)));
//     GPIOB->AFR[0] |=  ((6U   << (3 * 4)) | (6U   << (4 * 4)) | (6U   << (5 * 4)));
// }

// static inline void SPI3_Init() {
//     SPI3_GPIO_Init();

//     RCC->APB1ENR1 |= RCC_APB1ENR1_SPI3EN;
//     RCC->AHB1ENR  |= RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMAMUX1EN;

//     // DMAMUX1 routing: SPI3_RX = 14, SPI3_TX = 15
//     DMAMUX1_Channel0->CCR = 14U; 
//     DMAMUX1_Channel1->CCR = 15U; 

//     DMA1_Channel1->CCR &= ~DMA_CCR_EN;
//     DMA1_Channel1->CPAR = (uint32_t)&SPI3->DR;
//     DMA1_Channel1->CMAR = (uint32_t)SPI3_RX_BUF;
//     DMA1_Channel1->CCR = DMA_CCR_MINC | DMA_CCR_TCIE | (3U << DMA_CCR_PL_Pos);

//     DMA1_Channel2->CCR &= ~DMA_CCR_EN;
//     DMA1_Channel2->CPAR = (uint32_t)&SPI3->DR;
//     DMA1_Channel2->CMAR = (uint32_t)SPI3_TX_BUF;
//     DMA1_Channel2->CCR = DMA_CCR_MINC | DMA_CCR_DIR | (2U << DMA_CCR_PL_Pos);

//     SPI3->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_CPOL | SPI_CR1_CPHA | (4U << SPI_CR1_BR_Pos);
//     SPI3->CR2 = SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN | SPI_CR2_FRXTH | (7U << SPI_CR2_DS_Pos); 

//     // Clear stale flags and leftover data
//     (void)SPI3->DR; 
//     (void)SPI3->SR;
//     DMA1->IFCR = DMA_IFCR_CGIF1 | DMA_IFCR_CGIF2;

//     SPI3_ChipSelect(false); // Deassert CS before enabling SPI
//     SPI3->CR1 |= SPI_CR1_SPE;
// }

// // ==============================================================================
// // DMA TRANSACTION START (UNIVERSAL)
// // ==============================================================================
// static inline void Encoder_StartTransfer() {
//     if (ENCODER_USE_EXTERNAL) {
//         SPI3_ChipSelect(true);
//         DMA1_Channel1->CCR &= ~DMA_CCR_EN;
//         DMA1_Channel2->CCR &= ~DMA_CCR_EN;
        
//         DMA1->IFCR = DMA_IFCR_CGIF1 | DMA_IFCR_CGIF2;
        
//         DMA1_Channel1->CNDTR = 6;
//         DMA1_Channel2->CNDTR = 6;
        
//         DMA1_Channel1->CCR |= DMA_CCR_EN;
//         DMA1_Channel2->CCR |= DMA_CCR_EN;
//     } else {
//         SPI1_ChipSelect(true);
//         DMA1_Channel1->CCR &= ~DMA_CCR_EN;
//         DMA1_Channel2->CCR &= ~DMA_CCR_EN;
        
//         DMA1->IFCR = DMA_IFCR_CGIF1 | DMA_IFCR_CGIF2;
        
//         DMA1_Channel1->CNDTR = 4;
//         DMA1_Channel2->CNDTR = 4;
        
//         DMA1_Channel1->CCR |= DMA_CCR_EN;
//         DMA1_Channel2->CCR |= DMA_CCR_EN;
//     }
// }

// // ==============================================================================
// // SMART INITIALIZATION AND AUTO-DETECTION
// // ==============================================================================
// static inline void Encoder_SmartInit() {
//     // 1. Wait 100 ms before probing the external encoder (slow MT6835 startup)
//     delay(100); 

//     // 2. Initialize SPI3 with interrupts disabled
//     SPI3_Init();
//     NVIC_DisableIRQ(DMA1_Channel1_IRQn);
//     NVIC_ClearPendingIRQ(DMA1_Channel1_IRQn);

//     // 3. Send a test request
//     ENCODER_USE_EXTERNAL = true; 
//     Encoder_StartTransfer();

//     // 4. Wait for DMA completion (~5 ms timeout)
//     uint32_t start_time = millis();
//     while (!(DMA1->ISR & DMA_ISR_TCIF1)) {
//         if ((millis() - start_time) > 5) break;
//     }

//     bool mt_valid = false;
    
//     // 5. Validate the response
//     if (DMA1->ISR & DMA_ISR_TCIF1) {
//         DMA1->IFCR = DMA_IFCR_CGIF1;
//         // OPTIMIZATION (P1.3): sleep in WFE instead of a NOP spin; SysTick
//         // (1 ms) and any pending event wake the core until SPI finishes.
//         while (SPI3->SR & SPI_SR_BSY) { __WFE(); }
//         SPI3_ChipSelect(false);

//         // If MISO floats, data will be either 0x00 or 0xFF
//         uint32_t rx_sum = SPI3_RX_BUF[2] + SPI3_RX_BUF[3] + SPI3_RX_BUF[4] + SPI3_RX_BUF[5];
//         if (rx_sum != 0 && rx_sum != (0xFF * 4)) {
//             mt_valid = true; 
//         }
//     }

//     // Clear potentially stuck DMA/SPI flags after the test transfer
//     DMA1_Channel1->CCR &= ~DMA_CCR_EN;
//     DMA1_Channel2->CCR &= ~DMA_CCR_EN;
//     DMA1->IFCR = DMA_IFCR_CGIF1 | DMA_IFCR_CGIF2;

//     // 6. Route according to the probe result
//     if (mt_valid) {
//         // Encoder found: parse the first data packet
//         POSITION_ENCODER_MT_RAW = ((uint32_t)SPI3_RX_BUF[2] << 13) | ((uint32_t)SPI3_RX_BUF[3] << 5) | ((uint32_t)SPI3_RX_BUF[4] >> 3);
//         POSITION_ENCODER_MT_RAW <<= 11; // Format into 32-bit left-aligned representation
//         POSITION_MT_STATUS = (uint8_t)(SPI3_RX_BUF[4] & 0x07U);
//         POSITION_MT_CRC    = SPI3_RX_BUF[5];
        
//         // Enable interrupts for normal operation
//         NVIC_SetPriority(DMA1_Channel1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0));
//         NVIC_EnableIRQ(DMA1_Channel1_IRQn);
//     } else {
//         // No external encoder: shut down SPI3
//         ENCODER_USE_EXTERNAL = false;
        
//         SPI3->CR1 &= ~SPI_CR1_SPE;
//         RCC->APB1ENR1 &= ~RCC_APB1ENR1_SPI3EN;

//         // --- SWITCH PINS TO Hi-Z (Floating Input) ---
//         // Reset MODER of PB3/PB4/PB5 to 00 (input mode, disconnect SPI alternate function)
//         GPIOB->MODER &= ~((3U << (3 * 2)) | (3U << (4 * 2)) | (3U << (5 * 2))); 
//         GPIOB->PUPDR &= ~((3U << (3 * 2)) | (3U << (4 * 2)) | (3U << (5 * 2))); 

//         // Switch PA15 (Chip Select) to open-drain and drive HIGH (Hi-Z)
//         GPIOA->OTYPER |= GPIO_OTYPER_OT15; 
//         GPIOA->BSRR = GPIO_PIN_15;         

//         // Initialize the internal TLE5012
//         SPI1_Init();
        
//         NVIC_SetPriority(DMA1_Channel1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0));
//         NVIC_EnableIRQ(DMA1_Channel1_IRQn);
//     }
// }

// // ==============================================================================
// // UNIVERSAL ANGLE GETTER (32-bit Left Aligned)
// // ==============================================================================
// static inline uint32_t Encoder_GetAngle() {
//     if (ENCODER_USE_EXTERNAL) {
//         return POSITION_ENCODER_MT_RAW;
//     } else {
//         return ((uint32_t)POSITION_ENCODER_TLE) << 16;
//     }
// }

// static inline uint8_t Encoder_GetMTStatus() {
//     return ENCODER_USE_EXTERNAL ? POSITION_MT_STATUS : 0;
// }

// static inline bool Encoder_IsUsingExternal() {
//     return ENCODER_USE_EXTERNAL;
// }

// // ==============================================================================
// // SHARED DMA INTERRUPT HANDLER
// // ==============================================================================
// extern "C" void DMA1_Channel1_IRQHandler(void) {
//     if (DMA1->ISR & DMA_ISR_TCIF1) {
//         DMA1->IFCR = DMA_IFCR_CGIF1;

//         if (ENCODER_USE_EXTERNAL) {
//             // Process the external MT6835
//             // OPTIMIZATION (P1.3): at 2.7-5.3 MHz SPI the last bytes are still
//             // shifting after DMA-complete. Sleep in WFE (woken by the
//             // higher-priority ADC/TIM1 interrupts that fire every cycle)
//             // instead of busy-spinning NOPs inside this priority-5 ISR.
//             while (SPI3->SR & SPI_SR_BSY) { __WFE(); } // Wait for the physical SPI completion
//             SPI3_ChipSelect(false);

//             POSITION_ENCODER_MT_RAW = ((uint32_t)SPI3_RX_BUF[2] << 13) | ((uint32_t)SPI3_RX_BUF[3] << 5) | ((uint32_t)SPI3_RX_BUF[4] >> 3);
//             POSITION_ENCODER_MT_RAW <<= 11;
//             POSITION_MT_STATUS = (uint8_t)(SPI3_RX_BUF[4] & 0x07U);
//             POSITION_MT_CRC    = SPI3_RX_BUF[5];
//         } else {
//             // Process the internal TLE5012
//             GPIOA->BSRR = GPIO_PIN_4; // Deassert CS atomically (SPI1_ChipSelect(false))

//             POSITION_ENCODER_TLE = ((uint16_t)SPI1_RX_BUF[2] << 9) | ((uint16_t)SPI1_RX_BUF[3] << 1); 
//         }

//         // Invoke the motor-control algorithm
//         POSITION_Callback(); 
//     }
// }

// #endif // SMART_ENCODER_SETUP_H











// // #ifndef SMART_ENCODER_SETUP_H
// // #define SMART_ENCODER_SETUP_H

// // #include <Arduino.h>
// // #include "stm32g4xx.h"

// // extern void POSITION_Callback();

// // // =============================================================================
// // // MT6835 on the internal SPI1
// // //
// // // PA4 = CSN   (GPIO output)
// // // PA5 = SCK   (AF5 SPI1)
// // // PA6 = MISO  (AF5 SPI1)
// // // PA7 = MOSI  (AF5 SPI1)
// // //
// // // SPI Mode 3: CPOL = 1, CPHA = 1
// // // Burst read: 0xA003 followed by 4 dummy bytes.
// // // RX[2] = ANGLE[20:13]
// // // RX[3] = ANGLE[12:5]
// // // RX[4] = ANGLE[4:0] + STATUS[2:0]
// // // RX[5] = CRC
// // // =============================================================================

// // // Transmit 0xA003 + four dummy bytes.
// // static uint8_t SPI1_TX_BUF[6] __attribute__((aligned(4))) = {
// //     0xA0, 0x03, 0x00, 0x00, 0x00, 0x00
// // };

// // static uint8_t SPI1_RX_BUF[6] __attribute__((aligned(4))) = {0};

// // // MT6835 angle in Q0.32 / left-aligned format:
// // // bit 31 = ANGLE[20], bit 11 = ANGLE[0], bits [10:0] = 0.
// // static volatile uint32_t POSITION_ENCODER_MT_RAW = 0;

// // // STATUS from the three lowest bits of RX[4]:
// // // bit 0 = overspeed
// // // bit 1 = weak magnetic field
// // // bit 2 = undervoltage
// // static volatile uint8_t POSITION_MT_STATUS = 0;

// // // Received CRC from RX[5].
// // static volatile uint8_t POSITION_MT_CRC = 0;

// // // Can be used for diagnostics in the main code.
// // static volatile bool POSITION_MT_DATA_VALID = false;


// // // =============================================================================
// // // CS MT6835: PA4, active LOW
// // // =============================================================================
// // static inline void MT6835_ChipSelect(bool select) {
// //     if (select) {
// //         GPIOA->BRR = GPIO_PIN_4;
// //     } else {
// //         GPIOA->BSRR = GPIO_PIN_4;
// //     }
// // }


// // // =============================================================================
// // // GPIO: PA4 / PA5 / PA6 / PA7
// // // =============================================================================
// // static inline void SPI1_GPIO_Init() {
// //     RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
// //     __DSB();

// //     // ---- PA4: CSN, GPIO output push-pull ----
// //     GPIOA->MODER &= ~(3U << (4U * 2U));
// //     GPIOA->MODER |=  (1U << (4U * 2U));

// //     GPIOA->OTYPER &= ~GPIO_OTYPER_OT4;
// //     GPIOA->OSPEEDR |= (3U << (4U * 2U));
// //     GPIOA->PUPDR &= ~(3U << (4U * 2U));

// //     // CSN idle HIGH
// //     GPIOA->BSRR = GPIO_PIN_4;

// //     // ---- PA5=SCK, PA6=MISO, PA7=MOSI: AF5 SPI1 ----
// //     GPIOA->MODER &= ~(
// //         (3U << (5U * 2U)) |
// //         (3U << (6U * 2U)) |
// //         (3U << (7U * 2U))
// //     );

// //     GPIOA->MODER |= (
// //         (2U << (5U * 2U)) |
// //         (2U << (6U * 2U)) |
// //         (2U << (7U * 2U))
// //     );

// //     GPIOA->OTYPER &= ~(
// //         GPIO_OTYPER_OT5 |
// //         GPIO_OTYPER_OT6 |
// //         GPIO_OTYPER_OT7
// //     );

// //     GPIOA->OSPEEDR |= (
// //         (3U << (5U * 2U)) |
// //         (3U << (6U * 2U)) |
// //         (3U << (7U * 2U))
// //     );

// //     GPIOA->PUPDR &= ~(
// //         (3U << (5U * 2U)) |
// //         (3U << (6U * 2U)) |
// //         (3U << (7U * 2U))
// //     );

// //     // AF5: SPI1 on PA5/PA6/PA7
// //     GPIOA->AFR[0] &= ~(
// //         (0xFU << (5U * 4U)) |
// //         (0xFU << (6U * 4U)) |
// //         (0xFU << (7U * 4U))
// //     );

// //     GPIOA->AFR[0] |= (
// //         (5U << (5U * 4U)) |
// //         (5U << (6U * 4U)) |
// //         (5U << (7U * 4U))
// //     );
// // }


// // // =============================================================================
// // // SPI1 + DMA1
// // // DMA1 Channel 1 = RX
// // // DMA1 Channel 2 = TX
// // //
// // // DMAMUX:
// // // DMA1 Channel 1 <- SPI1_RX = request 10
// // // DMA1 Channel 2 <- SPI1_TX = request 11
// // // =============================================================================
// // static inline void SPI1_Init() {
// //     SPI1_GPIO_Init();

// //     RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
// //     RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMAMUX1EN;
// //     __DSB();

// //     // Both channels are guaranteed disabled before DMA reconfiguration.
// //     DMA1_Channel1->CCR &= ~DMA_CCR_EN;
// //     DMA1_Channel2->CCR &= ~DMA_CCR_EN;

// //     // SPI1_RX -> DMA1 Channel 1
// //     DMAMUX1_Channel0->CCR = 10U;

// //     // SPI1_TX -> DMA1 Channel 2
// //     DMAMUX1_Channel1->CCR = 11U;

// //     // ---- DMA RX: SPI1->DR -> SPI1_RX_BUF ----
// //     DMA1_Channel1->CPAR  = (uint32_t)&SPI1->DR;
// //     DMA1_Channel1->CMAR  = (uint32_t)SPI1_RX_BUF;
// //     DMA1_Channel1->CNDTR = 0;

// //     DMA1_Channel1->CCR =
// //         DMA_CCR_MINC |
// //         DMA_CCR_TCIE |
// //         (3U << DMA_CCR_PL_Pos);

// //     // ---- DMA TX: SPI1_TX_BUF -> SPI1->DR ----
// //     DMA1_Channel2->CPAR  = (uint32_t)&SPI1->DR;
// //     DMA1_Channel2->CMAR  = (uint32_t)SPI1_TX_BUF;
// //     DMA1_Channel2->CNDTR = 0;

// //     DMA1_Channel2->CCR =
// //         DMA_CCR_MINC |
// //         DMA_CCR_DIR |
// //         (2U << DMA_CCR_PL_Pos);

// //     // SPI master, software NSS, Mode 3:
// //     // CPOL=1, CPHA=1.
// //     //
// //     // BR=5: PCLK2 / 64.
// //     // At PCLK2=170 MHz this is approximately 2.66 MHz.
// //     SPI1->CR1 =
// //         SPI_CR1_MSTR |
// //         SPI_CR1_SSM |
// //         SPI_CR1_SSI |
// //         SPI_CR1_CPOL |
// //         SPI_CR1_CPHA |
// //         (5U << SPI_CR1_BR_Pos);

// //     // 8-bit frame, RX/TX DMA enabled.
// //     SPI1->CR2 =
// //         SPI_CR2_RXDMAEN |
// //         SPI_CR2_TXDMAEN |
// //         SPI_CR2_FRXTH |
// //         (7U << SPI_CR2_DS_Pos);

// //     // Clear stale states.
// //     (void)SPI1->DR;
// //     (void)SPI1->SR;

// //     DMA1->IFCR =
// //         DMA_IFCR_CGIF1 |
// //         DMA_IFCR_CGIF2;

// //     MT6835_ChipSelect(false);

// //     SPI1->CR1 |= SPI_CR1_SPE;
// // }


// // // =============================================================================
// // // MT6835 CRC-8 verification
// // // Polynomial: x^8 + x^2 + x + 1 = 0x07
// // // CRC is computed over RX[2], RX[3], RX[4].
// // // =============================================================================
// // static inline uint8_t MT6835_CalcCRC(const uint8_t* data, uint8_t len) {
// //     uint8_t crc = 0x00;

// //     while (len--) {
// //         crc ^= *data++;

// //         for (uint8_t bit = 0; bit < 8; bit++) {
// //             if (crc & 0x80U) {
// //                 crc = (uint8_t)((crc << 1U) ^ 0x07U);
// //             } else {
// //                 crc <<= 1U;
// //             }
// //         }
// //     }

// //     return crc;
// // }


// // // =============================================================================
// // // Parse a received MT6835 packet
// // // =============================================================================
// // static inline void MT6835_ParseResponse() {
// //     const uint32_t angle21 =
// //         ((uint32_t)SPI1_RX_BUF[2] << 13U) |
// //         ((uint32_t)SPI1_RX_BUF[3] << 5U)  |
// //         ((uint32_t)SPI1_RX_BUF[4] >> 3U);

// //     POSITION_ENCODER_MT_RAW = angle21 << 11U;
// //     POSITION_MT_STATUS = SPI1_RX_BUF[4] & 0x07U;
// //     POSITION_MT_CRC = SPI1_RX_BUF[5];

// //     const uint8_t crcCalculated = MT6835_CalcCRC(&SPI1_RX_BUF[2], 3);
// //     POSITION_MT_DATA_VALID = (crcCalculated == POSITION_MT_CRC);
// // }


// // // =============================================================================
// // // Start a single MT6835 burst-read via SPI1 + DMA
// // //
// // // Important: start the next transfer only after the previous one completes.
// // // Normally called from the control timer; the DMA IRQ invokes POSITION_Callback().
// // // =============================================================================
// // static inline void Encoder_StartTransfer() {
// //     // Guard against an erroneous re-start corrupting the current transaction.
// //     if (DMA1_Channel1->CCR & DMA_CCR_EN) {
// //         return;
// //     }

// //     MT6835_ChipSelect(true);

// //     DMA1_Channel1->CCR &= ~DMA_CCR_EN;
// //     DMA1_Channel2->CCR &= ~DMA_CCR_EN;

// //     DMA1->IFCR =
// //         DMA_IFCR_CGIF1 |
// //         DMA_IFCR_CGIF2;

// //     DMA1_Channel1->CNDTR = 6;
// //     DMA1_Channel2->CNDTR = 6;

// //     // RX is enabled first, otherwise the first received byte may be lost.
// //     DMA1_Channel1->CCR |= DMA_CCR_EN;
// //     DMA1_Channel2->CCR |= DMA_CCR_EN;
// // }


// // // =============================================================================
// // // Initialization: MT6835 on SPI1 only, no fallback and no auto-detection.
// // // =============================================================================
// // static inline void Encoder_SmartInit() {
// //     // Let the encoder stabilize after power-up.
// //     delay(20);

// //     SPI1_Init();

// //     NVIC_DisableIRQ(DMA1_Channel1_IRQn);
// //     NVIC_ClearPendingIRQ(DMA1_Channel1_IRQn);

// //     NVIC_SetPriority(
// //         DMA1_Channel1_IRQn,
// //         NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0)
// //     );

// //     NVIC_EnableIRQ(DMA1_Channel1_IRQn);

// //     // Optional: first request to fill in the position before starting FOC.
// //     Encoder_StartTransfer();
// // }


// // // =============================================================================
// // // API compatible with the legacy code
// // // =============================================================================

// // // Returns the angle in left-aligned 32-bit format:
// // // 0x00000000 = 0 deg
// // // 0x80000000 = 180 deg
// // // 0xFFFFF800 = nearly 360 deg
// // static inline uint32_t Encoder_GetAngle() {
// //     return POSITION_ENCODER_MT_RAW;
// // }

// // static inline uint8_t Encoder_GetMTStatus() {
// //     return POSITION_MT_STATUS;
// // }

// // static inline uint8_t Encoder_GetMTCRC() {
// //     return POSITION_MT_CRC;
// // }

// // static inline bool Encoder_IsDataValid() {
// //     return POSITION_MT_DATA_VALID;
// // }

// // // Kept only for compatibility with old calling code.
// // // The MT6835 is now always the single internal encoder.
// // static inline bool Encoder_IsUsingExternal() {
// //     return false;
// // }


// // // =============================================================================
// // // DMA RX complete IRQ
// // // =============================================================================
// // extern "C" void DMA1_Channel1_IRQHandler(void) {
// //     const uint32_t isr = DMA1->ISR;

// //     // Transfer error: finish the SPI transaction safely.
// //     if (isr & DMA_ISR_TEIF1) {
// //         DMA1->IFCR = DMA_IFCR_CTEIF1 | DMA_IFCR_CGIF1;

// //         DMA1_Channel1->CCR &= ~DMA_CCR_EN;
// //         DMA1_Channel2->CCR &= ~DMA_CCR_EN;

// //         while (SPI1->SR & SPI_SR_BSY) {
// //             __NOP();
// //         }

// //         MT6835_ChipSelect(false);
// //         POSITION_MT_DATA_VALID = false;
// //         return;
// //     }

// //     if (isr & DMA_ISR_TCIF1) {
// //         DMA1->IFCR = DMA_IFCR_CGIF1;

// //         // RX DMA completed. Always wait for the final SPI edge before raising CS.
// //         while (SPI1->SR & SPI_SR_BSY) {
// //             __NOP();
// //         }

// //         MT6835_ChipSelect(false);

// //         MT6835_ParseResponse();

// //         // Called after POSITION_ENCODER_MT_RAW has been updated.
// //         POSITION_Callback();
// //     }
// // }

// // #endif // SMART_ENCODER_SETUP_H