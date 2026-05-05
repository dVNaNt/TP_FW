// #ifndef SPI_SETUP_CLN17V3_H // Include guard to prevent multiple inclusions of this header file
// #define SPI_SETUP_CLN17V3_H // Define the include guard macro for the preprocessor

// #include <Arduino.h> // Include Arduino framework for basic standard types
// #include "stm32g4xx.h" // Include pure CMSIS register definitions for STM32G4, replacing HAL

// extern void POSITION_Callback(); // Forward declaration for the FOC position update callback

// // TLE5012 transaction: 4 bytes write, 4 bytes read
// // Force 32-bit memory boundary alignment for optimal 32-bit AHB bus matrix DMA transfers
// static uint8_t SPI1_TX_BUF[4] __attribute__((aligned(4))) = {0x80, 0x20, 0xFF, 0xFF}; // Command sequence to read TLE5012 angle
// static uint8_t SPI1_RX_BUF[4] __attribute__((aligned(4))) = {0}; // Zero-initialized reception buffer
// static volatile uint16_t POSITION_ENCODER = 0; // Volatile global variable to store the parsed encoder angle

// /**
//  * @brief Fast direct-register SPI1 chip select (PA4)
//  * Uses GPIO BSRR/BRR for single-cycle atomic execution.
//  * Kept as software GPIO (not hardware NSS) to guarantee it stays LOW for the entire 4-byte DMA stream.
//  */
// static inline void SPI1_ChipSelect(bool select) { // Inline function for ultra-fast CS toggling
//     if (select) { // If true, we want to select the chip (Active LOW)
//         GPIOA->BRR = GPIO_PIN_4; // Bit Reset Register: Pulls PA4 LOW atomically
//     } else { // If false, we want to release the chip (Idle HIGH)
//         GPIOA->BSRR = GPIO_PIN_4; // Bit Set/Reset Register: Pulls PA4 HIGH atomically
//     } // End of conditional check
// } // End of SPI1_ChipSelect function

// /**
//  * @brief Initialize SPI1 and DMA purely via bare-metal registers
//  */
// static inline void SPI1_Init() { // Inline initialization function bypassing HAL
//     RCC->APB2ENR |= RCC_APB2ENR_SPI1EN; // Enable hardware clock routing for the SPI1 peripheral
//     RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMAMUX1EN; // Enable clocks for DMA1 and the DMA Multiplexer

//     // 1. DMAMUX Routing (Crucial for STM32G4 bare-metal DMA)
//     DMAMUX1_Channel0->CCR = 10; // Route SPI1_RX request (ID 10) to DMA1 Channel 1
//     DMAMUX1_Channel1->CCR = 11; // Route SPI1_TX request (ID 11) to DMA1 Channel 2

//     // 2. DMA1 Channel 1 (RX) Configuration
//     DMA1_Channel1->CPAR = (uint32_t)&SPI1->DR; // Set peripheral address to SPI1 Data Register
//     DMA1_Channel1->CMAR = (uint32_t)SPI1_RX_BUF; // Set memory address to our RX buffer
//     // Config: Very High Priority (PL=3), Memory Increment (MINC), 8-bit sizes, Transfer Complete Interrupt (TCIE)
//     DMA1_Channel1->CCR = DMA_CCR_MINC | DMA_CCR_TCIE | (3U << DMA_CCR_PL_Pos); 

//     // 3. DMA1 Channel 2 (TX) Configuration
//     DMA1_Channel2->CPAR = (uint32_t)&SPI1->DR; // Set peripheral address to SPI1 Data Register
//     DMA1_Channel2->CMAR = (uint32_t)SPI1_TX_BUF; // Set memory address to our TX buffer
//     // Config: High Priority (PL=2), Memory Increment (MINC), Memory-to-Peripheral direction (DIR), 8-bit sizes
//     DMA1_Channel2->CCR = DMA_CCR_MINC | DMA_CCR_DIR | (2U << DMA_CCR_PL_Pos); 

//     // 4. SPI1 Peripheral Configuration
//     SPI1->CR1 = 0; // Clear CR1 to reset the peripheral state
//     // Config: Master Mode, Software Slave Management (SSM=1, SSI=1), Phase 2 Edge (CPHA=1), Baud Rate /64 (BR=5)
//     SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_CPHA | (5U << SPI_CR1_BR_Pos); 
    
//     // Config: Enable TX/RX DMA, 8-bit Data Size (DS=7), and CRITICAL: Set RX FIFO Threshold to 8-bit (FRXTH=1)
//     SPI1->CR2 = SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN | SPI_CR2_FRXTH | (7U << SPI_CR2_DS_Pos); 

//     // 5. NVIC Interrupt Enable for RX DMA completion
//     NVIC_SetPriority(DMA1_Channel1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0)); // Set optimal priority
//     NVIC_EnableIRQ(DMA1_Channel1_IRQn); // Enable hardware interrupt for DMA RX channel
// } // End of SPI1_Init function

// /**
//  * @brief Enable SPI peripheral (Run once after Init)
//  */
// static inline void SPI1_Start() { // Separate function to activate the hardware
//     SPI1_ChipSelect(false); // Ensure CS starts HIGH (deselected)
//     SPI1->CR1 |= SPI_CR1_SPE; // Enable SPI peripheral (Set SPE bit)
// } // End of SPI1_Start function

// /**
//  * @brief Fast direct-register start of SPI DMA transfer
//  */
// static inline void SPI1_StartTransfer() { // Ultra-fast inline function to trigger transactions
//     SPI1_ChipSelect(true); // Pull CS LOW to wake the TLE5012 and begin framing

//     DMA1_Channel1->CCR &= ~DMA_CCR_EN; // Disable RX DMA channel to allow CNDTR update
//     DMA1_Channel2->CCR &= ~DMA_CCR_EN; // Disable TX DMA channel to allow CNDTR update

//     DMA1->IFCR = DMA_IFCR_CGIF1 | DMA_IFCR_CGIF2; // Atomically clear all global interrupt flags from previous transfers

//     DMA1_Channel1->CNDTR = 4; // Load exactly 4 bytes into the RX DMA countdown register
//     DMA1_Channel2->CNDTR = 4; // Load exactly 4 bytes into the TX DMA countdown register

//     DMA1_Channel1->CCR |= DMA_CCR_EN; // Enable RX DMA channel FIRST to ensure no incoming bits are dropped
//     DMA1_Channel2->CCR |= DMA_CCR_EN; // Enable TX DMA channel SECOND to initiate the physical SPI clock generation
// } // End of SPI1_StartTransfer function

// static inline uint16_t SPI1_GetAngle() { // Inline getter function for the parsed angle
//     return POSITION_ENCODER; // Return the safely cached volatile encoder value
// } // End of SPI1_GetAngle function

// // ========================================================================= // Visual separator for ISR
// // Direct DMA RX IRQ Handler. Replaces HAL_DMA_IRQHandler overhead.          // Description of ISR replacement
// // ========================================================================= // Visual separator for ISR

// extern "C" void DMA1_Channel1_IRQHandler(void) { // Bare-metal interrupt vector for DMA1 Channel 1
//     if (DMA1->ISR & DMA_ISR_TCIF1) { // Check purely for the Transfer Complete (TC) flag for Channel 1
//         DMA1->IFCR = DMA_IFCR_CGIF1; // Clear all interrupt flags for Channel 1 immediately
        
//         GPIOA->BSRR = GPIO_PIN_4; // Raise CS HIGH atomically. DMA RX completion guarantees SPI shifting is fully done.
        
//         // Safely parse angle: cast to uint16_t before shifting to prevent aggressive compiler overflow bugs
//         POSITION_ENCODER = ((uint16_t)SPI1_RX_BUF[2] << 9) | ((uint16_t)SPI1_RX_BUF[3] << 1); 
        
//         POSITION_Callback(); // Directly invoke the FOC position update logic
//     } // End of Transfer Complete check
// } // End of DMA1_Channel1_IRQHandler function

// // TX and SPI interrupts remain completely eliminated to save vital CPU cycles. // Justification comment

// #endif // SPI_SETUP_CLN17V3_H // Final close of the header include guard


#ifndef SPI3_SETUP_MT6835_H
#define SPI3_SETUP_MT6835_H

#include <Arduino.h>
#include "stm32g4xx.h"

extern void POSITION_Callback();

// -----------------------------------------------------------------------------
// MT6835 burst read angle:
// First 2 bytes on MOSI = command 1010 + address 0x003
// Then 4 dummy bytes to clock out registers 0x003..0x006 from MISO
//
// Total frame = 6 bytes:
// TX[0] = 0b1010_0000 = 0xA0
// TX[1] = 0x03
// TX[2..5] = dummy
//
// RX[0..1] = undefined / Hi-Z phase during command
// RX[2]    = Reg 0x003 = ANGLE[20:13]
// RX[3]    = Reg 0x004 = ANGLE[12:5]
// RX[4]    = Reg 0x005 = ANGLE[4:0] | STATUS[2:0]
// RX[5]    = Reg 0x006 = CRC[7:0]
// -----------------------------------------------------------------------------

static uint8_t SPI3_TX_BUF[6] __attribute__((aligned(4))) = {
    0xA0, 0x03, 0x00, 0x00, 0x00, 0x00
};

static uint8_t SPI3_RX_BUF[6] __attribute__((aligned(4))) = {0};

static volatile uint32_t POSITION_ENCODER_RAW = 0;   // 21-bit raw angle
static volatile uint8_t  POSITION_STATUS = 0;        // STATUS[2:0]
static volatile uint8_t  POSITION_CRC = 0;           // CRC byte from MT6835

// -----------------------------------------------------------------------------
// CS = PA15, software-controlled
// -----------------------------------------------------------------------------
static inline void SPI3_ChipSelect(bool select) {
    if (select) {
        GPIOA->BRR = GPIO_PIN_15;   // CS LOW
    } else {
        GPIOA->BSRR = GPIO_PIN_15;  // CS HIGH
    }
}


// -----------------------------------------------------------------------------
// GPIO init for SPI3:
// PB3  = SPI3_SCK  AF6
// PB4  = SPI3_MISO AF6
// PB5  = SPI3_MOSI AF6
// PA15 = GPIO output for CS
//
// Note: PB3 is also SWO/JTDO on some debug configurations.
// If SWJ is using it, disable trace output in your debug config.
// -----------------------------------------------------------------------------
static inline void SPI3_GPIO_Init() {
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN;

    // ---- PA15 as GPIO output for CS ----
    GPIOA->MODER &= ~(3U << (15 * 2));
    GPIOA->MODER |=  (1U << (15 * 2));       // Output
    GPIOA->OTYPER &= ~GPIO_OTYPER_OT15;      // Push-pull
    GPIOA->OSPEEDR |= (3U << (15 * 2));      // Very high speed
    GPIOA->PUPDR &= ~(3U << (15 * 2));       // No pull
    GPIOA->BSRR = GPIO_PIN_15;               // CS HIGH

    // ---- PB3/PB4/PB5 -> AF6 SPI3 ----
    // MODER = AF
    GPIOB->MODER &= ~((3U << (3 * 2)) | (3U << (4 * 2)) | (3U << (5 * 2)));
    GPIOB->MODER |=  ((2U << (3 * 2)) | (2U << (4 * 2)) | (2U << (5 * 2)));

    // Push-pull
    GPIOB->OTYPER &= ~(GPIO_OTYPER_OT3 | GPIO_OTYPER_OT4 | GPIO_OTYPER_OT5);

    // Very high speed
    GPIOB->OSPEEDR |= ((3U << (3 * 2)) | (3U << (4 * 2)) | (3U << (5 * 2)));

    // Pulls:
    // SCK/MOSI usually no-pull, MISO no-pull too if encoder drives it push-pull
    GPIOB->PUPDR &= ~((3U << (3 * 2)) | (3U << (4 * 2)) | (3U << (5 * 2)));

    // AFRL: AF6 for PB3/PB4/PB5
    GPIOB->AFR[0] &= ~((0xFU << (3 * 4)) | (0xFU << (4 * 4)) | (0xFU << (5 * 4)));
    GPIOB->AFR[0] |=  ((6U   << (3 * 4)) | (6U   << (4 * 4)) | (6U   << (5 * 4)));
}

// -----------------------------------------------------------------------------
// SPI3 + DMA bare-metal init
// -----------------------------------------------------------------------------
static inline void SPI3_Init() {
    SPI3_GPIO_Init();

    RCC->APB1ENR1 |= RCC_APB1ENR1_SPI3EN;
    RCC->AHB1ENR  |= RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMAMUX1EN;

    // -------------------------------------------------------------------------
    // DMAMUX routing for STM32G4:
    // SPI3_RX = 14 (0x0E)
    // SPI3_TX = 15 (0x0F)
    // DMA1 Channel1 = RX
    // DMA1 Channel2 = TX
    // -------------------------------------------------------------------------
    DMAMUX1_Channel0->CCR = 14U;   // DMA1 Ch1 <- SPI3_RX
    DMAMUX1_Channel1->CCR = 15U;   // DMA1 Ch2 <- SPI3_TX

    // -------------------------------------------------------------------------
    // DMA RX: DMA1 Channel1
    // -------------------------------------------------------------------------
    DMA1_Channel1->CCR &= ~DMA_CCR_EN;
    DMA1_Channel1->CPAR = (uint32_t)&SPI3->DR;
    DMA1_Channel1->CMAR = (uint32_t)SPI3_RX_BUF;
    DMA1_Channel1->CCR =
        DMA_CCR_MINC |
        DMA_CCR_TCIE |
        (3U << DMA_CCR_PL_Pos);    // Very high priority

    // -------------------------------------------------------------------------
    // DMA TX: DMA1 Channel2
    // -------------------------------------------------------------------------
    DMA1_Channel2->CCR &= ~DMA_CCR_EN;
    DMA1_Channel2->CPAR = (uint32_t)&SPI3->DR;
    DMA1_Channel2->CMAR = (uint32_t)SPI3_TX_BUF;
    DMA1_Channel2->CCR =
        DMA_CCR_MINC |
        DMA_CCR_DIR |
        (2U << DMA_CCR_PL_Pos);    // High priority

    // -------------------------------------------------------------------------
    // SPI3 config
    // MT6835 requires SPI mode 3: CPOL=1, CPHA=1
    //
    // BR example:
    // if PCLK = 170 MHz, BR=4 => /32 => 5.3125 MHz
    // if PCLK = 85 MHz,  BR=4 => /32 => 2.656 MHz
    //
    // MT6835 allows up to 16 MHz SPI clock.
    // -------------------------------------------------------------------------
    SPI3->CR1 = 0;
    SPI3->CR1 =
        SPI_CR1_MSTR |
        SPI_CR1_SSM  |
        SPI_CR1_SSI  |
        SPI_CR1_CPOL |
        SPI_CR1_CPHA |
        (4U << SPI_CR1_BR_Pos);   // /32, safe start

    SPI3->CR2 =
        SPI_CR2_RXDMAEN |
        SPI_CR2_TXDMAEN |
        SPI_CR2_FRXTH   |
        (7U << SPI_CR2_DS_Pos);   // 8-bit frame

    // Clean stale RX data / flags if any
    (void)SPI3->DR;
    (void)SPI3->SR;

    NVIC_SetPriority(DMA1_Channel1_IRQn,
        NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0));
    NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

// -----------------------------------------------------------------------------
// Enable SPI after init
// -----------------------------------------------------------------------------
static inline void SPI3_Start() {
    SPI3_ChipSelect(false);
    SPI3->CR1 |= SPI_CR1_SPE;
}

// -----------------------------------------------------------------------------
// Start one MT6835 burst-read transfer (6 bytes)
// -----------------------------------------------------------------------------
static inline void SPI3_StartTransfer() {
    SPI3_ChipSelect(true);

    DMA1_Channel1->CCR &= ~DMA_CCR_EN;
    DMA1_Channel2->CCR &= ~DMA_CCR_EN;

    DMA1->IFCR = DMA_IFCR_CGIF1 | DMA_IFCR_CGIF2;

    DMA1_Channel1->CNDTR = 6;
    DMA1_Channel2->CNDTR = 6;

    DMA1_Channel1->CCR |= DMA_CCR_EN;  // RX first
    DMA1_Channel2->CCR |= DMA_CCR_EN;  // then TX
}

// -----------------------------------------------------------------------------
// Accessors
// -----------------------------------------------------------------------------
static inline uint32_t SPI3_GetAngle() {
    return POSITION_ENCODER_RAW;
}

static inline uint8_t SPI3_GetStatus() {
    return POSITION_STATUS;
}

static inline uint8_t SPI3_GetCRC() {
    return POSITION_CRC;
}

// -----------------------------------------------------------------------------
// DMA RX complete ISR
// -----------------------------------------------------------------------------
extern "C" void DMA1_Channel1_IRQHandler(void) {
    if (DMA1->ISR & DMA_ISR_TCIF1) {
        DMA1->IFCR = DMA_IFCR_CGIF1;

        // Wait until SPI is no longer busy before releasing CS
        while (SPI3->SR & SPI_SR_BSY) { __NOP(); }

        SPI3_ChipSelect(false);

        // RX[2] = ANGLE[20:13]
        // RX[3] = ANGLE[12:5]
        // RX[4] = ANGLE[4:0] in bits [7:3], STATUS[2:0] in bits [2:0]
        POSITION_ENCODER_RAW =
            ((uint32_t)SPI3_RX_BUF[2] << 13) |
            ((uint32_t)SPI3_RX_BUF[3] << 5)  |
            ((uint32_t)SPI3_RX_BUF[4] >> 3);
        POSITION_ENCODER_RAW <<= 11;

        POSITION_STATUS = (uint8_t)(SPI3_RX_BUF[4] & 0x07U);
        POSITION_CRC    = SPI3_RX_BUF[5];

        POSITION_Callback();
    }
}

#endif // SPI3_SETUP_MT6835_H