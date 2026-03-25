#ifndef SPI_SETUP_CLN17V3_H // Include guard to prevent multiple inclusions of this header file
#define SPI_SETUP_CLN17V3_H // Define the include guard macro for the preprocessor

#include <Arduino.h> // Include Arduino framework for basic standard types
#include "stm32g4xx.h" // Include pure CMSIS register definitions for STM32G4, replacing HAL

extern void POSITION_Callback(); // Forward declaration for the FOC position update callback

// TLE5012 transaction: 4 bytes write, 4 bytes read
// Force 32-bit memory boundary alignment for optimal 32-bit AHB bus matrix DMA transfers
static uint8_t SPI1_TX_BUF[4] __attribute__((aligned(4))) = {0x80, 0x20, 0xFF, 0xFF}; // Command sequence to read TLE5012 angle
static uint8_t SPI1_RX_BUF[4] __attribute__((aligned(4))) = {0}; // Zero-initialized reception buffer
static volatile uint16_t POSITION_ENCODER = 0; // Volatile global variable to store the parsed encoder angle

/**
 * @brief Fast direct-register SPI1 chip select (PA4)
 * Uses GPIO BSRR/BRR for single-cycle atomic execution.
 * Kept as software GPIO (not hardware NSS) to guarantee it stays LOW for the entire 4-byte DMA stream.
 */
static inline void SPI1_ChipSelect(bool select) { // Inline function for ultra-fast CS toggling
    if (select) { // If true, we want to select the chip (Active LOW)
        GPIOA->BRR = GPIO_PIN_4; // Bit Reset Register: Pulls PA4 LOW atomically
    } else { // If false, we want to release the chip (Idle HIGH)
        GPIOA->BSRR = GPIO_PIN_4; // Bit Set/Reset Register: Pulls PA4 HIGH atomically
    } // End of conditional check
} // End of SPI1_ChipSelect function

/**
 * @brief Initialize SPI1 and DMA purely via bare-metal registers
 */
static inline void SPI1_Init() { // Inline initialization function bypassing HAL
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN; // Enable hardware clock routing for the SPI1 peripheral
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMAMUX1EN; // Enable clocks for DMA1 and the DMA Multiplexer

    // 1. DMAMUX Routing (Crucial for STM32G4 bare-metal DMA)
    DMAMUX1_Channel0->CCR = 10; // Route SPI1_RX request (ID 10) to DMA1 Channel 1
    DMAMUX1_Channel1->CCR = 11; // Route SPI1_TX request (ID 11) to DMA1 Channel 2

    // 2. DMA1 Channel 1 (RX) Configuration
    DMA1_Channel1->CPAR = (uint32_t)&SPI1->DR; // Set peripheral address to SPI1 Data Register
    DMA1_Channel1->CMAR = (uint32_t)SPI1_RX_BUF; // Set memory address to our RX buffer
    // Config: Very High Priority (PL=3), Memory Increment (MINC), 8-bit sizes, Transfer Complete Interrupt (TCIE)
    DMA1_Channel1->CCR = DMA_CCR_MINC | DMA_CCR_TCIE | (3U << DMA_CCR_PL_Pos); 

    // 3. DMA1 Channel 2 (TX) Configuration
    DMA1_Channel2->CPAR = (uint32_t)&SPI1->DR; // Set peripheral address to SPI1 Data Register
    DMA1_Channel2->CMAR = (uint32_t)SPI1_TX_BUF; // Set memory address to our TX buffer
    // Config: High Priority (PL=2), Memory Increment (MINC), Memory-to-Peripheral direction (DIR), 8-bit sizes
    DMA1_Channel2->CCR = DMA_CCR_MINC | DMA_CCR_DIR | (2U << DMA_CCR_PL_Pos); 

    // 4. SPI1 Peripheral Configuration
    SPI1->CR1 = 0; // Clear CR1 to reset the peripheral state
    // Config: Master Mode, Software Slave Management (SSM=1, SSI=1), Phase 2 Edge (CPHA=1), Baud Rate /64 (BR=5)
    SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_CPHA | (5U << SPI_CR1_BR_Pos); 
    
    // Config: Enable TX/RX DMA, 8-bit Data Size (DS=7), and CRITICAL: Set RX FIFO Threshold to 8-bit (FRXTH=1)
    SPI1->CR2 = SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN | SPI_CR2_FRXTH | (7U << SPI_CR2_DS_Pos); 

    // 5. NVIC Interrupt Enable for RX DMA completion
    NVIC_SetPriority(DMA1_Channel1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0)); // Set optimal priority
    NVIC_EnableIRQ(DMA1_Channel1_IRQn); // Enable hardware interrupt for DMA RX channel
} // End of SPI1_Init function

/**
 * @brief Enable SPI peripheral (Run once after Init)
 */
static inline void SPI1_Start() { // Separate function to activate the hardware
    SPI1_ChipSelect(false); // Ensure CS starts HIGH (deselected)
    SPI1->CR1 |= SPI_CR1_SPE; // Enable SPI peripheral (Set SPE bit)
} // End of SPI1_Start function

/**
 * @brief Fast direct-register start of SPI DMA transfer
 */
static inline void SPI1_StartTransfer() { // Ultra-fast inline function to trigger transactions
    SPI1_ChipSelect(true); // Pull CS LOW to wake the TLE5012 and begin framing

    DMA1_Channel1->CCR &= ~DMA_CCR_EN; // Disable RX DMA channel to allow CNDTR update
    DMA1_Channel2->CCR &= ~DMA_CCR_EN; // Disable TX DMA channel to allow CNDTR update

    DMA1->IFCR = DMA_IFCR_CGIF1 | DMA_IFCR_CGIF2; // Atomically clear all global interrupt flags from previous transfers

    DMA1_Channel1->CNDTR = 4; // Load exactly 4 bytes into the RX DMA countdown register
    DMA1_Channel2->CNDTR = 4; // Load exactly 4 bytes into the TX DMA countdown register

    DMA1_Channel1->CCR |= DMA_CCR_EN; // Enable RX DMA channel FIRST to ensure no incoming bits are dropped
    DMA1_Channel2->CCR |= DMA_CCR_EN; // Enable TX DMA channel SECOND to initiate the physical SPI clock generation
} // End of SPI1_StartTransfer function

static inline uint16_t SPI1_GetAngle() { // Inline getter function for the parsed angle
    return POSITION_ENCODER; // Return the safely cached volatile encoder value
} // End of SPI1_GetAngle function

// ========================================================================= // Visual separator for ISR
// Direct DMA RX IRQ Handler. Replaces HAL_DMA_IRQHandler overhead.          // Description of ISR replacement
// ========================================================================= // Visual separator for ISR

extern "C" void DMA1_Channel1_IRQHandler(void) { // Bare-metal interrupt vector for DMA1 Channel 1
    if (DMA1->ISR & DMA_ISR_TCIF1) { // Check purely for the Transfer Complete (TC) flag for Channel 1
        DMA1->IFCR = DMA_IFCR_CGIF1; // Clear all interrupt flags for Channel 1 immediately
        
        GPIOA->BSRR = GPIO_PIN_4; // Raise CS HIGH atomically. DMA RX completion guarantees SPI shifting is fully done.
        
        // Safely parse angle: cast to uint16_t before shifting to prevent aggressive compiler overflow bugs
        POSITION_ENCODER = ((uint16_t)SPI1_RX_BUF[2] << 9) | ((uint16_t)SPI1_RX_BUF[3] << 1); 
        
        POSITION_Callback(); // Directly invoke the FOC position update logic
    } // End of Transfer Complete check
} // End of DMA1_Channel1_IRQHandler function

// TX and SPI interrupts remain completely eliminated to save vital CPU cycles. // Justification comment

#endif // SPI_SETUP_CLN17V3_H // Final close of the header include guard
