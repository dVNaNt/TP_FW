#ifndef SPI_SETUP_CLN17V3_H
#define SPI_SETUP_CLN17V3_H

#include <Arduino.h>
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_spi.h"
#include "stm32g4xx_hal_dma.h"

extern void POSITION_Callback();  // Forward declaration

// MT6835 transaction: 5 bytes write, 5 bytes read
static uint8_t SPI1_TX_BUF[5] = {0b00110000, 0b00000011, 0xFF, 0xFF, 0xFF};
static uint8_t SPI1_RX_BUF[5] = {0};
static volatile uint16_t POSITION_ENCODER = 0;

static SPI_HandleTypeDef hspi1;
static DMA_HandleTypeDef hdma_spi1_tx;
static DMA_HandleTypeDef hdma_spi1_rx;

/**
 * @brief Control SPI1 chip select (PA4)
 * @param select true = CS low (active), false = CS high (idle)
 */
static inline void SPI1_ChipSelect(bool select) {
    select ? HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET)  // CS active low
           : HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);   // CS idle high
}

/**
 * @brief Initialize SPI1 MSP (DMA and interrupts)
 */
static void SPI1_MspInit(void) {
    __HAL_RCC_SPI1_CLK_ENABLE();  // Enable SPI1 clock
    __HAL_RCC_DMA1_CLK_ENABLE();  // Enable DMA1 clock

    // DMA for SPI1 RX (DMA1 Channel 1)
    hdma_spi1_rx.Instance = DMA1_Channel1;
    hdma_spi1_rx.Init.Request = DMA_REQUEST_SPI1_RX;
    hdma_spi1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_spi1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spi1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_spi1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_spi1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_spi1_rx.Init.Mode = DMA_NORMAL;
    hdma_spi1_rx.Init.Priority = DMA_PRIORITY_HIGH;
    HAL_DMA_Init(&hdma_spi1_rx);
    __HAL_LINKDMA(&hspi1, hdmarx, hdma_spi1_rx);

    // DMA for SPI1 TX (DMA1 Channel 2)
    hdma_spi1_tx.Instance = DMA1_Channel2;
    hdma_spi1_tx.Init.Request = DMA_REQUEST_SPI1_TX;
    hdma_spi1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_spi1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spi1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_spi1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_spi1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_spi1_tx.Init.Mode = DMA_NORMAL;
    hdma_spi1_tx.Init.Priority = DMA_PRIORITY_HIGH;
    HAL_DMA_Init(&hdma_spi1_tx);
    __HAL_LINKDMA(&hspi1, hdmatx, hdma_spi1_tx);

    // Enable DMA interrupts
    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
    HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

    // Enable SPI1 interrupt
    HAL_NVIC_SetPriority(SPI1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(SPI1_IRQn);
}

/**
 * @brief Initialize SPI1 peripheral
 */
static inline void SPI1_Init() {
    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;  // Mode 3
    hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;       // Mode 3
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial = 7;
    hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
    hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;

    SPI1_MspInit();  // Initialize MSP
    HAL_SPI_Init(&hspi1);  // Initialize SPI1
}

/**
 * @brief Start SPI1 peripheral
 */
static inline void SPI1_Start() {
    SPI1_ChipSelect(false);  // CS idle high
    __HAL_SPI_ENABLE(&hspi1);  // Enable SPI1
}

/**
 * @brief Start SPI1 DMA transfer
 */
static inline void SPI1_StartTransfer() {
    SPI1_ChipSelect(true);  // CS active low
    HAL_SPI_TransmitReceive_DMA(&hspi1, SPI1_TX_BUF, SPI1_RX_BUF, sizeof(SPI1_TX_BUF));
}

/**
 * @brief Get encoder angle from last transfer
 * @return 16-bit encoder angle
 */
static inline uint16_t SPI1_GetAngle() {
    return POSITION_ENCODER;
}

/**
 * @brief SPI1 transfer complete callback
 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef* hspi) {
    if (hspi->Instance == SPI1) {
        // Extract angle from bytes 2 and 3
        POSITION_ENCODER = ((uint16_t)SPI1_RX_BUF[2] << 8) | SPI1_RX_BUF[3];
        SPI1_ChipSelect(false);  // CS idle high
        POSITION_Callback();  // Call position callback
    }
}

/**
 * @brief SPI1 error callback
 */
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef* hspi) {
    if (hspi->Instance == SPI1) {
        SPI1_ChipSelect(false);  // CS idle high on error
    }
}

// DMA1 Channel 1 IRQ handler (SPI1 RX)
extern "C" void DMA1_Channel1_IRQHandler(void) {
    HAL_DMA_IRQHandler(&hdma_spi1_rx);
}

// DMA1 Channel 2 IRQ handler (SPI1 TX)
extern "C" void DMA1_Channel2_IRQHandler(void) {
    HAL_DMA_IRQHandler(&hdma_spi1_tx);
}

// SPI1 IRQ handler
extern "C" void SPI1_IRQHandler(void) {
    HAL_SPI_IRQHandler(&hspi1);
}

#endif  // SPI_SETUP_CLN17V3_H
