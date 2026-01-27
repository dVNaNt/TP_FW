#ifndef SPI_SETUP_H
#define SPI_SETUP_H

#include <Arduino.h>
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_spi.h"
#include "stm32g4xx_hal_dma.h"

extern void POSITION_Callback();

// MT6835 transaction: 5 bytes write, 5 bytes read
static uint8_t SPI3_TX_BUF[5] = {0b00110000, 0b00000011, 0xFF, 0xFF, 0xFF};
static uint8_t SPI3_RX_BUF[5] = {0};
static volatile uint16_t POSITION_ENCODER = 0;

static SPI_HandleTypeDef hspi3;
static DMA_HandleTypeDef hdma_spi3_tx;
static DMA_HandleTypeDef hdma_spi3_rx;

static inline void SPI3_ChipSelect(bool select) {
    // Active low CS on PB7 (I2C1-SDA pin)
    select ? HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET) : HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
}

static void SPI3_MspInit(void) {
    __HAL_RCC_SPI3_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();

    // DMA for SPI3 RX
    hdma_spi3_rx.Instance = DMA1_Channel3;
    hdma_spi3_rx.Init.Request = DMA_REQUEST_SPI3_RX;
    hdma_spi3_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_spi3_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spi3_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_spi3_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_spi3_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_spi3_rx.Init.Mode = DMA_NORMAL;
    hdma_spi3_rx.Init.Priority = DMA_PRIORITY_HIGH;
    HAL_DMA_Init(&hdma_spi3_rx);
    __HAL_LINKDMA(&hspi3, hdmarx, hdma_spi3_rx);

    // DMA for SPI3 TX
    hdma_spi3_tx.Instance = DMA1_Channel4;
    hdma_spi3_tx.Init.Request = DMA_REQUEST_SPI3_TX;
    hdma_spi3_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_spi3_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spi3_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_spi3_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_spi3_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_spi3_tx.Init.Mode = DMA_NORMAL;
    hdma_spi3_tx.Init.Priority = DMA_PRIORITY_HIGH;
    HAL_DMA_Init(&hdma_spi3_tx);
    __HAL_LINKDMA(&hspi3, hdmatx, hdma_spi3_tx);

    HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
    HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);

    HAL_NVIC_SetPriority(SPI3_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(SPI3_IRQn);
}

static inline void SPI3_Init() {
    hspi3.Instance = SPI3;
    hspi3.Init.Mode = SPI_MODE_MASTER;
    hspi3.Init.Direction = SPI_DIRECTION_2LINES;
    hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi3.Init.CLKPolarity = SPI_POLARITY_HIGH;  // Mode 3
    hspi3.Init.CLKPhase = SPI_PHASE_2EDGE;       // Mode 3
    hspi3.Init.NSS = SPI_NSS_SOFT;
    hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
    hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi3.Init.CRCPolynomial = 7;
    hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
    hspi3.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;

    SPI3_MspInit();
    HAL_SPI_Init(&hspi3);
}

static inline void SPI3_Start() {
    SPI3_ChipSelect(false);  // idle high
    __HAL_SPI_ENABLE(&hspi3);
}

static inline void SPI3_StartTransfer() {
    SPI3_ChipSelect(true);
    HAL_SPI_TransmitReceive_DMA(&hspi3, SPI3_TX_BUF, SPI3_RX_BUF, sizeof(SPI3_TX_BUF));
}

static inline uint16_t SPI3_GetAngle() {
    return POSITION_ENCODER;
}

// Callbacks from HAL
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef* hspi) {
    if (hspi->Instance == SPI3) {
        // buf[2], buf[3] are angle
        POSITION_ENCODER = ((uint16_t)SPI3_RX_BUF[2] << 8) | SPI3_RX_BUF[3];
        SPI3_ChipSelect(false);
        POSITION_Callback();
    }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef* hspi) {
    if (hspi->Instance == SPI3) {
        SPI3_ChipSelect(false);
    }
}

// IRQ handlers
extern "C" void DMA1_Channel3_IRQHandler(void) {
    HAL_DMA_IRQHandler(&hdma_spi3_rx);
}

extern "C" void DMA1_Channel4_IRQHandler(void) {
    HAL_DMA_IRQHandler(&hdma_spi3_tx);
}

extern "C" void SPI3_IRQHandler(void) {
    HAL_SPI_IRQHandler(&hspi3);
}

#endif  // SPI_SETUP_H
