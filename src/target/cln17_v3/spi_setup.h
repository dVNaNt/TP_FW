#ifndef SMART_ENCODER_SETUP_H
#define SMART_ENCODER_SETUP_H

#include <Arduino.h>
#include "stm32g4xx.h"

extern void POSITION_Callback();

// ==============================================================================
// ПЕРЕМЕННЫЕ И БУФЕРЫ (SPI1 - ВНУТРЕННИЙ TLE5012)
// ==============================================================================
static uint8_t SPI1_TX_BUF[4] __attribute__((aligned(4))) = {0x80, 0x20, 0xFF, 0xFF};
static uint8_t SPI1_RX_BUF[4] __attribute__((aligned(4))) = {0};
static volatile uint16_t POSITION_ENCODER_TLE = 0;

// ==============================================================================
// ПЕРЕМЕННЫЕ И БУФЕРЫ (SPI3 - ВНЕШНИЙ MT6835)
// ==============================================================================
static uint8_t SPI3_TX_BUF[6] __attribute__((aligned(4))) = {0xA0, 0x03, 0x00, 0x00, 0x00, 0x00};
static uint8_t SPI3_RX_BUF[6] __attribute__((aligned(4))) = {0};

static volatile uint32_t POSITION_ENCODER_MT_RAW = 0;
static volatile uint8_t  POSITION_MT_STATUS = 0;
static volatile uint8_t  POSITION_MT_CRC = 0;

// Глобальный флаг текущего энкодера (true = MT6835, false = TLE5012)
static volatile bool ENCODER_USE_EXTERNAL = false;

// ==============================================================================
// НИЗКОУРОВНЕВОЕ УПРАВЛЕНИЕ CS
// ==============================================================================
static inline void SPI1_ChipSelect(bool select) {
    if (select) GPIOA->BRR = GPIO_PIN_4;       // CS LOW (Active)
    else        GPIOA->BSRR = GPIO_PIN_4;      // CS HIGH (Idle)
}

static inline void SPI3_ChipSelect(bool select) {
    if (select) GPIOA->BRR = GPIO_PIN_15;      // CS LOW (Active) PA15
    else        GPIOA->BSRR = GPIO_PIN_15;     // CS HIGH (Idle) PA15
}

// ==============================================================================
// ИНИЦИАЛИЗАЦИЯ SPI1 (TLE5012 - FALLBACK)
// ==============================================================================
static inline void SPI1_Init() {
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMAMUX1EN;

    // Роутинг DMAMUX1: SPI1_RX = 10, SPI1_TX = 11
    DMAMUX1_Channel0->CCR = 10U;
    DMAMUX1_Channel1->CCR = 11U;

    // Настройка DMA Channel 1 (RX)
    DMA1_Channel1->CCR &= ~DMA_CCR_EN;
    DMA1_Channel1->CPAR = (uint32_t)&SPI1->DR;
    DMA1_Channel1->CMAR = (uint32_t)SPI1_RX_BUF;
    DMA1_Channel1->CCR = DMA_CCR_MINC | DMA_CCR_TCIE | (3U << DMA_CCR_PL_Pos); 

    // Настройка DMA Channel 2 (TX)
    DMA1_Channel2->CCR &= ~DMA_CCR_EN;
    DMA1_Channel2->CPAR = (uint32_t)&SPI1->DR;
    DMA1_Channel2->CMAR = (uint32_t)SPI1_TX_BUF;
    DMA1_Channel2->CCR = DMA_CCR_MINC | DMA_CCR_DIR | (2U << DMA_CCR_PL_Pos); 

    // Настройка SPI1
    SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_CPHA | (5U << SPI_CR1_BR_Pos); 
    SPI1->CR2 = SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN | SPI_CR2_FRXTH | (7U << SPI_CR2_DS_Pos); 
    
    // Очистка возможных флагов и мусора
    (void)SPI1->DR;
    (void)SPI1->SR;
    DMA1->IFCR = DMA_IFCR_CGIF1 | DMA_IFCR_CGIF2;
    
    SPI1_ChipSelect(false); // Убеждаемся, что CS поднят
    SPI1->CR1 |= SPI_CR1_SPE;
}

// ==============================================================================
// ИНИЦИАЛИЗАЦИЯ SPI3 (MT6835 - PRIMARY)
// ==============================================================================
static inline void SPI3_GPIO_Init() {
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN;

    // ---- PA15 как GPIO output для CS ----
    GPIOA->MODER &= ~(3U << (15 * 2));
    GPIOA->MODER |=  (1U << (15 * 2));       // Выход
    GPIOA->OTYPER &= ~GPIO_OTYPER_OT15;      // Push-pull
    GPIOA->OSPEEDR |= (3U << (15 * 2));      // Very high speed
    GPIOA->PUPDR &= ~(3U << (15 * 2));       // Без подтяжки
    GPIOA->BSRR = GPIO_PIN_15;               // CS HIGH (неактивен)

    // ---- PB3/PB4/PB5 -> AF6 SPI3 ----
    GPIOB->MODER &= ~((3U << (3 * 2)) | (3U << (4 * 2)) | (3U << (5 * 2)));
    GPIOB->MODER |=  ((2U << (3 * 2)) | (2U << (4 * 2)) | (2U << (5 * 2)));

    GPIOB->OTYPER &= ~(GPIO_OTYPER_OT3 | GPIO_OTYPER_OT4 | GPIO_OTYPER_OT5);
    GPIOB->OSPEEDR |= ((3U << (3 * 2)) | (3U << (4 * 2)) | (3U << (5 * 2)));
    // Важно: отключаем подтяжки для SPI линий в нормальном режиме работы
    GPIOB->PUPDR &= ~((3U << (3 * 2)) | (3U << (4 * 2)) | (3U << (5 * 2)));

    GPIOB->AFR[0] &= ~((0xFU << (3 * 4)) | (0xFU << (4 * 4)) | (0xFU << (5 * 4)));
    GPIOB->AFR[0] |=  ((6U   << (3 * 4)) | (6U   << (4 * 4)) | (6U   << (5 * 4)));
}

static inline void SPI3_Init() {
    SPI3_GPIO_Init();

    RCC->APB1ENR1 |= RCC_APB1ENR1_SPI3EN;
    RCC->AHB1ENR  |= RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMAMUX1EN;

    // Роутинг DMAMUX1: SPI3_RX = 14, SPI3_TX = 15
    DMAMUX1_Channel0->CCR = 14U; 
    DMAMUX1_Channel1->CCR = 15U; 

    DMA1_Channel1->CCR &= ~DMA_CCR_EN;
    DMA1_Channel1->CPAR = (uint32_t)&SPI3->DR;
    DMA1_Channel1->CMAR = (uint32_t)SPI3_RX_BUF;
    DMA1_Channel1->CCR = DMA_CCR_MINC | DMA_CCR_TCIE | (3U << DMA_CCR_PL_Pos);

    DMA1_Channel2->CCR &= ~DMA_CCR_EN;
    DMA1_Channel2->CPAR = (uint32_t)&SPI3->DR;
    DMA1_Channel2->CMAR = (uint32_t)SPI3_TX_BUF;
    DMA1_Channel2->CCR = DMA_CCR_MINC | DMA_CCR_DIR | (2U << DMA_CCR_PL_Pos);

    SPI3->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_CPOL | SPI_CR1_CPHA | (4U << SPI_CR1_BR_Pos);
    SPI3->CR2 = SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN | SPI_CR2_FRXTH | (7U << SPI_CR2_DS_Pos); 

    // Очистка возможных флагов и мусора
    (void)SPI3->DR; 
    (void)SPI3->SR;
    DMA1->IFCR = DMA_IFCR_CGIF1 | DMA_IFCR_CGIF2;

    SPI3_ChipSelect(false); // Поднимаем CS перед включением SPI
    SPI3->CR1 |= SPI_CR1_SPE;
}

// ==============================================================================
// ЗАПУСК DMA ТРАНЗАКЦИЙ (УНИВЕРСАЛЬНЫЙ)
// ==============================================================================
static inline void Encoder_StartTransfer() {
    if (ENCODER_USE_EXTERNAL) {
        SPI3_ChipSelect(true);
        DMA1_Channel1->CCR &= ~DMA_CCR_EN;
        DMA1_Channel2->CCR &= ~DMA_CCR_EN;
        
        DMA1->IFCR = DMA_IFCR_CGIF1 | DMA_IFCR_CGIF2;
        
        DMA1_Channel1->CNDTR = 6;
        DMA1_Channel2->CNDTR = 6;
        
        DMA1_Channel1->CCR |= DMA_CCR_EN;
        DMA1_Channel2->CCR |= DMA_CCR_EN;
    } else {
        SPI1_ChipSelect(true);
        DMA1_Channel1->CCR &= ~DMA_CCR_EN;
        DMA1_Channel2->CCR &= ~DMA_CCR_EN;
        
        DMA1->IFCR = DMA_IFCR_CGIF1 | DMA_IFCR_CGIF2;
        
        DMA1_Channel1->CNDTR = 4;
        DMA1_Channel2->CNDTR = 4;
        
        DMA1_Channel1->CCR |= DMA_CCR_EN;
        DMA1_Channel2->CCR |= DMA_CCR_EN;
    }
}

// ==============================================================================
// УМНАЯ ИНИЦИАЛИЗАЦИЯ И АВТООПРЕДЕЛЕНИЕ
// ==============================================================================
static inline void Encoder_SmartInit() {
    // 1. Ждем 100мс перед опросом внешнего энкодера (на случай долгого старта MT6835)
    delay(100); 

    // 2. Инициализируем SPI3, но держим прерывания выключенными
    SPI3_Init();
    NVIC_DisableIRQ(DMA1_Channel1_IRQn);
    NVIC_ClearPendingIRQ(DMA1_Channel1_IRQn);

    // 3. Отправляем тестовый запрос
    ENCODER_USE_EXTERNAL = true; 
    Encoder_StartTransfer();

    // 4. Ждем завершения DMA (таймаут ~5мс)
    uint32_t start_time = millis();
    while (!(DMA1->ISR & DMA_ISR_TCIF1)) {
        if ((millis() - start_time) > 5) break;
    }

    bool mt_valid = false;
    
    // 5. Проверяем валидность ответа
    if (DMA1->ISR & DMA_ISR_TCIF1) {
        DMA1->IFCR = DMA_IFCR_CGIF1;
        while (SPI3->SR & SPI_SR_BSY) { __NOP(); }
        SPI3_ChipSelect(false);

        // Если MISO висит в воздухе, данные будут либо 0x00, либо 0xFF
        uint32_t rx_sum = SPI3_RX_BUF[2] + SPI3_RX_BUF[3] + SPI3_RX_BUF[4] + SPI3_RX_BUF[5];
        if (rx_sum != 0 && rx_sum != (0xFF * 4)) {
            mt_valid = true; 
        }
    }

    // Очищаем потенциально зависшие флаги DMA и самого SPI после тестовой посылки
    DMA1_Channel1->CCR &= ~DMA_CCR_EN;
    DMA1_Channel2->CCR &= ~DMA_CCR_EN;
    DMA1->IFCR = DMA_IFCR_CGIF1 | DMA_IFCR_CGIF2;

    // 6. Роутинг на основе результатов проверки
    if (mt_valid) {
        // Энкодер найден: парсим первый пакет данных
        POSITION_ENCODER_MT_RAW = ((uint32_t)SPI3_RX_BUF[2] << 13) | ((uint32_t)SPI3_RX_BUF[3] << 5) | ((uint32_t)SPI3_RX_BUF[4] >> 3);
        POSITION_ENCODER_MT_RAW <<= 11; // Форматируем в 32 бита
        POSITION_MT_STATUS = (uint8_t)(SPI3_RX_BUF[4] & 0x07U);
        POSITION_MT_CRC    = SPI3_RX_BUF[5];
        
        // Включаем прерывания для штатной работы
        NVIC_SetPriority(DMA1_Channel1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0));
        NVIC_EnableIRQ(DMA1_Channel1_IRQn);
    } else {
        // Энкодера нет: отключаем SPI3
        ENCODER_USE_EXTERNAL = false;
        
        SPI3->CR1 &= ~SPI_CR1_SPE;
        RCC->APB1ENR1 &= ~RCC_APB1ENR1_SPI3EN;

        // --- ПЕРЕВОД ПИНОВ В Hi-Z (Floating Input) ---
        // Сбрасываем MODER для PB3, PB4, PB5 в 00 (Input mode, отключаем Alternate Function SPI)
        GPIOB->MODER &= ~((3U << (3 * 2)) | (3U << (4 * 2)) | (3U << (5 * 2))); 
        GPIOB->PUPDR &= ~((3U << (3 * 2)) | (3U << (4 * 2)) | (3U << (5 * 2))); 

        // Переводим PA15 (Chip Select) в режим Open Drain и выдаем HIGH (Hi-Z)
        GPIOA->OTYPER |= GPIO_OTYPER_OT15; 
        GPIOA->BSRR = GPIO_PIN_15;         

        // Инициализируем внутренний TLE5012
        SPI1_Init();
        
        NVIC_SetPriority(DMA1_Channel1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0));
        NVIC_EnableIRQ(DMA1_Channel1_IRQn);
    }
}

// ==============================================================================
// УНИВЕРСАЛЬНЫЙ ГЕТТЕР УГЛА (32-bit Left Aligned)
// ==============================================================================
static inline uint32_t Encoder_GetAngle() {
    if (ENCODER_USE_EXTERNAL) {
        return POSITION_ENCODER_MT_RAW;
    } else {
        return ((uint32_t)POSITION_ENCODER_TLE) << 16;
    }
}

static inline uint8_t Encoder_GetMTStatus() {
    return ENCODER_USE_EXTERNAL ? POSITION_MT_STATUS : 0;
}

static inline bool Encoder_IsUsingExternal() {
    return ENCODER_USE_EXTERNAL;
}

// ==============================================================================
// ОБЩИЙ ОБРАБОТЧИК ПРЕРЫВАНИЙ DMA
// ==============================================================================
extern "C" void DMA1_Channel1_IRQHandler(void) {
    if (DMA1->ISR & DMA_ISR_TCIF1) {
        DMA1->IFCR = DMA_IFCR_CGIF1;

        if (ENCODER_USE_EXTERNAL) {
            // Обработка внешнего MT6835
            while (SPI3->SR & SPI_SR_BSY) { __NOP(); } // Ждем физического завершения SPI
            SPI3_ChipSelect(false);

            POSITION_ENCODER_MT_RAW = ((uint32_t)SPI3_RX_BUF[2] << 13) | ((uint32_t)SPI3_RX_BUF[3] << 5) | ((uint32_t)SPI3_RX_BUF[4] >> 3);
            POSITION_ENCODER_MT_RAW <<= 11;
            POSITION_MT_STATUS = (uint8_t)(SPI3_RX_BUF[4] & 0x07U);
            POSITION_MT_CRC    = SPI3_RX_BUF[5];
        } else {
            // Обработка внутреннего TLE5012
            GPIOA->BSRR = GPIO_PIN_4; // Поднимаем CS атомарно (SPI1_ChipSelect(false))

            POSITION_ENCODER_TLE = ((uint16_t)SPI1_RX_BUF[2] << 9) | ((uint16_t)SPI1_RX_BUF[3] << 1); 
        }

        // Вызов алгоритма управления мотором
        POSITION_Callback(); 
    }
}

#endif // SMART_ENCODER_SETUP_H