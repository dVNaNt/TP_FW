/**
 * @file can_setup.h
 * @brief Advanced CAN/CAN-FD driver for STM32G431CB with full configurability
 * @note Features lock-free queues with DMB instructions, pure CMSIS register IO, 
 * Zero-Copy RX, Zero-Allocation TX, and atomic counters.
 * Copyright 2026, creapunk. Licensed under the Apache License, Version 2.0.
 */

#ifndef CAN_SETUP_CLN17V3_H
#define CAN_SETUP_CLN17V3_H

#include <Arduino.h>
#include <atomic>    // Добавлено для потокобезопасных счетчиков
#include <algorithm> // Для std::min
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_fdcan.h"

/**
 * @brief CAN operating modes
 */
enum class CANMode : uint8_t { 
    NORMAL = 0, 
    INTERNAL_LOOPBACK = 1, 
    EXTERNAL_LOOPBACK = 2, 
    SILENT = 3, 
    SILENT_LOOPBACK = 4 
};

/**
 * @brief CAN hardware filter configuration structure
 */
struct CANFilter {
    uint32_t id1;
    uint32_t id2;
    bool is_range;
    uint8_t fifo;
};

/**
 * @brief Calculated CAN bit timings
 */
struct CANTiming {
    uint32_t prescaler;
    uint32_t tseg1;
    uint32_t tseg2;
    uint32_t sjw;
};

/**
 * @brief Array of Structures (AoS) element for RX Ring Buffer
 */
struct CANMessage {
    uint32_t id;
    uint8_t payload_size;
    uint8_t data[64];
};

/**
 * @class CANDriver
 * @brief Main driver class for handling STM32G4 FDCAN peripheral
 */
struct CANDriver {
public:
    FDCAN_HandleTypeDef hfdcan;

private:
    bool is_initialized = false;
    bool is_enabled = true;

    uint32_t baudrate_classic = 1000000;
    uint32_t baudrate_fd = 5000000;
    CANMode mode = CANMode::NORMAL;
    bool auto_retry_enabled = false;
    bool delay_comp_enabled = true;

    CANTiming timing_classic;
    CANTiming timing_fd;

    static constexpr uint8_t MAX_FILTERS = 28;
    CANFilter filters[MAX_FILTERS];
    uint8_t filter_count = 0;

    static constexpr uint8_t RX_BUFFER_SIZE = 16;
    
    // --- ИСПОЛЬЗУЕМ AoS ДЛЯ УЛУЧШЕНИЯ CACHE LOCALITY ---
    CANMessage rx_buffer[RX_BUFFER_SIZE];
    
    volatile uint8_t rx_buffer_head = 0;
    volatile uint8_t rx_buffer_tail = 0;

    // --- ATOMIC STATS (Защита от Data Race между main и ISR) ---
    std::atomic<uint32_t>tx_count{0};
    std::atomic<uint32_t>rx_count{0};
    std::atomic<uint32_t>error_count{0};
    std::atomic<uint32_t>rx_overrun_count{0};

    // Шаблон для сверхбыстрой отправки (избавляемся от лишних аллокаций в send)
    FDCAN_TxHeaderTypeDef tx_header_template;

    // --- ЛЮТЫ (Look-Up Tables) ДЛЯ УЛЬТРАБЫСТРОЙ КОНВЕРТАЦИИ ---
    static constexpr uint8_t dlc_to_bytes_lut[16] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};

    CANTiming calculate_timing(uint32_t baudrate, uint32_t clock_freq, bool is_data_phase) const {
        CANTiming timing = {1, 12, 4, 4};
        const uint32_t min_tq = 4;
        const uint32_t max_tq = is_data_phase ? 49 : 385;

        for (uint32_t prescaler = 1; prescaler <= 512; prescaler++) {
            uint32_t tq_clock = clock_freq / prescaler;
            uint32_t total_tq = tq_clock / baudrate;

            if (total_tq < min_tq || total_tq > max_tq)
                continue;

            uint32_t actual_baudrate = tq_clock / total_tq;
            uint32_t error = (actual_baudrate > baudrate) ? (actual_baudrate - baudrate) : (baudrate - actual_baudrate);

            if ((error * 200) < baudrate) {
                uint32_t tseg1 = (total_tq * 3) / 4;
                uint32_t tseg2 = total_tq - 1 - tseg1;

                if (tseg1 < 1) tseg1 = 1;
                if (tseg1 > 255) tseg1 = 255;
                if (tseg2 < 1) tseg2 = 1;
                if (tseg2 > 127) tseg2 = 127;

                timing.prescaler = prescaler;
                timing.tseg1 = tseg1;
                timing.tseg2 = tseg2;
                timing.sjw = (tseg2 < 4) ? tseg2 : 4;

                return timing;
            }
        }
        return timing;
    }

    inline uint32_t bytes_to_dlc(uint8_t bytes) const {
        if (bytes <= 8) return bytes;
        if (bytes <= 24) return 8 + ((bytes - 5) >> 2);
        if (bytes <= 32) return 13;
        if (bytes <= 48) return 14;
        return 15;
    }

    void init_gpio() {
        RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_GPIOCEN;

        // PB9: FDCAN1_TX -> Alternate Function 9, Push-Pull, High Speed
        GPIOB->MODER = (GPIOB->MODER & ~GPIO_MODER_MODE9_Msk) | (2U << GPIO_MODER_MODE9_Pos);
        GPIOB->AFR[1] = (GPIOB->AFR[1] & ~GPIO_AFRH_AFSEL9_Msk) | (9U << GPIO_AFRH_AFSEL9_Pos);
        GPIOB->OSPEEDR |= (3U << GPIO_OSPEEDR_OSPEED9_Pos);
        GPIOB->OTYPER &= ~GPIO_OTYPER_OT9_Msk; 

        // PB8: FDCAN1_RX -> Alternate Function 9, Pull-Up
        GPIOB->MODER = (GPIOB->MODER & ~GPIO_MODER_MODE8_Msk) | (2U << GPIO_MODER_MODE8_Pos);
        GPIOB->AFR[1] = (GPIOB->AFR[1] & ~GPIO_AFRH_AFSEL8_Msk) | (9U << GPIO_AFRH_AFSEL8_Pos);
        GPIOB->PUPDR = (GPIOB->PUPDR & ~GPIO_PUPDR_PUPD8_Msk) | (1U << GPIO_PUPDR_PUPD8_Pos);

        // PC14: Enable Pin -> Output, Push-Pull
        GPIOC->MODER = (GPIOC->MODER & ~GPIO_MODER_MODE14_Msk) | (1U << GPIO_MODER_MODE14_Pos);
        GPIOC->OTYPER &= ~GPIO_OTYPER_OT14_Msk; 
        
        // Оптимизация: использование CMSIS макросов для ясности кода
        if (is_enabled)
            GPIOC->BSRR = GPIO_BSRR_BR14; // Reset PC14
        else
            GPIOC->BSRR = GPIO_BSRR_BS14; // Set PC14
    }

    void configure_fdcan_clock() {
        RCC->CCIPR &= ~RCC_CCIPR_FDCANSEL_Msk;
        RCC->CCIPR |= (2U << RCC_CCIPR_FDCANSEL_Pos);
    }

public:
    CANDriver() {
        hfdcan.Instance = FDCAN1;

        // Преднастройка статических полей шаблона TX для оптимизации send()
        tx_header_template.IdType = FDCAN_STANDARD_ID;
        tx_header_template.TxFrameType = FDCAN_DATA_FRAME;
        tx_header_template.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
        tx_header_template.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
        tx_header_template.MessageMarker = 0;
    }

    void set_baudrate_classic(uint32_t baudrate) { baudrate_classic = baudrate; }
    void set_baudrate_fd(uint32_t baudrate) { baudrate_fd = baudrate; }
    void set_mode(CANMode new_mode) { mode = new_mode; }
    void enable_auto_retry(bool enable) { auto_retry_enabled = enable; }
    void enable_delay_compensation(bool enable) { delay_comp_enabled = enable; }

    bool add_filter(uint32_t id1, uint32_t id2, bool is_range, uint8_t fifo = 0) {
        if (filter_count >= MAX_FILTERS) return false;
        filters[filter_count].id1 = id1 & 0x7FF;
        filters[filter_count].id2 = id2 & 0x7FF;
        filters[filter_count].is_range = is_range;
        filters[filter_count].fifo = fifo;
        filter_count++;
        return true;
    }

    void clear_filters() { filter_count = 0; }

    void enable() {
        is_enabled = true;
        GPIOC->BSRR = GPIO_BSRR_BR14; // Reset PC14
    }

    void shutdown() {
        is_enabled = false;
        GPIOC->BSRR = GPIO_BSRR_BS14; // Set PC14
    }

    /**
     * @brief Safely shuts down the FDCAN peripheral and disables its interrupts.
     */
    void deinit() {
        if (!is_initialized) return;
        
        NVIC_DisableIRQ(FDCAN1_IT0_IRQn);
        HAL_FDCAN_Stop(&hfdcan);
        HAL_FDCAN_DeInit(&hfdcan);
        
        is_initialized = false;
    }

    /**
     * @brief Initializes or Re-initializes FDCAN. Includes default parameter injection.
     */
    bool init() {
        if (is_initialized) {
            deinit(); // Перезапуск периферии, если она уже была активна
        }

        init_gpio();
        
        // Enable FDCAN peripheral clock via direct register
        RCC->APB1ENR1 |= RCC_APB1ENR1_FDCANEN;
        configure_fdcan_clock();

        uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
        timing_classic = calculate_timing(baudrate_classic, pclk1, false);
        timing_fd = calculate_timing(baudrate_fd, pclk1, true);

        hfdcan.Instance = FDCAN1;
        hfdcan.Init.ClockDivider = FDCAN_CLOCK_DIV1;
        hfdcan.Init.FrameFormat = FDCAN_FRAME_FD_BRS;

        switch (mode) {
            case CANMode::NORMAL: hfdcan.Init.Mode = FDCAN_MODE_NORMAL; break;
            case CANMode::INTERNAL_LOOPBACK: hfdcan.Init.Mode = FDCAN_MODE_INTERNAL_LOOPBACK; break;
            case CANMode::EXTERNAL_LOOPBACK: hfdcan.Init.Mode = FDCAN_MODE_EXTERNAL_LOOPBACK; break;
            case CANMode::SILENT: hfdcan.Init.Mode = FDCAN_MODE_BUS_MONITORING; break;
            case CANMode::SILENT_LOOPBACK: hfdcan.Init.Mode = FDCAN_MODE_INTERNAL_LOOPBACK; break;
        }

        hfdcan.Init.AutoRetransmission = auto_retry_enabled ? ENABLE : DISABLE;
        hfdcan.Init.TransmitPause = ENABLE;
        hfdcan.Init.ProtocolException = DISABLE;

        hfdcan.Init.NominalPrescaler = timing_classic.prescaler;
        hfdcan.Init.NominalSyncJumpWidth = timing_classic.sjw;
        hfdcan.Init.NominalTimeSeg1 = timing_classic.tseg1;
        hfdcan.Init.NominalTimeSeg2 = timing_classic.tseg2;

        hfdcan.Init.DataPrescaler = timing_fd.prescaler;
        hfdcan.Init.DataSyncJumpWidth = timing_fd.sjw;
        hfdcan.Init.DataTimeSeg1 = timing_fd.tseg1;
        hfdcan.Init.DataTimeSeg2 = timing_fd.tseg2;

        hfdcan.Init.StdFiltersNbr = (filter_count > 0) ? filter_count : 1;
        hfdcan.Init.ExtFiltersNbr = 0;
        hfdcan.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;

        if (HAL_FDCAN_Init(&hfdcan) != HAL_OK) {
            error_count++;
            return false;
        }

        if (filter_count == 0) {
            FDCAN_FilterTypeDef filter = {0};
            filter.IdType = FDCAN_STANDARD_ID;
            filter.FilterIndex = 0;
            filter.FilterType = FDCAN_FILTER_RANGE;
            filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
            filter.FilterID1 = 0x000;
            filter.FilterID2 = 0x7FF;
            if (HAL_FDCAN_ConfigFilter(&hfdcan, &filter) != HAL_OK) {
                error_count++;
                return false;
            }
        } else {
            for (uint8_t i = 0; i < filter_count; i++) {
                FDCAN_FilterTypeDef filter = {0};
                filter.IdType = FDCAN_STANDARD_ID;
                filter.FilterIndex = i;
                filter.FilterType = filters[i].is_range ? FDCAN_FILTER_RANGE : FDCAN_FILTER_MASK;
                filter.FilterConfig = (filters[i].fifo == 0) ? FDCAN_FILTER_TO_RXFIFO0 : FDCAN_FILTER_TO_RXFIFO1;
                filter.FilterID1 = filters[i].id1;
                filter.FilterID2 = filters[i].id2;
                if (HAL_FDCAN_ConfigFilter(&hfdcan, &filter) != HAL_OK) {
                    error_count++;
                    return false;
                }
            }
        }

        if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK) {
            error_count++;
            return false;
        }

        if (HAL_FDCAN_Start(&hfdcan) != HAL_OK) {
            error_count++;
            return false;
        }

        if (HAL_FDCAN_ActivateNotification(&hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
            error_count++;
            return false;
        }

        NVIC_SetPriority(FDCAN1_IT0_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 7, 0));
        NVIC_EnableIRQ(FDCAN1_IT0_IRQn);

        is_initialized = true;
        return true;
    }

    void on_rx_interrupt() {
        if (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan, FDCAN_RX_FIFO0) == 0) return;

        uint8_t next_head = (rx_buffer_head + 1) % RX_BUFFER_SIZE;

        if (next_head == rx_buffer_tail) {
            // КРИТИЧНОЕ ИСПРАВЛЕНИЕ: Обязательно вычитываем сообщение (Flush), 
            // чтобы сбросить флаг прерывания в регистрах FDCAN. Иначе — зависание МК.
            FDCAN_RxHeaderTypeDef dummy_header;
            uint8_t dummy_data[64];
            HAL_FDCAN_GetRxMessage(&hfdcan, FDCAN_RX_FIFO0, &dummy_header, dummy_data);
            rx_overrun_count++;
            return;
        }

        FDCAN_RxHeaderTypeDef rx_header;

        // ZERO-COPY ОПТИМИЗАЦИЯ: читаем данные прямо в наш AoS буфер. 
        // Минус локальный массив на стеке, минус std::memcpy.
        if (HAL_FDCAN_GetRxMessage(&hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_buffer[rx_buffer_head].data) != HAL_OK) {
            error_count++;
            return;
        }

        rx_buffer[rx_buffer_head].id = rx_header.Identifier;
        rx_buffer[rx_buffer_head].payload_size = dlc_to_bytes_lut[rx_header.DataLength & 0x0FU];

        // КРИТИЧНОЕ ИСПРАВЛЕНИЕ: Ждем завершения записи в RAM до обновления индекса
        __DMB(); 

        rx_buffer_head = next_head;
        rx_count++;
    }

    /**
     * @brief Optimized send method. Uses pre-configured tx_header and fast hardware checks.
     */
    bool send(uint32_t id, const uint8_t* payload, uint8_t payload_size, bool use_fd = true) {
        if (!is_initialized || !is_enabled) return false;

        // Прямое чтение флага переполнения FIFO из регистра. Значительно быстрее вызова HAL.
        if ((hfdcan.Instance->TXFQS & FDCAN_TXFQS_TFQF) != 0U) {
            error_count++;
            return false;
        }

        // Оптимизированный clip размера (без if-else лестницы)
        uint8_t max_size = use_fd ? 64 : 8;
        payload_size = std::min(payload_size, max_size);

        // Обновляем только изменяемые поля в нашем заранее подготовленном шаблоне
        tx_header_template.Identifier = id & 0x7FF;
        tx_header_template.DataLength = bytes_to_dlc(payload_size);
        
        if (use_fd) {
            tx_header_template.BitRateSwitch = FDCAN_BRS_ON;
            tx_header_template.FDFormat = FDCAN_FD_CAN;
        } else {
            tx_header_template.BitRateSwitch = FDCAN_BRS_OFF;
            tx_header_template.FDFormat = FDCAN_CLASSIC_CAN;
        }

        // Отправка (копирование в Message RAM)
        if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan, &tx_header_template, (uint8_t*)payload) == HAL_OK) {
            tx_count++;
            return true;
        }

        error_count++;
        return false;
    }

    bool receive(uint32_t& id, uint8_t* payload, uint8_t& payload_size) {
        if (!is_initialized || rx_buffer_head == rx_buffer_tail) return false;

        // Чтение из новой структуры AoS
        id = rx_buffer[rx_buffer_tail].id;
        payload_size = rx_buffer[rx_buffer_tail].payload_size;

        if (payload_size > 0) {
            // Копируем данные вызывающей функции. Тут memcpy уместен.
            for(uint8_t i = 0; i < payload_size; ++i) {
                payload[i] = rx_buffer[rx_buffer_tail].data[i];
            }
        }

        // КРИТИЧНОЕ ИСПРАВЛЕНИЕ: Убеждаемся, что чтение RAM окончено до смещения хвоста
        __DMB(); 

        rx_buffer_tail = (rx_buffer_tail + 1) % RX_BUFFER_SIZE;
        return true;
    }

    uint8_t rx_available() const {
        return (rx_buffer_head >= rx_buffer_tail) ? (rx_buffer_head - rx_buffer_tail)
                                                  : (RX_BUFFER_SIZE - rx_buffer_tail + rx_buffer_head);
    }

    uint32_t get_tx_count() const { return tx_count.load(); }
    uint32_t get_rx_count() const { return rx_count.load(); }
    uint32_t get_error_count() const { return error_count.load(); }
    uint32_t get_rx_overrun_count() const { return rx_overrun_count.load(); }
    bool is_ready() const { return is_initialized; }
};

inline CANDriver CAN_BUS;

/**
 * @brief Default instantiation and initialization procedure for the board.
 */
inline void CAN_Driver_Init() {
    CAN_BUS.set_mode(CANMode::EXTERNAL_LOOPBACK);
    CAN_BUS.set_baudrate_fd(1000000);
    CAN_BUS.init();
    CAN_BUS.enable();
}

/**
 * @brief STM32 HAL weak callback override for FDCAN FIFO0 Reception.
 */
extern "C" void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo0ITs) {
    if (RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) {
        CAN_BUS.on_rx_interrupt();
    }
}

/**
 * @brief Fetches underlying hardware handle for complex HAL operations elsewhere.
 */
extern "C" FDCAN_HandleTypeDef* CAN_Get_Handle() {
    return &CAN_BUS.hfdcan;
}

#endif  // CAN_SETUP_CLN17V3_H