/**
 * @file motion_command.h
 * @brief Data structures describing queued motion segments.
 *
 * Defines MotionCommandTM (one trajectory segment: mode bitfield, duration,
 * displacement, end velocity, torque limit, trigger configuration) plus the
 * supporting enums MotionMode, TorqueTrigger and IO trigger masks. Commands
 * are produced by the application layer and consumed by MotionExecutor.
 */

#pragma once

#include "../../common.h"

/**
 * @struct MotionCommand
 * @brief Базовая математическая структура сегмента траектории.
 */
struct MotionCommand {
    uint32_t duration = 0;     // Длительность сегмента в мкс (0 = бесконечно или мгновенно, зависит от режима)
    int32_t displacement = 0;  // Перемещение (в тиках/шагах)
    float end_velocity = 0.0f; // Конечная скорость сегмента (RPS)
};

/**
 * @struct MotionExecutorOutput
 * @brief Выходная телеметрия генератора траектории на каждый тик часов.
 */
struct MotionExecutorOutput {
    int64_t position_q32; /**< Интерполированная абсолютная позиция Q32.32 */
    float velocity_rps;   /**< Мгновенная целевая скорость (RPS) */
    float torque;         /**< Нормализованный лимит момента (0.0f ... 1.0f) */
};

/**
 * @brief Перечисление базовых режимов движения (2 бита)
 */
enum class MotionMode : uint8_t {
    FREERUN = 0b00,       // Открытый контур: постоянная скорость (displacement - это лимит позиции)
    FREERUN_LERP = 0b01,  // Открытый контур: линейная интерполяция скорости LERP
    ANCHOR = 0b10,        // МГНОВЕННОЕ математическое смещение якоря (выполняется за 0 тактов)
    HERMITE = 0b11        // Закрытый контур: Сплайновая траектория Эрмита (может быть Hold при disp=0)
};

/**
 * @brief Режимы работы триггера по моменту/току (2 бита)
 */
enum class TorqueTrigger : uint8_t {
    DISABLED = 0b00,     // Триггер выключен
    LOWER_THAN = 0b01,   // Срабатывание при падении момента ниже порога
    HIGHER_THAN = 0b10,  // Срабатывание при превышении порога момента
    RESERVED = 0b11      // Резерв
};

/**
 * @brief Упакованный 16-битный регистр режима движения и триггеров.
 * Оптимизирован для атомарной передачи по сети и копирования.
 */
union MotionConfig {
    struct {
        uint16_t mode : 2;      // Бит 0-1: Режим движения (MotionMode)
        uint16_t torque : 2;    // Бит 2-3: Режим триггера момента (TorqueTrigger)
        uint16_t pos_mask : 6;  // Бит 4-9: Маска положительного направления IO
        uint16_t neg_mask : 6;  // Бит 10-15: Маска отрицательного направления IO (инверсная)
    } fields;

    uint16_t raw;

    // Конструктор по умолчанию: neg маска инициализируется как 0x3F (все единицы), 
    // что означает отсутствие триггера по низкому уровню.
    constexpr MotionConfig(MotionMode m = MotionMode::FREERUN,
                           TorqueTrigger t = TorqueTrigger::DISABLED,
                           uint8_t pos = 0,
                           uint8_t neg = 0x3F)
        : fields{
              static_cast<uint16_t>(m), 
              static_cast<uint16_t>(t),
              static_cast<uint16_t>(pos & 0x3F),
              static_cast<uint16_t>(neg & 0x3F)
          } {}

    constexpr explicit MotionConfig(uint16_t raw_val) : raw(raw_val) {}

    inline MotionMode getMode() const { return static_cast<MotionMode>(fields.mode); }
    inline TorqueTrigger getTorqueTrigger() const { return static_cast<TorqueTrigger>(fields.torque); }
    inline uint8_t getPosMask() const { return fields.pos_mask; }
    inline uint8_t getNegMask() const { return fields.neg_mask; }
};

/**
 * @struct MotionCommandTM
 * @brief Полный пакет команды (Телеметрия + Режим), отправляемый в буфер
 */
struct MotionCommandTM {
public:
    MotionCommand motion;
    uint16_t torque = UINT16_MAX; /**< Лимит момента ИЛИ порог для триггера (если триггер активен) */
    MotionConfig mode = MotionMode::ANCHOR; 
};