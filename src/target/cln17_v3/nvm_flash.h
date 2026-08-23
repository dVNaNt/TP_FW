/**
 * @file nvm_flash.h
 * @brief Raw internal-flash operations for STM32G4 (FlashG4_LL).
 *
 * Implements erase/read/write primitives matching the FlashEEPROM callback
 * signature: double-word (64-bit) programming rows, word reads and page
 * erases of the dedicated EEPROM-emulation region.
 */

#ifndef FLASH_EEPROM_HW_HPP
#define FLASH_EEPROM_HW_HPP

#include <cstdint>
#include "stm32g431xx.h" 

// Если не используем HAL, задаем размер памяти вручную (128 KB для STM32G431)
#ifndef FLASH_SIZE
#define FLASH_SIZE (128UL * 1024UL)
#endif

// Выносим вычисление адресов в макросы, чтобы избежать проблем с constexpr и reinterpret_cast
#define PAGE_SIZE      2048UL
#define EEPROM_SIZE    4096UL
#define EEPROM_BASE    ((uint32_t)FLASH_BASE + (uint32_t)FLASH_SIZE - EEPROM_SIZE)

/**
 * @class FlashG4_LL
 * @brief Low-level hardware driver for the last 4KB of STM32G4 Flash memory.
 */
class FlashG4_LL {
private:
    // Таймаут для операций с Flash (примерно 10-20 мс при 170 МГц)
    static constexpr uint32_t FLASH_TIMEOUT  = 0xFFFFF; 

    static bool wait_bsy() {
        uint32_t timeout = FLASH_TIMEOUT;
        while ((FLASH->SR & FLASH_SR_BSY) && (--timeout > 0)) {
            __NOP();
        }
        return (timeout > 0);
    }

    static void clear_errors() {
        // Очистка флагов ошибок (запись 1 сбрасывает бит)
        FLASH->SR = (FLASH_SR_PGSERR | FLASH_SR_SIZERR | FLASH_SR_PGAERR | 
                     FLASH_SR_WRPERR | FLASH_SR_PROGERR | FLASH_SR_OPERR);
    }

    static bool check_errors() {
        return (FLASH->SR & (FLASH_SR_PGSERR | FLASH_SR_SIZERR | FLASH_SR_PGAERR | 
                             FLASH_SR_WRPERR | FLASH_SR_PROGERR | FLASH_SR_OPERR)) != 0;
    }

    static void unlock_flash() {
        if (FLASH->CR & FLASH_CR_LOCK) {
            FLASH->KEYR = 0x45670123U;
            FLASH->KEYR = 0xCDEF89ABU;
        }
    }

    static void lock_flash() {
        FLASH->CR |= FLASH_CR_LOCK;
    }

    static void flush_caches() {
        // Сброс кэшей инструкций и данных (критично после изменения Flash!)
        if (FLASH->ACR & FLASH_ACR_DCEN) {
            FLASH->ACR &= ~FLASH_ACR_DCEN;
            FLASH->ACR |= FLASH_ACR_DCRST;
            FLASH->ACR &= ~FLASH_ACR_DCRST;
            FLASH->ACR |= FLASH_ACR_DCEN;
        }
        if (FLASH->ACR & FLASH_ACR_ICEN) {
            FLASH->ACR &= ~FLASH_ACR_ICEN;
            FLASH->ACR |= FLASH_ACR_ICRST;
            FLASH->ACR &= ~FLASH_ACR_ICRST;
            FLASH->ACR |= FLASH_ACR_ICEN;
        }
    }

public:
    static bool erase() {
        if (FLASH->SR & FLASH_SR_BSY) return false;
        
        unlock_flash();
        clear_errors();
        
        bool success = true;
        uint32_t start_page = (EEPROM_BASE - FLASH_BASE) / PAGE_SIZE;
        
        // Отключаем прерывания на время работы с Flash, чтобы избежать задержек и сбоев
        __disable_irq(); 
        
        for (uint32_t page = start_page; page < (start_page + 2); ++page) {
            if (!wait_bsy()) { success = false; break; }
            
            // Настройка страницы для стирания
            FLASH->CR &= ~FLASH_CR_PNB_Msk;
            FLASH->CR |= (page << FLASH_CR_PNB_Pos) | FLASH_CR_PER;
            
            // Старт
            FLASH->CR |= FLASH_CR_STRT;
            
            if (!wait_bsy() || check_errors()) {
                success = false;
                break;
            }
        }
        
        FLASH->CR &= ~FLASH_CR_PER;
        __enable_irq(); // Возвращаем прерывания
        
        clear_errors();
        lock_flash();
        flush_caches(); // Обязательно сбрасываем кэш
        
        return success;
    }

    static bool write(uint32_t addr, const uint32_t data[8]) {
        // Проверка: границы и выравнивание 8 байт (64 бит)
        if (addr > (EEPROM_SIZE - 32) || (addr % 8 != 0)) {
            return false; 
        }
        if (FLASH->SR & FLASH_SR_BSY) return false;
        
        uint32_t hw_addr = EEPROM_BASE + addr;
        
        unlock_flash();
        clear_errors();
        
        __disable_irq(); // Критическая секция
        FLASH->CR |= FLASH_CR_PG;
        
        bool success = true;
        
        for (uint32_t i = 0; i < 4; ++i) {
            if (!wait_bsy()) { success = false; break; }
            
            // ПРАВИЛЬНАЯ последовательность записи для STM32G4:
            // Строго два последовательных 32-битных присваивания!
            volatile uint32_t* p_reg = reinterpret_cast<volatile uint32_t*>(hw_addr + i * 8);
            
            p_reg[0] = data[i * 2];       // Сначала младшее слово
            __ISB();                      // Барьер инструкций (рекомендуется)
            p_reg[1] = data[i * 2 + 1];   // Затем старшее слово
            
            if (!wait_bsy() || check_errors()) {
                success = false;
                break;
            }
        }
        
        FLASH->CR &= ~FLASH_CR_PG;
        __enable_irq(); // Конец критической секции
        
        clear_errors();
        lock_flash();
        flush_caches(); // Обязательно сбрасываем кэш
        
        return success;
    }

    static bool read(uint32_t addr, int32_t* value) {
        if (addr > (EEPROM_SIZE - 4) || (addr % 4 != 0)) {
            return false; 
        }
        
        uint32_t hw_addr = EEPROM_BASE + addr;
        
        // ВНИМАНИЕ: Если по адресу hw_addr память стерта (или там мусор),
        // чтение вызовет HARDFAULT из-за ошибки ECC. 
        // Ловить это через FLASH->ECCR бесполезно, процессор упадет раньше.
        // Эту функцию можно вызывать ТОЛЬКО если вы уверены, 
        // что данные туда были ранее корректно записаны.
        
        *value = *reinterpret_cast<volatile int32_t*>(hw_addr);
        
        // Флаг одинарной ошибки ECC (исправляется аппаратно на лету)
        if (FLASH->ECCR & FLASH_ECCR_ECCC) {
            FLASH->ECCR = FLASH_ECCR_ECCC; // Сбрасываем флаг (записью 1)
        }
        
        return true;
    }
};

#endif // FLASH_EEPROM_HW_HPP