/**
 * @file nvm_flash.h
 * @brief Raw internal-flash operations for STM32G4 (FlashG4_LL).
 *
 * Implements erase/read/write primitives matching the FlashEEPROM callback
 * signature: double-word (64-bit) programming rows, word reads and page
 * erases of the dedicated EEPROM-emulation region.
 *
 * The EEPROM-emulation region is split into independent logical PARTITIONS
 * (calibration tables + user data). Each partition exposes its own set of
 * callbacks (calibration_* / user_*); all bounds checking and address
 * translation is centralized in the private section-guard helpers, so a
 * consumer bound to one partition can never access another one.
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

// ============================================================================
// NVM partition layout.
// The last pages of internal flash are split into independent logical
// sections; each section gets its own set of hardware callbacks so a runaway
// access from one consumer can never cross into another section's memory.
//
// Sizes are overridable from the build configuration / board layer
// (tunepulse.h includes this file AFTER its own overrides would be too late,
// therefore pass -DNVM_USER_SECTION_SIZE=... or edit the defaults below).
// Constraints: both sizes must be non-zero multiples of PAGE_SIZE.
// ============================================================================

/** Size of the calibration-table partition in bytes (default: one 2 KB page). */
#ifndef NVM_CALIB_SECTION_SIZE
#define NVM_CALIB_SECTION_SIZE 2048UL
#endif

/** Size of the user-data partition in bytes (default: one 2 KB page). */
#ifndef NVM_USER_SECTION_SIZE
#define NVM_USER_SECTION_SIZE 2048UL
#endif

/** Total EEPROM-emulation region: sum of all declared sections. */
#define NVM_TOTAL_SIZE ((uint32_t)(NVM_CALIB_SECTION_SIZE + NVM_USER_SECTION_SIZE))

/** Base address of the whole EEPROM-emulation region (end of flash minus total). */
#define EEPROM_BASE ((uint32_t)FLASH_BASE + (uint32_t)FLASH_SIZE - (uint32_t)NVM_TOTAL_SIZE)

/** Offset of each section inside the EEPROM-emulation region. */
#define NVM_CALIB_SECTION_OFFSET 0UL
#define NVM_USER_SECTION_OFFSET  ((uint32_t)NVM_CALIB_SECTION_SIZE)

/**
 * @class FlashG4_LL
 * @brief Low-level hardware driver for the EEPROM-emulation partitions at the
 *        end of the STM32G4 Flash memory (calibration + user data).
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

    // ========================================================================
    // Private section-boundary guards.
    // All partition safety lives here: every access is bounds-checked against
    // the size of ITS OWN section, so no consumer can ever reach past the end
    // of its logical region (and thus into a neighbouring section).
    // ========================================================================

    /**
     * @brief Overflow-safe range check: [addr, addr + bytes) must fit in limit.
     * @param addr  Offset to probe.
     * @param bytes Number of bytes the access will touch.
     * @param limit Size of the section being accessed.
     * @return true if the whole access lies inside [0, limit).
     */
    static bool in_bounds(uint32_t addr, uint32_t bytes, uint32_t limit) {
        return (bytes <= limit) && (addr <= (limit - bytes));
    }

    /**
     * @brief Generic erase core: erases every flash page overlapping the region
     *        [rel_offset, rel_offset + size) of the EEPROM-emulation area.
     * @param rel_offset Offset of the region start inside the EEPROM area.
     * @param size       Size of the region to erase (multiple of PAGE_SIZE).
     * @return true on success, false on bounds violation or HW error.
     */
    static bool erase_region(uint32_t rel_offset, uint32_t size) {
        if (!in_bounds(rel_offset, size, NVM_TOTAL_SIZE)) return false;

        unlock_flash();
        clear_errors();

        bool success = true;
        const uint32_t first_page = rel_offset / PAGE_SIZE;  // Page index within the region
        const uint32_t page_count = size / PAGE_SIZE;        // Pages covered by this section

        // Отключаем прерывания на время работы с Flash, чтобы избежать задержек и сбоев
        __disable_irq();

        for (uint32_t i = 0; i < page_count; ++i) {
            if (!wait_bsy()) { success = false; break; }

            // Настройка страницы для стирания
            FLASH->CR &= ~FLASH_CR_PNB_Msk;
            FLASH->CR |= ((first_page + i) << FLASH_CR_PNB_Pos) | FLASH_CR_PER;

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

    /**
     * @brief Generic write core: programs one 32-byte row at (rel_base + addr),
     *        bounds-checked against the caller's section limit.
     * @param rel_base Offset of the section inside the EEPROM area.
     * @param limit    Size of the section (upper bound for addr).
     * @param addr     Section-relative address to program.
     * @param data     8 x uint32_t words to program.
     * @return true on success.
     */
    static bool write_section(uint32_t rel_base, uint32_t limit,
                              uint32_t addr, const uint32_t data[8]) {
        // Проверка: границы СВОЕГО раздела и выравнивание 8 байт (64 бит)
        if (!in_bounds(addr, 32, limit) || (addr % 8 != 0)) {
            return false;
        }
        if (FLASH->SR & FLASH_SR_BSY) return false;

        uint32_t hw_addr = EEPROM_BASE + rel_base + addr;

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

    /**
     * @brief Generic read core: reads one 32-bit word at (rel_base + addr),
     *        bounds-checked against the caller's section limit.
     * @param rel_base Offset of the section inside the EEPROM area.
     * @param limit    Size of the section (upper bound for addr).
     * @param addr     Section-relative address to read.
     * @param value    Output word.
     * @return true on success.
     */
    static bool read_section(uint32_t rel_base, uint32_t limit,
                             uint32_t addr, int32_t* value) {
        if (!in_bounds(addr, 4, limit)) {
            return false;
        }

        uint32_t hw_addr = EEPROM_BASE + rel_base + addr;

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

public:
    // ========================================================================
    // Public partition-size getters (compile-time constants from the macros).
    // ========================================================================

    /** Size in bytes of the calibration partition. */
    static constexpr uint32_t calib_section_size() { return NVM_CALIB_SECTION_SIZE; }
    /** Size in bytes of the user-data partition. */
    static constexpr uint32_t user_section_size()  { return NVM_USER_SECTION_SIZE; }
    /** Total size in bytes of the EEPROM-emulation region. */
    static constexpr uint32_t total_size()         { return NVM_TOTAL_SIZE; }

    // ========================================================================
    // Calibration partition callbacks (FlashEEPROM signature-compatible).
    // Addresses are OFFSETS INSIDE the calibration section
    // [0, NVM_CALIB_SECTION_SIZE).
    // ========================================================================

    /** Erases only the flash pages belonging to the calibration partition. */
    static bool calibration_erase() {
        return erase_region(NVM_CALIB_SECTION_OFFSET, NVM_CALIB_SECTION_SIZE);
    }

    /** Programs one 32-byte row inside the calibration partition only. */
    static bool calibration_write(uint32_t addr, const uint32_t data[8]) {
        return write_section(NVM_CALIB_SECTION_OFFSET, NVM_CALIB_SECTION_SIZE, addr, data);
    }

    /** Reads one 32-bit word inside the calibration partition only. */
    static bool calibration_read(uint32_t addr, int32_t* value) {
        return read_section(NVM_CALIB_SECTION_OFFSET, NVM_CALIB_SECTION_SIZE, addr, value);
    }

    // ========================================================================
    // User-data partition callbacks (FlashEEPROM signature-compatible).
    // Addresses are OFFSETS INSIDE the user-data section
    // [0, NVM_USER_SECTION_SIZE).
    // ========================================================================

    /** Erases only the flash pages belonging to the user-data partition. */
    static bool user_erase() {
        return erase_region(NVM_USER_SECTION_OFFSET, NVM_USER_SECTION_SIZE);
    }

    /** Programs one 32-byte row inside the user-data partition only. */
    static bool user_write(uint32_t addr, const uint32_t data[8]) {
        return write_section(NVM_USER_SECTION_OFFSET, NVM_USER_SECTION_SIZE, addr, data);
    }

    /** Reads one 32-bit word inside the user-data partition only. */
    static bool user_read(uint32_t addr, int32_t* value) {
        return read_section(NVM_USER_SECTION_OFFSET, NVM_USER_SECTION_SIZE, addr, value);
    }
};

#endif // FLASH_EEPROM_HW_HPP