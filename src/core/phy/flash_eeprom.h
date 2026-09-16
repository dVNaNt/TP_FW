#ifndef FLASH_EEPROM_HPP
#define FLASH_EEPROM_HPP

/**
 * @file flash_eeprom.h
 * @brief EEPROM-emulation middleware over raw internal-flash callbacks.
 *
 * FlashEEPROM decouples the application (calibration-table persistence) from
 * the MCU-specific flash driver. All hardware access goes through three
 * function-pointer callbacks (erase / read / write) injected by the board
 * layer, so this class contains no register-level code.
 *
 * Addressing model: all addresses passed to write()/read() are OFFSETS within
 * the logical EEPROM region [0, max_size); base-address translation happens
 * inside the hardware callback implementation. The write path operates on
 * 32-byte records (8 x uint32_t words), matching typical STM32 double-word /
 * row programming granularity; reads are word-granular with an optional
 * sequential cursor for table scanning.
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#include <cstdint>

/**
 * @brief Result of a single flash read operation.
 */
struct FlashReadResult {
    bool success;    ///< true if the hardware read succeeded.
    uint32_t addr;   ///< Offset address that was read.
    int32_t value;   ///< Raw 32-bit value returned by the hardware.
};

/**
 * @brief De-energized window flag for flash programming (single-TU global).
 *
 * Flash programming (a 32-byte row write, and especially a page erase)
 * executes with __disable_irq() and stalls the single-bank CPU for several
 * PWM periods (tens of ms for an erase). During such a stall the previously
 * latched voltage vector stays on the motor bridge, so programming is only
 * allowed when that vector commands ZERO current (or the phases are
 * shorted / Hi-Z). TunePulseHeart::tick() updates this flag once per control
 * pass; the NVM write engines (Motor::save_tick, CalibrationTable::dump_tick)
 * poll it and simply retry on the next tick while the window is closed.
 *
 * Without this gate the first write tick of every save executed while an
 * energized 0.5 A vector was still latched: right after flashing (no stored
 * profile) the MotorCalibration-triggered save yanked the rotor at the very
 * start of the angle calibration (derailed Pass0 + electrical noise), and
 * the post-calibration profile save produced the same yank/noise right
 * after the calibration completed.
 */
extern volatile bool tp_flash_programming_window;


/**
 * @class FlashEEPROM
 * @brief Middleware layer between STM32 Hardware API and high-level application software.
 * 
 * Provides a safe and structured interface for reading, writing, and erasing 
 * flash memory configured for EEPROM emulation. Base address shifts are handled 
 * at the hardware level.
 */
class FlashEEPROM {
private:
    uint32_t max_size;              ///< Size of the logical EEPROM region in bytes.
    uint32_t current_read_offset;   ///< Cursor used by the sequential read() overload.

    // Hardware Callback Typedefs
    typedef bool (*EraseHwCb)(void);                                        ///< Erase whole region, no args.
    typedef bool (*ReadHwCb)(uint32_t addr, int32_t* value);                ///< Read one 32-bit word at offset.
    typedef bool (*WriteHwCb)(uint32_t addr, const uint32_t data[8]);       ///< Program one 32-byte record.

    // Hardware Callback Pointers
    EraseHwCb hw_erase_cb;  ///< Board-provided erase implementation (nullptr = not configured).
    ReadHwCb hw_read_cb;    ///< Board-provided read implementation (nullptr = not configured).
    WriteHwCb hw_write_cb;  ///< Board-provided write implementation (nullptr = not configured).

public:
    /**
     * @brief Nested helper enabling chained (fluent) configuration of the
     *        hardware callbacks without dynamic allocation.
     */
    class ZeroOverhead {
    private:
        FlashEEPROM& parent;  ///< Reference to the owning FlashEEPROM instance.

    public:
        ZeroOverhead(FlashEEPROM& p) : parent(p) {}

        /**
         * @brief Sets the hardware erase callback.
         * @param cb Function pointer to the hardware erase implementation.
         * @return ZeroOverhead& Reference to this object for method chaining.
         */
        ZeroOverhead& erase_cb(EraseHwCb cb) {
            parent.hw_erase_cb = cb;
            return *this;
        }

        /**
         * @brief Sets the hardware read callback.
         * @param cb Function pointer to the hardware read implementation.
         * @return ZeroOverhead& Reference to this object for method chaining.
         */
        ZeroOverhead& read_cb(ReadHwCb cb) {
            parent.hw_read_cb = cb;
            return *this;
        }

        /**
         * @brief Sets the hardware write callback.
         * @param cb Function pointer to the hardware write implementation.
         * @return ZeroOverhead& Reference to this object for method chaining.
         */
        ZeroOverhead& write_cb(WriteHwCb cb) {
            parent.hw_write_cb = cb;
            return *this;
        }
    };

    /**
     * @brief Construct a new FlashEEPROM object.
     */
    FlashEEPROM() 
        : max_size(0), current_read_offset(0),
          hw_erase_cb(nullptr), hw_read_cb(nullptr), hw_write_cb(nullptr) {}

    /**
     * @brief Entry point for setting hardware API callbacks using method chaining.
     * @return ZeroOverhead Temporary object for zero-overhead callback configuration.
     */
    ZeroOverhead set_callback() {
        return ZeroOverhead(*this);
    }

    /**
     * @brief Sets the maximum flash size allocated for this task.
     * @param size Maximum size in bytes.
     * @return FlashEEPROM& Reference to this object for method chaining.
     */
    FlashEEPROM& set_max_size(uint32_t size) {
        this->max_size = size;
        return *this;
    }

    /**
     * @brief Gets the maximum allocated flash size.
     * @return uint32_t Maximum size in bytes.
     */
    uint32_t get_max_size() const {
        return this->max_size;
    }

    /**
     * @brief Resets the internal sequential read offset to zero.
     * @return FlashEEPROM& Reference to this object for method chaining.
     */
    FlashEEPROM& reset_offset() {
        this->current_read_offset = 0;
        return *this;
    }

    /**
     * @brief Erases the allocated flash memory. Use 0xDEADC0DE to enable erasing
     * @param key Mandatory security key to prevent accidental erasure.
     * @return true If the erase operation is successful.
     * @return false If the key is invalid or the erase operation fails.
     */
    bool erase(uint32_t key) {
        if (key != 0xDEADC0DE) { // Mandatory key
            return false;
        }
        if (max_size == 0 || !hw_erase_cb) {
            return false;
        }
        return hw_erase_cb();
    }

    /**
     * @brief Writes exactly 32 bytes (8 x 32-bit words) to the specified address.
     * @param addr Offset address to write to.
     * @param data Array of 8 uint32_t words to write.
     * @return true If the write operation is successful.
     * @return false If the address is out of bounds or the write operation fails.
     */
    bool write(uint32_t addr, const uint32_t data[8]) {
        if (addr + 32 > max_size || !hw_write_cb) {
            return false;
        }
        // Address passed as-is; the section offset is added by the hardware layer
        return hw_write_cb(addr, data);
    }

    /**
     * @brief Sequentially reads the next 32-bit (int32) value.
     *        The internal offset increments by 4 bytes on each call.
     * @return FlashReadResult Struct containing success status, absolute address, and value.
     */
    FlashReadResult read() {
        FlashReadResult result = {false, 0, 0};
        
        if (current_read_offset + 4 > max_size || !hw_read_cb) {
            return result;
        }

        result.addr = current_read_offset;
        // Read at the current offset
        result.success = hw_read_cb(current_read_offset, &result.value);
        
        if (result.success) {
            current_read_offset += 4;
        }
        
        return result;
    }

    /**
     * @brief Reads a 32-byte block (8 x 32-bit words) starting at the given address.
     * @param addr Offset address to read from (must be 4-byte aligned).
     * @param data Output array of 8 uint32_t words.
     * @return true If the whole block was read successfully.
     *
     * Implemented as eight sequential hardware reads with a 4-byte address
     * shift, matching the 32-byte granularity of the write path so that a
     * full record transfer costs exactly one call per scheduler tick.
     */
    bool read_block(uint32_t addr, uint32_t data[8]) {
        if (addr + 32 > max_size || !hw_read_cb) {
            return false;
        }
        for (uint32_t i = 0; i < 8; ++i) { // Eight words per block
            int32_t value = 0;             // Hardware callback returns signed 32-bit
            if (!hw_read_cb(addr + i * 4, &value)) { // Address shifts by one word per step
                return false;              // Abort on first failed word
            }
            data[i] = (uint32_t)value;     // Store raw word into the output block
        }
        return true;
    }

    /**
     * @brief Reads a 32-bit (int32) value from a specific address.
     * @param addr Offset address to read from.
     * @return FlashReadResult Struct containing success status, absolute address, and value.
     */
    FlashReadResult read(uint32_t addr) {
        FlashReadResult result = {false, 0, 0};
        
        if (addr + 4 > max_size || !hw_read_cb) {
            return result;
        }

        result.addr = addr;
        // Address passed as-is
        result.success = hw_read_cb(addr, &result.value);
        
        return result;
    }
};

// Single-TU definition (tunepulse.h architecture): the control loop is the
// only writer, the flash engines are the only readers.
volatile bool tp_flash_programming_window = false;

#endif // FLASH_EEPROM_HPP