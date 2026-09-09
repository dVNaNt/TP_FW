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
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#ifndef FLASH_EEPROM_HW_HPP
#define FLASH_EEPROM_HW_HPP

#include <cstdint>
#include "stm32g431xx.h" 

// If HAL is not used, define the flash size manually (128 KB for STM32G431)
#ifndef FLASH_SIZE
#define FLASH_SIZE (128UL * 1024UL)
#endif

// Address computation kept in macros to avoid constexpr/reinterpret_cast issues
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

/**
 * Size of the node-configuration partition in bytes (default: one 2 KB page).
 *
 * Deliberately separate from the user-data partition: the motor-profile engine
 * appends a record per save and ERASES THE WHOLE PARTITION when it runs out of
 * slots. A node address, communication settings and protection limits sharing
 * that partition would be wiped by an unrelated profile save - and a drive
 * that forgets its own bus address after ~60 motor saves is a field failure,
 * not an inconvenience. Its own partition makes that impossible.
 */
#ifndef NVM_CONFIG_SECTION_SIZE
#define NVM_CONFIG_SECTION_SIZE 2048UL
#endif

/** Total EEPROM-emulation region: sum of all declared sections. */
#define NVM_TOTAL_SIZE \
    ((uint32_t)(NVM_CALIB_SECTION_SIZE + NVM_USER_SECTION_SIZE + NVM_CONFIG_SECTION_SIZE))

/** Base address of the whole EEPROM-emulation region (end of flash minus total). */
#define EEPROM_BASE ((uint32_t)FLASH_BASE + (uint32_t)FLASH_SIZE - (uint32_t)NVM_TOTAL_SIZE)

/** Offset of each section inside the EEPROM-emulation region. */
#define NVM_CALIB_SECTION_OFFSET  0UL
#define NVM_USER_SECTION_OFFSET   ((uint32_t)NVM_CALIB_SECTION_SIZE)
#define NVM_CONFIG_SECTION_OFFSET ((uint32_t)(NVM_CALIB_SECTION_SIZE + NVM_USER_SECTION_SIZE))

/**
 * @class FlashG4_LL
 * @brief Low-level hardware driver for the EEPROM-emulation partitions at the
 *        end of the STM32G4 Flash memory (calibration + user data).
 */
class FlashG4_LL {
private:
    // Timeout for flash operations (~10-20 ms at 170 MHz)
    static constexpr uint32_t FLASH_TIMEOUT  = 0xFFFFF; 

    static bool wait_bsy() {
        uint32_t timeout = FLASH_TIMEOUT;
        while ((FLASH->SR & FLASH_SR_BSY) && (--timeout > 0)) {
            __NOP();
        }
        return (timeout > 0);
    }

    static void clear_errors() {
        // Clear error flags (write 1 to reset each bit)
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
        // Invalidate instruction and data caches (critical after any flash modification!)
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

        // Disable interrupts during flash operations to avoid stalls and faults
        __disable_irq();

        for (uint32_t i = 0; i < page_count; ++i) {
            if (!wait_bsy()) { success = false; break; }

            // Select the page to erase
            FLASH->CR &= ~FLASH_CR_PNB_Msk;
            FLASH->CR |= ((first_page + i) << FLASH_CR_PNB_Pos) | FLASH_CR_PER;

            // Start the operation
            FLASH->CR |= FLASH_CR_STRT;

            if (!wait_bsy() || check_errors()) {
                success = false;
                break;
            }
        }

        FLASH->CR &= ~FLASH_CR_PER;
        __enable_irq(); // Re-enable interrupts

        clear_errors();
        lock_flash();
        flush_caches(); // Cache reset is mandatory

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
        // Check: own-section bounds and 8-byte (64-bit) alignment
        if (!in_bounds(addr, 32, limit) || (addr % 8 != 0)) {
            return false;
        }
        if (FLASH->SR & FLASH_SR_BSY) return false;

        uint32_t hw_addr = EEPROM_BASE + rel_base + addr;

        unlock_flash();
        clear_errors();

        __disable_irq(); // Critical section
        FLASH->CR |= FLASH_CR_PG;

        bool success = true;

        for (uint32_t i = 0; i < 4; ++i) {
            if (!wait_bsy()) { success = false; break; }

            // CORRECT write sequence for STM32G4:
            // Strictly two consecutive 32-bit assignments!
            volatile uint32_t* p_reg = reinterpret_cast<volatile uint32_t*>(hw_addr + i * 8);

            p_reg[0] = data[i * 2];       // Low word first
            __ISB();                      // Instruction barrier (recommended)
            p_reg[1] = data[i * 2 + 1];   // Then the high word

            if (!wait_bsy() || check_errors()) {
                success = false;
                break;
            }
        }

        FLASH->CR &= ~FLASH_CR_PG;
        __enable_irq(); // End of critical section

        clear_errors();
        lock_flash();
        flush_caches(); // Cache reset is mandatory

        return success;
    }

    // ========================================================================
    // Private ECC-guard infrastructure.
    //
    // On STM32G4 a read of an ERASED flash word raises a double-bit ECC error
    // which triggers an NMI; the stock NMI handler just stalls the system.
    // The guard below makes such reads recoverable: the custom NMI handler
    // patches the stacked context (skips the faulting LDR, injects the natural
    // 'erased' pattern 0xFFFFFFFF into its destination register) and lets
    // read_section() report a clean failure to the caller instead of crashing.
    // ========================================================================

    static constexpr uint32_t GUARD_ARMED   = 0xECCECCECUL;  ///< Guard-window marker.
    static constexpr uint32_t ERASED_WORD32 = 0xFFFFFFFFUL;  ///< Value of an erased word.

    static volatile uint32_t ecc_guard_state;  ///< Non-zero only inside a guarded load.
    static volatile bool     ecc_fault_flag;   ///< Set by the NMI handler on recovery.

    /**
     * @brief Arms the ECC trap for one speculative read.
     *
     * Any double-bit ECC NMI raised between enter/exit is recovered by the NMI
     * handler instead of stalling the CPU.
     */
    static void ecc_guard_enter() {
        ecc_fault_flag = false;
        ecc_guard_state = GUARD_ARMED;
        __asm volatile("" ::: "memory");  // Compile barrier: arm strictly before the load
        __DSB();
        __ISB();
    }

    /**
     * @brief Disarms the guard and reports whether an ECC error was recovered.
     * @return true if the guarded load faulted with a double-bit ECC error.
     */
    static bool ecc_guard_exit() {
        // DSB guarantees the synchronously-raised NMI has been taken and fully
        // processed BEFORE we sample the flag (ARM recommendation for faults).
        __DSB();
        __ISB();
        ecc_guard_state = 0;
        return ecc_fault_flag;
    }

    /**
     * @brief Single isolated 32-bit load used for speculative reads.
     *
     * Kept noinline so the faulting instruction is ALWAYS the well-known LDR
     * emitted inside this function body - this lets the NMI handler decode and
     * patch the stacked context deterministically. Low-register constraints
     * ("l") force r0-r7 so the destination register always lands in the basic
     * exception stack frame (r0-r3, r12) and is therefore restorable.
     *
     * @param addr Absolute address to read.
     * @return Word value; 0xFFFFFFFF if the location was erased/recovered.
     */
    __attribute__((noinline)) static uint32_t guarded_load(volatile const uint32_t* addr) {
        uint32_t v;
        __asm volatile("ldr %0, [%1]" : "=&l"(v) : "l"(addr));
        return v;
    }

    /**
     * @brief Generic read core: reads one 32-bit word at (rel_base + addr),
     *        bounds-checked against the caller's section limit.
     *
     * ERASED-FLASH SAFE: reading never-programmed (erased) words does NOT
     * hardfault anymore. The double-bit ECC NMI is recovered internally,
     * @p value receives 0xFFFFFFFF (-1 as int32_t) and the function returns
     * false, so callers can simply skip/treat the location as empty.
     *
     * @param rel_base Offset of the section inside the EEPROM area.
     * @param limit    Size of the section (upper bound for addr).
     * @param addr     Section-relative address to read.
     * @param value    Output word (0xFFFFFFFF when erased/unreadable).
     * @return true if a programmed word was read, false if erased/ECC-faulted.
     */
    static bool read_section(uint32_t rel_base, uint32_t limit,
                             uint32_t addr, int32_t* value) {
        if (!in_bounds(addr, 4, limit) || (addr % 4 != 0)) {
            return false;
        }

        const volatile uint32_t* hw_addr =
            reinterpret_cast<volatile const uint32_t*>(EEPROM_BASE + rel_base + addr);

        // Speculative read under ECC-guard protection: if the word is erased,
        // the NMI handler returns control with the placeholder value
        // 0xFFFFFFFF and sets the error flag - instead of a HARDFAULT/NMI stall.
        ecc_guard_enter();
        uint32_t raw = guarded_load(hw_addr);
        bool ok = !ecc_guard_exit();

        *value = static_cast<int32_t>(raw);

        // Single-bit ECC flag (corrected by hardware on the fly)
        if (ok && (FLASH->ECCR & FLASH_ECCR_ECCC)) {
            FLASH->ECCR |= FLASH_ECCR_ECCC; // Clear the flag (write 1)
        }

        return ok;
    }

public:
    // ========================================================================
    // Public partition-size getters (compile-time constants from the macros).
    // ========================================================================

    /** Size in bytes of the calibration partition. */
    static constexpr uint32_t calib_section_size() { return NVM_CALIB_SECTION_SIZE; }
    /** Size in bytes of the user-data partition. */
    static constexpr uint32_t user_section_size()  { return NVM_USER_SECTION_SIZE; }
    /** Size in bytes of the node-configuration partition. */
    static constexpr uint32_t config_section_size() { return NVM_CONFIG_SECTION_SIZE; }
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

    // ========================================================================
    // Node-configuration partition callbacks (FlashEEPROM signature-compatible).
    // Addresses are OFFSETS INSIDE the configuration section
    // [0, NVM_CONFIG_SECTION_SIZE). Holds the node address, link settings,
    // protection limits and telemetry configuration written by the CAN
    // protocol layer - never touched by the motor-profile engine.
    // ========================================================================

    /** Erases only the flash pages belonging to the configuration partition. */
    static bool config_erase() {
        return erase_region(NVM_CONFIG_SECTION_OFFSET, NVM_CONFIG_SECTION_SIZE);
    }

    /** Programs one 32-byte row inside the configuration partition only. */
    static bool config_write(uint32_t addr, const uint32_t data[8]) {
        return write_section(NVM_CONFIG_SECTION_OFFSET, NVM_CONFIG_SECTION_SIZE, addr, data);
    }

    /** Reads one 32-bit word inside the configuration partition only. */
    static bool config_read(uint32_t addr, int32_t* value) {
        return read_section(NVM_CONFIG_SECTION_OFFSET, NVM_CONFIG_SECTION_SIZE, addr, value);
    }

    // ========================================================================
    // NMI recovery for flash double-bit ECC errors (erased-word reads).
    // Called from NMI_Handler below with a pointer to the stacked exception
    // frame. Public so it can also be chained from an existing custom NMI
    // handler if one is ever introduced at board level.
    // ========================================================================

    /**
     * @brief Recovers from a double-bit ECC NMI raised inside guarded_load().
     *
     * Behaviour:
     * - If FLASH_ECCR_ECCD is set AND the ECC guard window is armed, the
     *   faulting instruction (the isolated LDR in guarded_load) is decoded
     *   from the stacked PC, its destination register is patched with the
     *   'erased' pattern 0xFFFFFFFF and the stacked PC is advanced past it.
     *   Execution resumes cleanly and read_section() returns false.
     * - Any other NMI source (e.g. clock security system) is NOT claimed:
     *   flags are acknowledged and the CPU stalls exactly like the stock
     *   weak Default_Handler would.
     *
     * @param stacked_ctx Pointer to the stacked basic exception frame
     *                    (r0,r1,r2,r3,r12,lr,pc,xPSR).
     */
    static void nmi_ecc_recover(uint32_t stacked_ctx) {
        uint32_t* frame = reinterpret_cast<uint32_t*>(stacked_ctx);

        const bool ours = (FLASH->ECCR & FLASH_ECCR_ECCD) &&
                          (ecc_guard_state == GUARD_ARMED);

        // Acknowledge both ECC status flags (write-1-to-clear) in any case,
        // so a stale flag can never re-trigger anything later.
        FLASH->ECCR |= (FLASH_ECCR_ECCC | FLASH_ECCR_ECCD);

        if (!ours) {
            // Not our fault (unknown NMI source): behave like Default_Handler.
            while (true) {
                __NOP();
            }
        }

        // Decode the faulting instruction at the stacked return address.
        const uint32_t pc = frame[6];
        const uint16_t op = *(volatile uint16_t*)pc;

        int32_t  rt         = -1;  // Destination register of the aborted load
        uint32_t insn_size  = 2;   // Thumb instruction size

        if ((op & 0xF800u) == 0x6800u || (op & 0xF800u) == 0x4800u) {
            // 16-bit LDR Rt,[Rn,#imm5] / LDR Rt,literal
            rt = op & 0x7u;
        } else {
            const uint16_t op2 = *(volatile uint16_t*)(pc + 2u);
            if ((op & 0xFFF0u) == 0xF8D0u || (op & 0xFFFFu) == 0xF8DFu) {
                // 32-bit LDR.W Rt,[Rn,#imm12] / LDR.W Rt,literal
                rt        = (op2 >> 12) & 0xFu;
                insn_size = 4;
            }
        }

        if (rt < 0) {
            // Unrecognized instruction shape: do NOT touch the context,
            // stall like the default handler rather than corrupting state.
            while (true) {
                __NOP();
            }
        }

        // The aborted load never wrote its destination register: inject the
        // natural 'erased' word. r0-r3 live at frame[0..3], r12 at frame[4].
        // (guarded_load is constrained to low registers, but r12 is handled
        // too for robustness against future changes.)
        if (rt <= 3) {
            frame[rt] = ERASED_WORD32;
        } else if (rt == 12) {
            frame[4] = ERASED_WORD32;
        }
        // Registers r4-r11 are not part of the basic stack frame; the load's
        // destination stays indeterminate there - acceptable, since the value
        // of an aborted load is undefined anyway.

        // Skip the faulting instruction and resume normal execution.
        frame[6] = pc + insn_size;

        ecc_fault_flag = true;
    }
};

// ============================================================================
// ECC-guard storage + NMI vector wiring (single-TU header: tunepulse.h).
// ============================================================================

volatile uint32_t FlashG4_LL::ecc_guard_state = 0;
volatile bool     FlashG4_LL::ecc_fault_flag  = false;

/** C-linkage trampoline called by the naked NMI_Handler with the stack frame. */
extern "C" void flashg4_ll_nmi_dispatch(uint32_t stacked_ctx) {
    FlashG4_LL::nmi_ecc_recover(stacked_ctx);
}

/**
 * Strong override of the weak NMI_Handler from the startup files.
 *
 * Selects the active stack pointer and hands the stacked exception frame to
 * the flash-ECC recovery dispatcher. NOTE: this claims ALL NMIs - currently
 * nothing else in this firmware uses them (no CSS/other NMI users); if that
 * ever changes, chain the other source through here or via nmi_ecc_recover().
 */
extern "C" __attribute__((naked)) void NMI_Handler(void) {
    __asm volatile(
        "tst     lr, #4          \n"
        "ite     eq              \n"
        "mrseq   r0, msp         \n"
        "mrsne   r0, psp         \n"
        "b       flashg4_ll_nmi_dispatch \n");
}

#endif // FLASH_EEPROM_HW_HPP