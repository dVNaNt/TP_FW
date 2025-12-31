#ifndef BOOTLOADER_TOOLS_H
#define BOOTLOADER_TOOLS_H

// Libraries
#include "Arduino.h"

#if defined(ARDUINO_GENERIC_G431C6UX) || defined(ARDUINO_GENERIC_G431C8UX) || \
    defined(ARDUINO_GENERIC_G431CBUX) || defined(ARDUINO_GENERIC_G441CBUX)
#define BOOTVTAB ((BootVectorTable*)BootAddr[G4])
#define MCU_IRQS 71u  // Page number 28 in the reference manual for STM32G431
//! Add new MCU families here

#else
#error "Unsupported MCU family"
#endif

enum {
  C0,
  F030x8,
  F030xC,
  F03xx,
  F05,
  F07,
  F09,
  F10xx,
  F105,
  F107,
  F10XL,
  F2,
  F3,
  F4,
  F7,
  G0,
  G4,
  H503,
  H563,
  H573,
  H7x,
  H7A,
  H7B,
  L0,
  L1,
  L4,
  L5,
  WBA,
  WBX,
  WL,
  U5
};

////////////////////////////////*
////* Struct and variable definitions *//////
////////////////////////////////*

extern volatile uint32_t BootAddr[];

typedef struct {
  uint32_t* Initial_SP;
  void (*Reset_Handler)(void);
} BootVectorTable;

////////////////////////////////*
////* Function Prototypes *//////
////////////////////////////////*

/**
 * @brief Initiates a jump to the system bootloader.
 *
 * This function prepares the system to enter the bootloader mode, typically for
 * firmware updates. It performs necessary system cleanup and state preparation,
 * ensuring a safe transition to the bootloader. The specific actions taken can
 * include disabling interrupts, resetting peripheral states, and configuring
 * system clocks. It may also involve setting specific flags or registers that
 * the bootloader checks upon startup to determine whether to enter bootloader
 * mode or continue with normal application execution.
 *
 * Note: The actual implementation details and the necessity of certain actions
 * depend on the target microcontroller architecture and the bootloader's
 * requirements. This function may require customization for different hardware
 * setups.
 *
 * Usage:
 *     JumpToBootloader();
 */
void JumpToBootloader(void);

#endif  // BOOTLOADER_TOOLS_H