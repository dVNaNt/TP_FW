
#include "bootloaderTools.h"

volatile uint32_t BootAddr[] = {
    [C0] = 0x1FFF0000,    [F030x8] = 0x1FFFEC00, [F030xC] = 0x1FFFD800,
    [F03xx] = 0x1FFFEC00, [F05] = 0x1FFFEC00,    [F07] = 0x1FFFC800,
    [F09] = 0x1FFFD800,   [F10xx] = 0x1FFFF000,  [F105] = 0x1FFFB000,
    [F107] = 0x1FFFB000,  [F10XL] = 0x1FFFE000,  [F2] = 0x1FFF0000,
    [F3] = 0x1FFFD800,    [F4] = 0x1FFF0000,     [F7] = 0x1FF00000,
    [G0] = 0x1FFF0000,    [G4] = 0x1FFF0000,     [H503] = 0x0BF87000,
    [H563] = 0x0BF97000,  [H573] = 0x0BF97000,   [H7x] = 0x1FF09800,
    [H7A] = 0x1FF0A800,   [H7B] = 0x1FF0A000,    [L0] = 0x1FF00000,
    [L1] = 0x1FF00000,    [L4] = 0x1FFF0000,     [L5] = 0x0BF90000,
    [WBA] = 0x0BF88000,   [WBX] = 0x1FFF0000,    [WL] = 0x1FFF0000,
    [U5] = 0x0BF90000,
    // Initialize new models here
};

void JumpToBootloader(void) {
  /*
  In addition to the patterns described above, the user can execute bootloader
  by performing a jump to system memory from user code. Before jumping to
  bootloader: • Disable all peripheral clocks • Disable used PLL • Disable
  interrupts • Clear pending interrupts
  */

  //! As we improve the code, we will add more steps to ensure a safe jump to
  //! the bootloader.

  /* Disable all interrupts */
  __disable_irq();

  /* Disable Systick timer */
  SysTick->CTRL = 0;
  SysTick->LOAD = 0;
  SysTick->VAL = 0;

  /* Set the clock to the default state */
  HAL_RCC_DeInit();

  /* Disable peripehral */
  HAL_DeInit();
  // If needed to disable other peripherals, do it here, SPI, I2C, UART, and CAN
  // should be disabled, but we can tested it first.

  /* Clear Interrupt Enable Register & Interrupt Pending Register */
  for (uint8_t i = 0; i < (MCU_IRQS + 31u) / 32; i++) {
    NVIC->ICER[i] = 0xFFFFFFFF;
    NVIC->ICPR[i] = 0xFFFFFFFF;
  }

  // REMAP
  SYSCFG->MEMRMP = 0x01;

  /* Re-enable all interrupts */
  __enable_irq();

  // Set the MSP
  __set_MSP((uint32_t)BOOTVTAB->Initial_SP);

  // Jump to app firmware
  BOOTVTAB->Reset_Handler();
}