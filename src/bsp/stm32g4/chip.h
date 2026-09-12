/**
 * @file chip.h
 * @brief Silicon facts that genuinely differ across the STM32G4 family.
 *
 * Almost everything a board needs is a board fact and belongs in that board's
 * target.h. This header is for the small residue that is a property of the PART
 * instead: things a board cannot choose and a descriptor value cannot express,
 * because the name itself only exists on some members of the family.
 *
 * The part is selected by the CMSIS define the board JSON already passes
 * (-DSTM32G431xx / -DSTM32G474xx), so nothing here needs a board name and
 * adding a board on a part already listed costs nothing.
 *
 * Keep this header small. If something here starts looking like a policy choice
 * rather than a fact about the die, it belongs in target.h.
 *
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#pragma once

#include "stm32g4xx.h"  // Family header; the CMSIS define selects the part

namespace chip {

#if defined(STM32G474xx)

/**
 * @brief Vector shared by COMP4, COMP5 and COMP6.
 *
 * The G474 has six comparators and gives the last three one shared vector, so
 * there is no COMP4_IRQn on this part. The G431 has four and gives COMP4 its
 * own. Only the name differs; the drive uses COMP4 identically on both.
 */
constexpr IRQn_Type kComp4Irq = COMP4_5_6_IRQn;

/** @brief True when the part has a third ADC (phase C is sampled there). */
constexpr bool kHasAdc3 = true;

/** @brief Human-readable part name, for boot banners and diagnostics. */
constexpr const char* kPartName = "STM32G474";

#elif defined(STM32G431xx)

/** @brief COMP4 has its own vector on this part. */
constexpr IRQn_Type kComp4Irq = COMP4_IRQn;

/** @brief The G431 has two ADCs; all three phases share them. */
constexpr bool kHasAdc3 = false;

/** @brief Human-readable part name, for boot banners and diagnostics. */
constexpr const char* kPartName = "STM32G431";

#else
#error "bsp/stm32g4/chip.h: unsupported STM32G4 part (define STM32G431xx or STM32G474xx)"
#endif

}  // namespace chip
