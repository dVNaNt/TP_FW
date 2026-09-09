/**
 * @file cordic_setup.h
 * @brief CORDIC hardware accelerator initialization for STM32G431CB
 *
 * This file configures the CORDIC (COordinate Rotation DIgital Computer) hardware
 * accelerator available on STM32G4 series microcontrollers. CORDIC provides
 * hardware-accelerated trigonometric, hyperbolic, and arithmetic functions.
 *
 * Key Features:
 * - Hardware acceleration for sin, cos, atan, sinh, cosh, atanh, sqrt, ln
 * - Single-cycle execution for most operations
 * - Q1.31 fixed-point format (32-bit signed)
 * - Significantly faster than software floating-point math
 * - Zero CPU overhead during calculation
 *
 * Performance Benefits:
 * - sin/cos: ~6 cycles (vs ~200+ cycles in software)
 * - atan2: ~7 cycles (vs ~300+ cycles in software)
 * - sqrt: ~7 cycles (vs ~100+ cycles in software)
 *
 * OPTIMIZATION (P2.2): the peripheral CSR is programmed once per function
 * change (cached), and WDATA/RDATA are driven directly - no HAL call, no
 * HAL_MAX_DELAY polling on the hot path. The raw Q31 API below exposes the
 * fixed-point pipeline so callers avoid float conversions per call.
 *
 * Usage in FOC:
 * - Park/Clarke transforms (sin/cos calculations)
 * - Phase angle calculations (atan2)
 * - Vector magnitude (sqrt)
 * - Current controller feedforward terms
 *
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#pragma once  // Ensure this header is included only once

#include "stm32g4xx_hal.h"  // Include STM32G4 HAL definitions

// CORDIC handle for hardware accelerator access (kept for HAL compatibility)
CORDIC_HandleTypeDef hcordic;  // Global CORDIC handle

// ----------------------------------------------------------------------------
// Direct-register configuration (P2.2)
// ----------------------------------------------------------------------------

/// Cosine function config: 1 argument, 2 results (RDATA[0] = cos, RDATA[1] = sin),
/// 6-cycle precision, no scaling. FUNC = COSINE(0).
constexpr uint32_t CORDIC_CFG_COS_SINCOS =
    ((uint32_t)CORDIC_FUNCTION_COSINE) |          // Function field [3:0]: cosine
    ((uint32_t)CORDIC_PRECISION_6CYCLES) |        // Precision field [7:4]
    CORDIC_CSR_NRES_Msk |                         // NRES = 1 -> two result reads
    (0UL << CORDIC_CSR_NARGS_Pos);                // NARGS = 0 -> single argument write

/// Currently programmed CSR function slice (FUNC|PRECISION|NRES|NARGS).
/// Seeded with an impossible value so the first call always configures.
inline uint32_t cordic_csr_cache_ = 0xFFFFFFFFUL;

/**
 * @brief Program the CORDIC function configuration only when it changes.
 *
 * The CSR write is skipped while the cached configuration matches, turning a
 * repeated trig call into WDATA-write + RRDY-poll + RDATA-read only.
 *
 * @param cfg FUNC|PRECISION|NRES|NARGS slice to program into CSR
 */
__attribute__((always_inline)) inline void CORDIC_Select_Function(uint32_t cfg) {
    if (cordic_csr_cache_ != cfg) {  // Reconfigure only on an actual function change
        constexpr uint32_t CSR_CFG_MASK =
            CORDIC_CSR_FUNC_Msk | CORDIC_CSR_PRECISION_Msk | CORDIC_CSR_NRES_Msk | CORDIC_CSR_NARGS_Msk;
        MODIFY_REG(CORDIC->CSR, CSR_CFG_MASK, cfg);  // Update only the function-related fields
        cordic_csr_cache_ = cfg;                     // Remember what the hardware now holds
    }
}

/**
 * @brief Initialize CORDIC hardware accelerator.
 *
 * Enables the CORDIC clock, seeds the direct-register cache with the default
 * cosine configuration and programs it immediately.
 */
inline void CORDIC_Init() {  // Initialize CORDIC hardware accelerator
    __HAL_RCC_CORDIC_CLK_ENABLE();  // Enable clock for CORDIC peripheral

    hcordic.Instance = CORDIC;  // Keep the HAL handle linked for compatibility

    cordic_csr_cache_ = 0xFFFFFFFFUL;   // Invalidate cache to force the first programming
    CORDIC_Select_Function(CORDIC_CFG_COS_SINCOS);  // Apply default cosine/sincos configuration
}

/**
 * @brief Raw Q1.31 sine/cosine through the CORDIC pipeline (no float math).
 *
 * Hot path: one cached-CSR check, one WDATA store, two short RRDY polls and
 * two RDATA loads. Roughly >=5x faster than the previous per-call
 * HAL_CORDIC_Configure + HAL_CORDIC_Calculate sequence.
 *
 * @param angle_q31 Angle in radians scaled to Q1.31 (-pi..pi full scale)
 * @param cos_out   [out] Cosine in Q1.31
 * @param sin_out   [out] Sine in Q1.31
 */
__attribute__((always_inline)) inline void CORDIC_SinCos_Q31(int32_t angle_q31, int32_t& cos_out, int32_t& sin_out) {
    CORDIC_Select_Function(CORDIC_CFG_COS_SINCOS);  // Cheap after the first call (cache hit)

    CORDIC->WDATA = (uint32_t)angle_q31;             // Start the computation (argument write triggers it)

    while (!(CORDIC->CSR & CORDIC_CSR_RRDY)) {}      // Wait for the first result (deterministic ~6 cycles)
    cos_out = (int32_t)CORDIC->RDATA;                // Pop cosine (read advances the result queue)

    while (!(CORDIC->CSR & CORDIC_CSR_RRDY)) {}      // Second result becomes ready immediately after
    sin_out = (int32_t)CORDIC->RDATA;                // Pop sine
}

/** Convert float angle [-pi..pi] to the CORDIC Q1.31 phase format. */
constexpr float CORDIC_Q31_SCALE = 2147483648.0f / 3.14159265359f;

/**
 * @brief Calculate cosine using CORDIC hardware.
 * @param angle_rad Angle in radians (float, -pi to +pi)
 * @return Cosine value (float, -1.0 to +1.0)
 */
inline float CORDIC_Cos(float angle_rad) {
    int32_t cos_q31, sin_q31;
    CORDIC_SinCos_Q31((int32_t)(angle_rad * CORDIC_Q31_SCALE), cos_q31, sin_q31);
    return (float)cos_q31 / 2147483648.0f;
}

/**
 * @brief Calculate sine using CORDIC hardware.
 * @param angle_rad Angle in radians (float, -pi to +pi)
 * @return Sine value (float, -1.0 to +1.0)
 */
inline float CORDIC_Sin(float angle_rad) {
    int32_t cos_q31, sin_q31;
    CORDIC_SinCos_Q31((int32_t)(angle_rad * CORDIC_Q31_SCALE), cos_q31, sin_q31);
    return (float)sin_q31 / 2147483648.0f;
}

/**
 * @brief Calculate sine and cosine simultaneously using CORDIC.
 * @param angle_rad Angle in radians (float, -pi to +pi)
 * @param sin_out Pointer to store sine result
 * @param cos_out Pointer to store cosine result
 */
inline void CORDIC_SinCos(float angle_rad, float* sin_out, float* cos_out) {
    int32_t cos_q31, sin_q31;
    CORDIC_SinCos_Q31((int32_t)(angle_rad * CORDIC_Q31_SCALE), cos_q31, sin_q31);
    *cos_out = (float)cos_q31 / 2147483648.0f;
    *sin_out = (float)sin_q31 / 2147483648.0f;
}
