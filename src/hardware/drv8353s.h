/**
 * @file drv8353s.h
 * @brief Compile-time register map and SPI command builders for the TI
 *        DRV8353S smart gate driver (datasheet SLVSDY6).
 *
 * Zero-RAM design: every constant is constexpr, so nothing is allocated in
 * RAM and only the constants actually referenced by the firmware occupy
 * flash. Field access goes through contiguous masks + __builtin_ctz (a
 * single-cycle RBIT/CLZ pair on Cortex-M4) instead of per-field getter
 * functions, which keeps the code footprint minimal.
 *
 * SPI frame (16 bits, mode 1: CPOL=0, CPHA=1, MSB first):
 *   [15]    R/W   (1 = read, 0 = write)
 *   [14:11] ADDR  (register number, 0x0..0x7)
 *   [10:0]  DATA
 * A read transaction returns the addressed register content during the
 * SAME frame. Reads never have side effects; a WRITE to a fault register
 * clears the latched faults (never done by this firmware).
 *
 * This header is target-agnostic (no MCU dependencies).
 *
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */
#ifndef HARDWARE_DRV8353S_H
#define HARDWARE_DRV8353S_H

#include <stdint.h>

namespace DRV8353S {

// ------------------------------------------------------------------ addresses
/** Register addresses (SLVSDY6 section 8.6). */
enum Reg : uint8_t {
    REG_FAULT1  = 0x0,  ///< Fault status 1: FAULT, VDS_OCP, GDF, UVLO, OTSD, VDS_x
    REG_VGS     = 0x1,  ///< Fault status 2: SA/SB/SC_OC, OTW, CPUV, VGS_x
    REG_CTRL    = 0x2,  ///< Driver control (OCP_ACT, PWM_MODE, 1x options)
    REG_GATE_HS = 0x3,  ///< High-side gate drive (TDRIVE, IDRIVEP_HS, IDRIVEN_HS)
    REG_GATE_LS = 0x4,  ///< Low-side gate drive (CBC, TDRIVE, IDRIVEP_LS, IDRIVEN_LS)
    REG_OCP     = 0x5,  ///< OCP control (TRETRY, DEAD_TIME, OCP_MODE, DEGLITCH, VDS_LVL)
    REG_CSA     = 0x6,  ///< Current-shunt amplifier (CSA_GAIN, CSA_CAL, SEN_LVL)
    REG_CONFIG  = 0x7,  ///< Driver configuration (CAL_MODE must be 1 on DRV8353)
};

/** Power-on register values (observed and stable on the CLN234 board). */
inline constexpr uint16_t RESET[8] = {
    0x000,  // FAULT1 : no faults
    0x000,  // VGS    : no faults
    0x000,  // CTRL   : 6x PWM mode
    0x1FF,  // GATE_HS: TDRIVE=01b, IDRIVEP/N_HS=1111b
    0x3FF,  // GATE_LS: CBC=1, TDRIVE=01b, IDRIVEP/N_LS=1111b
    0x024,  // OCP    : DEGLITCH=10b (4us), VDS_LVL=0100b (0.4V)
    0x001,  // CSA    : gain 5V/V, SEN_LVL=01b (500mV)
    0x000,  // CONFIG : CAL_MODE=0 (host must set 1 before CSA use)
};

// ------------------------------------------------------------------ frames
/** 16-bit READ frame: R/W=1, ADDR, zero data. */
inline constexpr uint16_t CmdRead(uint8_t reg) {
    return (uint16_t)(0x8000U | ((reg & 0x7U) << 11));
}

/** 16-bit WRITE frame: R/W=0, ADDR, DATA. */
inline constexpr uint16_t CmdWrite(uint8_t reg, uint16_t data) {
    return (uint16_t)(((reg & 0x7U) << 11) | (data & 0x7FFU));
}

/** Extract the 11-bit DATA payload from a response frame. */
inline constexpr uint16_t Data(uint16_t frame) {
    return (uint16_t)(frame & 0x7FFU);
}

// ------------------------------------------------------------------ fields
/** Read one contiguous bit-field: (v & mask) shifted to bit 0. */
inline constexpr uint16_t Get(uint16_t v, uint16_t mask) {
    return (uint16_t)((v & mask) >> __builtin_ctz(mask));
}

/** Write one contiguous bit-field, return the full register value. */
inline constexpr uint16_t Put(uint16_t v, uint16_t mask, uint16_t val) {
    return (uint16_t)((v & (uint16_t)~mask) | ((val << __builtin_ctz(mask)) & mask));
}

// ------------------------------------------------------------------ masks
/** Register field masks (contiguous; usable with Get/Put or as bit tests). */
namespace F {
    // 0x00 FAULT1 / 0x01 VGS status bits (single-bit tests)
    constexpr uint16_t FAULT    = 1U << 10;  ///< Fault indicator (any latched fault)
    constexpr uint16_t VDS_OCP  = 1U << 9;   ///< VDS overcurrent
    constexpr uint16_t GDF      = 1U << 8;   ///< Gate-driver fault
    constexpr uint16_t UVLO     = 1U << 7;   ///< VM supply undervoltage
    constexpr uint16_t OTSD     = 1U << 6;   ///< Overtemperature shutdown
    constexpr uint16_t SA_OC    = 1U << 10;  ///< (VGS) sense A overcurrent
    constexpr uint16_t SB_OC    = 1U << 9;   ///< (VGS) sense B overcurrent
    constexpr uint16_t SC_OC    = 1U << 8;   ///< (VGS) sense C overcurrent
    constexpr uint16_t OTW      = 1U << 7;   ///< (VGS) overtemperature warning
    constexpr uint16_t CPUV     = 1U << 6;   ///< (VGS) charge-pump undervoltage
    constexpr uint16_t VGS_ALL  = 0x003FU;   ///< Any half-bridge VGS fault (bits [5:0])

    // 0x02 DRIVER_CTRL
    constexpr uint16_t OCP_ACT      = 1U << 10;  ///< Shut down all half-bridges on fault
    constexpr uint16_t DIS_CPUV     = 1U << 9;   ///< Disable charge-pump UV protection
    constexpr uint16_t DIS_GDF      = 1U << 8;   ///< Disable gate-driver fault protection
    constexpr uint16_t OTW_REP      = 1U << 7;   ///< OTW report-only mode
    constexpr uint16_t PWM_MODE     = 3U << 5;   ///< PWM control mode (VAL::PWM_*)
    constexpr uint16_t PWM_1X_ASYNC = 1U << 4;   ///< 1x PWM asynchronous mode
    constexpr uint16_t PWM_1X_DIR   = 1U << 3;   ///< 1x PWM direction

    // 0x03 GATE_DRIVE_HS
    constexpr uint16_t TDRIVE     = 3U << 8;  ///< HS gate-drive time
    constexpr uint16_t IDRIVEP_HS = 0xFU << 4;  ///< HS peak source current
    constexpr uint16_t IDRIVEN_HS = 0xFU;       ///< HS peak sink current

    // 0x04 GATE_DRIVE_LS
    constexpr uint16_t CBC        = 1U << 10;  ///< LS current-limit chopping
    constexpr uint16_t TDRIVE_LS  = 3U << 8;   ///< LS gate-drive time
    constexpr uint16_t IDRIVEP_LS = 0xFU << 4;  ///< LS peak source current
    constexpr uint16_t IDRIVEN_LS = 0xFU;       ///< LS peak sink current

    // 0x05 OCP_CTRL
    constexpr uint16_t TRETRY    = 1U << 10;  ///< Retry period: 0=4ms, 1=50us
    constexpr uint16_t DEAD_TIME = 3U << 8;   ///< Inserted dead time
    constexpr uint16_t OCP_MODE  = 3U << 6;   ///< VDS OCP mode (VAL::OCP_*)
    constexpr uint16_t DEGLITCH  = 3U << 4;   ///< OCP deglitch time
    constexpr uint16_t VDS_LVL   = 0xFU;      ///< VDS overcurrent threshold

    // 0x06 CSA_CTRL
    constexpr uint16_t CSA_FET  = 1U << 10;  ///< 0=external shunt, 1=integrated FET
    constexpr uint16_t VREF_DIV = 1U << 9;   ///< 1=bidirectional (VREF/2 reference)
    constexpr uint16_t LS_REF   = 1U << 8;   ///< LS OCP reference point
    constexpr uint16_t CSA_GAIN = 3U << 6;   ///< Amplifier gain (VAL::CSA_GAIN_*)
    constexpr uint16_t DIS_SEN  = 1U << 5;   ///< Disable sense overcurrent detection
    constexpr uint16_t CSA_CAL  = 7U << 2;   ///< Per-amplifier calibration trigger
    constexpr uint16_t SEN_LVL  = 3U;        ///< Sense OCP level (250/500/750/1000mV)

    // 0x07 DRIVER_CONFIG
    constexpr uint16_t CAL_MODE = 1U;        ///< Must be 1 on the DRV8353
}

/** Encoded field values (feed into Put() with the matching mask). */
namespace VAL {
    // PWM_MODE (DRIVER_CTRL [6:5])
    constexpr uint16_t PWM_6X = 0U;           ///< 6x PWM (per-phase inputs)
    constexpr uint16_t PWM_3X = 1U;           ///< 3x PWM (three high-side inputs)
    constexpr uint16_t PWM_1X = 2U;           ///< 1x PWM (one PWM + direction)
    constexpr uint16_t PWM_INDEPENDENT = 3U;  ///< Independent half-bridge control

    // OCP_MODE (OCP_CTRL [7:6])
    constexpr uint16_t OCP_LATCHED = 0U;      ///< Latched shutdown on VDS OCP
    constexpr uint16_t OCP_RETRY   = 1U;      ///< Automatic retry
    constexpr uint16_t OCP_REPORT  = 2U;      ///< Report only, no action
    constexpr uint16_t OCP_IGNORE  = 3U;      ///< VDS OCP disabled

    // CSA_GAIN (CSA_CTRL [7:6])
    constexpr uint16_t CSA_GAIN_5V  = 0U;     ///< 5 V/V
    constexpr uint16_t CSA_GAIN_10V = 1U;     ///< 10 V/V
    constexpr uint16_t CSA_GAIN_20V = 2U;     ///< 20 V/V
    constexpr uint16_t CSA_GAIN_40V = 3U;     ///< 40 V/V
}

}  // namespace DRV8353S

#endif  // HARDWARE_DRV8353S_H