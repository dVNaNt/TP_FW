#pragma once

/**
 * @file tp_chunk_config.h
 * @brief TPCP v1 - read/write register layouts (CMD 0x00 - 0x3F, writes at 0x80 - 0xBF).
 *
 * Every read/write register is one 8-byte frame: byte 0 the CMD (write bit
 * stripped in a reply), bytes 1..7 the fields the specification lists. A
 * write carries the same layout with bit 7 of the CMD set, and is answered
 * with the read form holding the values now in force - so a clamped or
 * rounded write is visible in the very reply that acknowledges it.
 *
 * The structs are packed: several fields straddle the natural alignment the
 * compiler would otherwise pad to, and the wire admits no padding. Every
 * struct is followed by its own static_assert on sizeof, because that
 * assertion is the only thing standing between a careless edit and a silent
 * wire-format break. Keep them adjacent.
 *
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#include "../tp_wire.h"  // CMD map, identifier map and the codecs

/** Packed-struct attribute shared by every wire layout. */
#define TP_PACKED __attribute__((packed))

// ============================================================================
// 0x00 Direct register access
// ============================================================================

/**
 * @brief 0x00 Direct Register Access: read or write one 32-bit word of memory.
 *
 * A 24-bit address cannot reach a 32-bit address space, so the top nibble
 * selects a memory window and the low 20 bits are the byte offset inside it
 * (see TpMemWindow). Reads answer from any mapped window; writes are accepted
 * for RAM and peripheral windows and refused for flash.
 */
struct TP_PACKED TpDirectAccess {
    uint8_t  cmd;       ///< Byte 0: TP_REG_DIRECT (| TP_CMD_WRITE).
    uint8_t  addr[3];   ///< Bytes 1-3: window nibble in bits 23-20, byte offset in bits 19-0.
    uint32_t value;     ///< Bytes 4-7: value read, or the value to write.
};
static_assert(sizeof(TpDirectAccess) == 8, "TpDirectAccess must fill one frame");

/** Memory windows selected by the top nibble of a direct-access address. */
enum TpMemWindow : uint8_t {
    TP_MEM_FLASH   = 0x0u,  ///< 0x08000000: program flash (read only).
    TP_MEM_SRAM    = 0x1u,  ///< 0x20000000: SRAM1 / SRAM2.
    TP_MEM_CCM     = 0x2u,  ///< 0x10000000: CCM SRAM.
    TP_MEM_APB1    = 0x3u,  ///< 0x40000000: APB1 peripherals.
    TP_MEM_APB2    = 0x4u,  ///< 0x40010000: APB2 peripherals.
    TP_MEM_AHB1    = 0x5u,  ///< 0x40020000: AHB1 peripherals.
    TP_MEM_AHB2    = 0x6u,  ///< 0x48000000: AHB2 peripherals (GPIO).
    TP_MEM_AHB2_ADC = 0x7u, ///< 0x50000000: AHB2 peripherals (ADC, DAC, ...).
    TP_MEM_HEART   = 0x8u,  ///< Offset into the TunePulseHeart object (firmware state).
    TP_MEM_CORTEX  = 0xFu   ///< 0xE0000000: Cortex-M system control space.
};

// ============================================================================
// 0x01 Axis / mechanics sub-space
// ============================================================================

/** @brief 0x01/0x00 Axis Control: multi-axis id, direction inversion, position offset. */
struct TP_PACKED TpAxisControl {
    uint8_t  cmd;         ///< Byte 0: TP_REG_AXIS.
    uint8_t  sub;         ///< Byte 1: TP_SUB_AXIS_CONTROL.
    uint8_t  axis_id;     ///< Byte 2: axis id matched by broadcast motion tasks (0 = none).
    uint8_t  flags;       ///< Byte 3: bit0 direction inverted, bits 1-7 reserved.
    int32_t  offset_q16;  ///< Bytes 4-7: position offset [rev], q1.15.16.
};
static_assert(sizeof(TpAxisControl) == 8, "TpAxisControl must fill one frame");

/** @brief 0x01/0x01 LED Control: colour and blink pattern of a status LED. */
struct TP_PACKED TpLedControl {
    uint8_t cmd;      ///< Byte 0: TP_REG_AXIS.
    uint8_t sub;      ///< Byte 1: TP_SUB_LED.
    uint8_t mode;     ///< Byte 2: 0 = steady, 1 = blink with the on/off times, 2 = firmware status.
    uint8_t on_off;   ///< Byte 3: bits 0-3 on time [0.25 s], bits 4-7 off time [0.25 s].
    uint8_t led_id;   ///< Byte 4: LED element (0 = all).
    uint8_t r;        ///< Byte 5: red.
    uint8_t g;        ///< Byte 6: green.
    uint8_t b;        ///< Byte 7: blue.
};
static_assert(sizeof(TpLedControl) == 8, "TpLedControl must fill one frame");

/** @brief 0x01/0x02 Gearbox Control: backlash and output ratio. */
struct TP_PACKED TpGearbox {
    uint8_t  cmd;        ///< Byte 0: TP_REG_AXIS.
    uint8_t  sub;        ///< Byte 1: TP_SUB_GEARBOX.
    uint16_t backlash;   ///< Bytes 2-3: backlash [rev], bfloat16.
    int16_t  ratio_mul;  ///< Bytes 4-5: ratio numerator; the sign flips the output direction.
    uint16_t ratio_div;  ///< Bytes 6-7: ratio denominator.
};
static_assert(sizeof(TpGearbox) == 8, "TpGearbox must fill one frame");

/** @brief 0x01/0x03 IO Control Map: pin assignment of one device function. */
struct TP_PACKED TpIoMap {
    uint8_t  cmd;       ///< Byte 0: TP_REG_AXIS.
    uint8_t  sub;       ///< Byte 1: TP_SUB_IO_MAP.
    uint8_t  function;  ///< Byte 2: TpIoFunction being mapped (selects the record on a read too).
    uint8_t  on_pin;    ///< Byte 3: pin that activates the function (0 = none).
    uint8_t  off_pin;   ///< Byte 4: pin that resets the function (0 = none).
    uint8_t  flags;     ///< Byte 5: bit0 on-pin inverted, bit1 off-pin inverted, bit2 state active.
    uint16_t reserved;  ///< Bytes 6-7: reserved, transmitted as zero.
};
static_assert(sizeof(TpIoMap) == 8, "TpIoMap must fill one frame");

/** @brief 0x01/0x04, 0x01/0x05 Encoder Control: type, interface, direction, resolution. */
struct TP_PACKED TpEncoderConfig {
    uint8_t  cmd;         ///< Byte 0: TP_REG_AXIS.
    uint8_t  sub;         ///< Byte 1: TP_SUB_ENCODER_1 or TP_SUB_ENCODER_2.
    uint8_t  type_iface;  ///< Byte 2: bits 0-3 type (0 absolute, 1 incremental), bits 4-7 interface.
    uint8_t  flags;       ///< Byte 3: bit0 direction (1 = CCW), bit1 index pulse present.
    uint32_t ppr;         ///< Bytes 4-7: resolution [pulses per revolution].
};
static_assert(sizeof(TpEncoderConfig) == 8, "TpEncoderConfig must fill one frame");

/**
 * @brief 0x01/0x06 Quick homing settings.
 *
 * The specification's fine-homing record does not fit a frame (75 bits), so
 * its single trigger bit lives HERE, in bit 6 of the flags word, beside the
 * quick-search flags.
 */
struct TP_PACKED TpHomingQuick {
    uint8_t  cmd;       ///< Byte 0: TP_REG_AXIS.
    uint8_t  sub;       ///< Byte 1: TP_SUB_HOMING_QUICK.
    uint16_t flags;     ///< Bytes 2-3: bits 0-1 mode, bit2 trigger, bit3 direction (1 = CCW), bit4 bidirectional, bit5 validation, bit6 fine-search trigger.
    uint16_t velocity;  ///< Bytes 4-5: first search speed [rev/s], bfloat16.
    uint16_t torque;    ///< Bytes 6-7: stall detection torque threshold, bfloat16.
};
static_assert(sizeof(TpHomingQuick) == 8, "TpHomingQuick must fill one frame");

/** @brief 0x01/0x07 Fine homing settings (its trigger bit is in TpHomingQuick::flags). */
struct TP_PACKED TpHomingFine {
    uint8_t  cmd;       ///< Byte 0: TP_REG_AXIS.
    uint8_t  sub;       ///< Byte 1: TP_SUB_HOMING_FINE.
    uint16_t retract;   ///< Bytes 2-3: retract before the fine search [rev], bfloat16.
    uint16_t velocity;  ///< Bytes 4-5: fine search speed [rev/s], bfloat16.
    uint16_t torque;    ///< Bytes 6-7: stall detection torque threshold, bfloat16.
};
static_assert(sizeof(TpHomingFine) == 8, "TpHomingFine must fill one frame");

/** @brief 0x01/0x08, 0x01/0x09 Reference motion limit: expected offset and reference angle. */
struct TP_PACKED TpRefLimit {
    uint8_t  cmd;        ///< Byte 0: TP_REG_AXIS.
    uint8_t  sub;        ///< Byte 1: TP_SUB_REF_CW or TP_SUB_REF_CCW.
    uint16_t offset;     ///< Bytes 2-3: expected offset from the real boundary [rev], bfloat16 (< 64).
    uint32_t ref_angle;  ///< Bytes 4-7: precise absolute angle reference for the search.
};
static_assert(sizeof(TpRefLimit) == 8, "TpRefLimit must fill one frame");

/** @brief 0x01/0x0A Position Zero: reference angle of the work-field zero. */
struct TP_PACKED TpPosZero {
    uint8_t  cmd;        ///< Byte 0: TP_REG_AXIS.
    uint8_t  sub;        ///< Byte 1: TP_SUB_POS_ZERO.
    uint16_t reserved;   ///< Bytes 2-3: reserved, transmitted as zero.
    uint32_t ref_angle;  ///< Bytes 4-7: precise absolute angle reference for the zero search.
};
static_assert(sizeof(TpPosZero) == 8, "TpPosZero must fill one frame");

/** @brief 0x01/0x0B - 0x01/0x0F 48-bit position offsets and travel limits, q1.15.32. */
struct TP_PACKED TpPosOffset {
    uint8_t cmd;        ///< Byte 0: TP_REG_AXIS.
    uint8_t sub;        ///< Byte 1: TP_SUB_ZERO_OFFSET_CW .. TP_SUB_INTERNAL_OFFSET.
    uint8_t offset[6];  ///< Bytes 2-7: offset [rev], q1.15.32 little-endian.
};
static_assert(sizeof(TpPosOffset) == 8, "TpPosOffset must fill one frame");

// ============================================================================
// 0x02 Telemetry slots
// ============================================================================

/** @brief 0x02/slot Telemetry slot: which register to emit, when, and how often. */
struct TP_PACKED TpTelemetrySlotCfg {
    uint8_t  cmd;        ///< Byte 0: TP_REG_TELEMETRY.
    uint8_t  sub;        ///< Byte 1: slot index (0 .. TP_TELEMETRY_SLOTS-1).
    uint8_t  reg;        ///< Byte 2: read register to emit (0x00 - 0x7F); 0 frees the slot.
    uint8_t  flags;      ///< Byte 3: TpTelemetryFlag bits.
    uint16_t period_ms;  ///< Bytes 4-5: emission period [ms] (0 = on sync only).
    uint16_t reserved;   ///< Bytes 6-7: reserved, transmitted as zero.
};
static_assert(sizeof(TpTelemetrySlotCfg) == 8, "TpTelemetrySlotCfg must fill one frame");

// ============================================================================
// 0x10 - 0x12 Bus and discovery
// ============================================================================

/** @brief 0x10 Bus Settings: frame format, response policy, bit rates, master time. */
struct TP_PACKED TpBusSettings {
    uint8_t  cmd;        ///< Byte 0: TP_REG_BUS.
    uint8_t  type_comm;  ///< Byte 1: bits 0-3 TpBusType, bits 4-7 TpCommMode.
    uint8_t  baud1;      ///< Byte 2: arbitration bit rate, TpBaudCode (0 = keep).
    uint8_t  baud2;      ///< Byte 3: data-phase bit rate, TpBaudCode (0 = keep / unsupported).
    uint32_t sync;       ///< Bytes 4-7: master time [50 us]; a write re-bases the node clock.
};
static_assert(sizeof(TpBusSettings) == 8, "TpBusSettings must fill one frame");

/** @brief 0x11 Discovery Assignment: address plus HashUID48. */
struct TP_PACKED TpDiscoveryAssign {
    uint8_t cmd;                 ///< Byte 0: TP_REG_DISCOVERY_ASSIGN.
    uint8_t addr;                ///< Byte 1: node address (255 = unassigned / reset to discovery).
    uint8_t uid[TP_UID_BYTES];   ///< Bytes 2-7: HashUID48, little-endian (0 = wildcard on a reset).
};
static_assert(sizeof(TpDiscoveryAssign) == 8, "TpDiscoveryAssign must fill one frame");

/** @brief 0x12 Discovery Search: prefix depth in bits plus the prefix itself. */
struct TP_PACKED TpDiscoverySearch {
    uint8_t cmd;                    ///< Byte 0: TP_REG_DISCOVERY_SEARCH.
    uint8_t depth;                  ///< Byte 1: number of prefix bits that must match (0 .. 48).
    uint8_t prefix[TP_UID_BYTES];   ///< Bytes 2-7: prefix bits, little-endian, bit 0 first.
};
static_assert(sizeof(TpDiscoverySearch) == 8, "TpDiscoverySearch must fill one frame");

// ============================================================================
// 0x20 - 0x2F System, motor, protection, motion, watchdog, calibration table
// ============================================================================

/**
 * @brief 0x20 System Control: output stage, burst step, calibration requests, actions.
 *
 * On a write every byte is an independent request and zero means "no change".
 * On a read the enable byte reports TP_SYS_ENABLE / TP_SYS_DISABLE, the
 * calibration byte reports which calibrations are VALID and the action byte
 * reports which calibrations are RUNNING.
 */
struct TP_PACKED TpSystemControl {
    uint8_t cmd;          ///< Byte 0: TP_REG_SYSTEM_CONTROL.
    uint8_t enable;       ///< Byte 1: TP_SYS_ENABLE / TP_SYS_DISABLE (0 = no change).
    uint8_t burst_step;   ///< Byte 2: burst-mode period [250 us]; 0 = time-tagged PVT stream.
    uint8_t calibration;  ///< Byte 3: TP_CAL_* bits - request on write, validity on read.
    uint8_t action;       ///< Byte 4: TP_SYS_FAULT_RESET / TP_SYS_REBOOT / TP_SYS_REBOOT_DFU; running calibrations on read.
    uint8_t reserved[3];  ///< Bytes 5-7: reserved, write as zero.
};
static_assert(sizeof(TpSystemControl) == 8, "TpSystemControl must fill one frame");

/** @brief 0x21 Motor Properties 1: control mode, machine identity, ratings. */
struct TP_PACKED TpMotorProps1 {
    uint8_t  cmd;             ///< Byte 0: TP_REG_MOTOR_PROPS_1.
    uint8_t  mode_type;       ///< Byte 1: bits 0-3 TpControlMode, bits 4-7 motor type (0 none, 1 stepper, 2 BLDC, 3 error).
    uint16_t conn_dir_poles;  ///< Bytes 2-3: bits 0-1 phase connection, bits 2-3 direction (2-bit signed), bits 4-15 pole pairs.
    uint16_t max_voltage;     ///< Bytes 4-5: winding voltage rating [V], bfloat16.
    uint16_t max_current;     ///< Bytes 6-7: continuous phase current rating [A], bfloat16 (the control envelope).
};
static_assert(sizeof(TpMotorProps1) == 8, "TpMotorProps1 must fill one frame");

/** @brief 0x22 Motor Properties 2: thermal rating and the electrical constants. */
struct TP_PACKED TpMotorProps2 {
    uint8_t  cmd;         ///< Byte 0: TP_REG_MOTOR_PROPS_2.
    uint8_t  temp;        ///< Byte 1: winding temperature rating [degC].
    uint16_t resistance;  ///< Bytes 2-3: winding resistance [Ohm], bfloat16.
    uint16_t inductance;  ///< Bytes 4-5: effective inductance [H], bfloat16.
    uint16_t kt;          ///< Bytes 6-7: torque constant [N*m/A], bfloat16.
};
static_assert(sizeof(TpMotorProps2) == 8, "TpMotorProps2 must fill one frame");

/** @brief 0x23 Power limits and overdrive: baseline, peak and the energy budget. */
struct TP_PACKED TpPowerLimits {
    uint8_t  cmd;        ///< Byte 0: TP_REG_POWER_LIMITS.
    uint8_t  overdrive;  ///< Byte 1: maximum overpower duration [s]; 0 disables overdrive.
    uint16_t baseline;   ///< Bytes 2-3: power below which the budget is not drained [W], bfloat16.
    uint16_t peak;       ///< Bytes 4-5: maximum allowable power [W], bfloat16 (the overpower trip).
    uint16_t capacity;   ///< Bytes 6-7: energy budget above the baseline [J], bfloat16.
};
static_assert(sizeof(TpPowerLimits) == 8, "TpPowerLimits must fill one frame");

/** @brief 0x24 Motor thermal protection: measured / estimated temperature and the model. */
struct TP_PACKED TpMotorThermal {
    uint8_t  cmd;        ///< Byte 0: TP_REG_MOTOR_THERMAL.
    uint8_t  temp_now;   ///< Byte 1: motor temperature [degC] (read only; 0 = no data).
    uint8_t  temp_base;  ///< Byte 2: full-power threshold [degC].
    uint8_t  temp_limit; ///< Byte 3: maximum winding temperature [degC].
    uint16_t derating;   ///< Bytes 4-5: power derating above the baseline [%/degC], bfloat16.
    uint16_t rth;        ///< Bytes 6-7: thermal resistance [degC/W], bfloat16.
};
static_assert(sizeof(TpMotorThermal) == 8, "TpMotorThermal must fill one frame");

/** @brief 0x25 Driver thermal protection: measured temperature and the trip points. */
struct TP_PACKED TpDriverThermal {
    uint8_t cmd;          ///< Byte 0: TP_REG_DRIVER_THERMAL.
    uint8_t temp_now;     ///< Byte 1: driver temperature [degC] (read only).
    uint8_t ot_warn;      ///< Byte 2: overtemperature warning threshold [degC].
    uint8_t ot_fault;     ///< Byte 3: overtemperature fault threshold [degC].
    uint8_t hysteresis;   ///< Byte 4: release hysteresis [degC].
    uint8_t reserved[3];  ///< Bytes 5-7: reserved, write as zero.
};
static_assert(sizeof(TpDriverThermal) == 8, "TpDriverThermal must fill one frame");

/** @brief 0x26 Supply voltage protection. */
struct TP_PACKED TpSupplyProtection {
    uint8_t  cmd;         ///< Byte 0: TP_REG_SUPPLY_PROTECTION.
    uint16_t hysteresis;  ///< Bytes 1-2: restore hysteresis [V], bfloat16.
    uint8_t  recovery;    ///< Byte 3: recovery time [100 us]; 0 = ignored.
    uint16_t ov;          ///< Bytes 4-5: overvoltage trip [V], bfloat16.
    uint16_t uv;          ///< Bytes 6-7: undervoltage trip [V], bfloat16.
};
static_assert(sizeof(TpSupplyProtection) == 8, "TpSupplyProtection must fill one frame");

/** @brief 0x27 Driver current protection. */
struct TP_PACKED TpCurrentProtection {
    uint8_t  cmd;         ///< Byte 0: TP_REG_CURRENT_PROTECTION.
    uint8_t  disbalance;  ///< Byte 1: allowed phase disbalance, 0..255 = 0..100 % of full current.
    uint8_t  alpha;       ///< Byte 2: disbalance filter; 0 = ignored, 255 = unfiltered.
    uint8_t  recovery;    ///< Byte 3: recovery time [100 us]; 0 = ignored.
    uint16_t peak;        ///< Bytes 4-5: peak motor current [A], bfloat16 (ceiling of the envelope).
    uint16_t oc;          ///< Bytes 6-7: hardware overcurrent trip [A], bfloat16.
};
static_assert(sizeof(TpCurrentProtection) == 8, "TpCurrentProtection must fill one frame");

/** @brief 0x2A / 0x2B Motion limits: velocity, acceleration, jerk envelope. */
struct TP_PACKED TpMotionLimits {
    uint8_t  cmd;          ///< Byte 0: TP_REG_MOTION_LIMITS or TP_REG_ERROR_LIMITS.
    uint8_t  velocity[3];  ///< Bytes 1-3: velocity limit [RPS], float24.
    uint16_t accel;        ///< Bytes 4-5: acceleration limit [RPS^2], bfloat16.
    uint16_t jerk;         ///< Bytes 6-7: jerk limit [RPS^3], bfloat16.
};
static_assert(sizeof(TpMotionLimits) == 8, "TpMotionLimits must fill one frame");

/** @brief 0x2D Watchdog and PulseBeat control. */
struct TP_PACKED TpWatchdog {
    uint8_t  cmd;      ///< Byte 0: TP_REG_WATCHDOG.
    uint8_t  retry;    ///< Byte 1: bit7 retry mode (1 = force retry), bits 0-6 retries before a FaultBeat (0 = none, 127 = ignored).
    uint16_t period;   ///< Bytes 2-3: PulseBeat period [s], bfloat16; 0 = off, NaN = unchanged.
    uint16_t delay;    ///< Bytes 4-5: delay from the bus sync to the first beat [s], bfloat16.
    uint16_t timeout;  ///< Bytes 6-7: master-liveness timeout [s], bfloat16; 0 = off, NaN = unchanged.
};
static_assert(sizeof(TpWatchdog) == 8, "TpWatchdog must fill one frame");

/** @brief 0x2F Calibration table: one sample of one table, or the table size. */
struct TP_PACKED TpCalTableAccess {
    uint8_t  cmd;     ///< Byte 0: TP_REG_CAL_TABLE.
    uint8_t  select;  ///< Byte 1: bits 0-1 mode, bit2 absolute coordinates, bits 3-4 resolution, bits 5-7 TpCalTable.
    uint16_t sample;  ///< Bytes 2-3: sample index; the table size when TP_CAL_TABLE_SIZE_KEY was asked.
    uint32_t data;    ///< Bytes 4-7: sample value, or TP_CAL_TABLE_SIZE_KEY on a read to ask for the size.
};
static_assert(sizeof(TpCalTableAccess) == 8, "TpCalTableAccess must fill one frame");
