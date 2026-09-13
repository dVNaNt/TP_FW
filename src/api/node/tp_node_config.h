#pragma once

/**
 * @file tp_node_config.h
 * @brief TPCP v1 - the persistent node configuration record and its build-time defaults.
 *
 * Everything a node remembers across a power cycle: its address, link
 * settings, protection limits, control envelopes, the PulseBeat / watchdog
 * policy, the telemetry slots and the whole 0x01 mechanics group (axis, LED,
 * gearbox, IO map, encoders, homing, reference limits and offsets). One flat
 * POD, so the register handlers read and write it directly and the store
 * serializes it as a byte stream.
 *
 * Several of the mechanics fields describe hardware this firmware does not
 * yet drive (a second encoder, homing switches, a gearbox index). They are
 * still stored and reported so a master can configure a drive once and rely
 * on the values being there when the feature that consumes them lands.
 *
 * The defaults are #ifndef-guarded macros rather than literals so a board
 * environment can override them with -D (see the cln17_v3_bench env in
 * platformio.ini). TP_DEFAULT_NODE_ID expands to TP_NODE_UNASSIGNED, which is
 * why the wire header has to be included above before any of them is used.
 *
 * The layout is versioned (TP_CONFIG_VERSION): TpNodeStore refuses a record
 * whose version it does not recognize rather than reinterpreting old bytes.
 *
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#include "../tp_wire.h"  // TP_NODE_UNASSIGNED, TP_SAFE_*, TP_TELEMETRY_SLOTS, TP_IO_FUNCTIONS

/** Magic word identifying a node-configuration record inside the partition. */
#define TP_CONFIG_MAGIC 0x7C0Fu  // "TunePulse Config", low half of word 0

/** Layout version of the configuration record; bumped on any field change. */
#define TP_CONFIG_VERSION 2u

/** Words in one configuration record (eight 32-byte flash blocks). */
#define TP_CONFIG_WORDS 64u

/** Bytes in one configuration record. */
#define TP_CONFIG_BYTES (TP_CONFIG_WORDS * 4u)

/** Flash blocks (32 bytes each) in one configuration record. */
#define TP_CONFIG_BLOCKS (TP_CONFIG_BYTES / 32u)

/** Key FlashEEPROM demands before it will erase a partition. */
#define TP_FLASH_ERASE_KEY 0xDEADC0DEu

// ============================================================================
// Build-time defaults
// ============================================================================
//
// These are the values a node uses when its configuration partition is empty.
// They exist as build flags because the shipping defaults - no address, CAN-FD
// at 1 Mbit/s / 5 Mbit/s - are correct for a deployed drive but awkward on a
// bench: an unaddressed node answers nothing but discovery, and a classic-CAN
// USB adapter cannot talk to an FD link at all.
//
//   -D TP_DEFAULT_NODE_ID=1        first bring-up without running discovery
//   -D TP_DEFAULT_BUS_TYPE=0       TP_BUS_CLASSIC: classic CAN 2.0A
//   -D TP_DEFAULT_ARB_KBPS=500     match a cheap adapter's bit rate
//
// A STORED configuration always wins over these, so once a node has been saved
// the build flags stop having any effect on it.

/** Node address a node adopts when nothing is stored. */
#ifndef TP_DEFAULT_NODE_ID
#define TP_DEFAULT_NODE_ID TP_NODE_UNASSIGNED
#endif

/** Behaviour flags (TP_CFG_*) a node adopts when nothing is stored. */
#ifndef TP_DEFAULT_LINK_FLAGS
#define TP_DEFAULT_LINK_FLAGS 0u
#endif

/** Bus type (TpBusType) a node adopts when nothing is stored. */
#ifndef TP_DEFAULT_BUS_TYPE
#define TP_DEFAULT_BUS_TYPE TP_BUS_FD
#endif

/** Arbitration bit rate in kbit/s a node adopts when nothing is stored. */
#ifndef TP_DEFAULT_ARB_KBPS
#define TP_DEFAULT_ARB_KBPS 1000u
#endif

/** CAN-FD data-phase bit rate in kbit/s a node adopts when nothing is stored. */
#ifndef TP_DEFAULT_DATA_KBPS
#define TP_DEFAULT_DATA_KBPS 5000u
#endif

// ============================================================================
// Node configuration
// ============================================================================

/** @brief One cyclic telemetry producer: what to send, how often, and when. */
struct TpTelemetrySlot {
    uint8_t  reg;        ///< Read register to emit (0x00 - 0x7F); 0 = slot unused.
    uint8_t  flags;      ///< TpTelemetryFlag bits.
    uint16_t period_ms;  ///< Emission period [ms]; 0 means sync-triggered only.
};

/** @brief Pin assignment of one device function (IO Control Map). */
struct TpIoMapEntry {
    uint8_t on_pin;   ///< Pin that activates the function (0 = none).
    uint8_t off_pin;  ///< Pin that resets the function (0 = none).
    uint8_t flags;    ///< bit0 on-pin inverted, bit1 off-pin inverted, bit2 state active.
};

/** @brief One encoder's declared type, interface and resolution. */
struct TpEncoderCfg {
    uint8_t  type_iface;  ///< bits 0-3 type (0 absolute, 1 incremental), bits 4-7 interface.
    uint8_t  flags;       ///< bit0 direction (1 = CCW), bit1 index pulse present.
    uint32_t ppr;         ///< Resolution [pulses per revolution].
};

/** Flag bits of TpNodeConfig::flags. */
#define TP_CFG_AUTO_ENABLE (1u << 0)  ///< Enable the output stage as soon as the drive is calibrated.
#define TP_CFG_PROMISCUOUS (1u << 1)  ///< Accept every identifier (bus monitoring).

/**
 * @brief Everything the master can change and persist about this node.
 *
 * The struct is the RAM shadow: registers read and write it directly, and
 * tp_apply_config() pushes it into the live subsystems. That indirection
 * exists because several protection limits are write-only in the driver
 * layer (the proxies program a DAC threshold and keep nothing), so the shadow
 * is the only place a configured value can be read back from.
 */
struct TpNodeConfig {
    // ---- Identity and link ----------------------------------------------
    uint8_t  node_id;          ///< Bus address, TP_NODE_UNASSIGNED when none.
    uint8_t  flags;            ///< TP_CFG_* bits.
    uint8_t  safe_state;       ///< TpSafeState used while disabled or stopped.
    uint8_t  watchdog_action;  ///< TpSafeState entered when the watchdog expires.
    uint8_t  bus_type;         ///< TpBusType.
    uint8_t  comm_mode;        ///< TpCommMode: how much the node answers.
    uint16_t arb_kbps;         ///< Arbitration bit rate [kbit/s].
    uint16_t data_kbps;        ///< CAN-FD data-phase bit rate [kbit/s].
    uint8_t  burst_step;       ///< Burst-mode period [250 us]; 0 = time-tagged PVT.

    // ---- PulseBeat and watchdog -----------------------------------------
    float    heartbeat_s;      ///< PulseBeat period [s]; 0 disables it.
    float    heartbeat_delay_s;///< Delay from a bus time sync to the next beat [s].
    float    watchdog_s;       ///< Master-liveness timeout [s]; 0 disables it.
    uint8_t  hb_retry;         ///< Failed beats before a FaultBeat (0 = never, 127 = ignored).
    uint8_t  hb_retry_force;   ///< Retry mode: 0 wait for the next period, 1 retry at once.

    TpTelemetrySlot telemetry[TP_TELEMETRY_SLOTS];  ///< Cyclic telemetry configuration.

    // ---- Protection -----------------------------------------------------
    float    oc_limit_a;       ///< Phase-A hardware overcurrent threshold [A].
    float    oc_limit_b;       ///< Phase-B hardware overcurrent threshold [A].
    float    oc_limit_c;       ///< Phase-C hardware overcurrent threshold [A].
    float    peak_current_a;   ///< Peak motor current [A]: ceiling of the control envelope.
    uint8_t  oc_disbalance;    ///< Allowed phase disbalance, 0..255 = 0..100 %.
    uint8_t  oc_alpha;         ///< Disbalance filter constant (0 = ignored).
    uint8_t  oc_recovery;      ///< Overcurrent recovery time [100 us] (0 = default).
    float    ov_limit_v;       ///< Overvoltage trip [V].
    float    uv_limit_v;       ///< Undervoltage trip [V].
    float    supply_hyst_v;    ///< Supply restore hysteresis [V].
    uint8_t  supply_recovery;  ///< Supply recovery time [100 us] (0 = default).
    float    ot_limit_c;       ///< Driver overtemperature trip [degC].
    float    ot_hyst_c;        ///< Driver overtemperature release hysteresis [degC].
    float    ot_warn_c;        ///< Driver overtemperature warning [degC].
    float    op_limit_w;       ///< Overpower trip (peak power) [W].
    float    op_limit_a;       ///< Bus overcurrent trip [A].
    float    pt_baseline_w;    ///< Power below which the overdrive budget is untouched [W].
    float    pt_capacity_j;    ///< Overdrive energy budget [J].
    uint8_t  pt_overdrive_s;   ///< Maximum overdrive duration [s]; 0 disables overdrive.
    uint8_t  motor_temp_base_c;///< Motor full-power threshold [degC].
    float    motor_derating;   ///< Motor power derating above the baseline [%/degC].
    float    motor_rth;        ///< Motor thermal resistance [degC/W].
    float    current_limit_a;  ///< Control current envelope [A] (motor continuous rating).

    // ---- Motion envelopes ----------------------------------------------
    float    vel_limit_rps;    ///< Velocity envelope [RPS].
    float    accel_limit;      ///< Acceleration envelope [RPS^2].
    float    jerk_limit;       ///< Jerk envelope [RPS^3].
    float    err_vel_rps;      ///< Velocity limit under fault conditions [RPS].
    float    err_accel;        ///< Acceleration limit under fault conditions [RPS^2].
    float    err_jerk;         ///< Jerk limit under fault conditions [RPS^3].

    // ---- Mechanics (0x01 group) ----------------------------------------
    uint8_t  axis_id;          ///< Axis id matched by broadcast motion tasks (0 = none).
    uint8_t  axis_inverted;    ///< 1 = reported / commanded direction inverted.
    int32_t  axis_offset_q16;  ///< Position offset [rev], q1.15.16.
    uint8_t  led_mode;         ///< LED mode: 0 steady, 1 blink, 2 firmware status.
    uint8_t  led_on_off;       ///< bits 0-3 on time [0.25 s], bits 4-7 off time [0.25 s].
    uint8_t  led_id;           ///< LED element (0 = all).
    uint8_t  led_r;            ///< Red.
    uint8_t  led_g;            ///< Green.
    uint8_t  led_b;            ///< Blue.
    float    gear_backlash;    ///< Gearbox backlash [rev].
    int16_t  gear_ratio_mul;   ///< Gear ratio numerator (sign = output direction).
    uint16_t gear_ratio_div;   ///< Gear ratio denominator.
    TpIoMapEntry io_map[TP_IO_FUNCTIONS];  ///< Pin assignment per device function.
    TpEncoderCfg encoder[2];   ///< Declared encoders.
    uint16_t homing_flags;     ///< Quick-homing flags word (see TpHomingQuick).
    float    homing_quick_vel; ///< Coarse search speed [rev/s].
    float    homing_quick_torque;  ///< Coarse stall torque threshold.
    float    homing_fine_retract;  ///< Retract before the fine search [rev].
    float    homing_fine_vel;  ///< Fine search speed [rev/s].
    float    homing_fine_torque;   ///< Fine stall torque threshold.
    float    ref_offset_cw;    ///< Expected offset from the CW boundary [rev].
    float    ref_offset_ccw;   ///< Expected offset from the CCW boundary [rev].
    uint32_t ref_ang_cw;       ///< Reference angle for the CW search.
    uint32_t ref_ang_ccw;      ///< Reference angle for the CCW search.
    uint32_t ref_ang_zero;     ///< Reference angle for the zero search.
    int64_t  zero_offset_cw_q32;   ///< Zero to CW endstop [rev], q1.15.32.
    int64_t  zero_offset_ccw_q32;  ///< Zero to CCW endstop [rev], q1.15.32.
    int64_t  field_cw_q32;         ///< Allowed CW travel from zero [rev], q1.15.32.
    int64_t  field_ccw_q32;        ///< Allowed CCW travel from zero [rev], q1.15.32.
    int64_t  internal_offset_q32;  ///< System zero to encoder internal position [rev], q1.15.32.

    /**
     * @brief Load the compiled-in defaults.
     *
     * The protection values mirror what tunepulse_init() has always
     * programmed, so a board with an erased configuration partition behaves
     * exactly like one built before this layer existed.
     */
    void set_defaults() {
        node_id = TP_DEFAULT_NODE_ID;             // Normally unassigned: discovery hands out an address
        flags = TP_DEFAULT_LINK_FLAGS;            // Manual enable, not promiscuous
        safe_state = TP_SAFE_FREEWHEEL;           // Coast when de-energized
        watchdog_action = TP_SAFE_FREEWHEEL;      // Coast when the master goes quiet
        bus_type = TP_DEFAULT_BUS_TYPE;           // CAN-FD by default
        comm_mode = TP_COMM_ON_REQUEST;           // Answer requests, stream silently
        arb_kbps = TP_DEFAULT_ARB_KBPS;           // 1 Mbit/s arbitration by default
        data_kbps = TP_DEFAULT_DATA_KBPS;         // 5 Mbit/s FD data phase by default
        burst_step = 0u;                          // Time-tagged PVT stream
        heartbeat_s = 0.1f;                       // 10 Hz liveness
        heartbeat_delay_s = 0.0f;                 // No offset from the bus sync
        watchdog_s = 0.0f;                        // Watchdog off until the master arms it
        hb_retry = 0u;                            // No FaultBeat
        hb_retry_force = 0u;                      // Wait for the next period
        for (uint32_t i = 0; i < TP_TELEMETRY_SLOTS; ++i) {
            telemetry[i].reg = 0u;                // Slot unused
            telemetry[i].flags = 0u;              // Disabled
            telemetry[i].period_ms = 0u;          // No timer
        }
        oc_limit_a = 4.0f;                        // Matches the DRV8323H board wiring
        oc_limit_b = 4.0f;                        // Matches the DRV8323H board wiring
        oc_limit_c = 6.2f;                        // Phase C carries the sum current
        peak_current_a = 4.0f;                    // Ceiling of the control envelope
        oc_disbalance = 0u;                       // Disbalance check off
        oc_alpha = 0u;                            // Disbalance filter off
        oc_recovery = 0u;                         // Driver default recovery time
        ov_limit_v = 51.0f;                       // Bus capacitor rating headroom
        uv_limit_v = 16.0f;                       // Gate-driver dropout margin
        supply_hyst_v = 1.0f;                     // Release band around the trips
        supply_recovery = 0u;                     // Driver default recovery time
        ot_limit_c = 85.0f;                       // Silicon derating point
        ot_hyst_c = 10.0f;                        // Release at 75 degC, no chatter
        ot_warn_c = 70.0f;                        // Warn well before the trip
        op_limit_w = 60.0f;                       // Thermal budget of the stage
        op_limit_a = 2.5f;                        // Bus current budget
        pt_baseline_w = 30.0f;                    // Half the peak: nothing drained below it
        pt_capacity_j = 0.0f;                     // No overdrive budget
        pt_overdrive_s = 0u;                      // Overdrive off
        motor_temp_base_c = 60u;                  // Full power up to 60 degC
        motor_derating = 0.0f;                    // No derating model
        motor_rth = 0.0f;                         // No thermal model: temperature unknown
        current_limit_a = 1.0f;                   // Conservative default envelope
        vel_limit_rps = 50.0f;                    // 3000 rpm
        accel_limit = 200.0f;                     // Reaches the velocity limit in 0.25 s
        jerk_limit = 2000.0f;                     // Smooth ramp shaping
        err_vel_rps = 10.0f;                      // Gentle under fault conditions
        err_accel = 50.0f;                        // Gentle under fault conditions
        err_jerk = 500.0f;                        // Gentle under fault conditions
        axis_id = 0u;                             // No axis group
        axis_inverted = 0u;                       // Direction as measured
        axis_offset_q16 = 0;                      // No offset
        led_mode = 2u;                            // Firmware drives the status LED
        led_on_off = 0x44u;                       // 1 s on, 1 s off when blinking
        led_id = 0u;                              // All elements
        led_r = 0u;                               // Off
        led_g = 0u;                               // Off
        led_b = 0u;                               // Off
        gear_backlash = 0.0f;                     // Direct drive
        gear_ratio_mul = 1;                       // 1:1
        gear_ratio_div = 1u;                      // 1:1
        for (uint32_t i = 0; i < TP_IO_FUNCTIONS; ++i) {
            io_map[i].on_pin = 0u;                // Unmapped
            io_map[i].off_pin = 0u;               // Unmapped
            io_map[i].flags = 0u;                 // Direct polarity, inactive
        }
        encoder[0].type_iface = 0x40u;            // Absolute (0) over SPI (4)
        encoder[0].flags = 0u;                    // CW, no index
        encoder[0].ppr = 2097152u;                // 21-bit magnetic encoder
        encoder[1].type_iface = 0u;               // None declared
        encoder[1].flags = 0u;                    // None declared
        encoder[1].ppr = 0u;                      // None declared
        homing_flags = 0u;                        // Homing off
        homing_quick_vel = 1.0f;                  // 1 rev/s coarse search
        homing_quick_torque = 0.0f;               // No stall threshold
        homing_fine_retract = 0.1f;               // Back off a tenth of a turn
        homing_fine_vel = 0.1f;                   // 0.1 rev/s fine search
        homing_fine_torque = 0.0f;                // No stall threshold
        ref_offset_cw = 0.0f;                     // Unknown
        ref_offset_ccw = 0.0f;                    // Unknown
        ref_ang_cw = 0u;                          // Unknown
        ref_ang_ccw = 0u;                         // Unknown
        ref_ang_zero = 0u;                        // Unknown
        zero_offset_cw_q32 = 0;                   // Unknown
        zero_offset_ccw_q32 = 0;                  // Unknown
        field_cw_q32 = 0;                         // Unlimited
        field_ccw_q32 = 0;                        // Unlimited
        internal_offset_q32 = 0;                  // No offset
    }
};
