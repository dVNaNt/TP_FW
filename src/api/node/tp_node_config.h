#pragma once

/**
 * @file tp_node_config.h
 * @brief TPCP v1 - the persistent node configuration record and its build-time defaults.
 *
 * Everything a node remembers across a power cycle: its address, link bit
 * rates, control envelopes, watchdog policy and the cyclic-telemetry slot
 * table. One flat POD, so a record is written and restored as raw words.
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

#include "../tp_wire.h"  // TP_NODE_UNASSIGNED, TP_SAFE_* and the fixed-point codecs
/** Number of independently configurable cyclic telemetry producers. */
#define TP_TELEMETRY_SLOTS 4u

/** Magic word identifying a node-configuration record inside the partition. */
#define TP_CONFIG_MAGIC 0x7C0Fu  // "TunePulse Config", low half of word 0

/** Layout version of the configuration record; bumped on any field change. */
#define TP_CONFIG_VERSION 1u

/** Words in one configuration record (two 32-byte flash blocks). */
#define TP_CONFIG_WORDS 16u

/** Bytes in one configuration record. */
#define TP_CONFIG_BYTES (TP_CONFIG_WORDS * 4u)

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
//   -D TP_DEFAULT_LINK_FLAGS=4     TP_CFG_CLASSIC_LINK: classic CAN 2.0A
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
    uint8_t  opcode;     ///< Status opcode to emit (0x01-0x0F); 0 = slot unused.
    uint8_t  flags;      ///< TpTelemetryFlag bits.
    uint16_t period_ms;  ///< Emission period [ms]; 0 means SYNC-triggered only.
};

/** Flag bits of TpNodeConfig::flags. */
#define TP_CFG_AUTO_ENABLE  (1u << 0)  ///< Enable the output stage as soon as the drive is calibrated.
#define TP_CFG_PROMISCUOUS  (1u << 1)  ///< Accept every identifier (bus monitoring).
#define TP_CFG_CLASSIC_LINK (1u << 2)  ///< Run classic CAN 2.0A instead of CAN-FD.

/**
 * @brief Everything the master can change and persist about this node.
 *
 * The struct is the RAM shadow: registers read and write it directly, and
 * apply() pushes it into the live subsystems. That indirection exists because
 * several protection limits are write-only in the driver layer (the proxies
 * program a DAC threshold and keep nothing), so the shadow is the only place
 * a configured value can be read back from.
 */
struct TpNodeConfig {
    uint8_t  node_id;          ///< Bus address, 0 = unassigned.
    uint8_t  flags;            ///< TP_CFG_* bits.
    uint8_t  safe_state;       ///< TpSafeState used while disabled or stopped.
    uint8_t  watchdog_action;  ///< TpSafeState entered when the watchdog expires.
    uint16_t heartbeat_ms;     ///< Heartbeat period [ms]; 0 disables the heartbeat.
    uint16_t watchdog_ms;      ///< Master-liveness timeout [ms]; 0 disables it.
    uint16_t arb_kbps;         ///< Arbitration bit rate [kbit/s].
    uint16_t data_kbps;        ///< CAN-FD data-phase bit rate [kbit/s].

    TpTelemetrySlot telemetry[TP_TELEMETRY_SLOTS];  ///< Cyclic telemetry configuration.

    float oc_limit_a;      ///< Phase-A hardware overcurrent threshold [A].
    float oc_limit_b;      ///< Phase-B hardware overcurrent threshold [A].
    float oc_limit_c;      ///< Phase-C hardware overcurrent threshold [A].
    float ov_limit_v;      ///< Overvoltage trip [V].
    float uv_limit_v;      ///< Undervoltage trip [V].
    float ot_limit_c;      ///< Overtemperature trip [degC].
    float ot_hyst_c;       ///< Overtemperature release hysteresis [degC].
    float op_limit_w;      ///< Overpower trip [W].
    float op_limit_a;      ///< Bus overcurrent trip [A].
    float current_limit_a; ///< Control current envelope [A].
    float vel_limit_rps;   ///< Velocity envelope [RPS].
    float accel_limit;     ///< Acceleration envelope [RPS^2].
    float jerk_limit;      ///< Jerk envelope [RPS^3].

    /**
     * @brief Load the compiled-in defaults.
     *
     * These mirror the values tunepulse_init() has always programmed, so a
     * board with an erased configuration partition behaves exactly like one
     * built before this layer existed.
     */
    void set_defaults() {
        node_id = TP_DEFAULT_NODE_ID;             // Normally unassigned: discovery hands out an address
        flags = TP_DEFAULT_LINK_FLAGS;            // Manual enable, CAN-FD, not promiscuous
        safe_state = TP_SAFE_FREEWHEEL;           // Coast when de-energized
        watchdog_action = TP_SAFE_FREEWHEEL;      // Coast when the master goes quiet
        heartbeat_ms = 100u;                      // 10 Hz liveness
        watchdog_ms = 0u;                         // Watchdog off until the master arms it
        arb_kbps = TP_DEFAULT_ARB_KBPS;           // 1 Mbit/s arbitration by default
        data_kbps = TP_DEFAULT_DATA_KBPS;         // 5 Mbit/s FD data phase by default
        for (uint32_t i = 0; i < TP_TELEMETRY_SLOTS; ++i) {
            telemetry[i].opcode = 0u;             // Slot unused
            telemetry[i].flags = 0u;              // Disabled
            telemetry[i].period_ms = 0u;          // No timer
        }
        oc_limit_a = 4.0f;                        // Matches the DRV8323H board wiring
        oc_limit_b = 4.0f;                        // Matches the DRV8323H board wiring
        oc_limit_c = 6.2f;                        // Phase C carries the sum current
        ov_limit_v = 51.0f;                       // Bus capacitor rating headroom
        uv_limit_v = 16.0f;                       // Gate-driver dropout margin
        ot_limit_c = 85.0f;                       // Silicon derating point
        ot_hyst_c = 10.0f;                        // Release at 75 degC, no chatter
        op_limit_w = 60.0f;                       // Thermal budget of the stage
        op_limit_a = 2.5f;                        // Bus current budget
        current_limit_a = 1.0f;                   // Conservative default envelope
        vel_limit_rps = 50.0f;                    // 3000 rpm
        accel_limit = 200.0f;                     // Reaches the velocity limit in 0.25 s
        jerk_limit = 2000.0f;                     // Smooth ramp shaping
    }
};
