#pragma once

/**
 * @file tp_chunk_system.h
 * @brief TPCP v1 - the universal frame container.
 *
 * union TpFrame names every layout defined in the other three chunk headers,
 * so it can only be declared once all of them are visible - which is why it
 * sits in its own file at the end of the include chain rather than beside any
 * one family.
 *
 * Serialization is zero-copy: TpFrame is a union of a raw byte array, a
 * 64-bit word and one typed view per layout, so filling a view IS the packing
 * and reading a view IS the unpacking. No memcpy, no serialization loops.
 *
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#include "tp_chunk_config.h"  // Read/write register layouts
#include "tp_chunk_motion.h"  // Motion tasks, save, MIDI, burst
#include "tp_chunk_status.h"  // Read-only registers, replies, PulseBeat

// ============================================================================
// Universal frame container
// ============================================================================

/**
 * @brief The 8-byte protocol frame with one typed view per layout.
 *
 * The transport moves b8[]; the handlers read and write the typed views. All
 * views alias the same 64 bits, so no conversion step exists at all.
 */
union TpFrame {
    uint8_t  b8[8];  ///< Raw byte access, used by the transport layer.
    uint64_t u64;    ///< Whole-frame word: one instruction to clear or compare a frame.

    // ---- Read/write registers ---------------------------------------
    TpDirectAccess      direct;         ///< View: 0x00.
    TpAxisControl       axis;           ///< View: 0x01/0x00.
    TpLedControl        led;            ///< View: 0x01/0x01.
    TpGearbox           gearbox;        ///< View: 0x01/0x02.
    TpIoMap             io_map;         ///< View: 0x01/0x03.
    TpEncoderConfig     encoder_cfg;    ///< View: 0x01/0x04, 0x01/0x05.
    TpHomingQuick       homing_quick;   ///< View: 0x01/0x06.
    TpHomingFine        homing_fine;    ///< View: 0x01/0x07.
    TpRefLimit          ref_limit;      ///< View: 0x01/0x08, 0x01/0x09.
    TpPosZero           pos_zero;       ///< View: 0x01/0x0A.
    TpPosOffset         pos_offset;     ///< View: 0x01/0x0B - 0x01/0x0F.
    TpTelemetrySlotCfg  telemetry;      ///< View: 0x02/slot.
    TpBusSettings       bus;            ///< View: 0x10.
    TpDiscoveryAssign   discovery;      ///< View: 0x11.
    TpDiscoverySearch   search;         ///< View: 0x12.
    TpSystemControl     system;         ///< View: 0x20.
    TpMotorProps1       motor1;         ///< View: 0x21.
    TpMotorProps2       motor2;         ///< View: 0x22.
    TpPowerLimits       power_limits;   ///< View: 0x23.
    TpMotorThermal      motor_thermal;  ///< View: 0x24.
    TpDriverThermal     driver_thermal; ///< View: 0x25.
    TpSupplyProtection  supply_prot;    ///< View: 0x26.
    TpCurrentProtection current_prot;   ///< View: 0x27.
    TpMotionLimits      limits;         ///< View: 0x2A, 0x2B.
    TpWatchdog          watchdog;       ///< View: 0x2D.
    TpCalTableAccess    cal_table;      ///< View: 0x2F.

    // ---- Read-only registers ----------------------------------------
    TpBatchRead         batch;          ///< View: 0x40.
    TpRangeRead         range;          ///< View: 0x41.
    TpCurrents          currents;       ///< View: 0x42/x.
    TpDiagComm          diag_comm;      ///< View: 0x43/0x00.
    TpDiagCalibration   diag_calib;     ///< View: 0x43/0x01.
    TpDiagQueue         diag_queue;     ///< View: 0x43/0x02.
    TpDiagPersist       diag_persist;   ///< View: 0x43/0x03.
    TpDiagEncoder       diag_encoder;   ///< View: 0x43/0x04.
    TpDiagCalibResult   diag_result;    ///< View: 0x43/0x05.
    TpDiagFoc           diag_foc;       ///< View: 0x43/0x06.
    TpDiagEncoderLink   diag_enc_link;  ///< View: 0x43/0x07.
    TpDiagIdentFail     diag_ident;     ///< View: 0x43/0x08.
    TpTrace             trace;          ///< View: 0x44.
    TpDeviceInfo        device_info;    ///< View: 0x50.
    TpSystemStatus      system_status;  ///< View: 0x51.
    TpMotorStatus       motor_status;   ///< View: 0x52.
    TpMotionStatus1     motion1;        ///< View: 0x53.
    TpMotionStatus2     motion2;        ///< View: 0x54.
    TpMotionStatus3     motion3;        ///< View: 0x55.
    TpPosition56        position56;     ///< View: 0x56, 0x57.
    TpDriverStatus      driver_status;  ///< View: 0x58.
    TpAck               ack;            ///< View: 0x5E / 0x5F.
    TpPulseBeat         beat;           ///< View: 0x7x (telemetry and emergency channels).

    // ---- Write-only commands and burst -------------------------------
    TpMotionTask1       mt1;            ///< View: 0xC0.
    TpMotionTask2       mt2;            ///< View: 0xC1.
    TpMotionTask3       mt3;            ///< View: 0xC2.
    TpMotionTask4       mt4;            ///< View: 0xC3.
    TpSaveConfig        save;           ///< View: 0xCF.
    TpMidi              midi;           ///< View: 0xD0.
    TpBurst             burst;          ///< View: 0xE0 - 0xFF.
};
static_assert(sizeof(TpFrame) == 8, "TpFrame must be exactly one classic CAN payload");
