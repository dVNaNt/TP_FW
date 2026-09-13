#ifndef RUN_CLN17V3_H
#define RUN_CLN17V3_H

/**
 * @file run.h
 * @brief CLN17 v3 application layer: run_setup() / run_loop().
 *
 * main.cpp only forwards to these two entry points - everything board-specific
 * lives here (and in tunepulse.h, which owns the peripheral bring-up, the
 * protection limits and the board callbacks).
 *
 * This target brings the drive up and then hosts the TunePulse CAN Protocol:
 * the board answers requests, streams telemetry, executes motion and reports
 * faults on the bus, while the USB console stays a read-only diagnostic
 * channel.
 *
 * The console (boot banner, 1 Hz heartbeat) exists only when DEV_LOG_ENABLE is
 * defined (cln17_v3_dev / cln17_v3_bench). The release image prints nothing,
 * skips the wait for a host to open the CDC port, and is reached over TPCP
 * alone - the CDC endpoint still carries the binary protocol (tp_serial.h).
 *
 * The layering is strict: can_setup.h owns the FDCAN1 peripheral, src/api/
 * owns the protocol (wire format, registers, discovery, telemetry) and
 * drive_control.h turns protocol commands into control-loop targets. This file
 * only sequences bring-up and pumps the service calls.
 *
 * USB is opened HERE rather than in main.cpp: tp_hid_begin() enumerates a
 * composite device (CDC console plus the HID control pipe) and performs
 * USBD_Init() itself, so it replaces SerialUSB.begin() instead of following it.
 *
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#include <Arduino.h>

#include "tunepulse.h"          // Board bring-up (also pulls in the CAN driver)

#include "api/transport/tp_can.h"   // TunePulse CAN Protocol bound to FDCAN1

#ifdef DEV_LOG_ENABLE
/** Max time run_setup() waits for the USB host to open the CDC port [ms]. */
static constexpr uint32_t USB_ATTACH_TIMEOUT_MS = 5000;

/** Period of the diagnostic heartbeat line printed from run_loop() [ms]. */
static constexpr uint32_t HEARTBEAT_PERIOD_MS = 1000;

/** millis() timestamp of the last heartbeat line. */
static uint32_t hb_last_ms = 0;

/** TunePulseHeart::counter sampled at the previous heartbeat. */
static uint32_t hb_last_ticks = 0;

/**
 * @brief Print one diagnostic heartbeat line.
 *
 * Runs in THREAD mode (never from an ISR), but blocking here is NOT harmless
 * any more: run_loop() also services the CAN protocol, so a host that opens the
 * CDC port and then stops reading would take the node off the bus. The whole
 * line is therefore gated on free TX room and DROPPED rather than blocking -
 * the same rule the TP_LOG macros apply inside the control ISR.
 *
 * The important field is "tick/s": TunePulseHeart::counter
 * is incremented once per completed control pass, so the delta over one
 * second IS the measured control-loop rate. A healthy drive sits at ~20000;
 * any sustained dip means control passes are being lost (ISR stalled or
 * samples dropped), which is exactly what a blocking log call inside the
 * SPI1 ISR produces.
 */
static void print_heartbeat() {
    // One report is roughly 230 characters; if that much room is not free the
    // line is skipped entirely, which keeps run_loop() - and therefore the CAN
    // protocol service - free of any host-paced stall.
    if (!tp_log_room(240)) {
        hb_last_ms = millis();                           // Skip this window, keep the cadence
        hb_last_ticks = TP.counter;                      // ... and the rate baseline
        return;
    }
    const uint32_t now = millis();                       // Current uptime [ms]
    const uint32_t ticks = TP.counter;                   // Control passes completed so far
    const uint32_t d_ms = now - hb_last_ms;              // Elapsed time since the last line
    const uint32_t d_ticks = ticks - hb_last_ticks;      // Control passes in that window
    hb_last_ms = now;                                    // Latch for the next window
    hb_last_ticks = ticks;                               // Latch for the next window

    // Effective control-loop rate in Hz (integer math, no FPU rounding games).
    const uint32_t rate = (d_ms > 0) ? ((d_ticks * 1000UL) / d_ms) : 0;

    SerialUSB.print("[HB] up=");
    SerialUSB.print(now / 1000UL);                       // Uptime in whole seconds
    SerialUSB.print("s tick/s=");
    SerialUSB.print(rate);                               // Measured control-loop rate
    SerialUSB.print(" heart=");
    SerialUSB.print((unsigned)TP.motor_status);          // TunePulseHeart lifecycle state
    SerialUSB.print(" motor=");
    SerialUSB.print((unsigned)TP.current_driver.get_motor_status());  // Identification state
    SerialUSB.print(" cal[done=");
    SerialUSB.print((unsigned)TP.angle_calibrator.is_calibrated());   // Angle table usable
    SerialUSB.print(" scan=");
    SerialUSB.print((unsigned)TP.angle_calibrator.is_settling());     // CheckMemory stage
    SerialUSB.print(" save=");
    SerialUSB.print((unsigned)TP.angle_calibrator.is_persisting());   // Save stage
    SerialUSB.print("] prof[busy=");
    SerialUSB.print((unsigned)TP.motor_params.is_flash_busy());       // Record engine running
    SerialUSB.print(" loaded=");
    SerialUSB.print((unsigned)TP.motor_params.is_loaded());           // Profile restored at boot
    SerialUSB.print("] win=");
    SerialUSB.print((unsigned)tp_flash_programming_window);           // De-energized NVM window
    SerialUSB.print(" isense=");
    SerialUSB.print((unsigned)TP.driver_phy.current.is_calibrated()); // Offset calibration done
    SerialUSB.print(" vsup=");
    SerialUSB.print(TP.driver_phy.supply.get_voltage(), 2);           // Supply rail [V]
    SerialUSB.print("V can[node=");
    SerialUSB.print((unsigned)TP_API.config().node_id);               // Assigned bus address
    SerialUSB.print(" rx=");
    SerialUSB.print((unsigned long)CAN_BUS.rx_count);                 // Frames accepted by the filters
    SerialUSB.print(" tx=");
    SerialUSB.print((unsigned long)CAN_BUS.tx_count);                 // Frames handed to the transmitter
    SerialUSB.print(" drop=");
    SerialUSB.print((unsigned long)CAN_BUS.dropped);                  // Frames lost to ring overflow
    SerialUSB.print(" nack=");
    SerialUSB.print((unsigned)TP_API.rejected());                     // Requests answered with a NACK
    SerialUSB.print(" done=");
    SerialUSB.print((unsigned)TP_API.handled());                      // Requests decoded and executed
    SerialUSB.print("] hid[rx=");
    SerialUSB.print((unsigned long)TP_HID.rx_count());                // Frames taken from HID reports
    SerialUSB.print(" tx=");
    SerialUSB.print((unsigned long)TP_HID.tx_count());                // Frames packed into HID reports
    SerialUSB.print(" drop=");
    SerialUSB.print((unsigned long)(TP_HID.dropped() + TP_HID.overruns()));  // Lost to a full ring
    SerialUSB.print("] gw[on=");
    SerialUSB.print((unsigned)tp_gateway_armed);                      // USB<->CAN relay armed
    SerialUSB.print(" down=");
    SerialUSB.print((unsigned long)tp_gateway_down);                  // Host frames copied to the bus
    SerialUSB.print(" up=");
    SerialUSB.print((unsigned long)tp_gateway_up);                    // Bus answers copied to the host
    SerialUSB.print(" lost=");
    SerialUSB.print((unsigned long)tp_gateway_lost);                  // Relays dropped on a full ring
    SerialUSB.print("] mode=");
    DriveTelemetry dt;                                                // Live control snapshot
    TP.drive.read_telemetry(dt);
    SerialUSB.print((unsigned)dt.mode);                               // Active control mode
    SerialUSB.print(" en=");
    SerialUSB.print((unsigned)((dt.flags & DRIVE_TLM_ENABLED) != 0)); // Output stage armed
    SerialUSB.print(" q=");
    SerialUSB.println((unsigned)dt.queue_depth);                      // Queued motion segments
}
#endif  // DEV_LOG_ENABLE

/**
 * @brief One-time initialization: bootloader button, USB, motor stack, protocol.
 */
#if (ENCODER_TYPE == ENCODER_MT6835)
/**
 * @brief Board probe behind diagnostics record 0x43/0x07 (encoder link).
 *
 * Copies the SPI link supervisor's counters (spi_setup.h, written by the DMA
 * completion ISR) into the protocol's snapshot. Torn reads are acceptable:
 * these are statistics, and each field is a single aligned word.
 */
static void encoder_link_probe(TpEncoderLinkInfo& out) {
    out.check_proven = ENCODER_LINK.check_trusted;    // Failing frames are being dropped
    out.link_down = ENCODER_LINK.link_down;           // Nothing ingested, fault raised
    out.sensor_status = ENCODER_LINK.sensor_status;   // MT6835 STATUS[2:0] of the last accepted frame
    out.relinks = ENCODER_LINK.relinks;               // Bus resets since boot
    out.frames_bad = ENCODER_LINK.frames_bad;         // CRC failures since boot
    out.frames_ok = ENCODER_LINK.frames_ok;           // Frames accepted since boot
}
#endif

inline void run_setup() {
    BOOT::init_button();  // SW1 (PC6) -> 3 s hold enters the USB DFU bootloader

    // Power-rail / USB stack stabilization; SW1 is polled meanwhile, so
    // holding it during power-up counts towards the 3 s bootloader trigger.
    const uint32_t t0 = millis();
    while ((millis() - t0) < 2000U) {
        BOOT::poll();  // may never return (enters USB DFU bootloader)
        delay(BOOT::POLL_INTERVAL_MS);
    }

    // Composite USB bring-up: the CDC console this file prints through, plus
    // the vendor HID interface the control link runs on. Replaces
    // SerialUSB.begin() - the console behaves identically either way, the
    // device simply enumerates with a second function beside it.
    if (!tp_hid_begin()) {
        // Nothing can be reported yet: the console that would carry the message
        // is the thing that failed. The board still boots and stays reachable
        // over CAN, which is the point of not stopping here.
    }

#ifdef DEV_LOG_ENABLE
    // Wait for the host to actually OPEN the port (DTR asserted) before
    // printing anything. USBSerial::write() silently discards every byte
    // while !CDC_connected(), so a banner emitted right after begin() is
    // lost - which is why boot logs never showed up in the monitor. The
    // wait is bounded so the board still boots headless (no USB host).
    const uint32_t usb_t0 = millis();
    while (!SerialUSB && (millis() - usb_t0) < USB_ATTACH_TIMEOUT_MS) {
        BOOT::poll();  // Keep the hold-to-bootloader gesture alive while waiting
    }

    SerialUSB.println();
    SerialUSB.println("========================================================");
    SerialUSB.println("  TunePulse firmware - CLN17 v3");
    SerialUSB.print("  Build     : ");
    SerialUSB.print(__DATE__);
    SerialUSB.print(" ");
    SerialUSB.println(__TIME__);
    SerialUSB.print("  PWM       : ");
    SerialUSB.print((unsigned)PWM_TIMER_FREQ_HZ);
    SerialUSB.print(" Hz, period ");
    SerialUSB.println((unsigned)PWM_TIMER_PERIOD);
    SerialUSB.print("  NVM       : calib ");
    SerialUSB.print((unsigned)FlashG4_LL::calib_section_size());
    SerialUSB.print(" B / user ");
    SerialUSB.print((unsigned)FlashG4_LL::user_section_size());
    SerialUSB.print(" B / config ");
    SerialUSB.print((unsigned)FlashG4_LL::config_section_size());
    SerialUSB.println(" B");
    SerialUSB.println("  Bringing up peripherals + calibration...");
    SerialUSB.println("========================================================");
#endif  // DEV_LOG_ENABLE

    tunepulse_init();

    // Protocol bring-up AFTER the drive: tp_can_begin() reads the stored
    // configuration and applies its protection limits over the ones
    // tunepulse_init() programmed, so the persisted values are the ones that
    // end up in force. It also starts the link at the stored bit rates.
    if (!tp_can_begin(TP)) {
        TP_LOG("CAN protocol init FAILED");
    }
#if (ENCODER_TYPE == ENCODER_MT6835)
    TP_API.bind_encoder_link(encoder_link_probe);  // 0x43/0x07 reads the SPI link supervisor
#endif

    // A node with no address answers network management only, which is exactly
    // what makes it discoverable; say so once so an unconfigured board on the
    // bench is not mistaken for a dead one.
    if (TP_API.config().node_id == TP_NODE_UNASSIGNED) {
        TP_LOG("  CAN: no node address stored - listening on 0x3FF for discovery (registers 0x11 / 0x12)");
    }
}

/**
 * @brief Main loop: pump the CAN protocol and print the diagnostic heartbeat.
 *
 * Nothing here touches control state directly. Protocol commands are handed to
 * the control loop through the DriveControl mailbox, which the 20 kHz ISR
 * drains between passes, so this loop can be preempted at any instruction
 * without ever tearing a setpoint.
 */
inline void run_loop() {
    BOOT::poll();  // hold SW1 for 3 s to enter the USB bootloader

    // Receive, execute, answer, emit telemetry, drain pending NVM writes.
    tp_can_service();

#ifdef TP_BENCH_LOOPBACK
    // Bench self-test: with the controller in internal loopback, ask ourselves
    // for Device Info once a second. "done=" in the heartbeat line rising by one
    // per second proves the entire protocol chain works with no bus, no second
    // node and no adapter - which is otherwise impossible to check, because a
    // lone node on a real bus is never acknowledged and ends up bus-off.
    static uint32_t ping_last_ms = 0;                  // Timestamp of the last self-request
    if ((millis() - ping_last_ms) >= 1000U) {
        ping_last_ms = millis();
        tp_can_selftest_ping();
    }
#endif

#ifdef DEV_LOG_ENABLE
    // Periodic liveness + control-rate report. Dropped rather than blocked
    // when the console is not draining (see print_heartbeat).
    if ((millis() - hb_last_ms) >= HEARTBEAT_PERIOD_MS) {
        print_heartbeat();
    }
#endif
}

#endif  // RUN_CLN17V3_H
