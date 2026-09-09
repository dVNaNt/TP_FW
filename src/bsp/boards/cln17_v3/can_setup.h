/**
 * @file can_setup.h
 * @brief CAN-FD hardware driver + 8-byte chunk command channel (merged).
 *
 * Single-header CAN stack for STM32G431CB: FDCAN1 bring-up (GPIO PB8/PB9,
 * transceiver-enable PC14, bit timing calculation, TDC, IRQ) and a
 * chunk-oriented command API on top.
 *
 * ## Node addressing and filtering
 * The four standard filter slots are bound to one node address by
 * set_node_filters(): network management (0x100-0x10F) is always accepted,
 * the setpoint (0x200+n / 0x2FF) and request (0x300+n / 0x3FF) channels are
 * accepted only once an address has been assigned, and a fourth slot can be
 * opened for promiscuous bus monitoring. Everything else is rejected in
 * hardware, so traffic addressed to other drives costs this node no
 * interrupts at all. See src/api/CAN_PROTOCOL.md for the identifier map.
 *
 * ## Bus health
 * update() also drives bus-off recovery: a bus-off event latches CCCR.INIT in
 * hardware and the node stays off the bus until software clears it, so
 * without this call a single wiring fault would be permanent until reboot.
 *
 * ## Protocol
 * One chunk == one command == exactly 8 bytes; there are no START/END
 * markers and no service bytes on the wire. write(id, msg, len) splits the
 * message into ceil(len/8) chunks; CAN-FD frames pack up to 8 chunks (64 B,
 * BRS), the receiver derives the chunk count from the DLC. A tail of 5 or
 * 7 chunks is padded with all-zero chunks up to the DLC-representable 6 or
 * 8 (a chunk starting with 0 is a NOP). Classic frames carry one chunk.
 *
 * ## Concurrency
 * - RX: the FDCAN ISR is the only chunk-ring producer, read() the only
 *   consumer. One index per context; payload is published (DMB) before the
 *   index, rings are power-of-two masked - safe under IRQ preemption.
 * - TX: write()/send() produce, update() consumes; same pattern.
 * - Counters are volatile uint32_t (atomic single-writer on Cortex-M4).
 *
 * ## RAM budget
 * TX frame ring 8 x 67 B + RX chunk ring 32 x 10 B ~= 0.9 KiB total.
 *
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#ifndef CAN_SETUP_CLN17V3_H
#define CAN_SETUP_CLN17V3_H

#include <Arduino.h>
#include <cstring>
#include "stm32g4xx_hal.h"

/** @brief FDCAN1 interrupt vector (defined after the class, C linkage). */
extern "C" void FDCAN1_IT0_IRQHandler(void);
/** @brief HAL RX event callback (C linkage, weak override). */
extern "C" void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef*, uint32_t);

/**
 * @class CANDriver
 * @brief FDCAN1 hardware setup plus chunked command TX/RX queues.
 */
class CANDriver {
public:
    // ---- Fixed link configuration -------------------------------------------
    static constexpr uint32_t ARB_BITRATE = 1000000;  // Arbitration phase [bps]
    static constexpr uint32_t FD_BITRATE  = 5000000;  // FD data phase [bps]

    static constexpr uint8_t  CHUNK_SIZE = 8;         // Command size [bytes]
    static constexpr uint8_t  CHUNKS_PER_FRAME = 8;   // Max chunks per FD frame
    static constexpr uint8_t  TX_FRAMES  = 8;         // TX ring depth (pow2)
    static constexpr uint8_t  TX_MASK    = TX_FRAMES - 1U;
    static constexpr uint8_t  RX_CHUNKS  = 32;        // RX ring depth (pow2)
    static constexpr uint8_t  RX_MASK    = RX_CHUNKS - 1U;
    static constexpr uint16_t MAX_WRITE_LEN = 248;    // write() limit [bytes]

    /**
     * @brief Bring the link up: FD-BRS, 1 Mbps / 5 Mbps, NORMAL, auto-retry.
     * @return true when the peripheral is initialized and running.
     */
    bool begin() {
        baudrate_classic = ARB_BITRATE;
        baudrate_fd = FD_BITRATE;
        mode = CANMode::NORMAL;
        auto_retry_enabled = true;
        delay_comp_enabled = true;
        return init();                      // GPIO + FDCAN + IRQ bring-up
    }

    /**
     * @brief Non-blocking service call: bus-off recovery + TX ring pump.
     *
     * Must be called periodically from thread mode. It is the only place that
     * restarts a node the bus knocked offline, so skipping it makes a wiring
     * fault permanent.
     */
    void update() { service_bus(); }

    /**
     * @brief Transmit a whole message as 8-byte command chunks.
     *
     * FD: chunks are packed into 64-byte frames (8 each); a tail of 5 or 7
     * chunks is zero-padded to the next DLC-representable size (6 or 8).
     * Classic: one 8-byte frame per chunk. The call is atomic: either the
     * whole message fits into the TX ring now (true) or nothing is queued
     * (false) - a retry with the same buffer is always safe.
     * @param id  11-bit CAN identifier for every chunk of this message.
     * @param msg Pointer to the message bytes.
     * @param len Message length in bytes (1..MAX_WRITE_LEN).
     * @return true when every chunk was queued for transmission.
     */
    bool write(uint32_t id, const void* msg, uint16_t len) {
        if ((len == 0U) || (len > MAX_WRITE_LEN)) {
            return false;                              // Empty / oversized
        }
        const uint8_t* src = static_cast<const uint8_t*>(msg);  // Byte view
        const uint8_t per_frame =                      // Chunks per wire frame
            fd_mode ? CHUNKS_PER_FRAME : 1U;
        const uint16_t copy = per_frame * CHUNK_SIZE;  // Frame capacity [bytes]
        const uint8_t frames =                         // Ring slots needed
            static_cast<uint8_t>((len + copy - 1U) / copy);
        if (frames > tx_free()) {
            return false;                              // Would overflow the ring
        }
        uint16_t byte_off = 0;                         // Read cursor into msg
        while (byte_off < len) {                       // Queue frame by frame
            const uint16_t left = len - byte_off;      // Bytes still to send
            const uint16_t real = (left < copy) ? left : copy;  // Real bytes
            TxFrame& f = tx_ring[tx_w];                // Next free ring slot
            f.id = static_cast<uint16_t>(id & 0x7FFU); // 11-bit identifier
            std::memcpy(f.payload, &src[byte_off], real);       // Bulk payload
            std::memset(&f.payload[real], 0, copy - real);      // Tail padding
            // DLC must cover a legal FD size: chunk counts of 5 and 7 have
            // no DLC and are padded up to 6 and 8 chunks (zeros) directly.
            f.n = (per_frame == 1U)
                      ? 1U                             // Classic: one chunk
                      : legal_chunks(static_cast<uint8_t>(
                            (real + CHUNK_SIZE - 1U) / CHUNK_SIZE));
            __DMB();                                   // Payload before index
            tx_w = (tx_w + 1U) & TX_MASK;              // Publish the slot
            byte_off += copy;                          // Advance read cursor
        }
        return true;
    }

    /**
     * @brief Queue exactly one 8-byte frame (the protocol fast path).
     *
     * Unlike write(), this never pads the 64-byte FD payload: a protocol frame
     * is always one chunk, so only 8 bytes are copied and the DLC is fixed at
     * 8. That matters because the telemetry scheduler can emit several frames
     * per millisecond and write() would memset 56 unused bytes each time.
     *
     * @param id    11-bit CAN identifier of the frame.
     * @param frame Pointer to exactly 8 payload bytes.
     * @return true when the frame was accepted, false when the ring is full.
     */
    bool send(uint32_t id, const uint8_t* frame) {
        if (tx_free() == 0U) {
            return false;                            // Ring full: caller decides what to do
        }
        TxFrame& f = tx_ring[tx_w];                  // Next free ring slot
        f.id = static_cast<uint16_t>(id & 0x7FFU);   // 11-bit identifier
        std::memcpy(f.payload, frame, CHUNK_SIZE);   // Exactly one chunk of payload
        f.n = 1U;                                    // One chunk -> DLC 8
        __DMB();                                     // Payload visible before the index moves
        tx_w = (tx_w + 1U) & TX_MASK;                // Publish the slot
        return true;
    }

    /**
     * @brief Queue a data-less frame whose identifier IS the message.
     *
     * Used by the UID discovery arbitration, where the winning node is decided
     * by the CAN arbitration field itself and the payload carries nothing.
     *
     * @param id 11-bit CAN identifier to transmit with DLC 0.
     * @return true when the frame was accepted.
     */
    bool send_empty(uint32_t id) {
        if (tx_free() == 0U) {
            return false;                            // Ring full
        }
        TxFrame& f = tx_ring[tx_w];                  // Next free ring slot
        f.id = static_cast<uint16_t>(id & 0x7FFU);   // 11-bit identifier
        f.n = 0U;                                    // DLC 0: identifier only
        __DMB();                                     // Metadata visible before the index moves
        tx_w = (tx_w + 1U) & TX_MASK;                // Publish the slot
        return true;
    }

    /**
     * @brief Pop one received 8-byte chunk from the RX queue.
     * @param[out] id    Full 11-bit CAN identifier the chunk arrived with.
     * @param[out] chunk Output buffer, 8 bytes are written.
     * @return true when a chunk was delivered.
     */
    bool read(uint32_t& id, uint8_t* chunk) {
        uint8_t len = 0;                         // Length discarded by this overload
        return read(id, chunk, len);
    }

    /**
     * @brief Pop one received chunk together with its real payload length.
     *
     * The payload is always 8 bytes wide and zero-padded; len reports how many
     * of them the sender actually transmitted, which is what network-management
     * frames need in order to be told apart from a padded command.
     *
     * @param[out] id    Full 11-bit CAN identifier the chunk arrived with.
     * @param[out] chunk Output buffer, 8 bytes are always written.
     * @param[out] len   Real payload length of this chunk (1..8).
     * @return true when a chunk was delivered.
     */
    bool read(uint32_t& id, uint8_t* chunk, uint8_t& len) {
        if (rx_w == rx_r) {
            return false;                        // Ring empty
        }
        const uint8_t i = rx_r;                  // Oldest slot
        id = rx_ids[i];                          // Chunk identifier
        len = rx_lens[i];                        // Real payload length
        std::memcpy(chunk, &rx_data[static_cast<uint16_t>(i) * CHUNK_SIZE],
                    CHUNK_SIZE);                 // Payload out (always 8 bytes)
        __DMB();                                 // Data read before index release
        rx_r = (rx_r + 1U) & RX_MASK;            // Release the slot
        return true;
    }

    /** @brief true when at least one received chunk is waiting. */
    bool is_rx_available() const { return rx_w != rx_r; }

    /** @brief true when nothing is queued for transmission. */
    bool is_tx_empty() const { return (tx_w == tx_r); }

    /** @brief true when write()/send() would be rejected right now. */
    bool is_tx_full() const { return (tx_free() == 0U); }

    /** @brief Frames currently waiting in the TX ring. */
    uint8_t tx_queued() const { return static_cast<uint8_t>((tx_w - tx_r) & TX_MASK); }

    // ---- Status counters (single-writer 32-bit: atomic on Cortex-M4) --------
    volatile uint32_t tx_count = 0;      // Frames handed to the transmitter
    volatile uint32_t rx_count = 0;      // Chunks received and queued
    volatile uint32_t error_count = 0;   // Driver-level errors
    volatile uint32_t dropped = 0;       // Chunks lost to ring overflow

    /** @brief true when begin() completed successfully. */
    bool is_ready() const { return is_initialized; }

    // ========================================================================
    // Protocol support: hardware filtering, bus health, link reconfiguration
    // ========================================================================

    /**
     * @brief Install the acceptance filters for one node address.
     *
     * Filtering is done in HARDWARE so a busy multi-drive bus costs this node
     * nothing: frames addressed to other nodes never reach the RX FIFO, never
     * raise an interrupt and can never overflow the chunk ring.
     *
     * Four standard filter slots are used:
     *  0: range 0x100-0x10F - network management (always accepted).
     *  1: dual  {0x200+node, 0x2FF} - real-time setpoints + their broadcast.
     *  2: dual  {0x300+node, 0x3FF} - requests + their broadcast.
     *  3: range 0x000-0x7FF - promiscuous monitoring, disabled by default.
     *
     * An unaddressed node (node_id == 0) leaves slots 1 and 2 disabled and so
     * listens to network management only, which is exactly what the discovery
     * protocol requires of a device that has not been given an address yet.
     *
     * @param node_id     Assigned node address (0 = unassigned).
     * @param promiscuous true to additionally accept every standard identifier.
     * @return true when every filter slot was accepted by the peripheral.
     */
    bool set_node_filters(uint8_t node_id, bool promiscuous = false) {
        node_id_ = node_id;               // Remember for a later re-init
        promiscuous_ = promiscuous;       // Remember for a later re-init
        if (!is_initialized) {
            return false;                 // Filters are installed by init() instead
        }
        return apply_filters();
    }

    /** @brief Node address the acceptance filters are currently bound to. */
    uint8_t get_node_id() const { return node_id_; }

    /** @brief Snapshot of the CAN error state used by the diagnostics chunk. */
    struct BusStatus {
        uint8_t tec;          // Transmit error counter (0-255)
        uint8_t rec;          // Receive error counter (0-127)
        bool    warning;      // At least one counter reached 96
        bool    error_passive;// The node is error passive
        bool    bus_off;      // The node was removed from the bus
    };

    /**
     * @brief Read the protocol error state straight out of the peripheral.
     *
     * Register reads only (PSR is read-clear for some bits, so the latched
     * fields are taken from ECR which is not) - safe to call at any rate.
     */
    BusStatus get_bus_status() const {
        BusStatus st = {0, 0, false, false, false};
        if (!is_initialized) {
            return st;                                        // Nothing to report yet
        }
        const uint32_t ecr = hfdcan.Instance->ECR;            // Error counters
        const uint32_t psr = hfdcan.Instance->PSR;            // Protocol status
        st.tec = static_cast<uint8_t>(ecr & FDCAN_ECR_TEC_Msk);
        st.rec = static_cast<uint8_t>((ecr & FDCAN_ECR_REC_Msk) >> FDCAN_ECR_REC_Pos);
        st.warning = (psr & FDCAN_PSR_EW) != 0U;
        st.error_passive = (psr & FDCAN_PSR_EP) != 0U;
        st.bus_off = (psr & FDCAN_PSR_BO) != 0U;
        return st;
    }

    /**
     * @brief Recover from bus-off and pump the TX ring.
     *
     * A bus-off event sets CCCR.INIT in hardware and the node stops
     * transmitting for good until software clears it. Clearing INIT starts the
     * mandated recovery sequence (128 occurrences of 11 recessive bits), after
     * which the node rejoins on its own. Without this the first serious wiring
     * fault would silently take the drive off the bus until a power cycle.
     */
    void service_bus() {
        if (is_initialized && ((hfdcan.Instance->PSR & FDCAN_PSR_BO) != 0U)) {
            if ((hfdcan.Instance->CCCR & FDCAN_CCCR_INIT) != 0U) {
                hfdcan.Instance->CCCR &= ~FDCAN_CCCR_INIT;    // Start bus-off recovery
                bus_off_events++;                             // Count it for diagnostics
            }
        }
        tx_pump();                                            // Ring -> hardware FIFO
    }

    /**
     * @brief Drop everything queued for transmission, in software and hardware.
     *
     * The UID discovery arbitration needs this: a node that loses arbitration
     * must not let the peripheral re-send its stale reply into the next search
     * round, where it would answer a question that was never asked.
     */
    void abort_pending_tx() {
        tx_r = tx_w;                                          // Discard the software ring
        if (is_initialized) {
            // Cancel every one of the three hardware TX FIFO buffers.
            HAL_FDCAN_AbortTxRequest(&hfdcan, FDCAN_TX_BUFFER0 | FDCAN_TX_BUFFER1 | FDCAN_TX_BUFFER2);
        }
    }

    /**
     * @brief Reconfigure the link and restart the peripheral.
     *
     * Bit timing and the frame format can only change while the controller is
     * in initialization mode, so the peripheral is torn down and rebuilt. The
     * acceptance filters are reinstalled from the stored node address.
     *
     * @param arb_bps  Arbitration bit rate [bps].
     * @param data_bps Data-phase bit rate [bps] (ignored in classic mode).
     * @param fd       true for CAN-FD with bit-rate switching, false for classic CAN 2.0A.
     * @param loopback true to bring the controller up in INTERNAL LOOPBACK: it
     *                 receives everything it transmits and needs no second node
     *                 to acknowledge it. Bench use only - on a real bus a single
     *                 node gets no acknowledgement, drives its transmit error
     *                 counter up and ends up bus-off, so loopback is the only
     *                 way to exercise the stack with nothing else connected.
     * @return true when the link came back up.
     */
    bool configure_link(uint32_t arb_bps, uint32_t data_bps, bool fd, bool loopback = false) {
        baudrate_classic = arb_bps;   // New arbitration rate
        baudrate_fd = data_bps;       // New data-phase rate
        fd_mode = fd;                 // New frame format
        mode = loopback ? CANMode::INTERNAL_LOOPBACK : CANMode::NORMAL;
        // Automatic retransmission stays ON. Losing arbitration is the normal
        // case on a shared bus, and with DAR set every such frame would be
        // dropped instead of retried - command and response traffic would go
        // missing whenever two nodes talked at once. The one place that needs
        // no-retry semantics is the discovery arbitration, and that is handled
        // exactly where it matters by aborting the pending frame before the
        // next reply (see abort_pending_tx).
        auto_retry_enabled = true;
        delay_comp_enabled = true;    // Mandatory at the FD data rate
        return init();                // Tear down and rebuild
    }

    /** @brief true when the link currently runs CAN-FD frames. */
    bool is_fd_mode() const { return fd_mode; }

    /** @brief Configured arbitration bit rate [bps]. */
    uint32_t get_arb_bitrate() const { return baudrate_classic; }

    /** @brief Configured data-phase bit rate [bps]. */
    uint32_t get_data_bitrate() const { return baudrate_fd; }

    /** @brief Number of bus-off recoveries started since boot. */
    volatile uint32_t bus_off_events = 0;

private:
    /** CAN operating modes (subset used by init()). */
    enum class CANMode : uint8_t {
        NORMAL = 0,             // Standard bus node
        INTERNAL_LOOPBACK = 1,  // Self-reception without pins
        EXTERNAL_LOOPBACK = 2,  // Self-reception with pins (no ACK needed)
        SILENT = 3              // Bus monitoring, TX disabled
    };

    /** One queued CAN frame: identifier plus the aggregated chunk payload. */
    struct TxFrame {
        uint16_t id;         // 11-bit CAN identifier
        uint8_t n;           // Chunks covered by the frame DLC (1..8)
        uint8_t payload[CHUNKS_PER_FRAME * CHUNK_SIZE];  // Up to 64 bytes
    };

    /** DLC (4-bit) -> payload byte count, used by both TX and RX paths. */
    static constexpr uint8_t dlc_lut[16] = {
        0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};

    /** @brief Round a chunk count up to a DLC-representable FD count. */
    static constexpr uint8_t legal_chunks(uint8_t n) {
        return (n == 5U) ? 6U : ((n == 7U) ? 8U : n);  // 5->6, 7->8
    }

    /**
     * @brief (Re)install the four acceptance filter slots.
     *
     * Standard filters may be reconfigured while the controller is running
     * (the HAL accepts state READY and BUSY), so a node address change does
     * not require a link restart.
     *
     * @return true when every slot was accepted.
     */
    bool apply_filters() {
        FDCAN_FilterTypeDef flt = {0};                     // Reused slot descriptor
        flt.IdType = FDCAN_STANDARD_ID;                    // 11-bit identifiers only

        // Slot 0: network management block, always accepted.
        flt.FilterIndex = 0U;
        flt.FilterType = FDCAN_FILTER_RANGE;
        flt.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
        flt.FilterID1 = 0x100U;
        flt.FilterID2 = 0x10FU;
        if (HAL_FDCAN_ConfigFilter(&hfdcan, &flt) != HAL_OK) {
            return false;
        }

        // Slot 1: real-time setpoints for this node plus the broadcast address.
        flt.FilterIndex = 1U;
        flt.FilterType = FDCAN_FILTER_DUAL;
        flt.FilterConfig = (node_id_ != 0U) ? FDCAN_FILTER_TO_RXFIFO0 : FDCAN_FILTER_DISABLE;
        flt.FilterID1 = 0x200U + node_id_;
        flt.FilterID2 = 0x2FFU;
        if (HAL_FDCAN_ConfigFilter(&hfdcan, &flt) != HAL_OK) {
            return false;
        }

        // Slot 2: request channel for this node plus the broadcast address.
        flt.FilterIndex = 2U;
        flt.FilterType = FDCAN_FILTER_DUAL;
        flt.FilterConfig = (node_id_ != 0U) ? FDCAN_FILTER_TO_RXFIFO0 : FDCAN_FILTER_DISABLE;
        flt.FilterID1 = 0x300U + node_id_;
        flt.FilterID2 = 0x3FFU;
        if (HAL_FDCAN_ConfigFilter(&hfdcan, &flt) != HAL_OK) {
            return false;
        }

        // Slot 3: promiscuous monitoring, only for bus analysis / debugging.
        flt.FilterIndex = 3U;
        flt.FilterType = FDCAN_FILTER_RANGE;
        flt.FilterConfig = promiscuous_ ? FDCAN_FILTER_TO_RXFIFO0 : FDCAN_FILTER_DISABLE;
        flt.FilterID1 = 0x000U;
        flt.FilterID2 = 0x7FFU;
        return HAL_FDCAN_ConfigFilter(&hfdcan, &flt) == HAL_OK;
    }

    /** @brief Free slots in the TX frame ring. */
    uint8_t tx_free() const {
        return static_cast<uint8_t>(TX_FRAMES - 1U - ((tx_w - tx_r) & TX_MASK));
    }

    /** @brief Move ring frames into the hardware TX FIFO while it accepts. */
    void tx_pump() {
        while (tx_r != tx_w) {                       // While frames queued
            const TxFrame& f = tx_ring[tx_r];        // Oldest queued frame
            if (!send_frame(f.id, f.payload, static_cast<uint8_t>(f.n * CHUNK_SIZE))) {
                break;                               // FIFO full: retry next update()
            }
            tx_r = (tx_r + 1U) & TX_MASK;            // Release the frame slot
        }
    }

    /**
     * @brief FDCAN RX ISR: unpack hardware FIFO frames into the chunk ring.
     *
     * Drains the FIFO (register-polled fill level, no HAL call), unpacks
     * every chunk directly into the final ring - no middle queue, no extra
     * copy. A full ring drops the remaining chunks of the burst (counted
     * in dropped) instead of blocking the ISR.
     */
    void on_rx_interrupt() {
        FDCAN_RxHeaderTypeDef hdr;               // Frame metadata
        uint8_t fbuf[64];                        // Frame payload (stack)
        while (true) {
            // Register-polled fill level: faster than a HAL call.
            const uint32_t fill =
                ((hfdcan.Instance->RXF0S & FDCAN_RXF0S_F0FL_Msk) >>
                 FDCAN_RXF0S_F0FL_Pos);
            if (fill == 0U) {
                return;                          // Hardware FIFO drained
            }
            if (HAL_FDCAN_GetRxMessage(&hfdcan, FDCAN_RX_FIFO0, &hdr,
                                       fbuf) != HAL_OK) {
                error_count++;                   // Read failed: drop the frame
                continue;
            }
            const uint32_t id = hdr.Identifier;  // Frame identifier
            const uint8_t len =
                dlc_lut[hdr.DataLength & 0x0FU]; // Payload bytes (DLC table)
            // A frame shorter than one chunk still carries a whole message
            // (network management uses 1..7 byte frames); it becomes ONE
            // zero-padded slot whose recorded length is the real DLC. Dropping
            // such frames - which the plain len/8 chunk count did - made every
            // short protocol frame invisible to the application.
            const uint8_t count = (len < CHUNK_SIZE) ? 1U : (len / CHUNK_SIZE);
            for (uint8_t c = 0; c < count; c++) {    // Unpack every chunk
                const uint8_t next = (rx_w + 1U) & RX_MASK;
                if (next == rx_r) {              // Ring full: drop the burst
                    dropped += static_cast<uint32_t>(count - c);
                    return;
                }
                // Store the WHOLE 11-bit identifier. Truncating it to a byte
                // used to collapse every protocol channel onto the same value
                // (0x300+n, 0x400+n and 0x600+n all alias to n), which made
                // any identifier-addressed protocol impossible to decode.
                rx_ids[rx_w] = static_cast<uint16_t>(id & 0x7FFU);
                uint8_t* slot = &rx_data[static_cast<uint16_t>(rx_w) * CHUNK_SIZE];
                if (len < CHUNK_SIZE) {          // Short frame: zero-pad the tail
                    std::memset(slot, 0, CHUNK_SIZE);
                    std::memcpy(slot, fbuf, len);
                    rx_lens[rx_w] = len;         // Remember the real DLC
                } else {                         // Full chunk out of the frame
                    std::memcpy(slot, &fbuf[static_cast<uint16_t>(c) * CHUNK_SIZE], CHUNK_SIZE);
                    rx_lens[rx_w] = CHUNK_SIZE;
                }
                __DMB();                         // Payload before index publish
                rx_w = (rx_w + 1U) & RX_MASK;    // Commit the slot
                rx_count++;
            }
        }
    }

    friend void FDCAN1_IT0_IRQHandler(void);   // Vector needs hfdcan access
    friend void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef*, uint32_t);

    /**
     * @brief Push one frame into the hardware TX FIFO (register-level fast).
     * @param id  11-bit CAN identifier.
     * @param payload Frame payload (8..64 bytes).
     * @param len  Payload size in bytes (multiple of 8).
     * @return true when the frame was accepted by the TX FIFO.
     */
    bool send_frame(uint32_t id, const uint8_t* payload, uint8_t len) {
        if ((hfdcan.Instance->TXFQS & FDCAN_TXFQS_TFQF) != 0U) {  // FIFO full
            error_count++;
            return false;
        }
        // Only the variable header fields are updated here; the rest of
        // tx_header (std id, data frame, no events) is set once in init().
        tx_header.Identifier = id & 0x7FFU;
        tx_header.DataLength = bytes_to_dlc(len);
        if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan, &tx_header,
                                          const_cast<uint8_t*>(payload)) !=
            HAL_OK) {
            error_count++;
            return false;
        }
        tx_count++;
        return true;
    }

    /** @brief CAN-FD DLC encoding for a payload byte count (1..64). */
    static uint32_t bytes_to_dlc(uint8_t bytes) {
        // DLC 0..8 map 1:1; above 8 only 12/16/20/24/32/48/64 are legal.
        // Chunk-multiple lengths (8/24/32/48/64) hit the exact branches.
        if (bytes <= 8) return bytes;
        if (bytes <= 12) return 9U;
        if (bytes <= 16) return 10U;
        if (bytes <= 20) return 11U;
        if (bytes <= 24) return 12U;
        if (bytes <= 32) return 13U;
        if (bytes <= 48) return 14U;
        return 15U;                          // 64 bytes
    }

    // ---- Configuration state (init-time only) --------------------------------
    FDCAN_HandleTypeDef hfdcan;           // HAL handle (init + IRQ routing)
    FDCAN_TxHeaderTypeDef tx_header;      // Pre-configured TX header template
    uint32_t baudrate_classic = ARB_BITRATE;
    uint32_t baudrate_fd = FD_BITRATE;
    CANMode mode = CANMode::NORMAL;
    bool auto_retry_enabled = false;
    bool delay_comp_enabled = true;
    bool fd_mode = true;                  // Link format (FD-BRS vs classic)
    bool is_initialized = false;
    bool is_enabled = true;
    uint8_t node_id_ = 0;                 // Node address the filters are bound to
    bool promiscuous_ = false;            // Accept every identifier (bus monitoring)

    /** Standard filter slots reserved in the FDCAN message RAM. */
    static constexpr uint32_t FILTER_SLOTS = 4U;

    /** Bit timing parameters for one CAN bit (nominal or data phase). */
    struct CANTiming {
        uint32_t prescaler, tseg1, tseg2, sjw;
    };

    /**
     * @brief Bit timing search: prescaler + TSEG1/TSEG2 for a target rate.
     * @param baudrate Target bit rate [bps].
     * @param clock_freq Kernel clock [Hz] (170 MHz).
     * @param is_data_phase true for the FD data phase (tighter TQ limits).
     * @return Struct with prescaler/tseg1/tseg2/sjw (defaults on failure).
     */
    CANTiming calculate_timing(uint32_t baudrate, uint32_t clock_freq,
                               bool is_data_phase) const {
        CANTiming t = {1, 12, 4, 4};            // Fallback timing
        const uint32_t max_tq = is_data_phase ? 49U : 385U;  // TQ span limits
        for (uint32_t p = 1; p <= 512U; p++) {  // Prescaler search
            const uint32_t total = (clock_freq / p) / baudrate;  // TQ per bit
            if ((total < 4U) || (total > max_tq)) {
                continue;                       // Outside the TQ window
            }
            const uint32_t actual = (clock_freq / p) / total;  // Achieved rate
            const uint32_t err = (actual > baudrate) ? (actual - baudrate)
                                                     : (baudrate - actual);
            if ((err * 200U) < baudrate) {      // Within 0.5 % tolerance
                uint32_t tseg1 = (total * 3U) / 4U;      // 75 % sample point
                uint32_t tseg2 = total - 1U - tseg1;     // Remaining phases
                t.prescaler = p;
                t.tseg1 = (tseg1 < 1U) ? 1U : tseg1;
                t.tseg2 = (tseg2 < 1U) ? 1U : tseg2;
                t.sjw = (tseg2 < 4U) ? tseg2 : 4U;
                return t;
            }
        }
        return t;                               // Nothing matched: fallback
    }

    // ---- TX frame ring (producer: write/send, consumer: update) -------------
    TxFrame tx_ring[TX_FRAMES];           // Frames awaiting the hardware FIFO
    volatile uint8_t tx_w = 0;            // Producer index (write)
    volatile uint8_t tx_r = 0;            // Consumer index (update)

    // ---- RX chunk ring (SPSC: producer = ISR, consumer = read) --------------
    uint16_t rx_ids[RX_CHUNKS];           // Full 11-bit identifier per chunk slot
    uint8_t rx_data[RX_CHUNKS * CHUNK_SIZE];  // Payload slots (flat SoA)
    uint8_t rx_lens[RX_CHUNKS];           // Real payload length per slot (1..8)
    volatile uint8_t rx_w = 0;            // Producer index (ISR)
    volatile uint8_t rx_r = 0;            // Consumer index (read)

    /**
     * @brief FDCAN1 bring-up: GPIO, clock, timings, filters, TDC, IRQ.
     * @return true when every configuration step succeeded.
     */
    bool init() {
        if (is_initialized) {
            deinit();                           // Restart if already active
        }
        init_gpio();                            // PB8/PB9 AF9 + PC14 enable
        RCC->APB1ENR1 |= RCC_APB1ENR1_FDCANEN;  // FDCAN kernel clock
        RCC->CCIPR = (RCC->CCIPR & ~RCC_CCIPR_FDCANSEL_Msk) |
                     (2U << RCC_CCIPR_FDCANSEL_Pos);  // Kernel clock = PCLK1

        const uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();  // 170 MHz
        const CANTiming tc = calculate_timing(baudrate_classic, pclk1, false);
        const CANTiming tf = calculate_timing(baudrate_fd, pclk1, true);

        hfdcan.Instance = FDCAN1;
        hfdcan.Init.ClockDivider = FDCAN_CLOCK_DIV1;
        hfdcan.Init.FrameFormat = fd_mode ? FDCAN_FRAME_FD_BRS
                                          : FDCAN_FRAME_CLASSIC;
        switch (mode) {                         // Translate the mode enum
            case CANMode::INTERNAL_LOOPBACK:
                hfdcan.Init.Mode = FDCAN_MODE_INTERNAL_LOOPBACK; break;
            case CANMode::EXTERNAL_LOOPBACK:
                hfdcan.Init.Mode = FDCAN_MODE_EXTERNAL_LOOPBACK; break;
            case CANMode::SILENT:
                hfdcan.Init.Mode = FDCAN_MODE_BUS_MONITORING; break;
            default:
                hfdcan.Init.Mode = FDCAN_MODE_NORMAL; break;
        }
        hfdcan.Init.AutoRetransmission = auto_retry_enabled ? ENABLE : DISABLE;
        hfdcan.Init.TransmitPause = ENABLE;
        hfdcan.Init.ProtocolException = DISABLE;
        hfdcan.Init.NominalPrescaler = tc.prescaler;
        hfdcan.Init.NominalSyncJumpWidth = tc.sjw;
        hfdcan.Init.NominalTimeSeg1 = tc.tseg1;
        hfdcan.Init.NominalTimeSeg2 = tc.tseg2;
        hfdcan.Init.DataPrescaler = tf.prescaler;
        hfdcan.Init.DataSyncJumpWidth = tf.sjw;
        hfdcan.Init.DataTimeSeg1 = tf.tseg1;
        hfdcan.Init.DataTimeSeg2 = tf.tseg2;
        hfdcan.Init.StdFiltersNbr = FILTER_SLOTS;  // Node filtering needs four slots
        hfdcan.Init.ExtFiltersNbr = 0U;
        hfdcan.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;

        if (HAL_FDCAN_Init(&hfdcan) != HAL_OK) {
            error_count++;
            return false;
        }

        // Transmitter delay compensation (TDC) - mandatory for the 5 Mbps
        // data phase: without it the transmitter re-samples its own stale
        // bits and every BRS frame fails with CRC errors. TDCO is expressed
        // in kernel clock TQ (verified on silicon): 16 TQ (~94 ns) sits well
        // inside the data sample point at every supported data rate.
        if (delay_comp_enabled) {
            constexpr uint32_t TDCO = 16U;      // Verified-safe SSP offset
            if (HAL_FDCAN_ConfigTxDelayCompensation(&hfdcan, TDCO, 0) !=
                HAL_OK) {
                error_count++;
                return false;
            }
            HAL_FDCAN_EnableTxDelayCompensation(&hfdcan);
        }

        // Acceptance filters for the currently bound node address. Anything
        // that does not match a slot is dropped by hardware, so a busy bus
        // costs this node no interrupts at all.
        if (!apply_filters()) {
            error_count++;
            return false;
        }
        // Non-matching standard frames are REJECTED (they used to be accepted,
        // which defeated the filters entirely); extended and remote frames are
        // rejected outright since the protocol uses neither.
        if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan, FDCAN_REJECT,
                                         FDCAN_REJECT, FDCAN_FILTER_REMOTE,
                                         FDCAN_FILTER_REMOTE) != HAL_OK) {
            error_count++;
            return false;
        }

        // Static TX header template: only Identifier/DLC change per frame.
        // FDFormat/BitRateSwitch MUST mark the frames as FD-BRS - with the
        // default (CLASSIC_CAN) the controller transmits classic frames and
        // truncates every payload to 8 bytes on the wire.
        tx_header.IdType = FDCAN_STANDARD_ID;
        tx_header.TxFrameType = FDCAN_DATA_FRAME;
        tx_header.FDFormat = fd_mode ? FDCAN_FD_CAN : FDCAN_CLASSIC_CAN;
        tx_header.BitRateSwitch = fd_mode ? FDCAN_BRS_ON : FDCAN_BRS_OFF;
        tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
        tx_header.MessageMarker = 0U;
        if (HAL_FDCAN_Start(&hfdcan) != HAL_OK) {
            error_count++;
            return false;
        }
        if (HAL_FDCAN_ActivateNotification(&hfdcan,
                                           FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
                                           0U) != HAL_OK) {
            error_count++;
            return false;
        }
        NVIC_SetPriority(FDCAN1_IT0_IRQn,
                         NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 7, 0));
        NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
        is_initialized = true;
        return true;
    }

    /**
     * @brief Stop the peripheral and disable its interrupt.
     */
    void deinit() {
        if (!is_initialized) {
            return;
        }
        NVIC_DisableIRQ(FDCAN1_IT0_IRQn);
        HAL_FDCAN_Stop(&hfdcan);
        HAL_FDCAN_DeInit(&hfdcan);
        is_initialized = false;
    }

    /**
     * @brief GPIO setup: PB9=TX / PB8=RX (AF9), PC14 = transceiver enable.
     */
    void init_gpio() {
        RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_GPIOCEN;
        // PB9: FDCAN1_TX -> AF9, push-pull, very high speed.
        GPIOB->MODER = (GPIOB->MODER & ~GPIO_MODER_MODE9_Msk) |
                       (2U << GPIO_MODER_MODE9_Pos);
        GPIOB->AFR[1] = (GPIOB->AFR[1] & ~GPIO_AFRH_AFSEL9_Msk) |
                        (9U << GPIO_AFRH_AFSEL9_Pos);
        GPIOB->OSPEEDR |= (3U << GPIO_OSPEEDR_OSPEED9_Pos);
        // PB8: FDCAN1_RX -> AF9, pull-up.
        GPIOB->MODER = (GPIOB->MODER & ~GPIO_MODER_MODE8_Msk) |
                       (2U << GPIO_MODER_MODE8_Pos);
        GPIOB->AFR[1] = (GPIOB->AFR[1] & ~GPIO_AFRH_AFSEL8_Msk) |
                        (9U << GPIO_AFRH_AFSEL8_Pos);
        GPIOB->PUPDR = (GPIOB->PUPDR & ~GPIO_PUPDR_PUPD8_Msk) |
                       (1U << GPIO_PUPDR_PUPD8_Pos);
        // PC14: transceiver enable, push-pull output (low = enabled).
        GPIOC->MODER = (GPIOC->MODER & ~GPIO_MODER_MODE14_Msk) |
                       (1U << GPIO_MODER_MODE14_Pos);
        GPIOC->BSRR = is_enabled ? GPIO_BSRR_BR14 : GPIO_BSRR_BS14;
    }
};


/** Global CAN driver instance used by the application. */
inline CANDriver CAN_BUS;

/**
 * @brief FDCAN1 interrupt vector: routes the pending event to the HAL
 *        dispatcher, which calls HAL_FDCAN_RxFifo0Callback -> on_rx_interrupt.
 *        Without this override the startup default (infinite loop) executed
 *        on the first received frame and hung the firmware.
 */
// FDCAN1_IT0_IRQHandler: interrupt vector, defined in the board translation unit
// (bsp/boards/cln17_v3/board.cpp) so that exactly one object file owns it.

/**
 * @brief HAL callback: new-message events are handled by the driver ISR.
 */
// HAL_FDCAN_RxFifo0Callback: interrupt vector, defined in the board translation unit
// (bsp/boards/cln17_v3/board.cpp) so that exactly one object file owns it.

#endif  // CAN_SETUP_CLN17V3_H




