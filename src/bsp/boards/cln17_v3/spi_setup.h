/**
 * @file spi_setup.h
 * @brief SPI1 setup and DMA-driven polling for the magnetic encoder.
 *
 * The drive uses a single absolute magnetic encoder attached to the internal
 * SPI1 bus. The encoder model is selected at compile time via the
 * ENCODER_TYPE define (see "ENCODER SELECTION" below); there is no runtime
 * auto-detection.
 *
 * Encoder_Init() configures GPIO, SPI1 and DMA1; Encoder_StartTransfer()
 * launches a non-blocking burst read, and the DMA1_Channel1 ISR parses the
 * received packet and feeds the fresh angle into the FOC core through
 * POSITION_Callback(). The configured encoder type is reported over SerialUSB
 * via the TP_LOG macro (see dev_log.h).
 *
 * Hardware mapping:
 *   PA4 = CSN   (GPIO output, active LOW)
 *   PA5 = SCK   (AF5 SPI1)
 *   PA6 = MISO  (AF5 SPI1)
 *   PA7 = MOSI  (AF5 SPI1)
 *
 * Supported encoders:
 *   MT6835 (SPI mode 3: CPOL=1, CPHA=1), 6-byte burst read:
 *     TX: 0xA0 0x03 0x00 0x00 0x00 0x00 (read command + 4 dummy bytes)
 *     RX: [2] ANGLE[20:13], [3] ANGLE[12:5],
 *         [4] ANGLE[4:0] + STATUS[2:0], [5] CRC-8 (poly 0x07)
 *   TLE5012B (SPI mode 1: CPOL=0, CPHA=1), 4-byte angle read:
 *     TX: 0x80 0x20 0xFF 0xFF (read command + dummy data)
 *     RX: [2] ANGLE[14:7], [3] ANGLE[6:0]
 *
 * The angle is stored left-aligned in a 32-bit word (bit 31 = half turn) so
 * it can be used directly as a Q0.32 fixed-point turn fraction by the FOC
 * core.
 *
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#ifndef SMART_ENCODER_SETUP_H
#define SMART_ENCODER_SETUP_H

#include <Arduino.h>     // delay(), millis()
#include "stm32g4xx.h"   // CMSIS device header: SPI1, DMA1, GPIO registers
#include "lib/dev_log.h" // TP_LOG* macros: USB CDC diagnostics over SerialUSB
#include "bsp/chips/encoders/mt6835.h" // MT6835 frame decode and CRC-8 (target-agnostic)

// ==============================================================================
// ENCODER SELECTION (set manually, no runtime auto-detection)
// ==============================================================================
// Uncomment exactly ONE of the two type options below.
#define ENCODER_MT6835   1 // 21-bit absolute encoder, SPI mode 3, 6-byte frames
#define ENCODER_TLE5012B 2 // 15-bit absolute encoder, SPI mode 1, 4-byte frames

#define ENCODER_TYPE ENCODER_MT6835 // <<< CHANGE HERE to switch the encoder

// Compile-time sanity check: an unsupported selection must fail the build.
#if (ENCODER_TYPE != ENCODER_MT6835) && (ENCODER_TYPE != ENCODER_TLE5012B)
#error "spi_setup.h: ENCODER_TYPE must be either ENCODER_MT6835 or ENCODER_TLE5012B"
#endif

extern void POSITION_Callback(); // Motor-control hook, called after each new angle

static inline void Encoder_StartTransfer(); // Forward declaration: Encoder_Init() issues the first transfer

// ==============================================================================
// VARIABLES AND BUFFERS (SPI1)
// ==============================================================================
static uint8_t SPI1_RX_BUF[6] __attribute__((aligned(4))) = {0}; // RX frame filled by DMA

#if (ENCODER_TYPE == ENCODER_MT6835)
static uint8_t SPI1_TX_BUF[6] __attribute__((aligned(4))) = // MT6835 TX frame: burst-read command + dummy payload
    {0xA0, 0x03, 0x00, 0x00, 0x00, 0x00};
static volatile uint32_t POSITION_ENCODER_MT_RAW = 0; // 32-bit left-aligned angle (Q0.32 turn fraction)
static volatile uint8_t  POSITION_MT_STATUS = 0;      // STATUS[2:0]: overspeed / weak field / undervoltage
static volatile uint8_t  POSITION_MT_CRC = 0;         // CRC-8 received from the sensor

// ==============================================================================
// LINK SUPERVISION (frame validation, link-down detection, bus reset requests)
// ==============================================================================
/**
 * @brief Health of the SPI link to the MT6835, kept by the acquisition ISR.
 *
 * Every burst frame carries the sensor's CRC-8, and the DMA completion
 * handler (board.cpp) checks it before anything of the frame is published.
 * A frame that fails the check is not a position: it is a bus glitch, a
 * transfer cut short by a preempting interrupt, or a sensor that stopped
 * answering (an idle-high data line reads as the top of the range).
 * Integrating such a frame counts a whole revolution whenever the garbage
 * lands on the far side of the half-turn boundary, so a run of them walked
 * the turn counter hundreds of turns away from a stationary rotor and left
 * the position loop chasing a phantom. Bad frames are therefore dropped and
 * the last good angle stands for that control period.
 *
 * The check has to prove itself before it may drop anything: if the CRC
 * routine ever disagreed with the sensor (wrong polynomial, byte order),
 * rejecting every frame would blind the drive outright. Frames are only
 * rejected once kTrustAfter consecutive frames have passed on this boot with
 * no failure before them.
 *
 * A link that fails for kFaultRun consecutive frames is DOWN: NOTHING is
 * ingested any more, not even a frame that happens to pass (one corrupted
 * frame in 256 carries a matching CRC by chance, and on a persistently broken
 * link those few dozen a second were enough to walk the turn counter away by
 * themselves), and the ISR raises the generic encoder fault flag. What the
 * bridge does about that is the control loop's decision (lib.h: today it
 * releases the phases and refuses closed loop; a sensorless fallback or a
 * second encoder could take over there one day) - the board only RESTORES:
 * the encoder's fault action is Encoder_FaultAction(), which resets the bus,
 * and a further reset is requested once per kFaultRun failures while the
 * link stays down, so a dead sensor is retried a few hundred times a second
 * at the cost of one reset each, never in a tight loop. The link is up
 * again, and ingestion resumes, only after kTrustAfter consecutive valid
 * frames.
 *
 * This is deliberately BOARD code: what a valid frame looks like, how the
 * bus is reset and what the sensor's status bits mean are all properties of
 * this SPI link and this chip. The generic Encoder object only tracks the
 * position and carries the fault flag; it validates nothing.
 *
 * Single writer (the DMA completion ISR); the protocol reads the counters
 * from thread mode as a snapshot, torn reads are acceptable for statistics.
 */
struct EncoderLink {
    static constexpr uint32_t kTrustAfter = 256;  ///< Consecutive valid frames that prove the check / recover the link.
    static constexpr uint16_t kFaultRun = 64;     ///< Consecutive failures that mean the sensor is gone (3.2 ms at 20 kHz).

    uint32_t frames_ok = 0;       ///< Frames that passed the check since boot.
    uint32_t frames_bad = 0;      ///< Frames that failed it since boot.
    uint16_t bad_run = 0;         ///< Consecutive failures right now.
    uint16_t bad_run_max = 0;     ///< Longest failure run since boot.
    uint16_t good_run = 0;        ///< Consecutive passes while the link is proving its recovery.
    uint16_t relinks = 0;         ///< Bus resets requested since boot.
    uint8_t  sensor_status = 0;   ///< STATUS[2:0] of the last accepted frame: overspeed / weak field / undervoltage.
    bool     check_trusted = false;  ///< Check proven on this boot: failing frames are dropped.
    bool     link_down = false;      ///< Link declared down: ingestion suspended, fault flag raised by the ISR.
    bool     relink_request = false; ///< A bus reset is owed (consumed by Encoder_Relink).

    /**
     * @brief Decide whether a freshly received frame may be ingested.
     * @param valid true when the frame's CRC-8 matched.
     * @return true when the frame should be published (angle + status).
     */
    inline bool accept(bool valid) {
        if (valid) {
            frames_ok++;                                             // Statistics
            if (!check_trusted && (frames_ok >= kTrustAfter) && (frames_bad == 0u)) {
                check_trusted = true;                                // The check agrees with the sensor: arm the drop
            }
            bad_run = 0;                                             // Sensor is answering
            if (link_down) {
                if (++good_run >= kTrustAfter) {
                    link_down = false;                               // Link re-established
                    good_run = 0;
                } else {
                    return false;                                    // Still proving the recovered link
                }
            }
            return true;
        }
        frames_bad++;                                                // Statistics
        good_run = 0;                                                // A recovering link starts over
        if (bad_run < UINT16_MAX) {
            bad_run++;                                               // Length of the current failure run
        }
        if (bad_run > bad_run_max) {
            bad_run_max = bad_run;                                   // Worst run seen, for the diagnostics
        }
        if (check_trusted && (bad_run % kFaultRun) == 0u) {
            link_down = true;                                        // No valid frame for kFaultRun periods
            relink_request = true;                                   // Ask for a bus reset (once per run)
        }
        return !check_trusted;                                       // Unproven check: ingest as before
    }

};

/** The link supervisor of the encoder SPI bus (written by the DMA completion ISR). */
static EncoderLink ENCODER_LINK;
#else
static uint8_t SPI1_TX_BUF[4] __attribute__((aligned(4))) = // TLE5012B TX frame: angle-read command
    {0x80, 0x20, 0xFF, 0xFF};
static volatile uint16_t POSITION_ENCODER_TLE = 0; // 15-bit left-aligned angle
#endif

// ==============================================================================
// LOW-LEVEL CHIP SELECT CONTROL
// ==============================================================================
static inline void SPI1_ChipSelect(bool select) {
    if (select) GPIOA->BRR = GPIO_PIN_4;       // CS LOW (Active) PA4
    else        GPIOA->BSRR = GPIO_PIN_4;      // CS HIGH (Idle) PA4
}

// ==============================================================================
// SPI1 GPIO INITIALIZATION
// ==============================================================================
static inline void SPI1_GPIO_Init() {
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;      // Clock GPIOA (CS + SPI lines)

    // ---- PA4: CSN, GPIO output push-pull ----
    GPIOA->MODER &= ~(3U << (4 * 2));
    GPIOA->MODER |=  (1U << (4 * 2));         // Output mode
    GPIOA->OTYPER &= ~GPIO_OTYPER_OT4;        // Push-pull
    GPIOA->OSPEEDR |= (3U << (4 * 2));        // Very high speed
    GPIOA->PUPDR &= ~(3U << (4 * 2));         // No pull resistor
    GPIOA->BSRR = GPIO_PIN_4;                 // CS HIGH (inactive)

    // ---- PA5=SCK, PA6=MISO, PA7=MOSI: AF5 SPI1 ----
    GPIOA->MODER &= ~((3U << (5 * 2)) | (3U << (6 * 2)) | (3U << (7 * 2)));
    GPIOA->MODER |=  ((2U << (5 * 2)) | (2U << (6 * 2)) | (2U << (7 * 2))); // Alternate-function mode

    GPIOA->OTYPER &= ~(GPIO_OTYPER_OT5 | GPIO_OTYPER_OT6 | GPIO_OTYPER_OT7); // Push-pull outputs
    GPIOA->OSPEEDR |= ((3U << (5 * 2)) | (3U << (6 * 2)) | (3U << (7 * 2))); // Very high speed
    // Important: disable pull resistors on SPI lines in normal operation
    GPIOA->PUPDR &= ~((3U << (5 * 2)) | (3U << (6 * 2)) | (3U << (7 * 2)));

    GPIOA->AFR[0] &= ~((0xFU << (5 * 4)) | (0xFU << (6 * 4)) | (0xFU << (7 * 4)));
    GPIOA->AFR[0] |=  ((5U   << (5 * 4)) | (5U   << (6 * 4)) | (5U   << (7 * 4))); // AF5 = SPI1
}

// ==============================================================================
// SPI1 INITIALIZATION
// ==============================================================================
static inline void SPI1_Init() {
    SPI1_GPIO_Init();

    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;                       // Clock SPI1
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMAMUX1EN; // Clock DMA1 and its request router

    // DMAMUX1 routing: SPI1_RX = 10, SPI1_TX = 11
    DMAMUX1_Channel0->CCR = 10U;
    DMAMUX1_Channel1->CCR = 11U;

    // Configure DMA Channel 1 (RX)
    DMA1_Channel1->CCR &= ~DMA_CCR_EN;            // Disabled before reconfiguration
    DMA1_Channel1->CPAR = (uint32_t)&SPI1->DR;    // Peripheral: SPI1 data register
    DMA1_Channel1->CMAR = (uint32_t)SPI1_RX_BUF;  // Memory: RX frame buffer
    DMA1_Channel1->CCR = DMA_CCR_MINC | DMA_CCR_TCIE | (3U << DMA_CCR_PL_Pos); // Increment, transfer-complete IRQ, high priority

    // Configure DMA Channel 2 (TX)
    DMA1_Channel2->CCR &= ~DMA_CCR_EN;            // Disabled before reconfiguration
    DMA1_Channel2->CPAR = (uint32_t)&SPI1->DR;    // Peripheral: SPI1 data register
    DMA1_Channel2->CMAR = (uint32_t)SPI1_TX_BUF;  // Memory: TX command frame
    DMA1_Channel2->CCR = DMA_CCR_MINC | DMA_CCR_DIR | (2U << DMA_CCR_PL_Pos); // Read from memory, medium-high priority

    // SPI master, software NSS, 8-bit frame with RXNE pulse on byte boundary
    // (FRXTH), DMA requests enabled.
    SPI1->CR2 = SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN | SPI_CR2_FRXTH | (7U << SPI_CR2_DS_Pos);

#if (ENCODER_TYPE == ENCODER_MT6835)
    // SPI mode 3 (CPOL=1, CPHA=1) as required by the MT6835.
    //
    // BR=3: PCLK2 / 16 = 10.6 MHz at 170 MHz (the sensor takes up to 16 MHz).
    // The clock sets WHEN the control pass can start: the pass runs on the
    // completion of this 6-byte frame, and at the former 2.66 MHz (BR=5) the
    // frame took ~20 us of the 50 us period, after which the ~27 us pass
    // ended right on the next period boundary. Whether it finished just
    // before or just after decided whether the duty it computed reached the
    // bridge one or two periods later, and that per-pass 1 <-> 2 period
    // jitter of the loop delay - not any one build, mode or interrupt -
    // destabilised the current loop on long profiled moves (the extra
    // executor work pushed the end past the boundary on some passes only).
    // At 10.6 MHz the frame is ~5 us and the pass ends ~18 us before the
    // boundary, so the delay is one period every time. The CRC check on
    // every frame (EncoderLink) guards the faster clock: a bad frame is
    // dropped and counted in 0x43/0x07.
    SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_CPOL | SPI_CR1_CPHA | (3U << SPI_CR1_BR_Pos);
#else
    // SPI mode 1 (CPOL=0, CPHA=1) as required by the TLE5012B
    SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_CPHA | (5U << SPI_CR1_BR_Pos);
#endif

    // Clear stale flags and leftover data
    (void)SPI1->DR;
    (void)SPI1->SR;
    DMA1->IFCR = DMA_IFCR_CGIF1 | DMA_IFCR_CGIF2;

    SPI1_ChipSelect(false);   // Deassert CS before enabling SPI
    SPI1->CR1 |= SPI_CR1_SPE; // Enable the peripheral
}

// ==============================================================================
// INITIALIZATION (COMPILE-TIME SELECTED ENCODER)
// ==============================================================================
static inline void Encoder_Init() {
    delay(20); // Let the encoder stabilize after power-up

    SPI1_Init();

    // DMA transfer-complete interrupt drives the control loop. Priority 2:
    // below the protections (0), the control timer (0) and the ADC data (1),
    // above USB (moved to 4 in run.h) and the bus. Nothing a host does can
    // stretch a pass, so the pass ends at the same point of every period.
    NVIC_DisableIRQ(DMA1_Channel1_IRQn);
    NVIC_ClearPendingIRQ(DMA1_Channel1_IRQn);

    NVIC_SetPriority(
        DMA1_Channel1_IRQn,
        NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 2, 0)
    );

    NVIC_EnableIRQ(DMA1_Channel1_IRQn);

#if (ENCODER_TYPE == ENCODER_MT6835)
    TP_LOG("ENCODER: MT6835 on SPI1 (compile-time selected)"); // Report the configured encoder
#else
    TP_LOG("ENCODER: TLE5012B on SPI1 (compile-time selected)"); // Report the configured encoder
#endif

    // Optional: first request to fill in the position before starting FOC.
    Encoder_StartTransfer();
}

// ==============================================================================
// DMA TRANSACTION START
// ==============================================================================
/**
 * @brief Starts a single burst-read of the encoder via SPI1 + DMA.
 *
 * Frame length depends on the compile-time selected encoder:
 * 6 bytes (mode 3) for the MT6835, 4 bytes (mode 1) for the TLE5012B.
 *
 * Important: start the next transfer only after the previous one completes.
 * Normally called from the control timer; the DMA IRQ invokes POSITION_Callback().
 */
static volatile uint32_t ENCODER_OVERRUNS = 0;   // Control periods that found the previous transfer still in flight
static volatile bool     ENCODER_RELINKING = false; // Bus reset in progress: no transfer may start

static inline void Encoder_StartTransfer() {
    // A transfer still in flight (CS still asserted: the completion handler
    // has not released the bus yet, or a bus reset is under way) must not be
    // restarted. Cutting a byte in half leaves a partial byte in the SPI
    // receive FIFO and every following frame arrives shifted by one byte,
    // failing its CRC until the peripheral is reset - which is exactly how a
    // long control pass (a flash stall, a calibration step) turned into a
    // permanently corrupted encoder link. Skip this period instead: the
    // control pass repeats the last good angle, and the overrun is counted.
    if (ENCODER_RELINKING || ((GPIOA->ODR & GPIO_PIN_4) == 0U)) {
        ENCODER_OVERRUNS++;                // Previous frame not yet consumed
        return;
    }
    SPI1_ChipSelect(true);                 // Assert CS to frame the transaction
    DMA1_Channel1->CCR &= ~DMA_CCR_EN;     // Stop RX channel
    DMA1_Channel2->CCR &= ~DMA_CCR_EN;     // Stop TX channel

    DMA1->IFCR = DMA_IFCR_CGIF1 | DMA_IFCR_CGIF2; // Clear stale completion flags

#if (ENCODER_TYPE == ENCODER_MT6835)
    DMA1_Channel1->CNDTR = 6;              // 6 bytes to receive
    DMA1_Channel2->CNDTR = 6;              // 6 bytes to transmit
#else
    DMA1_Channel1->CNDTR = 4;              // 4 bytes to receive
    DMA1_Channel2->CNDTR = 4;              // 4 bytes to transmit
#endif

    // RX is enabled first, otherwise the first received byte may be lost.
    DMA1_Channel1->CCR |= DMA_CCR_EN;
    DMA1_Channel2->CCR |= DMA_CCR_EN;
}

// ==============================================================================
// BUS RESET (re-establish a corrupted link)
// ==============================================================================
/**
 * @brief Reset the encoder SPI link: abort any transfer, flush the receive
 *        FIFO, clear the DMA flags and re-enable the peripheral.
 *
 * Called through the encoder's fault action the moment EncoderLink::accept()
 * declares the link down, then from the completion handler once per further
 * run of failures while it stays down. A byte left in the SPI receive FIFO
 * shifts every later frame; disabling the SPI and draining the FIFO is the
 * only way to realign it. The next timer trigger starts a fresh transfer.
 * Cheap (a few microseconds) and idempotent, so it is safe to repeat while
 * the sensor stays silent. Consumes the supervisor's pending request, so a
 * reset already performed by the fault action is not repeated by the ISR.
 */
static inline void Encoder_Relink() {
#if (ENCODER_TYPE == ENCODER_MT6835)
    ENCODER_LINK.relink_request = false;                       // This reset serves the pending request
    if (ENCODER_LINK.relinks < UINT16_MAX) {
        ENCODER_LINK.relinks++;                                // Bus resets since boot
    }
#endif
    ENCODER_RELINKING = true;                                  // Hold off the timer-triggered start
    DMA1_Channel1->CCR &= ~DMA_CCR_EN;                         // Stop RX channel
    DMA1_Channel2->CCR &= ~DMA_CCR_EN;                         // Stop TX channel
    SPI1->CR1 &= ~SPI_CR1_SPE;                                 // Disable: aborts the shift register
    while (SPI1->SR & SPI_SR_FRLVL) {                          // Drain whatever the receive FIFO holds
        (void)*(volatile uint8_t*)&SPI1->DR;                   // Byte-wide read pops one entry
    }
    (void)SPI1->SR;                                            // Clear OVR (read DR, then SR)
    DMA1->IFCR = DMA_IFCR_CGIF1 | DMA_IFCR_CGIF2;              // Clear stale completion flags
    SPI1_ChipSelect(false);                                    // Release the bus
    SPI1->CR1 |= SPI_CR1_SPE;                                  // Enable the peripheral again
    ENCODER_RELINKING = false;                                 // Transfers may start again
}

/**
 * @brief The encoder's fault action: restore the link.
 *
 * Registered as Encoder::fault_cb, so it runs once on every rising edge of
 * the encoder fault flag, from the acquisition ISR. It is purely a RESTORE
 * hook - the board re-initialises what it owns, the SPI bus - and touches
 * nothing of the power stage: the control loop reads the same flag and
 * decides what the bridge does. A flag raised by the sensor's own status
 * bits (weak field, undervoltage) on a healthy link needs no bus reset.
 */
static inline void Encoder_FaultAction() {
#if (ENCODER_TYPE == ENCODER_MT6835)
    if (ENCODER_LINK.link_down) {
        Encoder_Relink();                                      // First reset, at once
    }
#endif
}

/** Control periods skipped because the previous transfer was still in flight. */
static inline uint32_t Encoder_Overruns() { return ENCODER_OVERRUNS; }

// ==============================================================================
// ANGLE GETTER (32-bit Left Aligned)
// ==============================================================================
// Returns the angle in left-aligned 32-bit format:
// 0x00000000 = 0 deg
// 0x80000000 = 180 deg
// 0xFFFFF800 = nearly 360 deg
static inline uint32_t Encoder_GetAngle() {
#if (ENCODER_TYPE == ENCODER_MT6835)
    return POSITION_ENCODER_MT_RAW;
#else
    return ((uint32_t)POSITION_ENCODER_TLE) << 16;
#endif
}

#if (ENCODER_TYPE == ENCODER_MT6835)
static inline uint8_t Encoder_GetMTStatus() {
    return POSITION_MT_STATUS;
}

static inline uint8_t Encoder_GetMTCRC() {
    return POSITION_MT_CRC;
}
#endif

// ==============================================================================
// DMA RX COMPLETE IRQ
// ==============================================================================
// DMA1_Channel1_IRQHandler: interrupt vector, defined in the board translation unit
// (bsp/boards/cln17_v3/board.cpp) so that exactly one object file owns it.

#endif // SMART_ENCODER_SETUP_H

