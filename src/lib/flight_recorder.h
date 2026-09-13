/**
 * @file flight_recorder.h
 * @brief Control-rate black box: the last N control passes before a fault, readable over the protocol.
 *
 * Everything a bench can read over the protocol is a snapshot taken a few
 * hundred times a second from thread mode, while the events that trip a
 * protection (a current-loop excursion, an angle glitch, a voltage spike)
 * live and die inside a handful of 50 us control passes. This ring keeps
 * one record per pass - the electrical angle, the measured d/q currents,
 * the commanded d/q voltages, the commanded q current and the position
 * error - and FREEZES itself a short while after the drive's fault byte
 * becomes non-zero, so the passes leading up to the trip, and the first ones
 * after it, can be pulled out afterwards through register 0x44 (see
 * CAN_PROTOCOL.md) at whatever pace the link allows.
 *
 * Cost is RAM only (16 bytes per record) and one struct copy per pass, so it
 * is compiled in only when TP_TRACE_DEPTH (the record count) is defined -
 * the cln17_v3_dev environment does that; the release image carries no
 * recorder and answers 0x44 with NOT_IMPLEMENTED.
 *
 * Single writer (the control ISR); the reader in thread mode looks at a
 * frozen ring, so it never races the writer for the interesting data. The
 * writer thaws only through thaw(), which the fault reset calls.
 *
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#pragma once

#include <stdint.h>  // Fixed-width integer types

/** One control pass, 24 bytes. Units chosen so int16 covers every sane value. */
struct FlightRecord {
    uint16_t angle_el;  ///< Electrical angle used by the current loop, 65536 per period.
    int16_t  iq_ma;     ///< Measured q-axis current [mA].
    int16_t  id_ma;     ///< Measured d-axis current [mA].
    int16_t  uq_cv;     ///< Commanded q-axis voltage [10 mV].
    int16_t  ud_cv;     ///< Commanded d-axis voltage [10 mV].
    int16_t  icmd_ma;   ///< q-axis current commanded to the loop this pass [mA] (FOC frame).
    int16_t  ia_ma;     ///< Coil A current as the loop sees it [mA] (after the KCL recombination).
    int16_t  ib_ma;     ///< Coil B current as the loop sees it [mA].
    int16_t  ua_cv;     ///< Commanded alpha (coil A) voltage handed to the bridge [10 mV].
    int16_t  ub_cv;     ///< Commanded beta (coil B) voltage handed to the bridge [10 mV].
    int16_t  vbus_cv;   ///< Supply voltage [10 mV].
    int16_t  perr_mrev; ///< Position error, target - measured [1/1000 rev], saturating.
};
static_assert(sizeof(FlightRecord) == 24, "FlightRecord must stay 24 bytes: 0x44 reads it in six parts");

/**
 * @brief Ring of the last N control passes, frozen shortly after a fault.
 * @tparam N Record count (a power of two keeps the index arithmetic to a mask).
 */
template <uint16_t N>
class FlightRecorder {
    static_assert((N & (N - 1u)) == 0u, "Depth must be a power of two");
    static constexpr uint16_t kPostTrigger = N / 8u;  ///< Passes kept after the fault edge before freezing.

public:
    /**
     * @brief Store one pass. Called from the control ISR.
     * @param r     The record.
     * @param fault The drive's fault byte this pass (0 = healthy).
     */
    inline void push(const FlightRecord& r, uint8_t fault) {
        if (frozen_) {
            return;                                    // Holding the trip for the reader
        }
        ring_[w_] = r;                                 // Overwrite the oldest slot
        w_ = (uint16_t)((w_ + 1u) & (N - 1u));
        if (count_ < N) {
            count_++;                                  // Valid records so far (saturates at N)
        }
        if (post_ != 0u) {
            if (--post_ == 0u) {
                frozen_ = true;                        // Post-trigger window done: freeze
            }
        } else if (fault != 0u) {
            freeze_fault_ = fault;                     // Remember what tripped
            post_ = kPostTrigger;                      // Keep a few passes of the aftermath
        }
    }

    /// Release a frozen ring so it records again (the operator's fault reset).
    inline void thaw() {
        frozen_ = false;
        post_ = 0u;
        freeze_fault_ = 0u;
    }

    /// True while the ring holds a trip for the reader.
    inline bool frozen() const { return frozen_; }
    /// Fault byte that froze the ring (0 when not frozen).
    inline uint8_t freeze_fault() const { return freeze_fault_; }
    /// Number of valid records (saturates at the depth).
    inline uint16_t count() const { return count_; }
    /// Depth of the ring.
    static constexpr uint16_t depth() { return N; }
    /// Number of records after the fault edge that the freeze keeps.
    static constexpr uint16_t post_trigger() { return kPostTrigger; }

    /**
     * @brief Read one record, 0 = oldest valid, count()-1 = newest.
     * @param index Record index in chronological order.
     * @param out   Receives the record.
     * @return false when the index is beyond the valid records.
     */
    inline bool read(uint16_t index, FlightRecord& out) const {
        if (index >= count_) {
            return false;
        }
        const uint16_t oldest = (count_ < N) ? 0u : w_;   // Slot of the oldest record
        out = ring_[(uint16_t)((oldest + index) & (N - 1u))];
        return true;
    }

private:
    FlightRecord ring_[N];                             ///< The records.
    uint16_t w_ = 0;                                   ///< Next slot to write.
    uint16_t count_ = 0;                               ///< Valid records.
    uint16_t post_ = 0;                                ///< Post-trigger passes still to record.
    uint8_t  freeze_fault_ = 0;                        ///< Fault byte at the trigger.
    bool     frozen_ = false;                          ///< Recording stopped, ring readable.
};
