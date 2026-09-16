/**
 * @file ripple_map.h
 * @brief Torque-ripple feed-forward map indexed by electrical angle ("anti-cogging").
 *
 * A hybrid stepper produces torque ripple that FOC does not remove: the
 * detent torque at four cycles per electrical period and the A/B stator
 * asymmetry at two, plus a smaller one-per-period term. At low speed the
 * position loop cannot reject a disturbance at 50-200 Hz, so the rotor rides
 * the ripple (an idling-engine growl) and the loop chases it with current.
 * Measured on the bench at 0.5 RPS: 50 mA at 2x, 24 mA at 4x, 10 mA at 1x,
 * with a +-0.12 deg position ripple.
 *
 * The ripple is periodic in ELECTRICAL angle (the pole pairs are near enough
 * identical), so one table of 2^BITS bins per electrical period captures every
 * harmonic up to 2^(BITS-1) and covers the whole revolution. The table holds
 * the q-axis current [mA] to add to the loop's command at that angle.
 *
 * Learning is closed-loop and iterative: while the motor cruises slowly under
 * position control, the loop's own command (BEFORE the feed-forward) is
 * averaged per bin. Its per-bin mean minus the mean over all bins is the
 * current the loop had to spend on the ripple at that angle; adding it to the
 * table lets the feed-forward supply it directly, and the loop's output drops
 * towards the DC (friction) term. A second pass learns the residual and adds
 * it, so the phase lag of the loop's reaction converges out.
 *
 * Concurrency: every table and accumulator mutation happens in the control
 * ISR (learn(), service()); thread mode only posts a request through one
 * atomic byte. Reads of the table from thread mode (diagnostics) may see a
 * commit in flight, which is harmless for a diagnostic view.
 *
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */
#pragma once

#include <stdint.h>  // Fixed-width integers
#include <atomic>    // std::atomic for the thread -> ISR request byte

/**
 * @tparam BITS log2 of the number of bins per electrical period (6 = 64 bins).
 */
template <uint8_t BITS = 6>
class RippleMap {
public:
    static constexpr uint16_t kBins = (uint16_t)(1u << BITS);         ///< Bins per electrical period.
    static constexpr uint16_t kShift = (uint16_t)(16u - BITS);        ///< Angle bits above the bin index.
    static constexpr uint16_t kFracMask = (uint16_t)((1u << kShift) - 1u);  ///< Angle bits within a bin.
    static constexpr int32_t kLimitMa = 600;                          ///< Table clamp [mA], per bin.

    /** @brief Requests thread mode may post; the ISR consumes them in service(). */
    enum Request : uint8_t {
        None = 0,    ///< Nothing pending.
        Learn = 1,   ///< Clear the accumulators and start averaging the loop's command.
        Commit = 2,  ///< Stop learning, fold the accumulated residual into the table, enable it.
        Clear = 3,   ///< Zero the table and disable the feed-forward.
    };

    // ---------------------------------------------------------------------
    // Thread mode
    // ---------------------------------------------------------------------

    /** @brief Post a request for the ISR; a later post replaces an unserviced one. */
    void post(Request r) { request_.store((uint8_t)r, std::memory_order_release); }

    /** @brief true while the feed-forward is added to the loop's command. */
    bool is_enabled() const { return enabled_; }

    /** @brief true while the loop's command is being averaged per bin. */
    bool is_learning() const { return learning_; }

    /** @brief Feed-forward current of one bin [mA] (diagnostic view). */
    int16_t bin_ma(uint16_t index) const { return table_[index & (kBins - 1u)]; }

    /** @brief Samples averaged since the last Learn request (diagnostic view). */
    uint32_t samples() const { return total_samples_; }

    // ---------------------------------------------------------------------
    // Control ISR
    // ---------------------------------------------------------------------

    /** @brief Consume a posted request. Call once per control pass. */
    void service() {
        const uint8_t r = request_.exchange((uint8_t)None, std::memory_order_acq_rel);  // Take it
        switch (r) {
            case Learn:
                begin_learn();
                break;
            case Commit:
                commit();
                break;
            case Clear:
                clear();
                break;
            default:
                break;  // Nothing posted
        }
    }

    /**
     * @brief Accumulate the loop's own command for this pass.
     * @param angle_el Electrical angle, 65536 per period (the FOC frame's).
     * @param loop_a   Current the position loop asked for, FOC frame [A],
     *                 WITHOUT the feed-forward (else the table would double).
     */
    void learn(uint16_t angle_el, float loop_a) {
        if (!learning_) {
            return;  // Not averaging
        }
        // Nearest bin: round the angle to the bin centre before truncating.
        const uint16_t i = (uint16_t)((uint16_t)(angle_el + (uint16_t)(1u << (kShift - 1u))) >> kShift) & (kBins - 1u);
        sum_[i] += (int32_t)(loop_a * 1000.0f);  // [mA], float -> int32 is one FPU instruction
        cnt_[i]++;
        total_samples_++;
    }

    /**
     * @brief Feed-forward current for this angle [A], linearly interpolated
     *        between the two neighbouring bins. Zero while disabled.
     */
    float feed_forward_a(uint16_t angle_el) const {
        if (!enabled_) {
            return 0.0f;  // Table not in use
        }
        const uint16_t i = (uint16_t)(angle_el >> kShift);          // Bin at or below the angle
        const uint16_t j = (uint16_t)((i + 1u) & (kBins - 1u));     // Next bin, wrapping the period
        const int32_t f = (int32_t)(angle_el & kFracMask);          // Position within the bin
        const int32_t v = (int32_t)table_[i] * ((1 << kShift) - f) + (int32_t)table_[j] * f;  // mA << kShift
        return (float)v * (0.001f / (float)(1 << kShift));          // -> amperes
    }

private:
    /** @brief Zero the accumulators and start averaging. */
    void begin_learn() {
        for (uint16_t i = 0; i < kBins; ++i) {
            sum_[i] = 0;
            cnt_[i] = 0;
        }
        total_samples_ = 0;
        learning_ = true;
    }

    /**
     * @brief Fold the accumulated residual into the table and enable it.
     *
     * Requires every bin to have been visited, so a pass that did not cover
     * a full electrical period leaves the table untouched. The mean over the
     * bins (friction and load) is removed: only the ripple goes in.
     */
    void commit() {
        learning_ = false;
        int32_t mean[kBins];  // Per-bin mean of the loop's command [mA]
        int32_t acc = 0;      // Sum of the per-bin means
        for (uint16_t i = 0; i < kBins; ++i) {
            if (cnt_[i] == 0u) {
                return;  // Incomplete coverage: keep the table as it is
            }
            mean[i] = sum_[i] / (int32_t)cnt_[i];
            acc += mean[i];
        }
        const int32_t dc = acc / (int32_t)kBins;  // What the loop spends on average: not ripple
        for (uint16_t i = 0; i < kBins; ++i) {
            int32_t v = (int32_t)table_[i] + mean[i] - dc;  // Previous table plus this pass's residual
            if (v > kLimitMa) {
                v = kLimitMa;  // Clamp: a runaway learn cannot command more than this
            } else if (v < -kLimitMa) {
                v = -kLimitMa;
            }
            table_[i] = (int16_t)v;
        }
        enabled_ = true;
    }

    /** @brief Forget the table and stop feeding forward. */
    void clear() {
        learning_ = false;
        enabled_ = false;
        for (uint16_t i = 0; i < kBins; ++i) {
            table_[i] = 0;
        }
    }

    int16_t table_[kBins] = {};           ///< Feed-forward current per bin [mA].
    int32_t sum_[kBins] = {};             ///< Learning: summed loop command per bin [mA].
    uint32_t cnt_[kBins] = {};            ///< Learning: samples per bin.
    uint32_t total_samples_ = 0;          ///< Learning: samples in total (diagnostics).
    bool enabled_ = false;                ///< Feed-forward active.
    bool learning_ = false;               ///< Accumulating.
    std::atomic<uint8_t> request_{(uint8_t)None};  ///< Thread -> ISR request.
};
