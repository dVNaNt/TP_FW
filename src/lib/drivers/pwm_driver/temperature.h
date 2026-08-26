/**
 * @file temperature.h
 * @brief MCU temperature sensing with a minimal integer low-pass filter and
 *        single-threshold overtemperature (OT) protection released by
 *        hysteresis.
 *
 * @details
 * This module mirrors the architecture of voltage.h, adapted for the thermal
 * signal whose dynamics are orders of magnitude slower than the electrical
 * ones:
 *
 * ```
 * Raw ADC -> [Atomic State Register] -> tick() -> Fast Integer EMA -> OT check
 *                                                     |
 *                      trip:    y >= ot_trip_raw_     -> FAULT_OVERHEAT
 *                      release: y <= ot_release_raw_  (hysteresis)
 * ```
 *
 * ## Bare-minimum filtering (u12 + u4 remainder)
 *
 * The incoming sample is a plain uint16 treated as Q12.4 fixed point: 12
 * integer bits plus a 4-bit fractional remainder. The exponential moving
 * average advances one integer step per tick, right inside tick():
 *
 *     acc += x - y            // total pending correction (+ carried rest)
 *     step = acc >> 12        // u12: whole part of err / 4096
 *     acc -= step << 12       // u4 : keep the division remainder for later
 *     y   += step
 *
 * The remainder accumulator makes the truncated division exact over time --
 * no fractional precision is ever lost, yet the hot path contains only one
 * subtract, two shifts, and three adds. No multiplications, no divisions,
 * no floating point, no branches, no helper class. Time constant is
 * 2^12 = 4096 ticks (~205 ms at the nominal 20 kHz control rate).
 *
 * ## Single limit with hysteresis release
 *
 * Only **ot** (overtemperature) remains: it sets the @c overheat proxy,
 * which DriverPWM maps onto FAULT_OVERHEAT (priority 1, Hi-Z safe state).
 * The fault is released not by the same threshold but by a lower one:
 *
 *     activate:   y >= ot_trip_raw_      (limit)
 *     deactivate: y <= ot_release_raw_   (limit - hysteresis)
 *
 * Thresholds are precomputed ONCE in the raw ADC domain when the limit or
 * the conversion callback is configured (float/binary-search math runs only
 * there, from the slow application context). The runtime path in tick() is
 * therefore pure integer comparison.
 *
 * @note Raw input convention matches the rest of the PHY layer: 16-bit
 *       left-aligned ADC words where full scale corresponds to
 *       @c max_temp_celsius (linear mode) or the factory-calibration curve
 *       (callback mode).
 *
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#pragma once

#include <atomic>
#include "../../common.h"

struct DriverPWM;

/**
 * @brief Callback that converts a raw filtered ADC word into degrees Celsius.
 *
 * Installed by the board layer (e.g. factory-calibration formula from
 * adc_setup.h). Receives the FILTERED raw ADC value in the same left-aligned
 * format as set_temperature_irq() input. It is invoked lazily from
 * get_temperature_c() (slow diagnostic paths only), never from tick().
 */
typedef float (*Temperature_Convert_Callback)(uint32_t raw_adc);

/**
 * @struct Temperature
 * @brief Filtered temperature sensing with a single hysteresis-released
 *        overtemperature threshold.
 *
 * Owns the temperature pipeline: raw ADC ingestion (atomic state register,
 * ISR-safe), minimal integer EMA filtering and the overheat protection
 * proxy (@c overheat).
 *
 * Conversion to degC:
 * - default: linear full-scale scaling by max_temp_celsius;
 * - when a conversion callback is installed via set_temperature_callback(),
 *   the callback maps the filtered raw value to degC (e.g. STM32 TS_CAL1/
 *   TS_CAL2 factory calibration). Limits are inverted into the raw domain
 *   numerically at configuration time.
 *
 * Usage (via DriverPWM):
 * @code
 *   driver.temperature.set_limit_celsius().ot_lim_celsius(85.0f).ot_hyst_celsius(10.0f);
 *   float t = driver.temperature.get_temperature_c();
 *   bool hot = driver.temperature.overheat.is_active();
 * @endcode
 */
struct Temperature {
    friend struct DriverPWM;

private:
    // Atomic state: combined raw ADC value and update flag
    // Bit 31: is_read flag (1 = data consumed, 0 = new data available)
    // Bits 0-15: raw_adc value (16-bit left-aligned word)
    std::atomic<uint32_t> temp_state_{FLAG_READ};
    static constexpr uint32_t FLAG_READ = 0x80000000;  ///< "data consumed" marker bit
    static constexpr uint32_t MASK_DATA = 0x0000FFFF;  ///< Raw sample field mask

    int32_t ema_acc_ = 0;     ///< EMA accumulator: pending error incl. carried u4 remainder
    uint16_t temp_raw_ = 0;   ///< Latest filtered raw ADC word (u16, left-aligned)
    bool primed_ = false;     ///< false until the first sample seeds the filter
    const float max_temp_c_;    ///< degC corresponding to ADC full scale (linear mode)

    Temperature_Convert_Callback convert_cb_ = nullptr;  ///< Optional raw -> degC callback

    float ot_limit_c_ = INFINITY;  ///< Configured OT limit [degC] (INFINITY = disabled)
    float ot_hyst_c_ = 0.0f;       ///< Release hysteresis band [degC]

    uint16_t ot_trip_raw_ = UINT16_MAX;    ///< Activation threshold, raw domain (UINT16_MAX = disabled)
    uint16_t ot_release_raw_ = 0;          ///< Hysteresis release threshold, raw domain

    /**
     * @brief Find the smallest raw code whose converted temperature >= target.
     *
     * Runs once per configuration change from the slow application context,
     * never from the control ISR. Assumes the conversion is monotonically
     * increasing with the raw code (true for the STM32 factory calibration
     * and for NTC beta models alike).
     *
     * @param target Temperature threshold in degC
     * @return Raw left-aligned 16-bit threshold
     */
    uint16_t celsius2raw(float target) const {
        if (convert_cb_ != nullptr) {
            // Binary search over the 16-bit raw domain using the real curve
            uint32_t lo = 0, hi = 0xFFFF;
            while (lo < hi) {
                uint32_t mid = (lo + hi) >> 1;                // Mid-point probe
                if (convert_cb_(mid) < target) lo = mid + 1;  // Still cold -> go up
                else hi = mid;                                // Hot enough -> keep mid
            }
            return (uint16_t)lo;
        }
        // Linear full-scale fallback: raw = frac * 65535
        if (max_temp_c_ <= 0.0f) return UINT16_MAX;
        int32_t raw = (int32_t)((target / max_temp_c_) * 65535.0f);
        return (uint16_t)((raw < 0) ? 0 : ((raw > 65535) ? 65535 : raw));
    }

    /// Recompute both raw thresholds after any limit/hysteresis change.
    void recompute_thresholds() {
        if (ot_limit_c_ == INFINITY) {   // Protection disabled
            ot_trip_raw_ = UINT16_MAX;   // Never trips
            ot_release_raw_ = 0;         // Always releases
            return;
        }
        ot_trip_raw_ = celsius2raw(ot_limit_c_);          // Activation level
        float release_c = ot_limit_c_ - ot_hyst_c_;       // Lower release level
        uint16_t release_raw = celsius2raw(release_c);    // Inverted release level
        ot_release_raw_ = (release_raw > ot_trip_raw_) ? ot_trip_raw_ : release_raw;  // Sanity clamp
    }

    /**
     * @struct TempProtectionProxy
     * @brief Hysteresis-released flag for the overtemperature threshold.
     *
     * There is deliberately NO recovery timer: activation and release are
     * governed solely by the two raw thresholds evaluated every tick().
     */
    struct TempProtectionProxy {
        friend struct Temperature;

    private:
        std::atomic<uint8_t> active_{0};  ///< 1 = threshold currently exceeded

        /// Activate the flag (threshold exceeded). ISR-safe.
        inline void activate() { active_.store(1, std::memory_order_relaxed); }

        /// Deactivate the flag (hysteresis released). ISR-safe.
        inline void deactivate() { active_.store(0, std::memory_order_relaxed); }

    public:
        /// Default constructor: protection inactive.
        TempProtectionProxy() = default;

        /// Tick-parity hook mirroring VoltageProtectionProxy::tick_timer().
        inline bool tick_timer() const { return active_.load(std::memory_order_relaxed) != 0; }

        /// @return true while the overtemperature condition is active.
        inline bool is_active() const { return active_.load(std::memory_order_relaxed) != 0; }
    };

public:
    /** Public proxy evaluated by Temperature::tick(). */
    TempProtectionProxy overheat;  ///< OT threshold: mapped to FAULT_OVERHEAT (Hi-Z)

    /**
     * @brief Construct a Temperature instance.
     * @param max_temp_celsius Full-scale temperature in degC (linear mode).
     */
    explicit Temperature(float max_temp_celsius = 150.0f)
        : max_temp_c_(max_temp_celsius) {}

    /**
     * @brief Main processing tick — called every cycle by DriverPWM::tick().
     *
     * Atomically reads and marks the state register, advances the integer
     * EMA filter and re-evaluates the hysteresis thresholds with pure
     * integer comparisons.
     *
     * @return true if a new temperature sample was processed in this tick
     */
    bool tick() {
        // Atomically read state AND set the read flag in one operation
        uint32_t state = temp_state_.fetch_or(FLAG_READ, std::memory_order_relaxed);
        if (state & FLAG_READ) return false;  // No new sample since last tick

        uint16_t raw_adc = (uint16_t)(state & MASK_DATA);  // Extract the fresh sample

        if (!primed_) {
            // First sample seeds the filter directly, skipping the convergence transient
            temp_raw_ = raw_adc;
            ema_acc_ = 0;
            primed_ = true;
        } else {
            // Q12.4 shift-EMA inlined (no helper class): 1 sub, 2 shifts, 3 adds.
            ema_acc_ += (int32_t)raw_adc - (int32_t)temp_raw_;  // Pending error + carried rest
            int32_t step = ema_acc_ >> 12;                      // u12: whole part of err / 4096
            ema_acc_ -= step << 12;                             // u4: keep the division remainder
            temp_raw_ = (uint16_t)((int32_t)temp_raw_ + step);  // Apply the integer step
        }

        // Hysteresis comparator in the raw domain (integer only):
        // trip at the limit, release only after dropping below limit-hysteresis.
        if (!overheat.is_active()) {
            if (temp_raw_ >= ot_trip_raw_) overheat.activate();       // Crossed the trip level upward
        } else {
            if (temp_raw_ <= ot_release_raw_) overheat.deactivate();  // Fell below the release level
        }
        return true;
    }

    /**
     * @brief Latest filtered temperature in degrees Celsius (lazy conversion).
     *
     * Intended for slow diagnostic paths (USB/CAN telemetry); performs the
     * float conversion on demand instead of burning cycles every tick.
     *
     * @return Filtered temperature [degC]
     */
    inline float get_temperature_c() const {
        if (convert_cb_ != nullptr) return convert_cb_(temp_raw_);  // Calibration-curve mode
        return ((float)temp_raw_) * (max_temp_c_ * (1.0f / 65535.0f));  // Linear full-scale mode
    }


    /**
     * @brief Atomically set raw ADC temperature and mark data as new.
     *
     * Stores the ADC value and clears bit 31 (FLAG_READ) to indicate new
     * data. Safe to call from any interrupt context; a single relaxed store.
     *
     * @param raw_adc Raw 16-bit left-aligned ADC value
     */
    inline void set_temperature_irq(uint32_t raw_adc) {
        temp_state_.store(raw_adc & MASK_DATA, std::memory_order_relaxed);
    }

    /**
     * @brief Set the raw -> degC conversion callback (callback setter).
     *
     * Passing nullptr restores the default linear full-scale scaling.
     * Thresholds are re-inverted immediately so the call order relative to
     * set_limit_celsius() does not matter.
     *
     * @param cb Conversion function pointer (nullptr = linear scaling)
     */
    inline void set_temperature_callback(Temperature_Convert_Callback cb) {
        convert_cb_ = cb;
        recompute_thresholds();  // Re-derive raw limits against the new curve
    }

    /**
     * @brief Chainable limit setter for configuring the OT threshold in degC.
     *
     * Usage:
     * @code
     *   temperature.set_limit_celsius().ot_lim_celsius(85.0f).ot_hyst_celsius(10.0f);
     * @endcode
     */
    struct LimitProxy {
    private:
        Temperature& parent;  ///< Reference to the owning Temperature

    public:
        /**
         * @brief Construct a LimitProxy bound to a Temperature.
         * @param p Reference to the owning Temperature
         */
        LimitProxy(Temperature& p) : parent(p) {}

        /**
         * @brief Set the overtemperature trip limit.
         * @param limit_celsius Overtemperature threshold in degC
         * @return Reference to this setter for chaining
         */
        inline LimitProxy& ot_lim_celsius(float limit_celsius) {
            parent.ot_limit_c_ = limit_celsius;
            parent.recompute_thresholds();
            return *this;
        }

        /**
         * @brief Set the hysteresis band used to release the OT fault.
         * @param hyst_celsius Release gap below the trip limit in degC
         * @return Reference to this setter for chaining
         */
        inline LimitProxy& ot_hyst_celsius(float hyst_celsius) {
            parent.ot_hyst_c_ = hyst_celsius;
            parent.recompute_thresholds();
            return *this;
        }
    };

    /**
     * @brief Begin a limit configuration chain.
     * @return LimitProxy bound to this Temperature
     */
    inline LimitProxy set_limit_celsius() { return LimitProxy(*this); }
};
