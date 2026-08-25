/**
 * @file temperature.h
 * @brief MCU / power-stage temperature sensing with a very long shift-based
 *        low-pass filter and two-threshold (warning / overtemperature)
 *        protection with immediate release semantics.
 *
 * @details
 * This module mirrors the architecture of voltage.h, adapted for thermal
 * signals whose dynamics are orders of magnitude slower than the electrical
 * ones:
 *
 * ```
 * Raw ADC → [Atomic State Register] → tick() → Long EMA Filter → Q15 Norm → °C
 *                                                   ↓
 *                              WT limit check → warning proxy (flag only)
 *                              OT limit check → overheat proxy (fault bit)
 * ```
 *
 * ## Very long filtering
 *
 * Thermal time constants are seconds, not milliseconds, so the classical
 * 8-bit-alpha EMA (alpha >= 1 -> tau ~= 256 ticks) is far too fast. The
 * dedicated FilterLongEMA replaces the multiplier with a barrel shift:
 *
 *     state += (input - state) >> shift        (Q16.16 accumulator)
 *
 * giving a step-response time constant of exactly @c 2^shift ticks with zero
 * multiplications. At the nominal 20 kHz control rate:
 *
 * | shift | ticks     | time constant |
 * |-------|-----------|---------------|
 * | 16    | 65'536    | ~3.3 s        |
 * | 18    | 262'144   | ~13.1 s       |
 * | 20    | 1'048'576 | ~52.4 s       |
 * | 24    | 16'777'216| ~14 min       |
 *
 * ## Super-fast math
 *
 * The hot path (set_temperature_irq + tick) contains no floating-point and no
 * division: one atomic fetch-or, one subtraction, two shifts and two adds per
 * sample. Float conversion to degrees Celsius happens lazily inside the
 * get_temperature_c() accessor, which is intended for slow diagnostic paths
 * (USB/CAN telemetry), never from the ISR context.
 *
 * ## Two limits
 *
 * - **wt** — warning temperature: sets the @c warning proxy when the filtered
 *   temperature crosses it. Purely informational (telemetry/derating hooks);
 *   it never forces a PWM safe state.
 * - **ot** — overtemperature: sets the @c overheat proxy, which PhyDriverPWM
 *   maps onto FAULT_OVERHEAT (priority 1, Hi-Z safe state).
 *
 * ## Immediate release
 *
 * Unlike the voltage/current protections that latch a recovery timer, both
 * temperature protections release *immediately* once the threshold is passed
 * back (filtered temperature drops below the configured limit on any later
 * tick). This is intentional: after such a long filter there is no
 * noise-induced chatter left, so a hysteresis/timer would only delay resuming
 * operation.
 *
 * @note Raw input convention matches the rest of the PHY layer: 16-bit
 *       left-aligned ADC words where full scale corresponds to
 *       @c max_temp_celsius.
 *
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#pragma once

#include <atomic>
#include "../../common.h"

struct PhyDriverPWM;

/**
 * @struct FilterLongEMA
 * @brief Shift-based integer exponential moving average for very slow signals.
 *
 * Implements y += (x - y) >> shift on a Q16.16 accumulator. Compared to
 * FilterEMA (8-bit alpha, minimum tau = 256 ticks) this reaches time constants
 * of 2^shift ticks while staying multiply-free and branch-free.
 */
struct FilterLongEMA {
private:
    int32_t state_;  ///< Q16.16 filter accumulator
    uint8_t shift_;  ///< Right-shift amount; time constant = 2^shift ticks

public:
    /// Default constructor: zero state, tau = 2^20 ticks (~52 s at 20 kHz).
    FilterLongEMA() : state_(0), shift_(20) {}

    /**
     * @brief Construct and seed the filter.
     * @param seed_value Initial output value (skips convergence transient).
     * @param shift Time-constant exponent (tau = 2^shift ticks).
     */
    FilterLongEMA(int16_t seed_value, uint8_t shift)
        : state_(((int32_t)seed_value) << 16), shift_(shift) {}

    /// Set the time-constant exponent (tau = 2^shift ticks).
    inline void set_shift(uint8_t shift) { shift_ = shift; }

    /// Get the current time-constant exponent.
    inline uint8_t get_shift() const { return shift_; }

    /// Reset the accumulator to a known value (avoids startup transient).
    inline void reset(int16_t value) { state_ = ((int32_t)value) << 16; }

    /**
     * @brief Advance the filter by one sample.
     *
     * Hot path: 1 subtract, 1 arithmetic shift, 1 add, 1 logical shift.
     *
     * @param input New raw sample (Q16 range, i.e. unfiltered ADC code).
     * @return Filtered output in Q16-range integer units.
     */
    inline int16_t tick_lpf(int16_t input) {
        // Difference between the new sample and the current state, scaled down
        // by the time constant. Arithmetic shift keeps the sign for negative diffs.
        int32_t diff = (((int32_t)input << 16) - state_) >> shift_;

        // Accumulate the fractional correction into the Q16.16 state.
        state_ += diff;

        // Extract the integer part of the accumulator as the filter output.
        return (int16_t)(((uint32_t)state_) >> 16);
    }

    /// Latest filtered output without advancing the filter.
    inline int16_t get_output() const { return (int16_t)(((uint32_t)state_) >> 16); }
};

/**
 * @struct TemperaturePHY
 * @brief Heavily-filtered temperature sensing with WT (warning) and
 *        OT (overtemperature) thresholds.
 *
 * Owns the temperature pipeline: raw ADC ingestion (atomic state register,
 * ISR-safe), very long EMA filtering, Q15 normalization and the two
 * protection proxies (@c warning and @c overheat).
 *
 * Usage (via PhyDriverPWM):
 * @code
 *   driver.temperature.set_limit_celsius().wt_lim_celsius(60.0f).ot_lim_celsius(85.0f);
 *   float t = driver.temperature.get_temperature_c();
 *   bool hot = driver.temperature.overheat.is_active();
 * @endcode
 */
struct TemperaturePHY {
    friend struct PhyDriverPWM;

private:
    // Atomic state: combined raw ADC value and update flag
    // Bit 31: is_read flag (1 = data consumed, 0 = new data available)
    // Bits 0-30: raw_adc value
    std::atomic<uint32_t> temp_state_{0x80000000};
    static constexpr uint32_t FLAG_READ = 0x80000000;
    static constexpr uint32_t MASK_DATA = 0x7FFFFFFF;

    FilterLongEMA filter_;      ///< Very long shift-based low-pass filter
    const float max_temp_c_;    ///< degC corresponding to ADC full scale
    float temp_c_;              ///< Latest filtered temperature in degC
    int16_t temp_norm_;         ///< Latest filtered temperature in Q15

    int16_t wt_limit_norm_ = INT16_MAX;  ///< Warning limit in Q15 (INT16_MAX = disabled)
    int16_t ot_limit_norm_ = INT16_MAX;  ///< Overtemperature limit in Q15 (INT16_MAX = disabled)

    /**
     * @brief Convert degrees Celsius to the internal Q15 representation.
     *
     * Float math is fine here: it runs only on configuration changes from the
     * slow application context, never in the control ISR.
     *
     * @param celsius Threshold in degrees Celsius.
     * @return Q15-normalized threshold clamped to INT16_MAX.
     */
    int16_t celsius2norm(float celsius) const {
        if (max_temp_c_ <= 0.0f) return INT16_MAX;
        int32_t norm_calc = (int32_t)((celsius / max_temp_c_) * 32768.0f);
        return clamp_i32(norm_calc, INT16_MAX);
    }

    /**
     * @struct TempProtectionProxy
     * @brief Immediate-release flag for one temperature threshold (WT or OT).
     *
     * There is deliberately NO recovery timer: the flag follows the filtered
     * temperature every tick — active while the measurement is at/above the
     * threshold, cleared on the first tick it falls below it ("Immediate
     * release" in the file header).
     */
    struct TempProtectionProxy {
        friend struct TemperaturePHY;

    private:
        std::atomic<uint8_t> active_{0};  ///< 1 = threshold currently exceeded

        /// Activate the flag (threshold exceeded). ISR-safe.
        inline void activate() { active_.store(1, std::memory_order_relaxed); }

        /// Deactivate the flag (threshold released). ISR-safe.
        inline void deactivate() { active_.store(0, std::memory_order_relaxed); }

    public:
        /// Default constructor: protection inactive.
        TempProtectionProxy() = default;

        /// Tick-parity hook mirroring VoltageProtectionProxy::tick_timer().
        inline bool tick_timer() const { return active_.load(std::memory_order_relaxed) != 0; }

        /// @return true while the threshold is currently exceeded.
        inline bool is_active() const { return active_.load(std::memory_order_relaxed) != 0; }
    };

public:
    /** Public proxies evaluated by TemperaturePHY::tick(). */
    TempProtectionProxy warning;   ///< WT threshold: informational flag (no safe state)
    TempProtectionProxy overheat;  ///< OT threshold: mapped to FAULT_OVERHEAT (Hi-Z)

    /**
     * @brief Construct a TemperaturePHY instance.
     * @param filter_shift Filter time-constant exponent (tau = 2^shift ticks).
     * @param max_temp_celsius Full-scale temperature in degC (ADC max code).
     */
    explicit TemperaturePHY(uint8_t filter_shift = 20, float max_temp_celsius = 150.0f)
        : filter_(0, filter_shift),
          max_temp_c_(max_temp_celsius),
          temp_c_(0.0f),
          temp_norm_(0) {}

    /**
     * @brief Main processing tick — called every cycle by PhyDriverPWM::tick().
     *
     * Atomically reads and marks the state register, advances the long EMA
     * filter, converts to Q15/degC and re-evaluates both thresholds with
     * immediate-release semantics.
     *
     * @return true if a new temperature sample was processed in this tick
     */
    bool tick() {
        // Atomically read state AND set the read flag in one operation
        uint32_t state = temp_state_.fetch_or(FLAG_READ, std::memory_order_relaxed);
        bool is_updated = !(state & FLAG_READ);
        if (is_updated) {
            uint32_t raw_adc = state & MASK_DATA;

            // Advance the ultra-long multiply-free filter.
            filter_.tick_lpf((int16_t)raw_adc);

            // Convert filtered output to Q15 (>>1 maps Q16-range to Q15).
            temp_norm_ = filter_.get_output() >> 1;

            // Lazy float conversion for slow diagnostic consumers only.
            temp_c_ = ((float)temp_norm_) * (max_temp_c_ * (1.0f / 32768.0f));

            // Fast integer threshold checks with immediate release:
            // active while above, released on the first tick below.
            if (temp_norm_ >= ot_limit_norm_) {
                overheat.activate();
            } else {
                overheat.deactivate();
            }
            if (temp_norm_ >= wt_limit_norm_) {
                warning.activate();
            } else {
                warning.deactivate();
            }
        }
        return is_updated;
    }
    /// Latest filtered temperature [degC].
    inline float get_temperature_c() const { return temp_c_; }

    /// Latest filtered temperature in Q15 (32768 = full scale).
    inline int16_t get_temperature_norm() const { return temp_norm_; }

    /**
     * @brief Check if new temperature data was written since the last tick().
     * @return true if new ADC data is pending processing
     */
    inline bool is_changed() const { return !(temp_state_.load(std::memory_order_relaxed) & FLAG_READ); }

    /**
     * @brief Atomically set raw ADC temperature and mark data as new.
     *
     * Stores the ADC value in bits 0-30 and clears bit 31 (FLAG_READ) to
     * indicate new data. Safe to call from any interrupt context.
     *
     * @param raw_adc Raw 16-bit left-aligned ADC value
     */
    inline void set_temperature_irq(uint32_t raw_adc) {
        temp_state_.store(raw_adc & MASK_DATA, std::memory_order_relaxed);
    }

    /**
     * @brief Reset the filter to a specific ADC value and process immediately.
     *
     * Useful for initialization to skip the multi-second filter convergence.
     *
     * @param raw_adc Initial ADC value to seed the filter
     */
    inline void reset(uint16_t raw_adc) {
        filter_.reset((int16_t)raw_adc);
        set_temperature_irq(raw_adc);
        tick();
    }

    /// Set the filter time-constant exponent (tau = 2^shift ticks).
    inline void set_filter_shift(uint8_t shift) { filter_.set_shift(shift); }

    /**
     * @brief Chainable limit setter for configuring WT/OT thresholds in degC.
     *
     * Usage:
     * @code
     *   temperature.set_limit_celsius().wt_lim_celsius(60.0f).ot_lim_celsius(85.0f);
     * @endcode
     */
    struct LimitSetter {
    private:
        TemperaturePHY& parent;  ///< Reference to the owning TemperaturePHY

    public:
        /**
         * @brief Construct a LimitSetter bound to a TemperaturePHY.
         * @param p Reference to the owning TemperaturePHY
         */
        LimitSetter(TemperaturePHY& p) : parent(p) {}

        /**
         * @brief Set the warning temperature limit.
         * @param limit_celsius Warning threshold in degC
         * @return Reference to this setter for chaining
         */
        inline LimitSetter& wt_lim_celsius(float limit_celsius) {
            parent.wt_limit_norm_ = parent.celsius2norm(limit_celsius);
            return *this;
        }

        /**
         * @brief Set the overtemperature limit.
         * @param limit_celsius Overtemperature threshold in degC
         * @return Reference to this setter for chaining
         */
        inline LimitSetter& ot_lim_celsius(float limit_celsius) {
            parent.ot_limit_norm_ = parent.celsius2norm(limit_celsius);
            return *this;
        }
    };

    /**
     * @brief Begin a limit configuration chain.
     * @return LimitSetter bound to this TemperaturePHY
     */
    inline LimitSetter set_limit_celsius() { return LimitSetter(*this); }
};


