/**
 * @file voltage.h
 * @brief Supply voltage sensing with low-pass filtering, normalization, and
 *        overvoltage/undervoltage protection with recovery timers.
 *
 * @details
 * This module provides filtered supply voltage sensing for a single DC bus.
 * It processes raw ADC readings through an EMA (Exponential Moving Average)
 * low-pass filter, converts to both Q15 normalized format and absolute volt measurements,
 * and provides two VoltageProtectionProxy instances (overvoltage and undervoltage)
 * with independent recovery timers and hardware comparator DAC callbacks.
 *
 * ## Processing Pipeline
 *
 * ```
 * Raw ADC → [Atomic State Register] → tick() → EMA Filter → Q15 Norm → Volts
 *                                                    ↓
 *                                         OV/UV limit checks → trigger()
 * ```
 *
 * ## Protection Features
 *
 * - **Overvoltage**: Triggers when voltage_norm_ >= overvoltage_limit_norm_
 * - **Undervoltage**: Triggers when voltage_norm_ <= undervoltage_limit_norm_
 * - Both have independent atomic recovery timers with configurable timeout
 * - Optional hardware comparator DAC callbacks for each threshold
 *
 * @note trigger() and reset() on VoltageProtectionProxy are private —
 *       only SupplyVoltage (tick) and DriverPWM (FaultTriggerProxy) may invoke them.
 *
 * ## Fluent Configuration API
 *
 * Callback setup:
 * @code
 *   driver.supply.set_callback()
 *       .ov_hw(ov_comp_cb)
 *       .uv_hw(uv_comp_cb);
 * @endcode
 *
 * Limit setup:
 * @code
 *   driver.supply.set_limit_volts()
 *       .ov_lim_volts(36.0f)
 *       .uv_lim_volts(9.0f);
 * @endcode
 *
 * @copyright Copyright 2026, creapunk
 * @license Apache-2.0
 */

#pragma once

#include <atomic>
#include "../../common.h"
#include "../../math/filters/ema.h"

struct DriverPWM;

/**
 * @struct SupplyVoltage
 * @brief Filtered supply voltage sensing with overvoltage and undervoltage protection.
 *
 * Owns the voltage sensing pipeline: raw ADC ingestion (atomic state register),
 * EMA low-pass filtering, Q15 normalization, and two VoltageProtectionProxy
 * sub-objects for overvoltage/undervoltage fault management.
 */
struct SupplyVoltage {
    friend struct DriverPWM;
    friend struct MotorControl;  ///< Phase stage consumes the V->duty conversions
    friend struct VoltageProtectionProxy;

private:
    static inline void empty_comp_callback(uint16_t) {}  ///< Noop callback replaces nullptr

    CompThresholdCallback comp_ov_callback_ = &empty_comp_callback;  ///< HW overvoltage comparator callback
    CompThresholdCallback comp_uv_callback_ = &empty_comp_callback;  ///< HW undervoltage comparator callback

    FilterEMA filter_;         ///< First-order EMA low-pass filter
    const float max_voltage_;  ///< Maximum representable voltage in VOLTS (Q15 full scale).
    int32_t scale_;            ///< Precomputed scale factor ((max_voltage_*1000) >> 5), in mV units.
    float voltage_v_;          ///< Latest filtered voltage in VOLTS.
    float inv_voltage_ = 0.0f;  ///< Cached 1/voltage_v_ (0 = invalid); recomputed only when the filtered voltage changes.
    int16_t voltage_norm_;     ///< Latest filtered voltage in Q15.

    int16_t overvoltage_limit_norm_ = INT16_MAX;   ///< Overvoltage limit in Q15
    int16_t undervoltage_limit_norm_ = INT16_MIN;  ///< Undervoltage limit in Q15

    // Atomic state: combined raw ADC value and update flag
    // Bit 31: is_read flag (1 = data consumed, 0 = new data available)
    // Bits 0-30: raw_adc value
    std::atomic<uint32_t> voltage_state_{0x80000000};
    static constexpr uint32_t FLAG_READ = 0x80000000;
    static constexpr uint32_t MASK_DATA = 0x7FFFFFFF;

    /**
     * @brief Construct a SupplyVoltage instance.
     * @param alpha EMA filter coefficient (0..255, higher = less filtering)
     * @param max_voltage_volts Maximum voltage in volts for Q15 scaling
     */
    SupplyVoltage(uint8_t alpha, float max_voltage_volts)
        : filter_(0, alpha),
          max_voltage_(max_voltage_volts),
          scale_(0),
          voltage_norm_(0),
          voltage_v_(0),
          overvoltage(*this, comp_ov_callback_, overvoltage_limit_norm_),
          undervoltage(*this, comp_uv_callback_, undervoltage_limit_norm_) {
        // Precompute scale so that the conversions reduce to shifts:
        //   volts = (norm * scale_) >> 10 / 1000  ==  norm/2^15 * max_voltage_
        //   norm  = (mv << 10) / scale_           ==  mv/max_mv * 2^15
        scale_ = (int32_t)(max_voltage_ * 1000.0) >> 5;
    }

    /**
     * @brief Main processing tick — called every cycle by DriverPWM::tick().
     *
     * Atomically reads and marks the voltage state register, advances the EMA
     * filter, and checks software OV/UV limits against the configured thresholds.
     *
     * @return true if new voltage data was processed in this tick
     */
    bool tick() {
        // Atomically read state AND set the read flag in one operation
        uint32_t state = voltage_state_.fetch_or(FLAG_READ, std::memory_order_relaxed);
        bool is_updated = !(state & FLAG_READ);
        if (is_updated) {
            uint32_t raw_adc = state & MASK_DATA;

            // Advance the EMA filter. Re-processing same value converges further.
            filter_.tick_lpf(raw_adc);

            // Convert filtered output to Q15. >>1 brings it to Q15 range.
            voltage_norm_ = filter_.get_output() >> 1;

            voltage_v_ = (float)(((int32_t)voltage_norm_ * scale_) >> 10) * 0.001;

            // OPTIMIZATION (P2.3): maintain the reciprocal here (slow path,
            // once per new sample) so volts2duty() stays multiply-only.
            inv_voltage_ = (voltage_v_ > 0.0f) ? (1.0f / voltage_v_) : 0.0f;

            // Software limit checks (always active). Restart timer if exceeded.
            if (voltage_norm_ >= overvoltage_limit_norm_) {
                overvoltage.trigger();
            }
            if (voltage_norm_ <= undervoltage_limit_norm_) {
                undervoltage.trigger();
            }
        }
        return is_updated;
    }

private:
    /**
     * @brief Convert volts to normalized Q15 value.
     * @param volts Voltage in volts
     * @return Normalized voltage in Q15, or 0 if scale_ is 0
     */
    int16_t volts2norm(float volts) const {
        if (scale_ == 0)
            return 0;
        int32_t mv = (int32_t)(volts * 1000.0f);
        int32_t norm_calc = (mv << 10) / scale_;
        return clamp_i32(norm_calc, INT16_MAX);
    }

    /**
     * @brief Convert target volts to duty cycle (Q15).
     *
     * Calculates the PWM duty cycle needed to achieve a target voltage,
     * given the current measured supply voltage. Uses the cached reciprocal
     * (inv_voltage_) so the per-cycle hot path performs multiplications only
     * — no float division (see optimization plan P2.3).
     *
     * @param target_volts Target voltage in volts
     * @return Duty cycle in Q15 (0..32767), or 0 if the bus voltage is not valid yet
     */
    inline int16_t volts2duty(float target_volts) const {
        float inv = inv_voltage_;  // Single volatile-free local load
        if (inv <= 0.0f)
            return 0;

        int32_t duty_int = (int32_t)(target_volts * inv * 32768.0f);
        return clamp_i32(duty_int, INT16_MAX);
    }

public:
    /**
     * @brief Get the latest filtered voltage in volts.
     * @return Voltage in volts
     */
    inline float get_voltage() const { return voltage_v_; }

    /**
     * @brief Get the maximum representable voltage for Q15 scaling.
     * @return Maximum voltage in volts
     */
    inline float get_max_voltage() const { return max_voltage_; }

    /**
     * @brief Check if new voltage data was written via set_voltage() since the last tick().
     * @return true if new ADC data is pending processing
     */
    inline bool is_changed() const { return !(voltage_state_.load(std::memory_order_relaxed) & FLAG_READ); }

    /**
     * @brief Atomically set raw ADC voltage and mark data as new.
     *
     * Stores the ADC value in bits 0-30 and clears bit 31 (FLAG_READ) to
     * indicate new data is available. Safe to call from any interrupt context.
     *
     * @param raw_adc Raw ADC value (up to 31 bits, higher bits are masked)
     */
    inline void set_voltage_irq(uint32_t raw_adc) {
        // Store data in bits 0-30 and clear bit 31 (FLAG_READ = 0) to indicate new data
        voltage_state_.store(raw_adc & MASK_DATA, std::memory_order_relaxed);
    }

    /**
     * @brief Seed the filter with a known ADC value and process immediately.
     *
     * Useful for initialization to avoid filter convergence delay on startup.
     *
     * @param raw_adc Initial ADC value to seed the filter
     */
    inline void seed_filter(uint16_t raw_adc) {
        filter_.reset(raw_adc);
        set_voltage_irq(raw_adc);
        tick();
    }

    /**
     * @brief Set the EMA filter alpha coefficient.
     * @param alpha Filter coefficient (0..255, higher = less filtering / faster response)
     */
    inline void set_alpha(uint8_t alpha) { filter_.set_alpha(alpha); }

    // =======================================================================
    // VOLTAGE PROTECTION PROXY SUB-CLASS
    // =======================================================================

    /**
     * @struct VoltageProtectionProxy
     * @brief Manages a single voltage protection threshold (overvoltage OR undervoltage).
     *
     * Each proxy maintains an independent atomic recovery timer and optional
     * hardware comparator DAC callback. Two instances are created as public
     * members of SupplyVoltage:
     * - @c overvoltage — protects against excessive supply voltage
     * - @c undervoltage — protects against insufficient supply voltage
     *
     * @note trigger() and reset() are private — accessible only to SupplyVoltage
     *       and DriverPWM via friend declarations.
     */
    struct VoltageProtectionProxy {
        friend struct SupplyVoltage;
        friend struct DriverPWM;
        friend struct FaultTriggerProxy;  ///< Fluent fault API (driver_pwm_detail.h)

    private:
        SupplyVoltage& self;                ///< Reference to parent SupplyVoltage
        CompThresholdCallback& callback_ref_;  ///< Reference to the hardware comparator callback
        int16_t& limit_norm_ref_;              ///< Reference to the Q15 limit value

        std::atomic<int32_t> error_cycles_{0};  ///< Active recovery timer (0 = no error)
        std::atomic<int32_t> error_timeout_cycles_{kDefaultRecoveryCycles};  ///< Configured timeout loaded on trigger()

        /**
         * @brief Decrement the recovery timer atomically.
         *
         * Uses compare_exchange to avoid losing a concurrent trigger() call
         * from an ISR.
         *
         * @return true if the timer was > 0 before the decrement (fault still active)
         */
        bool tick_timer() {
            int32_t err = error_cycles_.load(std::memory_order_relaxed);
            if (err > 0) {
                error_cycles_.compare_exchange_strong(err, err - 1, std::memory_order_relaxed);
                return true;
            }
            return false;
        }

        /**
         * @brief Update the hardware comparator DAC threshold.
         *
         * Converts the Q15 limit to the comparator's expected 16-bit range
         * and calls the registered callback. Noop if the callback is the
         * default empty function.
         */
        void update_dac() {
            if (callback_ref_ != &SupplyVoltage::empty_comp_callback) {
                // <<1 maps the Q15 half-scale representation onto the full
                // unsigned 16-bit DAC/comparator range (Q15 16384 -> 32768).
                uint16_t dac_value = (uint16_t)limit_norm_ref_ << 1;
                callback_ref_(dac_value);
            }
        }

        // --- Private: only SupplyVoltage and DriverPWM may invoke ---

        /**
         * @brief Start or restart the recovery timer.
         *
         * Atomically loads the configured timeout and stores it into the active
         * recovery timer. Safe to call from interrupt context.
         *
         * @note Private. Invoked by SupplyVoltage::tick() (software check)
         *       and DriverPWM::set_fault() proxy (external trigger).
         */
        inline void trigger() {
            error_cycles_.store(error_timeout_cycles_.load(std::memory_order_relaxed), std::memory_order_relaxed);
        }

        /**
         * @brief Immediately reset the error (clear timer to 0).
         *
         * @note Private. Does NOT call any callbacks — only clears internal state.
         *       Invoked by DriverPWM::set_fault() proxy when clearing a fault.
         */
        inline void reset() { error_cycles_.store(0, std::memory_order_relaxed); }

    public:
        /**
         * @brief Construct a VoltageProtectionProxy.
         *
         * @param s Reference to the owning SupplyVoltage
         * @param cb Reference to the hardware comparator callback variable
         * @param limit Reference to the Q15 limit variable
         */
        VoltageProtectionProxy(SupplyVoltage& s, CompThresholdCallback& cb, int16_t& limit)
            : self(s), callback_ref_(cb), limit_norm_ref_(limit) {}

        /**
         * @brief Check if protection error is currently active.
         * @return true if the recovery timer is still counting down
         */
        inline bool is_active() const { return error_cycles_.load(std::memory_order_relaxed) > 0; }

        /**
         * @brief Set the recovery timeout in tick cycles.
         *
         * When trigger() is called, the recovery timer is loaded with this value
         * and decrements each tick_timer() call. When it reaches 0, the fault auto-clears.
         *
         * @param cycles Number of ticks before the fault auto-clears
         */
        inline void set_timeout_cycles(int32_t cycles) { error_timeout_cycles_.store(cycles, std::memory_order_relaxed); }

        /**
         * @brief Set the hardware comparator callback.
         *
         * Passing nullptr disables hardware comparator integration (software-only mode).
         * The callback is automatically replaced with a noop function.
         *
         * @param cb Function pointer (nullptr = software-only mode)
         */
        void set_lim_callback(CompThresholdCallback cb) {
            callback_ref_ = cb ? cb : &SupplyVoltage::empty_comp_callback;
            update_dac();
        }

        /**
         * @brief Set the voltage protection limit in volts.
         *
         * Converts volts to Q15 internally and immediately updates the hardware
         * comparator DAC threshold.
         *
         * @param limit_volts Limit in volts
         */
        void set_limit_volts(float limit_volts) {
            if (limit_volts > self.max_voltage_)
                limit_volts = self.max_voltage_;

            limit_norm_ref_ = self.volts2norm(limit_volts);
            update_dac();
        }

    } overvoltage, undervoltage;  ///< Two proxy instances for OV and UV protection

    /**
     * @brief Chainable callback setter for configuring hardware comparator callbacks.
     *
     * Usage:
     * @code
     *   supply.set_callback().ov_hw(ov_cb).uv_hw(uv_cb);
     * @endcode
     */
    struct CallbackProxy {
    private:
        SupplyVoltage& parent;  ///< Reference to the owning SupplyVoltage

    public:
        /**
         * @brief Construct a CallbackProxy bound to a SupplyVoltage.
         * @param p Reference to the owning SupplyVoltage
         */
        CallbackProxy(SupplyVoltage& p) : parent(p) {}

        /**
         * @brief Set overvoltage hardware comparator callback.
         * @param cb Function pointer (nullptr disables hardware comparator)
         * @return Reference to this setter for chaining
         */
        inline CallbackProxy& ov_lim_cb(CompThresholdCallback cb) {
            parent.overvoltage.set_lim_callback(cb);
            return *this;
        }
        /**
         * @brief Set undervoltage hardware comparator callback.
         * @param cb Function pointer (nullptr disables hardware comparator)
         * @return Reference to this setter for chaining
         */
        inline CallbackProxy& uv_lim_cb(CompThresholdCallback cb) {
            parent.undervoltage.set_lim_callback(cb);
            return *this;
        }
    };

    /**
     * @brief Chainable limit setter for configuring OV/UV thresholds in volts.
     *
     * Usage:
     * @code
     *   supply.set_limit_volts().ov_lim_volts(36.0f).uv_lim_volts(9.0f);
     * @endcode
     */
    struct LimitProxy {
    private:
        SupplyVoltage& parent;  ///< Reference to the owning SupplyVoltage

    public:
        /**
         * @brief Construct a LimitProxy bound to a SupplyVoltage.
         * @param p Reference to the owning SupplyVoltage
         */
        LimitProxy(SupplyVoltage& p) : parent(p) {}

        /**
         * @brief Set overvoltage limit.
         * @param limit_volts Overvoltage threshold in volts
         * @return Reference to this setter for chaining
         */
        inline LimitProxy& ov_lim_volts(float limit_volts) {
            parent.overvoltage.set_limit_volts(limit_volts);
            return *this;
        }
        /**
         * @brief Set undervoltage limit.
         * @param limit_volts Undervoltage threshold in volts
         * @return Reference to this setter for chaining
         */
        inline LimitProxy& uv_lim_volts(float limit_volts) {
            parent.undervoltage.set_limit_volts(limit_volts);
            return *this;
        }
    };

    /**
     * @brief Begin a callback configuration chain.
     * @return CallbackProxy bound to this SupplyVoltage
     */
    inline CallbackProxy set_callback() { return CallbackProxy(*this); }

    /**
     * @brief Begin a limit configuration chain.
     * @return LimitProxy bound to this SupplyVoltage
     */
    inline LimitProxy set_limit_volts() { return LimitProxy(*this); }
};