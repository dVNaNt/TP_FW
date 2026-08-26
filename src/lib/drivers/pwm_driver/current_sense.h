/**
 * @file current.h
 * @brief Three-channel current sensing with automatic calibration, overcurrent protection,
 *        and always-active digital (software) overcurrent fallback.
 *
 * @details
 * This module provides high-performance current sensing for three independent
 * motor phase currents (A, B, C). It processes raw ADC readings from current sense amplifiers
 * and converts them to normalized Q15 fixed-point values or absolute amp measurements.
 *
 * ## Functional Overview
 *
 * The current sensing system operates in three stages:
 * 1. **Zero-current calibration** — Automatically measures ADC offset when motor is idle
 * 2. **Normalization** — Converts bipolar ADC readings to Q15 format (-32767 to +32767)
 * 3. **Overcurrent protection** — Hard limits with hardware comparator integration
 *    plus a fully digital fallback that **always** runs as a safety net
 *
 * ## Protection Architecture
 *
 * The overcurrent protection lives inside the OvercurrentProxy sub-class. It maintains:
 * - A configurable hard limit per channel (in Q15 normalized format)
 * - An atomic recovery timer (in tick cycles) that auto-clears the fault
 * - An optional hardware comparator callback (DAC threshold update)
 *
 * When the timer is active, OvercurrentProxy::is_active() returns true. The parent
 * DriverPWM reads this state every tick and reflects it into its fault_flags_ bitfield.
 * Any trigger (external ISR or internal software check) calls trigger(), which atomically
 * restarts the recovery timer.
 *
 * ## Digital Fallback
 *
 * During tick(), the absolute value of the normalized current for each channel is compared
 * against the configured hard limit. If it exceeds the limit, trigger() is called.
 * This runs unconditionally for all valid channels, ensuring that even if the hardware
 * comparator fails to fire or is not configured, the software will catch the overcurrent
 * event and keep the fault active.
 *
 * @note trigger() and reset() are private — only CurrentSense (tick) and
 *       DriverPWM (FaultProxy) may invoke them, never user code.
 * @warning The digital fallback is a safety net — it must NOT be disabled.
 *
 * @copyright Copyright 2026, creapunk
 * @license Apache-2.0
 */

#pragma once

#include <atomic>
#include "../../common.h"
#include "../generic/setup.h"
#include "motor_control.h"
#include "types.h"

// Forward declaration of DriverPWM to allow friend declaration
struct DriverPWM;

/// Function pointer type for calibration event callbacks (start/complete).
typedef void (*CalibrationCallback)();

/**
 * @struct CurrentSense
 * @brief Three-channel current sensing with calibration, normalization, and overcurrent protection.
 *
 * Owns the full current-sensing pipeline: raw ADC ingestion via SeqLock, zero-current
 * calibration, Q15 normalization with per-channel scale factors, and an OvercurrentProxy
 * sub-object that manages hardware comparator thresholds and fault recovery timers.
 */
struct CurrentSense {
    friend struct DriverPWM;         ///< Allow the driver to run private tick() and read outputs
    friend struct OvercurrentProxy;  ///< Allow proxy to access calibration/scale members
    friend struct MotorControl;      ///< Allow phase-stage access to normalization internals

private:
    // --- Dependencies ---
    Setup setup;  ///< Callback wrapper for hardware initialization
    MotorControl& motor_ctrl_;

    // --- Configuration ---
    /// Full-scale current amplitude in mA for each channel (0 => sensor missing).
    /// Negative values indicate an inverted sensor.
    /// @note Kept as integer mA internally for Q15 scaling precision.
    const Vec3D_i32 full_scale_ma_;
    int32_t full_scale_max_ma;  ///< Cached maximum absolute full-scale range among channels (mA)
    int16_t n_samples = 90;     ///< Number of samples for zero-current calibration (1..1000)

    CalibrationCallback cal_start_cb_ = nullptr;     ///< Callback invoked when calibration starts
    CalibrationCallback cal_complete_cb_ = nullptr;  ///< Callback invoked when calibration completes

    // --- State ---
    Vec3D_i32 cal_val_ = {0, 0, 0};                       ///< Per-channel calibration accumulator / zero-current offset
    Vec3D cal_n_ = {-1, -1, -1};                          ///< Per-channel calibration sample counter
    Vec3D scale_ = {0, 0, 0};                             ///< Precomputed per-channel ratio in Q15
    Vec3D out_norm_ = {INT16_MIN, INT16_MIN, INT16_MIN};  ///< Normalized current outputs in Q15

    /**
     * Latest alpha-beta phase currents converted to amperes.
     * Updated by tick() after normalization; alpha ~ phase A, beta ~ phase B
     * (phase C is reconstructed implicitly by the motor selector).
     */
    Vec2D_f32 current_ab_amps = {0, 0};

    // --- Input Buffer (SeqLock) ---
    alignas(4) uint16_t input_currents_[3] = {0, 0, 0};  ///< Input buffer holding the latest raw ADC readings
    std::atomic<uint32_t> input_seq_{0};                 ///< SeqLock counter for lock-free atomic updates
    std::atomic<bool> is_changed_{false};                ///< Flag indicating new currents were written since last tick

public:
    /**
     * @brief Construct a CurrentSense instance.
     * @param ch1_fs_amps Full-scale current for Phase A in amps (0.0f = sensor missing)
     * @param ch2_fs_amps Full-scale current for Phase B in amps (0.0f = sensor missing)
     * @param ch3_fs_amps Full-scale current for Phase C in amps (0.0f = sensor missing)
     * @note Negative full-scale values indicate an inverted current sensor.
     */
    CurrentSense(float ch1_fs_amps, float ch2_fs_amps, float ch3_fs_amps, MotorControl& motor_ctrl)
        : full_scale_ma_((int32_t)(ch1_fs_amps * 1000.0f), (int32_t)(ch2_fs_amps * 1000.0f), (int32_t)(ch3_fs_amps * 1000.0f)),
          overcurrent{*this},
          motor_ctrl_(motor_ctrl) {
        if (!(ch1_fs_amps == 0.0f && ch2_fs_amps == 0.0f && ch3_fs_amps == 0.0f)) {
            int32_t max_scale = 0;
            // Find maximum absolute full-scale to use as denominator for Q15 scaling
            for (uint8_t ch = 0u; ch < 3u; ++ch) {
                const int32_t abs_fs = (full_scale_ma_[ch] >= 0) ? full_scale_ma_[ch] : -full_scale_ma_[ch];
                if (abs_fs > max_scale)
                    max_scale = abs_fs;
            }
            full_scale_max_ma = max_scale;

            // Q15 ratio: (abs(full_scale_ma) << 14) / max_scale
            // Shift by 14 (not 15) to prevent overflow when dividing.
            for (uint8_t ch = 0u; ch < 3u; ++ch) {
                scale_[ch] = (full_scale_ma_[ch] << 14) / max_scale;
            }
        }
    }

private:
    /**
     * @brief Main processing tick — called every cycle by DriverPWM::tick().
     *
     * @details
     * When new ADC data is available (is_changed() == true):
     * 1. Reads 3×16-bit current values using a lock-free SeqLock pattern
     * 2. For each channel: if calibration is complete, normalizes to Q15
     *    and checks the digital overcurrent fallback; if calibrating,
     *    accumulates samples until n_samples is reached
     *
     * @note Called from DriverPWM::tick() — must be fast and ISR-safe on the read side.
     */
    void tick() {
        if (is_changed()) {
            is_changed_.store(false, std::memory_order_relaxed);

            uint16_t local_currents[3];
            read_currents_seqlock(local_currents);

            Vec3D abc_norm;

            // --- Per-channel calibration & normalization ---
            for (uint8_t ch = 0u; ch < 3u; ++ch) {
                abc_norm[ch] = process_channel(ch, local_currents[ch]);
            }
            out_norm_ = abc_norm;
            Vec2D currents_ab_norm = motor_ctrl_.tick_current(abc_norm);
            current_ab_amps.a = norm2amps(currents_ab_norm.a);
            current_ab_amps.b = norm2amps(currents_ab_norm.b);
        }
    }

    /**
     * @brief Lock-free read of 3x16-bit input currents using SeqLock pattern.
     */
    void read_currents_seqlock(uint16_t out_currents[3]) {
        uint32_t seq1, seq2;
        do {
            seq1 = input_seq_.load(std::memory_order_relaxed);
            std::atomic_thread_fence(std::memory_order_acquire);  // Data read after seq1

            out_currents[0] = input_currents_[0];
            out_currents[1] = input_currents_[1];
            out_currents[2] = input_currents_[2];

            std::atomic_thread_fence(std::memory_order_acquire);  // Seq2 read after data
            seq2 = input_seq_.load(std::memory_order_relaxed);
        } while (seq1 != seq2 || (seq1 & 1));  // Retry if changed or write in progress
    }

    /**
     * @brief Process calibration and normalization for a single channel.
     */
    int16_t process_channel(uint8_t ch, uint16_t local_current) {
        const int16_t cal_n_val = cal_n_[ch];

        if (cal_n_val > n_samples) {
            // Normal operation
            if (scale_[ch] == 0 || local_current == UINT16_MAX) {
                return INT16_MIN;  // Sensor missing
            } else {
                const int32_t bipolar = (int32_t)local_current - cal_val_[ch];
                // Scale delta by Q15 ratio: >> 15 compensates for the << 14 used in scale calculation
                const int32_t scaled = (bipolar * (int32_t)scale_[ch]) >> 15;

                int32_t abs_norm = (scaled < 0) ? -scaled : scaled;
                if (abs_norm > overcurrent.overcurrent_hard_norm_[ch]) {
                    overcurrent.trigger();  // Restart timer
                }
                return clamp_i32(scaled, INT16_MAX);
            }
        } else {
            // Calibration in progress
            if (cal_n_val < 0)
                return INT16_MIN;
            cal_n_[ch] = cal_n_val + 1u;
            if (cal_n_val < n_samples) {
                cal_val_[ch] += (int32_t)local_current;
            } else {
                cal_val_[ch] = cal_val_[ch] / n_samples;
                overcurrent.update_dac_threshold(ch);
                if (cal_complete_cb_)
                    cal_complete_cb_();
            }
            return INT16_MIN;
        }
    }

    /**
     * @brief Convert amperes to normalized Q15 value.
     * @param amps Current in amperes
     * @return Normalized current in Q15 (-32767..32767), or 0 if full_scale_max_ma <= 0
     */
    int16_t amps2norm(float amps) const {
        if (full_scale_max_ma <= 0)
            return 0;
        float max_amps = (float)full_scale_max_ma / 1000.0f;
        float norm = (amps / max_amps) * 32768.0f;
        if (norm > 32767.0f)
            norm = 32767.0f;
        if (norm < -32768.0f)
            norm = -32768.0f;
        return (int16_t)norm;
    }

    /**
     * @brief Convert normalized Q15 value to amperes.
     *
     * @param norm Normalized current in Q15 (INT16_MIN is the "sensor missing"
     *             sentinel and maps to 0 A).
     * @return Current in amperes.
     * @note Uses bitwise '|' instead of '||' - functionally equivalent here
     *       (both operands are side-effect-free booleans), but it evaluates
     *       both operands without short-circuiting.
     */
    inline float norm2amps(int16_t norm) const {
        if (full_scale_max_ma <= 0 || norm == INT16_MIN)
            return 0.0f;
        float max_amps = (float)full_scale_max_ma / 1000.0f;
        return ((float)norm / 32768.0f) * max_amps;
    }

public:
    /**
     * @brief Set the number of samples used for zero-current calibration.
     * @param n Number of samples (clamped to 1..1000)
     * @note Higher values improve calibration accuracy at the cost of longer calibration time.
     */
    inline void set_calibration_samples(int32_t n) { n_samples = (n < 1) ? 1 : ((n > 1000) ? 1000 : n); }

    /**
     * @brief Set the callback to be invoked when calibration starts.
     * @param cb Function pointer to call
     */
    inline void on_calibration_start(CalibrationCallback cb) { cal_start_cb_ = cb; }

    /**
     * @brief Set the callback to be invoked when calibration completes.
     * @param cb Function pointer to call
     */
    inline void on_calibration_complete(CalibrationCallback cb) { cal_complete_cb_ = cb; }

    /**
     * @brief Begin zero-current calibration for a specific channel.
     *
     * Resets the calibration accumulator and sample counter for the given channel.
     * The calibration runs automatically during subsequent tick() calls.
     *
     * @param channel Channel index (CH1, CH2, or CH3)
     * @note A running motor will produce invalid calibration. Ensure the motor is idle before calling.
     */
    void calibrate(Channel channel) {
        cal_n_[channel] = 0u;
        cal_val_[channel] = 0;
        if (cal_start_cb_)
            cal_start_cb_();
    }

    /// @brief Begin zero-current calibration for all three channels simultaneously.
    /// @see calibrate(Channel)
    inline void calibrate() {
        calibrate(CH1);
        calibrate(CH2);
        calibrate(CH3);
    }

    /**
     * @brief Check if a specific channel has completed calibration.
     * @param ch Channel index (CH1, CH2, or CH3)
     * @return true if calibration samples have been collected for this channel
     */
    inline bool is_calibrated(Channel ch) const { return (cal_n_[ch] > n_samples); }

    /**
     * @brief Check if all three channels have completed calibration.
     * @return true if all three channels have collected enough calibration samples
     */
    bool is_calibrated() const { return (cal_n_[0] > n_samples) && (cal_n_[1] > n_samples) && (cal_n_[2] > n_samples); }

    /**
     * @brief Check if new current values were written via set_currents() since the last tick().
     * @return true if pending ADC data has not yet been processed
     */
    inline bool is_changed() const { return is_changed_.load(std::memory_order_relaxed); }

    /**
     * @brief Atomically set 3 raw current ADC values. Safe to call from any interrupt.
     *
     * Uses a lock-free SeqLock pattern to avoid disabling interrupts globally.
     * The writer increments the sequence counter to an odd value, writes data,
     * then increments to an even value. The reader (tick()) detects in-progress
     * writes by checking the LSB of the sequence number.
     *
     * @param currents Array of 3 raw ADC values [ch1, ch2, ch3]
     * @note Must NOT be called with a nullptr array.
     */
    void set_currents(const uint16_t currents[3]) {
        if (currents == nullptr)
            return;

        // NOTE: load-then-store (instead of fetch_add) can drop an increment
        // if two writers race; safe only under the assumed single-writer ISR.
        uint32_t seq = input_seq_.load(std::memory_order_relaxed);

        // Start write: increment sequence to odd (writing in progress)
        input_seq_.store(seq + 1, std::memory_order_relaxed);
        std::atomic_thread_fence(std::memory_order_release);  // Seq visible before data

        input_currents_[0] = currents[0];
        input_currents_[1] = currents[1];
        input_currents_[2] = currents[2];

        std::atomic_thread_fence(std::memory_order_release);  // Data visible before seq
        // End write: increment sequence to even (write complete)
        input_seq_.store(seq + 2, std::memory_order_relaxed);

        is_changed_.store(true, std::memory_order_relaxed);
    }

    /**
     * @brief Get pointer to the array of normalized current outputs (Q15).
     * @return Pointer to 3-element int16_t array [CH1, CH2, CH3] in Q15 format
     */
    inline const int16_t* norm() const { return out_norm_.arr; }

    // =======================================================================
    // OVERCURRENT PROXY SUB-CLASS
    // =======================================================================

    /**
     * @struct OvercurrentProxy
     * @brief Zero-overhead wrapper holding all overcurrent protection state.
     *
     * Manages per-channel hard limits, an atomic recovery timer, and optional
     * hardware DAC callbacks. Designed as a public member of CurrentSense
     * so users can configure protection without touching internal current sensing logic.
     *
     * ## Fluent Configuration API
     *
     * Callback setup (chainable):
     * @code
     *   driver.current.overcurrent.set_callback()
     *       .oc_ia(hw_comp_callback)
     *       .oc_ib(hw_comp_callback);
     * @endcode
     *
     * Limit setup (chainable):
     * @code
     *   driver.current.overcurrent.set_limit_amps()
     *       .oc_ia(5.0f)
     *       .oc_ib(5.0f);
     * @endcode
     *
     * @note trigger() and reset() are private — accessible only to CurrentSense
     *       and DriverPWM via friend declarations.
     */
    struct OvercurrentProxy {
        friend struct CurrentSense;
        friend struct CallbackProxy;
        friend struct DriverPWM;

    private:
        CurrentSense& self;  ///< Reference to parent CurrentSense (zero overhead)

        static inline void empty_comp_oc_callback(uint16_t) {}  ///< Noop callback to eliminate nullptr checks

        CompThresholdCallback comp_oc_callback_[3] = {&empty_comp_oc_callback, &empty_comp_oc_callback, &empty_comp_oc_callback};

        std::atomic<int32_t> oc_error_cycles_{0};                          ///< Active recovery timer (0 = no error)
        std::atomic<int32_t> oc_error_timeout_cycles_{kDefaultRecoveryCycles};              ///< Configured timeout loaded on trigger()
        Vec3D overcurrent_hard_norm_ = {INT16_MAX, INT16_MAX, INT16_MAX};  ///< Hard limit in Q15

        /**
         * @brief Decrement the recovery timer atomically.
         *
         * Uses compare_exchange to avoid losing a concurrent trigger() call.
         * This is the only safe way to co-exist with ISR-triggered updates.
         *
         * @return true if the timer was > 0 before the decrement (fault still active)
         */
        bool tick_timer() {
            int32_t oc_err = oc_error_cycles_.load(std::memory_order_relaxed);

            if (oc_err > 0) {
                oc_error_cycles_.compare_exchange_strong(oc_err, oc_err - 1, std::memory_order_relaxed);
                return true;
            }
            return false;
        }

        /**
         * @brief Update the hardware comparator DAC threshold for a specific channel.
         *
         * Converts the Q15 hard limit back to an absolute ADC value by reversing the
         * normalization pipeline. The result is fed to the hardware comparator callback
         * to program the physical DAC threshold.
         *
         * @param ch Channel index (0, 1, or 2)
         * @note Called automatically after calibration completes and when limits change.
         */
        void update_dac_threshold(uint8_t ch) {
            if (self.scale_[ch] == 0 || (self.cal_n_[ch] < self.n_samples))
                return;

            // Reverse normalization: Q15 limit -> ADC delta. Negated to match
            // the sign convention of currents drawn through the low-side
            // shunt amplifier (positive motor current decreases the ADC word).
            int32_t dac_delta = -((int32_t)overcurrent_hard_norm_[ch] << 15) / self.scale_[ch];
            int32_t dac_value = dac_delta + self.cal_val_[ch];

            if (dac_value < 0)
                dac_value = 0;
            if (dac_value > UINT16_MAX)
                dac_value = UINT16_MAX;

            comp_oc_callback_[ch]((uint16_t)dac_value);
        }

        /**
         * @brief Set the hardware comparator callback for a specific channel.
         * @param ch Channel index (CH1, CH2, or CH3)
         * @param callback Function pointer (nullptr automatically replaced with a noop)
         */
        inline void set_callback(Channel ch, CompThresholdCallback callback) {
            comp_oc_callback_[ch] = callback ? callback : &empty_comp_oc_callback;
        }

        // --- Private: only CurrentSense and DriverPWM may invoke ---

        /**
         * @brief Trigger overcurrent protection — restart the recovery timer.
         *
         * Atomically loads the configured timeout and stores it into the active
         * recovery timer. Safe to call from interrupt context.
         *
         * @note Private. Invoked by CurrentSense::tick() (digital fallback)
         *       and DriverPWM::FaultProxy (external trigger).
         */
        inline void trigger() {
            oc_error_cycles_.store(oc_error_timeout_cycles_.load(std::memory_order_relaxed), std::memory_order_relaxed);
        }

        /**
         * @brief Immediately clear the overcurrent error (reset timer to 0).
         *
         * @note Private. Does NOT call any callbacks — only clears internal state.
         *       Invoked by DriverPWM::FaultProxy when clearing a fault.
         */
        inline void reset() { oc_error_cycles_.store(0, std::memory_order_relaxed); }

    public:
        /**
         * @brief Construct an OvercurrentProxy bound to a parent CurrentSense.
         * @param s Reference to the owning CurrentSense instance
         */
        OvercurrentProxy(CurrentSense& s) : self(s) {}

        /**
         * @brief Check if overcurrent error is currently active.
         * @return true if the recovery timer is still counting down (fault ongoing)
         */
        inline bool is_active() const { return oc_error_cycles_.load(std::memory_order_relaxed) > 0; }

        /**
         * @brief Set hard overcurrent limit in amps for a specific channel.
         *
         * Converts amps to Q15 internally, updates the limit, and immediately
         * re-programs the hardware comparator DAC threshold.
         *
         * @param ch Channel index (CH1, CH2, or CH3)
         * @param limit_amps Limit in amperes
         * @note Triggers update_dac_threshold() automatically.
         */
        void set_limit_amps(Channel ch, float limit_amps) {
            overcurrent_hard_norm_[ch] = self.amps2norm(limit_amps);
            update_dac_threshold(ch);
        }

        /**
         * @brief Chainable callback setter for configuring hardware comparator callbacks per channel.
         *
         * Usage:
         * @code
         *   overcurrent.set_callback().oc_ia(cb).oc_ib(cb).oc_ic(cb);
         * @endcode
         */
        struct CallbackProxy {
        private:
            OvercurrentProxy& parent;  ///< Reference to the owning OvercurrentProxy

        public:
            /**
             * @brief Construct a CallbackProxy bound to an OvercurrentProxy.
             * @param p Reference to the owning OvercurrentProxy
             */
            CallbackProxy(OvercurrentProxy& p) : parent(p) {}

            /**
             * @brief Set overcurrent comparator callback for Phase A.
             * @param cb Function pointer (nullptr is safely replaced by a noop)
             * @return Reference to this setter for chaining
             */
            inline CallbackProxy& oc_lim_ia_cb(CompThresholdCallback cb) {
                if (cb != nullptr) {
                    parent.set_callback(CH1, cb);
                }
                return *this;
            }
            /**
             * @brief Set overcurrent comparator callback for Phase B.
             * @param cb Function pointer (nullptr is safely replaced by a noop)
             * @return Reference to this setter for chaining
             */
            inline CallbackProxy& oc_lim_ib_cb(CompThresholdCallback cb) {
                if (cb != nullptr) {
                    parent.set_callback(CH2, cb);
                }
                return *this;
            }
            /**
             * @brief Set overcurrent comparator callback for Phase C.
             * @param cb Function pointer (nullptr is safely replaced by a noop)
             * @return Reference to this setter for chaining
             */
            inline CallbackProxy& oc_lim_ic_cb(CompThresholdCallback cb) {
                if (cb != nullptr) {
                    parent.set_callback(CH3, cb);
                }
                return *this;
            }
        };

        /**
         * @brief Chainable limit setter for configuring overcurrent limits per channel in amps.
         *
         * Usage:
         * @code
         *   overcurrent.set_limit_amps().oc_ia(5.0f).oc_ib(5.0f).oc_ic(5.0f);
         * @endcode
         */
        struct LimitProxy {
        private:
            OvercurrentProxy& parent;  ///< Reference to the owning OvercurrentProxy

        public:
            /**
             * @brief Construct a LimitProxy bound to an OvercurrentProxy.
             * @param p Reference to the owning OvercurrentProxy
             */
            LimitProxy(OvercurrentProxy& p) : parent(p) {}

            /**
             * @brief Set overcurrent limit for Phase A.
             * @param limit_amps Limit in amperes
             * @return Reference to this setter for chaining
             */
            inline LimitProxy& oc_ia(float limit_amps) {
                parent.set_limit_amps(CH1, limit_amps);
                return *this;
            }
            /**
             * @brief Set overcurrent limit for Phase B.
             * @param limit_amps Limit in amperes
             * @return Reference to this setter for chaining
             */
            inline LimitProxy& oc_ib(float limit_amps) {
                parent.set_limit_amps(CH2, limit_amps);
                return *this;
            }
            /**
             * @brief Set overcurrent limit for Phase C.
             * @param limit_amps Limit in amperes
             * @return Reference to this setter for chaining
             */
            inline LimitProxy& oc_ic(float limit_amps) {
                parent.set_limit_amps(CH3, limit_amps);
                return *this;
            }
        };

        /**
         * @brief Begin a callback configuration chain.
         * @return CallbackProxy bound to this proxy
         */
        inline CallbackProxy set_callback() { return CallbackProxy(*this); }

        /**
         * @brief Begin a limit configuration chain.
         * @return LimitProxy bound to this proxy
         */
        inline LimitProxy set_limit_amps() { return LimitProxy(*this); }

        /**
         * @brief Set the overcurrent recovery timeout in tick cycles.
         *
         * When trigger() is called, the recovery timer is loaded with this value
         * and decrements on each tick_timer() call. When it reaches 0, the fault auto-clears.
         *
         * @param cycles Number of ticks before the fault auto-clears
         */
        inline void set_timeout_cycles(int32_t cycles) { oc_error_timeout_cycles_.store(cycles, std::memory_order_relaxed); }

    } overcurrent;  ///< Public proxy instance for overcurrent configuration
};