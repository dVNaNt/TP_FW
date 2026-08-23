/**
 * @file pwm_wrapper.h
 * @brief Zero-overhead PWM wrapper with chainable callback setup and thread-safe duty-cycle API.
 *
 * @details
 * Fluent interface consuming zero extra memory:
 *
 * @code
 *   pwm.set_period(ARR);
 *   pwm.set_callback()
 *       .phases(hw_set_compare)
 *       .current_sample_point(hw_set_sample_point);
 *
 *   pwm.set_duty(duty_q15);
 * @endcode
 *
 * All proxy structs hold a single reference and are trivially inlined — zero runtime overhead.
 *
 * @copyright Copyright 2026, creapunk
 * @license Apache-2.0
 */

#pragma once

#include <stdint.h>
#include <atomic>

/// Callback invoked to write 3-phase compare values to the hardware timer.
/// @param phases Array of 3 timer-compare values in ticks
typedef void (*pwm_set_phases_cb)(const int16_t phases[3]);

/// Callback invoked to configure the current-sense sampling point.
/// Typically positions a hardware timer to trigger ADC sampling at the
/// optimal point in the PWM cycle (center of the active vector).
typedef void (*current_sample_point_cb)(void);


/**
 * @class PwmWrapper
 * @brief PWM output stage with chainable configuration and thread-safe duty-cycle updates.
 *
 * Owns the PWM timer period, hardware output callbacks, current-sampling-point callback,
 * and the SeqLock-protected tick buffer for 3-phase duty cycles.
 *
 * ## Usage
 *
 * One-time setup:
 * @code
 *   pwm.set_period(period_ticks);
 *   pwm.set_callback()
 *       .phases(hal_set_compare)
 *       .current_sample_point(hal_set_sample_point);
 * @endcode
 *
 * Runtime (ISR-safe):
 * @code
 *   pwm.set_duty(duty_q15);
 * @endcode
 */
class PwmWrapper {
public:
    PwmWrapper() : set_phases_cb_(nullptr), current_sample_point_cb_(nullptr), period_(0), seq_(0) {}

    // =======================================================================
    // SETUP API
    // =======================================================================

    /**
     * @brief Set the PWM timer period.
     * @param arr Timer Auto-Reload Register value in ticks
     */
    inline void set_period(uint32_t arr) { period_ = arr; }

    /**
     * @struct callback_chain
     * @brief Zero-overhead proxy returned by set_callback() for chaining.
     *
     * Holds a single reference to the owning PwmWrapper. All methods are inline
     * and return *this — the compiler collapses the chain into direct assignments.
     */
    struct callback_chain {
        PwmWrapper& pwm_;

        /**
         * @brief Set the 3-phase hardware update callback.
         * @param cb Function pointer (nullptr = unset)
         * @return Reference to this chain for further calls
         */
        inline callback_chain& apply_phases_cb(pwm_set_phases_cb cb) { pwm_.set_phases_cb_ = cb; return *this; }

        /**
         * @brief Set the current-sense sampling point callback.
         *
         * This callback is invoked after each PWM update to configure a hardware
         * timer that triggers ADC sampling at the optimal point in the PWM cycle
         * (center of the active vector for low-side current sensing).
         *
         * @param cb Function pointer (nullptr = unset)
         * @return Reference to this chain for further calls
         */
        inline callback_chain& current_sample_point_cb(current_sample_point_cb cb) { pwm_.current_sample_point_cb_ = cb; return *this; }
    };

    /**
     * @brief Begin the callback configuration chain.
     *
     * @return callback_chain bound to this PwmWrapper
     *
     * @par Example
     * @code
     *   pwm.set_callback()
     *       .phases(write_ccr)
     *       .current_sample_point(set_cs_timer);
     * @endcode
     */
    inline callback_chain set_callback() { return {*this}; }




    // =======================================================================
    // HARDWARE OUTPUT (called by PhyDriverPWM::tick_icr)
    // =======================================================================

    /**
     * @brief Write 3-phase compare values to hardware (via registered callback).
     * @param phases Array of 3 timer-compare ticks
     */
    inline void set_phases(const int16_t phases[3]) const {
        if (set_phases_cb_) set_phases_cb_(phases);
    }

    /**
     * @brief Invoke the current-sampling-point callback to configure ADC timing.
     */
    inline void apply_current_sample_point() const {
        if (current_sample_point_cb_) current_sample_point_cb_();
    }

private:
    friend struct PhyDriverPWM;
    
    pwm_set_phases_cb     set_phases_cb_;            ///< Hardware callback: write CCR registers
    current_sample_point_cb current_sample_point_cb_; ///< Hardware callback: configure ADC sampling point
    uint32_t              period_;                   ///< Timer ARR value in ticks

    std::atomic<uint32_t> seq_{0};        ///< SeqLock counter for lock-free tick updates
    int16_t                ticks_[3] = {0, 0, 0};  ///< Target PWM ticks protected by SeqLock

        /**
     * @brief Atomically read the latest duty-cycle ticks via SeqLock.
     * @param out Array of 3 int16_t filled with timer-compare ticks
     * @return true if a consistent snapshot was read
     */
    inline bool read_ticks(int16_t out[3]) const {
        for (int i = 0; i < 4; ++i) {
            uint32_t seq1 = seq_.load(std::memory_order_relaxed);
            if (!(seq1 & 1)) {
                std::atomic_thread_fence(std::memory_order_acquire);
                out[0] = ticks_[0];
                out[1] = ticks_[1];
                out[2] = ticks_[2];
                std::atomic_thread_fence(std::memory_order_acquire);
                uint32_t seq2 = seq_.load(std::memory_order_relaxed);
                if (seq1 == seq2) return true;
            }
        }
        return false;
    }

        // =======================================================================
    // RUNTIME API (ISR-SAFE)
    // =======================================================================

    /**
     * @brief Set 3-phase duty cycle from Q15-normalized values. Thread-safe via SeqLock.
     *
     * Converts Q15 to timer-compare ticks using the configured period.
     * The reader (read_ticks()) uses a lock-free SeqLock pattern.
     *
     * @param duty Array of 3 Q15 values (-32768..32767)
     * @note Must NOT be called with a nullptr array.
     */
    inline void set_duty(const Vec3D duty) {
        const int32_t period = period_;
        const int32_t max_val = INT16_MAX;

        int16_t ticks[3];
        // Convert Q15 to timer ticks. (v == max_val) adds 1 tick to fix 100% duty edge case.
        ticks[0] = (int16_t)(((duty[0] * period) >> 15) + (duty[0] == max_val));
        ticks[1] = (int16_t)(((duty[1] * period) >> 15) + (duty[1] == max_val));
        ticks[2] = (int16_t)(((duty[2] * period) >> 15) + (duty[2] == max_val));

        // SeqLock write: odd seq → write data → even seq
        seq_.fetch_add(1, std::memory_order_relaxed);
        std::atomic_thread_fence(std::memory_order_release);

        ticks_[0] = ticks[0];
        ticks_[1] = ticks[1];
        ticks_[2] = ticks[2];

        std::atomic_thread_fence(std::memory_order_release);
        seq_.fetch_add(1, std::memory_order_relaxed);
    }
};
