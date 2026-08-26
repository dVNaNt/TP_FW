/**
 * @file power.h
 * @brief Power-stage power measurement with EMA filtering and overpower (OP)
 *        protection based on dual symmetric power/current limits.
 *
 * @details
 * Power estimates the electrical power flowing through the power stage from
 * the quantities already available inside DriverPWM:
 *
 * - **Phase currents** — the measured alpha-beta currents (i_alpha ~ phase A,
 *   i_beta ~ phase B) delivered by CurrentSense every tick.
 * - **Commanded alpha-beta voltages** — the last voltage vector injected by
 *   the application through DriverPWM::set_voltage_ab_volts() (the setter).
 *
 * ## Power Estimation
 *
 * The instantaneous electrical power of a three-phase system is invariant
 * under the Clarke transform, so it can be computed directly in the
 * alpha-beta frame:
 *
 *     p(t) = v_alpha * i_alpha + v_beta * i_beta     [W]
 *
 * This is a signed quantity: positive during motoring, negative during
 * regeneration (energy returned to the DC link). The raw value is exposed
 * as-is (@ref get_raw_power_w) in parallel with the filtered path.
 *
 * ## Filtering
 *
 * A first-order exponential moving average runs on the raw power:
 *
 *     y += alpha * (x - y),  alpha = user_alpha / 255
 *
 * Alpha follows the codebase convention (0..255, 255 = no filtering). The
 * filtered value drives all protection decisions so that single-cycle PWM
 * transients cannot trip the fault.
 *
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#pragma once

#include <atomic>
#include <cmath>
#include "../../common.h"
#include "types.h"

struct DriverPWM;

/**
 * @struct Power
 * @brief Power-stage measurement, filtering, and overpower protection.
 *
 * Owns the power pipeline: commanded-voltage capture, instantaneous power
 * computation, EMA filtering, derived input-current estimation, the
 * overpower protection proxy (@c overpower), and the per-tick utilization
 * coefficient.
 */
struct Power {
    friend struct DriverPWM;

private:
    // --- Configuration ---
    uint8_t filter_alpha_ = 64;          ///< EMA coefficient 0..255 (255 = no filtering)
    float lim_power_w_ = 100.0f;         ///< Symmetric power limit magnitude [W] (trip at +/-)
    float lim_current_a_ = 15.0f;        ///< Symmetric input-current limit magnitude [A] (trip at +/-)

    // --- State ---
    Vec2D_f32 cmd_v_ = {0.0f, 0.0f};     ///< Last commanded alpha-beta voltages [V] (setter-injected)
    float raw_power_w_ = 0.0f;           ///< Instantaneous electrical power [W] (signed, unfiltered)
    float filt_power_w_ = 0.0f;          ///< EMA-filtered power [W] (signed)
    float filt_current_a_ = 0.0f;        ///< Derived input current: filt_power / filt_voltage [A]
    float filt_voltage_v_ = 0.0f;        ///< Cached filtered supply voltage [V]
    float usage_ = 0.0f;                 ///< Limit utilization coefficient [0..>1]
    float power_coeff_ = 0.0f;           ///< Signed power share of P_lim [-1..+1] (+ motor / - regen)

public:
    Power() = default;

    // =======================================================================
    // OVERPOWER PROTECTION PROXY SUB-CLASS
    // =======================================================================

    /**
     * @struct OverpowerProxy
     * @brief Atomic recovery-timer latch for the overpower fault.
     *
     * Mirrors the OC/OV/UV recovery-timer pattern: trigger() reloads the
     * timer, tick_timer() decrements it once per control cycle, and the
     * fault auto-clears when it reaches zero.
     *
     * @note trigger() and reset() are private — only Power::tick() and
     *       DriverPWM may invoke them, never user code.
     */
    struct OverpowerProxy {
        friend struct Power;
        friend struct DriverPWM;

    private:
        std::atomic<int32_t> op_error_cycles_{0};  ///< Active recovery timer (0 = no error)
        std::atomic<int32_t> op_error_timeout_cycles_{kDefaultRecoveryCycles};  ///< Timeout loaded on trigger() [ticks]

        /**
         * @brief Decrement the recovery timer atomically.
         *
         * Uses compare_exchange to avoid losing a concurrent trigger() call.
         *
         * @return true if the timer was > 0 before the decrement (fault still active)
         */
        bool tick_timer() {
            int32_t op_err = op_error_cycles_.load(std::memory_order_relaxed);
            if (op_err > 0) {
                op_error_cycles_.compare_exchange_strong(op_err, op_err - 1, std::memory_order_relaxed);
                return true;
            }
            return false;
        }

        // --- Private: only Power and DriverPWM may invoke ---

        /**
         * @brief Trigger overpower protection — restart the recovery timer.
         *
         * Atomically loads the configured timeout into the active timer.
         * Safe to call from interrupt context.
         */
        inline void trigger() {
            op_error_cycles_.store(op_error_timeout_cycles_.load(std::memory_order_relaxed),
                                   std::memory_order_relaxed);
        }

        /**
         * @brief Immediately reset the error (clear timer to 0).
         *
         * Does NOT call any callbacks — only clears internal state.
         */
        inline void reset() { op_error_cycles_.store(0, std::memory_order_relaxed); }

    public:
        /**
         * @brief Check if the overpower fault is currently active.
         * @return true if the recovery timer is still counting down
         */
        inline bool is_active() const { return op_error_cycles_.load(std::memory_order_relaxed) > 0; }

        /**
         * @brief Set the recovery timeout in tick cycles.
         *
         * When trigger() is called, the recovery timer is loaded with this
         * value and decrements on each tick_timer() call. When it reaches 0,
         * the fault auto-clears (provided the signal is within limits again).
         *
         * @param cycles Number of ticks before the fault auto-clears
         */
        inline void set_timeout_cycles(int32_t cycles) {
            op_error_timeout_cycles_.store(cycles, std::memory_order_relaxed);
        }
    } overpower;  ///< Public proxy instance for overpower configuration

    // =======================================================================
    // PROCESSING
    // =======================================================================

    /**
     * @brief Capture the commanded alpha-beta voltage vector (setter hook).
     *
     * Called by DriverPWM::set_voltage_ab_volts() every time the
     * application commands a new voltage vector. Non-finite components
     * (NaN/±INF sentinels used for HiZ/LowAll safe states) are sanitized to
     * zero so they cannot poison the power estimate.
     *
     * @param voltages Commanded alpha-beta voltages [V]
     */
    inline void set_voltage_command(Vec2D_f32 voltages) {
        // Sanitize NaN/Inf sentinels to zero — no valid power can be computed
        if (!std::isfinite(voltages.a) || !std::isfinite(voltages.b)) {
            cmd_v_ = {0.0f, 0.0f};
            return;
        }
        cmd_v_ = voltages;
    }

    /**
     * @brief Main processing tick — called every cycle by DriverPWM::tick().
     *
     * Pipeline: raw power -> EMA filter -> derived input current ->
     * utilization coefficient -> OP limit checks.
     *
     * ## Overpower (OP) Protection
     *
     * Two symmetric limits are enforced (identically in + and - directions,
     * covering regeneration as well as motoring):
     * - **max input power**   P_lim [W]
     * - **max input current** I_lim [A]
     *
     * Each cycle the derived input current is computed from the FILTERED
     * power and the FILTERED supply voltage:
     *
     *     i_in = p_filt / v_bus_filt     [A]
     *
     * The fault trips when EITHER criterion is met:
     *
     *     |p_filt| > P_lim   OR   |i_in| > I_lim
     *
     * i.e. the effective current threshold is implicitly
     * min(I_lim, P_lim / v_bus_filt) — at low bus voltage the current limit
     * becomes the binding constraint before the power limit is reached.
     * Tripping starts an atomic recovery timer (same pattern as OC/OV/UV);
     * while it counts down DriverPWM keeps FAULT_OVERPOWER active.
     *
     * ## Available-Power Coefficient
     *
     * Every tick the module also computes the signed power coefficient:
     *
     *     power_coeff = sat(p_filt / P_lim, -1, +1)
     *
     * Positive = share of the power limit currently invested into the motor,
     * negative = share being regenerated back. |power_coeff| == 1.0 means the
     * limit has been reached.
     *
     * @param current_ab Measured alpha-beta phase currents [A]
     *                   (alpha ~ phase A, beta ~ phase B)
     * @param supply_volts Filtered DC-bus voltage [V] from SupplyVoltage
     */
    inline void tick(Vec2D_f32 current_ab, float supply_volts) {
        filt_voltage_v_ = supply_volts;  // Cache filtered bus voltage for current derivation

        // Instantaneous electrical power in the alpha-beta frame (signed):
        // p = v_alpha*i_alpha + v_beta*i_beta (Clarke-transform invariant)
        raw_power_w_ = cmd_v_.a * current_ab.a + cmd_v_.b * current_ab.b;

        // First-order EMA low-pass: y += alpha*(x - y), alpha in [0..1]
        filt_power_w_ += ((float)filter_alpha_ * (1.0f / 255.0f)) * (raw_power_w_ - filt_power_w_);

        // Derived input current from FILTERED power and FILTERED voltage.
        // Guard against division by a near-zero/degenerate bus voltage.
        filt_current_a_ = (filt_voltage_v_ > 1.0f) ? (filt_power_w_ / filt_voltage_v_) : 0.0f;

        // Absolute magnitudes — limits are symmetric in + and -
        const float abs_p = std::fabs(filt_power_w_);
        const float abs_i = std::fabs(filt_current_a_);

        // Per-tick utilization: fraction of the binding limit [0..>1]
        usage_ = abs_p * (1.0f / lim_power_w_);
        const float usage_i = abs_i * (1.0f / lim_current_a_);
        if (usage_i > usage_) {
            usage_ = usage_i;
        }

        // Signed power coefficient: share of the max-input-power limit at the
        // current moment (+ = motoring / power investment, - = regeneration),
        // saturated to [-1..+1]. Magnitudes above 1.0 mean the OP fault zone.
        const float coeff = filt_power_w_ * (1.0f / lim_power_w_);
        power_coeff_ = (coeff > 1.0f) ? 1.0f : ((coeff < -1.0f) ? -1.0f : coeff);

        // OP criteria: EITHER the power or the derived current exceeds its
        // symmetric limit -> restart the recovery timer (fault latches while
        // the timer runs, auto-clears when it expires within limits).
        if (abs_p > lim_power_w_ || abs_i > lim_current_a_) {
            overpower.trigger();
        }
    }

    // =======================================================================
    // GETTERS (raw and filtered outputs run in parallel)
    // =======================================================================

    /**
     * @brief Latest RAW (unfiltered) power-stage power.
     * @return Instantaneous electrical power [W], signed (+ motor / - regen)
     */
    inline float get_raw_power_w() const { return raw_power_w_; }

    /**
     * @brief Latest filtered power-stage power.
     * @return EMA-filtered electrical power [W], signed (+ motor / - regen)
     */
    inline float get_power_w() const { return filt_power_w_; }

    /**
     * @brief Derived input current from filtered power and filtered voltage.
     * @return Input current estimate [A], signed
     */
    inline float get_input_current_a() const { return filt_current_a_; }

    /**
     * @brief Per-tick limit utilization coefficient (percent of usage).
     * @return max(|P|/P_lim, |I|/I_lim); < 1.0 = headroom, >= 1.0 = fault zone
     */
    inline float get_usage_norm() const { return usage_; }

    /**
     * @brief Signed power coefficient — share of the max input power at the
     *        current moment.
     *
     * Computed every tick as the filtered power normalized by the symmetric
     * power limit and saturated to [-1..+1]:
     *
     *     power_coeff = sat(P_filt / P_lim, -1, +1)
     *
     * Sign carries the energy-flow direction: positive values mean motoring
     * (power drawn from the supply), negative values mean regeneration
     * (power returned to the link). |coeff| == 1.0 means the power limit has
     * just been reached in the corresponding direction.
     *
     * @return Normalized signed power share [-1..+1]
     */
    inline float get_power_coeff() const { return power_coeff_; }

    // =======================================================================
    // CONFIGURATION
    // =======================================================================

    /**
     * @brief Set the EMA filter coefficient.
     * @param alpha Filter coefficient 0..255 (255 = no filtering, 0 = frozen)
     */
    inline void set_filter_alpha(uint8_t alpha) { filter_alpha_ = alpha; }

    /**
     * @brief Chainable limit setter for the symmetric OP thresholds.
     *
     * Usage:
     * @code
     *   driver.power.set_limit().op_lim_watts(60.0f).op_lim_amps(8.0f);
     * @endcode
     *
     * Limits are signed-symmetric: the applied magnitude is enforced both in
     * the + (motoring) and - (regeneration) directions.
     */
    struct LimitProxy {
    private:
        Power& parent;  ///< Reference to the owning Power

    public:
        /**
         * @brief Construct a LimitProxy bound to a Power.
         * @param p Reference to the owning Power
         */
        LimitProxy(Power& p) : parent(p) {}

        /**
         * @brief Set the symmetric max-input-power limit.
         * @param limit_watts Power limit in watts (sign ignored; applied to +/-)
         * @return Reference to this setter for chaining
         */
        inline LimitProxy& op_lim_watts(float limit_watts) {
            parent.lim_power_w_ = std::fabs(limit_watts);  // Store magnitude for symmetric enforcement
            return *this;
        }

        /**
         * @brief Set the symmetric max-input-current limit.
         * @param limit_amps Current limit in amperes (sign ignored; applied to +/-)
         * @return Reference to this setter for chaining
         */
        inline LimitProxy& op_lim_amps(float limit_amps) {
            parent.lim_current_a_ = std::fabs(limit_amps);  // Store magnitude for symmetric enforcement
            return *this;
        }
    };

    /**
     * @brief Begin a limit configuration chain.
     * @return LimitProxy bound to this Power
     */
    inline LimitProxy set_limit() { return LimitProxy(*this); }
};
