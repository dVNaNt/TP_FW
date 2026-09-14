/**
 * @file pt_control.h
 * @brief P*t (loss-power) overdrive limiter with motor thermal derating.
 *
 * Implements the motor protection model behind the TPCP registers 0x23
 * (P_Loss_t properties and overdrive) and 0x24 (motor thermal protection).
 * All the parameters the protocol exposes are coupled, and this module is the
 * single place where they meet:
 *
 * - **Thermal derating** (0x24: temp_base, temp_limit): the allowed power is
 *   a linear ramp - 100 % at the baseline temperature, 0 % at the limit - so
 *   the slope is implied by the temperature span itself and no separate
 *   per-degC parameter exists. The motor is gently squeezed instead of
 *   tripped when it runs hot.
 * - **Energy budget** (0x23: baseline, peak, capacity): every watt drawn above
 *   the power baseline drains a Joule budget; power at or below the baseline is
 *   free. The budget is the overdrive reserve - while it lasts, the drive may
 *   command up to the (derated) peak power.
 * - **Overdrive duration** (0x23: overdrive_s): the time the motor may safely
 *   sit exactly at peak power. It caps the budget at
 *   `(peak - baseline) * duration`, so a large capacity cannot buy an endless
 *   burst. A zero duration disables overdrive entirely.
 * - **Thermal resistance** (0x24: Rth): the dissipation channel. The motor
 *   sheds `p_cool = (T - T_amb) / Rth` watts at temperature T, and the ambient
 *   is estimated from the rating point - at `T = temp_base` the motor
 *   dissipates exactly the baseline, hence `T_amb = temp_base - baseline*Rth`.
 *   The freed power refills the budget, so a hot motor with a low Rth recovers
 *   its burst capability faster. The budget itself is additionally clamped by
 *   the remaining thermal headroom `(temp_limit - T)`, scaled against the
 *   baseline-to-limit span: at the temperature limit no overdrive energy is
 *   available at all.
 * - **Winding-temperature observer**: when no temperature sensor reports (or
 *   it reports "no data"), the module integrates its own first-order thermal
 *   estimate from the power history - using the same Rth and the heat
 *   capacity implied by the budget calibration - so the derating and the
 *   thermal clamp keep working even sensorless. A valid measurement re-syncs
 *   the estimate instantly.
 *
 * ## Tick contract
 *
 * `tick()` is called at a strict, constant period: the period itself is
 * schedule configuration, installed once via `set_dt()` (a call that happens
 * only when the loop rate actually changes), not a per-tick argument. Each
 * tick receives the measured electrical power and the measured motor
 * temperature, advances the budget and refreshes three published values,
 * kept as internal state and readable at any time through the getters:
 *
 * - `get_nominal_w()`   : continuous power allowed right now (derated
 *   baseline),
 * - `get_overdrive_w()` : burst power ceiling right now (derated peak, scaled
 *   by the remaining headroom so the ceiling decays smoothly to the nominal
 *   power as the budget empties instead of stepping),
 * - `get_headroom()`    : fraction of the overdrive energy budget still
 *   available (0..1); 0 means overdrive is disabled or spent.
 *
 * All three outputs are low-passed on the OUTPUT side - after every
 * protection computation: the nominal power with a ~0.5 s time constant, the
 * burst ceiling and the headroom with ~0.25 s, so configuration changes and
 * budget events reach the power path as smooth ramps instead of steps.
 *
 * ## Efficiency
 *
 * Everything the tick needs that does not depend on the samples (reciprocal
 * spans, per-degC gains, budget ceiling, its reciprocal) is precomputed once
 * per configuration change. The hot path is a handful of FMAs with FPU-friendly
 * clamping branches and a single multiply in place of the headroom division -
 * no `sqrt`, no `log`, no runtime divisions. The output low-pass filters use
 * the same trick: their alphas are the period multiplied by precomputed 1/tau
 * gains, sampled once per set_dt() call and cached for the hot path.
 *
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#pragma once

#include <cmath>
#include <cstdint>
#include "common.h"

/**
 * @struct PtControl
 * @brief P*t overdrive limiter and temperature-driven power derating.
 *
 * Owns the coupled motor-protection state: the configuration shadow mirroring
 * registers 0x23 / 0x24, the derived constants (computed once per write) and
 * the live energy budget. Pure algorithm - no hardware, no timing source of
 * its own; the caller feeds it at a fixed period.
 */
struct PtControl {
private:
    // =======================================================================
    // CONFIGURATION (register shadow)
    // =======================================================================
    uint8_t overdrive_s_ = 10u;     ///< 0x23: burst duration [s]; 0 disables overdrive.
    float baseline_w_ = 5.0f;      ///< 0x23: power free of charge [W].
    float peak_w_ = 10.0f;          ///< 0x23: maximum allowable power [W].
    float capacity_j_ = 100.0f;      ///< 0x23: overdrive energy budget [J].
    float temp_base_c_ = 60.0f;    ///< 0x24: full-power threshold [degC] (100 % power).
    float temp_limit_c_ = 100.0f;  ///< 0x24: maximum winding temperature [degC] (0 % power).
    float rth_ = 0.0f;             ///< 0x24: thermal resistance [degC/W].
    // Note: the 0x24 wire field `derating` is deliberately not consumed - the
    // slope is implied by the temperature span (100 % across T_limit - T_base).

    // =======================================================================
    // DERIVED CONSTANTS (refreshed on every configuration write)
    // =======================================================================
    float cool_gain_ = 0.10f;    ///< 1 / Rth: dissipated watts per degC above ambient.
    float t_amb_c_ = 0.0f;      ///< Estimated ambient: temp_base - baseline * Rth.
    float inv_span_c_ = 0.0f;   ///< 1 / (temp_limit - temp_base): budget clamp gain.
    float e_max_j_ = 0.0f;      ///< Budget ceiling: min(capacity, (peak-baseline)*duration).
    float inv_e_max_j_ = 0.0f;  ///< 1 / e_max_j_ (0 when overdrive is off): headroom gain.
    bool span_valid_ = false;   ///< True when temp_limit > temp_base (clamp active).

    // =======================================================================
    // OUTPUT FILTER CONSTANTS
    // =======================================================================
    // Published outputs are low-passed after the protection math (the filter
    // acts on the output, never on the measurements). First-order EMA with a
    // per-tick alpha dt/tau, precomputed as dt * (1 / tau).
    static constexpr float NOMINAL_INV_TAU_S_ = 2.0f;  ///< 1 / 0.5 s: nominal-power output LPF gain.
    static constexpr float OUTPUT_INV_TAU_S_ = 4.0f;   ///< 1 / 0.25 s: burst output LPF gain.

    // =======================================================================
    // SCHEDULING (set once via set_dt(); tick() is strictly periodic)
    // =======================================================================
    float dt_s_ = 0.0f;       ///< Cached scheduling period [s]; 0 = outputs frozen.
    float alpha_nom_ = 0.0f;  ///< Cached EMA gain, nominal channel (dt * 1/0.5 s).
    float alpha_out_ = 0.0f;  ///< Cached EMA gain, burst channels (dt * 1/0.25 s).

    // =======================================================================
    // LIVE STATE
    // =======================================================================
    float energy_j_ = 0.0f;     ///< Remaining overdrive energy budget [J].
    float headroom_ = 0.0f;     ///< Filtered energy headroom [0..1].
    float nominal_w_ = 0.0f;    ///< Filtered continuous power limit [W].
    float overdrive_w_ = 0.0f;  ///< Filtered burst power ceiling [W].

    // =======================================================================
    // WINDING-TEMPERATURE OBSERVER (sensorless fallback)
    // =======================================================================
    float temp_est_c_ = 60.0f;  ///< Observed winding temperature [degC].
    float inv_cth_j_c_ = 0.0f;  ///< 1 / C_th [degC/J]: observer gain (0 = disabled).
    bool obs_valid_ = false;    ///< True when span and capacity arm the observer.

public:
    /// Default constructor: everything off, nominal power zero until configured.
    PtControl() = default;

    // =======================================================================
    // CONFIGURATION
    // =======================================================================

    /**
     * @brief Begin a configuration chain (one call per register write).
     *
     * Float fields follow the wire semantics: 0, NaN and +-inf are ignored
     * (the previous value survives), so a read-modify-write of one register
     * never clobbers the other. The duration keeps 0 as a valid value because
     * "0 duration disables overdrive" is itself a configuration.
     */
    struct ConfigProxy {
    private:
        PtControl& parent;  ///< Reference to the owning PtControl.

    public:
        /// @param p Reference to the owning PtControl.
        explicit ConfigProxy(PtControl& p) : parent(p) {}

        /// Set the overdrive duration [s]; 0 disables overdrive.
        inline ConfigProxy& overdrive_s(uint8_t seconds) {
            parent.overdrive_s_ = seconds;  // 0 is meaningful: overdrive off
            parent.refresh();
            return *this;
        }

        /// Set the power baseline [W] (0 / NaN / inf ignored).
        inline ConfigProxy& baseline_w(float watts) {
            if (std::isfinite(watts) && watts > 0.0f) {
                parent.baseline_w_ = watts;  // Store the free-power threshold
                parent.refresh();
            }
            return *this;
        }

        /// Set the peak (burst) power [W] (0 / NaN / inf ignored).
        inline ConfigProxy& peak_w(float watts) {
            if (std::isfinite(watts) && watts > 0.0f) {
                parent.peak_w_ = watts;  // Store the burst ceiling
                parent.refresh();
            }
            return *this;
        }

        /// Set the overdrive energy budget [J] (0 / NaN / inf ignored).
        inline ConfigProxy& capacity_j(float joules) {
            if (std::isfinite(joules) && joules > 0.0f) {
                parent.capacity_j_ = joules;  // Store the energy reserve
                parent.refresh();
            }
            return *this;
        }

        /// Set the full-power threshold temperature [degC] (NaN / inf ignored).
        inline ConfigProxy& temp_base_c(float celsius) {
            if (std::isfinite(celsius)) {
                parent.temp_base_c_ = celsius;  // Rating point (100 % power)
                parent.refresh();
            }
            return *this;
        }

        /// Set the maximum winding temperature [degC] (NaN / inf ignored).
        inline ConfigProxy& temp_limit_c(float celsius) {
            if (std::isfinite(celsius)) {
                parent.temp_limit_c_ = celsius;  // Hard thermal ceiling
                parent.refresh();
            }
            return *this;
        }

        /// Set the thermal resistance [degC/W] (0 / NaN / inf ignored).
        inline ConfigProxy& rth(float degc_per_watt) {
            if (std::isfinite(degc_per_watt) && degc_per_watt > 0.0f) {
                parent.rth_ = degc_per_watt;  // Dissipation channel gain
                parent.refresh();
            }
            return *this;
        }
    };

    /// Begin a configuration chain bound to this instance.
    inline ConfigProxy configure() { return ConfigProxy(*this); }

    /**
     * @brief Set the strict scheduling period of tick() [s].
     *
     * tick() runs at a strictly constant period, so the period is schedule
     * configuration, not a per-tick argument. It is sampled here - a call
     * that happens only when the loop rate actually changes (boot, or a rare
     * reconfiguration) - and the output-filter gains are precomputed from it,
     * keeping tick() free of divisions and isfinite checks. A non-finite or
     * non-positive period freezes both the budget integration and the output
     * filters (gains become zero).
     *
     * @param dt_s Scheduling period [s].
     */
    inline void set_dt(float dt_s) {
        dt_s_ = (std::isfinite(dt_s) && dt_s > 0.0f) ? dt_s : 0.0f;  // Sanitize once
        alpha_nom_ = dt_s_ * NOMINAL_INV_TAU_S_;                     // ~0.5 s nominal channel
        alpha_out_ = dt_s_ * OUTPUT_INV_TAU_S_;                      // ~0.25 s burst channels
        if (alpha_nom_ > 1.0f) {
            alpha_nom_ = 1.0f;  // dt >> tau: snap, no overshoot
        }
        if (alpha_out_ > 1.0f) {
            alpha_out_ = 1.0f;  // dt >> tau: snap, no overshoot
        }
    }

    // =======================================================================
    // PROCESSING
    // =======================================================================

    /**
     * @brief Advance the protection model by one scheduling period.
     *
     * Called at the strict constant period configured by set_dt(). The energy
     * budget is integrated over that period: watts above the baseline drain
     * it, and the dissipation the Rth model grants at the current temperature
     * refills it. The budget is then clamped by the thermal headroom
     * remaining up to the temperature limit. Finally the instantaneous power
     * targets and the headroom are computed and low-passed on the output side
     * (nominal ~0.5 s, burst outputs ~0.25 s) so the caller sees smooth ramps
     * instead of steps.
     *
     * Cost: ~15 FMAs and five clamp branches on the Cortex-M4F, no divisions
     * and no isfinite checks on the hot path (the period and its filter gains
     * are precomputed in set_dt(), the remaining reciprocals in refresh()).
     *
     * @param power_w Filtered electrical power [W], signed (+ motor / - regen).
     * @param temp_c  Motor temperature [degC]; non-positive = no data, the
     *                observer (when armed) takes over, else the rating point.
     */
    inline void tick(float power_w, float temp_c) {
        // --- 0. Temperature source ------------------------------------------
        // A valid measurement (T > 0 degC) is the truth and re-syncs the
        // observer instantly. Without data the winding observer takes over
        // when it is armed (capacity + span configured); otherwise the blind
        // fallback assumes the rating point - no derating, loosest clamp:
        // safe, just not smart. Note T <= 0 counts as "no data": register
        // 0x24 carries the temperature as an unsigned byte, so negative
        // ambients cannot be reported anyway, and substituting T_base is the
        // conservative choice for a colder-than-baseline motor (k clamps to
        // 1, the thermal clamp sits at its loosest).
        float temp;
        if (temp_c > 0.0f) {
            temp = temp_c;         // Measurement is the truth
            temp_est_c_ = temp_c;  // Observer follows it
        } else {
            temp = obs_valid_ ? temp_est_c_ : temp_base_c_;  // Observed or rating fallback
        }

        // --- 1. Thermal derating --------------------------------------------
        // The allowed power is a linear ramp between the rating points:
        //   k = (T_limit - T) / (T_limit - T_base)
        // i.e. 100 % at T_base, 0 % at T_limit, clamped to [0..1]. The slope
        // is implied by the span itself - no separate per-degC parameter.
        // An invalid span (limit <= base) leaves k at 1: no sensible ramp.
        const float over_base = temp - temp_base_c_;  // Degrees above the rating point
        float k = 1.0f;                               // No span: full power
        if (span_valid_) {
            k = (temp_limit_c_ - temp) * inv_span_c_;  // Remaining power fraction
            if (k > 1.0f) {
                k = 1.0f;  // Cold motor: full power
            } else if (k < 0.0f) {
                k = 0.0f;  // At/past the limit: shut down
            }
        }

        // --- 2. Energy budget integration ------------------------------------
        if (power_w > baseline_w_) {
            // API rule: every watt above the baseline drains the budget.
            energy_j_ -= (power_w - baseline_w_) * dt_s_;
        } else {
            // Dissipation refills the budget. At T = T_base the motor
            // sheds exactly the baseline (t_amb = T_base - baseline*Rth),
            // so the recovery equals (baseline - P); above the baseline
            // temperature the surplus cooling speeds the recovery up.
            const float p_cool = baseline_w_ + over_base * cool_gain_;  // Watts shed right now
            const float refill = p_cool - power_w;                      // Net watts into the reserve
            if (refill > 0.0f) {
                energy_j_ += refill * dt_s_;
            }
        }

        // Thermal headroom clamp: the budget can never exceed the energy
        // the windings may still absorb before the limit temperature
        //   cap = capacity * (T_limit - T) / (T_limit - T_base).
        float cap = e_max_j_;  // Duration/energy ceiling
        if (span_valid_) {
            const float thermal_cap = capacity_j_ * (1.0f - over_base * inv_span_c_);
            if (thermal_cap < cap) {
                cap = thermal_cap;  // Temperature is the binding limit
            }
        }
        if (energy_j_ > cap) {
            energy_j_ = cap;  // Shrink with a heating motor
        } else if (energy_j_ < 0.0f) {
            energy_j_ = 0.0f;  // Never owe energy
        }

        // Winding-temperature observer: integrate the same first-order
        // thermal model, heating with the FULL electrical power (all of
        // it is treated as loss - conservative, the mechanical output is
        // ignored) and cooling through the Rth channel. Clamped to
        // [T_amb, T_limit] so a stale model cannot wander into nonsense.
        // The observer shares C_th with the budget clamp, so the two
        // integrators stay physically consistent.
        if (obs_valid_) {
            const float est_base = temp_est_c_ - temp_base_c_;             // Degrees above the rating point
            const float p_cool_est = baseline_w_ + est_base * cool_gain_;  // Cooling at the estimate
            temp_est_c_ += (power_w - p_cool_est) * inv_cth_j_c_ * dt_s_;
            if (temp_est_c_ > temp_limit_c_) {
                temp_est_c_ = temp_limit_c_;  // Saturate at the limit
            } else if (temp_est_c_ < t_amb_c_) {
                temp_est_c_ = t_amb_c_;  // Never colder than the ambient
            }
        }

        // --- 3. Instantaneous targets ----------------------------------------
        // Headroom straight from the (unfiltered) energy state; the
        // precomputed reciprocal turns the division into a multiply.
        const float headroom_raw = energy_j_ * inv_e_max_j_;

        // --- 4. Output filtering ---------------------------------------------
        // The published limits are low-passed on the OUTPUT side, after every
        // protection computation, with the gains cached by set_dt():
        //   y += alpha * (x - y),  alpha = dt/tau, clamped at 1 in set_dt().
        // A zero period (set_dt never called or invalid) yields zero gains,
        // so these updates become no-ops and the outputs stay frozen
        // together with the budget.

        nominal_w_ += (baseline_w_ * k - nominal_w_) * alpha_nom_;  // Derated continuous power
        headroom_ += (headroom_raw - headroom_) * alpha_out_;       // Fuel gauge
        overdrive_w_ += ((baseline_w_ + (peak_w_ - baseline_w_) * k * headroom_raw) - overdrive_w_) *
                        alpha_out_;  // Ceiling decays with the budget
    }

    // =======================================================================
    // STATE ACCESS (diagnostics / telemetry)
    // =======================================================================

    /// Remaining overdrive energy [J].
    inline float get_energy_j() const { return energy_j_; }

    /// Filtered continuous power limit [W] (LPF ~0.5 s).
    inline float get_nominal_w() const { return nominal_w_; }

    /// Filtered burst power ceiling [W] (LPF ~0.25 s).
    inline float get_overdrive_w() const { return overdrive_w_; }

    /// Filtered energy headroom [0..1] (LPF ~0.25 s); 0 = overdrive off/spent.
    inline float get_headroom() const { return headroom_; }

    /// Winding temperature in force: the last measurement or the observer
    /// estimate when the sensor is silent [degC].
    inline float get_temperature_c() const { return temp_est_c_; }

    /// True when the sensorless observer is armed (capacity + span present).
    inline bool is_observer_armed() const { return obs_valid_; }

    /// Budget ceiling currently in force [J] (already the smaller of the
    /// capacity and the duration products; the temperature clamp is dynamic).
    inline float get_budget_j() const { return e_max_j_; }

    /// Estimated ambient temperature the Rth model dissipates against [degC].
    inline float get_ambient_c() const { return t_amb_c_; }

    /// Refill the budget completely (e.g. after a disable or a cold start).
    inline void reset() {
        energy_j_ = e_max_j_;  // Full reserve available again
        headroom_ = energy_j_ * inv_e_max_j_;
    }

private:
    /**
     * @brief Recompute every derived constant from the configuration shadow.
     *
     * All divisions and sanity multiplications happen here, once per register
     * write, so tick() stays division-free. An inconsistent configuration
     * (peak below baseline, inverted temperature span, missing Rth) degrades
     * gracefully: overdrive simply ends up disabled.
     */
    void refresh() {
        // Dissipation channel: watts per degC above the estimated ambient.
        cool_gain_ = (rth_ > 0.0f) ? (1.0f / rth_) : 0.0f;

        // Ambient from the rating point: at T = temp_base the motor must be
        // able to shed the baseline forever (otherwise the rating is a lie),
        // so T_amb = T_base - baseline * Rth.
        t_amb_c_ = temp_base_c_ - baseline_w_ * rth_;

        // Temperature span for the thermal budget clamp.
        const float span_c = temp_limit_c_ - temp_base_c_;
        span_valid_ = (span_c > 0.0f);
        inv_span_c_ = span_valid_ ? (1.0f / span_c) : 0.0f;

        // Energy a full-duration burst at peak would consume above baseline.
        const float burst_j = (peak_w_ - baseline_w_) * (float)overdrive_s_;

        // The budget is whichever limit is tighter; a negative result (peak
        // under baseline) or a zero duration both disable overdrive.
        float e_max = capacity_j_;
        if (burst_j < e_max) {
            e_max = burst_j;  // Duration is binding
        }
        e_max_j_ = (e_max > 0.0f) ? e_max : 0.0f;

        // Reciprocal for the headroom multiply; 0 keeps the headroom at zero
        // when overdrive is off (energy * 0).
        inv_e_max_j_ = (e_max_j_ > 0.0f) ? (1.0f / e_max_j_) : 0.0f;

        // Winding-temperature observer: C_th is implied by the budget
        // calibration (the capacity is the energy of the full span), so
        // C_th = capacity / span and the observer gain is span / capacity.
        // A missing capacity or an inverted span disarms the observer: the
        // module then falls back to the measurement or to the rating point.
        obs_valid_ = span_valid_ && (capacity_j_ > 0.0f);
        inv_cth_j_c_ = obs_valid_ ? (span_c / capacity_j_) : 0.0f;

        // A configuration write resets the drift-prone observer to the
        // rating point; a live measurement re-syncs it on the very next tick.
        temp_est_c_ = temp_base_c_;

        // A configuration change re-scales the live budget into the new
        // ceiling so a shrinking configuration cannot leave phantom energy.
        if (energy_j_ > e_max_j_) {
            energy_j_ = e_max_j_;
        }

        // Snap the output filters to the new limits at once: a configuration
        // write is a deliberate act, not noise, so it must not spend ~0.5 s
        // slewing through the LPF. Temperature is unknown here - assume the
        // baseline (k = 1) and full instantaneous headroom.
        headroom_ = energy_j_ * inv_e_max_j_;
        nominal_w_ = baseline_w_;
        overdrive_w_ = baseline_w_ + (peak_w_ - baseline_w_) * headroom_;
    }
};
