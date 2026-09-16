#pragma once

/**
 * @file motor_control.h
 * @brief Unified motor-control phase stage combining inverse-Clarke voltage
 *        synthesis, Clarke current recombination, phase-pattern remapping and
 *        SVPWM centering/scaling.
 *
 * MotorControl provides two data paths used every control cycle:
 *
 * - @c tick_voltage : alpha-beta command voltages -> remapped 3-phase PWM
 *   duties (inverse Clarke for BLDC, direct coil mapping for steppers), with
 *   SVPWM zero-sequence centering or linear scale-down on saturation.
 * - @c tick_current : raw per-channel currents -> alpha-beta currents
 *   (Clarke transform), reconstructing a missing phase via Kirchhoff's
 *   Current Law and masking sensors sampled during high-duty intervals.
 *
 * MotorControl is the PHASE-DOMAIN I/O HUB of the driver: it owns both
 * phase-domain tensors — the commanded voltage vector (converted to duties by
 * @c set_voltage_ab_volts) and the measured alpha-beta currents published by
 * CurrentSense into @c current_ab_amps. All other modules read/write these
 * quantities here instead of keeping their own copies.
 *
 * Phase-pattern remapping (ABC / ACB / CBA wiring) is folded into both paths
 * using precomputed index permutations for zero runtime overhead.
 *
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#include "../../common.h"
#include "power.h"
#include "voltage.h"
#include "pwm_control.h"

struct DriverPWM;
struct CurrentSense;

struct MotorControl {
private:
    friend struct DriverPWM;
    friend struct CurrentSense;
    // --- Collaborators (injected once via attach()) ---
    PwmControl* pwm_ = nullptr;        ///< PWM output backend receiving computed duties
    Power* power_ = nullptr;           ///< Power estimator fed with commanded vectors
    SupplyVoltage* supply_ = nullptr;  ///< Bus-voltage scaler used by the voltage path

    // --- Phase-domain state ---
    /// Latest measured alpha-beta phase currents [A] (published by CurrentSense)
    Vec2D_f32 current_ab_amps = {0.0f, 0.0f};

    // --- State variables for Current processing (formerly ABC2AB) ---
    FilterEMA lpf_;                           // LPF filter for phase balance
    static constexpr uint8_t LPF_ALPHA = 42;  // Filter coefficient (0-255)

    int16_t epsilon_abs_;  // Absolute balance tolerance in Q15 format
    int16_t epsilon_rel_;  // Relative balance tolerance: Q15 fraction of the summed channel magnitudes
    Vec3D abc_voltages_;   // ABC voltages after phase remapping

    MotorType motor_type_;  // Motor type: 1=DC, 4=Stepper, 6=BLDC
    // Integrated phase remapping indices (precomputed for zero overhead)
    uint8_t idx0_;  // Index for output channel 0
    uint8_t idx1_;  // Index for output channel 1
    uint8_t idx2_;  // Index for output channel 2

    PhasePattern phase_pattern_;  // Phase connection pattern

    // Error flags
    uint8_t err_voltage_;  // Error flag for voltage path
    uint8_t err_current_;  // Error flag for current path

    // ============================================================================
    // Internal method: Voltage AB to ABC converter with SVPWM (formerly AB2ABC::tick)
    // ============================================================================
    Vec3D ab2abc_tick(Vec2D input, uint32_t motor_type, uint8_t& error) {
        int32_t voltg_a = input.alpha;
        int32_t voltg_b = input.beta;
        int32_t voltg_c;  // Temporary 32-bit voltages for calculations
        error = false;
        const int32_t neg_voltg_a_div2 = -voltg_a >> 1;  // precalculated value for math

        if (motor_type == MotorType_BLDC) {  // Handle BLDC motor type (3-phase with Clarke Transform)
            // sqrt(3)/2 in Q16.16 fixed-point format (floor(0.8660254037844386 * 65536) = 56755)
            constexpr static int32_t VOLTAGE_SQRT3DIV2_Q16_16 = 56755;
            // Calculate beta * sqrt(3)/2 using Q16.16 fixed-point multiplication
            int32_t beta_sqrt3_div2 = (VOLTAGE_SQRT3DIV2_Q16_16 * voltg_b) >> 16;

            // Phase A = V_alpha by default
            voltg_b = neg_voltg_a_div2 + beta_sqrt3_div2;  // Phase B = -V_alpha/2 + V_beta*sqrt(3)/2
            voltg_c = neg_voltg_a_div2 - beta_sqrt3_div2;  // Phase C = -V_alpha/2 - V_beta*sqrt(3)/2
        } else if (motor_type == MotorType_STEP) {         // Handle Stepper motor type (direct 2-phase mapping)
            // Phase A = alpha input (coil A) by default
            // Phase B = beta input (coil B) by default
            voltg_c = 0;  // Phase C: unused for stepper
        } else {          // Handle unexpected invalid motor type
            error = true;
            return Vec3D(INT16_MIN, INT16_MIN, INT16_MIN);  // Store phase C result
        }

        // Find minimum and maximum voltage among all three phases
        int32_t min_ab, max_ab;
        if (voltg_a < voltg_b) {
            min_ab = voltg_a;
            max_ab = voltg_b;
        } else {
            min_ab = voltg_b;
            max_ab = voltg_a;
        }
        int32_t voltg_min = (min_ab < voltg_c) ? min_ab : voltg_c;
        int32_t voltg_max = (max_ab > voltg_c) ? max_ab : voltg_c;

        int32_t voltg_offset;                              // Offset for SVPWM centering
        int32_t voltg_full_scale = voltg_max - voltg_min;  // Calculate peak-to-peak range

        // Check if voltage range exceeds available supply
        if (voltg_full_scale < INT16_MAX) {
            // Voltage fits - use SVPWM centering for optimal utilization
            voltg_offset = (INT16_MAX - voltg_max - voltg_min) >> 1;  // Center the voltage vector
            voltg_a += voltg_offset;                                  // Apply offset to phase A
            voltg_b += voltg_offset;                                  // Apply offset to phase B
            voltg_c += voltg_offset;                                  // Apply offset to phase C
        } else {
            // Scale down voltages to fit within max_output
            uint32_t scale = ((uint32_t)INT16_MAX << 15) / (uint32_t)voltg_full_scale;
            voltg_a = ((voltg_a - voltg_min) * scale) >> 15;
            voltg_b = ((voltg_b - voltg_min) * scale) >> 15;
            voltg_c = ((voltg_c - voltg_min) * scale) >> 15;  // Offset to shift into positive range
        }

        // Convert 32-bit results back to 16-bit outputs
        return Vec3D((int16_t)(voltg_a), (int16_t)(voltg_b), (int16_t)(voltg_c));  // Store phase C result
    }

    // ============================================================================
    // Internal method: ABC to AB current converter with sensor recombination
    // ============================================================================
    Vec2D abc2ab_tick(Vec3D abc, MotorType mt) {
        const bool valid_a = (abc.a != INT16_MIN);
        const bool valid_b = (abc.b != INT16_MIN);
        const bool valid_c = (abc.c != INT16_MIN);

        const int32_t present_cnt = (int32_t)valid_a + (int32_t)valid_b + (int32_t)valid_c;

        // Validate motor type is supported or no current sense we have only 1 current for
        // BLDC/STEP (short exit)
        if (present_cnt == 1) {
            err_current_ = true;
            return Vec2D(INT16_MIN, INT16_MIN);  // Exit early on invalid motor type
        }

        // making invalid currents as zero to simplify balance math
        int32_t ia = (abc.a != INT16_MIN) ? abc.a : 0;
        int32_t ib = (abc.b != INT16_MIN) ? abc.b : 0;
        int32_t ic = (abc.c != INT16_MIN) ? abc.c : 0;

        const int32_t sum = clamp_i32(ia + ib + ic, INT16_MAX);
        if (present_cnt < 3) {  // speedup logic if all 3 sensors present
            // Recombine missing phase using Kirchhoff's Current Law (ia + ib + ic = 0)
            if (!valid_a)
                ia = -sum;  // Phase A missing: ia = -(ib + ic)
            else if (!valid_b)
                ib = -sum;  // Phase B missing: ib = -(ia + ic)
            else if (!valid_c)
                ic = -sum;  // Phase C missing: ic = -(ia + ib)
        }

        int16_t alpha;
        int16_t beta;

        // BLDC/PMSM: full Clarke transform to the stationary alpha-beta frame.
        if (mt == MotorType_BLDC) {
            constexpr static int32_t INV_SQRT3_Q15 = 18919;  // 1/sqrt(3) in Q15 format (floor(0.577350269 * 2^15) = 18918.6)
            alpha = ia;
            // Calculate beta: I_beta = (I_B - I_C) * 1/sqrt(3)
            beta = clamp_i32(((ib - ic) * INV_SQRT3_Q15) >> 15, INT16_MAX);
        } else if (mt == MotorType_STEP) {
            // Stepper: two independent coils map directly onto alpha/beta.
            alpha = ia;
            beta = ib;
        } else {
            err_current_ = true;
            return Vec2D(INT16_MIN, INT16_MIN);;  // Exit early on invalid motor type
        }

        Vec2D ab_currents_ = {alpha, beta};
        err_current_ = false;  // Clear error flag before the balance check below

        if (present_cnt == 3) {  // Phase-balance supervision (all sensors present)
            // Low-pass filtered sum of the three phase currents should stay
            // near zero (Kirchhoff). A persistent offset indicates a sensor
            // gain/offset fault - or a coil leaking to ground - and raises the
            // current-path error flag.
            //
            // The tolerance has an absolute part (noise and offsets at rest)
            // and a part proportional to the current flowing: three shunts
            // and three amplifiers each carry a few percent of gain error, so
            // the sum of healthy sensors drifts away from zero in proportion
            // to the current. A fixed tolerance sized for standstill tripped
            // the electrical identification of a healthy motor at 3.5 A on the
            // shared channel, where a 4 % mismatch already exceeds it.
            const int32_t magnitude = (ia < 0 ? -ia : ia) + (ib < 0 ? -ib : ib) + (ic < 0 ? -ic : ic);
            const int32_t tolerance = (int32_t)epsilon_abs_ + ((magnitude * (int32_t)epsilon_rel_) >> 15);
            const int16_t bal = lpf_.tick_lpf(sum);
            if (bal > tolerance or bal < -tolerance)  // Outside tolerance...
                err_current_ = true;                  // ...raise error flag
        }
        return ab_currents_;
    }

    /**
     * Constructor: initializes the motor selector.
     *
     * @param motor_type      Motor type enum (MotorType_BLDC / MotorType_STEP).
     * @param phase_pattern   Physical wiring pattern (ABC, ACB, CBA).
     * @param duty_permille   Maximum duty cycle in permille (historical parameter,
     *                        currently unused inside this class).
     * @param current_epsilon Balance tolerance for the current-sum check (Q15).
     */
    MotorControl(MotorType motor_type = MotorType_STEP,
                  PhasePattern phase_pattern = PhasePattern_ABC,
                  uint16_t duty_permille = 995u,
                  int16_t current_epsilon = 256)
        : motor_type_(motor_type),
          phase_pattern_(phase_pattern),
          idx0_(0),
          idx1_(1),
          idx2_(2),
          abc_voltages_(0, 0, 0),
          err_voltage_(0u),
          err_current_(0u),
          lpf_(0, LPF_ALPHA),
          epsilon_abs_(0),
          epsilon_rel_(0) {
        set_phase_pattern(phase_pattern);
        set_current_epsilon(current_epsilon);
        set_current_epsilon_relative(BALANCE_REL_DEFAULT_PERCENT);
    }

    /**
     * Process the voltage path: alpha-beta command -> remapped 3-phase duties.
     *
     * @param voltage Alpha-beta voltages in Q15 (Vec2D first member = alpha,
     *                second member = beta).
     * @return Reference-free copy of the remapped ABC duty vector (Q15).
     */
    Vec3D tick_voltage(Vec2D voltage) {
        // Convert AB to ABC using internal voltage converter
        Vec3D abc = ab2abc_tick(voltage, motor_type_, err_voltage_);

        // Apply inline phase remapping (logical ABC -> physical channels)
        abc_voltages_.a = abc[idx0_];
        abc_voltages_.b = abc[idx1_];
        abc_voltages_.c = abc[idx2_];

        return abc_voltages_;
    }

    /// Process current path: ABC currents -> AB currents
    /// Physical channels are remapped to logical ABC, then converted to AB
    /// Current-sensor masking threshold in Q15 duty: samples taken above this
    /// duty are invalid (shunt not observable) and are replaced by INT16_MIN.
    static constexpr int16_t kDutyMaskThreshold = 30720;  // ~93.8% duty cycle

    /// Process current path: ABC currents -> AB currents
    /// Physical channels are remapped to logical ABC, then converted to AB
    /// @param physical Physical channel currents [Ch0, Ch1, Ch2] in Q15 format (INT16_MIN = sensor missing)
    Vec2D tick_current(Vec3D physical) {
        // Apply high duty cycle masking: if PWM duty exceeds kDutyMaskThreshold,
        // invalidate the corresponding current sensor (sampled during an
        // unobservable window). Combined with inline inverse phase remapping
        // (physical channels -> logical ABC).
        Vec3D logical;
        logical[idx0_] = (abc_voltages_.a > kDutyMaskThreshold) ? INT16_MIN : physical[0];
        logical[idx1_] = (abc_voltages_.b > kDutyMaskThreshold) ? INT16_MIN : physical[1];
        logical[idx2_] = (abc_voltages_.c > kDutyMaskThreshold) ? INT16_MIN : physical[2];

        // Convert logical ABC to AB using internal current converter
        return  abc2ab_tick(logical, motor_type_);
    }

    /// Get pointer to remapped ABC voltages (final output for PWM)
    /// @return Pointer to 3-element array [A, B, C] in Q15 format
    inline Vec3D get_voltages() const { return abc_voltages_; }

public:
    // =======================================================================
    // PHASE-DOMAIN I/O (commanded voltage / measured current)
    // =======================================================================

    /**
     * @brief Wire the collaborators of the phase stage (called once).
     *
     * The phase stage sits between the sensing sub-systems and the PWM
     * backend; instead of holding references at construction (member order
     * constraints) the owner injects them right after construction.
     *
     * @param pwm    PWM output backend receiving the computed duties
     * @param power  Power estimator fed with every commanded vector
     * @param supply Bus-voltage scaler used by the voltage path
     */
    void attach(PwmControl& pwm, Power& power, SupplyVoltage& supply) {
        pwm_ = &pwm;
        power_ = &power;
        supply_ = &supply;
    }

    /**
     * @brief Command alpha-beta voltages [V]; converts and drives the PWM.
     *
     * Single entry point of the voltage path: feeds the commanded vector to
     * the power estimator, scales it into per-phase duties through the
     * supply-voltage scaler, runs the inverse-Clarke/SVPWM stage
     * (@c tick_voltage) and pushes the result into the PWM backend.
     *
     * Non-finite components are safe-state sentinels that bypass conversion:
     * - NaN  -> all phases INT16_MIN ("release outputs" request)
     * - +INF -> all phases INT16_MAX (all-high-side / calibration request)
     *
     * @param voltages Commanded alpha-beta voltages [V]
     */
    void set_voltage_ab_volts(Vec2D_f32 voltages) {
        // Feed the commanded vector to the power estimator (sanitized inside)
        if (power_)
            power_->set_voltage_command(voltages);

        Vec3D duties;
        // NaN sentinel: proper NaN detection (NaN != NaN is always true)
        if (voltages.a != voltages.a || voltages.b != voltages.b) {
            duties = Vec3D{INT16_MIN, INT16_MIN, INT16_MIN};
        }
        // Infinity sentinel: all-high-side command
        else if (voltages.a == INFINITY || voltages.b == INFINITY) {
            duties = Vec3D{INT16_MAX, INT16_MAX, INT16_MAX};
        }
        // Normal path: volts -> duty via measured bus voltage, then remap
        else if (supply_) {
            const int16_t duty_a = supply_->volts2duty(voltages.a);
            const int16_t duty_b = supply_->volts2duty(voltages.b);
            duties = tick_voltage(Vec2D(duty_a, duty_b));
        }

        if (pwm_)
            pwm_->set_duty(duties);
    }

    /**
     * @brief Latest measured alpha-beta phase currents [A].
     *
     * Written by CurrentSense::tick() after normalization and Clarke
     * recombination; read by the power stage estimator and the application
     * layer.
     */
    inline Vec2D_f32 get_current_ab_amps() const { return current_ab_amps; }

    /// Set motor type for both current and voltage converters
    /// @param motor_type Motor type (1=DC, 4=Stepper, 6=BLDC)
    inline void set_motor_type(MotorType motor_type) { motor_type_ = motor_type; }

    /// Get current motor type
    /// @return Motor type (1=DC, 4=Stepper, 6=BLDC)
    inline MotorType get_motor_type() const { return motor_type_; }

    /**
     * Set the physical phase-connection pattern.
     *
     * Precomputed permutations (logical index -> physical channel):
     * - ABC -> [0,1,2]
     * - ACB -> [0,2,1]
     * - CBA -> [2,1,0]
     *
     * @param phase_pattern Phase pattern (ABC, ACB, CBA)
     */
    void set_phase_pattern(PhasePattern phase_pattern) {
        phase_pattern_ = phase_pattern;
        switch (phase_pattern) {
            case PhasePattern_CBA:
                idx0_ = 2;
                idx1_ = 1;
                idx2_ = 0;
                break;
            case PhasePattern_ACB:
                idx0_ = 0;
                idx1_ = 2;
                idx2_ = 1;
                break;
            case PhasePattern_ABC:
            default:
                idx0_ = 0;
                idx1_ = 1;
                idx2_ = 2;
                break;
        }
    }

    /// Get the currently configured phase-connection pattern.
    inline PhasePattern get_phase_pattern() const { return phase_pattern_; }

    /// Set balance tolerance for current converter
    /// @param epsilon Balance tolerance in Q15 format
    inline void set_current_epsilon(int16_t epsilon) { epsilon_abs_ = abs((int32_t)epsilon); }

    /// Default relative balance tolerance: two 3 % amplifiers and 1 % shunts, plus margin.
    static constexpr uint8_t BALANCE_REL_DEFAULT_PERCENT = 8;

    /// Set the relative part of the balance tolerance.
    /// @param percent Allowed |sum| as a percentage of |ia| + |ib| + |ic| (0..100)
    inline void set_current_epsilon_relative(uint8_t percent) {
        const uint32_t p = (percent > 100u) ? 100u : percent;      // Clamp to a fraction
        epsilon_rel_ = (int16_t)((p * 32768u) / 100u);             // Percent -> Q15
    }

    /// Check if voltage path error occurred
    /// @return 1 if error, 0 if no error
    uint8_t is_voltage_err() const { return err_voltage_; }

    /// Check if current path error occurred
    /// @return 1 if error, 0 if no error
    uint8_t is_current_err() const { return err_current_; }

    /// Check if any error occurred
    /// @return 1 if any error, 0 if no errors
    uint8_t is_err() const { return err_voltage_ | err_current_; }
};