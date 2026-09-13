/**
 * @file motor_calibration.h
 * @brief Automatic electrical parameter identification routines.
 *
 * Implements the MotorStatus state machine stages: winding-resistance
 * measurement (DC injection), high-frequency-injection inductance measurement
 * on the D/Q axes, and pole-pair/direction detection during slow rotation.
 *
 * Every fatal calibration path routes through set_error(), which emits a
 * detailed one-shot diagnostic dump over the USB log: the abort reason
 * (overpower / underpower / unrecognized phase connection), the failing
 * stage, detected motor type and phase pattern, supply/injected/limit
 * voltages, measured power against the limits, and the phase currents.
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

// 3-phase PWM driver with FOC (Field Oriented Control) current regulation

#include <cmath>  // For sqrtf and fabsf
#include "../../common.h"
#include "../../drivers/pwm_driver/driver_pwm.h"
#include "../../math/helpers/buzzer.h"
#include "../../math/helpers/frequency_generator.h"  // Frequency generator for HFI
#include "../../math/helpers/stability_check.h"      // Stability check for current monitoring
#include "../../math/helpers/timeout_timer.h"        // Timeout timer for countdown operations
#include "../motor.h"                                // Motor parameters structure
#include "foc-current.h"                             // FOC current controller with feedforward
#include "hfi_sense.h"                               // HFI sense for inductance measurement

// Phase patterns:
// ABC = 0, CBA = -16384, ACB = 32767

static constexpr uint16_t CAL_OVERSAMPLING_POW2 = 64;  // 64 samples

/** Why an electrical identification aborted (reported through 0x43/0x08). */
enum IdentFailure : uint8_t {
    IDENT_FAIL_NONE = 0,          ///< No abort since boot
    IDENT_FAIL_PATTERN = 1,       ///< Phase currents match no known connection pattern
    IDENT_FAIL_UNDERPOWER = 2,    ///< Injected power too small to measure (open phase, low bus)
    IDENT_FAIL_OVERPOWER = 3,     ///< Slow-rotation power exceeded the motor's limit (stalled / loaded)
    IDENT_FAIL_DRIVER_FAULT = 4,  ///< The driver raised a fault while the routine ran (fault byte recorded)
    IDENT_FAIL_INDUCTANCE = 5,    ///< HFI inductance missing on a stage or inconsistent between the probe frequencies
    IDENT_FAIL_UNKNOWN = 255      ///< Aborted through a path that did not classify itself
};
static constexpr uint8_t CAL_OVERSAMPLING_SHIFT = 6;   // log2(64) for shift optimization

// ~0.1 % of maximum driver output power. Otherwise we can bound ADC resolution easily and cant control driver
static constexpr float CAL_UNDERPOWER_RATIO = 0.001f;

// The old step was 2 Q15 units per tick.
// It is now a dimensionless fraction of Vbus, from which a per-tick step in volts is derived.
static constexpr float CAL_VOLTAGE_RAMP_STEP_RATIO = 0.0005;

static constexpr float CAL_VOLTAGE_LIMIT_RATIO = 0.5f;

// Cos(45 deg) for test-vector amplitude correction.
static constexpr float COS_45 = 0.70710678118f;

// Minimum current for safe division in the R and Z calculations.
static constexpr float MIN_MEASUREMENT_CURRENT_AMPS = 0.001f;

static constexpr int16_t CAL_ANGLE_45D = 8192;    // +45 deg
static constexpr int16_t CAL_ANGLE_90D = 16384;   // +90 deg
static constexpr int16_t CAL_ANGLE_135D = 24576;  // -135 deg
static constexpr int16_t CAL_ANGLE_180D = 32767;  // 180deg
static constexpr int16_t WARMUP_TIMEOUT = 4500;
static constexpr int16_t SETTLE_TIMEOUT = 900;
static constexpr int16_t ROTATION_TIMEOUT = 1800;
static constexpr uint32_t CALIB_PAUSE_CYCLES = 1800;  // 100ms pause @ 18kHz

// Increased inductance measurement time for accuracy (500 periods at 1 kHz)
static constexpr uint32_t INDUCTANCE_MEASURE_CYCLES_F1 = 9000;  // 0.5 sec at 18kHz
static constexpr uint32_t INDUCTANCE_MEASURE_CYCLES_F2 = 9000;  // 0.5 sec at 18kHz

// Upper bound on the two-point resistance-correction factor I1 / (2 * dI).
// With a series offset drop Voff in the bridge the factor evaluates to
// (V1 - Voff) / V1, so it is <= 1 by construction: the correction can only pull
// R DOWN from the raw V/I estimate. A factor above this bound means dI collapsed
// into the noise floor (the second operating point did not settle, or the
// voltage was not actually halved) and the "correction" would inflate R instead.
static constexpr float RES_CORRECTION_MAX_FACTOR = 1.0f;

// Frequencies for dual inductance validation
static constexpr uint16_t HFI_FREQ_1 = 1000;  // 1000 Hz
static constexpr uint16_t HFI_FREQ_2 = 1500;  // 1500 Hz

// Smallest inductance accepted from an HFI stage [uH]. Both HFI warning paths
// store exactly 0 when they cannot extract an estimate, so anything at or
// below this floor means "no measurement", not "a very small inductance".
static constexpr float HFI_MIN_VALID_INDUCTANCE_UH = 1.0f;

// Largest permissible relative spread between the HFI_FREQ_1 and HFI_FREQ_2
// estimates of the same axis. Both probe the same winding, so a healthy motor
// agrees to within a few percent; a wide spread means the series-RL model
// broke down (saturation, eddy-current losses, or an over-estimated R) and the
// averaged result is not trustworthy enough to tune the current loop with.
static constexpr float HFI_MAX_SPREAD_RATIO = 0.30f;

// Motor calibration structure with all calibration-related variables and states
struct MotorCalibration {
private:
    // Connection calibration ramp variables
    float voltage_volts_;               // Current output voltage (Volts)
    float voltage_increment_volts_;     // Voltage increment per tick (Volts)
    float voltage_limit_volts_;         // Maximum voltage limit (Volts)
    float overcurrent_threshold_amps_;  // Overcurrent detection threshold (Amps)

    // Inductance measurement variables
    FrequencyGenerator hfi_gen_;
    float voltage_d_out_volts_;
    float voltage_q_out_volts_;
    float holding_voltage_volts_;
    float hfi_amplitude_volts_;
    float target_current_amps_;
    float current_max_amps_;
    float current_min_amps_;
    float hfi_accum_amps_;
    uint16_t hfi_periods_;

    // Measurement results at the two frequencies
    float Ld_f1_, Ld_f2_;
    float Lq_f1_, Lq_f2_;

    // Slow rotation phase variables
    int16_t slow_rotation_angle_target_;     // Target angle for slow rotation (Q15)
    float slow_rotation_power_limit_watts_;  // Power limit during rotation (Watts)

    // Measurement reference values - accumulation for averaging
    float measure_current_a_sum_amps_;  // Phase A current accumulator for 100 samples
    float measure_current_b_sum_amps_;  // Phase B current accumulator for 100 samples

    // Inductance measurement accumulators
    int32_t inductance_a_sum_;  // Phase A inductance accumulator for HFI measurement
    int32_t inductance_b_sum_;  // Phase B inductance accumulator for HFI measurement

    // --- NEW VARIABLES FOR RESISTANCE CORRECTION ---
    bool is_resistance_correction_;
    float first_ia_amps_;
    float first_ib_amps_;
    float uncorrected_total_resistance_ohms_;
    // ----------------------------------------------------

    // Calibration-specific timers and checkers
    timeout_timer timeout_;
    StabilityCheck stability_check_;
    DriverPWM& drv;

    // Flash-restored profile snapshot (captured before identification overwrites
    // the values) - used for the post-calibration "same motor" decision.
    bool cfg_valid_ = false;    ///< A parameter profile was restored from system_dump.
    MotorType cfg_type_;        ///< Stored motor type.
    PhasePattern cfg_pattern_;  ///< Stored phase pattern.
    float cfg_resistance_ohm_;  ///< Stored winding resistance [Ohm].

    float max_power_watts_;
    float soft_power_limit_watts_;

    // Last-tick diagnostic snapshot for the error dump: refreshed on every
    // tick_calibration() call so set_error() can report the real measured
    // conditions at the moment of the failure (values in SI units).
    float diag_power_watts_;   // Measured electrical power [W]
    float diag_ia_amps_;       // Filtered phase A current [A]
    float diag_ib_amps_;       // Filtered phase B current [A]
    float diag_supply_volts_;  // Supply (Vbus) voltage [V]

    // ---- Last failure, machine readable ------------------------------------
    // The console dump in set_error() is the full story, but the console is
    // often not connected; these fields let the protocol report WHY the last
    // identification aborted (0x43/0x08) without it.
    uint8_t last_fail_code_ = 0;        // IdentFailure of the last abort (0 = none since boot)
    uint8_t last_fail_stage_ = 0;       // MotorStatus that was active when it aborted
    uint8_t last_fail_fault_ = 0;       // Driver fault byte handed to abort() (driver-fault aborts only)
    float last_fail_ia_amps_ = 0.0f;    // Phase A current at the abort [A]
    float last_fail_ib_amps_ = 0.0f;    // Phase B current at the abort [A]
    float last_fail_volts_ = 0.0f;      // Injected voltage at the abort [V]

public:
    // Constructor - initialize all calibration variables
    MotorCalibration(DriverPWM& drv)
        : hfi_gen_(20000),  // Default 20kHz sys freq
          voltage_volts_(0.0f),
          voltage_increment_volts_(0.0f),
          voltage_limit_volts_(0.0f),
          overcurrent_threshold_amps_(0.0f),
          voltage_d_out_volts_(0.0f),
          voltage_q_out_volts_(0.0f),
          holding_voltage_volts_(0.0f),
          hfi_amplitude_volts_(0.0f),
          target_current_amps_(0.0f),
          current_max_amps_(-INFINITY),
          current_min_amps_(INFINITY),
          hfi_accum_amps_(0.0f),
          hfi_periods_(0),
          Ld_f1_(0.0f),
          Ld_f2_(0.0f),
          Lq_f1_(0.0f),
          Lq_f2_(0.0f),
          slow_rotation_angle_target_(0),
          slow_rotation_power_limit_watts_(0.0f),
          measure_current_a_sum_amps_(0.0f),
          measure_current_b_sum_amps_(0.0f),
          inductance_a_sum_(0),
          inductance_b_sum_(0),
          is_resistance_correction_(false),       // Initialization
          first_ia_amps_(0.0f),                   // Initialization
          first_ib_amps_(0.0f),                   // Initialization
          uncorrected_total_resistance_ohms_(0),  // Initialization
          timeout_(CALIB_PAUSE_CYCLES),
          stability_check_(0.05, 240),
          drv(drv),
          max_power_watts_(500.0f),
          soft_power_limit_watts_(0.75f),
          diag_power_watts_(0.0f),
          diag_ia_amps_(0.0f),
          diag_ib_amps_(0.0f),
          diag_supply_volts_(0.0f) {}

    // Getters for calibration variables
    float get_voltage() const { return voltage_volts_; }
    float get_voltage_d() const { return voltage_d_out_volts_; }
    float get_voltage_q() const { return voltage_q_out_volts_; }
    float get_voltage_increment() const { return voltage_increment_volts_; }
    float get_voltage_limit() const { return voltage_limit_volts_; }
    int16_t get_slow_rotation_angle_target() const { return slow_rotation_angle_target_; }
    float get_slow_rotation_power_limit() const { return slow_rotation_power_limit_watts_; }

    float get_measure_current_a() const { return measure_current_a_sum_amps_ / static_cast<float>(CAL_OVERSAMPLING_POW2); }

    float get_measure_current_b() const { return measure_current_b_sum_amps_ / static_cast<float>(CAL_OVERSAMPLING_POW2); }

    // Setters for calibration variables
    void set_voltage(float voltage_volts) {
        voltage_volts_ = voltage_volts;

        if (voltage_volts_ > voltage_limit_volts_) {
            voltage_volts_ = voltage_limit_volts_;
        }

        if (voltage_volts_ < 0.0f) {
            voltage_volts_ = 0.0f;
        }
    }

    void accumulate_measure_currents(float current_a_amps, float current_b_amps) {
        measure_current_a_sum_amps_ += current_a_amps;
        measure_current_b_sum_amps_ += current_b_amps;
    }

    void reset_measure_accumulators() {
        measure_current_a_sum_amps_ = 0.0f;
        measure_current_b_sum_amps_ = 0.0f;
    }

    inline void update_voltage_range(float supply_voltage_volts) {
        voltage_limit_volts_ = supply_voltage_volts * CAL_VOLTAGE_LIMIT_RATIO;

        voltage_increment_volts_ = supply_voltage_volts * CAL_VOLTAGE_RAMP_STEP_RATIO;

        if (voltage_volts_ > voltage_limit_volts_) {
            voltage_volts_ = voltage_limit_volts_;
        }
    }

    inline void increase_voltage() {
        voltage_volts_ += voltage_increment_volts_;

        if (voltage_volts_ > voltage_limit_volts_) {
            voltage_volts_ = voltage_limit_volts_;
        }
    }

    inline void decrease_voltage() {
        voltage_volts_ -= voltage_increment_volts_;

        if (voltage_volts_ < 0.0f) {
            voltage_volts_ = 0.0f;
        }
    }

    void reset_voltage() {
        voltage_volts_ = 0.0f;
        voltage_d_out_volts_ = 0.0f;
        voltage_q_out_volts_ = 0.0f;
    }

    // ========================================================================
    // CALIBRATION LOGIC
    // ========================================================================
    void start_calibration(Motor& motor, MotorStatus& status_tracker, int16_t& angle_ref, float motor_ohmic_power_watts) {
        status_tracker = MotorStatus::SoftWarmup;

        // Snapshot the flash-restored profile (if any) BEFORE identification
        // overwrites these fields - needed later for the "same motor" check.
        cfg_valid_ = motor.is_loaded();
        cfg_type_ = motor.get_motor_type();
        cfg_pattern_ = motor.get_phase_pattern();
        cfg_resistance_ohm_ = motor.get_resistance();

        reset_voltage();
        stability_check_.reset();
        is_resistance_correction_ = false;  // Clear the flag

        soft_power_limit_watts_ = motor_ohmic_power_watts * 0.75f;

        // #REPLACE with just simple storing value + value threshold check

        drv.motor_ctrl.set_motor_type(MotorType_STEP);
        drv.motor_ctrl.set_phase_pattern(PhasePattern_ABC);

        angle_ref = CAL_ANGLE_45D;
        timeout_.set_timeout(WARMUP_TIMEOUT);
    }

    void tick_calibration(float supply_voltage_volts,
                          float power_watts,
                          float current_a_amps,
                          float current_b_amps,
                          float current_d_amps,
                          float current_q_amps,
                          MotorStatus& status_tracker,
                          Motor& motor,
                          DriverStatus& setup_status,
                          int16_t& angle_ref) {
        // Refresh the diagnostic snapshot used by the set_error() dump so any
        // later abort reports the actual measured electrical conditions.
        diag_supply_volts_ = supply_voltage_volts;
        diag_power_watts_ = power_watts;
        diag_ia_amps_ = current_a_amps;
        diag_ib_amps_ = current_b_amps;

        update_voltage_range(supply_voltage_volts);

        switch (status_tracker) {
            case MotorStatus::SoftWarmup: {
                voltage_d_out_volts_ = voltage_volts_;
                voltage_q_out_volts_ = 0.0f;

                // [OPT] measured_current is only needed here - compute it only here
                const float measured_current_amps = fabsf(current_a_amps + current_b_amps);

                // #REPLACE with just simple storing value + value threshold check
                const bool power_lim = power_watts >= soft_power_limit_watts_;

                const bool voltage_lim = voltage_volts_ >= voltage_limit_volts_;

                if ((stability_check_.tick(measured_current_amps) && power_watts > (max_power_watts_ * CAL_UNDERPOWER_RATIO)) ||
                    timeout_.tick() == 0) {
                    // Warn when warmup ends without reaching a stable current:
                    // a tiny measured current hints at a disconnected phase,
                    // too low Vbus, or a very high-resistance winding. This is
                    // not fatal by itself but degrades all later measurements.
                    if (power_watts <= (max_power_watts_ * CAL_UNDERPOWER_RATIO)) {
                        TP_LOG_VALS("ELEC CAL WARNING: warmup ended underpowered: P[W],threshold[W],I_meas[A],V_inj[V]",
                                    power_watts, (max_power_watts_ * CAL_UNDERPOWER_RATIO), measured_current_amps,
                                    voltage_volts_);
                        TP_LOG("ELEC CAL WARNING: hint: phase disconnected? Vbus too low? calibration continues with reduced accuracy");
                    }

                    target_current_amps_ = measured_current_amps;
                    TP_LOG_VALS("ELEC CAL: warmup end: V[V],I[A],P[W],Vbus[V],ticks_left", voltage_volts_, measured_current_amps,
                                power_watts, supply_voltage_volts, timeout_.get_timeout());

                    timeout_.set_timeout(SETTLE_TIMEOUT);
                    status_tracker = MotorStatus::Settle;
                } else {
                    if (power_lim) {
                        decrease_voltage();
                    } else {
                        increase_voltage();
                    }
                }
                break;
            }

            case MotorStatus::Settle: {
                voltage_d_out_volts_ = voltage_volts_;
                voltage_q_out_volts_ = 0.0f;

                if (timeout_.tick() == 0) {
                    status_tracker = MotorStatus::ResistanceMeasure;
                    timeout_.set_timeout(CAL_OVERSAMPLING_POW2);
                }
                break;
            }

            case MotorStatus::ResistanceMeasure: {
                voltage_d_out_volts_ = voltage_volts_;
                voltage_q_out_volts_ = 0.0f;

                // CHANGED: added the power argument for correction computation
                // #REPLACE norm value should be removed and replaced with float power
                process_resistance_measure(current_a_amps, current_b_amps, power_watts, supply_voltage_volts, status_tracker,
                                           motor, setup_status, angle_ref);
                break;
            }

            case MotorStatus::SlowRotation: {
                // ... (original code unchanged)
                voltage_d_out_volts_ = voltage_volts_;
                voltage_q_out_volts_ = 0.0f;

                if (power_watts >= max_power_watts_) {
                    // Overpower during the slow rotation sweep: the dragged
                    // vector consumes more power than the motor profile allows.
                    // Typical causes: rotor stalled/jammed, load or rotor
                    // inertia too high for the sweep speed, or P_max set too
                    // low for this motor.
                    set_error(status_tracker, motor, setup_status,
                              "OVERPOWER: rotation power reached the max limit "
                              "(rotor stalled/jammed? load or inertia too high? P_max too low?)",
                              IDENT_FAIL_OVERPOWER);
                    break;
                }

                if (timeout_.tick() == 0) {
                    angle_ref = slow_rotation_angle_target_;

                    timeout_.set_timeout(SETTLE_TIMEOUT);
                    status_tracker = MotorStatus::Settle;
                } else {
                    int16_t delta = slow_rotation_angle_target_ - angle_ref;

                    angle_ref += delta / static_cast<int32_t>(timeout_.get_timeout() + 1);
                }
                break;
            }

            // ... remaining HFI states unchanged ...
            case MotorStatus::InductanceMeasureWarmup: {
                voltage_d_out_volts_ = holding_voltage_volts_;
                voltage_q_out_volts_ = 0.0f;

                if (timeout_.tick() == 0) {
                    hfi_gen_.set_frequency_target(HFI_FREQ_1);

                    setup_hfi_state(MotorStatus::InductanceMeasureD, status_tracker, motor, INDUCTANCE_MEASURE_CYCLES_F1);
                }
                break;
            }

            case MotorStatus::InductanceMeasureD: {
                process_hfi_axis(current_d_amps, true, false, supply_voltage_volts, status_tracker, motor, setup_status,
                                 MotorStatus::InductanceMeasureD2, INDUCTANCE_MEASURE_CYCLES_F2, &Ld_f1_, HFI_FREQ_1);
                break;
            }

            case MotorStatus::InductanceMeasureD2: {
                hfi_gen_.set_frequency_target(HFI_FREQ_2);

                process_hfi_axis(current_d_amps, true, false, supply_voltage_volts, status_tracker, motor, setup_status,
                                 MotorStatus::InductanceMeasureQ, INDUCTANCE_MEASURE_CYCLES_F1, &Ld_f2_, HFI_FREQ_2);
                break;
            }

            case MotorStatus::InductanceMeasureQ: {
                hfi_gen_.set_frequency_target(HFI_FREQ_1);

                // Changed false, true -> false, false
                process_hfi_axis(current_q_amps, false, false, supply_voltage_volts, status_tracker, motor, setup_status,
                                 MotorStatus::InductanceMeasureQ2, INDUCTANCE_MEASURE_CYCLES_F2, &Lq_f1_, HFI_FREQ_1);
                break;
            }

            case MotorStatus::InductanceMeasureQ2: {
                hfi_gen_.set_frequency_target(HFI_FREQ_2);

                process_hfi_axis(current_q_amps, false, true, supply_voltage_volts, status_tracker, motor, setup_status,
                                 MotorStatus::Success, 0, &Lq_f2_, HFI_FREQ_2);
                break;
            }

            default:
                break;
        }
    }

    /**
     * @brief Abort a running identification into the CalibError terminal state.
     *
     * For an identification started at runtime: a driver fault mid-run holds
     * the bridge released while the routine keeps its ramp voltage, so the
     * moment the fault clears the routine resumes at full voltage and trips
     * it again, and the drive is never released from the loop. Aborting
     * instead reports the failure through CALIBRATION_STATUS and leaves the
     * stage released until a reboot, which is the documented recovery.
     */
    void abort(MotorStatus& status_tracker, Motor& motor, DriverStatus& setup_status, const char* reason, uint8_t fault_bits = 0) {
        last_fail_fault_ = fault_bits;                     // What the driver reported at that moment
        set_error(status_tracker, motor, setup_status, reason, IDENT_FAIL_DRIVER_FAULT);
    }

    /// Why the last identification aborted (IdentFailure), 0 when none has since boot.
    uint8_t last_failure_code() const { return last_fail_code_; }
    /// MotorStatus stage that was active at the last abort.
    uint8_t last_failure_stage() const { return last_fail_stage_; }
    /// Driver fault byte handed to abort() at the last driver-fault abort.
    uint8_t last_failure_fault() const { return last_fail_fault_; }
    /// Phase A current at the last abort [A].
    float last_failure_ia() const { return last_fail_ia_amps_; }
    /// Phase B current at the last abort [A].
    float last_failure_ib() const { return last_fail_ib_amps_; }
    /// Injected voltage at the last abort [V].
    float last_failure_volts() const { return last_fail_volts_; }

private:
    void setup_hfi_state(MotorStatus target_state, MotorStatus& status_tracker, Motor& motor, uint32_t timeout_cycles) {
        status_tracker = target_state;
        timeout_.set_timeout(timeout_cycles);
        current_max_amps_ = -INFINITY;
        current_min_amps_ = INFINITY;
        hfi_accum_amps_ = 0.0f;
        hfi_periods_ = 0;
        hfi_gen_.reset();
    }

    /**
     * @brief Relative spread between the two probe frequencies of one axis.
     *
     * Both estimates describe the same winding, so a valid measurement puts
     * them within a few percent of each other. A stage that bailed out through
     * either HFI warning path stores exactly 0, which is the ABSENCE of a
     * measurement rather than perfect agreement - such an axis is reported as
     * 1.0 (100% disagreement) so the caller rejects it instead of averaging a
     * zero into the committed result.
     *
     * @param l_f1 Inductance measured at HFI_FREQ_1 [uH].
     * @param l_f2 Inductance measured at HFI_FREQ_2 [uH].
     * @return |l_f1 - l_f2| / l_f1, or 1.0 when either estimate is missing.
     */
    static float hfi_axis_spread(float l_f1, float l_f2) {
        // Either stage missing => no usable pair => full disagreement.
        if (l_f1 <= HFI_MIN_VALID_INDUCTANCE_UH || l_f2 <= HFI_MIN_VALID_INDUCTANCE_UH) return 1.0f;
        return fabsf(l_f1 - l_f2) / l_f1;  // Normalised against the 1 kHz reference
    }

    void process_hfi_axis(float measured_current_amps,
                          bool is_d_axis,
                          bool is_final_step,
                          float supply_voltage_volts,
                          MotorStatus& status_tracker,
                          Motor& motor,
                          DriverStatus& setup_status,
                          MotorStatus next_state,
                          uint32_t next_timeout,
                          float* result_L,
                          uint16_t current_freq) {
        int16_t prev_angle = hfi_gen_.get_angle();
        Vec2D hfi_out = hfi_gen_.tick();
        int16_t curr_angle = hfi_gen_.get_angle();

        const float hfi_wave = static_cast<float>(hfi_out.cos) / static_cast<float>(INT16_MAX);

        const float hfi_voltage_volts = hfi_amplitude_volts_ * hfi_wave;

        if (is_d_axis) {
            voltage_d_out_volts_ = holding_voltage_volts_ + hfi_voltage_volts;

            voltage_q_out_volts_ = 0.0f;
        } else {
            voltage_d_out_volts_ = holding_voltage_volts_;
            voltage_q_out_volts_ = hfi_voltage_volts;
        }

        if (voltage_d_out_volts_ > voltage_limit_volts_) {
            voltage_d_out_volts_ = voltage_limit_volts_;
        }

        if (voltage_d_out_volts_ < -voltage_limit_volts_) {
            voltage_d_out_volts_ = -voltage_limit_volts_;
        }

        if (!is_d_axis && (holding_voltage_volts_ + hfi_amplitude_volts_) > voltage_limit_volts_) {
            hfi_amplitude_volts_ = voltage_limit_volts_ - holding_voltage_volts_;
        }

        if (hfi_amplitude_volts_ < 0.0f) {
            hfi_amplitude_volts_ = 0.0f;
        }

        if (measured_current_amps > current_max_amps_) {
            current_max_amps_ = measured_current_amps;
        }

        if (measured_current_amps < current_min_amps_) {
            current_min_amps_ = measured_current_amps;
        }

        if (prev_angle > 0 && curr_angle <= 0) {
            float ripple_amps = current_max_amps_ - current_min_amps_;

            if (hfi_periods_ > 0) {
                hfi_accum_amps_ += ripple_amps;  // Skip first partial period
            }

            hfi_periods_++;

            // PID-like adaptation for amplitude
            float target_ripple_min_amps = target_current_amps_ * 0.20f;

            // [OPT] 25% of target_current_ = divide by 4 = shift right by 2 bits
            float target_ripple_max_amps = target_current_amps_ * 0.25f;

            if (ripple_amps < target_ripple_min_amps) {
                hfi_amplitude_volts_ += voltage_increment_volts_;  // Slowed down adaptation for stability during long tests
            } else if (ripple_amps > target_ripple_max_amps) {
                hfi_amplitude_volts_ -= voltage_increment_volts_;
            }

            if ((holding_voltage_volts_ + hfi_amplitude_volts_) > voltage_limit_volts_) {
                hfi_amplitude_volts_ = voltage_limit_volts_ - holding_voltage_volts_;
            }

            if (hfi_amplitude_volts_ < 0.0f) {
                hfi_amplitude_volts_ = 0.0f;
            }

            current_max_amps_ = -INFINITY;
            current_min_amps_ = INFINITY;
        }

        if (timeout_.tick() == 0) {
            float avg_ripple_amps = (hfi_periods_ > 1) ? (hfi_accum_amps_ / static_cast<float>(hfi_periods_ - 1)) : 0.0f;

            // Calculate accurate Inductance taking Ohmic Resistance into account
            // [OPT] Cache norm2mv(hfi_amplitude_) - used twice: for v_peak and for logging
            float v_peak_volts = hfi_amplitude_volts_;

            float i_peak_amps = avg_ripple_amps * 0.5f;
            float omega_rad_per_sec = 2.0f * 3.14159265f * static_cast<float>(current_freq);

            // Motor API still returns milliohms; inside the class we work in ohms only.
            float resistance_ohms = motor.get_total_resistance();  // in Ohms

            // [OPT] Z is precomputed: removes repeated v_peak/i_peak division in logging
            //       and guards against ~0 division (i_peak <= 0.001f was outside the if below - a bug)
            float impedance_ohms = (i_peak_amps > MIN_MEASUREMENT_CURRENT_AMPS) ? (v_peak_volts / i_peak_amps) : 0.0f;

            if (i_peak_amps > MIN_MEASUREMENT_CURRENT_AMPS) {
                float X_L_sq = (impedance_ohms * impedance_ohms) - (resistance_ohms * resistance_ohms);

                if (X_L_sq > 0.0f)
                    *result_L = (sqrtf(X_L_sq) / omega_rad_per_sec) * 1e6f;  // uH
                else {
                    // R dominated Z entirely (shouldn't happen at 1kHz+): the
                    // measured impedance carries no usable inductive part.
                    TP_LOG_VALS("ELEC CAL WARNING: HFI Z has no inductive part, L=0: i_peak[A],v_peak[V],Z[Ohm],R[Ohm]",
                                i_peak_amps, v_peak_volts, impedance_ohms, resistance_ohms);
                    *result_L = 0.0f;
                }

            } else {
                // Current ripple too small for a valid inductance estimate:
                // the HFI amplitude adaptation failed to excite a measurable
                // current (winding impedance too high / phase disconnected).
                TP_LOG_VALS("ELEC CAL WARNING: HFI ripple too small, L=0: i_peak[A],v_peak[V],periods",
                            i_peak_amps, v_peak_volts, (long)hfi_periods_);
                *result_L = 0.0f;
            }

            // [OPT] Use the cached amp_mv instead of calling norm2mv again

            // [OPT] Use the cached Z instead of dividing v_peak / i_peak again

            if (is_final_step) {
                // Final validation: every stage must have produced a usable
                // estimate AND the two probe frequencies must agree per axis.
                // A missing stage reads as 1.0 here (see hfi_axis_spread).
                const float diff_d = hfi_axis_spread(Ld_f1_, Ld_f2_);
                const float diff_q = hfi_axis_spread(Lq_f1_, Lq_f2_);

                // Log the dual-frequency measurement results: a large spread
                // between the 1 kHz and 1.5 kHz estimates means the HFI model
                // assumptions broke down.
                TP_LOG_VALS("ELEC CAL: HFI results uH: Ld_f1,Ld_f2,Lq_f1,Lq_f2", Ld_f1_, Ld_f2_, Lq_f1_, Lq_f2_);
                TP_LOG_VALS("ELEC CAL: HFI spread: diff_d,diff_q", diff_d, diff_q);

                // Abort instead of committing a doubtful identification. The
                // current-loop gains are derived from L (kp = L * omega_c in
                // FocCurrentController::tick), while ki depends only on R, so
                // an under-estimated inductance destroys the pole-zero
                // cancellation the PI relies on and leaves an
                // integral-dominated loop that limit-cycles with the motor
                // energized. The anti-windup gain (k_aw = R / L) is inflated
                // by the same error. Refusing to arm is the safe outcome:
                // tick() drives zero current target in CalibError, and
                // tunepulse_init() exits its wait loop on that status.
                if (diff_d > HFI_MAX_SPREAD_RATIO || diff_q > HFI_MAX_SPREAD_RATIO) {
                    set_error(status_tracker, motor, setup_status, "HFI inductance missing or inconsistent",
                              IDENT_FAIL_INDUCTANCE);
                    return;
                }

                // Average the 1kHz and 1.5kHz results
                float final_Ld = (Ld_f1_ + Ld_f2_) * 0.5f;
                float final_Lq = (Lq_f1_ + Lq_f2_) * 0.5f;

                // Store the identified D/Q inductances into the motor profile
                // (measurements are computed in microhenries, stored in H).
                motor.set_inductance_d(final_Ld * 1e-6f);
                motor.set_inductance_q(final_Lq * 1e-6f);
                finish_calibration_success(status_tracker, motor, setup_status);
            } else {
                setup_hfi_state(next_state, status_tracker, motor, next_timeout);
            }
        }
    }

    void start_slow_rotation(int16_t target_angle, float power_limit_watts) {
        slow_rotation_angle_target_ = target_angle;
        slow_rotation_power_limit_watts_ = power_limit_watts;
    }

    /**
     * @brief Map a MotorStatus state-machine stage to a human-readable name.
     * @param s Stage to convert.
     * @return Static string with the stage name (never NULL).
     */
    static const char* stage_name(MotorStatus s) {
        switch (s) {
            case MotorStatus::SoftWarmup:               return "SoftWarmup";
            case MotorStatus::Settle:                   return "Settle";
            case MotorStatus::SlowRotation:             return "SlowRotation";
            case MotorStatus::ResistanceMeasure:        return "ResistanceMeasure";
            case MotorStatus::InductanceMeasure:        return "InductanceMeasure";
            case MotorStatus::InductanceMeasureWarmup:  return "HFI_Warmup";
            case MotorStatus::InductanceMeasureD:       return "HFI_D_f1";
            case MotorStatus::InductanceMeasureD2:      return "HFI_D_f2";
            case MotorStatus::InductanceMeasureQ:       return "HFI_Q_f1";
            case MotorStatus::InductanceMeasureQ2:      return "HFI_Q_f2";
            case MotorStatus::ElectricCalibrationDone:  return "ElectricCalibrationDone";
            case MotorStatus::CalibError:               return "CalibError";
            case MotorStatus::Success:                  return "Success";
            case MotorStatus::NotCalibrated:            return "NotCalibrated";
            default:                                    return "Unknown";
        }
    }

    /**
     * @brief Map a MotorType to a human-readable name.
     * @param t Motor type to convert.
     * @return Static string with the type name (never NULL).
     */
    static const char* type_name(MotorType t) {
        switch (t) {
            case MotorType_STEP:    return "STEP";
            case MotorType_BLDC:    return "BLDC";
            default:                return "Invalid";
        }
    }

    /**
     * @brief Map a PhasePattern to a human-readable name.
     * @param p Phase pattern to convert.
     * @return Static string with the pattern name (never NULL).
     */
    static const char* pattern_name(PhasePattern p) {
        switch (p) {
            case PhasePattern_ABC:  return "ABC";
            case PhasePattern_CBA:  return "CBA";
            case PhasePattern_ACB:  return "ACB";
            default:                return "Invalid";
        }
    }

    /**
     * @brief Abort the calibration with a detailed one-shot diagnostic dump.
     *
     * Called from every fatal error path (overpower, underpower, unrecognized
     * phase connection). Prints the abort reason plus the diagnostic snapshot
     * (failing stage, detected type/pattern, supply/applied/limit voltages,
     * measured power against the limits, phase currents) so the exact failure
     * cause can be identified from the USB log alone.
     *
     * All output uses the ISR-safe TP_LOG macros: lines are DROPPED (never
     * blocking) when the CDC TX queue is nearly full.
     *
     * @param status_tracker Calibration state machine reference (holds the
     *                       failing stage on entry, set to CalibError on exit).
     * @param motor          Motor profile (provides detected type/pattern).
     * @param setup_status   Driver status word (set to OperationStatus::Error).
     * @param reason         Short human-readable error description (RAM string).
     */
    void set_error(MotorStatus& status_tracker, Motor& motor, DriverStatus& setup_status, const char* reason,
                   uint8_t code = IDENT_FAIL_UNKNOWN) {
        const MotorStatus failed_stage = status_tracker;  // Remember the stage that aborted
        const float applied_volts = voltage_volts_;       // Remember the injected voltage before reset

        // Machine-readable copy of what the dump below prints, for 0x43/0x08.
        last_fail_code_ = code;
        last_fail_stage_ = (uint8_t)failed_stage;
        last_fail_ia_amps_ = diag_ia_amps_;
        last_fail_ib_amps_ = diag_ib_amps_;
        last_fail_volts_ = applied_volts;

        setup_status.set_driver_status(OperationStatus::Error);
        status_tracker = MotorStatus::CalibError;
        motor.set_status(MotorStatus::CalibError);
        reset_voltage();

        // One-shot detailed failure dump (lines are dropped, never blocking,
        // when the USB CDC TX queue runs low - see dev_log.h).
        TP_LOG("==================================================");
        TP_LOG_VAL("ELEC CALIBRATION FAILED: ", reason);
        TP_LOG_VALS("  failed_stage,phase_pattern,motor_type", stage_name(failed_stage),
                    pattern_name(motor.get_phase_pattern()), type_name(motor.get_motor_type()));
        TP_LOG_VALS("  Vbus[V],V_injected[V],V_limit[V]", diag_supply_volts_, applied_volts, voltage_limit_volts_);
        TP_LOG_VALS("  P_meas[W],P_max[W],P_soft_lim[W]", diag_power_watts_, max_power_watts_, soft_power_limit_watts_);
        TP_LOG_VALS("  Ia[A],Ib[A]", diag_ia_amps_, diag_ib_amps_);
        TP_LOG("==================================================");
    }

    void start_inductance_measurement(float supply_voltage_volts,
                                      MotorStatus& status_tracker,
                                      Motor& motor,
                                      DriverStatus& setup_status,
                                      int16_t& angle_ref) {
        motor.set_motor_type(drv.motor_ctrl.get_motor_type());
        motor.set_phase_pattern(drv.motor_ctrl.get_phase_pattern());
        setup_status.motor_type_not_verified = 0;
        setup_status.voltage_unsafe = supply_voltage_volts <= 0.0f;

        holding_voltage_volts_ = voltage_volts_ * 0.5f;
        hfi_amplitude_volts_ = voltage_increment_volts_ * 50.0f;

        if ((holding_voltage_volts_ + hfi_amplitude_volts_) > voltage_limit_volts_) {
            hfi_amplitude_volts_ = voltage_limit_volts_ - holding_voltage_volts_;
        }

        if (hfi_amplitude_volts_ < 0.0f) {
            hfi_amplitude_volts_ = 0.0f;
        }

        angle_ref = 0;  // Align to D-axis

        status_tracker = MotorStatus::InductanceMeasureWarmup;
        timeout_.set_timeout(SETTLE_TIMEOUT);
    }

    void finish_calibration_success(MotorStatus& status_tracker, Motor& motor, DriverStatus& setup_status) {
        status_tracker = MotorStatus::Success;
        motor.set_status(MotorStatus::Success);

        // Same-motor identification: compare the freshly identified set with
        // the profile restored from system_dump. Matching connection/type and
        // resistance within 15% => the same motor: keep the stored
        // configuration (Motor Identified). Otherwise overwrite it by
        // appending a new record - the loader always picks the newest one.
        bool same = cfg_valid_ && (motor.get_motor_type() == cfg_type_) && (motor.get_phase_pattern() == cfg_pattern_);
        if (same) {
            float r0 = cfg_resistance_ohm_;
            float r1 = motor.get_resistance();
            same = (fabsf(r0) > 1e-6f) && (fabsf(r1 - r0) <= 0.15f * fabsf(r0));
        }

        if (!same)
            // NOTE: Motor::begin_save() self-gates on the full-calibration
            // state: the record embeds mech fields (direction, cal_id, angle
            // offset) that are dummy until the angle/mech calibration table
            // exists. On a first boot the save is deferred here and re-armed
            // by the angle-calibrator commit path once the fresh table has
            // been written and read back verified.
            motor.begin_save();  // Non-blocking, serviced 8 words per tick

        reset_voltage();
    }

    void trigger_resistance_correction(MotorStatus& status_tracker, Motor& motor, float ia_amps, float ib_amps) {
        // Store first-pass currents (mA) for the dominant phase
        first_ia_amps_ = ia_amps;
        first_ib_amps_ = ib_amps;

        // Motor API still returns milliohms; inside the class we convert to ohms right away.
        uncorrected_total_resistance_ohms_ = motor.get_total_resistance();

        // The two-pass correction below divides by (I1 - I2): these numbers are
        // what to look at when it produces an absurd resistance.
        TP_LOG_VALS("ELEC CAL: R pass 1: V[V],Ia[A],Ib[A],R_unc[Ohm],Vbus[V],P[W]", voltage_volts_, ia_amps, ib_amps,
                    uncorrected_total_resistance_ohms_, diag_supply_volts_, diag_power_watts_);

        is_resistance_correction_ = true;

        voltage_volts_ *= 0.5f;

        reset_measure_accumulators();
        status_tracker = MotorStatus::Settle;
        timeout_.set_timeout(SETTLE_TIMEOUT);
    }
    // -----------------------------------------------------------------

    void process_resistance_measure(float current_a_amps,
                                    float current_b_amps,
                                    float power_watts,
                                    float supply_voltage_volts,
                                    MotorStatus& status_tracker,
                                    Motor& motor,
                                    DriverStatus& setup_status,
                                    int16_t& angle_ref) {
        // First cycle: check if the counter is initialized (initially equals CAL_OVERSAMPLING_POW2, i.e. 64)
        if (timeout_.get_timeout() == CAL_OVERSAMPLING_POW2)
            // Reset all current accumulators to start clean averaging
            reset_measure_accumulators();

        // Accumulate current samples for phases A and B into accumulators for subsequent averaging
        accumulate_measure_currents(current_a_amps, current_b_amps);

        // Decrement the timeout counter (samples) by 1. When it reaches 0, all 64 measurements are collected
        if (timeout_.tick() == 0) {
            // Cos(45 deg) in Q16 = round(cos(pi/4) * 65536)
            float voltage_corrected_volts = voltage_volts_ * COS_45;  // Voltage amplitude fix

            // Get the averaged phase A and B currents over the entire measurement period
            float ia_amps = get_measure_current_a();
            float ib_amps = get_measure_current_b();

            float applied_voltage_volts = voltage_corrected_volts;

            // Compute corrected voltage in microvolts (normalized → mV, then multiply by 1000)
            if (is_resistance_correction_) {
                const float abs_first_ia = fabsf(first_ia_amps_);
                const float abs_first_ib = fabsf(first_ib_amps_);

                // Select the dominant phase (best SNR)
                bool use_phase_a = abs_first_ia >= abs_first_ib;

                float abs_first_i_amps = use_phase_a ? abs_first_ia : abs_first_ib;

                float second_i_amps = use_phase_a ? ia_amps : ib_amps;

                float abs_second_i_amps = fabsf(second_i_amps);

                // delta_i must be > 0 since I1 > I2 when V1 > V2
                float delta_i_amps = abs_first_i_amps - abs_second_i_amps;

                float corrected_total_resistance_ohms = uncorrected_total_resistance_ohms_;

                // Formula: R_corrected = R_uncorrected * I1 / (2 * delta_i)
                // Voltage cancels out completely for exact V2 = V1/2
                if (delta_i_amps > 0.0f && abs_first_i_amps > MIN_MEASUREMENT_CURRENT_AMPS) {
                    const float correction_factor = abs_first_i_amps / (2.0f * delta_i_amps);

                    if (correction_factor <= RES_CORRECTION_MAX_FACTOR) {
                        corrected_total_resistance_ohms *= correction_factor;
                    } else {
                        // dI is noise, not signal: halving the drive barely moved
                        // the current. Keep the raw V/I estimate, which always
                        // over-states R slightly but stays physically sane. An
                        // inflated R makes every later HFI stage compute
                        // Z^2 - R^2 < 0 and report "no inductive part", so
                        // committing it would fail the whole identification.
                        TP_LOG_VALS("ELEC CAL WARNING: R correction rejected: factor,I1[A],I2[A],dI[A],R_unc[Ohm]",
                                    correction_factor, abs_first_i_amps, abs_second_i_amps, delta_i_amps,
                                    corrected_total_resistance_ohms);
                    }
                }
                TP_LOG_VALS("ELEC CAL: R pass 2: V[V],Ia[A],Ib[A],dI[A],R_corr[Ohm],Vbus[V],P[W]", voltage_volts_, ia_amps,
                            ib_amps, delta_i_amps, corrected_total_resistance_ohms, supply_voltage_volts, power_watts);

                // Diagnostic snapshot of the two-point fit that sets R for every
                // subsequent stage (the HFI inductance maths divides by it).
                TP_LOG_VALS("ELEC CAL: R two-point fit: I1[A],I2[A],dI[A],R_unc[Ohm],R_final[Ohm]", abs_first_i_amps,
                            abs_second_i_amps, delta_i_amps, uncorrected_total_resistance_ohms_,
                            corrected_total_resistance_ohms);

                // Legacy Motor API boundary: pass milliohms.
                motor.set_total_resistance(corrected_total_resistance_ohms);

                is_resistance_correction_ = false;

                start_inductance_measurement(supply_voltage_volts, status_tracker, motor, setup_status, angle_ref);
                return;
            }

            // Abort reason for the final set_error() below: defaults to the
            // "phase pattern not recognized" case, switched to underpower in
            // the else-branch when the injected power was too small to judge.
            const char* fail_reason =
                "PHASE PATTERN NOT RECOGNIZED: measured currents match no known connection type (check phase wiring / motor type)";
            uint8_t fail_code = IDENT_FAIL_PATTERN;      // Machine-readable twin of fail_reason

            if (power_watts >
                (max_power_watts_ * CAL_UNDERPOWER_RATIO)) {  // We have sufficient power, current calculations are reliable
                // Convert averaged normalized currents to milliamps
                float resistance_a_ohms =
                    (fabsf(ia_amps) > MIN_MEASUREMENT_CURRENT_AMPS) ? (applied_voltage_volts / ia_amps) : INFINITY;

                float resistance_b_ohms =
                    (fabsf(ib_amps) > MIN_MEASUREMENT_CURRENT_AMPS) ? (applied_voltage_volts / ib_amps) : INFINITY;

                // If angle equals 16384 (~+90 electrical degrees) — always the first pass for any motor
                if (angle_ref == CAL_ANGLE_45D) {
                    // Set the timeout counter to 1800 cycles (time allocated for slow rotation)
                    timeout_.set_timeout(ROTATION_TIMEOUT);

                    // What the first pass saw and what it decided, so the type /
                    // pattern detection can be checked from the boot log alone.
                    const bool balanced = (10.0f * fabsf(ib_amps - ia_amps) < (ia_amps + ib_amps));
                    TP_LOG_VALS("ELEC CAL: pass 45deg: V[V],Ia[A],Ib[A],Ra[Ohm],Rb[Ohm],P[W],balanced", applied_voltage_volts,
                                ia_amps, ib_amps, resistance_a_ohms, resistance_b_ohms, power_watts, (int)balanced);

                    // Stepper or BLDC motor, first pass: phases should be balanced (within ~15% of average)
                    if (balanced) {
                        // Store temporary resistance as arithmetic mean of phases A and B
                        float average_resistance_ohms = (resistance_a_ohms + resistance_b_ohms) * 0.5f;

                        // Legacy Motor API boundary: pass milliohms.
                        motor.set_total_resistance(average_resistance_ohms);

                        // Slowly rotate the vector by 30° for verification. Target angle 16384 (current + 45° el. (BLDC: +30°))
                        start_slow_rotation(CAL_ANGLE_90D, power_watts);

                        status_tracker = MotorStatus::SlowRotation;
                        return;
                    } else {  // Possibly stepper but in wrong phase pattern, fixing phase commutation
                        set_voltage(voltage_corrected_volts);  // Fix voltage level to seamless transfer - mult power by 2

                        // Slowly rotate the vector backwards to angle -24576, rotation by 45°
                        start_slow_rotation(-CAL_ANGLE_135D, power_watts);

                        status_tracker = MotorStatus::SlowRotation;

                        if ((ia_amps * 0.25f) >= ib_amps) {
                            TP_LOG("ELEC CAL: pass 45deg unbalanced, Ia dominant -> pattern CBA");
                            drv.motor_ctrl.set_phase_pattern(PhasePattern_CBA);

                            // Set target angle to -16384 (corresponds to CBA connection)
                            angle_ref = -CAL_ANGLE_90D;
                            return;
                        } else if ((ib_amps * 0.25f) >= ia_amps) {
                            TP_LOG("ELEC CAL: pass 45deg unbalanced, Ib dominant -> pattern ACB");
                            drv.motor_ctrl.set_phase_pattern(PhasePattern_ACB);

                            // Set target angle to -32768 (corresponds to ACB connection)
                            angle_ref = CAL_ANGLE_180D;
                            return;
                        }
                    }
                }  // Second pass for BLDC or Stepper in ABC connection
                else if (angle_ref == CAL_ANGLE_90D) {
                    // BLDC motor, second pass: check if phases are balanced (within ~15% of average)
                    // Expected: current iA — negative and dominant; current iB — positive and equal to half of iA (iA = iB + iC,
                    // where iB ~= iC)
                    if (14.0f * fabsf((2.0f * ia_amps) + ib_amps) < ((-ia_amps * 2.0f) + ib_amps)) {
                        // Calculate average resistance accounting for all three phases
                        float previous_resistance_ohms = motor.get_total_resistance();

                        float average_resistance_ohms = (previous_resistance_ohms + resistance_a_ohms) / 3.0f;

                        // Write averaged resistance into the motor profile
                        motor.set_total_resistance(average_resistance_ohms);

                        // Set motor type in the selector as BLDC
                        drv.motor_ctrl.set_motor_type(MotorType_BLDC);
                        TP_LOG_VALS("ELEC CAL: pass 90deg: Ia[A],Ib[A] -> BLDC", ia_amps, ib_amps);

                        trigger_resistance_correction(status_tracker, motor, ia_amps, ib_amps);
                        return;
                    }

                    // Stepper motor, second pass: check if iA is significantly 16x greater than iB
                    // (Expected: iA — negative dominant, iB close to 0 (any sign). Resistance was already calculated in first
                    // pass)
                    else if ((ib_amps * 0.0625f) > ia_amps) {
                        TP_LOG_VALS("ELEC CAL: pass 90deg: Ia[A],Ib[A] -> STEPPER, pattern kept", ia_amps, ib_amps);
                        trigger_resistance_correction(status_tracker, motor, ia_amps, ib_amps);
                        return;
                    }
                }

                // Check if angle is around -24576 (-135 degrees). This is the second pass for Stepper with CBA or ACB connection
                else if (angle_ref == -CAL_ANGLE_135D) {
                    // Check if phases B and C are balanced (difference within ~15% of average)
                    if (10.0f * fabsf(ib_amps - ia_amps) < (-ia_amps - ib_amps)) {
                        // Stepper motor: take average resistance of two phases with negative sign (due to vector angle)
                        float average_resistance_ohms = -(resistance_a_ohms + resistance_b_ohms) * 0.5f;

                        // Write averaged resistance into the motor profile
                        motor.set_total_resistance(average_resistance_ohms);

                        trigger_resistance_correction(status_tracker, motor, ia_amps, ib_amps);
                        return;
                    }
                }

                // None of the known winding/connection signatures matched the
                // measured phase currents: dump the full measured data set
                // together with the criteria that are expected for each pass,
                // so a wiring fault (swapped/disconnected phase) or an
                // unsupported motor type can be diagnosed from the log alone.
                TP_LOG_VALS("ELEC CAL: pattern check failed: angle_ref,Ia[A],Ib[A]",
                            (long)angle_ref, ia_amps, ib_amps);
                TP_LOG_VALS("ELEC CAL: R_est_a[Ohm],R_est_b[Ohm],V_inj[V],P[W]",
                            resistance_a_ohms, resistance_b_ohms, applied_voltage_volts, power_watts);
                TP_LOG("ELEC CAL: expected @+45deg: |Ia-Ib| balanced 10x; "
                       "@+90deg: BLDC iA<0 dominant & iB~-iA/2, or STEP |iB|<0.0625*|iA|; "
                       "@-135deg: |Ia-Ib| balanced 10x");
            } else {
                // UNDERPOWER: the injected voltage produced a current so small
                // that resistance/pattern measurements are unreliable. Typical
                // causes: phase wire disconnected, Vbus too low, or a winding
                // resistance far outside the expected range.
                TP_LOG_VALS("ELEC CAL: underpower at pattern check: P[W],threshold[W],Ia[A],Ib[A],V_inj[V]",
                            power_watts, (max_power_watts_ * CAL_UNDERPOWER_RATIO), ia_amps, ib_amps,
                            applied_voltage_volts);
                TP_LOG("ELEC CAL: hint: phase wire disconnected? Vbus too low? winding resistance out of range?");
                fail_reason = "UNDERPOWER: injected power below the reliable-measurement threshold";
                fail_code = IDENT_FAIL_UNDERPOWER;
            }

            drv.motor_ctrl.set_motor_type(MotorType_Invalid);
            set_error(status_tracker, motor, setup_status, fail_reason, fail_code);
        }
    }
};
