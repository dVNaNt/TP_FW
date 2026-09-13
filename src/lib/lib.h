#pragma once

/**
 * @file lib.h
 * @brief Unified library entry point and top-level application object.
 *
 * Aggregates the complete migrated control path (3-phase PWM only; legacy
 * variants were removed) into one header and defines TunePulseHeart - the
 * central object that owns every subsystem of the motor controller:
 *
 * - @c clock_             : system/PWM timing reference.
 * - @c motor_params       : motor description (type, pole pairs, ...).
 * - @c current_driver             : FOC current/voltage control current_driver (CurrentControl).
 * - @c calibration_dump   : flash-backed storage for calibration tables.
 * - @c system_dump        : flash-backed storage for generic user data (2 KB).
 * - @c angle_calibrator   : electrical-angle calibration state machine.
 * - @c position_controller: cascaded position/velocity/current synthesizer.
 * - @c drive              : control-mode machine, command mailbox and the
 *                           trajectory queue driven by the communication stack.
 * - @c driver_phy         : physical layer (PWM, ADC current sense, supply).
 * - @c encoder            : magnetic encoder interface.
 *
 * Integration notes:
 * - FlashEEPROM (@c calibration_dump) is declared BEFORE AngleCalibrator so
 *   its address can be injected into the calibrator at construction time.
 * - LED handling is platform-independent through the LedDriver (drivers/led_driver.h):
 *   a single apply callback registered by the board layer receives two packed
 *   RGB888 words, one per LED element (see tunepulse.h).
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#include "common.h"
#include "drivers/encoder_phy/encoder.h"
#include "drivers/led_driver.h"
#include "drivers/pwm_driver/driver_pwm.h"
#include "foc_vector_synthesis.h"  // Position-velocity cascaded controller
#include "math/helpers/frequency_generator.h"
#include "math/motion/motion_executor.h"  // Real-time motion trajectory execution
#include "math/motion/motion_governor.h"
#include "motor_control/calibration/angle_calibrator.h"
#include "motor_control/current_control/current_control.h"
#include "math/helpers/buzzer.h"       // MotorSpeaker: coils-as-speaker tone generator
#include "motor_control/drive_control.h"  // Runtime control-mode machine + trajectory front end
#include "motor_control/motor.h"
#include "motor_control/types.h"
#include "flight_recorder.h"  // Control-rate black box (compiled in with TP_TRACE_DEPTH)

#include "drivers/tunepulse_interlayer.h"
// Forward declaration of the board integration type (defined by target layer)
struct TunePulseHeart;

/** Default PWM carrier frequency in Hz if the board does not define one. */
#ifndef PWM_TIMER_FREQ_HZ
#define PWM_TIMER_FREQ_HZ 20000
#endif

/**
 * Default PWM timer auto-reload period (counts). Derived from a 170 MHz
 * timer clock divided by twice the carrier frequency (center-aligned mode:
 * up-count + down-count per PWM period), i.e. 170'000'000 / 40'000 = 4250.
 */
#ifndef PWM_TIMER_PERIOD
#define PWM_TIMER_PERIOD 170000000 / 40000
#endif

// ============================================================================
// TunePulseHeart - Complete motor control system
// ============================================================================

/**
 * @brief Top-level application object tying together all motor-control
 *        subsystems for a single drive.
 *
 * Lifecycle:
 * 1. Constructed once at file scope (TUNE_HEART in tunepulse.h) with the
 *    board's hardware constants.
 * 2. Initialized by tunepulse_init() (peripherals + protection limits).
 * 3. Driven periodically from the control-loop context via tick().
 *
 * Control flow inside tick():
 * - Waits until both a fresh current-sense sample AND a fresh supply-voltage
 *   sample are available (double-buffer handshake via is_changed()).
 * - Reads the encoder, applies sign corrections, then dispatches on
 *   motor_status:
 *     - Success          : run the angle calibration sweep while the table is
 *                          missing; once calibrated, hand the pass to
 *                          DriveControl (see below).
 *     - CalibError       : zero the current target (safe stop).
 *     - NotCalibrated    : hold phases shorted (INFINITY) until current-sense
 *                          offset calibration completes.
 *     - anything else    : release the bridge (NaN sentinel) and return early.
 * - Finally computes the alpha-beta voltage vector with FOC and pushes it to
 *   the physical PWM layer.
 *
 * Once the drive is calibrated the closed-loop pass is resolved entirely by
 * @c drive (DriveControl): it drains the command mailbox filled by the
 * communication stack, services the master-liveness watchdog, advances the
 * queued trajectory and returns one action plus its targets. tick() only
 * translates that action into the physical layer, so no control policy is
 * duplicated here and a remote master and a local caller reach the drive
 * through exactly the same path.
 *
 * PWM sentinel semantics (see timer_pwm_setup.h::pwm_phase_ccr):
 * - +INFINITY -> INT16_MAX duty on every phase: all HIGH-side switches on, so
 *   every motor terminal is tied to the DC bus (shorted winding = braking).
 * - NaN       -> INT16_MIN duty on every phase, whose magnitude exceeds the
 *   timer period, so BOTH FETs of every phase are disabled: true Hi-Z
 *   freewheel, not "all low-side on".
 */
struct TunePulseHeart {
public:
    Frequency clock_;               ///< System/PWM timing reference (170 MHz core, programmable carrier).
    Motor motor_params;             ///< Motor description parameters (type, poles, limits).
    CurrentControl current_driver;  ///< FOC control current_driver operating on driver_phy.

    /**
     * Flash-backed persistent storage for calibration tables.
     * Declared before angle_calibrator on purpose: its address is injected
     * into the calibrator during construction.
     */
    FlashEEPROM calibration_dump;

    /**
     * Flash-backed persistent storage for generic USER data.
     * A second, fully independent FlashEEPROM instance bound (in the board
     * layer) to the dedicated user-data partition callbacks. All bounds
     * checking and address translation is done inside the NVM flash driver,
     * so this instance can never touch the calibration partition and vice
     * versa. Size comes from the NVM_USER_SECTION_SIZE layout define.
     */
    FlashEEPROM system_dump;

    AngleCalibrator angle_calibrator;                                   ///< Electrical-angle calibration state machine.
    MotorVectorSynthesis position_controller = MotorVectorSynthesis();  ///< Cascaded position/velocity/current controller.
    DriveControl drive;                     ///< Control-mode machine, command mailbox and trajectory queue.
    MotorStatus motor_status;               ///< Current high-level motor status (calibration lifecycle).
    uint16_t angle_el_ = 0;                 ///< Last computed electrical rotor angle [0..65535] mapped to 0..360 deg.
    Vec2D_f32 pwm_voltage_ab = {0.0, 0.0};  ///< Last commanded alpha-beta voltage vector [V].
    float current_target_a = 0.5;           ///< Target q-axis current for the FOC loop [A].
    // FIX (P1.4): re-entrancy guard. Atomic exchange makes the test-and-set
    // indivisible, so an ISR racing with a running tick() can no longer skip
    // or duplicate a control pass; the late sample is intentionally dropped
    // as back-pressure.
    std::atomic<bool> is_running{false};

public:
    DriverPWM driver_phy;  ///< Physical driver layer: PWM output, phase-current ADCs, supply-voltage ADC.
    Encoder encoder;       ///< Magnetic encoder interface (SPI).
    LedDriver led;         ///< Two-element RGB status LED (single apply callback, board-implemented).

    /**
     * @brief Construct the application heart.
     *
     * @param freq_hz  PWM carrier frequency in Hz.
     * @param pwm_res  PWM timer auto-reload period (resolution in counts).
     * @param max_sup  Supply voltage corresponding to the ADC full scale [V].
     * @param iA_fs    Phase-A current corresponding to the ADC full scale [A].
     * @param iB_fs    Phase-B current corresponding to the ADC full scale [A].
     * @param iC_fs    Phase-C current corresponding to the ADC full scale [A].
     *
     * Member initialization order notes:
     * - clock_ runs at 170 MHz with the requested carrier frequency.
     * - current_driver receives references to motor_params and driver_phy.
     * - angle_calibrator receives the carrier frequency and the address of
     *   calibration_dump (flash persistence for the calibration table).
     * - driver_phy is configured with per-phase current full scales, an
     *   oversampling/averaging factor of 128 and the supply full scale.
     */
    TunePulseHeart(uint16_t freq_hz, uint16_t pwm_res, float max_sup, float iA_fs, float iB_fs, float iC_fs)
        : clock_(170000000, freq_hz),
          current_driver(motor_params, driver_phy),                    // Pass feedback and motor reference
          angle_calibrator(freq_hz, motor_params, &calibration_dump),  // Whole motor (with its table) + table flash
          drive(freq_hz),                                              // Control front end shares the loop rate
          driver_phy(iA_fs, iB_fs, iC_fs, 128, max_sup),
          motor_status(MotorStatus::NotCalibrated) {}

    /**
     * @brief "calibrate" command: starts a full angle calibration cycle
     *
     * Arms the calibrator state machine; the actual motion is executed inside
     * tick(). After successful validation the fresh table is persisted into
     * flash automatically (append or erase+rewrite). Load/auto-calibration on
     * boot is handled internally by the calibrator itself.
     */
    void calibrate() { angle_calibrator.calibrate(); }

    /**
     * @brief "load_calibration" command: rebind the flash driver and restore
     *
     * Restarts the non-blocking scan of the memory region; on success the
     * stored table becomes active and is_calibrated() returns true.
     */
    void load_calibration(FlashEEPROM* mem) { angle_calibrator.load_calibration(mem); }

    /**
     * @brief "dump_calibration" command: persist the current RAM table to flash
     */
    void dump_calibration() { angle_calibrator.dump_calibration(); }

    /**
     * @brief Persist ALL motor parameters into system_dump ("save" method).
     *
     * Non-blocking: the record is streamed 8 words per tick from tick().
     */
    void save_motor_params() { motor_params.begin_save(); }

    /**
     * @brief Restart the scan of system_dump for the newest stored parameter
     *        record matching the given Cal-ID ("load" method).
     * @param id Cal-ID to match, or MOTOR_ID_ANY for any record.
     */
    void load_motor_params(uint8_t id = MOTOR_ID_ANY) { motor_params.begin_load(id); }

    int32_t err_sign = -1;    ///< Sign correction applied to the mechanical encoder position.
    int32_t encoder_pos = 0;  ///< Last corrected mechanical encoder position (counts).

    /**
     * @brief Sign that maps the loop's torque command onto the FOC q axis.
     *
     * The position and velocity loops work in encoder units: a positive
     * command must move the reported position up. The FOC frame is fixed by
     * physics instead: a positive q-axis current always advances the rotor
     * in the direction the electrical angle grows, and the angle table makes
     * that angle grow in the sense the sweep rotated the field. Which way the
     * ENCODER counts for that rotation is exactly what Pass0 of the sweep
     * measures and publishes as the motor direction (+1: the encoder counts
     * down for a positive field rotation, -1: it counts up). With
     * @c err_sign = -1 turning the raw count into the reported position, a
     * positive q current moves the reported position by @c direction, so the
     * loop's command has to be multiplied by that direction before it reaches
     * the current controller. A hard-coded +1 was right for the motors the
     * firmware was tuned on and made the position loop positive feedback on
     * every direction = -1 motor: the rotor ran away from any setpoint at the
     * voltage-limited speed with a small measured current.
     *
     * Only the FOC command and the reported measured q current cross this
     * sign; everything the loops and the protocol see stays in encoder units.
     */
    float torque_sign() const {
        return (motor_params.get_direction() < 0) ? -1.0f : 1.0f;  // Unknown (0) counts as +1
    }

    // Position-hold target for the position controller. Captured at the moment
    // the angle calibration completes (and at every controller re-arm), so the
    // rotor HOLDS where it is instead of being yanked to the absolute
    // integrator zero. The hardcoded zero target made the drive fly up to a
    // full turn across the 50-pole-pair detent field right after every
    // calibration - which read as "the corrected angle is garbage" - while a
    // reboot looked fine only because the previous session had already parked
    // the rotor exactly at that zero.
    int32_t pos_hold_target_ = 0;

    /// Current commanded to the FOC loop on the previous pass [A]. Feeds the
    /// executor torque triggers and the telemetry snapshot, so a segment can
    /// end on a force condition without the control loop measuring anything new.
    float last_current_a_ = 0.0f;

    /// True while the previous pass was routed to a runtime electrical
    /// identification. The falling edge is where the front end is re-seeded:
    /// the routine rotates the shaft, so the targets it froze are stale.
    bool was_identifying_ = false;

    /**
     * @brief Latched copy of the encoder's fault flag.
     *
     * The encoder object only reports the LIVE condition (the board's link
     * supervisor or the sensor's own status bits raised it, the next healthy
     * frame clears it) and its fault action only restores the sensor link.
     * Position feedback that came and went is still a fault the operator
     * has to acknowledge: the rotor may have moved while the bridge was
     * released, and resuming the loop against a stale target on the first
     * healthy frame would snap it back. So the condition is latched here, in
     * the integration layer - it is not a power-stage fault and does not
     * belong in the driver's fault byte - until a fault reset releases it
     * (clear_encoder_fault()). While latched the control pass keeps the
     * bridge released and closed loop is refused.
     */
    bool encoder_fault_latched_ = false;

    /// Bit of fault_byte() that stands for the latched encoder fault, above the driver's seven.
    static constexpr uint8_t kEncoderFaultBit = 0x80u;

#ifdef TP_TRACE_DEPTH
    /**
     * @brief Black box: the last TP_TRACE_DEPTH closed-loop passes, frozen after a fault.
     *
     * Fed at the end of every closed-loop pass (angle, measured d/q current,
     * commanded d/q voltage, commanded q current, position error) and read
     * out through protocol register 0x44. A fault reset thaws it.
     */
    FlightRecorder<TP_TRACE_DEPTH> flight;
    int32_t trace_perr_q16_ = 0;   ///< Position error of this pass, captured at the dispatch for the record.
#endif

    /**
     * @brief The drive's fault byte: the driver's latched faults plus the encoder latch.
     *
     * This is what the protocol reports (system / driver status, the beat) and
     * what every "refuse while faulted" guard consults. Bits 0-6 are the
     * driver's own (DriverPWM::FAULT_*), bit 7 is kEncoderFaultBit.
     */
    uint8_t fault_byte() const {
        return (uint8_t)(driver_phy.get_fault() | (encoder_fault_latched_ ? kEncoderFaultBit : 0u));
    }

    /**
     * @brief Release the encoder fault latch (the operator's fault reset).
     *
     * A link that is still down re-latches on the very next control pass,
     * like every other latched condition, so this cannot mask a real fault.
     */
    void clear_encoder_fault() { encoder_fault_latched_ = false; }

    /// Endstop / probe input word handed to the trajectory triggers. No such
    /// inputs are wired on CLN17 v3 yet, so it stays zero; a board that adds
    /// them only has to publish their state here.
    uint8_t io_inputs_ = 0;

    /**
     * @brief Periodic control-loop entry point (called once per fresh sample).
     *
     * Detailed behavior:
     * 1. Gate on data freshness: proceeds only when BOTH the current-sense
     *    block and the supply-voltage block report new samples. The bitwise
     *    '&' (not '&&') is intentional so both is_changed() flags are always
     *    evaluated/cleared in the same pass.
     * 2. Reads the raw encoder angle and the signed mechanical position.
     *    Note: real_raw_pos multiplies real_pos by err_sign again, cancelling
     *    the earlier err_sign multiplication (see CODE_ISSUES.md).
     * 3. State dispatch on motor_status (see class description).
     * 4. Computes the alpha-beta voltage vector via current_driver.tick_control_foc()
     *    using the electrical angle and the target current, then forwards it
     *    to the physical layer.
     */

    float temperature = 0;
    uint32_t counter = 0;

    /**
     * @brief Coils-as-speaker tone generator (DriveMode::Tone).
     *
     * Constructed at the control-loop rate because it synthesizes the waveform
     * one sample per control pass: the frequency it is asked for is only
     * meaningful relative to that rate.
     */
    MotorSpeaker speaker{PWM_TIMER_FREQ_HZ};
    void tick() {
        if (is_running.exchange(true, std::memory_order_acq_rel))  // Single-owner entry: drop the sample if a pass is in progress
            return;
        if (driver_phy.current.is_changed() & driver_phy.supply.is_changed()) {
            driver_phy.tick();
            counter++;
            // Position feedback lost: latch it (see encoder_fault_latched_).
            // The encoder keeps its flag raised for as long as the condition
            // lasts, so a fault reset that comes too early is re-latched here
            // on the very next pass, like every other latched condition.
            if (encoder.is_fault()) {
                encoder_fault_latched_ = true;
            }
            led.set().led1().r(fault_byte() ? 255 : 0);  // LED1 = fault indicator (red channel)
            uint32_t faulte = fault_byte();
            // if (faulte) {
            //     SerialUSB.println(faulte);
            // }

            uint32_t real_raw_ang = encoder.get_angle();           // Raw mechanical angle (no sign fix)
            int32_t real_pos = encoder.get_position() * err_sign;  // Corrected mechanical position
            encoder_pos = real_pos;
            int32_t real_raw_pos = real_pos * err_sign;  // NOTE: double-applies err_sign

            // ========================================================================
            // Non-blocking flash service: advances the motor-parameter
            // load/save engine by at most one 8-word block per system tick
            // (boot-time restore from system_dump, post-calibration save).
            // ========================================================================
            motor_params.service_tick();

            // ========================================================================
            // Motor controller tick logic
            // ========================================================================
            bool flash_window = false;       // True when THIS pass leaves the bridge in a zero-current state
            float targt = current_target_a;  // Local working copy of the current target [A]
            float foc_sign = 1.0f;           // Encoder-units -> FOC-frame sign applied to targt (closed loop only)

            // Closed loop is reachable only with a valid angle table; every
            // other state (boot, identification, angle sweep, fault) drives the
            // bridge directly and bypasses the front end. An identification
            // started at runtime (TP_CALIB_ELECTRICAL over the protocol) counts
            // too: MotorCalibration owns the bridge until it reports Success.
            const bool identifying = current_driver.is_identifying();
            const bool closed_loop = (motor_status == MotorStatus::Success) && angle_calibrator.is_calibrated() &&
                                     !identifying && !encoder_fault_latched_;
            if (!closed_loop) {
                // Still service the front end so commands arriving during boot
                // or calibration are consumed rather than piling up in the
                // mailbox until it overflows, and so the telemetry snapshot
                // keeps reporting the real position instead of stale zeros.
                // inhibit = true freezes trajectory time and yields no output:
                // the branches below command the bridge themselves.
                (void)drive.tick(real_pos, last_current_a_, io_inputs_, true);
            }
            switch (motor_status) {
                case MotorStatus::Success: {
                    if (encoder_fault_latched_) {
                        // Position feedback lost (or was, until the operator
                        // resets): every position-dependent routine below - the
                        // sweep, the closed-loop dispatch - would act on a
                        // stale or phantom angle. Release the bridge (NaN
                        // sentinel: both FETs of every phase off) and hold it
                        // released until the fault reset. This is THE place
                        // that decides what the machine does without its
                        // position feedback (the encoder's own fault action only
                        // restores the sensor link): a sensorless estimator or
                        // a second encoder would be selected here instead of
                        // the release.
                        pwm_voltage_ab.a = pwm_voltage_ab.b = NAN;
                        driver_phy.motor_ctrl.set_voltage_ab_volts(pwm_voltage_ab);
                        last_current_a_ = 0.0f;             // Nothing commanded through FOC
                        tp_flash_programming_window = true;  // Bridge released: NVM programming safe
                        is_running.store(false, std::memory_order_release);
                        return;
                    }
                    if (!angle_calibrator.is_calibrated()) {
                        angle_el_ = angle_calibrator.tick(real_raw_pos, (int16_t)current_driver.motor.get_motor_type());
                        if (angle_calibrator.is_calibrated()) {
                            current_driver.foc_current_.reset();
                            // Hold the rotor exactly where the calibration left
                            // it: capturing the target HERE (instead of using a
                            // hardcoded absolute zero) prevents the post-
                            // calibration lurch across the 50-pole-pair detent
                            // field that read as "the corrected angle is garbage".
                            pos_hold_target_ = real_pos;
                            // Rebase the drive front end onto the same position:
                            // rearm() re-seeds every target and the trajectory
                            // queue, so the first commanded pass after the sweep
                            // can never contain a step either.
                            drive.rearm(real_pos, drive.now_us());
                            targt = current_target_a = 0.5;
                        } else if (angle_calibrator.is_settling() || angle_calibrator.is_persisting() ||
                                   motor_params.is_flash_busy()) {
                            // Flash I/O window (boot scan / table dump / read-back
                            // / MOTOR-PROFILE SAVE): every 32-byte flash programming
                            // stalls the single-bank CPU for several PWM periods (a
                            // full-region erase for tens of ms), so keep the phases
                            // de-energized instead of holding 0.5 A at a frozen
                            // angle. The motor-profile save started by
                            // MotorCalibration on a first boot (no stored profile)
                            // lands INSIDE this window and previously corrupted the
                            // Pass0 step statistics ("Motion too inconsistent").
                            targt = 0.0f;
                            flash_window = true;  // Zero-current vector latched: NVM programming allowed
                        } else if (angle_calibrator.is_failed() || angle_calibrator.is_parked()) {
                            // A sweep that ended in error, or none requested yet
                            // (the table was never stored and calibrate() has not
                            // been called): release the rotor. Holding 0.5 A at
                            // the angle the sweep stopped at pinned the shaft,
                            // heated the coil and closed the NVM window until the
                            // next reboot, so saves and erases were refused too.
                            targt = 0.0f;
                            flash_window = true;  // Zero-current vector latched: NVM programming allowed
                        }
                        break;
                    }

                    // ================================================================
                    // Runtime electrical identification.
                    //
                    // The routine is advanced inside tick_control_foc(), which
                    // supplies its own d/q test voltages and steers the angle
                    // itself, so the pass skips the front end (serviced
                    // inhibited above) and falls through to the FOC step with a
                    // zero target. Before this branch existed the protocol could
                    // arm the routine but the disabled drive took the Freewheel
                    // return below and it never ran - until the stage was next
                    // enabled, when it ran inside whatever mode that was.
                    // ================================================================
                    if (identifying) {
                        was_identifying_ = true;
                        // A fault releases the bridge but leaves the routine's
                        // ramp voltage armed: the instant the fault cleared it
                        // resumed at full voltage and tripped again, forever.
                        // Abort instead; the boot path is untouched.
                        if (fault_byte() != 0u) {
                            current_driver.abort_identification("DRIVER FAULT during runtime identification",
                                                                fault_byte());
                        }
                        targt = 0.0f;                      // MotorCalibration commands the voltages
                        break;                             // Straight to tick_control_foc()
                    }
                    if (was_identifying_) {
                        // Identification just finished: the slow-rotation stage
                        // moved the shaft while the front end's targets were
                        // frozen, so re-seed them from the measured position or
                        // the next enable would snap the rotor back to where it
                        // was before the routine started.
                        was_identifying_ = false;
                        drive.rearm(real_pos, drive.now_us());
                        pos_hold_target_ = real_pos;
                    }

                    // Real electrical angle (needed for closed loop and sync).
                    //
                    // The table returns the angle the sweep COMMANDED when the
                    // rotor stood at this encoder value, counted in the sweep's
                    // own rotation sense (the sense that makes the encoder count
                    // up). Two direction-dependent steps turn it into the rotor's
                    // electrical angle in the FOC frame:
                    //
                    //  1. De-rotate by the detected direction. The sweep ran the
                    //     field at -direction * step per sample, so the commanded
                    //     FOC angle at sample k is -direction * table angle.
                    //  2. Add a quarter period. The sweep held the rotor with a
                    //     q-axis current, whose field leads the commanded angle by
                    //     90 degrees electrical, and a free rotor aligns its
                    //     d-axis with that field.
                    //
                    // For direction = +1 this is exactly what the earlier fixed
                    // "-90 degrees inside the table" produced, so nothing changes
                    // for the motors it was tuned on; for direction = -1 that fixed
                    // shift landed 180 degrees off and made the position loop
                    // positive feedback - the rotor ran away from every setpoint
                    // until the overcurrent comparator tripped.
                    {
                        const int32_t table_el = (int16_t)angle_calibrator.get_el_angle_correction(real_raw_ang);
                        const int32_t rotor_el = (table_el * -(int32_t)motor_params.get_direction()) + 16384;
                        angle_el_ = (uint16_t)rotor_el;
                    }

                    // ================================================================
                    // Closed-loop dispatch through the drive front end.
                    //
                    // DriveControl owns everything a remote master can change:
                    // the control mode, the position/velocity/current targets,
                    // the trajectory queue and the master-liveness watchdog. It
                    // resolves all of that into ONE action plus its targets, so
                    // this branch stays a pure translation from that action to
                    // the physical layer and no control policy lives here.
                    //
                    // A motor-profile record write (e.g. the post-calibration
                    // save) is signalled as an NVM window: the front end then
                    // freezes trajectory time and commands zero current for the
                    // transfer ticks, which is exactly what this branch did
                    // before the front end existed.
                    // ================================================================
                    const bool nvm_window = motor_params.is_flash_busy();
                    const DriveOutput cmd = drive.tick(real_pos, last_current_a_, io_inputs_, nvm_window);

                    // The front end and the loops work in encoder units; the
                    // current they ask for crosses into the FOC frame through
                    // torque_sign(). The sweep and the identification above
                    // never reach this switch, so their frame is untouched.
                    foc_sign = torque_sign();
                    switch (cmd.action) {
                        case DriveAction::RunPosition:
                            // Cascaded position loop against the resolved target.
                            targt = foc_sign * position_controller.tick(real_pos, cmd.target_pos_q16, cmd.target_vel_rps,
                                                                        cmd.max_current_a, false);
#ifdef TP_TRACE_DEPTH
                            trace_perr_q16_ = cmd.target_pos_q16 - real_pos;  // For the black box
#endif
                            break;
                        case DriveAction::RunTorque:
                            // Position loop bypassed: command the q-axis current directly.
                            targt = foc_sign * cmd.torque_target_a;
                            break;
                        case DriveAction::Tone: {
                            // The coils are a speaker: no regulator runs and the
                            // alpha-beta vector comes straight from the tone
                            // generator, one sample per control pass.
                            //
                            // Amplitude is a Q15 fraction of the measured supply
                            // rail rather than a fixed voltage, so a note keeps
                            // the same loudness as the bus sags, and the same
                            // command is safe on a 12 V and a 48 V bus.
                            // Full-scale carrier: MotorSpeaker's second argument
                            // IS the generator amplitude, so passing 0 here
                            // silences it outright. Loudness is applied below in
                            // volts instead, which keeps it explicit and
                            // independent of the class's internal base level.
                            const Vec2D wave = speaker.tick(cmd.tone_freq_hz, 32767);
                            const float scale = (cmd.tone_amp_q15 * driver_phy.supply.get_voltage()) / 32768.0f / 32768.0f;
                            pwm_voltage_ab.a = (float)wave.a * scale;
                            pwm_voltage_ab.b = (float)wave.b * scale;
                            driver_phy.motor_ctrl.set_voltage_ab_volts(pwm_voltage_ab);
                            last_current_a_ = 0.0f;              // Nothing commanded through FOC
                            // The bridge IS driven here, so this is emphatically
                            // not a window in which flash may be programmed.
                            tp_flash_programming_window = false;
                            is_running.store(false, std::memory_order_release);
                            return;
                        }
                        case DriveAction::Brake: {
                            // INFINITY sentinel: all high-side switches ON, so every
                            // motor terminal is tied to the DC bus. The shorted
                            // winding brakes the rotor and no vector is latched.
                            pwm_voltage_ab.a = pwm_voltage_ab.b = INFINITY;
                            driver_phy.motor_ctrl.set_voltage_ab_volts(pwm_voltage_ab);
                            last_current_a_ = 0.0f;             // Nothing commanded through FOC
                            tp_flash_programming_window = true;  // Phases shorted: NVM programming safe
                            is_running.store(false, std::memory_order_release);
                            return;
                        }
                        case DriveAction::Freewheel:
                        default: {
                            // NaN sentinel: INT16_MIN duty on all phases, which the
                            // PWM backend resolves to |duty| >= period, i.e. BOTH
                            // FETs of every phase disabled (glitch-free Hi-Z).
                            pwm_voltage_ab.a = pwm_voltage_ab.b = NAN;
                            driver_phy.motor_ctrl.set_voltage_ab_volts(pwm_voltage_ab);
                            last_current_a_ = 0.0f;             // Nothing commanded through FOC
                            tp_flash_programming_window = true;  // Bridge released: NVM programming safe
                            is_running.store(false, std::memory_order_release);
                            return;
                        }
                    }

                    if (nvm_window) {
                        flash_window = true;  // Zero-current vector latched: NVM programming allowed
                    }

                    break;
                }
                case MotorStatus::NotCalibrated:
                    angle_calibrator.tick(real_raw_pos, (int16_t)current_driver.motor.get_motor_type());
                    if (driver_phy.current.is_calibrated() == false) {
                        // Current offsets unknown yet: short all phases (INFINITY
                        // sentinel handled by the PWM backend) and bail out.
                        pwm_voltage_ab.a = pwm_voltage_ab.b = INFINITY;
                        driver_phy.motor_ctrl.set_voltage_ab_volts(pwm_voltage_ab);
                        tp_flash_programming_window = true;  // Phases shorted: NVM programming safe
                        is_running.store(false, std::memory_order_release);
                        return;
                    } else if (current_driver.motor.get_status() == MotorStatus::Success) {
                        if (!angle_calibrator.is_settling()) {
                            if (!angle_calibrator.is_calibrated()) {
                                angle_calibrator.calibrate();
                                targt = current_target_a = 0.5;
                            }
                            motor_status = MotorStatus::Success;
                        }
                    }
                    break;
                default: {
                    // Any other transient state: apply a small negative voltage
                    // (active brake / idle) and skip FOC this cycle.
                    pwm_voltage_ab.a = pwm_voltage_ab.b = NAN;
                    driver_phy.motor_ctrl.set_voltage_ab_volts(pwm_voltage_ab);
                    tp_flash_programming_window = true;  // All low-side ON: NVM programming safe
                    is_running.store(false, std::memory_order_release);
                    return;
                }
            }

            // Final FOC step: convert (electrical angle, q-current target) into
            // an alpha-beta voltage vector and hand it to the physical layer.
            // targt is already in the FOC frame; the remembered value is
            // folded back into encoder units (foc_sign is +-1) so the
            // trajectory triggers and the telemetry see the loop's own sign.
            last_current_a_ = foc_sign * targt;  // What the loop asked for: next pass feeds it to the triggers
            pwm_voltage_ab = current_driver.tick_control_foc((int16_t)angle_el_, targt);
            driver_phy.motor_ctrl.set_voltage_ab_volts(pwm_voltage_ab);
#ifdef TP_TRACE_DEPTH
            {
                // Black box: what this pass measured and commanded, in units an
                // int16 covers (mA, 10 mV). Frozen a few passes after a fault.
                FlightRecord rec;
                rec.angle_el = (uint16_t)current_driver.get_angle();
                rec.iq_ma = (int16_t)(current_driver.current_dq.torque * 1000.0f);
                rec.id_ma = (int16_t)(current_driver.current_dq.flux * 1000.0f);
                rec.uq_cv = (int16_t)(current_driver.voltage_dq_.torque * 100.0f);
                rec.ud_cv = (int16_t)(current_driver.voltage_dq_.flux * 100.0f);
                rec.icmd_ma = (int16_t)(targt * 1000.0f);
                const Vec2D_f32 iab = driver_phy.motor_ctrl.get_current_ab_amps();  // Coil currents the loop used
                rec.ia_ma = (int16_t)(iab.a * 1000.0f);
                rec.ib_ma = (int16_t)(iab.b * 1000.0f);
                rec.ua_cv = (int16_t)(pwm_voltage_ab.a * 100.0f);                     // Vector handed to the bridge
                rec.ub_cv = (int16_t)(pwm_voltage_ab.b * 100.0f);
                rec.vbus_cv = (int16_t)(driver_phy.supply.get_voltage() * 100.0f);
                const int32_t perr_mrev = (trace_perr_q16_ * 1000) / 65536;          // 1/1000 rev
                rec.perr_mrev = (int16_t)((perr_mrev > 32767) ? 32767 : ((perr_mrev < -32768) ? -32768 : perr_mrev));
                flight.push(rec, fault_byte());
            }
#endif

            // Publish the NVM programming window: true only when THIS pass
            // commanded zero current (flash-I/O gates above), so the vector
            // latched on the bridge during the next flash programming stall
            // cannot yank an energized rotor. IMPORTANT: the flag describes
            // the vector that is ON the bridge RIGHT NOW - the write engines
            // sample it on the NEXT pass, which is exactly the stall case.
            tp_flash_programming_window = flash_window;
        }
        is_running.store(false, std::memory_order_release);
    }

};