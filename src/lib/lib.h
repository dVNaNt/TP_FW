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
 * - @c driver_phy         : physical layer (PWM, ADC current sense, supply).
 * - @c encoder            : magnetic encoder interface.
 *
 * Integration notes:
 * - FlashEEPROM (@c calibration_dump) is declared BEFORE AngleCalibrator so
 *   its address can be injected into the calibrator at construction time.
 * - LED handling is platform-independent through function-pointer callbacks
 *   registered by the board layer (see tunepulse.h).
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#include "common.h"
#include "drivers/encoder_phy/encoder.h"
#include "drivers/pwm_driver/driver_pwm.h"
#include "foc_vector_synthesis.h"  // Position-velocity cascaded controller
#include "math/helpers/frequency_generator.h"
#include "math/motion/motion_executor.h"  // Real-time motion trajectory execution
#include "math/motion/motion_governor.h"
#include "motor_control/calibration/angle_calibrator.h"
#include "motor_control/current_control/current_control.h"
#include "motor_control/motor.h"
#include "motor_control/types.h"

#include "drivers/tunepulse_interlayer.h"

// Forward declaration of the board integration type (defined by target layer)
struct TunePulseHeart;

/**
 * @brief Platform-independent LED control callback signature.
 * @param state true = turn LED on, false = turn LED off (polarity handled by
 *              the board-specific implementation).
 */
typedef void (*LED_Control_Callback)(bool state);

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
 *     - Success          : run angle correction / closed-loop position control.
 *     - CalibError       : zero the current target (safe stop).
 *     - NotCalibrated    : hold phases shorted (INFINITY) until current-sense
 *                          offset calibration completes.
 *     - anything else    : apply a small negative voltage (-0.1 V) as an
 *                          idle/brake state and return early.
 * - Finally computes the alpha-beta voltage vector with FOC and pushes it to
 *   the physical PWM layer.
 */
struct TunePulseHeart {
public:
    Frequency clock_;       ///< System/PWM timing reference (170 MHz core, programmable carrier).
    Motor motor_params;     ///< Motor description parameters (type, poles, limits).
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
    MotorStatus motor_status;               ///< Current high-level motor status (calibration lifecycle).
    uint16_t angle_el_ = 0;                 ///< Last computed electrical rotor angle [0..65535] mapped to 0..360 deg.
    Vec2D_f32 pwm_voltage_ab = {0.0, 0.0};  ///< Last commanded alpha-beta voltage vector [V].
    float current_target_a = 0.5;           ///< Target q-axis current for the FOC loop [A].
    LED_Control_Callback led_callbacks_[3] = {nullptr, nullptr, nullptr};  ///< Registered LED callbacks (index 0..2).
    // FIX (P1.4): re-entrancy guard. Atomic exchange makes the test-and-set
    // indivisible, so an ISR racing with a running tick() can no longer skip
    // or duplicate a control pass; the late sample is intentionally dropped
    // as back-pressure.
    std::atomic<bool> is_running{false};

public:
    DriverPWM driver_phy;  ///< Physical driver layer: PWM output, phase-current ADCs, supply-voltage ADC.
    Encoder encoder;       ///< Magnetic encoder interface (SPI).

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
          current_driver(motor_params, driver_phy),                            // Pass feedback and motor reference
          angle_calibrator(freq_hz, motor_params, &calibration_dump),  // Whole motor (with its table) + table flash
          driver_phy(iA_fs, iB_fs, iC_fs, 128, max_sup),
          motor_status(MotorStatus::NotCalibrated) {}

    /**
     * @brief Register a platform LED callback.
     * @param led_index LED slot index (0..2); out-of-range values are ignored.
     * @param callback  Function invoked with true=ON / false=OFF.
     */
    void set_led_callback(uint8_t led_index, LED_Control_Callback callback) {
        if (led_index < 3) {
            led_callbacks_[led_index] = callback;
        }
    }

    /**
     * @brief Invoke a registered LED callback.
     * @param led_index LED slot index (0..2); ignored if unset or out of range.
     * @param state true = LED on, false = LED off.
     */
    void led_control(uint8_t led_index, bool state) {
        if (led_index < 3 && led_callbacks_[led_index] != nullptr) {
            led_callbacks_[led_index](state);
        }
    }

    /**
     * @brief Direct access to the raw LED callback table (3 entries).
     * @return Pointer to the first element of the callback array.
     */
    LED_Control_Callback* get_led_callbacks() { return led_callbacks_; }

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

    int32_t el_sign = -1;     ///< Sign correction applied to the electrical angle readout.
    int32_t err_sign = -1;    ///< Sign correction applied to the mechanical encoder position.
    int32_t encoder_pos = 0;  ///< Last corrected mechanical encoder position (counts).

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
    void tick() {
        if (is_running.exchange(true, std::memory_order_acq_rel))  // Single-owner entry: drop the sample if a pass is in progress
            return;
        if (driver_phy.current.is_changed() & driver_phy.supply.is_changed()) {
            driver_phy.tick();
            counter++;
            led_control(0, driver_phy.get_fault());



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
            float targt = current_target_a;  // Local working copy of the current target [A]
            switch (motor_status) {
                case MotorStatus::Success: {
                    if (!angle_calibrator.is_calibrated()) {
                        angle_el_ = angle_calibrator.tick(real_raw_pos, (int16_t)current_driver.motor.get_motor_type());
                        if (angle_calibrator.is_calibrated()) {
                            targt = current_target_a = 0.5;
                        }
                        break;
                    } else {
                        // Real electrical angle (needed for closed loop and sync):
                        // apply flash-stored correction then the polarity sign.
                        angle_el_ = (uint16_t)((int16_t)(angle_calibrator.get_el_angle_correction(real_raw_ang)) * el_sign);
                        targt = position_controller.tick(real_pos, 0, 0.0, current_target_a, 0);
                    }

                    break;
                }
                case MotorStatus::CalibError:
                    // Calibration failed: force zero torque command.
                    targt = current_target_a = 0.0;
                    break;
                case MotorStatus::NotCalibrated:
                    angle_calibrator.tick(real_raw_pos, (int16_t)current_driver.motor.get_motor_type());
                    if (driver_phy.current.is_calibrated() == false) {
                        // Current offsets unknown yet: short all phases (INFINITY
                        // sentinel handled by the PWM backend) and bail out.
                        pwm_voltage_ab.a = pwm_voltage_ab.b = INFINITY;
                        driver_phy.motor_ctrl.set_voltage_ab_volts(pwm_voltage_ab);
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
                    is_running.store(false, std::memory_order_release);
                    return;
                }
            }

            if (!is_pos_used) {
                targt = current_target_a;
            }

            // Final FOC step: convert (electrical angle, q-current target) into
            // an alpha-beta voltage vector and hand it to the physical layer.
            pwm_voltage_ab = current_driver.tick_control_foc((int16_t)angle_el_, targt);
            driver_phy.motor_ctrl.set_voltage_ab_volts(pwm_voltage_ab);
        }
        is_running.store(false, std::memory_order_release);
    }

    bool is_pos_used = 1;
};