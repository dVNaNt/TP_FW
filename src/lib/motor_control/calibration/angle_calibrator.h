/**
 * @file angle_calibrator.h
 * @brief Motor angle calibration state machine and control
 *
 * Implements a multi-stage calibration process for motor encoder alignment.
 * The calibration performs the following stages:
 *
 * 1. Setup: Initialize calibration parameters
 * 2. Reset: Perform test steps to verify motor response
 * 3. Pass0: Test single pole motion to detect direction and consistency
 * 4. Pass1: Full rotation in positive direction with position sampling
 * 5. Pass2: Full rotation in negative direction with position sampling
 * 6. Check: Validate calibration data consistency
 * 7. Ready: Calibration complete, motor ready for operation
 *
 * Each calibration cycle consists of four sub-stages:
 * - Setup: Prepare for new cycle
 * - Rotating: Move motor at specified speed
 * - Waiting: Allow motor to settle
 * - Sampling: Oversample encoder position for stable reading
 *
 * The calibration removes hysteresis by collecting data in both directions
 * and builds a correction table for accurate position feedback.
 *
 * Flash-backed lifecycle (non-blocking, 8 words / 32 bytes transferred per tick):
 * - calibrate() first tries the stored calibration table referenced by the
 *   motor profile's Cal-ID (= chunk address of its 0xFA3E marker word).
 *   A hit moves straight to Ready; a miss falls through to the physical
 *   cycle (marker + sample count + strictly monotonic samples).
 * - calibrate() always restarts the cycle at Setup. After validation the new
 *   table MUST be persisted ("dump_calibration" runs automatically) before
 *   the machine reaches Ready: appended into free space, or erase + rewrite
 *   from offset zero when no room remains.
 *
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#pragma once

#include "../../common.h"
#include "../motor.h"  // Whole Motor object - owns the CalibrationTable filled here
#include "calibration_table.h"

/**
 * @brief Calibration stage enumeration
 *
 * Represents the current stage of the overall calibration process.
 */
enum CalStage {
    CalStage_Reset = 0,  // Initial setup stage for calibration
    CalStage_Setup = 1,  // Stage where motor is allowed to settle before starting
    CalStage_Pass0 = 2,  // Test single pole motion for direction detection
    CalStage_Pass1 = 3,  // First pass: full rotation with sampling
    CalStage_Pass2 = 4,  // Second pass: reverse rotation with sampling
    CalStage_Check = 5,  // Verify calibration data consistency
    CalStage_Ready = 6,  // Calibration complete and ready for operation
    CalStage_Error = 7,  // Error occurred during calibration

    CalStage_CheckMemory = 8,  // Non-blocking flash scan for a stored table (default stage)
    CalStage_Save = 9,         // Persist the freshly validated table into flash
};

/**
 * @brief Calibration sampling state enumeration
 *
 * Represents the state within each calibration cycle:
 * Setup -> Rotating -> Waiting -> Sampling
 */
enum CalSamplingState {
    CalSamplingState_Setup,     // Initial setup before starting rotation
    CalSamplingState_Rotating,  // Motor is rotating during calibration step
    CalSamplingState_Waiting,   // Motor is stationary, waiting for settling
    CalSamplingState_Sampling,  // Oversample encoder position for averaging
};

/**
 * @brief Calibration configuration constants
 */
enum {
    ANGLE_CAL_CAL_SETTLING_TIME_US = 10000,  // Settling time in microseconds
    ANGLE_CAL_CAL_SPEED_US = 2500,           // Speed in angle increments per microsecond
    ANGLE_CAL_CAL_OVERSEMPLING = 128,        // Number of samples per oversampling period
    ANGLE_CAL_CAL_FIRST_STEP_USTEPS = 8      // Number of test steps in Pass0
};

/**
 * @brief Motor angle calibrator state machine
 *
 * Manages the complete calibration process including motor movement,
 * position sampling, and calibration table generation.
 */
struct AngleCalibrator {
public:
    /**
     * @brief Creates a new AngleCalibrator instance
     * @param frequency Update frequency in ticks per second
     * @param motor_ref Whole motor object - its embedded CalibrationTable is
     *                  filled in place by this calibrator. Ownership: the
     *                  table ALWAYS lives inside the Motor (no fallback).
     * @param table_flash Flash EEPROM driver injected into the calibration
     *                    table (the calibration-table partition).
     * @return Initialized AngleCalibrator
     */
    AngleCalibrator(uint16_t freq, Motor& motor_ref, FlashEEPROM* table_flash = nullptr)
        : motor(motor_ref) {
        frequency = freq;
        settling_time = calculate_settling_time(frequency, ANGLE_CAL_CAL_SETTLING_TIME_US);  // Convert settling time to ticks
        ptab = &motor.cal_table;             // The calibration table lives INSIDE the Motor object
        *ptab = CalibrationTable::create();  // Fresh RAM table
        ptab->attach_flash(table_flash);     // Bind the persistence driver
        // NOTE: the boot-time table load cannot start here - the calibration
        // ID (chunk address) becomes known only after the motor parameter
        // profile has been restored from system_dump. calibrate() arms it.
    }

    /**
     * @brief Electrical-period step of the connected motor
     * @return Samples per electrical period: 6 for BLDC, 4 for STEPPER.
     */
    uint16_t cal_step() const {
        return (motor.get_motor_type() == MotorType_BLDC) ? 6 : 4;
    }

    /**
     * @brief Motor connectivity check
     * @return true once the motor type has been identified (motor present)
     *
     * A motor that was never identified (MotorType_Invalid) is treated as
     * not connected: neither the flash scan nor a calibration cycle makes
     * sense without it.
     */
    bool is_motor_connected() const { return motor.get_motor_type() != MotorType_Invalid; }

    /**
     * @brief Main calibration tick function
     * @param encoder_pos Current encoder position reading
     * @param motor_type_step Motor type calibration step (4 for Stepper, 6 for BLDC)
     * @return Current electrical angle for motor control
     *
     * Called at regular intervals. Every tick services exactly 8 words (32
     * bytes) of flash traffic (load scan or dump write), keeping the control
     * loop non-blocking. The motor-driving calibration sequence only advances
     * after an explicit or automatic calibration request.
     */
    uint16_t tick(int32_t encoder_pos, uint8_t motor_type_step) {
        position = encoder_pos;    // Update internal position from sensor

        switch (calibration_stage) {
            case CalStage_CheckMemory:  // Boot-time flash scan, 8 words (32 bytes) per tick
                if (ptab->load_tick()) {
                    if (ptab->is_loaded()) {   // Branch 1: valid table -> straight to Ready
                        TP_LOG("CALIBRATION: Valid table restored from flash");
                        loaded_from_flash = true;
                        calibration_stage = CalStage_Ready;
                    } else {                       // Branch 2: none/invalid -> park in Reset
                        TP_LOG("CALIBRATION: No valid table in flash - waiting for calibrate()");
                        calibration_stage = CalStage_Reset;
                    }
                }
                return angle_el;  // Motor is never driven while scanning memory

            case CalStage_Save:  // Persist the freshly validated table, 8 words (32 bytes) per tick
                if (ptab->dump_tick()) {
                    if (ptab->dump_ok()) {
                        TP_LOG("CALIBRATION: Table saved to flash");
                        // Publish the fresh calibration reference into the
                        // motor profile: the ID is the chunk address of the
                        // FA3E marker. Also store the measured angle offset,
                        // then re-save the parameter record into system_dump
                        // (non-blocking, 8 words per tick from TunePulseHeart::tick).
                        motor.set_cal_id((uint8_t)ptab->dumped_id());
                        motor.set_calibration_offset((int32_t)ptab->get_offst_val());
                        motor.begin_save();
                        calibration_requested = false;       // Cycle fully finished
                        calibration_stage = CalStage_Ready;  // Ready only after a successful save
                    } else {
                        TP_LOG("CALIBRATION: ERROR - flash save failed");
                        calibration_stage = CalStage_Error;
                    }
                }
                return angle_el;

            case CalStage_Reset:  // Idle parking spot until an external trigger arrives
                if (!calibration_requested) return angle_el;
                break;

            case CalStage_Ready:
            case CalStage_Error:
                return angle_el;

            default:
                break;
        }

        int32_t stable_pos = run_sampling_cycle(ang_el_step);  // Perform calibration cycle and get stable position

        if (stable_pos != INT32_MIN) {
            // Stable position obtained, advance calibration stage
            switch (calibration_stage) {
                case CalStage_Setup:
                    TP_LOG("CALIBRATION: Starting calibration process");         // Log calibration start
                    cal_idx = 10;                                                // Set initial iteration count
                    calibration_stage = CalStage_Reset;                          // Move to Reset stage
                    speed = calculate_speed(frequency, ANGLE_CAL_CAL_SPEED_US);  // Calculate speed in ticks
                    TP_LOG("CALIBRATION: Reset stage - performing test steps");  // Log stage transition
                    return angle_el;                                             // Return current angle

                case CalStage_Reset:
                    if (iter(&cal_idx)) {
                        // Reset stage complete, prepare for Pass0
                        init_pos = stable_pos;                                                   // Record initial position
                        temp_pos = stable_pos;                                                   // Initialize temporary position
                        dif_max = INT32_MIN;                                                     // Reset max difference
                        dif_min = INT32_MAX;                                                     // Reset min difference
                        ang_el_step = (uint16_t)(UINT16_MAX / ANGLE_CAL_CAL_FIRST_STEP_USTEPS);  // Calculate step size
                        cal_idx = ANGLE_CAL_CAL_FIRST_STEP_USTEPS - 1;                           // Set number of test steps
                        calibration_stage = CalStage_Pass0;                                      // Move to Pass0
                        TP_LOG("CALIBRATION: Test single pole motion");                          // Log stage transition
                    }
                    break;

                case CalStage_Pass0: {
                    // Test single pole motion to detect direction and consistency
                    int32_t dif = temp_pos - stable_pos;  // Calculate step difference
                    temp_pos = stable_pos;                // Update temporary position
                    if (dif < dif_min)
                        dif_min = dif;  // Track minimum difference
                    if (dif > dif_max)
                        dif_max = dif;  // Track maximum difference

                    if (iter(&cal_idx)) {
                        // Pass0 complete, analyze results
                        int32_t travel = init_pos - temp_pos;        // Calculate total travel distance
                        int32_t direction = (travel >= 0) ? 1 : -1;  // Determine rotation direction
                        this->direction = direction;                 // Store detected direction

                        int32_t avg_step = (travel * direction) / ANGLE_CAL_CAL_FIRST_STEP_USTEPS;  // Calculate average step size
                        int32_t deviation = dif_max - dif_min;  // Calculate step deviation range

                        if (avg_step < deviation) {
                            // Motion too inconsistent, calibration failed
                            TP_LOG("CALIBRATION: ERROR - Motion too inconsistent");  // Log calibration error
                            calibration_stage = CalStage_Error;                      // Move to Error stage
                            return angle_el;                                         // Return current angle
                        }

                        TP_LOG_VAL("CALIBRATION: Detected motion direction: ", (long)this->direction);  // Log detected direction
                        motor.set_direction((int16_t)this->direction);  // Publish detected rotation direction into the motor profile
                        calibration_stage = CalStage_Pass1;
                        // Move to Pass1
                        ang_el_step =
                            (uint16_t)(65536u /
                                       motor_type_step);  // Calculate step size based on motor type (4 for Stepper, 6 for BLDC)
                        speed = (int32_t)(-speed * this->direction);  // Adjust speed for detected direction
                        ptab->reset(motor_type_step);             // Reset calibration table with motor-specific step count
                        ptab->fill_first(0, (uint16_t)stable_pos);
                        cal_idx = 1;            // Reset calibration index
                        init_pos = stable_pos;  // Record starting position for Pass1
                        TP_LOG("CALIBRATION: Full rotation in positive direction with sampling");  // Log stage transition
                    }
                    break;
                }

                case CalStage_Pass1: {
                    // Full rotation in positive direction with position sampling
                    int32_t avg_step = (stable_pos - init_pos) / (int32_t)(cal_idx);  // Calculate average step size
                    if (stable_pos - init_pos > (int32_t)UINT16_MAX + (avg_step / 3)) {
                        // Completed full rotation, move to Pass2
                        calibration_stage = CalStage_Pass2;  // Move to Pass2
                        TP_LOG_VAL("CALIBRATION: Full rotation in negative direction with sampling ",
                                   cal_idx);  // Log stage transition
                        speed = -speed;       // Reverse speed for backward rotation
                        return angle_el;      // Return current angle
                    }
                    ptab->fill_first(cal_idx, (uint16_t)stable_pos);  // Store position in calibration table
                    cal_idx += 1;                                         // Increment calibration index
                    break;
                }

                case CalStage_Pass2:
                    // Full rotation in negative direction with position sampling
                    if (cal_idx > 0) {
                        cal_idx -= 1;  // Decrement calibration index
                    }
                    ptab->fill_second(cal_idx, (uint16_t)stable_pos);  // Store backward position in table

                    if (cal_idx == 0) {
                        // Pass2 complete, move to Check stage
                        TP_LOG("CALIBRATION: Pass2 complete - validating data");  // Log stage transition
                        calibration_stage = CalStage_Check;                       // Move to Check stage
                        angle_el = 0;                                             // Reset electrical angle
                    }
                    break;

                case CalStage_Check:
                    // Validate calibration data, then require persistence before Ready
                    if (ptab->check()) {
                        // The sample count must be a whole number of
                        // electrical periods: multiple of 6 for BLDC,
                        // multiple of 4 for STEPPER. Trim the excess and
                        // derive the final pole count (n / div).
                        uint16_t div = cal_step();
                        uint16_t n = ptab->get_sample_count();
                        if (n == 0 || (n % div) != 0) {
                            ptab->truncate((uint16_t)(n - (n % div)));
                            n = ptab->get_sample_count();
                            TP_LOG_VAL("CALIBRATION: samples trimmed to ", (long)n);
                        }
                        if (n == 0 || (n % div) != 0) {
                            TP_LOG("CALIBRATION: ERROR - invalid sample count");
                            calibration_stage = CalStage_Error;
                            break;
                        }
                        int32_t poles = (int32_t)n / (int32_t)div;
                        if (motor.get_pole_pairs() != poles) {
                            TP_LOG_VAL("CALIBRATION: pole pairs derived from samples: ", poles);
                            motor.set_pole_pairs(poles);  // Store the identified pole count
                        }

                        TP_LOG("CALIBRATION: Validation complete - saving to flash");
                        ptab->begin_dump();  // Dump is mandatory for the Ready stage
                        calibration_stage = CalStage_Save;
                    } else {
                        TP_LOG("CALIBRATION: Validation error");  // Log validation failure
                        calibration_stage = CalStage_Error;
                    }
                    break;

                case CalStage_Error:
                case CalStage_Ready:
                default:
                    break;  // No action in Error or Ready states
            }
        }
        return angle_el;  // Return current electrical angle
    }

    /**
     * @brief Starts a full calibration cycle ("calibrate" command)
     *
     * Resets the state machine to its initial Setup stage and arms the
     * calibration request. Until the new cycle reaches the Ready stage,
     * is_calibrated() reports false even if a table was previously loaded
     * from flash. After successful validation the fresh table is persisted
     * automatically: appended into free flash space, or written from offset
     * zero after erasing the full region when no space remains.
     */
    void calibrate() {
        if (!is_motor_connected()) {
            // No motor identified -> nothing to calibrate.
            TP_LOG("CALIBRATION: motor not connected - skipped");
            return;
        }

        // Same-motor fast path: when a stored parameter profile exists, try
        // to reuse its calibration table (referenced by the profile's Cal-ID
        // = chunk address). A hit goes straight to Ready; a miss falls
        // through to the full physical cycle below (request stays armed).
        if (motor.is_loaded()) {
            TP_LOG_VAL("CALIBRATION: trying stored table, id = ", (long)motor.get_cal_id());
            ptab->begin_load(motor.get_cal_id());
            loaded_from_flash = false;
            calibration_requested = true;   // Auto-continue with a fresh cycle on miss
            calibration_stage = CalStage_CheckMemory;
            cal_cycle_stage = CalSamplingState_Setup;
            time_in_state = 0;
            return;
        }

        TP_LOG("CALIBRATION: Manual full-cycle start requested");  // Log command reception
        calibration_requested = true;                              // Arm the calibration request
        loaded_from_flash = false;                                 // Flash-restored data no longer counts
        calibration_stage = CalStage_Setup;                        // Restart the overall state machine
        cal_cycle_stage = CalSamplingState_Setup;                  // Restart the sampling sub-machine
        time_in_state = 0;                                         // Clear any pending sub-stage countdown
        cal_idx = 0;                                               // Clear step/sample counters
        ang_el_step = 0;                                           // Clear angle step until Reset stage sets it
    }

    /**
     * @brief "load_calibration" command: bind a memory region and restore
     *
     * Receives the flash driver pointer and restarts the non-blocking scan.
     * When a valid table is found at the profile's Cal-ID chunk (FA3E marker,
     * sane sample count, samples strictly monotonic), is_calibrated() starts
     * returning true - unless calibrate() has been triggered meanwhile.
     */
    void load_calibration(FlashEEPROM* mem) {
        ptab->attach_flash(mem);          // Re-bind the memory pointer
        if (is_motor_connected() && motor.is_loaded()) {
            ptab->begin_load(motor.get_cal_id());  // Restart the direct-addressed scan
        }
        loaded_from_flash = false;            // Previous restore no longer counts
        calibration_requested = false;
        calibration_stage = CalStage_CheckMemory;
    }

    /**
     * @brief "dump_calibration" command: persist the RAM table into flash
     *
     * Serializes {FA3E marker, sample count, packed monotonic samples,
     * metadata}. Checks free space first: appends there when the record fits,
     * otherwise erases the whole region and rewrites from offset zero.
     */
    void dump_calibration() {
        if (!is_motor_connected()) return;  // Nothing sensible to stamp without a motor
        ptab->begin_dump();                 // Arm the non-blocking dump engine
        calibration_stage = CalStage_Save;    // tick() services it 8 words per call
    }

    /**
     * @brief Checks if a usable calibration is available
     * @return true if motor is ready for normal operation
     *
     * Returns true either after a freshly completed calibration cycle or when
     * a valid table was restored from flash - unless calibrate() has been
     * triggered and the new cycle has not finished yet.
     */
    bool is_calibrated() const {
        if (calibration_stage == CalStage_Ready) {  // Fresh cycle completed
            return true;                            // Definitely calibrated
        }
        return loaded_from_flash && !calibration_requested;  // Restored from flash and not superseded
    }


    bool is_settling() const {
        return calibration_stage == CalStage_CheckMemory;  // Check if in Ready stage
    }

    bool is_idle() const {
        return calibration_stage == CalStage_Reset;  // Check if in Ready stage
    }

    /**
     * @brief Gets corrected position using calibration table
     * @param pos Raw encoder position
     * @param mech_angle Output: mechanical angle
     */
    uint16_t get_el_angle_correction(uint32_t pos) const {
        return ptab->correct_el_angle(pos >> 16);  // Apply calibration correction
    }

    uint16_t get_position_correction(uint16_t pos) const {
        return ptab->correct_position(pos);  // Apply calibration correction
    }

private:
    uint16_t frequency;    // Update frequency in ticks per second
    int32_t position = 0;  // Current encoder position reading

    CalStage calibration_stage = CalStage_CheckMemory;          // Current stage of overall calibration process
    CalSamplingState cal_cycle_stage = CalSamplingState_Setup;  // Current sub-stage of calibration cycle

    uint16_t angle_el = 0;     // Electrical angle of motor (0..65535)
    uint16_t ang_el_step = 0;  // Angle step size for current calibration stage

    size_t cal_idx = 0;           // Index for counting steps during calibration
    int32_t oversampled_pos = 0;  // Accumulator for averaging positions during oversampling
    int32_t base = 0;

    size_t time_in_state = 0;  // Counter for ticks remaining in current sub-stage

    int32_t direction = 0;  // Rotation direction (1 forward, -1 backward)
    int32_t speed = 1;      // Speed in angle increments per tick
    size_t settling_time;   // Settling time in ticks

    int32_t init_pos = 0;         // Position recorded at start of calibration step
    int32_t temp_pos = 0;         // Temporary position for measuring movement
    int32_t dif_max = INT32_MIN;  // Maximum step difference for consistency checks
    int32_t dif_min = INT32_MAX;  // Minimum step difference for consistency checks

    CalibrationTable* ptab;      // Active table: always the Motor's embedded instance.
    Motor& motor;                ///< Whole motor object handed over by the board layer.

    uint16_t el_step_idx = 0;    // Target electrical angle after rotation

    bool calibration_requested = false;  // True between calibrate() call and cycle completion
    bool loaded_from_flash = false;      // True when a valid table was restored from flash
    bool boot_resolved_ = false;         // True once the boot-time load attempt produced an outcome

    /**
     * @brief Decrements a counter until it reaches zero
     * @param counter Pointer to counter variable
     * @return false while counter > 0, true when counter reaches 0
     */
    static bool iter(size_t* counter) {
        if (*counter > 0) {
            *counter -= 1;  // Decrement counter
            return false;   // Still have ticks left
        }
        return true;  // Counter reached zero
    }

    /**
     * @brief Oversamples encoder position for stable reading
     * @param position Current encoder position
     * @param iter Pointer to iteration counter
     * @param integral Pointer to accumulator
     * @return true when oversampling complete, false otherwise
     */
    bool oversampling(int32_t position, size_t& iter, int32_t& integral, int32_t& base) {
        if (iter == ANGLE_CAL_CAL_OVERSEMPLING) {
            base = position;  // Save the absolute position (including full turns)
            integral = 0;     // Strictly clear the accumulator!
        }

        // (uint16_t) cast solves the 65535<->0 jitter problem
        // (int16_t) cast gives the deviation sign without losing the macro-offset kept in base
        int16_t diff = (int16_t)(uint16_t)(position - base);
        integral += diff;

        if (iter > 0) {
            iter -= 1;  // Decrement iteration counter
        }

        if (iter == 0) {
            // Add the averaged delta back onto the absolute base
            integral = base + (integral / (int32_t)ANGLE_CAL_CAL_OVERSEMPLING);
            return true;
        }
        return false;
    }

    /**
     * @brief Rotates motor at specified speed
     * @param increment Angle increment per tick
     */
    void move_at_speed(int32_t increment) {
        uint16_t new_angle = (uint16_t)(angle_el + (uint16_t)increment);  // Calculate new angle (wraps automatically)
        angle_el = new_angle;                                             // Update electrical angle
    }

    /**
     * @brief Calculates speed in angle increments per tick
     * @param frequency Update frequency in Hz
     * @param speed_us Speed in microseconds
     * @return Speed in angle increments per tick
     */
    static int32_t calculate_speed(uint16_t frequency, size_t speed_us) {
        return (int32_t)((frequency * speed_us) / 1000000u);  // Convert to ticks
    }

    /**
     * @brief Calculates settling time in ticks
     * @param frequency Update frequency in Hz
     * @param settling_us Settling time in microseconds
     * @return Settling time in ticks
     */
    static size_t calculate_settling_time(uint16_t frequency, size_t settling_us) {
        return (size_t)((frequency * settling_us) / 1000000u);  // Convert to ticks
    }

    /**
     * @brief Executes one complete sampling cycle
     * @param steps Number of angle steps to move
     * @return Stable averaged position or INT32_MIN if not yet complete
     *
     * Cycle stages: Setup -> Rotating -> Waiting -> Sampling
     */
    int32_t run_sampling_cycle(uint16_t steps) {
        switch (cal_cycle_stage) {
            case CalSamplingState_Setup:
                // Prepare for new calibration cycle
                oversampled_pos = 0;                          // Reset oversampling accumulator
                cal_cycle_stage = CalSamplingState_Rotating;  // Move to Rotating state
                time_in_state =
                    (steps == 0) ? 0 : (size_t)steps / (size_t)abs(speed == 0 ? 1 : speed);  // Calculate rotation duration
                el_step_idx =
                    (uint16_t)(angle_el + (uint16_t)((int32_t)steps * (int32_t)(speed < 0 ? -1 : 1)));  // Calculate target angle
                return INT32_MIN;                                                                       // Not finished yet

            case CalSamplingState_Rotating:
                // Rotate motor for specified time
                move_at_speed(speed);  // Advance angle by speed each tick

                if (iter(&time_in_state)) {
                    // Rotation complete, move to Waiting state
                    cal_cycle_stage = CalSamplingState_Waiting;  // Move to Waiting state
                    time_in_state = settling_time;               // Set settling time duration
                    angle_el = el_step_idx;                      // Fine-tune angle to target
                }
                return INT32_MIN;  // Still rotating

            case CalSamplingState_Waiting:
                // Motor stationary, waiting for settling
                if (iter(&time_in_state)) {
                    // Settling complete, move to Sampling state
                    cal_cycle_stage = CalSamplingState_Sampling;  // Move to Sampling state
                    time_in_state = ANGLE_CAL_CAL_OVERSEMPLING;   // Set oversampling count
                }
                return INT32_MIN;  // Still waiting

            case CalSamplingState_Sampling:
                // Oversample encoder position for stable reading
                if (oversampling(position, time_in_state, oversampled_pos, base)) {
                    // Oversampling complete
                    cal_cycle_stage = CalSamplingState_Setup;  // Reset to Setup for next cycle
                    return oversampled_pos;                    // Return averaged stable position
                }
                return INT32_MIN;  // Still sampling
        }
        return INT32_MIN;  // Default return
    }
};