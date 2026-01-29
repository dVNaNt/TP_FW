#pragma once

#include "../../common.h"
#include "calibration_table.h"

/// Represents the current stage of the calibration process.
typedef enum {
    /// Test the motor's ability to respond linearly and consistently by performing a few test steps.
    CalStage_Reset = 0, // Initial setup stage for calibration
    CalStage_Setup = 1, // Stage where the motor is allowed to settle before starting calibration steps
    CalStage_Pass0 = 2,
    CalStage_Pass1 = 3, // First pass of the calibration
    CalStage_Pass2 = 4, // Second pass of the calibration
    CalStage_Check = 5, // State for verifying the calibration
    CalStage_Ready = 6, // Calibration is complete and ready
    CalStage_Error = 7, // An error occurred during calibration
} CalStage;

/// Represents the state within each calibration cycle quarter:
/// (Setup -> Rotating -> Waiting -> Sampling)
typedef enum {
    /// Initial setup state before starting rotation and sampling.
    CalSamplingState_Setup,
    /// Motor is rotating during a calibration step.
    CalSamplingState_Rotating,
    /// Motor is waiting (stationary) for a specified settling time.
    CalSamplingState_Waiting,
    /// Motor is sampling (oversampling the encoder position) for averaging.
    CalSamplingState_Sampling,
} CalSamplingState;

/// The main driver struct for the motor, holding all the state required for operation and calibration.
typedef struct {
    uint16_t frequency;    // Update frequency (ticks per second)
    int32_t position; // Current encoder position reading

    CalStage calibration_stage; // Current stage of the overall calibration process
    CalSamplingState cal_cycle_stage; // Current sub-stage of the calibration cycle (Setup, Rotating, Waiting, Sampling)

    uint16_t angle_el;    // Electrical angle of the motor (0..65535), used to control phase
    uint16_t ang_el_step; // Defines how many steps to move in one sub-cycle

    size_t cal_idx; // Index for counting steps during calibration cycles
    int32_t oversampled_pos; // Accumulator for averaging positions during oversampling (Sampling stage)

    size_t time_in_state; // Counter for how many ticks remain in the current calibration sub-stage

    int32_t direction; // Current rotation direction (1 for forward, -1 for backward)
    int32_t speed;     // Speed (steps per tick) during calibration
    size_t settling_time; // Settling time in milliseconds

    int32_t init_pos; // Position recorded at the start of a calibration step
    int32_t temp_pos; // Temporary position for measuring movement increments
    int32_t dif_max;  // Maximum difference in step measurement for consistency checks
    int32_t dif_min;  // Minimum difference in step measurement for consistency checks

    CalibrationTable cal_table;
    uint16_t el_step_idx;
} AngleCalibrator;

enum {
    ANGLE_CAL_CAL_SETTLING_TIME_US = 25000, // Settling time in milliseconds
    ANGLE_CAL_CAL_SPEED_US = 2500, // Speed in angle increments per millisecond

    ANGLE_CAL_CAL_OVERSEMPLING = 100, // Number of samples per oversampling period for averaging
    ANGLE_CAL_CAL_FIRST_STEP_USTEPS = 16,
    ANGLE_CAL_CAL_POINTS_PER_360EL = 4,
};

/// Inline method to decrement a counter until it reaches zero.
///
/// Returns:
/// * `false` while counter > 0
/// * `true` when the counter reaches 0
static inline bool angle_cal_iter(size_t* counter) {
    if (*counter > 0) {
        *counter -= 1; // Decrement the counter
        return false; // Still have ticks left
    }
    return true; // Counter reached zero
}

/// Inline method for oversampling the encoder position data.
static inline bool angle_cal_oversampling(int32_t position, size_t* iter, int32_t* integral) {
    *integral += position; // Add current position to the integral
    if (*iter > 0) {
        *iter -= 1; // Decrease iteration count
    }
    if (*iter == 0) {
        // If no more iterations left, finalize averaging
        *integral = *integral / ANGLE_CAL_CAL_OVERSEMPLING; // Compute the average
        return true; // Oversampling complete
    }
    return false; // Still sampling
}

/// Inline method for rotating the motor at a given increment per tick.
static inline void angle_cal_move_at_speed(AngleCalibrator* self, int32_t increment) {
    uint16_t new_angle = (uint16_t)(self->angle_el + (uint16_t)increment); // Wrap around if overflow
    self->angle_el = new_angle; // Update the motor's electrical angle
}

/// Calculate speed in ticks per millisecond.
static inline int32_t angle_cal_calculate_speed(uint16_t frequency, size_t speed_us) {
    return (int32_t)((frequency * speed_us) / 1000000u);
}

/// Calculate settling time in ticks based on frequency and milliseconds.
static inline size_t angle_cal_calculate_settling_time(uint16_t frequency, size_t settling_us) {
    return (size_t)((frequency * settling_us) / 1000000u);
}

static inline AngleCalibrator angle_calibrator_new(uint16_t frequency) {
    size_t settling_time = angle_cal_calculate_settling_time(frequency, ANGLE_CAL_CAL_SETTLING_TIME_US);
    AngleCalibrator a;
    a.frequency = frequency;
    a.position = 0;
    a.calibration_stage = CalStage_Setup;
    a.cal_cycle_stage = CalSamplingState_Setup;
    a.angle_el = 0;
    a.cal_idx = 0;
    a.oversampled_pos = 0;
    a.time_in_state = 0;
    a.ang_el_step = 0;
    a.direction = 0;
    a.speed = 1;
    a.settling_time = settling_time;
    a.init_pos = 0;
    a.temp_pos = 0;
    a.dif_max = INT32_MIN;
    a.dif_min = INT32_MAX;
    a.cal_table = calibration_table_new();
    a.el_step_idx = 0;
    return a;
}

/// Executes one complete cycle of moving, waiting, and sampling for calibration.
static inline int32_t angle_cal_run_sampling_cycle(AngleCalibrator* self, uint16_t steps) {
    switch (self->cal_cycle_stage) {
        case CalSamplingState_Setup:
            // Prepare for a new calibration cycle
            self->oversampled_pos = 0; // Reset oversampling accumulator
            self->cal_cycle_stage = CalSamplingState_Rotating; // Next state: Rotating
            self->time_in_state = (steps == 0) ? 0 : (size_t)steps / (size_t)abs(self->speed == 0 ? 1 : self->speed); // Calculate how long to rotate
            self->el_step_idx = (uint16_t)(self->angle_el + (uint16_t)((int32_t)steps * (int32_t)(self->speed < 0 ? -1 : 1)));
            return INT32_MIN; // Not finished yet

        case CalSamplingState_Rotating:
            // Rotate the motor for the specified time
            angle_cal_move_at_speed(self, self->speed); // Advance angle by 'speed' each tick

            if (angle_cal_iter(&self->time_in_state)) {
                // Once done rotating, move to Waiting state
                self->cal_cycle_stage = CalSamplingState_Waiting;
                self->time_in_state = self->settling_time; // Settle time
                self->angle_el = self->el_step_idx; // Fine tune angle to
            }
            return INT32_MIN; // Still not finished

        case CalSamplingState_Waiting:
            // Motor is now stationary, waiting
            if (angle_cal_iter(&self->time_in_state)) {
                // After waiting, move to Sampling state
                self->cal_cycle_stage = CalSamplingState_Sampling;
                self->time_in_state = ANGLE_CAL_CAL_OVERSEMPLING; // Perform oversampling
            }
            return INT32_MIN; // Still waiting

        case CalSamplingState_Sampling:
            // Oversample the encoder position to get a stable reading
            if (angle_cal_oversampling(self->position, &self->time_in_state, &self->oversampled_pos)) {
                // Once oversampling is complete:
                self->cal_cycle_stage = CalSamplingState_Setup; // Reset cycle stage
                return self->oversampled_pos; // Return the averaged stable position
            }
            return INT32_MIN; // Still sampling
    }
    return INT32_MIN;
}

/// High-level calibration method.
static inline uint16_t angle_calibrator_tick(AngleCalibrator* self, int32_t encoder_pos) {
    self->position = encoder_pos; // Update the internal position from the sensor
    int32_t stable_pos = angle_cal_run_sampling_cycle(self, self->ang_el_step); // Perform a calibration cycle and get stable position

    if (stable_pos != INT32_MIN) {
        switch (self->calibration_stage) {
            case CalStage_Setup:
                self->cal_idx = 10; // Arbitrary index setting for demonstration
                self->calibration_stage = CalStage_Reset;
                self->speed = angle_cal_calculate_speed(self->frequency, ANGLE_CAL_CAL_SPEED_US);
                return self->angle_el;

            case CalStage_Reset:
                if (angle_cal_iter(&self->cal_idx)) {
                    self->init_pos = stable_pos;
                    self->temp_pos = stable_pos;
                    self->dif_max = INT32_MIN;
                    self->dif_min = INT32_MAX;
                    self->ang_el_step = (uint16_t)(UINT16_MAX / ANGLE_CAL_CAL_FIRST_STEP_USTEPS);
                    self->cal_idx = ANGLE_CAL_CAL_FIRST_STEP_USTEPS;
                    self->calibration_stage = CalStage_Pass0;
                    TUNEPULSE_LOG_INFO("CALIBRATION: Test single pole motion");
                }
                break;

            case CalStage_Pass0: {
                int32_t dif = self->temp_pos - stable_pos;
                self->temp_pos = stable_pos;
                if (dif < self->dif_min) self->dif_min = dif;
                if (dif > self->dif_max) self->dif_max = dif;

                if (angle_cal_iter(&self->cal_idx)) {
                    int32_t travel = self->init_pos - self->temp_pos; // Total travel during test

                    int32_t direction = (travel >= 0) ? 1 : -1;
                    self->direction = direction;

                    int32_t avg_step = (travel * direction) / ANGLE_CAL_CAL_FIRST_STEP_USTEPS;
                    int32_t deviation = self->dif_max - self->dif_min;

                    if (avg_step < deviation) {
                        TUNEPULSE_LOG_ERROR("CALIBRATION: Too much deviation while moving");
                        self->calibration_stage = CalStage_Error;
                        return self->angle_el;
                    }

                    TUNEPULSE_LOG_INFO("CALIBRATION: Detected motion direction: %ld", (long)self->direction);
                    self->calibration_stage = CalStage_Pass1;
                    self->ang_el_step = (uint16_t)(UINT16_MAX / ANGLE_CAL_CAL_POINTS_PER_360EL);
                    self->speed = (int32_t)(-self->speed * self->direction);
                    self->cal_idx = 0;
                    self->init_pos = stable_pos;
                    calibration_table_reset(&self->cal_table, ANGLE_CAL_CAL_POINTS_PER_360EL);
                    TUNEPULSE_LOG_INFO("CALIBRATION: Full rotation in positive direction with sampling");
                }
                break;
            }

            case CalStage_Pass1: {
                int32_t avg_step = (stable_pos - self->init_pos) / (int32_t)(self->cal_idx + 1);
                if (stable_pos - self->init_pos > (int32_t)UINT16_MAX + (avg_step / 3)) {
                    self->calibration_stage = CalStage_Pass2;
                    TUNEPULSE_LOG_INFO("CALIBRATION: Full rotation in negative direction with sampling");
                    self->speed = -self->speed;
                    return self->angle_el;
                }
                calibration_table_fill_first(&self->cal_table, self->cal_idx, (uint16_t)stable_pos);
                self->cal_idx += 1;
                break;
            }

            case CalStage_Pass2:
                if (self->cal_idx > 0) {
                    self->cal_idx -= 1;
                }
                calibration_table_fill_second(&self->cal_table, self->cal_idx, (uint16_t)stable_pos);

                if (self->cal_idx == 0) {
                    self->calibration_stage = CalStage_Check;
                    TUNEPULSE_LOG_INFO("CALIBRATION: Finished. Next => NORMAL RUN");
                    self->angle_el = 0;
                }
                break;

            case CalStage_Check:
                calibration_table_check(&self->cal_table);
                self->calibration_stage = CalStage_Ready;
                break;
            case CalStage_Error:
            case CalStage_Ready:
            default:
                break;
        }
    }
    return self->angle_el;
}

/// Check if calibration is complete (motor is ready for normal operation).
static inline bool angle_calibrator_is_ready(const AngleCalibrator* self) {
    return self->calibration_stage == CalStage_Ready;
}

static inline void angle_calibrator_get_correction(const AngleCalibrator* self, uint16_t pos, uint16_t* corrected_pos, uint16_t* mech_angle) {
    calibration_table_correct_pos(&self->cal_table, pos, corrected_pos, mech_angle);
}


