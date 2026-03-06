#pragma once

// Implements the motor calibration module, handling calibration data collection,
// normalization, and position correction for the motor control system.
//
// Key Features:
// - Collects and stores encoder data during motor rotation for calibration.
// - Removes hysteresis effects through bidirectional data collection.
// - Determines calibration offset and start index based on minimal deviation.
// - Validates calibration data for consistency and accuracy.
// - Corrects motor position readings using the calibrated data.
//
// Licensed under the Apache License, Version 2.0
// Copyright 2024 Anton Khrustalev, creapunk.com

#include "../../common.h"

#ifndef CAL_TABLE_CAPACITY
#define CAL_TABLE_CAPACITY 200
#endif

/// The main driver struct for the motor, holding all the state required for operation and calibration.
typedef struct {
    /// Stores sampled encoder data across a full calibration cycle.
    uint16_t cal_table[CAL_TABLE_CAPACITY];

    /// Current number of valid points collected during calibration.
    size_t cal_size;

    /// Divider representing how many steps form one electrical period.
    size_t el_angle_div;

    /// Index of the first zero-position value after normalization.
    size_t offst_idx;

    /// Offset to align calibration data such that "zero" aligns to the correct mechanical angle.
    uint16_t offst_val;

    /// Maximum absolute deviation between ideal and actual calibration values.
    uint16_t max_deviation;

    /// Temporary index used during calibration data collection.
    size_t temp_idx;
} CalibrationTable;

/// Creates a new `CalibrationTable` with default values.
static inline CalibrationTable calibration_table_new(void) {
    CalibrationTable t;
    t.offst_val = 0;
    t.offst_idx = 0;
    t.cal_size = 0;
    for (size_t i = 0; i < CAL_TABLE_CAPACITY; ++i) {
        t.cal_table[i] = 0;
    }
    t.el_angle_div = 1;
    t.max_deviation = 0;
    t.temp_idx = 0;
    return t;
}

/// Resets the calibration table for a new calibration process.
/// `el_angle_div` sets how many steps form one electrical period.
static inline void calibration_table_reset(CalibrationTable* self, uint16_t el_angle_div) {
    self->offst_idx = 0; // Reset start index
    self->el_angle_div = (size_t)el_angle_div; // Convert el_angle_div to usize for indexing
    self->cal_size = 0; // Clear the size (no valid data points yet)
    self->offst_val = UINT16_MAX; // Initialize offset to max, so we can find the minimum value later
    self->max_deviation = 0; // Reset maximum deviation
    self->temp_idx = 0; // Reset temporary index
}

/// Stores the first round of calibration data at the given index.
/// This collects raw samples as the motor is rotated in one direction.
static inline bool calibration_table_fill_first(CalibrationTable* self, size_t idx, uint16_t val) {
    // Check if the index is within bounds and sequential
    if (idx < CAL_TABLE_CAPACITY && (idx >= self->temp_idx ? idx - self->temp_idx : self->temp_idx - idx) <= 1) {
        self->temp_idx = idx; // Update temporary index
        self->cal_table[idx] = val; // Store the sample value
        if (self->cal_size < idx + 1) {
            self->cal_size = idx + 1; // Update cal_size to reflect the maximum filled index
        }
        return true; // Indicate successful storage
    }

    return false; // Indicate failure
}

/// Performs a second pass of filling data in the opposite direction to remove hysteresis.
/// This method averages the forward and backward measurements to center the hysteresis loop.
static inline bool calibration_table_fill_second(CalibrationTable* self, size_t idx, uint16_t val) {
    // Check if the index is within bounds
    if (idx < CAL_TABLE_CAPACITY) {
        // Calculate the signed difference to handle potential wraparound (values near 0 or max range).
        int16_t dif = (int16_t)(val - self->cal_table[idx]);

        // Update the table with the midpoint of the hysteresis range.
        uint16_t mid = (uint16_t)(self->cal_table[idx] + (uint16_t)(dif / 2));
        self->cal_table[idx] = mid; // Store the averaged value

        // Every `el_angle_div` steps, check if this is the minimal offset position.
        if (((idx % self->el_angle_div) == 0) && (mid <= self->offst_val)) {
            self->offst_val = mid; // Update offset to the lowest encountered value
            self->offst_idx = idx; // Update start index
        }

        // If the current index is the last filled index, record the max deviation
        if (idx + 1 == self->cal_size) {
            self->max_deviation = mid; // Update maximum deviation
            return true; // Indicate successful completion
        }

        // Calculate the difference between current max deviation and the new value
        int16_t diff = (int16_t)(self->max_deviation - mid);
        self->max_deviation = mid; // Update max deviation to the new value

        // If the deviation exceeds the threshold, indicate successful completion
        if (diff > 10) {
            return true;
        }
    }
    return false; // Indicate failure
}

/// Retrieves a calibration value by an index relative to the `start_idx`.
/// The resulting index is wrapped around `cal_size` to handle modulo arithmetic over a circular table.
static inline uint16_t calibration_table_get_val_by_idx(const CalibrationTable* self, size_t index) {
    size_t actual_idx = (self->offst_idx + index) % self->cal_size; // Compute the actual index
    return self->cal_table[actual_idx]; // Return the table value at the computed position
}

/// Computes an ideal value for the given index `i` within a range.
/// This function assumes a linear increase from 0 to `u16::MAX` across `range` points.
static inline uint16_t calibration_table_get_ideal(size_t i, size_t range) {
    const size_t CAL_VAL_RANGE = UINT16_MAX; // Range of `u16` values (0 to 65535)
    return (uint16_t)((CAL_VAL_RANGE * i) / range); // Calculate ideal value based on linear progression
}

/// Calculates the maximum absolute deviation from the ideal linear progression.
/// Ideal value for each element `i` is `(u16::MAX * i) / cal_size`.
/// Returns the maximum found deviation.
static inline uint16_t calibration_table_abs_deviation(uint16_t val, size_t idx, size_t size) {
    // Compute the ideal value at index `i`.
    uint16_t ideal = calibration_table_get_ideal(idx, size);

    // Compute the deviation as the absolute difference from the measured value.
    uint16_t deviation = (uint16_t)abs((int32_t)ideal - (int32_t)val);

    return deviation; // Return the calculated deviation
}

/// Validates the calibration data by checking the consistency of the table.
/// Ensures that `cal_size` matches an integral number of poles (el_angle_div) and that
/// deviations from the ideal linear distribution are acceptable.
/// # TODO: make it without for loop
static inline bool calibration_table_check(CalibrationTable* self) {
    if (self->cal_size == 0) {
        return false;
    }
    // Calculate the average step size based on the total range and number of samples
    uint16_t avg_step = (uint16_t)(UINT16_MAX / (uint16_t)self->cal_size);
    self->max_deviation = 0; // Reset the maximum deviation counter

    // Iterate through each calibration point to compute deviations
    for (size_t i = 0; i < self->cal_size; ++i) {
        uint16_t val = (uint16_t)(self->cal_table[i] - self->offst_val); // Shift values so offset becomes zero
        size_t corrected_idx = ((self->cal_size + i) - self->offst_idx) % self->cal_size; // Compute corrected index
        uint16_t deviation = calibration_table_abs_deviation(val, corrected_idx, self->cal_size); // Calculate deviation

        self->cal_table[i] = val; // Update calibration table with normalized value
        if (deviation > self->max_deviation) {
            self->max_deviation = deviation; // Update maximum deviation if necessary
        }

        // Check if the deviation exceeds the average step size
        if (deviation >= avg_step) {
            return false; // Indicate validation failure
        }
    }

    return true; // Indicate validation success
}

/// Interpolates a position `c1` within a reference segment defined by points (a1, a2) and (b1, b2).
/// `a1` and `b1` form one pair (real and ideal), and `a2` and `b2` form the second pair.
/// Returns the ideal interpolated value corresponding to `c1`.
static inline uint16_t calibration_table_interpolate(uint16_t a1, uint16_t a2, uint16_t b1, uint16_t b2, uint16_t c1) {
    // Calculate the reference range in the real domain
    uint16_t ref_range = (uint16_t)(b1 - a1);

    // Calculate the target (ideal) range
    uint16_t trgt_range = (uint16_t)(b2 - a2);

    // Determine how far along C is within the A-B segment in real values
    uint16_t c1_ofst = (uint16_t)(c1 - a1);

    // Interpolate this fraction into the ideal domain
    uint32_t c2_ofst = ((uint32_t)c1_ofst * (uint32_t)trgt_range) / (uint32_t)ref_range;

    // Return the interpolated ideal value
    return (uint16_t)((uint32_t)a2 + c2_ofst);
}

/// Corrects a given position using the calibration table.
/// Given an actual encoder `position`, it accounts for the offset and searches near the expected index.
/// Uses a small loop to find the segment where real_pos transitions from positive to negative difference,
/// then interpolates the ideal position to achieve a corrected angle.
static inline void calibration_table_correct_pos(const CalibrationTable* self, uint16_t position, uint16_t* corrected_angle, uint16_t* mech_el_angle) {
    // Align the position so that zero aligns with the table's zero-offset point.
    uint16_t real_pos = (uint16_t)(position - self->offst_val);

    // Estimate a starting index by scaling `real_pos` down.
    size_t idx = ((size_t)(real_pos - self->max_deviation) * self->cal_size) >> 16;

    uint16_t result = UINT16_MAX; // Initialize result to max as a fallback

    // Starting comparison points
    uint16_t cal_pos1 = calibration_table_get_val_by_idx(self, idx); // Retrieve calibration value at current index
    uint16_t idl_pos1 = calibration_table_get_ideal(idx, self->cal_size); // Retrieve ideal value at current index

    // Iterate up to 8 steps to find where we cross from positive diff to negative diff.
    for (int iter = 0; iter < 8; ++iter) {
        idx = (idx + 1) % self->cal_size; // Move to the next index
        uint16_t cal_pos2 = calibration_table_get_val_by_idx(self, idx); // Retrieve calibration value at new index
        uint16_t idl_pos2 = calibration_table_get_ideal(idx, self->cal_size); // Retrieve ideal value at new index

        int16_t diff1 = (int16_t)(real_pos - cal_pos1); // Difference at previous index
        int16_t diff2 = (int16_t)(real_pos - cal_pos2); // Difference at current index

        // Once we find a boundary where diff changes sign (diff1 >= 0, diff2 < 0),
        // we interpolate the exact ideal position within that segment.
        if ((diff1 >= 0) && (diff2 < 0)) {
            result = calibration_table_interpolate(cal_pos1, idl_pos1, cal_pos2, idl_pos2, real_pos); // Perform interpolation
            break; // Exit the loop after interpolation
        }

        // Move to the next segment
        cal_pos1 = cal_pos2; // Update previous calibration position
        idl_pos1 = idl_pos2; // Update previous ideal position
    }

    // Re-apply offset to return the corrected angle to the global coordinate system.
    *corrected_angle = (uint16_t)(result + self->offst_val); // Adjust corrected angle with offset

    // Compute the mechanical angle mapped into one electrical period.
    *mech_el_angle = (uint16_t)(((size_t)result * self->cal_size) / self->el_angle_div); // Calculate mechanical to electrical angle
}


