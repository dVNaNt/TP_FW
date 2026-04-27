// /**
//  * @file calibration_table.h
//  * @brief Motor calibration data collection and position correction module
//  *
//  * Implements the motor calibration module, handling calibration data collection,
//  * normalization, and position correction for the motor control system.
//  *
//  * Key Features:
//  * - Collects and stores encoder data during motor rotation for calibration.
//  * - Removes hysteresis effects through bidirectional data collection.
//  * - Determines calibration offset and start index based on minimal deviation.
//  * - Validates calibration data for consistency and accuracy.
//  * - Corrects motor position readings using the calibrated data.
//  * - Features an ECC_TABLE_SIZE-point inverse lookup table for magnetic eccentricity compensation.
//  *
//  * Algorithm:
//  * 1. First pass: Collect encoder samples during forward rotation.
//  * 2. Second pass: Collect samples during backward rotation and average with first pass
//  *    to eliminate hysteresis effects.
//  * 3. Normalization: Find minimum value and use it as offset, align data to zero.
//  * 4. Validation: Check that deviations from ideal linear progression are within bounds.
//  * 5. ECC build: Build a compact inverse table of raw-angle eccentricity error.
//  * 6. Correction: Use fast interpolation to output stable corrected angles.
//  *
//  * Copyright 2026, creapunk
//  * Licensed under the Apache License, Version 2.0
//  */

// #pragma once  // Prevent multiple compiler inclusions of this header file

// #include "../../common.h"  // Include common project definitions, utilities, and macros

// #ifndef CAL_TABLE_CAPACITY      // Check if CAL_TABLE_CAPACITY is not defined externally
// #define CAL_TABLE_CAPACITY 200  // Maximum number of calibration points allocated in memory
// #endif                          // End of CAL_TABLE_CAPACITY definition block

// #ifndef ECC_TABLE_SIZE     // Check if ECC_TABLE_SIZE is not defined externally
// #define ECC_TABLE_SIZE 32  // Number of points for the eccentricity compensation table
// #endif                     // End of ECC_TABLE_SIZE definition block

// /**
//  * @brief Calibration table for motor encoder position correction
//  *
//  * Stores calibration data collected during motor rotation and provides
//  * methods for data collection, validation, and position correction.
//  */
// struct CalibrationTable {  // Begin declaration of the CalibrationTable structure
// public:                    // Make the following methods public for external module access
//     /**
//      * @brief Constructs a new CalibrationTable with default values
//      * @return Initialized CalibrationTable instance
//      */
//     static CalibrationTable create() {                     // Factory method to instantiate and initialize the table
//         CalibrationTable t;                                // Allocate a new table instance
//         t.offst_val = 0;                                   // Initialize angular offset value to zero
//         t.offst_idx = 0;                                   // Initialize physical offset index to zero
//         t.cal_size = 0;                                    // Set initial collected calibration points counter to zero
//         for (size_t i = 0; i < CAL_TABLE_CAPACITY; ++i) {  // Loop through all table entries
//             t.cal_table[i] = 0;                            // Clear all calibration table entries systematically
//         }  // End of initialization loop
//         t.el_angle_div = 1;   // Default electrical angle divider set to 1 to prevent division by zero
//         t.max_deviation = 0;  // Reset max deviation tracker to zero
//         t.temp_idx = 0;       // Reset temporary sequence tracker index

//         for (size_t i = 0; i < ECC_TABLE_SIZE; ++i) {  // Loop through eccentricity table entries
//             t.ecc_table[i] = 0;                        // Clear the eccentricity correction table to zero
//         }  // End of eccentricity table initialization loop

//         return t;  // Return the fully initialized instance
//     }  // End of create() method

//     /**
//      * @brief Resets the calibration table for a new calibration process
//      * @param el_angle_div Number of steps forming one electrical period
//      */
//     void reset(uint16_t el_angle_div) {                                       // Prepare table for a fresh calibration run
//         this->offst_idx = 0;                                                  // Reset start index to beginning
//         this->el_angle_div = (el_angle_div == 0) ? 1 : (size_t)el_angle_div;  // Store electrical angle divider safely (prevent 0)
//         this->cal_size = 0;                                                   // Clear the size (no valid data points yet)
//         this->offst_val = UINT16_MAX;  // Initialize offset to max to find minimum during search later
//         this->max_deviation = 0;       // Reset maximum deviation counter
//         this->temp_idx = 0;            // Reset temporary index for sequential filling validation

//         for (size_t i = 0; i < ECC_TABLE_SIZE; ++i) {  // Clear stale eccentricity data from previous calibration
//             this->ecc_table[i] = 0;                    // Prevent old ECC compensation from being used after reset
//         }  // End of ECC table reset loop
//     }  // End of reset() method

//     /**
//      * @brief Stores first round of calibration data (forward rotation)
//      * @param idx Index in calibration table
//      * @param val Encoder value to store
//      * @return true if storage successful, false if index out of bounds
//      *
//      * Collects raw samples as the motor rotates in one direction.
//      */
//     bool fill_first(size_t idx, uint16_t val) {  // Insert forward rotation samples
//         // Check if the index is within bounds and sequentially incremented
//         if (idx < CAL_TABLE_CAPACITY &&
//             (idx >= temp_idx ? idx - temp_idx : temp_idx - idx) <= 1) {  // Bounds and absolute difference check
//             temp_idx = idx;            // Update temporary index for sequential check on next call
//             cal_table[idx] = val;      // Store the sample value at target index
//             if (cal_size < idx + 1) {  // Verify if we are expanding the recorded size
//                 cal_size = idx + 1;    // Update cal_size to reflect maximum filled index
//             }  // End of size update condition
//             return true;  // Indicate successful storage to the caller
//         }  // End of bounds and sequence validation
//         return false;  // Index out of bounds or non-sequential, reject sample
//     }  // End of fill_first() method

//     /**
//      * @brief Performs second pass filling (backward rotation) to remove hysteresis
//      * @param idx Index in calibration table
//      * @param val Encoder value from backward rotation
//      * @return true if this is the last point, false otherwise
//      *
//      * Averages forward and backward measurements to center the hysteresis loop.
//      * Also identifies the minimum offset value for normalization.
//      */
//     bool fill_second(size_t idx, uint16_t val) {  // Insert backward rotation samples and average
//         if (idx < CAL_TABLE_CAPACITY) {           // Check if the index is strictly within array bounds
//             // Calculate signed difference to handle wraparound near 0 or max range perfectly
//             int16_t dif = (int16_t)(val - cal_table[idx]);  // Signed cast evaluates shortest modular angular path

//             // Update table with midpoint of hysteresis range (average of forward/backward)
//             uint16_t mid = (uint16_t)(cal_table[idx] + (uint16_t)(dif / 2));  // Calculate central point handling wrap-around
//             cal_table[idx] = mid;                                             // Store the averaged hysteresis-free value

//             // Every el_angle_div steps, check if this is the minimal offset position
//             if (((idx % el_angle_div) == 0) &&
//                 (mid <= offst_val)) {  // Modulo is safe here because el_angle_div is verified in reset()
//                 offst_val = mid;       // Update global offset to the lowest encountered value
//                 offst_idx = idx;       // Update start index to point to this exact lowest position
//             }  // End of offset tracking block

//             // If current index is the last filled index, record max deviation
//             if (idx + 1 == cal_size) {  // Detect if this is the final element of the loop
//                 max_deviation = mid;    // Store final maximum deviation base value
//                 return true;            // Indicate completion of second pass safely
//             }  // End of final element check

//             // Calculate difference between previous tracked value and new value
//             int16_t diff = (int16_t)(max_deviation - mid);  // Tracks sudden jumps or steps in backward progression
//             max_deviation = mid;                            // Update deviation tracker to new current value

//             // If deviation exceeds threshold, indicate completion (detects noise/slips)
//             if (diff > 10) {  // Large step difference triggers early exit
//                 return true;  // Significant deviation detected, finalize pass
//             }  // End of threshold verification
//         }  // End of bounds check
//         return false;  // Not yet complete or index was out of bounds
//     }  // End of fill_second() method

//     /**
//      * @brief Validates calibration data consistency
//      * @return true if calibration data is valid, false otherwise
//      *
//      * Checks that deviations from ideal linear distribution are acceptable.
//      * Normalizes all values by subtracting the offset.
//      */
//     bool check() {            // Run final verification and normalization on collected data
//         if (cal_size == 0) {  // Safety check to prevent divide-by-zero operations below
//             return false;     // No data to validate, return failure
//         }  // End of size check

//         // Calculate average step size based on total range and number of samples
//         uint16_t avg_step = (uint16_t)(UINT16_MAX / (uint16_t)cal_size);  // Theoretical step between array elements
//         max_deviation = 0;                                                // Reset maximum deviation counter for the check phase

//         TP_LOG_VAL("Offset ", offst_val);      // Output the discovered offset value
//         TP_LOG_VAL("Offset idx ", offst_idx);  // Output the starting index of the offset
//         TP_LOG_ARRAY("CalArr ", cal_table);    // Output the entire calibration array for debugging

//         // Iterate through each calibration point to compute deviations
//         for (size_t i = 0; i < cal_size; ++i) {                   // Loop over every recorded calibration point
//             uint16_t val = (uint16_t)(cal_table[i] - offst_val);  // Normalize: shift values so offset physically becomes zero
//             size_t corrected_idx =
//                 ((cal_size + i) - offst_idx) % cal_size;  // Compute corrected circular index compensating for offset
//             uint16_t deviation = abs_deviation(val, corrected_idx, cal_size);  // Calculate angular modular deviation from ideal

//             cal_table[i] = val;  // Update calibration table purely with the normalized value

//             if (deviation > max_deviation) {  // Check if this point has the worst deviation so far
//                 max_deviation = deviation;    // Track maximum deviation found globally
//             }  // End of max deviation update

//             // Check if deviation exceeds acceptable threshold (average step size)
//             if (deviation >= 3 * avg_step) {    // Verify linearity is within tolerable mechanical limits
//                 TP_LOG_VAL("Dev ", deviation);  // Output the failing deviation value for debugging
//                 return false;                   // Validation failed: sensor non-linearity is too large
//             }  // End of deviation threshold check
//         }  // End of validation loop

//         build_ecc_table();                   // Construct the compact inverse eccentricity table upon successful validation
//         TP_LOG_ARRAY("ECCArr ", ecc_table);  // Output the entire calibration array for debugging

//         return true;  // Validation successful: all deviations are within acceptable physical bounds
//     }  // End of check() method

//     /**
//      * @brief Builds the inverse eccentricity table in raw encoder angle space
//      *
//      * The ECC table must be independent from CAL zero alignment. It is built directly
//      * as a function of raw absolute encoder angle, so that correct_position() can be
//      * applied to any raw sample without using CAL_TABLE phase information.
//      *
//      * The algorithm:
//      * 1. Reconstruct the original averaged raw angle for each calibration sample.
//      * 2. Build an ideal linear ramp anchored to the first raw sample.
//      * 3. Compute signed error = ideal - raw using circular angle arithmetic.
//      * 4. Accumulate that error by raw angle bin, not by CAL-aligned angle bin.
//      * 5. Fill empty bins with circular linear interpolation.
//      * 6. Apply circular smoothing.
//      * 7. Remove the DC component so ECC only linearizes shape and does not shift phase.
//      */
//     void build_ecc_table() {  // Process collected calibration data into a fast raw-angle lookup table
//         if (cal_size == 0)
//             return;  // Guard against uninitialized state

//         // Using int16_t for intermediate processing arrays to minimize stack footprint (saves MCU memory)
//         int16_t error[CAL_TABLE_CAPACITY];

//         // 1. Reconstruct ideal linear ramp and compute signed error using circular logic
//         for (size_t i = 0; i < cal_size; ++i) {
//             size_t corrected_idx = ((cal_size + i) - offst_idx) % cal_size;
//             uint16_t ideal = get_ideal(corrected_idx, cal_size);
//             uint16_t normalized_val = cal_table[i];        // Normalized value
//             error[i] = angle_diff(normalized_val, ideal);  // Shortest signed path
//         }

//         // 6. Apply circular zero-phase smoothing (Double Centered Moving Average)
//         int16_t sm_pass1[CAL_TABLE_CAPACITY];
//         const int WINDOW = 7;

//         for (size_t i = 0; i < cal_size; ++i) {  // First smoothing pass
//             int32_t sum = 0;
//             for (int j = -WINDOW; j <= WINDOW; ++j) {
//                 int idx = ((int)i + j) % (int)cal_size;
//                 if (idx < 0)
//                     idx += (int)cal_size;  // Circular wrap
//                 sum += error[idx];
//             }
//             sm_pass1[i] = (int16_t)(sum / (2 * WINDOW + 1));
//         }

//         int16_t sm_pass2[CAL_TABLE_CAPACITY];
//         for (size_t i = 0; i < cal_size; ++i) {  // Second smoothing pass for deep high-freq noise rejection
//             int32_t sum = 0;
//             for (int j = -WINDOW; j <= WINDOW; ++j) {
//                 int idx = ((int)i + j) % (int)cal_size;
//                 if (idx < 0)
//                     idx += (int)cal_size;  // Circular wrap
//                 sum += sm_pass1[idx];
//             }
//             sm_pass2[i] = (int16_t)(sum / (2 * WINDOW + 1));
//         }

//         // 4 & 5. Accumulate and fill bins using linear interpolation driven by raw angle mapping
//         uint32_t bin_step = 65536 / ECC_TABLE_SIZE;
//         for (size_t k = 0; k < ECC_TABLE_SIZE; ++k) {
//             uint16_t target_raw = (uint16_t)(k * bin_step);             // Absolute raw encoder angle for this bin
//             uint16_t target_norm = (uint16_t)(target_raw - offst_val);  // Shift into normalized table space

//             bool found = false;
//             for (size_t step = 0; step < cal_size; ++step) {
//                 // Moving forward segment by segment to bracket the target normalized value
//                 size_t c_idx1 = step;
//                 size_t c_idx2 = (step + 1) % cal_size;

//                 size_t real_i1 = (offst_idx + c_idx1) % cal_size;
//                 size_t real_i2 = (offst_idx + c_idx2) % cal_size;

//                 uint16_t norm1 = cal_table[real_i1];
//                 uint16_t norm2 = cal_table[real_i2];

//                 // Using angle_diff correctly detects bracketing even across the 65535->0 overflow point
//                 int16_t diff1 = angle_diff(target_norm, norm1);
//                 int16_t diff2 = angle_diff(target_norm, norm2);

//                 if (diff1 >= 0 && diff2 < 0) {  // Target falls exactly between norm1 and norm2
//                     int16_t err1 = sm_pass2[real_i1];
//                     int16_t err2 = sm_pass2[real_i2];

//                     uint16_t ref_range = (uint16_t)(norm2 - norm1);
//                     if (ref_range == 0) {
//                         ecc_table[k] = err1;  // Extreme edge case fallback
//                     } else {
//                         uint16_t frac = (uint16_t)(target_norm - norm1);
//                         int32_t interp_err = err1 + ((err2 - err1) * (int32_t)frac) / (int32_t)ref_range;
//                         ecc_table[k] = (int16_t)interp_err;  // Safely mapped offset into the LUT
//                     }
//                     found = true;
//                     break;
//                 }
//             }
//             if (!found) {
//                 ecc_table[k] = 0;  // Safeguard if loop completes without bracketing
//             }
//         }

//         // 7. Remove the DC component so ECC only linearizes shape and does not shift phase
//         int32_t dc_sum = 0;
//         for (size_t k = 0; k < ECC_TABLE_SIZE; ++k) {
//             dc_sum += ecc_table[k];
//         }
//         int16_t dc_offset = (int16_t)(dc_sum / (int32_t)ECC_TABLE_SIZE);  // Compute DC average error

//         for (size_t k = 0; k < ECC_TABLE_SIZE; ++k) {
//             ecc_table[k] -= dc_offset;  // Zero-mean normalization
//         }
//     }  // End of build_ecc_table() method

//     /**
//      * @brief Corrects encoder position using calibration table
//      * @param position Raw encoder position
//      * @return Corrected electrical angle mapped to the electrical period
//      *
//      * Uses interpolation to map real encoder positions to corrected angles,
//      * compensating for non-linearities detected during calibration.
//      */
//     uint16_t correct_el_angle(uint16_t position) const {  // Hot path: corrects raw sensor data continuously
//         if (cal_size == 0) {  // CRITICAL SAFETY: Prevent division by zero and garbage output if uncalibrated
//             return 0;         // Fallback: default electrical angle to zero
//         }  // End of safety abort

//         // Align position so zero aligns with table's zero-offset point
//         uint16_t real_pos = (uint16_t)(position - offst_val);  // Shift input into the normalized table domain

//         // Estimate starting index by scaling real_pos down. Subtract max_deviation to bracket search correctly.
//         size_t idx = ((size_t)(uint16_t)(real_pos - max_deviation) * cal_size) >> 16;  // Fast division by 65536 using shift

//         // Get initial comparison points to prepare for linear segment search
//         uint16_t cal_pos1 = get_val_by_idx(idx);       // Fetch calibration value at estimated current index
//         uint16_t idl_pos1 = get_ideal(idx, cal_size);  // Fetch theoretical ideal value at estimated current index

//         uint16_t result = idl_pos1;  // Initialize result to the best available estimate

//         // Iterate up to 8 steps to find precise segment where real_pos physically falls
//         for (int iter = 0; iter < 8; ++iter) {             // Search loop with strict deterministic bounded execution
//             idx = (idx + 1) % cal_size;                    // Move to next index circularly
//             uint16_t cal_pos2 = get_val_by_idx(idx);       // Calibration value at next index boundary
//             uint16_t idl_pos2 = get_ideal(idx, cal_size);  // Ideal value at next index boundary

//             int16_t diff1 = (int16_t)(real_pos - cal_pos1);  // Signed difference from left bound
//             int16_t diff2 = (int16_t)(real_pos - cal_pos2);  // Signed difference from right bound

//             // Find boundary where diff changes sign, meaning real_pos is bracketed between cal_pos1 and cal_pos2
//             if ((diff1 >= 0) && (diff2 < 0)) {                                           // Verify zero-crossing logic
//                 result = interpolate(cal_pos1, idl_pos1, cal_pos2, idl_pos2, real_pos);  // Compute exact sub-segment position
//                 break;  // Exit loop after finding the correct segment
//             }  // End of bracketing condition

//             // Move to next segment window
//             cal_pos1 = cal_pos2;  // Left bound calibration becomes old right bound
//             idl_pos1 = idl_pos2;  // Left bound ideal becomes old right bound
//         }  // End of bounded segment search

//         // Compute mechanical angle mapped into one electrical period safely
//         uint16_t el = (uint16_t)(((uint32_t)result * (uint32_t)cal_size) / (uint32_t)el_angle_div);

//         // Компенсация того, что первый калибровочный сэмпл был снят после +1 electrical step
//         const uint16_t phase_shift = (uint16_t)(0x10000u / (uint32_t)el_angle_div);

//         return (uint16_t)(el - phase_shift);
//     }  // End of correct_el_angle() method

//     /**
//      * @brief Fast O(1) raw-angle correction using the ECC table
//      * @param position Raw absolute encoder position
//      * @return Corrected absolute encoder position compensating for magnetic eccentricity
//      *
//      * The ECC table stores inverse error as a function of raw encoder angle.
//      * Linear interpolation between neighboring bins provides a smooth correction.
//      */
//     // Финальная и самая точная версия коррекции:
//     uint16_t correct_position(uint16_t position) const {
//         if (cal_size == 0)
//             return position;

//         // Сдвиг на 11 бит для ECC_TABLE_SIZE = 32 (работает быстрее деления)
//         uint16_t idx = position >> 11;
//         uint16_t next_idx = (idx + 1) & 31;   // Маска для 32
//         uint16_t fraction = position & 2047;  // Маска остатка

//         int16_t offset1 = ecc_table[idx];
//         int16_t offset2 = ecc_table[next_idx];

//         // Интерполяция (сдвиг обратно на 11 бит)
//         int32_t interp_error = offset1 + (((offset2 - offset1) * (int32_t)fraction) >> 11);

//         return (uint16_t)(position - interp_error);
//     }

// private:                                     // Restrict direct access to internal state memory and helper routines
//     uint16_t cal_table[CAL_TABLE_CAPACITY];  // Array storing sampled encoder data across full physical calibration cycle
//     int16_t ecc_table[ECC_TABLE_SIZE];       // Array storing the compact inverse eccentricity correction table
//     size_t cal_size;                         // Current valid physical number of points actually collected
//     size_t el_angle_div;                     // Scaling divider representing steps forming one electrical period
//     size_t offst_idx;                        // Physical index of first zero-position value discovered after normalization
//     uint16_t offst_val;                      // Absolute offset to align raw calibration data to mathematical zero
//     uint16_t max_deviation;                  // Maximum absolute modular deviation tracked from ideal theoretical values
//     size_t temp_idx;                         // Temporary state index used strictly during sequential data collection checks

//     /**
//      * @brief Retrieves calibration value by index relative to offset
//      * @param index Index relative to start_idx
//      * @return Calibration value at computed circular index
//      */
//     uint16_t get_val_by_idx(size_t index) const {            // Helper to read from offset-shifted table circularly
//         size_t actual_idx = (offst_idx + index) % cal_size;  // Compute actual circular index safely (cal_size verified > 0)
//         return cal_table[actual_idx];                        // Return physical memory value at computed position
//     }  // End of get_val_by_idx() method

//     /**
//      * @brief Computes ideal value for given index in linear progression
//      * @param i Index in range
//      * @param range Total number of points
//      * @return Ideal value assuming linear increase from 0 to UINT16_MAX
//      */
//     static uint16_t get_ideal(size_t i, size_t range) {  // Predict perfect theoretical encoder position
//         const uint32_t CAL_VAL_RANGE = UINT16_MAX;       // Full 16-bit range scaling multiplier
//         return (uint16_t)((CAL_VAL_RANGE * i) / range);  // Calculate ideal fractional value based on linear progression
//     }  // End of get_ideal() method

//     /**
//      * @brief Calculates absolute deviation from ideal linear progression
//      * @param val Actual measured value
//      * @param idx Index in calibration table
//      * @param size Total size of calibration table
//      * @return Absolute deviation from ideal value
//      */
//     static uint16_t abs_deviation(uint16_t val, size_t idx, size_t size) {  // Calculate angular error reliably
//         uint16_t ideal = get_ideal(idx, size);                              // Compute perfectly linear ideal value at this index
//         int16_t diff = (int16_t)(ideal - val);        // Signed 16-bit cast handles circular angular wrap-around naturally
//         return (uint16_t)(diff >= 0 ? diff : -diff);  // Return absolute angular shortest-path distance
//     }  // End of abs_deviation() method

//     /**
//      * @brief Computes shortest signed circular difference between two uint16_t angles
//      * @param target Desired angle
//      * @param value Current angle
//      * @return Signed shortest-path difference target - value
//      */
//     static int16_t angle_diff(uint16_t target, uint16_t value) {  // Helper for circular signed angle arithmetic
//         return (int16_t)(target - value);  // Two's-complement wrap gives the shortest signed 16-bit angle difference
//     }  // End of angle_diff() method

//     /**
//      * @brief Interpolates position within calibration segment
//      * @param a1 Real position at segment start
//      * @param a2 Ideal position at segment start
//      * @param b1 Real position at segment end
//      * @param b2 Ideal position at segment end
//      * @param c1 Real position to interpolate
//      * @return Interpolated ideal position corresponding to c1
//      *
//      * Linear interpolation: maps c1 from [a1, b1] range to [a2, b2] range.
//      */
//     static uint16_t interpolate(uint16_t a1,
//                                 uint16_t a2,
//                                 uint16_t b1,
//                                 uint16_t b2,
//                                 uint16_t c1) {     // Perform precise piecewise linear interpolation
//         uint16_t ref_range = (uint16_t)(b1 - a1);  // Calculate true reference range in physical domain
//         if (ref_range == 0)
//             return a2;                              // Prevent division-by-zero if encoder is stuck
//         uint16_t trgt_range = (uint16_t)(b2 - a2);  // Calculate target ideal range
//         uint16_t c1_ofst = (uint16_t)(c1 - a1);     // Determine fractional offset of c1 within bounded segment
//         uint32_t c2_ofst =
//             ((uint32_t)c1_ofst * (uint32_t)trgt_range) / (uint32_t)ref_range;  // Project fraction into ideal domain
//         return (uint16_t)((uint32_t)a2 + c2_ofst);                             // Return properly scaled interpolated ideal value
//     }  // End of interpolate() method
// };  // End of CalibrationTable structure
