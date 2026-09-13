/**
 * @file calibration_table.h
 * @brief Motor calibration data collection and position correction module
 *
 * Implements the motor calibration module, handling calibration data collection,
 * normalization, and position correction for the motor control system.
 *
 * Key Features:
 * - Collects and stores encoder data during motor rotation for calibration.
 * - Removes hysteresis effects through bidirectional data collection.
 * - Determines calibration offset and start index based on minimal deviation.
 * - Validates calibration data for consistency and accuracy.
 * - Corrects motor position readings using the calibrated data.
 * - Features an ECC_TABLE_SIZE-point inverse lookup table for magnetic eccentricity compensation.
 *
 * Algorithm:
 * 1. First pass: Collect encoder samples during forward rotation.
 * 2. Second pass: Collect samples during backward rotation and average with first pass
 *    to eliminate hysteresis effects.
 * 3. Normalization: Find minimum value and use it as offset, align data to zero.
 * 4. Validation: Check that deviations from ideal linear progression are within bounds.
 * 5. ECC build: A compact inverse table of raw-angle eccentricity error, rebuilt at
 *    every flash-record commit (commit_candidate) so a boot-loaded table and a
 *    freshly calibrated one produce bit-identical RAM state.
 * 6. Correction: Use fast interpolation to output stable corrected angles.
 *
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#pragma once // Prevent multiple compiler inclusions of this header file

#include "../../common.h" // Include common project definitions, utilities, and macros
#include "../../drivers/flash_eeprom.h" // EEPROM-emulation driver used for calibration-table persistence

#ifndef CAL_TABLE_CAPACITY // Check if CAL_TABLE_CAPACITY is not defined externally
#define CAL_TABLE_CAPACITY 400 // Maximum number of calibration points allocated in memory
#endif // End of CAL_TABLE_CAPACITY definition block

#ifndef ECC_TABLE_SIZE // Check if ECC_TABLE_SIZE is not defined externally
#define ECC_TABLE_SIZE 32 // Number of points for the eccentricity compensation table
#endif // End of ECC_TABLE_SIZE definition block

/**
 * @brief Calibration table for motor encoder position correction
 *
 * Stores calibration data collected during motor rotation and provides
 * methods for data collection, validation, and position correction.
 */
struct CalibrationTable { // Begin declaration of the CalibrationTable structure
public: // Make the following methods public for external module access
    /**
     * @brief Constructs a new CalibrationTable with default values
     * @return Initialized CalibrationTable instance
     */
    static CalibrationTable create() { // Factory method to instantiate and initialize the table
        CalibrationTable t; // Allocate a new table instance
        t.offst_val = 0; // Initialize angular offset value to zero
        t.offst_idx = 0; // Initialize physical offset index to zero
        t.cal_size = 0; // Set initial collected calibration points counter to zero
        for (size_t i = 0; i < CAL_TABLE_CAPACITY; ++i) { // Loop through all table entries
            t.cal_table[i] = 0; // Clear all calibration table entries systematically
        } // End of initialization loop
        t.el_angle_div = 1; // Default electrical angle divider set to 1 to prevent division by zero
        t.max_deviation = 0; // Reset max deviation tracker to zero
        t.temp_idx = 0; // Reset temporary sequence tracker index

        for (size_t i = 0; i < ECC_TABLE_SIZE; ++i) { // Loop through eccentricity table entries
            t.ecc_table[i] = 0; // Clear the eccentricity correction table to zero
        } // End of eccentricity table initialization loop

        return t; // Return the fully initialized instance
    } // End of create() method

    // --------------------------------------------------------------------------
    // Flash persistence (non-blocking, exactly 8 words / 32 bytes moved per
    // tick call - never more than one programmed/read block per system tick)
    //
    // Stored-record word stream (records append upward, forming a chain; the
    // LAST fully valid record is the actual one):
    //   w0     : 0xFA3E0000 - table marker
    //   w1     : sample_count
    //   w2..wN : samples packed 2 x uint16_t per word (odd tail padded 0xFFFF)
    //   tail   : {el_angle_div, offst_val | max_deviation << 16, offst_idx}
    //
    // The record ID is NOT stored inside the record: it IS the start address
    // of the marker word expressed in 32-byte chunks (offset / 32). This ID
    // is handed over to the motor parameter profile after a successful dump,
    // so the table can be addressed directly on the next boot.
    // --------------------------------------------------------------------------

    /**
     * @brief Injects the flash driver used for load/dump transfers
     * @param f Pointer to FlashEEPROM instance (nullptr disables persistence)
     */
    void attach_flash(FlashEEPROM* f) { flash = f; } // Store the driver pointer

    /**
     * @brief Arms a direct-addressed table load by calibration ID
     *
     * The ID is the start address of the marker word expressed in 32-byte
     * chunks. The scan reads the record at that exact offset (one 8-word
     * block per tick), taking the sample count from the record itself.
     *
     * @param chunk_id Calibration ID (record start address / 32).
     */
    void begin_load(uint32_t chunk_id) {
        load_ok = false;                         // Invalidate the previous scan result
        f_hdr = false;                           // Parser expects a fresh marker word
        f_cnt = 0;                               // Reset the word-parser state
        f_meta = false;
        uint32_t addr = (uint32_t)chunk_id * 32;
        if (flash && flash->get_max_size() > addr) {
            f_addr = addr;                       // Jump straight to the referenced record
            f_mode = MODE_LOAD;
        } else {
            f_mode = MODE_IDLE;                  // No driver or ID out of range: nothing to read
        }
    }

    /**
     * @brief Services one 32-byte block of the flash scan
     * @return true when the scan is finished (outcome via is_loaded())
     */
    bool load_tick() {
        if (f_mode != MODE_LOAD) return true;         // Not armed / already committed: done
        if (flash->get_max_size() == 0) return false; // Driver not configured yet: wait
        for (int b = 0; b < 1 && f_mode == MODE_LOAD; ++b) { // 1 x 32 B = 8 words per tick
            if (f_addr + 32 > flash->get_max_size()) { f_mode = MODE_IDLE; break; } // Region end
            uint32_t w[8];
            if (!flash->read_block(f_addr, w)) { f_mode = MODE_IDLE; break; }       // HW fault
            f_addr += 32;                                                          // Advance cursor
            for (int i = 0; i < 8 && f_mode == MODE_LOAD; ++i) scan_word(w[i]);    // Parse words
            if (load_ok) f_mode = MODE_IDLE;                                       // Committed: done
        }
        return f_mode != MODE_LOAD; // true once scanning stopped (done or aborted)
    }

    /**
     * @brief Result of the last completed flash scan
     * @return true when a valid table was restored into the RAM arrays
     */
    bool is_loaded() const { return load_ok; }

    /**
     * @brief Arms a dump of the RAM table into flash ("dump_calibration")
     *
     * First locates free space (32 bytes per tick); the record is appended
     * there when it fits (aligned up to the 32-byte chunk boundary so the
     * marker always starts a chunk), otherwise the whole region is erased and
     * rewritten from offset zero. After a successful dump dumped_id() returns
     * the chunk address of the marker - the calibration ID for the motor
     * profile.
     */
    void begin_dump() {
        store_ok = false;                                           // Clear the previous outcome
        if (!flash || cal_size == 0) return;                        // Nothing to persist
        f_total = 2 + (cal_size + 1) / 2 + 3;                       // Marker + count + samples + trailer words
        if ((f_total + 7) / 8 * 32 > flash->get_max_size()) return; // Record can never fit: fail fast
        f_free = flash->get_max_size();                             // Assume no free tail yet
        f_addr = 0;                                                 // Space scan starts at zero
        f_mode = MODE_SPACE;                                        // Phase 1: locate free space
    }

    /**
     * @brief Calibration ID of the last completed dump
     * @return Start address of the FA3E marker in 32-byte chunks.
     */
    uint32_t dumped_id() const { return store_ok ? (f_wri0 / 32) : 0; }

    /**
     * @brief Number of samples currently collected in the RAM table
     */
    uint16_t get_sample_count() const { return (uint16_t)cal_size; }

    /**
     * @brief Read one stored calibration sample by index (protocol table access).
     * @param index Sample index, 0 .. get_sample_count()-1.
     * @return The sample value; 0 when the index is out of range.
     */
    uint16_t get_sample(size_t index) const {
        return (index < cal_size) ? get_val_by_idx(index) : 0u;  // Bounds-checked read of the table
    }

    /**
     * @brief Truncates the RAM table to exactly @p n samples
     *
     * Used to align the collected sample count to a multiple of the
     * electrical-period step before validation and persistence.
     */
    void truncate(uint16_t n) {
        if (n < cal_size) cal_size = n;
        if (temp_idx > n) temp_idx = n;
    }

    /**
     * @brief Clears the WHOLE RAM state to pristine power-on values
     *
     * Samples, ECC table and every metadata field return to exactly the
     * state a fresh power-up produces via create(). The flash binding is
     * preserved (create() returns an unbound instance), so a subsequent
     * begin_load()/load_tick() rebuilds the live state purely from the
     * stored record - the same code path, and the same bytes, as after an
     * HW reset. Used right after a dump completes: the running state is
     * then guaranteed to contain NOTHING that survived from the physical
     * calibration sweep itself.
     */
    void wipe() {
        FlashEEPROM* bound = flash;  // Preserve the persistence binding
        *this = create();            // Every field back to power-on defaults
        attach_flash(bound);         // Re-bind the flash driver
    }

    /**
     * @brief Services one 32-byte block of the space scan / record write
     * @return true when the dump is finished (outcome via dump_ok())
     */
    bool dump_tick() {
        if (f_mode == MODE_SPACE) {                                // Phase 1: find first erased word
            for (int b = 0; b < 1 && f_mode == MODE_SPACE; ++b) {  // 8 words per tick
                if (f_addr + 32 > flash->get_max_size()) f_mode = MODE_IDLE; // Scanned everything
                else {
                    uint32_t w[8];
                    if (!flash->read_block(f_addr, w)) {
                        // Read fault == the block was never programmed (erased
                        // flash raises a double-bit ECC NMI, read_section
                        // reports false). Free space therefore STARTS at this
                        // chunk - appending here is safe. Treating this as a
                        // HW fault (previous behavior) made every dump fall
                        // into the erase-everything path: needless flash wear
                        // plus a tens-of-ms __disable_irq() stall inside the
                        // control ISR on every calibration.
                        f_free = f_addr;
                        f_mode = MODE_IDLE;
                    } else {
                        f_addr += 32;
                        for (int i = 0; i < 8 && f_mode == MODE_SPACE; ++i)
                            if (w[i] == 0xFFFFFFFFul) {            // First unprogrammed word
                                f_free = f_addr - 32 + i * 4;      // Free tail starts here
                                f_mode = MODE_IDLE;
                            }
                    }
                }
            }
            if (f_mode == MODE_SPACE) return false;                // Scan continues next tick
            uint32_t need = (f_total + 7) / 8 * 32;                // Size padded to write granularity
            // Align the append point UP to the chunk boundary so the marker
            // word always lands at a chunk start - its chunk number IS the ID.
            uint32_t aligned = (f_free + 31) & ~31ul;
            if (aligned + need <= flash->get_max_size()) {         // Fits into the free tail: append
                f_wri0 = aligned; f_wri = aligned; f_word = 0; f_mode = MODE_WRITE;
            } else if (!tp_flash_programming_window) {             // Full, but the bridge still carries
                return false;                                      // an energized vector: WAIT for the
            } else if (flash->erase(0xDEADC0DE)) {                 // de-energized window (a page erase
                TP_LOG("CALIBRATION: Flash full - erasing region"); // stalls the ISR for tens of ms)
                f_wri0 = 0; f_wri = 0; f_word = 0; f_mode = MODE_WRITE;
            }                                                      // Erase failed: stays IDLE = fail
            return f_mode != MODE_WRITE;
        }
        if (f_mode == MODE_WRITE) {                                // Phase 2: stream the record out
            if (!tp_flash_programming_window) {                    // Zero-current vector must be latched
                return false;                                      // on the bridge BEFORE programming:
            }                                                      // each write tick stalls the ISR with
                                                                   // IRQs off, holding the previous vector
            for (int r = 0; r < 1 && f_word < f_total; ++r) {      // 1 record = 8 words per tick
                uint32_t rec[8];
                uint32_t n = f_total - f_word < 8 ? f_total - f_word : 8; // Words left in this record
                for (uint32_t i = 0; i < n; ++i) rec[i] = stream_word(f_word + i);
                for (uint32_t i = n; i < 8; ++i) rec[i] = 0xFFFFFFFFul;   // Pad with erased value
                if (!flash->write(f_wri, rec)) { f_mode = MODE_IDLE; return true; } // HW fault: fail
                f_wri += 32;                                       // Advance record address
                f_word += 8;                                       // Advance stream index
            }
            if (f_word >= f_total) { store_ok = true; f_mode = MODE_IDLE; } // Record fully written
            return f_mode == MODE_IDLE;
        }
        return true;                                               // Not armed: already done
    }

    /**
     * @brief Result of the last completed dump
     * @return true when the record was written to flash completely
     */
    bool dump_ok() const { return store_ok; }

    /**
     * @brief Resets the calibration table for a new calibration process
     * @param el_angle_div Number of steps forming one electrical period
     */
    void reset(uint16_t el_angle_div) { // Prepare table for a fresh calibration run
        this->offst_idx = 0; // Reset start index to beginning
        this->el_angle_div = (el_angle_div == 0) ? 1 : (size_t)el_angle_div; // Store electrical angle divider safely (prevent 0)
        this->cal_size = 0; // Clear the size (no valid data points yet)
        this->offst_val = UINT16_MAX; // Initialize offset to max to find minimum during search later
        this->max_deviation = 0; // Reset maximum deviation counter
        this->temp_idx = 0; // Reset temporary index for sequential filling validation

        for (size_t i = 0; i < ECC_TABLE_SIZE; ++i) { // Clear stale eccentricity data from previous calibration
            this->ecc_table[i] = 0; // Prevent old ECC compensation from being used after reset
        } // End of ECC table reset loop
    } // End of reset() method

    /**
     * @brief Stores first round of calibration data (forward rotation)
     * @param idx Index in calibration table
     * @param val Encoder value to store
     * @return true if storage successful, false if index out of bounds
     *
     * Collects raw samples as the motor rotates in one direction.
     */
    bool fill_first(size_t idx, uint16_t val) { // Insert forward rotation samples
        // Check if the index is within bounds and sequentially incremented
        if (idx < CAL_TABLE_CAPACITY && (idx >= temp_idx ? idx - temp_idx : temp_idx - idx) <= 1) { // Bounds and absolute difference check
            temp_idx = idx; // Update temporary index for sequential check on next call
            cal_table[idx] = val; // Store the sample value at target index
            if (cal_size < idx + 1) { // Verify if we are expanding the recorded size
                cal_size = idx + 1; // Update cal_size to reflect maximum filled index
            } // End of size update condition
            return true; // Indicate successful storage to the caller
        } // End of bounds and sequence validation
        return false; // Index out of bounds or non-sequential, reject sample
    } // End of fill_first() method

    /**
     * @brief Performs second pass filling (backward rotation) to remove hysteresis
     * @param idx Index in calibration table
     * @param val Encoder value from backward rotation
     * @return true if this is the last point, false otherwise
     *
     * Averages forward and backward measurements to center the hysteresis loop.
     * Also identifies the minimum offset value for normalization.
     */
    bool fill_second(size_t idx, uint16_t val) { // Insert backward rotation samples and average
        if (idx < CAL_TABLE_CAPACITY) { // Check if the index is strictly within array bounds
            // Calculate signed difference to handle wraparound near 0 or max range perfectly
            int16_t dif = (int16_t)(val - cal_table[idx]); // Signed cast evaluates shortest modular angular path

            // Update table with midpoint of hysteresis range (average of forward/backward)
            uint16_t mid = (uint16_t)(cal_table[idx] + (uint16_t)(dif / 2)); // Calculate central point handling wrap-around
            cal_table[idx] = mid; // Store the averaged hysteresis-free value

            // Every el_angle_div steps, check if this is the minimal offset position
            if (((idx % el_angle_div) == 0) && (mid <= offst_val)) { // Modulo is safe here because el_angle_div is verified in reset()
                offst_val = mid; // Update global offset to the lowest encountered value
                offst_idx = idx; // Update start index to point to this exact lowest position
            } // End of offset tracking block

            // If current index is the last filled index, record max deviation
            if (idx + 1 == cal_size) { // Detect if this is the final element of the loop
                max_deviation = mid; // Store final maximum deviation base value
                return true; // Indicate completion of second pass safely
            } // End of final element check

            // Calculate difference between previous tracked value and new value
            int16_t diff = (int16_t)(max_deviation - mid); // Tracks sudden jumps or steps in backward progression
            max_deviation = mid; // Update deviation tracker to new current value

            // If deviation exceeds threshold, indicate completion (detects noise/slips)
            if (diff > 10) { // Large step difference triggers early exit
                return true; // Significant deviation detected, finalize pass
            } // End of threshold verification
        } // End of bounds check
        return false; // Not yet complete or index was out of bounds
    } // End of fill_second() method

    /**
     * @brief Validates calibration data consistency
     * @return true if calibration data is valid, false otherwise
     *
     * Checks that deviations from ideal linear distribution are acceptable.
     * Normalizes all values by subtracting the offset.
     */
    bool check() { // Run final verification and normalization on collected data
        if (cal_size == 0) { // Safety check to prevent divide-by-zero operations below
            return false; // No data to validate, return failure
        } // End of size check

        // Calculate average step size based on total range and number of samples
        uint16_t avg_step = (uint16_t)(UINT16_MAX / (uint16_t)cal_size); // Theoretical step between array elements
        max_deviation = 0; // Reset maximum deviation counter for the check phase

        // NOTE: no TP_LOG here on purpose (neither the offset values nor the
        // per-sample deviations). This method runs inside the PWM-period
        // control ISR WITH THE MOTOR ENERGIZED at the calibration holding
        // current; any SerialUSB write can block on host back-pressure and
        // freeze the control loop on a frozen voltage vector - the proven
        // mechanism behind the post-calibration chaos. The same values are
        // reported once per commit by commit_candidate(), which runs in the
        // de-energized flash-I/O window instead.

        // Iterate through each calibration point to compute deviations
        for (size_t i = 0; i < cal_size; ++i) { // Loop over every recorded calibration point
            uint16_t val = (uint16_t)(cal_table[i] - offst_val); // Normalize: shift values so offset physically becomes zero
            size_t corrected_idx = ((cal_size + i) - offst_idx) % cal_size; // Compute corrected circular index compensating for offset
            uint16_t deviation = abs_deviation(val, corrected_idx, cal_size); // Calculate angular modular deviation from ideal

            cal_table[i] = val; // Update calibration table purely with the normalized value

            if (deviation > max_deviation) { // Check if this point has the worst deviation so far
                max_deviation = deviation; // Track maximum deviation found globally
            } // End of max deviation update

            // Check if deviation exceeds acceptable threshold (average step size)
            if (deviation >= 3 * avg_step) { // Verify linearity is within tolerable mechanical limits
                return false; // Validation failed: sensor non-linearity is too large
            } // End of deviation threshold check
        } // End of validation loop

        // NOTE: build_ecc_table() is deliberately NOT called here. check() runs on
        // the fresh-calibration path only, so an ECC table built here would exist
        // ONLY in the post-calibration RAM state. The table is rebuilt by
        // commit_candidate() instead - the single commit point shared by the boot
        // flash load and the post-save read-back - which guarantees the live
        // post-calibration state (cal_table AND ecc_table) is bit-identical to the
        // state a power cycle restores from the same record.

        return true; // Validation successful: all deviations are within acceptable physical bounds
    } // End of check() method

    /**
     * @brief Builds the inverse eccentricity table in raw encoder angle space
     *
     * The ECC table must be independent from CAL zero alignment. It is built directly
     * as a function of raw absolute encoder angle, so that correct_position() can be
     * applied to any raw sample without using CAL_TABLE phase information.
     *
     * The algorithm:
     * 1. Reconstruct the original averaged raw angle for each calibration sample.
     * 2. Build an ideal linear ramp anchored to the first raw sample.
     * 3. Compute signed error = ideal - raw using circular angle arithmetic.
     * 4. Accumulate that error by raw angle bin, not by CAL-aligned angle bin.
     * 5. Fill empty bins with circular linear interpolation.
     * 6. Apply circular smoothing.
     * 7. Remove the DC component so ECC only linearizes shape and does not shift phase.
     */
    void build_ecc_table() { // Process collected calibration data into a fast raw-angle lookup table
        if (cal_size == 0) return; // Guard against uninitialized state

        // Using int16_t for intermediate processing arrays to minimize stack footprint (saves MCU memory)
        int16_t error[CAL_TABLE_CAPACITY];

        // 1. Reconstruct ideal linear ramp and compute signed error using circular logic
        for (size_t i = 0; i < cal_size; ++i) {
            size_t corrected_idx = ((cal_size + i) - offst_idx) % cal_size;
            uint16_t ideal = get_ideal(corrected_idx, cal_size);
            uint16_t normalized_val = cal_table[i]; // Normalized value
            error[i] = angle_diff(normalized_val, ideal); // Shortest signed path
        }

        // 6. Apply circular zero-phase smoothing (Double Centered Moving Average)
        int16_t sm_pass1[CAL_TABLE_CAPACITY];
        const int WINDOW = 7;

        for (size_t i = 0; i < cal_size; ++i) { // First smoothing pass
            int32_t sum = 0;
            for (int j = -WINDOW; j <= WINDOW; ++j) {
                int idx = ((int)i + j) % (int)cal_size;
                if (idx < 0) idx += (int)cal_size; // Circular wrap
                sum += error[idx];
            }
            sm_pass1[i] = (int16_t)(sum / (2 * WINDOW + 1));
        }

        int16_t sm_pass2[CAL_TABLE_CAPACITY];
        for (size_t i = 0; i < cal_size; ++i) { // Second smoothing pass for deep high-freq noise rejection
            int32_t sum = 0;
            for (int j = -WINDOW; j <= WINDOW; ++j) {
                int idx = ((int)i + j) % (int)cal_size;
                if (idx < 0) idx += (int)cal_size; // Circular wrap
                sum += sm_pass1[idx];
            }
            sm_pass2[i] = (int16_t)(sum / (2 * WINDOW + 1));
        }

        // 4 & 5. Accumulate and fill bins using linear interpolation driven by raw angle mapping
        uint32_t bin_step = 65536 / ECC_TABLE_SIZE;
        for (size_t k = 0; k < ECC_TABLE_SIZE; ++k) {
            uint16_t target_raw = (uint16_t)(k * bin_step); // Absolute raw encoder angle for this bin
            uint16_t target_norm = (uint16_t)(target_raw - offst_val); // Shift into normalized table space

            bool found = false;
            for (size_t step = 0; step < cal_size; ++step) {
                // Moving forward segment by segment to bracket the target normalized value
                size_t c_idx1 = step;
                size_t c_idx2 = (step + 1) % cal_size;

                size_t real_i1 = (offst_idx + c_idx1) % cal_size;
                size_t real_i2 = (offst_idx + c_idx2) % cal_size;

                uint16_t norm1 = cal_table[real_i1];
                uint16_t norm2 = cal_table[real_i2];

                // Using angle_diff correctly detects bracketing even across the 65535->0 overflow point
                int16_t diff1 = angle_diff(target_norm, norm1);
                int16_t diff2 = angle_diff(target_norm, norm2);

                if (diff1 >= 0 && diff2 < 0) { // Target falls exactly between norm1 and norm2
                    int16_t err1 = sm_pass2[real_i1];
                    int16_t err2 = sm_pass2[real_i2];

                    uint16_t ref_range = (uint16_t)(norm2 - norm1);
                    if (ref_range == 0) {
                        ecc_table[k] = err1; // Extreme edge case fallback
                    } else {
                        uint16_t frac = (uint16_t)(target_norm - norm1);
                        int32_t interp_err = err1 + ((err2 - err1) * (int32_t)frac) / (int32_t)ref_range;
                        ecc_table[k] = (int16_t)interp_err; // Safely mapped offset into the LUT
                    }
                    found = true;
                    break;
                }
            }
            if (!found) {
                ecc_table[k] = 0; // Safeguard if loop completes without bracketing
            }
        }

        // 7. Remove the DC component so ECC only linearizes shape and does not shift phase
        int32_t dc_sum = 0;
        for (size_t k = 0; k < ECC_TABLE_SIZE; ++k) {
            dc_sum += ecc_table[k];
        }
        int16_t dc_offset = (int16_t)(dc_sum / (int32_t)ECC_TABLE_SIZE); // Compute DC average error

        for (size_t k = 0; k < ECC_TABLE_SIZE; ++k) {
            ecc_table[k] -= dc_offset; // Zero-mean normalization
        }
    } // End of build_ecc_table() method

    /**
     * @brief Corrects encoder position using calibration table
     * @param position Raw encoder position
     * @return Corrected electrical angle mapped to the electrical period
     *
     * Uses interpolation to map real encoder positions to corrected angles,
     * compensating for non-linearities detected during calibration.
     */
    uint16_t correct_el_angle(uint16_t position) const { // Hot path: corrects raw sensor data continuously
        if (cal_size == 0) { // CRITICAL SAFETY: Prevent division by zero and garbage output if uncalibrated
            return 0; // Fallback: default electrical angle to zero
        } // End of safety abort

        // Align position so zero aligns with table's zero-offset point
        uint16_t real_pos = (uint16_t)(position - offst_val); // Shift input into the normalized table domain

        // Estimate starting index by scaling real_pos down. Subtract max_deviation to bracket search correctly.
        size_t idx = ((size_t)(uint16_t)(real_pos - max_deviation) * cal_size) >> 16; // Fast division by 65536 using shift

        // Get initial comparison points to prepare for linear segment search
        uint16_t cal_pos1 = get_val_by_idx(idx); // Fetch calibration value at estimated current index
        uint16_t idl_pos1 = get_ideal(idx, cal_size); // Fetch theoretical ideal value at estimated current index

        uint16_t result = idl_pos1; // Initialize result to the best available estimate

        // Iterate up to 8 steps to find precise segment where real_pos physically falls
        for (int iter = 0; iter < 8; ++iter) { // Search loop with strict deterministic bounded execution
            idx = (idx + 1) % cal_size; // Move to next index circularly
            uint16_t cal_pos2 = get_val_by_idx(idx); // Calibration value at next index boundary
            uint16_t idl_pos2 = get_ideal(idx, cal_size); // Ideal value at next index boundary

            int16_t diff1 = (int16_t)(real_pos - cal_pos1); // Signed difference from left bound
            int16_t diff2 = (int16_t)(real_pos - cal_pos2); // Signed difference from right bound

            // Find boundary where diff changes sign, meaning real_pos is bracketed between cal_pos1 and cal_pos2
            if ((diff1 >= 0) && (diff2 < 0)) { // Verify zero-crossing logic
                result = interpolate(cal_pos1, idl_pos1, cal_pos2, idl_pos2, real_pos); // Compute exact sub-segment position
                break; // Exit loop after finding the correct segment
            } // End of bracketing condition

            // Move to next segment window
            cal_pos1 = cal_pos2; // Left bound calibration becomes old right bound
            idl_pos1 = idl_pos2; // Left bound ideal becomes old right bound
        } // End of bounded segment search

        // Map the mechanical angle into one electrical period. This is the
        // angle the SWEEP commanded when the rotor sat at this encoder value,
        // measured from the sweep's start, in the sweep's own rotation sense.
        // It is deliberately NOT the rotor's electrical angle: the sweep holds
        // the rotor with a q-axis current, so the rotor sits a quarter period
        // ahead of the commanded angle, and the sweep runs in whichever sense
        // makes the encoder count up. Both corrections depend on the detected
        // direction, which this table does not know - the control loop applies
        // them (see TunePulseHeart::tick), so that a motor whose encoder counts
        // up with a positive field rotation and one whose encoder counts down
        // land on the same rotor angle. A fixed quarter-period shift applied
        // here was right for one direction and 180 degrees off for the other,
        // which turned the position loop into positive feedback on every
        // direction = -1 motor.
        return (uint16_t)(((uint32_t)result * (uint32_t)cal_size) / (uint32_t)el_angle_div);
    } // End of correct_el_angle() method

    /**
     * @brief Fast O(1) raw-angle correction using the ECC table
     * @param position Raw absolute encoder position
     * @return Corrected absolute encoder position compensating for magnetic eccentricity
     *
     * The ECC table stores inverse error as a function of raw encoder angle.
     * Linear interpolation between neighboring bins provides a smooth correction.
     */
// Final and most accurate correction version:
uint16_t correct_position(uint16_t position) const {
    if (cal_size == 0) return position;

    // Shift by 11 bits for ECC_TABLE_SIZE = 32 (faster than division)
    uint16_t idx = position >> 11; 
    uint16_t next_idx = (idx + 1) & 31; // Mask for 32 entries
    uint16_t fraction = position & 2047; // Fractional remainder mask

    int16_t offset1 = ecc_table[idx];
    int16_t offset2 = ecc_table[next_idx];

    // Interpolation (shift back by 11 bits)
    int32_t interp_error = offset1 + (((offset2 - offset1) * (int32_t)fraction) >> 11);

    return (uint16_t)(position - interp_error); 
}



private: // Restrict direct access to internal state memory and helper routines
    uint16_t cal_table[CAL_TABLE_CAPACITY]; // Array storing sampled encoder data across full physical calibration cycle
    int16_t ecc_table[ECC_TABLE_SIZE]; // Array storing the compact inverse eccentricity correction table
    size_t cal_size; // Current valid physical number of points actually collected
    size_t el_angle_div; // Scaling divider representing steps forming one electrical period
    size_t offst_idx; // Physical index of first zero-position value discovered after normalization
    uint16_t offst_val; // Absolute offset to align raw calibration data to mathematical zero
    uint16_t max_deviation; // Maximum absolute modular deviation tracked from ideal theoretical values
    size_t temp_idx; // Temporary state index used strictly during sequential data collection checks

    // ---------------------- Flash persistence engine state ----------------------
    enum { MODE_IDLE = 0, MODE_LOAD, MODE_SPACE, MODE_WRITE }; // Transfer phase selector

    FlashEEPROM* flash = nullptr; // Injected EEPROM-emulation driver
    uint8_t f_mode = MODE_IDLE;   // Current transfer phase
    bool load_ok = false;         // Load outcome: usable table restored from flash
    bool store_ok = false;        // Dump outcome: record written completely
    bool f_meta = false;          // Parser currently inside the metadata trailer
    bool f_seq = false;           // Candidate is strictly monotonic so far
    uint32_t f_addr = 0;          // Byte cursor within the logical flash region
    uint32_t f_cnt = 0;           // Items remaining (candidate samples / trailer words)
    uint32_t f_sidx = 0;          // Sample index inside the current candidate
    uint16_t f_prev = 0;          // Previous sample kept for the monotonicity check
    uint32_t f_m[3] = {0, 0, 0};  // Staging buffer for the metadata trailer
    uint32_t f_total = 0;         // Total word count of the record being written
    uint32_t f_wri = 0;           // Byte address of the next 32-byte record
    uint32_t f_word = 0;          // Stream index of the next serialized word
    uint32_t f_free = 0;          // First free byte offset found by the space scan
    bool f_hdr = false;           // Marker word consumed, expecting the count word
    uint32_t f_wri0 = 0;          // Byte address where the current record started

    /**
     * @brief Consumes one scanned word of the stored table chain
     * @param w Raw 32-bit word read from flash
     * @return false when the scan must stop (chain end or corrupted header)
     *
     * Candidates are streamed straight into the RAM arrays; an invalid one is
     * simply never committed, so the newest valid table naturally wins.
     */
    bool scan_word(uint32_t w) {
        if (f_cnt == 0 && !f_meta) {                           // Expecting the two-word header
            if (!f_hdr) {                                      // Word 1: FA3E marker
                if (w == 0xFFFFFFFFul) { f_mode = MODE_IDLE; return false; } // Erased: end of chain
                if ((w & 0xFFFF0000ul) != 0xFA3E0000ul) { f_mode = MODE_IDLE; return false; } // Corrupt
                f_hdr = true;                                  // Marker ok: count word follows
                return true;
            }
            f_hdr = false;                                     // Word 2: sample count
            uint32_t n = w;
            if (n == 0 || n > CAL_TABLE_CAPACITY) { f_mode = MODE_IDLE; return false; } // Bad size
            f_cnt = n; f_sidx = 0; f_seq = true;               // Start collecting samples
            return true;
        }
        if (!f_meta) {                                         // Two packed samples per word
            for (int h = 0; h < 2 && f_sidx < f_cnt; ++h) {
                uint16_t s = (uint16_t)(w >> (16 * h));        // Unpack one half-word sample
                if (f_sidx > 0 && (int16_t)(s - f_prev) <= 0)  // Strict circular monotonicity
                    f_seq = false;                             // Duplicate/backstep: invalid
                cal_table[f_sidx++] = s;                       // Stage the sample in RAM
                f_prev = s;
            }
            if (f_sidx >= f_cnt) { f_meta = true; f_cnt = 3; } // Samples done: expect trailer
            return true;
        }
        f_m[3 - f_cnt] = w;                                    // Stage one metadata word
        if (--f_cnt == 0) {                                    // Trailer complete
            f_meta = false;
            if (f_seq) commit_candidate();                     // Promote only valid candidates
        }
        return true;
    }

    /**
     * @brief Promotes the last staged candidate into the live correction data
     */
    void commit_candidate() {
        el_angle_div = f_m[0] ? f_m[0] : 1;                    // Guard against divide-by-zero
        offst_val = (uint16_t)f_m[1];                          // Restore normalization offset
        max_deviation = (uint16_t)(f_m[1] >> 16);              // Restore deviation base value
        offst_idx = f_m[2] < f_sidx ? f_m[2] : 0;              // Bounds-checked start index
        cal_size = f_sidx;                                     // Adopt the candidate length
        temp_idx = 0;                                          // Reset sequential-fill tracker
        // Rebuild the eccentricity table from the committed state. This is the
        // SINGLE ECC build point: it runs both on the boot-time flash load and on
        // the post-save read-back, so the freshly calibrated live state and the
        // state restored after a power cycle are bit-identical (the ECC table is
        // RAM-only metadata derived deterministically from the record, never
        // stored in flash). The cost is O(cal_size + ECC_TABLE_SIZE * cal_size)
        // once per commit - a few microseconds for the usual ~40-sample table.
        build_ecc_table();
        // Commit diagnostics: printed on EVERY successful load (boot restore AND
        // post-save read-back). Compare the line printed right after a calibration
        // with the line printed at the next boot: identical values prove the
        // running state and the power-cycle state use the same bytes.
        TP_LOG_VALS("CAL: committed size,div,offst,oidx,maxdev",
                    (long)cal_size, (long)el_angle_div, (long)offst_val,
                    (long)offst_idx, (long)max_deviation);
        load_ok = true;                                        // Scan produced a usable table
    }

    /**
     * @brief Serializes one word of the record being written to flash
     * @param i Absolute word index within the record stream
     * @return Serialized 32-bit word number i
     */
    uint32_t stream_word(uint32_t i) const {
        uint32_t sw = (cal_size + 1) / 2;                      // Packed-sample word count
        if (i == 0) return 0xFA3E0000ul;                       // Word 1: FA3E table marker
        if (i == 1) return (uint32_t)cal_size;                 // Word 2: sample count
        if (i <= sw + 1) {                                     // Packed sample pairs
            uint32_t j = (i - 2) * 2;                          // Index of the pair's first sample
            uint32_t hi = (j + 1 < cal_size) ? (uint32_t)cal_table[j + 1] : 0xFFFFul; // Odd-tail pad
            return (uint32_t)cal_table[j] | (hi << 16);        // Two little-endian halves
        }
        i -= sw + 2;                                           // Metadata trailer follows samples
        if (i == 0) return (uint32_t)el_angle_div;
        if (i == 1) return (uint32_t)offst_val | ((uint32_t)max_deviation << 16);
        return (uint32_t)offst_idx;
    }

    /**
     * @brief Retrieves calibration value by index relative to offset
     * @param index Index relative to start_idx
     * @return Calibration value at computed circular index
     */
    uint16_t get_val_by_idx(size_t index) const { // Helper to read from offset-shifted table circularly
        size_t actual_idx = (offst_idx + index) % cal_size; // Compute actual circular index safely (cal_size verified > 0)
        return cal_table[actual_idx]; // Return physical memory value at computed position
    } // End of get_val_by_idx() method

    /**
     * @brief Read-only access to the normalization offset value
     * @return Absolute offset aligning raw calibration data to zero
     */
public:
    uint16_t get_offst_val() const { return offst_val; } // Public getter for the motor record

private:
    /**
     * @brief Computes ideal value for given index in linear progression
     * @param i Index in range
     * @param range Total number of points
     * @return Ideal value assuming linear increase from 0 to UINT16_MAX
     */
    static uint16_t get_ideal(size_t i, size_t range) { // Predict perfect theoretical encoder position
        const uint32_t CAL_VAL_RANGE = UINT16_MAX; // Full 16-bit range scaling multiplier
        return (uint16_t)((CAL_VAL_RANGE * i) / range); // Calculate ideal fractional value based on linear progression
    } // End of get_ideal() method

    /**
     * @brief Calculates absolute deviation from ideal linear progression
     * @param val Actual measured value
     * @param idx Index in calibration table
     * @param size Total size of calibration table
     * @return Absolute deviation from ideal value
     */
    static uint16_t abs_deviation(uint16_t val, size_t idx, size_t size) { // Calculate angular error reliably
        uint16_t ideal = get_ideal(idx, size); // Compute perfectly linear ideal value at this index
        int16_t diff = (int16_t)(ideal - val); // Signed 16-bit cast handles circular angular wrap-around naturally
        return (uint16_t)(diff >= 0 ? diff : -diff); // Return absolute angular shortest-path distance
    } // End of abs_deviation() method

    /**
     * @brief Computes shortest signed circular difference between two uint16_t angles
     * @param target Desired angle
     * @param value Current angle
     * @return Signed shortest-path difference target - value
     */
    static int16_t angle_diff(uint16_t target, uint16_t value) { // Helper for circular signed angle arithmetic
        return (int16_t)(target - value); // Two's-complement wrap gives the shortest signed 16-bit angle difference
    } // End of angle_diff() method

    /**
     * @brief Interpolates position within calibration segment
     * @param a1 Real position at segment start
     * @param a2 Ideal position at segment start
     * @param b1 Real position at segment end
     * @param b2 Ideal position at segment end
     * @param c1 Real position to interpolate
     * @return Interpolated ideal position corresponding to c1
     *
     * Linear interpolation: maps c1 from [a1, b1] range to [a2, b2] range.
     */
    static uint16_t interpolate(uint16_t a1, uint16_t a2, uint16_t b1, uint16_t b2, uint16_t c1) { // Perform precise piecewise linear interpolation
        uint16_t ref_range = (uint16_t)(b1 - a1); // Calculate true reference range in physical domain
        if (ref_range == 0) return a2; // Prevent division-by-zero if encoder is stuck
        uint16_t trgt_range = (uint16_t)(b2 - a2); // Calculate target ideal range
        uint16_t c1_ofst = (uint16_t)(c1 - a1); // Determine fractional offset of c1 within bounded segment
        uint32_t c2_ofst = ((uint32_t)c1_ofst * (uint32_t)trgt_range) / (uint32_t)ref_range; // Project fraction into ideal domain
        return (uint16_t)((uint32_t)a2 + c2_ofst); // Return properly scaled interpolated ideal value
    } // End of interpolate() method
}; // End of CalibrationTable structure