/**
 * @file anti_cogging.h
 * @brief Anti-Cogging calibration and compensation module
 * 
 * Implements a feed-forward compensation table for motor cogging torque.
 * Collects effort (Iq current) required to hold specific electrical angles,
 * averages forward and backward passes to eliminate Coulomb friction,
 * and provides fast, hardware-independent linear interpolation for real-time FOC.
 * 
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#pragma once // Prevent multiple compiler inclusions of this header file

#include "../../common.h" // Include common project definitions, utilities, and logging macros

#ifndef COG_TABLE_CAPACITY // Check if COG_TABLE_CAPACITY is defined externally by user configuration
#define COG_TABLE_CAPACITY 32 // Default to 128 points per electrical period (optimal balance of RAM and resolution)
#endif // End of COG_TABLE_CAPACITY definition block

/**
 * @brief Calibration table storing cogging torque compensation data
 * 
 * Stores Iq current offsets mapped to electrical angles and provides
 * fast integer-based interpolation for real-time compensation.
 */
struct CoggingTable { // Begin declaration of the CoggingTable structure
public: // Make structural methods publicly accessible
    /**
     * @brief Constructs a new initialized CoggingTable
     * @return Initialized CoggingTable instance
     */
    static CoggingTable create() { // Factory method to instantiate and initialize the table memory safely
        CoggingTable t; // Allocate a new table instance on stack
        t.cal_size = 0; // Initialize valid recorded points count to zero
        t.is_valid = false; // Mark table as invalid until fully calibrated and normalized
        for (size_t i = 0; i < COG_TABLE_CAPACITY; ++i) { // Iterate over entire allocated array bounds
            t.table[i] = 0; // Clear all memory slots to prevent undefined behavior
        } // End of initialization loop
        return t; // Return the fully zeroed instance
    } // End of create() method

    /**
     * @brief Resets the table for a fresh calibration run
     */
    void reset() { // Clear internal state tracking without redundant memory zeroing
        this->cal_size = 0; // Reset valid element counter
        this->is_valid = false; // Block compensation output until new calibration finishes
    } // End of reset() method

    /**
     * @brief Stores Iq effort from the forward rotation pass
     * @param idx Table index corresponding to electrical angle
     * @param iq_val Averaged Iq current required to hold position
     * @return true if successful, false if out of bounds
     */
    bool fill_first(size_t idx, int16_t iq_val) { // Record the first mechanical pass (contains cogging + friction)
        if (idx < COG_TABLE_CAPACITY) { // Strictly check array boundaries to prevent memory corruption
            table[idx] = iq_val; // Store the measured current effort directly
            if (cal_size < idx + 1) { // Check if we are expanding the known active range
                cal_size = idx + 1; // Safely increment the internal tracker of valid points
            } // End of size update condition
            return true; // Indicate successful storage to caller
        } // End of bounds check
        return false; // Reject sample gracefully if index is invalid
    } // End of fill_first() method

    /**
     * @brief Averages backward rotation pass with forward pass
     * @param idx Table index corresponding to electrical angle
     * @param iq_val Averaged Iq current from backward rotation
     * @return true if successful, false if out of bounds
     */
    bool fill_second(size_t idx, int16_t iq_val) { // Record reverse pass to cancel out directional friction hysteresis
        if (idx < COG_TABLE_CAPACITY) { // Ensure memory safety during array access
            // Average = (Forward + Backward) / 2. Upcast to int32_t prevents 16-bit overflow during addition!
            int32_t avg = ((int32_t)table[idx] + (int32_t)iq_val) / 2; // Compute pure cogging torque by canceling friction
            table[idx] = (int16_t)avg; // Downcast safely back to 16-bit storage format
            return true; // Return success status
        } // End of bounds check
        return false; // Reject sample if index is out of bounds
    } // End of fill_second() method

    /**
     * @brief Removes DC offset from collected table
     * @return true if normalization succeeds
     */
    bool normalize() { // Post-processing step to isolate AC ripple from continuous DC holding torque
        if (cal_size == 0) return false; // CRITICAL: Prevent division by zero if called prematurely

        int32_t sum = 0; // Initialize 32-bit accumulator to prevent summing overflow
        for (size_t i = 0; i < cal_size; ++i) { // Loop through all valid collected samples
            sum += table[i]; // Accumulate total effort across the entire electrical period
        } // End of summation loop

        int16_t dc_offset = (int16_t)(sum / (int32_t)cal_size); // Calculate mathematical mean (average friction/load)

        TP_LOG_VAL("COG_TABLE: Normalizing DC offset: ", dc_offset); // Log the subtracted DC offset for debugging

        for (size_t i = 0; i < cal_size; ++i) { // Second pass to remove the constant bias
            table[i] -= dc_offset; // Shift waveform so the integral over one period equals exactly zero
        } // End of normalization loop

        is_valid = true; // Mark the lookup table as mathematically sound and ready for real-time usage
        return true; // Return success flag
    } // End of normalize() method

    /**
     * @brief Gets interpolated compensation effort for given angle
     * @param el_angle Current electrical angle (0-65535)
     * @return Feed-forward Iq compensation value
     */
    int16_t get_compensation(uint16_t el_angle) const { // Fast path function called every FOC tick (must be highly optimized)
        if (!is_valid || cal_size == 0) return 0; // Immediate safe fallback if calibration is incomplete

        // Map 16-bit angle (0-65535) to array indices purely using bitwise math (avoids slow divisions)
        uint32_t scaled = ((uint32_t)el_angle * cal_size); // Scale angle by table size. Max value: 65535 * 256 = ~16.7M (fits in uint32_t)
        
        size_t idx1 = scaled >> 16; // Extract integer index by dividing by 65536 using fast bit shift
        size_t idx2 = (idx1 + 1) % cal_size; // Determine next index, wrapping around cleanly via modulo
        
        uint16_t remainder = (uint16_t)(scaled & 0xFFFF); // Extract lower 16 bits as the fractional interpolation factor

        int32_t val1 = table[idx1]; // Fetch lower bound value from table
        int32_t val2 = table[idx2]; // Fetch upper bound value from table
        
        // Fast linear interpolation: Base + (Difference * Fraction) / 65536
        int32_t interp = val1 + (((val2 - val1) * (int32_t)remainder) >> 16); // Compute exact effort using bit shift scaling

        return (int16_t)interp; // Return computed compensation current
    } // End of get_compensation() method

private: // Hide internal structural data
    int16_t table[COG_TABLE_CAPACITY]; // Static array holding actual effort values mapping to mechanical layout
    size_t cal_size; // Tracks number of effectively populated points in array
    bool is_valid; // Safety flag preventing uncalibrated data injection into FOC loop
}; // End of CoggingTable structure

// -----------------------------------------------------------------------------

/**
 * @brief Cogging calibration process stage enumeration
 * Flatted state machine for direct Step -> Settle -> Sample logic
 */
enum CoggingCalStage { // Single unified state machine for clarity and precision
    CoggingStage_Setup = 0,      // Initialize system variables and clear table
    CoggingStage_MoveToNext = 1, // Command position PI to move to the next angular target
    CoggingStage_Settling = 2,   // Wait strictly for physical rotor inertia to settle
    CoggingStage_Sampling = 3,   // Accumulate PI output effort to find average holding current
    CoggingStage_Check = 4,      // Finalize table, merge directions and normalize DC offset
    CoggingStage_Ready = 5,      // Calibration successful, ready for real-time compensation
    CoggingStage_Error = 6,      // Calibration failed, data rejected
}; // End of CoggingCalStage enum

/**
 * @brief Anti-Cogging configuration parameters
 */
enum { // Configuration block utilizing enum constants for compile-time optimization
    COG_CAL_SETTLING_TIME_US = 5000, // Settling time strictly set to 5ms per step as requested
    COG_CAL_OVERSAMPLING = 128       // Number of sequential control loops to average the target current
}; // End of configuration enum

/**
 * @brief Anti-Cogging Calibrator state machine
 * 
 * Dictates rigid position targets to an external position PI controller
 * and samples the resulting Iq effort to build the compensation table.
 */
struct AntiCoggingCalibrator { // Begin state machine structure definition
public: // Make external interfaces public
    /**
     * @brief Creates a new calibrator
     * @param freq Target loop frequency in Hz
     */
    AntiCoggingCalibrator(uint16_t freq) { // Constructor computing real-time tick dependencies
        frequency = freq; // Store the system control loop execution rate
        settling_time_ticks = (size_t)((frequency * COG_CAL_SETTLING_TIME_US) / 1000000u); // Calculate ticks required to wait exactly 5ms
        cog_table = CoggingTable::create(); // Instantiate and zero out the internal lookup table
    } // End of constructor

    /**
     * @brief Main flat state machine tick
     * @param current_iq The current effort output by the position PI controller
     * @return Target electrical angle FOC must forcefully track
     */
    uint16_t tick(int16_t current_iq) { // Hot path executed every single FOC loop during calibration
        switch (stage) { // Route logic based on the current explicit step of the process
            
            case CoggingStage_Setup: // Initial boot configuration
                TP_LOG("COG_CAL: Starting Anti-Cogging calibration"); // Alert user interface via logging
                cog_table.reset(); // Purge old data from table safely
                cal_idx = 0; // Start at physical array index zero
                is_reverse = false; // Start with forward mechanical rotation
                stage = CoggingStage_MoveToNext; // Proceed directly to generating the first movement command
                break; // Exit switch block

            case CoggingStage_MoveToNext: // Generate positional target
                target_el_angle = get_angle_for_idx(cal_idx); // Calculate the exact 16-bit angle corresponding to current index
                time_in_state = settling_time_ticks; // Prime the timer with 5ms tick equivalent
                stage = CoggingStage_Settling; // Transition state machine into waiting mode
                break; // Exit switch block

            case CoggingStage_Settling: // Wait for 5ms without changing ANYTHING
                // During this 5ms window, target_el_angle remains perfectly constant allowing P-controller to stabilize
                if (iter(&time_in_state)) { // Check if the 5ms timer has completely expired
                    time_in_state = COG_CAL_OVERSAMPLING; // Prime the timer for the data accumulation phase
                    oversampled_iq = 0; // Strictly clear the accumulator before starting to sample current
                    stage = CoggingStage_Sampling; // Transition into measurement mode
                } // End of expiration logic
                break; // Exit switch block

            case CoggingStage_Sampling: // Average the target holding current over defined period
                oversampled_iq += current_iq; // Unconditionally accumulate the instantaneous effort required to hold position
                
                if (iter(&time_in_state)) { // Check if we have collected enough samples (128 ticks)
                    int16_t avg_iq = (int16_t)(oversampled_iq / (int32_t)COG_CAL_OVERSAMPLING); // Calculate the exact mathematical average current
                    
                    if (!is_reverse) { // We are currently executing the forward pass
                        cog_table.fill_first(cal_idx, avg_iq); // Record the averaged effort into the forward dataset
                        cal_idx++; // Move to the next physical step
                        
                        if (cal_idx >= COG_TABLE_CAPACITY) { // Check if we reached the end of the electrical period
                            TP_LOG("COG_CAL: Forward pass done, starting reverse"); // Log transition
                            is_reverse = true; // Flip the direction flag for hysteresis cancellation
                            cal_idx = COG_TABLE_CAPACITY - 1; // Snap index back to the end to traverse backwards
                        } // End of forward boundary check
                    } else { // We are currently executing the backward pass
                        cog_table.fill_second(cal_idx, avg_iq); // Mathematically merge the backward current with the forward one
                        
                        if (cal_idx == 0) { // Check if we returned to the absolute origin
                            TP_LOG("COG_CAL: Reverse pass done, validating"); // Log completion
                            stage = CoggingStage_Check; // Transition to final mathematical processing
                            break; // Skip MoveToNext to avoid redundant commands
                        } // End of backward boundary check
                        cal_idx--; // Decrement physical step backwards
                    } // End of direction logic

                    stage = CoggingStage_MoveToNext; // Command the state machine to generate the next angle
                } // End of accumulation window check
                break; // Exit switch block

            case CoggingStage_Check: // Final normalization and mathematical validation
                if (cog_table.normalize()) { // Trigger DC offset elimination (removes average friction)
                    TP_LOG("COG_CAL: Table successfully normalized and ready!"); // Confirm success to user
                    stage = CoggingStage_Ready; // Lock state into operational mode
                } else { // Normalization failed due to empty table
                    TP_LOG("COG_CAL: ERROR - Normalization failed!"); // Report mathematical error
                    stage = CoggingStage_Error; // Lock into fault state
                } // End of validation logic
                break; // Exit switch block

            case CoggingStage_Ready: // Calibration done, normal operation active
            case CoggingStage_Error: // Calibration failed, system halted
            default: // Catch corrupted memory state
                break; // Do absolutely nothing, just return the target angle
        } // End of unified state machine switch

        return target_el_angle; // Continuously feed the rigid target position to the external P/PI controller
    } // End of tick() method

    /**
     * @brief Checks if calibration is fully complete
     * @return true if anti-cogging table can be used
     */
    bool is_ready() const { // External query to verify module availability
        return stage == CoggingStage_Ready; // Return true only if post-processing passed
    } // End of is_ready() method

    /**
     * @brief Get compensated feed-forward effort
     * @param el_angle Current electrical angle
     * @return Compensation current to add to Iq
     */
    int16_t get_compensation(uint16_t el_angle) const { // Direct pass-through wrapper for real-time operation
        return cog_table.get_compensation(el_angle); // Relay processing to table interpolator
    } // End of get_compensation() wrapper

private: // Hide internal state machine mechanics
    uint16_t frequency; // Loop execution frequency
    size_t settling_time_ticks; // Pre-calculated ticks representing 5ms physical settling time

    CoggingCalStage stage = CoggingStage_Setup; // Active macro calibration state tracker
    
    bool is_reverse = false; // Direction flag: false = forward, true = backward traversal
    uint16_t target_el_angle = 0; // Active constant setpoint angle for external position P-loop
    size_t cal_idx = 0; // Current operational array index corresponding to electrical angle
    
    size_t time_in_state = 0; // Generic countdown counter for tick-based delay scheduling
    int32_t oversampled_iq = 0; // Massive 32-bit accumulator for noise-free current averaging

    CoggingTable cog_table; // Instantiated internal memory lookup table

    /**
     * @brief Decrements a counter returning true on zero
     */
    static bool iter(size_t* counter) { // Helper function for lock-free, non-blocking time delays
        if (*counter > 0) { // Verify if waiting time still remains
            *counter -= 1; // Safely decrement the ticks remaining
            return false; // Indicate the delay is still actively running
        } // End of time check
        return true; // Indicate timer expiration (time reached zero)
    } // End of iter() helper

    /**
     * @brief Helper to convert array index to purely mapped electrical angle
     */
    static uint16_t get_angle_for_idx(size_t idx) { // Calculate angular setpoint cleanly avoiding division
        // Multiply index by max 16-bit value, divide by array size
        return (uint16_t)(((uint32_t)idx * 65536u) / COG_TABLE_CAPACITY); // Safe 32-bit transformation math perfectly scaling 0-65535
    } // End of get_angle_for_idx() helper
}; // End of AntiCoggingCalibrator structure
