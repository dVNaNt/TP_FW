#pragma once  // Ensure this header is included only once during compilation

// Implements a double-buffering system for storing ADC and sensor data,
// ensuring data consistency and minimizing synchronization overhead.
// This module allows safe and efficient data capture from interrupt routines
// while allowing the main loop or other tasks to access fully updated data.
//
// Key Features:
// - Implements a double-buffering system for ADC and sensor data.
// - Ensures each buffer is fully updated before it becomes available to reading tasks.
// - Minimizes synchronization overhead by using a lock bit only during brief data reads.
// - Allows interrupt routines to safely capture data snapshots without partial updates.
// - Guarantees that fetched data is complete, up-to-date, and consistent.
//
// Detailed Operation:
// The module uses two `DataInputs` buffers and a set of flags to manage data updates and reads.
// Each buffer can be in one of two states: being updated or ready for reading.
// A bitmask tracks the completion of each data field within a buffer.
// When all mandatory fields in a buffer are filled, the buffer is marked as ready.
// The `get_data()` function acquires a lock on the ready buffer to prevent modifications
// during the read operation, ensuring data consistency without requiring heavy synchronization.
// This approach allows interrupt routines to update data swiftly while the main loop can
// read complete and coherent data sets efficiently.
//
// Licensed under the Apache License, Version 2.0
// Copyright 2024 Anton Khrustalev, creapunk.com

#include "../common.h"  // Include common migration definitions and types

/// Data structure holding various ADC readings and raw angle measurements.
struct DataInputs {  // Define the DataInputs structure for sensor data
    uint16_t supply_adc;  // Supply ADC reading (0..65535)
    uint16_t temper_adc;  // Temperature ADC reading (0..65535)
    uint16_t currnt_adc[3];  // Current ADC readings for 3 channels (0..65535 each)
    uint16_t angle_raw;  // Raw angle measurement (0..65535)
    
    // Default constructor: Initialize all fields to zero
    DataInputs() : supply_adc(0), temper_adc(0), currnt_adc{0, 0, 0}, angle_raw(0) {}  // Constructor: Create with zero values
};  // End of DataInputs structure definition

/// Enum defining bit masks for each data field and a lock bit.
/// Each variant represents a specific field in the `DataInputs` struct.
/// The `LOCK` variant is used to prevent modifications during data reads.
enum DataInputsBit : uint32_t {  // Define bit masks for data field tracking
    DataInputsBit_SUPPLY = 1u << 0,  // Mask for supply ADC field bit
    DataInputsBit_TEMP = 1u << 1,  // Mask for temperature ADC field bit
    DataInputsBit_CURRENT = 1u << 2,  // Mask for current ADC array field bit
    DataInputsBit_ANGLE = 1u << 3,  // Mask for angle field bit
    DataInputsBit_LOCK = 1u << 31  // Mask for lock bit at MSB
};  // End of DataInputsBit enumeration

/// Structure for managing two buffers of `DataInputs` and related flags.
/// Utilizes double-buffering to ensure data consistency and minimize synchronization overhead.
struct InputsDump {  // Define the InputsDump structure for double-buffered sensor data
private:
    // ---------------------------------------------------------------------------
    // Private data members
    // ---------------------------------------------------------------------------
    
    DataInputs buffers_[2];  // Two buffers: one being updated, one ready for reading
    size_t idx2update_;  // Index of the buffer currently being updated (0 or 1)
    uint32_t flags_[2];  // Flags array holding field completion and lock status for each buffer
    size_t iter_;  // Iteration counter to track updates
    size_t prev_iter_;  // Previous iteration counter to detect updates
    uint32_t mandatory_fields_;  // Mandatory fields mask (bitmask of required fields)
    
    // ---------------------------------------------------------------------------
    // Private helper methods
    // ---------------------------------------------------------------------------
    
    // Checks if the given buffer index is ready (no fields pending)
    inline bool is_ready(size_t idx) const {  // Check if buffer is completely filled
        return flags_[idx] == 0;  // If flags are 0, the buffer is ready
    }
    
    // Gets the opposite buffer index (if idx=0 return 1, if idx=1 return 0)
    inline size_t get_opposite(size_t idx) const {  // Get opposite buffer index
        return 1U - idx;  // Toggle between 0 and 1
    }
    
    // Clears a particular field bit in the flags for the specified buffer
    inline void clear_field_bit(size_t idx, DataInputsBit bit) {  // Mark field as filled
        flags_[idx] &= ~(uint32_t)bit;  // Use NOT mask to clear the bit
    }
    
    // Checks if all fields of the current buffer are filled; if both buffers are ready,
    // reinitializes one for updating
    inline void check_fill(size_t idx) {  // Check and switch buffers if needed
        // If both buffers are ready (no pending fields),
        // re-initialize the opposite buffer for new data collection
        if (is_ready(0) && is_ready(1)) {  // Both buffers complete
            size_t next_idx = get_opposite(idx);  // Get the opposite buffer index
            flags_[next_idx] = mandatory_fields_;  // Set the opposite buffer to all fields pending
            idx2update_ = next_idx;  // Switch to updating the opposite buffer
            iter_ = iter_ + 1;  // Increment iteration counter
        }
    }
    
    // Checks if the data has been updated since the last read
    // PRIVATE: Only accessible through Feedback structure
    inline bool is_updated() const {  // Check for new data availability
        return iter_ != prev_iter_;  // Returns true if there has been an update
    }
    
    // Gets a fully updated `DataInputs` from the opposite buffer.
    // This method briefly sets a lock bit to prevent modifications during reading.
    // PRIVATE: Only accessible through Feedback structure
    inline DataInputs get_data() {  // Retrieve complete sensor data snapshot
        size_t ready_idx = get_opposite(idx2update_);  // Get the opposite buffer which should be ready
        flags_[ready_idx] |= (uint32_t)DataInputsBit_LOCK;  // Set the lock bit on the ready buffer
        DataInputs data = buffers_[ready_idx];  // Copy the data from the locked buffer
        prev_iter_ = iter_;  // Update the previous iteration counter
        flags_[ready_idx] &= ~(uint32_t)DataInputsBit_LOCK;  // Clear the lock bit after reading
        return data;  // Return the copied data
    }
    
    // Grant Feedback structure access to private methods
    friend struct Feedback;
    
public:
    // Constructor: Creates a new InputsDump with both buffers cleared and ready to be filled
    // mandatory_fields: Bitmask of required fields that must be filled before buffer is ready
    InputsDump(uint32_t mandatory_fields)  // Constructor: Initialize double-buffer system
        : buffers_{DataInputs(), DataInputs()},  // Initialize both buffers to default
          idx2update_(0),  // Start updating buffer 0
          flags_{mandatory_fields, 0},  // Buffer 0: all fields pending, Buffer 1: ready
          iter_(0),  // Initialize iteration counter
          prev_iter_(0),  // Initialize previous iteration counter
          mandatory_fields_(mandatory_fields)  // Store mandatory fields mask
    {
    }
    
    // Sets the `supply_adc` field in the currently updating buffer
    inline void set_supply_adc(uint16_t value) {  // Store supply ADC value
        size_t idx = idx2update_;  // Get the currently updating buffer index
        buffers_[idx].supply_adc = value;  // Store the supply ADC value
        clear_field_bit(idx, DataInputsBit_SUPPLY);  // Mark the supply field as filled
        check_fill(idx);  // Check if buffer filling is complete or if we need to switch
    }
    
    // Sets the `temper_adc` field in the currently updating buffer
    inline void set_temper_adc(uint16_t value) {  // Store temperature ADC value
        size_t idx = idx2update_;  // Get the currently updating buffer index
        buffers_[idx].temper_adc = value;  // Store the temperature ADC value
        clear_field_bit(idx, DataInputsBit_TEMP);  // Mark the temperature field as filled
        check_fill(idx);  // Check if buffer filling is complete or if we need to switch
    }
    
    // Sets the `currnt_adc` field in the currently updating buffer
    inline void set_current_adc(const uint16_t values[3]) {  // Store current ADC values
        size_t idx = idx2update_;  // Get the currently updating buffer index
        for (size_t i = 0; i < 3; ++i) {  // Copy all three current values
            buffers_[idx].currnt_adc[i] = values[i];  // Store each current ADC value
        }
        clear_field_bit(idx, DataInputsBit_CURRENT);  // Mark the current field as filled
        check_fill(idx);  // Check if buffer filling is complete or if we need to switch
    }
    
    // Sets the `angle_raw` field in the currently updating buffer
    inline void set_angle_raw(uint16_t value) {  // Store raw angle value
        size_t idx = idx2update_;  // Get the currently updating buffer index
        buffers_[idx].angle_raw = value;  // Store the angle data
        clear_field_bit(idx, DataInputsBit_ANGLE);  // Mark the angle field as filled
        check_fill(idx);  // Check if buffer filling is complete or if we need to switch
    }
};  // End of InputsDump structure definition



