#pragma once  // Ensure this header is included only once during compilation


// Implements a double-buffering system for storing ADC and sensor data,
// ensuring data consistency through atomic operations.
// This module allows safe and efficient data capture from interrupt routines
// while allowing the main loop or other tasks to access fully updated data.
//
// Key Features:
// - Implements a double-buffering system for ADC and sensor data.
// - Uses atomic operations to prevent race conditions on STM32.
// - Ensures each buffer is fully updated before it becomes available to reading tasks.
// - Minimizes synchronization overhead by using atomic flags and lock bit.
// - Allows interrupt routines to safely capture data snapshots without partial updates.
// - Guarantees that fetched data is complete, up-to-date, and consistent.
//
// Licensed under the Apache License, Version 2.0
// Copyright 2024 Anton Khrustalev, creapunk.com


#include "../common.h"  // Include common migration definitions and types
#include <atomic>  // C++ atomic operations support


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
    DataInputsBit_LOCK = 1u << 31,  // Mask for lock bit at MSB
    DataInputsBit_ALL_MASK = 0x0000000Fu  // Mask for all data fields (bits 0-3)
};  // End of DataInputsBit enumeration


/// Structure for managing two buffers of `DataInputs` and related flags.
/// Utilizes double-buffering with atomic operations to ensure data consistency.
struct InputsDump {  // Define the InputsDump structure for double-buffered sensor data
private:
    // ---------------------------------------------------------------------------
    // Private data members
    // ---------------------------------------------------------------------------
    
    DataInputs buffers_[2];  // Two buffers: one being updated, one ready for reading
    std::atomic<uint32_t> idx2update_;  // Index of the buffer currently being updated (0 or 1) - ATOMIC
    std::atomic<uint32_t> flags_[2];  // Flags array holding field completion and lock status for each buffer - ATOMIC
    std::atomic<uint32_t> iter_;  // Iteration counter to track updates - ATOMIC
    uint32_t prev_iter_;  // Previous iteration counter (only read in main loop, no sync needed)
    std::atomic<uint32_t> mandatory_fields_;  // Mandatory fields mask - ATOMIC
    
    // ---------------------------------------------------------------------------
    // Private helper methods
    // ---------------------------------------------------------------------------
    
    // Checks if the given buffer is ready (no data fields pending, ignoring lock bit)
    inline bool is_ready(uint32_t flags_snapshot) const {  // Check if buffer is completely filled
        return (flags_snapshot & DataInputsBit_ALL_MASK) == 0;  // If data field bits are 0, the buffer is ready
    }
    
    // Gets the opposite buffer index (if idx=0 return 1, if idx=1 return 0)
    inline uint32_t get_opposite(uint32_t idx) const {  // Get opposite buffer index
        return 1U - idx;  // Toggle between 0 and 1
    }
    
    // Atomically clears a particular field bit in the flags for the specified buffer
    // Returns the new flags value after clearing
    inline uint32_t atomic_clear_field_bit(uint32_t idx, DataInputsBit bit) {  // Mark field as filled atomically
        return flags_[idx].fetch_and(~(uint32_t)bit) & ~(uint32_t)bit;  // Clear bit and return new value
    }
    
    // Checks if all fields of the current buffer are filled; if both buffers are ready,
    // reinitializes one for updating. This is done atomically.
    inline void check_fill_and_switch(uint32_t idx, uint32_t new_flags) {  // Check and switch buffers if needed
        // If current buffer is now ready (all mandatory fields filled)
        if (is_ready(new_flags)) {  // Current buffer complete
            uint32_t next_idx = get_opposite(idx);  // Get the opposite buffer index
            
            // Atomically load the opposite buffer flags
            uint32_t opposite_flags = flags_[next_idx].load(std::memory_order_acquire);
            
            // Check if opposite buffer is ready AND not locked
            if (is_ready(opposite_flags) && !(opposite_flags & DataInputsBit_LOCK)) {  // Both buffers complete and not locked
                // Try to atomically reinitialize the opposite buffer
                uint32_t mandatory = mandatory_fields_.load(std::memory_order_relaxed);
                
                // Use compare-and-swap to ensure atomic transition
                // Only reinitialize if opposite buffer is still ready and not locked
                uint32_t expected = opposite_flags;
                if (flags_[next_idx].compare_exchange_strong(expected, mandatory,
                                                             std::memory_order_release, std::memory_order_relaxed)) {
                    // Successfully reinitialized opposite buffer, now switch to it
                    idx2update_.store(next_idx, std::memory_order_release);  // Switch to updating the opposite buffer
                    iter_.fetch_add(1, std::memory_order_release);  // Increment iteration counter
                }
            }
        }
    }
    
    // Checks if the data has been updated since the last read
    // PRIVATE: Only accessible through Feedback structure
    inline bool is_updated() const {  // Check for new data availability
        uint32_t current_iter = iter_.load(std::memory_order_acquire);
        return current_iter != prev_iter_;  // Returns true if there has been an update
    }
    
    // Gets a fully updated `DataInputs` from the ready buffer.
    // This method atomically sets a lock bit to prevent modifications during reading.
    // PRIVATE: Only accessible through Feedback structure
    inline DataInputs get_data() {  // Retrieve complete sensor data snapshot
        uint32_t current_idx = idx2update_.load(std::memory_order_acquire);
        uint32_t ready_idx = get_opposite(current_idx);  // Get the opposite buffer which should be ready
        
        // Atomically set the lock bit on the ready buffer
        uint32_t old_flags = flags_[ready_idx].fetch_or((uint32_t)DataInputsBit_LOCK, std::memory_order_acquire);
        
        // Read the data with memory fence to ensure visibility
        std::atomic_thread_fence(std::memory_order_acquire);  // Ensure all data writes are visible
        DataInputs data = buffers_[ready_idx];  // Copy the data from the locked buffer
        std::atomic_thread_fence(std::memory_order_release);  // Ensure data copy completes
        
        // Update previous iteration counter
        prev_iter_ = iter_.load(std::memory_order_acquire);
        
        // Atomically clear the lock bit after reading
        flags_[ready_idx].fetch_and(~(uint32_t)DataInputsBit_LOCK, std::memory_order_release);
        
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
        // Ensure atomic initialization is visible
        std::atomic_thread_fence(std::memory_order_release);
    }
    
    // Sets the `supply_adc` field in the currently updating buffer
    inline void set_supply_adc(uint16_t value) {  // Store supply ADC value
        uint32_t idx = idx2update_.load(std::memory_order_acquire);  // Get current buffer index atomically
        buffers_[idx].supply_adc = value;  // Store the supply ADC value
        std::atomic_thread_fence(std::memory_order_release);  // Ensure data write completes before flag update
        uint32_t new_flags = atomic_clear_field_bit(idx, DataInputsBit_SUPPLY);  // Mark the supply field as filled
        check_fill_and_switch(idx, new_flags);  // Check if buffer filling is complete
    }
    
    // Sets the `temper_adc` field in the currently updating buffer
    inline void set_temper_adc(uint16_t value) {  // Store temperature ADC value
        uint32_t idx = idx2update_.load(std::memory_order_acquire);  // Get current buffer index atomically
        buffers_[idx].temper_adc = value;  // Store the temperature ADC value
        std::atomic_thread_fence(std::memory_order_release);  // Ensure data write completes before flag update
        uint32_t new_flags = atomic_clear_field_bit(idx, DataInputsBit_TEMP);  // Mark the temperature field as filled
        check_fill_and_switch(idx, new_flags);  // Check if buffer filling is complete
    }
    
    // Sets the `currnt_adc` field in the currently updating buffer
    inline void set_current_adc(const uint16_t values[3]) {  // Store current ADC values
        uint32_t idx = idx2update_.load(std::memory_order_acquire);  // Get current buffer index atomically
        for (size_t i = 0; i < 3; ++i) {  // Copy all three current values
            buffers_[idx].currnt_adc[i] = values[i];  // Store each current ADC value
        }
        std::atomic_thread_fence(std::memory_order_release);  // Ensure data write completes before flag update
        uint32_t new_flags = atomic_clear_field_bit(idx, DataInputsBit_CURRENT);  // Mark the current field as filled
        check_fill_and_switch(idx, new_flags);  // Check if buffer filling is complete
    }
    
    // Sets the `angle_raw` field in the currently updating buffer
    inline void set_angle_raw(uint16_t value) {  // Store raw angle value
        uint32_t idx = idx2update_.load(std::memory_order_acquire);  // Get current buffer index atomically
        buffers_[idx].angle_raw = value;  // Store the angle data
        std::atomic_thread_fence(std::memory_order_release);  // Ensure data write completes before flag update
        uint32_t new_flags = atomic_clear_field_bit(idx, DataInputsBit_ANGLE);  // Mark the angle field as filled
        check_fill_and_switch(idx, new_flags);  // Check if buffer filling is complete
    }

    // Atomically updates the mandatory fields mask
    // Should be called when system is not actively updating buffers
    inline void set_mandatory(uint32_t mandatory_fields) {
        mandatory_fields_.store(mandatory_fields, std::memory_order_release);
    }
};  // End of InputsDump structure definition
