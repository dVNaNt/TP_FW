// #pragma once  // Ensure this header is included only once during compilation

// // Implements a high-performance double-buffering system for ADC and sensor data.
// // Designed for safe, lock-free (from the ISR side) data capture in embedded systems.
// // Ensures data consistency between high-frequency interrupts (e.g., FOC loops) 
// // and lower priority tasks (e.g., main loop) without disabling interrupts.
// //
// // Key Features:
// // - Atomic operations replace blocking mutexes to prevent priority inversion.
// // - Optimized memory ordering (acquire/release) minimizes pipeline stalls on ARM Cortex-M.
// // - Assures complete data snapshots (no torn reads or partial updates).
// //
// // Licensed under the Apache License, Version 2.0
// // Copyright 2024 Anton Khrustalev, creapunk.com

// #include "../common.h"  // Include common migration definitions and types
// #include <atomic>       // C++ atomic operations support

// /// Data structure holding various ADC readings and raw angle measurements.
// /// OPTIMIZATION: 32-bit variables are placed first to eliminate internal compiler padding.
// /// Total size is exactly 20 bytes (perfectly aligned to 4-byte boundaries).
// struct DataInputs {
//     uint32_t clock;
//     uint32_t angle_raw;      // Raw encoder/sensor angle measurement (full 32-bit)
//     uint32_t angle_ref;      // Reference angle measurement (full 32-bit)
//     uint16_t currnt_adc[3];  // Phase current ADC readings (0..65535 each)
//     uint16_t supply_adc;     // Supply voltage ADC reading (0..65535)
//     uint16_t temper_adc;     // Temperature ADC reading (0..65535)
    
//     // Default constructor: Zero-initializes all fields
//     DataInputs() : clock(0), angle_raw(0), angle_ref(0), currnt_adc{0, 0, 0}, supply_adc(0), temper_adc(0) {}
// };

// /// Enum defining bit masks for each data field and a lock bit.
// /// Each bit tracks whether a specific measurement has been stored in the current buffer.
// enum DataInputsBit : uint32_t {
//     DataInputsBit_SUPPLY    = 1u << 0,  // Supply ADC completed flag
//     DataInputsBit_TEMP      = 1u << 1,  // Temperature ADC completed flag
//     DataInputsBit_CURRENT   = 1u << 2,  // Current ADC array completed flag
//     DataInputsBit_ANGLE     = 1u << 3,  // Angle measurement completed flag
//     DataInputsBit_REF_ANGLE = 1u << 4,  // Reference angle measurement completed flag
//     DataInputsBit_CLOCK     = 1u << 5,  // Clock/Timestamp completed flag
//     DataInputsBit_LOCK      = 1u << 31, // Reader lock flag (prevents writer from overwriting)
//     DataInputsBit_ALL_MASK  = 0x0000003Fu // Mask covering all 6 sensor data fields
// };

// /// Double-buffered container ensuring synchronized reads/writes across execution contexts.
// struct InputsDump {
// private:
//    // ---------------------------------------------------------------------------
//     // Private data members
//     // ---------------------------------------------------------------------------
    
//     DataInputs buffers_[2];                 // Dual data buffers: writer fills one, reader reads the other
//     std::atomic<uint32_t> idx2update_;      // Active buffer index for the writer (ISR)
//     std::atomic<uint32_t> flags_[2];        // Bitfield for completion and lock status per buffer
//     std::atomic<uint32_t> iter_;            // Global iteration counter to detect fresh data
//     uint32_t prev_iter_;                    // Reader's local copy of iteration counter (no atomics needed)
//     std::atomic<uint32_t> mandatory_fields_;// Bitmask configuring which fields constitute a "complete" dump
    
//     // ---------------------------------------------------------------------------
//     // Private helper methods
//     // ---------------------------------------------------------------------------
    
//     // Checks if the buffer has gathered all mandatory data (ignores the LOCK bit)
//     inline bool is_ready(uint32_t flags_snapshot) const {
//         return (flags_snapshot & DataInputsBit_ALL_MASK) == 0;
//     }
    
//     // Flips the buffer index (0 -> 1, 1 -> 0)
//     inline uint32_t get_opposite(uint32_t idx) const {
//         return 1U - idx;
//     }
    
//     // Clears a specific field bit in the flags atomically.
//     // Uses memory_order_release to ensure data writes to buffers_[idx] are 
//     // committed to memory BEFORE the flag registers as cleared.
//     inline uint32_t atomic_clear_field_bit(uint32_t idx, DataInputsBit bit) {
//         return flags_[idx].fetch_and(~static_cast<uint32_t>(bit), std::memory_order_release) & ~static_cast<uint32_t>(bit);
//     }
    
//     // Verifies if the current buffer is full. If so, attempts to swap buffers.
//     inline void check_fill_and_switch(uint32_t idx, uint32_t new_flags) {
//         if (is_ready(new_flags)) {
//             uint32_t next_idx = get_opposite(idx);
            
//             // Acquire opposite flags to safely check reader lock status
//             uint32_t opposite_flags = flags_[next_idx].load(std::memory_order_acquire);
            
//             // Swap only if the opposite buffer has finished transmitting previous data AND the reader isn't locking it
//             if (is_ready(opposite_flags) && !(opposite_flags & DataInputsBit_LOCK)) {
//                 uint32_t mandatory = mandatory_fields_.load(std::memory_order_relaxed);
//                 uint32_t expected = opposite_flags;
                
//                 // Try to reset opposite buffer flags to mandatory_fields.
//                 // compare_exchange protects against the reader tweaking the LOCK bit simultaneously.
//                 if (flags_[next_idx].compare_exchange_strong(expected, mandatory,
//                                                              std::memory_order_release, 
//                                                              std::memory_order_relaxed)) {
//                     // Update index to point writer to the newly cleared buffer
//                     idx2update_.store(next_idx, std::memory_order_release);
//                     // Announce a completely new snapshot is available
//                     iter_.fetch_add(1, std::memory_order_release);
//                 }
//             }
//         }
//     }
    
//     // Checks if new data was committed since the last time get_data() was called.
//     // PRIVATE: Intended for Feedback struct usage.
//     inline bool is_updated() const {
//         return iter_.load(std::memory_order_acquire) != prev_iter_;
//     }
    
//     // Safely retrieves a full snapshot from the completely filled buffer.
//     // PRIVATE: Intended for Feedback struct usage.
//     inline DataInputs get_data() {
//         uint32_t current_idx = idx2update_.load(std::memory_order_acquire);
//         uint32_t ready_idx = get_opposite(current_idx);
        
//         // Assert lock on the ready buffer. 
//         // memory_order_acquire ensures subsequent memory reads (the copy below) happen AFTER locking.
//         flags_[ready_idx].fetch_or(static_cast<uint32_t>(DataInputsBit_LOCK), std::memory_order_acquire);
        
//         // Copy data safely while locked
//         DataInputs data = buffers_[ready_idx];
        
//         // Sync reader's iter tracker
//         prev_iter_ = iter_.load(std::memory_order_acquire);
        
//         // Release lock.
//         // memory_order_release ensures the data copy completes BEFORE the lock is removed.
//         flags_[ready_idx].fetch_and(~static_cast<uint32_t>(DataInputsBit_LOCK), std::memory_order_release);
        
//         return data;
//     }
    
//     // Grant the Feedback control structure access to private getters
//     friend struct Feedback;
    
// public:
//     // Initializes the dumping system. Buffer 0 is marked pending, Buffer 1 is ready.
//     InputsDump(uint32_t mandatory_fields)
//         : buffers_{DataInputs(), DataInputs()},
//           idx2update_(0),
//           flags_{mandatory_fields | DataInputsBit_CLOCK, 0},
//           iter_(0),
//           prev_iter_(0),
//           mandatory_fields_(mandatory_fields | DataInputsBit_CLOCK)
//     {
//         // One-time release fence to ensure constructor initialization propagates 
//         // across potential core/interrupt boundaries before execution continues.
//         std::atomic_thread_fence(std::memory_order_release);
//     }
    
//     // Pushes clock timestamp into the active buffer
//     inline void set_clock(uint32_t value) {
//         uint32_t idx = idx2update_.load(std::memory_order_acquire);
//         buffers_[idx].clock = value;
//         uint32_t new_flags = atomic_clear_field_bit(idx, DataInputsBit_CLOCK);
//         check_fill_and_switch(idx, new_flags);
//     }

//     // Pushes supply voltage ADC value into the active buffer
//     inline void set_supply_adc(uint16_t value) {
//         uint32_t idx = idx2update_.load(std::memory_order_acquire);
//         buffers_[idx].supply_adc = value;
//         // Flag clearance handles memory release synchronization
//         uint32_t new_flags = atomic_clear_field_bit(idx, DataInputsBit_SUPPLY);
//         check_fill_and_switch(idx, new_flags);
//     }
    
//     // Pushes temperature ADC value into the active buffer
//     inline void set_temper_adc(uint16_t value) {
//         uint32_t idx = idx2update_.load(std::memory_order_acquire);
//         buffers_[idx].temper_adc = value;
//         uint32_t new_flags = atomic_clear_field_bit(idx, DataInputsBit_TEMP);
//         check_fill_and_switch(idx, new_flags);
//     }
    
//     // Pushes three-phase current ADC values into the active buffer
//     inline void set_current_adc(const uint16_t values[3]) {
//         uint32_t idx = idx2update_.load(std::memory_order_acquire);
//         for (size_t i = 0; i < 3; ++i) {
//             buffers_[idx].currnt_adc[i] = values[i];
//         }
//         uint32_t new_flags = atomic_clear_field_bit(idx, DataInputsBit_CURRENT);
//         check_fill_and_switch(idx, new_flags);
//     }
    
//     // Pushes raw encoder/sensor angle into the active buffer
//     inline void set_angle_raw(uint32_t value) {
//         uint32_t idx = idx2update_.load(std::memory_order_acquire);
//         buffers_[idx].angle_raw = value;
//         uint32_t new_flags = atomic_clear_field_bit(idx, DataInputsBit_ANGLE);
//         check_fill_and_switch(idx, new_flags);
//     }

//     // Pushes reference angle into the active buffer
//     inline void set_ref_angle(uint32_t value) {
//         uint32_t idx = idx2update_.load(std::memory_order_acquire);
//         buffers_[idx].angle_ref = value;
//         uint32_t new_flags = atomic_clear_field_bit(idx, DataInputsBit_REF_ANGLE);
//         check_fill_and_switch(idx, new_flags);
//     }

//     // Updates which fields must be populated to consider a buffer full.
//     // Caution: Modify this only during initialization or halted states to avoid split-logic buffers.
//     inline void set_mandatory(uint32_t mandatory_fields) {
//         mandatory_fields_.store(mandatory_fields | DataInputsBit_CLOCK, std::memory_order_release);
//     }
// };