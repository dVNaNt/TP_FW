/**
 * @file encoder.h
 * @brief Position integrator with ISR-safe ingestion and HW callbacks.
 *
 * @copyright Copyright 2026, creapunk
 * @license Apache-2.0
 */

#pragma once

#include <atomic>
#include "../../common.h"

/** Signature of a parameterless encoder lifecycle callback (start/stop). */
typedef void (*EncoderCallback)();

/**
 * @struct Encoder
 * @brief Encoder position tracking with ISR-safe ingestion and lock-free reading.
 */
struct Encoder {
private:
    static inline void empty_callback() {}

    EncoderCallback start_callback_ = &empty_callback;  ///< Invoked by start() (typically enables the SPI polling ISR).
    EncoderCallback stop_callback_  = &empty_callback;  ///< Invoked by stop() (typically disables the polling ISR).

    // Union to access the 64-bit position as two 32-bit halves.
    // Note: Assumes Little-Endian architecture (standard for ARM Cortex-M, ESP32, etc.).
    // In Little-Endian, the first member in memory is the lower 32 bits.
    union PositionData {
        int64_t value;
        struct {
            uint32_t angle;     // Lower 32 bits
            int32_t rotations;  // Upper 32 bits
        } parts;
    };

    // Align to 8 bytes to ensure both halves are on their natural boundaries,
    // allowing single-instruction 32-bit atomic reads.
    alignas(8) PositionData position_{0};  ///< Current integrated position (angle + rotations).
    
    // SeqLock counter for consistent 64-bit reads only.
    /// Sequence lock: odd = write in progress, even = stable. Single writer.
    std::atomic<uint32_t> seq_{0};

public:
    /// Default constructor: zeroed position, empty callbacks.
    Encoder() {}

    /// Invoke the registered start callback (enable acquisition hardware).
    inline void start() { start_callback_(); }
    /// Invoke the registered stop callback (disable acquisition hardware).
    inline void stop()  { stop_callback_(); }

    // =======================================================================
    // ISR-safe ingestion (Single Writer)
    // =======================================================================

    /**
     * @brief Ingest a new raw encoder sample and integrate it into the
     *        64-bit position.
     *
     * Called from the single-writer context (SPI/ISR). The unwrapping uses
     * the wrap-around property of int32 arithmetic:
     * diff = (int32_t)(input_pos - last_low_word), so multi-revolution motion
     * accumulates correctly into the rotations half.
     *
     * @param input_pos Latest raw angle word from the sensor [0..2^32-1].
     */
    inline void set_raw_position(uint32_t input_pos) {
        // --- Begin Write ---
        seq_.fetch_add(1, std::memory_order_relaxed); // Makes counter odd
        std::atomic_thread_fence(std::memory_order_release);
        
        // Read current 64-bit value, calculate new 64-bit value
        int64_t curr = position_.value;
        int32_t diff = (int32_t)(input_pos - (uint32_t)curr);
        position_.value = curr + diff; // Single 64-bit write (compiles to two 32-bit STRs)

        std::atomic_thread_fence(std::memory_order_release);
        seq_.fetch_add(1, std::memory_order_relaxed); // Makes counter even
    }

    // =======================================================================
    // Fast 32-bit reads (Direct memory access)
    // =======================================================================

    /**
     * @brief Get the "clean" angle (32 bits, left-aligned).
     * @note Ultra-fast: single atomic 32-bit hardware load. No SeqLock loop needed.
     */
    inline uint32_t get_angle() const { 
        return position_.parts.angle; 
    }

    /**
     * @brief Get the clean full rotations (32 bits, signed).
     * @note Ultra-fast: single atomic 32-bit hardware load. No SeqLock loop needed.
     */
    inline int32_t get_rotations() const { 
        return position_.parts.rotations; 
    }

    // =======================================================================
    // Consistent 64-bit reads (Multiple Readers)
    // =======================================================================

private:
    /**
     * @brief Lock-free thread-safe read of the full 64-bit position.
     * Uses SeqLock to guarantee the upper and lower 32 bits are consistent.
     */
    inline int64_t load_position_64() const {
        uint32_t s1, s2;
        int64_t pos;
        do {
            s1 = seq_.load(std::memory_order_relaxed);
            std::atomic_thread_fence(std::memory_order_acquire);
            
            pos = position_.value; // Read the raw 64-bit value
            
            std::atomic_thread_fence(std::memory_order_acquire);
            s2 = seq_.load(std::memory_order_relaxed);
        } while (s1 != s2 || (s1 & 1));
        return pos;
    }

public:
    /**
     * @brief Combined position in Q16.16 format (spans both 32-bit halves).
     * @note Requires SeqLock because it uses bits from both halves.
     */
    inline int32_t get_position() const { 
        return (int32_t)(load_position_64() >> 16); 
    }

    /**
     * @brief Get the raw 64-bit position for filters (e.g., EMA/PLL).
     * @note Requires SeqLock.
     */
    inline int64_t get_position_64() const { 
        return load_position_64(); 
    }


    // =======================================================================
    // FLUENT CONFIGURATION API
    // =======================================================================

    /** Fluent proxy for registering start/stop callbacks. */
    struct CallbackSetter {
    private:
        Encoder& parent;  ///< Owning encoder instance.
    public:
        CallbackSetter(Encoder& p) : parent(p) {}

        /// Register the start callback (nullptr restores the empty no-op). Chainable.
        inline CallbackSetter& start_cb(EncoderCallback cb) {
            parent.start_callback_ = cb ? cb : &Encoder::empty_callback;
            return *this;
        }
        /// Register the stop callback (nullptr restores the empty no-op). Chainable.
        inline CallbackSetter& stop_cb(EncoderCallback cb) {
            parent.stop_callback_ = cb ? cb : &Encoder::empty_callback;
            return *this;
        }
    };

    /// Entry point of the fluent callback registration API.
    inline CallbackSetter set_callback() { return CallbackSetter(*this); }
};