#pragma once

#include "../../common.h"

/**
 * @brief Extremely fast signal stability detector for STM32 (O(1) complexity).
 * 
 * Detects if a signal stays within a dynamic 'epsilon' band for 'N' consecutive cycles.
 * Automatically finds the reference value (center of the band) without prior knowledge.
 * Zero division, zero floating point, zero memory allocation.
 */
class StabilityCheck {
private:
    int16_t epsilon_;        // Allowed noise amplitude (peak-to-peak)
    uint16_t window_;        // Required consecutive stable cycles
    uint16_t counter_;       // Current stable cycle count
    
    // Dynamic envelope tracking
    int16_t min_val_;        // Minimum value tracked in current window
    int16_t max_val_;        // Maximum value tracked in current window

public:
    /// Constructor
    /// @param epsilon Max allowed peak-to-peak noise (default 50)
    /// @param window Required stable cycles (default 180)
    StabilityCheck(int16_t epsilon = 50, uint16_t window = 180)
        : epsilon_(epsilon)
        , window_(window)
        , counter_(0)
        , min_val_(0)
        , max_val_(0)
    {}

    /// Configure parameters dynamically
    void setup(int16_t epsilon, uint16_t window) {
        epsilon_ = epsilon;
        window_ = window;
        reset();
    }

    /// Reset internal state
    inline void reset() {
        counter_ = 0;
        // min/max will be re-initialized on next tick
    }

    /**
     * @brief Core update function. Call this at FOC frequency.
     * 
     * Uses a "Greedy Min/Max" approach:
     * 1. Updates the min/max bounds of the current candidate sequence.
     * 2. Checks if the spread (max - min) exceeds epsilon.
     * 3. If valid, increments counter. If invalid, resets counter and starts a new sequence.
     * 
     * @param input Current measured signal value
     * @return true if signal has been stable for 'window' cycles
     */
    bool tick(int16_t input) {
        // Initialization or Restart on the very first sample of a new block
        if (counter_ == 0) {
            min_val_ = input;
            max_val_ = input;
            counter_ = 1;
            return false;
        }

        // 1. Update Envelope (Branchless optimization candidate for compiler)
        if (input < min_val_) min_val_ = input;
        if (input > max_val_) max_val_ = input;

        // 2. Check spread (using int32 to prevent overflow during subtraction, though unlikely for int16)
        int32_t spread = (int32_t)max_val_ - (int32_t)min_val_;

        if (spread <= epsilon_) {
            // Still within tolerance
            if (counter_ < window_) {
                counter_++;
            }
            return (counter_ >= window_);
        } else {
            // Signal broke the envelope bounds.
            // HARD RESET strategy: The previous 'window-1' samples are invalid as a group.
            // We immediately start a NEW group from the current sample to avoid dead-time.
            min_val_ = input;
            max_val_ = input;
            counter_ = 1; 
            return false;
        }
    }

    /// Check if currently stable (read-only)
    inline bool is_stable() const { 
        return counter_ >= window_; 
    }

    /**
     * @brief Returns the auto-detected reference value.
     * 
     * Valid only when is_stable() is true. 
     * Returns the center of the noise floor observed during the stable window.
     */
    int16_t get_detected_reference() const {
        return (min_val_ + max_val_) >> 1; // Fast integer division by 2
    }
};
