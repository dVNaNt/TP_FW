/**
 * @file stability_check.h
 * @brief Small numeric helpers guarding control-loop stability.
 *
 * Provides saturation/divergence checks used by the cascaded controllers to
 * detect runaway integrators or NaN propagation and force safe fallbacks.
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#pragma once


#include "../../common.h"


/**
 * @brief Extremely fast signal stability detector for STM32 (O(1) complexity).
 * 
 * Detects if a signal stays within a dynamic 'epsilon' band for 'N' consecutive cycles.
 * Automatically finds the reference value (center of the band) without prior knowledge.
 * Zero division, zero memory allocation.
 * Uses float physical units, for example Amperes for motor current.
 */
class StabilityCheck {
private:
    float epsilon_;         // Allowed noise amplitude (peak-to-peak), physical units
    uint16_t window_;       // Required consecutive stable cycles
    uint16_t counter_;      // Current stable cycle count
    
    // Dynamic envelope tracking
    float min_val_;         // Minimum value tracked in current window
    float max_val_;         // Maximum value tracked in current window


public:
    /// Constructor
    /// @param epsilon Max allowed peak-to-peak noise in physical units, for example Amperes
    /// @param window Required stable cycles (default 180)
    StabilityCheck(float epsilon = 0.05f, uint16_t window = 180)
        : epsilon_(epsilon)
        , window_(window)
        , counter_(0)
        , min_val_(0.0f)
        , max_val_(0.0f)
    {}


    /// Configure parameters dynamically
    void setup(float epsilon, uint16_t window) {
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
     * @param input Current measured signal value in physical units, for example Amperes
     * @return true if signal has been stable for 'window' cycles
     */
    bool tick(float input) {
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


        // 2. Check peak-to-peak spread in physical units
        float spread = max_val_ - min_val_;


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
    float get_detected_reference() const {
        return (min_val_ + max_val_) * 0.5f;
    }
};