// Timeout timer helper class for countdown operations
// Provides simple countdown timer with zero-detection

#pragma once

#include <stdint.h>

// Timeout timer class for countdown operations
struct timeout_timer {
private:
    int32_t counter; // Internal countdown counter

public:
    // Constructor - initialize counter to zero
    timeout_timer(int32_t preload) : counter(preload) {}

    // Set the counter value to start countdown
    inline void set_timeout(int32_t value) {
        counter = value; // Load new countdown value
    }

    // Decrement counter and return true when reaches zero
    int32_t tick() {
        counter--; // Decrement counter
        return counter; // Counter reached zero (expired)
    }

    // Get current counter value
    inline int32_t get_timeout() const {
        return counter; // Return current countdown value
    }

    // Reset timer to zero immediately
    inline void reset() {
        counter = 0; // Clear counter
    }
};
