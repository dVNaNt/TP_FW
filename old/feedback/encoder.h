#pragma once  // Ensure this header is included only once during compilation

#include "../../common.h"  // Include common migration definitions and types

/// Position integrator manages and calculates the absolute position from incremental encoder readings.
/// Uses a single 64-bit integer (q1.31.32 format) for zero-overhead wrap-around tracking.
struct Encoder {
private:
    // ---------------------------------------------------------------------------
    // Private data members
    // ---------------------------------------------------------------------------

    // Single 64-bit absolute position (q1.31.32 format):
    // - Upper 32 bits: signed full rotations (int32_t)
    // - Lower 32 bits: unsigned angle (0x00000000 = 0°, 0xFFFFFFFF = ~360°)
    int64_t position_ = 0;

    friend struct Feedback;

public:
    // Constructor: Initialize position to zero
    Encoder() {}

    // Main processing function: Update position with new encoder reading
    // input_pos: Current encoder angle u32 (left aligned, full 32-bit resolution)
    inline void tick(uint32_t input_pos) {
        // Calculate the shortest path (difference) between the new angle and the current one (lower 32 bits).
        // Casting to int32_t automatically provides the correct delta [-180°, 180°) taking overflow into account.
        int32_t diff = (int32_t)(input_pos - (uint32_t)position_);

        // Two's complement magic:
        // Add the signed 32-bit difference to the 64-bit position.
        // - If overflowing forward: the hardware carry flag from the lower 32 bits
        //   will automatically add +1 to the upper 32 bits (rotations).
        // - If wrapping backward through 0: diff will be negative; its sign-extension
        //   and hardware borrow will effectively subtract 1 rotation.
        position_ += diff;
    }

    // ---------------------------------------------------------------------------
    // Getters
    // ---------------------------------------------------------------------------

    // 1) Get the "clean" angle (32 bits, left-aligned)
    inline uint32_t get_angle() const { return (uint32_t)position_; }

    // 2) Get the clean full rotations (32 bits, signed)
    inline int32_t get_rotations() const { return (int32_t)(position_ >> 32); }

    // 3) Combined position in Q16.16 format (1 sign bit, 15 rotation bits, 16 angle bits).
    // Thanks to the single 64-bit variable, manual merging (rot << 16 | ang >> 16)
    // is no longer needed. Everything boils down to a single arithmetic right shift.
    inline int32_t get_position() const { return (int32_t)(position_ >> 16); }

    // 4) Optional: get the raw 64-bit position for filters (e.g., EMA/PLL velocity tracking)
    inline int64_t get_position_64() const { return position_; }
};