#pragma once  // Ensure this header is included only once during compilation

#include "../common.h"  // Include common migration definitions and types

/// Position integrator manages and calculates the absolute position from incremental encoder readings.
/// Tracks both angle (0..65535) and full rotations (signed) as a combined 32-bit value.
/// 
/// Key Features:
/// - Automatic wrap-around handling for 16-bit encoder values
/// - Combined position value: high 16 bits = rotations, low 16 bits = angle
/// - Zero reset capability for ABZ encoders
/// - Optional low-pass filtering for smooth position output
struct Encoder {  // Define the Position structure for encoder position tracking
private:
    // ---------------------------------------------------------------------------
    // Private data members
    // ---------------------------------------------------------------------------
    
    int32_t position_;  // Combined value: (rotations << 16) | angle

    // Main processing function: Update position with new encoder reading
    // input_pos: Current encoder angle (0..65535)
    void tick(uint16_t input_pos) {  // Main processing function called each cycle with new encoder reading
        // Retrieve the previous angle by casting the current position to u16
        uint16_t prev_angle = (uint16_t)position_;  // Extract low 16 bits as previous angle
        
        // Calculate the difference between the current and the previous angle with wrapping
        int16_t dif = (int16_t)(input_pos - prev_angle);  // Signed difference handles wrap-around
        
        // Update the current position by adding the difference, ensuring it wraps around correctly
        position_ = position_ + (int32_t)dif;  // Add difference to maintain continuous position
    }

    friend struct Feedback;
    
public:
    // Constructor: Initialize position to zero with default filter (no filtering)
    Encoder() : position_(0){}  // Constructor: Create with zero position and disabled filter
    
    // Get current encoder angle (0..65535)
    inline uint16_t get_angle() {  // Get current angle value
        return (uint16_t)position_;  // Return low 16 bits as angle
    }
        
    // Get number of full rotations (signed)
    inline int16_t get_rotations() const {  // Get rotation count
        return (int16_t)(position_ >> 16);  // Return high 16 bits as rotation count
    }
    
    // Get absolute position value (rotations + angle as int32)
    inline int32_t get_position() const {  // Get combined position value
        return position_;  // Return full 32-bit position
    }    
};  // End of Position structure definition
