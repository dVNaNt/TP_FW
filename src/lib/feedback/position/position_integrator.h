#pragma once  // Ensure this header is included only once during compilation

#include "../../common.h"  // Include common migration definitions and types
#include "../../math_integer/filters/ema.h"  // Include first-order filter for position smoothing

/// Position integrator manages and calculates the absolute position from incremental encoder readings.
/// Tracks both angle (0..65535) and full rotations (signed) as a combined 32-bit value.
/// 
/// Key Features:
/// - Automatic wrap-around handling for 16-bit encoder values
/// - Combined position value: high 16 bits = rotations, low 16 bits = angle
/// - Zero reset capability for ABZ encoders
/// - Optional low-pass filtering for smooth position output
struct Position {  // Define the Position structure for encoder position tracking
private:
    // ---------------------------------------------------------------------------
    // Private data members
    // ---------------------------------------------------------------------------
    
    int32_t position_;  // Combined value: (rotations << 16) | angle
    FilterEMA filter_;  // Low-pass filter for smoothing angle measurements

    // Main processing function: Update position with new encoder reading
    // input_pos: Current encoder angle (0..65535)
    void tick(uint16_t input_pos) {  // Main processing function called each cycle with new encoder reading
        // Retrieve the previous angle by casting the current position to u16
        uint16_t prev_angle = (uint16_t)position_;  // Extract low 16 bits as previous angle
        
        // Calculate the difference between the current and the previous angle with wrapping
        int16_t dif = (int16_t)(input_pos - prev_angle);  // Signed difference handles wrap-around
        
        // Update the current position by adding the difference, ensuring it wraps around correctly
        position_ = position_ + (int32_t)dif;  // Add difference to maintain continuous position
        
        // Update filtered angle using low-pass filter
        filter_.tick_lpf(input_pos);  // Apply LPF to raw angle for smooth output
    }

    friend struct Feedback;
    
public:
    // Constructor: Initialize position to zero with default filter (no filtering)
    Position() : position_(0), filter_(0, 255) {}  // Constructor: Create with zero position and disabled filter
    
    // Constructor: Initialize position with filter alpha
    // alpha: Filter coefficient (0..255, where 0=infinite filtering, 255=no filtering)
    Position(uint8_t alpha) : position_(0), filter_(0, alpha) {}  // Constructor: Create with specified filter alpha
    
    // Get current encoder angle (0..65535)
    inline uint16_t get_angle() const {  // Get current angle value
        return (uint16_t)position_;  // Return low 16 bits as angle
    }
    
    // Get filtered encoder angle (0..65535)
    inline uint16_t get_filtered_angle() const {  // Get filtered angle value
        return filter_.get_output();  // Return filtered angle from LPF
    }
    
    // Get filtered position value (rotations + filtered angle as int32)
    inline int32_t get_filtered_position() const {  // Get combined filtered position
        // Combine current rotations with filtered angle
        int16_t rotations = get_rotations();  // Get rotation count
        uint16_t filtered_angle = filter_.get_output();  // Get filtered angle
        return ((int32_t)rotations << 16) | (int32_t)filtered_angle;  // Combine into 32-bit value
    }
    
    // Get number of full rotations (signed)
    inline int16_t get_rotations() const {  // Get rotation count
        return (int16_t)(position_ >> 16);  // Return high 16 bits as rotation count
    }
    
    // Get absolute position value (rotations + angle as int32)
    inline int32_t get_value() const {  // Get combined position value
        return position_;  // Return full 32-bit position
    }
    
    // Reset position to zero (for ABZ encoders on zero index)
    inline void reset() {  // Reset position counter
        position_ = 0;  // Set position to zero
        filter_.reset(0);  // Reset filter state to zero
    }
    
    // Set filter alpha coefficient
    // alpha: Filter coefficient (0..255, where 0=infinite filtering, 255=no filtering)
    inline void set_filter_alpha(uint8_t alpha) {  // Update filter coefficient
        filter_.set_alpha(alpha);  // Set new alpha value
    }
    
};  // End of Position structure definition
