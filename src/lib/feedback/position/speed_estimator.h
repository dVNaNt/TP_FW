#pragma once  // Ensure this header is included only once during compilation

#include "../../common.h"  // Include common definitions and types

// SpeedEstimator: Estimates instantaneous speed of encoder using circular buffer
// Uses position difference over N samples to calculate speed with minimal latency

// Size of circular buffer for position samples (min 2, max 32)
#define SPEED_ESTIMATOR_SIZE 32  // Define buffer size for speed calculation

// SpeedEstimator structure: Calculates speed from position samples
struct SpeedEstimator {  // Define SpeedEstimator as C++ struct with methods
private:
    // ---------------------------------------------------------------------------
    // Private data members
    // ---------------------------------------------------------------------------
    
    uint16_t freq_;  // Sampling frequency in Hz for speed calculation
    int32_t speed_;  // Calculated instantaneous speed value
    int32_t pos_buffer_[SPEED_ESTIMATOR_SIZE];  // Circular buffer storing position samples
    size_t idx_;  // Current write index in circular buffer (0 to SIZE-1)
    
public:
    // ---------------------------------------------------------------------------
    // Constructor
    // ---------------------------------------------------------------------------
    
    // Default constructor: Initialize with zero position and frequency
    SpeedEstimator()  // Default constructor for zero initialization
        : freq_(0),  // Initialize frequency to zero
          speed_(0),  // Initialize speed to zero
          idx_(0)  // Initialize buffer index to zero
    {
        for (size_t i = 0; i < SPEED_ESTIMATOR_SIZE; ++i) {  // Loop through buffer
            pos_buffer_[i] = 0;  // Initialize all buffer positions to zero
        }
    }
    
    // Parameterized constructor: Initialize with specific position and frequency
    // init_position: Initial position value to fill buffer
    // freq: Sampling frequency in Hz
    SpeedEstimator(uint16_t freq)  // Constructor with parameters
        : freq_(freq),  // Store sampling frequency
          speed_(0),  // Initialize speed to zero
          idx_(0)  // Initialize buffer index to zero
    {
        for (size_t i = 0; i < SPEED_ESTIMATOR_SIZE; ++i) {  // Loop through buffer
            pos_buffer_[i] = 0;  // Fill buffer with initial position
        }
    }
    
    // ---------------------------------------------------------------------------
    // Public methods
    // ---------------------------------------------------------------------------
    
    // Update speed estimate with new position sample
    // new_position: Latest position value from encoder
    void tick(int32_t new_position) {  // Main processing function for speed calculation
        // Calculate position difference over N samples (current - oldest)
        int32_t difference = new_position - pos_buffer_[idx_];  // Compute position delta
        
        // Calculate speed: (delta_position * frequency) / buffer_size
        // This gives speed in position_units per second
        speed_ = (difference * (int32_t)freq_) / (int32_t)SPEED_ESTIMATOR_SIZE;  // Compute instantaneous speed
        
        // Store new position in circular buffer at current index
        pos_buffer_[idx_] = new_position;  // Update buffer with new sample
        
        // Advance index with wraparound (circular buffer behavior)
        idx_ = (idx_ + 1U) % SPEED_ESTIMATOR_SIZE;  // Increment index with modulo wraparound
    }
    
    // Get current speed estimate
    // Returns: Instantaneous speed in position units per second
    int32_t get_speed() const {  // Getter for calculated speed value
        return speed_;  // Return current speed estimate
    }
    
    // Get sampling frequency
    // Returns: Frequency in Hz
    uint16_t get_frequency() const {  // Getter for sampling frequency
        return freq_;  // Return configured frequency
    }
    
    // Set sampling frequency
    // freq: New frequency in Hz
    void set_frequency(uint16_t freq) {  // Setter for sampling frequency
        freq_ = freq;  // Update frequency value
    }
    
    // Reset speed estimator with new initial position
    // init_position: Position value to fill buffer
    void reset(int32_t init_position) {  // Reset estimator state
        speed_ = 0;  // Clear speed value
        idx_ = 0;  // Reset buffer index
        for (size_t i = 0; i < SPEED_ESTIMATOR_SIZE; ++i) {  // Loop through buffer
            pos_buffer_[i] = init_position;  // Fill buffer with new position
        }
    }
};  // End of SpeedEstimator structure definition