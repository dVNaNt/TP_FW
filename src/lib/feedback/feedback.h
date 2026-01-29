#pragma once  // Ensure this header is included only once during compilation

// Feedback module: Aggregates all sensor feedback data (inputs, supply voltage, current sensing, position)
// Provides unified interface for updating and accessing sensor data
// 
// Key Features:
// - Encapsulates InputsDump, SupplyVoltage, CurrentSense, and Position
// - tick() method is private and only accessible through Feedback structure
// - All getter methods are public for external access
// - Ensures data consistency by controlling update flow



#include "inputs_dump.h"  // Include inputs dump for ADC data buffering
#include "analog/supply_voltage.h"  // Include supply voltage sensing
#include "analog/current_sense.h"  // Include current sensing
#include "position/position_integrator.h"  // Include position/encoder tracking
#include "position/speed_estimator.h"  // Include position/encoder tracking
#include "analog/power.h"
#include "error.h"

// Forward declaration to allow friend access
struct Feedback;

// Feedback structure: Aggregates all sensor feedback components
struct Feedback {  // Define the Feedback structure for unified sensor data management
public:    
    InputsDump inputs;  // Double-buffered ADC and sensor data storage
    SupplyVoltage supply;  // Supply voltage sensing with filtering
    CurrentSense current;  // 3-channel current sensing with calibration
    Position position;  // Encoder position tracking and integration
    PowerMonitor power;
    ErrorManager error;
    
private:
    // Make tick() methods of internal structures accessible only to Feedback
    friend struct SupplyVoltage;
    friend struct CurrentSense;
    friend struct InputsDump;
    friend struct Position;
    friend struct PowerMonitor;
    friend struct DriverPWM;  // Allow DriverPWM to access private methods for calibration
    bool is_updated_;
    DataInputs data_;  // Private copy of latest sensor data for external access
    uint32_t enabled_modules_;  // Bitmask of enabled modules for conditional tick execution

    
    
public:
    // Constructor: Initialize feedback with configuration parameters
    // supply_max_mv: Maximum supply voltage in millivolts
    // current_ch1_fs_ma: Full-scale current for channel 1 in milliamps
    // current_ch2_fs_ma: Full-scale current for channel 2 in milliamps
    // current_ch3_fs_ma: Full-scale current for channel 3 in milliamps
    Feedback(int32_t supply_max_mv, int16_t current_ch1_fs_ma, int16_t current_ch2_fs_ma, int16_t current_ch3_fs_ma)  // Constructor: Create and initialize feedback system
        : inputs(0),  // Initialize inputs dump with mandatory fields using constructor
          supply(200, supply_max_mv),  // Initialize supply voltage sensor
          current(current_ch1_fs_ma, current_ch2_fs_ma, current_ch3_fs_ma),  // Initialize current sensors
          position(),  // Initialize position integrator with default constructor
          is_updated_(false),  // Initialize update flag to false
          data_(),  // Initialize data copy with default constructor
          enabled_modules_(0),  // Initialize enabled modules from mandatory fields
          power(current.full_scale_max_ma, supply_max_mv)
    {
    }

    // Enable supply voltage module (adds SUPPLY bit to enabled modules)
    void enable_supply() {  // Enable supply voltage processing
        enabled_modules_ |= (uint32_t)DataInputsBit_SUPPLY;  // Set SUPPLY bit in enabled modules mask
        inputs = InputsDump(enabled_modules_);  // Recreate inputs dump with updated mandatory fields
    }

    // Enable temperature module (adds TEMP bit to enabled modules)
    void enable_temperature() {  // Enable temperature processing
        enabled_modules_ |= (uint32_t)DataInputsBit_TEMP;  // Set TEMP bit in enabled modules mask
        inputs = InputsDump(enabled_modules_);  // Recreate inputs dump with updated mandatory fields
    }

    // Enable current sensing module (adds CURRENT bit to enabled modules)
    void enable_current() {  // Enable current sensing processing
        enabled_modules_ |= (uint32_t)DataInputsBit_CURRENT;  // Set CURRENT bit in enabled modules mask
        inputs = InputsDump(enabled_modules_);  // Recreate inputs dump with updated mandatory fields
    }

    // Enable encoder/position module (adds ANGLE bit to enabled modules)
    void enable_encoder() {  // Enable encoder/position processing
        enabled_modules_ |= (uint32_t)DataInputsBit_ANGLE;  // Set ANGLE bit in enabled modules mask
        inputs = InputsDump(enabled_modules_);  // Recreate inputs dump with updated mandatory fields
    }


    bool is_updated() const {  // Make const for use in const methods
        return is_updated_;
    }
    
    // Get latest sensor data (public accessor for external use)
    const DataInputs& get_data() const {  // Return const reference to avoid copying
        return data_;  // Return private data copy
    }
    
    // Main processing function: Update all feedback components with new sensor data
    // This is the only method that calls tick() on internal structures
    // Only processes modules that are enabled via enable_* functions
    void tick() {  // Main processing function called each cycle to update all sensors
        // Check if new data is available
        is_updated_ = false;
        if (!inputs.is_updated()) {  // Use private method through friend access
            return;  // Exit if no new data
        }
        
        // Get latest complete sensor data from inputs dump and store in private copy
        data_ = inputs.get_data();  // Use private method through friend access and store in member
        
        // Update supply voltage only if enabled
        if (enabled_modules_ & (uint32_t)DataInputsBit_SUPPLY) {  // Check if SUPPLY bit is set
            supply.tick(data_.supply_adc);  // Process supply voltage ADC value
        }
        
        // Update current sensors only if enabled
        if (enabled_modules_ & (uint32_t)DataInputsBit_CURRENT) {  // Check if CURRENT bit is set
            current.tick(data_.currnt_adc);  // Process current ADC array
        }
        
        // Update position integrator only if enabled
        if (enabled_modules_ & (uint32_t)DataInputsBit_ANGLE) {  // Check if ANGLE bit is set
            position.tick(data_.angle_raw);  // Process encoder angle value using member function
        }
        
        is_updated_ = true;  // Mark that processing is complete
    }
    
};  // End of Feedback structure definition
