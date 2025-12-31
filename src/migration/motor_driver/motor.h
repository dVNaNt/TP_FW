#pragma once

#include "../common.h"
#include "types.h"

// Motor and driver system parameters structure
// Contains electrical and mechanical constants for motor control
struct Motor {
private:
    // Motor type and configuration
    MotorType motor_type_;                // Type of motor (DC, STEP, BLDC, Invalid)
    PhasePattern phase_pattern_;          // Phase connection pattern (ABC, BCA, CAB)
    DriverStatus status_;                 // Current driver status
    
    // Motor mechanical parameters
    int32_t pole_count_;                  // Number of pole pairs
    int16_t direction_;                   // Motor rotation direction (+1/-1)
    
    // Phase resistance [mOhm]
    int32_t phase_resistance_mohm_[3];    // Phase resistance in milliohms for each phase
    int32_t average_resistance_mohm_;     // Average resistance across phases
    int16_t deviation_percent_;           // Resistance deviation between phases (-1 if invalid)
    
    // Phase inductance [uH]
    int32_t phase_inductance_uh_[3];      // Phase inductance in microhenries
    
    // Operating limits
    int32_t current_limit_ma_;            // Current limit for calibration in milliamps
    int32_t max_voltage_mv_;              // Maximum voltage in millivolts

public:
    // Constructor with default values
    Motor()
        : motor_type_(MotorType_Invalid)
        , phase_pattern_(ABC)
        , status_(DriverStatus_CalibrateCurrent)
        , pole_count_(7)
        , direction_(1)
        , average_resistance_mohm_(0)
        , deviation_percent_(-1)
        , current_limit_ma_(600)
        , max_voltage_mv_(24000)
    {
        // Initialize resistance arrays to zero
        for (uint8_t i = 0; i < 3; i++) {
            phase_resistance_mohm_[i] = 0;
            phase_inductance_uh_[i] = 0;
        }
    }
    
    // === Setters ===
    
    // Set motor type
    inline void set_motor_type(MotorType type) { motor_type_ = type; }
    
    // Set phase pattern
    inline void set_phase_pattern(PhasePattern pattern) { phase_pattern_ = pattern; }
    
    // Set driver status
    inline void set_status(DriverStatus status) { status_ = status; }
    
    // Set pole count
    inline void set_pole_count(int32_t poles) { pole_count_ = poles; }
    
    // Set direction
    inline void set_direction(int16_t dir) { direction_ = dir; }
    
    // Set phase resistance for specific phase (0-2)
    void set_phase_resistance(uint8_t phase_idx, int32_t resistance_mohm) {
        if (phase_idx >= 3) return;                    // Validate index
        phase_resistance_mohm_[phase_idx] = resistance_mohm;  // Store resistance
    }
    
    // Set average resistance directly
    inline void set_average_resistance(int32_t resistance_mohm) { 
        average_resistance_mohm_ = resistance_mohm; 
    }
    
    // Set deviation percentage
    inline void set_deviation_percent(int16_t deviation) { deviation_percent_ = deviation; }
    
    // Set phase inductance for specific phase (0-2)
    void set_phase_inductance(uint8_t phase_idx, int32_t inductance_uh) {
        if (phase_idx >= 3) return;                    // Validate index
        phase_inductance_uh_[phase_idx] = inductance_uh;  // Store inductance
    }
    
    // Set current limit for calibration
    inline void set_current_limit(int32_t current_ma) { current_limit_ma_ = current_ma; }
    
    // Set maximum voltage
    inline void set_max_voltage(int32_t voltage_mv) { max_voltage_mv_ = voltage_mv; }
    
    // === Getters ===
    
    // Get motor type
    inline MotorType get_motor_type() const { return motor_type_; }
    
    // Get phase pattern
    inline PhasePattern get_phase_pattern() const { return phase_pattern_; }
    
    // Get driver status
    inline DriverStatus get_status() const { return status_; }
    
    // Get pole count
    inline int32_t get_pole_count() const { return pole_count_; }
    
    // Get direction
    inline int16_t get_direction() const { return direction_; }
    
    // Get phase resistance for specific phase
    inline int32_t get_phase_resistance(uint8_t phase_idx) const {
        return (phase_idx < 3) ? phase_resistance_mohm_[phase_idx] : 0;
    }
    
    // Get average resistance
    inline int32_t get_average_resistance() const { return average_resistance_mohm_; }
    
    // Get deviation percentage
    inline int16_t get_deviation_percent() const { return deviation_percent_; }
    
    // Get phase inductance for specific phase
    inline int32_t get_phase_inductance(uint8_t phase_idx) const {
        return (phase_idx < 3) ? phase_inductance_uh_[phase_idx] : 0;
    }
    
    // Get current limit for calibration
    inline int32_t get_current_limit() const { return current_limit_ma_; }
    
    // Get maximum voltage
    inline int32_t get_max_voltage() const { return max_voltage_mv_; }
    
    // Calculate and return average resistance from phase measurements
    int32_t calc_average_resistance() const {
        int64_t sum = 0;                               // Accumulator for sum
        uint8_t count = 0;                             // Valid measurement counter
        
        for (uint8_t i = 0; i < 3; i++) {
            if (phase_resistance_mohm_[i] > 0) {       // Only count valid measurements
                sum += phase_resistance_mohm_[i];
                ++count;
            }
        }
        
        return (count > 0) ? (int32_t)(sum / count) : 0;  // Return average or 0
    }
    
    // Calculate and return average inductance from phase measurements
    int32_t calc_average_inductance() const {
        int64_t sum = 0;                               // Accumulator for sum
        uint8_t count = 0;                             // Valid measurement counter
        
        for (uint8_t i = 0; i < 3; i++) {
            if (phase_inductance_uh_[i] > 0) {         // Only count valid measurements
                sum += phase_inductance_uh_[i];
                ++count;
            }
        }
        
        return (count > 0) ? (int32_t)(sum / count) : 0;  // Return average or 0
    }
    
    // Print parameters to serial
    template<typename SerialType>
    void print(SerialType& serial) const {
        serial.println("=== Motor Parameters ===");
        
        // Motor type
        serial.print("Type: ");
        switch (motor_type_) {
            case MotorType_BLDC: serial.println("BLDC"); break;
            case MotorType_STEP: serial.println("Stepper"); break;
            case MotorType_DC:   serial.println("DC"); break;
            default:             serial.println("Invalid"); break;
        }
        
        // Phase pattern
        serial.print("Pattern: ");
        switch (phase_pattern_) {
            case ABC: serial.println("ABC"); break;
            case BCA: serial.println("BCA"); break;
            case CAB: serial.println("CAB"); break;
        }
        
        // Status
        serial.print("Status: ");
        switch (status_) {
            case DriverStatus_Ready:    serial.println("Ready"); break;
            case DriverStatus_Error:    serial.println("Error"); break;
            case DriverStatus_Calibrating: serial.println("Calibrating"); break;
            default: serial.println("..."); break;
        }
        
        // Pole count and direction
        serial.print("Poles: ");
        serial.print(pole_count_);
        serial.print(", Dir: ");
        serial.println(direction_);
        
        // Phase resistances
        serial.println("Resistances [mOhm]:");
        for (uint8_t i = 0; i < 3; i++) {
            serial.print("  ");
            serial.print((char)('A' + i));
            serial.print(": ");
            serial.println(phase_resistance_mohm_[i]);
        }
        
        // Operating limits
        serial.print("I_lim: ");
        serial.print(current_limit_ma_);
        serial.print(" mA, V_max: ");
        serial.print(max_voltage_mv_);
        serial.println(" mV");
        
        serial.println("========================");
    }
};
