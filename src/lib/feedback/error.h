#pragma once

// Error management system for motor control
// Tracks Fatal, Alert, and Warning level errors with automatic recovery
#include "../common.h"  // Include common migration definitions and types

// Error severity levels
enum ErrorLevel {
    ErrorLevel_None = 0,     // No error
    ErrorLevel_Warning = 1,  // Warning - operation continues
    ErrorLevel_Alert = 2,    // Alert - degraded operation
    ErrorLevel_Fatal = 3     // Fatal - immediate shutdown required
};

// Fatal error types (immediate shutdown)
enum FatalError {
    Fatal_None = 0,
    Fatal_DC_OVLO = (1 << 0),           // DC overvoltage lockout
    Fatal_DC_UVLO = (1 << 1),           // DC undervoltage lockout
    Fatal_Phase1_OC = (1 << 2),         // Phase 1 overcurrent
    Fatal_Phase2_OC = (1 << 3),         // Phase 2 overcurrent
    Fatal_Phase3_OC = (1 << 4),         // Phase 3 overcurrent
    Fatal_OC = (0b1111 << 2),                // General overcurrent
    Fatal_CurrentLeakage = (1 << 6),    // Current leakage detected
    Fatal_Overheat = (1 << 7),          // Overtemperature
    Fatal_NoLoad = (1 << 8),            // No load detected
    Fatal_NoCurrentFeedback = (1 << 9), // Current sensor failure
    Fatal_EmergencyStop = (1 << 10),    // Emergency stop triggered
    Fatal_MotorError = (1 << 11)        // Motor configuration error
};

// Alert error types (degraded operation)
enum AlertError {
    Alert_None = 0,
    Alert_DC_OVLO_Soft = (1 << 0),      // DC overvoltage warning
    Alert_DC_UVLO_Soft = (1 << 1),      // DC undervoltage warning
    Alert_OC_Soft = (1 << 2),           // Overcurrent warning
    Alert_Overheat_Soft = (1 << 3),     // Overtemperature warning
    Alert_Stall = (1 << 4)              // Motor stall detected
};

// Warning error types (reserved for future use)
enum WarningError {
    Warning_None = 0,
    Warning_Reserved = (1 << 0)         // Reserved for future warnings
};

// Error management structure
struct ErrorManager {
private:
    uint32_t fatal_flags_;      // Fatal error bitmask
    uint32_t alert_flags_;      // Alert error bitmask
    uint32_t warning_flags_;    // Warning error bitmask

public:

    uint8_t oc_error_soft = 0;  // Soft overcurrent error flags (bitmask for 3 channels)
    uint8_t oc_error_hard = 0;  // Hard overcurrent error flags (bitmask for 3 channels)
    
    // Constructor - initialize all error flags to zero
    ErrorManager() 
        : fatal_flags_(0)
        , alert_flags_(0)
        , warning_flags_(0)
    {}

    
    inline void set_overcurrent_hard_fault(Channel ch) { oc_error_hard |= 1 << ch; }

    inline void reset_overcurrent_hard_fault(Channel ch) { oc_error_hard &= ~(1UL << ch); }

    // Check if any channel exceeds hard overcurrent limit
    inline bool is_overcurrent_hard() const { return oc_error_hard; }

    
    // ========================================================================
    // Fatal error setters (inline)
    // ========================================================================
    
    inline void set_fatal_dc_ovlo() {
        fatal_flags_ |= Fatal_DC_OVLO;  // Set overvoltage flag
    }

    inline void reset_fatal_dc_ovlo() {
        fatal_flags_ &= ~Fatal_DC_OVLO;  // Clear overvoltage flag
    }

    inline void set_fatal_dc_uvlo() {
        fatal_flags_ |= Fatal_DC_UVLO;  // Set undervoltage flag
    }
    
    inline void reset_fatal_dc_uvlo() {
        fatal_flags_ &= ~Fatal_DC_UVLO;  // Clear undervoltage flag
    }
    
    inline void set_fatal_phase1_oc() {
        fatal_flags_ |= Fatal_Phase1_OC;  // Set phase 1 overcurrent flag
    }
    
    inline void reset_fatal_phase1_oc() {
        fatal_flags_ &= ~Fatal_Phase1_OC;  // Clear phase 1 overcurrent flag
    }
    
    inline void set_fatal_phase2_oc() {
        fatal_flags_ |= Fatal_Phase2_OC;  // Set phase 2 overcurrent flag
    }
    
    inline void reset_fatal_phase2_oc() {
        fatal_flags_ &= ~Fatal_Phase2_OC;  // Clear phase 2 overcurrent flag
    }
    
    inline void set_fatal_phase3_oc() {
        fatal_flags_ |= Fatal_Phase3_OC;  // Set phase 3 overcurrent flag
    }
    
    inline void reset_fatal_phase3_oc() {
        fatal_flags_ &= ~Fatal_Phase3_OC;  // Clear phase 3 overcurrent flag
    }
    
    inline void set_fatal_oc() {
        fatal_flags_ |= Fatal_OC;  // Set general overcurrent flag
    }
    
    inline void reset_fatal_oc() {
        fatal_flags_ &= ~Fatal_OC;  // Clear general overcurrent flag
    }
    
    inline void set_fatal_current_leakage() {
        fatal_flags_ |= Fatal_CurrentLeakage;  // Set current leakage flag
    }
    
    inline void reset_fatal_current_leakage() {
        fatal_flags_ &= ~Fatal_CurrentLeakage;  // Clear current leakage flag
    }
    
    inline void set_fatal_overheat() {
        fatal_flags_ |= Fatal_Overheat;  // Set overtemperature flag
    }
    
    inline void reset_fatal_overheat() {
        fatal_flags_ &= ~Fatal_Overheat;  // Clear overtemperature flag
    }
    
    inline void set_fatal_no_load() {
        fatal_flags_ |= Fatal_NoLoad;  // Set no load flag
    }
    
    inline void reset_fatal_no_load() {
        fatal_flags_ &= ~Fatal_NoLoad;  // Clear no load flag
    }
    
    inline void set_fatal_no_current_feedback() {
        fatal_flags_ |= Fatal_NoCurrentFeedback;  // Set current sensor failure flag
    }
    
    inline void reset_fatal_no_current_feedback() {
        fatal_flags_ &= ~Fatal_NoCurrentFeedback;  // Clear current sensor failure flag
    }
    
    inline void set_fatal_emergency_stop() {
        fatal_flags_ |= Fatal_EmergencyStop;  // Set emergency stop flag
    }
    
    inline void reset_fatal_emergency_stop() {
        fatal_flags_ &= ~Fatal_EmergencyStop;  // Clear emergency stop flag
    }
    
    inline void set_fatal_motor_error() {
        fatal_flags_ |= Fatal_MotorError;  // Set motor configuration error flag
    }
    
    inline void reset_fatal_motor_error() {
        fatal_flags_ &= ~Fatal_MotorError;  // Clear motor configuration error flag
    }
    
    // ========================================================================
    // Alert error setters (inline)
    // ========================================================================
    
    inline void set_alert_dc_ovlo_soft() {
        alert_flags_ |= Alert_DC_OVLO_Soft;  // Set soft overvoltage alert
    }
    
    inline void reset_alert_dc_ovlo_soft() {
        alert_flags_ &= ~Alert_DC_OVLO_Soft;  // Clear soft overvoltage alert
    }
    
    inline void set_alert_dc_uvlo_soft() {
        alert_flags_ |= Alert_DC_UVLO_Soft;  // Set soft undervoltage alert
    }
    
    inline void reset_alert_dc_uvlo_soft() {
        alert_flags_ &= ~Alert_DC_UVLO_Soft;  // Clear soft undervoltage alert
    }
    
    inline void set_alert_oc_soft() {
        alert_flags_ |= Alert_OC_Soft;  // Set soft overcurrent alert
    }
    
    inline void reset_alert_oc_soft() {
        alert_flags_ &= ~Alert_OC_Soft;  // Clear soft overcurrent alert
    }
    
    inline void set_alert_overheat_soft() {
        alert_flags_ |= Alert_Overheat_Soft;  // Set soft overtemperature alert
    }
    
    inline void reset_alert_overheat_soft() {
        alert_flags_ &= ~Alert_Overheat_Soft;  // Clear soft overtemperature alert
    }
    
    inline void set_alert_stall() {
        alert_flags_ |= Alert_Stall;  // Set motor stall alert
    }
    
    inline void reset_alert_stall() {
        alert_flags_ &= ~Alert_Stall;  // Clear motor stall alert
    }
    
    // ========================================================================
    // Fatal error getters (inline)
    // ========================================================================
    
    inline bool get_fatal_dc_ovlo() const { return (fatal_flags_ & Fatal_DC_OVLO) != 0; }
    inline bool get_fatal_dc_uvlo() const { return (fatal_flags_ & Fatal_DC_UVLO) != 0; }
    inline bool get_fatal_phase1_oc() const { return (fatal_flags_ & Fatal_Phase1_OC) != 0; }
    inline bool get_fatal_phase2_oc() const { return (fatal_flags_ & Fatal_Phase2_OC) != 0; }
    inline bool get_fatal_phase3_oc() const { return (fatal_flags_ & Fatal_Phase3_OC) != 0; }
    inline bool get_fatal_oc() const { return (fatal_flags_ & Fatal_OC) != 0; }
    inline bool get_fatal_current_leakage() const { return (fatal_flags_ & Fatal_CurrentLeakage) != 0; }
    inline bool get_fatal_overheat() const { return (fatal_flags_ & Fatal_Overheat) != 0; }
    inline bool get_fatal_no_load() const { return (fatal_flags_ & Fatal_NoLoad) != 0; }
    inline bool get_fatal_no_current_feedback() const { return (fatal_flags_ & Fatal_NoCurrentFeedback) != 0; }
    inline bool get_fatal_emergency_stop() const { return (fatal_flags_ & Fatal_EmergencyStop) != 0; }
    inline bool get_fatal_motor_error() const { return (fatal_flags_ & Fatal_MotorError) != 0; }
    
    // ========================================================================
    // Alert error getters (inline)
    // ========================================================================
    
    inline bool get_alert_dc_ovlo_soft() const { return (alert_flags_ & Alert_DC_OVLO_Soft) != 0; }
    inline bool get_alert_dc_uvlo_soft() const { return (alert_flags_ & Alert_DC_UVLO_Soft) != 0; }
    inline bool get_alert_oc_soft() const { return (alert_flags_ & Alert_OC_Soft) != 0; }
    inline bool get_alert_overheat_soft() const { return (alert_flags_ & Alert_Overheat_Soft) != 0; }
    inline bool get_alert_stall() const { return (alert_flags_ & Alert_Stall) != 0; }
    
    // ========================================================================
    // General error status getters (inline)
    // ========================================================================
    
    inline bool has_fatal() const { return fatal_flags_ != 0; }  // Check if any fatal flag is set
    inline bool has_alert() const { return alert_flags_ != 0; }  // Check if any alert flag is set
    inline bool has_warning() const { return warning_flags_ != 0; }  // Check if any warning flag is set
    inline bool has_any_error() const { return (fatal_flags_ != 0) || (alert_flags_ != 0) || (warning_flags_ != 0); }  // Check if any error exists
    
    inline uint32_t get_fatal_flags() const { return fatal_flags_; }  // Get fatal error bitmask
    inline uint32_t get_alert_flags() const { return alert_flags_; }  // Get alert error bitmask
    inline uint32_t get_warning_flags() const { return warning_flags_; }  // Get warning error bitmask
    
    inline ErrorLevel get_error_level() const {
        if (fatal_flags_ != 0) return ErrorLevel_Fatal;  // Fatal errors have highest priority
        if (alert_flags_ != 0) return ErrorLevel_Alert;  // Alert errors have medium priority
        if (warning_flags_ != 0) return ErrorLevel_Warning;  // Warning errors have lowest priority
        return ErrorLevel_None;  // No errors
    }
    
    // ========================================================================
    // Error clearing (inline)
    // ========================================================================
    
    inline void clear_fatal() {
        fatal_flags_ = 0;  // Clear all fatal error flags
    }

    inline void clear_alert() {
        alert_flags_ = 0;  // Clear all alert error flags
    }
    
    inline void clear_warning() {
        warning_flags_ = 0;  // Clear all warning error flags
    }
    
    inline void clear_all() {
        clear_fatal();  // Clear all fatal errors
        clear_alert();  // Clear all alert errors
        clear_warning();  // Clear all warning errors
    }
};
