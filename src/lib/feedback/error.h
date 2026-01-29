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
    Fatal_OC = (1 << 5),                // General overcurrent
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
    
    uint32_t fatal_timestamp_;  // Timestamp when fatal error occurred (ms)
    uint32_t alert_timestamp_;  // Timestamp when alert error occurred (ms)
    
    bool fatal_active_;         // Fatal error active flag
    bool alert_active_;         // Alert error active flag
    
    static constexpr uint32_t ERROR_CLEAR_TIMEOUT_MS = 1000;  // Auto-clear after 1 second

public:
    // Constructor - initialize all error flags to zero
    ErrorManager() 
        : fatal_flags_(0)
        , alert_flags_(0)
        , warning_flags_(0)
        , fatal_timestamp_(0)
        , alert_timestamp_(0)
        , fatal_active_(false)
        , alert_active_(false)
    {}
    
    // ========================================================================
    // Fatal error setters (inline)
    // ========================================================================
    
    inline void set_fatal_dc_ovlo() {
        fatal_flags_ |= Fatal_DC_OVLO;
        fatal_timestamp_ = millis();
        fatal_active_ = true;
    }
    
    inline void set_fatal_dc_uvlo() {
        fatal_flags_ |= Fatal_DC_UVLO;
        fatal_timestamp_ = millis();
        fatal_active_ = true;
    }
    
    inline void set_fatal_phase1_oc() {
        fatal_flags_ |= Fatal_Phase1_OC;
        fatal_timestamp_ = millis();
        fatal_active_ = true;
    }
    
    inline void set_fatal_phase2_oc() {
        fatal_flags_ |= Fatal_Phase2_OC;
        fatal_timestamp_ = millis();
        fatal_active_ = true;
    }
    
    inline void set_fatal_phase3_oc() {
        fatal_flags_ |= Fatal_Phase3_OC;
        fatal_timestamp_ = millis();
        fatal_active_ = true;
    }
    
    inline void set_fatal_oc() {
        fatal_flags_ |= Fatal_OC;
        fatal_timestamp_ = millis();
        fatal_active_ = true;
    }
    
    inline void set_fatal_current_leakage() {
        fatal_flags_ |= Fatal_CurrentLeakage;
        fatal_timestamp_ = millis();
        fatal_active_ = true;
    }
    
    inline void set_fatal_overheat() {
        fatal_flags_ |= Fatal_Overheat;
        fatal_timestamp_ = millis();
        fatal_active_ = true;
    }
    
    inline void set_fatal_no_load() {
        fatal_flags_ |= Fatal_NoLoad;
        fatal_timestamp_ = millis();
        fatal_active_ = true;
    }
    
    inline void set_fatal_no_current_feedback() {
        fatal_flags_ |= Fatal_NoCurrentFeedback;
        fatal_timestamp_ = millis();
        fatal_active_ = true;
    }
    
    inline void set_fatal_emergency_stop() {
        fatal_flags_ |= Fatal_EmergencyStop;
        fatal_timestamp_ = millis();
        fatal_active_ = true;
    }
    
    inline void set_fatal_motor_error() {
        fatal_flags_ |= Fatal_MotorError;
        fatal_timestamp_ = millis();
        fatal_active_ = true;
    }
    
    // ========================================================================
    // Alert error setters (inline)
    // ========================================================================
    
    inline void set_alert_dc_ovlo_soft() {
        alert_flags_ |= Alert_DC_OVLO_Soft;
        alert_timestamp_ = millis();
        alert_active_ = true;
    }
    
    inline void set_alert_dc_uvlo_soft() {
        alert_flags_ |= Alert_DC_UVLO_Soft;
        alert_timestamp_ = millis();
        alert_active_ = true;
    }
    
    inline void set_alert_oc_soft() {
        alert_flags_ |= Alert_OC_Soft;
        alert_timestamp_ = millis();
        alert_active_ = true;
    }
    
    inline void set_alert_overheat_soft() {
        alert_flags_ |= Alert_Overheat_Soft;
        alert_timestamp_ = millis();
        alert_active_ = true;
    }
    
    inline void set_alert_stall() {
        alert_flags_ |= Alert_Stall;
        alert_timestamp_ = millis();
        alert_active_ = true;
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
    
    inline bool has_fatal() const { return fatal_active_; }
    inline bool has_alert() const { return alert_active_; }
    inline bool has_warning() const { return warning_flags_ != 0; }
    inline bool has_any_error() const { return fatal_active_ || alert_active_ || (warning_flags_ != 0); }
    
    inline uint32_t get_fatal_flags() const { return fatal_flags_; }
    inline uint32_t get_alert_flags() const { return alert_flags_; }
    inline uint32_t get_warning_flags() const { return warning_flags_; }
    
    inline ErrorLevel get_error_level() const {
        if (fatal_active_) return ErrorLevel_Fatal;
        if (alert_active_) return ErrorLevel_Alert;
        if (warning_flags_ != 0) return ErrorLevel_Warning;
        return ErrorLevel_None;
    }
    
    // ========================================================================
    // Error clearing (inline)
    // ========================================================================
    
    inline void clear_fatal() {
        fatal_flags_ = 0;
        fatal_active_ = false;
    }
    
    inline void clear_alert() {
        alert_flags_ = 0;
        alert_active_ = false;
    }
    
    inline void clear_warning() {
        warning_flags_ = 0;
    }
    
    inline void clear_all() {
        clear_fatal();
        clear_alert();
        clear_warning();
    }
    
    // ========================================================================
    // Automatic error timeout handling
    // ========================================================================
    
    inline void tick() {
        uint32_t current_time = millis();
        
        // Auto-clear fatal errors after timeout
        if (fatal_active_ && (current_time - fatal_timestamp_ >= ERROR_CLEAR_TIMEOUT_MS)) {
            clear_fatal();
        }
        
        // Auto-clear alert errors after timeout
        if (alert_active_ && (current_time - alert_timestamp_ >= ERROR_CLEAR_TIMEOUT_MS)) {
            clear_alert();
        }
    }
};
