#pragma once

#include "../common.h"

/// MotorType enumeration with predefined motor types
typedef enum : int8_t {
    MotorType_Invalid = 0,
    MotorType_DC = 1,
    MotorType_STEP = 4,
    MotorType_BLDC = 6,
} MotorType;

/// PhasePattern enumeration for 3-phase remap (output ordering)
/// PhasePattern_CBA is default for BLDC motors (positive calibration polarity)
/// Calibration sequence: CBA(0) -> ACB(1) -> ABC(2)
typedef enum : int16_t {
    PhasePattern_ABC = 0,       // Negative calibration polarity
    PhasePattern_CBA = -16384,  // Default BLDC pattern, positive calibration polarity
    PhasePattern_ACB = 32767,   // Negative calibration polarity
} PhasePattern;

// Driver status enum values (stored in bits 0-1)
enum class OperationStatus : uint8_t {
    Ready = 0,  // Driver fully operational (must be 0 for fast zero-check)
    DriverCalibration = 1,
    MotorCalibration = 2,  // Driver in calibration mode
    Disabled = 3,          // Driver disabled
    Error = 4,             // Driver in error state
    NotTested = 0xF
};

// Driver setup status structure - highly optimized for memory and CPU efficiency
// Uses union to allow both bitfield access and direct word comparison
// When status_word == 0, driver is fully ready (all checks passed, calibration complete)
struct DriverStatus {
    union {
        struct {
            // Byte 0: Driver status and error flags
            uint8_t driver_status : 4;  // Bits 0-3: Driver status enum (0 = Ready, 1 = Calibrating, 2 = Error, 3 = Disabled)
            uint8_t current_sensors_faulty : 1;   // Bit 4: 0 = sensors functional, 1 = sensors faulty
            uint8_t current_not_calibrated : 1;   // Bit 5: 0 = current sensors calibrated, 1 = not calibrated
            uint8_t motor_type_not_verified : 1;  // Bit 6: 0 = motor type verified, 1 = not verified
            uint8_t voltage_unsafe : 1;           // Bit 7: 0 = voltage in safe range, 1 = voltage unsafe
        };
        uint8_t not_ready;  // Direct 16-bit word access for ultra-fast zero-check
    };

    // Constructor: initialize to "not ready" state (all flags set)
    DriverStatus() : not_ready(0xFF) {}  // Set all bits to 1 (not ready, calibration incomplete)

    // Ultra-fast ready check: single 16-bit comparison with zero
    // Returns true only when driver_status == Ready AND all error flags == 0 AND motor_calibstatus == Success (0)
    inline bool is_ready() const {
        return not_ready == 0;  // Single CMP instruction on ARM (MOVW + CMP)
    }

    // Set driver status (bits 0-3)
    inline void set_driver_status(OperationStatus status) {
        driver_status = (uint8_t)status;  // Direct bitfield assignment
    }

    // Get driver status
    inline OperationStatus get_driver_status() const { return static_cast<OperationStatus>(driver_status); }
};
