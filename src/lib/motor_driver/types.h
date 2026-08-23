#pragma once

/**
 * @file types.h
 * @brief Shared enumerations and status structures for the motor driver layer.
 *
 * Defines:
 * - @c MotorType      : supported motor categories (stepper / BLDC-PMSM).
 * - @c PhasePattern   : physical phase wiring pattern; the enumerator VALUES
 *                       double as calibration angle offsets in Q15-ish units.
 * - @c OperationStatus: high-level driver lifecycle states.
 * - @c DriverStatus   : packed one-byte status word combining the operation
 *                       status with per-subsystem fault flags.
 */

#include "../common.h"

/**
 * Motor type enumeration with predefined motor categories.
 *
 * The numeric values are historical protocol identifiers:
 * - 4 = stepper (two independent coils, direct alpha/beta mapping)
 * - 6 = BLDC/PMSM (three phases, Clarke/Park transforms apply)
 *
 * @note Older revisions also listed a DC-motor type (value 1); it is no
 *       longer present in this enum, although some comments elsewhere still
 *       reference "1 = DC".
 */
typedef enum : int8_t {
    MotorType_Invalid = 0,  ///< Unknown / unsupported motor type.
    MotorType_STEP = 4,     ///< Two-phase stepper motor.
    MotorType_BLDC = 6,     ///< Three-phase BLDC / PMSM motor.
} MotorType;

/**
 * Phase-pattern enumeration for 3-phase remapping (output ordering).
 *
 * @warning The enumerator VALUES are NOT small indices: they are used as
 *          signed angle/polarity constants by the calibration logic.
 * - PhasePattern_CBA is the default for BLDC motors (positive calibration
 *   polarity).
 * - Calibration sequence order: CBA(0) -> ACB(1) -> ABC(2).
 */
typedef enum : int16_t {
    PhasePattern_ABC = 0,       ///< Direct wiring; negative calibration polarity.
    PhasePattern_CBA = -16384,  ///< Default BLDC pattern; positive calibration polarity.
    PhasePattern_ACB = 32767,   ///< Swapped B/C wiring; negative calibration polarity.
} PhasePattern;

/**
 * High-level driver lifecycle status.
 *
 * @note The historical comment claimed these values are "stored in bits 0-1",
 *       but DriverStatus actually reserves FOUR bits (0-3) for this enum, so
 *       all five named states plus NotTested fit without truncation.
 */
enum class OperationStatus : uint8_t {
    Ready = 0,             ///< Driver fully operational (must be 0 for fast zero-check).
    DriverCalibration = 1, ///< Driver-layer (current-sense) calibration running.
    MotorCalibration = 2,  ///< Motor (electrical-angle) calibration running.
    Disabled = 3,          ///< Driver disabled by command.
    Error = 4,             ///< Driver in error state.
    NotTested = 0xFF       ///< Self-test has not been executed yet.
};

/**
 * Packed driver setup/status byte.
 *
 * Uses a union so the whole state can be tested with a single zero-compare:
 * when status_word == 0 the driver is fully ready (Ready + all flags clear).
 *
 * Bit layout of the single byte:
 * - Bits 0-3 : OperationStatus value
 * - Bit  4   : current_sensors_faulty
 * - Bit  5   : current_not_calibrated
 * - Bit  6   : motor_type_not_verified
 * - Bit  7   : voltage_unsafe
 */
struct DriverStatus {
    union {
        struct {
            // Byte 0: Driver status and error flags
            uint8_t driver_status : 4;            ///< Bits 0-3: OperationStatus value.
            uint8_t current_sensors_faulty : 1;   ///< Bit 4: 0 = sensors functional, 1 = faulty.
            uint8_t current_not_calibrated : 1;   ///< Bit 5: 0 = calibrated, 1 = not calibrated.
            uint8_t motor_type_not_verified : 1;  ///< Bit 6: 0 = verified, 1 = not verified.
            uint8_t voltage_unsafe : 1;           ///< Bit 7: 0 = safe range, 1 = unsafe.
        };
        uint8_t not_ready;  ///< Whole-byte view for an ultra-fast zero-check.
    };

    /// Constructor: initialize to "not ready" state (all bits set).
    DriverStatus() : not_ready(0xFF) {}

    /**
     * Ultra-fast readiness check: single byte comparison against zero.
     * True only when driver_status == Ready AND every fault flag is clear.
     * Compiles to a single CMP instruction on ARM Cortex-M.
     */
    inline bool is_ready() const {
        return not_ready == 0;
    }

    /// Store a new OperationStatus into bits 0-3.
    inline void set_driver_status(OperationStatus status) {
        driver_status = (uint8_t)status;
    }

    /// Read back the current OperationStatus from bits 0-3.
    inline OperationStatus get_driver_status() const { return static_cast<OperationStatus>(driver_status); }
};