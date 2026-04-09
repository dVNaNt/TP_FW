#ifndef MOTOR_CALIBRATION_H
#define MOTOR_CALIBRATION_H

#include <Arduino.h>
#include <EEPROM.h>
#include "calibration_table.h"
#include "../types.h"

#define TUNE_PULSE_MAGIC_NUMBER 0x54504D43 // "TPMC" (TunePulse Motor Calibration)

// Structure representing the data to be stored in EEPROM
struct CalibrationData {
    uint32_t magic_number;       // Used to check if EEPROM contains valid data
    MotorType motor_type;        // Type of motor detected
    PhasePattern phase_pattern;  // Connection pattern detected
    int32_t direction;           // Motor direction
    int32_t total_resistance;    // Measured total resistance (mOhm)
    int32_t default_inductance;  // Inductance (uH)

    uint16_t offst_val;                 // Zero electrical angle offset
    size_t el_angle_div;                // Electrical angle divider
    int16_t ecc_table[ECC_TABLE_SIZE];  // Eccentricity correction table

    uint32_t checksum;           // CRC checksum for data integrity
};

class PersistentCalibration {
public:
    static bool load_from_eeprom(CalibrationData& data);
    static void save_to_eeprom(const CalibrationData& data);

private:
    static uint32_t calculate_checksum(const CalibrationData& data);
};

#endif // MOTOR_CALIBRATION_H
