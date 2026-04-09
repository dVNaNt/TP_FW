#include "motor_calibration.h"

uint32_t PersistentCalibration::calculate_checksum(const CalibrationData& data) {
    uint32_t sum = 0;
    const uint8_t* ptr = reinterpret_cast<const uint8_t*>(&data);

    // Checksum skips the last 4 bytes (the checksum field itself)
    size_t size_to_check = sizeof(CalibrationData) - sizeof(uint32_t);
    for (size_t i = 0; i < size_to_check; i++) {
        sum += ptr[i];
    }
    return sum;
}

bool PersistentCalibration::load_from_eeprom(CalibrationData& data) {
    uint8_t* ptr = reinterpret_cast<uint8_t*>(&data);

    for (size_t i = 0; i < sizeof(CalibrationData); i++) {
        ptr[i] = EEPROM.read(i);
    }

    // Check if magic number matches to confirm this is valid TunePulse data
    if (data.magic_number != TUNE_PULSE_MAGIC_NUMBER) {
        return false;
    }

    // Validate checksum to prevent corrupted data loading
    if (data.checksum != calculate_checksum(data)) {
        return false;
    }

    return true;
}

void PersistentCalibration::save_to_eeprom(const CalibrationData& data) {
    CalibrationData copy = data;
    copy.magic_number = TUNE_PULSE_MAGIC_NUMBER;
    copy.checksum = calculate_checksum(copy);

    const uint8_t* ptr = reinterpret_cast<const uint8_t*>(&copy);

    for (size_t i = 0; i < sizeof(CalibrationData); i++) {
        EEPROM.update(i, ptr[i]);
    }
}
