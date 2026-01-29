#include <Arduino.h>

#ifdef TARGET_CLN17V3
// CLN17V3 - Full TunePulse motor control

#include "target.h"
#include "tunepulse.h"
#include "tunepulse_heart.h"

TunePulseHeart TUNE_HEART(PWM_TIMER_FREQ_HZ,
                          PWM_TIMER_PERIOD,
                          SYS_PARAMS::MAX_SUPPLY_FS,
                          SYS_PARAMS::IA_MAX_CURRENT_FS,
                          SYS_PARAMS::IB_MAX_CURRENT_FS,
                          SYS_PARAMS::IC_MAX_CURRENT_FS);  // Main motor control object

// Global pointer for comparator interrupt handlers to access error manager
TunePulseHeart* g_heart_ptr = &TUNE_HEART;

// Track status changes
static DriverStatus last_status = DriverStatus_Error;

void setup() {
    // Initialize all hardware modules
    tunepulse_init();

    // Set current limit (in mA)
    TUNE_HEART.motor_params.set_current_limit(2500);

    // Link comparator DAC update callbacks to current sense
    TUNE_HEART.feedback.current.set_overcurrent_hard_callback(CurrentSense::CH1,
                                                              COMP_Update_DAC_Channel_A  // Channel A (COMP2, Phase A, I_A)

    );

    // Link comparator DAC update callbacks to current sense
    TUNE_HEART.feedback.current.set_overcurrent_hard_callback(
        CurrentSense::CH2, COMP_Update_DAC_Channel_C  // Channel A (COMP2, Phase A, I_A)

    );

    // Link comparator DAC update callbacks to current sense
    TUNE_HEART.feedback.current.set_overcurrent_hard_callback(CurrentSense::CH3,
                                                              COMP_Update_DAC_Channel_C  // Channel A (COMP2, Phase A, I_A)

    );

    // TUNE_HEART.feedback.current.set_overcurrent_hard_ma(1000);

    // Initialize USB serial communication
    SerialUSB.begin();

    // Wait for USB connection
    delay(2000);

    SerialUSB.println("\n\n=== CLN17V3 Motor Control Starting ===");
    SerialUSB.println("Hardware initialized successfully");
    SerialUSB.println("Hard overcurrent limit set to 500mA per channel");
    SerialUSB.println("Comparator callbacks linked to DAC updates");
    SerialUSB.println("Waiting for calibration...");
    SerialUSB.flush();
}

void loop() {
    // Main motor control tick
    TUNE_HEART.tick();

    // Update overcurrent fault state (handles 1s lockout timeout)
    COMP_Update_Fault_State();

    // Print driver status changes
    DriverStatus current_status = TUNE_HEART.get_driver_status();
    if (current_status != last_status) {
        last_status = current_status;

        SerialUSB.print("Driver status: ");
        switch (current_status) {
            case DriverStatus_Calibrating:
                SerialUSB.println("Calibrating");
                break;
            case DriverStatus_CalibrateCurrent:
                SerialUSB.println("Calibrating Current Sensors");
                break;
            case DriverStatus_CalibratingMotor:
                SerialUSB.println("Calibrating Motor");
                break;
            case DriverStatus_Ready:
                SerialUSB.println("Ready");
                // Print motor parameters after successful calibration
                TUNE_HEART.motor_params.print(SerialUSB);
                // Set hard overcurrent limit to 500mA for all channels

                break;
            case DriverStatus_Error:
                SerialUSB.println("Error");
                break;
        }
        SerialUSB.flush();
    }
}

#else
// Original code for other targets
#include "target.h"
#include "tunepulse.h"
#include "tunepulse_heart.h"

// Initialize TunePulseHeart with Radix board parameters
// freq_hz=20000, pwm_res=4250, max_supply, iA_fs, iB_fs, iC_fs
TunePulseHeart TUNE_HEART(20000,
                          4250,
                          SYS_PARAMS::MAX_SUPPLY_FS,
                          SYS_PARAMS::IA_MAX_CURRENT_FS,
                          SYS_PARAMS::IB_MAX_CURRENT_FS,
                          SYS_PARAMS::IC_MAX_CURRENT_FS);

// Global pointer for comparator interrupt handlers to access error manager
TunePulseHeart* g_heart_ptr = &TUNE_HEART;

// Track status changes
static MigrationDriverStatus last_status = DriverStatus_Error;

void setup() {
    tunepulse_init();

    SerialUSB.begin();

    // Wait for USB connection
    delay(2000);

    SerialUSB.println("=== Motor Test Starting ===");
    SerialUSB.println("Waiting for calibration...");
}

void loop() {
    TUNE_HEART.tick();

    // Update overcurrent fault state (handles 1s lockout timeout)
    COMP_Update_Fault_State();

    // Print driver status changes
    MigrationDriverStatus current_status = TUNE_HEART.get_driver_status();
    if (current_status != last_status) {
        last_status = current_status;

        SerialUSB.print("Driver status: ");
        switch (current_status) {
            case DriverStatus_Calibrating:
                SerialUSB.println("Calibrating");
                break;
            case DriverStatus_CalibrateCurrent:
                SerialUSB.println("Calibrating Current Sensors");
                break;
            case DriverStatus_CalibratingMotor:
                SerialUSB.println("Calibrating Motor");
                break;
            case DriverStatus_Ready:
                SerialUSB.println("Ready");
                // Print motor parameters after successful calibration
                TUNE_HEART.motor_params.print(SerialUSB);
                break;
            case DriverStatus_Error:
                SerialUSB.println("Error");
                break;
        }
    }
}
#endif
