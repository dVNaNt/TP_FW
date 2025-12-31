#include <Arduino.h>

#include "target.h"
#include "tunepulse.h"
#include "tunepulse_heart.h"
#include "setup/comp_setup.h"

TunePulseHeart TUNE_HEART;

// Track status changes
static MigrationDriverStatus last_status = DriverStatus_Error;

void setup() {
    tunepulse_init();
    TUNE_HEART.motor_params.set_current_limit(1500);

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
            case DriverStatus_MotorConnectionCalibration:
                SerialUSB.println("Testing Motor Connections");
                break;
            case DriverStatus_Ready:
                SerialUSB.println("Ready");
                break;
            case DriverStatus_Error:
                SerialUSB.println("Error");
                break;
        }
    }
    

    
    // Periodic status update every 1 second
    static uint32_t last_status_print = 0;
    uint32_t now = millis();
    if (now - last_status_print >= 1000) {
        last_status_print = now;
        SerialUSB.println("Status");
        switch (current_status) {
            case DriverStatus_CalibrateCurrent:
                SerialUSB.println("Calibrating current sensors...");
                break;
            case DriverStatus_Ready:
                SerialUSB.println("Ready");
                break;
            default:
                break;
        }
    }
}
