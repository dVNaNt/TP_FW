// Motor Test Debug Example
// Copy this to src/main.cpp to debug motor test

#include <Arduino.h>

#include "target.h"
#include "tunepulse.h"
#include "tunepulse_heart.h"

TunePulseHeart TUNE_HEART;

// Flags
static bool motor_test_printed = false;
static MigrationDriverStatus last_status = DriverStatus_Error;

void setup() {
    tunepulse_init();

    SerialUSB.begin();
    
    // Wait for USB connection
    delay(2000);
    
    SerialUSB.println("=== Motor Test Debug ===");
    SerialUSB.println("Firmware started");
    SerialUSB.print("Initial status: ");
    SerialUSB.println((int)TUNE_HEART.get_driver_status());

    pinMode(PINOUT::LED_GRN, OUTPUT);
    digitalWrite(PINOUT::LED_GRN, LOW);
}

void loop() {
    TUNE_HEART.tick();
    
    // Print driver status changes
    MigrationDriverStatus current_status = TUNE_HEART.get_driver_status();
    if (current_status != last_status) {
        last_status = current_status;
        
        SerialUSB.print("[");
        SerialUSB.print(millis());
        SerialUSB.print("ms] Status changed to: ");
        
        switch (current_status) {
            case DriverStatus_Calibrating:
                SerialUSB.println("Calibrating (0)");
                break;
            case DriverStatus_CalibrateCurrent:
                SerialUSB.println("Calibrating Current Sensors (1)");
                break;
            case DriverStatus_MotorConnectionCalibration:
                SerialUSB.println("Testing Motor Connections (2)");
                break;
            case DriverStatus_Ready:
                SerialUSB.println("Ready (3)");
                break;
            case DriverStatus_Error:
                SerialUSB.println("Error (4)");
                break;
            default:
                SerialUSB.print("Unknown (");
                SerialUSB.print((int)current_status);
                SerialUSB.println(")");
                break;
        }
    }
    
    // Print motor test results once when complete
    if (TUNE_HEART.is_motor_test_complete() && !motor_test_printed) {
        SerialUSB.println();
        SerialUSB.println("=== MOTOR TEST COMPLETE ===");
        SerialUSB.println();
        TUNE_HEART.print_motor_test_results(SerialUSB);
        SerialUSB.println();
        SerialUSB.println("=== END OF RESULTS ===");
        motor_test_printed = true;
    }
    
    // Periodic heartbeat every 1 second
    static uint32_t last_heartbeat = 0;
    uint32_t now = millis();
    if (now - last_heartbeat >= 1000) {
        last_heartbeat = now;
        
        SerialUSB.print("[");
        SerialUSB.print(now);
        SerialUSB.print("ms] Heartbeat - Status: ");
        SerialUSB.print((int)current_status);
        SerialUSB.print(", Test complete: ");
        SerialUSB.println(TUNE_HEART.is_motor_test_complete() ? "YES" : "NO");
    }
    
    // Blink LED
    static uint32_t last_blink = 0;
    if (now - last_blink >= 500) {
        last_blink = now;
        digitalWrite(PINOUT::LED_GRN, !digitalRead(PINOUT::LED_GRN));
    }
}
