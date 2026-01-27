#include <Arduino.h>

// NOTE:
// - For CLN17_V3 "release" we want the normal motor firmware path.
// - The bring-up / hello-world test is gated by CLN17V3_BRINGUP to avoid
//   "hang" on SerialUSB when no host is connected.
#if defined(TARGET_CLN17V3) && defined(CLN17V3_BRINGUP)
// CLN17V3 Bring-up Test Code - Hello World with LED blink and analog reading

#include "target.h"  // Pin definitions for CLN17V3
#include "stm32g4xx_ll_bus.h"   // Bus peripheral library for clock enable
#include "stm32g4xx_ll_gpio.h"  // GPIO low-level library

// Function to initialize GPIO for LEDs
void LED_Init() {
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};  // GPIO initialization structure
    
    // Enable peripheral clocks for GPIO ports
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);  // Enable GPIOB clock
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);  // Enable GPIOC clock
    
    // Configure LED pins as open-drain outputs (active low)
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;           // Output mode
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;       // Low speed sufficient for LEDs
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN; // Open-drain for active-low LEDs
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;               // No pull resistors
    
    // PB1 (LED_RED), PB2 (LED_BLU)
    GPIO_InitStruct.Pin = LL_GPIO_PIN_1 | LL_GPIO_PIN_2;  // Configure red and blue LED pins
    LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_1 | LL_GPIO_PIN_2);  // Turn LEDs off (high = off)
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);  // Initialize GPIOB pins
    
    // PC4 (LED_GRN)
    GPIO_InitStruct.Pin = LL_GPIO_PIN_4;  // Configure green LED pin
    LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_4);  // Turn LED off (high = off)
    LL_GPIO_Init(GPIOC, &GPIO_InitStruct);  // Initialize GPIOC pin
}

// Function to initialize analog pins as inputs
void Analog_Init() {
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};  // GPIO initialization structure
    
    // Enable GPIO clocks for analog pins
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);  // Enable GPIOA clock
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);  // Enable GPIOB clock
    
    // Configure analog pins as analog inputs (no ADC, just read voltage level)
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;  // Analog mode
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;      // No pull-up/pull-down
    
    // PA0 (VSENS), PA1 (I_B), PA2 (PWR_V), PA3 (I_A)
    GPIO_InitStruct.Pin = LL_GPIO_PIN_0 | LL_GPIO_PIN_1 | LL_GPIO_PIN_2 | LL_GPIO_PIN_3;  // Configure analog pins
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);  // Initialize GPIOA analog pins
    
    // PB0 (I_C)
    GPIO_InitStruct.Pin = LL_GPIO_PIN_0;  // Configure analog pin
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);  // Initialize GPIOB analog pin
}

void setup() {
    // Initialize USB serial communication
    SerialUSB.begin();  // Start USB serial
    
    // Wait for USB connection
    delay(2000);  // 2 second delay for USB enumeration
    
    // Print startup message
    SerialUSB.println("=== CLN17V3 Hello World Test ===");  // Print header
    SerialUSB.println("Initializing...");  // Print status
    
    // Initialize LEDs
    LED_Init();  // Configure LED GPIO pins
    SerialUSB.println("LEDs initialized");  // Confirm initialization
    
    // Initialize analog pins
    Analog_Init();  // Configure analog GPIO pins
    SerialUSB.println("Analog pins initialized");  // Confirm initialization
    
    SerialUSB.println("Starting LED blink sequence...");  // Print status
    SerialUSB.println("Note: Analog values shown using Arduino analogRead()");  // Info message
}

void loop() {
    // LED blink sequence: Blue -> Red -> Green
    
    // Turn on BLUE LED
    SerialUSB.println("\n--- BLUE LED ON ---");  // Print status
    LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_2);  // Blue LED on (low = on)
    delay(500);  // Wait 500ms
    
    // Read and print all analog pins using Arduino analogRead
    SerialUSB.println("Analog Pin Readings (0-4095):");  // Print header
    
    uint16_t vsens_raw = analogRead(PA_0);  // PA0 - VSENS (raw ADC value)
    SerialUSB.print("  PA0 (VSENS):  ");  // Print pin name
    SerialUSB.print(vsens_raw);  // Print raw ADC value
    // Convert to voltage: ADC range 0-4095 maps to 0-64000 mV (0-64V)
    float vsens_volts = (vsens_raw * 64000.0f) / 4095.0f / 1000.0f;  // Convert to volts
    SerialUSB.print(" (");  // Print opening bracket
    SerialUSB.print(vsens_volts, 2);  // Print voltage with 2 decimal places
    SerialUSB.println(" V)");  // Print unit and newline
    
    uint16_t i_b = analogRead(PA_1);  // PA1 - I_B
    SerialUSB.print("  PA1 (I_B):    ");  // Print pin name
    SerialUSB.println(i_b);  // Print ADC value
    
    uint16_t pwr_v_raw = analogRead(PA_2);  // PA2 - PWR_V (raw ADC value)
    SerialUSB.print("  PA2 (PWR_V):  ");  // Print pin name
    SerialUSB.print(pwr_v_raw);  // Print raw ADC value
    // Convert to voltage: ADC range 0-4095 maps to 0-64000 mV (0-64V)
    float pwr_v_volts = (pwr_v_raw * 64000.0f) / 4095.0f / 1000.0f;  // Convert to volts
    SerialUSB.print(" (");  // Print opening bracket
    SerialUSB.print(pwr_v_volts, 2);  // Print voltage with 2 decimal places
    SerialUSB.println(" V)");  // Print unit and newline
    
    uint16_t i_a = analogRead(PA_3);  // PA3 - I_A
    SerialUSB.print("  PA3 (I_A):    ");  // Print pin name
    SerialUSB.println(i_a);  // Print ADC value
    
    uint16_t i_c = analogRead(PB_0);  // PB0 - I_C
    SerialUSB.print("  PB0 (I_C):    ");  // Print pin name
    SerialUSB.println(i_c);  // Print ADC value
    
    // Turn off BLUE LED
    LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_2);  // Blue LED off (high = off)
    delay(500);  // Wait 500ms
    
    // Turn on RED LED
    SerialUSB.println("\n--- RED LED ON ---");  // Print status
    LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_1);  // Red LED on (low = on)
    delay(500);  // Wait 500ms
    
    // Turn off RED LED
    LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_1);  // Red LED off (high = off)
    delay(500);  // Wait 500ms
    
    // Turn on GREEN LED
    SerialUSB.println("\n--- GREEN LED ON ---");  // Print status
    LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_4);  // Green LED on (low = on)
    delay(500);  // Wait 500ms
    
    // Turn off GREEN LED
    LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_4);  // Green LED off (high = off)
    delay(500);  // Wait 500ms
}

#else
// Original code for other targets
#include "target.h"
#include "tunepulse.h"
#include "tunepulse_heart.h"

TunePulseHeart TUNE_HEART;

// Global pointer for comparator interrupt handlers to access error manager
TunePulseHeart* g_heart_ptr = &TUNE_HEART;

// Track status changes
static MigrationDriverStatus last_status = DriverStatus_Error;

void setup() {
    tunepulse_init();
    TUNE_HEART.motor_params.set_current_limit(2500);

    SerialUSB.begin();
    
    // Wait for USB connection
    delay(2000);
    
    // Avoid "hang" if USB host is not connected/enumerated yet
    if (SerialUSB) {
        SerialUSB.println("=== Motor Test Starting ===");
        SerialUSB.println("Waiting for calibration...");
    }

}


void loop() {
    TUNE_HEART.tick();
    
    // Update overcurrent fault state (handles 1s lockout timeout)
    COMP_Update_Fault_State();
    
    // Print driver status changes
    MigrationDriverStatus current_status = TUNE_HEART.get_driver_status();
    if (current_status != last_status) {
        last_status = current_status;

        // Avoid blocking prints when USB is not connected
        if (!SerialUSB) return;

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
