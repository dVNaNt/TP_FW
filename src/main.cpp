#include <Arduino.h>

#include "tunepulse.h"
#include "lib/math/motion/ramp_gen.h"  // Include S-curve ramp generator
#include "lib/math/controllers/pi.h"
#include "lib/feedback/MT6835_encoder.h"
#include <SPI.h>
#include <SimpleCANio.h>

// ============================================================================
// Motion Pattern Generation - Continuous Loop with S-Curve Ramp Generator
// ============================================================================

// Global MT6835 encoder instance
#if defined(TARGET_CLN17V3)
SPIClass CTRL_SPI(PB5, PB4, PB3);
MT6835Encoder outer_encoder(CTRL_SPI, PA8);
#else
// Ensure these defaults exist, or just fail cleanly for other boards
SPIClass CTRL_SPI(PB5, PB4, PB3);
MT6835Encoder outer_encoder(CTRL_SPI, PA8);
#endif

// Global CAN interface
#if defined(TARGET_CLN17V3)
CANio can(PB8, PB9);
#else
CANio can(PINOUT::CAN_RX, PINOUT::CAN_TX);
#endif

uint32_t MY_NODE_ID = 0x101;
volatile float target_linear_pos = 0.0f;

// Operating Modes
enum class ControlMode {
    OptionalSCurve,   // Generate an S-curve internally (current default)
    DirectCAN         // Follow CAN target linear position directly
};

ControlMode current_mode = ControlMode::DirectCAN;

// State machine for continuous motion pattern generation
enum class PatternState {
    WaitingForCalibration,  // Waiting for motor calibration to complete
    GeneratingForward,      // Generating forward motion pattern
    GeneratingReverse,      // Generating reverse motion pattern
    Running                 // Motion pattern is executing
};

PatternState motion_state = PatternState::WaitingForCalibration;
uint32_t last_pattern_time = 0;

// Motion parameters (constant for all patterns)
volatile int32_t START_POSITION_Q16 = 0;                    // Starting position: 0 revolutions in Q16.16
volatile int32_t TRAVEL_DISTANCE_Q16 = 15 * 65536;         // Travel distance: 15 revolutions in Q16.16
volatile float ACCELERATION_RPS2 = 5.0f;                    // Acceleration: 5 rev/s²
volatile float TARGET_VELOCITY_RPS = 5.0f;                  // Target velocity: 5 rev/s
volatile float START_VELOCITY_RPS = 0.0f;                   // Start velocity: 0 rev/s
volatile float END_VELOCITY_RPS = START_VELOCITY_RPS;                     // End velocity: 0 rev/s
volatile float JERK_RPS3 = 5.0f;                            // Jerk: 0 rev/s³ (trapezoidal profile)
volatile float MIN_PHASE_TIME_S = 0.00005f;                    // Minimum phase duration threshold

// Current position tracking for alternating patterns
int32_t current_position_q16 = START_POSITION_Q16;

/// @brief Generate forward motion pattern using S-curve ramp generator
/// Moves from current position to current position + 15 revolutions
void generate_forward_pattern() {
    // Setup S-curve parameters for forward motion
    SCurveParams params;
    params.pos_start = 0;                          // Start from current position
    params.pos_end = TRAVEL_DISTANCE_Q16;      // Move forward 15 revolutions
    params.vel_start = START_VELOCITY_RPS;                            // Start from rest
    params.vel_end = END_VELOCITY_RPS;                                // End at rest
    params.acc_max = ACCELERATION_RPS2;                               // Acceleration limit
    params.dec_max = ACCELERATION_RPS2;                               // Deceleration limit (same as acceleration)
    params.j1 = JERK_RPS3;                                            // Jerk for acceleration ramp-up
    params.j2 = JERK_RPS3;                                            // Jerk for acceleration ramp-down
    params.j3 = JERK_RPS3;                                            // Jerk for deceleration ramp-up
    params.j4 = JERK_RPS3;                                            // Jerk for deceleration ramp-down
    params.vel_lim = TARGET_VELOCITY_RPS;                             // Maximum velocity limit
    params.threshold = MIN_PHASE_TIME_S;                              // Minimum phase time threshold
    
    // Generate and push segments to motion executor
    bool success = SCurvePlanner::generate_and_push(params, TUNE_HEART.motion_executor);
    
    // if (success) {
    //     current_position_q16 = params.pos_end;  // Update current position
    //     TP_LOG("Forward pattern generated (0 → +15 rev)");
    // } else {
    //     TP_LOG("ERROR: Failed to generate forward pattern (buffer full)");
    // }
}

/// @brief Generate reverse motion pattern using S-curve ramp generator
/// Moves from current position to current position - 15 revolutions
void generate_reverse_pattern() {
    // Setup S-curve parameters for reverse motion
    SCurveParams params;
    params.pos_start = TRAVEL_DISTANCE_Q16;                          // Start from current position
    params.pos_end = 0;      // Move backward 15 revolutions
    params.vel_start = START_VELOCITY_RPS;                            // Start from rest
    params.vel_end = END_VELOCITY_RPS;                                // End at rest
    params.acc_max = -ACCELERATION_RPS2;                               // Acceleration limit
    params.dec_max = -ACCELERATION_RPS2;                               // Deceleration limit (same as acceleration)
    params.j1 = -JERK_RPS3;                                            // Jerk for acceleration ramp-up
    params.j2 = -JERK_RPS3;                                            // Jerk for acceleration ramp-down
    params.j3 = -JERK_RPS3;                                            // Jerk for deceleration ramp-up
    params.j4 = -JERK_RPS3;                                            // Jerk for deceleration ramp-down
    params.vel_lim = -TARGET_VELOCITY_RPS;                            // Maximum velocity limit (negative for reverse)
    params.threshold = MIN_PHASE_TIME_S;                              // Minimum phase time threshold
    
    // Generate and push segments to motion executor
    bool success = SCurvePlanner::generate_and_push(params, TUNE_HEART.motion_executor);
    
    // if (success) {
    //     current_position_q16 = params.pos_end;  // Update current position
    //     TP_LOG("Reverse pattern generated (+15 → 0 rev)");
    // } else {
    //     TP_LOG("ERROR: Failed to generate reverse pattern (buffer full)");
    // }
}

void setup() {
    delay(2000);

    SerialUSB.begin();
    tunepulse_init();

    SerialUSB.println("Going LOOP");

    // Initialize the outer MT6835 encoder
    CTRL_SPI.begin();
    if (outer_encoder.init()) {
        SerialUSB.println("MT6835 Encoder Initialized Successfully");
    } else {
        SerialUSB.println("Failed to initialize MT6835 Encoder");
    }

    // Initialize CAN-FD interface
    if (can.begin(1000000)) {
        SerialUSB.println("CAN-FD Initialized Successfully");
    } else {
        SerialUSB.println("Failed to initialize CAN-FD");
    }
    
    // Configure position controller (cascaded position-velocity control)
    SerialUSB.println("Position controller configured");
    
    // Enable position control mode
    TUNE_HEART.use_position_control = true;
    SerialUSB.println("Position control mode enabled");
    
    // Initial motion pattern will be generated in loop() after calibration
    SerialUSB.println("Motion pattern will start after calibration");
}

void loop() {  

    // State machine for continuous motion pattern generation
    switch (motion_state) {
        case PatternState::WaitingForCalibration:
            // Wait for angle calibration to complete
            if (TUNE_HEART.angle_calibrator.is_ready()) {
                TP_LOG("Calibration complete, starting motion pattern");
                motion_state = PatternState::GeneratingForward;
                last_pattern_time = millis();
            }
            break;
            
        case PatternState::GeneratingForward:
            if (current_mode == ControlMode::OptionalSCurve) {
                generate_forward_pattern();
                motion_state = PatternState::Running;
                last_pattern_time = millis();
            } else {
                motion_state = PatternState::Running;
            }
            break;
            
        case PatternState::GeneratingReverse:
            if (current_mode == ControlMode::OptionalSCurve) {
                generate_reverse_pattern();
                motion_state = PatternState::Running;
                last_pattern_time = millis();
            } else {
                motion_state = PatternState::Running;
            }
            break;
            
        case PatternState::Running:
            // Handle CAN inputs if in DirectCAN mode
            if (can.available()) {
                CanMsg msg = can.read();
                if (msg.getStandardId() == MY_NODE_ID && msg.data_length == 4) {
                    memcpy((void*)&target_linear_pos, msg.data, 4);
                }
            }

            if (current_mode == ControlMode::OptionalSCurve) {
                // In S-Curve Mode, the inner TunePulseHeart::tick() naturally pulls from
                // the motion_executor. We disable the cascaded PID so it doesn't interfere.
                TUNE_HEART.cascaded_loop_enabled = false;

                // Check if motion pattern has completed
                // Pattern duration: 15 rev at 5 rev/s = 3s + accel/decel time ≈ 5s total
                uint32_t pattern_duration_ms = 5000; // 5 seconds with margin

                if (millis() - last_pattern_time > pattern_duration_ms) {
                    // Pattern completed, switch direction
                    static bool forward = false; // Start with reverse after initial forward
                    if (forward) {
                        motion_state = PatternState::GeneratingForward;
                        TP_LOG("Switching to forward pattern");
                    } else {
                        motion_state = PatternState::GeneratingReverse;
                        TP_LOG("Switching to reverse pattern");
                    }
                    forward = !forward; // Toggle direction
                }
            } else {
                // In tightly coupled Cascaded Dual-Loop Control, we simply feed the
                // `target_linear_pos` straight into the `TunePulseHeart` globally,
                // and `TunePulseHeart` handles reading the outer encoder and computing the PID
                // inside its fast tick() interrupt/loop.
                TUNE_HEART.target_linear_pos = target_linear_pos;
                TUNE_HEART.cascaded_loop_enabled = true;
            }
            break;
    }
}
