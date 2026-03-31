#include <Arduino.h>

#include "tunepulse.h"
#include "lib/math/motion/ramp_gen.h"  // Include S-curve ramp generator

// ============================================================================
// Motion Pattern Generation - Continuous Loop with S-Curve Ramp Generator
// ============================================================================

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
    
    // Configure position controller (cascaded position-velocity control)
    // Position loop gains (outer loop) - NEGATIVE for proper error correction
    float pos_kp = -2.0f;    // Position proportional gain (negative)
    float pos_ki = -0.1f;    // Position integral gain [1/s] (negative)
    float pos_kd = -0.05f;   // Position derivative gain [s] (negative)
    
    // Velocity loop gains (inner loop) - NEGATIVE for proper error correction
    float vel_kp = -0.5f;    // Velocity proportional gain (negative)
    float vel_ki = -5.0f;    // Velocity integral gain [1/s] (negative)
    float vel_kd = -0.001f;  // Velocity derivative gain [s] (negative)
    
    // Control loop parameters
    float dt = 1.0f / PWM_TIMER_FREQ_HZ; // Control timestep [s] (e.g., 0.00005 for 20kHz)
    
    // Maximum velocity limit (5 rev/s in Q16.16 format)
    const int32_t counts_per_rev = 16384; // Encoder resolution
    int32_t max_vel_q16 = (int32_t)(5.0f * counts_per_rev * 65536.0f); // 5 rev/s
    
    // // Initialize position controller
    // TUNE_HEART.position_controller = PositionController(
    //     pos_kp, pos_ki, pos_kd,
    //     vel_kp, vel_ki, vel_kd,
    //     dt, max_vel_q16
    // );
    
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
            // Generate forward motion pattern
            generate_forward_pattern();
            motion_state = PatternState::Running;
            last_pattern_time = millis();
            break;
            
        case PatternState::GeneratingReverse:
            // Generate reverse motion pattern
            generate_reverse_pattern();
            motion_state = PatternState::Running;
            last_pattern_time = millis();
            break;
            
        case PatternState::Running:
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
            break;
    }
}
