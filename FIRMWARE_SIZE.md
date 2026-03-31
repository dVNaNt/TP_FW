# Firmware Size Changelog

## 2026-03-29 - Position Controller with S-Curve Ramp Generator

**Changes:**
- Replaced manual segment generation with SCurvePlanner from ramp_gen.h
- Simplified motion pattern generation using 7-phase S-curve profile
- Motion parameters: 15 revolutions travel, 5 rev/s² acceleration, 5 rev/s target velocity, 0 jerk (trapezoidal)
- Automatic segment stitching and position tracking handled by ramp generator
- Cleaner code with single function call for complete motion profile generation

**Rationale:**
- S-curve ramp generator provides mathematically correct trajectory planning
- Eliminates manual calculation of segment parameters and timing
- Ensures smooth transitions between acceleration, constant velocity, and deceleration phases
- Reduces code complexity and potential for calculation errors

**Size Impact:**
- RAM: 6956 bytes (21.2% of 32768 bytes) - **+12 bytes** (from 6944 bytes)
- Flash: 54328 bytes (41.4% of 131072 bytes) - **-216 bytes** (from 54544 bytes)

**Notes:**
- Ramp generator automatically handles position accumulation and velocity continuity
- Pattern alternates between forward (+15 rev) and reverse (-15 rev) motion
- Zero jerk configuration produces trapezoidal velocity profile

## 2026-03-29 - Position Controller and Motion Planning Integration (v3 - Additive Segments)

**Changes:**
- Implemented additive position tracking between segments (absolute positioning)
- Added velocity continuity between segments (start velocity = previous end velocity)
- Changed from RELATIVE to ABSOLUTE positioning mode for proper segment stitching
- Added motion state tracking: `current_position_q16` (i32 Q16.16) and `current_velocity_rps` (f32)
- All motion parameters now use correct units: time (u32 μs), position (i32 Q16.16 rev), velocity/acceleration/jerk (f32 rev/s)
- Proper calculation of segment distances based on initial velocity (v0*t + 0.5*a*t²)

**Rationale:**
- Additive positioning ensures smooth continuous motion without position jumps
- Velocity continuity eliminates acceleration spikes at segment boundaries
- Absolute positioning mode provides better trajectory tracking
- Correct physics calculations for non-zero initial velocities

**Size Impact:**
- RAM: 6944 bytes (21.2% of 32768 bytes) - **+8 bytes** (from 6936 bytes, motion state tracking)
- Flash: 54544 bytes (41.6% of 131072 bytes) - **-360 bytes** (from 54904 bytes, optimized calculations)

**Notes:**
- Position accumulates across all segments for continuous motion
- Each segment starts with the velocity where the previous segment ended
- Motion pattern now has smooth transitions without velocity discontinuities

## 2026-03-29 - Position Controller and Motion Planning Integration (v2 - Continuous Loop)

**Changes:**
- Changed PID controller gains to negative values for proper error correction
- Implemented continuous motion pattern generation with state machine
- Motion pattern now repeats infinitely, alternating between forward and reverse directions
- Removed single-shot motion pattern generation in favor of dynamic loop-based approach
- Fixed enum name conflict (MotionState → PatternState)

**Rationale:**
- Negative PID gains ensure proper error correction in cascaded control loops
- Continuous motion pattern enables long-term testing and demonstration
- State machine approach provides clean separation of calibration, generation, and execution phases

**Size Impact:**
- RAM: 6936 bytes (21.2% of 32768 bytes) - **+12 bytes** (from 6924 bytes)
- Flash: 54904 bytes (41.9% of 131072 bytes) - **-944 bytes** (from 55848 bytes)

**Notes:**
- Flash size decreased due to removal of large single-shot pattern generation function
- RAM increase is minimal (state machine variables)
- Pattern automatically starts after motor calibration completes

## 2026-03-29 - Position Controller and Motion Planning Integration (v1)

**Changes:**
- Added cascaded position-velocity controller (`position_controller.h`)
- Integrated motion planner and motion executor into `TunePulseHeart`
- Added motion pattern generation in `main.cpp` with acceleration/constant velocity/deceleration phases
- Modified `TunePulseHeart::tick()` to support position control mode using motion trajectory tracking
- Added `MotionState.trq` field for torque/current tracking
- Fixed `MotionExecutor::load_next_segment()` to use correct BufferFIFO API

**Rationale:**
- Enables precise position control with smooth velocity profiles
- Provides trajectory planning for complex motion patterns
- Decouples position and velocity control loops for independent tuning
- Supports both manual velocity control and automated position control modes

**Size Impact:**
- RAM: 6924 bytes (21.1% of 32768 bytes) - **+596 bytes** (from 6328 bytes)
- Flash: 55848 bytes (42.6% of 131072 bytes) - **-2868 bytes** (from 58716 bytes)

**Notes:**
- Flash size decreased due to compiler optimizations with new control structure
- RAM increase is due to motion executor buffer (16 segments) and position controller state

## 2026-02-04 - HFI Generator Instance Relocation

**Changes:**
- Moved HFI frequency generator instance from `TunePulseHeart` (lib.h) to `DriverPWM` (driver_pwm.h)
- Added public methods `configure_hfi_generator()` and `tick_hfi_generator()` to DriverPWM
- Updated `TunePulseHeart::enable_hfi()` to use driver's HFI generator instead of local instance
- Updated `TunePulseHeart::tick()` to call `driver_.tick_hfi_generator()` instead of local generator

**Rationale:**
- HFI generator is already used in DriverPWM for inductance calibration
- Eliminates duplicate instance in TunePulseHeart
- Centralizes HFI signal generation in motor driver layer
- Reduces memory footprint by removing redundant FrequencyGenerator instance

**Size Impact:**
- RAM: 6328 bytes (19.3% of 32768 bytes) - **No change** (0 bytes)
- Flash: 58716 bytes (44.8% of 131072 bytes) - **No change** (0 bytes)

---

## 2026-03-29: Added Motion Segment Logging

**Changes:**
- Added serial logging when motion executor switches to new segment
- Added debug logging in SCurvePlanner to track segments being added to buffer
- Added validation to ensure at least one segment is generated before returning success
- Moved segment logging to immediately after buffer read for accurate data display

**Rationale:**
- Provides visibility into motion profile execution for debugging
- Helps identify issues with segment generation and buffer management
- Enables real-time monitoring of motion control state transitions

**Size Impact:**
- RAM: 6956 bytes (21.2% of 32768 bytes) - **+628 bytes** from previous 6328 bytes
- Flash: 55240 bytes (42.1% of 131072 bytes) - **-3476 bytes** from previous 58716 bytes
