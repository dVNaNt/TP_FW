1. Motion Control Modes & Strategies
* Open-Loop Stepper Control: Standard stepping without encoder feedback.
    * Feature: Current reduction at standstill (Idle Current).
    * Feature: Resonance compensation / Anti-resonance algorithms.
* Closed-Loop Stepper Control: Position maintenance using encoder feedback.
    * Feature: Automatic switch to open-loop if encoder fails (optional fallback).
* Field Oriented Control (FOC): Vector control for silent operation and torque efficiency.
    * Feature: $D$-axis current injection (Field Weakening) for higher speeds.
* Velocity Modes:
    * Profile Velocity: Target speed with acceleration/deceleration ramps.
    * Cyclic Synchronous Velocity: Follow a velocity stream from a master controller (no internal ramps).
    * Slow Speed Mode: Hybrid open/closed loop for smooth low-speed operation (prevents hunting).
* Position Modes:
    * Profile Position: Move to target with defined accel/decel/speed profile.
    * Interpolated Position (Cyclic Sync): Follow a stream of position setpoints from a master (CNC/Robotics usage).
    * Pulse/Direction Interface: Standard step/dir input handling.
    * Clock-Direction Mode: Alternate step interface.
* Torque Modes:
    * Profile Torque: Maintain constant torque with defined slope.
    * Cyclic Synchronous Torque: Follow a torque stream from a master.
2. Motion Commands (Operational)
* Movement Primitives
    * Move Absolute (Steps/Pulses): Go to raw encoder count $N$.
    * Move Relative (Steps/Pulses): Move $\pm N$ counts from current.
    * Move Absolute (User Units): Go to coordinate $X$ (mm/deg) based on gearbox/ratio settings.
    * Move Relative (User Units): Move $\pm X$ units.
    * Run at Speed: Spin at $RPM$ indefinitely.
    * Stop / Halt: Decelerate to stop using defined ramp.
    * Emergency Stop: Immediate hard stop (short coils or max braking torque).
    * Change Setpoint on-the-fly: Update target position/speed while moving.
* Homing & Zeroing
    * Set Current as Zero: Overwrite current register to 0.
    * Home to Index: Move until encoder Z-pulse is found.
    * Home to Switch (Limit/Home): Move direction $D$ until input triggers, then back off.
    * Home to Stall (Sensorless): Move until Current $>$ Threshold for Time $T$.
    * Single-Turn Zeroing: Zero within 0-360° of the current rotation.
    * Coordinate Zeroing: Zero the internal coordinate system without moving.
    * Torque Homing: Move until a specific torque is reached (clamping).
* Multi-Axis Synchronization
    * Broadcast Move: Send one command ID that all motors listen to.
    * Group Move: Send command to specific Group ID (e.g., "Gantry Y-Axis").
    * Sync Mark/Trigger: Arm motion to start only upon receiving a specific "Sync" signal or IO pin trigger.
3. Control Loop Tuning & Parameters
* PID Gains (Separate banks for Position, Velocity, Current)
    * Kp (Proportional): Stiffness.
    * Ki (Integral): Steady-state error correction.
    * Kd (Derivative): Damping.
    * Kv (Velocity Feed-forward): Prediction based on target speed.
    * Ka (Acceleration Feed-forward): Prediction based on target acceleration.
* Current Vector Control
    * Q-Axis Current (Torque): Tuning for torque generation.
    * D-Axis Current (Flux): Tuning for field weakening/magnetizing.
* Filters
    * Input Smoothing: Filter incoming step/dir pulses to reduce jitter.
    * Feedback Filter: Low-pass filter on velocity calculation to reduce noise.
4. Safety & Protection Mechanisms
	- Power limiting
	- Current limiting
* Voltage Safety
    * Over-Voltage Threshold: Cut MOSFETs if V > Limit (regenerative braking protection).
    * Under-Voltage Threshold: Disable if V < Limit (brownout protection).
    * Double level - soft warn limit and hard limit
* Thermal Safety
    * $I^2t$ Protection: Accumulate "thermal energy" based on current over time; derate or fault if exceeded.
    * Over-Temperature: Soft warning limit and Hard shutdown limit.
* Mechanical Safety
    * Following Error (Window & Time): Fault if |Target - Actual| > Limit for Time $T$.
    * Stall Detection: Fault if motor is stuck (Back-EMF or Encoder mismatch).
    * Slippage Error: Detection of lost steps in open-loop emulation.
    * Over-Speed: Fault if Velocity > Max Velocity.
    * Soft Limits: Virtual end-stops (Min/Max coordinate boundaries).
* Communication Safety
    * Heartbeat / Watchdog: Emergency stop if host communication stops for $T$ ms.
    * Interlock: Hardware enable pin must be active to move.
5. I/O & Hardware Configuration
- IO connector mode - SPI/I2C/UART/ABZ/IO

* Input Configuration
    * Remapping: Map physical PIN_1 to function "Limit+" or "Enable".
    * Logic Inversion: Set Active High vs. Active Low.
    * Debounce Time: Filter signal noise on switch inputs.
    * Analog Input Scaling: Map 0-3.3V input to Target Velocity or Torque (Joystick mode).
* Output Configuration
    * Brake Output: Dedicated pin for mechanical brake control.
        * Command: Brake Engage Delay (Wait $T$ ms after stop before locking).
        * Command: Brake Release Delay (Wait $T$ ms after energizing before moving).
        * Command: Brake PWM (Reduce holding voltage to save power).
    * Signal Output: Map internal events (Error, Position Reached, Moving) to physical output pins.
    * Pulse Echo: Output encoder pulses for external controller tracking.
6. Status & Telemetry Registers (Read-Back)
* Real-Time Data
    * Actual Position: 32-bit or 64-bit counter.
    * Actual Velocity: Current RPM/RPS.
    * Actual Torque/Current: Measured load.
    * Position Error: Deviation from target.
    * Bus Voltage: $V_{in}$.
    * Temperature: Driver and/or Motor temp.
* Status Flags (Bitmask)
    * Motion Status: Stationary / Accelerating / Decelerating / Constant Speed.
    * Homing Status: Homed / Homing Active / Not Homed.
    * Error Flags: Stall / OverHeat / OverVolt / FollowingError / Estop.
    * I/O Status: State of all digital inputs (Limit switches, Enable pin).
* Cumulative Data
    * Uptime: Seconds since boot.
    * Total Travel: Odometer for maintenance.
    * Cycle Count: Number of movements executed.
7. Setup & System Commands
- Motor Validation check

* Motor Identification
    * Auto-Calibration Routine: Measure Resistance, Inductance, and Encoder Offset automatically.
    * Pole Pair Count: Set number of magnetic poles.
    * Encoder Resolution: Set ticks per revolution.
    * Phase Inversion: Swap A/B phases via software.
    * Direction Inversion: Invert logical positive direction.
* Communication Setup
    * Set Node ID: CAN/RS485 Address.
    * Set Baud Rate: Bus speed.
    * Termination: Enable software-controllable termination resistor (if hardware supports).
* Persistence
    * Save to NVM: Burn active settings to EEPROM/Flash.
    * Restore Defaults: Factory reset.
    * Reboot: Soft reset device.
8. Advanced Features
* Scripting / Macros:
    * NanoJ / User Programs: Ability to store simple motion scripts (e.g., "Move back and forth forever") to run without a master.
* Ballast / Braking Resistor Control:
    * Active control of an external dump resistor to dissipate regenerative energy.
* Anti-Cogging / Ripple Compensation:
    * Map encoder position to torque modifications to smooth out motor "notches".
