# TunePulse vs CLN-SimpleFOC Parity & Performance TODO

This document tracks the feature parity, architectural differences, and remaining work needed to bring the `TunePulse` custom firmware up to par (and beyond) with the `CLN-SimpleFOC` reference implementation. Both target the same hardware (STM32G4 based boards like CLN17), but take different approaches to motor control.

## 1. What TunePulse does better (Current Advantages)

*   **Custom Highly Optimized Motor Control Loop:**
    *   Instead of relying on the generalized `SimpleFOC` library, TunePulse implements a bespoke, deeply integrated control loop (`TunePulseHeart`).
    *   It heavily utilizes STM32G4 hardware features directly:
        *   **CORDIC hardware accelerator:** Fast trigonometry calculations for Clarke/Park transforms without software overhead.
        *   **Advanced Analog/Comparator Protection:** Hardware-level overcurrent and overvoltage protection tied directly to the DAC and internal comparators, shutting down the PWM faster than software could react.
*   **Built-in S-Curve Motion Planning:**
    *   Includes a robust S-Curve ramp generator (`SCurvePlanner`) directly in the firmware, allowing for extremely smooth point-to-point movements with acceleration and jerk limits.
*   **Higher Potential Loop Rates:**
    *   Due to the hardware acceleration and avoidance of library abstractions, the inner FOC loop can potentially run at significantly higher frequencies and lower latencies than `SimpleFOC`.
*   **Clean Architecture:**
    *   Separation of math, feedback, motor driver, and target-specific BSP setups, making it very scalable for different hardware targets.

## 2. What TunePulse is missing (Compared to CLN-SimpleFOC)

*   **Cascaded Dual-Loop Position Control (In Progress):**
    *   `CLN-SimpleFOC` uses an inner loop for motor velocity/commutation and an outer loop (PID) reading from an external linear encoder to correct for backlash and mechanical slop. TunePulse requires a tight integration of this cascaded structure.
*   **External Linear Encoder Support (MT6835) (In Progress):**
    *   `CLN-SimpleFOC` includes a driver and SPI integration for the MT6835 absolute magnetic encoder used for the outer position loop.
*   **CAN-FD Communication (In Progress):**
    *   `CLN-SimpleFOC` uses `SimpleCANio` to receive absolute linear position targets over CAN-FD.
*   **Calibration Data Persistence (EEPROM/Flash):**
    *   `CLN-SimpleFOC` saves motor and encoder calibration data (like zero electrical angle and LUTs) to EEPROM, skipping the calibration sequence on subsequent boots. TunePulse requires a calibration routine on every startup.
*   **Stepper Motor Support (Specifically 4-PWM):**
    *   While TunePulse is designed to be multi-motor, `CLN-SimpleFOC` specifically implements `StepperDriver4PWMExtended` and a dedicated Stepper FOC pipeline. TunePulse currently focuses heavily on 3-phase BLDC/PMSM structures in its existing driver codebase.

## 3. Implementation TODO List

### High Priority (Reaching Baseline Parity)
- [x] **Port MT6835 Encoder Driver:**
    - [x] Bring over `MT6835_encoder.h/.cpp`.
    - [x] Setup SPI interface on CTRL pins (`PA8` CS, `PB5` MOSI, `PB4` MISO, `PB3` SCK).
- [x] **Implement CAN-FD Stack:**
    - [x] Enable `HAL_FDCAN_MODULE_ENABLED` in build.
    - [x] Implement robust CAN-FD receive logic for continuous trajectory streaming via `SimpleCANio`.
- [x] **Implement Cascaded Dual-Loop Position Control (Tight Coupling):**
    - [x] Move the outer PID loop directly into the `TunePulseHeart::tick()` method.
    - [x] Read the MT6835 actual pos synchronously inside the high-frequency tick.
    - [x] Pipe outer PID output (target velocity) into the existing TunePulse inner velocity controller.
- [x] **Refactor Application Loop (`main.cpp`):**
    - [x] Support "Direct CAN Mode" (passthrough of target position from CAN).
    - [x] Support "Interpolated S-Curve Mode" (S-curve planner smoothing out coarse CAN targets or moving autonomously).

### Medium Priority (Reliability & Usability)
- [ ] **Persistent Calibration:**
    - [ ] Implement Flash memory emulation for storing motor calibration data (zero angle, direction).
    - [ ] Implement skip-calibration logic on boot if valid data exists.
- [ ] **Refine Stepper Support:**
    - [ ] Verify or adapt the TunePulse `driver_pwm` and `foc_vector_synthesis` to properly handle the specific Stepper motor topology used on the CLN17 hardware (if not fully implemented).

### Low Priority (Enhancements)
- [ ] **Telemetry over CAN:** Return current state, errors, and actual position back over the CAN bus.
- [ ] **Advanced Error Recovery:** Handle CAN timeouts or encoder signal loss gracefully.
