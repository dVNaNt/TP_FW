---
inclusion: always
---

# TunePulse Firmware

## Project Overview

TunePulse is an advanced motor control firmware for STM32G431CB microcontrollers, designed for precise stepper and BLDC motor control applications. The firmware provides:

- **Multi-Motor Support**: Stepper motors (2-phase, 3-phase) and BLDC motors with field-oriented control (FOC)
- **Advanced Control Modes**: Position control, velocity control, torque control with PID feedback loops
- **High-Frequency PWM**: Optimized PWM generation for smooth motor operation
- **Sensor Integration**: Support for encoders, hall sensors, and sensorless control algorithms
- **Real-Time Feedback**: Current sensing, voltage monitoring, and position tracking
- **Calibration System**: Automatic motor parameter detection and angle calibration
- **Communication**: USB CDC serial interface for configuration and monitoring
- **Safety Features**: Overcurrent protection, thermal monitoring, and error handling

The firmware is built using a modular, header-only library architecture for maximum performance and minimal memory footprint on resource-constrained embedded systems.


## Technology Stack

## Build System

- **Platform**: PlatformIO
- **Framework**: Arduino (STM32duino)
- **Microcontroller**: STM32G431CB
- **Platform**: ststm32

## Languages

- **Primary**: C++ (header-only libraries)

## Key Libraries

- STM32 HAL (limited modules enabled)

## Build Configuration

### Available Environments

- `cln17_v3_release` - CLN17 V3 release (default)
- `radix_release` - Radix board

### Build Flags

- Debug mode: `-O0` optimization
- USB CDC enabled for SerialUSB communication
- Selective HAL modules: `HAL_TIM_MODULE_ONLY`, `HAL_ADC_MODULE_ONLY`
- Assembly output: `-save-temps -fverbose-asm`

## Common Commands

### Build
```bash
python -m platformio run -e cln17_v3_release
```

### Upload
```bash
python -m platformio run -e cln17_v3_release -t upload
```

### Clean Build
```bash
python -m platformio run -e cln17_v3_release -t clean
```

### Check Size
```bash
python -m platformio run -e cln17_v3_release
```
And see output log for details. Example
RAM:   [==        ]  x% (used x bytes from 32768 bytes)
Flash: [====      ]  x% (used x bytes from 131072 bytes)

### Debug
```bash
python -m platformio debug -e cln17_v3_release
```

## Upload Protocols

- **ST-Link**: Default, requires ST-Link programmer
- **LOGGING**: SerialUSB everywhere if serial/USB requested
  - For any SerialUSB log use macross: TP_LOG(msg), TP_LOG_VAL(msg, val), TP_LOG_ARRAY(msg, arr), TP_LOG_VALS(msg, ...)


## Development Guidelines

### Copyright and License Headers

Every source file MUST include the following copyright and license header at the top:

```cpp
/**
 * @file filename.h
 * @brief Brief description of the file's purpose
 * 
 * Detailed description of functionality, algorithms, and usage.
 * 
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */
```

### File Documentation

Every file MUST begin with:
1. **Copyright and license notice** (as shown above)
2. **Detailed functional description** explaining:
   - What the code does
   - Why it exists (purpose and use cases)
   - How it fits into the overall system
   - Key algorithms or techniques used
   - Dependencies and relationships with other modules

### Code Documentation Standards

#### Language
- **All code comments MUST be in English**
- **Translate any existing non-English comments to English** when modifying code
- Use clear, technical English appropriate for embedded systems development

#### Comment Density
- **Add English comments to every line** of code for clarity
- Explain the purpose and logic of each operation
- For simple operations, brief inline comments are sufficient
- For complex logic, add detailed block comments with logic description

#### Algorithm Documentation
- **Complex algorithms require detailed descriptions** including:
  - High-level overview of the algorithm's purpose
  - Step-by-step explanation of the logic flow
  - Mathematical formulas or equations (if applicable)
  - Time and space complexity considerations
  - Edge cases and limitations
  - References to papers, standards, or documentation sources with links where possible

Example:
```cpp
/**
 * @brief S-curve motion profile generator
 * 
 * Implements a 7-segment S-curve trajectory with continuous acceleration.
 * The profile consists of:
 * 1. Jerk-limited acceleration ramp-up
 * 2. Constant acceleration phase
 * 3. Jerk-limited acceleration ramp-down
 * 4. Constant velocity phase
 * 5. Jerk-limited deceleration ramp-up
 * 6. Constant deceleration phase
 * 7. Jerk-limited deceleration ramp-down
 * 
 * Algorithm complexity: O(1) per step
 * 
 * Reference: "Trajectory Planning for Automatic Machines and Robots"
 */
```

### Documentation Maintenance

- **When modifying code, update documentation accordingly**
- **Maintain the same level of detail** as the original documentation
  - If original had detailed descriptions, keep them detailed after changes
  - If original had algorithm explanations, update them to reflect new logic
- **Update file headers** if functionality changes significantly
- **Keep copyright year current** when making substantial modifications

### Code Structure
- **Always use C++ structures** with private and public fields
- Prefer encapsulation and proper access control
- Use `struct` for data-only types, `class` for types with invariants
- Mark member functions as `const` when they don't modify state
- Use `constexpr` for compile-time constants and functions
- DO NOT create getters and setters if not asked directly
- Group methods by functional meaning (e.g. getters/setters for same variable should be close)

### Code Comments Best Practices
- **Inline comments**: Explain what the line does
  ```cpp
  uint32_t pwm_value = (duty * ARR) >> 16; // Scale duty cycle to timer period
  ```
- **Block comments**: Explain why and how
  ```cpp
  // Calculate phase voltages using inverse Clarke transform
  // This converts alpha-beta frame back to three-phase ABC frame
  // for PWM generation on the motor driver
  ```
- **Function documentation**: Use Doxygen-style comments
  ```cpp
  /**
   * @brief Converts electrical angle to PWM pulse width
   * @param angle Electrical angle in fixed-point format (0-65535 = 0-360°)
   * @param amplitude Current amplitude in milliamps
   * @return PWM duty cycle value (0-4095 for 12-bit resolution)
   */
  ```

### Code Optimization
- **Write maximally optimized code for runtime performance**
- Minimize memory allocations and CPU cycles
- Consider embedded constraints (limited Flash/RAM)
- Prefer compile-time optimizations where possible
- Use fixed-point arithmetic instead of floating-point when appropriate
- Profile critical paths and optimize hot loops

### Build Verification
- **Always verify compilation** after code changes
- Run the report script from the command line, passing all 6 arguments wrapped in quotes. Use a semicolon (;) inside the text to create multiple bullet points.
python changelog/changelogger.py "Summary" "Changes" "Reasons" "Notes" <FlashSize> <RAMSize>
Example: python changelog/changelogger.py"Interpolator optimization" "Switch to Q31 fixed-point math; Optimize BEMF calculation" "Reduce cycle count for high-performance control" "SMMUL and SMMLA instructions utilized" 45120 12048
- Check for warnings and treat them as errors
- Verify that optimizations don't break functionality

### Documentation Files
- **Never create README files** unless explicitly requested by the user
- Keep documentation in code headers and this tech.md file
