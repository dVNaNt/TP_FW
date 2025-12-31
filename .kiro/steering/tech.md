---
inclusion: always
---

# Technology Stack

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

- `radix_release` - Radix board

### Build Flags

- Debug mode: `-O0` optimization
- USB CDC enabled for SerialUSB communication
- Selective HAL modules: `HAL_TIM_MODULE_ONLY`, `HAL_ADC_MODULE_ONLY`
- Assembly output: `-save-temps -fverbose-asm`

## Common Commands

### Build
```bash
python -m platformio run -e radix_release
```

### Upload
```bash
python -m platformio run -e radix_release -t upload
```

### Clean Build
```bash
python -m platformio run -e radix_release -t clean
```

### Check Size
```bash
python -m platformio run -e radix_release
```
And see output log for details. Example
RAM:   [==        ]  x% (used x bytes from 32768 bytes)
Flash: [====      ]  x% (used x bytes from 131072 bytes)

### Debug
```bash
python -m platformio debug -e radix_release
```

## Upload Protocols

- **ST-Link**: Default, requires ST-Link programmer


## Memory Constraints

- Flash: 128 KB (typical usage ~33%)
- RAM: 32 KB (typical usage ~17%)

## Development Guidelines

### Documentation
- **Never create README files** unless explicitly requested by the user

### Build Verification
- **Always verify compilation** after code changes
- **Always report firmware size difference** before and after changes (in bytes compared to previous record)

### Code Structure
- **Always use C++ structures** with private and public fields
- Prefer encapsulation and proper access control

### Code Comments
- **Add English comments to every line** of code for clarity
- Explain the purpose and logic of each operation

### Code Optimization
- **Write maximally optimized code for runtime performance**
- Minimize memory allocations and CPU cycles
- Consider embedded constraints (limited Flash/RAM)
- Prefer compile-time optimizations where possible
