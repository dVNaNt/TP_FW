# Motor Electric Test Algorithm - Technical Documentation

## Overview

The `motor_electric_test.h` module implements an automated motor parameter identification system that measures electrical characteristics (inductance and resistance) of connected motors using HFI (High Frequency Injection) and DC current injection. The algorithm identifies motor type (BLDC, DC, Stepper), phase wiring pattern, and winding parameters through systematic testing of all three phase combinations.

## Key Features

- **DQ Coordinate System**: All calculations performed in D-Q (Direct-Quadrature) coordinates
- **HFI-Based Inductance Measurement**: Uses peak-to-peak current method for accurate inductance detection
- **Adaptive PI Controller**: Kp calculated from measured inductance for optimal current control
- **Polarity-Aware Calibration**: Positive polarity for CBA(0), negative for ACB(1) and ABC(2)
- **Default BLDC Pattern**: CBA is the default commutation pattern for BLDC motors

## Algorithm Workflow

### Phase Calibration Sequence

The test cycles through three phase patterns in order: **CBA(0) -> ACB(1) -> ABC(2)**

For each pattern, the algorithm performs:
1. **HFI Inductance Measurement** - Inject sine wave, measure peak-to-peak current
2. **PI Controller Setup** - Calculate Kp from measured inductance
3. **Resistance Measurement** - Two-point DC measurement with PI current control

### Polarity Rules

| Pattern | Index | Polarity | Description |
|---------|-------|----------|-------------|
| CBA     | 0     | Positive | Default BLDC pattern |
| ACB     | 1     | Negative | Alternative pattern |
| ABC     | 2     | Negative | Alternative pattern |

## Detailed Algorithm

### Phase 1: Warmup (100ms)

- All phases in Hi-Z state
- Allow system to stabilize before measurements

### Phase 2: HFI Inductance Measurement

**Purpose**: Measure phase inductance using high-frequency injection

**Process**:
1. Inject 1kHz sine wave on D-axis (starting at 100mV amplitude)
2. Track minimum and maximum current during injection
3. Calculate peak-to-peak current: I_pp = I_max - I_min
4. Calculate inductance: L = V / (pi * f * I_pp)

**Amplitude Adjustment**:
- If current too high (>target): reduce amplitude proportionally
- If current too low (<target/2): double amplitude (max 2V)
- Target: 25% of motor current limit
- Maximum 10 iterations for amplitude tuning

**Inductance Formula**:
L_uH = V_mV * 318 / I_pp_mA
where 318 = 1000000 / (pi * 1000Hz)

### Phase 3: PI Controller Configuration

**Purpose**: Calculate optimal Kp from measured inductance

**Formula**:
Kp = L * 2pi * f_bandwidth
For f_bw = 100Hz: Kp = L_uH * 628 / 1000

**Constraints**:
- Minimum Kp: 10
- Maximum Kp: 1000
- Ki: Fixed at 5 (low for stability)

### Phase 4: Resistance Measurement

**Two-Point Method**:

**First Point**:
- PI controller ramps current to target (75% of limit)
- Wait for stability (20ms within tolerance)
- Settle for 50ms
- Average 256 samples of voltage and current

**Second Point**:
- Apply half voltage from first point
- Wait for stability
- Settle for 50ms
- Average 256 samples

**Resistance Calculation**:
dV = |V1 - V2|  (voltage difference in mV)
dI = |I1 - I2|  (current difference in mA)
R = (dV * 1000) / dI  (resistance in mOhm)

### Phase 5: Pause (50ms)

- Hi-Z state between patterns
- Allow magnetic field to dissipate

### Phase 6: Motor Type Detection

After all three patterns tested, detect_type() analyzes results:

**BLDC Motor**:
- All 3 measurements valid
- Deviation between min/max <= 25%
- Default pattern: CBA

**Stepper Motor**:
- All 3 measurements valid
- One resistance approx sum of other two (+/-20%)
- Pattern: Index of maximum resistance

**DC Motor**:
- Only 1 valid measurement
- Pattern: Index of valid measurement

## Timing Summary

**Single Pattern Test:**
- Warmup: 100ms
- HFI Measurement: ~50ms per iteration (1-10 iterations)
- Resistance Ramp: ~50-200ms (variable)
- Settle: 50ms x 2 points
- Measure: 14.2ms x 2 points
- Pause: 50ms

**Total per pattern**: ~250-500ms

**Complete Test** (3 patterns): ~750-1500ms

## DQ Coordinate System

All measurements use DQ (Direct-Quadrature) coordinates:

- **D-axis (Direct/Flux)**: Aligned with rotor flux, used for:
  - HFI injection
  - Current measurement
  - Voltage application

- **Q-axis (Quadrature/Torque)**: Perpendicular to D-axis
  - Set to zero during DC testing
  - Used for torque production in normal operation

## Safety Features

### Overcurrent Protection
- Threshold: 50% of full-scale current
- Checked every PWM cycle
- Immediate Hi-Z on detection

### Amplitude Limiting
- HFI: Maximum 2V amplitude
- DC: Maximum 50% of supply voltage

### Open Circuit Detection
- If dI < 10mA: mark as open circuit
- Resistance set to INT32_MAX

## Usage Example

```cpp
// Initialize
DCPhaseCheck motor_test(feedback, motor, motor_selector);

// Start test
motor_test.start();

// Call every PWM cycle (18kHz)
void PWM_IRQ_Handler() {
    motor_test.tick();
    
    if (motor_test.is_complete()) {
        motor.print(SerialUSB);
    }
}
```
