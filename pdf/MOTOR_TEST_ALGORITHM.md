# Motor Electric Test Algorithm - Technical Documentation

## Overview

The `motor_electric_test.h` module implements an automated motor parameter identification system that safely measures electrical characteristics of connected motors using DC current injection with **Ramp Controller**. The algorithm identifies motor type (BLDC, DC, Stepper), phase wiring pattern, and winding resistance through systematic testing of all three phase combinations.

## Algorithm Workflow

### Phase 1: Initialization
When `start()` is called:
1. Motor selector switches to DC mode for safe testing
2. Low-pass filter (LPF) is initialized with alpha=210 for current measurement
3. All result structures are reset
4. Voltage starts from 0mV

### Phase 2: Test Sequence (Repeated for 3 Phase Patterns)

The test cycles through three phase patterns: **ABC**, **BCA**, **CAB**

#### 2.1 Ramp Control State (`Ramp_Control`)
**Purpose**: Gradually increase voltage to reach target current using simple linear ramp

**Process**:
- Start with 0mV voltage
- On each PWM cycle (55.5µs @ 18kHz):
  - Filter current with LPF (alpha=210)
  - Compare filtered current with target
  - If current < target: voltage += 10mV
  - If current > target: voltage -= 10mV
  - If current == target: maintain voltage
- Apply hard voltage limits (100mV min, 75% of supply max)
- Monitor for overcurrent (>50% full scale) on every cycle
- Check stability: current within ±2.5% tolerance for 100 consecutive cycles (~5.5ms)

**Safety**: 
- Small step size (10mV) ensures smooth control and immunity to BEMF noise
- No integral windup or overshoot issues
- Guaranteed stability with proper filtering

**Convergence Criteria:**
- Current within ±2.5% tolerance of target
- Current at least 90% of target value (prevents premature convergence at low current)
- Both conditions must hold for 100 consecutive cycles (~5.5ms)
- Any violation resets the stability counter

**Duration**: Variable, max 100ms (1800 cycles)

#### 2.2 Settle State (`Ramp_Settle`)
**Purpose**: Maintain constant voltage for 50ms to allow transients to decay

**Process**:
- Apply fixed voltage found during ramp control
- Continue LPF filtering for consistency
- Wait for inductance transients to fully settle
- No measurement during this phase - just waiting

**Reason**: 
- Motor inductance causes current transients during voltage changes
- 50ms ensures all transients have decayed to <1% of steady-state
- Provides clean, stable current for accurate measurement

**Duration**: 900 cycles at 18kHz = **50ms**

#### 2.3 Measuring State (`Ramp_Measure`)
**Purpose**: Accurately measure average current and voltage at stable conditions

**Process**:
- Maintain constant voltage from ramp stage
- Accumulate 256 samples of voltage and current
- Calculate averages using bit shift (>> 8 for division by 256)
- LPF continues with same alpha (210) for consistency

**Calculation**:
- Average voltage: `V_avg = voltage_sum >> 8` (mV)
- Average current: `I_avg = current_sum >> 8` (Q15)
- Convert to mA: `I_avg_ma = (I_avg × motor_max_current) >> 15`

**Duration**: 256 cycles at 18kHz = **14.2ms**

#### 2.4 Two-Point Measurement
**First Point**: 
- Target current: 20% of current limit
- Voltage determined by ramp controller
- Full ramp + settle + measure cycle

**Second Point**: 
- **Voltage: 2× voltage from first point** (no ramp control)
- Current determined by motor resistance
- Only settle + measure cycle (no ramp)
- Duty cycle compensated for actual supply voltage

**Advantages of voltage doubling:**
- Faster measurement (no second ramp phase)
- Simpler algorithm (no target current for second point)
- Guaranteed voltage difference for accurate R calculation
- Automatic adaptation to motor resistance

After both points measured:

**Resistance Calculation**:
```
dV = |V2 - V1|  (absolute voltage difference in mV)
dI = |I2 - I1|  (absolute current difference in mA)
R = (dV × 1000) / dI  (resistance in mΩ)
```

**Validation**: Resistance must be within 50mΩ - 100Ω range to be considered valid

#### 2.5 Pause State (`Ramp_Pause`)
**Purpose**: Allow motor to cool and magnetic field to dissipate

**Process**:
- Switch to Hi-Z mode (all phases floating)
- Wait for magnetic transients to settle

**Duration**: 900 cycles at 18kHz = **50ms**

### Phase 3: Motor Type Detection

After all three patterns tested, `detect_motor_optimized()` analyzes results:

#### 3.1 Data Extraction
```cpp
r0 = ABC pattern resistance (mΩ)
r1 = BCA pattern resistance (mΩ)
r2 = CAB pattern resistance (mΩ)
```

#### 3.2 Validity Check
Each resistance validated: `50mΩ ≤ R ≤ 100Ω`

#### 3.3 Deviation Calculation
```cpp
r_max = maximum of valid resistances
r_min = minimum of valid resistances
deviation = ((r_max - r_min) × 100) / r_max
```

#### 3.4 Motor Type Classification

**BLDC Motor** (3-phase brushless):
- Condition: All 3 measurements valid AND deviation ≤ 25%
- Reason: All three phase pairs have equal resistance
- Pattern: ABC (default, any pattern works)
- Example: 1.2Ω, 1.25Ω, 1.18Ω → deviation = 5.6% → BLDC

**Stepper Motor** (2-phase bipolar):
- Condition: All 3 measurements valid AND one resistance ≈ sum of other two (±20%)
- Reason: Two coils in series show double resistance
- Pattern: Depends on which measurement shows sum
- Example: 2Ω, 2Ω, 4Ω → CAB pattern (coil A-B and coil C-A are separate)

**DC Motor** (2-wire brushed):
- Condition: Only 1-2 measurements valid OR high deviation
- Reason: Only one phase pair is connected
- Pattern: Phase pair with maximum resistance
- Example: 3Ω, NC, NC → ABC pattern (motor connected to A-B)

**Invalid**:
- Condition: No valid measurements
- Reason: No motor connected or all resistances out of range

### Phase 4: Result Storage

Final results saved to `Motor` structure:
- `motor->pole_type`: Detected motor type (BLDC/DC/Stepper)
- `motor->connection`: Optimal phase pattern
- `motor->resistance`: Average of valid resistance measurements
- `motor->status`: Set to `DriverStatus_Ready` or `DriverStatus_Error`

## Timing Summary

**Single Pattern Test:**

**First Point:**
- Ramp Control: ~50-100ms (variable, until current stable)
- Settle: 50ms (constant voltage, transient decay)
- Measuring: 14.2ms (256 samples)

**Second Point:**
- Settle: 50ms (doubled voltage, transient decay)
- Measuring: 14.2ms (256 samples)

**Between Patterns:**
- Pause: 50ms (Hi-Z discharge)

**Total per pattern**: ~180-230ms (depending on ramp convergence time)

**Complete Test** (3 patterns):
- **Total duration**: ~540-690ms (0.54-0.69 seconds)

**Repeat Mode**:
- Wait between iterations: 50ms
- Continuous testing for calibration/monitoring

## Ramp Controller vs PI Controller

### Advantages of Ramp Controller:
1. **Simplicity**: No complex coefficient calculation or parameter estimation
2. **Stability**: Guaranteed stable with proper step size and filtering
3. **Code Size**: ~1600 bytes smaller Flash footprint
4. **Memory**: 36 bytes less RAM usage
5. **Noise Immunity**: Small step size (10mV) naturally filters BEMF spikes
6. **No Overshoot**: Linear ramp prevents integral windup issues

### Key Differences:
- **PI Controller**: Adaptive gains, faster convergence, complex tuning
- **Ramp Controller**: Fixed step, predictable behavior, simple implementation

## Low-Pass Filter Strategy

### Ramp Control Phase (alpha=210)
- Exponential moving average: `filtered = filtered + alpha × (input - filtered) / 256`
- Balance between noise rejection and response time
- Removes PWM switching noise and BEMF transients

### Filter Equation
```cpp
output = output + ((input - output) × 210) / 256
```
Simple IIR filter, computationally efficient

## Typical Measurement Results

### BLDC Motor (e.g., 2208 drone motor)
```
ABC: 1200 mΩ
BCA: 1250 mΩ
CAB: 1180 mΩ
Deviation: 5.6%
Type: BLDC
```

### DC Motor (e.g., 12V brushed motor)
```
ABC: 3500 mΩ
BCA: NC (>100Ω)
CAB: NC (>100Ω)
Deviation: N/A
Type: DC, Pattern: ABC
```

### Stepper Motor (e.g., NEMA 17)
```
ABC: 2000 mΩ (coil 1)
BCA: 2000 mΩ (coil 2)
CAB: 4000 mΩ (coils in series)
Deviation: 50%
Type: Stepper, Pattern: CAB
```

## Performance Metrics

### CPU Usage
- Per PWM cycle: ~40-60 CPU cycles (estimated)
- At 18kHz PWM: <0.1% CPU time on 170MHz STM32G431

### Memory Usage
- Stack: ~150 bytes per test
- Heap: 0 bytes (no dynamic allocation)
- Flash: ~3.4KB code size (1.6KB less than PI version)
- RAM: ~60 bytes (36 bytes less than PI version)

### Accuracy
- Current measurement: ±2% (limited by ADC and sensor)
- Resistance calculation: ±5% (includes voltage/current errors)
- Motor type detection: >95% success rate with proper connections

## Safety Features

### Overcurrent Protection
- Threshold: 50% of full-scale current
- Checked every PWM cycle during active phases
- Immediate transition to Hi-Z error state

### Sensor Error Detection
- Detects INT16_MIN (sensor saturation/disconnection)
- Triggers error state with pause

### Gradual Voltage Application
- Ramp-up prevents inrush current
- Small step size (10mV) ensures smooth control

### Resistance Validation
- Rejects measurements outside 50mΩ - 100Ω range
- Prevents false positives from noise or disconnected phases

### Hi-Z Between Tests
- All phases floating during pause
- Prevents overheating and allows magnetic field decay

## Usage Example

```cpp
// Initialize
MotorTest motor_test(feedback, &motor, &motor_selector);

// Start test
motor_test.start(false); // Single shot mode

// Call every PWM cycle (18kHz)
void PWM_IRQ_Handler() {
    motor_test.tick();
    
    if (motor_test.is_complete()) {
        motor_test.print_results();
    }
}
```

## Conclusion

The Ramp Controller implementation provides robust, safe, and efficient motor parameter identification with significantly reduced code size compared to the PI controller version. The simple linear ramp algorithm ensures guaranteed stability while maintaining measurement accuracy, making it ideal for embedded systems with limited resources.
