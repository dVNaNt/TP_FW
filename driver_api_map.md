# DriverPWM — External API Map

Everything callable on the PWM motor driver stack (`src/lib/drivers/pwm_driver/`)
from OUTSIDE the module — i.e. from the application layer (`tunepulse.h`,
`lib.h`, `motor_control/`) and the board layer (`src/target/*`).
Friend-only internals (sub-module `tick()` paths, SeqLock readers, phase math)
are intentionally excluded and summarized at the bottom.

## Module layout

```
src/lib/drivers/
├── flash_eeprom.h            EEPROM-emulation middleware (unrelated)
├── tunepulse_interlayer.h    umbrella header for the driver stack
├── generic/                  input.h / output.h / setup.h wrappers
└── pwm_driver/
    ├── types.h               shared typedefs (CompThresholdCallback, kDefaultRecoveryCycles)
    ├── driver_pwm.h          DriverPWM — top-level driver
    ├── current_sense.h       CurrentSense  — 3-ch current sense + OC protection
    ├── voltage.h             SupplyVoltage — bus voltage sense + OV/UV protection
    ├── power.h               Power         — power measurement + OP protection
    ├── temperature.h         Temperature   — MCU temp sense + OT protection
    ├── motor_control.h       MotorControl  — Clarke/SVPWM phase stage
    └── pwm_wrapper.h         PwmWrapper    — SeqLock-protected PWM output stage
```

## Composition and external methods (mermaid)
```mermaid
classDiagram
    class DriverPWM {
        <<top-level driver>>
        +MotorControl motor_ctrl_
        +PwmWrapper pwm
        +CurrentSense current
        +SupplyVoltage supply
        +Temperature temperature
        +Power power
        +tick() void
        +tick_icr() void
        +set_voltage_ab_volts(Vec2D_f32) void
        +set_fault() FaultProxy
        +get_fault() uint8_t
        +is_oc_fault() bool
        +FAULT_* constexpr uint8_t
        +get_current_ab_amps() Vec2D_f32
        +get_supply_volts() float
        +get_temperature_c() float
        +get_raw_power_w() float
        +get_power_w() float
        +get_input_current_a() float
        +get_power_usage() float
        +get_power_coeff() float
        +get_enable() OutputWrapper&
        +get_error() InputWrapper&
    }

    class CurrentSense {
        +overcurrent : OvercurrentProxy
        +current_ab_amps : Vec2D_f32
        +set_currents(uint16_t[3]) void «ISR»
        +calibrate() void
        +is_calibrated() bool
        +is_changed() bool
        +norm() const int16_t*
        +on_calibration_start(cb) void
        +on_calibration_complete(cb) void
    }
    class OvercurrentProxy {
        +set_callback().oc_ia/ib/ic_cb(cb)
        +set_limit_amps().oc_ia/ib/ic(A)
        +set_timeout_cycles(cycles) void
        +is_active() bool
    }

    class SupplyVoltage {
        +overvoltage : VoltageProtectionProxy
        +undervoltage : VoltageProtectionProxy
        +set_voltage_irq(uint32_t) void «ISR»
        +seed_filter(raw_adc) void
        +is_changed() bool
        +get_voltage() float
        +get_max_voltage() float
        +volts2norm(V) int16_t
        +volts2duty(V) int16_t
        +set_callback().ov_lim_cb / uv_lim_cb
        +set_limit_volts().ov_lim_volts / uv_lim_volts
    }
    class VoltageProtectionProxy {
        +set_lim_callback(cb) void
        +set_limit_volts(V) void
        +set_timeout_cycles(cycles) void
        +is_active() bool
    }

    class Power {
        +overpower : OverpowerProxy
        +set_voltage_command(Vec2D_f32) void
        +set_filter_alpha(uint8_t) void
        +set_limit().op_lim_watts(W).op_lim_amps(A)
        +get_raw_power_w() float
        +get_power_w() float
        +get_input_current_a() float
        +get_usage_norm() float
        +get_power_coeff() float
    }
    class OverpowerProxy {
        +set_timeout_cycles(cycles) void
        +is_active() bool
    }

    class Temperature {
        -overheat : TempProtectionProxy «internal»
        +set_temperature_irq(uint32_t) void «ISR»
        +get_temperature_c() float
        +set_temperature_callback(cb) void
        +set_limit_celsius().ot_lim_celsius(C).ot_hyst_celsius(C)
    }

    class MotorControl {
        +set_motor_type(MotorType) void
        +get_motor_type() MotorType
        +set_phase_pattern(PhasePattern) void
        +get_phase_pattern() PhasePattern
        +set_current_epsilon(Q15) void
        +is_err() uint8_t
        +is_voltage_err() uint8_t
        +is_current_err() uint8_t
    }

    class PwmWrapper {
        +set_period(arr) void
        +set_callback().apply_phases_cb / sample_point_cb
        +set_duty(Vec3D q15) void «ISR-safe»
    }

    DriverPWM *-- MotorControl
    DriverPWM *-- PwmWrapper
    DriverPWM *-- CurrentSense
    DriverPWM *-- SupplyVoltage
    DriverPWM *-- Power
    DriverPWM *-- Temperature
    CurrentSense *-- OvercurrentProxy
    SupplyVoltage *-- "2" VoltageProtectionProxy
    Power *-- OverpowerProxy
## Fault system (external view)

| Bit | Constant              | Source                       | Safe state | Recovery                          |
|-----|-----------------------|------------------------------|------------|-----------------------------------|
| 0   | `FAULT_OVERVOLTAGE`   | `supply` software check      | LowAll     | `VoltageProtectionProxy` timer    |
| 1   | `FAULT_OVERCURRENT`   | HW comparator + digital fallback | HiZ    | `OvercurrentProxy` timer          |
| 2   | `FAULT_OVERHEAT`      | `temperature` filtered value | HiZ        | Hysteresis release (no timer)     |
| 3   | `FAULT_EXTERNAL`      | hardware fault pin (`error_`)| HiZ        | None (pin level)                  |
| 4   | `FAULT_UNDERVOLTAGE`  | `supply` software check      | HiZ        | `VoltageProtectionProxy` timer    |
| 5   | `FAULT_OVERPOWER`     | `power` dual-limit check     | HiZ        | `OverpowerProxy` timer            |

Priority (high -> low): EXT(6) > OV(5) > UV(4) > OC(3) > OP(2) > OT(1).
Callbacks chain via `driver.set_fault().set_callback().oc_cb(...)...restore_cb(...)`.
All `FAULT_*` bit constants are public and may be used to decode `get_fault()`.

## Canonical wiring example (board layer)

```cpp
// --- ISR inputs ---
phy->current.set_currents(raw_currents);        // ADC ISR
TUNE_HEART.driver_phy.supply.set_voltage_irq(v_raw);
TUNE_HEART.driver_phy.temperature.set_temperature_irq(t_raw);

// --- one-time configuration ---
driver.current.overcurrent.set_limit_amps().oc_ia(4.0f).oc_ib(4.0f).oc_ic(6.2f);
driver.supply.set_limit_volts().uv_lim_volts(16.0f).ov_lim_volts(48.0f);
driver.temperature.set_limit_celsius().ot_lim_celsius(85.0f).ot_hyst_celsius(10.0f);
driver.power.set_limit().op_lim_watts(10.0f).op_lim_amps(2.0f);
driver.pwm.set_period(PWM_TIMER_PERIOD);
driver.pwm.set_callback().apply_phases_cb(TIM1_Set_PWM_Values);
driver.set_fault().set_callback()
    .oc_cb(PWM_Set_All_HiZ).ov_cb(PWM_Set_All_Low)
    .uv_cb(PWM_Set_All_HiZ).ot_cb(PWM_Set_All_HiZ)
    .op_cb(PWM_Set_All_HiZ).ext_cb(PWM_Set_All_HiZ);

// --- control loop ---
driver.tick();                                  // sensors + protections
driver.tick_icr();                              // fault manager + PWM apply
driver.set_voltage_ab_volts(v_alpha_beta);      // FOC output
```

## Internal (friend-only) surface — NOT part of the external API

These paths exist but are reachable only from `DriverPWM` itself via friendship;
listed here so the split is explicit:

- `CurrentSense::tick()`, SeqLock reader, per-channel normalization/OC fallback,
  `norm2amps()` / DAC-threshold inversion.
- `SupplyVoltage::tick()` — EMA filter advance + software OV/UV limit checks.
- `Temperature::tick()`, `overheat.activate()/deactivate()` hysteresis latch.
- `Power::tick(current_ab, supply_volts)` — raw/filtered power, OP criteria,
  usage/power_coeff computation; `set_voltage_command()` is fed automatically
  by `DriverPWM::set_voltage_ab_volts()`.
- `MotorControl::tick_voltage()` / `tick_current()` — inverse Clarke + SVPWM
  centering and Clarke recombination with duty masking and phase remapping.
- `PwmWrapper` internals: `set_phases()`, `apply_current_sample_point()`,
  `read_ticks()` (SeqLock reader).