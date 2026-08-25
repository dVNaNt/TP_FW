# Refactor Plan

Architectural recommendations for `src/lib` + board layer (`src/tunepulse.h`,
`src/target/cln17_v3`). Ordered from "do first / low risk" to "structural".

---

## R1. Unify the protection-proxy family

Today there are three structurally identical protection implementations:

- `VoltageProtectionProxy` (voltage.h) — recovery-timer policy
- `OvercurrentProxy` (current.h) — recovery-timer policy, per-channel limits
- `TempProtectionProxy` (temperature.h) — immediate-release policy

All three: atomic flag + limit value + trigger/clear + fluent setter. Extract a common
template:

```cpp
template <typename ReleasePolicy>   // TimerRelease | ImmediateRelease
struct ProtectionProxy { /* shared state + API */ };
```

Benefits: one place to add hysteresis, event counters, callback-on-edge; ~200 lines removed;
new protections (e.g. I2t) become one-liners.

## R2. Narrow the ISR-to-application interface

Board ISRs currently reach deep into the application object
(`TUNE_HEART.driver_phy.current_sense.set_currents(...)`, `g_heart_ptr->driver_phy.set_fault()...`).
This couples the target layer to library internals and makes multi-board support fragile.

Introduce a minimal C-ABI event surface owned by the library:

```cpp
namespace tp { extern "C" void on_adc_ready(const AdcFrame&); }
```

The target layer only captures hardware data into a POD frame and calls it; the library decides
what to do. `TUNE_HEART`/`g_heart_ptr` become implementation details of `lib.h`.

## R3. Decompose `TunePulseHeart::tick()`

`tick()` mixes: data-freshness gating, sign conventions, flash service scheduling, calibration
state machine dispatch, position control and FOC voltage synthesis. Refactor into explicit
stages with single responsibilities:

1. `Sampler::gather()` — encoder/ADC snapshot (with the err_sign bug fixed);
2. `Supervisor::dispatch()` — MotorStatus state machine returning a command enum;
3. `Controller::run()` — calibration tick or position/current loop;
4. `Modulator::apply()` — FOC + PWM hand-off.

This enables unit-testing each stage on the host without hardware.

## R4. Kill the duplicated configuration channels

PWM frequency/period arrive through **four** independent paths: `Frequency` class,
`PWM_TIMER_PERIOD` macro (defined in both lib.h and timer_pwm_setup.h), `pwm.set_period()` and
the TunePulseHeart constructor. Collapse into one `BoardConfig` constexpr struct defined by the
target layer and consumed everywhere else. Same for current/voltage full scales (constructor
args vs `SYS_PARAMS`).

## R5. Header-only single-TU strategy

Everything is header-only and glued in `tunepulse.h`, giving long builds and accidental
inline-duplication. Keep header-only for hot templates, but move cold code (CAN driver,
FlashEEPROM logic, calibration state machines) into .cpp files with explicit interfaces. This
also removes the "statics-in-headers" hazard (e.g. `SPI1_TX_BUF` static buffers, NVM statics)
that silently duplicate per translation unit.

## R6. Consistent error taxonomy

Mixed conventions: bool returns, `uint8_t is_err()` flags, atomic fault bitfields,
`MotorStatus` enums. Define one `Result<T>`-style light type (status + payload) for slow-path
APIs and keep the atomic bitfield strictly for ISR paths. Document which APIs are ISR-safe.

## R7. Dead-code disposition

`cntrl_setup.h` (fully commented), legacy encoder block in `spi_setup.h`, commented design
sketch in `motion_governor.h`, `old/` tree: either revive behind a build flag or delete. Keeping
large commented bodies inside active headers costs review time and build time.

## R8. Naming and ownership pass

- `MotorSelectorPHY` renamed to `MotorControlPHY` (done) — continue: `motor_ctrl_` member naming,
  `driver` vs `driver_phy` vs `motor_ctrl_` triad is confusing; rename `DriverPWM driver` to
  something like `control_loop_`.
- `TUNE_HEART` global should live in an anonymous namespace + accessor.
- Hungarian-ish suffixes (`_q32`, `_norm`, `_amps`) are good — enforce via clang-format/
  clang-tidy config already present in the repo.

## R9. Testing infrastructure

No unit tests exist for the fixed-point math (vectors.h, motor_control transforms, calibration
table interpolation, scurve profiler). These are pure functions — ideal for host-side tests
(native platformio test env). Priority: `calibration_table` interpolation, `MotorControlPHY`
phase remapping permutations, `FilterEMA`/`FilterLongEMA` convergence, motion executor state
transitions.

## R10. Suggested migration order

1. R1 (proxy unification) — mechanical, low risk.
2. R4 (config consolidation) — removes a whole bug class.
3. R2 (ISR interface) — needed before multi-target work (radix board shares lib).
4. R3 (tick decomposition) + R9 (tests) — together.
5. R5/R7/R8 — continuous hygiene.

