# Factual Issues Found in the Codebase

This document lists **factual errors / inconsistencies discovered during the documentation pass**.
Per request, **none of them have been fixed in code** — this is an inventory only. Each entry explains what is wrong and what it negatively affects.

---

## 1. `src/lib/motor_driver/motor.h` — Inductance unit mismatch

**Where:** `Motor::default_inductance_`, `set_phase_inductance()`, `get_effective_inductance()`.

**Problem:** The constructor stores `default_inductance_(0.003)` while comments and parameter names claim *microhenries* (`inductance_uh`). `0.003 µH` would be 3 nanohenries; the intended value "3 mH" is only correct if the unit is actually **henries**. The setter/getter names say `uh`.

**Impact:** Any consumer that interprets the value as µH (as the API names suggest) will be off by **6 orders of magnitude**. Current/velocity loop gains derived from L (e.g., PI current-controller bandwidth `L/Ts`) become meaningless; HFI-based inductance measurements written through `set_phase_inductance()` may silently use a different unit than the default fallback, producing inconsistent control behavior depending on whether calibration ran or not.

---

## 2. `src/lib/drivers/pwm_driver_phy/pwm_driver.h` — Constructor doc vs. actual units

**Where:** `PhyDriverPWM::PhyDriverPWM(...)` Doxygen block.

**Problem:** The doc says parameters are `ch1_fs_ma` (mA) and `v_max_mv` (mV), but the signature takes floats in **amps/volts** (`ch1_fs = 0 … v_max = 24.0f`) and the board layer passes amps/volts.

**Impact:** Documentation-only, but a developer following the docs would pass mA/mV and get currents scaled ×1000 wrong and voltage protection thresholds off by ×1000 (OV/UV would never trigger or always trigger).

---

## 3. `src/lib/drivers/pwm_driver_phy/pwm_driver.h` — Sentinel semantics of NaN/+Inf duties

**Where:** `PhyDriverPWM::set_voltage_ab_volts()`.

**Problem:** `+INFINITY` maps to `INT16_MAX` on all phases = **all high-side switches ON** (phases tied to VBUS); `NAN` maps to `INT16_MIN` = **all low-side switches ON** (phases tied to GND). Several call sites/comments historically described `INFINITY` as "short to GND". Also, `-INFINITY` is not handled by either special branch and falls into the normal conversion path (producing a saturated negative duty instead of a defined safe state).

**Impact:** Misleading documentation of the safe states; a `-INFINITY` input produces an undefined-ish drive state rather than the intended release state.

---

## 4. `src/lib/drivers/pwm_driver_phy/current.h` — Possible 2× gain ambiguity in normalization

**Where:** `CurrentSensePHY` constructor (`scale_[ch] = (fs << 14) / max`) combined with `process_channel()` (`>> 15`) and `amps2norm()/norm2amps()` (full scale = 32768).

**Problem:** Net scaling factor is `(fs/max) · 2⁻¹`: a sensor reading equal to its full-scale delta maps to Q15 ≈ 16384, yet `amps2norm/norm2amps` treat 32768 as full-scale current. The two conventions are consistent **only if** `SYS_PARAMS::*_MAX_CURRENT_FS` denotes the peak-to-peak ADC span (i.e., ±fs/2 around zero). Nothing documents this requirement.

**Impact:** If the constants are actually per-side full scales, all reported currents are **half** their true value and software OC limits trip at twice the configured amperes. Hardware DAC thresholds are internally consistent (they reverse the same pipeline), so HW and SW protection would disagree.

---

## 5. `src/lib/drivers/pwm_driver_phy/current.h` — Bitwise `|` instead of `||`

**Where:** `norm2amps()`: `if (full_scale_max_ma <= 0 | norm == INT16_MIN)`.

**Impact:** No short-circuit evaluation. Functionally equivalent here (both operands side-effect-free), but fragile: if the second operand ever gains side effects or expensive computation, it will always execute.

---

## 6. `src/lib/drivers/pwm_driver_phy/current.h` — SeqLock writer uses load-then-store

**Where:** `set_currents()`.

**Problem:** The sequence counter is updated via `load(); store(seq+1); store(seq+2)` instead of `fetch_add`. Two concurrent writers can interleave and produce `seq+1 → seq+1` (a dropped increment), leaving the counter even while data changed.

**Impact:** Safe under the assumed single-writer ISR model, but silently corrupt if a second writer context is ever added — readers could accept torn samples without any error indication.

---

## 7. `src/lib/drivers/pwm_driver_phy/motor_selector.h` — Stale "DC motor" references

**Where:** Comments on `motor_type_` ("1=DC, 4=Stepper, 6=BLDC") and `set_motor_type()/get_motor_type()`.

**Problem:** `MotorType` no longer contains a DC type (only `STEP=4`, `BLDC=6`). `abc2ab_tick()` treats anything else as an error.

**Impact:** Misleading docs; a developer attempting to use a DC mode gets silent `err_current_/err_voltage_` flags instead of a compile-time guarantee.

---

## 8. `src/lib/drivers/pwm_driver_phy/motor_selector.h` — SVPWM centering includes the stepper's dummy phase C

**Where:** `ab2abc_tick()` for `MotorType_STEP`.

**Problem:** For steppers, phase C is fixed at 0 but still participates in min/max centering, shifting A/B by a common-mode offset that depends on the coil vector angle.

**Impact:** Reduces effective headroom asymmetrically across the electrical cycle and makes the duty pattern less predictable; electrically harmless (common-mode) but wastes PWM range near saturation.

---

## 9. `src/lib/drivers/pwm_driver_phy/motor_selector.h` — Hardcoded high-duty masking threshold

**Where:** `tick_current()`, `DUTY_THRESHOLD = 30720`.

**Impact:** The masking validity depends on the relationship between Q15 duty and the shunt sampling window; changing PWM resolution/frequency or the ADC sample point silently invalidates the 93.8 % figure. Not parameterized anywhere.

---

## 10. `src/lib/drivers/pwm_driver_phy/motor_selector.h` — Unused constructor parameter

**Where:** `duty_permille` parameter of `MotorSelectorPHY()`.

**Impact:** Dead API surface; callers believe they are limiting duty but nothing enforces it.

---

## 11. `src/lib/motor_driver/types.h` — Historical bit-width comment

**Where:** `OperationStatus` comment claimed values are "stored in bits 0-1", while `DriverStatus` reserves **4 bits** (0-3).

**Impact:** Doc-only; someone packing the enum into an external protocol assuming 2 bits would truncate `NotTested (0xFF)` and `Error (4)`.

---

## 12. `src/lib/motor_driver/types.h` — `PhasePattern` values double as angle offsets

**Where:** `PhasePattern_ABC=0, PhasePattern_CBA=-16384, PhasePattern_ACB=32767`.

**Problem:** An enum whose enumerators are used as signed calibration-polarity/angle constants. Comparing patterns with `==` works, but arithmetic or serialization of the enum value corrupts meaning.

**Impact:** High misuse potential; any switch table indexed by the raw value breaks.

---

## 13. `src/lib/motor_driver/calibration/calibration_table.h` — ECC lookup hardcodes ECC_TABLE_SIZE == 32

**Where:** `correct_position()`: `>>11`, `&31`, `&2047`.

**Impact:** Changing `ECC_TABLE_SIZE` at build time compiles fine but silently misindexes the eccentricity LUT (wrong corrections, position jitter). Should derive shifts from the macro.

---

## 14. `src/lib/motor_driver/calibration/calibration_table.h` — Flash monotonicity rejects duplicate samples

**Where:** `parse_load_word()`: `(int16_t)(val - cand_prev_) <= 0` marks a record invalid.

**Impact:** Two identical consecutive encoder readings (possible when the rotor dwells between steps) invalidate an otherwise-good stored table after power cycle, forcing a full recalibration.

---

## 15. `src/lib/motor_driver/calibration/angle_calibrator.h` — Speed constant comment inverted

**Where:** `ANGLE_CAL_CAL_SPEED_US = 2500 // Speed in angle increments per microsecond`.

**Problem:** It is actually **microseconds per increment** (used as a period in `calculate_speed(frequency, speed_us) = frequency*speed_us/1e6`).

**Impact:** Doc-only; anyone tuning "speed" by intuition multiplies/divides in the wrong direction by 10⁶.

---

## 16. `src/lib/motor_driver/calibration/angle_calibrator.h` — Contradictory Pass1 log/comment

**Where:** Stage comment says "positive direction"; the transition log prints "negative direction".

**Impact:** Confusing field diagnostics during calibration debugging.

---

## 17. `src/lib/motor_driver/calibration/angle_calibrator.h` — Failure message claims readiness

**Where:** `CalStage_Check` failure path logs `"Validation error - motor ready for use"`.

**Impact:** Operators reading serial logs are told the motor is ready even though validation failed and the correction table may be garbage.

---

## 18. `src/tunepulse.h` — Blocking calibration waits with tick disabled

**Where:** `tunepulse_init()`: `while (!is_calibrated()) { /* TUNE_HEART.tick(); */ }`.

**Problem:** Both calibration wait loops rely entirely on interrupt-driven progress; the periodic tick calls are commented out.

**Impact:** If the relevant interrupts are not yet enabled or misconfigured at this point in bring-up, the firmware hangs forever with no timeout and no error report.

---

## 19. `src/tunepulse.h` — Serial log contradicts actual protection configuration

**Where:** `SerialUSB.println("   [OK] Protection limits configured (OC: 5A/7.5A, OV: 53V)")` vs. actual `oc 4.0/4.0/6.2 A`, `UV 16 V`, `OV 48 V`.

**Impact:** Field technicians tuning protection from the log will set wrong expectations; kept verbatim intentionally for protocol compatibility, but it remains factually wrong output.

---

## 20. `src/tunepulse.h` — CAN left in EXTERNAL_LOOPBACK mode in production init

**Where:** `CAN_BUS.set_mode(CANMode::EXTERNAL_LOOPBACK)` inside `tunepulse_init()`.

**Problem:** In external loopback the node hears its own frames; real bus traffic/participants are ignored (depending on hardware loopback semantics).

**Impact:** In production the device will not respond to external CAN commands; looks like a test-mode leftover.

---

## 21. `src/tunepulse.h` — Duplicate include

**Where:** `#include "target/cln17_v3/interrupt_setup.h"` appears twice.

**Impact:** Harmless (include-guarded) but signals copy-paste drift.

---

## 22. `src/lib/drivers/generic/input.h` — `setCallback` returns a copy, implying chaining

**Where:** `InputWrapper::setCallback()` returns `InputWrapper` **by value**.

**Problem:** Unlike every other fluent API in the codebase (which returns references/proxies), returning by value suggests chainable usage like `in.setCallback(a).setCallback(b)`, which operates on temporaries.

**Impact:** API inconsistency; chained calls compile but confuse readers about which instance is configured.

---

## 23. `src/lib/drivers/pwm_driver_phy/voltage.h` — Undervoltage/OV protection disabled by default

**Where:** `overvoltage_limit_norm_ = INT16_MAX`, `undervoltage_limit_norm_ = INT16_MIN`.

**Impact:** Until `set_limit_volts()` is called, the software OV/UV checks can never trip; combined with issue #2-style unit mistakes, protection may remain silently inactive.

---

## 24. `src/lib/drivers/pwm_driver_phy/pwm.h` — `read_ticks` gives up after 4 attempts

**Where:** `PwmWrapper::read_ticks()` retries the SeqLock read only 4 times, then returns false.

**Impact:** Under sustained writer contention (continuous set_duty at high rate) the reader may repeatedly fail; `tick_icr()` then simply skips one PWM update (acceptable glitch, but undocumented behavior).

---

## 25. `src/lib/math/motion/motion_executor.h` — Magic HALT marker via cast

**Where:** `base_mode_ = static_cast<MotionMode>(0xFF)` used as a sentinel "HALT" state.

**Impact:** Relies on `MotionMode` having no enumerator 0xFF and on `static_cast<uint8_t>(base_mode_) == 0xFF` checks scattered through the logic; adding a new mode or changing the enum breaks the sentinel silently.

---

## Summary Table

| # | File | Severity | Category |
|---|------|----------|----------|
| 1 | motor.h | High | Unit mismatch |
| 2 | pwm_driver.h | Medium | Doc/unit mismatch |
| 3 | pwm_driver.h | Medium | Undefined sentinel path (-Inf) |
| 4 | current.h | High | Potential 2× gain error |
| 5 | current.h | Low | Style/fragility |
| 6 | current.h | Medium | Concurrency fragility |
| 7 | motor_selector.h | Low | Stale docs |
| 8 | motor_selector.h | Low | Headroom waste |
| 9 | motor_selector.h | Medium | Hardcoded threshold |
| 10 | motor_selector.h | Low | Dead parameter |
| 11 | types.h | Low | Stale docs |
| 12 | types.h | Medium | Dangerous enum design |
| 13 | calibration_table.h | High | Silent breakage on config change |
| 14 | calibration_table.h | Medium | Over-strict validation |
| 15 | angle_calibrator.h | Low | Inverted comment |
| 16 | angle_calibrator.h | Low | Contradictory log |
| 17 | angle_calibrator.h | Medium | Misleading log |
| 18 | tunepulse.h | High | Potential infinite hang |
| 19 | tunepulse.h | Medium | Wrong log values |
| 20 | tunepulse.h | High | Loopback mode in production |
| 21 | tunepulse.h | Low | Duplicate include |
| 22 | generic/input.h | Low | API inconsistency |
| 23 | voltage.h | Medium | Protection off by default |
| 24 | pwm.h | Low | Undocumented retry limit |
| 25 | motion_executor.h | Medium | Magic sentinel value |