# Suggestions — Missing Functionality

## S2. I2t winding thermal model

`can_api.md` already specifies it: nominal power / peak power / I2t budget with winding resistance
and normalization coefficient, called at a fixed rate. Natural home: next to `TemperaturePHY`,
feeding torque limiting (scale `max_c` in `MotorVectorSynthesis`) before the hard OT trip.

## S3. Motion stack is not connected

`motion_executor/motion_governor/scurve_profiler/hermite_interpolator` are complete but
`TunePulseHeart::tick()` hardcodes `position_controller.tick(real_pos, 0, 0.0f, ...)` — target
position 0, target velocity 0. Wire the executor output (position_q32, velocity_rps, torque) into
the controller inputs and expose command ingestion via CAN/USB.

## S4. Control-mode selection

The project claims position/velocity/torque modes, but there is no runtime mode switch; the tick
path is fixed to the epsilon-spring synthesizer. Add a `ControlMode` state (torque / velocity /
position / step-dir) with per-mode limit enforcement and smooth bumpless transitions.

## S5. Step/Dir interface

Pins are mapped and `cntrl_setup.h` sketches the mode matrix, but no active step-dir receiver
exists (edge-capture based step counter feeding `target_pos`). High demand for stepper-replacement
use cases.

## S6. CAN application protocol not wired

`CANDriver` works (filters, TX/RX ring), yet nothing in the control path uses it: no telemetry
streaming, no command parser, no node-ID scheme, no heartbeat/sync. The `can_api.md` UID-enumeration
protocol is specified but unimplemented.

## S7. Watchdog and clock security

No IWDG, no CSS (clock security system) enablement, no BOR configuration check. A stalled FOC ISR
currently means uncontrolled PWM (only mitigated by hardware comparators). Enable IWDG with a
window slightly above worst-case loop jitter; refresh from the control interrupt only.

## S8. Encoder integrity monitoring

MT6835 status bits (overspeed / weak field / undervoltage) and CRC are captured
(`POSITION_MT_STATUS`, `POSITION_MT_CRC`) but never checked; TLE5012 has no supervision either.
Add: CRC validation + status-bit aggregation into driver faults (encoder fault => controlled stop),
plus timeout detection when encoder DMA stops completing.

## S9. Parameter store schema/versioning

`FlashEEPROM` provides raw partitioned storage and `Motor` can save/load, but there is no record
schema version, CRC over records, or migration path — a layout change silently reads garbage.
Add: magic + version + length + CRC header per record, and a defaults-reset policy.

## S10. Host command interface

Calibration/save/load functions exist but there is no USB CDC command parser (the firmware logs
over SerialUSB but cannot be commanded). Define a small text/binary CLI: mode set, targets,
limits get/set, telemetry streaming, calibrate, save/load, dfu.

## S11. Field weakening and MTPA

`foc-current.h` has power limiting (regen + consumption) but no explicit field-weakening (Id<0 vs
speed) or MTPA for salient machines; high-speed operation saturates the voltage circle without a
graceful torque roll-off. Both slots already exist in the current-loop structure.

## S12. Regenerative handling options

OV protection currently shorts phases (low-side braking). Add support for a brake chopper /
resistor output and/or bus-voltage-dependent regen current limiting so OV events don't force
full stop on high-inertia loads.



## S15. Multi-axis readiness

Global singletons (`TUNE_HEART`, `CAN_BUS`, static DMA buffers in spi_setup.h) prevent a second
drive instance on the same MCU or shared code in a multi-node CAN network. The refactor plan
(R2/R5) addresses this; the application layer (S6) should be designed node-ID-relative from the
start.


## S16. Resistance identification is unreliable

The DC two-point fit in `process_resistance_measure()` is ill-conditioned on real hardware: it
solves `R = V1 / (2*dI)` from a small difference of two large currents, and measured `dI` lands at
11-13% of `I1` where the model expects ~50%. Observed across three boots of one board: `dI` =
0.26 / 0.11 / 0.23 A, mapping to R = 15.8 / 37.4 / 17.4 Ohm. The correction is now rejected by
`RES_CORRECTION_MAX_FACTOR`, so R falls back to the raw `V/I` estimate — biased high by the bridge
offset drop the correction existed to remove, and still noisy (4.11 / 4.67 / 4.64 Ohm).

Prime suspect is a large residual current-sense offset (~1.4 A): it cancels out of `dI` and out of
the HFI ripple, but inflates `I1`, which is exactly the disagreement observed. The HFI impedances
are ripple-based and therefore offset-immune, and solving `Z(1 kHz) = 9.55` / `Z(1.5 kHz) = 14.77`
for both unknowns yields L ~1604 uH and a small R — self-consistent, and matching the 1.6 mH the
board later measures.

Next step: log the phase currents with zero voltage commanded, right after the current-sense offset
calibration completes, to confirm or rule out the offset. Likely real fix: derive R and L jointly
from the two HFI frequencies (data already collected every boot, offset-immune) and retire the DC
two-point correction along with its guard.

## S17. Flash-programming window is published from the target, not the bridge vector

`TunePulseHeart::tick()` opens the NVM window in the calibrated branch by zeroing the current
*target* and immediately setting `flash_window = true`. The target feeds `tick_control_foc()`, whose
PI integrator does not empty in one pass, so the bridge is still driving while the flag claims a
zero-current vector is latched. Flash programming then stalls the ISR with interrupts off and
freezes that live vector, yanking the rotor: observed as `tick/s` collapsing 20000 -> ~1500 and the
supply pumped from 23 V to 37 V by regenerative energy.

Every other path that opens the window bypasses FOC and drives a sentinel directly (`INFINITY` in
the `NotCalibrated` branch, `NAN` in the default branch). The calibrated branch should do the same —
short the phases and reset the FOC integrator for the duration of the transfer — so the published
flag describes the vector actually on the bridge, as the invariant requires.

## S18. Motor-profile save churns on the 15% resistance band

`finish_calibration_success()` re-saves the profile whenever the freshly measured resistance differs
from the stored one by more than 15%. With R as noisy as S16 leaves it, that test flips irregularly
between boots, so the profile is rewritten to flash on a large fraction of startups — each write
triggering S17. Fixing S16 removes the trigger; fixing S17 makes it harmless. Hysteresis on the
comparison would be a cheap third line of defence.

## S19. HFI generator sample rate is hardcoded

`MotorCalibration` constructs `hfi_gen_(20000)` and never calls
`FrequencyGenerator::set_frequency_system()`, so the injection sample rate is fixed at compile time
rather than taken from `TunePulseHeart::clock_`. It matches `PWM_TIMER_FREQ_HZ` today, but the
surrounding cycle-count comments are written against 18 kHz (`CALIB_PAUSE_CYCLES`,
`INDUCTANCE_MEASURE_CYCLES_F1/F2`), so the two already disagree by ~10%. If the PWM frequency is
ever changed, the injected pitch and the measured inductance drift with it silently.
