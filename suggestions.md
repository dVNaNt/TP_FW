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

