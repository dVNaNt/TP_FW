# Suggestions — Missing Functionality

## S2. I2t winding thermal model

`can_api.md` already specifies it: nominal power / peak power / I2t budget with winding resistance
and normalization coefficient, called at a fixed rate. Natural home: next to `TemperaturePHY`,
feeding torque limiting (scale `max_c` in `MotorVectorSynthesis`) before the hard OT trip.

## S3. Motion stack — DONE (2026-09-05)

`MotionExecutor` is wired into the control path through `DriveControl`
(`src/lib/motor_control/drive_control.h`): its output (position_q32, velocity_rps, torque) feeds the
position controller in `DriveMode::Motion`, and segments are ingested over CAN as bit-packed
PVT/PVcTF burst frames. `MotionGovernor` remains fully commented out; its only live responsibility
was the Q10 time scale, which `DriveControl` now owns directly. `SCurveProfiler` is still unused:
`TP_OP_MOTION_SCURVE` synthesizes a single Hermite segment from a trapezoidal time estimate rather
than emitting a true four-phase jerk-limited profile. Wiring `SCurveProfiler` in is the remaining
work here.

## S4. Control-mode selection — DONE (2026-09-05)

`DriveMode` (disabled / torque / velocity / position / motion) is selected at runtime over CAN
(`TP_OP_CONTROL_MODE`, register 0x0300). Transitions are bumpless: every mode entry re-seeds the
state it integrates from, and velocity commands are slewed under the acceleration envelope. Limits
are enforced per mode through the shared current/velocity/acceleration envelope. STEP-DIR is still
missing — see S5.

## S5. Step/Dir interface

Pins are mapped and `cntrl_setup.h` sketches the mode matrix, but no active step-dir receiver
exists (edge-capture based step counter feeding `target_pos`). High demand for stepper-replacement
use cases.

## S6. CAN application protocol — DONE (2026-09-05)

TPCP v1 is implemented in `src/api/` and specified in `src/api/CAN_PROTOCOL.md`: identifier map with
hardware per-node filtering, UID discovery, request/response and fire-and-forget setpoint channels,
70-register typed parameter map, persistence in a dedicated NVM partition, cyclic telemetry, a
heartbeat, emergency frames and a master-liveness watchdog. Two defects in the underlying driver
were fixed on the way: the RX path truncated the 11-bit identifier to 8 bits (collapsing every
protocol channel onto one value) and dropped every frame shorter than 8 bytes.

Not implemented in v1 (see CAN_PROTOCOL.md §9): calibration abort, full-calibration sequencing,
absolute S-curve targets, and firmware update over CAN.

## S6b. Trajectory streaming starves on fine segments

Measured on CLN17 v3 over the USB CDC transport, 2 rev per leg in 1.0 s:

| Segments | Segment | Cold leg error | Warm leg error |
| :------- | :------ | :------------- | :------------- |
| 20       | 50 ms   | -0.092 rev     | -0.005 rev     |
| 50       | 20 ms   | -0.677 rev     | +0.676 rev     |
| 100      | 10 ms   | -1.118 rev     | (not measured) |

Two separate problems.

**Streaming rate.** The host manages roughly a frame every few milliseconds over
USB CDC, so segments shorter than ~50 ms starve the executor. Sending the whole
burst in one write makes it far WORSE (-1.40 rev at 20 segments): 260 bytes in
one go overruns the board's CDC receive buffer and frames are lost silently,
with nothing counting the loss. A batch of four frames per write is the
compromise the host tool now uses. Over real CAN the frame rate is far higher,
so this is largely a serial-transport limit - but it needs re-measuring on a bus
before that is assumed.

**Loss on starvation.** When the queue empties mid-trajectory `MotionExecutor`
enters HALT, and the elapsed time is not recovered when segments arrive again -
the move simply comes up short instead of stretching. A streaming interface
wants the opposite: hold the trajectory clock while starved, and warn. The
`TP_WARN_QUEUE_LOW` flag already exists to signal it; nothing acts on it.

Also unexplained: the FIRST leg after entering motion mode is consistently
~0.09 rev short at 20 segments while subsequent legs land within 0.005 rev.
That is on the cold-start path through `enter_halt_mode`/halt recovery, where
the first segment's displacement is applied to the anchor in the same pass it is
entered. Worth tracing with the queue pre-filled before motion mode is armed.

Nothing here is a protocol or DriveControl defect: the frames arrive, are
accepted (`dropped=0`) and the following error is zero at rest.

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

## S9b. Node configuration record is versioned and CRC-checked

Unlike the motor-profile record (S9), the node configuration record written by `TpNodeStore` carries
a magic word, a layout version and a CRC-32 over the whole record, and is appended so the newest
valid record wins the boot scan. It is the template to follow when S9 is addressed for the motor
profile and the calibration table.

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



## S15. Multi-axis readiness — PARTIALLY DONE (2026-09-05)

The application layer is now node-ID-relative end to end: every identifier is derived from the
assigned address, acceptance filters are programmed per node, and the discovery protocol hands out
addresses on a shared bus. `TpProtocol` takes the drive by pointer and the transport by callback, so
nothing in `src/api/` assumes a singleton.

Still open: `TP` and `CAN_BUS` remain file-scope globals and spi_setup.h keeps static DMA buffers, so
a SECOND drive instance on one MCU is still not possible (R2/R5).


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
