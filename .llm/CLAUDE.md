# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

TunePulse Firmware — open-source motor control firmware for STM32G431CB. Controls steppers and
BLDC/PMSM with FOC, closed-loop position/velocity/torque. PlatformIO + Arduino (STM32duino),
almost entirely header-only C++.

## Build / Upload / Debug

```bash
python -m platformio run -e cln17_v3_release            # build
python -m platformio run -e cln17_v3_release -t upload  # flash via ST-Link
python -m platformio run -e cln17_v3_release -t clean
python -m platformio debug -e cln17_v3_release
```

If `platformio` is not on PATH, use `~/.platformio/penv/bin/pio` (already allow-listed in
`.claude/settings.local.json`).

**`cln17_v3_release` is the only live environment.** `radix_release` and `cln17_v2_0_release` are
commented out in `platformio.ini`. `src/target/radix/` still exists (left at the old path deliberately) but does not compile in any
current build — do not assume changes there are covered by a build.

**Upload flashes every connected ST-Link in parallel.** `extra_scripts = post:tools/upload_all_stlinks.py`
replaces the upload target: it enumerates all ST-Link probes and programs each attached board
concurrently. It detects probes through the Windows PnP subsystem, so `-t upload` is Windows-only;
on macOS/Linux flash with STM32CubeProgrammer or a manual OpenOCD invocation.

**Flash budget is 114688 bytes, not 131072.** `board_upload.maximum_size` reserves the top of flash
for the persistent NVM partitions (`nvm_flash.h`: calibration + user + node-config sections, 2 KiB
each by default = 6 KiB; the 16 KiB reservation leaves 10 KiB of slack). PlatformIO's percentage is
against 114688; `changelogger.py` computes against 131072, so the two disagree — always quote raw
byte counts.

The node-config partition is deliberately separate: the motor-profile engine erases its whole
partition when it runs out of slots (~60 saves), which would otherwise destroy the stored node
address.

Reference build (clean `main`): RAM 7420 B, Flash 64664 B.

### After any code change

1. Build and confirm it succeeds; report the flash/RAM delta in bytes.
2. Treat compiler warnings as errors.
3. Record the change (see Changelog below).

## Changelog

Every substantive change gets an entry via the generator — do not hand-edit `changelog/changelog.md`:

```bash
python changelog/changelogger.py "Summary" "Change A; Change B" "Reason A; Reason B" "Notes" <flash_bytes> <ram_bytes>
```

Semicolons split a field into bullets. The script diffs against the previous entry's sizes, so the
byte counts must come from the build you just ran.

## Tests

`test/test_tpcp/` holds host-side conformance tests for the CAN protocol wire format (Unity):

```bash
python -m platformio test -e native_tpcp    # 15 tests: codecs, frame layouts, identifier map
```

They build nothing from the firmware tree — `tp_wire.h` / `tp_chunks.h` are hardware-independent by
design, which is what makes the wire format testable without a board. `test_filter` keeps this
environment away from `test/test_native/`.

`test/test_native/` holds GoogleTest suites, but **they are currently orphaned**: no environment
builds them, and the sources still `#include` the pre-refactor tree (`lib/feedback/`,
`lib/math_integer/`, `lib/motor_driver/`) which no longer exists. Before relying on one, expect to
add an environment and repoint its includes at `src/lib/…`. `test/support/arduino_mock.h` stubs the
Arduino layer for host builds.

## Architecture

### Real-time control path (the important part)

The control loop does **not** run from `loop()`. It is entirely interrupt-driven; `main.cpp` brings
the drive up and then pumps the CAN protocol and a diagnostic console heartbeat:

```
TIM1_CC ISR  → FOC_TimTrig_Callback()   : start ADC DMA, start encoder SPI DMA,
                                          TP.driver_phy.tick_icr() pushes new duties
ADC DMA done → ADC_READY_Callback()     : range-check phases, feed current / supply / temperature
SPI DMA done → POSITION_Callback()      : encoder.set_raw_position(), then TP.tick()
```

The callback bodies live in `src/bsp/boards/<board>/interrupt_setup.h`; the interrupt VECTORS that
invoke them are defined in `src/bsp/boards/<board>/board.cpp`. `TP.tick()` is the single control
pass, gated on `driver_phy.current.is_changed() & driver_phy.supply.is_changed()` — the bitwise `&`
is deliberate so both freshness flags are cleared every pass. A `std::atomic<bool> is_running`
re-entrancy guard drops a sample rather than letting an ISR reenter a running pass.

### `TunePulseHeart` (`src/lib/lib.h`)

The single global is `TP`, defined in `src/bsp/boards/<board>/board.cpp` from board constants and
declared `extern` in that board's `tunepulse.h`.
It owns: `clock_` (Frequency), `motor_params` (Motor), `current_driver` (CurrentControl, the FOC
current loop), `calibration_dump` + `system_dump` (two independent `FlashEEPROM` instances bound to
separate NVM partitions), `angle_calibrator`, `position_controller` (MotorVectorSynthesis),
`driver_phy` (DriverPWM: PWM + current sense + supply + temperature + power), and `encoder`.

Declaration order matters: `calibration_dump` is declared before `angle_calibrator` because its
address is injected into the calibrator's constructor.

`tick()` dispatches on `motor_status`:
- `Success` → run calibration sweep if not yet calibrated, else hand the pass to `drive`
  (`DriveControl`), which resolves the control mode, the queued trajectory and the master watchdog
  into one action plus its targets; `tick()` only translates that action to the physical layer
- `NotCalibrated` → short all phases (`INFINITY` sentinel) until current-sense offsets are known
- `CalibError` → zero current target
- default → release the bridge / Hi-Z freewheel (`NAN` sentinel), return early

The `INFINITY` / `NAN` values passed to `set_voltage_ab_volts()` are **sentinels**, not math:
`+INFINITY` → `INT16_MAX` duty on every phase = all HIGH-side on (every terminal tied to the DC bus,
i.e. a shorted winding = braking). `NaN` → `INT16_MIN` duty, whose magnitude exceeds the timer
period, so `pwm_phase_ccr()` disables BOTH FETs of every phase: that is a true Hi-Z **freewheel**,
not "all low-side on". Low-side braking exists only as `PWM_Set_All_Low()` (the OV fault callback),
which is not reachable through a sentinel.

### `DriveControl` (`src/lib/motor_control/drive_control.h`)

Everything a remote master can change about how the motor behaves lives here: the control mode
(disabled / torque / velocity / position / motion), the position, velocity and current targets, the
trajectory queue (`MotionExecutor`, 32 segments) and the master-liveness watchdog.

Concurrency is the point of the design. Commands originate in THREAD mode (the protocol polled from
`loop()`) and every target is consumed by the 20 kHz control ISR, so **all** mutation is funnelled
through one lock-free SPSC mailbox: `post()` from thread mode, `drain()` inside `tick()`. The mode
variable, the setpoints and the executor are therefore touched by the ISR only. Telemetry flows back
through a sequence lock written by the ISR (reader may spin, writer never does). Never reach into
control state from thread mode — post a `DriveRequest` instead.

`tick()` takes an `inhibit` flag: when an NVM programming window is open it freezes trajectory time
and commands zero current, so a flash stall can never advance a queued trajectory.

### Flash-programming window invariant

`tp_flash_programming_window` is published by `tick()` and describes the vector *currently latched
on the bridge*. Flash programming stalls the single-bank CPU for several PWM periods (tens of ms for
an erase), so NVM writes are only permitted when the last pass commanded zero current. The write
engines sample the flag on the *next* pass — that is the stall case. Any new code path that writes
flash must run inside this window, or the rotor gets yanked against a frozen vector.

### Layout

- `src/lib/` — algorithms, header-only, hardware-agnostic
  - `drivers/pwm_driver/` — `DriverPWM`, current sense, supply voltage, temperature, power/OP limits
  - `drivers/encoder_phy/`, `drivers/flash_eeprom.h` — encoder interface, EEPROM emulation
  - `motor_control/` — motor params, angle calibration + table, current control (`foc-current.h`, HFI),
    `drive_control.h` (control-mode machine + command mailbox + trajectory front end)
  - `math/` — `filters/` (EMA, median3, PLL), `motion/` (S-curve, Hermite, executor, governor,
    observer), `helpers/`, `trigonometry.h`
- `src/bsp/` — board support, split three ways by what a thing actually depends on:
  - `stm32g4/` — peripheral drivers shared by every board on this MCU family: `cordic.h`,
    `tim1_pwm.h`, `bootloader.h`, `nvm_flash.h`, `comp_dac.h`, `fdcan.h`. They take their
    board-dependent values from the active board's `target.h`, which the per-environment include
    path resolves, so none of them branches on a board name.
  - `stm32g4/chip.h` — the small residue that is a property of the DIE rather than the board
    (COMP4's vector name, whether the part has ADC3), keyed off the CMSIS define from the board
    JSON. Keep it small: anything a board could choose belongs in `target.h`.
  - `boards/<board>/` — what is genuinely board-specific: `target.h` (the descriptor: pinout,
    full scales, and the entries the shared drivers read), `board.cpp`, plus the peripherals whose
    two versions encode real hardware differences rather than duplication — clock (HSE 12 MHz vs
    HSI16), ADC (the G474 samples phase C on ADC3), GPIO, SPI (CLN234 shares SPI1 between the
    encoder and the DRV8353), and the application body in `run.h`.
  - `chips/` — target-agnostic chip register maps and command builders (DRV8353S, MT6835, NTC).
    No MCU dependency; usable from any board.
- `src/bsp/boards/<board>/board.cpp` — the board's ONE translation unit. Everything that must
  exist exactly once lives here: the interrupt vectors, the HAL weak-override callbacks, the naked
  NMI handler, and the `TP` object. Every other board file is a header of inline definitions that
  this unit is the sole includer of.
- `src/bsp/board.h` — the whole contract between `main()` and the board: `board_setup()` and
  `board_loop()`. It names no board. `main.cpp` includes only this, never a board's peripheral
  headers, which is what keeps a board's vectors and globals confined to one object file.
- `src/bsp/boards/<board>/tunepulse.h` — board integration: declares `TP`, registers
  LED/fault/limit callbacks, `tunepulse_init()` bring-up sequence
- `src/api/` — TunePulse CAN Protocol (TPCP) v1, header-only, wired into the build via `main.cpp`
  - `tp_wire.h` / `tp_chunks.h` — identifier map, opcodes, error codes, fixed-point codecs and the
    8-byte frame layouts. Hardware-independent: a host tool includes just these two.
  - `tp_registers.h` — 70-entry typed register map (binary searched, so it MUST stay sorted by id;
    `TpProtocol::begin()` logs a warning at boot if it is not) plus config → hardware application
  - `tp_node.h` — 56-bit UID, persistent node configuration record, UID discovery state machine
  - `tp_engine.h` — dispatch, cyclic telemetry, emergency frames, watchdog, persistence
  - `tp_can.h` — binds the engine to FDCAN1; owns the config-partition `FlashEEPROM`
  - `CAN_PROTOCOL.md` — the specification (identifier map, opcodes, registers, worked examples)
- `old/` — pre-refactor sources kept for reference, excluded from the build

`suggestions.md` tracks known gaps. The `math/motion/` stack and the runtime control-mode switch are
now wired in through `DriveControl`; `src/api/CAN_PROTOCOL.md` §9 lists what the protocol
deliberately does not implement yet (calibration abort, full-calibration sequencing, absolute
S-curve targets, firmware update over CAN).

### Adding or changing board code

Two rules keep the split working, and both fail at LINK time rather than compile time, so they are
easy to break without noticing:

1. **`main.cpp` must not include anything from a board.** It includes `bsp/board.h` and nothing
   else board-related. Pulling a peripheral header in there puts that board's definitions into a
   second object file and the duplicate symbols come back.
2. **Mutable state in a shared `bsp/stm32g4/` header must be `inline`, never `static`.** A shared
   header has to give every includer the same object; `static` hands each translation unit its own
   copy and lets them silently disagree.

Verify a refactor with `python tools/mapdiff.py BASE.map NEW.map`. `firmware.bin` is not
byte-reproducible, so comparing images proves nothing; the map compares symbol SIZES and ignores
addresses and which object file a symbol landed in, which is exactly what a pure refactor may
change. Maps live at `.pio/build/<env>/firmware.map`.

### Compile-time hardware switches

- Encoder selection is a `#define` in `src/bsp/boards/cln17_v3/spi_setup.h`
  (`ENCODER_TYPE ENCODER_MT6835` or `ENCODER_TLE5012B`) — there is no runtime detection. It also
  selects the DRV_EN pin in `tunepulse.h` (PB12 for MT6835 boards, PB11 for TLE5012B boards).
- Board selection is a preprocessor define (`TARGET_CLN17V3`), not runtime branching.
- `tunepulse_init()` **blocks** waiting on current-sense offset calibration and then motor angle
  calibration; both are advanced only from interrupt context.

## Logging

Use the macros from `src/lib/dev_log.h` — `TP_LOG`, `TP_LOG_VAL`, `TP_LOG_ARRAY`, `TP_LOG_VALS` —
never bare `SerialUSB.print` in ISR-reachable code. `USBSerial::write()` spins on host back-pressure;
called from the 20 kHz control ISR with the motor energized it latches the last voltage vector on the
bridge. The macros check free TX room first and drop output instead of blocking. They compile to
nothing when `DEV_LOG_ENABLE` is undefined.

## Code Style

Clang-format: Chromium base, 130-char columns, 4-space indent, `AccessModifierOffset: -4`.

- **Every file starts with a Doxygen header** containing `@file`, `@brief`, a detailed description of
  what it does and why, then `Copyright 2026, creapunk` / `Licensed under the Apache License, Version 2.0`.
- **Comment every line**, in English. Translate any non-English comments you encounter while editing.
- Complex algorithms get a block comment covering the logic flow, formulas, complexity, edge cases,
  and a reference link where one exists.
- When you change code, update its documentation to the same level of detail it already had.
- C++ structs/classes with explicit access control; `const` members where applicable; `constexpr` for
  compile-time constants. Do not add getters/setters unless asked. Group related methods together.
- Integer / fixed-point math wherever possible (Q15/Q31, bit shifts); the FPU is available
  (`__FPU_PRESENT=1`) and floats are used in the driver layer, but hot paths should stay integer.
- Do not create README or other doc files unless explicitly asked.

Note: `.kiro/steering/tech.md` carries the original long-form version of these rules but is
gitignored, so it may be absent in a fresh clone — the above is the authoritative copy.

## Git Workflow

Branches: `main` (release, tagged), `develop` (pre-release), simple branches, feature branches.
Naming: `<issue#>_<description>` (e.g. `1234_fix_voltage_control`); branches for the same task share
the issue prefix. PRs merge into `develop`, then into `main` after hardware testing. Feature-branch
sub-branches merge into their feature branch. Versioning `v<major>.<minor>`; tag on merge to `main`.
Full rules in `CONTRIBUTION.md`.
