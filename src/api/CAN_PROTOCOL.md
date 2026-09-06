# TunePulse CAN Protocol (TPCP) v1.0

Application protocol for TunePulse drives on CAN 2.0A and CAN-FD.

Reference implementation: `src/api/` (header-only, hardware-independent except
for `transport/`). The wire format lives entirely in `tp_wire.h` + `tp_chunks.h`;
a host tool can include those two files and be bit-compatible with the drive
without pulling in anything else.

---

## 1. Design rules

1. **Every frame is exactly 8 bytes.** That is the whole payload of a classic
   CAN 2.0A frame, so the protocol runs unchanged on a classic bus. On a CAN-FD
   link the same 8-byte frames are sent as FD frames; payloads of 9–64 bytes
   stay reserved for bulk transfer (v1.1).
2. **Byte 0 is the opcode.** The one exception is the bit-packed motion burst
   frame, which is marked by bit 7 of byte 0 — a bit no opcode ever sets.
3. **Multi-byte integers are little-endian**, naturally aligned wherever the
   layout allows. Sub-byte fields are allocated from the LSB upward. The single
   documented exception is the motion burst frame (§6.2), which is a bit field.
4. **Semantic length never rides in the DLC.** A DLC is not a byte count on
   CAN-FD (sizes above 8 are quantized to 12/16/20/24/32/48/64), so anything
   variable-length carries its length in the payload.
5. **A node without an address is silent.** It accepts network management only
   and transmits nothing until an address is assigned.

---

## 2. Identifier map

11-bit standard identifiers only. Lower identifier = higher bus priority. The
map is ordered so a tripping drive outranks everything, network management comes
next, commands outrank responses, and status telemetry can never delay a command.

| Range          | Direction      | Purpose                                    |
| :------------- | :------------- | :----------------------------------------- |
| `0x000 + node` | node → bus     | Emergency / fault report (highest priority)|
| `0x100 – 0x10F`| master → bus   | Network management, sync, discovery        |
| `0x200 + node` | master → node  | Real-time setpoint stream (never answered) |
| `0x300 + node` | master → node  | Request / command channel                  |
| `0x400 + node` | node → master  | Response channel                           |
| `0x500 \| byte`| node → master  | Discovery byte arbitration                 |
| `0x600 + node` | node → bus     | Heartbeat and cyclic telemetry             |
| `0x700 – 0x7FF`| —              | Reserved: bulk transfer / firmware update  |

**Node addresses:** `0` = unassigned, `1–254` = assigned, `255` = broadcast.
`0x2FF` and `0x3FF` therefore address every node at once.

`0x100`, `0x300+n` and `0x400+n` are inherited verbatim from the original
`can_api.md` draft. The discovery reply moved off `0x400` (where it collided
with the response channel of an already-addressed node) onto its own `0x500`
block.

### Hardware filtering

A node programs four FDCAN standard filter slots:

| Slot | Type  | Accepts                        | Active when       |
| :--- | :---- | :----------------------------- | :---------------- |
| 0    | range | `0x100 – 0x10F`                | always            |
| 1    | dual  | `0x200 + node`, `0x2FF`        | address assigned  |
| 2    | dual  | `0x300 + node`, `0x3FF`        | address assigned  |
| 3    | range | `0x000 – 0x7FF`                | promiscuous mode  |

Everything else is rejected in hardware: traffic addressed to other drives costs
this node no interrupts at all. Non-matching standard frames, extended frames
and remote frames are all rejected globally.

---

## 3. Data types

| Name        | Bits | Encoding                                                     |
| :---------- | :--- | :----------------------------------------------------------- |
| `bf16`      | 16   | Top 16 bits of an IEEE-754 float. Full range, ~0.4 % error.   |
| `f24`       | 24   | Top 24 bits of an IEEE-754 float. Full range, ~0.008 % error. |
| `q1.31.16`  | 48   | Signed position in revolutions, 1/65536 steps, ±32768 rev.    |
| `q1.7.16`   | 24   | Signed following error in revolutions, ±128 rev.              |
| `q1.3.8`    | 12   | Signed time scale, 256 = 1.0×, range −8.0 … +7.996.           |
| `q1.1.6`    | 8    | Signed normalized ratio, 64 = 1.0, range −2.0 … +1.98.        |
| `volts_q2`  | 8    | Unsigned volts, 0.25 V per LSB, 0 … 63.75 V, saturating.      |
| `temp_c`    | 8    | Signed whole degrees Celsius; `−128` means "no data".         |

`f24` and `bf16` are **truncations** of an IEEE-754 float, not fixed-point
integers: decoding is a pure left shift, with no sign extension. Sign-extending
them overwrites the exponent and turns every negative value into an infinity or
a NaN — a bug the draft codec shipped with, now covered by `test/test_tpcp`.

---

## 4. Opcodes

`R` = read, answered with the same opcode. `W` = write, answered with `ACK`
(`0x7E`) or `NACK` (`0x7F`) unless noted.

### 4.1 Status and telemetry (`0x00 – 0x0F`)

These double as cyclic telemetry chunk identifiers.

| Op     | Access | Chunk                                                        |
| :----- | :----- | :----------------------------------------------------------- |
| `0x00` | —      | NOP. An all-zero frame is padding and is always ignored.      |
| `0x01` | R      | Device Info: hardware revision, firmware version, device code |
| `0x02` | R      | Device UID: the 56-bit identity discovery arbitrates on       |
| `0x03` | R      | Device Status: uptime + the aggregated status word            |
| `0x04` | R      | Motor Status: mode, type, phase order, poles, temperature     |
| `0x05` | R      | Electric Status: efficiency, power, supply, current           |
| `0x06` | R      | Motion Status 1: motion mode + 48-bit absolute position       |
| `0x07` | R      | Motion Status 2: velocity, acceleration, torque               |
| `0x08` | R      | Motion Status 3: following errors, queue depth, time scale    |
| `0x09` | RW     | Motor Properties: thermal limit, R, L, Kt                     |
| `0x0A` | R      | Encoder Status: raw angle, revolutions, health                |
| `0x0B` | R      | Driver Status: state, mode, faults, warnings, sub-states      |
| `0x0C` | R      | Calibration Status: routine, stage, progress, result flags    |
| `0x0D` | R      | Comm Status: bus state, error counters, drops, rejects        |
| `0x0E` | R      | Power Status: load, bus current, bus power, thermal headroom  |
| `0x0F` | R      | Heartbeat (§7)                                                |

### 4.2 Real-time control (`0x10 – 0x1F`)

Accepted on **both** the request channel (answered) and the setpoint stream
(never answered). Identical behaviour on either.

| Op     | Payload                                                          |
| :----- | :--------------------------------------------------------------- |
| `0x10` | Control Mode: `[1] mode`, `[2] enable`                            |
| `0x11` | Target Position: `[1] flags`, `[2..7] q1.31.16`                   |
| `0x12` | Target Velocity: `[1..3] f24 RPS`, `[4..5] bf16 accel`, `[6] flags`|
| `0x13` | Target Torque: `[2..3] bf16 A`, `[4..5] bf16 limit`, `[1] flags`  |
| `0x14` | Motion Limits: `[1..3] f24 vel`, `[4..5] bf16 acc`, `[6..7] bf16 jerk` |
| `0x15` | Stop: `[1]` 0 hold, 1 freewheel, 2 brake, 3 quick stop            |
| `0x16` | Anchor: `[2..7] q1.31.16` — redefine "here" without moving        |
| `0x1A` | S-curve move: `[1] flags`, `[2..4] q1.7.16`, `[5..6] bf16 vel`    |
| `0x1B` | Queue Control: `[1] action`, `[2..3] int16 time scale (Q10)`      |
| `0x1C` | R — Queue Status: state, depth, free slots, time scale, drops     |

`0x18` and `0x19` are **acknowledgement labels only**: a PVT / PVcTF segment
needs all 64 bits and travels as the bit-packed burst frame (§6.2), so it has no
opcode byte to echo. Sending `0x18` or `0x19` as a request is rejected with
`TP_ERR_UNKNOWN_OPCODE`.

Flags: `0x11` bit 0 = relative, bit 1 = also switch to POSITION mode. `0x12`
and `0x13` bit 1 = also switch to the matching mode. `0x1A` bit 0 = relative
(the only form supported), bit 2 = append.

Queue actions: `0` nop, `1` clear, `2` pause, `3` resume, `4` set time scale.

### 4.3 Registers and persistence (`0x20 – 0x2F`)

| Op     | Payload                                                       |
| :----- | :------------------------------------------------------------ |
| `0x20` | Register read: `[2..3] id` → `[1] type`, `[4..7] value`        |
| `0x21` | Register write: `[2..3] id`, `[4..7] value` → value now in force|
| `0x22` | Register info: `[2..3] id` → type, access flags, group, count  |
| `0x28` | Save: `[1] target` (0 all, 1 config, 2 motor profile)          |
| `0x29` | Load: `[1] target`                                             |
| `0x2A` | Restore defaults: `[1] target`, `[4..7]` key `0x54455344`      |
| `0x2B` | R — Persistence status. State bits: 0 busy, 1 config restored at boot, 2 motor profile restored, 3 last save failed (set only once a save has actually run) |

### 4.4 Calibration and system (`0x30 – 0x3F`)

| Op     | Payload                                                        |
| :----- | :-------------------------------------------------------------- |
| `0x30` | Calibration start: `[1] kind`, `[2] flags` (bit0 persist — reserved; bit1 force) |
| `0x31` | Calibration abort — **not implemented**, see §9                  |
| `0x32` | R — Calibration result: R, Ld, Lq, validity flags                |
| `0x38` | Enable: `[1]` 0 disable / 1 enable                               |
| `0x39` | Fault reset: `[1] mask` (0 = all)                                |
| `0x3A` | Reboot: `[1]` 0 warm / 1 USB DFU, `[4..7]` key `0x544F4F42`      |
| `0x3B` | Node address: `[1] id`, `[2] flags`, `[4..7]` UID guard          |
| `0x3C` | Telemetry slot: `[1] slot`, `[2] opcode`, `[3] flags`, `[4..5] ms`|
| `0x3D` | Watchdog: `[1] action`, `[2] flags`, `[4..5] timeout ms`         |

Calibration kinds: `0` current-sense offset, `1` electrical identification,
`2` angle sweep, `3` full (**not implemented**, see §9).

`0x3B` with address byte `0` is a **read** (0 is the unassigned address and is
never a legal assignment target). A write requires the low 32 bits of the
device UID in bytes 4–7; without that guard a broadcast frame could renumber
the whole bus at once.

`0x3C` and `0x3D` use **bit 7 of the flags byte as an apply bit**. Without it
the frame reads the slot back. An all-zero body is exactly how a telemetry slot
gets switched off, so it cannot also mean "read".

### 4.5 Generic replies

| Op     | Payload                                                   |
| :----- | :--------------------------------------------------------- |
| `0x7E` | ACK: `[1]` opcode executed, `[3]` context                  |
| `0x7F` | NACK: `[1]` opcode refused, `[2]` error code, `[3]` context|

### 4.6 Error codes

| Code   | Meaning                                     |
| :----- | :------------------------------------------ |
| `0x01` | Unknown opcode                              |
| `0x02` | Frame shorter than the opcode requires      |
| `0x03` | Undefined value in a payload field          |
| `0x04` | Write on a read-only register               |
| `0x05` | Read on a write-only register               |
| `0x06` | Unknown register identifier                 |
| `0x07` | Value rejected by the register range check  |
| `0x08` | Not permitted in the current drive state    |
| `0x09` | Subsystem busy (flash write, calibration)   |
| `0x0A` | Motion queue full                           |
| `0x0B` | Reserved but not implemented in this build  |
| `0x0C` | Guard key missing, or stage must be disabled|
| `0x0D` | Refused: a latched fault is pending         |
| `0x0E` | Persistent storage failure                  |
| `0x0F` | Refused: the drive is not calibrated        |

---

## 5. Network management (`0x100 – 0x10F`)

| ID      | Payload                                                            |
| :------ | :----------------------------------------------------------------- |
| `0x100` | Discovery begin — arm the UID search on every unaddressed node      |
| `0x101` | Discovery prefix — `[0]` prefix length, `[1..7]` prefix bytes       |
| `0x102` | Discovery assign — `[0..6]` full UID, `[7]` address to assign       |
| `0x103` | Discovery end — resume normal filtering                            |
| `0x104` | Global control — `[0]` sub-command, `[4..7]` key when required     |
| `0x105` | SYNC — latch setpoints, emit SYNC-triggered telemetry              |
| `0x106` | TIME — `[0..3]` master wall clock in µs (informational)            |

Global sub-commands: `0` emergency stop, `1` disable, `2` enable, `3` fault
reset, `4` emit one heartbeat, `5` reboot (needs the reboot key).

### 5.1 Discovery

The master isolates one node at a time by walking the UID tree:

1. Broadcast `0x100`. Every node **without an address** arms its search.
2. For step *i* = 0…6, broadcast `0x101` carrying the prefix resolved so far.
   Every node whose UID starts with that prefix replies with a **data-less
   frame** on identifier `0x500 | UID[i]`.

   CAN arbitrates identifiers bit by bit and a dominant bit wins, so the frame
   that survives arbitration is automatically the one carrying the **lowest**
   byte value. The bus performs the minimum search itself, at zero software
   cost. The master records that byte and moves to step *i+1*.
3. After seven steps the master holds a complete UID. It sends `0x102` with the
   full UID plus the address to assign. The matching node adopts the address,
   reprograms its filters, persists the address, replies `ACK` on its **new**
   `0x400 + node`, and stays quiet for the rest of the sweep.
4. Repeat from step 2 to find the next device; finish with `0x103`.

A node that loses arbitration must not let the controller re-send its stale
reply into the next round, where it would answer a question that was never
asked. The reference implementation aborts any pending transmission before each
reply. Disabling automatic retransmission (DAR) on the master's link makes that
unnecessary, but the protocol does not require it — and a drive must **not**
run with DAR globally enabled, because losing arbitration is the normal case for
ordinary traffic and every such frame would then be dropped instead of retried.

The UID is 56 bits folded from the MCU's 96-bit factory identifier with an
FNV-1a mix: deterministic across reboots, and every input bit reaches every
output bit so two dies differing only in wafer coordinate land far apart.

---

## 6. Motion

### 6.1 Modes

| Mode | Name     | Behaviour                                                |
| :--- | :------- | :------------------------------------------------------- |
| `0`  | DISABLED | Output stage held in the configured safe state            |
| `1`  | TORQUE   | Direct q-axis current command, position loop bypassed     |
| `2`  | VELOCITY | Velocity command; the position target ramps at that rate  |
| `3`  | POSITION | Absolute position command                                 |
| `4`  | MOTION   | Trajectory queue executed by the motion executor          |

Mode entry is **bumpless**: every mode re-seeds the state it integrates from, so
the first pass in a new mode commands exactly what the last pass of the old one
did. Velocity commands are slewed under the configured acceleration limit, so a
step in the velocity command never becomes a torque step.

### 6.2 Motion burst frame (PVT / PVcTF)

The streaming trajectory format needs 22 + 24 + 16 = 62 payload bits plus two
format flags — the whole 64-bit frame, with nothing left over for an opcode
byte. The flags therefore live in the two most significant bits of byte 0, and
because every opcode is ≤ `0x7F`, "byte 0 has bit 7 set" unambiguously marks a
burst frame. This is the one frame laid out **MSB-first**.

| Byte  | Bits  | Field                                                       |
| :---- | :---- | :---------------------------------------------------------- |
| 0     | 7     | `burst`, always 1                                            |
| 0     | 6     | `const_t`: 0 = PVT (bytes 6–7 are a duration), 1 = PVcTF     |
| 0     | 5–0   | displacement bits 21–16                                      |
| 1–2   | 7–0   | displacement bits 15–0 — `q1.5.16`, ±32 rev, 1/65536 steps   |
| 3–5   | 7–0   | end velocity, `f24`, revolutions per second                  |
| 6–7   | 7–0   | duration in 100 µs units, or normalized force when `const_t` |

- **PVT** (`const_t = 0`): a time-parameterized Hermite segment. Duration in
  100 µs units keeps a 16-bit field useful out to 6.5 s per segment.
- **PVcTF** (`const_t = 1`): open loop until the displacement is reached, with
  the trailing field capping the torque — a move that ends on contact rather
  than on the clock.

Burst frames are accepted on both the setpoint stream and the request channel.

### 6.3 Queue behaviour

The executor queue holds 32 segments. When it runs dry the drive **holds** the
last commanded position: a starved queue reports zero segment torque, and the
drive deliberately keeps the full current envelope in that state so a loaded
axis cannot fall.

`Motion Status 3` and the heartbeat both report the remaining depth, saturating
at 15, and `TP_WARN_QUEUE_LOW` is raised with one segment left.

---

## 7. Heartbeat and telemetry

The heartbeat (opcode `0x0F`) is emitted cyclically on `0x600 + node` and,
**unchanged**, on `0x000 + node` the instant the fault byte changes. A master
needs exactly one decoder for both channels.

| Byte | Field                                                        |
| :--- | :----------------------------------------------------------- |
| 0    | `0x0F`                                                        |
| 1    | bits 0–3 drive state, bits 4–7 control mode                   |
| 2    | fault flags (1:1 with the driver's fault byte, 0 = healthy)   |
| 3    | warning flags                                                 |
| 4    | supply voltage, `volts_q2`                                    |
| 5    | temperature, `temp_c`                                         |
| 6    | load, signed; `−128` = overload                               |
| 7    | bits 0–3 sequence counter, bits 4–7 queued segments           |

> The draft's 64-bit heartbeat is replaced by this 7-byte layout because the
> heartbeat now has its own identifier and no longer needs a 4-bit opcode
> squeezed into the payload. Time scale and following error moved to
> `Motion Status 3` (`0x08`), which is where a master looks for them anyway.

**Drive states:** `0` boot, `1` calibrating, `2` idle, `3` ready, `4` running,
`5` fault, `6` config/discovery, `7` rebooting.

**Warning bits:** `0x01` not calibrated, `0x02` NVM busy, `0x04` watchdog
expired, `0x08` queue low, `0x10` segment dropped, `0x20` comm degraded,
`0x40` temperature above 80 % of the trip, `0x80` supply near a rail limit.

### Cyclic telemetry

Four independent slots, each configured with `0x3C`: which status opcode to
emit, on a millisecond timer or on every `SYNC`. Telemetry sits below the
request and response channels in priority, so it can never delay a command.

---

### Power-on behaviour

A drive does **not** energize by itself. After calibration it sits in
`DISABLED` with the configured safe state on the bridge until the master sends
`0x10` (control mode + enable) or `0x38` (enable). Set bit 0 of register
`0x0003` to make it energize itself in `POSITION` mode, holding wherever the
calibration left the rotor, as soon as the angle table is valid and no fault is
latched — for a machine that has to come back up unattended after a power cycle.

---

## 8. Safety

- **Master-liveness watchdog.** Configure with `0x3D` or register `0x0005`. Any
  frame addressed to this node reloads it. On expiry the drive enters its
  configured safe state (freewheel / brake / hold) and raises the watchdog
  warning; it stays there until a new command arrives.
- **Emergency frames.** `0x000 + node` is the lowest identifier on the bus, so
  a drive that trips announces it ahead of every command and every other node's
  telemetry.
- **Enable guards.** Energizing is refused while the drive is uncalibrated
  (`0x0F`) or has a latched fault (`0x0D`).
- **Register guards.** Registers marked `D` are writable only while the output
  stage is disabled — programming an analog protection threshold under load
  would move a trip point beneath a live current.
- **Flash guards.** Every NVM write waits for the de-energized programming
  window that the control loop publishes. Flash programming stalls the
  single-bank CPU for milliseconds with the control ISR frozen and whatever
  vector is latched still on the bridge. Save commands are refused while the
  stage is enabled, and a save that never finds a window is abandoned after 2 s
  and reported as a failed write.
- **Bus-off recovery.** A bus-off event latches `CCCR.INIT` in hardware; the
  service call clears it to start the mandated recovery sequence, so a wiring
  fault is not permanent until reboot.

---

## 9. What v1 does not implement

Stated explicitly so a master can plan around it rather than discover it:

- **`0x31` Calibration abort** → `TP_ERR_NOT_IMPLEMENTED`. Neither calibration
  state machine exposes an abort entry point; both run to their own terminal
  state. Reboot (`0x3A`) is the way to stop one.
- **`0x30` without the force flag on a known motor** does not sweep. When a motor
  profile is loaded the drive reloads its stored angle table by Cal-ID and returns
  to Ready without turning the shaft — the right default, since re-sweeping a known
  motor on every boot would move machinery for no reason. Set `[2]` bit1
  (`TP_CALIB_FLAG_FORCE`) to demand a physical sweep, which is what you want after
  the mechanics change.
- **`0x30` kind 3 (full calibration)** → `TP_ERR_NOT_IMPLEMENTED`. Starting both
  engines at once would run the HFI identification and the angle sweep
  concurrently on the same bridge. The master sequences it: start kind `1`, poll
  `0x0C` until the stage reports success, then start kind `2`.
- **`0x1A` absolute S-curve targets** → `TP_ERR_BAD_PARAM`. The queue works in
  displacements; only relative moves are expressible as a queued segment.
- **`0x40 – 0x6F` bulk transfer / firmware update** → `TP_ERR_NOT_IMPLEMENTED`.
  Identifier range `0x700 – 0x7FF` is reserved for it. USB DFU (`0x3A` mode 1)
  is the field update path in v1.
- **`0x04` Motor Status is read-only.** Its fields are writable through the
  register map (`0x0100`–`0x0103`), where each gets a range check and the
  disabled-stage guard.
- **Encoder health (`0x0A` byte 1) always reads 0.** MT6835 status bits and CRC
  are captured by the driver but not yet validated (see `suggestions.md` S8).

---

## 10. Register map

Value payloads are 32 bits; the type field of every reply says how to read them.
`P` = persisted, `D` = writable only while disabled, `B` = takes effect after a
save and a reboot.

### `0x00xx` — identity and communication

| ID       | Type | Access | Description                          |
| :------- | :--- | :----- | :----------------------------------- |
| `0x0000` | u8   | RW P   | Node address (0 = unassigned)        |
| `0x0001` | u32  | RW P B | Arbitration bit rate [bit/s]         |
| `0x0002` | u32  | RW P B | CAN-FD data-phase bit rate [bit/s]   |
| `0x0003` | u8   | RW P B | Link flags: bit 0 auto-enable, bit 1 promiscuous, bit 2 classic CAN |
| `0x0004` | u16  | RW P   | Heartbeat period [ms], 0 = off       |
| `0x0005` | u16  | RW P   | Watchdog timeout [ms], 0 = off       |
| `0x0006` | u8   | RW P   | Watchdog action (safe state)         |
| `0x0007` | u8   | RW P   | Safe state while disabled            |
| `0x0008` | u8   | R      | Hardware revision                    |
| `0x0009` | u16  | R      | Firmware version                     |
| `0x000A` | u32  | R      | Device code                          |
| `0x000B` | u32  | R      | Uptime [s]                           |
| `0x000C` | u16  | R      | Protocol version                     |
| `0x000D` | u32  | R      | Device UID, low 32 bits              |
| `0x000E` | u32  | R      | Device UID, high 24 bits             |

### `0x01xx` — motor identity

| ID       | Type | Access | Description                              |
| :------- | :--- | :----- | :--------------------------------------- |
| `0x0100` | u8   | RW D   | Motor type (4 stepper, 6 BLDC)           |
| `0x0101` | u16  | RW D   | Pole pairs                               |
| `0x0102` | u8   | RW D   | Phase order (1 ABC, 2 CAB, 3 ACB)        |
| `0x0103` | i8   | RW D   | Direction (+1 / −1)                      |
| `0x0104` | f32  | RW     | Winding resistance [Ω]                   |
| `0x0105` | f32  | RW     | d-axis inductance [H]                    |
| `0x0106` | f32  | RW     | q-axis inductance [H]                    |
| `0x0107` | f32  | RW     | Torque constant Kt [N·m/A]               |
| `0x0108` | f32  | RW     | PM flux linkage [Wb]                     |
| `0x0109` | u8   | R      | Calibration table identifier             |
| `0x010A` | i32  | R      | Angle calibration offset                 |
| `0x010B` | f32  | RW     | Motor torque rating [N·m]                |

### `0x02xx` — protection limits

| ID       | Type | Access | Description                        |
| :------- | :--- | :----- | :--------------------------------- |
| `0x0200` | f32  | RW P   | Control current envelope [A]       |
| `0x0201` | f32  | RW     | Motor voltage rating [V]           |
| `0x0202` | f32  | RW     | Motor power rating [W]             |
| `0x0203` | f32  | RW P D | Phase-A hardware overcurrent [A]   |
| `0x0204` | f32  | RW P D | Phase-B hardware overcurrent [A]   |
| `0x0205` | f32  | RW P D | Phase-C hardware overcurrent [A]   |
| `0x0206` | f32  | RW P D | Overvoltage trip [V]               |
| `0x0207` | f32  | RW P D | Undervoltage trip [V]              |
| `0x0208` | f32  | RW P   | Overtemperature trip [°C]          |
| `0x0209` | f32  | RW P   | Overtemperature hysteresis [°C]    |
| `0x020A` | f32  | RW P   | Overpower trip [W]                 |
| `0x020B` | f32  | RW P   | Bus overcurrent trip [A]           |
| `0x020C` | u8   | RW     | Motor thermal rating [°C]          |
| `0x020D` | f32  | RW D   | Identification current [mA]        |

### `0x03xx` — control loop

| ID       | Type | Access | Description                                  |
| :------- | :--- | :----- | :------------------------------------------- |
| `0x0300` | u8   | RW     | Control mode                                 |
| `0x0301` | u8   | RW     | Output stage enable                          |
| `0x0302` | f32  | RW     | Spring saturation width [rev]                |
| `0x0303` | f32  | RW     | Velocity damper gain                         |
| `0x0304` | f32  | RW     | Damper softening velocity [RPS]              |
| `0x0305` | f32  | RW     | Observer bandwidth at standstill [Hz]        |
| `0x0306` | f32  | RW     | Observer bandwidth at speed [Hz]             |
| `0x0307` | f32  | RW     | Observer bandwidth slope [Hz per rev/s]      |
| `0x0308` | f32  | RW     | Observer bandwidth slew rate [Hz/s]          |
| `0x0309` | f32  | RW     | Observer acceleration clamp [rev/s²]         |

### `0x04xx` — motion envelope

| ID       | Type | Access | Description                              |
| :------- | :--- | :----- | :--------------------------------------- |
| `0x0400` | f32  | RW P   | Velocity envelope [RPS]                  |
| `0x0401` | f32  | RW P   | Acceleration envelope [RPS²]             |
| `0x0402` | f32  | RW P   | Jerk envelope [RPS³]                     |
| `0x0403` | i16  | RW     | Trajectory playback rate, Q10            |
| `0x0404` | u8   | R      | Queued segments                          |
| `0x0405` | q16  | RW     | Position target [rev]                    |
| `0x0406` | f32  | RW     | Velocity target [RPS]                    |
| `0x0407` | f32  | RW     | q-axis current target [A]                |

### `0x05xx` — live telemetry (read only)

| ID       | Type | Description                          |
| :------- | :--- | :----------------------------------- |
| `0x0500` | q16  | Measured position [rev]              |
| `0x0501` | f32  | Observed velocity [RPS]              |
| `0x0502` | f32  | Current commanded to the FOC loop [A]|
| `0x0503` | f32  | DC bus voltage [V]                   |
| `0x0504` | f32  | Motor power [W]                      |
| `0x0505` | f32  | Driver temperature [°C]              |
| `0x0506` | u8   | Fault flags                          |
| `0x0507` | u32  | Raw encoder angle                    |
| `0x0508` | i32  | Accumulated revolutions              |
| `0x0509` | u32  | Completed control passes             |
| `0x050A` | f32  | DC bus current [A]                   |

> Registers routed through the control mailbox (targets, mode, enable) take
> effect on the next control pass, so a write reply can still show the previous
> value. Read the register again to confirm one of those.

---

## 11. Persistence

Node configuration lives in its **own** 2 KiB flash partition
(`NVM_CONFIG_SECTION_SIZE`), separate from the calibration table and the motor
profile. The motor-profile engine erases its whole partition when it runs out of
slots (roughly 60 saves); a node address stored there would be destroyed by an
unrelated profile save, and a drive that forgets its bus address is a field
failure.

Records are 64 bytes (two flash blocks) with a magic word, a layout version and
a CRC-32. They are appended, so a save never has to erase; the newest valid
record wins the boot scan. Reads are memory-mapped and never stall the CPU.

`0x2A` (restore defaults) has two strengths, both guarded by the key and both
refused while the output stage is enabled:

- target `config` (1) resets the **live shadow only** and keeps the node
  address. Reversible until saved, so a bench never loses contact mid-session.
- target `all` (0) additionally **erases the stored record**, so the node
  really does come back on compiled-in defaults after a reboot. This surrenders
  the node address — the drive returns unaddressed and must be found by a
  discovery sweep again. That is what a factory reset means, and why it needs
  the key.

---

## 12. Worked examples

Bring a drive up on a fresh bus and jog it:

```text
# 1. Discovery (drive has no address)
→ 0x100  00 00 00 00 00 00 00 00        begin
→ 0x101  00 00 00 00 00 00 00 00        prefix length 0
← 0x5A3                                 (arbitrated: UID[0] = 0xA3)
→ 0x101  01 A3 00 00 00 00 00 00        prefix = A3
← 0x517                                 (UID[1] = 0x17)
  ... five more rounds ...
→ 0x102  A3 17 .. .. .. .. .. 07        assign UID -> node 7
← 0x407  7E 3B 00 07 00 00 00 00        ACK, now addressed as node 7
→ 0x103  00 00 00 00 00 00 00 00        end

# 2. Configure: 1 A envelope, 200 ms watchdog, 20 Hz position telemetry
→ 0x307  21 00 00 02 00 00 80 3F        REG_WRITE 0x0200 = 1.0 A
← 0x407  21 06 00 02 00 00 80 3F
→ 0x307  3D 00 81 00 C8 00 00 00        watchdog: apply, enabled, 200 ms
← 0x407  3D 00 01 00 C8 00 00 00
→ 0x307  3C 00 06 81 32 00 00 00        slot 0: emit 0x06 every 50 ms
← 0x407  3C 00 06 01 32 00 00 00
← 0x607  06 03 .. .. .. .. .. ..        telemetry starts flowing

# 3. Enable in position mode and move one revolution
→ 0x307  10 03 01 00 00 00 00 00        mode = POSITION, enable
← 0x407  7E 10 00 00 00 00 00 00
→ 0x207  11 01 00 00 01 00 00 00        +1.0 rev, relative (setpoint stream)

# 4. Stream a trajectory instead (MOTION mode, PVT segments)
→ 0x307  10 04 01 00 00 00 00 00        mode = MOTION
→ 0x207  80 40 00 41 20 00 03 E8        +0.25 rev, 10 RPS, 100 ms
→ 0x207  80 40 00 41 20 00 03 E8        ... and the next segment

# 5. Park and persist
→ 0x307  15 00 00 00 00 00 00 00        stop, hold
→ 0x307  38 00 00 00 00 00 00 00        disable (opens the flash window)
→ 0x307  28 01 00 00 00 00 00 00        save the node configuration
← 0x407  7E 28 00 00 00 00 00 00
```

---

## 13. Implementation map

Each umbrella header aggregates a subfolder; include the umbrella, not the parts.

| File                                      | Contents                                       |
| :---------------------------------------- | :--------------------------------------------- |
| `src/api/tp_wire.h`                       | Umbrella: everything that appears on the bus    |
| `src/api/wire/tp_ids.h`                   | Protocol identity, identifier map, predicates   |
| `src/api/wire/tp_opcodes.h`               | The opcode space and its guard keys             |
| `src/api/wire/tp_enums.h`                 | Error, state, warning and selector enums        |
| `src/api/wire/tp_codecs.h`                | Fixed-point and mini-float codecs               |
| `src/api/tp_chunks.h`                     | Umbrella + `union TpFrame`                      |
| `src/api/chunks/tp_chunk_status.h`        | `0x01–0x0F` status and telemetry layouts        |
| `src/api/chunks/tp_chunk_motion.h`        | `0x10–0x1F` control, targets, trajectory        |
| `src/api/chunks/tp_chunk_config.h`        | `0x20–0x2F` register access and persistence     |
| `src/api/chunks/tp_chunk_system.h`        | `0x30–0x3F` calibration, system, replies        |
| `src/api/tp_registers.h`                  | Umbrella: the register layer                    |
| `src/api/registers/tp_register_desc.h`    | Descriptor model, context, value helpers        |
| `src/api/registers/tp_register_ids.h`     | The `TpRegId` identifier space                  |
| `src/api/registers/tp_register_table.h`   | `TP_REGISTERS` + lookup + normalization         |
| `src/api/registers/tp_config_apply.h`     | `tp_apply_config`: shadow → hardware            |
| `src/api/tp_node.h`                       | Umbrella: the node layer                        |
| `src/api/node/tp_node_config.h`           | Persisted record and build-time defaults        |
| `src/api/node/tp_node_identity.h`         | The 56-bit UID folded from the die id           |
| `src/api/node/tp_node_store.h`            | Flash-backed load / save engine                 |
| `src/api/node/tp_discovery.h`             | Byte-by-byte UID search participant             |
| `src/api/tp_engine.h`                     | `TpProtocol` declaration and state              |
| `src/api/engine/*_impl.h`                 | Its method bodies, nine files by concern        |
| `src/api/transport/tp_can.h`              | FDCAN1 binding, transport mux and bring-up      |
| `src/api/transport/tp_serial.h`           | Framed transport over the USB CDC console       |
| `src/api/transport/tp_hid.h`              | Report transport over the vendor HID endpoints  |
| `src/target/cln17_v3/usb_hid_setup.h`     | Composite USB device: CDC console + HID pipe    |
| `src/lib/motor_control/drive_control.h`   | Mode machine, command mailbox, trajectory       |
| `src/target/cln17_v3/can_setup.h`         | FDCAN1 driver, filtering, bus-off recovery      |
| `test/test_tpcp/test_wire.cpp`            | Host-side wire-format conformance tests         |

A host tool needs only `tp_wire.h` and `tp_chunks.h`; both are free of any
firmware dependency, and nothing either of them reaches may include `src/lib/` —
that invariant is what keeps the host suite compiling. Run it with
`pio test -e native_tpcp`.

### Transports

The engine names no peripheral: it moves frames through four function pointers,
so the same protocol runs over anything that can carry an identifier and eight
bytes. Three bindings ship:

- **CAN** (`transport/tp_can.h`) — the deployment transport.
- **HID** (`transport/tp_hid.h`) — the control transport, over the vendor HID interrupt
  endpoints of the composite USB device. Each 64-byte report carries a count
  byte and up to six ten-byte records:

  | Offset | Field                                                  |
  | :----- | :----------------------------------------------------- |
  | `0`    | record count, `0..6`                                   |
  | `1+`   | per record: `id_lo`, `id_hi(2:0) \| len(7:4)`, `d0..d7` |

- **Serial** (`transport/tp_serial.h`) — the same protocol over the board's USB CDC
  console, framed as `A5 5A | id_lo id_hi | d0..d7 | xor`. Because the sync
  byte has bit 7 set and the log output is ASCII, protocol frames and console
  text share one endpoint with no mode switch.

They are multiplexed: a **response** goes back to whichever transport delivered
the request, while unsolicited frames (telemetry, emergencies) go to all of
them. Discovery works on all three. On CAN the reply is a data-less frame whose
identifier is the answer, resolved between competing nodes by bit-level
arbitration; a point-to-point USB link has nobody to arbitrate against, so the
single reply is the answer. Routing it this way is what makes an unaddressed
node reachable on a bench at all.

HID is the transport to use for control. A report is a message, not a byte
stream, so it arrives whole or not at all — there is no framing, no checksum and
no resynchronization — and the host is paced by the endpoint's own flow control
rather than by a guess. The CDC transport has neither property: the host cannot
see how much room the board has, so `tools/tpcp_host.py` streams trajectory
bursts over it four frames at a time with a sleep between batches, because any
faster overruns the receive buffer silently and any slower starves the queue
mid-move. Over HID the same burst goes out at full speed, six frames per 1 ms
transfer.

Both USB transports cover everything above the physical layer: dispatch,
registers, telemetry, motion, calibration, persistence, the watchdog, and the
discovery handshake. Neither exercises bit timing, arbitration between real
competing nodes, hardware filtering or bus-off recovery — those need a bus.

### Threading model

`TpProtocol::poll()` runs in thread mode from `loop()`. Nothing in the protocol
touches control state directly: every command that changes what the motor does
goes through the `DriveControl` mailbox, a lock-free single-producer queue the
20 kHz control ISR drains between passes. Telemetry flows back through a
sequence lock written by the ISR. That is what makes it safe for a protocol
handler to be preempted at any instruction by the control loop.
