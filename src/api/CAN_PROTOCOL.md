# TunePulse CAN Protocol (TPCP) v1.0

Application protocol for TunePulse drives on CAN 2.0A and CAN-FD.

Reference implementation: `src/api/` (header-only, hardware-independent except
for `transport/`). The wire format lives entirely in `tp_wire.h` + `tp_chunks.h`;
a host tool can include those two files and be bit-compatible with the drive
without pulling in anything else. `tools/tpcp_host.py` is the reference master.

This document is the `api_final.md` design with every gap it left filled in.
Each such decision is marked **[decision]** so it can be revisited.

---

## 1. Design rules

1. **Every frame is exactly 8 bytes.** That is the whole payload of a classic
   CAN 2.0A frame, so the protocol runs unchanged on a classic bus. On a CAN-FD
   link the same 8-byte frames are sent as FD frames.
2. **Byte 0 is the CMD**: a register address with two group bits folded in
   (§3). The read form and the write form of a register differ only in bit 7.
3. **Multi-byte integers are little-endian**, packed with no padding. Sub-byte
   fields are allocated from the LSB upward. The single documented exception is
   the burst frame (§8), which is a bit field laid out MSB-first.
4. **Every write of a read/write register is answered with the read form of the
   same register**, carrying the values now in force - so a clamped, rounded or
   ignored field is visible in the acknowledgement itself. Write-only commands
   are answered with ACK / NACK (§6.3).
5. **A node without an address is silent.** It accepts the two discovery
   registers only and transmits nothing else until an address is assigned.

---

## 2. Identifier map

11-bit standard identifiers only. Lower identifier = higher bus priority. The
map is ordered so a tripping drive outranks everything, commands outrank
responses, and status telemetry can never delay a command.

| Range          | Direction      | Purpose                                      |
| :------------- | :------------- | :------------------------------------------- |
| `0x000 – 0x0FF`| —              | Reserved                                     |
| `0x100 + node` | node → bus     | Emergency / fault report (highest priority)  |
| `0x200 – 0x2FF`| —              | Reserved                                     |
| `0x300 + node` | master → node  | Commands: registers, setpoints, flashing     |
| `0x400 + node` | node → master  | Responses, discovery replies                 |
| `0x500 + node` | node → bus     | PulseBeat and cyclic telemetry               |
| `0x600 – 0x7FF`| —              | Reserved                                     |

**Node addresses:** `0` = broadcast, `1–254` = assigned, `255` = unassigned.
`0x300` therefore reaches every node at once, and an unassigned node listens on
`0x3FF` and answers on `0x4FF`. **[decision]** The design's "0x2FF and 0x3FF
address every node" line was a leftover of an earlier map; with 0 as the
broadcast address it is `0x300`.

### Hardware filtering

A node programs four FDCAN standard filter slots:

| Slot | Type | Accepts                     | Active when                    |
| :--- | :--- | :-------------------------- | :----------------------------- |
| 0    | dual | `0x300 + node`, `0x300`     | always (unassigned: `0x3FF`)   |
| 1    | mask | `0x1xx`                     | promiscuous / gateway armed    |
| 2    | mask | `0x4xx`                     | promiscuous / gateway armed    |
| 3    | mask | `0x5xx`                     | promiscuous / gateway armed    |

Slots 1–3 are the design's "master device" set: a node opens them only when it
acts as a master itself, i.e. when it relays the bus to a USB host (§11).
Everything else is rejected in hardware: traffic addressed to other drives
costs this node no interrupts at all. Non-matching standard frames, extended
frames and remote frames are all rejected globally.

---

## 3. CMD byte and groups

| Bit 7 | Bit 6 | Bit 5 | Range         | Group                            | Qty |
| :---: | :---: | :---: | :------------ | :------------------------------- | :-: |
|   0   |   0   |   x   | `0x00 – 0x3F` | Read of a read/write register    | 64  |
|   1   |   0   |   x   | `0x80 – 0xBF` | Write of a read/write register   | 64  |
|   0   |   1   |   x   | `0x40 – 0x7F` | Read-only register               | 64  |
|   1   |   1   |   0   | `0xC0 – 0xDF` | Write-only register (command)    | 32  |
|   1   |   1   |   1   | `0xE0 – 0xFF` | Burst mode (bit-packed stream)   | 1   |

A read is a frame whose CMD is the register address; the payload is ignored
except for the record selector of a keyed register (a sub identifier in byte 1,
the IO function in byte 2, the table sample or the memory address). A write
sets bit 7. Reads are answered even when broadcast (every node answers on its
own `0x400 + node`); broadcast writes are executed by every node and answered
by none.

**Sub-spaces.** Registers `0x01`, `0x02`, `0x42` and `0x43` carry a sub
identifier in byte 1 that selects one of several records behind one CMD.

### Response policy (Bus Settings, `Comm` field)

| Comm | Name           | Behaviour                                                              |
| :--- | :------------- | :--------------------------------------------------------------------- |
| 0    | silent         | Executes everything, transmits nothing (no beat, no emergency either)   |
| 1    | on request     | Reads and register writes answered; accepted setpoints (motion tasks, burst) silent, rejected ones NACKed |
| 2    | every message  | Everything answered, ACK per motion task and burst frame               |

The default is 1.

---

## 4. Data types

| Name        | Bits | Encoding                                                        |
| :---------- | :--- | :-------------------------------------------------------------- |
| `bf16`      | 16   | Top 16 bits of an IEEE-754 float. Full range, ~0.4 % error. NaN (`0x7FC0`) = "unknown" on reads, "leave unchanged" on writes |
| `f24`       | 24   | Top 24 bits of an IEEE-754 float. Full range, ~0.008 % error. NaN = unused |
| `q1.23.24`  | 48   | Signed revolutions, 24 fractional bits, ±8388608 rev             |
| `q1.23.32`  | 56   | Signed revolutions, 32 fractional bits (raw encoder word + turns) |
| `q1.15.32`  | 48   | Signed revolutions, 32 fractional bits, ±32768 rev               |
| `q1.15.16`  | 32   | Signed revolutions, 16 fractional bits (the firmware's native unit) |
| `q1.7.16`   | 24   | Signed revolutions, 16 fractional bits, ±128 rev                 |
| `q1.3.8`    | 12   | Signed time scale, 256 = 1.0×, range −8.0 … +7.996               |
| `q1.1.4`    | 6    | Signed normalized error, 16 = 1.0, range −2.0 … +1.94            |
| `temp_u8`   | 8    | Degrees Celsius; `0` = no data, `255` = over range               |
| `usage_i8`  | 8    | Signed fraction of a limit, 127 = 100 %; `−128` = overload       |

`f24` and `bf16` are **truncations** of an IEEE-754 float, not fixed-point
integers: decoding is a pure left shift, with no sign extension.

**Torque units.** **[decision]** Every torque field is in N·m when the torque
constant Kt (register `0x22`) is known and in **amperes** when it is zero. One
rule for the whole protocol, so a bench without a Kt drives the current
directly and nothing silently changes scale when Kt is entered.

**Axis frame.** Every position, velocity and displacement leaves and enters
through the axis offset and direction of register `0x01/0x00`: reported
position = (native + offset) × direction; a commanded absolute target is
converted back. Displacements and velocities take the sign only.

---

## 5. Read/write registers (`0x00 – 0x3F`, write at `+0x80`)

### 5.1 `0x00` Direct register access

| Byte | Field   | Type   | Description                                             |
| :--- | :------ | :----- | :------------------------------------------------------ |
| 1–3  | Address | u24    | Window nibble in bits 23–20, byte offset in bits 19–0   |
| 4–7  | Value   | u32    | Value read, or the value to write                       |

**[decision]** A 24-bit address cannot reach a 32-bit address space, so the
top nibble selects a window: `0` flash `0x08000000` (read only), `1` SRAM
`0x20000000`, `2` CCM `0x10000000`, `3` APB1 `0x40000000`, `4` APB2
`0x40010000`, `5` AHB1 `0x40020000`, `6` AHB2 `0x48000000`, `7` AHB2/ADC
`0x50000000`, `8` the firmware's `TunePulseHeart` object, `F` Cortex-M system
space `0xE0000000`. Addresses must be word aligned. This is a developer port:
an unmapped peripheral offset faults the MCU.

### 5.2 `0x01` Axis and mechanics (sub-space)

**[decision]** The design lists these records under `0x01??`; the sub
identifiers are allocated in the order given.

| Sub    | Record                     | Bytes 2–7                                                                    |
| :----- | :------------------------- | :--------------------------------------------------------------------------- |
| `0x00` | Axis Control               | `[2]` AxisID, `[3]` bit0 inverted, `[4..7]` offset q1.15.16                   |
| `0x01` | LED Control                | `[2]` mode (0 steady, 1 blink, 2 firmware), `[3]` on:4 / off:4 [0.25 s], `[4]` LedID, `[5..7]` R G B |
| `0x02` | Gearbox                    | `[2..3]` backlash bf16, `[4..5]` ratio mul i16, `[6..7]` ratio div u16        |
| `0x03` | IO Control Map (keyed)     | `[2]` function, `[3]` on pin, `[4]` off pin, `[5]` bit0 on inverted, bit1 off inverted, bit2 state |
| `0x04` | Encoder 1                  | `[2]` type:4 (0 absolute, 1 incremental) / interface:4, `[3]` bit0 CCW, bit1 index, `[4..7]` PPR u32 |
| `0x05` | Encoder 2                  | same layout                                                                  |
| `0x06` | Quick homing               | `[2..3]` flags: bits 0–1 mode, bit2 trigger, bit3 direction, bit4 bidir, bit5 validation, **bit6 fine-search trigger**; `[4..5]` velocity bf16, `[6..7]` torque bf16 |
| `0x07` | Fine homing                | `[2..3]` retract bf16, `[4..5]` velocity bf16, `[6..7]` torque bf16           |
| `0x08` | CW reference limit         | `[2..3]` offset bf16 (< 64 rev), `[4..7]` reference angle u32                 |
| `0x09` | CCW reference limit        | same layout                                                                  |
| `0x0A` | Position zero              | `[4..7]` reference angle u32                                                 |
| `0x0B` | Zero offset CW             | `[2..7]` q1.15.32                                                            |
| `0x0C` | Zero offset CCW            | `[2..7]` q1.15.32                                                            |
| `0x0D` | Motion field CW            | `[2..7]` q1.15.32                                                            |
| `0x0E` | Motion field CCW           | `[2..7]` q1.15.32                                                            |
| `0x0F` | Internal position offset   | `[2..7]` q1.15.32                                                            |

The IO map is keyed: a read names the device function in byte 2 and gets that
function's record. Functions: `0x00` StatusIO, `0x01` HardStop, `0x02` Home,
`0x03` LimCW, `0x04` WarnCW, `0x05` LimCCW, `0x06` WarnCCW, `0x07`
GearboxIDX, `0x08` Brake **[decision: the design left Brake unnumbered]**,
`0x10–0x17` Probe1–8.

**[decision]** The design's fine-homing record is 75 bits and does not fit a
frame; its trigger bit moved into bit 6 of the quick-homing flags word.

All of these records are stored and reported. The firmware applies Axis
Control (to every position and velocity on the wire) and LED Control; the
gearbox, IO map, encoder declarations, homing settings and reference limits
are held for the features that will consume them.

### 5.3 `0x02` Telemetry slots (sub = slot 0–3)

**[decision]** The design names the cyclic telemetry channel but no way to
configure it.

| Byte | Field   | Description                                                   |
| :--- | :------ | :------------------------------------------------------------ |
| 2    | Register| Read register to emit (`0x00–0x7F`); 0 frees the slot          |
| 3    | Flags   | bit0 enable, bit1 emit on bus time sync instead of on a timer  |
| 4–5  | Period  | Emission period [ms]                                          |

A sub-space register named in a slot emits its record 0.

### 5.4 `0x10` Bus settings

| Byte | Field    | Description                                                                  |
| :--- | :------- | :--------------------------------------------------------------------------- |
| 1    | Type/Comm| bits 0–3 bus type (0 classic, 1 FD, 2 FD without BRS), bits 4–7 comm mode (§3) |
| 2    | Baud1    | Arbitration rate code (0 keep, 1 125k, 2 250k, 3 500k, 4 1M)                 |
| 3    | Baud2    | Data-phase rate code (0 keep, 5 2M, 6 4M, 7 5M, 8 8M; 1–4 accepted too)      |
| 4–7  | Sync     | Master time [50 µs]; a non-zero write re-bases the node clock and fires sync-triggered telemetry |

Bus type and bit rates take effect at the next boot (save them first); the comm
mode takes effect at once. A read returns the node clock in the Sync field.

### 5.5 `0x11` Discovery assignment

| Byte | Field     | Description                          |
| :--- | :-------- | :----------------------------------- |
| 1    | Addr      | Node address                         |
| 2–7  | HashUID48 | 48-bit device identity, little-endian|

- **Write:** `HashUID48 = this node` → adopt `Addr` (1–254 assign, 255 return
  to discovery); persisted at once. `HashUID48 = 0` and `Addr = 255` → reset
  to discovery: broadcast resets the whole bus, directed resets every node that
  answers to that address (the collision case). A wildcard with any other
  address is ignored. A directed assignment is confirmed with the read form on
  the **new** address.
- **Read:** `HashUID48 = this node` or `0` → the node answers with its address
  and identity. Broadcast with a specific HashUID pings one node; broadcast
  with 0 inventories every addressed node; sent to `0x3FF` with 0 it is the
  one-node fast path of a sweep.

### 5.6 `0x12` Discovery search

| Byte | Field     | Description                                    |
| :--- | :-------- | :--------------------------------------------- |
| 1    | Depth     | Number of prefix bits that must match (0–48)   |
| 2–7  | HashUID48 | Prefix bits, little-endian, bit 0 first        |

Every unassigned node whose HashUID starts with the first `Depth` bits of the
prefix answers with `Depth + 1` and **the prefix it matched** (the same
`Depth` bits, the rest zero). An assigned node ignores broadcast searches but
answers a search sent to its own address, which is how a master checks two
drives were not given the same number.

**[decision]** The design has the reply carry the first `Depth + 1` bits of
each node's own HashUID. Two nodes disagreeing on that extra bit collide in
the data field, which CAN does not arbitrate - both frames are destroyed and
the master cannot tell "no node here" from "two nodes here". Echoing the
matched prefix makes every reply in a subtree bit-identical, so CAN merges
them and a reply means exactly "this subtree is populated". The master then
walks the tree: probe `(d+1, P|0)` and `(d+1, P|1)`; a probe at depth 48 is
answered by one node with its full HashUID. At most 96 probes per node, a
few hundred milliseconds on a 1 Mbit/s bus.

### 5.7 `0x20` System control

| Byte | Field       | Write                                                       | Read                          |
| :--- | :---------- | :---------------------------------------------------------- | :---------------------------- |
| 1    | Enable      | `0xE1` enable, `0x0F` disable, 0 no change                  | `0xE1` / `0x0F`               |
| 2    | Burst step  | Burst period [250 µs]; 0 = time-tagged PVT stream (always applied) | current value          |
| 3    | Calibration | `TP_CAL_*` bits to start (§5.7.1)                           | bits whose result is valid    |
| 4    | Action      | `0xC1` fault reset, `0xB0` reboot, `0xDF` reboot to DFU, 0 none | bits of routines running  |
| 5–7  | —           | reserved                                                    | 0                             |

**[decision]** The design's bytes 4–7 were "TBD"; fault reset and reboot had
no home elsewhere, and the magic values guard against a stray write the same
way the enable byte does. A reboot is executed 20 ms after the reply leaves.
The fault reset runs before the enable, so one frame can clear and re-arm.

#### 5.7.1 Calibration bits

| Bit    | Name  | Routine                                                             |
| :----- | :---- | :------------------------------------------------------------------ |
| `0x01` | P_CAL | Encoder angle sweep (always a physical sweep, the stored table is ignored) |
| `0x02` | I_CAL | Current-sense offset calibration                                     |
| `0x04` | E_CAL | Phase connection and motor check ┐                                   |
| `0x08` | R_CAL | Resistance identification         ├ one routine serves all three     |
| `0x10` | L_CAL | Inductance identification        ┘                                   |
| `0x20` | F_CAL | Flux calibration - **not implemented** (NACK)                        |
| `0x40` | M_CAL | Mechanical calibration - **not implemented** (NACK)                  |

Requesting an electrical bit together with P_CAL runs the identification
first and starts the sweep when it succeeds. Every request first drops the
queued trajectory, selects DISABLED and releases the stage: calibration owns
the bridge. Progress is read from `0x43/0x01`.

### 5.8 `0x21` Motor properties 1

| Byte | Field                | Description                                                           |
| :--- | :------------------- | :-------------------------------------------------------------------- |
| 1    | Mode / Type          | bits 0–3 control mode (§7.1), bits 4–7 motor type (0 unchanged, 1 stepper, 2 BLDC, 3 error) |
| 2–3  | Conn / Dir / Poles   | bits 0–1 phase connection (0 unchanged, 1 ABC, 2 CAB, 3 ACB), bits 2–3 direction (2-bit signed: `01` +1, `11` −1, else unchanged), bits 4–15 pole pairs (0 unchanged) |
| 4–5  | Max voltage          | bf16 [V]                                                              |
| 6–7  | Max current          | bf16 [A]: continuous rating, this is the control current envelope (never above the peak of `0x27`) |

A machine identity field that differs from the current value is refused while
the stage is enabled or an identification runs; a field equal to the current
value is not a change, so a read-modify-write that only touches the mode is
accepted on a live drive. The mode goes through the control mailbox.

### 5.9 `0x22` Motor properties 2 (writable only while disabled)

`[1]` temperature rating u8 [°C], `[2..3]` resistance bf16 [Ω], `[4..5]`
inductance bf16 [H] (sets Ld = Lq), `[6..7]` Kt bf16 [N·m/A]. An R or L that
differs from the reported value re-tunes the current loop at once and is
refused during an identification or an angle sweep; an unchanged value is
ignored, so a read-modify-write of the other fields never rebuilds the loop.

### 5.10 `0x23` Power limits and overdrive

`[1]` overdrive duration u8 [s] (0 disables), `[2..3]` power baseline bf16 [W],
`[4..5]` peak power bf16 [W] (the overpower trip), `[6..7]` overpower capacity
bf16 [J]. Zero, NaN and infinity leave a field unchanged. Only the peak is
enforced by the firmware today; the baseline, capacity and duration are
stored for the I²t budget.

### 5.11 `0x24` Motor thermal protection

`[1]` motor temperature (read only, `temp_u8`), `[2]` baseline u8 [°C],
`[3]` limit u8 [°C], `[4..5]` derating bf16 [%/°C], `[6..7]` Rth bf16 [°C/W].
Without a winding sensor the temperature is **estimated** as driver
temperature + I²R × Rth once Rth is set; until then it reads "no data".

### 5.12 `0x25` Driver thermal protection

`[1]` driver temperature (read only), `[2]` OT warning [°C], `[3]` OT fault
[°C], `[4]` hysteresis [°C]. Zero leaves a field unchanged.

### 5.13 `0x26` Supply voltage protection (disabled only)

`[1..2]` hysteresis bf16 [V], `[3]` recovery [100 µs] (0 ignored), `[4..5]` OV
bf16 [V], `[6..7]` UV bf16 [V]. The hysteresis also sets the width of the
supply warning band.

### 5.14 `0x27` Driver current protection (disabled only)

`[1]` allowed disbalance (0–255 = 0–100 %), `[2]` disbalance filter alpha, `[3]`
recovery [100 µs], `[4..5]` peak current bf16 [A] (ceiling of the envelope),
`[6..7]` OC trip bf16 [A]. **[decision]** The OC trip is programmed on phases
A and B; phase C, which carries their sum on a stepper, keeps the board's
1.55 ratio above it. Disbalance and recovery are stored, not yet enforced.

### 5.15 `0x2A` System motion limits, `0x2B` Error motion limits

`[1..3]` velocity f24 [RPS], `[4..5]` acceleration bf16 [RPS²], `[6..7]` jerk
bf16 [RPS³]. `0x2A` is the live envelope every profile is shaped by; `0x2B`
is stored for fault handling.

### 5.16 `0x2D` Watchdog and PulseBeat

| Byte | Field           | Description                                                           |
| :--- | :-------------- | :-------------------------------------------------------------------- |
| 1    | Retry           | bit7 force retry, bits 0–6 failed beats before a FaultBeat (0 none, 127 unchanged) |
| 2–3  | PulseBeat period| bf16 [s]; 0 off, NaN unchanged                                        |
| 4–5  | Delay           | bf16 [s] from a bus time sync to the next beat                        |
| 6–7  | Watchdog timeout| bf16 [s] since the last command before the safe state; 0 off, NaN unchanged |

### 5.17 `0x2F` Calibration table (read only)

`[1]` bits 0–1 mode, bit2 absolute, bits 3–4 resolution, bits 5–7 table (0
encoder; others not present → NACK); `[2..3]` sample; `[4..7]` data. A read
with data `0xCA1B513E` returns the table size in the sample field. Writing the
table is not implemented.

---

## 6. Read-only registers (`0x40 – 0x7F`)

### 6.1 `0x40` Batch read, `0x41` Range read

`0x40`: bytes 1–7 list up to seven read registers; `0x40` itself is the
filler. A trailing filler is ignored; a single filler **between** two registers
spans them: `[01 40 08 40 40 40 40]` reads `0x01` through `0x08`. The node
answers one frame per register, paced at a few frames per service pass so a
long list cannot flood the transmit queue. A sub-space register in a batch
emits every record.

`0x41`: `[1]` register, `[2..4]` first record, `[5..7]` last record (u24,
inclusive). Walks sub identifiers for a sub-space, addresses in 4-byte steps
for `0x00`, samples for `0x2F`; a plain register is emitted once. Bounded to
4096 replies.

### 6.2 `0x42` System currents (sub-space)

`[2..3]` supply current bf16 [A], then `0x42/0x00`: `[4..5]` Iq, `[6..7]` Id;
`0x42/0x01`: `[4..5]` Ia (alpha / coil A), `[6..7]` Ib (beta / coil B).

### 6.3 `0x43` Diagnostics (sub-space) **[decision]**

The design reserves `0x42–0x4F` for sub-spaces; this one holds what a bench
needs and the design did not list.

| Sub    | Record             | Bytes 2–7                                                                          |
| :----- | :----------------- | :--------------------------------------------------------------------------------- |
| `0x00` | Communication      | `[2]` bus state (bit0 error-passive, bit1 bus-off, bit2 warning, bit3 gateway armed), `[3]` TEC, `[4]` REC, `[5]` dropped, `[6..7]` NACKs sent |
| `0x01` | Calibration        | `[2]` running bits, `[3]` stage, `[4]` progress 0–255, `[5]` flags (bit0 table valid, bit1 settling, bit2 persisting, bit3 failed, bit4 identified, bit5 profile loaded), `[6]` cal id, `[7]` pole pairs |
| `0x02` | Queue              | `[2]` state (bit0 enabled, bit1 paused, bit2 halted, bit3 starving), `[3]` queued, `[4]` free, `[5..6]` time scale i16 Q10, `[7]` dropped |
| `0x03` | Persistence        | `[2]` state (bit0 busy, bit1 config restored, bit2 profile restored, bit3 last save failed), `[3]` record version, `[4]` cal id, `[5..6]` free bytes |
| `0x04` | Encoder            | `[2..3]` raw single-turn angle (top 16 bits), `[4..7]` turns i32                    |
| `0x05` | Calibration result | `[2..3]` R bf16, `[4..5]` Ld bf16, `[6..7]` Lq bf16                                 |
| `0x06` | FOC frame          | `[2..3]` electrical angle the current loop uses (65536 per period), `[4..5]` u_d bf16 [V], `[6..7]` u_q bf16 [V] |
| `0x08` | Identification failure | `[2]` reason (0 none, 1 pattern not recognised, 2 underpower, 3 overpower in rotation, 4 driver fault, 5 HFI inductance missing or inconsistent between the probe frequencies, 255 unclassified), `[3]` stage at abort (MotorStatus), `[4]` driver fault byte at abort, `[5]` test voltage 0.1 V, `[6]` Ia 0.05 A i8, `[7]` Ib 0.05 A i8 |
| `0x07` | Encoder link       | `[2]` flags (bit0 integrity check proven and failing frames dropped, bit1 link down = encoder fault, bit2 sensor overspeed, bit3 weak magnetic field, bit4 sensor undervoltage), `[3]` bus resets since boot, `[4..5]` frames failed u16, `[6..7]` frames passed u16 (wraps). Filled by the board's link supervisor (frame validation is board code); a board without one answers NOT_IMPLEMENTED |

Stage and progress follow whichever routine owns the bridge. While the
electrical identification runs, `stage` is its MotorStatus value and progress
counts its stages. Otherwise `stage` is the angle sweep's CalStage with bit 7
set (`0x80 | CalStage`: 1 set-up, 2 direction test, 3 forward pass, 4 reverse
pass, 5 check, 6 ready, 7 error, 8 reading the stored table, 9 storing) and
progress runs 32→128 through the forward pass and 128→224 through the reverse
pass, so a bar moves with the rotor. The flag bits are valid throughout.

### 6.3a `0x44` Black box (development builds) **[decision]**

The drive keeps the last N control passes (`TP_TRACE_DEPTH`, 512 in
`cln17_v3_dev`; a release image answers NOT_IMPLEMENTED) and freezes the
ring N/8 passes after the fault byte becomes non-zero, so the passes that
led to a trip can be read afterwards. A record is 24 bytes, read in parts:
`[1]` part, `[2..3]` record index (0 = oldest), `[4..7]` two u16 fields.

| Part | `[4..5]`                    | `[6..7]`                         |
| :--- | :-------------------------- | :------------------------------- |
| 0    | valid records               | bit0 frozen, bits 8–15 fault at the freeze; `[2..3]` = depth |
| 1    | electrical angle (65536/period) | measured Iq [mA] i16          |
| 2    | measured Id [mA] i16        | commanded Uq [10 mV] i16         |
| 3    | commanded Ud [10 mV] i16    | commanded Iq [mA] i16 (FOC frame)|
| 4    | coil A current [mA] i16     | coil B current [mA] i16          |
| 5    | commanded Uα [10 mV] i16    | commanded Uβ [10 mV] i16         |
| 6    | supply [10 mV] i16          | position error [1/1000 rev] i16  |

A fault reset thaws the ring. `tools/tpcp_host.py trace <node>` dumps it as
CSV.

### 6.4 `0x50 – 0x58` Status

| CMD    | Register          | Bytes 1–7                                                                                |
| :----- | :---------------- | :--------------------------------------------------------------------------------------- |
| `0x50` | Device info       | `[1]` HW version, `[2..3]` FW version, `[4..7]` device code                               |
| `0x51` | System status     | `[1]` IO word, `[2..3]` uptime bf16 [s], `[4..7]` fault word: bits 0–7 faults, 8–15 warnings, 16–19 state, 20–23 mode |
| `0x52` | Motor status      | `[1]` state:4 / mode:4, `[2]` type:2, phase:2, bit4 CCW, bit5 enabled, bit6 identified, bit7 calibrated, `[3]` motor temp_u8, `[4..5]` power bf16 [W], `[6..7]` torque bf16 |
| `0x53` | Motion status 1   | `[1..3]` position error q1.7.16, `[4..5]` velocity error bf16, `[6..7]` queue:4 / time scale q1.3.8 |
| `0x54` | Motion status 2   | `[1]` executing motion mode (0xFF halted), `[2..7]` position q1.23.24                     |
| `0x55` | Motion status 3   | `[1..3]` velocity f24, `[4..5]` acceleration bf16, `[6..7]` torque bf16                  |
| `0x56` | Homing travel     | **not implemented** → NACK                                                               |
| `0x57` | Encoder position  | `[1..7]` q1.23.32: turns above the raw 32-bit single-turn word                            |
| `0x58` | Driver status     | `[1]` faults, `[2]` warnings, `[3]` operation status, `[4]` driver temp_u8, `[5]` usage_i8, `[6..7]` supply bf16 [V] |

**[decision]** The design numbers Driver Status `0x45`, which lies inside the
sub-space block; it lives at `0x58` so byte 1 can carry the fault byte. Its
reserved word holds the warning byte and the operation status. Motor Status
byte 1 (design: TBD) packs state and mode, byte 2 the identity bits.

**Fault bits:** bits 0–6 are the power stage's own latches, 1:1 with the
driver: `0x01` overvoltage, `0x02` overcurrent, `0x04` overheat, `0x08`
external, `0x10` undervoltage, `0x20` overpower, `0x40` phase. `0x80` encoder
is composed by the drive from the position sensor's fault flag (the board's
link supervisor found the link down - 64 consecutive frames failed the
integrity check, the bus is reset until 256 valid frames are back - or the
sensor reported overspeed / weak field / undervoltage). The condition itself
clears when the feedback is healthy again; the bit stays latched, the bridge
released and closed loop refused, until a fault reset. **Warning bits:** `0x01` not calibrated, `0x02` NVM busy, `0x04`
watchdog expired, `0x08` queue low, `0x10` segment dropped, `0x20` comm
degraded, `0x40` temperature above the warning threshold, `0x80` supply
inside the hysteresis band of a rail limit. **States:** `0` boot, `1`
calibrating, `2` idle, `3` ready, `4` running, `5` fault, `6` unaddressed, `7`
rebooting.

### 6.5 `0x5E` ACK, `0x5F` NACK **[decision]**

The design defines no reply for write-only commands. `[1]` the CMD the reply
refers to, `[2]` error code (0 in an ACK), `[3]` detail (sub id, segment id,
fault mask).

| Code   | Meaning                                       |
| :----- | :-------------------------------------------- |
| `0x01` | Unknown CMD                                   |
| `0x02` | Frame shorter than the command requires       |
| `0x03` | Undefined value in a payload field            |
| `0x04` | Write on a read-only register                 |
| `0x05` | Read on a write-only register                 |
| `0x06` | Unknown sub identifier                        |
| `0x07` | Value rejected by the range check             |
| `0x08` | Not permitted in the current drive state      |
| `0x09` | Subsystem busy (flash, calibration, mailbox)  |
| `0x0A` | Motion queue full                             |
| `0x0B` | Defined but not implemented in this build     |
| `0x0C` | Guard key missing, or the stage must be off   |
| `0x0D` | Refused: a latched fault is pending           |
| `0x0E` | Persistent storage failure                    |
| `0x0F` | Refused: the drive is not calibrated          |
| `0x10` | Motion task part does not match the segment being assembled |

### 6.6 `0x7x` PulseBeat

Emitted cyclically on `0x500 + node` and, unchanged, on `0x100 + node` the
instant the fault byte changes. One decoder serves both channels.

| Bits    | Field      | Description                                            |
| :------ | :--------- | :----------------------------------------------------- |
| 0:7–4   | CMD        | `0x7`                                                  |
| 0:3–0   | Beats      | Sequence counter                                       |
| 1–2:3–0 | CMDQ       | Queued segments (saturating at 15)                      |
| 1–2:15–4| TimeScale  | q1.3.8                                                 |
| 3:3–0   | Status     | Drive state                                            |
| 3:7–4   | Fault      | Group: 0 none, 1 OV, 2 OC, 3 OT, 4 external, 5 UV, 6 OP, 7 phase, F several |
| 4–5:3–0 | Warn       | Group: 0 none, else 1 + index of the highest warning bit |
| 4–5:9–4 | Mode       | Control mode                                           |
| 4–5:15–10| Error     | Following error / spring width, q1.1.4                 |
| 6       | Load       | usage_i8 of the current envelope; −128 overload         |
| 7       | IO         | Input word                                             |

**FaultBeat:** when the transport refuses the beat `nRetry` times in a row
(register `0x2D`), the beat is sent on the emergency identifier instead.

---

## 7. Write-only registers (`0xC0 – 0xDF`)

### 7.1 Control modes

| Mode | Name     | Behaviour                                                   |
| :--- | :------- | :---------------------------------------------------------- |
| `0`  | DISABLED | Output stage held in the configured safe state               |
| `1`  | TORQUE   | Direct q-axis current command                                |
| `2`  | VELOCITY | Velocity command; the position target ramps at that rate     |
| `3`  | POSITION | Absolute position command                                    |
| `4`  | MOTION   | Trajectory queue executed by the motion executor             |
| `5`  | TONE     | Coils driven as a speaker (MIDI control)                     |

The mode is written through `0x21`, but **the frame kind implies it**
**[decision]**: a single-frame position task selects POSITION, a velocity task
VELOCITY, a torque-only task TORQUE, and anything that lands in the queue
(multi-frame segments, burst frames) MOTION. Enable is separate (`0x20`).

### 7.2 `0xC0 – 0xC3` Motion tasks

A segment is up to four frames sharing one non-zero **SegmentID**; the header
(`0xC0`) announces which parts follow and the segment is queued as soon as the
last of them arrives, in any order. A new header with a different SegmentID
discards an unfinished segment (counted in `0x43/0x02`). **SegmentID 0** marks
a single-frame setpoint applied at once. **AxisID** 0 addresses this node; any
other value is executed only by nodes whose Axis Control (`0x01/0x00`) carries
the same id, so one broadcast path drives several axes.

**`0xC0` Motion task 1** - `[1]` SegmentID, `[2]` AxisID, `[3..4]` flags
(LSB first): mode:4, isAbs:1, PosMode:2, TorqMode:2, TimeMode:2, io_trig:1,
isVel:1, isAcc:1, isJrk:1; `[5..7]` time u24 [100 µs].

| Mode | Name         | Needs                | Effect                                                        |
| :--- | :----------- | :------------------- | :------------------------------------------------------------ |
| `0`  | HOLD         | —                    | Hold position, for the time when TimeMode is set, else until the next segment |
| `1`  | FREERUN      | velocity             | Constant velocity until a trigger (position, torque, time, IO) |
| `2`  | FREERUN_LERP | velocity             | Velocity interpolated to the end value until a trigger        |
| `3`  | FREEWHEEL    | —                    | Immediate: release the bridge                                 |
| `4`  | BRAKE        | —                    | Immediate: short the phases                                   |
| `5`  | S-CURVE      | position             | Point-to-point move shaped by the velocity / acceleration carried or the limits |
| `6`  | PVT          | position, time       | Hermite segment ending at the position with the velocity carried |
| `7`  | ANCHOR       | position             | Immediate: redefine "here" (absolute) or shift it (relative) by changing the axis offset of `0x01/0x00` - the rotor does not move; save the node configuration to keep it |
| `8`  | LIMIT        | —                    | Immediate: adopt the velocity / acceleration / jerk carried   |
| `9`  | HOMING       | —                    | **not implemented**                                           |
| `A`  | STEP-DIR     | —                    | **not implemented**                                           |
| `F`  | RESET        | —                    | Immediate: clear the queue and hold                           |

PosMode: 1 target, 2 trigger (the run ends when the position is reached).
TorqMode: 1 limit, 2 trigger (the run ends when the torque is reached).
TimeMode: 1 duration, 2 trigger. Parts required: task 2 when PosMode ≠ 0,
task 3 when isVel or TorqMode ≠ 0, task 4 when isAcc, isJrk or io_trig.

**`0xC1` Motion task 2** - `[1]` SegmentID, `[2..7]` position q1.23.24
(absolute, or a displacement when isAbs = 0). Single frame: absolute position
setpoint (CSP).

**`0xC2` Motion task 3** - `[1]` SegmentID, `[2..3]` torque bf16 (NaN unused),
`[4]` reserved, `[5..7]` velocity f24 (NaN unused). Single frame: velocity
setpoint (CSV) when the velocity is finite - a finite torque beside it caps
the current envelope - otherwise a torque setpoint (CST).

**`0xC3` Motion task 4** - `[1]` SegmentID, `[2..3]` acceleration bf16,
`[4..5]` jerk bf16, `[6]` IO mask (probes that end the segment when active),
`[7]` IO mask (probes that end it when inactive). Single frame: adopts the
acceleration and jerk as the live limits.

Absolute queued targets are converted to displacements by the control ISR
against the end of the queue, so a mixed absolute / relative stream stays
consistent. The queue end follows the live target whenever nothing is
queued (after a velocity run, a setpoint, a stop, or while parked), so the
first profiled move after any of those is sized from where the rotor is.

### 7.3 `0xCF` Save config

`[1..3]` catalog u24 (bit0 node configuration, bit1 motor profile, `0xFFFFFF`
all), `[4..7]` keyword: `0x000010AD` save, `0x0000DEF0` restore the compiled-in
defaults into the live shadow (keeps the address) **[decision]**, `0x0000E7A5`
erase the stored configuration - a factory reset that surrenders the address
**[decision]**. Every keyword is refused while the stage is enabled: flash
programming stalls the CPU with whatever vector is latched on the bridge.

### 7.4 `0xD0` MIDI control

`[1]` waveform:4 (0 sine; others rejected) / mode:4, `[2]` MIDI note (69 = A4;
0 = silence), `[3]` volume 0–255, `[4..5]` duration bf16 [ms] (0 = until the
next command), `[6]` attack share 0–255, `[7]` decay share 0–255. Selects TONE
mode and enables the stage by itself; the note is bounded to 40–2000 Hz and the
amplitude to a quarter of the bus voltage.

---

## 8. Burst mode (`0xE0 – 0xFF`)

One segment per frame, MSB first, for dense synchronous streaming. The format
is selected by the burst step of System Control: 0 = PVT (time-tagged),
otherwise PVcTF with that fixed period.

| Byte | Bits | PVT                                  | PVcTF                            |
| :--- | :--- | :----------------------------------- | :------------------------------- |
| 0    | 7–5  | `111`                                | `111`                            |
| 0    | 4–0  | time21 bits 20–16                    | frame counter                    |
| 1–2  |      | time21 bits 15–0                     | force bf16                       |
| 3–4  |      | velocity bf16 [RPS]                  | velocity bf16 [RPS]              |
| 5–7  |      | position, low 24 bits of q1.15.16    | position, low 24 bits of q1.15.16|

`time21 = (system time [µs] / 250) mod 2²¹`: an **absolute** tag, so a lost
frame shifts nothing; the master's clock is handed to the node through the
Sync field of Bus Settings. Each PVT segment lasts the distance to the
previous tag (or to the node clock for the first point). Each PVcTF segment
lasts `steps × burst step × 250 µs`, where `steps` is the counter distance to
the previous frame - a lost frame stretches the segment instead of shifting
everything after it.

The position is the low 24 bits of the absolute position; the node resolves
the absolute value nearest the end of its queue, which is unambiguous while a
segment stays under 128 revolutions (the design asks for under 8). Keep
segments under 8 s. **[decision]** The design's "float16" velocity is bf16,
like every other 16-bit float in the protocol.

Burst frames select MOTION mode by themselves. In comm mode 1 they are silent
unless rejected.

---

## 9. Safety

- **Master-liveness watchdog.** Any command frame addressed to this node
  reloads it. On expiry the drive enters its configured safe state and raises
  the watchdog warning; it stays there until a new command arrives.
- **Emergency frames.** `0x100 + node` outranks every command and every other
  node's telemetry.
- **Enable guards.** Energizing is refused while uncalibrated (`0x0F`), with a
  latched fault (`0x0D`) or during an identification (`0x09`).
- **Register guards.** Protection thresholds and the machine identity are
  writable only while the output stage is disabled (`0x08`).
- **Flash guards.** Every NVM write waits for the de-energized programming
  window the control loop publishes; a save that never finds one is abandoned
  after 2 s and reported in `0x43/0x03`.
- **Bus-off recovery.** Handled in the transport service call.

---

## 10. What v1 does not implement

- `0x56` Homing travel, `0xC0` modes HOMING / STEP-DIR, F_CAL / M_CAL,
  writing `0x2F`, tables other than the encoder table → `TP_ERR_NOT_IMPLEMENTED`.
- The disbalance, recovery-time, I²t budget, error-limit and motor-derating
  fields are **stored and reported** but not yet enforced.
- Gearbox, IO map, encoder declarations, homing settings and reference limits
  are stored for the features that will consume them; the IO word of `0x51`
  and the PulseBeat reads 0 on CLN17 v3, which wires no inputs.
- Bit-rate and bus-type changes take effect after a save and a reboot.
- Calibration abort: reboot (`0x20` action `0xB0`) is the way to stop one.

---

## 11. Transports and the USB-to-CAN gateway

The engine names no peripheral: it moves frames through function pointers, so
the same protocol runs over anything that carries an identifier and eight
bytes. Three bindings ship: **CAN** (`transport/tp_can.h`), **HID**
(`transport/tp_hid.h`, the control transport: 64-byte reports of up to six
ten-byte records `id_lo, id_hi | len<<4, d0..d7`) and **Serial**
(`transport/tp_serial.h`, framed `A5 5A | id_lo id_hi | d0..d7 | xor` on the
CDC console). A response goes back to the transport that asked; unsolicited
frames go to all of them.

The gateway relays between USB and CAN: command frames the host sends for
other nodes or the broadcast go down, node → master frames (`0x1xx`, `0x4xx`,
`0x5xx`) come up. It arms on the first USB protocol frame and opens the master
filter slots. Discovery works through it because every reply is a plain "this
subtree is populated": identical replies merge on the bus, this node adds its
own while unaddressed, and the host sees one answer either way.

---

## 12. Worked examples

```text
# 1. Discovery on a fresh bus (drive has no address)
→ 0x300  12 00 00 00 00 00 00 00        search depth 0
← 0x4FF  12 01 00 00 00 00 00 00        populated
→ 0x300  12 01 00 00 00 00 00 00        probe prefix bit0 = 0
← 0x4FF  12 02 00 00 00 00 00 00        populated ... walk to depth 48
→ 0x300  12 30 A3 17 5C 02 B9 41        probe the full identity
← 0x4FF  12 30 A3 17 5C 02 B9 41        exactly one node
→ 0x300  91 07 A3 17 5C 02 B9 41        assign node 7 (write of 0x11)
← 0x407  11 07 A3 17 5C 02 B9 41        confirmed on the NEW address
→ 0x300  11 00 A3 17 5C 02 B9 41        double-check: ping by identity
← 0x407  11 07 A3 17 5C 02 B9 41

# 2. Configure: 1 A envelope, 200 ms watchdog, 20 Hz position telemetry
→ 0x307  A1 00 00 00 00 00 80 3F        Motor Properties 1 write: max current 1.0 A
← 0x407  21 ..                          read-back
→ 0x307  AD 7F C0 7F C0 7F 4D 3E        Watchdog write: timeout 0.2 s (period / delay NaN = unchanged)
← 0x407  2D ..
→ 0x307  82 00 54 01 32 00 00 00        Telemetry slot 0: emit 0x54 every 50 ms
← 0x407  02 00 54 01 32 00 00 00
← 0x507  54 ..                          telemetry starts flowing

# 3. Enable and move one revolution (CSP: a lone Motion Task 2)
→ 0x307  A0 E1 00 00 00 00 00 00        System Control: enable
← 0x407  20 E1 00 07 00 00 00 00
→ 0x307  C1 00 00 00 00 01 00 00        position 1.0 rev, absolute
                                        (silent: an accepted setpoint in comm mode 1)

# 4. Queue a point-to-point move (S-curve segment, two frames)
→ 0x307  C0 05 00 35 10 00 00 00        segment 5: S-CURVE, absolute, pos target, no time
→ 0x307  C1 05 00 00 00 02 00 00        target 2.0 rev -> queued, MOTION mode selected

# 5. Stream PVT (burst step 0)
→ 0x307  E0 00 28 41 20 00 40 00        t=40 x 250 us, 10 RPS, pos 0x004000 = 0.25 rev
→ 0x307  E0 00 50 41 20 00 80 00        t=80, 10 RPS, 0.5 rev

# 6. Park and persist
→ 0x307  C0 00 00 00 00 00 00 00        lone task 1, HOLD
→ 0x307  A0 0F 00 00 00 00 00 00        disable (opens the flash window)
→ 0x307  CF 01 00 00 AD 10 00 00        save the node configuration
← 0x407  5E CF 00 01 00 00 00 00        ACK
```

---

## 13. Implementation map

| File                                      | Contents                                        |
| :---------------------------------------- | :---------------------------------------------- |
| `src/api/tp_wire.h`                       | Umbrella: everything that appears on the bus     |
| `src/api/wire/tp_ids.h`                   | Identity, identifier map, predicates             |
| `src/api/wire/tp_cmds.h`                  | CMD groups, register addresses, sub ids, keys    |
| `src/api/wire/tp_enums.h`                 | Error, state, warning and selector enums         |
| `src/api/wire/tp_codecs.h`                | Fixed-point and mini-float codecs                |
| `src/api/tp_chunks.h`                     | Umbrella + `union TpFrame`                       |
| `src/api/chunks/tp_chunk_config.h`        | `0x00–0x3F` read/write register layouts          |
| `src/api/chunks/tp_chunk_status.h`        | `0x40–0x7F` read-only registers, ACK, PulseBeat  |
| `src/api/chunks/tp_chunk_motion.h`        | Motion tasks, save, MIDI, burst                  |
| `src/api/tp_registers.h`                  | Umbrella: register map + config apply            |
| `src/api/registers/tp_register_map.h`     | `TP_REGISTERS`: access rules, sub-space sizes    |
| `src/api/registers/tp_config_apply.h`     | `tp_apply_config`: shadow → hardware             |
| `src/api/tp_node.h`                       | Umbrella: the node layer                         |
| `src/api/node/tp_node_config.h`           | Persisted record and build-time defaults         |
| `src/api/node/tp_node_identity.h`         | The 48-bit HashUID folded from the die id        |
| `src/api/node/tp_node_store.h`            | Flash-backed load / save engine (256 B records)  |
| `src/api/node/tp_discovery.h`             | Binary HashUID search participant                |
| `src/api/tp_engine.h`                     | `TpProtocol` declaration and state               |
| `src/api/engine/*_impl.h`                 | Its method bodies, eight files by concern        |
| `src/api/transport/tp_can.h`              | FDCAN1 binding, transport mux, USB-CAN gateway   |
| `src/api/transport/tp_serial.h`           | Framed transport over the USB CDC console        |
| `src/api/transport/tp_hid.h`              | Report transport over the vendor HID endpoints   |
| `test/test_tpcp/test_wire.cpp`            | Host-side wire-format conformance tests          |
| `tools/tpcp_host.py`                      | Reference master (CLI)                           |
| `tools/tpcp_mcp.py`                       | The same master as MCP tools                     |

### Threading model

`TpProtocol::poll()` runs in thread mode from `loop()`. Nothing in the protocol
touches control state directly: every command that changes what the motor does
goes through the `DriveControl` mailbox, a lock-free single-producer queue the
20 kHz control ISR drains between passes. Telemetry flows back through a
sequence lock written by the ISR. That is what makes it safe for a protocol
handler to be preempted at any instruction by the control loop.
