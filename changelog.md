## 2026-09-06 - hardware/ restructure: encoders + temperature, int32 WS2812 API, BSRR CLN17 LED
**Changes:**
- Added hardware/encoders/ (mt6835.h moved there + readme.md mandating 32-bit left-aligned get_angle output)
- Added hardware/temperature/ntc.h and ptc.h - constexpr NtcConfig/PtcConfig template converters (divider topology, beta table / slope k, standard KTY84-130 and B25/100=4250 defaults) returning float degC
- CLN234 temperature.h refactored onto NTC_GetCelsius<kNtcSensor>
- CLN234 WS2812 API switched to two packed RGB888 int32 words (LED_SetRGB/LED_SetRGB2/LED_Off) matching LedApplyCallback - tunepulse.h forwards unchanged, selftest packs palette
- CLN17 LED_Apply now byte-non-zero=ON semantics via direct GPIO BSRR register writes (LL removed)

**Reasoning:**
- Uniform encoder angle contract for the Q31 motion pipeline
- reusable hardware-level thermistor converters instead of per-target math
- zero-copy int32 color path from LedDriver to WS2812
- faster atomic LED updates without LL dependency

**Notes:**
- Include path updated: hardware/encoders/mt6835.h (cln234/spi_setup.h)
- NTC conversion is bit-exact with the previous piecewise-beta implementation (same clamps: short=+150C, open=-60C)
- cln17_v3: Flash 67804 / RAM 7832
- cln234: Flash 71912 / RAM 7976

FLASH: 67804 bytes (51.73% of 131072 bytes): +0 bytes
RAM: 7832 bytes (23.90% of 32768 bytes): +0 bytes


## 2026-09-06 - hardware/temperature rewritten: minimal int-NTTP templates, uint16 in / float degC out
**Changes:**
- ntc.h and ptc.h rewritten as minimal templates: mandatory R25/B25 (NTC) or R25/K_mOhm (PTC), optional Pullup/Pulldown [Ohm], Vref_mV and Rth_x100 - scenario (low-side / high-side divider, self-heating dT=P*Rth) is selected by argument presence via if constexpr, absent options are compiled out (zero flash/RAM overhead)
- Added hardware/temperature/readme.md documenting the contract: input is ALWAYS uint16 left-aligned ADC data, output is ALWAYS float degC unchanged
- CLN234 temperature.h uses NTC_GetCelsius<100000, 4250, 40200> (single B25/50 beta, piecewise table dropped)

**Reasoning:**
- Float NTTPs are C++20-only and rejected by the toolchain - integer scaled arguments keep C++17 compatibility
- remove over-engineered config struct / beta-table machinery
- unify both sensors on one contract

**Notes:**
- Open/short clamps built-in (+150/-60 degC)
- Rth without Vref or two pull resistors is a static_assert error
- cln234: Flash 71844 / RAM 7976
- cln17: Flash 67804 / RAM 7832

FLASH: 71844 bytes (54.81% of 131072 bytes): +4040 bytes
RAM: 7976 bytes (24.34% of 32768 bytes): +144 bytes


## 2026-09-06 - hardware/temperature: internal code inversion + doxygen topology diagrams
**Changes:**
- NTC_GetCelsius/PTC_GetCelsius now normalize the divider code INTERNALLY into the monotonic domain based on the topology argument: Pullup (low side) -> mono = 4095 - code, Pulldown (high side) -> mono = code
- after normalization both topologies share one unified path (R_sensor = Rref*(4095-mono)/mono, clamps and self-heating are topology-independent)
- Added \\verbatim ASCII doxygen diagrams of both divider topologies (low/high side, polarity, where Pullup/Pulldown sit) into both file headers and hardware/temperature/readme.md
- cln234 Temperature_NTC_To_Celsius fixed back to uint32_t signature (Temperature_Convert_Callback) - it undoes the feed inversion and forwards the TRUE word so the template clamps stay on the TRUE domain

**Reasoning:**
- Callers must never pre-invert: polarity is a property of the topology, so it belongs inside the getter
- unified post-normalization path removes the duplicated branch logic
- ASCII diagrams document the option set without binary image assets

**Notes:**
- cln234: Flash 71828 / RAM 7976
- cln17: SUCCESS unchanged
- numeric equivalence verified: mid-range R(T) identical to the previous implementation, shorted -> +150C, open -> -60C in both topologies

FLASH: 71828 bytes (54.80% of 131072 bytes): -16 bytes
RAM: 7976 bytes (24.34% of 32768 bytes): +0 bytes


## 2026-09-06 - hardware/temperature: self-heating removed, custom safe window -> NaN, full 16-bit domain
**Changes:**
- Removed Vref_mV/Rth_x100 self-heating scenario
- Added Tmin_x10/Tmax_x10 custom safe window (defaults -600/+1500 = -60..+150 degC): results outside the window (shorted sensor above / not connected below) return NaN instead of hardcoded clamps
- Input is now strictly uint16 using ALL 16 bits (full scale 65535, low nibble no longer discarded)
- Documented binary-search compatibility with Temperature::celsius2raw (NaN compares as 'not less' -> search converges at the valid-band boundary
- tick() is integer-only)
- Restored the feed inversion in CLN234 Temperature_NTC_Feed (driver domain must stay monotonic for the threshold search) and refreshed its docs

**Reasoning:**
- NaN is the correct IEEE semantics for a sensor fault - it propagates through telemetry math and cannot be mistaken for a real temperature
- full 16-bit domain removes the hidden 12-bit assumption
- monotonic driver domain is mandatory for the raw threshold search

**Notes:**
- cln234 pipeline: feed inverts (hot->large), callback undoes, template re-inverts internally
- configure OT limits inside the safe window
- cln234: Flash 71856 / RAM 7976
- cln17: SUCCESS unchanged

FLASH: 71856 bytes (54.82% of 131072 bytes): +28 bytes
RAM: 7976 bytes (24.34% of 32768 bytes): +0 bytes


## 2026-09-06 - Runtime current-sensor gain + encoder fault + PWM rename
**Changes:**
- CurrentSense: runtime set_i_fs()/i_fs() per-channel full-scale with dedicated SeqLock (fs_seq_) snapshot in tick()
- recompute_scales_() helper
- OC limits kept in amps and re-normalized via OvercurrentProxy::refresh_limits() with DAC threshold reprogramming
- Encoder: atomic encoder_fault flag, set_fault(bool)/is_fault() with rising-edge fault action callback and fluent fault_cb()
- PwmWrapper renamed to PwmControl (file pwm_control.h, includes/types/docs updated in driver_pwm.h and motor_control.h)

**Reasoning:**
- Allow realtime sensor gain adjustment while preserving int16 normalization relative to the highest-current sensor full-scale
- keep ISR hot path lock-free and single-pass
- add encoder fault handling hooks
- finish the incomplete pwm_wrapper rename

**Notes:**
- Hot-path overhead: one relaxed load+compare in common case
- set_i_fs is single-writer (config context)
- re-calibrate zero-current offsets if analog front-end gain physically changes

FLASH: 67852 bytes (51.77% of 131072 bytes): -4004 bytes
RAM: 7856 bytes (23.97% of 32768 bytes): -120 bytes


## 2026-09-06 - CurrentSense: simplified runtime gain API (set_gain)
**Changes:**
- Replaced SeqLock-based set_i_fs with single set_gain(ch1, ch2, ch3) for all channels
- constructor scale math extracted into recompute_scales_() precompute helper
- OC limits stored in amps and re-normalized via refresh_limits() on gain change
- tick()/process_channel() hot path left byte-identical to the original

**Reasoning:**
- Gain changes are rare and performed from a safe context, so locking is unnecessary
- zero-cycle overhead in the control ISR
- keep int16 normalization relative to the highest-current sensor full-scale

**Notes:**
- set_gain must be called only when tick()/set_currents_irq() cannot run (motor stopped)
- re-calibrate zero offsets if analog front-end gain physically changes

FLASH: 67792 bytes (51.72% of 131072 bytes): -60 bytes
RAM: 7856 bytes (23.97% of 32768 bytes): +0 bytes


## 2026-09-05 - Full CAN application protocol (TPCP v1) with runtime control modes and trajectory execution
**Changes:**
- Added src/api/: tp_wire.h, tp_chunks.h, tp_registers.h, tp_node.h, tp_engine.h, tp_can.h and CAN_PROTOCOL.md implementing TunePulse CAN Protocol v1
- Added drive_control.h: runtime control-mode machine (disabled/torque/velocity/position/motion), lock-free SPSC command mailbox, seqlock telemetry, master-liveness watchdog and MotionExecutor front end
- Wired DriveControl into TunePulseHeart::tick(), replacing the hardcoded pos_hold_target_/is_pos_used path
- Added a dedicated 2 KiB node-config NVM partition (NVM_CONFIG_SECTION_SIZE) with a magic/version/CRC32 record
- can_setup.h: RX ring now keeps the full 11-bit identifier and preserves frames shorter than 8 bytes, plus per-node hardware filters, bus-off recovery, TX abort and runtime link reconfiguration
- Fixed the float24 decoder, which sign-extended downward and destroyed the exponent of every negative value
- Replaced the main.cpp console-over-CAN demo with the protocol host
- Removed the superseded src/api/tp_api.h and API.md drafts
- Added test/test_tpcp (15 Unity wire-format tests) and the native_tpcp environment
- Drive no longer energizes itself after calibration; opt in with the auto-enable config flag

**Reasoning:**
- Nothing in the control path used CAN: no command parser, no node addressing, no telemetry, no heartbeat (suggestions.md S6)
- The motion stack and the control-mode switch were complete but unwired (S3, S4), so the protocol had nothing real to command
- Truncating the received identifier to 8 bits aliased every protocol channel onto one value, making identifier-based addressing impossible
- Frames shorter than 8 bytes were silently dropped, which network management needs
- The node address must survive motor-profile saves, which erase the whole user partition roughly every 60 writes
- Sign-extending an IEEE-754 truncation overwrites the exponent, turning every negative velocity into an infinity or a NaN

**Notes:**
- Protocol is 8-byte-frame based so it runs unchanged on classic CAN 2.0A
- FD 9-64 byte payloads stay reserved for bulk transfer. Every command reaches the 20 kHz ISR through the DriveControl mailbox, so no protocol handler touches control state. NOT verified on hardware yet: watch tick/s in the console heartbeat, since the executor now runs inside the control ISR at -O0. Calibration abort, full-calibration sequencing, absolute S-curve targets and CAN firmware update are deliberately unimplemented (CAN_PROTOCOL.md section 9).

FLASH: 95320 bytes (72.72% of 131072 bytes): +25576 bytes
RAM: 10104 bytes (30.83% of 32768 bytes): +1772 bytes


## 2026-09-05 - Bench bring-up path for the CAN protocol: host master, loopback self-test, bench build
**Changes:**
- Added tools/tpcp_host.py: python-can master implementing discovery, register get/set, motion commands, telemetry configuration, calibration, save and a decoding bus monitor
- Added build-time node defaults (TP_DEFAULT_NODE_ID / TP_DEFAULT_LINK_FLAGS / TP_DEFAULT_ARB_KBPS / TP_DEFAULT_DATA_KBPS)
- Added CANDriver internal-loopback option to configure_link() and the TP_BENCH_LOOPBACK self-test in main.cpp
- Added the cln17_v3_bench environment: node 1, classic CAN at 500 kbit/s, loopback
- Exposed TpProtocol::handled() and added it to the console heartbeat line
- Wired TpNodeStore::erase_all() into PARAM_DEFAULTS target 'all' so a factory reset actually clears the stored record

**Reasoning:**
- The protocol had no counterpart: a drive is a pure slave and nothing on a bench could speak TPCP to it
- A shipping node comes up unaddressed and answers only discovery, so first contact needed a discovery-capable master or a build-time address
- The default FD link at 1 Mbit/s / 5 Mbit/s cannot be reached by the classic-CAN USB adapters most benches have
- A lone CAN node is never acknowledged, so its transmit error counter climbs until it goes bus-off - the whole stack is untestable with one board unless the controller runs in loopback

**Notes:**
- Host tool codecs, frame builders and the full 70-entry register map are cross-checked against the firmware headers. Loopback proves framing, filters, dispatch and telemetry with no adapter at all, but not the physical layer: remove TP_BENCH_LOOPBACK before connecting a real bus, since a node in loopback is invisible to every other node.

FLASH: 95648 bytes (72.97% of 131072 bytes): +328 bytes
RAM: 10112 bytes (30.86% of 32768 bytes): +8 bytes


## 2026-09-05 - Serial protocol transport, transport multiplexer and bounded calibration waits
**Changes:**
- Added src/api/tp_serial.h: the protocol over USB CDC, framed as A5 5A | id | 8 bytes | xor, sharing one endpoint with the ASCII console
- Added a transport multiplexer in tp_can.h: responses return to whichever transport asked, telemetry and emergency frames go to both
- Added --interface serial to tools/tpcp_host.py, including console log passthrough
- Bounded the two blocking calibration waits in tunepulse_init() with TP_CURRENT_CAL_TIMEOUT_MS and TP_MOTOR_CAL_TIMEOUT_MS

**Reasoning:**
- A bench with no USB-CAN adapter could not exercise the protocol at all
- the engine was already transport-agnostic, so a second binding costs one file
- The sync byte has bit 7 set and log output is ASCII, so binary frames and console text cannot be confused and need no mode switch
- The calibration waits were unbounded and advanced only from interrupt context, so anything that stopped those interrupts hung the firmware with no console, no CAN and nothing to diagnose with

**Notes:**
- Serial covers dispatch, registers, telemetry, motion, calibration, persistence and the watchdog
- it does not cover bit timing, arbitration, hardware filtering, bus-off recovery or the discovery sweep, which need a real bus. Framing round trip, log interleaving, split reads, false-sync recovery and checksum rejection are covered by a host-side parser test.

FLASH: 96308 bytes (73.48% of 131072 bytes): +660 bytes
RAM: 10144 bytes (30.96% of 32768 bytes): +32 bytes


## 2026-09-05 - Hardware validation fixes: serial discovery routing, host auto-detect, persistence status bit
**Changes:**
- Routed discovery replies through the transport multiplexer so an unaddressed node is reachable over serial
- Host tool auto-detects whether a port is a TunePulse console or a CAN adapter, rewrites /dev/tty.* to /dev/cu.*, and reports an unassigned node instead of timing out on every request
- Persistence status no longer reports a write failure on a node that has simply never saved

**Reasoning:**
- An unaddressed node could not be discovered over serial: replies went out on a CAN bus that was not connected, which is a chicken-and-egg on any bench without an adapter
- Sending slcan adapter commands at a drive console produced sixteen silent timeouts with no indication of the cause
- is_saved() is false both when a save failed and when none was ever attempted, so a freshly booted node reported a failure it never suffered

**Notes:**
- Validated on CLN17 v3 hardware over the serial transport: discovery, address assignment, NVM persistence across a reboot (cfg_ver=1 restored, node address recovered without a rescan), all 16 status chunks with zero NACKs, and the control loop holding 20000 tick/s with the motion executor now running inside the ISR. The CAN controller reports error-passive (rec=127) as expected while alone on an unterminated bus
- recheck once an adapter is attached.

FLASH: 96280 bytes (73.46% of 131072 bytes): -28 bytes
RAM: 10136 bytes (30.93% of 32768 bytes): -8 bytes


## 2026-09-05 - Host tool finds the drive on its own
**Changes:**
- tools/tpcp_host.py auto-detects the drive's console by passively sniffing every USB serial device when --channel is omitted
- TPCP_CHANNEL environment variable overrides the search

**Reasoning:**
- Passing --channel on every invocation invited shell-variable workarounds that break under zsh, which does not word-split unquoted parameter expansions the way bash does
- naming the wrong port (an ST-Link VCP rather than the board's own USB) was a recurring dead end

**Notes:**
- Firmware unchanged by this entry. Sniffing is passive - nothing is transmitted - so probing an unrelated adapter cannot disturb it.

FLASH: 96280 bytes (73.46% of 131072 bytes): +0 bytes
RAM: 10136 bytes (30.93% of 32768 bytes): +0 bytes


## 2026-09-05 - Host tool command latency cut from 4.3 s to 0.2 s
**Changes:**
- Replaced passive console sniffing with an active broadcast Device Info probe whose reply identifier also reveals the node address
- Collapsed channel resolution and transport detection into one pass so a port is never interrogated twice
- Cached the last port that answered in ~/.cache/tpcp_host_port
- Passive sniffing, still used for unaddressed drives, now exits as soon as it has its answer instead of always burning the full window

**Reasoning:**
- Auto-detection listened to every USB serial device for 1.2 s and then sniffed the winner again for 1.5 s, so every command paid roughly 3.9 s of blocking waits to do 0.08 s of work
- asking the drive a question is far quicker than waiting for it to speak on its own

**Notes:**
- Measured on CLN17 v3 hardware: 4.29 s before, 1.94 s cold, 0.20 s warm. Firmware unchanged.

FLASH: 96280 bytes (73.46% of 131072 bytes): +0 bytes
RAM: 10136 bytes (30.93% of 32768 bytes): +0 bytes


## 2026-09-05 - Report frozen observer state honestly; add a settle wait to the host move command
**Changes:**
- Motion Status 2 and 3 now report zero velocity and acceleration when the position loop did not run, instead of the observer's last value from before the bridge was released
- Drive state reports IDLE rather than READY whenever the safe-state gate has released the bridge, even if the output stage is still armed
- Added DRIVE_TLM_LOOP_LIVE to the control telemetry snapshot
- Added move --wait to the host tool, polling the following error until the rotor arrives

**Reasoning:**
- The observer only advances while the position controller is ticked, so a released bridge left it frozen: a stationary rotor reported -10 RPS of velocity error
- Enabled is the output-stage arm and is not the same as controlling the motor, so a drive with mode DISABLED and enable set reported READY while actually coasting
- Setpoint commands are asynchronous by design, and once host latency dropped from 4.3 s to 0.2 s the gap between commands no longer accidentally let a move finish

**Notes:**
- Diagnosed on hardware: a two-revolution move followed 0.2 s later by its reverse and 0.2 s later by a disable released the bridge at 600 rpm, so the rotor coasted to an arbitrary stop. Nothing was wrong with the position arithmetic - the target returned to its start exactly.

FLASH: 96336 bytes (73.50% of 131072 bytes): +56 bytes
RAM: 10136 bytes (30.93% of 32768 bytes): +0 bytes


## 2026-09-05 - Profiled back-and-forth moves in the host tool
**Changes:**
- pvt gains --back, --cycles and --wait, and streams each leg separately instead of queueing every segment at once
- Settle detection now keys on the following error and queue depth held over consecutive polls rather than the velocity error

**Reasoning:**
- A back-and-forth move needs its legs paced, or they queue up and run as one continuous motion
- The velocity error is frozen whenever the position loop is not running, so waiting on it hangs against a drive that has already stopped and against any firmware predating that fix

**Notes:**
- Firmware unchanged by this entry. Position-mode moves are step commands with no profile, so multi-revolution moves at a controlled speed belong in the trajectory queue.

FLASH: 96336 bytes (73.50% of 131072 bytes): +0 bytes
RAM: 10136 bytes (30.93% of 32768 bytes): +0 bytes


## 2026-09-05 - Host tool refuses trajectories a drive cannot execute, and can clear the queue
**Changes:**
- pvt verifies the drive is in motion mode with the output stage enabled before streaming, and offers --set-mode to do it
- Added a queue subcommand (status/clear/pause/resume/scale)
- Settle failures now distinguish a queue that is not draining from a rotor that is not following

**Reasoning:**
- Segments pushed at a drive in any other mode are accepted and queued, then never execute because the executor only advances in motion mode
- that looked identical to a mechanical failure and the tool reported it as one, blaming a current limit that was not involved

**Notes:**
- Reproduced on hardware: 20 segments sat queued for 16 s with the following error at zero because a disabled drive pins its target to the measured position. The guard now refuses in 0.24 s without streaming anything. Firmware unchanged.

FLASH: 96336 bytes (73.50% of 131072 bytes): +0 bytes
RAM: 10136 bytes (30.93% of 32768 bytes): +0 bytes


## 2026-09-05 - Streaming batch size for trajectory segments, and a fine-segment guard
**Changes:**
- Trajectory legs stream in batches of four frames rather than one frame at a time or all at once
- pvt warns when the requested segment duration is finer than the transport can sustain

**Reasoning:**
- One frame at a time is slower than the drive consumes fine segments so the queue starves
- the whole burst in one write overruns the board's USB CDC receive buffer and frames are lost silently, which measured three times worse

**Notes:**
- Measured on hardware, 2 rev in 1.0 s: 50 ms segments land within 0.005 rev on a warm leg, 20 ms segments lose a third of the move. Recorded as suggestions.md S6b, including the unexplained ~0.09 rev shortfall on the first leg after entering motion mode. Firmware unchanged.

FLASH: 96336 bytes (73.50% of 131072 bytes): +0 bytes
RAM: 10136 bytes (30.93% of 32768 bytes): +0 bytes


## 2026-09-05 - USB HID control transport: composite CDC + vendor HID device
**Changes:**
- Composite USB device in src/target/cln17_v3/usb_hid_setup.h: CDC console on interfaces 0/1 plus a vendor HID function on interface 2, endpoints 0x84 IN / 0x04 OUT, 64-byte reports at 1 ms
- New TPCP transport src/api/tp_hid.h packing six frames into each report, multiplexed into tp_can.h alongside CAN and the framed serial link
- tp_hid_begin() replaces SerialUSB.begin() in setup(), tp_can_service() drains the HID transmit ring
- Host tool gains --interface hid (HidLink) and picks it automatically when the board exposes one
- Heartbeat line reports hid[rx/tx/drop]

**Reasoning:**
- A byte stream is the wrong shape for a control link: CDC has no message boundary so every frame needs sync bytes and a checksum, and the host cannot see the board's buffer, so trajectory bursts are streamed four frames at a time with a sleep between batches - faster overruns the receive buffer silently, slower starves the queue mid-move
- A HID interrupt endpoint frames in hardware and flow-controls itself, so bursts go out at full speed with bounded 1 ms latency and no driver to install
- The stock STM32duino stack cannot do this - CDC and its HID are mutually exclusive and its HID is IN-only - so the composite class is registered against the framework's own CDC device handle and forwards every CDC callback to ST's driver, leaving SerialUSB and the log macros untouched

**Notes:**
- Endpoint 4 extends the buffer descriptor table to 40 bytes over the EP0 OUT buffer the framework places at 32, so EP0 OUT is relocated to 512 and the HID buffers sit at 640/704. Product ID moves to 0x5741 because hosts cache descriptors against VID/PID and would otherwise reuse the cached CDC-only layout. Verified from the built ELF: the configuration descriptor walks exactly to its 107-byte wTotalLength across 3 interfaces and 5 endpoints, and the report descriptor is a balanced item stream. NOT yet tested on hardware - not flashed.

FLASH: 98260 bytes (74.97% of 131072 bytes): +1924 bytes
RAM: 11192 bytes (34.16% of 32768 bytes): +1056 bytes


## 2026-09-05 - PVT streaming paces against the drive's queue instead of the transport
**Changes:**
- stream_pvt_leg() now reads free slots from the Motion Queue Status chunk and refills as the executor drains, instead of pushing the whole leg in one burst
- New query_queue() helper returning queued/free/dropped
- The fine-segment warning is now raised only on transports that cannot refill fast enough, so it no longer fires on HID

**Reasoning:**
- The trajectory queue is 32 segments deep and silently rejects the overflow
- a 60-segment leg lost 28 of them and ran half the commanded distance on both legs
- This was masked by how slow CDC was - four frames per write with a sleep between batches drained the queue as it filled - so the HID transport delivering a burst in under a millisecond exposed it
- Polling is race-free because both transports deliver in order and the engine dispatches in arrival order, so a status reply reflects every setpoint sent before it

**Notes:**
- Root cause confirmed on hardware: the drive reported dropped=470 after the truncated runs, with free=32 matching DRIVE_MOTION_QUEUE_DEPTH. query_queue() verified over HID against the live board. Host tool only, firmware unchanged. The fixed streaming path has NOT been run on hardware yet - acceptance test is that dropped stops climbing and both legs cover the full distance.

FLASH: 98260 bytes (74.97% of 131072 bytes): +0 bytes
RAM: 11192 bytes (34.16% of 32768 bytes): +0 bytes


## 2026-09-05 - Motion Queue Status reported one free slot more than the queue has
**Changes:**
- DriveTelemetry carries a queue_free field taken from MotionExecutor::free_slots()
- Motion Queue Status (0x1C) reports that instead of deriving free slots as depth minus queue_depth
- PVT streaming warns when the drive rejects a segment it had just advertised room for

**Reasoning:**
- queue_depth is future_count(), the segments waiting BEHIND the one executing, so the in-flight segment and any retained history were not counted as occupied
- A full 32-slot queue therefore advertised 1 free slot forever, and a host pacing itself on that number fed segments into a queue that rejected every one
- Measured on hardware: steady state queued=31 free=1 with the drive's dropped counter incrementing once per frame sent

**Notes:**
- Root cause of the 3-revolution move running 1.5 revolutions. The executor already exposed the correct free_count() - (head-tail >= N) ? 0 : N-(head-tail) - and nothing used it. Host-side flow control alone could not fix this: it was pacing against a number the firmware got wrong. REQUIRES A REFLASH
- the host tool changes do not.

FLASH: 98292 bytes (74.99% of 131072 bytes): +32 bytes
RAM: 11200 bytes (34.18% of 32768 bytes): +8 bytes


## 2026-09-05 - Motor speaker: DriveMode::Tone and MIDI playback from the host
**Changes:**
- New DriveMode::Tone (wire value 5) and DriveAction::Tone driving the alpha-beta vector straight from MotorSpeaker, one sample per control pass
- New DriveReq::SetTone carrying frequency and amplitude in one mailbox slot
- Registers 0x050B tone_freq and 0x050C tone_amp
- Host gains music (plays a MIDI file through the coils) and music-stop
- Minimal standard-MIDI parser reducing a polyphonic arrangement to the highest sounding voice

**Reasoning:**
- MotorSpeaker already existed to play melodies through the coils but only its test-tone path was wired up, so the melody engine was unreachable
- A motor is monophonic, so a piano arrangement needs reducing to one voice and the top note is where the melody sits in nearly all keyboard writing
- Frequency and amplitude share one request so a note change cannot tear across two control passes

**Notes:**
- Amplitude is a Q15 fraction of the MEASURED supply rail, not a fixed voltage, so a note holds its loudness as the bus sags and the same command is safe on a 12 V and a 48 V bus. The path is open loop - no current regulation - so amplitude is capped at 8192/32768 and frequency at 40-4000 Hz in the register accessors, and notes outside 80-2000 Hz are octave-shifted host-side rather than driven near DC into a winding that looks like a short. Tone mode publishes tp_flash_programming_window = false because the bridge is energized. Register table re-verified sorted at 72 entries. REQUIRES A REFLASH. Not yet run on hardware.

FLASH: 99312 bytes (75.77% of 131072 bytes): +1020 bytes
RAM: 11248 bytes (34.33% of 32768 bytes): +48 bytes


## 2026-09-05 - Tone mode was silent: the carrier amplitude was being set to zero
**Changes:**
- Pass full scale as MotorSpeaker::tick()'s amplitude argument instead of 0
- Clamp the tone_freq register to 2000 Hz, matching FrequencyGenerator's own ceiling

**Reasoning:**
- MotorSpeaker::tick(freq, amp) treats its second argument as the generator amplitude, and any value >= 0 overrides the internal one, so passing 0 set the oscillator amplitude to zero and the output vector was identically zero - the later voltage scaling then multiplied zero and the motor stayed silent
- The register accepted up to 4000 Hz but set_frequency_target() rejects anything above 2000 Hz, so notes in between were dropped with no error

**Notes:**
- Verified natively by compiling buzzer.h on the host and measuring the waveform: the fixed path gives peak 32766 at 80/220/440/785/2000 Hz (all within 5% of the requested pitch) and exactly zero at 0 Hz, while the old amp=0 path gives peak 0 at every frequency. Also measured the control-loop rate through TP_REG_CONTROL_TICKS at 20000 passes/s, which confirms the tone generator's sample rate assumption and contradicts the heartbeat tick/s field - the heartbeat display is unreliable, the loop is not. REQUIRES A REFLASH.

FLASH: 99312 bytes (75.77% of 131072 bytes): +0 bytes
RAM: 11248 bytes (34.33% of 32768 bytes): +0 bytes


## 2026-09-05 - music command plays an arbitrary window of a MIDI file
**Changes:**
- parse_midi_melody() takes start and end seconds instead of a length
- New --start and --end options, with --seconds kept as the duration used when --end is omitted
- A note still sounding when the window opens is emitted at t=0 rather than lost
- Renamed the track-boundary local that was shadowing the new end parameter

**Reasoning:**
- Picking a passage matters more than the opening for a 156 s arrangement
- Skipping events before the window outright would drop a note struck earlier and still held, so the note set is tracked across the whole file and only emission is gated

**Notes:**
- The shadowing was caught by a dry run, not by inspection: the track loop already bound end to the track byte offset, so the window end silently became the file length (11223, the file size in bytes) and every window played the entire piece. Verified on the real file: 0-30s gives 295 notes, 30-60s 237 notes starting on the held 440 Hz, 60-75s 116 notes, whole file 1459 notes, each ending exactly at its requested length. Host tool only, firmware unchanged.

FLASH: 99312 bytes (75.77% of 131072 bytes): +0 bytes
RAM: 11248 bytes (34.33% of 32768 bytes): +0 bytes


## 2026-09-05 - MIDI playback: group chords, drop inaudible events, and follow the melody voice
**Changes:**
- All MIDI events sharing a tick are applied before the top note is read, instead of one event at a time
- Schedule entries closer together than 20 ms replace the previous entry rather than queueing another write
- The melody channel is auto-detected as the most monophonic voice and everything else is ignored
- New --channel option to override that choice

**Reasoning:**
- A chord is several note-ons at one instant and the previous chord's note-offs share it, so reacting per event made the top note flicker through intermediate pitches and dip to silence between chords - heard as a stutter, and one register write per event with a median gap of 0 ms
- Taking the highest note across all voices masks the melody whenever a held accompaniment chord sits above it, which is most of the time in a keyboard arrangement because the hands overlap in pitch

**Notes:**
- Found with our-last-summer-abba.mid, which splits melody and accompaniment across two channels: ch0 has 70 onsets in the first 30 s of which 1 is a chord, ch1 has 29 onsets all of which are chords. Pitch cannot separate them - ch0 spans 57-74 and ch1 spans 37-71 - so the voice is chosen by how monophonic it is. That file went from 20 notes per 30 s to 71, matching its onset count
- the single-channel Stevie Ray Vaughan file is unchanged at 142 notes but lost its 0 ms duplicate entries. Host tool only, firmware unchanged.

FLASH: 99312 bytes (75.77% of 131072 bytes): +0 bytes
RAM: 11248 bytes (34.33% of 32768 bytes): +0 bytes


## 2026-09-05 - music --channel collided with the global transport option
**Changes:**
- Renamed the MIDI channel selector to --voice, restricted to 0-15

**Reasoning:**
- The global --channel names the transport (serial port or HID device path) and main() overwrites it with the detected device before dispatching, so a subcommand redefining the same option received the device path instead of the user's value
- The MIDI filter then matched no events and the command played silence while reporting a channel of b'DevSrvsID:...'

**Notes:**
- Caught from a user's session output, not from a test: both parsers accept the option so nothing errors, and the only symptom is 0 notes and a nonsense channel in the summary. Verified by running the exact command line through the real argument parser: --start 30 --end 60 now yields 65 notes from channel 0, and --voice 1 selects the accompaniment. A scan of the file confirms --channel was the only subcommand option shadowing a global one. Host tool only, firmware unchanged.

FLASH: 99312 bytes (75.77% of 131072 bytes): +0 bytes
RAM: 11248 bytes (34.33% of 32768 bytes): +0 bytes


## 2026-09-05 - Calibration can be forced to sweep instead of reusing the stored table
**Changes:**
- New TP_CALIB_FLAG_FORCE (bit 1 of the CALIB_START flags byte)
- AngleCalibrator::calibrate() takes a force argument that skips the same-motor fast path
- Host tool gains calibrate --force

**Reasoning:**
- A drive whose motor profile is loaded reloads its saved angle table by Cal-ID and returns to Ready without turning the shaft, which is the right default but left no way to ask for a fresh measurement
- The request was acknowledged and nothing moved, which is indistinguishable from a command that did nothing
- Re-measuring matters whenever the mechanics change - a new coupling, a moved magnet - and the only alternative was erasing the stored profile

**Notes:**
- Found from the GUI: an angle calibration ACKed on hardware and the shaft never moved. Confirmed against the drive that kinds 0, 1 and 2 are all acknowledged while kind 3 (full) and abort return not-implemented. The flags byte was already defined in TpCalibCmd and never read. Electrical identification has no such fast path and always runs. REQUIRES A REFLASH. The forced sweep has NOT been run on hardware - it rotates the shaft, so it is left for the operator to trigger.

FLASH: 99400 bytes (75.84% of 131072 bytes): +88 bytes
RAM: 11248 bytes (34.33% of 32768 bytes): +0 bytes


## 2026-09-05 - MCP server exposing the drive to an assistant
**Changes:**
- Added tools/tpcp_mcp.py: a stdio Model Context Protocol server with 27 tools over the same TPCP link the host tool uses
- Structured JSON decoders for every status chunk, alongside the CLI's text rendering
- A --read-only mode that withholds the 20 mutating tools rather than refusing them at call time
- confirm=true gates on erasing the configuration and rebooting into DFU
- Registered the drive's register map and CAN_PROTOCOL.md as MCP resources

**Reasoning:**
- An assistant needs the fields, not a printed line, and shelling out to the host tool re-enumerated USB and re-probed the port on every command - most of a second each time, which makes polling useless
- One long-lived link turns a status read into a millisecond and lets a MIDI clip play on a background thread while status reads interleave

**Notes:**
- Host tool only, firmware unchanged. Implemented against the MCP specification directly rather than through the reference SDK, so the only imports are the standard library plus tpcp_host: the hidapi bindings live in PlatformIO's own virtualenv and requiring an SDK install into it is a bigger imposition than the protocol code. stdout is rebound to stderr at startup because tpcp_host prints freely and stdout carries JSON-RPC. Verified against hardware over HID: handshake, tools/list, resources, ACK and NACK decoding, both confirm gates, telemetry configure -> monitor -> disable, and every tool schema checked against its implementation signature. Found while probing: an undefined opcode (0xEE) sent on the request channel is accepted as a motion segment and ACKed as opcode 0x19 instead of being NACKed as unknown - one segment was queued and then cleared
- the drive was left idle and disabled

FLASH: 99400 bytes (75.84% of 131072 bytes): +0 bytes
RAM: 11248 bytes (34.33% of 32768 bytes): +0 bytes


## 2026-09-06 - Split src/api into concern-sized files
**Changes:**
- tp_wire.h split four ways into wire/ (identifier map and predicates, opcodes, status enums, fixed-point codecs) and kept as an umbrella
- tp_chunks.h split by opcode family into chunks/ with union TpFrame left in the umbrella
- tp_node.h split into node/ as config record, identity, flash store and discovery
- tp_registers.h split into registers/ with the 446-line TP_REGISTERS table in its own file
- TpProtocol method bodies moved out of the class into nine engine/*_impl.h files as out-of-class inline definitions, leaving tp_engine.h as declarations plus state
- tp_can.h, tp_hid.h and tp_serial.h moved to transport/ with run.h repointed
- CAN_PROTOCOL.md section 13 implementation map rewritten for the new layout

**Reasoning:**
- tp_engine.h was one 1849-line class in which pack_chunk alone ran 227 lines and handle_request 165, so no single file answered a question like how persistence works without scrolling past telemetry packing and motion queueing
- the register table was 59 percent of tp_registers.h and TpNodeStore 49 percent of tp_node.h, so both files read as one concern buried under another
- largest file drops from 1849 to 523 lines and that 523 is now pure declarations, which doubles as the index to the nine engine files

**Notes:**
- Pure code motion with no behaviour change - all 2747 symbol sizes in the map file are identical to the pre-refactor build and flash and RAM are unchanged
- every old top-level header survives as an umbrella so existing includes keep working, and the only edit outside src/api is run.h repointed at api/transport/tp_can.h
- the register table stays one sorted array because tp_find_register binary-searches it and verify_register_table asserts the ordering at boot
- tp_chunks.h still compiles standalone with no include path and reaches nothing under src/lib, so native_tpcp needed no change and passes 15 of 15

FLASH: 99512 bytes (75.92% of 131072 bytes): +0 bytes
RAM: 10816 bytes (33.01% of 32768 bytes): +0 bytes


## 2026-09-06 - HFI: probe amplitude linearity within each stage
**Changes:**
- Split each stage's measurement window in two - the first half at the frozen amplitude, the second at half of it - capturing the full-amplitude demodulation before the step and logging CAL Znl hi,lo with the impedance magnitude from each
- R and L still come from the full-amplitude half

**Reasoning:**
- The |Z| ratio across the two frequencies exceeds the 1.5 bound a series R-L allows, and the excess tracks the injection amplitude: 1.6 percent at I = 0.06 A rising to 5.5 percent at I = 0.34 A, reproduced across runs (1.576 and 1.586 on the two axes at I = 0.3 A). Frequency and amplitude are confounded in the four main stages because the higher-frequency measurement always runs at higher amplitude, so this measures the two amplitudes at one frequency, on the same winding seconds apart, where a linear system must return identical |Z|

**Notes:**
- Implemented inside the existing stages rather than as a new MotorStatus, which would change the protocol stage codes. The step lands mid-window and accumulation restarts at that period boundary, discarding the settling transient - the electrical time constant is under a millisecond against a window over a hundred milliseconds. Halves the averaging time for the primary measurement

FLASH: 99752 bytes (76.10% of 131072 bytes): +240 bytes
RAM: 10832 bytes (33.06% of 32768 bytes): +16 bytes


## 2026-09-06 - HFI: switch frequency mid-window to separate frequency dependence from drift
**Changes:**
- Repurpose the in-stage probe: the second half of each measurement window now runs the other injection frequency at the same amplitude instead of half amplitude, logging CAL Zf 1k,1k5 with the pair ordered by frequency regardless of which the stage ran first

**Reasoning:**
- The amplitude probe answered its question - the Q axis returned |Z| within 0.9 and 1.5 percent between full and half injection, and the frequency ratio held at 1.548 versus 1.538, so |Z| does not depend on amplitude and the earlier cross-run correlation with current was confounded. That leaves one untested confound: all four stages measure f1 before f2, so any drift over a run - winding heating, holding current settling, adaptation state carried in - aliases exactly into an apparent frequency dependence

**Notes:**
- A ratio that persists when both frequencies are measured about a hundred milliseconds apart inside one stage is genuinely frequency related
- one that collapses was drift. D2 read a 23.8 percent amplitude difference but its half-amplitude window ran at about 0.085 A against the DC bias, which is SNR rather than physics

FLASH: 99808 bytes (76.15% of 131072 bytes): +56 bytes
RAM: 10832 bytes (33.06% of 32768 bytes): +0 bytes


## 2026-09-09 - BSP layer: one translation unit per board and six shared STM32G4 drivers
**Changes:**
- src/target and src/hardware move to src/bsp as boards/, chips/ and stm32g4/
- each board gains a board.cpp owning its interrupt vectors, HAL weak overrides, naked NMI handler and the TP object
- main.cpp includes only bsp/board.h and names no board
- CORDIC, TIM1 PWM, bootloader jump, NVM flash, comparators and FDCAN are shared once in bsp/stm32g4 and parameterized through each board's target.h
- bsp/stm32g4/chip.h carries the G431/G474 die differences
- tools/mapdiff.py added to prove a refactor changed no symbol sizes

**Reasoning:**
- The firmware was a single translation unit rooted at main.cpp, so vectors and mutable globals sat in headers and any second .cpp would have produced duplicate symbols
- the two boards carried the same peripheral code at 79 to 100 percent similarity and had already drifted, leaving CLN234 with a stale NVM layout and a pre-protocol CAN driver

**Notes:**
- CLN17 v3 is byte-identical apart from two 8-byte entry-point forwarders. CLN234 gains 340 bytes of flash and 68 of RAM, all inside CANDriver, from adopting the protocol-capable FDCAN driver. Clock, ADC, GPIO and SPI stay board-specific: their two versions encode real hardware differences rather than duplication. FLASH_SIZE is defined by the HAL as a runtime density read, so declaring it in a descriptor silently moves every NVM partition
- do not.

FLASH: 98432 bytes (75.10% of 131072 bytes): -1376 bytes
RAM: 10764 bytes (32.85% of 32768 bytes): -68 bytes


