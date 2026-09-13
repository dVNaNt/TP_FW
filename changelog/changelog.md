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


## 2026-09-05 - Reject invalid HFI inductance instead of committing it
**Changes:**
- hfi_axis_spread() reports 1.0 when either per-frequency estimate is missing, replacing the inline guard that returned 0.0 (perfect agreement) exactly when a stage failed
- final HFI step aborts via set_error() when either axis spread exceeds HFI_MAX_SPREAD_RATIO, instead of averaging a zero into the committed inductance
- added HFI_MIN_VALID_INDUCTANCE_UH and HFI_MAX_SPREAD_RATIO constants

**Reasoning:**
- A failed HFI stage stored L=0 and was averaged with the good frequency, committing an inductance up to 4x too low
- current-loop kp = L * omega_c scales with L while ki = R * omega_c does not, so the PI zero no longer cancels the winding pole and the loop limit-cycles with the motor energized
- the old spread guard reported 0.00 disagreement in exactly the failure case, so nothing flagged the corruption and the bad profile was saved to system_dump and restored on later boots

**Notes:**
- Behavior change: a motor whose HFI measurement fails now ends in CalibError and does not arm, where it previously armed with unstable gains. tunepulse_init() already breaks its wait loop on CalibError and tick() commands zero current in that state, so boot completes. Spread threshold 0.30 against 0.14 observed on a healthy run.

FLASH: 68944 bytes (52.60% of 131072 bytes): +1188 bytes
RAM: 7436 bytes (22.69% of 32768 bytes): +8 bytes


## 2026-09-05 - Fix truncated diagnostic logging (TP_LOG_VALS label/value split)
**Changes:**
- TP_LOG_VALS and TP_LOG_ARRAY now room-check the label instead of printing it with a bare unguarded SerialUSB.print()
- TP_LOG_VALS reserves label + TP_LOG_VALS_PAYLOAD (48 B) so a line is emitted whole or not at all
- _tp_print_args and _tp_print_array emit the terminating newline even when the value tail is dropped
- CDC_TRANSMIT_QUEUE_BUFFER_PACKET_NUMBER raised from the framework default 2 to 16, enlarging the CDC TX queue from 128 B to 1 KiB

**Reasoning:**
- The CDC TX queue was 128 B (64 B FS packet x 2) while a single ELEC CAL warning label is ~84 chars, so the label alone left too little room for its own values and every diagnostic number was dropped behind it
- the dropped tail also swallowed the newline, so consecutive warnings concatenated into one unreadable line
- the unguarded label print contradicted the header's own ISR-safety contract and could spin USBSerial::write() inside the 20 kHz control ISR, latching the last voltage vector on the bridge

**Notes:**
- Instrumentation fix only - does not change calibration behaviour. Needed because the HFI failure diagnosis depends on the i_peak/v_peak/Z/R values that were being dropped. RAM +896 B is the enlarged CDC queue.

FLASH: 69476 bytes (53.01% of 131072 bytes): +532 bytes
RAM: 8332 bytes (25.43% of 32768 bytes): +896 bytes


## 2026-09-05 - Reject implausible two-point resistance corrections
**Changes:**
- Bound the resistance-correction factor I1/(2*dI) by RES_CORRECTION_MAX_FACTOR (1.0)
- a factor above it keeps the uncorrected V/I estimate instead of inflating R
- log the rejection with factor, I1, I2, dI and R_unc
- always log the two-point fit inputs and both R values

**Reasoning:**
- On a board with an erased system_dump the correction inflated R to 14.91 Ohm against a raw V/I estimate near 2.5 Ohm, a factor of ~6
- with a series offset drop Voff the factor evaluates to (V1-Voff)/V1 and is <= 1 by construction, so a factor above 1 means dI collapsed into the noise floor rather than a real measurement
- the inflated R then made every HFI stage compute Z^2-R^2 < 0 and report no inductive part, failing the whole identification

**Notes:**
- Diagnosed from HFI logs: Z=9.55 Ohm at 1 kHz and 14.77 Ohm at 1.5 kHz against the stored R=14.91 Ohm. Solving Z2^2-Z1^2 = L^2(w2^2-w1^2) gives L ~1604 uH, matching the 1585/1630 uH measured on this motor when a stored profile was present. Root cause of the dI collapse itself is not yet identified - the added logging targets it.

FLASH: 69744 bytes (53.21% of 131072 bytes): +268 bytes
RAM: 8332 bytes (25.43% of 32768 bytes): +0 bytes


## 2026-09-12 - Merge main into bsp_layer_shared_stm32g4 (PR #5 prep)
**Changes:**
- Resolved conflicts in platformio.ini, changelog.md and cln17_v3 spi_setup.h
- Kept the CDC TX queue 16-packet flag from main
- Kept the BSP ISR-in-board.cpp architecture
- Ported the encoder fault setter into bsp/boards/cln17_v3/board.cpp

**Reasoning:**
- Integrate main's logging and encoder-fault changes without undoing the BSP restructure or the Windows build fix

**Notes:**
- -save-temps/-fverbose-asm stay dropped (Windows toolchain break)
- STATUS bits now both feed the getter and raise TP.encoder.set_fault

FLASH: 99520 bytes (75.93% of 131072 bytes): +29776 bytes
RAM: 11660 bytes (35.58% of 32768 bytes): +3328 bytes


## 2026-09-12 - Repo hygiene and build speed
**Changes:**
- Ignore .vscode/ and remove it from git tracking
- Enable PlatformIO build cache (build_cache_dir)
- Add toolchain and build-cache caching to CI
- Add tools/protect_main.ps1 to enforce required CI checks on main

**Reasoning:**
- Faster rebuilds between environments and CI runs
- Prevent merging branches that do not compile

**Notes:**
- Run tools/protect_main.ps1 -Token <PAT> once to activate merge protection on GitHub

FLASH: 99520 bytes (75.93% of 131072 bytes): +0 bytes
RAM: 11660 bytes (35.58% of 32768 bytes): +0 bytes

## 2026-09-11 - USB-to-CAN gateway: one USB cable reaches every node on the bus
**Changes:**
- tp_can.h: gateway section between the transport callbacks and the multiplexer - relays every master->bus frame a USB host sends onto CAN (except frames directed at the USB node itself) and every node->master frame received from CAN back to both USB transports, withholding those from the local engine
- Gateway arms itself on the first protocol frame heard over HID or serial, latched until reset, and never arms under TP_BENCH_LOOPBACK
- tp_can_bind_filters() keeps the promiscuous slot open while the gateway is armed, whatever the stored flag says
- tp_mux_read() relays at most TP_GATEWAY_UP_BUDGET (8) bus frames per call so a bus storm cannot starve the engine's poll budget
- Relay counters tp_gateway_down / up / lost, shown in the console heartbeat as gw[on= down= up= lost=]
- tools/tpcp_host.py HidLink.recv(): return when the wait window has elapsed instead of calling hid.read(timeout_ms=0), which the hidapi binding treats as a blocking read - this is what hung the MCP scan and connect tools against a silent drive
- CAN_PROTOCOL.md Transports section and CLAUDE.md describe the gateway

**Reasoning:**
- A USB host could only ever talk to the drive on the cable: the multiplexer answers from the local engine and nothing reaches the bus, so a second drive on the USB node's CAN connector was unreachable without a separate CAN adapter
- Auto-arming on the first USB frame needs no address and no register write first, so the discovery sweep that finds the bus nodes is itself what arms it, and a deployed node that never hears USB pays nothing
- Only master->bus classes go down and only node->master classes come up, so the two directions cannot form a loop and responses still return on the transport that asked
- Discovery needs no host change because cmd_scan already takes the minimum of the replies it collects per step - the bus arbitrates its own nodes, the USB node adds its own while unaddressed

**Notes:**
- Verified on hardware: USB node (UID dae8172daab933, node 1) relayed a sweep that found and addressed the bus node (UID a5491e4b82efe1, node 2)
- status, identity and comm_status of node 2 read through the gateway, TEC/REC 0 on both
- Frames go down with DLC 8 and are dropped (counted in tp_gateway_lost) when the 8-deep CAN TX ring is full - a HID-speed burst can outrun a classic 1 Mbit/s bus, pacing is the host's job
- Pre-existing, not fixed here: the text heartbeat in run.h never prints because tp_log_room(200) can never pass against the 128-byte CDC TX queue (2 x 64)
- it has been dead since 9d3b7f2

FLASH: 98976 bytes (75.51% of 131072 bytes): +544 bytes
RAM: 10780 bytes (32.90% of 32768 bytes): +16 bytes


## 2026-09-11 - MotionExecutor: leaving HALT no longer swallows the first queued segments
**Changes:**
- motion_executor.h do_forward_transition(): after entering the first segment out of HALT, return unless needs_forward_transition() says it finishes on the spot (ANCHOR, or a hold with a queued successor) - previously the loop fell straight into the segment-complete branch
- Host simulation of the executor (scratch, not committed) reproduced the fault and confirmed the fix for a pre-queued leg, a trickle-fed leg, a second leg after HALT and an ANCHOR basis shift

**Reasoning:**
- The segment just started was treated as complete in the same pass: its displacement was folded into the anchor untravelled and its successor started with a start time one duration in the future, so the wrapped elapsed time completed every following segment one tick apart
- With a whole leg pre-queued the target jumped to the end of the trajectory within a few ticks
- With a leg trickling in over a HID burst the chain ate the 2-3 segments already queued, then halted, re-anchored to the measured position and played the rest at the right pace - a full-revolution leg came out 2 segments short (14/16, 6/8) with no visible jump, which looks exactly like lost frames and was chased through the HID ring, the mailbox and the gateway before the executor was suspected

**Notes:**
- Verified on hardware: 1 rev over 6 s as 12 PVT segments lands at +0.9995 rev on the reflashed node
- A board still on the old firmware can be worked around by queuing one zero-displacement 10 ms hold before the leg - the executor halts with an active segment and later arrivals take the normal path (validated against the old executor on the host, +0.9963 rev on hardware)
- Only the USB-attached board was reflashed - the second drive sits on CAN with no ST-Link or USB, so it still runs the old executor

FLASH: 99192 bytes (75.68% of 131072 bytes): +216 bytes
RAM: 10780 bytes (32.90% of 32768 bytes): +0 bytes


## 2026-09-11 - Runtime electrical identification runs; R/L edits re-tune the loop; measured currents and sense offsets exposed
**Changes:**
- lib.h tick(): a pass with CurrentControl::is_identifying() true bypasses the drive front end (serviced inhibited) and falls through to tick_control_foc() with a zero target, so an identification started over the protocol actually runs
- on its completion the front end is re-armed from the measured position (the slow-rotation stage moves the shaft)
- current_control.h: tick_control_foc() adopts the caller's angle only while no identification is running (keyed on motor_calib_status, not Motor::status_), new is_identifying() and retune()
- tp_register_table.h: resistance / inductance_d / inductance_q are TP_RF_DISABLED and apply through retune(), guarded by tp_reg_can_retune() (no identification, angle table present)
- MOTOR_PROPERTIES write applies the same guard and re-tunes
- New read-only registers 0x050D-0x0513: measured d/q current, measured alpha/beta current, per-phase zero-current ADC offsets (CurrentSense::get_zero_offset())
- CAN_PROTOCOL.md: kind-1 semantics, RW D on R/L, new register rows
- tpcp_host.py REGS
- CLAUDE.md register count corrected to 79

**Reasoning:**
- TP_CALIB_ELECTRICAL was ACKed but never ran: handle_calibration() disables the drive, so tick() took the Freewheel early return and tick_control_foc() - where MotorCalibration is advanced - was never reached
- the routine stayed armed and ran inside whatever mode was next enabled, with the encoder angle overwriting its test vector every pass, and ended in CalibError plus a phase fault (seen on hardware)
- Writing R or L only changed the profile: nothing called init_foc_controller() again, so the loop kept its old gains until the next identification
- No chunk or register reported a measured current - electric_status.current is the commanded value - so a sense offset or gain fault could not be diagnosed over the link

**Notes:**
- Motivation: two identical motors, one drive identifies R=1.1-1.6 ohm / L~1.9 mH at boot, the other 7 ohm then 29 ohm with L->0 on successive boots and vibrates on the resulting gains
- the new registers and the working runtime identification let that board be measured and re-tuned without reflashing or rebooting
- Not yet flashed: the ST-Link is off the target
- Untested on hardware until then

FLASH: 99756 bytes (76.11% of 131072 bytes): +564 bytes
RAM: 10780 bytes (32.90% of 32768 bytes): +0 bytes


## 2026-09-12 - Build at -Os: PlatformIO's debug build type was silently discarding -O0 and compiling at -Og
**Changes:**
- platformio.ini: debug_build_flags = -Os -g2 -ggdb2 in cln17_v3_release and cln234_release (comment explains why it must live there)
- the inert -O0 lines removed from both build_flags

**Reasoning:**
- build_type = debug makes PlatformIO strip every -O*/-g* flag - the ini's -O0 AND the framework's default -Os - and inject -Og -g2 -ggdb2, confirmed from the DWARF producer string of board.cpp.o
- the firmware is one translation unit of header-only C++ templates, where -Og vs -Os is worth 18.6 KiB
- a true -O0 image overflows flash by 68484 B and -O2 by 1180 B, so neither is an option

**Notes:**
- Measured baseline on this exact tree before the change: 102008 B flash / 11152 B RAM (the previous entry predates later uncommitted edits)
- pio debug keeps full DWARF, debug info costs no flash
- first step of the flash-size series (next: unused UART, dev-log gating, soft-double, LTO)
- needs a bench pass - heartbeat tick/s ~20000, a calibration run, TPCP over USB HID and CAN

FLASH: 83412 bytes (63.64% of 131072 bytes): -16344 bytes
RAM: 11152 bytes (34.03% of 32768 bytes): +372 bytes


## 2026-09-12 - Drop the unused HardwareSerial / UART stack from the image
**Changes:**
- platformio.ini: -D HAL_UART_MODULE_ONLY next to the TIM/ADC _ONLY flags in both live envs, with a comment explaining what it removes

**Reasoning:**
- Nothing in src/ opens a UART - the console is USB CDC (SerialUSB) - but the Arduino core's serialEventRun() constructs SerialLP1 and keeps HAL UART, uart.c, HardwareSerial and the Stream/String parsers linked: 5.5 KiB of flash and 372 B of RAM for a port that never runs

**Notes:**
- mapdiff against the -Os map: 279 symbols removed, all UART / USART IRQ stubs, HardwareSerial, Stream, String and the libc string/float parsers they pulled in, nothing added
- PIO_FRAMEWORK_ARDUINO_SERIAL_DISABLED measures identically, the _ONLY spelling matches the existing flags
- the HAL UART driver still compiles, only the Arduino wrapper is gone
- native_tpcp: 15/15 pass

FLASH: 77836 bytes (59.38% of 131072 bytes): -5576 bytes
RAM: 10780 bytes (32.90% of 32768 bytes): -372 bytes


## 2026-09-12 - Console output is a build flag: DEV_LOG_ENABLE off in cln17_v3_release, new cln17_v3_dev env keeps it
**Changes:**
- dev_log.h: the hard-coded #define DEV_LOG_ENABLE 1 removed, the switch is -D DEV_LOG_ENABLE=1 from platformio.ini
- platformio.ini: new [env:cln17_v3_dev] = release + DEV_LOG_ENABLE, cln17_v3_bench also defines it (its loopback check reads the heartbeat)
- run.h: heartbeat (constants, statics, print_heartbeat and its call), the boot banner and the wait for the host to open the CDC port sit inside #ifdef DEV_LOG_ENABLE, the two remaining status lines use TP_LOG
- tunepulse.h: five bring-up messages through TP_LOG
- board.cpp: irq_log() body inside #ifdef DEV_LOG_ENABLE
- comp_dac.h: the four threshold echoes through TP_LOG_VAL
- file headers and comments updated

**Reasoning:**
- Logging was hard-enabled in the header, so no build could ever drop it: 71 TP_LOG call sites, 26 _tp_print_args template instantiations, print_heartbeat (70 raw prints, the only float print), the banner and ~5 KiB of string literals shipped in every image
- turning the flag off did not even compile because run.h and board.cpp called tp_log_room() outside the macros

**Notes:**
- cln17_v3_release 65760 B, cln17_v3_dev 77976 B, cln17_v3_bench 77972 B, all warning-free
- printable strings in the release image 5614 -> 777 B
- the release image no longer waits up to 5 s for a CDC host at boot (that wait only served the banner) - TPCP over CDC, HID and CAN is unchanged
- native_tpcp 15/15

FLASH: 65760 bytes (50.17% of 131072 bytes): -12076 bytes
RAM: 10772 bytes (32.87% of 32768 bytes): -8 bytes


## 2026-09-12 - No more soft-double: float literals, an int32 velocity step, lrintf instead of rintf/lroundf, -fno-math-errno
**Changes:**
- voltage.h: 1000.0 -> 1000.0f and 0.001 -> 0.001f (the only double arithmetic in the firmware)
- ema.h set_cutoff(): 6.283185307f instead of Arduino's double TWO_PI macro
- drive_control.h Velocity mode: the per-pass position step is converted float -> int32_t and widened, instead of a direct float -> int64_t cast
- pll.h: e -= (float)lrintf(e) in place of rintf() (twice)
- hermite_interpolator.h: __builtin_lrintf instead of __builtin_lroundf for the two coefficient roundings
- platformio.ini: -fno-math-errno in both live envs

**Reasoning:**
- A float -> int64_t cast is __aeabi_f2lz, which libgcc implements with double arithmetic, so that one cast in DriveControl::tick kept _arm_muldivdf3 and _arm_addsubdf3 (2 KiB) alive together with the two double literals in voltage.h
- the M4F has no VRINT, so rintf() is always a libm call while lrintf() inlines to vcvtr
- sqrtf() inlines to vsqrt only when it need not set errno, and nothing here reads errno

**Notes:**
- nm: no __aeabi_d*, __adddf3, __muldf3, __divdf3, __aeabi_f2lz, __ieee754_sqrtf or rintf left in the image
- a -Wdouble-promotion build of src/ reports nothing
- the int32 step covers |v| < 10000 rev/s (vel_to_pos_q32_ = 2^32 / 20 kHz)
- lrintf vs lroundf differ only on exact .5 ties of the Hermite coefficients
- __udivmoddi4 (746 B) remains for hermite_interpolator.h's one 64-bit reciprocal - left alone, optional
- RAM -392 B is the errno / reent plumbing

FLASH: 63356 bytes (48.34% of 131072 bytes): -2404 bytes
RAM: 10380 bytes (31.68% of 32768 bytes): -392 bytes


## 2026-09-12 - Link-time optimisation on: -flto=auto, NMI trampoline marked used
**Changes:**
- platformio.ini: -flto=auto in both live envs
- nvm_flash.h: __attribute__((used)) on flashg4_ll_nmi_dispatch with a comment explaining why it is load-bearing

**Reasoning:**
- With one firmware TU plus 20 KiB of separately compiled HAL and USB code, cross-object inlining and dead-code removal at -Os are worth 2.2 KiB
- the trampoline is referenced only from the naked NMI_Handler's inline asm, which LTO cannot see, so without the attribute the link fails with an undefined reference
- plain -flto emits an lto-wrapper warning about serial LTRANS jobs, =auto does not

**Notes:**
- Verified in the image: NMI vector -> NMI_Handler -> b.w flashg4_ll_nmi_dispatch
- the set of strongly defined interrupt vectors (TIM1_CC, DMA1_Channel1/5, FDCAN1_IT0, COMP1_2_3, COMP4, EXTI9_5, EXTI15_10, USB_LP/HP/WakeUp, SysTick, NMI) is identical to the pre-LTO build - only HAL-internal PCD_EP_ISR_Handler was inlined
- cln17_v3_dev 72112 B, cln17_v3_bench 71904 B
- LTO is the one change here that can alter cross-TU inlining, so it went in last and needs its own bench pass

FLASH: 61140 bytes (46.65% of 131072 bytes): -2216 bytes
RAM: 10264 bytes (31.32% of 32768 bytes): -116 bytes


## 2026-09-12 - platformio.ini hygiene, changelogger budget, CLAUDE.md build-flag notes
**Changes:**
- platformio.ini: removed -D ARM_MATH_CM4 and -D __FPU_PRESENT=1U (arm_math.h is never included
- the device header defines __FPU_PRESENT), -D USBCON (the framework script adds it with USBD_USE_CDC when CDC is enabled), -D HAL_Module_Enabled (no such symbol anywhere in the framework) and lib_deps = migration (no such library
- .pio/libdeps holds only Unity)
- changelogger.py: MAX_FLASH = 114688 so its percentage matches PlatformIO's
- CLAUDE.md: environments, reference build, the build flags that decide the size, and the DEV_LOG_ENABLE rule in the Logging section

**Reasoning:**
- Each of these knobs suggested a dependency or a capability that does not exist and would mislead the next size investigation
- the two percentage bases disagreeing was documented as a known wart

**Notes:**
- mapdiff against the previous map: 0 added, 0 removed, 0 resized - a pure no-op for the image
- envdump confirms USBCON, USBD_USE_CDC and HAL_PCD_MODULE_ENABLED are still injected by the framework
- cln17_v3_dev 72112 B, cln17_v3_bench 71904 B
- native_tpcp 15/15
- flash series total 102008 -> 61140 B (-40868 B, 53.3 percent of the 114688 B budget), RAM 11164 -> 10264 B

FLASH: 61140 bytes (53.31% of 114688 bytes): +0 bytes
RAM: 10264 bytes (31.32% of 32768 bytes): +0 bytes


## 2026-09-12 - cln17_v3_release is a real release build type; debug symbols and pio debug move to cln17_v3_dev
**Changes:**
- platformio.ini: cln17_v3_release and cln234_release use build_type = release (the framework's default -Os applies and nothing strips it), their debug_build_flags removed
- cln17_v3_dev is build_type = debug with debug_build_flags = -Os -g2 -ggdb2 and is the environment for pio debug
- CLAUDE.md: build commands, environment paragraph and the optimisation-level note updated

**Reasoning:**
- A shipping image must not come out of the debug build type
- the earlier step kept build_type = debug only to preserve DWARF for pio debug, which the dev environment now provides on the same -Os code

**Notes:**
- mapdiff between the debug-type and the release-type -Os images: 0 added, 0 removed, 0 resized - the same code byte for byte
- the release ELF shrinks from 900 KB to 102 KB because it carries no DWARF (flash is unchanged)
- cln17_v3_dev 72112 B, cln17_v3_bench 71904 B, cln234_release 50420 B of 520192
- the release image has no source-level debug symbols by design

FLASH: 61140 bytes (53.31% of 114688 bytes): +0 bytes
RAM: 10264 bytes (31.32% of 32768 bytes): +0 bytes


## 2026-09-12 - Rewrite the CAN API to the api_final.md register design (TPCP v1)
**Changes:**
- Identifier map: 0x100 emergency, 0x300 commands, 0x400 responses, 0x500 PulseBeat/telemetry
- node 0 broadcast, 255 unassigned
- FDCAN filters reprogrammed (own+broadcast dual slot, three master mask slots for the gateway)
- CMD byte with group bits: RW registers 0x00-0x3F (write = +0x80, answered with the read form), RO 0x40-0x7F, write-only 0xC0-0xDF, burst 0xE0-0xFF
- every register of the design implemented as a packed 8-byte layout (direct access, 0x01 axis/LED/gearbox/IO map/encoders/homing/limits sub-space, bus settings, discovery, system control, motor props, power/thermal/supply/current protection, motion and error limits, watchdog, calibration table, currents, device/system/motor/driver/motion status, encoder position, PulseBeat)
- added 0x02 telemetry slots, a 0x43 diagnostics sub-space and ACK/NACK at 0x5E/0x5F
- Driver Status moved from 0x45 (inside the sub-space block) to 0x58
- discovery as a binary HashUID search (48-bit identity) whose replies echo the matched prefix so competing nodes never collide
- multi-frame motion tasks assembled by SegmentID with AxisID filtering, single-frame setpoints (CSP/CSV/CST) that select their control mode, PVT time-tagged and PVcTF burst streams with masked absolute positions, S-curve/hold/freerun/anchor/limit/reset modes
- save config keywords (save, defaults, erase), MIDI control with attack/decay envelope, sequenced electrical-then-angle calibration, comm-mode response policy, FaultBeat escalation
- DriveControl: PushSegmentAbs and queue-tail tracking
- AngleCalibrator: is_running()/is_failed()
- CalibrationTable: get_sample()
- node config record grown to 256 B (version 2) serialized through a byte cursor
- tools/tpcp_host.py and tools/tpcp_mcp.py rewritten with a generic register field model (get/set by field name)
- test_wire.cpp rewritten (21 tests)
- CAN_PROTOCOL.md rewritten from the design with every gap marked [decision]
- CLAUDE.md updated

**Reasoning:**
- The api_final.md design replaces the opcode-based v1 with a register map that reads back on write, batches reads, streams synchronous motion and discovers nodes by HashUID
- the design's +1-bit discovery reply would collide in the CAN data field, which CAN does not arbitrate, so replies echo the matched prefix instead
- the fine-homing record overflowed a frame, so its trigger bit lives in the quick-homing flags
- 0x45 lies in the SubCMD block, so Driver Status is 0x58

**Notes:**
- cln17_v3_dev 79564 B, cln17_v3_bench 79284 B
- native_tpcp 21/21
- a -Wall -Wextra build of the dev env reports nothing in src/api
- the bench env now sets TP_DEFAULT_BUS_TYPE=0 instead of the removed TP_DEFAULT_LINK_FLAGS=4
- not bench-tested on hardware yet: discovery walk, burst timing and the axis inversion path need a bus/USB session
- stored config records of the old version 1 are ignored (the node comes back on defaults, unaddressed)

FLASH: 68948 bytes (60.12% of 114688 bytes): +7808 bytes
RAM: 11200 bytes (34.18% of 32768 bytes): +936 bytes


## 2026-09-12 - Identity read-modify-write no longer refused; CLN Studio ported to TPCP v1
**Changes:**
- tp_engine_regwrite_impl.h: Motor Properties 1 counts a type / wiring / direction / pole field as an identity write only when it differs from the current value, and Motor Properties 2 re-tunes the loop only when R or L differ from the reported bfloat16 - a read-modify-write that touches the mode or the thermal rating alone is accepted on an enabled drive
- CAN_PROTOCOL.md §5.8 / §5.9 say so
- CLN Studio (AndroidStudioProjects/CLNStudio) protocol layer, client, repository, view model, tests and the register / calibration / motion / tone / control / connection screens rewritten for the register map, binary HashUID discovery, batch reads, motion-task setpoints and MIDI control

**Reasoning:**
- Found on hardware from the Studio client: a mode change through Motor Properties 1 sent the identity back unchanged and the firmware refused it as an identity write, which made the mode unreachable on an enabled drive

**Notes:**
- Bench pass over USB HID against CLN17 v3 (motor supply absent, UV fault latched): broadcast ping finds the unassigned node and reports its HashUID, direct assignment confirmed on the new address, reset to 255 and the 48-probe tree walk recover the identity in 3.4 s, register read-modify-write of the watchdog reads back, the seven-register batch read completes, the mode change lands, enable / MIDI refused with not-calibrated, setpoints accepted silently and an S-curve segment queued, emergency PulseBeat on the fault edge
- observed: with the boot identification stalled by the UV fault no NVM window opens, so the assigned address is not persisted and the node comes back unassigned after a reset - not addressed here
- Studio: jvmTest 17/17, Android / WasmJs / iOS-simulator targets compile
- cln17_v3_dev 79964 B

FLASH: 69796 bytes (60.86% of 114688 bytes): +848 bytes
RAM: 11200 bytes (34.18% of 32768 bytes): +0 bytes


## 2026-09-12 - Ping inventory keyed by HashUID; overcurrent on the bench traced to calibration, not the protocol
**Changes:**
- tools/tpcp_host.py, tools/tpcp_mcp.py and CLN Studio TpcpClient.ping() key the broadcast identity replies by HashUID instead of node address

**Reasoning:**
- Two unassigned drives both answer on 0x4FF and collapsed into one inventory entry

**Notes:**
- Bench: a 0.05 rev position setpoint on the USB drive (node 1, NEMA17, 23 V) runs the rotor away against the target and trips the overcurrent comparator
- the pre-rewrite firmware (current tree with the old src/api swapped back in, built in a scratch copy) does exactly the same, and its forced angle sweep never completes, so the fault is the stored angle table / direction not matching this motor plus the sweep not converging - outside the protocol layer
- a runtime identification on the new firmware returned Ld 0.20 mH / Lq 0.44 mH against a stored 2.0 mH and the following sweep failed in 2.5 s
- a failed sweep leaves the drive holding 0.5 A at a fixed angle with no NVM window, so saves and erases are refused until reboot
- second drive (node 2) reached through the USB-to-CAN gateway is healthy and idle
- sizes unchanged

FLASH: 69796 bytes (60.86% of 114688 bytes): +0 bytes
RAM: 11200 bytes (34.18% of 32768 bytes): +0 bytes


## 2026-09-13 - Rotor electrical angle derived from the sweep direction; Hermite position widened to 64 bits
**Changes:**
- lib.h derives the FOC angle as -direction * table angle + a quarter period, replacing the fixed -90 degree shift inside CalibrationTable::correct_el_angle (which now returns the plain sweep-frame angle) and the bare -direction multiply in the control loop
- calibration_table.h documents why the alignment is direction dependent
- HermiteInterpolator::eval outputs the relative position as int64 Q32.32 and MotionExecutor adds it with a saturating add
- diag:calibration reports the angle sweep's CalStage (0x80 | stage) and a 0..255 progress whenever the electrical identification is not running (AngleCalibrator::stage / progress_255 / is_running / is_failed added)
- CalibrationTable::get_sample exposes stored samples to the 0x2F table read

**Reasoning:**
- The sweep holds the rotor with a q-axis current at the commanded angle in whichever sense makes the encoder count up, so the rotor angle in the FOC frame is a direction-dependent function of the table and the old fixed shift was 180 degrees off for direction = -1 motors
- the 32-bit Q0.32 Hermite output wrapped at half a revolution so any longer segment folded back on itself mid-way
- a bench needs to follow a sweep by stage and progress, not only by flag bits

**Notes:**
- Bench (NEMA17, 23 V, direction -1): with this build a position setpoint no longer trips the overcurrent comparator and 1 rev S-curves track for the first half revolution, but every queued move still ran away in the other direction - see the next entry for the actual root cause
- cln17_v3_dev 80276 B

FLASH: 70132 bytes (61.15% of 114688 bytes): +336 bytes
RAM: 11200 bytes (34.18% of 32768 bytes): +0 bytes


## 2026-09-13 - Position-loop torque sign follows the detected motor direction; queue holds when dry; encoder frames validated by CRC
**Changes:**
- lib.h multiplies the position / velocity / torque loop's current command by TunePulseHeart::torque_sign() (the sweep-detected direction, +1 when unknown) at the closed-loop dispatch only, keeps last_current_a_ in encoder units, and the 0x42/0x00 q current is reported with the same sign
- MotionExecutor::init starts in the HALT state and tick() reports halt whenever the executor is halted, so DriveControl keeps the full torque envelope on an empty or exhausted queue
- AngleCalibrator resets the sweep's electrical angle to zero at Setup
- Encoder::accept_frame keeps CRC pass / fail counters, drops failing frames once 256 consecutive frames have proven the check, and latches the encoder fault after 64 consecutive failures
- the CLN17 DMA handler checks each MT6835 burst frame against MT6835::Crc8 before publishing it
- diagnostics records 0x43/0x06 (FOC angle in use, u_d, u_q) and 0x43/0x07 (encoder link: check proven, fault, longest failure run, frames failed / passed) added
- CAN_PROTOCOL.md 6.3, tools/tpcp_host.py, tools/tpcp_mcp.py and CLN Studio (TpRegisters, CalibrationStatus sweep stage names, calibration screen) updated

**Reasoning:**
- With the electrical angle correct, a positive q current always advances the rotor in the sense the table's angle grows, which is the sense the encoder counts UP for a direction = -1 motor - and real_pos is -encoder, so the hard-coded sign made the position loop positive feedback: measured on the bench through the new 0x43/0x06 record, the FOC angle at the identification's d-axis alignment was within 6 degrees of zero yet every setpoint ran the rotor away at the voltage-limited speed with a small measured current
- a HALTed executor reported a zero torque limit so the axis went limp the moment its last segment finished, and a fresh queue started open-loop for the same reason
- a sweep that was aborted part way left its start angle behind for the next cycle
- a run of corrupted SPI frames (idle-high MISO decodes as the top of the range) walked the turn counter hundreds of turns from a stationary rotor and the position loop chased it

**Notes:**
- Bench (NEMA17, 23 V, direction -1, USB HID): hold, 0.05 rev position step (settles in 100 ms, error 0.0002 rev), S-curves 0.5 / 1 / 1 / 3 rev at 0.5 / 1 / 2 / 2 rev/s and -3 rev back all track within 0.002 rev and hold at the target in HALT, jog +-1 rev/s, absolute S-curve, verified again after a reboot and after a fresh sweep
- a forced angle sweep completes in 10 s, redetects direction -1 and its table matches the stored one within 28 counts (1.7 degrees electrical) - the stored calibration was never at fault
- runtime identification aligns the rotor at FOC angle 0 and the table then reads 930 counts, 5 degrees electrical
- the CRC check proves itself at boot and 4-7 isolated failures per boot are dropped
- unexplained: the previous build tracked correctly for one boot and ran away on every later boot with the same code, table and direction, which only a 180 degree change of the rotor-to-encoder relation between those boots accounts for - watch 0x43/0x06 at the identification alignment if it recurs
- still open: a runtime identification failed once while the encoder link was corrupted and leaves the drive in the error state until reboot
- the default motion limits (50 rev/s, 200 rev/s^2) let an absolute S-curve without a cruise velocity run at the voltage-limited speed
- cln17_v3_dev 80748 B

FLASH: 70388 bytes (61.37% of 114688 bytes): +256 bytes
RAM: 11216 bytes (34.23% of 32768 bytes): +16 bytes


## 2026-09-13 - Encoder link loss is a fault and heals itself; a fault reset recovers a failed identification
**Changes:**
- DriverPWM gains FAULT_ENCODER (bit 7, Hi-Z, priority below external) with enc_cb / encoder(bool)
- lib.h raises it every pass while Encoder::is_fault() reports the link down and the protocol's fault reset releases it
- Encoder::accept_frame declares the link DOWN after 64 consecutive failures, ingests nothing (not even CRC-passing frames) until 256 consecutive valid frames prove the recovery, and hands the owner a relink request once per 64 failures
- CLN17 Encoder_StartTransfer skips a period whose previous transfer is still in flight (CS still asserted) instead of cutting it, counting the overrun, and Encoder_Relink() disables the SPI, drains its receive FIFO and re-enables it when the encoder asks
- 0x43/0x07 now carries the sensor's own overspeed / weak-field / undervoltage bits and the relink count instead of the longest failure run
- CurrentControl::recover_identification() leaves CalibError with the last known parameters and is called from the fault reset
- service_calibration checks CalibError before is_identifying() so a failed identification no longer leaves the queued sweep pending forever
- fault bit 0x80 named encoder in CAN_PROTOCOL.md, tools/tpcp_host.py and CLN Studio

**Reasoning:**
- After the operator's full calibration plus save the drive was found with the encoder link dead (failure run saturated, 65535 failed frames), the turn counter at -1789 turns from a stationary rotor and the identification in CalibError: a loop chasing that position at full current is where the reported overcurrent comes from, and the CalibError state refused every enable until a reboot
- one corrupted frame in 256 passes the CRC by chance, enough on a persistently broken link to keep walking the turn counter
- restarting a transfer that is still in flight leaves a partial byte in the SPI FIFO and shifts every later frame, which is the most plausible origin of the persistent corruption
- a failed identification reporting itself as still identifying hid the queued sweep and locked the drive

**Notes:**
- Bench (NEMA17, 23 V, USB HID): after a reflash the boot identification recorded R = 3.0 ohm and a runtime identification aborts within 30 ms because the 45 degree pass reads Ia 0.67 A / Ib 0.49 A at 1.12 V (phase B about 0.45 ohm higher than phase A, balanced yesterday) - an intermittent motor phase connection, to be checked on the hardware
- with those gains a fault reset recovers the drive, and hold, 0.05 rev step, velocity +-1 / +-3 rev/s and S-curves up to 3 rev at 2 rev/s run clean with zero frame failures and no relink
- the relink path could not be provoked on demand and is verified by construction only
- cln17_v3_dev 81612 B

FLASH: 70764 bytes (61.70% of 114688 bytes): +376 bytes
RAM: 11224 bytes (34.25% of 32768 bytes): +8 bytes


## 2026-09-13 - Phase-balance tolerance scales with the current; identification failures and the current-sum residual are readable over the protocol
**Changes:**
- MotorControl::abc2ab_tick allows |sum| up to epsilon_abs + 8 % of |ia|+|ib|+|ic| (set_current_epsilon_relative, default BALANCE_REL_DEFAULT_PERCENT) instead of a fixed 0.13 A
- MotorCalibration records the reason, stage, driver fault byte, injected voltage and phase currents of its last abort (IdentFailure codes), abort() takes the fault byte from lib.h
- new diagnostics record 0x43/0x08 exposes that abort and 0x42/0x02 exposes the third sensed channel plus the Kirchhoff residual the supervision trips on (CurrentSense::channels_amps)
- CAN_PROTOCOL.md, tools/tpcp_host.py and CLN Studio register tables updated

**Reasoning:**
- With the console unavailable the identification kept aborting within 15 ms and nothing said why
- the new record showed reason 4 stage 1 fault 0x40 at 2.9 V with Ia 1.85 A and Ib 1.70 A: the phase-stage fault, i.e. the balance supervision, because 3.5 A on the shared channel with a few percent of amplifier and shunt gain mismatch exceeds a tolerance sized for standstill
- three shunts and three amplifiers each carry gain error, so a healthy motor's residual grows with the current and the tolerance has to follow it

**Notes:**
- Bench (NEMA17 rewired to J4 pin 4 BLK, 3 GRN, 2 RED, 1 BLU): with the fixed tolerance every runtime identification aborted and partial R values of 2.5 to 4.3 ohm were left behind, while a 110 Hz tone showed a residual of at most 0.1 A at 1.2 A
- with the relative tolerance the identification completes (R 1.078 ohm, Ld 2.1 mH, Lq 1.9 mH, identical to the day before), the angle sweep completes (direction -1, table 14), position step, +-1 and +-3 rev/s velocity and a 3 rev S-curve run clean with zero encoder frame failures, and address plus profile survive a reboot after a save
- the earlier suspicion of a bad phase B crimp was wrong - the unbalanced snapshots were taken during the current rise and the imbalance the identification reported never existed
- cln17_v3_dev 82100 B

FLASH: 71156 bytes (62.04% of 114688 bytes): +392 bytes
RAM: 11248 bytes (34.33% of 32768 bytes): +24 bytes


## 2026-09-13 - Angle sweep waits for flash, refuses a motionless Pass0 and a never-ending Pass1; a failed or unrequested sweep releases the rotor
**Changes:**
- AngleCalibrator holds in Setup while Motor::is_flash_busy() (the profile save that follows an identification de-energizes the sweep)
- Pass0 reports an error when the encoder travelled less than ANGLE_CAL_MIN_STEP_COUNTS per step instead of decoding a zero travel as direction +1
- Pass1 stops with an error when the table would overflow without a full turn
- lib.h commands zero current and opens the NVM window while the calibrator is in Error or parked without a request (AngleCalibrator::is_parked)

**Reasoning:**
- On a second CLN17 (HashUID EF824B1EE5E8) a runtime full calibration ran its sweep underneath the pending profile save: the rotor was not driven during Pass0, the zero travel became direction +1, Pass1 rotated the encoder the wrong way and waited forever for a full positive turn with the calibration flagged running - only a reboot got out
- a sweep that had failed used to hold 0.5 A at a frozen angle until reboot

**Notes:**
- Bench (second board, same NEMA17 wiring): boot table healthy (200 monotonic samples, deviation -23..+113), identification R 1.06 ohm Ld 2.09 mH Lq 1.85 mH
- with the guards a runtime FULL calibration completes (identification, then sweep with direction -1, table 14, profile saved), position step, +-1 / +-3 rev/s and a 3 rev S-curve track within 0.001 rev with no faults and zero encoder frame failures, and node address, profile and table survive a reboot
- the frame check via diag:foc after a runtime identification is only indicative (+80 deg once, -13 deg after boot) because the HFI stages do not park the rotor exactly on the d axis
- cln17_v3_dev 82348 B

FLASH: 71196 bytes (62.08% of 114688 bytes): +40 bytes
RAM: 11248 bytes (34.33% of 32768 bytes): +0 bytes


## 2026-09-13 - Rebase onto main: sensor-status encoder fault kept beside the CRC link check; HFI spread abort classified
**Changes:**
- board.cpp: an accepted MT6835 frame with any sensor status bit (overspeed, weak field, undervoltage) raises the encoder fault, as main does, inside the CRC-validated path so it never clears the fault accept_frame() holds while the link is down
- IdentFailure code 5 (IDENT_FAIL_INDUCTANCE) for main's HFI spread abort, reported through 0x43/0x08 and documented in CAN_PROTOCOL.md
- platformio.ini: main's build cache, monitor timestamps and 16-packet CDC queue kept, the legacy commented environments dropped, release build type / -flto / -fno-math-errno / HAL_UART_MODULE_ONLY and the cln17_v3_dev environment kept
- .llm/CLAUDE.md follows main's move of CLAUDE.md and changelog/, and no longer describes the removed upload hook, suggestions.md and old/ tree

**Reasoning:**
- The branch that carried this work (bsp_layer_shared_stm32g4) was merged as PR #5 and deleted, so the work is re-based onto main to keep the pull request diff to its own changes
- main's encoder fault setter and HFI validation landed in the same files

**Notes:**
- Behaviour change inherited from main: an identification whose HFI estimates disagree by more than 30 percent ends in CalibError (reason 5) instead of committing an averaged inductance. cln17_v3_dev 83192 B
- cln234_release 51332 B / 9184 B. RAM +512 B is the CDC queue (8 -> 16 packets)

FLASH: 71444 bytes (62.29% of 114688 bytes): +248 bytes
RAM: 11760 bytes (35.89% of 32768 bytes): +512 bytes


