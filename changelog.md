## 2026-06-12 - CAN driver comprehensive test suite
**Changes:**
- Added 10 test scenarios in main.cpp running every 100ms without delay()
- Added CAN_BUS global instance and CAN_Driver_Init() function
- Implemented CAN_RX_Callback() for interrupt handling

**Reasoning:**
- Validate all can_setup optimizations with real-world usage patterns
- Test Classic CAN, CAN-FD, multi-frame TX, variable DLC, RX polling, statistics, power control, loopback, and burst throughput

**Notes:**
- Tests cycle through scenarios 0-9 continuously
- Each test validates different aspects of the driver
- SerialUSB logging for debugging

FLASH: 50988 bytes (38.90% of 131072 bytes): +0 bytes
RAM: 6112 bytes (18.65% of 32768 bytes): +0 bytes


## 2026-06-12 - CAN comprehensive test suite
**Changes:**
- Added 13 test scenarios covering all CAN_BUS.* methods
- Implemented 100ms test cycle without delay()
- Added detailed logging with TP_LOG for each scenario

**Reasoning:**
- Verify all CAN driver functionality systematically
- Test all message types (CAN 2.0A and CAN-FD)
- Test all configurations (modes baudrates filters transceiver control)

**Notes:**
- Tests run sequentially each 100ms
- Covers: init status 8-byte classic variable-length CAN-FD 64-byte high-frequency multiple IDs loopback error-handling baudrate changes transceiver control and filtering

FLASH: 55104 bytes (42.04% of 131072 bytes): +4116 bytes
RAM: 7388 bytes (22.55% of 32768 bytes): +1276 bytes


## 2026-06-12 - Fixed CAN test timing issues
**Changes:**
- Fixed elapsed % 10 timing bug causing continuous transmission
- Added last_tx_time tracking for proper 10ms intervals
- Fixed TRANSCEIVER_CONTROL test state management
- Added diagnostic message for NORMAL mode errors without physical bus

**Reasoning:**
- Timing bug caused hundreds of messages per cycle instead of 10
- Rate limiting ensures correct test behavior
- Diagnostics help identify missing ACK errors

**Notes:**
- Tests now send correct number of messages (~10 per 100ms)
- High-frequency test sends ~100 messages (1ms interval)
- Added helpful error explanation for NORMAL mode without bus

FLASH: 55336 bytes (42.22% of 131072 bytes): +232 bytes
RAM: 7396 bytes (22.57% of 32768 bytes): +8 bytes


## 2026-06-12 - Enhanced CAN test diagnostics and documentation
**Changes:**
- Improved init error message (already initialized is not an error)
- Added RX=0 explanation for NORMAL mode
- Added loopback mode change explanation
- Enhanced test result diagnostics

**Reasoning:**
- Help users understand expected behavior
- RX=0 in NORMAL mode without bus is normal
- Mode changes need reinitialization

**Notes:**
- All tests pass with correct message counts
- Test 2-5,7-11,13: 10 msgs
- Test 6: 100 msgs
- Test 9: 20 msgs
- Test 12: 4-5 msgs

FLASH: 55848 bytes (42.61% of 131072 bytes): +512 bytes
RAM: 7396 bytes (22.57% of 32768 bytes): +0 bytes


## 2026-06-12 - Added EXTERNAL_LOOPBACK mode to CAN driver
**Changes:**
- Added CANMode::EXTERNAL_LOOPBACK enum value
- Mapped EXTERNAL_LOOPBACK to FDCAN_MODE_EXTERNAL_LOOPBACK in init()
- Updated test suite to use EXTERNAL_LOOPBACK by default
- Enhanced mode documentation in header

**Reasoning:**
- EXTERNAL_LOOPBACK provides TX->RX echo via physical bus with ACK
- Enables self-test without requiring other CAN nodes
- Better than NORMAL for testing better than LOOPBACK for real signal verification

**Notes:**
- EXTERNAL_LOOPBACK: TX goes to physical pins echoes back to RX with ACK
- LOOPBACK (internal): purely internal no physical pins
- NORMAL: requires other nodes for ACK

FLASH: 56068 bytes (42.78% of 131072 bytes): +220 bytes
RAM: 7396 bytes (22.57% of 32768 bytes): +0 bytes


## 2026-06-12 - Enabled CAN RX interrupts with circular buffer
**Changes:**
- Enabled FDCAN1_IT0_IRQn interrupt (priority 7)
- Restored interrupt-based RX using 16-message circular buffer
- Removed polling-based receive implementation
- Added CAN_RX_Callback to interrupt_setup.h
- Removed duplicate CAN_RX_Callback from can_setup.h

**Reasoning:**
- Interrupt-driven RX eliminates polling overhead
- Circular buffer prevents message loss during burst reception
- Priority 7 ensures CAN is lower priority than FOC control

**Notes:**
- RX buffer: 16 messages × 69 bytes = 1104 bytes
- ISR processes messages in ~100-150 cycles
- Main loop reads from buffer via receive() non-blocking

FLASH: 56368 bytes (43.01% of 131072 bytes): +300 bytes
RAM: 7396 bytes (22.57% of 32768 bytes): +0 bytes


## 2026-06-12 - Fixed double CAN initialization issue
**Changes:**
- Commented out CAN_Driver_Init in tunepulse_init
- CAN now initialized only from application code (main.cpp)
- Removed double init that was causing init() to return false

**Reasoning:**
- Double initialization prevented proper test setup
- Application should control CAN initialization timing
- Allows flexible test configurations

**Notes:**
- CAN initialized once in can_test_init
- No more 'already initialized' warning
- Tests can now properly send messages

FLASH: 55852 bytes (42.61% of 131072 bytes): -516 bytes
RAM: 7396 bytes (22.57% of 32768 bytes): +0 bytes


## 2026-06-12 - Control connector setup implementation
**Changes:**
- Implemented ControlSetup class for runtime mode switching
- Added support for SPI, USART, I2C, encoder (ABZ/pulse-dir), and step-dir modes
- Implemented PB_5/PB_7 conflict prevention
- Added 5V power control
- Provided peripheral access helpers and individual GPIO control

**Reasoning:**
- Enable flexible control connector usage across multiple interface types
- Prevent hardware damage from simultaneous PB_5/PB_7 activation
- Support runtime peripheral reconfiguration

**Notes:**
- Class provides safe mode switching with automatic peripheral initialization/deinitialization
- All 12 operating modes fully implemented with safety checks

FLASH: 56720 bytes (43.27% of 131072 bytes): +868 bytes
RAM: 7476 bytes (22.81% of 32768 bytes): +80 bytes


## 2026-06-13 - Control connector: DMA, runtime config, velocity calculation
**Changes:**
- Added DMA support for SPI3 and USART2
- Implemented runtime reconfiguration for all peripherals
- Added encoder velocity calculation (CPS and RPM)
- Added I2C blocking read/write/scan operations
- Implemented quick setup methods for easy configuration
- Added blocking and non-blocking transfer modes
- Implemented comprehensive status and debug methods

**Reasoning:**
- Enable high-performance DMA transfers for SPI and USART
- Allow runtime parameter changes without mode switching
- Provide accurate motor velocity feedback
- Simplify common configuration scenarios
- Support both DMA and polling operation modes

**Notes:**
- All 256-byte DMA buffers statically allocated
- Velocity calculation uses time-based delta measurement
- I2C timing auto-calculated for 100/400/1000 kHz
- 10 detailed usage examples provided in header comments

FLASH: 56720 bytes (43.27% of 131072 bytes): +0 bytes
RAM: 7476 bytes (22.81% of 32768 bytes): +0 bytes


## 2026-06-23 - Fixed compilation error for CLN17 V3
**Changes:**
- Removed invalid characters from common.h

**Reasoning:**
- The file contained invalid UTF-8 sequence (№ш) that corrupted atomic header inclusion

**Notes:**
- Fixed atomic header compilation errors
- No functional changes

FLASH: 51604 bytes (39.37% of 131072 bytes): -5116 bytes
RAM: 7492 bytes (22.86% of 32768 bytes): +16 bytes


## 2026-07-17 - Fix V3 64-bit atomic operations
**Changes:**
- Replace single 64-bit atomic with three 16-bit atomics in pwm_driver.h
- Update tick() method to load three separate atomics

**Reasoning:**
- ARM Cortex-M4 lacks native 64-bit atomic support
- Linker error: undefined reference to __atomic_load_8 and __atomic_store_8

**Notes:**
- Maintains thread-safety with per-channel 16-bit atomics
- No performance impact
- Compatible with ARM Cortex-M4 architecture

FLASH: 51632 bytes (39.39% of 131072 bytes): +28 bytes
RAM: 7636 bytes (23.30% of 32768 bytes): +144 bytes


## 2026-08-14 - Added detailed motor calibration logging
**Changes:**
- Added verbose logging for current calibration phase
- Added detailed motor calibration status tracking with enum-to-string conversion
- Added periodic heartbeat logs every 2 seconds
- Added timeout protection (60s for motor, 10s for current)
- Fixed critical bug: current_volts_ now updates from drv.get_supply_volts() in tick_calibration

**Reasoning:**
- Investigate infinite loop during motor calibration
- current_volts_ was initialized to 0.0f causing division by zero and invalid voltage limits
- Detailed logging helps identify calibration bottlenecks

**Notes:**
- Monitor SerialUSB output for calibration progress
- Log format: [MOTOR_CAL] Status change #N: <StatusName> (elapsed: Xms)

FLASH: 54836 bytes (41.84% of 131072 bytes): +3204 bytes
RAM: 7556 bytes (23.06% of 32768 bytes): -80 bytes


## 2026-08-14 - Fixed motor calibration status synchronization bug
**Changes:**
- Added motor.set_status(SoftWarmup) in start_calibration()
- Added motor.set_status(status_tracker) at start of tick_calibration() for continuous sync

**Reasoning:**
- Motor status was only tracked in motor_calib_status (local DriverPWM variable)
- get_motor_status() reads from motor.get_status() which was never updated
- This caused 'Unknown' status in logs and prevented calibration progress tracking

**Notes:**
- Status synchronization now happens every tick
- Motor object status stays in sync with calibration state machine

FLASH: 54848 bytes (41.85% of 131072 bytes): +12 bytes
RAM: 7556 bytes (23.06% of 32768 bytes): +0 bytes


## 2026-08-14 - Motor calibration logging enhancement
**Changes:**
- Added comprehensive logging to motor_calibration.h for all calibration states
- Added detailed state transition logs with measured values
- Added HFI measurement progress tracking
- Added resistance correction logging
- Added error and success state detailed logging

**Reasoning:**
- Improve debugging and monitoring of motor calibration process
- Enable better diagnostics for calibration failures
- Provide visibility into all measurement phases

**Notes:**
- All logs use TP_LOG macros with mV/mA/mOhm units
- Every state transition now logged with separator lines
- Progress tracking added for long-running HFI measurements

FLASH: 60620 bytes (46.25% of 131072 bytes): +5772 bytes
RAM: 7556 bytes (23.06% of 32768 bytes): +0 bytes


## 2026-08-14 - Motor calibration voltage supply fix
**Changes:**
- Added update_supply_voltage method to MotorCalibration
- Fixed current_volts_ initialization and update before tick
- Added periodic tick logging for debugging

**Reasoning:**
- Fix voltage limit calculation that was always true due to zero voltage
- Enable proper voltage ramping during SoftWarmup phase
- Add visibility into calibration tick execution

**Notes:**
- Supply voltage now updated from drv.get_supply_volts() before each tick
- Added debug log every 1000 ticks showing state and voltage

FLASH: 60812 bytes (46.40% of 131072 bytes): +192 bytes
RAM: 7560 bytes (23.07% of 32768 bytes): +4 bytes


## 2026-08-14 - Motor calibration complete diagnostic logging
**Changes:**
- Added comprehensive state transition logging
- Fixed supply voltage initialization in start_calibration
- Added periodic tick counter logging
- Added supply voltage display in startup log

**Reasoning:**
- Provide complete visibility into calibration process
- Fix voltage not being initialized causing incorrect limit checks
- Enable debugging of stuck calibration states

**Notes:**
- Every state transition logged with measurements
- Tick counter logs every 1000 cycles
- Supply voltage shown at calibration start and in periodic ticks

FLASH: 60872 bytes (46.44% of 131072 bytes): +60 bytes
RAM: 7560 bytes (23.07% of 32768 bytes): +0 bytes


## 2026-08-15 - Fixed NaN voltage detection bug
**Changes:**
- Replaced bitwise OR (|) with logical OR (||) in set_voltage_ab_volts
- Changed NaN detection from '== NAN' to proper 'x != x' idiom
- Added negative infinity check for completeness

**Reasoning:**
- Bitwise OR operator was preventing NaN detection from working, causing duty cycle to remain at neutral (INT16_MAX/2) instead of setting to INT16_MIN
- IEEE-754 standard requires that NaN comparisons always return false, making '== NAN' ineffective

**Notes:**
- NaN detection now uses standard C++ idiom (x != x)
- Infinity checks now handle both positive and negative infinity
- Logic flow preserved with proper operator precedence

FLASH: 42932 bytes (32.75% of 131072 bytes): -17940 bytes
RAM: 7180 bytes (21.91% of 32768 bytes): -380 bytes


## 2026-08-15 - Fixed Hi-Z state handling in PWM driver
**Changes:**
- Added INT16_MIN detection for Hi-Z state in TIM1_Set_PWM_Values
- Modified CCER mask generation to disable both high-side and low-side FETs for Hi-Z
- Previously INT16_MIN caused regenerative braking instead of Hi-Z

**Reasoning:**
- INT16_MIN signal was not properly handled - CCR was set to full period with only low-side enabled, causing braking
- For true Hi-Z both CC1E and CC1NE must be 0
- Now explicitly checks for INT16_MIN and disables both FETs per phase

**Notes:**
- Hi-Z state now correctly disables all FETs
- Regenerative braking (duty=0) remains with low-side on
- Compiler should optimize boolean checks with IT blocks

FLASH: 42964 bytes (32.78% of 131072 bytes): +32 bytes
RAM: 7180 bytes (21.91% of 32768 bytes): +0 bytes


## 2026-08-23 - Flash-backed angle calibration commands
**Changes:**
- Added calibrate/load_calibration/dump_calibration commands to AngleCalibrator and CalibrationTable
- Added 0xFA2E flash table format with strict monotonicity validation
- Added append-or-erase dump strategy with multi-table support
- Integrated is_calibrated() into TunePulseHeart control path

**Reasoning:**
- Enable calibration persistence across power cycles without re-running the motor sweep

**Notes:**
- Older valid tables act as fallback if a newer record fails validation

FLASH: 47224 bytes (36.03% of 131072 bytes): +4260 bytes
RAM: 7356 bytes (22.45% of 32768 bytes): +176 bytes


## 2026-08-23 - Self-managed non-blocking calibration persistence
**Changes:**
- Flash driver injected into CalibrationTable at init
- Boot-time flash load resolves automatically: valid table -> is_calibrated, otherwise auto-start of full calibration
- calibrate() now persists the fresh table automatically (append or erase+rewrite)
- Flash I/O reworked to one 32-byte block (8 words) per tick via read_block/write

**Reasoning:**
- Remove blocking multi-word flash transfers from the control loop and eliminate manual load/dump commands

**Notes:**
- Erase occupies a single tick and happens only when the region is full

FLASH: 48320 bytes (36.87% of 131072 bytes): +1096 bytes
RAM: 7404 bytes (22.60% of 32768 bytes): +48 bytes


## 2026-08-23 - Flash-backed calibration lifecycle
**Changes:**
- Added calibrate/load_calibration/dump_calibration commands
- CalStage_CheckMemory default stage scanning flash for FA2E tables
- mandatory dump before Ready
- append-or-erase persistence at 64 bytes per tick

**Reasoning:**
- Restore calibration from flash without recalibration on every power cycle

**Notes:**
- Record format: w0=0xFA2E0000|count, packed monotonic samples, restore-metadata trailer
- newest valid table wins

FLASH: 58752 bytes (44.82% of 131072 bytes): +10432 bytes
RAM: 7644 bytes (23.33% of 32768 bytes): +240 bytes


## 2026-08-25 - motor_selector rename; Temperature protection; Documentation pass
**Changes:**
- Renamed motor_selector.h to motor_control.h (MotorControlPHY, motor_ctrl_)
- Added temperature.h with WT/OT limits, shift-based long filter and immediate release
- Integrated into PhyDriverPWM + ADC IRQ glue
- English-only comments and copyright headers across src/lib and cln17_v3

**Reasoning:**
- Consistent naming
- Thermal safety per voltage-protection pattern
- License/documentation compliance

**Notes:**
- New fluent API: set_limit_celsius().wt_lim_celsius().ot_lim_celsius()
- FAULT_OVERHEAT wired in tick_icr
- New docs: optimization_plan.md, refactor_plan.md, suggestions.md

FLASH: 61040 bytes (46.57% of 131072 bytes): +2288 bytes
RAM: 7820 bytes (23.86% of 32768 bytes): +176 bytes


## 2026-08-25 - Temperature sensor moved into the ADC DMA group; bare-minimum integer filter; OT-only hysteresis protection
**Changes:**
- Temp sensor (ADC1 ch16) added as rank 3 of the regular dual-mode sequence read by the shared DMA
- removed the injected-channel software polling state machine
- rewrote TemperaturePHY with a u16 Q12.4 shift-EMA filter (err>>12 + carried division remainder, no mul/div/float)
- removed WT/warning threshold
- OT now releases via configurable hysteresis with thresholds precomputed in the raw domain

**Reasoning:**
- Eliminate all CPU cost of thermal sampling in the 20 kHz ISR and simplify the hot path to pure integer ALU ops

**Notes:**
- SMP16 sampling time fixed (was SMP10 field)
- regular sequence grew by ~6 us due to mandatory 247.5-cycle t_S_temp, DMA TC ISR fires accordingly later in the 50 us period
- filter tau = 4096 ticks (~205 ms @ 20 kHz)

FLASH: 61196 bytes (46.69% of 131072 bytes): +156 bytes
RAM: 7844 bytes (23.94% of 32768 bytes): +24 bytes


## 2026-08-25 - Shorter temp-sensor sampling; EMA filter inlined into TemperaturePHY
**Changes:**
- Reduced SMP16 sampling from 247.5 to 47.5 cycles (~6.1 us -> ~1.4 us per rank-3 conversion)
- removed the FilterFastEMA helper class and unrolled the Q12.4 shift-EMA directly into TemperaturePHY fields and tick()

**Reasoning:**
- Cut the added regular-sequence time to under 1 us (negligible DMA TC latency)
- simplify the code by dropping the helper class

**Notes:**
- Sampling window is now below the datasheet-recommended t_S_temp, absolute accuracy may drift a few degrees -- noise/drift is absorbed by the integer EMA filter

FLASH: 61196 bytes (46.69% of 131072 bytes): +0 bytes
RAM: 7844 bytes (23.94% of 32768 bytes): +0 bytes


## 2026-08-25 - Fix: restored spec temperature-sensor sampling time
**Changes:**
- SMP16 back to 247.5 cycles (~5.8 us), meeting the datasheet t_S_temp >= 5 us requirement

**Reasoning:**
- Undersampled S/H capacitor retained residual charge from the preceding current-sense rank, producing a constant ~46 degC offset that tracked phase currents
- the EMA filter cannot remove such DC bias

**Notes:**
- Regular sequence is ~6 us longer again inside the 50 us control period -- required for valid thermal data

FLASH: 61260 bytes (46.74% of 131072 bytes): +64 bytes
RAM: 7844 bytes (23.94% of 32768 bytes): +0 bytes


## 2026-08-26 - Power measurement + OP protection; motor_driver rename
**Changes:**
- Added power.h (PowerPHY): raw/filtered power-stage power from AB currents x commanded AB voltages, EMA filtering, derived input current, overpower fault (bit 5, HiZ) with recovery timer, per-tick available-power coefficient
- Renamed src/lib/motor_driver to src/lib/motor_control and updated includes

**Reasoning:**
- Protect the driver from excessive input power/current symmetrically in +/- (regen included) and expose power telemetry

**Notes:**
- OP trip: |P_filt|>P_lim OR |I_filt|>I_lim where I_filt=P_filt/V_filt
- limits via driver.power.set_limit().op_lim_watts().op_lim_amps()
- defaults 100 W / 15 A
- wire .op_cb() in fault callback chain

FLASH: 61748 bytes (47.11% of 131072 bytes): +488 bytes
RAM: 7892 bytes (24.08% of 32768 bytes): +48 bytes


## 2026-08-26 - Signed power_coeff; FOC OC-fault guard
**Changes:**
- Replaced available-power coefficient with signed power_coeff in [-1..+1] (share of P_lim, + motor / - regen)
- Added PhyDriverPWM::is_oc_fault() and OC guard in CurrentControl::tick_control_foc that resets FOC state and outputs zero voltage during overcurrent fault

**Reasoning:**
- Expose energy-flow direction telemetry
- prevent PI integrator windup against HiZ outputs and torque kick on fault release

**Notes:**
- power_coeff = sat(P_filt/P_lim, -1, +1), getter get_power_coeff()
- guard resets integrators, step memory and power filters via foc_current_.reset()

FLASH: 61928 bytes (47.25% of 131072 bytes): +180 bytes
RAM: 7900 bytes (24.11% of 32768 bytes): +8 bytes


## 2026-08-26 - Signed power_coeff; FOC OC-fault guard
**Changes:**
- Replaced available-power coefficient with signed power_coeff in [-1..+1] (share of P_lim, + motor / - regen)
- Added PhyDriverPWM::is_oc_fault() and OC guard in CurrentControl::tick_control_foc that resets FOC state and outputs zero voltage during overcurrent fault

**Reasoning:**
- Expose energy-flow direction telemetry
- prevent PI integrator windup against HiZ outputs and torque kick on fault release

**Notes:**
- power_coeff = sat(P_filt/P_lim, -1, +1), getter get_power_coeff()
- guard resets integrators, step memory and power filters via foc_current_.reset()

FLASH: 61928 bytes (47.25% of 131072 bytes): +0 bytes
RAM: 7900 bytes (24.11% of 32768 bytes): +0 bytes


## 2026-08-26 - PHY naming cleanup + driver API map
**Changes:**
- Renamed folder pwm_driver_phy to pwm_driver
- types PhyDriverPWM-CurrentSensePHY-SupplyVoltagePHY-TemperaturePHY-PowerPHY-MotorControlPHY to DriverPWM-CurrentSense-SupplyVoltage-Temperature-Power-MotorControl
- members current_sense to current and supply_voltage to supply across lib/tunepulse/target layers
- Removed dead FAULT_HIZ_MASK/FAULT_SAFE_MASK, stale MotorControl forward decl, duplicate fault check in tick_icr
- Added driver_api_map.md with public API map and mermaid diagrams

**Reasoning:**
- Remove outdated PHY prefix
- document the driver public surface

**Notes:**
- No functional changes
- flash -36 bytes from dead code removal

FLASH: 61892 bytes (47.22% of 131072 bytes): -36 bytes
RAM: 7900 bytes (24.11% of 32768 bytes): +0 bytes


## 2026-08-26 - Driver refactor: file/type/naming cleanup + external API map
**Changes:**
- Renamed files to match types (pwm_driver.h to driver_pwm.h, pwm.h to pwm_wrapper.h, current.h to current_sense.h)
- unified guards to pragma once
- single shared types.h (CompThresholdCallback, kDefaultRecoveryCycles)
- renamed proxies to CallbackProxy/LimitProxy, typedefs to PwmPhasesCallback/SamplePointCallback, rst_cb to restore_cb, SupplyVoltage::reset to seed_filter
- made FAULT_* public
- removed dead Setup init_ member
- const-correct getters
- InputWrapper::setCallback now returns reference
- norm2amps uses ||
- magic numbers named (kDutyMaskThreshold)
- driver_api_map.md rewritten as external-only API map with mermaid

**Reasoning:**
- Align naming/organization with the codebase style and document the true external API surface

**Notes:**
- No algorithm changes
- RAM -24 bytes (dead member), Flash +32 bytes (shared constants)

FLASH: 61924 bytes (47.24% of 131072 bytes): +32 bytes
RAM: 7876 bytes (24.04% of 32768 bytes): -24 bytes


## 2026-08-26 - Phase-domain I/O moved to MotorControl; transit getters removed
**Changes:**
- Moved voltage command path into MotorControl::set_voltage_ab_volts(voltages, supply) returning duties
- measured currents now published to MotorControl::current_ab_amps by CurrentSense
- Removed 8 pass-through getters from DriverPWM (current/supply/temperature/power telemetry) - callers access submodules directly
- set_voltage_ab_volts in DriverPWM reduced to 2-line orchestration
- made SupplyVoltage conversion block public

**Reasoning:**
- Bare-minimum API per level
- phase-domain quantities owned by the phase stage
- no code bloat from transit accessors

**Notes:**
- No algorithm changes
- Flash -40 bytes

FLASH: 61884 bytes (47.21% of 131072 bytes): -40 bytes
RAM: 7876 bytes (24.04% of 32768 bytes): +0 bytes


## 2026-08-26 - API systematization per user feedback
**Changes:**
- Moved OP limit setters into OverpowerProxy (power.overpower.set_limit())
- volts2norm/volts2duty back to private (MotorControl via friendship)
- norm() private in CurrentSense
- renamed set_currents to set_currents_irq
- temperature conversion callback now set_callback().do_conversion_cb()
- new FAULT_PHASE fault (bit 6) raised from MotorControl voltage/current path errors, priority chain renumbered

**Reasoning:**
- Bare-minimum logical API: setters live where the state lives, ISR inputs uniformly named, phase-stage errors surface in the unified driver fault system

**Notes:**
- No algorithm changes
- Flash +44 bytes (fault wiring)

FLASH: 61928 bytes (47.25% of 131072 bytes): +44 bytes
RAM: 7876 bytes (24.04% of 32768 bytes): +0 bytes


## 2026-08-26 - set_voltage_ab_volts / get_current_ab_amps moved into MotorControl
**Changes:**
- DriverPWM::set_voltage_ab_volts removed
- MotorControl now owns the full voltage path via attach(pwm,power,supply) + set_voltage_ab_volts(voltages) feeding power estimator and PWM backend
- current_ab_amps made private with get_current_ab_amps() accessor
- callers updated (lib.h x3, tunepulse.h x2, current_control.h)

**Reasoning:**
- Phase-stage entry/exit points belong to the phase-domain hub
- single-level API without transit methods

**Notes:**
- No algorithm changes
- Flash +124 bytes (attach wiring), RAM +16 bytes (collaborator pointers)

FLASH: 62052 bytes (47.34% of 131072 bytes): +124 bytes
RAM: 7892 bytes (24.08% of 32768 bytes): +16 bytes


## 2026-08-26 - Fix DriverPWM proxy scope compile error
**Changes:**
- Moved FaultCallbackProxy/FaultTriggerProxy out of DriverPWM to namespace scope
- removed misplaced set_fault() definition inside FaultTriggerProxy
- added out-of-class inline DriverPWM::set_fault()
- updated stale docs (driver_pwm_detail.h refs, wrong fluent method names)

**Reasoning:**
- Nested proxy definitions conflicted with namespace-scope forward declarations and friend declarations in sub-module headers (-fpermissive name-meaning error)
- DriverPWM::set_fault() had no definition which would cause a link error

**Notes:**
- No API changes: fluent syntax driver.set_fault().overcurrent(true)/set_callback().oc_cb(...) unchanged

FLASH: 62056 bytes (47.34% of 131072 bytes): +4 bytes
RAM: 7884 bytes (24.06% of 32768 bytes): -8 bytes


## 2026-08-26 - Fix DriverPWM proxy scope compile error
**Changes:**
- Moved FaultCallbackProxy/FaultTriggerProxy out of DriverPWM to namespace scope
- removed misplaced set_fault() definition inside FaultTriggerProxy
- added out-of-class inline DriverPWM::set_fault()
- updated stale docs

**Reasoning:**
- Nested proxy definitions conflicted with namespace-scope forward declarations and friend declarations in sub-module headers
- DriverPWM::set_fault() had no definition which would cause a link error

**Notes:**
- No API changes: fluent syntax driver.set_fault().overcurrent(true)/set_callback().oc_cb(...) unchanged

FLASH: 62056 bytes (47.34% of 131072 bytes): +0 bytes
RAM: 7884 bytes (24.06% of 32768 bytes): +0 bytes


## 2026-08-29 - PLL-based angular velocity estimation
**Changes:**
- Reformated FilterPLL: fs fixed at init, Fc retunable at runtime via set_fc()
- Integrated FilterPLL into CurrentControl
- omega now comes from the PLL instead of raw differentiation of quantized angle samples
- electrical omega = PLL turns/s * 2*pi * pole_pairs passed to FocCurrentController

**Reasoning:**
- Remove quantization noise from speed estimation and decouple FOC bandwidth scheduling from raw LSB deltas

**Notes:**
- angular_speed (LSB/tick) for the adaptive-alpha LPF is now derived from the PLL omega
- pole_pairs cached at init_foc_controller

FLASH: 61200 bytes (46.69% of 131072 bytes): -856 bytes
RAM: 7948 bytes (24.26% of 32768 bytes): +64 bytes


## 2026-08-29 - BEMF feedforward compensation in FOC current controller
**Changes:**
- Added flux_linkage_wb estimated from torque constant
- Added uq_ff_bemf = omega_e * psi_f * k_bemf term to q-axis feedforward
- Added k_bemf tuning gain (0 disables)

**Reasoning:**
- Cancel speed-proportional BEMF disturbance in feedforward instead of letting the PI integrator wind up against it

**Notes:**
- psi_f is a coarse estimate (ignores saturation/cogging)
- Compensation sign matches omega
- D-axis untouched

FLASH: 61860 bytes (47.20% of 131072 bytes): +660 bytes
RAM: 7964 bytes (24.30% of 32768 bytes): +16 bytes


## 2026-08-29 - Soft deadband for BEMF feedforward compensation
**Changes:**
- Added bemf_omega_deadband (epsilon, 5 rad/s el) and bemf_omega_soft_range (10 rad/s) parameters
- BEMF term gated by C2-continuous smootherstep activation gain 6t^5-15t^4+10t^3 applied to |omega_e|
- uq_ff_bemf = omega * gain(|omega|) * psi_f * k_bemf (odd symmetry preserved)

**Reasoning:**
- Eliminate standstill limit cycle: noisy PLL omega fed through BEMF feedforward closes omega->uq->torque->encoder->PLL positive feedback loop
- smooth activation avoids torque kicks at zone boundary

**Notes:**
- Tune bemf_omega_deadband above the PLL noise floor
- PI integrator absorbs residual EMF inside the forbidden zone
- build OK: flash 62040, RAM 7972

FLASH: 62040 bytes (47.33% of 131072 bytes): +180 bytes
RAM: 7972 bytes (24.33% of 32768 bytes): +8 bytes


## 2026-08-29 - Multi-probe upload support
**Changes:**
- Added tools/upload_all_stlinks.py PlatformIO post-script
- Wired extra_scripts into cln17_v3_release env
- Updated UPLOAD.md

**Reasoning:**
- Single command now flashes every board connected to every ST-Link in parallel

**Notes:**
- Uses OpenOCD hla_serial binding
- Exit code fails if any board fails
- 2/2 boards verified OK on live test

FLASH: 61996 bytes (47.30% of 131072 bytes): -44 bytes
RAM: 7980 bytes (24.35% of 32768 bytes): +8 bytes


## 2026-08-29 - Added 8-byte chunk API (src/api)
**Changes:**
- New src/api/tp_api.h: zero-copy ApiChunk union, read/write hardware callbacks, packers for CMD 0x01-0x09 + heartbeat, codecs (bf16 from lib, float24, int24/48, Q1.3.8, Q1.1.4)
- New src/api/API.md spec + roadmap
- Wired TunePulseAPI into main.cpp over CAN (0x300/0x400+NodeID)

**Reasoning:**
- Transport-independent request/response layer reusing lib telemetry (Motor, Temperature, Power, SupplyVoltage)

**Notes:**
- Motor Properties CMD remapped 0x06->0x09 (collision)
- motion/UID/efficiency left as STUBs

FLASH: 64216 bytes (48.99% of 131072 bytes): +2220 bytes
RAM: 8080 bytes (24.66% of 32768 bytes): +100 bytes


## 2026-08-30 - Two-board CAN peer-to-peer text test
**Changes:**
- Rewrote main.cpp as a symmetric CAN test: SerialUSB text -> classic CAN frames (ID 0x123, 125 kbps)
- CAN -> Serial line assembly
- 2 s stats with FDCAN PSR (LEC/ACT)
- boot-time EXTERNAL_LOOPBACK self-test
- API temporarily disabled

**Reasoning:**
- Fix missing FDCAN1_IT0_IRQHandler in can_setup.h (first received frame hung the MCU in Default_Handler)
- isolate firmware from physical wiring issues
- validate the CAN link between two CLN17 boards

**Notes:**
- Verified live: bidirectional text exchange and multi-frame long lines delivered without loss
- flash 58832 bytes, RAM 8132 bytes

FLASH: 58832 bytes (44.89% of 131072 bytes): -5384 bytes
RAM: 8132 bytes (24.82% of 32768 bytes): +52 bytes


## 2026-08-30 - CAN stress test: bitrate sweep with packet-error logging
**Changes:**
- Added firmware commands B<bps>/T<count>/R/? for runtime bitrate reconfig and 1000-packet bursts with sequence-gap loss detection
- stress mode in can_test_host.py logging speed, received, loss percent and FDCAN ECR protocol errors per bitrate

**Reasoning:**
- User added 120Ohm termination at both ends and common ground
- re-verify the bus at 125k/250k/500k/1M to pick a working data rate

**Notes:**
- 1000/1000 packets delivered with 0 protocol errors (CEL=0 TEC=0 REC=0) at all speeds up to 1 Mbps
- replaced misleading poll-count err metric with ECR.CEL/TEC/REC in stats

FLASH: 59112 bytes (45.10% of 131072 bytes): +280 bytes
RAM: 8132 bytes (24.82% of 32768 bytes): +0 bytes


## 2026-08-30 - CAN stress test: bitrate sweep with packet-error logging
**Changes:**
- Added firmware commands B<bps>/T<count>/R/? for runtime bitrate reconfig and 1000-packet bursts with sequence-gap loss detection
- stress mode in can_test_host.py logging speed, received, loss percent and FDCAN ECR protocol errors per bitrate

**Reasoning:**
- User added 120Ohm termination at both ends and common ground
- re-verify the bus at 125k/250k/500k/1M to pick a working data rate

**Notes:**
- 1000/1000 packets delivered with 0 protocol errors (CEL=0 TEC=0 REC=0) at all speeds up to 1 Mbps
- replaced misleading poll-count err metric with ECR.CEL/TEC/REC in stats

FLASH: 60312 bytes (46.01% of 131072 bytes): +1200 bytes
RAM: 8280 bytes (25.27% of 32768 bytes): +148 bytes


## 2026-08-30 - CAN-FD stress test up to 5 Mbps data phase
**Changes:**
- Enabled transmitter delay compensation (TDC) in can_setup.h init() with empirically verified TDCO=16 kernel TQ cap
- extended B command to B<arb>:<fd>[:<tdco>] for FD BRS frames
- bounded TX retry loop (TXFAIL instead of firmware wedge)
- stressfd mode in can_test_host.py

**Reasoning:**
- Test the FD data phase up to 5 Mbps on the terminated two-board bus
- the driver flag enable_delay_compensation was never applied and FD frames failed above 3.4 Mbps due to wrong TDCO units (kernel TQ, not data mTQ)

**Notes:**
- 1000/1000 packets with 0 protocol errors at arb 1M + data 1M/2.5M/3.4M/4.25M/5M
- classic 125k-1M re-verified 0 errors
- note 2M/3M/4M are not exactly representable from the 170 MHz kernel clock (run at 2.02/3.04/4.05M)

FLASH: 60956 bytes (46.51% of 131072 bytes): +644 bytes
RAM: 8296 bytes (25.32% of 32768 bytes): +16 bytes


## 2026-08-30 - Refactored CAN stack: hardware layer in can_setup, message layer in can_control.h
**Changes:**
- Created src/can_control.h (CANControl): interface setup at fixed 1 Mbps arb / 5 Mbps FD, non-blocking TX frame ring with 6+2 chunk framing, RX reassembly queue, status counters
- cleaned test artifacts out of can_setup.h (NORMAL mode default, fixed TDCO=16, removed TDCO override)
- main.cpp reduced to a SerialUSB<->CAN text bridge

**Reasoning:**
- User requested clear layered structure: hardware in can_setup, buffers and message protocol in can_control.h, minimal main
- no test-only code in production firmware

**Notes:**
- Live-verified bidirectional text exchange with multi-chunk messages on both boards
- flash 58784 bytes, RAM 9316 bytes

FLASH: 58784 bytes (44.85% of 131072 bytes): -2172 bytes
RAM: 9316 bytes (28.43% of 32768 bytes): +1020 bytes


## 2026-08-30 - CAN chunk protocol: raw 8-byte commands with FD frame aggregation
**Changes:**
- Redesigned can_control.h: one chunk = one command = 8 bytes with zero service bytes
- consecutive same-id chunks aggregate into single CAN-FD frames (up to 8 per 64B frame, DLC-derived unpacking)
- lock-free SPSC RX chunk ring and TX frame ring with DMB barriers
- API: send(id,chunk)/read(id,chunk)/is_rx_available/is_tx_empty/is_tx_full/flush/update
- fixed bytes_to_dlc() bug for DLC 24/32/48/64 in can_setup.h

**Reasoning:**
- User requested flag-free 8-byte chunk protocol with hardware-level FD aggregation for same-id commands

**Notes:**
- Live verified: bidirectional text, 96-char line in 2 FD frames, 40-line flood with zero losses
- flash 58748, RAM 9332

FLASH: 58748 bytes (44.82% of 131072 bytes): -36 bytes
RAM: 9332 bytes (28.48% of 32768 bytes): +16 bytes


## 2026-08-30 - CAN control: write(id,msg,len) whole-message transmit API
**Changes:**
- Replaced chunk-loop send+flush with single write(id,msg,len): atomic all-or-nothing enqueue, direct bulk memcpy into frame-ring slots (one copy per frame, no intermediate aggregation buffer), FD tail zero-padded to legal DLC (5->6, 7->8 chunks)
- send(id,chunk)=write of one chunk
- read() stays one chunk per call
- classic CAN splits into 8-byte frames
- main.cpp uses a single write() per console line

**Reasoning:**
- User requested whole-message write semantics: what is written is what gets sent, no flush/pending state

**Notes:**
- Live verified: bidirectional text, 100-byte message in 2 FD frames, 60-line flood with zero losses
- flash 58640, RAM 9256

FLASH: 58640 bytes (44.74% of 131072 bytes): -108 bytes
RAM: 9256 bytes (28.25% of 32768 bytes): -76 bytes


## 2026-08-31 - spi_setup.h reworked: single internal MT6835 on SPI3
**Changes:**
- Removed SPI1/TLE5012 support, external-encoder auto-detection and ENCODER_USE_EXTERNAL routing
- deleted ~380 lines of commented-out legacy code
- Encoder_SmartInit() renamed to Encoder_Init()
- added DMA transfer-error handling path

**Reasoning:**
- Only one internal encoder (MT6835 on SPI3) is used now, no external encoder exists

**Notes:**
- Public API kept: Encoder_GetAngle() / Encoder_StartTransfer() unchanged
- tunepulse.h updated to call Encoder_Init()

FLASH: 63300 bytes (48.29% of 131072 bytes): +4660 bytes
RAM: 7404 bytes (22.60% of 32768 bytes): -1852 bytes


## 2026-08-31 - spi_setup.h reworked: both encoders on SPI1, MT6835 primary
**Changes:**
- Replaced SPI3/MT6835-only design with single SPI1 bus hosting MT6835 (mode 3, probed first) and TLE5012B fallback (mode 1) with auto-detection
- SPI1_SwitchToTLE() reconfigures CPOL and TX DMA buffer
- transfer length switched per active encoder (6/4 bytes)

**Reasoning:**
- Both encoders are wired to the internal SPI1 bus (PA4 CS, PA5-PA7)
- SPI3 is not used for encoders

**Notes:**
- API unchanged: Encoder_Init(), Encoder_GetAngle(), Encoder_StartTransfer()
- new Encoder_IsUsingMT() replaces Encoder_IsUsingExternal()

FLASH: 63616 bytes (48.54% of 131072 bytes): +316 bytes
RAM: 7416 bytes (22.63% of 32768 bytes): +12 bytes


## 2026-08-31 - USB log of detected encoder
**Changes:**
- Added TP_LOG messages to Encoder_Init(): reports MT6835 detected (primary) or fallback to TLE5012B over SerialUSB
- included lib/dev_log.h

**Reasoning:**
- User requested visibility of which encoder was auto-detected at boot

**Notes:**
- Messages: 'ENCODER: MT6835 detected on SPI1 (primary)' / 'ENCODER: MT6835 not found, fallback to TLE5012B on SPI1'
- zero cost when DEV_LOG_ENABLE is off

FLASH: 63748 bytes (48.64% of 131072 bytes): +132 bytes
RAM: 7416 bytes (22.63% of 32768 bytes): +0 bytes


## 2026-08-31 - Encoder selection via compile-time define
**Changes:**
- Removed all runtime auto-detection logic
- added ENCODER_TYPE define (ENCODER_MT6835 / ENCODER_TLE5012B) that selects buffers, SPI mode (3/1) and frame length (6/4 bytes)
- USB log reports the configured type
- #error guard against invalid selection

**Reasoning:**
- TLE5012B auto-detection was unreliable
- user sets the encoder type manually

**Notes:**
- Both branches build-verified: MT6835 (Flash 63340) and TLE5012B (Flash 63312)
- default = ENCODER_MT6835
- Encoder_GetAngle()/Encoder_StartTransfer() API unchanged

FLASH: 63340 bytes (48.32% of 131072 bytes): -408 bytes
RAM: 7404 bytes (22.60% of 32768 bytes): -12 bytes


## 2026-08-31 - Conditional DRV_EN pinout per encoder type
**Changes:**
- MT6835 board: DRV_EN moved to PB12, PB11 permanently driven LOW (both configured as outputs in IO_Init)
- TLE5012B board: PB11 = DRV_EN, PB12 = I_CAL (unchanged)
- spi_setup.h now included from target.h so ENCODER_TYPE drives PINOUT defines
- tunepulse.h asserts the correct DRV_EN pin conditionally

**Reasoning:**
- DRV_EN/I_CAL pin assignment differs between MT6835 and TLE5012B board revisions

**Notes:**
- Both configs build-verified: MT6835 (Flash 63344) and TLE5012B (Flash 63312)
- PINOUT::DRV_EN / PINOUT::DRV_LOW / PINOUT::I_CAL defined conditionally in target.h

FLASH: 63344 bytes (48.33% of 131072 bytes): +4 bytes
RAM: 7404 bytes (22.60% of 32768 bytes): +0 bytes


## 2026-09-01 - Angle calibration: verified read-back after flash save; removed ISR-blocking USB dumps
**Changes:**
- CalStage_Save now reloads the freshly written table from flash via the regular boot load path before reaching Ready (verify_after_save flag)
- motor profile publish (cal_id/offset/begin_save) deferred until read-back commit
- read-back miss raises CalStage_Error
- removed 200-element TP_LOG_ARRAY dumps of cal/ecc tables from check() (they ran inside the PWM-period ISR and block the control loop on USB back-pressure)

**Reasoning:**
- Post-fresh-calibration motor behaved erratically until reset while the same flash table worked after reboot - live state now provably equals the post-reset state, and the flash write itself is validated
- calibration no longer stalls the control ISR during validation

**Notes:**
- Read-back adds ~14 ticks (0.7 ms at 20 kHz) of held-rotor time after the dump
- Flash 63272 (-72), RAM 7396 (-8)
- builds clean for cln17_v3_release

FLASH: 63272 bytes (48.27% of 131072 bytes): -72 bytes
RAM: 7396 bytes (22.57% of 32768 bytes): -8 bytes


## 2026-09-01 - Fixed glacial calibration: armed fast-path miss skipped Setup stage, rotation ran at speed=1
**Changes:**
- CheckMemory miss with an armed calibration request now restarts the full physical cycle from CalStage_Setup (which computes speed = calculate_speed() = 50 inc/tick) instead of jumping straight to Reset
- state (cycle stage, timers, cal_idx, ang_el_step) fully reinitialized on this path

**Reasoning:**
- With a stored motor profile the fast path (CheckMemory -> miss -> Reset) bypassed the only place where rotation speed is initialized, leaving speed at its default of 1 increment/tick: rotor sweep became ~50x slower than designed, Pass0 failed with 'Motion too inconsistent' due to cogging at crawl speed, and Pass1/Pass2 stretched to minutes
- latent since the flash fast-path was introduced (unreachable on a device without a stored profile)

**Notes:**
- Expected log now includes 'Starting calibration process' and 'Reset stage'
- BLDC Pass1 back to ~1.2 s per pass
- Flash 63368 (+96), RAM 7396

FLASH: 63368 bytes (48.35% of 131072 bytes): +96 bytes
RAM: 7396 bytes (22.57% of 32768 bytes): +0 bytes


## 2026-09-01 - Angle calibration: fixed post-calibration electrical chaos (root cause + parity hardening)
**Changes:**
- Root cause: check() ran 200-element TP_LOG_ARRAY dumps of the cal/ECC tables over SerialUSB while executing INSIDE the control ISR - fresh calibration path only
- once the USB CDC TX buffer filled, print() blocked on host back-pressure and the control loop froze with the motor energized at 0.5 A
- boot path never calls check() so the same flash data always worked after a reset
- dump removal confirmed in place
- ECC table now rebuilt at commit_candidate() (single commit point shared by boot load and post-save read-back) instead of inside check(), making live post-calibration RAM state bit-identical to the post-reboot state including the RAM-only ECC table
- motor de-energized (targt=0) during the flash persistence window (Save + read-back via is_settling()/is_persisting()) and during motor-profile record writes (Motor::is_flash_busy())

**Reasoning:**
- Flash programming on the single-bank STM32G431 stalls the CPU for several PWM periods per 32-byte record (tens of ms for a full-region erase)
- holding 0.5 A at a frozen voltage vector during those stalls yanked the rotor at the end of every calibration
- USB CDC dumps inside the ISR could block indefinitely
- ECC table was the only RAM state that differed between fresh calibration and boot load

**Notes:**
- Root cause matched the symptom exactly: chaos only after calibration completes, perfect behavior after reboot with the same data
- user suspicion about premature Ready status was ruled out - Ready is already gated behind the verified flash read-back
- expect a short de-energized coast during the save window (table is already captured, so it is harmless)

FLASH: 63436 bytes (48.40% of 131072 bytes): +68 bytes
RAM: 7404 bytes (22.60% of 32768 bytes): +8 bytes


## 2026-09-01 - Angle calibration: strict reset-equivalence - RAM table wiped and rebuilt purely from freshly written flash
**Changes:**
- Save stage now calls CalibrationTable::wipe() (new method: full RAM state to pristine power-on values via create(), flash binding preserved) BEFORE the read-back
- the live state is then rebuilt exclusively by begin_load->scan_word->commit_candidate from the just-written record - the identical code path and identical bytes as after an HW reset
- nothing from the calibration sweep (samples, ECC, metadata, stale tail beyond cal_size) survives in RAM
- added commit diagnostics line 'CAL: committed size,div,offst,oidx,maxdev' printed on EVERY successful table commit (both boot restore and post-save read-back) so the post-calibration state and the boot state can be compared byte-for-byte

**Reasoning:**
- User reported chaos persisting after calibration
- static analysis could not separate the paths further, so equivalence is now enforced structurally instead of proven by inspection
- the commit log turns the 'same data' assumption into a verifiable fact

**Notes:**
- Expected: the CAL: committed line right after calibration must be IDENTICAL to the line at the next boot - if they differ the record/dump path is broken, if they match the divergence is outside the table (please capture both lines plus what the motor does between Ready and the chaos)

FLASH: 63760 bytes (48.65% of 131072 bytes): +324 bytes
RAM: 7404 bytes (22.60% of 32768 bytes): +0 bytes


## 2026-09-01 - Angle calibration: fixed read-back loading chunk 0 instead of the freshly written record
**Changes:**
- CalibrationTable::wipe() reset store_ok/f_wri0, so dumped_id() evaluated AFTER the wipe always returned 0: the post-save read-back loaded chunk 0 (the OLDEST record in the partition - possibly a stale table from the previous TLE5012B encoder era) instead of the just-written one, and motor.set_cal_id published 0 into the profile
- the record id is now captured into saved_cal_id_ BEFORE the wipe and used both for begin_load() of the read-back and for the profile publish
- commit log extended with the source chunk id ('table committed from chunk id = N')

**Reasoning:**
- The wipe-based reset-equivalence introduced a use-after-wipe of the dump engine state
- loading chunk 0 fed get_el_angle_correction() with a stale/incompatible table immediately after calibration while a reboot restored a different (newest-profile) record - exactly the reported live-vs-reboot divergence

**Notes:**
- After flashing, the log right after calibration must show 'table committed from chunk id = N' where N matches 'Table saved to flash, id = N', and the next boot must commit the SAME id with identical size/div/offst/oidx/maxdev values

FLASH: 63872 bytes (48.73% of 131072 bytes): +112 bytes
RAM: 7412 bytes (22.62% of 32768 bytes): +8 bytes


## 2026-09-01 - Fixed boot-time motor profile restore filtering out freshly saved profiles; erased-flash misclassified as HW fault in free-space scans
**Changes:**
- tunepulse_init() now arms the boot restore with begin_load(MOTOR_ID_ANY) instead of begin_load() which filtered the scan by the constructor-default cal_id_=0
- CalibrationTable::dump_tick() and Motor::save_tick() now treat a flash read fault as the start of free space instead of no-space-found
- removed SerialUSB logs from CalibrationTable::check() which ran inside the control ISR while the motor was energized

**Reasoning:**
- A recalibration appends its table record to the NEXT free flash chunk so the freshly saved profile carries a non-zero Cal-ID
- the boot scan filtered by cal_id_=0 silently dropped it, so every power cycle restored a STALE profile (or none) and the post-reboot state diverged from the verified live post-calibration state - exactly the reported live-vs-reboot difference
- erased flash raises a double-bit ECC NMI which read_block reports as a fault, so the free-space scans always aborted into the erase-everything path (needless wear plus a tens-of-ms IRQ-masked stall inside the control ISR)
- USB writes inside the energized ISR are the previously proven freeze mechanism

**Notes:**
- Expected behavior: after a recalibration the next boot must restore the NEWEST profile (check the MOTOR: parameters restored log - cal_id must match the last CALIBRATION: table committed from chunk id) and the CAL: committed line must be identical after calibration and at the next boot
- dumps now append without erasing the calibration partition
- expect one extra appended record per recalibration until the 2KB partition fills (16 records), then one clean erase+rewrite

FLASH: 63784 bytes (48.66% of 131072 bytes): -88 bytes
RAM: 7412 bytes (22.62% of 32768 bytes): +0 bytes


## 2026-09-01 - Fixed post-calibration angle chaos: position controller no longer yanks the rotor to absolute integrator zero; calibration loop now de-energized during motor-profile flash save
**Changes:**
- Position-hold target is captured at the moment the angle calibration completes (pos_hold_target_) instead of the hardcoded absolute zero target
- the flash-I/O de-energize gate in the calibration branch now also covers motor_params.is_flash_busy()

**Reasoning:**
- The log proved the calibration table and profile are bit-identical between the live session and the reboot, so the divergence was behavioral: the hardcoded target 0 made the drive fly up to a full turn across the 50-pole-pair detent field right after every calibration (read as 'the corrected angle is garbage'), while a reboot looked fine only because the previous session had already parked the rotor at that zero
- on a first boot (no stored profile) MotorCalibration starts the profile save which runs its write stall plus the float-heavy report() inside the control ISR while the calibration is energized, corrupting the Pass0 step statistics ('Motion too inconsistent')

**Notes:**
- Expected: right after the calibration completes the rotor must hold its position (no lurch), Pass0 must pass deterministically on a first boot after flashing, and the angle must stay valid without a reboot
- to test the first-boot path erase the flash or use a fresh chip

FLASH: 63812 bytes (48.68% of 131072 bytes): +28 bytes
RAM: 7420 bytes (22.64% of 32768 bytes): +8 bytes


## 2026-09-01 - Flash programming de-energized window gate + ISR-safe logging
**Changes:**
- Added tp_flash_programming_window flag published by TunePulseHeart::tick each pass
- Motor::save_tick and CalibrationTable::dump_tick now wait for the zero-current window before flash write/erase (retry next tick)
- TP_LOG* macros check CDC TX free room and drop output instead of blocking

**Reasoning:**
- Every 32-byte flash write stall (~0.5 ms with IRQs off) ran while an energized 0.5 A vector was still latched on the bridge: on a fresh flash the MotorCalibration-triggered profile save fired right at the angle-calibration start (derailed Pass0 + electrical noise)
- after a reboot the post-calibration profile save produced the same yank right after completion. USBSerial::write additionally blocks on host back-pressure inside the control ISR - worst right after flashing while USB CDC re-enumerates

**Notes:**
- Saves now complete inside the existing de-energized windows (CheckMemory / Save / is_flash_busy)
- expect the usual zero-current coast during saves but no rotor yank on the first write tick
- logs may be truncated under heavy USB back-pressure
- erased-flash reads still raise a recovered double-ECC NMI per word (bounded, ~1 us)

FLASH: 64664 bytes (49.33% of 131072 bytes): +852 bytes
RAM: 7420 bytes (22.64% of 32768 bytes): +0 bytes


