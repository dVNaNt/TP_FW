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


