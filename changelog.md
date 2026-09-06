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


