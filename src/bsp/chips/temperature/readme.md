# Temperature — hardware thermistor converters

Target-agnostic, header-only converters for divider-based temperature
sensors (`ntc.h` for NTC thermistors, `ptc.h` for linear silicon PTCs).
No MCU dependencies; everything is resolved at compile time.

## Data contract (identical for both files)

> **INPUT is always `uint16_t` LEFT-ALIGNED ADC data — ALL 16 BITS are
> used**: full scale is 65535 (a 12-bit ADC left-aligned word simply has its
> low nibble at zero, the math never discards or shifts it away).
>
> **OUTPUT is always `float` — temperature in degrees Celsius, unchanged**:
> a pure physics conversion with no filtering, averaging, hysteresis or
> protection logic applied. Those concerns belong to the upper layers
> (e.g. `core/phy/pwm_driver/temperature.h`).

## Divider topologies & internal code inversion

The topology argument also fixes the code polarity. Both getters invert the
code ONCE internally into the monotonic domain (hotter = LARGER code), so
the raw DMA word can be fed as-is in every case:

```text
  LOW SIDE  (Pullup given):          HIGH SIDE (Pulldown given):

      VREF  <--- Pullup                 VREF
       |                                 |
     [Rpull]                           [NTC/PTC]  sensor to VREF
       |--- ADC tap (code)               |--- ADC tap (code)
     [NTC/PTC] sensor to GND           [Rpull]
       |                                 |
      GND                               GND

  hot sensor -> SMALLER code         hot sensor -> LARGER code
  inverted internally:               used as-is:
    mono = 65535 - code                mono = code
```

After normalization both topologies share one code path:
`R_sensor = R_ref * (65535 - mono) / mono`.

## Safe window -> NaN (short / not connected)

`Tmin_x10` / `Tmax_x10` (tenths of degC, defaults -600/+1500 = -60..+150 °C)
define the custom safe range. A result outside the window means a degenerate
divider — **shorted** sensor (above the window) or **not connected** sensor
(below the window) — and the getters return **NaN** instead of a fake value.

Compatibility with the generic Temperature driver
(`core/phy/pwm_driver/temperature.h`):

- `celsius2raw()` binary search uses the predicate `T(mid) < target`; NaN
  compares as "not less", so probes in the NaN regions simply terminate the
  search at the nearest boundary of the valid band — no infinite loop, no
  misinversion. Configure the OT limits **inside** the safe window.
- `tick()` compares raw words with integer arithmetic only and never calls
  the conversion callback — NaN cannot leak into the ISR hot path.

## Usage — scenarios selected by template arguments

The sensor configuration is passed as **integer compile-time template
arguments** (C++17-compatible NTTPs; units below). Depending on which
arguments are PRESENT or ABSENT, a different scenario is compiled in; every
absent option removes its code entirely (`if constexpr`) — **zero flash /
zero RAM overhead**:

| Argument    | Unit      | Meaning                                          |
|-------------|-----------|--------------------------------------------------|
| `R25`       | Ohm       | Sensor resistance at 25 °C (mandatory)           |
| `B25` (NTC) | K         | Beta number (mandatory)                          |
| `K_mOhm`    | mOhm/K    | PTC slope ×1000 (mandatory; 4670 = 4.67 Ω/K)     |
| `Pullup`    | Ohm       | Reference leg to VREF, sensor to GND (low side)  |
| `Pulldown`  | Ohm       | Reference leg to GND, sensor to VREF (high side) |
| `Tmin_x10`  | degC ×10  | Safe window lower bound (default -600)           |
| `Tmax_x10`  | degC ×10  | Safe window upper bound (default +1500)          |

Exactly one of `Pullup`/`Pulldown` must be given. Usage:

```cpp
// NTC 100k, B25 = 4250 K, 40.2k pull-up (low side), default -60..+150 degC window:
float t = NTC_GetCelsius<100000, 4250, 40200>(raw_word);

// NTC with a custom -20..+110 degC safe window:
float t = NTC_GetCelsius<100000, 4250, 40200, 0, -200, 1100>(raw_word);

// KTY84-130 PTC, 2.7k pull-down to GND (high side):
float t = PTC_GetCelsius<980, 4670, 0, 2700>(raw_word);
```
