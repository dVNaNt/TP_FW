# Encoders — hardware register maps

Target-agnostic, compile-time register maps and SPI command builders for the
absolute magnetic encoders used across TunePulse boards. Everything here is
`constexpr` and free of MCU dependencies (see `mt6835.h` for the MT6835 map).

## Angle data convention (MANDATORY for every encoder driver)

> **`get_angle()` MUST return the raw angle as a 32-bit LEFT-ALIGNED value.**
>
> The encoder output is always scaled so that a full mechanical revolution
> occupies the most significant bits of the word: the sensor's native
> resolution (e.g. 21 bits for the MT6835) is shifted up until the MSB of the
> angle sits at bit 31. The unused low bits are zero.

Example (MT6835, 21-bit sensor): `angle32 = raw21 << 11`.

Rationale: the downstream motion pipeline (interpolators, position loops,
speed estimators) is fixed-point Q31 and expects every position source to
share the same scale, independent of the physical sensor resolution. A
left-aligned 32-bit word gives a uniform 0..(2^32-1) per-revolution domain
and lets the math layer treat all encoders identically without runtime
rescaling.

Reference implementation of the consumer side:
`src/core/phy/encoder_phy/encoder.h` — `get_angle()` is documented as
*"Get the 'clean' angle (32 bits, left-aligned)"*.
