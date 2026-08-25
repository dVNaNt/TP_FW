# Optimization Plan

## P1 — Correctness-adjacent hot-path issues

### 1.3 Blocking `while (SPIx->SR & SPI_SR_BSY)` inside the encoder DMA ISR — `spi_setup.h`
**Where:** `DMA1_Channel1_IRQHandler` (both branches) and `Encoder_SmartInit`.
**Problem:** At ~2.7–5.3 MHz SPI the last bytes are still shifting after DMA-complete; the ISR
busy-spins up to tens of microseconds at priority 5 every control cycle.
**Fix options (best first):**
1. Use the SPI EOT/RXNE interrupt to close CS;
2. Defer CS raise + parse to the `Encoder_GetAngle()` consumption point;
3. Minimum: replace the `NOP` spin with `__WFE()`.

### 1.4 Non-reentrant `is_running` flag in `TunePulseHeart::tick()` — `lib.h`
**Problem:** Plain volatile bool written from ISR context without atomics; a torn interleaving can
skip or duplicate a control pass. Samples are silently dropped on re-entry.
**Fix:** Use `std::atomic<bool>::exchange` (or LDREX/STREX) and document dropping as intentional
back-pressure.

---

## P2 — Per-cycle CPU savings

### 2.1 Redundant per-cycle register rewrites — `timer_pwm_setup.h`
`TIM1_Set_PWM_Values` rewrites `CCR4` and `BDTR` every PWM period although `ADC_DELAY_TICKS` and
deadtime change rarely. Cache last written values; write only on change.

### 2.2 `HAL_CORDIC_Configure` on every trig call — `cordic_setup.h`
Each call re-programs CSR through HAL, then polls with `HAL_MAX_DELAY`. Cache the configured
function, drive WDATA/RDATA directly. Expected >=5x latency reduction; also expose Q31 in/out to
avoid float conversions per call.

### 2.3 Float division in `volts2duty()` — `voltage.h`
Two float divisions per control cycle in `set_voltage_ab_volts()`. Maintain `inv_voltage_`
recomputed only when the filtered voltage changes; hot path becomes multiply-only.

### 2.4 Byte-wise copy in `CANDriver::receive()` — `can_setup.h`
Replace the byte loop with `std::memcpy` (aligned AoS source expands into word copies).

### 2.5 Repeated volatile global loads — `interrupt_setup.h`
Cache `&TUNE_HEART.driver_phy` once after init instead of walking the volatile pointer chain per
sample.

---


## P5 — Minor cleanups

1. `signf()` duplicated in `foc_vector_synthesis.h` (private static) — reuse `common.h`.


