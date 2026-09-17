/**
 * @file midi.h
 * @brief Coils-as-speaker MIDI synthesizer: waveforms, envelopes and LFO helpers.
 *
 * This module turns the motor's stator coils into a speaker. The DriveAction::Tone
 * path calls MotorSpeaker::tick_poly() once per control pass (20 kHz) and the
 * returned alpha-beta vector is latched onto the bridge with no regulator in
 * between, so every cycle of CPU spent here is directly audible budget. Up to
 * three notes may overlap; everything in this file runs on integer
 * fixed-point math:
 *
 * - **Sine** - the classic polar-rotation oscillator (FrequencyGenerator): one
 *   vector rotate (4 multiplies + 2 shifts) per sample amortizes the trigonometry
 *   over the whole period. No LUT walk, no float.
 * - **Square / Triangle / Sawtooth** - pure bit operations on the same phase
 *   accumulator, then one i1.15 multiply to apply the amplitude. Both phase
 *   components (alpha = phase, beta = phase + 90 deg) are shaped identically so
 *   the two coils carry the same timbre in quadrature.
 * - **Exponential envelopes** - a 17-entry Q15 LUT of exp2(-k/16) with linear
 *   interpolation covers 2^0 .. 2^-8 with <0.5 % error; one integer multiply
 *   per envelope sample (see midi_exp2_neg_q15()).
 * - **LFO** (tremolo / vibrato / trill) - the quarter-wave angle2sincos() LUT
 *   from trigonometry.h driven by a 16-bit phase accumulator stepped by the
 *   millis() delta, so the modulation rate is wall-clock true and free of drift.
 *
 * The 0xD0 MIDI frame (see api.md) maps here as:
 * - Waveform nibble -> MotorSpeaker::Wave (all four values implemented).
 * - Mode nibble bits 0-1 -> envelope shape (linear / exponential / percussive /
 *   organ), evaluated thread-side in the protocol engine and posted to the drive
 *   as an amplitude (and, for pitch effects, a frequency).
 * - Mode nibble bits 2-3 -> note effect (none / tremolo / vibrato / trill),
 *   also evaluated thread-side; only the waveform reaches the control ISR.
 *
 * The class keeps its legacy duty as the test-tone source for inductance/HFI
 * measurement: tick(freq, amp) with a non-zero frequency overrides the melody
 * and plays continuously, which the calibration paths rely on.
 *
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#pragma once

#include "../../common.h"
#include "../trigonometry.h"          // angle2sincos(): quarter-wave LUT for the LFO
#include "frequency_generator.h"      // Polar-rotation sine oscillator

namespace Notes {
    enum Pitch : uint8_t {
        REST = 0, C4, D4, E4, F4, G4, A4, B4, 
        C5, D5, E5, F5, G5, A5, B5, C6, D6, E6
    };

    static constexpr uint16_t FREQ_LUT[] = {
        0, 262, 294, 330, 349, 392, 440, 494, 
        523, 587, 659, 698, 784, 880, 988, 1047, 1175, 1319
    };

    enum Len : uint8_t {
        SIXTEENTH = 0, EIGHTH = 1, QUARTER = 2, HALF = 3, WHOLE = 4
    };

    static constexpr uint8_t pack(Pitch p, Len l) { return (p << 3) | l; }
}

#define N(pitch, len) Notes::pack(Notes::pitch, Notes::len)

// Memory-compact melody (1 byte per note)
static constexpr uint8_t STARTUP_MELODY[] = {
    N(E5, QUARTER), N(E5, QUARTER), N(F5, QUARTER), N(G5, QUARTER),
    N(G5, QUARTER), N(F5, QUARTER), N(E5, QUARTER), N(D5, QUARTER),
    N(C5, QUARTER), N(C5, QUARTER), N(D5, QUARTER), N(E5, QUARTER),
    N(E5, HALF),    N(D5, QUARTER), N(D5, HALF),    N(REST, QUARTER),
    
    N(E5, QUARTER), N(E5, QUARTER), N(F5, QUARTER), N(G5, QUARTER),
    N(G5, QUARTER), N(F5, QUARTER), N(E5, QUARTER), N(D5, QUARTER),
    N(C5, QUARTER), N(C5, QUARTER), N(D5, QUARTER), N(E5, QUARTER),
    N(D5, HALF),    N(C5, QUARTER), N(C5, HALF),    N(REST, QUARTER),
    
    N(D5, QUARTER), N(D5, QUARTER), N(E5, QUARTER), N(C5, QUARTER),
    N(D5, QUARTER), N(E5, EIGHTH),  N(F5, EIGHTH),  N(E5, QUARTER),
    N(C5, QUARTER), N(D5, QUARTER), N(E5, EIGHTH),  N(F5, EIGHTH),
    N(E5, QUARTER), N(D5, QUARTER), N(C5, QUARTER), N(D5, QUARTER),
    N(G4, HALF),    N(REST, QUARTER),
    
    N(E5, QUARTER), N(E5, QUARTER), N(F5, QUARTER), N(G5, QUARTER),
    N(G5, QUARTER), N(F5, QUARTER), N(E5, QUARTER), N(D5, QUARTER),
    N(C5, QUARTER), N(C5, QUARTER), N(D5, QUARTER), N(E5, QUARTER),
    N(D5, HALF),    N(C5, QUARTER), N(C5, WHOLE),
    
    N(REST, QUARTER), N(G5, EIGHTH), N(E5, EIGHTH), N(C5, QUARTER), 
    N(G5, QUARTER), N(C6, WHOLE)
};

#undef N

// ============================================================================
// Fixed-point envelope / LFO math (shared with the protocol engine)
// ============================================================================

/**
 * @brief Q15 LUT of exp2(-k/16) for k in [0..16].
 *
 * 2^-f decays from 1.0 to 0.5 across one unit of argument, so a 17-entry table
 * plus 4-bit-interpolated linear interpolation bounds the relative error well
 * under 1 % over 2^0 .. 2^-8 - inaudible on an envelope, and far cheaper in
 * flash than a full 256-entry table. Values are round(v * 32768); every entry
 * is strictly below 1.0 so all of them fit i1.15 without clamping.
 */
static constexpr int16_t MIDI_EXP2_Q15[17] = {
    32767, 31379, 30048, 28774, 27554, 26386, 25268, 24196, 23170,
    22188, 21247, 20347, 19484, 18658, 17867, 17109, 16384
};

/**
 * @brief Fixed-point 2^-y for y in [0, 8), argument in Q4.8.
 *
 * Computes exp2(-y) = exp2(-int(y)) * exp2(-frac(y)) entirely in integers:
 * the fractional part walks the LUT with 4-bit-interpolated linear
 * interpolation, the integer part is an arithmetic right shift (each unit
 * halves the result). One multiply, one shift - no float, no division.
 *
 * @param y_q4_8  Argument as 8.8 fixed point, 0..2047 (values >= 2048 clamp to 0).
 * @return exp2(-y) in i1.15 (32767 = 1.0, 0 = silence).
 */
static inline int16_t midi_exp2_neg_q15(uint32_t y_q4_8) {
    if (y_q4_8 >= 2048u) {
        return 0;                                        // Below -8 octaves: fully decayed
    }
    const uint32_t idx  = (y_q4_8 & 0xFFu) >> 4;         // LUT index (16ths of one octave)
    const uint32_t frac = y_q4_8 & 0x0Fu;                // Sub-table fraction (4 bits)
    const int32_t  lo   = MIDI_EXP2_Q15[idx];            // exp2 at the lower edge
    const int32_t  hi   = MIDI_EXP2_Q15[idx + 1];        // exp2 at the upper edge
    const int32_t  v    = lo - (((lo - hi) * (int32_t)frac) >> 4);  // Interpolate downward
    return (int16_t)(v >> (y_q4_8 >> 8));                // Apply the whole-octave shift
}

/**
 * @brief Q15 sine of a free-running phase, for tremolo / vibrato / trill.
 * @param phase 16-bit phase, 65536 counts = one full turn.
 * @return Sine in i1.15, [-32767, 32767].
 */
static inline int16_t midi_sine_q15(uint16_t phase) {
    int16_t s, c;                                        // Cosine is discarded
    angle2sincos((int16_t)phase, &s, &c);
    return s;
}

/** @brief LFO step per millisecond at 8 Hz (65536 * 8 / 1000). */
static constexpr uint16_t MIDI_LFO_STEP_8HZ = 524u;
/** @brief LFO step per millisecond at 6 Hz (65536 * 6 / 1000). */
static constexpr uint16_t MIDI_LFO_STEP_6HZ = 393u;

// ============================================================================
// Tone generator (runs inside the control ISR)
// ============================================================================

class MotorSpeaker {
private:
    /// Waveforms of the MIDI control command (0xD0 waveform nibble).
    enum Wave : uint8_t {
        SINE     = 0,  ///< Pure rotation, the polar oscillator's native output.
        SQUARE   = 1,  ///< Bang-bang sign of the phase.
        TRIANGLE = 2,  ///< Piecewise-linear, sine-aligned.
        SAWTOOTH = 3   ///< Full-range ramp, zero crossing at 180 deg.
    };

    /// Simultaneous synth voices (the 0xD0 API overlaps up to three notes).
    static constexpr uint8_t VOICES = 3;

    FrequencyGenerator gen_[VOICES];     // One polar oscillator per voice (owns its phase)
    const uint8_t* melody_;
    uint16_t note_counter_;
    int16_t base_amplitude_;
    int16_t current_amplitude_;
    uint8_t melody_length_;
    uint8_t current_note_;
    Wave waveform_;                      // Timbre selected by the 0xD0 waveform nibble
    uint16_t voice_freq_[VOICES];        // Per-voice pitch [Hz]; 0 = silent slot
    int16_t  voice_amp_[VOICES];         // Per-voice amplitude, i1.15
    
    // Motor parameters for RL amplitude compensation
    float r_ohm_;
    float l_h_;

    enum State : uint8_t { IDLE = 0, PLAYING = 1, FINISHED = 2 } state_;

public:
    MotorSpeaker(uint16_t freq_system, int16_t base_amplitude = 3276)
        : gen_{freq_system, freq_system, freq_system}
        , melody_(STARTUP_MELODY), note_counter_(0)
        , base_amplitude_(base_amplitude), current_amplitude_(0)
        , melody_length_(sizeof(STARTUP_MELODY)), current_note_(0)
        , waveform_(SINE), voice_freq_{}, voice_amp_{}
        , r_ohm_(0.0f), l_h_(0.0f), state_(IDLE) {}
        
    void set_motor_params(float r_ohm, float l_h) {
        r_ohm_ = r_ohm;
        l_h_ = l_h;
    }
    
    /** @brief Select the timbre (0xD0 waveform nibble value; invalid values are ignored). */
    void set_waveform(uint8_t w) {
        if (w <= SAWTOOTH) {
            waveform_ = (Wave)w;                         // Valid nibble: adopt it
        }
    }
    
    /**
     * @brief Assign one synth voice (0xD0 notes arrive through the mailbox).
     *
     * A zero frequency or amplitude silences the slot; a pitch above the
     * oscillator's 2000 Hz ceiling is silenced too rather than left stale.
     */
    void set_voice(uint8_t idx, uint16_t freq, int16_t amp) {
        if (idx >= VOICES) {
            return;                                      // No such slot
        }
        if (freq > 2000u) {
            freq = 0u;                                   // Refused pitch: silence, never a stale note
        }
        gen_[idx].set_frequency_target((uint16_t)freq);  // Oscillator tracks the pitch (no-op when 0)
        voice_freq_[idx] = (uint16_t)freq;
        voice_amp_[idx] = amp;
    }
    
    void start() {
        current_note_ = 0;
        state_ = PLAYING;
        if (melody_length_ > 0) load_note(0);
        else state_ = FINISHED;
    }
    
    void stop() {
        state_ = IDLE;
        for (uint8_t i = 0; i < VOICES; ++i) {
            set_voice(i, 0u, 0);                         // Every slot silent
        }
    }
    
    bool is_playing() const { return state_ == PLAYING; }
    bool is_finished() const { return state_ == FINISHED; }
    int16_t get_current_amplitude() const { return voice_amp_[0]; }
    uint16_t get_current_frequency() const { return voice_freq_[0]; }
    
    /**
     * @brief Advance the synth one control pass and return the drive vector.
     * @param override_freq Non-zero to play a continuous test tone (calibration path).
     * @param override_amp  Amplitude in i1.15; -1 applies RL amplitude compensation.
     * @param waveform      0xD0 waveform nibble; 0xFF (default) keeps the selected timbre.
     * @return Alpha-beta vector in i1.15, ready to be scaled by the bus voltage.
     */
    Vec2D tick(uint16_t override_freq = 0, int16_t override_amp = -1, uint8_t waveform = 0xFF) {
        if (waveform != 0xFF) {
            set_waveform(waveform);                      // Per-pass timbre from the mailbox
        }
        if (override_freq > 0) {
            // Manual test tone generation (e.g., for inductance measurement):
            // the single tone owns voice 0, the others stay silent.
            current_amplitude_ = (override_amp >= 0) ? override_amp : calculate_corrected_amplitude(override_freq);
            set_voice(0, override_freq, current_amplitude_);
            set_voice(1, 0u, 0);
            set_voice(2, 0u, 0);
            return tick_poly();
        }
        
        if (state_ != PLAYING) return {0, 0};
        
        if (--note_counter_ == 0) {
            current_note_++;
            if (current_note_ >= melody_length_) {
                state_ = FINISHED;
                set_voice(0, 0u, 0);                     // Melody over: voice 0 silent
                return {0, 0};
            }
            load_note(current_note_);
        }
        
        if (voice_freq_[0] == 0u) return {0, 0};
        return tick_poly();
    }

    /**
     * @brief Sum every active voice into one alpha-beta sample.
     *
     * Per 20 kHz pass: three polar rotations (4 multiplies each) plus three
     * waveform shapings and two saturating adds - about 12 multiplies worst
     * case, integer only, no allocation. The voices share the waveform but
     * not the phase, so overlapping notes beat naturally instead of
     * hard-syncing into one tone.
     *
     * @param waveform 0xD0 waveform nibble; 0xFF (default) keeps the selected timbre.
     * @return Alpha-beta vector, peak-saturated to +/-32767 (i1.15).
     */
    Vec2D tick_poly(uint8_t waveform = 0xFF) {
        if (waveform != 0xFF) {
            set_waveform(waveform);                      // Per-pass timbre from the mailbox
        }
        int32_t a = 0, b = 0;                            // 32-bit accumulators
        for (uint8_t i = 0; i < VOICES; ++i) {
            if ((voice_amp_[i] == 0) || (voice_freq_[i] == 0u)) {
                continue;                                // Silent slot: no rotation, no add
            }
            gen_[i].set_amplitude(voice_amp_[i]);        // Envelope amplitude of this voice
            const Vec2D w = shape(i, gen_[i].tick(), voice_amp_[i]);
            a += w.a;                                    // Accumulate, saturating once
            b += w.b;                                    // at the end (cheaper than per-voice)
        }
        a = (a > 32767) ? 32767 : ((a < -32767) ? -32767 : a);
        b = (b > 32767) ? 32767 : ((b < -32767) ? -32767 : b);
        return {(int16_t)a, (int16_t)b};
    }

private:

    /**
     * @brief Apply the selected timbre to one voice's oscillator sample.
     *
     * The sine needs no work (the polar oscillator already emits it scaled by
     * the amplitude). Every other shape is a pure function of the 16-bit phase:
     * alpha uses the phase as-is, beta leads it by exactly one quadrant, so the
     * two coils stay in quadrature while carrying the same waveform. Shaping is
     * a handful of shifts and one i1.15 multiply per component.
     *
     * @param idx  Voice index (its oscillator owns the phase).
     * @param raw  Sine/cosine pair from the oscillator (i1.15, amplitude-scaled).
     * @param peak Amplitude of this voice (i1.15), the shapes' scale reference.
     * @return Shaped alpha-beta vector, same amplitude envelope as the sine.
     */
    Vec2D shape(uint8_t idx, const Vec2D& raw, int16_t peak) {
        if (waveform_ == SINE) {
            return raw;                                  // Native output, nothing to do
        }
        const uint16_t phase = (uint16_t)gen_[idx].get_angle();  // This voice's phase accumulator
        Vec2D out{0, 0};                                 // Vec2D has no default constructor
        switch (waveform_) {
            case SQUARE: {
                // Sign of the phase, aligned so the positive half matches the sine's:
                // one compare and a negate per component.
                out.a = (phase < 0x8000u) ? peak : (int16_t)-peak;
                out.b = ((uint16_t)(phase + 0x4000u) < 0x8000u) ? peak : (int16_t)-peak;
                break;
            }
            case TRIANGLE: {
                // Piecewise linear rise/fall locked to the sine's extrema:
                // peak at 90 deg, zero at 180 deg, trough at 270 deg.
                out.a = (int16_t)(((int32_t)wave_triangle(phase) * peak) >> 15);
                out.b = (int16_t)(((int32_t)wave_triangle((uint16_t)(phase + 0x4000u)) * peak) >> 15);
                break;
            }
            case SAWTOOTH:
            default: {
                // Full-range ramp from -max to +max, zero crossing at 180 deg.
                out.a = (int16_t)(((int32_t)wave_sawtooth(phase) * peak) >> 15);
                out.b = (int16_t)(((int32_t)wave_sawtooth((uint16_t)(phase + 0x4000u)) * peak) >> 15);
                break;
            }
        }
        return out;
    }

    /** @brief Sine-aligned triangle in i1.15 from the raw 16-bit phase. */
    static int16_t wave_triangle(uint16_t q) {
        if (q < 0x4000u) {
            return (int16_t)(q << 1);                          // Rising: 0 -> +max over Q1
        }
        if (q < 0xC000u) {
            return (int16_t)(0x7FFFu - ((q - 0x4000u) << 1));  // Falling: +max -> -max over Q2+Q3
        }
        return (int16_t)(-0x7FFF + ((q - 0xC000u) << 1));      // Rising: -max -> 0 over Q4
    }

    /** @brief Sawtooth in i1.15: ramp from -max to +max, zero at 180 deg. */
    static int16_t wave_sawtooth(uint16_t q) {
        return (int16_t)(q - 0x8000u);                         // Reinterpret the phase as a signed ramp
    }

    int16_t calculate_corrected_amplitude(uint16_t freq) {
        if (freq == 0) return 0;
        // Amplitude correction to maintain constant current (volume) over inductive load:
        // V_amp = Base_V_amp * (Z_f / Z_base) -> simplified to Z_f / R
        if (r_ohm_ > 0.0f && l_h_ > 0.0f) {
            float wL = 2.0f * 3.14159f * freq * l_h_;
            float z = sqrtf(r_ohm_ * r_ohm_ + wL * wL);
            float correction = z / r_ohm_;
            int32_t new_amp = (int32_t)(base_amplitude_ * correction);
            return (new_amp > 32767) ? 32767 : (int16_t)new_amp;
        }
        // Fallback simple linear correction
        int32_t new_amp = base_amplitude_ + (base_amplitude_ * freq * 2) / 1000;
        return (new_amp > 32767) ? 32767 : (int16_t)new_amp;
    }

    void load_note(uint8_t index) {
        uint8_t packed_note = melody_[index];
        uint8_t pitch_id = packed_note >> 3;
        uint8_t len_id   = packed_note & 0x07;
        
        note_counter_ = 500 << len_id; 
        uint16_t freq = Notes::FREQ_LUT[pitch_id];
        
        current_amplitude_ = calculate_corrected_amplitude(freq);
        set_voice(0, freq, current_amplitude_);          // The melody plays on voice 0
    }
};