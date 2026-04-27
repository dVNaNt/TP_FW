#pragma once

// Motor speaker - plays melody through motor coils and provides test tones for HFI/Inductance measurement
// Includes dynamic amplitude correction to overcome inductive impedance at high frequencies.

#include "../../common.h"
#include "../../math/helpers/frequency_generator.h"
#include <cmath>

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

class MotorSpeaker {
private:
    FrequencyGenerator freq_gen_;
    const uint8_t* melody_;
    uint16_t note_counter_;
    int16_t base_amplitude_;
    int16_t current_amplitude_;
    uint8_t melody_length_;
    uint8_t current_note_;
    
    // Motor parameters for RL amplitude compensation
    float r_ohm_;
    float l_h_;

    enum State : uint8_t { IDLE = 0, PLAYING = 1, FINISHED = 2 } state_;

public:
    MotorSpeaker(uint16_t freq_system, int16_t base_amplitude = 3276)
        : freq_gen_(freq_system), melody_(STARTUP_MELODY), note_counter_(0)
        , base_amplitude_(base_amplitude), current_amplitude_(0)
        , melody_length_(sizeof(STARTUP_MELODY)), current_note_(0)
        , r_ohm_(0.0f), l_h_(0.0f), state_(IDLE) {}
        
    void set_motor_params(float r_ohm, float l_h) {
        r_ohm_ = r_ohm;
        l_h_ = l_h;
    }
    
    void start() {
        current_note_ = 0;
        state_ = PLAYING;
        if (melody_length_ > 0) load_note(0);
        else state_ = FINISHED;
    }
    
    void stop() {
        state_ = IDLE;
        freq_gen_.set_frequency_target(0);
    }
    
    bool is_playing() const { return state_ == PLAYING; }
    bool is_finished() const { return state_ == FINISHED; }
    int16_t get_current_amplitude() const { return current_amplitude_; }
    uint16_t get_current_frequency() const { return freq_gen_.get_frequency_target(); }
    
    // Supports continuous melody playing or an instant manual override for calibration tests
    Vec2D tick(uint16_t override_freq = 0, int16_t override_amp = -1) {
        if (override_freq > 0) {
            // Manual test tone generation (e.g., for inductance measurement)
            freq_gen_.set_frequency_target(override_freq);
            current_amplitude_ = (override_amp >= 0) ? override_amp : calculate_corrected_amplitude(override_freq);
            freq_gen_.set_amplitude(current_amplitude_);
            return freq_gen_.tick();
        }
        
        if (state_ != PLAYING) return {0, 0};
        
        if (--note_counter_ == 0) {
            current_note_++;
            if (current_note_ >= melody_length_) {
                state_ = FINISHED;
                freq_gen_.set_frequency_target(0);
                return {0, 0};
            }
            load_note(current_note_);
        }
        
        if (freq_gen_.get_frequency_target() == 0) return {0, 0};
        return freq_gen_.tick();
    }

private:
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
        
        freq_gen_.set_frequency_target(freq);
        current_amplitude_ = calculate_corrected_amplitude(freq);
        freq_gen_.set_amplitude(current_amplitude_);
    }
};