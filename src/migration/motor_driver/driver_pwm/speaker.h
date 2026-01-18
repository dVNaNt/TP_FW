#pragma once

// Motor speaker - plays melody through motor coils using frequency generator
// Uses motor as a speaker by injecting audio frequency signals

#include "../../common.h"
#include "../../math_integer/helpers/frequency_generator.h"

// Musical note frequencies in Hz (octave 4-5 range, 300-1500Hz)
namespace Notes {
    static constexpr uint16_t C4  = 262;   // Do
    static constexpr uint16_t D4  = 294;   // Re
    static constexpr uint16_t E4  = 330;   // Mi
    static constexpr uint16_t F4  = 349;   // Fa
    static constexpr uint16_t G4  = 392;   // Sol
    static constexpr uint16_t A4  = 440;   // La
    static constexpr uint16_t B4  = 494;   // Si
    static constexpr uint16_t C5  = 523;   // Do (octave 5)
    static constexpr uint16_t D5  = 587;   // Re
    static constexpr uint16_t E5  = 659;   // Mi
    static constexpr uint16_t F5  = 698;   // Fa
    static constexpr uint16_t G5  = 784;   // Sol
    static constexpr uint16_t A5  = 880;   // La
    static constexpr uint16_t B5  = 988;   // Si
    static constexpr uint16_t C6  = 1047;  // Do (octave 6)
    static constexpr uint16_t D6  = 1175;  // Re
    static constexpr uint16_t E6  = 1319;  // Mi
    static constexpr uint16_t REST = 0;    // Silence/pause
}

// Note duration in ticks (at 20kHz, 1 tick = 50us)
// 20000 ticks = 1 second
namespace Duration {
    static constexpr uint16_t WHOLE     = 8000;  // 400ms - whole note
    static constexpr uint16_t HALF      = 4000;  // 200ms - half note
    static constexpr uint16_t QUARTER   = 2000;  // 100ms - quarter note
    static constexpr uint16_t EIGHTH    = 1000;  // 50ms - eighth note
    static constexpr uint16_t SIXTEENTH = 500;   // 25ms - sixteenth note
}

// Single note structure
struct Note {
    uint16_t frequency;  // Frequency in Hz (0 = rest)
    uint16_t duration;   // Duration in ticks
};

// Startup melody - "Ode to Joy" theme (Beethoven) simplified
// Total duration ~20 seconds at 20kHz tick rate
static const Note STARTUP_MELODY[] = {
    // First phrase (Ode to Joy main theme)
    {Notes::E5, Duration::QUARTER}, {Notes::E5, Duration::QUARTER},
    {Notes::F5, Duration::QUARTER}, {Notes::G5, Duration::QUARTER},
    {Notes::G5, Duration::QUARTER}, {Notes::F5, Duration::QUARTER},
    {Notes::E5, Duration::QUARTER}, {Notes::D5, Duration::QUARTER},
    {Notes::C5, Duration::QUARTER}, {Notes::C5, Duration::QUARTER},
    {Notes::D5, Duration::QUARTER}, {Notes::E5, Duration::QUARTER},
    {Notes::E5, Duration::HALF},    {Notes::D5, Duration::QUARTER},
    {Notes::D5, Duration::HALF},
    {Notes::REST, Duration::QUARTER},
    
    // Second phrase
    {Notes::E5, Duration::QUARTER}, {Notes::E5, Duration::QUARTER},
    {Notes::F5, Duration::QUARTER}, {Notes::G5, Duration::QUARTER},
    {Notes::G5, Duration::QUARTER}, {Notes::F5, Duration::QUARTER},
    {Notes::E5, Duration::QUARTER}, {Notes::D5, Duration::QUARTER},
    {Notes::C5, Duration::QUARTER}, {Notes::C5, Duration::QUARTER},
    {Notes::D5, Duration::QUARTER}, {Notes::E5, Duration::QUARTER},
    {Notes::D5, Duration::HALF},    {Notes::C5, Duration::QUARTER},
    {Notes::C5, Duration::HALF},
    {Notes::REST, Duration::QUARTER},
    
    // Bridge section (variation)
    {Notes::D5, Duration::QUARTER}, {Notes::D5, Duration::QUARTER},
    {Notes::E5, Duration::QUARTER}, {Notes::C5, Duration::QUARTER},
    {Notes::D5, Duration::QUARTER}, {Notes::E5, Duration::EIGHTH},
    {Notes::F5, Duration::EIGHTH},  {Notes::E5, Duration::QUARTER},
    {Notes::C5, Duration::QUARTER}, {Notes::D5, Duration::QUARTER},
    {Notes::E5, Duration::EIGHTH},  {Notes::F5, Duration::EIGHTH},
    {Notes::E5, Duration::QUARTER}, {Notes::D5, Duration::QUARTER},
    {Notes::C5, Duration::QUARTER}, {Notes::D5, Duration::QUARTER},
    {Notes::G4, Duration::HALF},
    {Notes::REST, Duration::QUARTER},
    
    // Final phrase (repeat main theme)
    {Notes::E5, Duration::QUARTER}, {Notes::E5, Duration::QUARTER},
    {Notes::F5, Duration::QUARTER}, {Notes::G5, Duration::QUARTER},
    {Notes::G5, Duration::QUARTER}, {Notes::F5, Duration::QUARTER},
    {Notes::E5, Duration::QUARTER}, {Notes::D5, Duration::QUARTER},
    {Notes::C5, Duration::QUARTER}, {Notes::C5, Duration::QUARTER},
    {Notes::D5, Duration::QUARTER}, {Notes::E5, Duration::QUARTER},
    {Notes::D5, Duration::HALF},    {Notes::C5, Duration::QUARTER},
    {Notes::C5, Duration::WHOLE},
    
    // Ending flourish
    {Notes::REST, Duration::QUARTER},
    {Notes::G5, Duration::EIGHTH},  {Notes::E5, Duration::EIGHTH},
    {Notes::C5, Duration::QUARTER}, {Notes::G5, Duration::QUARTER},
    {Notes::C6, Duration::WHOLE},
};

// Calculate melody length at compile time
static constexpr uint8_t MELODY_LENGTH = sizeof(STARTUP_MELODY) / sizeof(Note);

// Motor speaker class - plays melody through motor coils
struct MotorSpeaker {
private:
    FrequencyGenerator freq_gen_;  // Frequency generator for tone generation
    const Note* melody_;           // Pointer to melody array
    uint8_t melody_length_;        // Number of notes in melody
    uint8_t current_note_;         // Current note index
    uint16_t note_counter_;        // Tick counter for current note duration
    int16_t amplitude_;            // Output amplitude (volume)
    bool playing_;                 // Is melody currently playing
    bool finished_;                // Has melody finished playing

public:
    // Constructor
    // @param freq_system System frequency in Hz (e.g., 20000 for 20kHz PWM)
    // @param amplitude Output amplitude in i1.15 format (default ~10% for safety)
    MotorSpeaker(uint16_t freq_system, int16_t amplitude = 3276)
        : freq_gen_(freq_system)
        , melody_(STARTUP_MELODY)
        , melody_length_(MELODY_LENGTH)
        , current_note_(0)
        , note_counter_(0)
        , amplitude_(amplitude)
        , playing_(false)
        , finished_(false)
    {
        freq_gen_.set_amplitude(amplitude_);  // Set initial amplitude
    }
    
    // Start playing the melody
    void start() {
        current_note_ = 0;           // Reset to first note
        note_counter_ = 0;           // Reset tick counter
        playing_ = true;             // Enable playback
        finished_ = false;           // Reset finished flag
        
        // Load first note
        if (melody_length_ > 0) {
            load_note(0);            // Load first note into generator
        }
    }
    
    // Stop playing
    void stop() {
        playing_ = false;            // Disable playback
        freq_gen_.set_frequency_target(0);  // Silence
    }
    
    // Check if melody is currently playing
    bool is_playing() const {
        return playing_;
    }
    
    // Check if melody has finished
    bool is_finished() const {
        return finished_;
    }
    
    // Set playback amplitude (volume)
    // @param amplitude Volume in i1.15 format (0-32767)
    void set_amplitude(int16_t amplitude) {
        amplitude_ = amplitude;
        freq_gen_.set_amplitude(amplitude_);
    }
    
    // Main tick function - call at system frequency rate
    // @return Vec2D voltage output for motor (inject into AB frame)
    Vec2D tick() {
        if (!playing_) {
            return Vec2D(0, 0);       // Return zero if not playing
        }
        
        // Increment note duration counter
        note_counter_++;
        
        // Check if current note duration has elapsed
        if (note_counter_ >= melody_[current_note_].duration) {
            note_counter_ = 0;       // Reset counter
            current_note_++;         // Move to next note
            
            // Check if melody is complete
            if (current_note_ >= melody_length_) {
                playing_ = false;    // Stop playback
                finished_ = true;    // Mark as finished
                freq_gen_.set_frequency_target(0);  // Silence
                return Vec2D(0, 0);
            }
            
            // Load next note
            load_note(current_note_);
        }
        
        // Generate waveform if not a rest note
        if (melody_[current_note_].frequency > 0) {
            return freq_gen_.tick(); // Return sin/cos waveform
        }
        
        return Vec2D(0, 0);          // Rest note - return silence
    }
    
    // Get current note index (for debugging)
    uint8_t get_current_note() const {
        return current_note_;
    }
    
    // Get current frequency (for debugging)
    uint16_t get_current_frequency() const {
        return freq_gen_.get_frequency_target();
    }
    
    // Get progress as percentage (0-100)
    uint8_t get_progress() const {
        if (melody_length_ == 0) return 100;
        return (uint8_t)((current_note_ * 100) / melody_length_);
    }

private:
    // Load a note into the frequency generator
    // @param index Note index in melody array
    void load_note(uint8_t index) {
        if (index >= melody_length_) return;
        
        uint16_t freq = melody_[index].frequency;
        
        if (freq > 0 && freq < 2000) {
            freq_gen_.set_frequency_target(freq);  // Set note frequency
            freq_gen_.set_amplitude(amplitude_);   // Restore amplitude
        } else {
            freq_gen_.set_frequency_target(0);     // Rest - silence
        }
    }
};
