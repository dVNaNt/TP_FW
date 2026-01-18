#pragma once  // Ensure this header is included only once during compilation

#include "../../common.h"  // Include common migration definitions and types
#include "../../math_integer/filters/filter1st.h"  // Include first-order filter for demodulation
#include "../../math_integer/helpers/frequency_generator.h"  // Include frequency generator

// ============================================================================
// HFI Sense - Heterodyne Demodulation for High Frequency Injection (Single Axis)
// ============================================================================
// Implements heterodyne demodulation to extract position information from
// high-frequency current response in sensorless motor control.
//
// Single-axis implementation:
// - Generator provides cosine signal for voltage injection
// - Method receives sine signal for current demodulation
// - Extracts single demodulated output (position-dependent signal)
//
// Algorithm:
// 1. Multiply measured current by reference sine (mixing)
// 2. Low-pass filter to extract DC component (demodulated signal)
// 3. Output represents rotor position-dependent signal
//
// The LPF alpha is calculated to filter out the carrier frequency while
// preserving the modulation envelope.
//
// Input:
//   - Current measurement (single axis, Q15)
//   - Reference sine from frequency generator (Q15)
//
// Output:
//   - Demodulated signal (filtered position-dependent component)
// ============================================================================

struct HFISense {
private:
    // ---------------------------------------------------------------------------
    // Private data members
    // ---------------------------------------------------------------------------
    
    Filter1st input_hpf_;  // High-pass filter to remove DC component from input current
    Filter1st demod_lpf_;  // Low-pass filter for demodulated signal
    
    int16_t demod_output_;  // Demodulated output signal (Q15 format)
    
    uint16_t carrier_freq_hz_;  // Carrier frequency in Hz
    uint16_t system_freq_hz_;   // System frequency in Hz (sampling rate)

public:
    // ---------------------------------------------------------------------------
    // Public interface
    // ---------------------------------------------------------------------------
    
    /// Constructor: Initialize HFI sense with carrier and system frequencies
    /// @param carrier_freq_hz Carrier frequency in Hz (HFI injection frequency)
    /// @param system_freq_hz System frequency in Hz (sampling rate, e.g., 20000 Hz)
    HFISense(uint16_t carrier_freq_hz, uint16_t system_freq_hz)
        : input_hpf_()  // Initialize input high-pass filter with default constructor
        , demod_lpf_()  // Initialize demodulation low-pass filter with default constructor
        , demod_output_(0)  // Initialize demodulated output to zero
        , carrier_freq_hz_(carrier_freq_hz)  // Store carrier frequency
        , system_freq_hz_(system_freq_hz)    // Store system frequency
    {
        // Set cutoff frequency for input high-pass filter (cutoff at fc/10 to remove DC)
        input_hpf_.set_cutoff(carrier_freq_hz_ / 10, system_freq_hz_);  // Configure HPF using set_cutoff method
        
        // Set cutoff frequency for demodulation low-pass filter (cutoff at fc/6)
        demod_lpf_.set_cutoff(carrier_freq_hz_ / 6, system_freq_hz_);  // Configure LPF using set_cutoff method
    }
    
    /// Main processing: Heterodyne demodulation of current signal (single axis)
    /// @param current Measured current (single axis, Q15 format)
    /// @param ref_sin Reference sine from generator (Q15 format)
    void tick(int16_t current, int16_t ref_sin) {
        // Remove DC component from input current using high-pass filter
        int16_t current_ac = input_hpf_.tick_hpf(current);  // Filter out DC offset
        
        // Heterodyne mixing: multiply AC current by reference sine
        // This extracts the position-dependent component from HFI response
        int32_t mixed = ((int32_t)current_ac * (int32_t)ref_sin) >> 15;  // Q15 * Q15 -> Q15
        
        // Saturate to int16_t range (should not overflow in practice)
        if (mixed > INT16_MAX) mixed = INT16_MAX;  // Clamp to maximum
        if (mixed < INT16_MIN) mixed = INT16_MIN;  // Clamp to minimum
        
        // Low-pass filter to extract DC component (demodulated signal)
        demod_lpf_.tick_lpf((int16_t)mixed);  // Filter mixed signal
        
        // Store filtered output
        demod_output_ = demod_lpf_.get_output();  // Get demodulated signal
    }
    
    /// Get demodulated output signal
    /// @return Demodulated signal in Q15 format (position-dependent component)
    inline int16_t get_output() const {
        return demod_output_;  // Return filtered demodulated signal
    }
    
    /// Update carrier frequency and recalculate filter cutoffs
    /// @param carrier_freq_hz New carrier frequency in Hz
    void set_carrier_frequency(uint16_t carrier_freq_hz) {
        carrier_freq_hz_ = carrier_freq_hz;  // Update carrier frequency
        input_hpf_.set_cutoff(carrier_freq_hz_ / 10, system_freq_hz_);  // Recalculate HPF cutoff (fc/10)
        demod_lpf_.set_cutoff(carrier_freq_hz_ / 6, system_freq_hz_);  // Recalculate LPF cutoff (fc/6)
    }
    
    /// Update system frequency and recalculate filter cutoffs
    /// @param system_freq_hz New system frequency in Hz
    void set_system_frequency(uint16_t system_freq_hz) {
        system_freq_hz_ = system_freq_hz;  // Update system frequency
        input_hpf_.set_cutoff(carrier_freq_hz_ / 10, system_freq_hz_);  // Recalculate HPF cutoff (fc/10)
        demod_lpf_.set_cutoff(carrier_freq_hz_ / 6, system_freq_hz_);  // Recalculate LPF cutoff (fc/6)
    }
    
    /// Reset filters to zero state
    void reset() {
        input_hpf_.reset(0);  // Reset input high-pass filter to zero
        demod_lpf_.reset(0);  // Reset demodulation low-pass filter to zero
        demod_output_ = 0;    // Clear demodulated output
    }
    
    /// Get current carrier frequency
    /// @return Carrier frequency in Hz
    inline uint16_t get_carrier_frequency() const {
        return carrier_freq_hz_;  // Return carrier frequency
    }
    
    /// Get current system frequency
    /// @return System frequency in Hz
    inline uint16_t get_system_frequency() const {
        return system_freq_hz_;  // Return system frequency
    }
};  // End of HFISense structure definition
