/**
 * @file encoder.h
 * @brief Position integrator with ISR-safe ingestion, HW callbacks, and an
 *        atomic encoder-fault flag with a fault-action callback.
 *
 * @copyright Copyright 2026, creapunk
 * @license Apache-2.0
 */

#pragma once

#include <atomic>
#include "../../common.h"

/** Signature of a parameterless encoder lifecycle callback (start/stop). */
typedef void (*EncoderCallback)();

/** Signature of the encoder fault action callback (executed when the fault is raised). */
typedef void (*EncoderFaultCallback)();

/**
 * @struct Encoder
 * @brief Encoder position tracking with ISR-safe ingestion, lock-free reading,
 *        and an atomic fault flag driving a registered fault-action callback.
 */
struct Encoder {
private:
    static inline void empty_callback() {}

    EncoderCallback start_callback_ = &empty_callback;  ///< Invoked by start() (typically enables the SPI polling ISR).
    EncoderCallback stop_callback_  = &empty_callback;  ///< Invoked by stop() (typically disables the polling ISR).
    EncoderFaultCallback fault_cb_  = nullptr;          ///< Invoked on the false->true fault transition (fault action).

    std::atomic<bool> encoder_fault_{false};  ///< Fault flag (atomic setter/getter, ISR-safe)

    // Union to access the 64-bit position as two 32-bit halves.
    // Note: Assumes Little-Endian architecture (standard for ARM Cortex-M, ESP32, etc.).
    // In Little-Endian, the first member in memory is the lower 32 bits.
    union PositionData {
        int64_t value;
        struct {
            uint32_t angle;     // Lower 32 bits
            int32_t rotations;  // Upper 32 bits
        } parts;
    };

    // Align to 8 bytes to ensure both halves are on their natural boundaries,
    // allowing single-instruction 32-bit atomic reads.
    alignas(8) PositionData position_{0};  ///< Current integrated position (angle + rotations).
    
    // SeqLock counter for consistent 64-bit reads only.
    /// Sequence lock: odd = write in progress, even = stable. Single writer.
    std::atomic<uint32_t> seq_{0};

    static constexpr uint32_t kTrustAfter = 256;  ///< Consecutive valid frames that prove the integrity check.
    static constexpr uint16_t kFaultRun = 64;     ///< Consecutive failures that mean the sensor is gone (3.2 ms at 20 kHz).
    uint32_t frames_ok_ = 0;                      ///< Frames that passed the check since boot.
    uint32_t frames_bad_ = 0;                     ///< Frames that failed it since boot.
    uint16_t bad_run_ = 0;                        ///< Consecutive failures right now.
    uint16_t bad_run_max_ = 0;                    ///< Longest failure run since boot.
    uint16_t good_run_ = 0;                       ///< Consecutive passes while the link is proving its recovery.
    uint16_t relinks_ = 0;                        ///< Bus resets requested since boot.
    uint8_t sensor_status_ = 0;                   ///< Sensor status bits of the last accepted frame.
    bool check_trusted_ = false;                  ///< Check proven on this boot: failing frames are dropped.
    bool link_down_ = false;                      ///< Link declared down: ingestion suspended, fault raised.
    bool relink_request_ = false;                 ///< A bus reset is owed to the owner.

public:
    /// Default constructor: zeroed position, empty callbacks.
    Encoder() {}

    /// Invoke the registered start callback (enable acquisition hardware).
    inline void start() { start_callback_(); }
    /// Invoke the registered stop callback (disable acquisition hardware).
    inline void stop()  { stop_callback_(); }

    // =======================================================================
    // ISR-safe ingestion (Single Writer)
    // =======================================================================

    /**
     * @brief Ingest a new raw encoder sample and integrate it into the
     *        64-bit position.
     *
     * Called from the single-writer context (SPI/ISR). The unwrapping uses
     * the wrap-around property of int32 arithmetic:
     * diff = (int32_t)(input_pos - last_low_word), so multi-revolution motion
     * accumulates correctly into the rotations half.
     *
     * @param input_pos Latest raw angle word from the sensor [0..2^32-1].
     */
    inline void set_raw_position(uint32_t input_pos) {
        // --- Begin Write ---
        seq_.fetch_add(1, std::memory_order_relaxed); // Makes counter odd
        std::atomic_thread_fence(std::memory_order_release);
        
        // Read current 64-bit value, calculate new 64-bit value
        int64_t curr = position_.value;
        int32_t diff = (int32_t)(input_pos - (uint32_t)curr);
        position_.value = curr + diff; // Single 64-bit write (compiles to two 32-bit STRs)

        std::atomic_thread_fence(std::memory_order_release);
        seq_.fetch_add(1, std::memory_order_relaxed); // Makes counter even
    }

    // =======================================================================
    // Frame health (single writer: the acquisition ISR)
    // =======================================================================

    /**
     * @brief Decide whether a freshly received sensor frame may be ingested.
     *
     * A frame that fails the sensor's own integrity check is not a position:
     * it is a bus glitch, a transfer cut short by a preempting interrupt, or
     * a sensor that stopped answering (an idle-high data line reads as the
     * top of the range). Integrating such a frame counts a whole revolution
     * whenever the garbage lands on the far side of the half-turn boundary,
     * so a run of them walked the turn counter hundreds of turns away from a
     * stationary rotor and left the position loop chasing a phantom. Bad
     * frames are therefore dropped and the last good angle stands for that
     * control period.
     *
     * The check has to prove itself before it may drop anything: if the
     * caller's CRC ever disagreed with the sensor (wrong polynomial, byte
     * order), rejecting every frame would blind the drive outright. Frames are
     * only rejected once kTrustAfter consecutive frames have passed on this
     * boot with no failure before them.
     *
     * A link that fails for kFaultRun consecutive frames is DOWN: the fault
     * flag is raised (the control loop turns it into a driver fault, so the
     * bridge goes Hi-Z and enabling is refused) and NOTHING is ingested any
     * more, not even a frame that happens to pass - one corrupted frame in
     * 256 carries a matching CRC by chance, and on a persistently broken
     * link those few dozen a second were enough to walk the turn counter
     * away by themselves. The owner is asked to re-establish the link (the
     * board resets its bus peripheral) once per kFaultRun failures while it
     * stays down. The link is up again, and ingestion resumes, only after
     * kTrustAfter consecutive valid frames.
     *
     * @param valid true when the frame passed the sensor's integrity check.
     * @return true when the frame should be ingested (set_raw_position()).
     */
    inline bool accept_frame(bool valid) {
        if (valid) {
            frames_ok_++;                                        // Statistics
            if (!check_trusted_ && (frames_ok_ >= kTrustAfter) && (frames_bad_ == 0u)) {
                check_trusted_ = true;                           // The check agrees with the sensor: arm the drop
            }
            bad_run_ = 0;                                        // Sensor is answering
            if (link_down_) {
                if (++good_run_ >= kTrustAfter) {
                    link_down_ = false;                          // Link re-established
                    good_run_ = 0;
                    set_fault(false);                            // Feedback is trustworthy again
                } else {
                    return false;                                // Still proving the recovered link
                }
            }
            return true;
        }
        frames_bad_++;                                           // Statistics
        good_run_ = 0;                                           // A recovering link starts over
        if (bad_run_ < UINT16_MAX) {
            bad_run_++;                                          // Length of the current failure run
        }
        if (bad_run_ > bad_run_max_) {
            bad_run_max_ = bad_run_;                             // Worst run seen, for the diagnostics
        }
        if (check_trusted_ && (bad_run_ % kFaultRun) == 0u) {
            link_down_ = true;                                   // No valid frame for kFaultRun periods
            relink_request_ = true;                              // Ask the owner to reset the bus (once per run)
            set_fault(true);
        }
        return !check_trusted_;                                  // Unproven check: ingest as before
    }

    /**
     * @brief Hand over a pending request to re-establish the link.
     *
     * The owner (the board's acquisition handler) calls this after every
     * accept_frame() and, when it returns true, resets the sensor bus. One
     * request is raised each time the link has been down for another
     * kFaultRun frames, so a dead sensor is retried a few hundred times a
     * second at a cost of one bus reset each, never in a tight loop.
     *
     * @return true exactly once per request.
     */
    inline bool take_relink_request() {
        if (!relink_request_) {
            return false;
        }
        relink_request_ = false;
        if (relinks_ < UINT16_MAX) {
            relinks_++;                                          // Bus resets since boot
        }
        return true;
    }

    /// Publish the sensor's own status bits from the last accepted frame (sensor specific).
    inline void set_sensor_status(uint8_t status) { sensor_status_ = status; }
    /// Sensor status bits of the last accepted frame.
    inline uint8_t sensor_status() const { return sensor_status_; }
    /// Frames that passed the integrity check since boot.
    inline uint32_t frames_ok() const { return frames_ok_; }
    /// Frames that failed it since boot.
    inline uint32_t frames_bad() const { return frames_bad_; }
    /// Longest run of consecutive failures since boot.
    inline uint16_t bad_run_max() const { return bad_run_max_; }
    /// Bus resets requested since boot (each one after kFaultRun consecutive failures).
    inline uint16_t relinks() const { return relinks_; }
    /// True once the check has proven itself and failing frames are dropped.
    inline bool check_trusted() const { return check_trusted_; }
    /// True while the link is down: nothing is ingested and the fault flag is raised.
    inline bool link_down() const { return link_down_; }

    // =======================================================================
    // Fast 32-bit reads (Direct memory access)
    // =======================================================================

    /**
     * @brief Get the "clean" angle (32 bits, left-aligned).
     * @note Ultra-fast: single atomic 32-bit hardware load. No SeqLock loop needed.
     */
    inline uint32_t get_angle() const { 
        return position_.parts.angle; 
    }

    /**
     * @brief Get the clean full rotations (32 bits, signed).
     * @note Ultra-fast: single atomic 32-bit hardware load. No SeqLock loop needed.
     */
    inline int32_t get_rotations() const { 
        return position_.parts.rotations; 
    }

    // =======================================================================
    // Consistent 64-bit reads (Multiple Readers)
    // =======================================================================

private:
    /**
     * @brief Lock-free thread-safe read of the full 64-bit position.
     * Uses SeqLock to guarantee the upper and lower 32 bits are consistent.
     */
    inline int64_t load_position_64() const {
        uint32_t s1, s2;
        int64_t pos;
        do {
            s1 = seq_.load(std::memory_order_relaxed);
            std::atomic_thread_fence(std::memory_order_acquire);
            
            pos = position_.value; // Read the raw 64-bit value
            
            std::atomic_thread_fence(std::memory_order_acquire);
            s2 = seq_.load(std::memory_order_relaxed);
        } while (s1 != s2 || (s1 & 1));
        return pos;
    }

public:
    /**
     * @brief Combined position in Q16.16 format (spans both 32-bit halves).
     * @note Requires SeqLock because it uses bits from both halves.
     */
    inline int32_t get_position() const { 
        return (int32_t)(load_position_64() >> 16); 
    }

    /**
     * @brief Get the raw 64-bit position for filters (e.g., EMA/PLL).
     * @note Requires SeqLock.
     */
    inline int64_t get_position_64() const { 
        return load_position_64(); 
    }


    // =======================================================================
    // ATOMIC FAULT FLAG (encoder_fault)
    // =======================================================================

    /**
     * @brief Set or clear the encoder fault flag (atomic, ISR-safe).
     *
     * Uses an atomic exchange so the flag is always consistent regardless of
     * the calling context (ISR or main thread). On a false->true transition
     * the registered fault-action callback fires exactly once — clearing the
     * fault never invokes it.
     *
     * @param state true to raise the fault (fires fault_cb_ on rising edge),
     *              false to clear it
     */
    inline void set_fault(bool state) {
        // exchange returns the previous value; fire only on an actual rising edge
        if (encoder_fault_.exchange(state, std::memory_order_relaxed) != state) {
            if (state && fault_cb_)
                fault_cb_();  // Execute the registered fault action (single fire)
        }
    }

    /**
     * @brief Check the encoder fault flag (atomic, ISR-safe).
     * @return true if the fault flag is currently set
     */
    inline bool is_fault() const { return encoder_fault_.load(std::memory_order_relaxed); }

    // =======================================================================
    // FLUENT CONFIGURATION API
    // =======================================================================

    /** Fluent proxy for registering start/stop/fault callbacks. */
    struct CallbackSetter {
    private:
        Encoder& parent;  ///< Owning encoder instance.
    public:
        CallbackSetter(Encoder& p) : parent(p) {}

        /// Register the start callback (nullptr restores the empty no-op). Chainable.
        inline CallbackSetter& start_cb(EncoderCallback cb) {
            parent.start_callback_ = cb ? cb : &Encoder::empty_callback;
            return *this;
        }
        /// Register the stop callback (nullptr restores the empty no-op). Chainable.
        inline CallbackSetter& stop_cb(EncoderCallback cb) {
            parent.stop_callback_ = cb ? cb : &Encoder::empty_callback;
            return *this;
        }
        /// Register the fault action callback (nullptr = no action on fault). Chainable.
        inline CallbackSetter& fault_cb(EncoderFaultCallback cb) {
            parent.fault_cb_ = cb;  // Null is allowed: fault raising simply performs no action
            return *this;
        }
    };

    /// Entry point of the fluent callback registration API.
    inline CallbackSetter set_callback() { return CallbackSetter(*this); }
};