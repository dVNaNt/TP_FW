#pragma once // Prevents multiple inclusions of this header file during compilation

#include <cstdint>
#include <type_traits> // For static_assert

#include "motion_executor.h" // Includes the low-level structures and executor

// ========================================================================
// 1. FLAGS & BITMASKS
// ========================================================================

/// @brief Flags defining the control mode and active physical parameters for a MotionTask
enum class MotionFlag : uint32_t {
    MODE_ABSOLUTE = 0,          // Bit 0: 0 (Absolute mode)
    MODE_RELATIVE = (1 << 0),   // Bit 0: 1 (Relative mode)
    
    DO_POS        = (1 << 1),   // Bit 1: Position control active
    DO_VEL        = (1 << 2),   // Bit 2: Velocity control active
    DO_ACC        = (1 << 3),   // Bit 3: Acceleration control active
    DO_JRK        = (1 << 4),   // Bit 4: Jerk control active
    DO_TRQ        = (1 << 5)    // Bit 5: Torque/Current control active
};

// Bitwise operators to safely combine flags (e.g., DO_POS | DO_VEL)
inline constexpr MotionFlag operator|(MotionFlag a, MotionFlag b) {
    return static_cast<MotionFlag>(static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
}
inline constexpr MotionFlag operator&(MotionFlag a, MotionFlag b) {
    return static_cast<MotionFlag>(static_cast<uint32_t>(a) & static_cast<uint32_t>(b));
}

// ========================================================================
// 2. PARSER & COMMUNICATION PACKET
// ========================================================================

/// @brief A command packet received from the Master, parses variable arguments into a segment
struct MotionTask {
    union {
        uint32_t header; // Raw 32-bit header
        struct {
            uint32_t is_relative : 1; 
            uint32_t ctrl_p      : 1; 
            uint32_t ctrl_v      : 1; 
            uint32_t ctrl_a      : 1; 
            uint32_t ctrl_j      : 1; 
            uint32_t ctrl_q      : 1; 
            uint32_t duration_us : 26; // Segment time in microseconds (max ~67 sec)
        };
    };

    MotionSegment segment; // The parsed math segment ready for the Executor

    // Default constructor for empty tasks
    MotionTask() : header(0), segment() {}

    /// @brief Helper to pack flags and duration into a valid 32-bit CAN header
    static constexpr uint32_t build_header(MotionFlag flags, uint32_t duration_us) {
        return static_cast<uint32_t>(flags) | (duration_us << 6);
    }

    /// @brief Dynamic constructor parsing values sequentially based on the header bits
    template <typename... Args>
    explicit MotionTask(uint32_t raw_header, Args... args) 
        : header(raw_header), segment() 
    {
        uint8_t expected_args = ctrl_p + ctrl_v + ctrl_a + ctrl_j + ctrl_q;
        static_assert(sizeof...(Args) <= 5, "Maximum 5 arguments allowed (P, V, A, J, Q)");
        
        if (sizeof...(Args) != expected_args) return; // Safety guard

        // Initialize the segment's non-dynamic fields
        segment.t = duration_us;
        // Construct the 8-bit mode mask by dropping the relative bit and duration
        segment.mode = static_cast<uint8_t>(raw_header & 0x3E) | static_cast<uint8_t>(is_relative); 

        // Start unpacking variable arguments
        unpack_args(0, args...);
    }

private:
    void unpack_args(int) {} // Recursion base case

    template <typename T, typename... Rest>
    void unpack_args(int arg_index, T current_val, Rest... rest_args) {
        while (arg_index < 5) {
            bool is_active = false;
            switch (arg_index) {
                case 0: is_active = ctrl_p; break;
                case 1: is_active = ctrl_v; break;
                case 2: is_active = ctrl_a; break;
                case 3: is_active = ctrl_j; break;
                case 4: is_active = ctrl_q; break;
            }

            if (is_active) {
                // Assign to the flat MotionSegment structure
                switch (arg_index) {
                    case 0: segment.pos = static_cast<int32_t>(current_val); break;
                    case 1: segment.vel = static_cast<float>(current_val); break;
                    case 2: segment.acc = static_cast<float>(current_val); break;
                    case 3: segment.jrk = static_cast<float>(current_val); break;
                    case 4: segment.trq = static_cast<float>(current_val); break;
                }
                unpack_args(arg_index + 1, rest_args...);
                return;
            }
            arg_index++;
        }
    }
};

// ========================================================================
// 3. MOTION PLANNER CLASS (High-Level Manager)
// ========================================================================

/// @brief Manager that receives MotionTasks from CAN/Serial and feeds them to the Executor
template <size_t N>
class MotionPlanner {
private:
    MotionExecutor<N>* executor; // Pointer to the hardware-level executor

public:
    explicit MotionPlanner(MotionExecutor<N>* exec_ptr) : executor(exec_ptr) {}

    /// @brief Parses a raw network packet and attempts to push it to the executor
    /// @return true if successfully added to the FIFO buffer
    [[nodiscard]] bool process_incoming_task(const MotionTask& parsed_task) {
        if (executor == nullptr) return false;
        
        // You can add high-level blending logic, sanity checks, or limit bounding here
        
        return executor->add_segment(parsed_task.segment);
    }
};
