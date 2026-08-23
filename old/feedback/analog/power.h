// #pragma once  // Ensure this header is included only once during compilation

// #include "../../../common.h"  // Include common migration definitions and types

// struct PowerMonitor {  // Helper structure for power scaling and conversion
// private:
//     // ---------------------------------------------------------------------------
//     // Private data members
//     // ---------------------------------------------------------------------------

//     // Scale factor for mW -> Q30 conversion.
//     // Calculated as: k = 2^46 / P_max_mW
//     // Used with a right shift of 16 bits to fit the result into Q30.
//     uint32_t k_mw2norm;

//     // Scale factor for Q30 -> mW conversion.
//     // Calculated as: k = 4 * P_max_mW
//     // Used with a right shift of 32 bits.
//     uint32_t k_norm2mw;

//     int32_t overpower_soft_norm;  // Soft overpower limit in Q30 format
//     int32_t overpower_hard_norm;  // Hard overpower limit in Q30 format

//     // Constructor: Initialize scaling factors based on maximum system ratings.
//     // max_current_ma: Maximum current in milliamps (must be > 0).
//     // max_voltage_mv: Maximum voltage in millivolts (must be > 0).
//     PowerMonitor(int32_t max_current_ma, int32_t max_voltage_mv) 
//         : overpower_soft_norm(INT32_MAX), overpower_hard_norm(INT32_MAX) {
//         // Calculate the maximum possible power product (P_max)
//         uint64_t vi_product = (uint64_t)max_voltage_mv * (uint64_t)max_current_ma;

//         // 1. Calculate k_norm2mw (Q30 -> mW)
//         // We use a multiplier of 4 to compensate for the 32-bit shift used later (instead of 30).
//         // Formula: k = P_max_mW * (2^32 / 2^30) = P_max_mW * 4
//         // Division by 250 is equivalent to (vi_product / 1000) * 4.
//         uint64_t p_fs_x4 = vi_product / 250ULL;
//         k_norm2mw = (uint32_t)p_fs_x4;

//         // 2. Calculate k_mw2norm (mW -> Q30)
//         // First, recover the actual P_max in mW.
//         uint64_t p_max_mw = vi_product / 1000ULL;

//         if (p_max_mw > 0) {
//             // Formula: k = 2^46 / P_max_mW.
//             // We use 1ULL << 46 to ensure 64-bit arithmetic and avoid overflow.
//             // This factor pairs with a >> 16 shift in runtime to produce a Q30 result.
//             uint64_t k_mw_calc = (1ULL << 46) / p_max_mw;
//             k_mw2norm = (uint32_t)k_mw_calc;
//         } else {
//             k_mw2norm = 0;  // Prevent division by zero
//         }
//     }
//     friend struct Feedback;

// public:
//     // Convert power in milliwatts to normalized Q30 format.
//     // mw: Power in milliwatts.
//     // Returns: Normalized power in Q30 format (1.0 = P_max).
//     int32_t mw2norm(int32_t mw) const {
//         // 1. Scale using 64-bit intermediate result to prevent overflow.
//         int64_t scaled = (int64_t)mw * (int64_t)k_mw2norm;

//         // 2. Shift right by 16.
//         // Combined with k_mw2norm (~2^46/P_max), this effectively performs:
//         // result = (mw * 2^46 / P_max) / 2^16 = mw * 2^30 / P_max.
//         return (int32_t)(scaled >> 16);
//     }

//     // Convert normalized Q30 power value to milliwatts.
//     // norm: Normalized power in Q30 format.
//     // Returns: Power in milliwatts.
//     inline int32_t norm2mw(int32_t norm) const {
//         // 1. Scale using 64-bit multiplication.
//         int64_t scaled = (int64_t)norm * (int64_t)k_norm2mw;

//         // 2. Shift right by 32 (effectively dividing by 2^32).
//         // On ARM architectures, this compiles to a very efficient operation (register move or ASR).
//         return (int32_t)(scaled >> 32);
//     }

//     // Calculate normalized power from normalized voltage and current.
//     // v_norm: Normalized voltage in Q15 format.
//     // i_norm: Normalized current in Q15 format.
//     // Returns: Normalized power in Q30 format.
//     inline int32_t power_norm(int16_t v_norm, int16_t i_norm) const {
//         // Q15 * Q15 results in Q30 format automatically.
//         return (int32_t)(v_norm) * (int32_t)(i_norm);
//     }

//     // Set soft overpower limit in normalized Q30 format
//     inline void set_overpower_soft_norm(int32_t limit) {
//         overpower_soft_norm = limit;
//     }

//     // Set soft overpower limit in milliwatts
//     void set_overpower_soft_mw(int32_t limit) {
//         overpower_soft_norm = mw2norm(limit);
//     }

//     // Set hard overpower limit in normalized Q30 format
//     inline void set_overpower_hard_norm(int32_t limit) {
//         overpower_hard_norm = limit;
//     }

//     // Set hard overpower limit in milliwatts
//     void set_overpower_hard_mw(int32_t limit) {
//         overpower_hard_norm = mw2norm(limit);
//     }

//     // Check if power exceeds soft limit (normalized Q30 format)
//     inline bool check_overpower_soft_norm(int32_t power_norm) const {
//         return power_norm >= overpower_soft_norm;
//     }

//     // Check if power exceeds hard limit (normalized Q30 format)
//     inline bool check_overpower_hard_norm(int32_t power_norm) const {
//         return power_norm >= overpower_hard_norm;
//     }

//     // Check if power from voltage and current exceeds soft limit
//     inline bool is_overpower_soft_norm(int16_t v_norm, int16_t i_norm) const {
//         return power_norm(v_norm, i_norm) >= overpower_soft_norm;
//     }

//     inline bool is_overpower_soft_norm(int32_t p_norm) const {
//         return p_norm >= overpower_soft_norm;
//     }

//     // Check if power from voltage and current exceeds hard limit
//     inline bool is_overpower_hard_norm(int16_t v_norm, int16_t i_norm) const {
//         return power_norm(v_norm, i_norm) >= overpower_hard_norm;
//     }

// };  // End of PowerMonitor structure definition
