#include <gtest/gtest.h>
#include "lib/common.h"

// ============================================================================
// clamp_i32 tests
// ============================================================================

TEST(ClampI32, ValueWithinLimit) {
    EXPECT_EQ(clamp_i32(50, 100), 50);    // Positive within range
    EXPECT_EQ(clamp_i32(-50, 100), -50);   // Negative within range
    EXPECT_EQ(clamp_i32(0, 100), 0);       // Zero is within any range
}

TEST(ClampI32, ValueExceedsPositiveLimit) {
    EXPECT_EQ(clamp_i32(200, 100), 100);   // Clamped to positive limit
    EXPECT_EQ(clamp_i32(INT32_MAX, 100), 100);
}

TEST(ClampI32, ValueExceedsNegativeLimit) {
    EXPECT_EQ(clamp_i32(-200, 100), -100); // Clamped to negative limit
    EXPECT_EQ(clamp_i32(INT32_MIN, 100), -100);
}

TEST(ClampI32, ValueAtExactLimit) {
    EXPECT_EQ(clamp_i32(100, 100), 100);   // Exactly at positive limit
    EXPECT_EQ(clamp_i32(-100, 100), -100); // Exactly at negative limit
}

TEST(ClampI32, ZeroLimit) {
    EXPECT_EQ(clamp_i32(0, 0), 0);        // Zero with zero limit
    EXPECT_EQ(clamp_i32(5, 0), 0);        // Positive clamped to zero
    EXPECT_EQ(clamp_i32(-5, 0), 0);       // Negative clamped to zero
}

// ============================================================================
// Frequency struct tests
// ============================================================================

TEST(Frequency, DefaultConstructor) {
    Frequency freq;
    EXPECT_EQ(freq.get_frequency_core(), 170000000); // Default 170 MHz
    EXPECT_EQ(freq.get_frequency_pwm(), 18000);      // Default 18 kHz
    EXPECT_EQ(freq.get_period_pwm(), 170000000 / 18000); // Auto-calculated
}

TEST(Frequency, CustomConstructor) {
    Frequency freq(100000000, 20000);
    EXPECT_EQ(freq.get_frequency_core(), 100000000);
    EXPECT_EQ(freq.get_frequency_pwm(), 20000);
    EXPECT_EQ(freq.get_period_pwm(), 100000000 / 20000); // 5000
}

TEST(Frequency, SetFrequencyCore) {
    Frequency freq;
    freq.set_frequency_core(200000000);
    EXPECT_EQ(freq.get_frequency_core(), 200000000);
    EXPECT_EQ(freq.get_period_pwm(), 200000000 / 18000); // Recalculated
}

TEST(Frequency, SetFrequencyPwm) {
    Frequency freq;
    freq.set_frequency_pwm(25000);
    EXPECT_EQ(freq.get_frequency_pwm(), 25000);
    EXPECT_EQ(freq.get_period_pwm(), 170000000 / 25000); // Recalculated
}

TEST(Frequency, ZeroPwmFrequency) {
    Frequency freq(170000000, 0);
    EXPECT_EQ(freq.get_period_pwm(), 0); // Division by zero guarded
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
