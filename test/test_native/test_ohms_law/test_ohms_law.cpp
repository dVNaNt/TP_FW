#include <gtest/gtest.h>
#include "lib/math_integer/ohms_law.h"

// ============================================================================
// ohms_current: I = V / R
// ============================================================================

TEST(OhmsLaw, CurrentBasic) {
    // V=12000 mV, R=1000 mΩ => I = 12000 * 1000 / 1000 = 12000 mA
    EXPECT_EQ(ohms_current(12000, 1000), 12000);
}

TEST(OhmsLaw, CurrentSmallResistance) {
    // V=5000 mV, R=100 mΩ => I = 5000 * 1000 / 100 = 50000 mA
    EXPECT_EQ(ohms_current(5000, 100), 50000);
}

TEST(OhmsLaw, CurrentDivisionByZero) {
    EXPECT_EQ(ohms_current(12000, 0), 0); // Guarded against division by zero
}

// ============================================================================
// ohms_voltage: V = I * R
// ============================================================================

TEST(OhmsLaw, VoltageBasic) {
    // I=5000 mA, R=2000 mΩ => V = 5000 * 2000 / 1000 = 10000 mV
    EXPECT_EQ(ohms_voltage(5000, 2000), 10000);
}

TEST(OhmsLaw, VoltageZeroCurrent) {
    EXPECT_EQ(ohms_voltage(0, 1000), 0);
}

// ============================================================================
// ohms_resistance: R = V / I
// ============================================================================

TEST(OhmsLaw, ResistanceBasic) {
    // V=12000 mV, I=6000 mA => R = 12000 * 1000 / 6000 = 2000 mΩ
    EXPECT_EQ(ohms_resistance(12000, 6000), 2000);
}

TEST(OhmsLaw, ResistanceDivisionByZero) {
    EXPECT_EQ(ohms_resistance(12000, 0), 0); // Guarded
}

// ============================================================================
// ohms_power: P = V * I / 1000
// ============================================================================

TEST(OhmsLaw, PowerBasic) {
    // V=12000 mV, I=2000 mA => P = 12000 * 2000 / 1000 = 24000 mW
    EXPECT_EQ(ohms_power(12000, 2000), 24000);
}

TEST(OhmsLaw, PowerZero) {
    EXPECT_EQ(ohms_power(0, 5000), 0);
    EXPECT_EQ(ohms_power(5000, 0), 0);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
