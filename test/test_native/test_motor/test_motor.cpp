#include <gtest/gtest.h>
#include "lib/motor_driver/motor.h"

// ============================================================================
// Motor tests - setter/getter round-trips and defaults
// ============================================================================

TEST(Motor, DefaultValues) {
    Motor m;
    EXPECT_EQ(m.get_motor_type(), MotorType_Invalid);
    EXPECT_EQ(m.get_phase_pattern(), PhasePattern_ABC);
    EXPECT_EQ(m.get_status(), DriverStatus_CalibrateCurrent);
    EXPECT_EQ(m.get_calib_status(), MotorStatus::CurrentCalibration);
    EXPECT_EQ(m.get_pole_count(), 7);
    EXPECT_EQ(m.get_direction(), 1);
    EXPECT_EQ(m.get_electric_angle(), 0);
    EXPECT_EQ(m.get_mechanic_angle(), 0);
    EXPECT_EQ(m.get_total_resistance(), 0);
    EXPECT_EQ(m.get_current_limit(), 600);
    EXPECT_EQ(m.get_max_voltage(), 24000);
}

TEST(Motor, SetMotorType) {
    Motor m;
    m.set_motor_type(MotorType_BLDC);
    EXPECT_EQ(m.get_motor_type(), MotorType_BLDC);

    m.set_motor_type(MotorType_STEP);
    EXPECT_EQ(m.get_motor_type(), MotorType_STEP);

    m.set_motor_type(MotorType_DC);
    EXPECT_EQ(m.get_motor_type(), MotorType_DC);
}

TEST(Motor, SetMotorTypeFromInt) {
    Motor m;
    m.set_motor_type((int8_t)6); // BLDC = 6
    EXPECT_EQ(m.get_motor_type(), MotorType_BLDC);
}

TEST(Motor, SetPhasePattern) {
    Motor m;
    m.set_phase_pattern(PhasePattern_CBA);
    EXPECT_EQ(m.get_phase_pattern(), PhasePattern_CBA);

    m.set_phase_pattern(PhasePattern_ACB);
    EXPECT_EQ(m.get_phase_pattern(), PhasePattern_ACB);
}

TEST(Motor, SetStatus) {
    Motor m;
    m.set_status(DriverStatus_Ready);
    EXPECT_EQ(m.get_status(), DriverStatus_Ready);

    m.set_status(DriverStatus_Error);
    EXPECT_EQ(m.get_status(), DriverStatus_Error);
}

TEST(Motor, SetCalibStatus) {
    Motor m;
    m.set_calib_status(MotorStatus::Ready);
    EXPECT_EQ(m.get_calib_status(), MotorStatus::Ready);
}

TEST(Motor, SetPoleCount) {
    Motor m;
    m.set_pole_count(14);
    EXPECT_EQ(m.get_pole_count(), 14);
}

TEST(Motor, SetDirection) {
    Motor m;
    m.set_direction(-1);
    EXPECT_EQ(m.get_direction(), -1);

    m.set_direction(1);
    EXPECT_EQ(m.get_direction(), 1);
}

TEST(Motor, SetAngles) {
    Motor m;
    m.set_electric_angle(12345);
    EXPECT_EQ(m.get_electric_angle(), 12345);

    m.set_mechanic_angle(-5678);
    EXPECT_EQ(m.get_mechanic_angle(), -5678);
}

TEST(Motor, SetPhaseInductance) {
    Motor m;
    m.set_phase_inductance(0, 100);
    m.set_phase_inductance(1, 200);
    m.set_phase_inductance(2, 300);
    EXPECT_EQ(m.get_phase_inductance(0), 100);
    EXPECT_EQ(m.get_phase_inductance(1), 200);
    EXPECT_EQ(m.get_phase_inductance(2), 300);
}

TEST(Motor, SetPhaseInductanceOutOfBounds) {
    Motor m;
    m.set_phase_inductance(3, 999);  // Invalid index, should be ignored
    EXPECT_EQ(m.get_phase_inductance(3), 0); // Out-of-bounds returns 0
}

TEST(Motor, SetCurrentLimit) {
    Motor m;
    m.set_current_limit(2000);
    EXPECT_EQ(m.get_current_limit(), 2000);
}

TEST(Motor, SetMaxVoltage) {
    Motor m;
    m.set_max_voltage(48000);
    EXPECT_EQ(m.get_max_voltage(), 48000);
}

TEST(Motor, SetTotalResistance) {
    Motor m;
    m.set_total_resistance(1500);
    EXPECT_EQ(m.get_total_resistance(), 1500);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
