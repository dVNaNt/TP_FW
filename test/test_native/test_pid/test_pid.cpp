#include <gtest/gtest.h>
#include "lib/common.h"

// ============================================================================
// PID_Int tests
// ============================================================================

TEST(PID, InitialOutputIsZero) {
    PID_Int pid(100, 0, 0, 0);
    EXPECT_EQ(pid.output(), 0);
}

TEST(PID, ProportionalResponse) {
    // Kp=100 (100%), Ki=0, Kd=0, Kff=0
    PID_Int pid(100, 0, 0, 0);
    pid.tick(1000, 0, INT16_MAX);
    // Output should be proportional to error
    EXPECT_GT(pid.output(), 0);
    // With Kp=100% and error=1000, output should be close to 1000
    EXPECT_NEAR(pid.output(), 1000, 50);
}

TEST(PID, ProportionalNegativeError) {
    PID_Int pid(100, 0, 0, 0);
    pid.tick(-1000, 0, INT16_MAX);
    EXPECT_LT(pid.output(), 0);
    EXPECT_NEAR(pid.output(), -1000, 50);
}

TEST(PID, IntegralAccumulates) {
    PID_Int pid(0, 100, 0, 0);
    // Feed constant error for several ticks
    pid.tick(1000, 0, INT16_MAX);
    int16_t out1 = pid.output();
    pid.tick(1000, 0, INT16_MAX);
    int16_t out2 = pid.output();
    pid.tick(1000, 0, INT16_MAX);
    int16_t out3 = pid.output();
    // Integral should accumulate, so output should grow
    EXPECT_GT(out2, out1);
    EXPECT_GT(out3, out2);
}

TEST(PID, IntegralAntiWindup) {
    PID_Int pid(0, 100, 0, 0);
    int16_t limit = 5000;
    // Feed large error many times
    for (int i = 0; i < 100; i++) {
        pid.tick(INT16_MAX, 0, limit);
    }
    // Output should be clamped to limit
    EXPECT_LE(pid.output(), limit);
    EXPECT_GE(pid.output(), -limit);
}

TEST(PID, DerivativeResponse) {
    PID_Int pid(0, 0, 100, 0);
    // First tick: derivative = error - 0 = 1000
    pid.tick(1000, 0, INT16_MAX);
    int16_t out1 = pid.output();
    EXPECT_GT(out1, 0); // Rising error -> positive derivative

    // Second tick with same error: derivative = 0
    pid.tick(1000, 0, INT16_MAX);
    int16_t out2 = pid.output();
    EXPECT_EQ(out2, 0); // No change in error -> zero derivative
}

TEST(PID, FeedForwardResponse) {
    PID_Int pid(0, 0, 0, 100);
    pid.tick(0, 5000, INT16_MAX);
    // Output should respond to feedforward, not error
    EXPECT_NEAR(pid.output(), 5000, 50);
}

TEST(PID, OutputClamping) {
    PID_Int pid(100, 0, 0, 0);
    int16_t limit = 100;
    pid.tick(INT16_MAX, 0, limit);
    EXPECT_LE(pid.output(), limit);
    EXPECT_GE(pid.output(), -limit);
}

TEST(PID, Reset) {
    PID_Int pid(0, 100, 0, 0);
    for (int i = 0; i < 10; i++) {
        pid.tick(1000, 0, INT16_MAX);
    }
    EXPECT_NE(pid.output(), 0);
    pid.reset();
    EXPECT_EQ(pid.output(), 0);
}

TEST(PID, DynamicGainChange) {
    PID_Int pid(100, 0, 0, 0);
    pid.tick(1000, 0, INT16_MAX);
    int16_t out_kp100 = pid.output();

    pid.reset();
    pid.setKp(200);
    pid.tick(1000, 0, INT16_MAX);
    int16_t out_kp200 = pid.output();

    // Doubling Kp should roughly double the proportional output
    EXPECT_NEAR(out_kp200, out_kp100 * 2, 50);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
