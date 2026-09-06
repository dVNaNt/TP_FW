#include <gtest/gtest.h>
#include "lib/common.h"

// ============================================================================
// Filter1st tests
// ============================================================================

TEST(Filter1st, DefaultConstructor) {
    Filter1st f;
    EXPECT_EQ(f.get_output(), 0);
}

TEST(Filter1st, ConstructorWithInitialValue) {
    Filter1st f(1000, 128);
    EXPECT_EQ(f.get_output(), 1000);
}

TEST(Filter1st, LpfAlpha255NoFiltering) {
    // Alpha 255 = no filtering, output should track input immediately
    Filter1st f(0, 255);
    int16_t out = f.tick_lpf(10000);
    EXPECT_NEAR(out, 10000, 50); // Nearly immediate tracking
}

TEST(Filter1st, LpfAlpha0InfiniteFiltering) {
    // Alpha 0 = infinite filtering, output should never change
    Filter1st f(0, 0);
    f.tick_lpf(10000);
    EXPECT_EQ(f.get_output(), 0); // Should not change
    f.tick_lpf(10000);
    EXPECT_EQ(f.get_output(), 0);
}

TEST(Filter1st, LpfStepResponse) {
    // With moderate alpha, filter should approach step input over time
    Filter1st f(0, 128);
    int16_t prev = 0;
    for (int i = 0; i < 100; i++) {
        int16_t out = f.tick_lpf(10000);
        EXPECT_GE(out, prev); // Output should monotonically increase
        prev = out;
    }
    // After many iterations, should be close to input
    EXPECT_NEAR(f.get_output(), 10000, 100);
}

TEST(Filter1st, HpfRemovesDC) {
    // HPF should remove DC component after settling
    Filter1st f(0, 128);
    // Feed constant DC for many cycles
    for (int i = 0; i < 500; i++) {
        f.tick_hpf(10000);
    }
    // HPF output for DC input should approach zero
    EXPECT_NEAR(f.get_output(), 0, 200);
}

TEST(Filter1st, HpfPassesAC) {
    // HPF should pass changes (AC component)
    Filter1st f(0, 10); // Slow filter
    // Feed DC for a while to settle
    for (int i = 0; i < 1000; i++) {
        f.tick_hpf(5000);
    }
    // Now apply a step change
    int16_t out = f.tick_hpf(15000);
    // HPF should show the change
    EXPECT_GT(out, 1000);
}

TEST(Filter1st, SetAlpha) {
    Filter1st f(0, 0);
    f.set_alpha(255);
    int16_t out = f.tick_lpf(20000);
    EXPECT_NEAR(out, 20000, 100);
}

TEST(Filter1st, Reset) {
    Filter1st f(0, 128);
    f.tick_lpf(10000);
    f.tick_lpf(10000);
    f.reset(5000);
    EXPECT_EQ(f.get_output(), 5000);
}

TEST(Filter1st, SetCutoffBasic) {
    Filter1st f;
    f.set_cutoff(100, 20000); // 100 Hz cutoff at 20 kHz sampling
    // alpha = 1602 * 100 / 20000 = 8
    // Just verify it doesn't crash and produces reasonable value
    f.tick_lpf(10000);
    // With very low alpha, output should barely change from 0
    EXPECT_LT(f.get_output(), 5000);
}

TEST(Filter1st, SetCutoffHighFrequency) {
    Filter1st f;
    f.set_cutoff(10000, 20000); // High cutoff relative to sampling
    // alpha = 1602 * 10000 / 20000 = 801 -> clamped to 255
    f.tick_lpf(10000);
    // With max alpha, output should be near input
    EXPECT_NEAR(f.get_output(), 10000, 200);
}

TEST(Filter1st, SetCutoffZeroSampling) {
    Filter1st f;
    f.set_cutoff(100, 0); // Division by zero guard
    // Should not crash, alpha stays unchanged
}

TEST(Filter1st, LpfWithINT16MAX) {
    Filter1st f(0, 255);
    int16_t out = f.tick_lpf(INT16_MAX);
    EXPECT_NEAR(out, INT16_MAX, 200);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
