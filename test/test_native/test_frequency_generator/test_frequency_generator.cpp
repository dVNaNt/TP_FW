#include <gtest/gtest.h>
#include "lib/math_integer/helpers/frequency_generator.h"

// ============================================================================
// FrequencyGenerator tests
// ============================================================================

TEST(FrequencyGenerator, InitialState) {
    FrequencyGenerator gen(20000);
    EXPECT_EQ(gen.get_frequency_system(), 20000);
    EXPECT_EQ(gen.get_frequency_target(), 0);
    EXPECT_EQ(gen.get_angle(), 0);
    EXPECT_EQ(gen.get_amplitude(), INT16_MAX);
}

TEST(FrequencyGenerator, SetValidFrequency) {
    FrequencyGenerator gen(20000);
    EXPECT_TRUE(gen.set_frequency_target(1000));
    EXPECT_EQ(gen.get_frequency_target(), 1000);
}

TEST(FrequencyGenerator, RejectFrequencyOver2000Hz) {
    FrequencyGenerator gen(20000);
    EXPECT_FALSE(gen.set_frequency_target(2001));
    EXPECT_EQ(gen.get_frequency_target(), 0); // Unchanged
}

TEST(FrequencyGenerator, Accept2000Hz) {
    FrequencyGenerator gen(20000);
    EXPECT_TRUE(gen.set_frequency_target(2000));
    EXPECT_EQ(gen.get_frequency_target(), 2000);
}

TEST(FrequencyGenerator, AngleIncrementsOnTick) {
    FrequencyGenerator gen(20000);
    gen.set_frequency_target(1000);
    gen.tick();
    EXPECT_NE(gen.get_angle(), 0); // Angle should have changed
}

TEST(FrequencyGenerator, FullCycleAccuracy) {
    // At 1000 Hz target with 20000 Hz system, one full cycle = 20 ticks
    FrequencyGenerator gen(20000);
    gen.set_frequency_target(1000);
    // Run for 20 ticks (one full cycle)
    for (int i = 0; i < 20; i++) {
        gen.tick();
    }
    // Angle should be approximately back to 0 (within one increment)
    int16_t angle = gen.get_angle();
    // One full cycle: 20 * (65536/20) = 65536 -> wraps to ~0
    EXPECT_NEAR(angle, 0, 200);
}

TEST(FrequencyGenerator, AmplitudeControl) {
    FrequencyGenerator gen(20000);
    gen.set_amplitude(16384); // Half amplitude
    EXPECT_EQ(gen.get_amplitude(), 16384);
}

TEST(FrequencyGenerator, ResetAngle) {
    FrequencyGenerator gen(20000);
    gen.set_frequency_target(500);
    gen.tick();
    gen.tick();
    gen.reset();
    EXPECT_EQ(gen.get_angle(), 0);
}

TEST(FrequencyGenerator, SetSystemFrequency) {
    FrequencyGenerator gen(20000);
    gen.set_frequency_target(1000);
    gen.set_frequency_system(40000);
    EXPECT_EQ(gen.get_frequency_system(), 40000);
    // Angle increment should be halved
}

TEST(FrequencyGenerator, TickReturnsVec2D) {
    FrequencyGenerator gen(20000);
    gen.set_frequency_target(500);
    Vec2D result = gen.tick();
    // At first tick from angle 0, sin should be small and cos close to amplitude
    // The vector should have reasonable values
    int32_t sum_sq = (int32_t)result.sin * result.sin + (int32_t)result.cos * result.cos;
    EXPECT_GT(sum_sq, 0); // Non-zero output
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
