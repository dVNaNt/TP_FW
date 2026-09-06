#include <gtest/gtest.h>
#include "lib/feedback/feedback.h"

// PowerMonitor constructor is private (friend of Feedback), so we create via Feedback
static Feedback make_fb() {
    return Feedback(24000, 5000, 5000, 5000);
}

// ============================================================================
// PowerMonitor tests
// ============================================================================

TEST(PowerMonitor, PowerNormBasic) {
    Feedback fb = make_fb();
    // Q15 * Q15 = Q30
    int32_t p = fb.power.power_norm(INT16_MAX, INT16_MAX);
    // (32767 * 32767) = 1,073,676,289 (~2^30)
    EXPECT_EQ(p, (int32_t)INT16_MAX * (int32_t)INT16_MAX);
}

TEST(PowerMonitor, PowerNormZero) {
    Feedback fb = make_fb();
    EXPECT_EQ(fb.power.power_norm(0, INT16_MAX), 0);
    EXPECT_EQ(fb.power.power_norm(INT16_MAX, 0), 0);
}

TEST(PowerMonitor, Mw2NormAndBack) {
    Feedback fb = make_fb();
    // Full-scale power: 24000 mV * 5000 mA / 1000 = 120000 mW
    // Round-trip: mw -> norm -> mw
    int32_t norm = fb.power.mw2norm(60000);  // Half power
    int32_t back = fb.power.norm2mw(norm);
    EXPECT_NEAR(back, 60000, 500);
}

TEST(PowerMonitor, Mw2NormFullScale) {
    Feedback fb = make_fb();
    // Full-scale power = 24000 * 5000 / 1000 = 120000 mW
    int32_t norm = fb.power.mw2norm(120000);
    // Should be close to 2^30 = 1,073,741,824
    EXPECT_NEAR(norm, (int32_t)(1 << 30), (int32_t)(1 << 30) / 20);
}

TEST(PowerMonitor, Mw2NormZero) {
    Feedback fb = make_fb();
    EXPECT_EQ(fb.power.mw2norm(0), 0);
}

TEST(PowerMonitor, Norm2MwZero) {
    Feedback fb = make_fb();
    EXPECT_EQ(fb.power.norm2mw(0), 0);
}

TEST(PowerMonitor, OverpowerSoftLimit) {
    Feedback fb = make_fb();
    fb.power.set_overpower_soft_mw(100000);
    // Power below limit
    EXPECT_FALSE(fb.power.is_overpower_soft_norm(fb.power.mw2norm(50000)));
    // Power at/above limit
    EXPECT_TRUE(fb.power.is_overpower_soft_norm(fb.power.mw2norm(100000)));
}

TEST(PowerMonitor, OverpowerHardLimit) {
    Feedback fb = make_fb();
    fb.power.set_overpower_hard_mw(110000);
    EXPECT_FALSE(fb.power.is_overpower_hard_norm(INT16_MAX / 2, INT16_MAX / 2));
    // Maximum power should exceed
    EXPECT_TRUE(fb.power.is_overpower_hard_norm(INT16_MAX, INT16_MAX));
}

TEST(PowerMonitor, SetOverpowerNorm) {
    Feedback fb = make_fb();
    fb.power.set_overpower_soft_norm(500000000);
    EXPECT_TRUE(fb.power.check_overpower_soft_norm(500000001));
    EXPECT_FALSE(fb.power.check_overpower_soft_norm(499999999));
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
