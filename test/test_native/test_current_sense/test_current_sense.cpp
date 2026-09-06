#include <gtest/gtest.h>
#include "lib/feedback/feedback.h"

// ============================================================================
// CurrentSense - public conversion method tests (direct)
// ============================================================================

TEST(CurrentSense, Ma2NormFullScale) {
    CurrentSense cs(5000, 5000, 5000);
    int16_t norm = cs.ma2norm(5000);
    EXPECT_NEAR(norm, INT16_MAX, 10);
}

TEST(CurrentSense, Ma2NormZero) {
    CurrentSense cs(5000, 5000, 5000);
    EXPECT_EQ(cs.ma2norm(0), 0);
}

TEST(CurrentSense, Ma2NormNegative) {
    CurrentSense cs(5000, 5000, 5000);
    int16_t norm = cs.ma2norm(-5000);
    // Should be -INT16_MAX (not INT16_MIN which is reserved)
    EXPECT_NEAR(norm, -INT16_MAX, 10);
}

TEST(CurrentSense, Norm2MaFullScale) {
    CurrentSense cs(5000, 5000, 5000);
    int32_t ma = cs.norm2ma(INT16_MAX);
    EXPECT_NEAR(ma, 5000, 5);
}

TEST(CurrentSense, Norm2MaZero) {
    CurrentSense cs(5000, 5000, 5000);
    EXPECT_EQ(cs.norm2ma(0), 0);
}

TEST(CurrentSense, RoundTrip) {
    CurrentSense cs(5000, 5000, 5000);
    int16_t norm = cs.ma2norm(3000);
    int32_t ma = cs.norm2ma(norm);
    EXPECT_NEAR(ma, 3000, 10);
}

TEST(CurrentSense, DifferentChannelScales) {
    // CH1=5000mA, CH2=3000mA, CH3=5000mA -> max is 5000mA
    CurrentSense cs(5000, 3000, 5000);
    EXPECT_EQ(cs.get_full_scale_max_ma(), 5000);
}

TEST(CurrentSense, ZeroFullScaleGuard) {
    CurrentSense cs(0, 0, 0);
    EXPECT_EQ(cs.ma2norm(1000), 0); // Should return 0, not crash
}

TEST(CurrentSense, InitialOutputSensorMissing) {
    CurrentSense cs(5000, 5000, 5000);
    // Before calibration, outputs should be INT16_MIN (sensor missing)
    EXPECT_EQ(cs.norm_ch(CurrentSense::CH1), INT16_MIN);
    EXPECT_EQ(cs.norm_ch(CurrentSense::CH2), INT16_MIN);
    EXPECT_EQ(cs.norm_ch(CurrentSense::CH3), INT16_MIN);
}

TEST(CurrentSense, NotCalibratedInitially) {
    CurrentSense cs(5000, 5000, 5000);
    EXPECT_FALSE(cs.is_calibrated());
    EXPECT_FALSE(cs.is_calibrated(CurrentSense::CH1));
}

// ============================================================================
// CurrentSense calibration via Feedback
// ============================================================================

TEST(CurrentSense, CalibrationViaFeedback) {
    Feedback fb(24000, 5000, 5000, 5000);
    fb.enable_supply();
    fb.enable_current();

    // Start calibration
    fb.current.calibrate();
    EXPECT_FALSE(fb.current.is_calibrated());

    // Feed zero-current ADC values (midpoint ~32768) for n_samples (default 100)
    uint16_t zero_current[3] = {32768, 32768, 32768};
    for (int i = 0; i < 101; i++) {
        fb.inputs.set_supply_adc(32768);
        fb.inputs.set_current_adc(zero_current);
        fb.tick();
    }

    // Should be calibrated after n_samples + 1 ticks
    EXPECT_TRUE(fb.current.is_calibrated());
}

TEST(CurrentSense, OvercurrentHardLimit) {
    CurrentSense cs(5000, 5000, 5000);
    // Set overcurrent at 4000 mA
    cs.set_overcurrent_hard_ma(4000);
    int16_t norm_limit = cs.get_overcurrent_hard_norm(CurrentSense::CH1);
    int16_t norm_4000 = cs.ma2norm(4000);
    EXPECT_NEAR(norm_limit, norm_4000, 5);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
