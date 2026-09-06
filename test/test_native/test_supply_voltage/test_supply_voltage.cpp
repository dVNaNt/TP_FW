#include <gtest/gtest.h>
#include "lib/feedback/feedback.h"

// ============================================================================
// SupplyVoltage - public conversion method tests (direct)
// ============================================================================

TEST(SupplyVoltage, Norm2MvAtFullScale) {
    SupplyVoltage sv(255, 24000);
    // norm = INT16_MAX (32767) should give ~24000 mV
    int32_t mv = sv.norm2mv(INT16_MAX);
    EXPECT_NEAR(mv, 24000, 50);
}

TEST(SupplyVoltage, Norm2MvAtZero) {
    SupplyVoltage sv(255, 24000);
    EXPECT_EQ(sv.norm2mv(0), 0);
}

TEST(SupplyVoltage, Mv2NormAtFullScale) {
    SupplyVoltage sv(255, 24000);
    int16_t norm = sv.mv2norm(24000);
    EXPECT_NEAR(norm, INT16_MAX, 50);
}

TEST(SupplyVoltage, Mv2NormAtZero) {
    SupplyVoltage sv(255, 24000);
    EXPECT_EQ(sv.mv2norm(0), 0);
}

TEST(SupplyVoltage, Mv2NormRoundTrip) {
    SupplyVoltage sv(255, 24000);
    int16_t norm = sv.mv2norm(12000);
    int32_t mv = sv.norm2mv(norm);
    EXPECT_NEAR(mv, 12000, 50);
}

TEST(SupplyVoltage, Mv2DutyFullSupply) {
    // When target equals supply, duty should be 100% (INT16_MAX)
    SupplyVoltage sv(255, 24000);
    // Need to set voltage_mv_ through tick via Feedback
    Feedback fb(24000, 5000, 5000, 5000);
    fb.enable_supply();
    // Feed full-scale ADC value
    fb.inputs.set_supply_adc(65535);
    fb.tick();
    // Now supply should have a measured voltage
    int32_t measured_mv = fb.supply.get_voltage_mv();
    EXPECT_GT(measured_mv, 0);
    // Duty for the measured voltage should be ~100%
    int16_t duty = fb.supply.mv2duty(measured_mv);
    EXPECT_NEAR(duty, INT16_MAX, 100);
}

TEST(SupplyVoltage, OvervoltageLimit) {
    SupplyVoltage sv(255, 24000);
    sv.set_overvoltage_soft_mv(22000);
    int16_t norm_22v = sv.mv2norm(22000);
    EXPECT_TRUE(sv.is_overvoltage_soft_norm(norm_22v));
    int16_t norm_20v = sv.mv2norm(20000);
    EXPECT_FALSE(sv.is_overvoltage_soft_norm(norm_20v));
}

TEST(SupplyVoltage, UndervoltageLimit) {
    SupplyVoltage sv(255, 24000);
    sv.set_undervoltage_soft_mv(6000);
    int16_t norm_5v = sv.mv2norm(5000);
    EXPECT_TRUE(sv.is_undervoltage_soft_norm(norm_5v));
    int16_t norm_8v = sv.mv2norm(8000);
    EXPECT_FALSE(sv.is_undervoltage_soft_norm(norm_8v));
}

// ============================================================================
// SupplyVoltage tick() via Feedback
// ============================================================================

TEST(SupplyVoltage, TickViaFeedback) {
    Feedback fb(24000, 5000, 5000, 5000);
    fb.enable_supply();

    // Set midrange ADC (32768 -> ~half of full scale)
    fb.inputs.set_supply_adc(32768);
    fb.tick();
    EXPECT_TRUE(fb.is_updated());

    int16_t norm = fb.supply.get_voltage_norm();
    int32_t mv = fb.supply.get_voltage_mv();
    // At half-scale ADC (32768/65535 = ~50%), voltage should be around half of 24V
    // Filter alpha=200 means first tick won't fully converge; supply ADC is unsigned
    // norm = (32768 >> 1) = 16384, then mv = norm2mv(16384) depends on scale_
    EXPECT_GT(norm, 0);
    EXPECT_GT(mv, 0);
    EXPECT_NEAR(mv, 12000, 3000);
}

TEST(SupplyVoltage, Norm2DutyStatic) {
    // Static method test
    int16_t duty = SupplyVoltage::norm2duty(16384, INT16_MAX);
    // 16384/32767 * 32768 = ~16384
    EXPECT_NEAR(duty, 16384, 10);
}

TEST(SupplyVoltage, Norm2DutyDivByZero) {
    EXPECT_EQ(SupplyVoltage::norm2duty(1000, 0), 0);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
