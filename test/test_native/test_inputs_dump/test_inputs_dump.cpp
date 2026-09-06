#include <gtest/gtest.h>
#include "lib/feedback/feedback.h"

// ============================================================================
// InputsDump tests via Feedback (get_data/is_updated are private, friend)
// ============================================================================

TEST(InputsDump, NotUpdatedInitially) {
    Feedback fb(24000, 5000, 5000, 5000);
    fb.enable_supply();
    fb.tick();
    EXPECT_FALSE(fb.is_updated()); // No data fed yet
}

TEST(InputsDump, UpdatedAfterMandatoryFieldsFilled) {
    Feedback fb(24000, 5000, 5000, 5000);
    fb.enable_supply();
    // Feed supply ADC (the only mandatory field)
    fb.inputs.set_supply_adc(40000);
    fb.tick();
    EXPECT_TRUE(fb.is_updated());
}

TEST(InputsDump, DataContentCorrect) {
    Feedback fb(24000, 5000, 5000, 5000);
    fb.enable_supply();
    fb.inputs.set_supply_adc(12345);
    fb.tick();
    EXPECT_TRUE(fb.is_updated());
    const DataInputs& data = fb.get_data();
    EXPECT_EQ(data.supply_adc, 12345);
}

TEST(InputsDump, MultipleFieldsMandatory) {
    Feedback fb(24000, 5000, 5000, 5000);
    fb.enable_supply();
    fb.enable_current();
    // Only supply set - not all mandatory fields filled
    fb.inputs.set_supply_adc(30000);
    fb.tick();
    EXPECT_FALSE(fb.is_updated()); // Current not set yet

    // Now set current too
    uint16_t currents[3] = {32000, 32000, 32000};
    fb.inputs.set_current_adc(currents);
    fb.tick();
    EXPECT_TRUE(fb.is_updated());
}

TEST(InputsDump, DoubleBufferSwap) {
    Feedback fb(24000, 5000, 5000, 5000);
    fb.enable_supply();

    // First update
    fb.inputs.set_supply_adc(10000);
    fb.tick();
    EXPECT_TRUE(fb.is_updated());
    EXPECT_EQ(fb.get_data().supply_adc, 10000);

    // Second update with different value
    fb.inputs.set_supply_adc(20000);
    fb.tick();
    EXPECT_TRUE(fb.is_updated());
    EXPECT_EQ(fb.get_data().supply_adc, 20000);
}

TEST(InputsDump, AllFieldsFilled) {
    Feedback fb(24000, 5000, 5000, 5000);
    fb.enable_supply();
    fb.enable_current();
    fb.enable_encoder();

    fb.inputs.set_supply_adc(50000);
    uint16_t currents[3] = {30000, 31000, 32000};
    fb.inputs.set_current_adc(currents);
    fb.inputs.set_angle_raw(45000);
    fb.tick();

    EXPECT_TRUE(fb.is_updated());
    const DataInputs& data = fb.get_data();
    EXPECT_EQ(data.supply_adc, 50000);
    EXPECT_EQ(data.currnt_adc[0], 30000);
    EXPECT_EQ(data.currnt_adc[1], 31000);
    EXPECT_EQ(data.currnt_adc[2], 32000);
    EXPECT_EQ(data.angle_raw, 45000);
}

TEST(InputsDump, NoUpdateWithoutNewData) {
    Feedback fb(24000, 5000, 5000, 5000);
    fb.enable_supply();

    fb.inputs.set_supply_adc(10000);
    fb.tick();
    EXPECT_TRUE(fb.is_updated());

    // Tick again without new data
    fb.tick();
    EXPECT_FALSE(fb.is_updated());
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
