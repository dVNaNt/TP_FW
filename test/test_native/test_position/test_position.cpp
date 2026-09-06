#include <gtest/gtest.h>
#include "lib/feedback/feedback.h"

// ============================================================================
// Position tests via Feedback (tick() is private, friend of Feedback)
// ============================================================================

TEST(Position, InitialState) {
    Position pos;
    EXPECT_EQ(pos.get_angle(), 0);
    EXPECT_EQ(pos.get_rotations(), 0);
    EXPECT_EQ(pos.get_value(), 0);
}

TEST(Position, ForwardMovement) {
    Feedback fb(24000, 5000, 5000, 5000);
    fb.enable_supply();
    fb.enable_encoder();

    // Move from 0 to 1000
    fb.inputs.set_supply_adc(32768);
    fb.inputs.set_angle_raw(1000);
    fb.tick();

    EXPECT_EQ(fb.position.get_angle(), 1000);
    EXPECT_EQ(fb.position.get_rotations(), 0);
}

TEST(Position, BackwardMovement) {
    Feedback fb(24000, 5000, 5000, 5000);
    fb.enable_supply();
    fb.enable_encoder();

    // First tick to establish position
    fb.inputs.set_supply_adc(32768);
    fb.inputs.set_angle_raw(1000);
    fb.tick();

    // Move backward to 500
    fb.inputs.set_supply_adc(32768);
    fb.inputs.set_angle_raw(500);
    fb.tick();

    EXPECT_EQ(fb.position.get_angle(), 500);
}

TEST(Position, ForwardWrapAround) {
    Feedback fb(24000, 5000, 5000, 5000);
    fb.enable_supply();
    fb.enable_encoder();

    // Gradually step to near max (incremental encoder simulation)
    // Use 65 steps of 1000 to reach 65000
    for (int i = 1; i <= 65; i++) {
        fb.inputs.set_supply_adc(32768);
        fb.inputs.set_angle_raw((uint16_t)(i * 1000));
        fb.tick();
    }
    EXPECT_EQ(fb.position.get_rotations(), 0);
    EXPECT_EQ(fb.position.get_angle(), 65000);

    // Wrap forward past 65535 to a small value (65000 -> 100 = +636 step)
    fb.inputs.set_supply_adc(32768);
    fb.inputs.set_angle_raw(100);
    fb.tick();

    // Should register one forward rotation
    EXPECT_EQ(fb.position.get_rotations(), 1);
    EXPECT_EQ(fb.position.get_angle(), 100);
}

TEST(Position, BackwardWrapAround) {
    Feedback fb(24000, 5000, 5000, 5000);
    fb.enable_supply();
    fb.enable_encoder();

    // Start at a small angle (small jump from 0 is fine)
    fb.inputs.set_supply_adc(32768);
    fb.inputs.set_angle_raw(100);
    fb.tick();
    EXPECT_EQ(fb.position.get_rotations(), 0);

    // Wrap backward past 0 (100 -> 65500 is a step of -136, which is a small backward step)
    fb.inputs.set_supply_adc(32768);
    fb.inputs.set_angle_raw(65500);
    fb.tick();

    // Should register one backward rotation (-1)
    EXPECT_EQ(fb.position.get_rotations(), -1);
    EXPECT_EQ(fb.position.get_angle(), 65500);
}

TEST(Position, MultipleRotations) {
    Feedback fb(24000, 5000, 5000, 5000);
    fb.enable_supply();
    fb.enable_encoder();

    // Simulate 3 full forward rotations
    uint16_t angle = 0;
    for (int rot = 0; rot < 3; rot++) {
        for (int step = 0; step < 10; step++) {
            angle = (uint16_t)(angle + 6554); // ~10 steps per rotation
            fb.inputs.set_supply_adc(32768);
            fb.inputs.set_angle_raw(angle);
            fb.tick();
        }
    }
    EXPECT_EQ(fb.position.get_rotations(), 3);
}

TEST(Position, Reset) {
    Feedback fb(24000, 5000, 5000, 5000);
    fb.enable_supply();
    fb.enable_encoder();

    fb.inputs.set_supply_adc(32768);
    fb.inputs.set_angle_raw(30000);
    fb.tick();

    fb.position.reset();
    EXPECT_EQ(fb.position.get_angle(), 0);
    EXPECT_EQ(fb.position.get_rotations(), 0);
    EXPECT_EQ(fb.position.get_value(), 0);
}

TEST(Position, GetValueCombinesRotationsAndAngle) {
    Feedback fb(24000, 5000, 5000, 5000);
    fb.enable_supply();
    fb.enable_encoder();

    // Move to angle 1000 with no rotations
    fb.inputs.set_supply_adc(32768);
    fb.inputs.set_angle_raw(1000);
    fb.tick();

    int32_t value = fb.position.get_value();
    // value = (0 << 16) | 1000 = 1000
    EXPECT_EQ(value, 1000);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
