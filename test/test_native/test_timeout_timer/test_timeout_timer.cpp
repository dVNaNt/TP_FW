#include <gtest/gtest.h>
#include "lib/math_integer/helpers/timeout_timer.h"

// ============================================================================
// timeout_timer tests
// ============================================================================

TEST(TimeoutTimer, InitialValue) {
    timeout_timer t(100);
    EXPECT_EQ(t.get_timeout(), 100);
}

TEST(TimeoutTimer, Countdown) {
    timeout_timer t(5);
    EXPECT_EQ(t.tick(), 4);
    EXPECT_EQ(t.tick(), 3);
    EXPECT_EQ(t.tick(), 2);
    EXPECT_EQ(t.tick(), 1);
    EXPECT_EQ(t.tick(), 0);  // Reached zero
}

TEST(TimeoutTimer, ContinuesPastZero) {
    timeout_timer t(2);
    t.tick(); // 1
    t.tick(); // 0
    EXPECT_EQ(t.tick(), -1); // Goes negative
    EXPECT_EQ(t.tick(), -2);
}

TEST(TimeoutTimer, SetTimeout) {
    timeout_timer t(0);
    t.set_timeout(50);
    EXPECT_EQ(t.get_timeout(), 50);
}

TEST(TimeoutTimer, Reset) {
    timeout_timer t(100);
    t.tick();
    t.tick();
    t.reset();
    EXPECT_EQ(t.get_timeout(), 0);
}

TEST(TimeoutTimer, SetTimeoutAfterExpiry) {
    timeout_timer t(1);
    t.tick(); // 0
    t.tick(); // -1
    t.set_timeout(10);
    EXPECT_EQ(t.get_timeout(), 10);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
