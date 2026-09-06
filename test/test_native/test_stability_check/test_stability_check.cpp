#include <gtest/gtest.h>
#include "lib/common.h"

// ============================================================================
// StabilityCheck tests
// ============================================================================

TEST(StabilityCheck, NotStableInitially) {
    StabilityCheck sc(50, 10);
    EXPECT_FALSE(sc.is_stable());
}

TEST(StabilityCheck, StableConstantSignal) {
    StabilityCheck sc(50, 10); // epsilon=50, window=10
    // Feed exact same value for 10+ ticks
    for (int i = 0; i < 15; i++) {
        sc.tick(1000);
    }
    EXPECT_TRUE(sc.is_stable());
}

TEST(StabilityCheck, StableWithSmallNoise) {
    StabilityCheck sc(50, 10); // epsilon=50
    // Feed values within epsilon range
    for (int i = 0; i < 15; i++) {
        int16_t noise = (i % 3) * 10 - 10; // -10, 0, 10 pattern
        sc.tick(1000 + noise);
    }
    EXPECT_TRUE(sc.is_stable());
}

TEST(StabilityCheck, UnstableWithLargeChange) {
    StabilityCheck sc(50, 10);
    // Feed stable signal for a while
    for (int i = 0; i < 15; i++) {
        sc.tick(1000);
    }
    EXPECT_TRUE(sc.is_stable());
    // Now introduce a large change
    sc.tick(2000);
    EXPECT_FALSE(sc.is_stable()); // Should reset
}

TEST(StabilityCheck, EpsilonBoundary) {
    StabilityCheck sc(100, 5); // epsilon=100, window=5
    // Feed values at exactly epsilon spread (max - min = 100)
    sc.tick(0);
    sc.tick(100);
    sc.tick(50);
    sc.tick(25);
    sc.tick(75);
    EXPECT_TRUE(sc.is_stable());
}

TEST(StabilityCheck, EpsilonExceeded) {
    StabilityCheck sc(100, 5);
    sc.tick(0);
    sc.tick(101); // Spread = 101 > 100
    EXPECT_FALSE(sc.is_stable());
}

TEST(StabilityCheck, Reset) {
    StabilityCheck sc(50, 5);
    for (int i = 0; i < 10; i++) {
        sc.tick(1000);
    }
    EXPECT_TRUE(sc.is_stable());
    sc.reset();
    EXPECT_FALSE(sc.is_stable());
}

TEST(StabilityCheck, GetDetectedReference) {
    StabilityCheck sc(100, 5);
    // Feed values: min=950, max=1050
    sc.tick(1000);
    sc.tick(950);
    sc.tick(1050);
    sc.tick(1000);
    sc.tick(1000);
    EXPECT_TRUE(sc.is_stable());
    // Reference should be (950 + 1050) / 2 = 1000
    EXPECT_EQ(sc.get_detected_reference(), 1000);
}

TEST(StabilityCheck, WindowBehavior) {
    StabilityCheck sc(50, 5);
    // Should not be stable with only 4 samples
    sc.tick(100);
    sc.tick(100);
    sc.tick(100);
    sc.tick(100);
    EXPECT_FALSE(sc.is_stable());
    // 5th sample should make it stable
    sc.tick(100);
    EXPECT_TRUE(sc.is_stable());
}

TEST(StabilityCheck, Setup) {
    StabilityCheck sc(10, 5);
    for (int i = 0; i < 10; i++) {
        sc.tick(100);
    }
    EXPECT_TRUE(sc.is_stable());
    // Reconfigure with wider window
    sc.setup(10, 20);
    EXPECT_FALSE(sc.is_stable()); // Reset on setup
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
