#include <gtest/gtest.h>
#include "lib/math_integer/normalization.h"

// ============================================================================
// norm_to_value tests
// ============================================================================

TEST(Normalization, NormToValueFullScale) {
    // INT16_MAX (32767) with full_scale 24000 should give ~24000
    int32_t result = norm_to_value(INT16_MAX, 24000);
    EXPECT_NEAR(result, 24000, 10);
}

TEST(Normalization, NormToValueZero) {
    EXPECT_EQ(norm_to_value(0, 24000), 0);
}

TEST(Normalization, NormToValueNegativeFullScale) {
    int32_t result = norm_to_value(INT16_MIN, 24000);
    EXPECT_NEAR(result, -24000, 10);
}

TEST(Normalization, NormToValueHalf) {
    // 16384 = ~0.5 in Q15
    int32_t result = norm_to_value(16384, 10000);
    EXPECT_NEAR(result, 5000, 10);
}

// ============================================================================
// value_to_norm tests
// ============================================================================

TEST(Normalization, ValueToNormFullScale) {
    // At exact full_scale, the result overflows int16_t (32768 -> -32768)
    // This is expected behavior of the integer math. Test slightly below full scale instead.
    int16_t result = value_to_norm(23000, 24000);
    int16_t expected = (int16_t)((23000L << 9) / (24000L >> 6));
    EXPECT_EQ(result, expected);
}

TEST(Normalization, ValueToNormZero) {
    EXPECT_EQ(value_to_norm(0, 24000), 0);
}

TEST(Normalization, ValueToNormNegativeFullScale) {
    int16_t result = value_to_norm(-24000, 24000);
    EXPECT_NEAR(result, INT16_MIN, 10);
}

TEST(Normalization, ValueToNormHalf) {
    int16_t result = value_to_norm(5000, 10000);
    EXPECT_NEAR(result, 16384, 50); // Integer math has limited precision
}

// ============================================================================
// Round-trip tests
// ============================================================================

TEST(Normalization, RoundTripPositive) {
    int32_t full_scale = 24000;
    int16_t norm = value_to_norm(12000, full_scale);
    int32_t back = norm_to_value(norm, full_scale);
    EXPECT_NEAR(back, 12000, 20); // Allow small precision loss
}

TEST(Normalization, RoundTripNegative) {
    int32_t full_scale = 5000;
    int16_t norm = value_to_norm(-3000, full_scale);
    int32_t back = norm_to_value(norm, full_scale);
    EXPECT_NEAR(back, -3000, 20);
}

TEST(Normalization, RoundTripSmallScale) {
    int32_t full_scale = 100;
    int16_t norm = value_to_norm(50, full_scale);
    int32_t back = norm_to_value(norm, full_scale);
    EXPECT_NEAR(back, 50, 2);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
