#include <gtest/gtest.h>
#include "lib/common.h"

// Helper: int16 angle for a given degree
// Full circle: INT16_MIN..INT16_MAX maps to -180..+180 degrees
// 0 deg = 0, 90 deg = 16384, 180 deg = -32768 (wraps), -90 deg = -16384
static int16_t deg2angle(int deg) {
    return (int16_t)((int32_t)deg * 65536 / 360);
}

// ============================================================================
// angle2sincos tests at cardinal angles
// ============================================================================

TEST(Trigonometry, Angle0Degrees) {
    int16_t s, c;
    angle2sincos(deg2angle(0), &s, &c);
    EXPECT_NEAR(s, 0, 2);
    EXPECT_NEAR(c, 32767, 2);    // cos(0) = 1.0 in Q15
}

TEST(Trigonometry, Angle90Degrees) {
    int16_t s, c;
    angle2sincos(deg2angle(90), &s, &c);
    EXPECT_NEAR(s, 32767, 2);    // sin(90) = 1.0
    EXPECT_NEAR(c, 0, 2);        // cos(90) = 0
}

TEST(Trigonometry, Angle180Degrees) {
    int16_t s, c;
    // 180 degrees = INT16_MIN in normalized angle
    angle2sincos((int16_t)INT16_MIN, &s, &c);
    EXPECT_NEAR(s, 0, 2);         // sin(180) = 0
    EXPECT_NEAR(c, -32767, 2);    // cos(180) = -1.0
}

TEST(Trigonometry, AngleNeg90Degrees) {
    int16_t s, c;
    angle2sincos(deg2angle(-90), &s, &c);
    EXPECT_NEAR(s, -32767, 2);    // sin(-90) = -1.0
    EXPECT_NEAR(c, 0, 2);         // cos(-90) = 0
}

TEST(Trigonometry, PythagoreanIdentity) {
    // sin^2 + cos^2 should be approximately 32767^2 for all angles
    const int32_t expected = (int32_t)32767 * 32767;
    for (int deg = -180; deg < 180; deg += 15) {
        int16_t s, c;
        angle2sincos(deg2angle(deg), &s, &c);
        int32_t sum = (int32_t)s * s + (int32_t)c * c;
        // Allow ~1% tolerance due to table quantization
        EXPECT_NEAR(sum, expected, expected / 50)
            << "Failed at " << deg << " degrees";
    }
}

TEST(Trigonometry, Angle45Degrees) {
    int16_t s, c;
    angle2sincos(deg2angle(45), &s, &c);
    // sin(45) = cos(45) ≈ 0.7071 * 32767 ≈ 23170
    EXPECT_NEAR(s, 23170, 50);
    EXPECT_NEAR(c, 23170, 50);
}

// ============================================================================
// scale_sincos tests
// ============================================================================

TEST(ScaleSinCos, ScaleByOne) {
    int16_t out_s, out_c;
    scale_sincos(10000, 20000, INT16_MAX, &out_s, &out_c);
    EXPECT_NEAR(out_s, 10000, 1);
    EXPECT_NEAR(out_c, 20000, 1);
}

TEST(ScaleSinCos, ScaleByZero) {
    int16_t out_s, out_c;
    scale_sincos(10000, 20000, 0, &out_s, &out_c);
    EXPECT_EQ(out_s, 0);
    EXPECT_EQ(out_c, 0);
}

TEST(ScaleSinCos, ScaleByHalf) {
    int16_t out_s, out_c;
    scale_sincos(10000, 20000, 16384, &out_s, &out_c); // 0.5 in Q15
    EXPECT_NEAR(out_s, 5000, 1);
    EXPECT_NEAR(out_c, 10000, 1);
}

// ============================================================================
// rotate_sincos tests
// ============================================================================

TEST(RotateSinCos, RotateByZero) {
    // Rotation by (sin=0, cos=1) = identity
    int16_t out_s, out_c;
    rotate_sincos(10000, 20000, 0, INT16_MAX, &out_s, &out_c);
    EXPECT_NEAR(out_s, 10000, 1);
    EXPECT_NEAR(out_c, 20000, 1);
}

TEST(RotateSinCos, RotateBy90) {
    // Rotate vector (sin=32767, cos=0) by 90 degrees (sin=32767, cos=0)
    // Result should be (sin=0, cos=-32767)
    int16_t out_s, out_c;
    rotate_sincos(INT16_MAX, 0, INT16_MAX, 0, &out_s, &out_c);
    EXPECT_NEAR(out_s, 0, 2);
    EXPECT_NEAR(out_c, -INT16_MAX, 2);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
