#include <gtest/gtest.h>
#include "lib/vectors.h"

// ============================================================================
// Vec2D tests
// ============================================================================

TEST(Vec2D, ConstructionAndUnionAliases) {
    Vec2D v(1000, 2000);
    // All union aliases should share the same memory
    EXPECT_EQ(v.cos, 1000);
    EXPECT_EQ(v.sin, 2000);
    EXPECT_EQ(v.x, 1000);
    EXPECT_EQ(v.y, 2000);
    EXPECT_EQ(v.a, 1000);
    EXPECT_EQ(v.b, 2000);
    EXPECT_EQ(v.alpha, 1000);
    EXPECT_EQ(v.beta, 2000);
    EXPECT_EQ(v.d, 1000);
    EXPECT_EQ(v.q, 2000);
    EXPECT_EQ(v.flux, 1000);
    EXPECT_EQ(v.torque, 2000);
}

TEST(Vec2D, ScaleByOne) {
    // Scale factor INT16_MAX (32767) ~ 1.0 in Q15
    Vec2D v(10000, 20000);
    Vec2D result = v.scale(INT16_MAX);
    // (10000 * 32767) >> 15 = 9999 (rounding down)
    EXPECT_NEAR(result.sin, 10000, 1);
    EXPECT_NEAR(result.cos, 20000, 1);
}

TEST(Vec2D, ScaleByZero) {
    Vec2D v(10000, 20000);
    Vec2D result = v.scale(0);
    EXPECT_EQ(result.sin, 0);
    EXPECT_EQ(result.cos, 0);
}

TEST(Vec2D, ScaleByHalf) {
    // 16384 = 0.5 in Q15
    Vec2D v(10000, 20000);
    Vec2D result = v.scale(16384);
    EXPECT_NEAR(result.sin, 5000, 1);
    EXPECT_NEAR(result.cos, 10000, 1);
}

TEST(Vec2D, RotateByZero) {
    // Rotation by (0, INT16_MAX) is identity rotation (cos=1, sin=0)
    Vec2D v(10000, 5000);
    Vec2D identity(INT16_MAX, 0); // cos=32767, sin=0
    Vec2D result = v.rotate(identity);
    EXPECT_NEAR(result.x, 10000, 1);
    EXPECT_NEAR(result.y, 5000, 1);
}

TEST(Vec2D, ValidateNormalized) {
    // sin=0, cos=32767 should pass validation
    Vec2D v(INT16_MAX, 0);
    EXPECT_TRUE(v.validate(INT16_MAX));
}

TEST(Vec2D, ValidateExceedsThreshold) {
    // Both at maximum -> sin^2 + cos^2 = 2*32767^2 >> 32767^2
    Vec2D v(INT16_MAX, INT16_MAX);
    EXPECT_FALSE(v.validate(INT16_MAX));
}

// ============================================================================
// Pol2D tests
// ============================================================================

TEST(Pol2D, DefaultConstructor) {
    Pol2D p;
    EXPECT_EQ(p.angle, 0);
    EXPECT_EQ(p.amplitude, 0);
}

TEST(Pol2D, Construction) {
    Pol2D p(1000, 2000);
    EXPECT_EQ(p.angle, 1000);
    EXPECT_EQ(p.amplitude, 2000);
}

TEST(Pol2D, ScalePreservesAngle) {
    Pol2D p(5000, 20000);
    Pol2D result = p.scale(16384); // Scale by 0.5
    EXPECT_EQ(result.angle, 5000);             // Angle unchanged
    EXPECT_NEAR(result.amplitude, 10000, 1);   // Amplitude halved
}

TEST(Pol2D, RotateAddsAngle) {
    Pol2D p(1000, 20000);
    Pol2D result = p.rotate(500);
    EXPECT_EQ(result.angle, 1500);             // Angle = 1000 + 500
    EXPECT_EQ(result.amplitude, 20000);        // Amplitude unchanged
}

TEST(Pol2D, RotateWrapsAround) {
    // Test wrapping via int16_t overflow
    Pol2D p(INT16_MAX, 10000);
    Pol2D result = p.rotate(1);
    EXPECT_EQ(result.angle, INT16_MIN);        // 32767 + 1 = -32768 (wrap)
}

TEST(Pol2D, ConvertZeroAngle) {
    // angle=0: angle2sincos gives sin=0, cos=~32767
    // convert() calls Vec2D(sin_scaled, cos_scaled) which maps to .cos=sin_scaled, .sin=cos_scaled
    Pol2D p(0, INT16_MAX);
    Vec2D result = p.convert();
    EXPECT_NEAR(result.cos, 0, 2);        // First arg to Vec2D = sin from angle2sincos
    EXPECT_NEAR(result.sin, INT16_MAX, 2); // Second arg to Vec2D = cos from angle2sincos
}

// ============================================================================
// Vec3D tests
// ============================================================================

TEST(Vec3D, DefaultConstructor) {
    Vec3D v;
    EXPECT_EQ(v.a, 0);
    EXPECT_EQ(v.b, 0);
    EXPECT_EQ(v.c, 0);
}

TEST(Vec3D, ParameterizedConstructor) {
    Vec3D v(100, 200, 300);
    EXPECT_EQ(v.a, 100);
    EXPECT_EQ(v.b, 200);
    EXPECT_EQ(v.c, 300);
}

TEST(Vec3D, UnionAliases) {
    Vec3D v(10, 20, 30);
    EXPECT_EQ(v.ch1, 10);
    EXPECT_EQ(v.ch2, 20);
    EXPECT_EQ(v.ch3, 30);
    EXPECT_EQ(v.x, 10);
    EXPECT_EQ(v.y, 20);
    EXPECT_EQ(v.z, 30);
}

TEST(Vec3D, ArrayAccess) {
    Vec3D v(10, 20, 30);
    EXPECT_EQ(v[0], 10);
    EXPECT_EQ(v[1], 20);
    EXPECT_EQ(v[2], 30);
    v[1] = 99;
    EXPECT_EQ(v.b, 99);
}

// ============================================================================
// Vec3D_i32 tests
// ============================================================================

TEST(Vec3D_i32, DefaultConstructor) {
    Vec3D_i32 v;
    EXPECT_EQ(v.a, 0);
    EXPECT_EQ(v.b, 0);
    EXPECT_EQ(v.c, 0);
}

TEST(Vec3D_i32, ParameterizedConstructor) {
    Vec3D_i32 v(100000, 200000, 300000);
    EXPECT_EQ(v.a, 100000);
    EXPECT_EQ(v.b, 200000);
    EXPECT_EQ(v.c, 300000);
}

TEST(Vec3D_i32, ArrayAccess) {
    Vec3D_i32 v(10, 20, 30);
    EXPECT_EQ(v[0], 10);
    EXPECT_EQ(v[1], 20);
    EXPECT_EQ(v[2], 30);
    v[2] = 999;
    EXPECT_EQ(v.c, 999);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
