#pragma once

/**
 * @file vectors.h
 * @brief Fixed-point and floating-point 2D/3D vector types used by the FOC
 *        pipeline.
 *
 * Provided types:
 * - @c Vec2D     : 2D Cartesian vector in i1.15 (Q15) fixed point.
 * - @c Vec2D_f32 : 2D Cartesian vector in single-precision float.
 * - @c Pol2D     : 2D polar vector (angle + amplitude) in Q15.
 * - @c Vec3D     : 3-phase quantity container in int16_t.
 * - @c Vec3D_i32 : 3-phase quantity container in int32_t.
 *
 * The 2D types use an anonymous union of overlapping named views so the same
 * storage can be interpreted as x/y, cos/sin, alpha/beta (Clarke frame),
 * d/q (Park frame) or flux/torque depending on context.
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#include "common.h"
#include "math/trigonometry.h"

// Forward declaration for conversion
struct Pol2D;

/**
 * @brief 2D vector in Cartesian coordinates using i1.15 fixed-point format.
 *
 * Component convention: the FIRST member holds cos/x/alpha/d and the SECOND
 * holds sin/y/beta/q. Union allows accessing components as sin/cos, a/b,
 * alpha/beta or d/q pairs.
 *
 * @note All arithmetic helpers widen to int32_t before multiplying to avoid
 *       overflow, then shift right by 15 to return to Q15.
 */
struct Vec2D {
    /**
     * Overlapping component views of the same 4 bytes of storage:
     * - x/y            : generic Cartesian coordinates.
     * - cos/sin        : trigonometric pair (cos first!).
     * - a/b, alpha/beta: stationary alpha-beta (Clarke) frame.
     * - d/q            : rotating direct-quadrature (Park) frame.
     * - flux/torque    : physical interpretation of d/q for PMSM control.
     */
    union {
        struct { int16_t x;   int16_t y;   };
        struct { int16_t cos; int16_t sin; };  // Sine/Cosine components (trigonometric)
        struct { int16_t a;   int16_t b;   };  // Alpha/Beta components (Clarke transform)
        struct { int16_t alpha; int16_t beta; };
        struct { int16_t d;   int16_t q;   };  // Direct/Quadrature components (Park transform)
        struct { int16_t flux;   int16_t torque;   };
    };

    /// Construct from (cos, sin) / (x, y) components in Q15.
    /// @param cos_val First component (cos / x).
    /// @param sin_val Second component (sin / y).
    Vec2D(int16_t cos_val, int16_t sin_val) : cos(cos_val), sin(sin_val) {}

    /// Scale the vector by a coefficient in i1.15 format
    /// @param scale_factor Scaling factor in i1.15 (32768 = 1.0)
    /// @return Scaled vector
    Vec2D scale(int16_t scale_factor) const {
        int32_t s = (int32_t)sin;
        int32_t c = (int32_t)cos;
        int32_t sc = (int32_t)scale_factor;
        
        return Vec2D(
            (int16_t)((c * sc) >> 15),
            (int16_t)((s * sc) >> 15)
        );
    }

    /// Rotate the vector by another vector (rotation offset)
    /// @param rotation Rotation vector (sin, cos) in i1.15 format
    /// @return Rotated vector
    Vec2D rotate(const Vec2D& rotation) const {
        // Current vector components
        int32_t x_val = (int32_t)x; // cos
        int32_t y_val = (int32_t)y; // sin
        
        // Rotation vector components
        int32_t rot_cos = (int32_t)rotation.cos;
        int32_t rot_sin = (int32_t)rotation.sin;

        // Standard Rotation Matrix:
        // X' = X*cos - Y*sin
        // Y' = X*sin + Y*cos
        
        int32_t new_x = x_val * rot_cos - y_val * rot_sin;
        int32_t new_y = x_val * rot_sin + y_val * rot_cos;

        // Return (X, Y)
        return Vec2D(
            (int16_t)(new_x >> 15), 
            (int16_t)(new_y >> 15)
        );
    }

    /**
     * @brief Check whether the vector magnitude stays within a threshold.
     *
     * Computes sin² + cos² in 32-bit arithmetic and compares against
     * threshold². With the default threshold (INT16_MAX = 32767) a perfectly
     * normalized unit vector (components ±32768) will FAIL this check, since
     * 32768² > 32767²; callers must pick the threshold accordingly.
     *
     * @param threshold Magnitude limit applied per axis (default INT16_MAX).
     * @return true if sin²+cos² ≤ threshold², false otherwise.
     */
    bool validate(int16_t threshold = INT16_MAX) {
        int32_t sq_sin = (int32_t)sin;           
        int32_t sq_cos = (int32_t)cos;           
        int32_t th = (int32_t)threshold;
        uint32_t sum = (uint32_t)(sq_sin * sq_sin) + (uint32_t)(sq_cos * sq_cos);  // Calculate sin² + cos²

        // Check if sum is within valid range
        return sum <= (th * th); 
    }
};

/**
 * @brief 2D vector in Cartesian coordinates using single-precision floats.
 *
 * Same overlapping-view layout as Vec2D but in float, used by the voltage /
 * current control paths where dynamic range matters more than determinism.
 */
struct Vec2D_f32 {
public:
    /** Overlapping component views (x/y, cos/sin, alpha/beta, d/q, flux/torque). */
    union {
        struct { float x;     float y;      };
        struct { float cos;   float sin;    };
        struct { float a;     float b;      };
        struct { float alpha; float beta;   };
        struct { float d;     float q;      };
        struct { float flux;  float torque; };
    };

    /// Default constructor: zero vector.
    constexpr Vec2D_f32() : x(0.0f), y(0.0f) {}

    /// Construct from (cos, sin) / (x, y) components.
    constexpr Vec2D_f32(float cos_val, float sin_val)
        : cos(cos_val), sin(sin_val) {}

    /// Explicit conversion from a Q15 fixed-point vector (scales by 1/32768).
    explicit constexpr Vec2D_f32(const Vec2D& v)
        : x(q15ToF32(v.x)),
          y(q15ToF32(v.y)) {}

    /// Uniform scaling of both components by @p scale_factor.
    constexpr Vec2D_f32 scale(float scale_factor) const {
        return Vec2D_f32(
            x * scale_factor,
            y * scale_factor
        );
    }

    /// Rotate this vector using a Q15 sin/cos rotation vector.
    inline Vec2D_f32 rotate(const Vec2D& rotation) const {
        return rotateQ15(rotation.sin, rotation.cos);
    }

    /// Rotate this vector by a normalized int16 angle (-pi..pi mapped to full int16 range).
    inline Vec2D_f32 rotate(int16_t angle) const {
        int16_t s, c;
        angle2sincos(angle, &s, &c);
        return rotateQ15(s, c);
    }

    /// Check whether the squared magnitude fits within threshold² (unit circle check by default).
    constexpr bool validate(float threshold = 1.0f) const {
        return magnitudeSq() <= threshold * threshold;
    }


private:
    /// Conversion constant from Q15 integer units to float (1/32768).
    static constexpr float kQ15ToF32 = 1.0f / 32768.0f;

    /// Convert a Q15 integer to its float representation.
    static constexpr float q15ToF32(int16_t value) {
        return static_cast<float>(value) * kQ15ToF32;
    }

    /**
     * Core rotation helper applying the standard rotation matrix
     *   X' = X*cos - Y*sin
     *   Y' = X*sin + Y*cos
     * with Q15 inputs (argument order: sin first, then cos); the extra
     * kQ15ToF32 factor rescales the Q15 rotation components back to float.
     */
    inline Vec2D_f32 rotateQ15(int16_t rot_sin, int16_t rot_cos) const {
        const float c = static_cast<float>(rot_cos);
        const float s = static_cast<float>(rot_sin);

        return Vec2D_f32(
            (x * c - y * s) * kQ15ToF32,
            (x * s + y * c) * kQ15ToF32
        );
    }

    /// Squared Euclidean magnitude (avoids sqrt for comparisons).
    constexpr float magnitudeSq() const {
        return x * x + y * y;
    }
};

/**
 * @brief 2D vector in polar coordinates (angle, amplitude) in i1.15 format.
 */
struct Pol2D {
    int16_t angle;      ///< Angle in normalized format (INT16_MIN..INT16_MAX maps to -pi..pi).
    int16_t amplitude;  ///< Amplitude (magnitude) in i1.15 format (32768 = 1.0).

    /// Constructor with optional initial angle/amplitude.
    Pol2D(int16_t ang = 0, int16_t amp = 0) : angle(ang), amplitude(amp) {}

    /// Scale the polar vector by a coefficient in i1.15 format
    /// @param scale_factor Scaling factor in i1.15 (32768 = 1.0)
    /// @return Scaled polar vector
    Pol2D scale(int16_t scale_factor) const {
        int32_t amp = (int32_t)amplitude;
        int32_t sc = (int32_t)scale_factor;
        
        return Pol2D(
            angle,  // Angle remains unchanged
            (int16_t)((amp * sc) >> 15)
        );
    }

    /// Rotate the polar vector by an angle offset
    /// @param angle_offset Angle offset to add (normalized format)
    /// @return Rotated polar vector
    Pol2D rotate(int16_t angle_offset) const {
        // Simple angle addition (wraps around automatically due to int16_t overflow)
        return Pol2D(
            (int16_t)(angle + angle_offset),
            amplitude  // Amplitude remains unchanged
        );
    }

    /**
     * @brief Convert polar coordinates to Cartesian (Vec2D).
     *
     * Uses angle2sincos() from math/trigonometry.h (which must be included
     * beforehand) to obtain the unit direction, then scales both components
     * by the amplitude in Q15 arithmetic.
     *
     * @return Cartesian vector with (cos*x, sin*y) scaled by amplitude.
     */
    Vec2D convert() const {
        // Declare output variables for sin and cos
        int16_t s, c;
        
        // Use angle2sincos to convert angle to sin/cos components
        // This function is defined in math/trigonometry.h
        angle2sincos(angle, &s, &c);
        
        // Scale the sin/cos by amplitude
        int32_t cos_scaled = ((int32_t)c * (int32_t)amplitude) >> 15;
        int32_t sin_scaled = ((int32_t)s * (int32_t)amplitude) >> 15;
        
        return Vec2D((int16_t)cos_scaled, (int16_t)sin_scaled);
    }
};

/**
 * @brief 3-phase value container using i1.15 fixed-point format.
 *
 * The same 6 bytes can be addressed as motor phases (a/b/c), generic channels
 * (ch1/ch2/ch3), Cartesian triple (x/y/z) or as a raw array via operator[].
 *
 * @note Reading one union member while another was written relies on the
 *       common-initial-sequence guarantee / compiler extension (type punning
 *       through unions is technically UB in ISO C++ but universally supported
 *       on embedded toolchains).
 */
struct Vec3D {
    /** Overlapping phase/channel/array views of the same storage. */
    union {
        struct {int16_t a;   int16_t b;   int16_t c;   };   // Phase naming (A, B, C)
        struct { int16_t ch1; int16_t ch2; int16_t ch3; };   // Channel naming (CH1, CH2, CH3)
        struct { int16_t x; int16_t y; int16_t z; };   // Cartesian naming (X, Y, Z)
        int16_t arr[3];                           // Array access
    };

    /// Default constructor - zero initialize
    Vec3D() : a(0), b(0), c(0) {}

    /// Constructor with individual values
    Vec3D(int16_t val_a, int16_t val_b, int16_t val_c) : a(val_a), b(val_b), c(val_c) {}

    /// Array subscript operator for indexed access (no bounds checking)
    int16_t& operator[](size_t idx) { return arr[idx]; }
    /// Const array subscript operator for indexed access (no bounds checking)
    const int16_t& operator[](size_t idx) const { return arr[idx]; }
};

/**
 * @brief 3-phase value container using int32_t for extended range.
 *
 * Identical layout philosophy to Vec3D but with 32-bit lanes, useful for
 * intermediate accumulation results that must not overflow Q15 arithmetic.
 */
struct Vec3D_i32 {
    /** Overlapping phase/channel/array views of the same storage. */
    union {
        struct {int32_t a;   int32_t b;   int32_t c;   };   // Phase naming (A, B, C)
        struct { int32_t ch1; int32_t ch2; int32_t ch3; };   // Channel naming (CH1, CH2, CH3)
        struct { int32_t x; int32_t y; int32_t z; };   // Cartesian naming (X, Y, Z)
        int32_t arr[3];                           // Array access
    };

    /// Default constructor - zero initialize
    Vec3D_i32() : a(0), b(0), c(0) {}

    /// Constructor with individual values
    Vec3D_i32(int32_t val_a, int32_t val_b, int32_t val_c) : a(val_a), b(val_b), c(val_c) {}

    /// Array subscript operator for indexed access (no bounds checking)
    int32_t& operator[](size_t idx) { return arr[idx]; }
    /// Const array subscript operator for indexed access (no bounds checking)
    const int32_t& operator[](size_t idx) const { return arr[idx]; }
};
