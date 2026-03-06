#pragma once
#include "common.h"
#include "math_integer/trigonometry.h"

// Forward declaration for conversion
struct Pol2D;

/// 2D Vector in Cartesian coordinates using i1.15 fixed-point format
/// Union allows accessing components as sin/cos, a/b, or d/q pairs
struct Vec2D {
    union {
        struct { int16_t x;   int16_t y;   }; 
        struct { int16_t cos; int16_t sin; };  // Sine/Cosine components (trigonometric)
        struct { int16_t a;   int16_t b;   };  // Alpha/Beta components (Clarke transform)
        struct { int16_t alpha; int16_t beta; };
        struct { int16_t d;   int16_t q;   };  // Direct/Quadrature components (Park transform)
        struct { int16_t flux;   int16_t torque;   };
    };

    /// Constructor
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

        // Возвращаем (X, Y)
        return Vec2D(
            (int16_t)(new_x >> 15), 
            (int16_t)(new_y >> 15)
        );
    }

    /// Check if sin/cos values are properly normalized
    /// Validates that sin²+cos² ≤ 32767² * 2 (allows for some numerical tolerance)
    /// @param sin_q15 Sine value in Q15 format
    /// @param cos_q15 Cosine value in Q15 format
    /// @return true if normalized, false if out of range
    bool validate(int16_t threshold = INT16_MAX) {
        int32_t sq_sin = (int32_t)sin;           
        int32_t sq_cos = (int32_t)cos;           
        int32_t th = (int32_t)threshold;
        uint32_t sum = (uint32_t)(sq_sin * sq_sin) + (uint32_t)(sq_cos * sq_cos);  // Calculate sin² + cos²

        // Check if sum is within valid range
        return sum <= (th * th); 
    }
};

/// 2D Vector in Polar coordinates (angle, amplitude) using i1.15 fixed-point format
struct Pol2D {
    int16_t angle;      // Angle in normalized format (INT16_MIN to INT16_MAX = -π to π)
    int16_t amplitude;  // Amplitude (magnitude) in i1.15 format

    /// Constructor
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

    /// Convert polar coordinates to Cartesian (Vec2D)
    /// Note: This method requires angle2sincos from math_integer/trigonometry.h
    /// Include trigonometry.h before using this method
    /// @return Cartesian vector (sin, cos)
    Vec2D convert() const {
        // Declare output variables for sin and cos
        int16_t s, c;
        
        // Use angle2sincos to convert angle to sin/cos components
        // This function is defined in math_integer/trigonometry.h
        angle2sincos(angle, &s, &c);
        
        // Scale the sin/cos by amplitude
        int32_t cos_scaled = ((int32_t)c * (int32_t)amplitude) >> 15;
        int32_t sin_scaled = ((int32_t)s * (int32_t)amplitude) >> 15;
        
        return Vec2D((int16_t)cos_scaled, (int16_t)sin_scaled);
    }
};

/// 3-Phase value container using i1.15 fixed-point format
struct Vec3D {
    union {
        struct {int16_t a;   int16_t b;   int16_t c;   };   // Phase naming (A, B, C)
        struct { int16_t ch1; int16_t ch2; int16_t ch3; };   // Channel naming (CH1, CH2, CH3)
        struct { int16_t x; int16_t y; int16_t z; };   // Channel naming (CH1, CH2, CH3)
        int16_t arr[3];                           // Array access
    };

    /// Default constructor - zero initialize
    Vec3D() : a(0), b(0), c(0) {}

    /// Constructor with individual values
    Vec3D(int16_t val_a, int16_t val_b, int16_t val_c) : a(val_a), b(val_b), c(val_c) {}

    /// Array subscript operator for indexed access
    int16_t& operator[](size_t idx) { return arr[idx]; }
    const int16_t& operator[](size_t idx) const { return arr[idx]; }
};

/// 3-Phase value container using int32_t for extended range
struct Vec3D_i32 {
    union {
        struct {int32_t a;   int32_t b;   int32_t c;   };   // Phase naming (A, B, C)
        struct { int32_t ch1; int32_t ch2; int32_t ch3; };   // Channel naming (CH1, CH2, CH3)
        struct { int32_t x; int32_t y; int32_t z; };   // Channel naming (CH1, CH2, CH3)
        int32_t arr[3];                           // Array access
    };

    /// Default constructor - zero initialize
    Vec3D_i32() : a(0), b(0), c(0) {}

    /// Constructor with individual values
    Vec3D_i32(int32_t val_a, int32_t val_b, int32_t val_c) : a(val_a), b(val_b), c(val_c) {}

    /// Array subscript operator for indexed access
    int32_t& operator[](size_t idx) { return arr[idx]; }
    const int32_t& operator[](size_t idx) const { return arr[idx]; }
};