#ifndef NOISE_H_
#define NOISE_H_

#include "vector3.h"
#include "mymath.h"
#include "matrix3x3.h"

/*! \file noise.h
\brief Provides noise generation functions and a Noise class for procedural effects.

This file includes implementations of hash-based 3D value noise, fractal Brownian motion (FBM),
and a general-purpose `Noise` class for generating procedural noise based on different types.

Terminology note:
  The standalone `noise()` function implements a **hash-based 3D value noise**: scalar hash values
  are placed at integer lattice points and trilinearly interpolated using quintic smoothstep weights
  (the same construction used in Inigo Quilez's Rainforest ShaderToy). This is visually equivalent
  to classic Perlin noise and is referred to as `NoiseType::Perlin` throughout the engine for
  historical/legacy reasons.  True gradient Perlin noise (which uses random gradient vectors at
  lattice points rather than scalar hashes) is not currently required and has not been implemented.

*/

// --------------------------------------------//
//               Noise Functions
// --------------------------------------------//
// Taken from Inigo Quilez's Rainforest ShaderToy:
// https://www.shadertoy.com/view/4ttSWf

// Computes the fractional part of a float value
inline float fract(float value) {
    return value - std::floor(value);
}

// Hash function for generating pseudo-random values
inline float hash1(float n) {
    return fract(n * 17.0f * fract(n * 0.3183099f));
}

// Computes hash-based 3D value noise for a given point.
// Scalar pseudo-random values are placed at integer lattice corners via hash1() and
// trilinearly interpolated using quintic smoothstep weights (6t^5 - 15t^4 + 10t^3).
// Output range: [-1, 1].
// Source: Inigo Quilez's Rainforest ShaderToy (https://www.shadertoy.com/view/4ttSWf)
inline float noise(const Vector3& x) {
    Vector3 p = Vector3(std::floor(x.x), std::floor(x.y), std::floor(x.z)); // Integer part
    Vector3 w = Vector3(x.x - p.x, x.y - p.y, x.z - p.z); // Fractional part

    // Smoothstep interpolation weights
    Vector3 u = w * w * w * (w * (w * 6.0f - Vector3(15.0f, 15.0f, 15.0f)) + Vector3(10.0f, 10.0f, 10.0f));

    float n = p.x + 317.0f * p.y + 157.0f * p.z;

    // Compute noise contributions from the corners of the cube
    float a = hash1(n + 0.0f);
    float b = hash1(n + 1.0f);
    float c = hash1(n + 317.0f);
    float d = hash1(n + 318.0f);
    float e = hash1(n + 157.0f);
    float f = hash1(n + 158.0f);
    float g = hash1(n + 474.0f);
    float h = hash1(n + 475.0f);

    // Interpolate the noise values
    float k0 = a;
    float k1 = b - a;
    float k2 = c - a;
    float k3 = e - a;
    float k4 = a - b - c + d;
    float k5 = a - c - e + g;
    float k6 = a - b - e + f;
    float k7 = -a + b + c - d + e - f - g + h;

    return -1.0f + 2.0f * (k0 + k1 * u.x + k2 * u.y + k3 * u.z + k4 * u.x * u.y + k5 * u.y * u.z + k6 * u.z * u.x + k7 * u.x * u.y * u.z);
}

// Predefined rotation matrix for fractal Brownian motion (FBM)
const Matrix3x3 m3 = Matrix3x3(
    0.00f, 0.80f, 0.60f,
    -0.80f, 0.36f, -0.48f,
    -0.60f, -0.48f, 0.64f
);

// Computes fractal Brownian motion (FBM) with 4 octaves
inline float fbm_4(Vector3 x) {  // Removed reference to make copy
    float f = 2.0; // Frequency multiplier
    float s = 0.5; // Amplitude scaling factor
    float a = 0.0; // Accumulated noise value
    float b = 0.5; // Initial amplitude
    int octaves = 4;

    for (int i = 0; i < octaves; i++) {
        float n = noise(x);
        a += b * n;
        b *= s;
        x = f * m3 * x; // Rotate and scale the input point (now safely modifying copy)
    }
    return a;
}

/*! \class Noise
\brief A class for generating procedural noise.

This class provides an interface for generating noise based on different types, such as Perlin noise
and fractal Brownian motion (FBM). It supports scaling and strength adjustments for customization.

*/
class Noise {
public:
    /*! \enum NoiseType
    \brief Specifies the type of noise to generate.
    */
    enum class NoiseType {
        None,                   /*!< No noise applied */
        Perlin,                 /*!< Hash-based 3D value noise (delegates to standalone noise()) */
        FractalBrownianMotion   /*!< Fractal Brownian Motion — 4 octaves of value noise (fbm_4()) */
    };

    // Constructor: Initializes the noise generator with a type, scale, and strength
    Noise(const NoiseType type = NoiseType::None, const float scale = 1.0f, const float strength = 1.0f)
        : type_(type), scale_(scale), strength_(strength) {
    }

    // Generates noise for a given point
    [[nodiscard]] float Generate(const Vector3& point) const {
        Vector3 scaledPoint = point * scale_;  // Create scaled copy
        
        switch (type_) {
        case NoiseType::Perlin:
            return PerlinNoise(scaledPoint) * strength_;
        case NoiseType::FractalBrownianMotion:
            return FractalBrownianMotion(scaledPoint) * strength_;
        case NoiseType::None:
        default:
            return 0.0f;
        }
    }

    // Sets the type of noise to generate
    void SetType(const NoiseType type) { type_ = type; }

    // Sets the scale of the noise
    void SetScale(const float scale) { scale_ = scale; }

    // Sets the strength of the noise
    void SetStrength(const float strength) { strength_ = strength; }

private:
    NoiseType type_;  // Type of noise to generate
    float scale_;     // Scale factor for the input point
    float strength_;  // Strength of the noise

    // Computes hash-based 3D value noise for a given point.
    // Delegates to the standalone noise() function, which places scalar hash values at
    // integer lattice corners and interpolates them with quintic smoothstep weights.
    // The 'point' argument has already been multiplied by scale_ in Generate(), and
    // the result will be multiplied by strength_ there — no further scaling is applied here.
    // Output range: [-1, 1].
    [[nodiscard]] float PerlinNoise(const Vector3& point) const {
        return noise(point);
    }

    // Computes fractal Brownian motion (FBM) for a given point
    [[nodiscard]] float FractalBrownianMotion(const Vector3& point) const {
        return fbm_4(point);  // fbm_4 now takes copy, so this is safe
    }
};

#endif
