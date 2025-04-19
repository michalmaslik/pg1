#ifndef NOISE_H_
#define NOISE_H_

#include "vector3.h"

class Noise {
public:
    enum class NoiseType {
        None,
        Perlin,
        FractalBrownianMotion
    };

    Noise(const NoiseType type = NoiseType::None, const float scale = 1.0f, const float strength = 0.0f)
        : type_(type), scale_(scale), strength_(strength) {}

    float Generate(const Vector3& point) const {
        switch (type_) {
            case NoiseType::Perlin:
                return PerlinNoise(point * scale_) * strength_;
            case NoiseType::FractalBrownianMotion:
                return FractalBrownianMotion(point * scale_) * strength_;
            case NoiseType::None:
            default:
                return 0.0f;
        }
    }

    void SetType(const NoiseType type) { type_ = type; }

    void SetScale(const float scale) { scale_ = scale; }
    void SetStrength(const float strength) { strength_ = strength; }

private:
    NoiseType type_;  
    float scale_;      
    float strength_;   

    float PerlinNoise(const Vector3& point) const {
        return sin(point.x) * cos(point.y) * sin(point.z);
    }

    float FractalBrownianMotion(const Vector3& point) const {
        float total = 0.0f;
        float amplitude = 1.0f;
        float frequency = 1.0f;
        const int octaves = 5;

        for (int i = 0; i < octaves; ++i) {
            total += PerlinNoise(point * frequency) * amplitude;
            frequency *= 2.0f;
            amplitude *= 0.5f;
        }

        return total;
    }
};

#endif
