#ifndef BOX_H_
#define BOX_H_

#include "shape.h"
#include "mymath.h"

/*! \class Box
\brief Represents an axis-aligned box in 3D space.

This class implements the Signed Distance Function (SDF) for an axis-aligned box.
The box is defined by its center and half-size along each axis.

*/
class Box : public Shape {
public:
    // Constructor: Initializes the box with a center, half-size, smoothing factor, and noise
    Box(const Vector3& center, const Vector3& halfSize, const float k = 1.0f, const Noise& noise = {})
        : Shape(k, noise) {
        center_ = center;
        halfSize_ = halfSize;
    }

    // Computes the Signed Distance Function (SDF) for a given point
    float SDF(Vector3 point) const override {
        if (Shape::useNoise) {
            return SDFNoise(point);
        }
        Vector3 q = (point - center_).Abs() - halfSize_;
        return q.Max(0.0f).L2Norm() + std::min(std::max(q.x, std::max(q.y, q.z)), 0.0f);
    }

    // Computes the SDF with noise applied
    float SDFNoise(Vector3 point) const override {
        return SDFNoise(point, noise_);
    }

    // Computes the SDF with a specific noise configuration
    float SDFNoise(Vector3 point, const Noise& noise) const override {
        Vector3 q = (point - center_).Abs() - halfSize_;
        float distance = q.Max(0.0f).L2Norm() + std::min(std::max(q.x, std::max(q.y, q.z)), 0.0f);
        return distance + noise.Generate(point);
    }

private:
    Vector3 center_;   // Center of the box
    Vector3 halfSize_; // Half-size of the box along each axis
};

#endif

