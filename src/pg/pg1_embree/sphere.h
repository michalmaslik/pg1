#ifndef SPHERE_H_
#define SPHERE_H_

#include "shape.h"
#include "mymath.h"

/*! \class Sphere
\brief Represents a 3D sphere.

This class implements the Signed Distance Function (SDF) for a sphere and supports
adding noise to the SDF for procedural effects.

*/
class Sphere : public Shape {
public:
    // Constructor: Initializes the sphere with a center, radius, smoothing factor, and noise
    Sphere(const Vector3& center, const float radius, const float k = 1.0f, const Noise& noise = {})
        : Shape(k, noise) {
        center_ = center;
        radius_ = radius;
    }

    // Computes the Signed Distance Function (SDF) for a given point
    float SDF(Vector3 point) const override {
        if (Shape::useNoise) {
            return SDFNoise(point);
        }
        return (point - center_).L2Norm() - radius_;
    }

    // Computes the SDF with noise applied
    float SDFNoise(Vector3 point) const override {
        return SDFNoise(point, noise_);
    }

    // Computes the SDF with a specific noise configuration
    float SDFNoise(Vector3 point, const Noise& noise) const override {
        float distance = (point - center_).L2Norm() - radius_;
        return distance + noise.Generate(point);
    }

private:
    Vector3 center_;  // Center of the sphere
    float radius_;    // Radius of the sphere
};

#endif
