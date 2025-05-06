#ifndef PLANE_H_
#define PLANE_H_

#include "shape.h"

/*! \class Plane
\brief Represents an infinite plane in 3D space.

This class implements the Signed Distance Function (SDF) for a plane. The plane is defined
by the equation `z + d = 0`, where `d` is the distance from the origin along the z-axis.

*/
class Plane : public Shape {
public:
    // Constructor: Initializes the plane with a distance from the origin and a smoothing factor
    Plane(const float d, const float k = 1.0f)
        : Shape(k, {}), d_(d) {
    }

    // Computes the Signed Distance Function (SDF) for a given point
    float SDF(Vector3 point) const override {
        return point.z + d_;
    }

    // Computes the SDF with noise applied (no noise is applied for a plane)
    float SDFNoise(Vector3 point) const override {
        return SDF(point);
    }

    // Computes the SDF with a specific noise configuration (no noise is applied for a plane)
    float SDFNoise(Vector3 point, const Noise& noise) const override {
        return SDF(point);
    }

private:
    float d_; // Distance of the plane from the origin along the z-axis
};

#endif

