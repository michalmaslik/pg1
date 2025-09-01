#ifndef PLANE_H_
#define PLANE_H_

#include "shape.h"

/*! \class Plane
\brief Represents an infinite plane in 3D space.

This class implements the Signed Distance Function (SDF) for a plane.

MATHEMATICAL REFERENCE:
For a plane defined by normal vector n and distance d from origin:
float sdPlane(vec3 p, vec4 n) { 
    return dot(p, n.xyz) + n.w; 
}

Currently implemented as z-axis aligned plane (normal = (0,0,1)):
SDF(p) = p.z + d

ENHANCEMENT SUGGESTION: Support arbitrary plane orientation with normal vector.

*/
class Plane : public Shape {
public:
    // Constructor: Initializes a Z-aligned plane with distance from origin
    Plane(const float d, const float k = 1.0f)
        : Shape(k, {}), d_(d), normal_(0.0f, 0.0f, 1.0f) {
    }
    
    // Enhanced constructor: Plane defined by normal vector and distance
    Plane(const Vector3& normal, const float d, const float k = 1.0f)
        : Shape(k, {}), d_(d), normal_(normal) {
    }

    // Computes the Signed Distance Function (SDF) for a given point
    float SDF(Vector3 point) const override {
        if (Shape::useNoise) {
            return SDFNoise(point);
        }
        
        // General plane SDF: dot(point, normal) + distance
        return point.DotProduct(normal_) + d_;
    }

    // Computes the SDF with noise applied
    float SDFNoise(Vector3 point) const override {
        return SDFNoise(point, noise_);
    }

    // Computes the SDF with a specific noise configuration
    float SDFNoise(Vector3 point, const Noise& noise) const override {
        float baseSDF = point.DotProduct(normal_) + d_;
        // NOTE: Planes are infinite, so noise application should be careful
        // We apply noise based on the point position for consistency
        return baseSDF + noise.Generate(point);
    }

private:
    float d_;        // Distance of the plane from the origin along normal
    Vector3 normal_; // Normal vector of the plane (should be normalized)
};

#endif

