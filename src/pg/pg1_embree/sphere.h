#ifndef SPHERE_H_
#define SPHERE_H_

#include "shape.h"
#include "mymath.h"

class Sphere : public Shape {
public:
    Sphere(const Vector3& center, const float radius, const Noise& noise = {}, const bool useNoise = false)
        : Shape(noise, useNoise) {
        center_ = center;
        radius_ = radius;
    }

    float SDF(const Vector3& point) const override {
        float distance = (point - center_).L2Norm() - radius_;

        if (useNoise_) {
            distance += noise_.Generate(point);
        }

        return distance;
    }

private:
    Vector3 center_;       
    float radius_;        
};

#endif