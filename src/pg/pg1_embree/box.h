#ifndef BOX_H_
#define BOX_H_

#include "shape.h"
#include "mymath.h"

class Box : public Shape {
public:
    Box(const Vector3& center, const Vector3& halfSize, const Noise& noise = {}, const bool useNoise = false)
        : Shape(noise, useNoise) {
        center_ = center;
        halfSize_ = halfSize;
    }

    float SDF(const Vector3& point) const override {
        Vector3 q = (point - center_).Abs() - halfSize_;

        float distance = q.Max(0.0f).L2Norm() + min(max(q.x, max(q.y, q.z)), 0.0f);

        if (useNoise_) {
            distance += noise_.Generate(point);
        }

        return distance;
    }

private:
    Vector3 center_;
    Vector3 halfSize_;
};

#endif