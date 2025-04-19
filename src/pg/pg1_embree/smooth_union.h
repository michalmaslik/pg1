#ifndef SMOOTH_UNION_H_
#define SMOOTH_UNION_H_

#include "shape.h"

class SmoothUnion : public Shape {
public:
    SmoothUnion(Shape& a, Shape& b, const float k, const Noise& noise = {}, const bool useNoise = false) : Shape(noise, useNoise) {
		a_ = &a;
		b_ = &b;
		k_ = k;

        if (useNoise) {
            a_->SetNoise(noise);
            a_->SetUseNoise(true);
            b_->SetNoise(noise);
            b_->SetUseNoise(true);
        }
    }

    float SDF(const Vector3& point) const override {
        float d1 = a_->SDF(point);
        float d2 = b_->SDF(point);
        float h = max(k_ - std::abs(d1 - d2), 0.0f) / k_;
        return min(d1, d2) - h * h * k_ * 0.25f;
    }

private:
    Shape* a_;
    Shape* b_;
    float k_;  // parameter pre hladkosť spojenia
};

#endif