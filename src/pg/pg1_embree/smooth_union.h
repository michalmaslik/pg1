#ifndef SMOOTH_UNION_H_
#define SMOOTH_UNION_H_

#include "shape.h"

/*! \class SmoothUnion
\brief Represents a smooth union of multiple 3D shapes.

This class implements the Signed Distance Function (SDF) for a smooth union of shapes.
The smoothness is controlled by a blending factor `k`.

*/
class SmoothUnion : public Shape {
public:
    // Constructor: Initializes the smooth union with a list of shapes and noise
    SmoothUnion(std::vector<Shape*> shapes, const Noise& noise) : Shape(1.0f, noise), shapes_(shapes) {}

    // Adds a shape to the union
    void AddShape(Shape* shape) {
        shapes_.push_back(shape);
    }

    // Computes the Signed Distance Function (SDF) for a given point
    float SDF(Vector3 point) const override {
        if (Shape::useNoise) {
            return SDFNoise(point);
        }

        if (shapes_.empty()) return FLT_MAX;

        float sdfValue = shapes_[0]->SDF(point);
        for (size_t i = 1; i < shapes_.size(); ++i) {
            float d1 = sdfValue;
            float d2 = shapes_[i]->SDF(point);
            sdfValue = Smooth(d1, d2, shapes_[i]->k_);
        }
        return sdfValue;
    }

    // Computes the SDF with noise applied
    float SDFNoise(Vector3 point) const override {
        return SDFNoise(point, noise_);
    }

    // Computes the SDF with a specific noise configuration
    float SDFNoise(Vector3 point, const Noise& noise) const override {
        if (shapes_.empty()) return FLT_MAX;

        float sdfValue = shapes_[0]->SDFNoise(point, noise);
        for (size_t i = 1; i < shapes_.size(); ++i) {
            float d1 = sdfValue;
            float d2 = shapes_[i]->SDFNoise(point, noise);
            sdfValue = Smooth(d1, d2, shapes_[i]->k_);
        }
        return sdfValue;
    }

    // Computes the smooth union of two SDF values
    static float Smooth(const float d1, const float d2, const float k) {
        float h = clamp(0.5f + 0.5f * (d2 - d1) / k, 0.0f, 1.0f);
        return (d2 * (1.0f - h) + d1 * h) - k * h * (1.0f - h);
    }

    std::vector<Shape*> shapes_; // List of shapes in the union

};

#endif

