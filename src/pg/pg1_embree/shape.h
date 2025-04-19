#ifndef SHAPE_H_
#define SHAPE_H_

#include "vector3.h"
#include "noise.h"

class Shape {
public:
	Shape(const Noise& noise = {}, const bool useNoise = false) : noise_(noise), useNoise_(useNoise) {}
    virtual ~Shape() = default;
    virtual float SDF(const Vector3& point) const = 0;
    void SetNoise(const Noise& noise) { noise_ = noise; }
    void SetUseNoise(const bool useNoise) { useNoise_ = useNoise; }

protected:
    Noise noise_;
    bool useNoise_;
};

#endif