#ifndef SHAPE_H_
#define SHAPE_H_

#include "vector3.h"

class Shape {
public:
    virtual ~Shape() = default;
    virtual float SDF(const Vector3& point) const = 0;
};

#endif // SHAPE_H_ 