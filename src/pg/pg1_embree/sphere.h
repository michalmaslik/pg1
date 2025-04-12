#ifndef SPHERE_H_
#define SPHERE_H_

#include "shape.h"

class Sphere : public Shape {
public:
    Sphere(const Vector3& center, float radius) : center_(center), radius_(radius) {}
    
    float SDF(const Vector3& point) const override {
        return (point - center_).L2Norm() - radius_;
    }
    
private:
    Vector3 center_;
    float radius_;
};

#endif // SPHERE_H_ 