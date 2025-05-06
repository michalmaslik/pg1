#include "stdafx.h"
#include "vector4.h"
#include "mymath.h"
#include <iostream>

Vector4::Vector4(const float* v) : x(v[0]), y(v[1]), z(v[2]), w(v[3]) {}

float Vector4::L2Norm() const {
    return std::sqrt(x * x + y * y + z * z + w * w);
}

float Vector4::SqrL2Norm() const {
    return x * x + y * y + z * z + w * w;
}

void Vector4::Normalize() {
    float norm = L2Norm();
    if (norm > 0.0f) {
        x /= norm;
        y /= norm;
        z /= norm;
        w /= norm;
    }
}

float Vector4::DotProduct(const Vector4& v) const {
    return x * v.x + y * v.y + z * v.z + w * v.w;
}

char Vector4::LargestComponent(const bool absolute_value) const {
    float values[4] = { x, y, z, w };
    if (absolute_value) {
        for (int i = 0; i < 4; ++i) {
            values[i] = std::abs(values[i]);
        }
    }
    int largestIndex = 0;
    for (int i = 1; i < 4; ++i) {
        if (values[i] > values[largestIndex]) {
            largestIndex = i;
        }
    }
    return largestIndex;
}

void Vector4::Print() {
    std::cout << "Vector4(" << x << ", " << y << ", " << z << ", " << w << ")" << std::endl;
}

Vector4 operator-(const Vector4& v) {
    return Vector4(-v.x, -v.y, -v.z, -v.w);
}

Vector4 operator+(const Vector4& u, const Vector4& v) {
    return Vector4(u.x + v.x, u.y + v.y, u.z + v.z, u.w + v.w);
}

Vector4 operator-(const Vector4& u, const Vector4& v) {
    return Vector4(u.x - v.x, u.y - v.y, u.z - v.z, u.w - v.w);
}

Vector4 operator*(const Vector4& v, const float a) {
    return Vector4(v.x * a, v.y * a, v.z * a, v.w * a);
}

Vector4 operator*(const float a, const Vector4& v) {
    return Vector4(v.x * a, v.y * a, v.z * a, v.w * a);
}

Vector4 operator*(const Vector4& u, const Vector4& v) {
    return Vector4(u.x * v.x, u.y * v.y, u.z * v.z, u.w * v.w);
}

Vector4 operator/(const Vector4& v, const float a) {
    return Vector4(v.x / a, v.y / a, v.z / a, v.w / a);
}

void operator+=(Vector4& u, const Vector4& v) {
    u.x += v.x;
    u.y += v.y;
    u.z += v.z;
    u.w += v.w;
}

void operator-=(Vector4& u, const Vector4& v) {
    u.x -= v.x;
    u.y -= v.y;
    u.z -= v.z;
    u.w -= v.w;
}

void operator*=(Vector4& v, const float a) {
    v.x *= a;
    v.y *= a;
    v.z *= a;
    v.w *= a;
}

void operator/=(Vector4& v, const float a) {
    v.x /= a;
    v.y /= a;
    v.z /= a;
    v.w /= a;
}

bool operator==(const Vector4& u, const Vector4& v) {
    return u.x == v.x && u.y == v.y && u.z == v.z && u.w == v.w;
}
