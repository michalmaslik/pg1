#include "stdafx.h"
#include "camera.h"
#include "mymath.h"
#include <iostream>

// Constructor: Initializes the camera with specific parameters
Camera::Camera(const int width, const int height, const float fovY,
    const Vector3& viewFrom, const Vector3& viewAt)
{
    width_ = width;
    height_ = height;
    fov_y_ = fovY;

    viewFrom_ = viewFrom;
    viewAt_ = viewAt;

    // Calculate the focal length based on the field of view
    f_y_ = height / (2 * tan(fovY / 2));

    // Update the camera-to-world transformation matrix
    Update_m_c_w_();
}

// Updates the camera-to-world transformation matrix (m_c_w_)
// This matrix is used to transform directions from camera space to world space
void Camera::Update_m_c_w_()
{
    Vector3 z_c = viewFrom_ - viewAt_; // Camera's forward direction
    z_c.Normalize();
    Vector3 x_c = up_.CrossProduct(z_c); // Camera's right direction
    x_c.Normalize();
    Vector3 y_c = z_c.CrossProduct(x_c); // Camera's up direction
    m_c_w_ = Matrix3x3(x_c, y_c, z_c); // Construct the transformation matrix
}

// Sets the camera's position (viewFrom) and updates the transformation matrix
void Camera::SetViewFrom(const Vector3& newViewFrom)
{
    viewFrom_ = newViewFrom;
    Update_m_c_w_();
}

// Gets the camera's current position (viewFrom)
Vector3 Camera::GetViewFrom() const
{
    return viewFrom_;
}

// Sets the camera's target point (viewAt) and updates the transformation matrix
void Camera::SetViewAt(const Vector3& newViewAt)
{
    viewAt_ = newViewAt;
    Update_m_c_w_();
}

// Gets the camera's current target point (viewAt)
Vector3 Camera::GetViewAt() const
{
    return viewAt_;
}

float NormalizeAngle(float angle) {
    return fmodf(angle + 360.0f, 360.0f);
}

// Sets the camera's rotation angles (in degrees)
void Camera::SetRotation(const Vector3& newRotation) {
    // Normalize rotation angles to the range [0, 360)
    rotation_.x = NormalizeAngle(newRotation.x);
    rotation_.y = NormalizeAngle(newRotation.y);
    rotation_.z = NormalizeAngle(newRotation.z);
}

// Gets the camera's current rotation angles (in degrees)
Vector3 Camera::GetRotation() const
{
    return rotation_;
}

// Generates a rotation matrix for rotating around the X-axis
Matrix3x3 Camera::RotateCameraX(const float degreeAngle) const
{
    return m_c_w_ * Matrix3x3::RotationMatrixX(degreeAngle);
}

// Generates a rotation matrix for rotating around the Y-axis
Matrix3x3 Camera::RotateCameraY(const float degreeAngle) const
{
    return m_c_w_ * Matrix3x3::RotationMatrixY(degreeAngle);
}

// Generates a rotation matrix for rotating around the Z-axis
Matrix3x3 Camera::RotateCameraZ(const float degreeAngle) const
{
    return m_c_w_ * Matrix3x3::RotationMatrixZ(degreeAngle);
}

// Generates a rotation matrix for rotating around all three axes
Matrix3x3 Camera::RotateCamera(const Vector3& degreeAngles) const
{
    return m_c_w_ * Matrix3x3::RotationMatrixX(degreeAngles.x) * Matrix3x3::RotationMatrixY(degreeAngles.y) * Matrix3x3::RotationMatrixZ(degreeAngles.z);
}

// Generates a rotation matrix using the camera's current rotation angles
Matrix3x3 Camera::RotateCamera() const
{
    return m_c_w_ * Matrix3x3::RotationMatrixX(rotation_.x) * Matrix3x3::RotationMatrixY(rotation_.y) * Matrix3x3::RotationMatrixZ(rotation_.z);
}

// Generates a primary ray for a given pixel coordinate (x_i, y_i)
RTCRay Camera::GenerateRay(const float x_i, const float y_i) const
{
    RTCRay ray = RTCRay();
    ray.org_x = viewFrom_.x;
    ray.org_y = viewFrom_.y;
    ray.org_z = viewFrom_.z;
    ray.tnear = FLT_MIN;

    // Calculate the direction in camera space
    Vector3 d_c = Vector3{ x_i - width_ / 2, height_ / 2 - y_i, -f_y_ };
    d_c.Normalize();

    // Transform the direction to world space
    Vector3 d_w = RotateCamera() * d_c;
    ray.dir_x = d_w.x;
    ray.dir_y = d_w.y;
    ray.dir_z = d_w.z;
    ray.time = 0.0f;

    ray.tfar = FLT_MAX;

    ray.mask = 0;
    ray.id = 0;
    ray.flags = 0;

    return ray;
}
