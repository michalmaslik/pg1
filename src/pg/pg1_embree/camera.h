#ifndef CAMERA_H_
#define CAMERA_H_

#include "vector3.h"
#include "matrix3x3.h"

/*! \class Camera
\brief A simple pin-hole camera.

This class represents a pin-hole camera model used for ray tracing. It supports generating primary rays,
setting the camera's position and orientation, and applying rotations.

*/
class Camera
{
public:
    // Default constructor: Initializes the camera with default parameters
    Camera() {}

    // Constructor: Initializes the camera with specific parameters
    Camera(const int width, const int height, const float fovY,
        const Vector3& viewFrom, const Vector3& viewAt);

    // Generates a primary ray for a given pixel coordinate (x_i, y_i)
    // Pixel coordinates are normalized to the range <0, 1) x <0, 1)
    RTCRay GenerateRay(const float x_i, const float y_i) const;

    // Sets the camera's position (viewFrom)
    void SetViewFrom(const Vector3& newViewFrom);

    // Gets the camera's current position (viewFrom)
    Vector3 GetViewFrom() const;

    // Sets the camera's target point (viewAt)
    void SetViewAt(const Vector3& newViewAt);

    // Gets the camera's current target point (viewAt)
    Vector3 GetViewAt() const;

    // Sets the camera's rotation angles (in degrees)
    void SetRotation(const Vector3& newRotation);

    // Gets the camera's current rotation angles (in degrees)
    Vector3 GetRotation() const;

private:
    // Updates the camera-to-world transformation matrix (m_c_w_)
    void Update_m_c_w_();

    int width_{ 640 }; // Image width in pixels
    int height_{ 480 }; // Image height in pixels
    float fov_y_{ 0.785f }; // Vertical field of view in radians

    Vector3 viewFrom_; // Camera position (eye or origin)
    Vector3 viewAt_; // Target point the camera is looking at
    Vector3 up_{ Vector3(0.0f, 0.0f, 1.0f) }; // Up vector for orientation

    float f_y_{ 1.0f }; // Focal length in pixels

    Matrix3x3 m_c_w_; // Transformation matrix from camera space to world space

    Vector3 rotation_{ 0.0f, 0.0f, 0.0f }; // Rotation angles in degrees

    // Helper functions for generating rotation matrices
    Matrix3x3 RotateCameraX(const float degreeAngle) const;
    Matrix3x3 RotateCameraY(const float degreeAngle) const;
    Matrix3x3 RotateCameraZ(const float degreeAngle) const;
    Matrix3x3 RotateCamera(const Vector3& degreeAngles) const;
    Matrix3x3 RotateCamera() const;
};

#endif

