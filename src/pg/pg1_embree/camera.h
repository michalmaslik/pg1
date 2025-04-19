#ifndef CAMERA_H_
#define CAMERA_H_

#include "vector3.h"
#include "matrix3x3.h"

/*! \class Camera
\brief A simple pin-hole camera.

\author Tomáš Fabián
\version 1.0
\date 2018
*/
class Camera
{
public:
	Camera() {}

	Camera(const int width, const int height, const float fovY,
		const Vector3& viewFrom, const Vector3& viewAt);

	/* generate primary ray, top-left pixel image coordinates (xi, yi) are in the range <0, 1) x <0, 1) */
	RTCRay GenerateRay(const float x_i, const float y_i) const;

	void SetViewFrom(const Vector3& newViewFrom);

	Vector3 GetViewFrom() const;

	void SetViewAt(const Vector3& newViewAt);

	Vector3 GetViewAt() const;

	void SetRotation(const Vector3& newRotation);

	Vector3 GetRotation() const;

private:
	void Update_m_c_w_();
	
	int width_{ 640 }; // image width (px)
	int height_{ 480 };  // image height (px)
	float fovY_{ 0.785f }; // vertical field of view (rad)

	Vector3 viewFrom_; // ray origin or eye or O
	Vector3 viewAt_; // target T
	Vector3 up_{ Vector3(0.0f, 0.0f, 1.0f) }; // up vector

	float fY_{ 1.0f }; // focal lenght (px)

	Matrix3x3 m_c_w_; // transformation matrix from CS -> WS	

	Vector3 rotation_{ 0.0f, 0.0f, 0.0f };

	Matrix3x3 RotateCameraX(const float degreeAngle) const;
	Matrix3x3 RotateCameraY(const float degreeAngle) const;
	Matrix3x3 RotateCameraZ(const float degreeAngle) const;
	Matrix3x3 RotateCamera(const Vector3& degreeAngles) const;
	Matrix3x3 RotateCamera() const;

};

#endif
