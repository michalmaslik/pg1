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

	Camera(const int width, const int height, const float fov_y,
		const Vector3 view_from, const Vector3 view_at);

	/* generate primary ray, top-left pixel image coordinates (xi, yi) are in the range <0, 1) x <0, 1) */
	RTCRay GenerateRay(const float xi, const float yi) const;

	void SetViewFrom(const Vector3 new_view_from);

	Vector3 GetViewFrom();

	void SetViewAt(const Vector3 new_view_at);

	Vector3 GetViewAt();

	void SetRotation(const Vector3 new_rotation);

	Vector3 GetRotation();

private:
	int width_{ 640 }; // image width (px)
	int height_{ 480 };  // image height (px)
	float fov_y_{ 0.785f }; // vertical field of view (rad)

	Vector3 view_from_; // ray origin or eye or O
	Vector3 view_at_; // target T
	Vector3 up_{ Vector3(0.0f, 0.0f, 1.0f) }; // up vector

	float f_y_{ 1.0f }; // focal lenght (px)

	Matrix3x3 M_c_w_; // transformation matrix from CS -> WS	

	Vector3 rotation_{ 0.0f, 0.0f, 0.0f };

	Matrix3x3 rotate_camera_x(const float degree_angle) const;
	Matrix3x3 rotate_camera_y(const float degree_angle) const;
	Matrix3x3 rotate_camera_z(const float degree_angle) const;
	Matrix3x3 rotate_camera(const Vector3 degree_angles) const;
	Matrix3x3 rotate_camera() const;

};

#endif
