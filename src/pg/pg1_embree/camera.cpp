#include "stdafx.h"
#include "camera.h"
#include "mymath.h"
#include <iostream>


Camera::Camera(const int width, const int height, const float fov_y,
	const Vector3 view_from, const Vector3 view_at)
{
	width_ = width;
	height_ = height;
	fov_y_ = fov_y;

	view_from_ = view_from;
	view_at_ = view_at;

	f_y_ = height / (2 * tan(fov_y / 2));

	Vector3 z_c = view_from_ - view_at_;
	z_c.Normalize();
	Vector3 x_c = up_.CrossProduct(z_c);
	x_c.Normalize();
	Vector3 y_c = z_c.CrossProduct(x_c);
	y_c.Normalize();
	M_c_w_ = Matrix3x3(x_c, y_c, z_c);
}

void Camera::SetViewFrom(const Vector3 new_view_from)
{
	view_from_ = new_view_from;
}

Vector3 Camera::GetViewFrom()
{
	return view_from_;
}

void Camera::SetViewAt(const Vector3 new_view_at)
{
	view_at_ = new_view_at;
}

Vector3 Camera::GetViewAt()
{
	return view_at_;
}

void Camera::SetRotation(const Vector3 new_rotation)
{
	rotation_.x = (static_cast<int>(new_rotation.x) % 360 + 360) % 360;
	rotation_.y = (static_cast<int>(new_rotation.y) % 360 + 360) % 360;
	rotation_.z = (static_cast<int>(new_rotation.z) % 360 + 360) % 360;
}

Vector3 Camera::GetRotation()
{
	return rotation_;
}

Matrix3x3 Camera::rotate_camera_x(const float degree_angle) const
{
	return M_c_w_ * Matrix3x3::RotationMatrixX(degree_angle);
}

Matrix3x3 Camera::rotate_camera_y(const float degree_angle) const
{
	return M_c_w_ * Matrix3x3::RotationMatrixY(degree_angle);
}

Matrix3x3 Camera::rotate_camera_z(const float degree_angle) const
{
	return M_c_w_ * Matrix3x3::RotationMatrixZ(degree_angle);
}

Matrix3x3 Camera::rotate_camera(const Vector3 degree_angles) const
{
	return M_c_w_ * Matrix3x3::RotationMatrixX(degree_angles.x) * Matrix3x3::RotationMatrixY(degree_angles.y) * Matrix3x3::RotationMatrixZ(degree_angles.z);
}

Matrix3x3 Camera::rotate_camera() const
{
	return M_c_w_ * Matrix3x3::RotationMatrixX(rotation_.x) * Matrix3x3::RotationMatrixY(rotation_.y) * Matrix3x3::RotationMatrixZ(rotation_.z);
}

RTCRay Camera::GenerateRay(const float x_i, const float y_i) const
{
	RTCRay ray = RTCRay();
	ray.org_x = view_from_.x;
	ray.org_y = view_from_.y;
	ray.org_z = view_from_.z;
	ray.tnear = FLT_MIN;

	Vector3 d_c = Vector3{ x_i - width_ / 2, height_ / 2 - y_i, -f_y_ };
	d_c.Normalize();
	Vector3 d_w = rotate_camera() * d_c; // rotate_camera returns rotated M_c_w_ matrix
	d_c.Normalize();
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