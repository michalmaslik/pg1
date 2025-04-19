#include "stdafx.h"
#include "camera.h"
#include "mymath.h"
#include <iostream>


Camera::Camera(const int width, const int height, const float fovY,
	const Vector3& viewFrom, const Vector3& viewAt)
{
	width_ = width;
	height_ = height;
	fovY_ = fovY;

	viewFrom_ = viewFrom;
	viewAt_ = viewAt;

	fY_ = height / (2 * tan(fovY / 2));

	Update_m_c_w_();
}

void Camera::Update_m_c_w_()
{
	Vector3 z_c = viewFrom_ - viewAt_;
	z_c.Normalize();
	Vector3 x_c = up_.CrossProduct(z_c);
	x_c.Normalize();
	Vector3 y_c = z_c.CrossProduct(x_c);
	y_c.Normalize();
	m_c_w_ = Matrix3x3(x_c, y_c, z_c);
}

void Camera::SetViewFrom(const Vector3& newViewFrom)
{
	viewFrom_ = newViewFrom;
	Update_m_c_w_();
}

Vector3 Camera::GetViewFrom() const
{
	return viewFrom_;
}

void Camera::SetViewAt(const Vector3& newViewAt)
{
	viewAt_ = newViewAt;
	Update_m_c_w_();
}

Vector3 Camera::GetViewAt() const
{
	return viewAt_;
}

void Camera::SetRotation(const Vector3& newRotation)
{
	rotation_.x = (float) ( ((int)(newRotation.x) % 360 + 360) % 360 );
	rotation_.y = (float) ( ((int)(newRotation.y) % 360 + 360) % 360 );
	rotation_.z = (float) ( ((int)(newRotation.z) % 360 + 360) % 360 );
}

Vector3 Camera::GetRotation() const
{
	return rotation_;
}

Matrix3x3 Camera::RotateCameraX(const float degreeAngle) const
{
	return m_c_w_ * Matrix3x3::RotationMatrixX(degreeAngle);
}

Matrix3x3 Camera::RotateCameraY(const float degreeAngle) const
{
	return m_c_w_ * Matrix3x3::RotationMatrixY(degreeAngle);
}

Matrix3x3 Camera::RotateCameraZ(const float degreeAngle) const
{
	return m_c_w_ * Matrix3x3::RotationMatrixZ(degreeAngle);
}

Matrix3x3 Camera::RotateCamera(const Vector3& degreeAngles) const
{
	return m_c_w_ * Matrix3x3::RotationMatrixX(degreeAngles.x) * Matrix3x3::RotationMatrixY(degreeAngles.y) * Matrix3x3::RotationMatrixZ(degreeAngles.z);
}

Matrix3x3 Camera::RotateCamera() const
{
	return m_c_w_ * Matrix3x3::RotationMatrixX(rotation_.x) * Matrix3x3::RotationMatrixY(rotation_.y) * Matrix3x3::RotationMatrixZ(rotation_.z);
}

RTCRay Camera::GenerateRay(const float x_i, const float y_i) const
{
	RTCRay ray = RTCRay();
	ray.org_x = viewFrom_.x;
	ray.org_y = viewFrom_.y;
	ray.org_z = viewFrom_.z;
	ray.tnear = FLT_MIN;

	Vector3 d_c = Vector3{ x_i - width_ / 2, height_ / 2 - y_i, -fY_ };
	d_c.Normalize();
	Vector3 d_w = RotateCamera() * d_c; // RotateCamera returns rotated m_c_w_ matrix
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