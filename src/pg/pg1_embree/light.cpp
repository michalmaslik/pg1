#include "stdafx.h"
#include "light.h"

Light::Light(const Vector3& origin)
{
	origin_ = origin;
}

void Light::SetOrigin(const Vector3& newOrigin)
{
	origin_ = newOrigin;
}

Vector3 Light::GetOrigin() const
{
	return origin_;
}
