#include "stdafx.h"
#include "light.h"

Light::Light(const Vector3 origin)
{
	org = origin;
}

void Light::SetOrigin(const Vector3 new_origin)
{
	org = new_origin;
}

Vector3 Light::GetOrigin()
{
	return org;
}
