#ifndef LIGHT_H_
#define LIGHT_H_

#include "vector3.h"
#include "matrix3x3.h"


class Light
{
public:
	Light() {}

	Light(const Vector3 origin);

	void SetOrigin(const Vector3 new_origin);

	Vector3 GetOrigin();

private:

	Vector3 org;

};

#endif
#pragma once
