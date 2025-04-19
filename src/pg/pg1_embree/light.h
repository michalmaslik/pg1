#ifndef LIGHT_H_
#define LIGHT_H_

#include "vector3.h"
#include "matrix3x3.h"


class Light
{
public:
	Light() {}

	Light(const Vector3& origin);

	void SetOrigin(const Vector3& newOrigin);

	Vector3 GetOrigin() const;

private:

	Vector3 origin_;

};

#endif