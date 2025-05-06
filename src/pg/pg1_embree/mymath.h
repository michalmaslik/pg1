#ifndef MY_MATH_H_
#define MY_MATH_H_

#define _USE_MATH_DEFINES
#include <math.h>
#include <float.h>
#include "structs.h"

template <class T> inline T sqr(const T x)
{
	return x * x;
}

inline Normal3f normalize(const Normal3f& n)
{
	float tmp = sqr(n.x) + sqr(n.y) + sqr(n.z);

	if (fabsf(tmp) > FLT_EPSILON)
	{
		tmp = 1.0f / tmp;
		return Normal3f{ n.x * tmp, n.y * tmp, n.z * tmp };
	}

	return n;
}

inline float deg2rad(const float x)
{
	return x * float(M_PI) / 180.0f;
}


inline float rad2deg(const float x)
{
	return x * 180.0f / float(M_PI);
}

inline float clamp(const float x, const float x0 = 0.0f, const float x1 = 1.0f) {
	return max(min(x, x1), x0);
}

#endif
