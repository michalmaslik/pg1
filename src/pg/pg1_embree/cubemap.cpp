#include "stdafx.h"
#include "cubeMap.h"

CubeMap::CubeMap(const char* fileNames[6])
{
	for (int i = 0; i < 6; i++) {
		textures_[i] = new Texture(fileNames[i]);
	}
}

CubeMap::~CubeMap()
{
	for (int i = 0; i < 6; ++i) {
		if (textures_[i]) {
			delete textures_[i];
			textures_[i] = nullptr;
		}
	}
}

Color3f CubeMap::GetTexel(Vector3& direction) const
{
	direction.Normalize();
	float u = 0.0f;
	float v = 0.0f;
	switch (direction.LargestComponent(true))
	{

	case 0:
	{
		const float tmp = 1.0f / abs(direction.x);
		u = (direction.y * tmp + 1) * 0.5f;
		v = (direction.z * tmp + 1) * 0.5f;
		if (direction.x > 0.0f) {
			v = 1 - v;
			u = 1 - u;
			return textures_[0]->get_texel(u, v);
		}
		else {
			v = 1 - v;
			return textures_[3]->get_texel(u, v);
		}
		break;
	}
	case 1:
	{
		const float tmp = 1.0f / abs(direction.y);
		u = (direction.x * tmp + 1) * 0.5f;
		v = (direction.z * tmp + 1) * 0.5f;
		if (direction.y > 0.0f) {
			v = 1 - v;
			return textures_[2]->get_texel(u, v);
		}
		else {
			u = 1 - u;
			v = 1 - v;
			return textures_[5]->get_texel(u, v);
		}
		break;
	}
	case 2:
	{
		const float tmp = 1.0f / abs(direction.z);
		u = (direction.x * tmp + 1) * 0.5f;
		v = (direction.y * tmp + 1) * 0.5f;
		if (direction.z > 0.0f) {
			return textures_[1]->get_texel(u, v);
		}
		else {
			v = 1 - v;
			return textures_[4]->get_texel(u, v);
		}
		break;
	}
	}

	return Color3f();
}
