#include "stdafx.h"
#include "cubeMap.h"

// Constructor: Loads six textures from the given file paths
CubeMap::CubeMap(const char* fileNames[6])
{
	for (int i = 0; i < 6; i++) {
		textures_[i] = new Texture(fileNames[i]);
	}
}

// Destructor: Cleans up allocated textures
CubeMap::~CubeMap()
{
	for (int i = 0; i < 6; ++i) {
		if (textures_[i]) {
			delete textures_[i];
			textures_[i] = nullptr;
		}
	}
}

// Samples a texel from the cubemap based on the given direction vector
Color3f CubeMap::GetTexel(const Vector3& direction) const
{
	Vector3 dir = direction;
	dir.Normalize(); // Normalize the direction vector
	float u = 0.0f;
	float v = 0.0f;

	// Determine the dominant axis of the direction vector
	switch (dir.LargestComponent(true))
	{
	case 0: // X-axis
	{
		const float tmp = 1.0f / abs(dir.x);
		u = (dir.y * tmp + 1.0f) * 0.5f;
		v = (dir.z * tmp + 1.0f) * 0.5f;
		if (dir.x > 0.0f) {
			v = 1.0f - v;
			u = 1.0f - u;
			return textures_[0]->get_texel(u, v); // Positive X face
		}
		else {
			v = 1.0f - v;
			return textures_[3]->get_texel(u, v); // Negative X face
		}
		break;
	}
	case 1: // Y-axis
	{
		const float tmp = 1.0f / abs(dir.y);
		u = (dir.x * tmp + 1.0f) * 0.5f;
		v = (dir.z * tmp + 1.0f) * 0.5f;
		if (dir.y > 0.0f) {
			v = 1.0f - v;
			return textures_[2]->get_texel(u, v); // Positive Y face
		}
		else {
			u = 1.0f - u;
			v = 1.0f - v;
			return textures_[5]->get_texel(u, v); // Negative Y face
		}
		break;
	}
	case 2: // Z-axis
	{
		const float tmp = 1.0f / abs(dir.z);
		u = (dir.x * tmp + 1.0f) * 0.5f;
		v = (dir.y * tmp + 1.0f) * 0.5f;
		if (dir.z > 0.0f) {
			return textures_[1]->get_texel(u, v); // Positive Z face
		}
		else {
			v = 1.0f - v;
			return textures_[4]->get_texel(u, v); // Negative Z face
		}
		break;
	}
	}

	return Color3f(); // Return a default color if no face is selected
}

