#ifndef CUBEMAP_H_
#define CUBEMAP_H_

#include "vector3.h"
#include "matrix3x3.h"
#include <string>
#include <memory>
#include "texture.h"


class CubeMap
{
public:
	CubeMap(const char* file_names[6]);
	~CubeMap();
	Color3f get_texel(Vector3 direction) const;
	Texture* textures_[6];

private:

};


#endif
#pragma once
