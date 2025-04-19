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
	CubeMap(const char* fileNames[6]);
	~CubeMap();
	Color3f GetTexel(Vector3& direction) const;
	

private:
	Texture* textures_[6];

};

#endif