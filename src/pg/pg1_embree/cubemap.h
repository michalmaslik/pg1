#ifndef CUBEMAP_H_
#define CUBEMAP_H_

#include "vector3.h"
#include "matrix3x3.h"
#include <string>
#include <memory>
#include "texture.h"

/*! \class CubeMap
\brief Represents a cubemap texture for environment mapping.

This class manages six textures that form a cubemap and provides functionality to sample a texel
based on a 3D direction vector.

*/
class CubeMap
{
public:
    // Constructor: Loads six textures from the given file paths
    CubeMap(const char* fileNames[6]);

    // Destructor: Cleans up allocated textures
    ~CubeMap();

    // Samples a texel from the cubemap based on the given direction vector
    Color3f GetTexel(Vector3& direction) const;

private:
    Texture* textures_[6]; // Array of six textures representing the cubemap faces
};

#endif