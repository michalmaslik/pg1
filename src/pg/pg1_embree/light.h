#ifndef LIGHT_H_
#define LIGHT_H_

#include "vector3.h"
#include "matrix3x3.h"

/*! \class Light
\brief Represents a light source in the scene.

This class provides basic functionality for managing a light source, including its position in 3D space.

*/
class Light
{
public:
    // Default constructor: Initializes the light with no specific position
    Light() {}

    // Constructor: Initializes the light with a given origin
    Light(const Vector3& origin) : origin_(origin) {};

    Vector3 origin_; // Position of the light source in 3D space

};

#endif