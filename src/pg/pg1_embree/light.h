#ifndef LIGHT_H_
#define LIGHT_H_

#include "vector3.h"
#include "matrix3x3.h"

/*! \enum LightType
\brief Defines the type of light source.
*/
enum class LightType
{
    Point,        // Point light with position and distance-based attenuation
    Directional   // Directional light with direction and no attenuation
};

/*! \class Light
\brief Represents a light source in the scene.

This class provides functionality for managing different types of light sources,
including point lights and directional lights, with color and intensity control.
*/
class Light
{
public:
    // Default constructor: Initializes a white point light at origin
    Light() 
        : type(LightType::Point)
        , position(Vector3(0.0f, 0.0f, 0.0f))
        , direction(Vector3(0.0f, -1.0f, 0.0f))
        , color(Vector3(1.0f, 1.0f, 1.0f))
        , intensity(1.0f) {}

    // Constructor: Initializes a point light with given position (legacy compatibility)
    Light(const Vector3& origin) 
        : type(LightType::Point)
        , position(origin)
        , direction(Vector3(0.0f, -1.0f, 0.0f))
        , color(Vector3(1.0f, 1.0f, 1.0f))
        , intensity(1.0f) {}

    // Full constructor: Initializes a light with all parameters
    Light(LightType lightType, const Vector3& pos, const Vector3& dir, const Vector3& col, float intens)
        : type(lightType)
        , position(pos)
        , direction(dir)
        , color(col)
        , intensity(intens) {}

    LightType type;       // Type of light (POINT or DIRECTIONAL)
    Vector3 position;     // Position of the light (used for POINT lights)
    Vector3 direction;    // Direction of the light (used for DIRECTIONAL lights)
    Vector3 color;        // RGB color of the light (default: white)
    float intensity;      // Intensity/brightness multiplier (default: 1.0)
};

#endif