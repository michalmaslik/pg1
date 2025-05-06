#ifndef SHAPE_H_  
#define SHAPE_H_  

#include "mymath.h"  
#include "noise.h"  

/*! \class Shape
\brief Abstract base class for 3D shapes.

This class provides a common interface for all shapes, including methods for computing
the Signed Distance Function (SDF) and its noisy variants.

*/
class Shape {
public:
	// Constructor: Initializes the shape with a smoothing factor and noise
	Shape(const float k = 1.0f, const Noise& noise = {}) : k_(k), noise_(noise) {}

	// Virtual destructor: Ensures proper cleanup of derived classes
	virtual ~Shape() = default;

	// Computes the Signed Distance Function (SDF) for a given point
	virtual float SDF(Vector3 point) const = 0;

	// Computes the SDF with noise applied
	virtual float SDFNoise(Vector3 point) const = 0;

	// Computes the SDF with a specific noise configuration
	virtual float SDFNoise(Vector3 point, const Noise& noise) const = 0;

	float k_;  // Smoothing factor for blending shapes
	Noise noise_;  // Noise configuration for the shape

	static bool useNoise; // Global flag to enable or disable noise
};

#endif
