#ifndef RAYTRACER_H_
#define RAYTRACER_H_

#include "simpleguidx11.h"
#include "surface.h"
#include "camera.h"
#include "light.h"
#include "cubemap.h"
#include "mymath.h"
#include "material.h"
#include "shape.h"
#include "sphere.h"
#include "box.h"
#include "plane.h"
#include "smooth_union.h"
#include "vector4.h"

/*! \class RayTracer
\brief General ray tracer class.

This class implements a ray tracer capable of rendering 3D scenes using ray tracing and ray marching techniques.
It supports features such as volumetric rendering, shading models, and camera movement.

*/

// Represents a transformation in 3D space, including position, scale, and rotation
struct Transform {
    Vector3 position = Vector3(0.0f, 0.0f, 0.0f); // Position in 3D space
    Vector3 scale = Vector3(1.0f, 1.0f, 1.0f);    // Scale factors along each axis
    Vector3 rotation = Vector3(0.0f, 0.0f, 0.0f); // Euler angles in degrees
};

// Stores information about a 3D model, including its file path and transformation
struct ModelInfo {
    std::string filePath; // Path to the model file
    Transform transform;  // Transformation to apply to the model
};

// Main ray tracer class, inheriting from SimpleGuiDX11 for GUI and rendering support
class RayTracer : public SimpleGuiDX11
{
public:
    // Constructor: Initializes the ray tracer with the given parameters
    RayTracer(const int width, const int height,
        const float fovY, const Vector3& viewFrom, const Vector3& viewAt, const Vector3& lightOrigin,
        const char* config = "threads=0,verbose=3");

    // Destructor: Cleans up allocated resources
    ~RayTracer();

    // Initializes the Embree device and scene
    int InitDeviceAndScene(const char* config);

    // Releases the Embree device and scene
    int ReleaseDeviceAndScene();

    // Loads a scene with models, volumetric shapes, and a cubemap
    void LoadScene(
        const std::vector<ModelInfo>& models, // List of models to load
        const std::vector<Shape*>& shapes,   // List of volumetric shapes
        const char* cubeMapFileNames[6]      // File paths for the cubemap textures
    );

    // Loads a single 3D model and applies the given transformation
    void LoadModel(const std::string& fileName, const Transform& transform = Transform());

    // Checks if a point is visible from a light source
    bool IsHitPointVisible(const Vector3& hitPoint, const Vector3& lightPoint);

    // Computes the color of a single pixel in the rendered image
    Color4f GetPixel(const int x, const int y, const float t = 0.0f) override;

    // Moves the camera in a circular path around the target point
    void MoveCamera() override;

    // Traces a ray through the scene and determines the color at the intersection point
    Vector3 TraceRay(const RTCRay& ray, const float n_1 = 1.0f, const int depth = 0, const int maxDepth = 10);

    // Shading models
    Vector3 NormalShader(const Vector3& normalVector); // Visualizes the normal vector as a color
    Vector3 LambertShader(const Material& material, const Coord2f& texCoord, const Vector3& hitPoint, const Vector3& normalVector); // Diffuse shading
    Vector3 PhongShader(const Material& material, const Coord2f& texCoord, const Vector3& hitPoint, const Vector3& normalVector, const Vector3& directionVector, const int depth); // Specular highlights
    Vector3 TransparentShader(const RTCRay& ray, const Vector3& hitPoint, const Vector3& normalVector, const Vector3& directionVector, const Material& material, const float n_1, const int depth); // Refraction and reflection

    // Volumetric rendering
    Vector4 VolumetricRender(const RTCRay& ray); // Handles volumetric rendering
    Vector4 VolumetricEffect(const RTCRay& ray); // Computes volumetric effects using ray marching
    Vector4 SurfaceEffect(const RTCRay& ray);   // Computes surface effects for volumetric shapes

    // Renders the user interface for controlling ray tracing parameters
    int Ui() override;

    // Public settings for ray tracing and rendering
    bool useNoise_{ true };               // Enables or disables noise
    float noiseScale_{ 0.7f };            // Scale of the noise
    float noiseStrength_{ 3.0f };         // Strength of the noise
    float smoothFactor_{ 1.0f };          // Smoothness factor for volumetric shapes

    bool rayMarching_{ false };           // Enables or disables ray marching
    float stepSize_{ 0.01f };             // Step size for ray marching
    float maxDistance_{ 50.0f };          // Maximum distance for ray marching
    int maxSteps_ = static_cast<int>(maxDistance_ / stepSize_); // Maximum number of steps for ray marching
    float absorptionCoefficient_{ 0.5f }; // Absorption coefficient for volumetric rendering
    float lightAttenuationFactor_{ 1.65f }; // Light attenuation factor
    float volumetricAlbedo_[3]{ 0.0f, 0.0f, 0.0f }; // Albedo for volumetric rendering
    Vector3 volumetricAlbedoVec_{ 0.0f, 0.0f, 0.0f }; // Albedo as a vector

    bool sampling_{ false };              // Enables or disables supersampling

    bool backgroundEnabled_{ false };    // Enables or disables the background
    float backgroundColor_[3]{ 1.0f, 1.0f, 1.0f }; // Background color
    Vector3 backgroundColorVec_{ 1.0f, 1.0f, 1.0f }; // Background color as a vector

    bool cameraMovementEnabled_{ true }; // Enables or disables camera movement
    float cameraX_{ 0.0f };              // Camera position along the X-axis
    float cameraY_{ 0.0f };              // Camera position along the Y-axis
    float cameraZ_{ 0.0f };              // Camera position along the Z-axis

private:
    // Private members for managing the scene and rendering
    std::vector<Shape*> volumetricShapes_; // List of volumetric shapes in the scene
    std::vector<Surface*> surfaces_;       // List of surfaces in the scene
    std::vector<Material*> materials_;     // List of materials in the scene
    CubeMap* cubemap_ = nullptr;           // Cubemap for the scene

    RTCDevice device_;                     // Embree device
    RTCScene scene_;                       // Embree scene
    Camera camera_;                        // Camera for the scene
    Light light_;                          // Light source for the scene
};

#endif