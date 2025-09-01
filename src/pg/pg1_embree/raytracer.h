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
\brief Hybrid Ray Tracer with dual rendering pipeline.

This class implements a sophisticated hybrid renderer that combines two complementary
rendering techniques for maximum flexibility and visual quality:

1. SURFACE RAY TRACING (Intel Embree):
   - Uses Intel Embree library for high-performance triangle mesh rendering
   - BVH (Bounding Volume Hierarchy) acceleration structure
   - Supports traditional polygon-based models (OBJ files)
   - Standard surface shading: Lambert, Phong, transparent materials
   - Hardware-optimized ray-triangle intersections

2. VOLUMETRIC RENDERING (SDF Ray Marching):
   - Custom SDF (Signed Distance Function) ray marching implementation
   - Procedural volumetric shapes with noise
   - Supports both surface (sphere tracing) and volume (ray marching) modes
   - Beer-Lambert absorption for realistic volumetric lighting
   - Smooth unions and CSG operations on procedural shapes

The two pipelines are composited using alpha blending to create the final image.

Key Features:
- Real-time interactive preview with ImGui controls
- Supersampling anti-aliasing support
- Environment mapping with cubemaps
- Shadow ray optimization
- Recursive reflection/refraction rays
- Camera animation and manual controls
- Video export functionality

\author Michal Maslik
\version 1.0
\date 2024
*/

// Represents a transformation in 3D space, including position, scale, and rotation
struct Transform {
    Vector3 position = Vector3(0.0f, 0.0f, 0.0f); // Position in 3D space
    Vector3 scale = Vector3(1.0f, 1.0f, 1.0f);    // Scale factors along each axis
    Vector3 rotation = Vector3(0.0f, 0.0f, 0.0f); // Euler angles in degrees
};

// Stores information about a 3D model, including its file path and transformation
struct ModelInfo {
    std::string filePath; // Path to the OBJ model file
    Transform transform;  // Transformation to apply to the model
};

// Main ray tracer class, inheriting from SimpleGuiDX11 for GUI and rendering support
class RayTracer : public SimpleGuiDX11
{
public:
    //=============================================================================
    // INITIALIZATION & CLEANUP
    //=============================================================================
    
    // Constructor: Initializes the hybrid ray tracer with the given parameters
    RayTracer(const int width, const int height,
        const float fovY, const Vector3& viewFrom, const Vector3& viewAt, const Vector3& lightOrigin,
        const char* config = "threads=0,verbose=3");

    // Destructor: Cleans up allocated resources (Embree, shapes, materials, cubemap)
    ~RayTracer();

    //=============================================================================
    // INTEL EMBREE MANAGEMENT (Surface Ray Tracing)
    //=============================================================================
    
    // Initializes the Intel Embree device and scene for triangle mesh ray tracing
    int InitDeviceAndScene(const char* config);

    // Releases the Intel Embree device and scene
    int ReleaseDeviceAndScene();

    //=============================================================================
    // SCENE LOADING & SETUP
    //=============================================================================
    
    // Loads a complete scene with polygon models, volumetric shapes, and environment map
    void LoadScene(
        const std::vector<ModelInfo>& models, // List of OBJ models for surface rendering
        const std::vector<Shape*>& shapes,    // List of SDF shapes for volumetric rendering
        const char* cubeMapFileNames[6]       // File paths for the cubemap textures
    );

    // Loads a single 3D model (OBJ) and applies the given transformation
    void LoadModel(const std::string& fileName, const Transform& transform = Transform());

    //=============================================================================
    // RAY TRACING UTILITIES
    //=============================================================================
    
    // Checks if a point is visible from a light source (shadow ray test)
    bool IsHitPointVisible(const Vector3& hitPoint, const Vector3& lightPoint);

    //=============================================================================
    // RENDERING PIPELINE
    //=============================================================================
    
    // Computes the color of a single pixel using hybrid rendering pipeline
    Color4f GetPixel(const int x, const int y, const float t = 0.0f) override;

    // Animates camera in circular orbit around target point
    void MoveCamera() override;

    // Main ray tracing function: Traces a ray through the scene using Intel Embree
    Vector3 TraceRay(const RTCRay& ray, const float n_1 = 1.0f, const int depth = 0, const int maxDepth = 10);

    //=============================================================================
    // SURFACE SHADING MODELS (for Embree-traced triangle meshes)
    //=============================================================================
    
    Vector3 NormalShader(const Vector3& normalVector); // Visualizes normals as RGB colors
    Vector3 LambertShader(const Material& material, const Coord2f& texCoord, const Vector3& hitPoint, const Vector3& normalVector); // Diffuse-only shading
    Vector3 PhongShader(const Material& material, const Coord2f& texCoord, const Vector3& hitPoint, const Vector3& normalVector, const Vector3& directionVector, const int depth); // Diffuse + specular highlights
    Vector3 TransparentShader(const RTCRay& ray, const Vector3& hitPoint, const Vector3& normalVector, const Vector3& directionVector, const Material& material, const float n_1, const int depth); // Glass with Fresnel reflections

    //=============================================================================
    // VOLUMETRIC RENDERING (SDF-based procedural shapes)
    //=============================================================================
    
    // Dispatcher: Chooses between surface (sphere tracing) and volume (ray marching) rendering
    Vector4 VolumetricRender(const RTCRay& ray);
    
    // RAY MARCHING: True volumetric rendering with Beer-Lambert absorption
    Vector4 VolumetricEffect(const RTCRay& ray);
    
    // SPHERE TRACING: Surface-only rendering of SDF shapes using adaptive stepping
    Vector4 SurfaceEffect(const RTCRay& ray);

    //=============================================================================
    // USER INTERFACE
    //=============================================================================
    
    // Renders the ImGui interface for controlling ray tracing parameters
    int Ui() override;

    //=============================================================================
    // RENDERING SETTINGS
    //=============================================================================

    // === SDF & NOISE PARAMETERS ===
    bool useNoise_{ true };               // Enable/disable procedural noise on SDF shapes
    float noiseScale_{ 0.7f };            // Spatial scale of the noise pattern
    float noiseStrength_{ 3.0f };         // Amplitude/strength of noise displacement
    float smoothFactor_{ 1.0f };          // Smoothness factor for SDF boolean operations

    // === VOLUMETRIC RENDERING PARAMETERS ===
    bool rayMarching_{ false };           // true = Volume mode, false = Surface mode
    float stepSize_{ 0.01f };             // Step size for ray marching (smaller = higher quality)
    float maxDistance_{ 50.0f };          // Maximum ray marching distance
    int maxSteps_ = static_cast<int>(maxDistance_ / stepSize_); // Maximum number of ray marching steps
    float absorptionCoefficient_{ 0.5f }; // Beer-Lambert absorption coefficient
    float lightAttenuationFactor_{ 1.65f }; // Distance-based light attenuation exponent
    
    // Volumetric material properties
    float volumetricAlbedo_[3]{ 0.0f, 0.0f, 0.0f };    // Albedo color picker for UI
    Vector3 volumetricAlbedoVec_{ 0.0f, 0.0f, 0.0f };  // Volumetric albedo as Vector3

    // === ANTI-ALIASING ===
    bool sampling_{ false };              // Enable/disable 4x4 supersampling anti-aliasing

    // === BACKGROUND SETTINGS ===
    bool backgroundEnabled_{ false };     // true = use cubemap, false = solid color
    float backgroundColor_[3]{ 1.0f, 1.0f, 1.0f };     // Background color picker for UI
    Vector3 backgroundColorVec_{ 1.0f, 1.0f, 1.0f };   // Background color as Vector3

    // === CAMERA ANIMATION ===
    bool cameraMovementEnabled_{ true };  // Enable/disable automatic camera orbit animation
    float cameraX_{ 0.0f };               // Manual camera X position
    float cameraY_{ 0.0f };               // Manual camera Y position
    float cameraZ_{ 0.0f };               // Manual camera Z position

private:
    //=============================================================================
    // SCENE DATA
    //=============================================================================
    
    // VOLUMETRIC SHAPES: Procedural SDF-based shapes for ray marching
    std::vector<Shape*> volumetricShapes_;
    
    // SURFACE GEOMETRY: Triangle meshes loaded from OBJ files for Embree
    std::vector<Surface*> surfaces_;       // Triangle mesh surfaces
    std::vector<Material*> materials_;     // Materials associated with surfaces
    
    // ENVIRONMENT MAPPING
    CubeMap* cubemap_ = nullptr;           // Environment map for background and reflections

    //=============================================================================
    // INTEL EMBREE RAY TRACING ENGINE
    //=============================================================================
    
    RTCDevice device_;                     // Embree device handle
    RTCScene scene_;                       // Embree scene containing BVH acceleration structure
    
    //=============================================================================
    // SCENE COMPONENTS
    //=============================================================================
    
    Camera camera_;                        // Pin-hole camera for ray generation
    Light light_;                          // Point light source for illumination
};

#endif