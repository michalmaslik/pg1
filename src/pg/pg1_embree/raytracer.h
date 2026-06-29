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
#include <openvkl/openvkl.h>
#include "vdb_loader.h"
#include <memory>
#include <shared_mutex>


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

// Stores the result of an extended surface ray query (color + hit distance)
struct SurfaceHit {
	Vector3 color;       // Shaded surface color (or background color on miss)
	float   tHit;        // Ray parameter t at hit point; FLT_MAX if no geometry was hit
	bool    hasHit;      // true if a triangle was intersected
};

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
	[[nodiscard]] bool IsHitPointVisible(const Vector3& hitPoint, const Vector3& lightPoint) const;

	//=============================================================================
	// RENDERING PIPELINE
	//=============================================================================

	// Computes the color of a single pixel using hybrid rendering pipeline
	Color4f GetPixel(const int x, const int y, const float t = 0.0f) override;

	// Animates camera in circular orbit around target point
	void MoveCamera() override;

	// Main ray tracing function: Traces a ray through the scene using Intel Embree
	Vector3 TraceRay(const RTCRay& ray, const float n_1 = 1.0f, const int depth = 0, const int maxDepth = 10);

	// Extended ray query: returns shaded color AND the Embree hit distance (tHit).
	// Used by COMBINED_SDF mode so the volume marcher can stop at the surface boundary.
	SurfaceHit TraceRayExtended(const RTCRay& ray, const float n_1 = 1.0f, const int depth = 0, const int maxDepth = 10);

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
	Vector4 VolumetricEffect(const RTCRay& ray, float tMax = FLT_MAX);

	// SPHERE TRACING: Surface-only rendering of SDF shapes using adaptive stepping
	Vector4 SurfaceEffect(const RTCRay& ray);

	//=============================================================================
	// USER INTERFACE
	//=============================================================================

	// Renders the ImGui interface for controlling ray tracing parameters
	int Ui() override;

	// NOTE: All rendering-parameter data members are in the private section below.
	// They are accessible from Ui() and all other member functions without restriction.

	//=============================================================================
	// VDB VOLUME MANAGEMENT
	//=============================================================================

	//! Initialize OpenVKL for VDB volume rendering
	bool InitializeOpenVKL();

	//! Load VDB file and set up volume for rendering
	bool LoadVdbVolume(const std::string& filename, const std::string& gridName = "density");

	//! Clean up OpenVKL resources
	void CleanupOpenVKL();

	//! VDB Volume ray marching implementation
	Vector4 VdbVolumeRayMarching(const RTCRay& ray);

	//! Check if VDB volume is loaded
	bool HasVdbVolume() const { return vklVolume_ != VKLVolume{}; }

	//! Get VDB volume bounding box
	void GetVdbVolumeBounds(Vector3& minBounds, Vector3& maxBounds) const;

	//! Get current FPS
	float GetCurrentFPS() const;

// RENDERING MODE MANAGEMENT
//=============================================================================

	enum class RenderingMode {
		SURFACE_EMBREE,     // Traditional triangle mesh rendering with Embree
		VOLUMETRIC_SDF,     // SDF-based volumetric shapes
		VOLUMETRIC_VDB,     // VDB volumes with OpenVKL
		COMBINED_SDF        // Deep composite: SDF volume in front of Embree surface
	};

	void SetRenderingMode(RenderingMode mode) { currentRenderingMode_ = mode; }
	RenderingMode GetRenderingMode() const { return currentRenderingMode_; }

	//=============================================================================
// SCENE MANAGEMENT
//=============================================================================

	enum class SceneType {
		SCENE_SHADERTOY_SDF,  // ShaderToy-style SDF volumetric scene with 3 animated coloured lights
		SCENE_CUSTOM_OBJ,     // Arbitrary OBJ file loaded at runtime, rendered with Embree
		SCENE_CUSTOM_VDB      // Arbitrary VDB file loaded at runtime, rendered with OpenVKL
	};

	// Load a predefined scene configuration
	void LoadPredefinedScene(SceneType type);

	// Update scene animation (called each frame)
	void UpdateScene(float time);

	// Main pixel rendering dispatch
	Vector4 RenderPixel(const RTCRay& ray);

	//=============================================================================
// DYNAMIC MODEL LOADING
//=============================================================================

// Load OBJ model dynamically during runtime
	bool LoadObjModel(const std::string& fileName);

	// Clear all loaded surface models
	void ClearSurfaceModels();

	// Get number of loaded models
	int GetLoadedModelCount() const { return static_cast<int>(surfaces_.size()); }


private:
	// =========================================================================
	// THREAD SYNCHRONISATION
	// =========================================================================

	/// Protects all scene assets (Embree scene, surfaces, materials, VKL volume)
	/// from concurrent access during reload operations.
	/// Rendering threads acquire a shared_lock; asset reload takes a unique_lock
	/// that waits for all in-flight renders to drain before proceeding.
	mutable std::shared_mutex sceneMutex_;

	// =========================================================================
	// INTEL EMBREE RAY TRACING ENGINE
	// =========================================================================

	RTCDevice device_;      ///< Embree device handle
	RTCScene  scene_;       ///< Active scene — rebuilt by ClearSurfaceModels()

	// =========================================================================
	// SCENE GEOMETRY & ASSETS  (smart-pointer ownership)
	// =========================================================================

	/// Owned triangle-mesh surfaces; raw Surface* passed to Embree via .get().
	std::vector<std::unique_ptr<Surface>>  surfaces_;

	/// Owned materials; raw Material* stored in Embree geometry user-data via .get().
	std::vector<std::unique_ptr<Material>> materials_;

	/// Non-owning observer pointers to volumetric SDF shapes.
	/// Actual ownership lives in fixedSdfScene_ (or future per-scene owners).
	std::vector<Shape*> volumetricShapes_;

	/// The procedural 3-sphere cloud, owned exclusively here.
	std::unique_ptr<SmoothUnion> fixedSdfScene_;

	/// Environment map for background and reflections.
	std::unique_ptr<CubeMap> cubemap_;

	// =========================================================================
	// OPENVKL VDB VOLUME RENDERING
	// =========================================================================

	VKLDevice                  vklDevice_{};  ///< OpenVKL device
	VKLVolume                  vklVolume_{};  ///< OpenVKL VDB volume
	VKLSampler                 vklSampler_{}; ///< OpenVKL sampler
	std::unique_ptr<VdbLoader> vdbLoader_;    ///< VDB file loader

	// =========================================================================
	// CAMERA & LIGHTS
	// =========================================================================

	Camera             camera_;  ///< Pin-hole camera for primary ray generation
	Light              light_;   ///< Legacy single-light (Lambert/Phong/SurfaceEffect)
	std::vector<Light> lights_;  ///< Multi-light list for volumetric illumination

	// =========================================================================
	// RENDERING PARAMETERS  (formerly public — still accessible from Ui() etc.)
	// =========================================================================

	// --- SDF & Noise ---
	bool  useNoise_{ true };       ///< Enable procedural fBm noise on SDF shapes
	float noiseScale_{ 0.7f };     ///< Spatial frequency of the noise pattern
	float noiseStrength_{ 3.0f };  ///< Amplitude of noise displacement
	float smoothFactor_{ 1.0f };   ///< Blending radius for smooth-union operations

	// --- Volumetric ray marching ---
	bool  rayMarching_{ true };               ///< true=volume march, false=sphere-trace surface
	float stepSize_{ 0.6f };                  ///< Primary march step Δt
	float maxDistance_{ 500.0f };             ///< Far-plane clamp for the march
	int   maxSteps_{ 256 };                   ///< Hard cap on march iterations
	float absorptionCoefficient_{ 0.5f };     ///< σ_t for Beer-Lambert extinction
	float lightAttenuationFactor_{ 1.65f };   ///< Exponent n in 1/d^n attenuation
	float lightAnimSpeed_{ 1.0f };            ///< Speed multiplier for orbiting lights
	float phaseG_{ 0.0f };                    ///< Henyey-Greenstein g (0=isotropic)
	float emissiveLightSphereRadius_{ 0.8f }; ///< Radius of the emissive light proxy sphere

	float   volumetricAlbedo_[3]{ 0.8f, 0.8f, 0.8f }; ///< Albedo for ImGui colour picker
	Vector3 volumetricAlbedoVec_{ 0.8f, 0.8f, 0.8f }; ///< Albedo as Vector3

	// --- Whitted surface tracing ---
	int   maxRecursionDepth_{ 5 };            ///< Max recursion depth for reflections/refractions
	float shadowBias_{ 0.001f };              ///< Ray-origin offset to prevent self-shadowing
	bool  enableShadows_{ true };             ///< Toggle shadow rays
	bool  materialOverride_{ false };         ///< Override all materials with global values
	float globalShininess_{ 32.0f };          ///< Global shininess override
	float globalAmbient_[3]{ 0.1f, 0.1f, 0.1f };  ///< Ambient colour for ImGui
	Vector3 globalAmbientVec_{ 0.1f, 0.1f, 0.1f }; ///< Global ambient as Vector3

	// --- Anti-aliasing ---
	bool sampling_{ false }; ///< Enable 4×4 supersampling MSAA

	// --- Debug ---
	bool showDebugAxes_{ false }; ///< Draw RGB XYZ axis lines from the world origin

	// --- Background ---
	bool    backgroundEnabled_{ false };                     ///< true=cubemap, false=solid colour
	float   backgroundColor_[3]{ 0.0f, 0.0f, 0.0f };       ///< Background colour for ImGui
	Vector3 backgroundColorVec_{ 0.0f, 0.0f, 0.0f };        ///< Background colour as Vector3

	// --- Camera animation ---
	bool cameraMovementEnabled_{ true }; ///< Enable automatic camera orbit
	bool mouseCameraInput_{ false };     ///< Enable mouse-driven orbit

	// =========================================================================
	// CAMERA ORBITAL CONTROL STATE
	// =========================================================================

	float cameraDistance_{ 20.0f };         ///< Orbital radius from viewAt
	float cameraAzimuth_{ 0.0f };           ///< Horizontal angle (degrees)
	float cameraElevation_{ 0.0f };         ///< Vertical angle (degrees)
	float cameraX_{ 0.0f };
	float cameraY_{ 0.0f };
	float cameraZ_{ 0.0f };
	float initialCameraDistance_{ 20.0f };
	float initialCameraAzimuth_{ 0.0f };
	float initialCameraElevation_{ 0.0f };
	Vector3 initialLightOrigin_;
	float minCameraDistance_{ 2.0f };       ///< Near zoom clamp
	float maxCameraDistance_{ 300.0f };     ///< Far zoom clamp
	float maxElevation_{ 89.0f };           ///< Elevation clamp (prevent gimbal lock)
	bool  leftMouseDragging_{ false };
	bool  rightMouseDragging_{ false };
	ImVec2 lastMousePos_{ 0.0f, 0.0f };

	// =========================================================================
	// SCENE MANAGEMENT STATE
	// =========================================================================

	RenderingMode currentRenderingMode_{ RenderingMode::SURFACE_EMBREE };
	SceneType     currentScene_{ SceneType::SCENE_SHADERTOY_SDF };
	float         sceneTime_{ 0.0f }; ///< Accumulated animation time (seconds)

	// =========================================================================
	// UI VISIBILITY FLAGS
	// =========================================================================

	bool showVdbControls_{ true };
	bool showRayMarchingControls_{ true };
	bool showCameraControls_{ true };
	bool showLightingControls_{ true };
	bool showRenderingControls_{ true };
	bool renderingPaused_{ false };
	bool singleFrameRender_{ false };
	bool autoRotateCamera_{ false };

	// VDB file path buffers for ImGui text inputs
	char vdbFilePath_[512]{ "C:\\Users\\micha\\Desktop\\file.vdb" };
	char vdbGridName_[64]{ "density" };

	// =========================================================================
	// PRIVATE HELPER METHODS
	// =========================================================================

	void InitializeFixedSdfScene();
	void UpdateCameraPosition();
	void HandleLeftMouseDrag(const ImVec2& mouseDelta);
	void HandleRightMouseDrag(const ImVec2& mouseDelta);

	/// Marches a secondary shadow ray through the SDF volume (Beer-Lambert).
	/// @returns Transmittance ∈ [0,1]; 1=fully lit, 0=fully shadowed.
	[[nodiscard]] float ComputeVolumeShadowTransmittance(
		const Vector3& samplePoint, const Vector3& lightPos) const;

	/// Evaluates the normalised Henyey-Greenstein phase function.
	/// @param cosTheta  cos(angle between incoming and outgoing directions).
	/// @param g         Asymmetry factor ∈ (-1,1); 0=isotropic.
	[[nodiscard]] static float EvaluateHenyeyGreenstein(float cosTheta, float g);
};

#endif
