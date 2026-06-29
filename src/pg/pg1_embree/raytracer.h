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
#include "VdbRenderer.h"
#include "SdfRenderer.h"
#include "PathTracer.h"
#include "RayTracerUI.h"
#include "SceneLoader.h"
#include <memory>
#include <shared_mutex>
#include <atomic>
#include <chrono>
#include "SceneManager.h"
#include "RenderTypes.h"


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



// Dopredna deklarace -- RayTracerUI je friend a implementuje Ui() panel
class RayTracerUI;

// Main ray tracer class, inheriting from SimpleGuiDX11 for GUI and rendering support
class RayTracer : public SimpleGuiDX11
{
	friend class RayTracerUI;
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
	int initDeviceAndScene(const char* config);

	// Releases the Intel Embree device and scene
	int releaseDeviceAndScene();

	//=============================================================================
	// SCENE LOADING & SETUP
	//=============================================================================

	// Loads a complete scene with polygon models, volumetric shapes, and environment map
	void loadScene(
		const std::vector<ModelInfo>& models, // List of OBJ models for surface rendering
		const std::vector<Shape*>& shapes,    // List of SDF shapes for volumetric rendering
		const char* cubeMapFileNames[6]       // File paths for the cubemap textures
	);

	// Loads a single 3D model (OBJ) and applies the given transformation
	void loadModel(const std::string& fileName, const Transform& transform = Transform());

	//=============================================================================
	// RAY TRACING UTILITIES
	//=============================================================================

	// Checks if a point is visible from a light source (shadow ray test)
	[[nodiscard]] bool isHitPointVisible(const Vector3& hitPoint, const Vector3& lightPoint) const;

	//=============================================================================
	// RENDERING PIPELINE
	//=============================================================================

	// Computes the color of a single pixel using hybrid rendering pipeline
	Color4f GetPixel(const int x, const int y, const float t = 0.0f) override;

	// Animates camera in circular orbit around target point
	void MoveCamera() override;

	// Main ray tracing function: Traces a ray through the scene using Intel Embree
	Vector3 traceRay(const RTCRay& ray, const float n_1 = 1.0f, const int depth = 0, const int maxDepth = 10);

	// Extended ray query: returns shaded color AND the Embree hit distance (tHit).
	// Used by COMBINED_SDF mode so the volume marcher can stop at the surface boundary.
	SurfaceHit traceRayExtended(const RTCRay& ray, const float n_1 = 1.0f, const int depth = 0, const int maxDepth = 10);

	// -----------------------------------------------------------------------
	// PATH TRACING PIPELINE  (does not touch Whitted traceRay / shader functions)
	// -----------------------------------------------------------------------

	/// Iterative Monte Carlo path tracer.  Accumulates one path sample.
	/// Call ptSamplesPerPixel_ times and average for a converged estimate.
	[[nodiscard]] Vector3 tracePath(const RTCRay& ray, int maxDepth) const;

	//=============================================================================
	// SURFACE SHADING MODELS (for Embree-traced triangle meshes)
	//=============================================================================

	Vector3 normalShader(const Vector3& normalVector); // Visualizes normals as RGB colors
	Vector3 lambertShader(const Material& material, const Coord2f& texCoord, const Vector3& hitPoint, const Vector3& normalVector); // Diffuse-only shading
	Vector3 phongShader(const Material& material, const Coord2f& texCoord, const Vector3& hitPoint, const Vector3& normalVector, const Vector3& directionVector, const int depth); // Diffuse + specular highlights
	Vector3 transparentShader(const RTCRay& ray, const Vector3& hitPoint, const Vector3& normalVector, const Vector3& directionVector, const Material& material, const float n_1, const int depth); // Glass with Fresnel reflections

	//=============================================================================
	// VOLUMETRIC RENDERING (SDF-based procedural shapes)
	//=============================================================================

	// Dispatcher: Chooses between surface (sphere tracing) and volume (ray marching) rendering
	Vector4 volumetricRender(const RTCRay& ray);

	// RAY MARCHING: True volumetric rendering with Beer-Lambert absorption
	Vector4 volumetricEffect(const RTCRay& ray, float tMax = FLT_MAX);

	// SPHERE TRACING: Surface-only rendering of SDF shapes using adaptive stepping
	Vector4 surfaceEffect(const RTCRay& ray);

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
	bool initializeOpenVKL();

	//! Load VDB file and set up volume for rendering
	bool loadVdbVolume(const std::string& filename, const std::string& gridName = "density");

	//! Clean up OpenVKL resources
	void cleanupOpenVKL();

	//! VDB Volume ray marching implementation
	Vector4 vdbVolumeRayMarching(const RTCRay& ray, bool compositeBg = true);

	//! Check if VDB volume is loaded (delegates to VdbRenderer)
	[[nodiscard]] bool hasVdbVolume() const;

	//! Get VDB volume bounding box
	void getVdbVolumeBounds(Vector3& minBounds, Vector3& maxBounds) const;

	//! Get current FPS
	float getCurrentFPS() const;


	void setRenderingMode(RenderingMode mode) { currentRenderingMode_ = mode; }
	RenderingMode getRenderingMode() const { return currentRenderingMode_; }

	//=============================================================================
// SCENE MANAGEMENT
//=============================================================================


	// Load a predefined scene configuration
	void loadPredefinedScene(SceneType type);

	// Update scene animation (called each frame)
	void updateScene(float time);

	// Main pixel rendering dispatch
	Vector4 renderPixel(const RTCRay& ray);

	//=============================================================================
// DYNAMIC MODEL LOADING
//=============================================================================

// Load OBJ model dynamically during runtime.
	// If animOut is non-null the geometry uses shared vertex buffers so the
	// positions can be updated each frame; state is written into *animOut.
	bool loadObjModel(const std::string& fileName, EntityAnimState* animOut = nullptr);

	// Clear all loaded surface models
	void clearSurfaceModels();

	// Get number of loaded models
	int getLoadedModelCount() const { return static_cast<int>(surfaces_.size()); }


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
	RTCScene  scene_;       ///< Active scene — rebuilt by clearSurfaceModels()

	// =========================================================================
	// SCENE GEOMETRY & ASSETS  (smart-pointer ownership)
	// =========================================================================

	/// Owned triangle-mesh surfaces; raw Surface* passed to Embree via .get().
	std::vector<std::unique_ptr<Surface>>  surfaces_;

	/// Owned materials; raw Material* stored in Embree geometry user-data via .get().
	std::vector<std::unique_ptr<Material>> materials_;

	/// Uniquely-owned texture objects loaded from OBJ material files.
	/// Multiple Material slots may share the same Texture* (TextureProxy caches);
	/// storing them here allows a single safe delete rather than per-material
	/// deletion that would cause double-free crashes.
	std::vector<Texture*> ownedTextures_;

	/// Non-owning observer pointers to volumetric SDF shapes.
	/// Actual ownership lives in fixedSdfScene_ (or future per-scene owners).
	std::vector<Shape*> volumetricShapes_;

	/// The procedural 3-sphere cloud, owned exclusively here.
	std::unique_ptr<SmoothUnion> fixedSdfScene_;

	/// Environment map for background and reflections.
	std::unique_ptr<CubeMap> cubemap_;

	// =========================================================================
	// VDB RENDERER (OpenVKL subsystem � zapouzdreno v samostatne tride)
	// =========================================================================

	/// Vlastni cely OpenVKL subsystem (device, volume, sampler, loader).
	std::unique_ptr<VdbRenderer> vdbRenderer_;

	/// Bezstavova trida pro SDF volumetricke krochlovani a sphere-tracing.
	std::unique_ptr<SdfRenderer> sdfRenderer_;

	/// Bezstavova trida pro Monte Carlo sledovani cest.
	std::unique_ptr<PathTracer> pathTracer_;

	/// ImGui UI kontroler -- implementuje Ui() panel pres friend pristup.
	std::unique_ptr<RayTracerUI> uiController_;

	/// Spravce stavu sceny -- animace, popis scen, svetla.
	std::unique_ptr<SceneManager> sceneManager_;

	// =========================================================================
	// CAMERA & LIGHTS
	// =========================================================================

	Camera             camera_;  ///< Pin-hole camera for primary ray generation
	Light              light_;   ///< Legacy single-light (Lambert/Phong/surfaceEffect)
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

	// --- Path tracing ---
	int   ptMaxDepth_{ 8 };        ///< Max path length before hard termination
	int   ptSamplesPerPixel_{ 4 }; ///< Samples accumulated per renderPixel call
	int   ptRRMinDepth_{ 3 };      ///< Bounce depth at which Russian Roulette begins

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

	RenderingMode      currentRenderingMode_{ RenderingMode::SURFACE_EMBREE };
	SceneType          currentScene_{ SceneType::SCENE_SHADERTOY_SDF };
	float              sceneTime_{ 0.0f }; ///< Casovac animace sceny (sekund)
	/// High-level render filter — set by the UI, consumed by resolveActiveMode().
	/// Cached into currentRenderingMode_ once per frame in MoveCamera().
	GlobalRenderFilter currentFilter_{ GlobalRenderFilter::COMBINED };

	/// If true, resolveActiveMode() selects a path-tracing pipeline for the mesh.
	bool objUsePathTracing_{ false };

	// =========================================================================
	// GLOBAL SUN — UI-controlled state (main thread only)
	// =========================================================================

	/// Toggle in the "Environment" UI panel.
	bool    enableGlobalSun_{ false };

	/// Raw (possibly un-normalised) direction toward the sun, from surface to light.
	/// Stored as float[3] for direct use with ImGui::SliderFloat3.
	float   sunDirUI_[3]{ 0.5f, 1.0f, 0.5f };

	/// Colour tint of the sun.
	Vector3 sunColor_{ 1.0f, 0.9f, 0.8f };

	/// Scalar intensity multiplier (no 1/d² attenuation — directional light).
	float   sunIntensity_{ 5.0f };

	// -------------------------------------------------------------------------
	// Frame-stable copies set in MoveCamera() BEFORE the OMP pixel dispatch.
	// Render threads read ONLY these; the UI vars above are main-thread-only.
	// -------------------------------------------------------------------------
	bool    frameSunEnabled_{ false };
	Vector3 frameSunDir_{ 0.0f };      ///< Normalised, pointing TOWARD the sun
	Vector3 frameSunColor_{ 0.0f };
	float   frameSunIntensity_{ 0.0f };

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

	// VDB file path buffers for ImGui text inputs (legacy path, kept for internal use)
	char vdbFilePath_[512]{ "C:\\Users\\micha\\Desktop\\file.vdb" };
	char vdbGridName_[64]{ "density" };

	// =========================================================================
	// SCENE CONFIGURATION LOADER  (delegovano na SceneManager)
	// =========================================================================

	// =========================================================================
	// RENDER PROGRESS TRACKING
	// =========================================================================

	/// Number of fully rendered rows in the current frame.
	/// Incremented once per row from within OMP render threads.
	std::atomic<int> completedRows_{0};

	/// Total rows in the current frame (set to height() before each frame).
	int totalRows_{1};

	/// Wall-clock time when the current frame's render loop began.
	std::chrono::time_point<std::chrono::high_resolution_clock> renderStartTime_;

	/// True while the OMP pixel loop is executing for the current frame.
	std::atomic<bool> isRendering_{false};

	/// Duration (seconds) of the most recently completed frame.
	/// Written by the last OMP thread to finish, read by the UI thread.
	float lastFrameDuration_{0.0f};

	/// Exponential-moving-average of the raw ETA, updated each UI frame.
	/// Seeded to -1 so the first valid sample initialises it directly (no warm-up lag).
	float smoothedEta_{-1.0f};

	/// Translation offsets applied each frame by updateEntityTransforms().
	/// Ray-march code subtracts these offsets from sample positions to simulate
	/// the SDF/VDB volume being displaced from the world origin.
	Vector3 sdfAnimOffset_{0.0f, 0.0f, 0.0f};
	Vector3 vdbAnimOffset_{0.0f, 0.0f, 0.0f};

	// =========================================================================
	// PRIVATE HELPER METHODS
	// =========================================================================

	void initializeFixedSdfScene();
	void updateCameraPosition();

	/// Recomputes and uploads animated entity transforms for the current frame.
	/// For MESH entities: re-fills shared Embree vertex buffers and re-commits.
	/// For SDF/VDB entities: updates sdfAnimOffset_ / vdbAnimOffset_ offsets.
	/// Called from updateScene() on the main thread between frames.
	void updateEntityTransforms(float time);
	void handleLeftMouseDrag(const ImVec2& mouseDelta);
	void handleRightMouseDrag(const ImVec2& mouseDelta);

	/// Tears down ALL scene assets atomically under a unique_lock, ensuring
	/// every in-flight GetPixel() call (which holds a shared_lock) has
	/// completed before any pointer/handle is released.  Covers:
	///   - Embree RTCScene + Surface / Material smart-pointer vectors
	///   - OpenVKL sampler + volume (device is preserved for reuse)
	///   - VdbLoader (clears stale grid / bounding-box caches)
	///   - Volumetric SDF observer list (fixedSdfScene_ ownership untouched)
	/// Do NOT hold sceneMutex_ before calling this function.
	void clearScene();

	/// Clears current geometry/volume, configures lights, loads assets described
	/// by @p desc, and sets the rendering mode specified by the scene entry.
	void loadSceneFromDescription(const SceneDescription& desc);

	/// Sestavi SdfRenderContext z aktualniho (frame-stabilniho) stavu rendereru.
	[[nodiscard]] SdfRenderContext buildSdfContext() const;

	/// Sestavi VdbRenderContext z aktualniho stavu RayTraceru (volano pred rayMarch).
	[[nodiscard]] VdbRenderContext buildVdbContext() const;

	/// Sestavi PathTracingContext z aktualniho stavu rendereru.
	[[nodiscard]] PathTracingContext buildPathTracingContext() const;

	/// Maps a mode token string to the RenderingMode enum (kept for legacy/debug use).
	/// Defaults to SURFACE_EMBREE for unrecognised strings.
	RenderingMode modeFromString(const std::string& modeStr) const;

	/// Derives the active RenderingMode from currentFilter_ and objUsePathTracing_,
	/// taking into account which assets are actually loaded.
	/// Called once per frame from MoveCamera() to update currentRenderingMode_.
	RenderingMode resolveActiveMode() const;

	/// Marches a secondary shadow ray through the SDF volume (Beer-Lambert).
	/// @returns Transmittance ∈ [0,1]; 1=fully lit, 0=fully shadowed.
	[[nodiscard]] float ComputeVolumeShadowTransmittance(
		const Vector3& samplePoint, const Vector3& lightPos) const;

	/// Evaluates the normalised Henyey-Greenstein phase function.
	[[nodiscard]] static float evaluateHenyeyGreenstein(float cosTheta, float g);

	// --- Path tracing helpers ---

	/// Balanced MIS heuristic (Veach 1997, n_i = 1, n = 2):
	///   w_a = p_a / (p_a + p_b)
	/// For delta (point) lights p_b→0, so w_a = 1 (reduces to pure NEE).
	[[nodiscard]] static float balancedHeuristic(float p_a, float p_b);

	/// Builds a right-handed orthonormal basis (Duff et al. 2017).
	/// @param n   Unit surface normal (the "up" axis of the basis).
	/// @param t   Output tangent vector.
	/// @param b   Output bitangent vector.
	static void buildONB(const Vector3& n, Vector3& t, Vector3& b);

	/// Generates a cosine-weighted direction on the hemisphere around @p normal.
	/// PDF = cos(θ)/π.  With a Lambertian BRDF the weight reduces to albedo.
	[[nodiscard]] static Vector3 sampleHemisphereCosine(const Vector3& normal);

	/// Evaluates direct illumination at a surface point via Next Event Estimation.
	/// Iterates over all scene lights, casts shadow rays, and accumulates
	/// the Lambert BRDF-weighted contribution.  No PDF division needed for
	/// point/directional lights (delta distributions).
	[[nodiscard]] Vector3 sampleDirectLightPT(
		const Vector3& hitPoint,
		const Vector3& normal,
		const Vector3& albedo) const;
};

#endif
