

#include "stdafx.h"
#include "raytracer.h"
#include "objloader.h"
#include "tutorials.h"
#include "utils.h"
#include "ShadingUtils.h"
#include "VdbRenderer.h"
#include "RayTracerUI.h"
#include <iostream>
#include "smooth_union.h"
#include <opencv2/opencv.hpp>
#include <openvkl/devices/cpu/openvkl/device/openvkl.h>

//=============================================================================
// CONSTRUCTOR & DESTRUCTOR
//=============================================================================

RayTracer::RayTracer(const int width, const int height,
	const float fovY, const Vector3& viewFrom, const Vector3& viewAt, const Vector3& lightOrigin,
	const char* config) : SimpleGuiDX11(width, height)
{
	std::cout << "[RAY TRACER] Initializing Ray Tracer..." << std::endl;

	// Initialize Intel Embree device and scene for surface ray tracing
	if (InitDeviceAndScene(config) != S_OK) {
		throw std::runtime_error("Failed to initialize Embree device and scene");
	}

	// Setup camera with specified parameters
	camera_ = Camera(width, height, fovY, viewFrom, viewAt);

	// Setup light source
	light_ = Light(lightOrigin);

	// Initialize fixed SDF scene
	InitializeFixedSdfScene();

	// Initialize camera orbital parameters (Y-up: azimuth in XZ plane, Y = elevation)
	Vector3 currentViewFrom = camera_.GetViewFrom();

	Vector3 direction = currentViewFrom - viewAt;
	cameraDistance_ = direction.L2Norm();
	cameraAzimuth_   = rad2deg(atan2(direction.z, direction.x)); // angle in XZ plane
	cameraElevation_ = rad2deg(asin(direction.y / cameraDistance_)); // Y is vertical

	while (cameraAzimuth_ < 0.0f) cameraAzimuth_ += 360.0f;

	cameraX_ = currentViewFrom.x;
	cameraY_ = currentViewFrom.y;
	cameraZ_ = currentViewFrom.z;

	// Vytvor VdbRenderer a inicializuj OpenVKL
	vdbRenderer_ = std::make_unique<VdbRenderer>();
	// Vytvor SdfRenderer (bezstavovy, okamzite pripraveny)
	sdfRenderer_ = std::make_unique<SdfRenderer>();
	// Vytvor PathTracer (bezstavovy, okamzite pripraveny)
	pathTracer_ = std::make_unique<PathTracer>();
	sceneManager_  = std::make_unique<SceneManager>();
	uiController_  = std::make_unique<RayTracerUI>(*this);
	if (!InitializeOpenVKL()) {
		std::cerr << "[RAY TRACER WARNING] Failed to initialize OpenVKL" << std::endl;
	}

	// Load default cubemap (ak existujÃº sÃºbory)
	const char* cubeMapFileNames[6] = {
		"../../../data/cube_map/posx.jpg",
		"../../../data/cube_map/posy.jpg",
		"../../../data/cube_map/posz.jpg",
		"../../../data/cube_map/negx.jpg",
		"../../../data/cube_map/negy.jpg",
		"../../../data/cube_map/negz.jpg"
	};

	// Try to load cubemap, but don't fail if files don't exist
	try {
		cubemap_ = std::make_unique<CubeMap>(cubeMapFileNames);
		std::cout << "[RAY TRACER] Cubemap loaded successfully" << std::endl;
	}
	catch (const std::exception& e) {
		std::cerr << "[RAY TRACER WARNING] Failed to load cubemap: " << e.what() << std::endl;
		cubemap_.reset();
		backgroundEnabled_ = false; // Disable background if cubemap failed to load
	}

	// Seed the default scene so lights_ and camera are correct from frame 1.
	LoadPredefinedScene(SceneType::SCENE_SHADERTOY_SDF);

	// Load scene list from scenes.scn.  Try project-root relative path first
	// (working directory is pg1_embree/, so 3 levels up reaches pg1/).
	// Fall back to current directory for alternative working-directory setups.
	// Nacti seznam scen pres SceneManager
	if (sceneManager_->loadScenesFromFile("../../../scenes.scn") == 0)
		sceneManager_->loadScenesFromFile("scenes.scn");
	sceneManager_->setSelectedIndex(0);

	// Auto-nacti vychozi scenu; preferuj tu s nazvem obsahujicim 'ShaderToy'
	if (sceneManager_->hasScenes()) {
		const auto& scenes = sceneManager_->getScenes();
		for (int i = 0; i < static_cast<int>(scenes.size()); ++i) {
			if (scenes[i].name.find("ShaderToy") != std::string::npos) {
				sceneManager_->setSelectedIndex(i);
				break;
			}
		}
		LoadSceneFromDescription(sceneManager_->getSelectedScene());
	}

	std::cout << "[RAY TRACER] Ray Tracer initialized successfully" << std::endl;
}

RayTracer::~RayTracer()
{
	// Cleanup OpenVKL resources first
	CleanupOpenVKL();

	// fixedSdfScene_ and cubemap_ are unique_ptr â€” destroyed automatically at scope exit.
	volumetricShapes_.clear();  // non-owning observer list

	// ClearSurfaceModels releases scene_ and clears surfaces_/materials_ (unique_ptr auto-delete)
	ClearSurfaceModels();

	// Release the fresh empty scene created by ClearSurfaceModels, then the device
	rtcReleaseScene(scene_);
	rtcReleaseDevice(device_);
}

//===========================================================================

//=============================================================================
// HLAVNI RENDEROVACI PIPELINE
//=============================================================================

Color4f RayTracer::GetPixel(const int x, const int y, const float t)
{
	// --- Progress tracking: frame start (single-frame-while-paused only) ---
	// MoveCamera() already resets the counters for every normal (unpaused) frame
	// before the OMP loop starts.  This guard fires only for "RENDER FRAME"
	// requests made while paused, where MoveCamera is skipped.  The condition
	// !isRendering_ ensures we do NOT double-reset when MoveCamera already ran
	// (which would zero out rows that OMP threads have already counted).
	if (x == 0 && y == 0 && !isRendering_.load(std::memory_order_relaxed)) {
		completedRows_.store(0, std::memory_order_relaxed);
		totalRows_ = height();
		renderStartTime_ = std::chrono::high_resolution_clock::now();
		isRendering_.store(true, std::memory_order_relaxed);
	}

	// Shared lock: many render threads may hold this simultaneously.
	// A unique_lock in LoadObjModel/LoadVdbVolume blocks until all renders finish.
	std::shared_lock<std::shared_mutex> renderLock(sceneMutex_);

	Vector3 finalColor(0.0f);
	float finalAlpha = 1.0f;

	if (sampling_) {
		// SUPERSAMPLING: Average multiple rays per pixel for anti-aliasing
		Vector3 accumulator(0.0f);
		float alphaAccumulator = 0.0f;

		for (int j = 0; j < 4; j++) {
			for (int i = 0; i < 4; i++) {
				float ksi_x = Random() / 4; // Random jitter for anti-aliasing
				float ksi_y = Random() / 4;

				// Generate ray for sub-pixel sample
				RTCRay ray = camera_.GenerateRay(float(x) + i * (1.0f / 4.0f) + ksi_x,
					float(y) + j * (1.0f / 4.0f) + ksi_y);

				// Render based on current mode
				Vector4 result = RenderPixel(ray);
				accumulator += Vector3(result.x, result.y, result.z);
				alphaAccumulator += result.w;
			}
		}

		accumulator /= 16.0f; // Average accumulated color (4x4 = 16 samples)
		alphaAccumulator /= 16.0f;
		finalColor = accumulator;
		finalAlpha = alphaAccumulator;
	}
	else {
		// SINGLE SAMPLE: Trace one ray per pixel
		RTCRay ray = camera_.GenerateRay(float(x), float(y));
		Vector4 result = RenderPixel(ray);
		finalColor = Vector3(result.x, result.y, result.z);
		finalAlpha = result.w;
	}

	// --- Progress tracking: row completion --------------------------------
	// Fires once per row after all pixels in that row have been computed.
	// Using relaxed ordering is safe: we only need approximate progress values
	// for the UI, and the final store to isRendering_ is visible to the UI
	// within at most one display frame (16 ms).
	if (x == width() - 1) {
		const int done = completedRows_.fetch_add(1, std::memory_order_relaxed) + 1;
		if (done >= totalRows_) {
			const auto now = std::chrono::high_resolution_clock::now();
			lastFrameDuration_ =
				std::chrono::duration<float>(now - renderStartTime_).count();
			isRendering_.store(false, std::memory_order_relaxed);
		}
	}

	// Return final color compressed to [0,1] range
	return Color4f{ finalColor.x, finalColor.y, finalColor.z, finalAlpha };
}

Vector4 RayTracer::RenderPixel(const RTCRay& ray) {
	const Vector3 ro(ray.org_x, ray.org_y, ray.org_z);
	const Vector3 rd(ray.dir_x, ray.dir_y, ray.dir_z);

	// --- DEBUG AXES -------------------------------------------------------
	// Analytic ray-to-line distance for each world axis (X=red, Y=green, Z=blue).
	// Uses the Gram-Schmidt closest-point formula.  Axes are finite segments
	// of half-length 20 so they don't extend to infinity.
	if (showDebugAxes_) {
		constexpr float kAxisRadius = 0.12f;
		constexpr float kAxisLen    = 500.0f;
		const Vector3 axesDirs[3]   = { Vector3(1,0,0), Vector3(0,1,0), Vector3(0,0,1) };
		const Vector3 axesColors[3] = { Vector3(1,0,0), Vector3(0,1,0), Vector3(0,0,1) };

		float  bestT  = FLT_MAX;
		int    bestAx = -1;

		for (int i = 0; i < 3; ++i) {
			const Vector3& D = axesDirs[i];
			const float b    = rd.DotProduct(D);          // cosine between ray and axis
			const float denom = 1.0f - b * b;             // sinT-(angle)
			if (denom < 1e-6f) continue;                  // ray parallel to axis

			const float d    = rd.DotProduct(ro);
			const float e    = D.DotProduct(ro);
			const float tRay = (b * e - d) / denom;       // t on the ray
			const float sAx  = (e - b * d) / denom;       // s on the axis line

			if (tRay < 0.001f || fabsf(sAx) > kAxisLen) continue;

			const Vector3 closest = (ro + rd * tRay) - (D * sAx);
			if (closest.L2Norm() < kAxisRadius && tRay < bestT) {
				bestT  = tRay;
				bestAx = i;
			}
		}
		if (bestAx >= 0)
			return Vector4(axesColors[bestAx].x, axesColors[bestAx].y, axesColors[bestAx].z, 1.0f);
	}

	// --- EMISSIVE SPHERE INTERSECTION (VOLUMETRIC_SDF only) ---------------
	// Collect the nearest emissive sphere hit distance (tEmissive) and its
	// colour.  We do NOT early-return here; instead tEmissive is passed as
	// tMax to VolumetricEffect so the cloud march stops at the sphere and
	// correctly attenuates spheres that sit behind the volume.
	float   tEmissive    = FLT_MAX;
	Vector3 emissiveCol(0.0f);
	if (currentRenderingMode_ == RenderingMode::VOLUMETRIC_SDF) {
		for (const Light& light : lights_) {
			if (light.type != LightType::Point) continue;
			const Vector3 oc   = ro - light.position;
			const float   b    = oc.DotProduct(rd);
			const float   c    = oc.DotProduct(oc)
			                     - emissiveLightSphereRadius_ * emissiveLightSphereRadius_;
			const float   disc = b * b - c;
			if (disc < 0.0f) continue;
			const float tHit = -b - sqrtf(disc);
			if (tHit > 0.001f && tHit < tEmissive) {
				tEmissive = tHit;
				const float s = (light.intensity > 0.0f) ? 1.0f / light.intensity : 1.0f;
				emissiveCol = Vector3(std::min(light.color.x * s, 1.0f),
				                     std::min(light.color.y * s, 1.0f),
				                     std::min(light.color.z * s, 1.0f));
			}
		}
	}

	// --- BACKGROUND HELPER ------------------------------------------------
	auto getBg = [&]() -> Vector3 {
		if (backgroundEnabled_ && cubemap_) {
			const Color3f c = cubemap_->GetTexel(rd);
			return Vector3(c.r, c.g, c.b);
		}
		return backgroundColorVec_;
	};

	// --- RENDER DISPATCH --------------------------------------------------
	switch (currentRenderingMode_) {
	case RenderingMode::SURFACE_EMBREE: {
		const Vector3 sc = TraceRay(ray);
		return Vector4(sc.x, sc.y, sc.z, 1.0f);
	}
	case RenderingMode::VOLUMETRIC_SDF: {
		// March the volume only up to the nearest emissive sphere (tEmissive).
		// Depth compositing:
		//   - Sphere in front of cloud  Ôæå march exits early (low vol.w), sphere at full brightness
		//   - Sphere behind cloud       Ôæå vol.w is large, sphere attenuated by remaining transmittance
		//   - No sphere hit             Ôæå tEmissive == FLT_MAX, behaves like ordinary background
		const Vector4 vol = VolumetricEffect(ray, tEmissive);
		const Vector3 bg  = (tEmissive < FLT_MAX) ? emissiveCol : getBg();
		const Vector3 out = Vector3(vol.x, vol.y, vol.z) + bg * (1.0f - vol.w);
		return Vector4(out.x, out.y, out.z, 1.0f);
	}
	case RenderingMode::VOLUMETRIC_VDB: {
		return VdbVolumeRayMarching(ray);
	}
	case RenderingMode::COMBINED_SDF: {
		const SurfaceHit surf  = TraceRayExtended(ray);
		const float      tSurf = surf.hasHit ? surf.tHit : FLT_MAX;
		const Vector4    vol   = VolumetricEffect(ray, tSurf);
		const Vector3    out   = Vector3(vol.x, vol.y, vol.z) + surf.color * (1.0f - vol.w);
		return Vector4(out.x, out.y, out.z, 1.0f);
	}
	case RenderingMode::PATH_TRACING_EMBREE: {
		// Accumulate ptSamplesPerPixel_ independent path samples and average.
		// More samples Ôæå lower variance (converges as O(1/Ô³ÜN)).
		Vector3 accumulated(0.0f);
		for (int s = 0; s < ptSamplesPerPixel_; ++s)
			accumulated += TracePath(ray, ptMaxDepth_);
		accumulated /= static_cast<float>(ptSamplesPerPixel_);
		return Vector4(accumulated.x, accumulated.y, accumulated.z, 1.0f);
	}
	case RenderingMode::COMBINED_PT_SDF: {
		// Path-traced surface composited under (or over) the SDF volumetric cloud.
		// The surface is first queried for its hit depth so the volume marcher
		// clips correctly at the opaque boundary (same portal-Duff pattern as
		// COMBINED_SDF, but surface radiance comes from full PT instead of Whitted).
		const SurfaceHit surf  = TraceRayExtended(ray);
		const float      tSurf = surf.hasHit ? surf.tHit : FLT_MAX;
		const Vector4    vol   = VolumetricEffect(ray, tSurf);
		// Replace Whitted surface colour with Monte Carlo PT estimate
		Vector3 ptSurf(0.0f);
		for (int s = 0; s < ptSamplesPerPixel_; ++s)
			ptSurf += TracePath(ray, ptMaxDepth_);
		ptSurf /= static_cast<float>(ptSamplesPerPixel_);
		// Porter-Duff over: vol_rgb + pt_surface * T_volume
		const Vector3 out = Vector3(vol.x, vol.y, vol.z) + ptSurf * (1.0f - vol.w);
		return Vector4(out.x, out.y, out.z, 1.0f);
	}
	case RenderingMode::COMBINED_PT_VDB: {
		// Path-traced surface composited beneath a VDB smoke volume.
		// VdbVolumeRayMarching(compositeBg=false) returns raw (rgb, opacity) so
		// we can Porter-Duff the PT surface into the transparent regions.
		const Vector4 vdb = VdbVolumeRayMarching(ray, false);
		Vector3 ptSurf(0.0f);
		for (int s = 0; s < ptSamplesPerPixel_; ++s)
			ptSurf += TracePath(ray, ptMaxDepth_);
		ptSurf /= static_cast<float>(ptSamplesPerPixel_);
		// Porter-Duff over: vdb in front, pt surface fills transparent regions
		const Vector3 out = Vector3(vdb.x, vdb.y, vdb.z) + ptSurf * (1.0f - vdb.w);
		return Vector4(out.x, out.y, out.z, 1.0f);
	}
	default: {
		const Vector3 bg = getBg();
		return Vector4(bg.x, bg.y, bg.z, 1.0f);
	}
	}
}

//=============================================================================
// CAMERA ANIMATION

//=============================================================================
// POHYB KAMERY A UI DELEGACE
//=============================================================================

//=============================================================================

void RayTracer::MoveCamera()
{
	if (cameraMovementEnabled_) {
		static float angle = 0.0f;
		const float speed = 0.025f;

		Vector3 viewAt = camera_.GetViewAt();
		Vector3 viewFrom = camera_.GetViewFrom();

		// Y-up auto-orbit: rotate in the XZ plane around the Y axis.
		// The orbital radius is the horizontal (XZ) distance from the target.
		Vector3 direction = viewFrom - viewAt;
		float currentRadius = sqrt(direction.x * direction.x + direction.z * direction.z);

		cameraX_ = viewAt.x + currentRadius * cosf(angle);
		cameraZ_ = viewAt.z + currentRadius * sinf(angle);
		cameraY_ = viewFrom.y; // Maintain Y height (elevation unchanged)

		// Increment angle for next frame
		angle += speed;
		if (angle > 2.0f * (float)M_PI) {
			angle -= 2.0f * (float)M_PI;
		}

		// Update camera position
		camera_.SetViewFrom(Vector3(cameraX_, cameraY_, cameraZ_));
	}

	// Always advance scene time so lights orbit every frame,
	// independent of whether the camera auto-rotation is enabled.
	sceneTime_ += 1.0f / 60.0f;
	UpdateScene(sceneTime_);

	// Resolve and cache the active rendering mode from the UI filter state.
	// This runs on the main thread before the OMP pixel loop, so render threads
	// see a stable currentRenderingMode_ for the entire frame.
	currentRenderingMode_ = ResolveActiveMode();

	// Cache Global Sun state into frame-stable copies.
	// UI variables (enableGlobalSun_, sunDirUI_, sunColor_, sunIntensity_) are
	// written by the ImGui main thread; render threads read ONLY the frame* copies.
	frameSunEnabled_   = enableGlobalSun_;
	if (frameSunEnabled_) {
		frameSunDir_       = Vector3(sunDirUI_[0], sunDirUI_[1], sunDirUI_[2]);
		const float len    = frameSunDir_.L2Norm();
		if (len > 1e-4f) frameSunDir_ /= len; else frameSunDir_ = Vector3(0.0f, 1.0f, 0.0f);
		frameSunColor_     = sunColor_;
		frameSunIntensity_ = sunIntensity_;
	}

	// Reset progress counters here (main thread, before OMP dispatch) so the
	// ETA calculation uses the most accurate possible render-start timestamp.
	// GetPixel(0,0) performs the same reset as a fallback for single-frame
	// requests made while paused (where MoveCamera is not called).
	completedRows_.store(0, std::memory_order_relaxed);
	totalRows_ = height();
	renderStartTime_ = std::chrono::high_resolution_clock::now();
	isRendering_.store(true, std::memory_order_relaxed);
}

//=============================================================================
// USER INTERFACE
//=============================================================================

int RayTracer::Ui()
{
{
	return uiController_->build();
}

void RayTracer::UpdateCameraPosition() {
void RayTracer::UpdateCameraPosition() {
	// Y-up spherical coordinates:
	//   azimuth  = angle in XZ plane (horizontal orbit around Y axis)
	//   elevation = angle from XZ plane toward +Y (vertical)
	//   x = dist * cos(el) * cos(az)
	//   y = dist * sin(el)              ÔæÉ Y is the vertical component
	//   z = dist * cos(el) * sin(az)
	float azimuthRad   = deg2rad(cameraAzimuth_);
	float elevationRad = deg2rad(cameraElevation_);

	Vector3 viewAt(0.0f, 0.0f, 0.0f);

	float x = cameraDistance_ * cos(elevationRad) * cos(azimuthRad);
	float y = cameraDistance_ * sin(elevationRad);            // Y up
	float z = cameraDistance_ * cos(elevationRad) * sin(azimuthRad);

	Vector3 newViewFrom(x, y, z);

	// Update camera
	camera_.SetViewAt(viewAt);
	camera_.SetViewFrom(newViewFrom);

	// Update UI values
	cameraX_ = newViewFrom.x;
	cameraY_ = newViewFrom.y;
	cameraZ_ = newViewFrom.z;
}

void RayTracer::HandleLeftMouseDrag(const ImVec2& mouseDelta) {
	// Left mouse: Orbital rotation around viewAt
	const float rotationSpeed = 0.5f; // degrees per pixel

	cameraAzimuth_ += mouseDelta.x * rotationSpeed;
	cameraElevation_ += mouseDelta.y * rotationSpeed;

	// Normalize azimuth to [0, 360)
	while (cameraAzimuth_ < 0.0f) cameraAzimuth_ += 360.0f;
	while (cameraAzimuth_ >= 360.0f) cameraAzimuth_ -= 360.0f;

	// Clamp elevation to prevent gimbal lock
	cameraElevation_ = std::max(-maxElevation_, std::min(maxElevation_, cameraElevation_));

	UpdateCameraPosition();
}

void RayTracer::HandleRightMouseDrag(const ImVec2& mouseDelta) {
	// Right mouse: Zoom in/out
	// Up/Right = zoom in (decrease distance)
	// Down/Left = zoom out (increase distance)
	const float zoomSpeed = 0.1f;

	float zoomDelta = -(mouseDelta.x + mouseDelta.y) * zoomSpeed;
	cameraDistance_ += zoomDelta;

	// Clamp distance
	cameraDistance_ = std::max(minCameraDistance_, std::min(maxCameraDistance_, cameraDistance_));

	UpdateCameraPosition();
}

float RayTracer::GetCurrentFPS() const {
	return SimpleGuiDX11::GetCurrentFPS();
}
//=============================================================================

//=============================================================================
// VOLUMETRIC HELPER FUNCTIONS
//=============================================================================

/// Evaluates the normalised Henyey-Greenstein phase function.
/// p(cos_theta, g) = (1 - g^2) / (4*pi * (1 + g^2 - 2*g*cos_theta)^(3/2))
/// @param cosTheta  Dot product of incoming and outgoing direction.
/// @param g         Asymmetry factor in (-1,1); 0 = isotropic.
//=============================================================================
// PATH TRACING: STATICKE POMOCNE METODY (deleguji na PathTracer)
//=============================================================================

float RayTracer::EvaluateHenyeyGreenstein(const float cosTheta, const float g) {
	return PathTracer::evaluateHenyeyGreenstein(cosTheta, g);
}

float RayTracer::BalancedHeuristic(const float p_a, const float p_b) {
	return PathTracer::balancedHeuristic(p_a, p_b);
}

void RayTracer::BuildONB(const Vector3& n, Vector3& t, Vector3& b) {
	PathTracer::buildONB(n, t, b);
}

Vector3 RayTracer::SampleHemisphereCosine(const Vector3& normal) {
	return PathTracer::sampleHemisphereCosine(normal);
}

Vector3 RayTracer::SampleDirectLightPT(const Vector3& hitPoint,
	const Vector3& normal, const Vector3& albedo) const
{
	if (!pathTracer_) return Vector3(0.0f);
	return pathTracer_->sampleDirectLight(hitPoint, normal, albedo, buildPathTracingContext());
}

Vector3 RayTracer::TracePath(const RTCRay& initialRay, const int maxDepth) const {
	if (!pathTracer_) return Vector3(0.0f);
	return pathTracer_->tracePath(initialRay, maxDepth, buildPathTracingContext());
}

/// Sestavi PathTracingContext z aktualniho (frame-stabilniho) stavu rendereru.
PathTracingContext RayTracer::buildPathTracingContext() const {
	PathTracingContext ctx{
		scene_,
		!surfaces_.empty(),
		lights_,
		backgroundEnabled_,
		backgroundColorVec_,
		cubemap_.get(),
		ptRRMinDepth_,
		maxDistance_,
		lightAttenuationFactor_,
		frameSunEnabled_,
		frameSunDir_,
		frameSunColor_,
		frameSunIntensity_,
		[this](const Vector3& p, const Vector3& l) { return IsHitPointVisible(p, l); }
	};
	return ctx;
}
//=============================================================================
// COMBINED MODE: Embree surface hit with depth (used by COMBINED_SDF)
//=============================================================================

/// Traces a primary ray through the Embree scene and returns both the shaded
/// colour and the hit distance t, enabling VolumetricEffect to stop marching
/// at the opaque surface boundary.
SurfaceHit RayTracer::TraceRayExtended(const RTCRay& ray,
	const float n_1, const int depth, const int maxDepth)
{
	auto bgColor = [&]() -> Vector3 {
		if (backgroundEnabled_ && cubemap_) {
			const Vector3 d(ray.dir_x, ray.dir_y, ray.dir_z);
			const Color3f c = cubemap_->GetTexel(d);
			return Vector3(c.r, c.g, c.b);
		}
		return backgroundColorVec_;
	};

	if (depth >= maxDepth)   return { bgColor(), FLT_MAX, false };
	if (surfaces_.empty())   return { bgColor(), FLT_MAX, false };

	RTCHit hit{};
	hit.geomID = RTC_INVALID_GEOMETRY_ID;
	hit.primID = RTC_INVALID_GEOMETRY_ID;

	RTCRayHit rayHit{};
	rayHit.ray = ray;
	rayHit.hit = hit;

	RTCIntersectContext ctx;
	rtcInitIntersectContext(&ctx);
	rtcIntersect1(scene_, &ctx, &rayHit);

	if (rayHit.hit.geomID == RTC_INVALID_GEOMETRY_ID)
		return { bgColor(), FLT_MAX, false };

	const float   tHit  = rayHit.ray.tfar;
	const Vector3 dir(rayHit.ray.dir_x, rayHit.ray.dir_y, rayHit.ray.dir_z);
	const Vector3 hitPt{
		rayHit.ray.org_x + dir.x * tHit,
		rayHit.ray.org_y + dir.y * tHit,
		rayHit.ray.org_z + dir.z * tHit
	};

	RTCGeometry geom = rtcGetGeometry(scene_, rayHit.hit.geomID);
	if (!geom) return { Vector3(1.0f, 0.0f, 1.0f), tHit, true }; // geometry invalidated
	Material*   mat  = static_cast<Material*>(rtcGetGeometryUserData(geom));
	if (!mat) return { Vector3(1.0f, 0.0f, 1.0f), tHit, true };

	Normal3f nrm{};
	rtcInterpolate0(geom, rayHit.hit.primID, rayHit.hit.u, rayHit.hit.v,
		RTC_BUFFER_TYPE_VERTEX_ATTRIBUTE, 0, &nrm.x, 3);
	Vector3 n(nrm.x, nrm.y, nrm.z);
	if (n.DotProduct(dir) > 0.0f) n *= -1.0f;

	Coord2f uv{};
	rtcInterpolate0(geom, rayHit.hit.primID, rayHit.hit.u, rayHit.hit.v,
		RTC_BUFFER_TYPE_VERTEX_ATTRIBUTE, 1, &uv.u, 2);

	Vector3 color;
	switch (mat->GetShader()) {
	case 1:  color = NormalShader(n); break;
	case 2:  color = LambertShader(*mat, uv, hitPt, n); break;
	case 4:  color = TransparentShader(ray, hitPt, n, dir, *mat, n_1, depth); break;
	default: color = PhongShader(*mat, uv, hitPt, n, dir, depth); break;
	}
	return { color, tHit, true };
}

//=============================================================================
