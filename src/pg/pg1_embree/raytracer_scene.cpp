#include "stdafx.h"
/**
 * @file  raytracer_scene.cpp
 * @brief Konfigurace sceny, nacitani ze souboru .scn a aktualizace animaci.
 *
 * Obsahuje: loadPredefinedScene, modeFromString, resolveActiveMode,
 * loadSceneFromDescription, updateScene, updateEntityTransforms.
 */#include "raytracer.h"
#include "objloader.h"

void RayTracer::loadPredefinedScene(SceneType type)
{
	currentScene_ = type;
	sceneTime_    = 0.0f;
	lights_.clear();

	// Clear any data-driven animation state so updateScene falls back to the
	// legacy hardcoded animation path when this function is called directly.
	sceneManager_->getLightDescs().clear();

	// Reset volumetric shapes to just the fixed SDF cloud
	volumetricShapes_.clear();
	if (fixedSdfScene_)
		volumetricShapes_.push_back(fixedSdfScene_.get());

	switch (type) {
	case SceneType::SCENE_SHADERTOY_SDF: {
		// Three orbiting point lights matching the reference ShaderToy.
		// Colors are pre-multiplied by 17.0 (reference: `LightColor * 17.0`).
		// At t=0 the lights are evenly spaced 120 degrees apart on a ring of
		// radius 18.5 at height y=6 (reference: `GetLightPosition`).
		const float kTwoThirdsPi = static_cast<float>(M_PI) * 2.0f / 3.0f;
		const float kRadius      = 18.5f;
		const Vector3 colors[3]  = {
			Vector3(1.0f, 0.0f, 1.0f) * 17.0f,  // Magenta
			Vector3(0.0f, 1.0f, 0.0f) * 17.0f,  // Green
			Vector3(0.0f, 0.0f, 1.0f) * 17.0f,  // Blue
		};
		for (int i = 0; i < 3; ++i) {
			const float theta = float(i) * kTwoThirdsPi;
			Light light;
			light.type      = LightType::Point;
			light.position  = Vector3(kRadius * cosf(theta), 6.0f, kRadius * sinf(theta));
			light.color     = colors[i];
			light.intensity = 1.0f;
			lights_.push_back(light);
		}
		currentRenderingMode_ = RenderingMode::VOLUMETRIC_SDF;
		rayMarching_          = true;

		// Position camera to give a good view of the SDF cloud.
		// IMPORTANT: updateCameraPosition() always orbits around the world ORIGIN
		// (hardcoded viewAt = (0,0,0)).  We must use that SAME origin here so that
		// the azimuth/elevation stored in cameraAzimuth_/cameraElevation_ exactly
		// reproduce kRefPos when updateCameraPosition is first called on mouse drag,
		// preventing the violent camera snap.
		const Vector3 kRefPos(0.0f, 40.0f, -100.0f);
		const Vector3 kRefAt(0.0f, 0.0f, 0.0f);  // MUST match updateCameraPosition's hardcoded origin
		camera_.SetViewFrom(kRefPos);
		camera_.SetViewAt(kRefAt);
		cameraX_ = kRefPos.x;
		cameraY_ = kRefPos.y;
		cameraZ_ = kRefPos.z;
		// Reverse-project into the Y-up spherical coordinates used by updateCameraPosition:
		//   x = dist*cos(el)*cos(az)   y = dist*sin(el)   z = dist*cos(el)*sin(az)
		cameraDistance_  = kRefPos.L2Norm();
		cameraAzimuth_   = rad2deg(atan2f(kRefPos.z, kRefPos.x)); // XZ plane
		cameraElevation_ = rad2deg(asinf(kRefPos.y / cameraDistance_)); // Y vertical
		while (cameraAzimuth_ < 0.0f) cameraAzimuth_ += 360.0f;

		// Dark blue-grey background: clearly non-zero so we can distinguish
		// missed rays from shadow.  backgroundEnabled_ stays false so the
		// cubemap (if loaded) is NOT used -- the SDF scene has no environment map.
		backgroundEnabled_  = false;
		backgroundColorVec_ = Vector3(0.05f, 0.05f, 0.08f);
		std::cout << "[SCENE] Loaded SCENE_SHADERTOY_SDF (3 animated point lights)" << std::endl;
		break;
	}

	case SceneType::SCENE_CUSTOM_OBJ: {
		Light key;
		key.type      = LightType::Directional;
		key.direction = Vector3(-0.5f, -1.0f, -0.5f);
		key.color     = Vector3(1.0f, 0.95f, 0.8f);
		key.intensity = 1.5f;
		lights_.push_back(key);
		currentRenderingMode_ = RenderingMode::SURFACE_EMBREE;
		std::cout << "[SCENE] Loaded SCENE_CUSTOM_OBJ" << std::endl;
		break;
	}

	case SceneType::SCENE_CUSTOM_VDB: {
		Light key;
		key.type      = LightType::Point;
		key.position  = Vector3(10.0f, 20.0f, 10.0f);
		key.color     = Vector3(1.0f, 0.9f, 0.7f);
		key.intensity = 200.0f;
		lights_.push_back(key);
		Light fill;
		fill.type      = LightType::Directional;
		fill.direction = Vector3(0.5f, -0.3f, 0.5f);
		fill.color     = Vector3(0.4f, 0.5f, 0.8f);
		fill.intensity = 0.5f;
		lights_.push_back(fill);
		currentRenderingMode_ = RenderingMode::VOLUMETRIC_VDB;
		std::cout << "[SCENE] Loaded SCENE_CUSTOM_VDB" << std::endl;
		break;
	}
	}
}

//===========================================================================

RenderingMode enum.
RenderingMode RayTracer::modeFromString(const std::string& modeStr) const
{
	if (modeStr == "VOLUMETRIC_SDF")      return RenderingMode::VOLUMETRIC_SDF;
	if (modeStr == "VOLUMETRIC_VDB")      return RenderingMode::VOLUMETRIC_VDB;
	if (modeStr == "COMBINED_SDF")        return RenderingMode::COMBINED_SDF;
	if (modeStr == "PATH_TRACING_EMBREE") return RenderingMode::PATH_TRACING_EMBREE;
	if (modeStr == "COMBINED_PT_SDF")     return RenderingMode::COMBINED_PT_SDF;
	if (modeStr == "COMBINED_PT_VDB")     return RenderingMode::COMBINED_PT_VDB;
	return RenderingMode::SURFACE_EMBREE;
}

/// Derives the active RenderingMode from currentFilter_ + objUsePathTracing_,
/// checking which assets are actually loaded so invalid combinations are avoided.
/// Called from MoveCamera() each frame to update currentRenderingMode_ before
/// the OMP pixel dispatch, ensuring thread-safe read of a stable cached value.
RenderingMode RayTracer::resolveActiveMode() const
{
	const bool hasMesh = !surfaces_.empty();
	const bool hasVdb  = hasVdbVolume();

	switch (currentFilter_) {

	case GlobalRenderFilter::ONLY_OBJ:
		return objUsePathTracing_
			? RenderingMode::PATH_TRACING_EMBREE
			: RenderingMode::SURFACE_EMBREE;

	case GlobalRenderFilter::ONLY_VDB:
		return RenderingMode::VOLUMETRIC_VDB;

	case GlobalRenderFilter::ONLY_SDF:
		return RenderingMode::VOLUMETRIC_SDF;

	case GlobalRenderFilter::OBJ_SDF:
		return objUsePathTracing_
			? RenderingMode::COMBINED_PT_SDF
			: RenderingMode::COMBINED_SDF;

	case GlobalRenderFilter::OBJ_VDB:
		// No Whitted+VDB composite mode exists; always use the PT variant.
		return RenderingMode::COMBINED_PT_VDB;

	case GlobalRenderFilter::ONLY_VOLUME:
		return hasVdb ? RenderingMode::VOLUMETRIC_VDB : RenderingMode::VOLUMETRIC_SDF;

	case GlobalRenderFilter::COMBINED:
	default:
		// Pick the most comprehensive pipeline for whatever is loaded.
		if (hasMesh && hasVdb)
			return RenderingMode::COMBINED_PT_VDB;
		if (hasMesh)
			return objUsePathTracing_
				? RenderingMode::COMBINED_PT_SDF
				: RenderingMode::COMBINED_SDF;
		return hasVdb ? RenderingMode::VOLUMETRIC_VDB : RenderingMode::VOLUMETRIC_SDF;
	}
}

/// Loads all assets described by @p desc and sets the engine into the
/// correct rendering mode.  Sequence:
///   1. Release previously loaded geometry / volume (safe under scene mutex).
///   2. Call the legacy loadPredefinedScene to configure lights and camera
///      to sensible defaults matching the entity type.
///   3. Load actual file assets (OBJ / VDB).  SDF shapes need no file load.
///   4. Restore the rendering mode specified in the .scn file (loadObjModel
///      hard-codes SURFACE_EMBREE, so we override it here).
void RayTracer::loadSceneFromDescription(const SceneDescription& desc)
{
	// Drain any in-flight render frames before swapping scene assets.
	// Save the caller's pause state and restore it afterwards so a manually
	// paused session doesn't inadvertently resume after a hot-swap.
	const bool wasPaused = renderingPaused_;
	renderingPaused_ = true;
	PauseRendering();

	// --- Categorise entities -------------------------------------------------
	bool hasMesh = false, hasVdb = false, hasSdf = false;
	bool isShaderToy = false;
	for (const auto& e : desc.entities) {
		if      (e.type == "MESH") hasMesh = true;
		else if (e.type == "VDB")  hasVdb  = true;
		else if (e.type == "SDF")  { hasSdf = true; if (e.value == "ShaderToy") isShaderToy = true; }
	}

	// --- Atomically drain render threads and tear down all stale assets ------
	// clearScene() acquires sceneMutex_ as unique_lock, which blocks until
	// every in-flight GetPixel() shared_lock holder has returned.  This is the
	// only serialisation point needed to prevent Embree/VDB/texture crashes.
	// NOTE: loadObjModel() and loadVdbVolume() each take their own unique_lock
	// internally, so the lock is fully released before those calls are made.
	clearScene();

	// --- Call legacy setup for Embree/VKL context + SDF volumetric shapes ---
	// This also clears activeLightDescs_ and lights_ (we rebuild them below).
	if (hasMesh)
		loadPredefinedScene(SceneType::SCENE_CUSTOM_OBJ);
	else if (hasVdb)
		loadPredefinedScene(SceneType::SCENE_CUSTOM_VDB);
	else
		loadPredefinedScene(SceneType::SCENE_SHADERTOY_SDF);

	sceneTime_ = 0.0f;

	// --- Apply camera from the .scn description ------------------------------
	if (desc.hasCamera) {
		const Vector3 pos(desc.camera.px, desc.camera.py, desc.camera.pz);
		const Vector3 at (desc.camera.tx, desc.camera.ty, desc.camera.tz);
		camera_.SetViewFrom(pos);
		camera_.SetViewAt(at);
		cameraX_ = pos.x;  cameraY_ = pos.y;  cameraZ_ = pos.z;
		// Recompute orbital parameters (Y-up spherical) so mouse-drag doesn't snap on first use.
		const Vector3 dir = pos - at;
		cameraDistance_ = dir.L2Norm();
		if (cameraDistance_ > 0.0f) {
			cameraAzimuth_   = rad2deg(atan2f(dir.z, dir.x)); // XZ plane
			cameraElevation_ = rad2deg(asinf(dir.y / cameraDistance_)); // Y vertical
			while (cameraAzimuth_ < 0.0f) cameraAzimuth_ += 360.0f;
		}
		std::cout << "[SceneLoader] Camera set from .scn: pos=("
		          << pos.x << "," << pos.y << "," << pos.z << ")\n";
	}

	// --- Apply lights from the .scn description ------------------------------
	// Rebuilding lights_ from the parsed descriptors replaces any lights that
	// loadPredefinedScene installed above.
	if (!desc.lights.empty()) {
		lights_.clear();
		for (const auto& ld : desc.lights) {
			Light l;
			if (ld.type == "POINT") {
				l.type     = LightType::Point;
				l.position = Vector3(ld.px, ld.py, ld.pz);
			} else {
				// DIRECTIONAL: px/py/pz encodes the direction vector
				l.type      = LightType::Directional;
				l.direction = Vector3(ld.px, ld.py, ld.pz);
			}
			// r/g/b may already be pre-multiplied by intensity in the .scn file
			l.color     = Vector3(ld.r, ld.g, ld.b);
			l.intensity = 1.0f;
			lights_.push_back(l);
		}
		// Store descriptors so updateScene can drive per-frame ORBIT animation.
		sceneManager_->getLightDescs() = desc.lights;
		std::cout << "[SceneLoader] " << lights_.size() << " light(s) applied from .scn\n";
	}
	// If no lights defined in .scn, the lights from loadPredefinedScene remain
	// (activeLightDescs_ is empty â†’ legacy animation path in updateScene).

	// --- Load file assets and register entity animation state ----------------
	for (const auto& e : desc.entities) {
		const bool hasAnim = !e.animType.empty();

		// Lambda fills the common animation fields shared by all entity kinds.
		auto fillAnim = [&](EntityAnimState& anim) {
			anim.animType   = e.animType;
			anim.animSpeed  = e.animSpeed;
			anim.animParam1 = e.animParam1;
			anim.targetX    = e.animTargetX;
			anim.targetY    = e.animTargetY;
			anim.targetZ    = e.animTargetZ;
		};

		if (e.type == "MESH") {
			if (hasAnim) {
				EntityAnimState anim;
				anim.entityKind = "MESH";
				fillAnim(anim);
				// Mesh is loaded at the OBJ origin; base position is (0,0,0).
				if (loadObjModel(e.value, &anim))
					sceneManager_->getEntityAnims().push_back(std::move(anim));
				else
					std::cout << "[SceneLoader] Failed to load MESH: " << e.value << "\n";
			} else {
				if (!loadObjModel(e.value))
					std::cout << "[SceneLoader] Failed to load MESH: " << e.value << "\n";
			}
		}
		else if (e.type == "VDB") {
			if (!loadVdbVolume(e.value, "density"))
				std::cout << "[SceneLoader] Failed to load VDB: " << e.value << "\n";
			if (hasAnim) {
				EntityAnimState anim;
				anim.entityKind = "VDB";
				fillAnim(anim);
				sceneManager_->getEntityAnims().push_back(std::move(anim));
			}
		}
		else if (e.type == "SDF") {
			// SDF: no file to load; the SDF cloud is managed by fixedSdfScene_.
			if (hasAnim) {
				EntityAnimState anim;
				anim.entityKind = "SDF";
				fillAnim(anim);
				sceneManager_->getEntityAnims().push_back(std::move(anim));
			}
		}
	}

	// currentRenderingMode_ is now set each frame by resolveActiveMode() in MoveCamera().

	std::cout << "[SceneLoader] Scene \"" << desc.name
	          << "\" ready  (lights: " << lights_.size() << ")\n";

	// Restore caller's pause state.
	renderingPaused_ = wasPaused;
	if (!wasPaused)
		ResumeRendering();
}

//=============================================================================
// SCENE ANIMATION (animates lights and other time-varying scene elements)
//=============================================================================

/// Advances the scene simulation to the given time.
///
/// DATA-DRIVEN PATH (scenes.scn loaded via loadSceneFromDescription):
///   Iterates over activeLightDescs_.  For each POINT light with animType=="ORBIT"
///   the position is updated in the XZ plane around the light's base position:
///     x = base_x + cos(t * speed + phase) * radius
///     z = base_z + sin(t * speed + phase) * radius
///     y = base_y  (unchanged â€” orbit is around the Y axis)
///   DIRECTIONAL lights and lights with no animType are left untouched.
///
/// LEGACY PATH (loaded via loadPredefinedScene directly):
///   Preserves the original hardcoded ShaderToy SDF orbit for backward compat.
///
/// THREAD SAFETY: called from MoveCamera() on the main/UI thread, between
/// frames.  Render threads hold a shared_lock while reading lights_; this
/// function runs without a lock (same pattern as the pre-existing legacy path).
/// Single-float position writes on x86/x64 are effectively atomic, and the
/// update precedes the next frame's pixel dispatch.
///
/// @param time  Accumulated scene time in seconds.
void RayTracer::updateScene(const float time)
{
	sceneTime_ = time;
	// Sestavi zdroje pro SceneManager a deleguj aktualizaci animaci
	SceneAnimResources res{
		lights_,
		scene_,
		sceneMutex_,
		sdfAnimOffset_,
		vdbAnimOffset_,
		lightAnimSpeed_,
		currentScene_
	};
	sceneManager_->update(time, res);
}

//=============================================================================
// TRANSFORMACE ENTIT -- deleguje na SceneManager
//=============================================================================

void RayTracer::updateEntityTransforms(const float time)
{
	SceneAnimResources res{
		lights_,
		scene_,
		sceneMutex_,
		sdfAnimOffset_,
		vdbAnimOffset_,
		lightAnimSpeed_,
		currentScene_
	};
	sceneManager_->updateEntityTransforms(time, res);
}