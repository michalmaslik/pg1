#include "stdafx.h"
#include "raytracer.h"
#include "objloader.h"
#include "ShadingUtils.h"

==
// INTEL EMBREE INITIALIZATION (Surface Ray Tracing)
//=============================================================================

int RayTracer::initDeviceAndScene(const char* config)
{
	std::cout << "[RAY TRACER] Initializing Embree device..." << std::endl;

	// Create Intel Embree device
	device_ = rtcNewDevice(config);
	error_handler(nullptr, rtcGetDeviceError(device_), "Unable to create a new device.\n");
	rtcSetDeviceErrorFunction(device_, error_handler, nullptr);

	// Verify triangle geometry support
	ssize_t triangleSupported = rtcGetDeviceProperty(device_, RTC_DEVICE_PROPERTY_TRIANGLE_GEOMETRY_SUPPORTED);
	if (!triangleSupported) {
		std::cerr << "[RAY TRACER ERROR] Triangle geometry not supported!" << std::endl;
		return E_FAIL;
	}

	// Create scene bound to the device
	scene_ = rtcNewScene(device_);
	if (!scene_) {
		std::cerr << "[RAY TRACER ERROR] Failed to create Embree scene!" << std::endl;
		return E_FAIL;
	}

	// Commit empty scene (this is safe and allows ray tracing calls)
	rtcCommitScene(scene_);

	std::cout << "[RAY TRACER] Embree device and scene initialized successfully" << std::endl;
	return S_OK;
}

int RayTracer::releaseDeviceAndScene()
{
	rtcReleaseScene(scene_);
	rtcReleaseDevice(device_);
	return S_OK;
}

//=============================================================================
// SURFACE GEOMETRY LOADING (Triangle Meshes for Embree)
//=============================================================================

void RayTracer::loadModel(const std::string& fileName, const Transform& transform)
{
	// LoadOBJ expects raw-pointer vectors (its interface is unchanged).
	// We use temporary raw vectors and transfer ownership into unique_ptr
	// vectors immediately after the Embree setup loop.
	std::vector<Surface*>  tempSurfs;
	std::vector<Material*> tempMats;
	const int noSurfaces = LoadOBJ(fileName.c_str(), tempSurfs, tempMats);

	// For each surface (triangle mesh) in the OBJ file
	for (Surface* surface : tempSurfs)
	{
		// Create Embree triangle geometry
		RTCGeometry mesh = rtcNewGeometry(device_, RTC_GEOMETRY_TYPE_TRIANGLE);

		// === VERTEX BUFFER SETUP ===
		// Allocate vertex buffer for triangle vertices
		Vertex3f* vertices = (Vertex3f*)rtcSetNewGeometryBuffer(
			mesh, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3,
			sizeof(Vertex3f), 3 * surface->no_triangles());

		// === INDEX BUFFER SETUP ===
		// Allocate index buffer for triangle indices (3 vertices per triangle)
		Triangle3ui* triangles = (Triangle3ui*)rtcSetNewGeometryBuffer(
			mesh, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3,
			sizeof(Triangle3ui), surface->no_triangles());

		// Associate material data with geometry (for shading)
		rtcSetGeometryUserData(mesh, (void*)(surface->get_material()));

		// Set up vertex attributes (normals and texture coordinates)
		rtcSetGeometryVertexAttributeCount(mesh, 2);

		// === NORMAL BUFFER SETUP ===
		Normal3f* normals = (Normal3f*)rtcSetNewGeometryBuffer(
			mesh, RTC_BUFFER_TYPE_VERTEX_ATTRIBUTE, 0, RTC_FORMAT_FLOAT3,
			sizeof(Normal3f), 3 * surface->no_triangles());

		// === TEXTURE COORDINATE BUFFER SETUP ===
		Coord2f* texCoords = (Coord2f*)rtcSetNewGeometryBuffer(
			mesh, RTC_BUFFER_TYPE_VERTEX_ATTRIBUTE, 1, RTC_FORMAT_FLOAT2,
			sizeof(Coord2f), 3 * surface->no_triangles());

		// === POPULATE BUFFERS ===
		// Fill vertex, normal, texture coordinate and index buffers
		for (int i = 0, k = 0; i < surface->no_triangles(); ++i)
		{
			Triangle& triangle = surface->get_triangle(i);

			// Process each vertex of the triangle
			for (int j = 0; j < 3; ++j, ++k)
			{
				const Vertex& vertex = triangle.vertex(j);

				// Apply transformation to vertex position
				vertices[k].x = vertex.position.x * transform.scale.x + transform.position.x;
				vertices[k].y = vertex.position.y * transform.scale.y + transform.position.y;
				vertices[k].z = vertex.position.z * transform.scale.z + transform.position.z;

				// Transform and normalize normals
				normals[k].x = vertex.normal.x * transform.scale.x;
				normals[k].y = vertex.normal.y * transform.scale.y;
				normals[k].z = vertex.normal.z * transform.scale.z;

				float length = sqrt(normals[k].x * normals[k].x + normals[k].y * normals[k].y + normals[k].z * normals[k].z);
				if (length > 0.0f) {
					normals[k].x /= length;
					normals[k].y /= length;
					normals[k].z /= length;
				}

				// Copy texture coordinates
				texCoords[k].u = vertex.texture_coords[0].u;
				texCoords[k].v = vertex.texture_coords[0].v;
			}

			// Set triangle indices (last 3 vertices added)
			triangles[i].v0 = k - 3;
			triangles[i].v1 = k - 2;
			triangles[i].v2 = k - 1;
		}

		// Commit geometry changes to Embree
		rtcCommitGeometry(mesh);

		// Attach geometry to scene and get geometry ID
		unsigned int geom_id = rtcAttachGeometry(scene_, mesh);

		// Release geometry handle (scene holds reference)
		rtcReleaseGeometry(mesh);

		// Transfer ownership into the smart-pointer vector
		surfaces_.push_back(std::unique_ptr<Surface>(surface));
	}

	// Transfer material ownership
	for (Material* m : tempMats)
		materials_.push_back(std::unique_ptr<Material>(m));
}

void RayTracer::loadScene(
	const std::vector<ModelInfo>& models,
	const std::vector<Shape*>& shapes,
	const char* cubeMapFileNames[6])
{
	// Load environment map for background and reflections
	cubemap_ = std::make_unique<CubeMap>(cubeMapFileNames);

	// Load all polygonal models (for surface ray tracing with Embree)
	for (const auto& model : models)
	{
		loadModel(model.filePath, model.transform);
	}

	// Add procedural volumetric shapes (for SDF ray marching)
	for (Shape* shape : shapes)
	{
		volumetricShapes_.push_back(shape);
	}

	// Commit scene to build BVH acceleration structure
	rtcCommitScene(scene_);
}

//=============================================================================
// UTILITY FUNCTIONS
//=============================================================================

// makeSecondaryRay -> presunuto do ShadingUtils.cpp

bool RayTracer::isHitPointVisible(const Vector3& hitPoint, const Vector3& lightPoint) const {
	// If no surface geometry is loaded, assume point is always visible
	if (surfaces_.empty()) {
		return true;
	}

	// Create shadow ray from hit point to light
	Vector3 l = lightPoint - hitPoint;
	float dist = l.L2Norm();
	l *= 1.0f / dist;  // Normalize direction

	RTCRay ray = RTCRay();
	ray.org_x = hitPoint.x;  ray.org_y = hitPoint.y;  ray.org_z = hitPoint.z;
	ray.tnear = 0.001f;      // Small bias to avoid self-shadowing

	ray.dir_x = l.x;  ray.dir_y = l.y;  ray.dir_z = l.z;
	ray.time = 0.0f;
	ray.tfar = dist;         // Stop at light distance

	ray.mask = 0;  ray.id = 0;  ray.flags = 0;

	// Setup hit structure
	RTCHit hit;
	hit.geomID = RTC_INVALID_GEOMETRY_ID;
	hit.primID = RTC_INVALID_GEOMETRY_ID;
	hit.Ng_x = 0.0f;  hit.Ng_y = 0.0f;  hit.Ng_z = 0.0f;

	// Merge ray and hit structures
	RTCRayHit rayHit;
	rayHit.ray = ray;
	rayHit.hit = hit;

	// Intersect shadow ray with scene using Embree
	RTCIntersectContext context;
	rtcInitIntersectContext(&context);
	rtcIntersect1(scene_, &context, &rayHit);

	// Return true if no geometry blocks the light
	return rayHit.hit.geomID == RTC_INVALID_GEOMETRY_ID;
}

//=============================================================================
// LIGHTING & SHADING UTILITIES
//=============================================================================

// getLightAttenuation -> presunuto do ShadingUtils.cpp

// getAmbientLight -> presunuto do ShadingUtils.cpp

// computeSdfNormal -> presunuto do ShadingUtils.cpp

//===========================================================================

void RayTracer::initializeFixedSdfScene() {
	// Three smooth-unioned spheres matching the reference ShaderToy SDF at t=0:
	//   sdSphere(p, (-8, 2,-1), 5.6)  -- dense core
	//   sdSphere(p, ( 8, 8, 3), 5.6)  -- upper-right lobe
	//   sdSphere(p, ( 0, 3, 0), 8.0)  -- large central mass
	// Smoothing k=3.0 matches sdSmoothUnion blending radius in the reference.
	// fBm scale 1/3.2 and strength 7.0 reproduce the noisy cloud silhouette.
	std::vector<std::unique_ptr<Shape>> cloudShapes;
	cloudShapes.push_back(std::make_unique<Sphere>(Vector3(-8.0f,  2.0f, -1.0f), 5.6f, 3.0f));
	cloudShapes.push_back(std::make_unique<Sphere>(Vector3( 8.0f,  8.0f,  3.0f), 5.6f, 3.0f));
	cloudShapes.push_back(std::make_unique<Sphere>(Vector3( 0.0f,  3.0f,  0.0f), 8.0f, 3.0f));

	fixedSdfScene_ = std::make_unique<SmoothUnion>(
		std::move(cloudShapes),
		Noise(Noise::NoiseType::FractalBrownianMotion, 1.0f / 3.2f, 7.0f));

	// Push a non-owning observer pointer into the volumetric dispatch list
	volumetricShapes_.push_back(fixedSdfScene_.get());
	// Synchronizuj instancni priznak sumu s aktualnim stavem useNoise_
	fixedSdfScene_->setNoiseEnabled(useNoise_);
	std::cout << "[RAY TRACER] Fixed SDF scene initialized (3-sphere cloud)" << std::endl;
}

bool RayTracer::loadObjModel(const std::string& fileName, EntityAnimState* animOut) {
	std::cout << "[RAY TRACER] Loading OBJ model: " << fileName << std::endl;

	// Exclusive lock: blocks until every in-flight GetPixel() call releases its
	// shared_lock, guaranteeing no thread is mid-sample when we destroy assets.
	std::unique_lock<std::shared_mutex> reloadLock(sceneMutex_);

	// Clear existing surface models first
	clearSurfaceModels();

	// Load new OBJ file
	std::vector<Surface*> newSurfaces;
	std::vector<Material*> newMaterials;

	const int noSurfaces = LoadOBJ(fileName.c_str(), newSurfaces, newMaterials);

	if (noSurfaces <= 0) {
		std::cerr << "[RAY TRACER ERROR] Failed to load model: " << fileName << std::endl;
		return false;
	}

	// clearSurfaceModels() above already released the old scene and created a
	// fresh empty one.  Re-using that scene avoids a window where scene_ is a
	// dangling pointer between a second release and the next rtcNewScene call.

	// Whether to use application-owned (shared) vertex buffers for animation updates.
	const bool needsAnim = (animOut != nullptr);

	// Add loaded surfaces to the new scene
	for (Surface* surface : newSurfaces) {
		const int nVerts = 3 * surface->no_triangles();

		// Create Embree geometry for this surface
		RTCGeometry mesh = rtcNewGeometry(device_, RTC_GEOMETRY_TYPE_TRIANGLE);

		// --- Vertex buffer ---------------------------------------------------
		// For animated entities we use a shared (application-owned) buffer so
		// we can rewrite positions each frame via rtcUpdateGeometryBuffer.
		// For static entities the Embree-owned allocation is simpler/cheaper.
		Vertex3f* vertices = nullptr;
		EntityAnimState::PerGeom pg;

		if (needsAnim) {
			pg.animVerts.resize(nVerts);
			pg.baseVerts.resize(nVerts);
			rtcSetSharedGeometryBuffer(mesh, RTC_BUFFER_TYPE_VERTEX, 0,
				RTC_FORMAT_FLOAT3, pg.animVerts.data(), 0,
				sizeof(Vertex3f), static_cast<size_t>(nVerts));
			vertices = pg.animVerts.data();
		} else {
			vertices = (Vertex3f*)rtcSetNewGeometryBuffer(
				mesh, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3,
				sizeof(Vertex3f), static_cast<size_t>(nVerts));
		}

		// Index buffer (always Embree-owned â€” indices never change)
		Triangle3ui* triangles = (Triangle3ui*)rtcSetNewGeometryBuffer(
			mesh, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3,
			sizeof(Triangle3ui), surface->no_triangles());

		// Pass raw material pointer as Embree user data (safe: unique_ptr below keeps it alive)
		rtcSetGeometryUserData(mesh, static_cast<void*>(surface->get_material()));

		// Vertex attributes
		rtcSetGeometryVertexAttributeCount(mesh, 2);

		// Normal buffer
		Normal3f* normals = (Normal3f*)rtcSetNewGeometryBuffer(
			mesh, RTC_BUFFER_TYPE_VERTEX_ATTRIBUTE, 0, RTC_FORMAT_FLOAT3,
			sizeof(Normal3f), static_cast<size_t>(nVerts));

		// Texture coordinate buffer
		Coord2f* texCoords = (Coord2f*)rtcSetNewGeometryBuffer(
			mesh, RTC_BUFFER_TYPE_VERTEX_ATTRIBUTE, 1, RTC_FORMAT_FLOAT2,
			sizeof(Coord2f), static_cast<size_t>(nVerts));

		// Fill buffers
		for (int i = 0, k = 0; i < surface->no_triangles(); ++i) {
			Triangle& triangle = surface->get_triangle(i);

			for (int j = 0; j < 3; ++j, ++k) {
				const Vertex& vertex = triangle.vertex(j);

				vertices[k].x = vertex.position.x;
				vertices[k].y = vertex.position.y;
				vertices[k].z = vertex.position.z;

				normals[k].x = vertex.normal.x;
				normals[k].y = vertex.normal.y;
				normals[k].z = vertex.normal.z;

				texCoords[k].u = vertex.texture_coords[0].u;
				texCoords[k].v = vertex.texture_coords[0].v;
			}

			triangles[i].v0 = k - 3;
			triangles[i].v1 = k - 2;
			triangles[i].v2 = k - 1;
		}

		// Copy initial positions into baseVerts so updateEntityTransforms can
		// reconstruct the correct world positions each frame from scratch.
		if (needsAnim) {
			pg.baseVerts = pg.animVerts;  // snapshot of t=0 positions
		}

		rtcCommitGeometry(mesh);
		const unsigned int geomID = rtcAttachGeometry(scene_, mesh);
		rtcReleaseGeometry(mesh);

		if (needsAnim) {
			pg.geomID = geomID;
			animOut->geoms.push_back(std::move(pg));
		}

		// Transfer ownership into the unique_ptr vector
		surfaces_.push_back(std::unique_ptr<Surface>(surface));
	}

	// Transfer material ownership
	for (Material* m : newMaterials)
		materials_.push_back(std::unique_ptr<Material>(m));

	// Collect unique texture pointers into ownedTextures_ for safe single-delete.
	// Multiple materials from the same OBJ may share a Texture* (TextureProxy
	// deduplication); iterating with an explicit duplicate check avoids double-free.
	for (const auto& mat : materials_) {
		for (int i = 0; i < NO_TEXTURES; ++i) {
			Texture* tex = mat->get_texture(i);
			if (!tex) continue;
			bool alreadyOwned = false;
			for (Texture* existing : ownedTextures_)
				if (existing == tex) { alreadyOwned = true; break; }
			if (!alreadyOwned) ownedTextures_.push_back(tex);
		}
	}

	// Commit BVH
	rtcCommitScene(scene_);

	std::cout << "[RAY TRACER] Successfully loaded " << noSurfaces << " surfaces" << std::endl;
	return true;
}

void RayTracer::clearSurfaceModels() {
	// Release the Embree scene FIRST: it holds raw Material* via rtcSetGeometryUserData.
	// Clearing the unique_ptr vectors before releasing the scene would create dangling
	// pointers inside Embree's geometry user-data slots.
	rtcReleaseScene(scene_);
	scene_ = rtcNewScene(device_);
	rtcCommitScene(scene_);

	// Null out every texture slot before clearing materials.
	// TextureProxy in the OBJ loader reuses the same Texture* across multiple
	// materials (shared ownership by filename); ~Material() would call delete on
	// the same pointer N times causing a double-free crash (texture.cpp:57).
	// Ownership is tracked in ownedTextures_ and deleted exactly once below.
	for (auto& mat : materials_) {
		for (int i = 0; i < NO_TEXTURES; ++i)
			mat->set_texture(i, nullptr);
	}
	for (Texture* tex : ownedTextures_) delete tex;
	ownedTextures_.clear();

	surfaces_.clear();
	materials_.clear();  // ~Material() sees all-null texture slots â€” safe

	std::cout << "[RAY TRACER] Surface models cleared" << std::endl;
}

void RayTracer::clearScene()
{
	// Acquire an exclusive lock.  This BLOCKS until every GetPixel() call that
	// currently holds a shared_lock has returned, guaranteeing no thread is
	// mid-rtcIntersect1, mid-vklComputeSample, or mid-get_texture() when we
	// start freeing memory.  This is the single serialisation point that fixes:
	//   â€˘ Embree crash  (rtcore_geometry.h:305) â€” scene released under active render
	//   â€˘ VDB crash     (line ~1406)            â€” sampler released under active march
	//   â€˘ Texture crash (texture.cpp:57)        â€” Material dtors run under active shade
	std::unique_lock<std::shared_mutex> lock(sceneMutex_);

	// ---- Embree geometry teardown -------------------------------------------
	// Release the scene BEFORE clearing the smart-pointer vectors.
	// Embree stores raw Material* in geometry user-data; releasing the scene
	// first ensures Embree's internal book-keeping is finished before those
	// pointers are deleted by unique_ptr<Material>::~unique_ptr().
	rtcReleaseScene(scene_);
	scene_ = rtcNewScene(device_);
	rtcCommitScene(scene_);
	surfaces_.clear();
	// Null out every texture slot before clearing materials to prevent the
	// double-free caused by shared Texture* pointers across materials.
	for (auto& mat : materials_) {
		for (int i = 0; i < NO_TEXTURES; ++i)
			mat->set_texture(i, nullptr);
	}
	for (Texture* tex : ownedTextures_) delete tex;
	ownedTextures_.clear();
	materials_.clear();  // ~Material() now safe: all texture slots are null

	// ---- OpenVKL teardown (per-scene data only, device preserved) -----------
	// Uvolni pouze svazek a sampler, zarizeni zachovej (VdbRenderer::clearVolume).
	// Volani clearVolume() (ne cleanup()) zachovava device pro dalsi hot-swap.
	if (vdbRenderer_) vdbRenderer_->clearVolume();

	// ---- SDF observer list --------------------------------------------------
	// volumetricShapes_ holds non-owning raw pointers; just clear the list.
	// fixedSdfScene_ (the actual owner) is never released here.
	volumetricShapes_.clear();
	if (fixedSdfScene_)
		volumetricShapes_.push_back(fixedSdfScene_.get());

	// ---- Entity animation state ---------------------------------------------
	// Shared vertex buffer memory lives inside EntityAnimState::PerGeom::animVerts.
	// Clearing AFTER the Embree scene release above ensures the scene no longer
	// holds any reference to those buffers before the vectors are freed.
	sceneManager_->clearAnimations();
	sdfAnimOffset_ = Vector3(0.0f, 0.0f, 0.0f);
	vdbAnimOffset_ = Vector3(0.0f, 0.0f, 0.0f);

	std::cout << "[clearScene] All scene assets torn down safely.\n";
}

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
