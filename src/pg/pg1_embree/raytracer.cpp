

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

	// Load default cubemap (ak existujú súbory)
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

	// fixedSdfScene_ and cubemap_ are unique_ptr — destroyed automatically at scope exit.
	volumetricShapes_.clear();  // non-owning observer list

	// ClearSurfaceModels releases scene_ and clears surfaces_/materials_ (unique_ptr auto-delete)
	ClearSurfaceModels();

	// Release the fresh empty scene created by ClearSurfaceModels, then the device
	rtcReleaseScene(scene_);
	rtcReleaseDevice(device_);
}

//=============================================================================
// INTEL EMBREE INITIALIZATION (Surface Ray Tracing)
//=============================================================================

int RayTracer::InitDeviceAndScene(const char* config)
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

int RayTracer::ReleaseDeviceAndScene()
{
	rtcReleaseScene(scene_);
	rtcReleaseDevice(device_);
	return S_OK;
}

//=============================================================================
// SURFACE GEOMETRY LOADING (Triangle Meshes for Embree)
//=============================================================================

void RayTracer::LoadModel(const std::string& fileName, const Transform& transform)
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

void RayTracer::LoadScene(
	const std::vector<ModelInfo>& models,
	const std::vector<Shape*>& shapes,
	const char* cubeMapFileNames[6])
{
	// Load environment map for background and reflections
	cubemap_ = std::make_unique<CubeMap>(cubeMapFileNames);

	// Load all polygonal models (for surface ray tracing with Embree)
	for (const auto& model : models)
	{
		LoadModel(model.filePath, model.transform);
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

bool RayTracer::IsHitPointVisible(const Vector3& hitPoint, const Vector3& lightPoint) const {
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

//=============================================================================
// SURFACE SHADING MODELS (for Embree-traced geometry)
//=============================================================================

Vector3 RayTracer::NormalShader(const Vector3& normalVector) {
	// Visualize normal as RGB color (map from [-1,1] to [0,1])
	return (normalVector + Vector3(1.0f, 1.0f, 1.0f)) * 0.5f;
}

Vector3 RayTracer::LambertShader(const Material& material, const Coord2f& texCoord,
	const Vector3& hitPoint, const Vector3& normalVector) {

	Vector3 outputColor;

	// Get diffuse colour from material or texture
	Vector3 diffuseColor = material.GetDiffuse();
	Texture* diffuseTexture = material.get_texture(Material::kDiffuseMapSlot);
	if (diffuseTexture) {
		const Color3f diffuseTexel = diffuseTexture->get_texel(texCoord.u, 1 - texCoord.v);
		diffuseColor = Vector3(diffuseTexel.r, diffuseTexel.g, diffuseTexel.b);
	}

	const Vector3 omniLightPosition = light_.position;
	Vector3 l = omniLightPosition - hitPoint;
	l.Normalize();

	// Ambient light contribution
	outputColor += material.GetAmbient();

	// Diffuse lighting with shadow test
	if (IsHitPointVisible(hitPoint, omniLightPosition)) {
		// Lambert's Law: I_diffuse = I_light * albedo * max(0, N·L)
		outputColor += diffuseColor * clamp(normalVector.DotProduct(l));
	}

	return outputColor;
}

Vector3 RayTracer::PhongShader(const Material& material, const Coord2f& texCoord,
	const Vector3& hitPoint, const Vector3& normalVector, const Vector3& directionVector, const int depth) {

	Vector3 outputColor;

	// Get diffuse colour from material or texture
	Vector3 diffuseColor = material.GetDiffuse();
	Texture* diffuseTexture = material.get_texture(Material::kDiffuseMapSlot);
	if (diffuseTexture) {
		const Color3f diffuseTexel = diffuseTexture->get_texel(texCoord.u, 1 - texCoord.v);
		diffuseColor = Vector3(diffuseTexel.r, diffuseTexel.g, diffuseTexel.b);
	}

	// Get specular colour from material or texture
	Vector3 specularColor = material.GetSpecular();
	Texture* specularTexture = material.get_texture(Material::kSpecularMapSlot);
	if (specularTexture) {
		const Color3f specularTexel = specularTexture->get_texel(texCoord.u, 1.0f - texCoord.v);
		specularColor = Vector3(specularTexel.r, specularTexel.g, specularTexel.b);
	}

	const Vector3 omniLightPosition = light_.position;
	Vector3 l = omniLightPosition - hitPoint;
	l.Normalize();

	// Ambient light contribution
	outputColor += material.GetAmbient();

	// Diffuse and specular lighting with shadow test
	if (IsHitPointVisible(hitPoint, omniLightPosition)) {
		// Diffuse lighting (Lambert)
		outputColor += diffuseColor * clamp(normalVector.DotProduct(l));

		// Specular lighting (Phong)
		Vector3 l_r = 2.0f * (l.DotProduct(normalVector) * normalVector) - l; // Reflection vector
		l_r.Normalize();

		// Trace secondary ray for indirect specular contribution
		RTCRay secondaryRay = makeSecondaryRay(hitPoint, l);
		Vector3 l_i = TraceRay(secondaryRay, depth + 1.0f);

		// Phong specular: I_spec = I_light * k_s * max(0, R·V)^shininess
		outputColor += specularColor * powf(clamp(l_r.DotProduct(-directionVector)), material.GetShininess());
		outputColor += l_i * specularColor * 0.2f; // Indirect specular contribution
	}

	// --- Global Sun contribution (directional, no 1/d² attenuation) -----------
	// frameSunDir_ points TOWARD the sun, matching the convention of local l.
	if (frameSunEnabled_) {
		const float nDotSun = normalVector.DotProduct(frameSunDir_);
		if (nDotSun > 0.0f) {
			// Shadow: push a virtual point far along the sun direction.
			const Vector3 sunPt = hitPoint + frameSunDir_ * maxDistance_;
			if (IsHitPointVisible(hitPoint, sunPt)) {
				const Vector3 Li = frameSunColor_ * frameSunIntensity_;
				// Diffuse
				outputColor += diffuseColor * nDotSun * Li;
				// Specular
				Vector3 sunRefl = 2.0f * (nDotSun * normalVector) - frameSunDir_;
				sunRefl.Normalize();
				outputColor += specularColor * Li *
				               powf(clamp(sunRefl.DotProduct(-directionVector)),
				                    material.GetShininess());
			}
		}
	}

	return outputColor;
}

Vector3 t_b_l(const float length, const Vector3 attenuation) {
	// Beer-Lambert Law: T = e^(-σ * d) for each color channel
	Vector3 a = Vector3{ powf(exp(1.0f), -attenuation.x * length),
						 powf(exp(1.0f), -attenuation.y * length),
						 powf(exp(1.0f), -attenuation.z * length) };
	return a;
}

Vector3 RayTracer::TransparentShader(const RTCRay& ray, const Vector3& hitPoint, const Vector3& normalVector,
	const Vector3& directionVector, const Material& material, const float n_1, const int depth) {

	float n_2 = material.GetIor();
	if (n_1 == material.GetIor()) {
		n_2 = 1.0f; // Transition to air
	}

	float n_ratio = n_1 / n_2;
	float r; // Reflection coefficient from Fresnel equations

	// Clamp dot product for robustness
	float cos_theta1 = clamp(normalVector.DotProduct(-directionVector), -1.0f, 1.0f);
	float temp = 1.0f - powf(n_ratio, 2.0f) * (1.0f - powf(cos_theta1, 2.0f));
	float cos_theta2 = 0.0f;
	if (temp > 0.0f) cos_theta2 = sqrtf(temp);

	Vector3 refractedRayDirection;
	Vector3 reflectedRayDirection;

	// Compute reflection direction (always valid):
	reflectedRayDirection = directionVector - 2.0f * normalVector.DotProduct(directionVector) * normalVector;
	reflectedRayDirection.Normalize();

	// Compute refraction direction, only if not total internal reflection:
	bool totalInternalReflection = (temp < 0.0f);
	if (!totalInternalReflection) {
		refractedRayDirection = n_ratio * directionVector + (n_ratio * cos_theta1 - cos_theta2) * normalVector;
		refractedRayDirection.Normalize();
	}

	// Fresnel equations or Schlick approximation
	if (totalInternalReflection) {
		r = 1.0f; // All energy goes to reflection
	}
	else {
		// Robust Fresnel (or try Schlick for speed)
		float r_s = powf((n_2 * cos_theta2 - n_1 * cos_theta1) / (n_2 * cos_theta2 + n_1 * cos_theta1), 2.0f);
		float r_p = powf((n_2 * cos_theta1 - n_1 * cos_theta2) / (n_2 * cos_theta1 + n_1 * cos_theta2), 2.0f);
		r = (r_s + r_p) / 2.0f;
		// Alternatíva (stačí na vizuálne efekty):
		// float R0 = powf((n_1 - n_2) / (n_1 + n_2), 2.0f);
		// r = R0 + (1.0f - R0) * powf(1.0f - cos_theta1, 5.0f);
	}

	// Trace rays
	Vector3 refl = TraceRay(makeSecondaryRay(hitPoint, reflectedRayDirection), n_1, depth + 1);
	Vector3 refr = Vector3(0.0f);
	if (!totalInternalReflection) {
		refr = TraceRay(makeSecondaryRay(hitPoint, refractedRayDirection), n_2, depth + 1);
		// Fallback na cubemap ak refr je úplne čierna a backgroundEnabled_
		if (refr == Vector3(0.0f) && backgroundEnabled_ && cubemap_ != nullptr) {
			Color3f bg = cubemap_->GetTexel(refractedRayDirection);
			refr = Vector3(bg.r, bg.g, bg.b);
		}
	}

	// Beer-Lambert attenuation len ak sme vo vnútri materiálu
	const float travelLength = (n_1 == 1.0f) ? 0.0f : Vector3{ ray.org_x, ray.org_y, ray.org_z }.EuclideanDistance(hitPoint);
	const Vector3 attenuation = t_b_l(travelLength, material.GetAttenuation());

	return (refl * r + refr * (1.0f - r)) * attenuation;
}

//=============================================================================
// VOLUMETRIC RENDERING DISPATCH
//=============================================================================

Vector4 RayTracer::VolumetricRender(const RTCRay& ray) {
	// Dispatcher: vybere mezi volumetrickym krochovanim a sphere-tracingem
	if (!sdfRenderer_) return Vector4(0.0f);
	return sdfRenderer_->render(ray, buildSdfContext(), rayMarching_);
}

//=============================================================================
// VOLUMETRICKE KROCHLOVANI PRES SDF (deleguje na SdfRenderer)
//=============================================================================

/// Front-to-back volumetricke krochlovani skrze SDF oblak.
/// @param tMax  Orizi krochlovani na tuto vzdalenost (pouziva se v COMBINED_SDF modu).
Vector4 RayTracer::VolumetricEffect(const RTCRay& ray, const float tMax)
{
	if (!sdfRenderer_) return Vector4(0.0f);
	return sdfRenderer_->volumetricEffect(ray, buildSdfContext(), tMax);
}

//=============================================================================
// SPHERE-TRACING (deleguje na SdfRenderer)
//=============================================================================

Vector4 RayTracer::SurfaceEffect(const RTCRay& ray)
{
	if (!sdfRenderer_) return Vector4(0.0f);
	return sdfRenderer_->surfaceEffect(ray, buildSdfContext());
}
//=============================================================================
// MAIN RAY TRACING ENGINE (Embree-based surface ray tracing)
//=============================================================================

Vector3 RayTracer::TraceRay(const RTCRay& ray, const float n_1, const int depth, const int maxDepth) {
	// Prevent infinite recursion
	if (depth >= maxDepth) {
		return Vector3(0.0f);
	}

	// Check if we have any surfaces to intersect with
	if (surfaces_.empty()) {
		// No surface geometry loaded - return background
		Vector3 directionVector{ ray.dir_x, ray.dir_y, ray.dir_z };
		if (backgroundEnabled_ && cubemap_) {
			Color3f backgroundColor = cubemap_->GetTexel(directionVector);
			return Vector3{ backgroundColor.r, backgroundColor.g, backgroundColor.b };
		}
		return backgroundColorVec_;
	}

	// Initialize hit structure for Embree
	RTCHit hit;
	hit.geomID = RTC_INVALID_GEOMETRY_ID;
	hit.primID = RTC_INVALID_GEOMETRY_ID;
	hit.Ng_x = 0.0f;  hit.Ng_y = 0.0f;  hit.Ng_z = 0.0f;

	// Combine ray and hit for Embree intersection
	RTCRayHit rayHit;
	rayHit.ray = ray;
	rayHit.hit = hit;

	// EMBREE INTERSECTION: Find closest triangle hit
	RTCIntersectContext context;
	rtcInitIntersectContext(&context);
	rtcIntersect1(scene_, &context, &rayHit);

	Vector3 directionVector{ rayHit.ray.dir_x, rayHit.ray.dir_y, rayHit.ray.dir_z };

	// Handle miss: No geometry hit
	if (rayHit.hit.geomID == RTC_INVALID_GEOMETRY_ID) {
		if (backgroundEnabled_ && cubemap_) {
			Color3f backgroundColor = cubemap_->GetTexel(directionVector);
			return Vector3{ backgroundColor.r, backgroundColor.g, backgroundColor.b };
		}
		return backgroundColorVec_;
	}

	// Calculate hit point from ray equation: P = O + t*D
	const Vector3 hitPoint{
		rayHit.ray.org_x + rayHit.ray.dir_x * rayHit.ray.tfar,
		rayHit.ray.org_y + rayHit.ray.dir_y * rayHit.ray.tfar,
		rayHit.ray.org_z + rayHit.ray.dir_z * rayHit.ray.tfar
	};

	// Retrieve geometry and material at hit point.
	// rtcGetGeometry is the fast (non-thread-safe) variant, correct here
	// because we are inside GetPixel which holds shared_lock, preventing
	// any concurrent scene teardown.
	RTCGeometry geometry = rtcGetGeometry(scene_, rayHit.hit.geomID);
	if (!geometry) return Vector3(1.0f, 0.0f, 1.0f); // geometry invalidated
	Material* material = (Material*)rtcGetGeometryUserData(geometry);
	if (!material) {
		// Fallback if no material found
		return Vector3(1.0f, 0.0f, 1.0f); // Magenta error color
	}

	// Interpolate normal vector at hit point using barycentric coordinates
	Normal3f normal;
	rtcInterpolate0(geometry, rayHit.hit.primID, rayHit.hit.u, rayHit.hit.v,
		RTC_BUFFER_TYPE_VERTEX_ATTRIBUTE, 0, &normal.x, 3);
	Vector3 normalVector{ normal.x, normal.y, normal.z };

	// Flip normal if it points away from the ray
	if (normalVector.DotProduct(directionVector) > 0.0f) {
		normalVector *= -1.0f;
	}

	// Interpolate texture coordinates at hit point
	Coord2f texCoord;
	rtcInterpolate0(geometry, rayHit.hit.primID, rayHit.hit.u, rayHit.hit.v,
		RTC_BUFFER_TYPE_VERTEX_ATTRIBUTE, 1, &texCoord.u, 2);

	// Apply appropriate shader based on material type
	switch (material->GetShader()) {
	case 1:
		return NormalShader(normalVector);
	case 2:
		return LambertShader(*material, texCoord, hitPoint, normalVector);
	case 3:
		return PhongShader(*material, texCoord, hitPoint, normalVector, directionVector, depth);
	case 4:
		return TransparentShader(ray, hitPoint, normalVector, directionVector, *material, n_1, depth);
	default:
		return PhongShader(*material, texCoord, hitPoint, normalVector, directionVector, depth);
	}
}

//=============================================================================
// MAIN RENDERING PIPELINE
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
			const float denom = 1.0f - b * b;             // sin²(angle)
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
		//   - Sphere in front of cloud  → march exits early (low vol.w), sphere at full brightness
		//   - Sphere behind cloud       → vol.w is large, sphere attenuated by remaining transmittance
		//   - No sphere hit             → tEmissive == FLT_MAX, behaves like ordinary background
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
		// More samples → lower variance (converges as O(1/√N)).
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
	return uiController_->build();
}

void RayTracer::UpdateCameraPosition() {
	// Y-up spherical coordinates:
	//   azimuth  = angle in XZ plane (horizontal orbit around Y axis)
	//   elevation = angle from XZ plane toward +Y (vertical)
	//   x = dist * cos(el) * cos(az)
	//   y = dist * sin(el)              ← Y is the vertical component
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
// SIMPLIFIED MODEL LOADING
//=============================================================================

void RayTracer::InitializeFixedSdfScene() {
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

bool RayTracer::LoadObjModel(const std::string& fileName, EntityAnimState* animOut) {
	std::cout << "[RAY TRACER] Loading OBJ model: " << fileName << std::endl;

	// Exclusive lock: blocks until every in-flight GetPixel() call releases its
	// shared_lock, guaranteeing no thread is mid-sample when we destroy assets.
	std::unique_lock<std::shared_mutex> reloadLock(sceneMutex_);

	// Clear existing surface models first
	ClearSurfaceModels();

	// Load new OBJ file
	std::vector<Surface*> newSurfaces;
	std::vector<Material*> newMaterials;

	const int noSurfaces = LoadOBJ(fileName.c_str(), newSurfaces, newMaterials);

	if (noSurfaces <= 0) {
		std::cerr << "[RAY TRACER ERROR] Failed to load model: " << fileName << std::endl;
		return false;
	}

	// ClearSurfaceModels() above already released the old scene and created a
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

		// Index buffer (always Embree-owned — indices never change)
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

		// Copy initial positions into baseVerts so UpdateEntityTransforms can
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

void RayTracer::ClearSurfaceModels() {
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
	materials_.clear();  // ~Material() sees all-null texture slots — safe

	std::cout << "[RAY TRACER] Surface models cleared" << std::endl;
}

void RayTracer::ClearScene()
{
	// Acquire an exclusive lock.  This BLOCKS until every GetPixel() call that
	// currently holds a shared_lock has returned, guaranteeing no thread is
	// mid-rtcIntersect1, mid-vklComputeSample, or mid-get_texture() when we
	// start freeing memory.  This is the single serialisation point that fixes:
	//   • Embree crash  (rtcore_geometry.h:305) — scene released under active render
	//   • VDB crash     (line ~1406)            — sampler released under active march
	//   • Texture crash (texture.cpp:57)        — Material dtors run under active shade
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

	std::cout << "[ClearScene] All scene assets torn down safely.\n";
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
// SCENE LOADING (SetS up lights, geometry and rendering mode per scene type)
//=============================================================================

/// Configures lights, rendering mode, and background for a given scene type.
/// This is the single entry-point for scene switching; call it from the UI
/// combo box or at startup.
void RayTracer::LoadPredefinedScene(SceneType type)
{
	currentScene_ = type;
	sceneTime_    = 0.0f;
	lights_.clear();

	// Clear any data-driven animation state so UpdateScene falls back to the
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
		// IMPORTANT: UpdateCameraPosition() always orbits around the world ORIGIN
		// (hardcoded viewAt = (0,0,0)).  We must use that SAME origin here so that
		// the azimuth/elevation stored in cameraAzimuth_/cameraElevation_ exactly
		// reproduce kRefPos when UpdateCameraPosition is first called on mouse drag,
		// preventing the violent camera snap.
		const Vector3 kRefPos(0.0f, 40.0f, -100.0f);
		const Vector3 kRefAt(0.0f, 0.0f, 0.0f);  // MUST match UpdateCameraPosition's hardcoded origin
		camera_.SetViewFrom(kRefPos);
		camera_.SetViewAt(kRefAt);
		cameraX_ = kRefPos.x;
		cameraY_ = kRefPos.y;
		cameraZ_ = kRefPos.z;
		// Reverse-project into the Y-up spherical coordinates used by UpdateCameraPosition:
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

//=============================================================================
// SCENE CONFIGURATION LOADER — .scn file integration
//=============================================================================

/// Maps the mode token string written in scenes.scn to the internal RenderingMode enum.
RenderingMode RayTracer::ModeFromString(const std::string& modeStr) const
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
RenderingMode RayTracer::ResolveActiveMode() const
{
	const bool hasMesh = !surfaces_.empty();
	const bool hasVdb  = HasVdbVolume();

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
///   2. Call the legacy LoadPredefinedScene to configure lights and camera
///      to sensible defaults matching the entity type.
///   3. Load actual file assets (OBJ / VDB).  SDF shapes need no file load.
///   4. Restore the rendering mode specified in the .scn file (LoadObjModel
///      hard-codes SURFACE_EMBREE, so we override it here).
void RayTracer::LoadSceneFromDescription(const SceneDescription& desc)
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
	// ClearScene() acquires sceneMutex_ as unique_lock, which blocks until
	// every in-flight GetPixel() shared_lock holder has returned.  This is the
	// only serialisation point needed to prevent Embree/VDB/texture crashes.
	// NOTE: LoadObjModel() and LoadVdbVolume() each take their own unique_lock
	// internally, so the lock is fully released before those calls are made.
	ClearScene();

	// --- Call legacy setup for Embree/VKL context + SDF volumetric shapes ---
	// This also clears activeLightDescs_ and lights_ (we rebuild them below).
	if (hasMesh)
		LoadPredefinedScene(SceneType::SCENE_CUSTOM_OBJ);
	else if (hasVdb)
		LoadPredefinedScene(SceneType::SCENE_CUSTOM_VDB);
	else
		LoadPredefinedScene(SceneType::SCENE_SHADERTOY_SDF);

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
	// LoadPredefinedScene installed above.
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
		// Store descriptors so UpdateScene can drive per-frame ORBIT animation.
		sceneManager_->getLightDescs() = desc.lights;
		std::cout << "[SceneLoader] " << lights_.size() << " light(s) applied from .scn\n";
	}
	// If no lights defined in .scn, the lights from LoadPredefinedScene remain
	// (activeLightDescs_ is empty → legacy animation path in UpdateScene).

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
				if (LoadObjModel(e.value, &anim))
					sceneManager_->getEntityAnims().push_back(std::move(anim));
				else
					std::cout << "[SceneLoader] Failed to load MESH: " << e.value << "\n";
			} else {
				if (!LoadObjModel(e.value))
					std::cout << "[SceneLoader] Failed to load MESH: " << e.value << "\n";
			}
		}
		else if (e.type == "VDB") {
			if (!LoadVdbVolume(e.value, "density"))
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

	// currentRenderingMode_ is now set each frame by ResolveActiveMode() in MoveCamera().

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
/// DATA-DRIVEN PATH (scenes.scn loaded via LoadSceneFromDescription):
///   Iterates over activeLightDescs_.  For each POINT light with animType=="ORBIT"
///   the position is updated in the XZ plane around the light's base position:
///     x = base_x + cos(t * speed + phase) * radius
///     z = base_z + sin(t * speed + phase) * radius
///     y = base_y  (unchanged — orbit is around the Y axis)
///   DIRECTIONAL lights and lights with no animType are left untouched.
///
/// LEGACY PATH (loaded via LoadPredefinedScene directly):
///   Preserves the original hardcoded ShaderToy SDF orbit for backward compat.
///
/// THREAD SAFETY: called from MoveCamera() on the main/UI thread, between
/// frames.  Render threads hold a shared_lock while reading lights_; this
/// function runs without a lock (same pattern as the pre-existing legacy path).
/// Single-float position writes on x86/x64 are effectively atomic, and the
/// update precedes the next frame's pixel dispatch.
///
/// @param time  Accumulated scene time in seconds.
void RayTracer::UpdateScene(const float time)
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
// ENTITY TRANSFORMACE -- deleguje na SceneManager
//=============================================================================

void RayTracer::UpdateEntityTransforms(const float time)
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