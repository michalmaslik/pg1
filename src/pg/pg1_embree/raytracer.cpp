#include "stdafx.h"
#include "raytracer.h"
#include "objloader.h"
#include "tutorials.h"
#include "utils.h"
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

	// Initialize camera orbital parameters
	Vector3 currentViewFrom = camera_.GetViewFrom();

	Vector3 direction = currentViewFrom - viewAt;
	cameraDistance_ = direction.L2Norm();
	cameraAzimuth_ = rad2deg(atan2(direction.y, direction.x));
	cameraElevation_ = rad2deg(asin(direction.z / cameraDistance_));

	while (cameraAzimuth_ < 0.0f) cameraAzimuth_ += 360.0f;

	cameraX_ = currentViewFrom.x;
	cameraY_ = currentViewFrom.y;
	cameraZ_ = currentViewFrom.z;

	// Initialize OpenVKL
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
		cubemap_ = new CubeMap(cubeMapFileNames);
		std::cout << "[RAY TRACER] Cubemap loaded successfully" << std::endl;
	}
	catch (const std::exception& e) {
		std::cerr << "[RAY TRACER WARNING] Failed to load cubemap: " << e.what() << std::endl;
		cubemap_ = nullptr;
		backgroundEnabled_ = false; // Disable background if cubemap failed to load
	}

	std::cout << "[RAY TRACER] Ray Tracer initialized successfully" << std::endl;
}

RayTracer::~RayTracer()
{
	// Cleanup OpenVKL resources first
	CleanupOpenVKL();

	// Cleanup fixed SDF scene
	if (fixedSdfScene_) {
		delete fixedSdfScene_;
		fixedSdfScene_ = nullptr;
	}

	// Clean volumetric shapes vector (now contains only the fixed scene)
	volumetricShapes_.clear();

	// Cleanup surface geometry and materials
	ClearSurfaceModels();

	// Cleanup environment map
	delete cubemap_;
	cubemap_ = nullptr;

	// Release Intel Embree resources
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
	// Load OBJ file - creates triangle meshes and materials
	const int noSurfaces = LoadOBJ(fileName.c_str(), surfaces_, materials_);

	// For each surface (triangle mesh) in the OBJ file
	for (auto surface : surfaces_)
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
	}
}

void RayTracer::LoadScene(
	const std::vector<ModelInfo>& models,
	const std::vector<Shape*>& shapes,
	const char* cubeMapFileNames[6])
{
	// Load environment map for background and reflections
	cubemap_ = new CubeMap(cubeMapFileNames);

	// Load all polygonal models (for surface ray tracing with Embree)
	for (const auto& model : models)
	{
		LoadModel(model.filePath, model.transform);
	}

	// Add procedural volumetric shapes (for SDF ray marching)
	for (const auto& shape : shapes)
	{
		volumetricShapes_.push_back(shape);
	}

	// Commit scene to build BVH acceleration structure
	rtcCommitScene(scene_);
}

//=============================================================================
// UTILITY FUNCTIONS
//=============================================================================

RTCRay MakeSecondaryRay(const Vector3& origin, const Vector3& dir) {
	RTCRay ray = RTCRay();
	ray.org_x = origin.x;  ray.org_y = origin.y;  ray.org_z = origin.z;
	ray.tnear = 0.001f;    // Small offset to avoid self-intersection

	ray.dir_x = dir.x;     ray.dir_y = dir.y;     ray.dir_z = dir.z;
	ray.time = 0.0f;       // Time for motion blur (not used)

	ray.tfar = FLT_MAX;    // Maximum ray distance

	ray.mask = 0;  ray.id = 0;  ray.flags = 0;
	return ray;
}

bool RayTracer::IsHitPointVisible(const Vector3& hitPoint, const Vector3& lightPoint) {
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

float GetLightAttenuation(const float distanceToLight, const float lightAttenuationFactor)
{
	// Distance-based light attenuation: 1/d^n
	return 1.0f / pow(distanceToLight, lightAttenuationFactor);
}

Vector3 GetAmbientLight()
{
	// Constant ambient illumination (slight reddish tint)
	return 1.2f * Vector3(0.03f, 0.018f, 0.018f);
}

Vector3 ComputeNormal(const Vector3& p, const Shape& shape) {
	// Compute SDF gradient using finite differences (central differences)
	const float eps = 0.001f; // Small offset for numerical differentiation

	float dx = shape.SDF(p + Vector3(eps, 0.0f, 0.0f)) - shape.SDF(p - Vector3(eps, 0.0f, 0.0f));
	float dy = shape.SDF(p + Vector3(0.0f, eps, 0.0f)) - shape.SDF(p - Vector3(0.0f, eps, 0.0f));
	float dz = shape.SDF(p + Vector3(0.0f, 0.0f, eps)) - shape.SDF(p - Vector3(0.0f, 0.0f, eps));

	// Return normalized gradient (surface normal)
	return Vector3(dx, dy, dz) / (2.0f * eps);
}

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

	// Get diffuse color from material or texture
	Vector3 diffuseColor = Vector3{ material.diffuse.x, material.diffuse.y, material.diffuse.z };
	Texture* diffuseTexture = material.get_texture(Material::kDiffuseMapSlot);
	if (diffuseTexture) {
		Color3f diffuseTexel = diffuseTexture->get_texel(texCoord.u, 1 - texCoord.v);
		diffuseColor.x = diffuseTexel.r;
		diffuseColor.y = diffuseTexel.g;
		diffuseColor.z = diffuseTexel.b;
	}

	const Vector3 omniLightPosition = light_.origin_;
	Vector3 l = omniLightPosition - hitPoint;
	l.Normalize();

	// Ambient light contribution
	outputColor += material.ambient;

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

	// Get diffuse color from material or texture
	Vector3 diffuseColor = Vector3{ material.diffuse.x, material.diffuse.y, material.diffuse.z };
	Texture* diffuseTexture = material.get_texture(Material::kDiffuseMapSlot);
	if (diffuseTexture) {
		Color3f diffuseTexel = diffuseTexture->get_texel(texCoord.u, 1 - texCoord.v);
		diffuseColor.x = diffuseTexel.r;
		diffuseColor.y = diffuseTexel.g;
		diffuseColor.z = diffuseTexel.b;
	}

	// Get specular color from material or texture
	Vector3 specularColor = Vector3{ material.specular.x, material.specular.y, material.specular.z };
	Texture* specularTexture = material.get_texture(Material::kSpecularMapSlot);
	if (specularTexture) {
		Color3f specularTexel = specularTexture->get_texel(texCoord.u, 1.0f - texCoord.v);
		specularColor.x = specularTexel.r;
		specularColor.y = specularTexel.g;
		specularColor.z = specularTexel.b;
	}

	const Vector3 omniLightPosition = light_.origin_;
	Vector3 l = omniLightPosition - hitPoint;
	l.Normalize();

	// Ambient light contribution
	outputColor += material.ambient;

	// Diffuse and specular lighting with shadow test
	if (IsHitPointVisible(hitPoint, omniLightPosition)) {
		// Diffuse lighting (Lambert)
		outputColor += diffuseColor * clamp(normalVector.DotProduct(l));

		// Specular lighting (Phong)
		Vector3 l_r = 2.0f * (l.DotProduct(normalVector) * normalVector) - l; // Reflection vector
		l_r.Normalize();

		// Trace secondary ray for indirect specular contribution
		RTCRay secondaryRay = MakeSecondaryRay(hitPoint, l);
		Vector3 l_i = TraceRay(secondaryRay, depth + 1.0f);

		// Phong specular: I_spec = I_light * k_s * max(0, R·V)^shininess
		outputColor += specularColor * powf(clamp(l_r.DotProduct(-directionVector)), material.shininess);
		outputColor += l_i * specularColor * 0.2f; // Indirect specular contribution
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

	float n_2 = material.ior;
	if (n_1 == material.ior) {
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
	Vector3 refl = TraceRay(MakeSecondaryRay(hitPoint, reflectedRayDirection), n_1, depth + 1);
	Vector3 refr = Vector3(0.0f);
	if (!totalInternalReflection) {
		refr = TraceRay(MakeSecondaryRay(hitPoint, refractedRayDirection), n_2, depth + 1);
		// Fallback na cubemap ak refr je úplne čierna a backgroundEnabled_
		if (refr == Vector3(0.0f) && backgroundEnabled_ && cubemap_ != nullptr) {
			Color3f bg = cubemap_->GetTexel(refractedRayDirection);
			refr = Vector3(bg.r, bg.g, bg.b);
		}
	}

	// Beer-Lambert attenuation len ak sme vo vnútri materiálu
	float travelLength = (n_1 == 1.0f) ? 0.0f : Vector3{ ray.org_x, ray.org_y, ray.org_z }.EuclideanDistance(hitPoint);
	Vector3 attenuation = t_b_l(travelLength, material.attenuation);

	return (refl * r + refr * (1.0f - r)) * attenuation;
}

//=============================================================================
// VOLUMETRIC RENDERING DISPATCH
//=============================================================================

Vector4 RayTracer::VolumetricRender(const RTCRay& ray) {
	// This method now only handles SDF-based volumetric rendering
	if (rayMarching_) {
		// RAY MARCHING: Trace through entire SDF volume accumulating color/opacity
		return VolumetricEffect(ray);
	}
	// SPHERE TRACING: Find first SDF surface intersection only  
	return SurfaceEffect(ray);
}

//=============================================================================
// VOLUMETRIC RAY MARCHING (True volumetric rendering)
//=============================================================================

Vector4 RayTracer::VolumetricEffect(const RTCRay& ray)
{
	/*
	 * VOLUMETRIC RAY MARCHING ALGORITHM:
	 * 1. March through volume with small constant steps
	 * 2. Sample SDF at each step to determine if inside volume
	 * 3. Apply Beer-Lambert law for light absorption
	 * 4. Accumulate light contribution from all light sources
	 * 5. Continue until ray exits volume or reaches maximum distance
	 */

	const int numLights = 1;

	Vector3 position(ray.org_x, ray.org_y, ray.org_z); // Current ray position
	Vector3 direction(ray.dir_x, ray.dir_y, ray.dir_z); // Ray direction
	direction.Normalize();

	float accumulatedOpacity = 1.0f;   // Track how much light gets through (1.0 = fully visible)
	Vector3 volumetricColor(0.0f);     // Accumulate emitted/scattered light

	// Ray marching loop - take small steps through volume
	for (int i = 0; i < maxSteps_; ++i) {
		// Stop if ray travels too far from origin
		if (position.L2Norm() > maxDistance_) break;

		// Get volumetric shape (assume first shape for now)
		if (volumetricShapes_.empty()) break;
		Shape* shape = volumetricShapes_[0];
		float sdfValue = shape->SDF(position);

		// Check if ray is inside the volume (SDF < 0 means inside)
		if (sdfValue < 0.1f) {
			float previousOpacity = accumulatedOpacity;

			// BEER-LAMBERT LAW: Light absorption in participating medium
			// T(s) = e^(-σ_a * s) where σ_a is absorption coefficient, s is distance
			accumulatedOpacity *= exp(-absorptionCoefficient_ * stepSize_);

			// Calculate how much light was absorbed in this step
			float absorptionFromMarch = previousOpacity - accumulatedOpacity;

			// Add light contribution from each light source
			for (int lightIndex = 0; lightIndex < numLights; ++lightIndex) {
				Vector3 lightPosition = light_.origin_;
				Vector3 lightDirection = lightPosition - position;
				float lightDistance = lightDirection.L2Norm();
				lightDirection.Normalize();

				// Calculate light attenuation with distance
				Vector3 lightColor = Vector3(1.0f) * GetLightAttenuation(lightDistance, lightAttenuationFactor_);

				// Add light contribution: absorbed_light * albedo * light_intensity
				volumetricColor += absorptionFromMarch * volumetricAlbedoVec_ * lightColor;
			}

			// Add ambient light contribution
			volumetricColor += absorptionFromMarch * volumetricAlbedoVec_ * GetAmbientLight();
		}

		// Advance ray position by constant step size
		position += direction * stepSize_;
	}

	// Return accumulated color and final opacity
	return Vector4(volumetricColor, 1.0f - accumulatedOpacity);
}

//=============================================================================
// SDF SPHERE TRACING (Surface rendering for SDF shapes)
//=============================================================================

Vector4 RayTracer::SurfaceEffect(const RTCRay& ray)
{
	/*
	 * SPHERE TRACING ALGORITHM:
	 * 1. Use SDF value as safe step distance (won't overshoot surface)
	 * 2. March ray by SDF distance at each step
	 * 3. Stop when very close to surface (SDF ≈ 0)
	 * 4. Apply surface shading (Phong model)
	 * 5. Return single surface color with full opacity
	 */

	Vector3 position(ray.org_x, ray.org_y, ray.org_z);
	Vector3 direction(ray.dir_x, ray.dir_y, ray.dir_z);
	direction.Normalize();

	// Sphere tracing loop
	for (int i = 0; i < 1000; ++i) {
		if (position.L2Norm() > maxDistance_) break;

		if (volumetricShapes_.empty()) break;
		Shape* shape = volumetricShapes_[0];
		float sdfValue = shape->SDF(position);

		// Check if we hit the surface (SDF very close to zero)
		if (sdfValue < 0.001f) {
			// Compute surface normal using SDF gradient
			Vector3 normal = ComputeNormal(position, *shape);
			normal.Normalize();

			Vector3 lightDir = light_.origin_ - position;
			lightDir.Normalize();

			// PHONG LIGHTING MODEL
			// Ambient component
			Vector3 ambientColor(0.025f, 0.025f, 0.025f);
			Vector3 color = ambientColor;

			// Diffuse component: Lambert's Law
			float diffuse = std::max(0.0f, normal.DotProduct(lightDir));
			if (IsHitPointVisible(position, light_.origin_)) {
				color += Vector3(diffuse);
			}

			// Specular component: Phong reflection model
			Vector3 viewDir = -direction;
			Vector3 reflectDir = 2.0f * normal.DotProduct(lightDir) * normal - lightDir;
			float spec = powf(std::max(viewDir.DotProduct(reflectDir), 0.0f), 32.0f); // Shininess = 32
			color += Vector3(spec);

			// Return surface color with full opacity (solid surface)
			return Vector4(color, 1.0f);
		}

		// SPHERE TRACING: Step by SDF distance (safe - won't overshoot surface)
		position += direction * sdfValue;
	}

	// No surface hit - return transparent
	return Vector4(0.0f);
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

	// Retrieve geometry and material at hit point
	RTCGeometry geometry = rtcGetGeometry(scene_, rayHit.hit.geomID);
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
	switch (material->shader) {
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

	// Return final color compressed to [0,1] range
	return Color4f{ finalColor.x, finalColor.y, finalColor.z, finalAlpha };
}

Vector4 RayTracer::RenderPixel(const RTCRay& ray) {
	switch (currentRenderingMode_) {
	case RenderingMode::SURFACE_EMBREE: {
		// Traditional Embree surface rendering
		Vector3 surfaceColor = TraceRay(ray);
		return Vector4(surfaceColor.x, surfaceColor.y, surfaceColor.z, 1.0f);
	}

	case RenderingMode::VOLUMETRIC_SDF: {
		// SDF-based volumetric rendering
		Vector4 volumetricColor = VolumetricRender(ray);

		// If volume is transparent, blend with background
		if (volumetricColor.w < 1.0f) {
			Vector3 backgroundColor;
			if (backgroundEnabled_) {
				Vector3 direction(ray.dir_x, ray.dir_y, ray.dir_z);
				Color3f bgColor = cubemap_->GetTexel(direction);
				backgroundColor = Vector3(bgColor.r, bgColor.g, bgColor.b);
			}
			else {
				backgroundColor = backgroundColorVec_;
			}

			// Alpha blend
			Vector3 finalColor = Vector3(volumetricColor.x, volumetricColor.y, volumetricColor.z) * volumetricColor.w +
				backgroundColor * (1.0f - volumetricColor.w);

			return Vector4(finalColor.x, finalColor.y, finalColor.z, 1.0f);
		}

		return volumetricColor;
	}

	case RenderingMode::VOLUMETRIC_VDB: {
		// VDB volumetric rendering
		return VdbVolumeRayMarching(ray);
	}

	default: {
		// Error case - return background
		if (backgroundEnabled_) {
			Vector3 direction(ray.dir_x, ray.dir_y, ray.dir_z);
			Color3f backgroundColor = cubemap_->GetTexel(direction);
			return Vector4(backgroundColor.r, backgroundColor.g, backgroundColor.b, 1.0f);
		}
		return Vector4(backgroundColorVec_.x, backgroundColorVec_.y, backgroundColorVec_.z, 1.0f);
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

		// Calculate orbital radius in x-y plane
		Vector3 direction = viewFrom - viewAt;
		float currentRadius = sqrt(direction.x * direction.x + direction.y * direction.y);

		// Update camera position in circular orbit
		cameraX_ = viewAt.x + currentRadius * cosf(angle);
		cameraY_ = viewAt.y + currentRadius * sinf(angle);
		cameraZ_ = viewFrom.z; // Maintain original height

		// Increment angle for next frame
		angle += speed;
		if (angle > 2.0f * (float)M_PI) {
			angle -= 2.0f * (float)M_PI;
		}

		// Update camera position
		camera_.SetViewFrom(Vector3(cameraX_, cameraY_, cameraZ_));
	}
}

//=============================================================================
// USER INTERFACE
//=============================================================================


int RayTracer::Ui()
{
	static float f = 0.0f;
	static int counter = 0;

	ImGui::Begin("Ray Tracer Control Panel");

	// === HEADER INFO ===
	ImGui::Text("Surfaces: %d | Materials: %d", surfaces_.size(), materials_.size());

	// Display correct FPS
	float currentFPS = GetCurrentFPS(); // This will be 0 when paused
	if (currentFPS > 0.0f) {
		ImGui::Text("FPS: %.1f (%.3f ms/frame)", currentFPS, 1000.0f / currentFPS);
	}
	else {
		ImGui::Text("FPS: 0.0 (PAUSED)");
	}
	ImGui::Separator();

	// === RENDERING CONTROL ===
	if (ImGui::CollapsingHeader("Rendering Control", ImGuiTreeNodeFlags_DefaultOpen)) {
		// Main render control buttons
		ImGui::PushStyleColor(ImGuiCol_Button, renderingPaused_ ? ImVec4(0.0f, 0.7f, 0.0f, 1.0f) : ImVec4(0.7f, 0.0f, 0.0f, 1.0f));
		if (ImGui::Button(renderingPaused_ ? "▶ RESUME" : "⏸ PAUSE", ImVec2(100, 30))) {
			renderingPaused_ = !renderingPaused_;
			if (renderingPaused_) {
				PauseRendering();
			}
			else {
				ResumeRendering();
			}
		}
		ImGui::PopStyleColor();

		ImGui::SameLine();
		if (renderingPaused_) {
			if (ImGui::Button("🖼 RENDER FRAME", ImVec2(120, 30))) {
				RequestSingleFrame();
			}
		}

		ImGui::SameLine();
		ImGui::Checkbox("Auto Camera Rotation", &autoRotateCamera_);
		cameraMovementEnabled_ = autoRotateCamera_ && !renderingPaused_;

		ImGui::SameLine();
		ImGui::Checkbox("Mouse Camera Input", &mouseCameraInput_);

		if (mouseCameraInput_) {
			// === MOUSE INTERACTION HANDLING ===
// Handle mouse input for camera control
			ImVec2 mousePos = ImGui::GetMousePos();
			ImVec2 mouseDelta = ImVec2(mousePos.x - lastMousePos_.x, mousePos.y - lastMousePos_.y);

			// Left mouse button (orbital rotation)
			if (ImGui::IsMouseDown(0)) {
				if (!leftMouseDragging_) {
					leftMouseDragging_ = true;
					lastMousePos_ = mousePos;
				}
				else {
					HandleLeftMouseDrag(mouseDelta);
				}
			}
			else {
				leftMouseDragging_ = false;
			}

			// Right mouse button (zoom)
			if (ImGui::IsMouseDown(1)) {
				if (!rightMouseDragging_) {
					rightMouseDragging_ = true;
					lastMousePos_ = mousePos;
				}
				else {
					HandleRightMouseDrag(mouseDelta);
				}
			}
			else {
				rightMouseDragging_ = false;
			}

			lastMousePos_ = mousePos;
		}


		ImGui::Separator();

		// RENDERING MODE SELECTION
		ImGui::Text("🎨 Rendering Mode:");

		if (ImGui::RadioButton("Surface (Embree)", currentRenderingMode_ == RenderingMode::SURFACE_EMBREE)) {
			currentRenderingMode_ = RenderingMode::SURFACE_EMBREE;
		}
		ImGui::SameLine();
		if (ImGui::RadioButton("Volumetric SDF", currentRenderingMode_ == RenderingMode::VOLUMETRIC_SDF)) {
			currentRenderingMode_ = RenderingMode::VOLUMETRIC_SDF;
		}
		ImGui::SameLine();
		if (ImGui::RadioButton("VDB Volume", currentRenderingMode_ == RenderingMode::VOLUMETRIC_VDB)) {
			currentRenderingMode_ = RenderingMode::VOLUMETRIC_VDB;
			if (!HasVdbVolume()) {
				ImGui::SameLine();
				ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.0f, 1.0f), "(No VDB loaded!)");
			}
		}

		ImGui::Separator();

		ImGui::Checkbox("VSync", &vsync_);
		ImGui::SameLine();
		ImGui::Checkbox("Anti-aliasing (4x4)", &sampling_);
	}

	// === SURFACE MODEL LOADING === (pridať do UI za VDB sekciu)
	if (ImGui::CollapsingHeader("Surface Model Loading")) {
		ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.7f, 1.0f, 0.3f, 1.0f));
		ImGui::Text("🎯 Triangle Mesh Models (OBJ)");
		ImGui::PopStyleColor();

		static char objFilePath[512] = "C:\\Users\\micha\\Documents\\pg1\\data\\6887_allied_avenger.obj";
		ImGui::InputText("OBJ File Path", objFilePath, sizeof(objFilePath));

		if (ImGui::Button("📁 Load OBJ Model", ImVec2(150, 25))) {
			if (LoadObjModel(std::string(objFilePath))) {
				std::cout << "[UI] OBJ model loaded successfully!" << std::endl;
			}
			else {
				std::cout << "[UI ERROR] Failed to load OBJ model!" << std::endl;
			}
		}

		ImGui::SameLine();
		if (ImGui::Button("🗑 Clear Models", ImVec2(100, 25))) {
			ClearSurfaceModels();
			// Recreate empty scene
			rtcReleaseScene(scene_);
			scene_ = rtcNewScene(device_);
			rtcCommitScene(scene_);
		}

		// Model status
		int modelCount = GetLoadedModelCount();
		if (modelCount > 0) {
			ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.0f, 1.0f, 0.0f, 1.0f));
			ImGui::Text("✅ Loaded Models: %d surfaces", modelCount);
			ImGui::PopStyleColor();
		}
		else {
			ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.5f, 0.0f, 1.0f));
			ImGui::Text("⚠ No surface models loaded");
			ImGui::PopStyleColor();
		}
	}

	// === VDB VOLUME CONTROLS ===
	if (ImGui::CollapsingHeader("VDB Volume Loading")) {
		ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.3f, 0.7f, 1.0f, 1.0f));
		ImGui::Text("🗃 OpenVDB File Management");
		ImGui::PopStyleColor();

		ImGui::InputText("VDB File Path", vdbFilePath_, sizeof(vdbFilePath_));
		ImGui::InputText("Grid Name", vdbGridName_, sizeof(vdbGridName_));

		if (ImGui::Button("📁 Load VDB Volume", ImVec2(150, 25))) {
			if (LoadVdbVolume(std::string(vdbFilePath_), std::string(vdbGridName_))) {
				std::cout << "[VDB] Volume loaded successfully!" << std::endl;
			}
			else {
				std::cout << "[VDB ERROR] Failed to load VDB volume!" << std::endl;
			}
		}

		ImGui::SameLine();
		if (ImGui::Button("🗑 Clear VDB", ImVec2(100, 25))) {
			CleanupOpenVKL();
			InitializeOpenVKL();
		}

		// VDB Status Display
		if (HasVdbVolume()) {
			ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.0f, 1.0f, 0.0f, 1.0f));
			ImGui::Text("✅ VDB Volume: LOADED");
			ImGui::PopStyleColor();

			Vector3 minBounds, maxBounds;
			GetVdbVolumeBounds(minBounds, maxBounds);
			ImGui::Text("📏 Bounds: min(%.2f,%.2f,%.2f) max(%.2f,%.2f,%.2f)",
				minBounds.x, minBounds.y, minBounds.z,
				maxBounds.x, maxBounds.y, maxBounds.z);

			if (vdbLoader_) {
				ImGui::Text("📊 Active Voxels: %zu", vdbLoader_->GetActiveVoxelCount());
				ImGui::Text("📈 Value Range: [%.3f, %.3f]", vdbLoader_->GetMinValue(), vdbLoader_->GetMaxValue());
			}
		}
		else {
			ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.5f, 0.0f, 1.0f));
			ImGui::Text("⚠ VDB Volume: NOT LOADED");
			ImGui::PopStyleColor();
		}
	}

	// === RAY MARCHING CONTROLS ===
	if (ImGui::CollapsingHeader("Volumetric Ray Marching")) {
		ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.7f, 0.3f, 1.0f));
		ImGui::Text("🔬 SDF & Volumetric Rendering");
		ImGui::PopStyleColor();

		// Rendering mode toggle
		ImGui::Text("Rendering Mode:");
		if (ImGui::RadioButton("Surface Mode (Sphere Tracing)", !rayMarching_)) {
			rayMarching_ = false;
		}
		if (ImGui::RadioButton("Volume Mode (Ray Marching)", rayMarching_)) {
			rayMarching_ = true;
		}

		ImGui::Separator();

		// Ray marching parameters
		if (ImGui::SliderFloat("Step Size", &stepSize_, 0.001f, 1.0f, "%.4f")) {
			maxSteps_ = static_cast<int>(maxDistance_ / stepSize_);
		}
		if (ImGui::SliderFloat("Max Distance", &maxDistance_, 0.0f, 1000000.0f)) {
			maxSteps_ = static_cast<int>(maxDistance_ / stepSize_);
		}
		ImGui::Text("Max Steps: %d", maxSteps_);

		ImGui::SliderFloat("Absorption Coeff.", &absorptionCoefficient_, 0.0f, 3.0f);
		ImGui::SliderFloat("Light Attenuation", &lightAttenuationFactor_, 0.0f, 3.0f);

		if (ImGui::ColorEdit3("Volumetric Albedo", volumetricAlbedo_)) {
			volumetricAlbedoVec_ = Vector3(volumetricAlbedo_[0], volumetricAlbedo_[1], volumetricAlbedo_[2]);
		}

		// SDF Noise controls
		ImGui::Separator();
		ImGui::Text("🌊 Procedural Noise (SDF only):");
		if (ImGui::Checkbox("Enable Noise", &useNoise_)) {
			Shape::useNoise = useNoise_;
		}
		if (ImGui::SliderFloat("Noise Scale", &noiseScale_, 0.0f, 1.0f)) {
			for (Shape* shape : volumetricShapes_) {
				shape->noise_.SetScale(noiseScale_);
			}
		}
		if (ImGui::SliderFloat("Noise Strength", &noiseStrength_, 0.0f, 10.0f)) {
			for (Shape* shape : volumetricShapes_) {
				shape->noise_.SetStrength(noiseStrength_);
			}
		}
		if (ImGui::SliderFloat("SDF Smooth Factor", &smoothFactor_, 0.0f, 3.0f)) {
			for (Shape* shape : volumetricShapes_) {
				if (auto* smoothUnion = dynamic_cast<SmoothUnion*>(shape)) {
					for (Shape* subShape : smoothUnion->shapes_) {
						subShape->k_ = smoothFactor_;
					}
				}
				else {
					shape->k_ = smoothFactor_;
				}
			}
		}
	}

	// === CAMERA CONTROLS ===
	if (ImGui::CollapsingHeader("Camera Controls")) {
		ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.7f, 0.7f, 1.0f, 1.0f));
		ImGui::Text("📷 Interactive Camera Control");
		ImGui::PopStyleColor();

		ImGui::Text("🖱 Mouse Controls:");
		ImGui::BulletText("Left Mouse + Drag: Rotate around scene");
		ImGui::BulletText("Right Mouse + Drag: Zoom in/out");

		ImGui::Separator();

		// Current camera info
		ImGui::Text("Camera Position: (%.2f, %.2f, %.2f)", cameraX_, cameraY_, cameraZ_);
		ImGui::Text("Distance from center: %.2f", cameraDistance_);
		ImGui::Text("Azimuth: %.1f°, Elevation: %.1f°", cameraAzimuth_, cameraElevation_);

		ImGui::Separator();

		// Manual controls
		ImGui::Text("Manual Controls:");
		if (ImGui::SliderFloat("Distance", &cameraDistance_, minCameraDistance_, maxCameraDistance_)) {
			UpdateCameraPosition();
		}
		if (ImGui::SliderFloat("Azimuth", &cameraAzimuth_, 0.0f, 360.0f)) {
			UpdateCameraPosition();
		}
		if (ImGui::SliderFloat("Elevation", &cameraElevation_, -maxElevation_, maxElevation_)) {
			UpdateCameraPosition();
		}

		ImGui::Separator();

		// Reset button
		if (ImGui::Button("🎯 Reset Camera", ImVec2(120, 25))) {
			cameraDistance_ = 20.0f;
			cameraAzimuth_ = 0.0f;
			cameraElevation_ = 0.0f;
			UpdateCameraPosition();
		}
	}

	// === LIGHTING CONTROLS ===
	if (ImGui::CollapsingHeader("Lighting & Background")) {
		ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 1.0f, 0.3f, 1.0f));
		ImGui::Text("💡 Scene Lighting");
		ImGui::PopStyleColor();

		// Light position
		static int lightX = (int)light_.origin_.x;
		static int lightY = (int)light_.origin_.y;
		static int lightZ = (int)light_.origin_.z;
		if (ImGui::SliderInt("Light X", &lightX, -100, 100)) {
			light_.origin_ = Vector3(static_cast<float>(lightX), static_cast<float>(lightY), static_cast<float>(lightZ));
		}
		if (ImGui::SliderInt("Light Y", &lightY, -100, 100)) {
			light_.origin_ = Vector3(static_cast<float>(lightX), static_cast<float>(lightY), static_cast<float>(lightZ));
		}
		if (ImGui::SliderInt("Light Z", &lightZ, -100, 100)) {
			light_.origin_ = Vector3(static_cast<float>(lightX), static_cast<float>(lightY), static_cast<float>(lightZ));
		}

		ImGui::Separator();

		// Background settings
		ImGui::Text("🌌 Background:");
		ImGui::Checkbox("Use Cubemap Background", &backgroundEnabled_);
		if (ImGui::ColorEdit3("Background Color", backgroundColor_)) {
			backgroundColorVec_ = Vector3(backgroundColor_[0], backgroundColor_[1], backgroundColor_[2]);
		}
	}

	// === VIDEO EXPORT ===
	if (ImGui::CollapsingHeader("Video Export")) {
		ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.3f, 0.3f, 1.0f));
		ImGui::Text("🎬 Video Creation");
		ImGui::PopStyleColor();

		ImGui::Text("Frames captured: %d", frameCount_);

		if (ImGui::Button("💾 Save Video and Exit", ImVec2(200, 30))) {
			cv::VideoWriter writer("frames/output.avi",
				cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
				60,
				cv::Size(width(), height()));

			if (writer.isOpened()) {
				for (int i = 0; i < frameCount_; ++i) {
					std::stringstream filename;
					filename << "frames/frame_" << std::setw(6) << std::setfill('0') << i << ".ppm";

					cv::Mat frame = cv::imread(filename.str());
					if (!frame.empty()) {
						writer.write(frame);
					}
				}
				writer.release();
			}

			std::cout << "Video successfully created: frames/output.avi" << std::endl;
			MessageBoxA(NULL, "Video saved successfully", "Success", MB_OK);
			PostQuitMessage(0);
		}
	}

	ImGui::End();
	return 0;
}

//=============================================================================
// OPENVKL VDB INITIALIZATION
//=============================================================================

bool RayTracer::InitializeOpenVKL() {
	std::cout << "[VKL] Initializing OpenVKL..." << std::endl;

	// Initialize OpenVKL
	vklInit();

	// Create CPU device
	vklDevice_ = vklNewDevice("cpu");
	if (!vklDevice_) {
		std::cerr << "[VKL ERROR] Failed to create OpenVKL CPU device!" << std::endl;
		return false;
	}

	// Set device parameters
	vklDeviceSetInt(vklDevice_, "logLevel", VKL_LOG_WARNING);
	vklDeviceSetString(vklDevice_, "logOutput", "cout");
	vklDeviceSetString(vklDevice_, "errorOutput", "cerr");

	// Commit device
	vklCommitDevice(vklDevice_);

	// Create VDB loader
	vdbLoader_ = std::make_unique<VdbLoader>();

	std::cout << "[VKL] OpenVKL initialized successfully" << std::endl;
	return true;
}

bool RayTracer::LoadVdbVolume(const std::string& filename, const std::string& gridName) {
	if (!vklDevice_) {
		std::cerr << "[VKL ERROR] OpenVKL not initialized! Call InitializeOpenVKL() first." << std::endl;
		return false;
	}

	// Clean up previous volume
	if (vklSampler_) {
		vklRelease(vklSampler_);
		vklSampler_ = VKLSampler{};
	}
	if (vklVolume_) {
		vklRelease(vklVolume_);
		vklVolume_ = VKLVolume{};
	}

	// Load VDB volume
	vklVolume_ = vdbLoader_->LoadVdbFile(filename, gridName, vklDevice_);
	if (!vklVolume_) {
		std::cerr << "[VKL ERROR] Failed to load VDB volume from: " << filename << std::endl;
		return false;
	}

	// Create sampler
	vklSampler_ = vklNewSampler(vklVolume_);
	if (!vklSampler_) {
		std::cerr << "[VKL ERROR] Failed to create VKL sampler!" << std::endl;
		vklRelease(vklVolume_);
		vklVolume_ = VKLVolume{};
		return false;
	}

	// Configure sampler
	vklSetInt(vklSampler_, "filter", VKL_FILTER_LINEAR);
	vklSetInt(vklSampler_, "gradientFilter", VKL_FILTER_LINEAR);
	vklCommit(vklSampler_);

	// Get volume info
	vkl_box3f volumeBounds = vklGetBoundingBox(vklVolume_);
	vkl_range1f valueRange = vklGetValueRange(vklVolume_, 0);

	std::cout << "[VKL] Volume bounds: min(" << volumeBounds.lower.x << "," << volumeBounds.lower.y << "," << volumeBounds.lower.z << ")"
		<< " max(" << volumeBounds.upper.x << "," << volumeBounds.upper.y << "," << volumeBounds.upper.z << ")" << std::endl;
	std::cout << "[VKL] Value range: [" << valueRange.lower << ", " << valueRange.upper << "]" << std::endl;

	return true;
}

void RayTracer::CleanupOpenVKL() {
	if (vklSampler_) {
		vklRelease(vklSampler_);
		vklSampler_ = VKLSampler{};
	}
	if (vklVolume_) {
		vklRelease(vklVolume_);
		vklVolume_ = VKLVolume{};
	}
	if (vklDevice_) {
		vklReleaseDevice(vklDevice_);
		vklDevice_ = VKLDevice{};
	}

	vdbLoader_.reset();
	std::cout << "[VKL] OpenVKL cleanup completed" << std::endl;
}

void RayTracer::GetVdbVolumeBounds(Vector3& minBounds, Vector3& maxBounds) const {
	if (vdbLoader_) {
		minBounds = vdbLoader_->GetBoundingBoxMin();
		maxBounds = vdbLoader_->GetBoundingBoxMax();
	}
	else {
		minBounds = Vector3(0.0f, 0.0f, 0.0f);
		maxBounds = Vector3(1.0f, 1.0f, 1.0f);
	}
}

//=============================================================================
// VDB VOLUME RAY MARCHING
//=============================================================================

Vector4 RayTracer::VdbVolumeRayMarching(const RTCRay& ray) {
	if (!vklSampler_) {
		// Return background if no VDB volume loaded
		if (backgroundEnabled_) {
			Vector3 direction(ray.dir_x, ray.dir_y, ray.dir_z);
			Color3f backgroundColor = cubemap_->GetTexel(direction);
			return Vector4(backgroundColor.r, backgroundColor.g, backgroundColor.b, 1.0f);
		}
		return Vector4(backgroundColorVec_.x, backgroundColorVec_.y, backgroundColorVec_.z, 1.0f);
	}

	Vector3 rayOrigin(ray.org_x, ray.org_y, ray.org_z);
	Vector3 rayDirection(ray.dir_x, ray.dir_y, ray.dir_z);
	rayDirection.Normalize();

	// Get volume bounds from OpenVKL
	vkl_box3f volumeBounds = vklGetBoundingBox(vklVolume_);
	Vector3 minBounds(volumeBounds.lower.x, volumeBounds.lower.y, volumeBounds.lower.z);
	Vector3 maxBounds(volumeBounds.upper.x, volumeBounds.upper.y, volumeBounds.upper.z);

	// Calculate ray-box intersection
	Vector3 invDir(
		rayDirection.x != 0.0f ? 1.0f / rayDirection.x : FLT_MAX,
		rayDirection.y != 0.0f ? 1.0f / rayDirection.y : FLT_MAX,
		rayDirection.z != 0.0f ? 1.0f / rayDirection.z : FLT_MAX
	);

	Vector3 t1 = (minBounds - rayOrigin) * invDir;
	Vector3 t2 = (maxBounds - rayOrigin) * invDir;

	Vector3 tMin(std::min(t1.x, t2.x), std::min(t1.y, t2.y), std::min(t1.z, t2.z));
	Vector3 tMax(std::max(t1.x, t2.x), std::max(t1.y, t2.y), std::max(t1.z, t2.z));

	float tNear = std::max({ tMin.x, tMin.y, tMin.z, 0.001f });
	float tFar = std::min({ tMax.x, tMax.y, tMax.z });

	if (tNear >= tFar) {
		// No intersection with volume - return background
		if (backgroundEnabled_) {
			Vector3 direction(ray.dir_x, ray.dir_y, ray.dir_z);
			Color3f backgroundColor = cubemap_->GetTexel(direction);
			return Vector4(backgroundColor.r, backgroundColor.g, backgroundColor.b, 1.0f);
		}
		return Vector4(backgroundColorVec_.x, backgroundColorVec_.y, backgroundColorVec_.z, 1.0f);
	}

	// VDB Ray marching parameters
	const float stepSize = stepSize_;
	const unsigned int attributeIndex = 0;
	const float time = 0.0f;

	// Volumetric rendering variables
	float accumulatedOpacity = 1.0f;
	Vector3 accumulatedColor(0.0f, 0.0f, 0.0f);

	// Get value range for better visualization
	vkl_range1f valueRange = vklGetValueRange(vklVolume_, attributeIndex);
	float maxDensity = valueRange.upper;
	if (maxDensity <= 0.0f) maxDensity = 1.0f;

	// Basic ray marching loop through VDB volume
	int steps = 0;
	const int maxSteps = static_cast<int>((tFar - tNear) / stepSize) + 1;

	for (float t = tNear; t < tFar && accumulatedOpacity > 0.01f && steps < maxSteps; t += stepSize, steps++) {
		Vector3 currentPos = rayOrigin + rayDirection * t;
		vkl_vec3f vklPos = { currentPos.x, currentPos.y, currentPos.z };

		// Sample VDB volume using OpenVKL
		float density = vklComputeSample(&vklSampler_, &vklPos, attributeIndex, time);

		if (density > 0.0f) {
			// Normalize density for visualization  
			float normalizedDensity = std::min(density / maxDensity, 1.0f);

			// Beer-Lambert absorption
			float absorption = normalizedDensity * absorptionCoefficient_ * stepSize;
			float transmittance = exp(-absorption);

			float previousOpacity = accumulatedOpacity;
			accumulatedOpacity *= transmittance;
			float absorptionFromMarch = previousOpacity - accumulatedOpacity;

			// Simple lighting calculation
			Vector3 lightDir = light_.origin_ - currentPos;
			float lightDistance = lightDir.L2Norm();
			lightDir.Normalize();

			Vector3 lightColor = Vector3(1.0f) * GetLightAttenuation(lightDistance, lightAttenuationFactor_);
			Vector3 volumeColor = volumetricAlbedoVec_ * lightColor;
			volumeColor += GetAmbientLight() * volumetricAlbedoVec_;

			// Accumulate color
			accumulatedColor += absorptionFromMarch * volumeColor * normalizedDensity;
		}
	}

	float finalOpacity = 1.0f - accumulatedOpacity;

	// If volume is transparent, blend with background
	if (finalOpacity < 1.0f) {
		Vector3 backgroundColor;
		if (backgroundEnabled_) {
			Vector3 direction(ray.dir_x, ray.dir_y, ray.dir_z);
			Color3f bgColor = cubemap_->GetTexel(direction);
			backgroundColor = Vector3(bgColor.r, bgColor.g, bgColor.b);
		}
		else {
			backgroundColor = backgroundColorVec_;
		}

		// Alpha blend: C_final = C_volume * alpha + C_background * (1 - alpha)
		accumulatedColor = accumulatedColor * finalOpacity + backgroundColor * (1.0f - finalOpacity);
		finalOpacity = 1.0f; // Final result is opaque
	}

	// Clamp colors
	accumulatedColor.x = std::min(accumulatedColor.x, 1.0f);
	accumulatedColor.y = std::min(accumulatedColor.y, 1.0f);
	accumulatedColor.z = std::min(accumulatedColor.z, 1.0f);

	return Vector4(accumulatedColor.x, accumulatedColor.y, accumulatedColor.z, finalOpacity);
}

//=============================================================================
// CAMERA CONTROL IMPLEMENTATION
//=============================================================================

void RayTracer::UpdateCameraPosition() {
	// Convert spherical coordinates to Cartesian coordinates
	float azimuthRad = deg2rad(cameraAzimuth_);
	float elevationRad = deg2rad(cameraElevation_);

	// Calculate camera position relative to viewAt (0,0,0)
	Vector3 viewAt(0.0f, 0.0f, 0.0f);

	float x = cameraDistance_ * cos(elevationRad) * cos(azimuthRad);
	float y = cameraDistance_ * cos(elevationRad) * sin(azimuthRad);
	float z = cameraDistance_ * sin(elevationRad);

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
	// Create fixed SDF scene that won't change during runtime
	fixedSdfScene_ = new SmoothUnion({
		new Sphere(Vector3(0.0f, 0.0f, 1.0f), 4.0f, 1.0f),
		new Sphere(Vector3(-8.0f, 2.0f, -1.0f), 5.6f, 1.0f),
		new Plane(0.01f)
		}, Noise(Noise::NoiseType::FractalBrownianMotion, 1.0f / 3.2f, 7.0f));

	volumetricShapes_.push_back(fixedSdfScene_);

	std::cout << "[RAY TRACER] Fixed SDF scene initialized" << std::endl;
}

bool RayTracer::LoadObjModel(const std::string& fileName) {
	std::cout << "[RAY TRACER] Loading OBJ model: " << fileName << std::endl;

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

	// Create new Embree scene
	rtcReleaseScene(scene_);
	scene_ = rtcNewScene(device_);

	// Add loaded surfaces to the new scene
	for (auto surface : newSurfaces) {
		// Create Embree geometry for this surface
		RTCGeometry mesh = rtcNewGeometry(device_, RTC_GEOMETRY_TYPE_TRIANGLE);

		// Vertex buffer
		Vertex3f* vertices = (Vertex3f*)rtcSetNewGeometryBuffer(
			mesh, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3,
			sizeof(Vertex3f), 3 * surface->no_triangles());

		// Index buffer
		Triangle3ui* triangles = (Triangle3ui*)rtcSetNewGeometryBuffer(
			mesh, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3,
			sizeof(Triangle3ui), surface->no_triangles());

		// Set material
		rtcSetGeometryUserData(mesh, (void*)(surface->get_material()));

		// Vertex attributes
		rtcSetGeometryVertexAttributeCount(mesh, 2);

		// Normal buffer
		Normal3f* normals = (Normal3f*)rtcSetNewGeometryBuffer(
			mesh, RTC_BUFFER_TYPE_VERTEX_ATTRIBUTE, 0, RTC_FORMAT_FLOAT3,
			sizeof(Normal3f), 3 * surface->no_triangles());

		// Texture coordinate buffer
		Coord2f* texCoords = (Coord2f*)rtcSetNewGeometryBuffer(
			mesh, RTC_BUFFER_TYPE_VERTEX_ATTRIBUTE, 1, RTC_FORMAT_FLOAT2,
			sizeof(Coord2f), 3 * surface->no_triangles());

		// Fill buffers
		for (int i = 0, k = 0; i < surface->no_triangles(); ++i) {
			Triangle& triangle = surface->get_triangle(i);

			for (int j = 0; j < 3; ++j, ++k) {
				const Vertex& vertex = triangle.vertex(j);

				// Vertex position (no transformation - load as-is)
				vertices[k].x = vertex.position.x;
				vertices[k].y = vertex.position.y;
				vertices[k].z = vertex.position.z;

				// Normal
				normals[k].x = vertex.normal.x;
				normals[k].y = vertex.normal.y;
				normals[k].z = vertex.normal.z;

				// Texture coordinates
				texCoords[k].u = vertex.texture_coords[0].u;
				texCoords[k].v = vertex.texture_coords[0].v;
			}

			// Triangle indices
			triangles[i].v0 = k - 3;
			triangles[i].v1 = k - 2;
			triangles[i].v2 = k - 1;
		}

		rtcCommitGeometry(mesh);
		unsigned int geom_id = rtcAttachGeometry(scene_, mesh);
		rtcReleaseGeometry(mesh);
	}

	// Commit scene
	rtcCommitScene(scene_);

	// Store loaded data
	surfaces_ = newSurfaces;
	materials_ = newMaterials;

	std::cout << "[RAY TRACER] Successfully loaded " << noSurfaces << " surfaces" << std::endl;
	return true;
}

void RayTracer::ClearSurfaceModels() {
	// Clean up existing surfaces and materials
	for (auto surface : surfaces_) {
		delete surface;
	}
	surfaces_.clear();

	for (auto material : materials_) {
		delete material;
	}
	materials_.clear();

	std::cout << "[RAY TRACER] Surface models cleared" << std::endl;
}