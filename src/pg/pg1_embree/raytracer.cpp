#include "stdafx.h"
#include "raytracer.h"
#include "objloader.h"
#include "tutorials.h"
#include "utils.h"
#include <iostream>
#include "smooth_union.h"
#include <opencv2/opencv.hpp>

//=============================================================================
// CONSTRUCTOR & DESTRUCTOR
//=============================================================================

RayTracer::RayTracer(const int width, const int height,
	const float fovY, const Vector3& viewFrom, const Vector3& viewAt, const Vector3& lightOrigin,
	const char* config) : SimpleGuiDX11(width, height)
{
	// Initialize Intel Embree device and scene for surface ray tracing
	InitDeviceAndScene(config);

	// Setup camera with specified parameters
	camera_ = Camera(width, height, fovY, viewFrom, viewAt);

	// Setup light source
	light_ = Light(lightOrigin);
}

RayTracer::~RayTracer()
{
	// Cleanup volumetric shapes (SDF-based)
	for (auto shape : volumetricShapes_) {
		delete shape;
	}
	volumetricShapes_.clear();

	// Cleanup surface geometry (polygonal meshes)
	for (auto surface : surfaces_) {
		delete surface;
	}
	surfaces_.clear();

	// Cleanup materials
	for (auto material : materials_) {
		delete material;
	}
	materials_.clear();

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
	// Create Intel Embree device - this handles BVH construction and ray-triangle intersections
	device_ = rtcNewDevice(config);
	error_handler(nullptr, rtcGetDeviceError(device_), "Unable to create a new device.\n");
	rtcSetDeviceErrorFunction(device_, error_handler, nullptr);

	// Verify triangle geometry support
	ssize_t triangleSupported = rtcGetDeviceProperty(device_, RTC_DEVICE_PROPERTY_TRIANGLE_GEOMETRY_SUPPORTED);

	// Create scene bound to the device - this will contain all triangle geometries
	scene_ = rtcNewScene(device_);

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

	// Return true if no geometry blocks the light (shadow ray missed all geometry)
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
	
	float n_2 = material.ior; // Index of refraction
	if (n_1 == material.ior) {
		n_2 = 1.0f; // Transition to air
	}

	float n_ratio = n_1 / n_2;
	float r; // Reflection coefficient from Fresnel equations

	// Calculate angles for reflection and refraction
	float cos_theta1 = normalVector.DotProduct(-directionVector);
	float temp = 1.0f - powf(n_1 / n_2, 2.0f) * (1.0f - powf(cos_theta1, 2.0f));
	float cos_theta2 = sqrt(temp);
	
	if (temp < 0.0f) {
		r = 1.0f; // Total internal reflection
	}
	else {
		// Fresnel equations for reflection coefficient
		float r_s = powf((n_2 * cos_theta2 - n_1 * cos_theta1) / (n_2 * cos_theta2 + n_1 * cos_theta1), 2.0f);
		float r_p = powf((n_2 * cos_theta1 - n_1 * cos_theta2) / (n_2 * cos_theta1 + n_1 * cos_theta2), 2.0f);
		r = (r_s + r_p) / 2.0f; // Average reflection coefficient
	}

	// Calculate refracted and reflected ray directions
	Vector3 refractedRayDirection = n_1 / n_2 * directionVector + (n_1 / n_2 * cos_theta1 - cos_theta2) * normalVector;
	Vector3 reflectedRayDirection = (2.0f * (-directionVector).DotProduct(normalVector)) * normalVector - (-directionVector);

	// Trace secondary rays for reflection and refraction
	Vector3 refl = TraceRay(MakeSecondaryRay(hitPoint, reflectedRayDirection), n_1, depth + 1);
	Vector3 refr = TraceRay(MakeSecondaryRay(hitPoint, refractedRayDirection), n_2, depth + 1);

	// Combine reflection and refraction using Fresnel coefficient
	// Apply Beer-Lambert attenuation if traveling through medium
	return (refl * r + refr * (1.0f - r)) * 
		   t_b_l(n_1 == 1.0f ? 0.0f : Vector3{ ray.org_x, ray.org_y, ray.org_z }.EuclideanDistance(hitPoint), 
		         material.attenuation);
}

//=============================================================================
// VOLUMETRIC RENDERING DISPATCH
//=============================================================================

Vector4 RayTracer::VolumetricRender(const RTCRay& ray) {
	if (rayMarching_) {
		// RAY MARCHING: Trace through entire volume accumulating color/opacity
		return VolumetricEffect(ray);
	}
	// SPHERE TRACING: Find first surface intersection only  
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
	/*
	 * EMBREE RAY TRACING ALGORITHM:
	 * 1. Use Intel Embree to find closest triangle intersection
	 * 2. Retrieve material and geometry information at hit point
	 * 3. Apply appropriate shader based on material type
	 * 4. Support recursive rays for reflections/refractions
	 * 5. Return final shaded color
	 */

	// Prevent infinite recursion
	if (depth >= maxDepth) {
		return Vector3(0.0f); // Return black if maximum recursion depth reached
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
	rtcIntersect1(scene_, &context, &rayHit); // This is where the BVH traversal happens!

	Vector3 directionVector{ rayHit.ray.dir_x, rayHit.ray.dir_y, rayHit.ray.dir_z };

	// Handle miss: No geometry hit
	if (rayHit.hit.geomID == RTC_INVALID_GEOMETRY_ID) {
		if (backgroundEnabled_) {
			// Sample environment map (cubemap) for background
			Color3f backgroundColor = cubemap_->GetTexel(directionVector);
			return Vector3{ backgroundColor.r, backgroundColor.g, backgroundColor.b };
		}
		return backgroundColorVec_; // Solid background color
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
	assert(material);

	// Interpolate normal vector at hit point using barycentric coordinates
	Normal3f normal;
	rtcInterpolate0(geometry, rayHit.hit.primID, rayHit.hit.u, rayHit.hit.v,
		RTC_BUFFER_TYPE_VERTEX_ATTRIBUTE, 0, &normal.x, 3);
	Vector3 normalVector{ normal.x, normal.y, normal.z };

	// Flip normal if it points away from the ray (ensure it points toward ray origin)
	if (normalVector.DotProduct(directionVector) > 0.0f) {
		normalVector *= -1.0f;
	}

	// Interpolate texture coordinates at hit point
	Coord2f texCoord;
	rtcInterpolate0(geometry, rayHit.hit.primID, rayHit.hit.u, rayHit.hit.v,
		RTC_BUFFER_TYPE_VERTEX_ATTRIBUTE, 1, &texCoord.u, 2);

	// SHADER DISPATCH: Apply appropriate shader based on material type
	switch (material->shader) {
	case 1:
		return NormalShader(normalVector);  // Visualize normals as colors
	case 2:
		return LambertShader(*material, texCoord, hitPoint, normalVector); // Diffuse only
	case 3:
		return PhongShader(*material, texCoord, hitPoint, normalVector, directionVector, depth); // Diffuse + Specular
	case 4:
		return TransparentShader(ray, hitPoint, normalVector, directionVector, *material, n_1, depth); // Glass/transparent
	default:
		return LambertShader(*material, texCoord, hitPoint, normalVector); // Default to Lambert
	}
}

//=============================================================================
// MAIN RENDERING PIPELINE
//=============================================================================

Color4f RayTracer::GetPixel(const int x, const int y, const float t)
{
	/*
	 * HYBRID RENDERING PIPELINE:
	 * 1. SURFACE RAY TRACING: Use Embree for triangle meshes
	 * 2. VOLUMETRIC RENDERING: Use SDF ray marching for procedural volumes
	 * 3. COMPOSITE: Combine both results with proper alpha blending
	 */

	Vector3 rtc_color = Vector3(0.0f);

	if (sampling_) {
		// SUPERSAMPLING: Average multiple rays per pixel for anti-aliasing
		Vector3 accumulator;

		for (int j = 0; j < 4; j++) {
			for (int i = 0; i < 4; i++) {
				float ksi_x = Random() / 4; // Random jitter for anti-aliasing
				float ksi_y = Random() / 4;

				// Generate ray for sub-pixel sample
				RTCRay ray = camera_.GenerateRay(float(x) + i * (1.0f / 4.0f) + ksi_x, 
					float(y) + j * (1.0f / 4.0f) + ksi_y);
				accumulator += TraceRay(ray); // Trace ray and accumulate color
			}
		}
		accumulator /= 16.0f; // Average accumulated color (4x4 = 16 samples)
		rtc_color = accumulator;
	}
	else {
		// SINGLE SAMPLE: Trace one ray per pixel
		rtc_color = TraceRay(camera_.GenerateRay(float(x), float(y)));
	}

	// VOLUMETRIC RENDERING: Render procedural SDF volumes
	Vector4 volumetric_color = VolumetricRender(camera_.GenerateRay(float(x), float(y)));

	// COMPOSITE: Blend volumetric and surface results
	// Formula: C_final = C_volume + C_surface * (1 - α_volume)
	// This assumes volume is in front of surface
	Vector3 final_color = Vector3(volumetric_color.x, volumetric_color.y, volumetric_color.z) + 
		rtc_color * (1.0f - volumetric_color.w);

	// Return final color compressed to [0,1] range
	return final_color.ToColor4fCompressed();
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

	ImGui::Begin("Ray Tracer Params");

	ImGui::Text("Surfaces = %d", surfaces_.size());
	ImGui::Text("Materials = %d", materials_.size());
	ImGui::Separator();

	// === SDF NOISE SETTINGS ===
	if (ImGui::Checkbox("Noise", &useNoise_)) {
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

	// === SDF SMOOTHING ===
	if (ImGui::SliderFloat("Smooth Factor", &smoothFactor_, 0.0f, 3.0f)) {
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

	// === VOLUMETRIC RENDERING SETTINGS ===
	ImGui::Checkbox("Ray Marching", &rayMarching_);
	if (ImGui::SliderFloat("Step Size", &stepSize_, 0.00001f, 0.1f)) {
		maxSteps_ = static_cast<int>(maxDistance_ / stepSize_);
	}
	if (ImGui::SliderFloat("Max Distance", &maxDistance_, 1.0f, 100.0f)) {
		maxSteps_ = static_cast<int>(maxDistance_ / stepSize_);
	}
	ImGui::SliderFloat("Absorption Coefficient", &absorptionCoefficient_, 0.0f, 3.0f);
	ImGui::SliderFloat("Attenuation Factor", &lightAttenuationFactor_, 0.0f, 3.0f);
	if (ImGui::ColorEdit3("Volumetric Albedo", volumetricAlbedo_)) {
		volumetricAlbedoVec_ = Vector3(volumetricAlbedo_[0], volumetricAlbedo_[1], volumetricAlbedo_[2]);
	};

	// === BACKGROUND SETTINGS ===
	ImGui::Checkbox("Background", &backgroundEnabled_);
	if (ImGui::ColorEdit3("Background Color", backgroundColor_)) {
		backgroundColorVec_ = Vector3(backgroundColor_[0], backgroundColor_[1], backgroundColor_[2]);
	};

	// === RENDERING SETTINGS ===
	ImGui::Checkbox("Camera Movement", &cameraMovementEnabled_);
	ImGui::Checkbox("Vsync", &vsync_);
	ImGui::Checkbox("Sampling", &sampling_);

	// === CAMERA CONTROLS ===
	if (ImGui::SliderFloat("Camera X", &cameraX_, -100.0f, 100.0f)) {
		camera_.SetViewFrom(Vector3(cameraX_, cameraY_, cameraZ_));
	}
	if (ImGui::SliderFloat("Camera Y", &cameraY_, -100.0f, 100.0f)) {
		camera_.SetViewFrom(Vector3(cameraX_, cameraY_, cameraZ_));
	}
	if (ImGui::SliderFloat("Camera Z", &cameraZ_, -100.0f, 100.0f)) {
		camera_.SetViewFrom(Vector3(cameraX_, cameraY_, cameraZ_));
	}

	// === CAMERA ROTATION ===
	static int cameraRotationX = 0;
	static int cameraRotationY = 0;
	static int cameraRotationZ = 0;
	static int previousDragDeltaX = 0;
	static int previousDragDeltaY = 0;

	if (ImGui::SliderInt("Camera Rotation X", &cameraRotationX, 0, 359)) {
		Vector3 currentRotation = camera_.GetRotation();
		currentRotation.x = (float)cameraRotationX;
		camera_.SetRotation(currentRotation);
	}
	if (ImGui::SliderInt("Camera Rotation Y", &cameraRotationY, 0, 359)) {
		Vector3 currentRotation = camera_.GetRotation();
		currentRotation.y = (float)cameraRotationY;
		camera_.SetRotation(currentRotation);
	}
	if (ImGui::SliderInt("Camera Rotation Z", &cameraRotationZ, 0, 359)) {
		Vector3 currentRotation = camera_.GetRotation();
		currentRotation.z = (float)cameraRotationZ;
		camera_.SetRotation(currentRotation);
	}

	// Mouse drag camera control
	if (ImGui::IsMouseDown(1)) {
		if (ImGui::IsMouseDragging(1)) {
			ImVec2 dragDelta = ImGui::GetMouseDragDelta(1);
			int currentDragDeltaY = static_cast<int>(dragDelta.x);
			int currentDragDeltaX = static_cast<int>(dragDelta.y);
			Vector3 currentRotation = camera_.GetRotation();
			currentRotation.x += currentDragDeltaX / 4 - previousDragDeltaX / 4;
			currentRotation.y += currentDragDeltaY / 4 - previousDragDeltaY / 4;
			camera_.SetRotation(currentRotation);
			previousDragDeltaX = currentDragDeltaX;
			previousDragDeltaY = currentDragDeltaY;

			currentRotation = camera_.GetRotation();
			cameraRotationX = (int)currentRotation.x;
			cameraRotationY = (int)currentRotation.y;
		}
	}
	if (ImGui::IsMouseReleased(1)) {
		previousDragDeltaX = 0;
		previousDragDeltaY = 0;
	}

	// === LIGHT CONTROLS ===
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

	// === VIDEO EXPORT ===
	if (ImGui::Button("Save Video and Exit")) {
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

	ImGui::End();
	return 0;
}