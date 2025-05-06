#include "stdafx.h"
#include "raytracer.h"
#include "objloader.h"
#include "tutorials.h"
#include "utils.h"
#include <iostream>
#include "smooth_union.h"
#include <opencv2/opencv.hpp>

// Constructor for the RayTracer class
// Initializes the rendering device, camera, and light source
RayTracer::RayTracer(const int width, const int height,
	const float fovY, const Vector3& viewFrom, const Vector3& viewAt, const Vector3& lightOrigin,
	const char* config) : SimpleGuiDX11(width, height)
{
	InitDeviceAndScene(config);

	// Initialize the camera with the given parameters
	camera_ = Camera(width, height, fovY, viewFrom, viewAt);

	// Initialize the light source
	light_ = Light(lightOrigin);
}

// Destructor for the RayTracer class
// Cleans up allocated resources such as shapes, surfaces, materials, and the cubemap
RayTracer::~RayTracer()
{
	for (auto shape : volumetricShapes_) {
		delete shape;
	}
	volumetricShapes_.clear();

	for (auto surface : surfaces_) {
		delete surface;
	}
	surfaces_.clear();

	for (auto material : materials_) {
		delete material;
	}
	materials_.clear();

	delete cubemap_;
	cubemap_ = nullptr;

	// Release the Embree scene and device
	rtcReleaseScene(scene_);
	rtcReleaseDevice(device_);
}

// Initializes the Embree device and scene
int RayTracer::InitDeviceAndScene(const char* config)
{
	device_ = rtcNewDevice(config);
	error_handler(nullptr, rtcGetDeviceError(device_), "Unable to create a new device.\n");
	rtcSetDeviceErrorFunction(device_, error_handler, nullptr);

	ssize_t triangleSupported = rtcGetDeviceProperty(device_, RTC_DEVICE_PROPERTY_TRIANGLE_GEOMETRY_SUPPORTED);

	// create a new scene bound to the specified device
	scene_ = rtcNewScene(device_);

	return S_OK;
}

// Releases the Embree device and scene
int RayTracer::ReleaseDeviceAndScene()
{
	rtcReleaseScene(scene_);
	rtcReleaseDevice(device_);

	return S_OK;
}

// Loads a 3D model from a file and applies the given transformation
void RayTracer::LoadModel(const std::string& fileName, const Transform& transform)
{
	const int noSurfaces = LoadOBJ(fileName.c_str(), surfaces_, materials_);

	for (auto surface : surfaces_)
	{
		RTCGeometry mesh = rtcNewGeometry(device_, RTC_GEOMETRY_TYPE_TRIANGLE);

		// Allocate and set vertex buffer
		Vertex3f* vertices = (Vertex3f*)rtcSetNewGeometryBuffer(
			mesh, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3,
			sizeof(Vertex3f), 3 * surface->no_triangles());

		// Allocate and set index buffer
		Triangle3ui* triangles = (Triangle3ui*)rtcSetNewGeometryBuffer(
			mesh, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3,
			sizeof(Triangle3ui), surface->no_triangles());

		// Associate material data with the geometry
		rtcSetGeometryUserData(mesh, (void*)(surface->get_material()));

		// Set vertex attribute count (normals and texture coordinates)
		rtcSetGeometryVertexAttributeCount(mesh, 2);

		// Allocate and set normal buffer
		Normal3f* normals = (Normal3f*)rtcSetNewGeometryBuffer(
			mesh, RTC_BUFFER_TYPE_VERTEX_ATTRIBUTE, 0, RTC_FORMAT_FLOAT3,
			sizeof(Normal3f), 3 * surface->no_triangles());

		// Allocate and set texture coordinate buffer
		Coord2f* texCoords = (Coord2f*)rtcSetNewGeometryBuffer(
			mesh, RTC_BUFFER_TYPE_VERTEX_ATTRIBUTE, 1, RTC_FORMAT_FLOAT2,
			sizeof(Coord2f), 3 * surface->no_triangles());

		// Loop through triangles and set vertex data
		for (int i = 0, k = 0; i < surface->no_triangles(); ++i)
		{
			Triangle& triangle = surface->get_triangle(i);

			// Loop through vertices and apply transformations
			for (int j = 0; j < 3; ++j, ++k)
			{
				const Vertex& vertex = triangle.vertex(j);

				// Apply translation and scale to vertex positions
				vertices[k].x = vertex.position.x * transform.scale.x + transform.position.x;
				vertices[k].y = vertex.position.y * transform.scale.y + transform.position.y;
				vertices[k].z = vertex.position.z * transform.scale.z + transform.position.z;

				// Apply scale to normals and normalize them
				normals[k].x = vertex.normal.x * transform.scale.x;
				normals[k].y = vertex.normal.y * transform.scale.y;
				normals[k].z = vertex.normal.z * transform.scale.z;

				float length = sqrt(normals[k].x * normals[k].x + normals[k].y * normals[k].y + normals[k].z * normals[k].z);
				if (length > 0.0f) {
					normals[k].x /= length;
					normals[k].y /= length;
					normals[k].z /= length;
				}

				// Set texture coordinates
				texCoords[k].u = vertex.texture_coords[0].u;
				texCoords[k].v = vertex.texture_coords[0].v;
			}

			// Set triangle indices
			triangles[i].v0 = k - 3;
			triangles[i].v1 = k - 2;
			triangles[i].v2 = k - 1;
		}

		// Commit geometry changes and attach to the scene
		rtcCommitGeometry(mesh);
		unsigned int geom_id = rtcAttachGeometry(scene_, mesh);
		rtcReleaseGeometry(mesh);
	}
}

// Loads the scene with models, volumetric shapes, and a cubemap
void RayTracer::LoadScene(
	const std::vector<ModelInfo>& models,
	const std::vector<Shape*>& shapes,
	const char* cubeMapFileNames[6]
)
{
	// Load the cubemap textures
	cubemap_ = new CubeMap(cubeMapFileNames);

	// Load all OBJ models with their transformations
	for (const auto& model : models)
	{
		LoadModel(model.filePath, model.transform);
	}

	// Add volumetric shapes to the scene
	for (const auto& shape : shapes)
	{
		volumetricShapes_.push_back(shape);
	}

	// Commit the scene to finalize changes
	rtcCommitScene(scene_);
}

// Creates a secondary ray for reflection or refraction
RTCRay MakeSecondaryRay(const Vector3& origin, const Vector3& dir) {

	RTCRay ray = RTCRay();
	ray.org_x = origin.x;
	ray.org_y = origin.y;
	ray.org_z = origin.z;
	ray.tnear = 0.001f;

	ray.dir_x = dir.x;
	ray.dir_y = dir.y;
	ray.dir_z = dir.z;
	ray.time = 0.0f;

	ray.tfar = FLT_MAX;

	ray.mask = 0;
	ray.id = 0;
	ray.flags = 0;

	return ray;
}

// Checks if a point is visible from a light source
bool RayTracer::IsHitPointVisible(const Vector3& hitPoint, const Vector3& lightPoint) {
	Vector3 l = lightPoint - hitPoint;
	float dist = l.L2Norm();
	l *= 1.0f / dist;

	RTCRay ray = RTCRay();
	ray.org_x = hitPoint.x;
	ray.org_y = hitPoint.y;
	ray.org_z = hitPoint.z;
	ray.tnear = 0.001f;

	ray.dir_x = l.x;
	ray.dir_y = l.y;
	ray.dir_z = l.z;
	ray.time = 0.0f;

	ray.tfar = dist;

	ray.mask = 0;
	ray.id = 0;
	ray.flags = 0;

	// Setup a hit structure
	RTCHit hit;
	hit.geomID = RTC_INVALID_GEOMETRY_ID;
	hit.primID = RTC_INVALID_GEOMETRY_ID;
	hit.Ng_x = 0.0f;
	hit.Ng_y = 0.0f;
	hit.Ng_z = 0.0f;

	// Merge ray and hit structures
	RTCRayHit rayHit;
	rayHit.ray = ray;
	rayHit.hit = hit;

	// Intersect the ray with the scene
	RTCIntersectContext context;
	rtcInitIntersectContext(&context);
	rtcIntersect1(scene_, &context, &rayHit);

	// Return true if no geometry blocks the light
	return rayHit.hit.geomID == RTC_INVALID_GEOMETRY_ID;
}

// Calculates the attenuation of light based on the distance to the light source
// and a user-defined attenuation factor
float GetLightAttenuation(const float distanceToLight, const float lightAttenuationFactor)
{
	return 1.0f / pow(distanceToLight, lightAttenuationFactor);
}

// Returns the ambient light contribution in the scene
// This is a constant value representing low-level illumination
Vector3 GetAmbientLight()
{
	return 1.2f * Vector3(0.03f, 0.018f, 0.018f); // Slight reddish ambient light
}

// Computes the normal vector at a given point on a shape using the SDF (Signed Distance Function)
// The normal is calculated by sampling the SDF in small offsets along each axis
Vector3 ComputeNormal(const Vector3& p, const Shape& shape) {
	const float eps = 0.001f; // Small offset for numerical differentiation

	float dx = shape.SDF(p + Vector3(eps, 0.0f, 0.0f)) - shape.SDF(p - Vector3(eps, 0.0f, 0.0f));
	float dy = shape.SDF(p + Vector3(0.0f, eps, 0.0f)) - shape.SDF(p - Vector3(0.0f, eps, 0.0f));
	float dz = shape.SDF(p + Vector3(0.0f, 0.0f, eps)) - shape.SDF(p - Vector3(0.0f, 0.0f, eps));

	return Vector3(dx, dy, dz) / (2.0f * eps); // Normalize the gradient
}

// A simple shader that visualizes the normal vector as a color
// The normal is mapped to the range [0, 1] for display purposes
Vector3 RayTracer::NormalShader(const Vector3& normalVector) {
	return (normalVector + Vector3(1.0f, 1.0f, 1.0f)) * 0.5f;
}

// Implements the Lambert shading model for diffuse lighting
// Calculates the color contribution based on the material's diffuse color,
// the light direction, and the surface normal
Vector3 RayTracer::LambertShader(const Material& material, const Coord2f& texCoord, const Vector3& hitPoint, const Vector3& normalVector) {
	Vector3 outputColor;

	// Retrieve the diffuse color from the material or texture
	Vector3 diffuseColor = Vector3{ material.diffuse.x, material.diffuse.y, material.diffuse.z };
	Texture* diffuseTexture = material.get_texture(Material::kDiffuseMapSlot);
	if (diffuseTexture) {
		Color3f diffuseTexel = diffuseTexture->get_texel(texCoord.u, 1 - texCoord.v);
		diffuseColor.x = diffuseTexel.r;
		diffuseColor.y = diffuseTexel.g;
		diffuseColor.z = diffuseTexel.b;
	}

	const Vector3 omniLightPosition = light_.origin_;

	// Calculate the light direction
	Vector3 l = omniLightPosition - hitPoint;
	l.Normalize();

	// Add ambient light contribution
	outputColor += material.ambient;

	// Add diffuse light contribution if the point is visible from the light
	if (IsHitPointVisible(hitPoint, omniLightPosition)) {
		outputColor += diffuseColor * clamp(normalVector.DotProduct(l));
	}

	return outputColor;
}

// Implements the Phong shading model for specular highlights
// Combines ambient, diffuse, and specular lighting contributions
Vector3 RayTracer::PhongShader(const Material& material, const Coord2f& texCoord, const Vector3& hitPoint, const Vector3& normalVector, const Vector3& directionVector, const int depth) {
	Vector3 outputColor;

	// Retrieve the diffuse color from the material or texture
	Vector3 diffuseColor = Vector3{ material.diffuse.x, material.diffuse.y, material.diffuse.z };
	Texture* diffuseTexture = material.get_texture(Material::kDiffuseMapSlot);
	if (diffuseTexture) {
		Color3f diffuseTexel = diffuseTexture->get_texel(texCoord.u, 1 - texCoord.v);
		diffuseColor.x = diffuseTexel.r;
		diffuseColor.y = diffuseTexel.g;
		diffuseColor.z = diffuseTexel.b;
	}

	// Retrieve the specular color from the material or texture
	Vector3 specularColor = Vector3{ material.specular.x, material.specular.y, material.specular.z };
	Texture* specularTexture = material.get_texture(Material::kSpecularMapSlot);
	if (specularTexture) {
		Color3f specularTexel = specularTexture->get_texel(texCoord.u, 1.0f - texCoord.v);
		specularColor.x = specularTexel.r;
		specularColor.y = specularTexel.g;
		specularColor.z = specularTexel.b;
	}

	const Vector3 omniLightPosition = light_.origin_;

	// Calculate the light direction
	Vector3 l = omniLightPosition - hitPoint;
	l.Normalize();

	// Add ambient light contribution
	outputColor += material.ambient;

	// Add diffuse and specular light contributions if the point is visible from the light
	if (IsHitPointVisible(hitPoint, omniLightPosition)) {
		// Diffuse lighting
		outputColor += diffuseColor * clamp(normalVector.DotProduct(l));

		// Specular lighting
		Vector3 l_r = 2.0f * (l.DotProduct(normalVector) * normalVector) - l;
		l_r.Normalize();

		RTCRay secondaryRay = MakeSecondaryRay(hitPoint, l);

		Vector3 l_i = TraceRay(secondaryRay, depth + 1.0f);

		outputColor += specularColor * powf(clamp(l_r.DotProduct(-directionVector)), material.shininess);
		outputColor += l_i * specularColor * 0.2f; // Indirect specular contribution
	}

	return outputColor;
}

// Computes the Beer-Lambert law for light attenuation through a medium
// The attenuation is calculated based on the distance and the medium's properties
Vector3 t_b_l(const float length, const Vector3 attenuation) {
	Vector3 a = Vector3{ powf(exp(1.0f), -attenuation.x * length), powf(exp(1.0f), -attenuation.y * length), powf(exp(1.0f), -attenuation.z * length) };
	return a;
}

// Implements a transparent shader for materials with refraction and reflection
// Combines reflection and refraction contributions based on the Fresnel equations
Vector3 RayTracer::TransparentShader(const RTCRay& ray, const Vector3& hitPoint, const Vector3& normalVector, const Vector3& directionVector, const Material& material, const float n_1, const int depth) {
	float n_2 = material.ior; // Index of refraction
	if (n_1 == material.ior) {
		n_2 = 1.0f; // Transition to air
	}

	float n_ratio = n_1 / n_2;
	float r; // Reflection coefficient

	// Calculate the angles for reflection and refraction
	float cos_theta1 = normalVector.DotProduct(-directionVector);
	float temp = 1.0f - powf(n_1 / n_2, 2.0f) * (1.0f - powf(cos_theta1, 2.0f));
	float cos_theta2 = sqrt(temp);
	if (temp < 0.0f) {
		r = 1.0f; // Total internal reflection
	}
	else {
		float r_s = powf((n_2 * cos_theta2 - n_1 * cos_theta1) / (n_2 * cos_theta2 + n_1 * cos_theta1), 2.0f);
		float r_p = powf((n_2 * cos_theta1 - n_1 * cos_theta2) / (n_2 * cos_theta1 + n_1 * cos_theta2), 2.0f);
		r = (r_s + r_p) / 2.0f; // Average reflection coefficient
	}

	// Calculate the refracted and reflected ray directions
	Vector3 refractedRayDirection = n_1 / n_2 * directionVector + (n_1 / n_2 * cos_theta1 - cos_theta2) * normalVector;
	Vector3 reflectedRayDirection = (2.0f * (-directionVector).DotProduct(normalVector)) * normalVector - (-directionVector);

	// Trace the reflected and refracted rays
	Vector3 refl = TraceRay(MakeSecondaryRay(hitPoint, reflectedRayDirection), n_1, depth + 1);
	Vector3 refr = TraceRay(MakeSecondaryRay(hitPoint, refractedRayDirection), n_2, depth + 1);

	// Combine reflection and refraction contributions
	return (refl * r + refr * (1.0f - r)) * t_b_l(n_1 == 1.0f ? 0.0f : Vector3{ ray.org_x, ray.org_y, ray.org_z }.EuclideanDistance(hitPoint), material.attenuation);
}

// Handles volumetric rendering by determining whether to use ray marching or surface effects
// Returns a color and opacity value for the volumetric effect
Vector4 RayTracer::VolumetricRender(const RTCRay& ray) {
	if (rayMarching_) {
		// Perform ray marching to compute volumetric effects
		return VolumetricEffect(ray);
	}
	// Compute surface effects for non-volumetric rendering
	return SurfaceEffect(ray);
}

// Computes volumetric effects using ray marching
// Accumulates color and opacity based on light absorption and scattering
Vector4 RayTracer::VolumetricEffect(const RTCRay& ray)
{
	const int numLights = 1; // Number of lights in the scene

	Vector3 position(ray.org_x, ray.org_y, ray.org_z); // Starting position of the ray
	Vector3 direction(ray.dir_x, ray.dir_y, ray.dir_z); // Direction of the ray
	direction.Normalize();

	float accumulatedOpacity = 1.0f; // Initial visibility (1.0 = fully visible)
	Vector3 volumetricColor(0.0f, 0.0f, 0.0f); // Accumulated volumetric color

	for (int i = 0; i < maxSteps_; ++i) {
		if (position.L2Norm() > maxDistance_) break; // Stop if the ray exceeds max distance from the origin

		// Query the volumetric distance field (assume the first shape in volumetricShapes_)
		if (volumetricShapes_.empty()) break; // No volumetric shapes to process
		Shape* shape = volumetricShapes_[0];
		float sdfValue = shape->SDF(position);

		// Check if the ray is inside the volume
		if (sdfValue < 0.1f) {
			float previousOpacity = accumulatedOpacity;

			// Apply Beer-Lambert law to calculate light absorption
			accumulatedOpacity *= exp(-absorptionCoefficient_ * stepSize_);

			// Calculate the amount of light absorbed in this step
			float absorptionFromMarch = previousOpacity - accumulatedOpacity;

			// Process each light in the scene
			for (int lightIndex = 0; lightIndex < numLights; ++lightIndex) {
				Vector3 lightPosition = light_.origin_; // Assume a single light source
				Vector3 lightDirection = lightPosition - position;
				float lightDistance = lightDirection.L2Norm();
				lightDirection.Normalize();

				// Calculate light attenuation
				Vector3 lightColor = Vector3(1.0f, 1.0f, 1.0f) * GetLightAttenuation(lightDistance, lightAttenuationFactor_);

				// Add the contribution of this light to the volumetric color
				volumetricColor += absorptionFromMarch * volumetricAlbedoVec_ * lightColor;
			}

			// Add ambient light contribution
			volumetricColor += absorptionFromMarch * volumetricAlbedoVec_ * GetAmbientLight();
		}

		// Advance the ray position
		position += direction * stepSize_;
	}

	return Vector4(volumetricColor, 1.0f - accumulatedOpacity);
}


// Computes surface effects for volumetric shapes using ray marching
// Returns the color and opacity of the surface
Vector4 RayTracer::SurfaceEffect(const RTCRay& ray)
{
	Vector3 position(ray.org_x, ray.org_y, ray.org_z); // Starting position of the ray
	Vector3 direction(ray.dir_x, ray.dir_y, ray.dir_z); // Direction of the ray
	direction.Normalize();

	for (int i = 0; i < 1000; ++i) {
		if (position.L2Norm() > maxDistance_) break; // Stop if the ray exceeds max distance from the origin

		if (volumetricShapes_.empty()) break; // No volumetric shapes to process
		Shape* shape = volumetricShapes_[0];
		float sdfValue = shape->SDF(position);

		// Check if the ray hits the surface
		if (sdfValue < 0.001f) {
			Vector3 normal = ComputeNormal(position, *shape);
			normal.Normalize();

			Vector3 lightDir = light_.origin_ - position;
			lightDir.Normalize();

			// Ambient lighting
			Vector3 ambientColor(0.025f, 0.025f, 0.025f);
			Vector3 color = ambientColor;

			// Diffuse lighting
			float diffuse = std::max(0.0f, normal.DotProduct(lightDir));
			if (IsHitPointVisible(position, light_.origin_)) {
				color += Vector3(diffuse, diffuse, diffuse);
			}

			// Specular lighting
			Vector3 viewDir = -direction;
			Vector3 reflectDir = 2.0f * normal.DotProduct(lightDir) * normal - lightDir;
			float spec = powf(std::max(viewDir.DotProduct(reflectDir), 0.0f), 32.0f); // Shininess = 32
			color += Vector3(spec, spec, spec);

			return Vector4(color, 1.0f); // Return the surface color and full opacity
		}

		// Advance the ray position
		position += direction * sdfValue;
	}

	return Vector4(0.0f); // Return no color if no surface is hit
}


// Traces a ray through the scene and determines the color at the intersection point
// Supports recursive tracing for reflection, refraction, and other effects
Vector3 RayTracer::TraceRay(const RTCRay& ray, const float n_1, const int depth, const int maxDepth) {
	if (depth >= maxDepth) {
		return Vector3(0.0f, 0.0f, 0.0f); // Return black if the maximum recursion depth is reached
	}

	// Setup a hit structure
	RTCHit hit;
	hit.geomID = RTC_INVALID_GEOMETRY_ID;
	hit.primID = RTC_INVALID_GEOMETRY_ID;
	hit.Ng_x = 0.0f; // Geometry normal
	hit.Ng_y = 0.0f;
	hit.Ng_z = 0.0f;

	// Merge ray and hit structures
	RTCRayHit rayHit;
	rayHit.ray = ray;
	rayHit.hit = hit;

	// Intersect the ray with the scene
	RTCIntersectContext context;
	rtcInitIntersectContext(&context);
	rtcIntersect1(scene_, &context, &rayHit);

	Vector3 directionVector{ rayHit.ray.dir_x, rayHit.ray.dir_y, rayHit.ray.dir_z };

	// If no geometry is hit, return the background color
	if (rayHit.hit.geomID == RTC_INVALID_GEOMETRY_ID) {
		if (backgroundEnabled_) {
			Color3f backgroundColor = cubemap_->GetTexel(directionVector);
			return Vector3{ backgroundColor.r, backgroundColor.g, backgroundColor.b };
		}
		return backgroundColorVec_;
	}

	// Calculate the hit point
	const Vector3 hitPoint{
		rayHit.ray.org_x + rayHit.ray.dir_x * rayHit.ray.tfar,
		rayHit.ray.org_y + rayHit.ray.dir_y * rayHit.ray.tfar,
		rayHit.ray.org_z + rayHit.ray.dir_z * rayHit.ray.tfar
	};

	// Retrieve the geometry and material at the hit point
	RTCGeometry geometry = rtcGetGeometry(scene_, rayHit.hit.geomID);
	Material* material = (Material*)rtcGetGeometryUserData(geometry);
	assert(material);

	// Retrieve the normal vector at the hit point
	Normal3f normal;
	rtcInterpolate0(geometry, rayHit.hit.primID, rayHit.hit.u, rayHit.hit.v,
		RTC_BUFFER_TYPE_VERTEX_ATTRIBUTE, 0, &normal.x, 3);
	Vector3 normalVector{ normal.x, normal.y, normal.z };

	// Flip the normal if it points in the same direction as the ray
	if (normalVector.DotProduct(directionVector) > 0.0f) {
		normalVector *= -1.0f;
	}

	// Retrieve the texture coordinates at the hit point
	Coord2f texCoord;
	rtcInterpolate0(geometry, rayHit.hit.primID, rayHit.hit.u, rayHit.hit.v,
		RTC_BUFFER_TYPE_VERTEX_ATTRIBUTE, 1, &texCoord.u, 2);

	// Select the appropriate shader based on the material
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
		return LambertShader(*material, texCoord, hitPoint, normalVector);
	}
}

// Computes the color of a single pixel in the rendered image
// Combines ray tracing and volumetric rendering to produce the final color
Color4f RayTracer::GetPixel(const int x, const int y, const float t)
{
	Vector3 rtc_color = Vector3(0.0f, 0.0f, 0.0f);

	if (sampling_) {
		// Perform supersampling by averaging colors from multiple rays per pixel
		Vector3 accumulator;

		for (int j = 0; j < 4; j++) {
			for (int i = 0; i < 4; i++) {
				float ksi_x = Random() / 4; // Random offset for anti-aliasing
				float ksi_y = Random() / 4;

				// Generate a ray for the current sub-pixel
				RTCRay ray = camera_.GenerateRay(float(x) + i * (1.0f / 4.0f) + ksi_x, float(y) + j * (1.0f / 4.0f) + ksi_y);
				accumulator += TraceRay(ray); // Trace the ray and accumulate the color
			}
		}
		accumulator /= 4.0f * 4.0f; // Average the accumulated color
		rtc_color = accumulator;
	}
	else {
		// Trace a single ray for the pixel
		rtc_color = TraceRay(camera_.GenerateRay(float(x), float(y)));
	}

	// Perform volumetric rendering for the pixel
	Vector4 volumetric_color = VolumetricRender(camera_.GenerateRay(float(x), float(y)));

	// Combine the ray-traced color and the volumetric color
	Vector3 final_color = Vector3(volumetric_color.x, volumetric_color.y, volumetric_color.z) + rtc_color * (1.0f - volumetric_color.w);

	// Return the final color as a compressed 4-channel color
	return final_color.ToColor4fCompressed();
}

// Moves the camera in a circular path around the target point (viewAt)
// This function is called if camera movement is enabled
void RayTracer::MoveCamera()
{
	if (cameraMovementEnabled_) {
		static float angle = 0.0f; // Current angle of rotation
		const float speed = 0.025f; // Speed of rotation

		Vector3 viewAt = camera_.GetViewAt(); // Target point the camera is looking at
		Vector3 viewFrom = camera_.GetViewFrom(); // Current position of the camera

		// Calculate the current radius of the camera's circular path in the x-y plane
		Vector3 direction = viewFrom - viewAt;
		float currentRadius = sqrt(direction.x * direction.x + direction.y * direction.y); // Ignore the z-coordinate

		// Update the camera's x and y position based on the angle
		cameraX_ = viewAt.x + currentRadius * cosf(angle);
		cameraY_ = viewAt.y + currentRadius * sinf(angle);

		// Maintain the original height of the camera
		cameraZ_ = viewFrom.z;

		// Increment the angle for the next frame
		angle += speed;
		if (angle > 2.0f * (float)M_PI) {
			angle -= 2.0f * (float)M_PI; // Wrap the angle to stay within [0, 2π]
		}

		// Update the camera's position
		camera_.SetViewFrom(Vector3(cameraX_, cameraY_, cameraZ_));
	}
}

// Renders the user interface for controlling ray tracing parameters
// Uses ImGui to create sliders, checkboxes, and buttons for user interaction
int RayTracer::Ui()
{
	static float f = 0.0f;
	static int counter = 0;

	// Create a new ImGui window for ray tracer parameters
	ImGui::Begin("Ray Tracer Params");

	// Display the number of surfaces and materials in the scene
	ImGui::Text("Surfaces = %d", surfaces_.size());
	ImGui::Text("Materials = %d", materials_.size());
	ImGui::Separator();

	// Noise settings
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

	// Smooth factor for volumetric shapes
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

	// Ray marching settings
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

	// Background settings
	ImGui::Checkbox("Background", &backgroundEnabled_);
	if (ImGui::ColorEdit3("Background Color", backgroundColor_)) {
		backgroundColorVec_ = Vector3(backgroundColor_[0], backgroundColor_[1], backgroundColor_[2]);
	};

	// Camera movement and rendering settings
	ImGui::Checkbox("Camera Movement", &cameraMovementEnabled_);
	ImGui::Checkbox("Vsync", &vsync_);
	ImGui::Checkbox("Sampling", &sampling_);

	// Camera position sliders
	if (ImGui::SliderFloat("Camera X", &cameraX_, -100.0f, 100.0f)) {
		camera_.SetViewFrom(Vector3(cameraX_, cameraY_, cameraZ_));
	}
	if (ImGui::SliderFloat("Camera Y", &cameraY_, -100.0f, 100.0f)) {
		camera_.SetViewFrom(Vector3(cameraX_, cameraY_, cameraZ_));
	}
	if (ImGui::SliderFloat("Camera Z", &cameraZ_, -100.0f, 100.0f)) {
		camera_.SetViewFrom(Vector3(cameraX_, cameraY_, cameraZ_));
	}

	// Camera rotation sliders
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

	// Mouse rotation control
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

	// Light position sliders
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

	// Save video button
	if (ImGui::Button("Save Video and Exit")) {
		// Save the rendered frames as a video
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

	// Hint: Show another simple window.
	/*if ( show_another_window )
	{
		ImGui::Begin( "Another Window", &show_another_window ); // Pass a pointer to our bool variable (the window will have a closing button that will clear the bool when clicked)
		ImGui::Text( "Hello from another window!" );
		if ( ImGui::Button( "Close Me" ) )
		show_another_window = false;
		ImGui::End();
	}*/

	return 0;
}