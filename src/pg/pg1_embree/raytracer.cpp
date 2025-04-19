#include "stdafx.h"
#include "raytracer.h"
#include "objloader.h"
#include "tutorials.h"
#include "utils.h"
#include <iostream>
#include "smooth_union.h"
#include <opencv2/opencv.hpp>

RayTracer::RayTracer(const int width, const int height,
	const float fovY, const Vector3& viewFrom, const Vector3& viewAt, const Vector3& lightOrigin,
	const char* config) : SimpleGuiDX11(width, height)
{
	InitDeviceAndScene(config);

	camera_ = Camera(width, height, fovY, viewFrom, viewAt);
	light_ = Light(lightOrigin);
}

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

	rtcReleaseScene(scene_);
	rtcReleaseDevice(device_);
}

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

int RayTracer::ReleaseDeviceAndScene()
{
	rtcReleaseScene(scene_);
	rtcReleaseDevice(device_);

	return S_OK;
}

void RayTracer::LoadModel(const std::string& fileName, const Transform& transform)
{
	const int noSurfaces = LoadOBJ(fileName.c_str(), surfaces_, materials_);

	for (auto surface : surfaces_)
	{
		RTCGeometry mesh = rtcNewGeometry(device_, RTC_GEOMETRY_TYPE_TRIANGLE);

		Vertex3f* vertices = (Vertex3f*)rtcSetNewGeometryBuffer(
			mesh, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3,
			sizeof(Vertex3f), 3 * surface->no_triangles());

		Triangle3ui* triangles = (Triangle3ui*)rtcSetNewGeometryBuffer(
			mesh, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3,
			sizeof(Triangle3ui), surface->no_triangles());

		rtcSetGeometryUserData(mesh, (void*)(surface->get_material()));

		rtcSetGeometryVertexAttributeCount(mesh, 2);

		Normal3f* normals = (Normal3f*)rtcSetNewGeometryBuffer(
			mesh, RTC_BUFFER_TYPE_VERTEX_ATTRIBUTE, 0, RTC_FORMAT_FLOAT3,
			sizeof(Normal3f), 3 * surface->no_triangles());

		Coord2f* texCoords = (Coord2f*)rtcSetNewGeometryBuffer(
			mesh, RTC_BUFFER_TYPE_VERTEX_ATTRIBUTE, 1, RTC_FORMAT_FLOAT2,
			sizeof(Coord2f), 3 * surface->no_triangles());

		// triangles loop
		for (int i = 0, k = 0; i < surface->no_triangles(); ++i)
		{
			Triangle& triangle = surface->get_triangle(i);

			// vertices loop
			for (int j = 0; j < 3; ++j, ++k)
			{
				const Vertex& vertex = triangle.vertex(j);

				// Apply translation and scale
				vertices[k].x = vertex.position.x * transform.scale.x + transform.position.x;
				vertices[k].y = vertex.position.y * transform.scale.y + transform.position.y;
				vertices[k].z = vertex.position.z * transform.scale.z + transform.position.z;

				// Normals don't need translation, just scale
				normals[k].x = vertex.normal.x * transform.scale.x;
				normals[k].y = vertex.normal.y * transform.scale.y;
				normals[k].z = vertex.normal.z * transform.scale.z;

				// Normalize the normal
				float length = sqrt(normals[k].x * normals[k].x + normals[k].y * normals[k].y + normals[k].z * normals[k].z);
				if (length > 0.0f) {
					normals[k].x /= length;
					normals[k].y /= length;
					normals[k].z /= length;
				}

				texCoords[k].u = vertex.texture_coords[0].u;
				texCoords[k].v = vertex.texture_coords[0].v;
			} // end of vertices loop

			triangles[i].v0 = k - 3;
			triangles[i].v1 = k - 2;
			triangles[i].v2 = k - 1;
		} // end of triangles loop

		rtcCommitGeometry(mesh);
		unsigned int geom_id = rtcAttachGeometry(scene_, mesh);
		rtcReleaseGeometry(mesh);
	} // end of surfaces loop
}

void RayTracer::LoadScene(
	const std::vector<ModelInfo>& models,
	const std::vector<Shape*>& shapes,
	const char* cubeMapFileNames[6]
)
{
	cubemap_ = new CubeMap(cubeMapFileNames);

	// Load all OBJ models with their transformations
	for (const auto& model : models)
	{
		LoadModel(model.filePath, model.transform);
	}

	for (const auto& shape : shapes)
	{
		volumetricShapes_.push_back(shape);
	}

	// Commit scene
	rtcCommitScene(scene_);
}

float clamp(const float x, const float x0 = 0.0f, const float x1 = 1.0f) {
	return std::max(std::min(x, x1), x0);
}

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

	// setup a hit
	RTCHit hit;
	hit.geomID = RTC_INVALID_GEOMETRY_ID;
	hit.primID = RTC_INVALID_GEOMETRY_ID;
	hit.Ng_x = 0.0f; // geometry normal
	hit.Ng_y = 0.0f;
	hit.Ng_z = 0.0f;

	// merge ray and hit structures
	RTCRayHit rayHit;
	rayHit.ray = ray;
	rayHit.hit = hit;

	// intersect ray with the scene
	RTCIntersectContext context;
	rtcInitIntersectContext(&context);
	rtcIntersect1(scene_, &context, &rayHit);

	if (rayHit.hit.geomID != RTC_INVALID_GEOMETRY_ID) {
		return false;
	}
	else {
		return true;
	}
}

Vector3 RayTracer::TraceSDFRay(const RTCRay& ray, const int depth, const int maxDepth) {
	if (depth >= maxDepth) {
		return Vector3(0.0f, 0.0f, 0.0f);
	}

	const float stepSize = 0.01f;
	const float maxDistance = 100.0f;
	const int maxSteps = 1000;

	float distance = 0.0f;
	Vector3 position(ray.org_x, ray.org_y, ray.org_z);
	Vector3 direction(ray.dir_x, ray.dir_y, ray.dir_z);

	for (int i = 0; i < maxSteps; ++i) {
		float minDist = FLT_MAX;
		Shape* closestShape = nullptr;

		for (const auto& shape : volumetricShapes_) {
			float dist = shape->SDF(position);
			if (dist < minDist) {
				minDist = dist;
				closestShape = shape;
			}
		}

		if (minDist < 0.001f && closestShape != nullptr) {
			Vector3 normal = ComputeNormal(position, *closestShape);
			normal.Normalize();

			Vector3 lightDir = light_.GetOrigin() - position;
			lightDir.Normalize();

			// ambient
			Vector3 ambientColor(0.025f, 0.025f, 0.025f);
			Vector3 color = ambientColor;

			// diffuse
			float diffuse = std::max(0.0f, normal.DotProduct(lightDir));
			if (IsHitPointVisible(position, light_.GetOrigin())) {
				color += Vector3(diffuse, diffuse, diffuse);
			}

			// specular
			Vector3 viewDir = -direction;
			Vector3 reflectDir = 2.0f * normal.DotProduct(lightDir) * normal - lightDir;
			float spec = powf(std::max(viewDir.DotProduct(reflectDir), 0.0f), 32.0f); // Shininess = 32
			color += Vector3(spec, spec, spec);

			return color;
		}

		if (distance > maxDistance) {
			break;
		}

		distance += minDist;
		position = position + direction * minDist;
	}

	Vector3 directionVector(ray.dir_x, ray.dir_y, ray.dir_z);
	directionVector.Normalize();
	Color3f backgroundColor = cubemap_->GetTexel(directionVector);
	return Vector3(backgroundColor.r, backgroundColor.g, backgroundColor.b);
}


Vector3 RayTracer::ComputeNormal(const Vector3& p, const Shape& shape) {
	const float eps = 0.001f;

	float dx = shape.SDF(p + Vector3(eps, 0.0f, 0.0f)) - shape.SDF(p - Vector3(eps, 0.0f, 0.0f));
	float dy = shape.SDF(p + Vector3(0.0f, eps, 0.0f)) - shape.SDF(p - Vector3(0.0f, eps, 0.0f));
	float dz = shape.SDF(p + Vector3(0.0f, 0.0f, eps)) - shape.SDF(p - Vector3(0.0f, 0.0f, eps));

	return Vector3(dx, dy, dz) / (2.0f * eps);
}


Vector3 RayTracer::TraceRay(const RTCRay& ray, const float n_1, const int depth, const int maxDepth) {
    if (sdfMode_) {
        return TraceSDFRay(ray, depth, maxDepth);
    }

    if (depth >= maxDepth) {
        return Vector3(0.0f, 0.0f, 0.0f);
    }

    // setup a hit
    RTCHit hit;
    hit.geomID = RTC_INVALID_GEOMETRY_ID;
    hit.primID = RTC_INVALID_GEOMETRY_ID;
    hit.Ng_x = 0.0f; // geometry normal
    hit.Ng_y = 0.0f;
    hit.Ng_z = 0.0f;

    // merge ray and hit structures
    RTCRayHit rayHit;
    rayHit.ray = ray;
    rayHit.hit = hit;

    // intersect ray with the scene
    RTCIntersectContext context;
    rtcInitIntersectContext(&context);
    rtcIntersect1(scene_, &context, &rayHit);

    Vector3 directionVector{ rayHit.ray.dir_x, rayHit.ray.dir_y, rayHit.ray.dir_z };

    if (rayHit.hit.geomID == RTC_INVALID_GEOMETRY_ID)
    {
        Color3f backgroundColor = cubemap_->GetTexel(directionVector);
        return Vector3{ backgroundColor.r, backgroundColor.g, backgroundColor.b };
    }

    const Vector3 originPoint{ rayHit.ray.org_x, rayHit.ray.org_y, rayHit.ray.org_z };
    const Vector3 hitPoint{
        rayHit.ray.org_x + rayHit.ray.dir_x * rayHit.ray.tfar,
        rayHit.ray.org_y + rayHit.ray.dir_y * rayHit.ray.tfar,
        rayHit.ray.org_z + rayHit.ray.dir_z * rayHit.ray.tfar
    };

    // we hit something
    RTCGeometry geometry = rtcGetGeometry(scene_, rayHit.hit.geomID);

    Normal3f normal;
    // get interpolated normal
    rtcInterpolate0(geometry, rayHit.hit.primID, rayHit.hit.u, rayHit.hit.v,
        RTC_BUFFER_TYPE_VERTEX_ATTRIBUTE, 0, &normal.x, 3);
    Vector3 normalVector{ normal.x, normal.y, normal.z };

    if (normalVector.DotProduct(directionVector) > 0.0f) {
        normalVector *= -1.0f;
    }

    // and texture coordinates
    Coord2f texCoord;
    rtcInterpolate0(geometry, rayHit.hit.primID, rayHit.hit.u, rayHit.hit.v,
        RTC_BUFFER_TYPE_VERTEX_ATTRIBUTE, 1, &texCoord.u, 2);

    Material* material = (Material*)rtcGetGeometryUserData(geometry);
    assert(material);

    if (normalShader_) {
        return NormalShader(normalVector);
    }
    else if (lambertShader_) {
        return LambertShader(*material, texCoord, hitPoint, normalVector);
    }
    else {
        if (material->shader == 3) {
            return PhongShader(*material, texCoord, hitPoint, normalVector, directionVector, depth);
        }
        else if (material->shader == 4) {
            return TransparentShader(ray, hitPoint, normalVector, directionVector, *material, n_1, depth);
        }
    }
    return normalVector;
}

Vector3 RayTracer::NormalShader(const Vector3& normalVector) {
	return (normalVector + Vector3(1.0f, 1.0f, 1.0f)) * 0.5f;
}

Vector3 RayTracer::LambertShader(const Material& material, const Coord2f& texCoord, const Vector3& hitPoint, const Vector3& normalVector) {
	Vector3 outputColor;

	Vector3 diffuseColor = Vector3{ material.diffuse.x, material.diffuse.y, material.diffuse.z };
	Texture* diffuseTexture = material.get_texture(Material::kDiffuseMapSlot);
	if (diffuseTexture) {
		Color3f diffuseTexel = diffuseTexture->get_texel(texCoord.u, 1 - texCoord.v);
		diffuseColor.x = diffuseTexel.r;
		diffuseColor.y = diffuseTexel.g;
		diffuseColor.z = diffuseTexel.b;
	}

	const Vector3 omniLightPosition = light_.GetOrigin();

	Vector3 l = omniLightPosition - hitPoint;
	l.Normalize();

	outputColor += material.ambient;

	if (IsHitPointVisible(hitPoint, omniLightPosition)) {
		outputColor += diffuseColor * clamp(normalVector.DotProduct(l));
	}

	return outputColor;
}

Vector3 RayTracer::PhongShader(const Material& material, const Coord2f& texCoord, const Vector3& hitPoint, const Vector3& normalVector, const Vector3& directionVector, const int depth) {
	Vector3 outputColor;

	Vector3 diffuseColor = Vector3{ material.diffuse.x, material.diffuse.y, material.diffuse.z };
	Texture* diffuseTexture = material.get_texture(Material::kDiffuseMapSlot);
	if (diffuseTexture) {
		Color3f diffuseTexel = diffuseTexture->get_texel(texCoord.u, 1 - texCoord.v);
		diffuseColor.x = diffuseTexel.r;
		diffuseColor.y = diffuseTexel.g;
		diffuseColor.z = diffuseTexel.b;
	}

	Vector3 specularColor = Vector3{ material.specular.x, material.specular.y, material.specular.z };
	Texture* specularTexture = material.get_texture(Material::kSpecularMapSlot);
	if (specularTexture) {
		Color3f specularTexel = specularTexture->get_texel(texCoord.u, 1.0f - texCoord.v);
		specularColor.x = specularTexel.r;
		specularColor.y = specularTexel.g;
		specularColor.z = specularTexel.b;
	}

	const Vector3 omniLightPosition = light_.GetOrigin();

	Vector3 l = omniLightPosition - hitPoint;
	l.Normalize();

	outputColor += material.ambient;

	if (IsHitPointVisible(hitPoint, omniLightPosition)) {

		outputColor += diffuseColor * clamp(normalVector.DotProduct(l));

		Vector3 l_r = 2.0f * (l.DotProduct(normalVector) * normalVector) - l;
		l_r.Normalize();

		RTCRay secondaryRay = MakeSecondaryRay(hitPoint, l);

		Vector3 l_i = TraceRay(secondaryRay, depth + 1.0f);

		outputColor += specularColor * powf(clamp(l_r.DotProduct(-directionVector)), material.shininess);

		outputColor += l_i * specularColor * 0.2f;
	}

	return outputColor;
}

Vector3 t_b_l(const float length, const Vector3 attenuation) {
	Vector3 a = Vector3{ powf(exp(1.0f), -attenuation.x * length), powf(exp(1.0f), -attenuation.y * length), powf(exp(1.0f), -attenuation.z * length) };
	return a;
}

Vector3 RayTracer::TransparentShader(const RTCRay& ray, const Vector3& hitPoint, const Vector3& normalVector, const Vector3& directionVector, const Material& material, const float n_1, const int depth) {
	float n_2 = material.ior;
	if (n_1 == material.ior) {
		n_2 = 1.0f;
	}

	float n_ratio = n_1 / n_2;
	float r;

	float cos_theta1 = normalVector.DotProduct(-directionVector);
	float temp = 1.0f - powf(n_1 / n_2, 2.0f) * (1.0f - powf(cos_theta1, 2.0f));
	float cos_theta2 = sqrt(temp);
	if (temp < 0.0f) {
		r = 1.0f;
	}
	else {
		float r_s = powf((n_2 * cos_theta2 - n_1 * cos_theta1) / (n_2 * cos_theta2 + n_1 * cos_theta1), 2.0f);
		float r_p = powf((n_2 * cos_theta1 - n_1 * cos_theta2) / (n_2 * cos_theta1 + n_1 * cos_theta2), 2.0f);
		r = (r_s + r_p) / 2.0f;
	}


	Vector3 refractedRayDirection = n_1 / n_2 * directionVector + (n_1 / n_2 * cos_theta1 - cos_theta2) * normalVector;
	Vector3 reflectedRayDirection = (2.0f * (-directionVector).DotProduct(normalVector)) * normalVector - (-directionVector);
	Vector3 refl = TraceRay(MakeSecondaryRay(hitPoint, reflectedRayDirection), n_1, depth + 1);
	Vector3 refr = TraceRay(MakeSecondaryRay(hitPoint, refractedRayDirection), n_2, depth + 1);
	return (refl * r + refr * (1.0f - r)) * t_b_l(n_1 == 1.0f ? 0.0f : Vector3{ ray.org_x, ray.org_y, ray.org_z }.EuclideanDistance(hitPoint), material.attenuation);
}

Color4f RayTracer::GetPixel(const int x, const int y, const float t)
{
    if (sampling_) {
        Vector3 accumulator;

        for (int j = 0; j < 4; j++) {
            for (int i = 0; i < 4; i++) {
                float ksi_x = Random() / 4;
                float ksi_y = Random() / 4;

                RTCRay ray = camera_.GenerateRay(float(x) + i * (1.0f / 4.0f) + ksi_x, float(y) + j * (1.0f / 4.0f) + ksi_y);
                accumulator += TraceRay(ray);
            }
        }
        accumulator /= 4.0f * 4.0f;

        return accumulator.ToColor4fCompressed();
    }
    else {
        return TraceRay(camera_.GenerateRay(float(x), float(y))).ToColor4fCompressed();
    }
}

void RayTracer::MoveCamera()
{
	if (cameraMovementEnabled_) {
		static float angle = 0.0f;
		const float speed = 0.025f; 

		Vector3 viewAt = camera_.GetViewAt();
		Vector3 viewFrom = camera_.GetViewFrom();

		// Vypočítame aktuálnu vzdialenosť medzi viewFrom a viewAt v rovine x-y
		Vector3 direction = viewFrom - viewAt;
		float currentRadius = sqrt(direction.x * direction.x + direction.y * direction.y); // Ignorujeme z-ovú súradnicu

		// Meníme iba x a y pozíciu kamery
		cameraX_ = viewAt.x + currentRadius * cosf(angle);
		cameraY_ = viewAt.y + currentRadius * sinf(angle);
		// Zachováme pôvodnú výšku kamery
		cameraZ_ = viewFrom.z;

		angle += speed;
		if (angle > 2.0f * (float)M_PI) {
			angle -= 2.0f * (float)M_PI;
		}

		// Nastavíme novú pozíciu kamery
		camera_.SetViewFrom(Vector3(cameraX_, cameraY_, cameraZ_));
	}
}

int RayTracer::Ui()
{
	static float f = 0.0f;
	static int counter = 0;

	// Use a Begin/End pair to created a named window
	ImGui::Begin("Ray Tracer Params");

	ImGui::Text("Surfaces = %d", surfaces_.size());
	ImGui::Text("Materials = %d", materials_.size());
	ImGui::Separator();
	if (ImGui::RadioButton("Normal", normalShader_)) {
		normalShader_ = true;
		lambertShader_ = false;
		phongShader_ = false;
		sdfMode_ = false;
	}
	ImGui::SameLine();
	if (ImGui::RadioButton("Lambert", lambertShader_)) {
		normalShader_ = false;
		lambertShader_ = true;
		phongShader_ = false;
		sdfMode_ = false;
	}
	ImGui::SameLine();
	if (ImGui::RadioButton("Phong/Whitted", phongShader_)) {
		normalShader_ = false;
		lambertShader_ = false;
		phongShader_ = true;
		sdfMode_ = false;
	}
	ImGui::SameLine();
	if (ImGui::RadioButton("SDF", sdfMode_)) {
		normalShader_ = false;
		lambertShader_ = false;
		phongShader_ = false;
		sdfMode_ = true;
	}

	ImGui::Checkbox("Camera Movement", &cameraMovementEnabled_);
	ImGui::Checkbox("Vsync", &vsync_);
	ImGui::Checkbox("Sampling", &sampling_);

    //ImGui::Checkbox( "Demo Window", &show_demo_window ); // Edit bools storing our window open/close state
    //ImGui::Checkbox( "Another Window", &show_another_window );

    //ImGui::SliderFloat( "float", &f, 0.0f, 1.0f ); // Edit 1 float using a slider from 0.0f to 1.0f  
    //ImGui::ColorEdit3( "clear color", ( float* )&clear_color ); // Edit 3 floats representing a color
	
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

    // Camera rotation
    static int cameraRotationX = 0;
    static int cameraRotationY = 0;
    static int cameraRotationZ = 0;
    static int previousDragDeltaX = 0;
    static int previousDragDeltaY = 0;
    static int previousRotationZ = 0;

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

    // Light position
    static int lightX = (int)light_.GetOrigin().x;
    static int lightY = (int)light_.GetOrigin().y;
    static int lightZ = (int)light_.GetOrigin().z;
    if (ImGui::SliderInt("Light X", &lightX, -1000, 1000)) {
        light_.SetOrigin(Vector3(static_cast<float>(lightX), static_cast<float>(lightY), static_cast<float>(lightZ)));
    }
    if (ImGui::SliderInt("Light Y", &lightY, -1000, 1000)) {
        light_.SetOrigin(Vector3(static_cast<float>(lightX), static_cast<float>(lightY), static_cast<float>(lightZ)));
    }
    if (ImGui::SliderInt("Light Z", &lightZ, -1000, 1000)) {
        light_.SetOrigin(Vector3(static_cast<float>(lightX), static_cast<float>(lightY), static_cast<float>(lightZ)));
    }

    // Save video button
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

	// 3. Show another simple window.
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