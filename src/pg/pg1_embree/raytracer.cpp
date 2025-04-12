#include "stdafx.h"
#include "raytracer.h"
#include "objloader.h"
#include "tutorials.h"
#include "utils.h"
#include <iostream>
#include "smooth_union.h"


Raytracer::Raytracer(const int width, const int height,
	const float fov_y, const Vector3 view_from, const Vector3 view_at, const Vector3 light_org,
	const char* config) : SimpleGuiDX11(width, height)
{
	InitDeviceAndScene(config);

	camera_ = Camera(width, height, fov_y, view_from, view_at);
	light_ = Light(light_org);
}

Raytracer::~Raytracer()
{
	ReleaseDeviceAndScene();
}

int Raytracer::InitDeviceAndScene(const char* config)
{
	device_ = rtcNewDevice(config);
	error_handler(nullptr, rtcGetDeviceError(device_), "Unable to create a new device.\n");
	rtcSetDeviceErrorFunction(device_, error_handler, nullptr);

	ssize_t triangle_supported = rtcGetDeviceProperty(device_, RTC_DEVICE_PROPERTY_TRIANGLE_GEOMETRY_SUPPORTED);

	// create a new scene bound to the specified device
	scene_ = rtcNewScene(device_);

	return S_OK;
}

int Raytracer::ReleaseDeviceAndScene()
{
	rtcReleaseScene(scene_);
	rtcReleaseDevice(device_);

	return S_OK;
}

void Raytracer::LoadScene(const std::string file_name, const char* cube_map_file_names[6])
{
	cubemap_ = new CubeMap(cube_map_file_names);

	const int no_surfaces = LoadOBJ(file_name.c_str(), surfaces_, materials_);

	// Pridáme testovaciu guľu
	//Sphere* test_sphere = new Sphere(Vector3(0.0f, 0.0f, 0.0f), 0.5f);
	//volumetric_objects_.push_back(test_sphere);

	// Vytvoríme dve gule
	Sphere* sphere1 = new Sphere(Vector3(0.0f, 0.0f, 0.0f), 0.5f);
	Sphere* sphere2 = new Sphere(Vector3(0.5f, 0.0f, 0.0f), 0.1f);
	
	// Vytvoríme smooth union z týchto dvoch gúľ
	SmoothUnion* smooth_union = new SmoothUnion(sphere1, sphere2, 0.5f);
	volumetric_objects_.push_back(smooth_union);

	// surfaces loop
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

		Coord2f* tex_coords = (Coord2f*)rtcSetNewGeometryBuffer(
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

				vertices[k].x = vertex.position.x;
				vertices[k].y = vertex.position.y;
				vertices[k].z = vertex.position.z;

				normals[k].x = vertex.normal.x;
				normals[k].y = vertex.normal.y;
				normals[k].z = vertex.normal.z;

				tex_coords[k].u = vertex.texture_coords[0].u;
				tex_coords[k].v = vertex.texture_coords[0].v;
			} // end of vertices loop

			triangles[i].v0 = k - 3;
			triangles[i].v1 = k - 2;
			triangles[i].v2 = k - 1;
		} // end of triangles loop

		rtcCommitGeometry(mesh);
		unsigned int geom_id = rtcAttachGeometry(scene_, mesh);
		rtcReleaseGeometry(mesh);
	} // end of surfaces loop

	rtcCommitScene(scene_);
}

float clamp(const float x, const float x0 = 0.0f, const float x1 = 1.0f) {
	return max(min(x, x1), x0);
}

RTCRay MakeSecondaryRay(const Vector3 origin, const Vector3 dir) {

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

bool Raytracer::IsHitPointVisible(const Vector3 hitPoint, const Vector3 lightPoint) {
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
	RTCRayHit ray_hit;
	ray_hit.ray = ray;
	ray_hit.hit = hit;

	// intersect ray with the scene
	RTCIntersectContext context;
	rtcInitIntersectContext(&context);
	rtcIntersect1(scene_, &context, &ray_hit);

	if (ray_hit.hit.geomID != RTC_INVALID_GEOMETRY_ID) {
		return false;
	}
	else {
		return true;
	}
}

Vector3 Raytracer::TraceSDFRay(RTCRay ray, const int depth, const int max_depth) {
    if (depth >= max_depth) {
        return Vector3(0.0f, 0.0f, 0.0f);
    }

    const float step_size = 0.01f;
    const float max_distance = 100.0f;
    const int max_steps = 1000;
    
    float distance = 0.0f;
    Vector3 position(ray.org_x, ray.org_y, ray.org_z);
    Vector3 direction(ray.dir_x, ray.dir_y, ray.dir_z);
    
    for (int i = 0; i < max_steps; ++i) {
        float min_dist = FLT_MAX;
        
        // Prechádzame všetkými volumetrickými objektmi
        for (const auto& shape : volumetric_objects_) {
            float dist = shape->SDF(position);
            min_dist = min(min_dist, dist);
        }
        
        if (min_dist < 0.001f) {
            // Priesečník s SDF guľou
            return Vector3(1.0f, 1.0f, 1.0f); // Biela farba pre guľu
        }
        
        if (distance > max_distance) {
            break;
        }
        
        distance += min_dist;
        position = position + direction * min_dist;
    }
    
    // Ak nenájdeme priesečník, vrátime farbu z cubemap
    Vector3 direction_vector(ray.dir_x, ray.dir_y, ray.dir_z);
    direction_vector.Normalize();
    Color3f background_color = cubemap_->get_texel(direction_vector);
    return Vector3(background_color.r, background_color.g, background_color.b);
}

Vector3 Raytracer::TraceRay(RTCRay ray, const float n_1, const int depth, const int max_depth) {
    if (sdf_mode) {
        return TraceSDFRay(ray, depth, max_depth);
    }

    if (depth >= max_depth) {
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
    RTCRayHit ray_hit;
    ray_hit.ray = ray;
    ray_hit.hit = hit;

    // intersect ray with the scene
    RTCIntersectContext context;
    rtcInitIntersectContext(&context);
    rtcIntersect1(scene_, &context, &ray_hit);

    Vector3 direction_vector{ ray_hit.ray.dir_x, ray_hit.ray.dir_y, ray_hit.ray.dir_z };

    if (ray_hit.hit.geomID == RTC_INVALID_GEOMETRY_ID)
    {
        Color3f background_color = cubemap_->get_texel(direction_vector);
        return Vector3{ background_color.r, background_color.g, background_color.b };
    }

    const Vector3 origin_point{ ray_hit.ray.org_x, ray_hit.ray.org_y, ray_hit.ray.org_z };
    const Vector3 hit_point{
        ray_hit.ray.org_x + ray_hit.ray.dir_x * ray_hit.ray.tfar,
        ray_hit.ray.org_y + ray_hit.ray.dir_y * ray_hit.ray.tfar,
        ray_hit.ray.org_z + ray_hit.ray.dir_z * ray_hit.ray.tfar
    };

    // we hit something
    RTCGeometry geometry = rtcGetGeometry(scene_, ray_hit.hit.geomID);

    Normal3f normal;
    // get interpolated normal
    rtcInterpolate0(geometry, ray_hit.hit.primID, ray_hit.hit.u, ray_hit.hit.v,
        RTC_BUFFER_TYPE_VERTEX_ATTRIBUTE, 0, &normal.x, 3);
    Vector3 normal_vector{ normal.x, normal.y, normal.z };

    if (normal_vector.DotProduct(direction_vector) > 0.0f) {
        normal_vector *= -1.0f;
    }

    // and texture coordinates
    Coord2f tex_coord;
    rtcInterpolate0(geometry, ray_hit.hit.primID, ray_hit.hit.u, ray_hit.hit.v,
        RTC_BUFFER_TYPE_VERTEX_ATTRIBUTE, 1, &tex_coord.u, 2);

    Material* material = (Material*)rtcGetGeometryUserData(geometry);
    assert(material);

    if (normal_shader) {
        return normal_vector;
    }
    else if (lambert_shader) {
        return lambertShader(material, tex_coord, hit_point, normal_vector);
    }
    else {
        if (material->shader == 3) {
            return phongShader(material, tex_coord, hit_point, normal_vector, direction_vector, depth);
        }
        else if (material->shader == 4) {
            return transparentShader(ray, hit_point, normal_vector, direction_vector, material, n_1, depth);
        }
    }
    return normal_vector;
}

Vector3 Raytracer::lambertShader(const Material* material, const Coord2f tex_coord, const Vector3 hit_point, Vector3 normal_vector) {
	Vector3 output_color;

	Vector3 diffuse_color = Vector3{ material->diffuse.x, material->diffuse.y, material->diffuse.z };
	Texture* diffuse_texture = material->get_texture(Material::kDiffuseMapSlot);
	if (diffuse_texture) {
		Color3f diffuse_texel = diffuse_texture->get_texel(tex_coord.u, 1 - tex_coord.v);
		diffuse_color.x = diffuse_texel.r;
		diffuse_color.y = diffuse_texel.g;
		diffuse_color.z = diffuse_texel.b;
	}

	const Vector3 omni_light_position = light_.GetOrigin();

	Vector3 l = omni_light_position - hit_point;
	l.Normalize();

	output_color += material->ambient;

	if (IsHitPointVisible(hit_point, omni_light_position)) {
		output_color += diffuse_color * clamp(normal_vector.DotProduct(l));
	}

	return output_color;
}

Vector3 Raytracer::phongShader(const Material* material, const Coord2f tex_coord, const Vector3 hit_point, Vector3 normal_vector, const Vector3 direction_vector, const int depth) {
	Vector3 output_color;

	Vector3 diffuse_color = Vector3{ material->diffuse.x, material->diffuse.y, material->diffuse.z };
	Texture* diffuse_texture = material->get_texture(Material::kDiffuseMapSlot);
	if (diffuse_texture) {
		Color3f diffuse_texel = diffuse_texture->get_texel(tex_coord.u, 1 - tex_coord.v);
		diffuse_color.x = diffuse_texel.r;
		diffuse_color.y = diffuse_texel.g;
		diffuse_color.z = diffuse_texel.b;
	}

	Vector3 specular_color = Vector3{ material->specular.x, material->specular.y, material->specular.z };
	Texture* specular_texture = material->get_texture(Material::kSpecularMapSlot);
	if (specular_texture) {
		Color3f specular_texel = specular_texture->get_texel(tex_coord.u, 1 - tex_coord.v);
		specular_color.x = specular_texel.r;
		specular_color.y = specular_texel.g;
		specular_color.z = specular_texel.b;
	}

	const Vector3 omni_light_position = light_.GetOrigin();

	Vector3 l = omni_light_position - hit_point;
	l.Normalize();

	output_color += material->ambient;

	if (IsHitPointVisible(hit_point, omni_light_position)) {

		output_color += diffuse_color * clamp(normal_vector.DotProduct(l));

		Vector3 l_r = 2 * (l.DotProduct(normal_vector) * normal_vector) - l;
		l_r.Normalize();

		RTCRay secondary_ray = MakeSecondaryRay(hit_point, l);

		Vector3 l_i = TraceRay(secondary_ray, depth + 1);

		output_color += specular_color * powf(clamp(l_r.DotProduct(-direction_vector)), material->shininess);

		output_color += l_i * specular_color * 0.2f;
	}

	return output_color;
}

Vector3 t_b_l(const float length, const Vector3 attenuation) {
	Vector3 a = Vector3{ powf(exp(1.0f), -attenuation.x * length), powf(exp(1.0f), -attenuation.y * length), powf(exp(1.0f), -attenuation.z * length) };
	return a;
}

double euclid_distance(Vector3 a, Vector3 b) {
	return sqrt(powf(b.x - a.x, 2) + powf(b.y - a.y, 2) + powf(b.z - a.z, 2));
}

Vector3 Raytracer::transparentShader(RTCRay ray, const Vector3 hit_point, Vector3 normal_vector, const Vector3 direction_vector, const Material* material, float n_1, const int depth) {
	float n_2 = material->ior;
	if (n_1 == material->ior) {
		n_2 = 1.0f;
	}

	float n_ratio = n_1 / n_2;
	float r;

	float cos_theta1 = normal_vector.DotProduct(-direction_vector);
	float temp = 1.0f - powf(n_1 / n_2, 2.0f) * (1.0f - powf(cos_theta1, 2.0f));
	float cos_theta2 = sqrt(temp);
	if (temp < 0.0f) {
		r = 1.0f;
	}
	else {
		float r_s = powf((n_2 * cos_theta2 - n_1 * cos_theta1) / (n_2 * cos_theta2 + n_1 * cos_theta1), 2);
		float r_p = powf((n_2 * cos_theta1 - n_1 * cos_theta2) / (n_2 * cos_theta1 + n_1 * cos_theta2), 2);
		r = (r_s + r_p) / 2.0f;
	}


	Vector3 refracted_ray_direction = n_1 / n_2 * direction_vector + (n_1 / n_2 * cos_theta1 - cos_theta2) * normal_vector;
	Vector3 reflected_ray_direction = (2.0f * (-direction_vector).DotProduct(normal_vector)) * normal_vector - (-direction_vector);
	Vector3 refl = TraceRay(MakeSecondaryRay(hit_point, reflected_ray_direction), n_1, depth + 1);
	Vector3 refr = TraceRay(MakeSecondaryRay(hit_point, refracted_ray_direction), n_2, depth + 1);
	return (refl * r + refr * (1.0 - r)) * t_b_l(n_1 == 1.0f ? 0.0f : euclid_distance(Vector3{ ray.org_x, ray.org_y, ray.org_z }, hit_point), material->attenuation);
}

Vector3 Raytracer::RayMarching(RTCRay ray, float& t) {
	const float step_size = 0.01f;
	const float max_distance = 100.0f;
	const int max_steps = 1000;
	
	float distance = 0.0f;
	Vector3 position(ray.org_x, ray.org_y, ray.org_z);
	Vector3 direction(ray.dir_x, ray.dir_y, ray.dir_z);
	
	for (int i = 0; i < max_steps; ++i) {
		float min_dist = FLT_MAX;
		
		// Prechádzame všetkými volumetrickými objektmi
		for (const auto& shape : volumetric_objects_) {
			float dist = shape->SDF(position);
			min_dist = min(min_dist, dist);
		}
		
		if (min_dist < 0.001f) {
			t = distance;
			return position;
		}
		
		if (distance > max_distance) {
			break;
		}
		
		distance += min_dist;
		position = position + direction * min_dist;
	}
	
	t = FLT_MAX;
	return Vector3(0.0f, 0.0f, 0.0f);
}

Vector3 Raytracer::SampleVolume(const Vector3& position) {
	float min_dist = FLT_MAX;
	Shape* closest_shape = nullptr;
	
	for (const auto& shape : volumetric_objects_) {
		float dist = shape->SDF(position);
		if (dist < min_dist) {
			min_dist = dist;
			closest_shape = shape;
		}
	}
	
	if (closest_shape) {
		// Tu môžeme pridať vlastnú logiku pre vzorkovanie objemu
		// Napríklad na základe vzdialenosti od stredu gule
		return Vector3(1.0f, 1.0f, 1.0f); // Predvolená hodnota
	}
	
	return Vector3(0.0f, 0.0f, 0.0f);
}

float Raytracer::HenyeyGreenstein(const Vector3& wi, const Vector3& wo, float g) {
	float cos_theta = wi.DotProduct(wo);
	float g2 = g * g;
	return (1.0f - g2) / (4.0f * M_PI * pow(1.0f + g2 - 2.0f * g * cos_theta, 1.5f));
}

Vector3 Raytracer::ComputeTransmittance(const Vector3& start, const Vector3& end, float step_size) {
	Vector3 transmittance(1.0f, 1.0f, 1.0f);
	Vector3 direction = end - start;
	direction.Normalize();
	float distance = (end - start).L2Norm();
	int steps = static_cast<int>(distance / step_size);
	
	for (int i = 0; i < steps; ++i) {
		Vector3 position = start + direction * (i * step_size);
		Vector3 density = SampleVolume(position);
		
		transmittance = transmittance * Vector3(
			exp(-density.x * step_size),
			exp(-density.y * step_size),
			exp(-density.z * step_size)
		);
	}
	
	return transmittance;
}

Vector3 Raytracer::ComputeInScattering(const Vector3& start, const Vector3& end, float step_size, const Vector3& light_dir) {
	Vector3 in_scattering(0.0f, 0.0f, 0.0f);
	Vector3 direction = end - start;
	direction.Normalize();
	float distance = (end - start).L2Norm();
	int steps = static_cast<int>(distance / step_size);
	
	for (int i = 0; i < steps; ++i) {
		Vector3 position = start + direction * (i * step_size);
		Vector3 density = SampleVolume(position);
		
		// Vypočítame transmittance pre aktuálny bod
		Vector3 transmittance = ComputeTransmittance(position, end, step_size);
		
		// Pridáme príspevok k in-scattering
		in_scattering = in_scattering + density * transmittance * step_size;
	}
	
	return in_scattering;
}

Vector3 Raytracer::TraceVolumetricRay(RTCRay ray, const int depth, const int max_depth) {
	if (depth >= max_depth) {
		return Vector3(0.0f, 0.0f, 0.0f);
	}
	
	float t;
	Vector3 hit_point = RayMarching(ray, t);
	
	if (t == FLT_MAX) {
		return Vector3(0.0f, 0.0f, 0.0f); // Žiadny priesečník
	}
	
	// Vypočítame in-scattering
	Vector3 light_dir = light_.GetOrigin() - hit_point;
	light_dir.Normalize();
	Vector3 in_scattering = ComputeInScattering(
		Vector3(ray.org_x, ray.org_y, ray.org_z),
		hit_point,
		0.01f,
		light_dir
	);
	
	// Vypočítame transmittance
	Vector3 transmittance = ComputeTransmittance(
		Vector3(ray.org_x, ray.org_y, ray.org_z),
		hit_point,
		0.01f
	);
	
	// Kombinujeme výsledky
	return in_scattering + transmittance;
}



Color4f Raytracer::get_pixel(const int x, const int y, const float t)
{
	if (sampling) {
		Vector3 accumulator;

		for (int j = 0; j < 4; j++) {
			for (int i = 0; i < 4; i++) {
				float ksi_x = Random() / 4;
				float ksi_y = Random() / 4;

				RTCRay ray = camera_.GenerateRay(float(x) + i * (1.0f / 4) + ksi_x, float(y) + j * (1.0f / 4) + ksi_y);
				accumulator += TraceRay(ray);
			}
		}
		accumulator /= 4 * 4;

		return accumulator.ToColor4fCompressed();

	}
	else {
		return TraceRay(camera_.GenerateRay(float(x), float(y))).ToColor4fCompressed();
	}
}


int Raytracer::Ui()
{
	static float f = 0.0f;
	static int counter = 0;

	// Use a Begin/End pair to created a named window
	ImGui::Begin("Ray Tracer Params");

	ImGui::Text("Surfaces = %d", surfaces_.size());
	ImGui::Text("Materials = %d", materials_.size());
	ImGui::Separator();
	if (ImGui::RadioButton("Normal", normal_shader)) {
		normal_shader = true;
		lambert_shader = false;
		phong_shader = false;
		sdf_mode = false;
	}
	ImGui::SameLine();
	if (ImGui::RadioButton("Lambert", lambert_shader)) {
		normal_shader = false;
		lambert_shader = true;
		phong_shader = false;
		sdf_mode = false;
	}
	ImGui::SameLine();
	if (ImGui::RadioButton("Phong/Whitted", phong_shader)) {
		normal_shader = false;
		lambert_shader = false;
		phong_shader = true;
		sdf_mode = false;
	}
	ImGui::SameLine();
	if (ImGui::RadioButton("SDF", sdf_mode)) {
		normal_shader = false;
		lambert_shader = false;
		phong_shader = false;
		sdf_mode = true;
	}

	ImGui::Checkbox("Vsync", &vsync_);
	ImGui::Checkbox("Sampling", &sampling);

	//ImGui::Checkbox( "Demo Window", &show_demo_window ); // Edit bools storing our window open/close state
	//ImGui::Checkbox( "Another Window", &show_another_window );

	//ImGui::SliderFloat( "float", &f, 0.0f, 1.0f ); // Edit 1 float using a slider from 0.0f to 1.0f  
	//ImGui::ColorEdit3( "clear color", ( float* )&clear_color ); // Edit 3 floats representing a color

	static int camera_x = camera_.GetViewFrom().x;
	static int camera_y = camera_.GetViewFrom().y;
	static int camera_z = camera_.GetViewFrom().z;
	if (ImGui::SliderInt("Camera X", &camera_x, -100, 100)) {
		camera_.SetViewFrom(Vector3(static_cast<float>(camera_x), static_cast<float>(camera_y), static_cast<float>(camera_z)));
	}
	if (ImGui::SliderInt("Camera Y", &camera_y, -100, 100)) {
		camera_.SetViewFrom(Vector3(static_cast<float>(camera_x), static_cast<float>(camera_y), static_cast<float>(camera_z)));
	}
	if (ImGui::SliderInt("Camera Z", &camera_z, -100, 100)) {
		camera_.SetViewFrom(Vector3(static_cast<float>(camera_x), static_cast<float>(camera_y), static_cast<float>(camera_z)));
	}

	static int camera_rotation_x = 0;
	static int camera_rotation_y = 0;
	static int camera_rotation_z = 0;
	static int previous_drag_delta_x = 0;
	static int previous_drag_delta_y = 0;
	static int previous_rotation_z = 0;
	if (ImGui::SliderInt("Camera Rotation X", &camera_rotation_x, 0, 359)) {
		Vector3 current_rotation = camera_.GetRotation();
		current_rotation.x = camera_rotation_x;
		camera_.SetRotation(current_rotation);
	}
	if (ImGui::SliderInt("Camera Rotation Y", &camera_rotation_y, 0, 359)) {
		Vector3 current_rotation = camera_.GetRotation();
		current_rotation.y = camera_rotation_y;
		camera_.SetRotation(current_rotation);
	}
	if (ImGui::SliderInt("Camera Rotation Z", &camera_rotation_z, 0, 359)) {
		Vector3 current_rotation = camera_.GetRotation();
		current_rotation.z = camera_rotation_z;
		camera_.SetRotation(current_rotation);
	}

	if (ImGui::IsMouseDown(1)) {
		if (ImGui::IsMouseDragging(1)) {
			ImVec2 drag_delta = ImGui::GetMouseDragDelta(1);
			int current_drag_delta_y = static_cast<int>(drag_delta.x);
			int current_drag_delta_x = static_cast<int>(drag_delta.y);
			Vector3 current_rotation = camera_.GetRotation();
			current_rotation.x += current_drag_delta_x / 4 - previous_drag_delta_x / 4;
			current_rotation.y += current_drag_delta_y / 4 - previous_drag_delta_y / 4;
			camera_.SetRotation(current_rotation);
			previous_drag_delta_x = current_drag_delta_x;
			previous_drag_delta_y = current_drag_delta_y;

			current_rotation = camera_.GetRotation();
			camera_rotation_x = current_rotation.x;
			camera_rotation_y = current_rotation.y;
		}
	}
	if (ImGui::IsMouseReleased(1)) {
		previous_drag_delta_x = 0;
		previous_drag_delta_y = 0;
	}

	static int light_x = light_.GetOrigin().x;
	static int light_y = light_.GetOrigin().y;
	static int light_z = light_.GetOrigin().z;
	if (ImGui::SliderInt("Light X", &light_x, -1000, 1000)) {
		light_.SetOrigin(Vector3(static_cast<float>(light_x), static_cast<float>(light_y), static_cast<float>(light_z)));
	}
	if (ImGui::SliderInt("Light Y", &light_y, -1000, 1000)) {
		light_.SetOrigin(Vector3(static_cast<float>(light_x), static_cast<float>(light_y), static_cast<float>(light_z)));
	}
	if (ImGui::SliderInt("Light Z", &light_z, -1000, 1000)) {
		light_.SetOrigin(Vector3(static_cast<float>(light_x), static_cast<float>(light_y), static_cast<float>(light_z)));
	}

	// Buttons return true when clicked (most widgets return true when edited/activated)
	if (ImGui::Button("Button"))
		counter++;
	ImGui::SameLine();
	ImGui::Text("counter = %d", counter);

	ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
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