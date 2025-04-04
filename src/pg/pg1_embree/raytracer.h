#pragma once
#include "simpleguidx11.h"
#include "surface.h"
#include "camera.h"
#include "light.h"
#include "cubemap.h"
#include "mymath.h"

/*! \class Raytracer
\brief General ray tracer class.

\author Tomáš Fabián
\version 0.1
\date 2018
*/
class Raytracer : public SimpleGuiDX11
{
public:
	Raytracer(const int width, const int height,
		const float fov_y, const Vector3 view_from, const Vector3 view_at, const Vector3 light_org,
		const char* config = "threads=0,verbose=3");
	~Raytracer();

	int InitDeviceAndScene(const char* config);

	int ReleaseDeviceAndScene();

	void LoadScene(const std::string file_name, const char* cube_map_file_names[6]);

	bool IsHitPointVisible(const Vector3 hitPoint, const Vector3 lightPoint);

	Color4f get_pixel(const int x, const int y, const float t = 0.0f) override;

	Vector3 TraceRay(RTCRay ray, const float n_1 = 1.0f, const int depth = 0, const int max_Depth = 10);

	Vector3 lambertShader(const Material* material, const Coord2f tex_coord, const Vector3 hit_point, Vector3 normal_vector);

	Vector3 phongShader(const Material* material, const Coord2f tex_coord, const Vector3 hit_point, Vector3 normal_vector, const Vector3 direction_vector, const int depth);

	Vector3 transparentShader(RTCRay ray, const Vector3 hit_point, const Vector3 normal_vector, const Vector3 direction_vector, const Material* material, float n_1, const int depth);

	int Ui();

	bool sampling = false;
	bool normal_shader = false;
	bool lambert_shader = false;
	bool phong_shader = true;

private:
	std::vector<Surface*> surfaces_;
	std::vector<Material*> materials_;
	CubeMap* cubemap_;

	RTCDevice device_;
	RTCScene scene_;
	Camera camera_;
	Light light_;
};
