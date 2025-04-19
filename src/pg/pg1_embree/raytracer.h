#ifndef RAYTRACER_H_
#define RAYTRACER_H_

#include "simpleguidx11.h"
#include "surface.h"
#include "camera.h"
#include "light.h"
#include "cubemap.h"
#include "mymath.h"
#include "material.h"
#include "shape.h"
#include "sphere.h"
#include "box.h"
#include "smooth_union.h"

/*! \class Raytracer
\brief General ray tracer class.

\author Tom Fabin
\version 0.1
\date 2018
*/
struct Transform {
	Vector3 position = Vector3(0.0f, 0.0f, 0.0f);
	Vector3 scale = Vector3(1.0f, 1.0f, 1.0f);
	Vector3 rotation = Vector3(0.0f, 0.0f, 0.0f); // Euler angles in degrees
};

struct ModelInfo {
	std::string filePath;
	Transform transform;
};




class RayTracer : public SimpleGuiDX11
{
public:
	RayTracer(const int width, const int height,
		const float fovY, const Vector3& viewFrom, const Vector3& viewAt, const Vector3& lightOrigin,
		const char* config = "threads=0,verbose=3");
	~RayTracer();

	int InitDeviceAndScene(const char* config);

	int ReleaseDeviceAndScene();

	void LoadScene(
		const std::vector<ModelInfo>& models,
		const std::vector<Shape*>& shapes,
		const char* cubeMapFileNames[6]
	);

	void LoadModel(const std::string& fileName, const Transform& transform = Transform());

	bool IsHitPointVisible(const Vector3& hitPoint, const Vector3& lightPoint);

	Color4f GetPixel(const int x, const int y, const float t = 0.0f) override;

	void MoveCamera() override;

	Vector3 TraceRay(const RTCRay& ray, const float n_1 = 1.0f, const int depth = 0, const int maxDepth = 10);

	Vector3 NormalShader(const Vector3& normalVector);

	Vector3 LambertShader(const Material& material, const Coord2f& texCoord, const Vector3& hitPoint, const Vector3& normalVector);

	Vector3 PhongShader(const Material& material, const Coord2f& texCoord, const Vector3& hitPoint, const Vector3& normalVector, const Vector3& directionVector, const int depth);

	Vector3 TransparentShader(const RTCRay& ray, const Vector3& hitPoint, const Vector3& normalVector, const Vector3& directionVector, const Material& material, const float n_1, const int depth);

	Vector3 ComputeNormal(const Vector3& p, const Shape& shape);

	Vector3 TraceSDFRay(const RTCRay& ray, const int depth, const int max_depth);

	int Ui() override;

	// modes
	bool normalShader_ = false;
	bool lambertShader_ = false;
	bool phongShader_ = false;
	bool sdfMode_ = true;

	bool sampling_ = false;

	bool cameraMovementEnabled_ = true;
	float cameraX_{ 0.0f };
	float cameraY_{ 0.0f };
	float cameraZ_{ 0.0f };
	

private:
	std::vector<Shape*> volumetricShapes_;
	std::vector<Surface*> surfaces_;
	std::vector<Material*> materials_;
	CubeMap* cubemap_ = nullptr;

	RTCDevice device_;
	RTCScene scene_;
	Camera camera_;
	Light light_;
};
#endif

