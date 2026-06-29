#include "stdafx.h"
#include "raytracer.h"
#include "ShadingUtils.h"

==
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
		// Lambert's Law: I_diffuse = I_light * albedo * max(0, NÂ·L)
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

		// Phong specular: I_spec = I_light * k_s * max(0, RÂ·V)^shininess
		outputColor += specularColor * powf(clamp(l_r.DotProduct(-directionVector)), material.GetShininess());
		outputColor += l_i * specularColor * 0.2f; // Indirect specular contribution
	}

	// --- Global Sun contribution (directional, no 1/dÂ˛ attenuation) -----------
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
	// Beer-Lambert Law: T = e^(-Ď * d) for each color channel
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
		// AlternatĂ­va (staÄŤĂ­ na vizuĂˇlne efekty):
		// float R0 = powf((n_1 - n_2) / (n_1 + n_2), 2.0f);
		// r = R0 + (1.0f - R0) * powf(1.0f - cos_theta1, 5.0f);
	}

	// Trace rays
	Vector3 refl = TraceRay(makeSecondaryRay(hitPoint, reflectedRayDirection), n_1, depth + 1);
	Vector3 refr = Vector3(0.0f);
	if (!totalInternalReflection) {
		refr = TraceRay(makeSecondaryRay(hitPoint, refractedRayDirection), n_2, depth + 1);
		// Fallback na cubemap ak refr je Ăşplne ÄŤierna a backgroundEnabled_
		if (refr == Vector3(0.0f) && backgroundEnabled_ && cubemap_ != nullptr) {
			Color3f bg = cubemap_->GetTexel(refractedRayDirection);
			refr = Vector3(bg.r, bg.g, bg.b);
		}
	}

	// Beer-Lambert attenuation len ak sme vo vnĂştri materiĂˇlu
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
