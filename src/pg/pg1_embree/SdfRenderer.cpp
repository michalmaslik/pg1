#include "stdafx.h"
#include "SdfRenderer.h"

//=============================================================================
// DISPATCHER
//=============================================================================

Vector4 SdfRenderer::render(const RTCRay& ray,
                             const SdfRenderContext& ctx,
                             const bool rayMarching) const
{
    if (rayMarching) {
        // Volumetricke krochlovani — prochazi cely objem, akumuluje barvu a opacitu
        return volumetricEffect(ray, ctx);
    }
    // Sphere-tracing — najde prvni prusecik s povrchu SDF
    return surfaceEffect(ray, ctx);
}

//=============================================================================
// VOLUMETRICKE KROCHLOVANI PRES SDF OBLAK
//=============================================================================

Vector4 SdfRenderer::volumetricEffect(const RTCRay& ray,
                                       const SdfRenderContext& ctx,
                                       const float tMax) const
{
    // Pokud nejsou zadna telesa, vrat pruhlednost
    if (ctx.shapes.empty())
        return Vector4(0.0f, 0.0f, 0.0f, 0.0f);

    const Vector3 rayOrigin(ray.org_x, ray.org_y, ray.org_z);
    const Vector3 direction(ray.dir_x, ray.dir_y, ray.dir_z);

    // Akumulatory pro front-to-back Porter-Duff kompozici
    float   remainingT  = 1.0f; // zbyvajici propustnost (1 = nic nepohlteno)
    Vector3 inScattered(0.0f);  // akumulovana in-scattered radiance
    float   t   = 0.0f;
    float   sdf = 0.0f;

    for (int i = 0; i < ctx.maxSteps; ++i) {
        // Adaptivni krok: mimo oblak pouzij SDF jako bezpecny krok (sphere-tracing)
        t += std::max(ctx.stepSize, sdf);
        if (t > tMax || t > ctx.maxDistance) break;
        if (remainingT < 0.01f)              break;  // predcasne ukonceni (earyl exit)

        // Vzorkovaci bod ve svetovem prostoru, korigovan o animacni posuv
        const Vector3 pos = rayOrigin + direction * t;
        sdf = ctx.shapes[0]->SDF(pos - ctx.animOffset);

        // Jsme uvnitr oblaku (sdf < 0)?
        if (sdf < 0.0f) {
            // Hustota = |sdf| orizene na [0,1] (hloubeji = hustejsi)
            const float density     = std::min(-sdf, 1.0f);
            const float prevT       = remainingT;
            remainingT             *= expf(-ctx.absorptionCoeff * density * ctx.stepSize);
            const float absorptionW = prevT - remainingT; // vahovana pohltena energie

            // --- Prispevky od vsech svetel (bodovych i smeroveych) ---
            for (const Light& light : ctx.lights) {
                Vector3 lightDir;
                float   atten;

                if (light.type == LightType::Point) {
                    const Vector3 toLight = light.position - pos;
                    const float   dist    = toLight.L2Norm();
                    lightDir = toLight / dist;
                    atten    = getLightAttenuation(dist, ctx.lightAttenuationFactor);
                } else {
                    // Smerove svetlo — zadny pokles, normalizovany opacny smer
                    lightDir = -light.direction;
                    lightDir.Normalize();
                    atten = 1.0f;
                }

                // Luma test: preskoc svetla, ktera jsou prakticky cerna
                const Vector3 lc   = light.color * (light.intensity * atten);
                const float   luma = lc.x * 0.3f + lc.y * 0.59f + lc.z * 0.11f;
                if (luma < 0.009f) continue;

                // Volumetricky stin: pouze pro bodova svetla (smerova = 1.0)
                const float shadowT = (light.type == LightType::Point)
                    ? computeShadowTransmittance(pos, light.position, ctx)
                    : 1.0f;

                inScattered += absorptionW * shadowT * ctx.volumetricAlbedo * lc;
            }

            // --- Globalni slunce (smerove, s volumetrickym stinem) ---
            if (ctx.sunEnabled) {
                // Virtualni bod daleko ve smeru slunce pro computeShadowTransmittance
                const Vector3 sunPt  = pos + ctx.sunDir * ctx.maxDistance;
                const float   sunT   = computeShadowTransmittance(pos, sunPt, ctx);
                const Vector3 sunLc  = ctx.sunColor * ctx.sunIntensity;
                inScattered += absorptionW * sunT * ctx.volumetricAlbedo * sunLc;
            }

            // --- Ambientni osvetleni ---
            inScattered += absorptionW * ctx.volumetricAlbedo * getAmbientLight();
        }
    }

    // Vraci: RGB = nastrimovana barva, A = opacita (1 - zbyvajici propustnost)
    return Vector4(inScattered, 1.0f - remainingT);
}

//=============================================================================
// SPHERE-TRACING (povrchove renderovani SDF teles)
//=============================================================================

Vector4 SdfRenderer::surfaceEffect(const RTCRay& ray,
                                    const SdfRenderContext& ctx) const
{
    if (ctx.shapes.empty())
        return Vector4(0.0f);

    Vector3 position(ray.org_x, ray.org_y, ray.org_z);
    Vector3 direction(ray.dir_x, ray.dir_y, ray.dir_z);
    direction.Normalize();

    Shape* const shape = ctx.shapes[0];

    // Sphere-tracing smycka: krok = SDF hodnota (bezpecny, nepresahne povrch)
    for (int i = 0; i < 1000; ++i) {
        if (position.L2Norm() > ctx.maxDistance) break;

        const float sdfValue = shape->SDF(position);

        // Zasah: SDF velmi blizko nule = jsme na povrchu
        if (sdfValue < 0.001f) {
            // Normala = normalizovany gradient SDF
            Vector3 normal = computeSdfNormal(position, *shape);
            normal.Normalize();

            // Smer ke svetlu
            Vector3 lightDir = ctx.primaryLight.position - position;
            lightDir.Normalize();

            // Phongov osvetlovaci model
            const Vector3 ambientColor(0.025f, 0.025f, 0.025f);
            Vector3 color = ambientColor;

            // Lambertova difuze s testom viditelnosti (pres Embree shadow ray)
            const float diffuse = std::max(0.0f, normal.DotProduct(lightDir));
            const bool  visible = ctx.isVisible
                ? ctx.isVisible(position, ctx.primaryLight.position)
                : true;
            if (visible) {
                color += Vector3(diffuse);
            }

            // Phongova odrazivost
            const Vector3 viewDir    = -direction;
            const Vector3 reflectDir = 2.0f * normal.DotProduct(lightDir) * normal - lightDir;
            const float   spec       = powf(std::max(viewDir.DotProduct(reflectDir), 0.0f), 32.0f);
            color += Vector3(spec);

            return Vector4(color, 1.0f);
        }

        // Krok sphere-tracingu: bezpecna vzdalenost = SDF hodnota
        position += direction * sdfValue;
    }

    // Paprsek minul vsechna telesa — pruhledny vysledek
    return Vector4(0.0f);
}

//=============================================================================
// VOLUMETRICKY STIN (Beer-Lambert stinovy paprsek skrze SDF oblak)
//=============================================================================

float SdfRenderer::computeShadowTransmittance(const Vector3& samplePoint,
                                               const Vector3& lightPos,
                                               const SdfRenderContext& ctx) const
{
    if (ctx.shapes.empty()) return 1.0f;

    const Vector3 toLight    = lightPos - samplePoint;
    const float   lightDist  = toLight.L2Norm();
    const Vector3 shadowDir  = toLight / lightDist;

    // lightMarchSize = 0.65 * MARCH_MULTIPLIER v referencnim ShaderToy (pomer 0.65/0.6)
    const float shadowStep = ctx.stepSize * (0.65f / 0.6f);

    float t          = 0.0f;
    float visibility = 1.0f;
    float sdf        = 0.0f;

    // 25 kroku odpovidaji referenci GetLightVisibility() v ShaderToy
    for (int i = 0; i < 25; ++i) {
        t += std::max(shadowStep, sdf);
        if (t >= lightDist || visibility < 0.01f) break;

        sdf = ctx.shapes[0]->SDF(samplePoint + shadowDir * t - ctx.animOffset);
        if (sdf < 0.0f) {
            const float density = std::min(-sdf, 1.0f);
            visibility *= expf(-ctx.absorptionCoeff * density * shadowStep);
        }
    }
    return visibility;
}
