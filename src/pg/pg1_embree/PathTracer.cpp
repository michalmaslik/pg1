#include "stdafx.h"
#include "PathTracer.h"
#include "texture.h"

//=============================================================================
// STATICKE POMOCNE METODY
//=============================================================================

void PathTracer::buildONB(const Vector3& n, Vector3& t, Vector3& b)
{
    // Metoda Duff et al. 2017 — numericky stabilni na rozdil od klasickeho
    // Gram-Schmidta (eliminuje singularity blizko n = (0,0,+-1))
    const float sign = (n.z >= 0.0f) ? 1.0f : -1.0f;
    const float a    = -1.0f / (sign + n.z);
    const float c    = n.x * n.y * a;
    t = Vector3(1.0f + sign * n.x * n.x * a,  sign * c,              -sign * n.x);
    b = Vector3(c,                              sign + n.y * n.y * a,  -n.y);
}

Vector3 PathTracer::sampleHemisphereCosine(const Vector3& normal)
{
    // Mallleyova metoda: vzorkuj jednotkovy disk, promitni na polokouli.
    // PDF = cos(theta)/pi; pro Lambertovske BRDF se throughput vaha rusi.
    const float r1  = Random();
    const float r2  = Random();
    const float phi = 2.0f * static_cast<float>(M_PI) * r1;
    const float r   = std::sqrt(r2);

    // Lokalni souradnice vzorku (z = cos(theta) = sqrt(1-r2))
    const float lx = r * std::cos(phi);
    const float ly = r * std::sin(phi);
    const float lz = std::sqrt(std::max(0.0f, 1.0f - r2));

    // Otoc vzork do svetoveho prostoru pres ONB sestavenou z normaly
    Vector3 t, b;
    buildONB(normal, t, b);
    return t * lx + b * ly + normal * lz;
}

float PathTracer::balancedHeuristic(const float p_a, const float p_b)
{
    // Vyvazena MIS heuristika: w_a = p_a / (p_a + p_b)
    const float denom = p_a + p_b;
    return (denom > 0.0f) ? p_a / denom : 0.0f;
}

float PathTracer::evaluateHenyeyGreenstein(const float cosTheta, const float g)
{
    // Normalizovana Henyey-Greenstein fazova funkce pro anizoprni rozptyl
    constexpr float kInv4Pi = 1.0f / (4.0f * static_cast<float>(M_PI));
    const float g2  = g * g;
    const float den = 1.0f + g2 - 2.0f * g * cosTheta;
    return kInv4Pi * (1.0f - g2) / (den * sqrtf(den));
}

//=============================================================================
// PRIMY SVETELNY PRISPEVEK (Next Event Estimation)
//=============================================================================

Vector3 PathTracer::sampleDirectLight(const Vector3& hitPoint,
                                       const Vector3& normal,
                                       const Vector3& albedo,
                                       const PathTracingContext& ctx) const
{
    constexpr float kInvPi = 1.0f / static_cast<float>(M_PI);
    Vector3 directLight(0.0f);

    // Iteruj pres vsechna svetla ve scene
    for (const Light& light : ctx.lights) {
        Vector3 lightDir;
        float   atten;
        float   shadowDist;

        if (light.type == LightType::Point) {
            const Vector3 toLight = light.position - hitPoint;
            const float   dist    = toLight.L2Norm();
            if (dist < 1e-4f) continue;
            lightDir   = toLight / dist;
            atten      = getLightAttenuation(dist, ctx.lightAttenuationFactor);
            shadowDist = dist;
        } else {
            // Smerove svetlo — zadny pokles, normalizovany opacny smer
            lightDir   = -light.direction;
            lightDir.Normalize();
            atten      = 1.0f;
            shadowDist = ctx.maxDistance;
        }

        // Kosinus dopadu (Lambertova BRDF)
        const float nDotL = normal.DotProduct(lightDir);
        if (nDotL <= 0.0f) continue;

        // Shadow ray: pro smerove svetlo vytvor virtualni bod daleko
        const Vector3 lightPt = (light.type == LightType::Point)
            ? light.position
            : hitPoint + lightDir * shadowDist;

        if (ctx.isVisible && !ctx.isVisible(hitPoint, lightPt)) continue;

        // Li = barva * intenzita * utlum
        const Vector3 Li = light.color * (light.intensity * atten);

        // Lambert BRDF: (albedo/pi) * Li * nDotL
        directLight += Li * albedo * kInvPi * nDotL;
    }

    // --- Globalni slunce (smerove delta svetlo, PDF = 1) ---
    if (ctx.sunEnabled) {
        const float nDotSun = normal.DotProduct(ctx.sunDir);
        if (nDotSun > 0.0f) {
            const Vector3 sunPt = hitPoint + ctx.sunDir * ctx.maxDistance;
            if (!ctx.isVisible || ctx.isVisible(hitPoint, sunPt)) {
                const Vector3 Li = ctx.sunColor * ctx.sunIntensity;
                directLight += Li * albedo * kInvPi * nDotSun;
            }
        }
    }

    return directLight;
}

//=============================================================================
// SLEDOVANI CEST (Monte Carlo Path Tracing)
//=============================================================================

Vector3 PathTracer::tracePath(const RTCRay& initialRay,
                               const int maxDepth,
                               const PathTracingContext& ctx) const
{
    // Bez telesa vrat okamzite pozadi
    if (!ctx.hasSurfaces) {
        const Vector3 dir(initialRay.dir_x, initialRay.dir_y, initialRay.dir_z);
        if (ctx.backgroundEnabled && ctx.cubemap) {
            const Color3f bg = ctx.cubemap->GetTexel(dir);
            return Vector3(bg.r, bg.g, bg.b);
        }
        return ctx.backgroundColor;
    }

    Vector3 radiance(0.0f);    // akumulovana radiance cesty
    Vector3 throughput(1.0f);  // aktualni vahova propustnost cesty
    RTCRay  ray = initialRay;

    for (int depth = 0; depth < maxDepth; ++depth) {

        // --- 1. Prusecik s BVH pres Embree ---
        RTCHit hit{};
        hit.geomID = RTC_INVALID_GEOMETRY_ID;
        hit.primID = RTC_INVALID_GEOMETRY_ID;

        RTCRayHit rayHit{};
        rayHit.ray = ray;
        rayHit.hit = hit;

        RTCIntersectContext embreeCtx;
        rtcInitIntersectContext(&embreeCtx);
        rtcIntersect1(ctx.scene, &embreeCtx, &rayHit);

        // --- 2. Prosel paprsek — pricti barvu pozadi a ukoncuj cestu ---
        if (rayHit.hit.geomID == RTC_INVALID_GEOMETRY_ID) {
            const Vector3 dir(ray.dir_x, ray.dir_y, ray.dir_z);
            if (ctx.backgroundEnabled && ctx.cubemap) {
                const Color3f bg = ctx.cubemap->GetTexel(dir);
                radiance += throughput * Vector3(bg.r, bg.g, bg.b);
            } else {
                radiance += throughput * ctx.backgroundColor;
            }
            break;
        }

        // --- 3. Zrekonstruuj geometrii v bode zasahu ---
        const float   tHit = rayHit.ray.tfar;
        const Vector3 dir(ray.dir_x, ray.dir_y, ray.dir_z);
        const Vector3 hitPt{
            ray.org_x + dir.x * tHit,
            ray.org_y + dir.y * tHit,
            ray.org_z + dir.z * tHit
        };

        RTCGeometry     geom = rtcGetGeometry(ctx.scene, rayHit.hit.geomID);
        const Material* mat  = static_cast<const Material*>(rtcGetGeometryUserData(geom));
        if (!mat) break;

        // Interpoluj stinovu normalu
        Normal3f nrm{};
        rtcInterpolate0(geom, rayHit.hit.primID, rayHit.hit.u, rayHit.hit.v,
            RTC_BUFFER_TYPE_VERTEX_ATTRIBUTE, 0, &nrm.x, 3);
        Vector3 n(nrm.x, nrm.y, nrm.z);
        if (n.DotProduct(dir) > 0.0f) n *= -1.0f; // front-facing normala
        n.Normalize();

        // Interpoluj UV a ziskej albedo (material nebo textura)
        Coord2f uv{};
        rtcInterpolate0(geom, rayHit.hit.primID, rayHit.hit.u, rayHit.hit.v,
            RTC_BUFFER_TYPE_VERTEX_ATTRIBUTE, 1, &uv.u, 2);

        Vector3 albedo = mat->GetDiffuse();
        Texture* diffTex = mat->get_texture(Material::kDiffuseMapSlot);
        if (diffTex) {
            const Color3f c = diffTex->get_texel(uv.u, 1.0f - uv.v);
            albedo = Vector3(c.r, c.g, c.b);
        }

        // --- 4. NEE — prispevek primiho svetla ---
        radiance += throughput * sampleDirectLight(hitPt, n, albedo, ctx);

        // --- 5. Ruska ruleta — nezkresle ukonceni cesty ---
        // Pravdepodobnost preziti = max(r, g, b) alba (temne povrchy konci drive)
        if (depth >= ctx.rrMinDepth) {
            const float p_rr = std::min(
                std::max({ albedo.x, albedo.y, albedo.z }), 0.95f);
            if (Random() > p_rr) break;
            throughput /= p_rr; // kompenzace: E[output] = p_rr * (L/p_rr) = L
        }

        // --- 6. Vzorkuj smer dalsiho odrazu dle materialu ---
        const int   shader       = mat->GetShader();
        const float reflectivity = mat->GetReflectivity();
        Vector3     newDir(0.0f);

        if (shader == 4) {
            // Dielektrikum (sklo) — Fresnelovo vzorkovani odrazu / lomu
            const float ior    = (mat->GetIor() > 0.0f) ? mat->GetIor() : 1.5f;
            const float cosT1  = clamp(-dir.DotProduct(n), -1.0f, 1.0f);
            const float nRatio = 1.0f / ior; // vstup ze vzduchu (n1=1)
            const float sin2T2 = nRatio * nRatio * (1.0f - cosT1 * cosT1);

            const Vector3 reflected = dir - 2.0f * dir.DotProduct(n) * n;

            if (sin2T2 >= 1.0f) {
                // Uplny vnitrni odraz
                newDir = reflected;
                newDir.Normalize();
            } else {
                const float cosT2 = std::sqrt(std::max(0.0f, 1.0f - sin2T2));
                const float rs    = powf((ior * cosT2 - cosT1) / (ior * cosT2 + cosT1), 2.0f);
                const float rp    = powf((ior * cosT1 - cosT2) / (ior * cosT1 + cosT2), 2.0f);
                const float Fr    = (rs + rp) * 0.5f;

                if (Random() < Fr) {
                    newDir = reflected;
                    newDir.Normalize();
                } else {
                    // Fresnelova refrakce — nezmeneny throughput (energeticky konzervativni)
                    newDir = nRatio * dir + (nRatio * cosT1 - cosT2) * n;
                    newDir.Normalize();
                }
            }
            // throughput se nemeni (dielektrikum zachovava energii)

        } else if (shader == 3 && Random() < reflectivity) {
            // Phongova odrazivost — stochasticka volba dle reflectivity
            newDir = dir - 2.0f * dir.DotProduct(n) * n;
            newDir.Normalize();
            throughput = throughput * mat->GetSpecular();

        } else {
            // Lambertovska difuze — cosinove vzorkovani polokoule
            newDir = sampleHemisphereCosine(n);
            throughput = throughput * albedo;
        }

        // Dalsi krok cesty: makeSecondaryRay aplikuje tnear bias (zamezi self-intersection)
        ray = makeSecondaryRay(hitPt, newDir);
    }

    return radiance;
}
