#include "stdafx.h"
#include "VdbRenderer.h"
#include <iostream>

//=============================================================================
// DESTRUKTOR
//=============================================================================

VdbRenderer::~VdbRenderer()
{
    cleanup();
}

//=============================================================================
// INICIALIZACE A UVOLNENI
//=============================================================================

bool VdbRenderer::initialize()
{
    std::cout << "[VKL] Inicializace OpenVKL..." << std::endl;

    // Inicializuj OpenVKL runtime
    vklInit();

    // Vytvor CPU zarizeni
    vklDevice_ = vklNewDevice("cpu");
    if (!vklDevice_) {
        std::cerr << "[VKL CHYBA] Nepodarilo se vytvorit OpenVKL CPU zarizeni!" << std::endl;
        return false;
    }

    // Nastav parametry logovania a vystupu chyb
    vklDeviceSetInt(vklDevice_,    "logLevel",   VKL_LOG_WARNING);
    vklDeviceSetString(vklDevice_, "logOutput",  "cout");
    vklDeviceSetString(vklDevice_, "errorOutput","cerr");

    // Potvrď konfiguraci zarizeni
    vklCommitDevice(vklDevice_);

    // Vytvor loader VDB souboru
    vdbLoader_ = std::make_unique<VdbLoader>();

    std::cout << "[VKL] OpenVKL uspesne inicializovano." << std::endl;
    return true;
}

void VdbRenderer::cleanup()
{
    // Uvolni sampler pred svazkem (zavislost)
    if (vklSampler_) {
        vklRelease(vklSampler_);
        vklSampler_ = VKLSampler{};
    }
    // Uvolni svazek pred zarizenim (zavislost)
    if (vklVolume_) {
        vklRelease(vklVolume_);
        vklVolume_ = VKLVolume{};
    }
    // Uvolni zarizeni jako posledni
    if (vklDevice_) {
        vklReleaseDevice(vklDevice_);
        vklDevice_ = VKLDevice{};
    }

    // Zrus loader
    vdbLoader_.reset();
    std::cout << "[VKL] OpenVKL uklid dokoncen." << std::endl;
}

//=============================================================================
// NACITANI VDB SVAZKU
//=============================================================================

void VdbRenderer::clearVolume()
{
    // Uvolni pouze sampler a svazek � zarizeni zachovej pro dalsi nacitani.
    // Tato metoda je pouzivana z clearScene() pro hot-swap sceny bez nutnosti
    // znovu inicializovat OpenVKL zarizeni (coz je drahe).
    if (vklSampler_) {
        vklRelease(vklSampler_);
        vklSampler_ = VKLSampler{};
    }
    if (vklVolume_) {
        vklRelease(vklVolume_);
        vklVolume_ = VKLVolume{};
    }
    // Resetuj loader, aby GetBounds() nevracelo data z predchozi sceny
    if (vdbLoader_) {
        vdbLoader_ = std::make_unique<VdbLoader>();
    }
}

bool VdbRenderer::loadVolume(const std::string& filename, const std::string& gridName)
{
    if (!vklDevice_) {
        std::cerr << "[VKL CHYBA] OpenVKL neni inicializovano! Zavolej initialize() nejdrive." << std::endl;
        return false;
    }

    // Uvolni predchozi svazek pred nahrazenim novym
    if (vklSampler_) {
        vklRelease(vklSampler_);
        vklSampler_ = VKLSampler{};
    }
    if (vklVolume_) {
        vklRelease(vklVolume_);
        vklVolume_ = VKLVolume{};
    }

    // Nacti VDB soubor pres VdbLoader
    vklVolume_ = vdbLoader_->LoadVdbFile(filename, gridName, vklDevice_);
    if (!vklVolume_) {
        std::cerr << "[VKL CHYBA] Nepodarilo se nacist VDB svazek ze souboru: " << filename << std::endl;
        return false;
    }

    // Vytvor vzorkovac pro pristup k datam svazku
    vklSampler_ = vklNewSampler(vklVolume_);
    if (!vklSampler_) {
        std::cerr << "[VKL CHYBA] Nepodarilo se vytvorit VKL vzorkovac!" << std::endl;
        vklRelease(vklVolume_);
        vklVolume_ = VKLVolume{};
        return false;
    }

    // Nastav linearni interpolaci pro vzorkovani a gradienty
    vklSetInt(vklSampler_, "filter",         VKL_FILTER_LINEAR);
    vklSetInt(vklSampler_, "gradientFilter", VKL_FILTER_LINEAR);
    vklCommit(vklSampler_);

    // Vypis informace o nactenem svazku
    const vkl_box3f   volumeBounds = vklGetBoundingBox(vklVolume_);
    const vkl_range1f valueRange   = vklGetValueRange(vklVolume_, 0);

    std::cout << "[VKL] Hranice svazku: min("
              << volumeBounds.lower.x << "," << volumeBounds.lower.y << "," << volumeBounds.lower.z << ")"
              << " max("
              << volumeBounds.upper.x << "," << volumeBounds.upper.y << "," << volumeBounds.upper.z << ")"
              << std::endl;
    std::cout << "[VKL] Rozsah hodnot: [" << valueRange.lower << ", " << valueRange.upper << "]" << std::endl;

    return true;
}

//=============================================================================
// DOTAZY
//=============================================================================

void VdbRenderer::getBounds(Vector3& minBounds, Vector3& maxBounds) const
{
    if (vdbLoader_) {
        minBounds = vdbLoader_->GetBoundingBoxMin();
        maxBounds = vdbLoader_->GetBoundingBoxMax();
    } else {
        minBounds = Vector3(0.0f, 0.0f, 0.0f);
        maxBounds = Vector3(1.0f, 1.0f, 1.0f);
    }
}

//=============================================================================
// VOLUMETRICKE KROCHLOVANI PRES VDB
//=============================================================================

Vector4 VdbRenderer::rayMarch(const RTCRay& ray,
                               const VdbRenderContext& ctx,
                               const bool compositeBg) const
{
    // Pokud neni nacteny zadny svazek, vrat pozadi
    if (!vklSampler_) {
        if (ctx.backgroundEnabled && ctx.cubemap) {
            const Vector3 direction(ray.dir_x, ray.dir_y, ray.dir_z);
            const Color3f bgColor = ctx.cubemap->GetTexel(direction);
            return Vector4(bgColor.r, bgColor.g, bgColor.b, 1.0f);
        }
        return Vector4(ctx.backgroundColor.x, ctx.backgroundColor.y, ctx.backgroundColor.z, 1.0f);
    }

    const Vector3 rayOrigin(ray.org_x, ray.org_y, ray.org_z);
    Vector3       rayDirection(ray.dir_x, ray.dir_y, ray.dir_z);
    rayDirection.Normalize();

    // Ziskej hranice svazku z OpenVKL a aplikuj animacni posuv
    const vkl_box3f volumeBounds = vklGetBoundingBox(vklVolume_);
    const Vector3   minBounds(volumeBounds.lower.x + ctx.animOffset.x,
                               volumeBounds.lower.y + ctx.animOffset.y,
                               volumeBounds.lower.z + ctx.animOffset.z);
    const Vector3   maxBounds(volumeBounds.upper.x + ctx.animOffset.x,
                               volumeBounds.upper.y + ctx.animOffset.y,
                               volumeBounds.upper.z + ctx.animOffset.z);

    // Vypocti prusecik paprsku s obalovym kvadrem (slab method)
    const Vector3 invDir(
        rayDirection.x != 0.0f ? 1.0f / rayDirection.x : FLT_MAX,
        rayDirection.y != 0.0f ? 1.0f / rayDirection.y : FLT_MAX,
        rayDirection.z != 0.0f ? 1.0f / rayDirection.z : FLT_MAX
    );

    const Vector3 t1 = (minBounds - rayOrigin) * invDir;
    const Vector3 t2 = (maxBounds - rayOrigin) * invDir;

    const Vector3 tMinVec(std::min(t1.x, t2.x), std::min(t1.y, t2.y), std::min(t1.z, t2.z));
    const Vector3 tMaxVec(std::max(t1.x, t2.x), std::max(t1.y, t2.y), std::max(t1.z, t2.z));

    const float tNear = std::max({ tMinVec.x, tMinVec.y, tMinVec.z, 0.001f });
    const float tFar  = std::min({ tMaxVec.x, tMaxVec.y, tMaxVec.z });

    // Paprsek minul objem — vrat pozadi
    if (tNear >= tFar) {
        if (ctx.backgroundEnabled && ctx.cubemap) {
            const Vector3 direction(ray.dir_x, ray.dir_y, ray.dir_z);
            const Color3f bgColor = ctx.cubemap->GetTexel(direction);
            return Vector4(bgColor.r, bgColor.g, bgColor.b, 1.0f);
        }
        return Vector4(ctx.backgroundColor.x, ctx.backgroundColor.y, ctx.backgroundColor.z, 1.0f);
    }

    // Parametry krochlovani
    constexpr unsigned int attributeIndex = 0;
    constexpr float        vklTime        = 0.0f;

    // Akumulatory pro Porter-Duff kompozici (front-to-back)
    float   accumulatedOpacity = 1.0f;  // zbyvajici propustnost (1 = nic nepohlteno)
    Vector3 accumulatedColor(0.0f);

    // Rozsah hodnot hustoty pro normalizaci
    const vkl_range1f valueRange = vklGetValueRange(vklVolume_, attributeIndex);
    float maxDensity = valueRange.upper;
    if (maxDensity <= 0.0f) maxDensity = 1.0f;

    // Hlavni smycka krochlovani
    const int maxSteps = static_cast<int>((tFar - tNear) / ctx.stepSize) + 1;
    int       steps    = 0;

    for (float t = tNear;
         t < tFar && accumulatedOpacity > 0.01f && steps < maxSteps;
         t += ctx.stepSize, ++steps)
    {
        // Aktualni vzorkovaci bod ve svetovem prostoru
        const Vector3 currentPos = rayOrigin + rayDirection * t;

        // Prevod do lokalniho prostoru svazku (odecteni animacniho posuvu)
        const vkl_vec3f vklPos = {
            currentPos.x - ctx.animOffset.x,
            currentPos.y - ctx.animOffset.y,
            currentPos.z - ctx.animOffset.z
        };

        // Vzorkuj hustotu VDB svazku pres OpenVKL
        const float density = vklComputeSample(&vklSampler_, &vklPos, attributeIndex, vklTime);

        if (density > 0.0f) {
            // Normalizuj hustotu pro vizualizaci
            const float normalizedDensity = std::min(density / maxDensity, 1.0f);

            // Beer-Lambertova extinkce: T = exp(-sigma_t * rho * dt)
            const float absorption       = normalizedDensity * ctx.absorptionCoeff * ctx.stepSize;
            const float transmittance    = expf(-absorption);
            const float previousOpacity  = accumulatedOpacity;
            accumulatedOpacity          *= transmittance;
            const float absorbedWeight   = previousOpacity - accumulatedOpacity;

            // Jednoduche bodove osvetleni (bez stinoveho paprsku pro VDB)
            Vector3 lightDir    = ctx.primaryLight.position - currentPos;
            const float lightDistance = lightDir.L2Norm();
            lightDir.Normalize();

            Vector3 lightAtten = Vector3(1.0f) * getLightAttenuation(lightDistance, ctx.lightAttenuationFactor);
            Vector3 volumeColor = ctx.volumetricAlbedo * lightAtten;
            volumeColor        += getAmbientLight() * ctx.volumetricAlbedo;

            // Globalni slunce — rownobezne paprsky, zadne 1/d^2 utlumeni
            if (ctx.sunEnabled) {
                volumeColor += ctx.volumetricAlbedo * ctx.sunColor * ctx.sunIntensity;
            }

            // Akumuluj barvu vazenou pohlenutou hustotou
            accumulatedColor += absorbedWeight * volumeColor * normalizedDensity;
        }
    }

    const float finalOpacity = 1.0f - accumulatedOpacity;

    // Volitelna kompozice s pozadim (Porter-Duff over)
    Vector3 resultColor = accumulatedColor;
    float   resultAlpha = finalOpacity;

    if (compositeBg && finalOpacity < 1.0f) {
        Vector3 backgroundColor;
        if (ctx.backgroundEnabled && ctx.cubemap) {
            const Vector3 direction(ray.dir_x, ray.dir_y, ray.dir_z);
            const Color3f bgColor = ctx.cubemap->GetTexel(direction);
            backgroundColor = Vector3(bgColor.r, bgColor.g, bgColor.b);
        } else {
            backgroundColor = ctx.backgroundColor;
        }
        resultColor = accumulatedColor * finalOpacity + backgroundColor * (1.0f - finalOpacity);
        resultAlpha = 1.0f;
    }

    // Orizni barvy do rozsahu [0, 1]
    resultColor.x = std::min(resultColor.x, 1.0f);
    resultColor.y = std::min(resultColor.y, 1.0f);
    resultColor.z = std::min(resultColor.z, 1.0f);

    return Vector4(resultColor.x, resultColor.y, resultColor.z, resultAlpha);
}
