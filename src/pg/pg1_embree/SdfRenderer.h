#pragma once

#include "stdafx.h"
#include "ShadingUtils.h"
#include "RenderTypes.h"
#include "vector3.h"
#include "vector4.h"
#include "light.h"
#include "shape.h"
#include <embree3/rtcore.h>
#include <vector>
#include <functional>

/**
 * @file  SdfRenderer.h
 * @brief Trida pro volumetricke krochlovani a sphere-tracing pres SDF telesa.
 *
 * SdfRenderer je bezstavova vypocetni trida — nevlastni zadna data,
 * vsechny potrebne informace dostava pres SdfRenderContext pri kazdem volani.
 * RayTracer ji volá z metod volumetricEffect, surfaceEffect a volumetricRender.
 *
 * Zavislost na Embree (pro viditelnost ve sphere-tracingu) je abstrahovana pres
 * std::function<bool(Vector3, Vector3)> v kontextu, cimz se zamezi priame
 * zavislosti SdfRenderer na Embree API.
 */

/**
 * @brief Kontext renderovani predavany metodam SdfRenderer pro jeden snimek.
 *
 * Vsechna data jsou predavana hodnotou nebo const-ref — zadne mutovatelne stav.
 */
struct SdfRenderContext {
    // --- SDF telesa ---
    const std::vector<Shape*>& shapes;    ///< Neprislusejici ukazatele na SDF telesa.
    Vector3                    animOffset; ///< Animacni posuv SDF sceny.

    // --- Parametry krochlovani ---
    float stepSize;           ///< Krok krochlovani Dt.
    float maxDistance;        ///< Vzdalenost reze paprsku (far plane).
    int   maxSteps;           ///< Maximalni pocet kroku krochlovani.
    float absorptionCoeff;    ///< sigma_t pro Beer-Lambertovu extinckci.
    float lightAttenuationFactor; ///< Exponent n pro pokles 1/d^n.

    // --- Osvetleni ---
    Vector3                    volumetricAlbedo; ///< Albedo rozptylu.
    const std::vector<Light>&  lights;           ///< Vsechna svetla pro volumetricke osvetleni.
    Light                      primaryLight;     ///< Primerni svetlo pro sphere-tracing.

    // --- Globalni slunce ---
    bool    sunEnabled;   ///< true = globalni slunce je zapnuto.
    Vector3 sunDir;       ///< Normalizovany smer ke slunci.
    Vector3 sunColor;     ///< Barva slunecniho svetla.
    float   sunIntensity; ///< Intenzita slunecniho svetla.

    // --- Callback pro viditelnost (abstrakce Embree shadow ray) ---
    /// Vraci true pokud je hitPoint videt ze smer lightPoint (zadny blokator).
    /// Ve sphere-tracingu se pouziva pro tvrd stin. Muze byt nullptr (= vzdy videt).
    std::function<bool(const Vector3&, const Vector3&)> isVisible;
};

/**
 * @brief Bezstavova trida pro SDF volumetricke krochlovani a sphere-tracing.
 *
 * Vsechny metody jsou const a nepristupuji k zadnemu stavovemu clenu.
 * Pristup k datum sceny probiha vyhradne pres SdfRenderContext.
 */
class SdfRenderer {
public:
    SdfRenderer()  = default;
    ~SdfRenderer() = default;

    // Zakaz kopirovani/presunu (trida je bezstavova, ale explicitnost je dobra praxe)
    SdfRenderer(const SdfRenderer&)            = delete;
    SdfRenderer& operator=(const SdfRenderer&) = delete;

    // =========================================================================
    // RENDEROVACI METODY
    // =========================================================================

    /// @brief Dispatcher: vybere mezi volumetricEffect a surfaceEffect dle ctx.rayMarching.
    /// @param ray        Primerni paprsek z kamery.
    /// @param ctx        Kontext renderovani pro tento snimek.
    /// @param rayMarching  true = volumetricke krochlovani; false = sphere-tracing.
    /// @return           Barva a opacita jako Vector4.
    [[nodiscard]] Vector4 render(const RTCRay& ray,
                                 const SdfRenderContext& ctx,
                                 bool rayMarching) const;

    /// @brief Volumetricke krochlovani pres SDF oblak (Beer-Lambert + NEE).
    ///
    /// Presna implementace referecniho ShaderToy raymarch() s adaptivnim krokem
    /// (sphere-tracing mimo oblak) a stinsovymi paprsky pro kazde svetlo.
    ///
    /// @param ray   Primerni paprsek.
    /// @param ctx   Kontext renderovani.
    /// @param tMax  Orizi krochlovani na tuto vzdalenost (pro COMBINED_SDF mod).
    /// @return      RGBA: RGB = nastrimovana barva, A = opacita.
    [[nodiscard]] Vector4 volumetricEffect(const RTCRay& ray,
                                           const SdfRenderContext& ctx,
                                           float tMax = FLT_MAX) const;

    /// @brief Sphere-tracing pro nalezeni prvniho pruseciku se SDF povrchem.
    ///
    /// Pouziva adaptivni krok SDF, ktery nikdy nepresahne povrch.
    /// Barva se pocita Phongovym modelem s tvrdymi stiny pres ctx.isVisible.
    ///
    /// @param ray  Primerni paprsek.
    /// @param ctx  Kontext renderovani.
    /// @return     RGBA: RGB = barva povrchu, A = 1.0 pri zasahu, 0.0 pri prubehu.
    [[nodiscard]] Vector4 surfaceEffect(const RTCRay& ray,
                                        const SdfRenderContext& ctx) const;

    /// @brief Vypocita propustnost volumetrickeho stinu od bodu k bodovemu svetlu.
    ///
    /// Odpovida funkci GetLightVisibility() referecniho ShaderToy (25 kroku).
    /// Pouziva Beer-Lambertovu extinckci skrze SDF oblak.
    ///
    /// @param samplePoint  Bod uvnitr oblaku, ze ktereho se pocita stin.
    /// @param lightPos     Pozice svetla (nebo virtualni bod pro slunce).
    /// @param ctx          Kontext renderovani (shapes, animOffset, absorptionCoeff, atd.).
    /// @return             Propustnost v rozsahu [0, 1]: 1 = plne osvetleno.
    [[nodiscard]] float computeShadowTransmittance(const Vector3& samplePoint,
                                                    const Vector3& lightPos,
                                                    const SdfRenderContext& ctx) const;
};
