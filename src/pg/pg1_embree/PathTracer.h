#pragma once

#include "stdafx.h"
#include "ShadingUtils.h"
#include "RenderTypes.h"
#include "vector3.h"
#include "light.h"
#include "cubemap.h"
#include "material.h"
#include "structs.h"
#include <embree3/rtcore.h>
#include <vector>
#include <functional>

/**
 * @file  PathTracer.h
 * @brief Trida pro Monte Carlo sledovani cest (path tracing) pres Embree BVH.
 *
 * PathTracer je bezstavova vypocetni trida — vsechny potrebne informace dostava
 * pres PathTracingContext. Implementuje iterativni (nerekurzivni) sledovani cest
 * s nasledujicimi vlastnostmi:
 *
 *   - Next Event Estimation (NEE) pro primy svetelny prispevek.
 *   - Ruske rulety pro nezkreslenou terminaci cesty (od hloubky rrMinDepth).
 *   - Lambertovska difuze s cosinovym vzorkovanim polokoule.
 *   - Phongova odrazivost (stochasticka volba dle reflectivity).
 *   - Dielektrika (sklo) s Fresnelovym odrazem/lomem (Snelluv zakon).
 *   - Podpora globalniho slunce (smerove svetlo bez poklesu 1/d^2).
 */

/**
 * @brief Kontext renderovani predavany metodam PathTracer pro jeden snimek.
 *
 * Obsahuje kopie nebo const-reference vsech dat potrebnych behem sledovani cest.
 * Embree scena je predavana primo (handle), ostatni data pres reference/kopie.
 */
struct PathTracingContext {
    RTCScene                scene;              ///< Embree scena pro BVH pruseciky.
    bool                    hasSurfaces;        ///< true pokud jsou nahrana nejaka telesa.
    const std::vector<Light>& lights;           ///< Vsechna svetla pro NEE.
    bool                    backgroundEnabled;  ///< true = pouzij cubemap jako pozadi.
    Vector3                 backgroundColor;    ///< Barva pevneho pozadi.
    const CubeMap*          cubemap;            ///< Environment mapa (muze byt nullptr).
    int                     rrMinDepth;         ///< Hloubka, od ktere zacina ruska ruleta.
    float                   maxDistance;        ///< Maximalni vzdalenost (far plane).
    float                   lightAttenuationFactor; ///< Exponent n pro 1/d^n pokles.

    // --- Globalni slunce ---
    bool    sunEnabled;   ///< true = globalni slunce je zapnuto.
    Vector3 sunDir;       ///< Normalizovany smer ke slunci.
    Vector3 sunColor;     ///< Barva slunecniho svetla.
    float   sunIntensity; ///< Intenzita slunecniho svetla.

    /// Callback pro test viditelnosti bodu ze svetla (abstrakce Embree shadow ray).
    std::function<bool(const Vector3&, const Vector3&)> isVisible;
};

/**
 * @brief Bezstavova trida pro Monte Carlo sledovani cest pres Embree BVH.
 */
class PathTracer {
public:
    PathTracer()  = default;
    ~PathTracer() = default;

    PathTracer(const PathTracer&)            = delete;
    PathTracer& operator=(const PathTracer&) = delete;

    // =========================================================================
    // HLAVNI RENDEROVACI METODY
    // =========================================================================

    /// @brief Sleduje jednu cestu sceny metodou Monte Carlo.
    ///
    /// Iterativni implementace (bez rekurze). Akumuluje radianci pres celu cestu
    /// s NEE na kazdem odrazu a ruskou ruletou pro terminaci.
    ///
    /// @param initialRay  Primerni paprsek z kamery.
    /// @param maxDepth    Maximalni delka cesty (hard limit).
    /// @param ctx         Kontext renderovani pro tento snimek.
    /// @return            Odhadovana radiance pro dany paprsek.
    [[nodiscard]] Vector3 tracePath(const RTCRay& initialRay,
                                    int maxDepth,
                                    const PathTracingContext& ctx) const;

    /// @brief Vypocita prispevek primiho svetla (NEE) v danem bode povrchu.
    ///
    /// Iteruje pres vsechna svetla a sectova Lambert-difuzni prispevky
    /// pro vsechna viditelna svetla. Bodova svetla pouzivaji shadow ray
    /// pres ctx.isVisible.
    ///
    /// @param hitPoint  Bod na povrchu kde pocitame osvetleni.
    /// @param normal    Normalizovana normala povrchu.
    /// @param albedo    Albedo difuze povrchu (barva materialu).
    /// @param ctx       Kontext renderovani.
    /// @return          Celkovy prispevek primiho svetla.
    [[nodiscard]] Vector3 sampleDirectLight(const Vector3& hitPoint,
                                             const Vector3& normal,
                                             const Vector3& albedo,
                                             const PathTracingContext& ctx) const;

    // =========================================================================
    // STATICKE POMOCNE METODY (matematika — zadny pristup ke stavu)
    // =========================================================================

    /// @brief Sestavi pravohlou ortonormalni bazi ze zadane normaly.
    /// Pouziva numericky robustni metodu Duff et al. 2017.
    /// @param n  Jednotkova normala (osa "up" baze).
    /// @param t  [out] Tangenta.
    /// @param b  [out] Bitangenta.
    static void buildONB(const Vector3& n, Vector3& t, Vector3& b);

    /// @brief Vygeneruje nahodny smer na polokouli s cosinovou vahovou funkci.
    /// PDF = cos(theta)/pi. Pro Lambertovske BRDF throughput vahovani se rusí.
    /// @param normal  Normala povrchu (osa polokoule).
    /// @return        Normalizovany smer ve svetovem prostoru.
    [[nodiscard]] static Vector3 sampleHemisphereCosine(const Vector3& normal);

    /// @brief Vyvazena MIS heuristika (Veach 1997, n_i = 1, n = 2).
    /// Pro bodova svetla (delta distribuce) se zjednodusuje na w_a = 1.
    [[nodiscard]] static float balancedHeuristic(float p_a, float p_b);

    /// @brief Vyhodnoceni normalizovane faze Henyey-Greenstein.
    /// g = 0: izotropni rozptyl, g > 0: dopredny rozptyl, g < 0: zpetny.
    [[nodiscard]] static float evaluateHenyeyGreenstein(float cosTheta, float g);
};
