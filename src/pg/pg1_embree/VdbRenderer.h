#pragma once

#include "stdafx.h"
#include "ShadingUtils.h"
#include "vector3.h"
#include "vector4.h"
#include "light.h"
#include "cubemap.h"
#include "vdb_loader.h"
#include <openvkl/openvkl.h>
#include <openvkl/devices/cpu/openvkl/device/openvkl.h>
#include <string>
#include <memory>

/**
 * @file  VdbRenderer.h
 * @brief Trida zapouzdrujici cely OpenVKL subsystem pro renderovani VDB obejmu.
 *
 * VdbRenderer vlastni vse potrebne pro praci s VDB objemy:
 *   - OpenVKL zarizeni (VKLDevice)
 *   - OpenVKL objem   (VKLVolume)
 *   - OpenVKL vzorkovac (VKLSampler)
 *   - Loader VDB souboru (VdbLoader)
 *
 * RayTracer vlastni instanci teto tridy jako unique_ptr<VdbRenderer>
 * a deleguje vsechny VDB operace na ni. Vlakno-bezpecnost (sceneMutex_)
 * je resena na strane RayTraceru — metody teto tridy nejsou thread-safe.
 */

/**
 * @brief Kontext renderovani predavany metode rayMarch() pro jeden snimek.
 *
 * Obsahuje kopie vsech nastaveni potrebnych behem pruchodu VDB objemem.
 * Kopie jsou ulozeny hodnotou (nikoliv referencemi), aby byl kontext
 * bezpecny pro pouziti ve vicevlaknovem prostredi.
 */
struct VdbRenderContext {
    Vector3       animOffset;             ///< Animacni posuv objemu ve svetovem prostoru.
    float         stepSize;               ///< Krok krochlovani Dt.
    float         absorptionCoeff;        ///< sigma_t pro Beer-Lambertovu extinckci.
    float         lightAttenuationFactor; ///< Exponent n pro pokles intenzity 1/d^n.
    Vector3       volumetricAlbedo;       ///< Albedo rozptylu pro koloraci objemu.
    Light         primaryLight;           ///< Primerni svetlo pro vnitrni osvetleni objemu.
    bool          backgroundEnabled;      ///< true = pouzij cubemap jako pozadi.
    Vector3       backgroundColor;        ///< Barva pevneho pozadi (pouziva se kdyz backgroundEnabled=false).
    const CubeMap* cubemap;              ///< Ukazatel na environment mapu (muze byt nullptr).
    bool          sunEnabled;             ///< true = globalni slunce je zapnuto.
    Vector3       sunColor;               ///< Barva slunecniho svetla.
    float         sunIntensity;           ///< Intenzita slunecniho svetla.
};

/**
 * @brief Zapouzdruje OpenVKL subsystem pro nacitani a renderovani VDB obejmu.
 */
class VdbRenderer {
public:
    /// @brief Vytvori VdbRenderer s neinicializovanym stavem.
    VdbRenderer() = default;

    /// @brief Destruktor; uvolni vsechny OpenVKL zdroje pres cleanup().
    ~VdbRenderer();

    // Zakaz kopirovani (OpenVKL handles nejsou kopirovatelne)
    VdbRenderer(const VdbRenderer&)            = delete;
    VdbRenderer& operator=(const VdbRenderer&) = delete;

    // Povoleni presunu
    VdbRenderer(VdbRenderer&&)            = default;
    VdbRenderer& operator=(VdbRenderer&&) = default;

    // =========================================================================
    // INICIALIZACE A UVOLNENI
    // =========================================================================

    /// @brief Inicializuje OpenVKL zarizeni a vytvori VdbLoader.
    /// @return true pokud inicializace probehla uspesne, jinak false.
    bool initialize();

    /// @brief Uvolni vsechny OpenVKL zdroje (sampler, volume, device, loader).
    void cleanup();

    // =========================================================================
    // NACITANI VDB SVAZKU
    // =========================================================================

    /// @brief Uvolni pouze svazek a sampler, zarizeni zachova (pro hot-swap sceny).
    /// Pouziva se v ClearScene() kde je nutne zachovat device mezi nacitanimi scen.
    void clearVolume();

    /// @brief Nacte VDB soubor a pripravi OpenVKL sampler.
    /// Predchozi svazek je automaticky uvolnen pred nacitanim noveho.
    /// @param filename  Cesta k souboru .vdb.
    /// @param gridName  Nazev gridu v souboru (vychozi: "density").
    /// @return true pokud bylo nacitani uspesne, jinak false.
    bool loadVolume(const std::string& filename, const std::string& gridName = "density");

    // =========================================================================
    // DOTAZY
    // =========================================================================

    /// @brief Vraci true pokud je nacteny platny VDB svazek.
    [[nodiscard]] bool hasVolume() const { return vklVolume_ != VKLVolume{}; }

    /// @brief Vrati obalove teleso nacteneho VDB svazku.
    /// @param minBounds  [out] Minimalni roh obaloveho kvadru.
    /// @param maxBounds  [out] Maximalni roh obaloveho kvadru.
    void getBounds(Vector3& minBounds, Vector3& maxBounds) const;

    // =========================================================================
    // RENDEROVANI
    // =========================================================================

    /// @brief Provede volumetricke krochlovani pres VDB svazek pro dany paprsek.
    ///
    /// Pouziva Beer-Lambertovu extinckci a jednoduche osvetleni (bod + slunce).
    /// Vraci RGBA: RGB = nastrimovana barva, A = opacita.
    ///
    /// @param ray          Primerni paprsek z kamery.
    /// @param ctx          Kontext renderovani pro tento snimek.
    /// @param compositeBg  true = slucuj s pozadim uvnitr metody;
    ///                     false = vrat surove (RGB, opacita) pro externi slucovani.
    /// @return             Vypoctena barva a opacita jako Vector4.
    [[nodiscard]] Vector4 rayMarch(const RTCRay& ray,
                                   const VdbRenderContext& ctx,
                                   bool compositeBg = true) const;

private:
    VKLDevice                  vklDevice_{};  ///< OpenVKL zarizeni.
    VKLVolume                  vklVolume_{};  ///< OpenVKL VDB svazek.
    VKLSampler                 vklSampler_{}; ///< OpenVKL vzorkovac.
    std::unique_ptr<VdbLoader> vdbLoader_;    ///< Loader VDB souboru.
};
