#pragma once

/**
 * @file  RenderTypes.h
 * @brief Vycty a typy pouzivane v renderovacim jadre.
 *
 * Tento soubor definuje vsechny sdilene vycty (enum class) pouzivane
 * v renderovacim pipelinu. Jsou oddeleny od tridy RayTracer, aby je
 * mohly pouzivat i nove extrahované tridy (SdfRenderer, VdbRenderer,
 * PathTracer, SceneManager, RayTracerUI) bez cyklicke zavislosti.
 */

/**
 * @brief Interni renderovaci rezim — urcuje, ktery pipeline se spusti pro kazdy pixel.
 *
 * Hodnota je urcena funkci ResolveActiveMode() podle nahranych asetu
 * a aktualne vybraneho filtru (GlobalRenderFilter).
 */
enum class RenderingMode {
    SURFACE_EMBREE,       ///< Whittovske sledovani paprsku pres Embree BVH.
    VOLUMETRIC_SDF,       ///< Volumetricke krochlovani pres SDF telesa.
    VOLUMETRIC_VDB,       ///< Volumetricke krochlovani pres VDB objem (OpenVKL).
    COMBINED_SDF,         ///< Hloubkova kompozice: SDF objem pred Embree povrchem.
    PATH_TRACING_EMBREE,  ///< Monte Carlo sledovani cest (Embree BVH + NEE + RR).
    COMBINED_PT_SDF,      ///< Sledovani cest s SDF objemem nad povrchem.
    COMBINED_PT_VDB       ///< Sledovani cest s VDB dymem nad povrchem.
};

/**
 * @brief Vysokourovnovy filtr renderovani voleny uzivatelem v UI.
 *
 * Funkce ResolveActiveMode() prevede tento filtr spolecne s priznakem
 * objUsePathTracing_ na konkretni hodnotu RenderingMode.
 */
enum class GlobalRenderFilter {
    ONLY_OBJ,     ///< Pouze mesh (Whitted nebo PT dle objUsePathTracing_).
    ONLY_VDB,     ///< Pouze VDB objem.
    ONLY_SDF,     ///< Pouze SDF objem.
    OBJ_SDF,      ///< Mesh kompozovany se SDF objemem.
    OBJ_VDB,      ///< Mesh kompozovany s VDB dymem (vzdy COMBINED_PT_VDB).
    ONLY_VOLUME,  ///< Nejlepsi dostupny objem: VDB pokud je nacteny, jinak SDF.
    COMBINED,     ///< Nejkompletniejsi rezim podle nactenych asetu.
};

/**
 * @brief Typ predefinovane sceny — urcuje, jake assety a svetla se nactou.
 */
enum class SceneType {
    SCENE_SHADERTOY_SDF, ///< ShaderToy-styl SDF oblak se 3 animovanymi bodovymi svetly.
    SCENE_CUSTOM_OBJ,    ///< Libovolny OBJ soubor nacteny za behu, renderovany Embree.
    SCENE_CUSTOM_VDB     ///< Libovolny VDB soubor nacteny za behu, renderovany OpenVKL.
};
