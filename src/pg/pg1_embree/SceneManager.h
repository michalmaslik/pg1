#pragma once

#include "stdafx.h"
#include "RenderTypes.h"
#include "light.h"
#include "SceneLoader.h"
#include "structs.h"
#include <embree3/rtcore.h>
#include <shared_mutex>
#include <vector>
#include <string>

/**
 * @file  SceneManager.h
 * @brief Spravce stavu sceny -- animace, popis scen a svetla.
 *
 * SceneManager vlastni veskerou data specificka pro spraveni sceny:
 *   - Animacni popisy svetel (activeLightDescs_)
 *   - Animacni stavy entit MESH/SDF/VDB (activeEntityAnims_)
 *   - Nacteny seznam scen ze souboru .scn (scnScenes_)
 *   - Casovac animace sceny (sceneTime_)
 *
 * Nakladani asetu (Embree, VDB, textury) zustava v RayTracer, ktery
 * deleguje pristup ke stavu pres SceneAnimResources.
 */

/**
 * @brief Animacni stav jedne entity (MESH, SDF nebo VDB) v aktivni scene.
 *
 * Pro MESH entity: sdilene vertex buffery jsou prepsiavany kazdy snimek.
 * Pro SDF/VDB entity: animacni posuv je aplikovan na vstupni paprsky.
 */
struct EntityAnimState {
    std::string entityKind;  ///< "MESH", "SDF", nebo "VDB"
    std::string animType;    ///< "ORBIT", "HOVER", "PINGPONG", "TOWARDS", nebo "" (staticke)
    float animSpeed{ 0.0f };  ///< Uhlova rychlost (ORBIT/HOVER) nebo rychlost lerpu
    float animParam1{ 0.0f }; ///< Polomer orbity (ORBIT) nebo amplituda (HOVER)
    float baseX{ 0.0f };      ///< Zakladni X ve svetovem prostoru
    float baseY{ 0.0f };      ///< Zakladni Y
    float baseZ{ 0.0f };      ///< Zakladni Z
    float targetX{ 0.0f };    ///< Cilove X pro PINGPONG/TOWARDS
    float targetY{ 0.0f };    ///< Cilove Y
    float targetZ{ 0.0f };    ///< Cilove Z

    /// Jedna zaznam na Surface uvnitr nacteneho OBJ (pouze pro entityKind == "MESH").
    struct PerGeom {
        unsigned int geomID{ RTC_INVALID_GEOMETRY_ID };
        std::vector<Vertex3f> baseVerts; ///< Puvodni pozice (nikdy nemenene)
        std::vector<Vertex3f> animVerts; ///< Sdilene s Embree; prepsiavane kazdy snimek
    };
    std::vector<PerGeom> geoms; ///< Naplneno pouze pro MESH entity
};

/**
 * @brief Zdroje predavane metodam SceneManager pro aktualizaci animaci.
 */
struct SceneAnimResources {
    std::vector<Light>&   lights;           ///< Svetla sceny
    RTCScene              scene;            ///< Embree scena (pro recommit)
    std::shared_mutex&    sceneMutex;       ///< Mutex pro sdileny zamek
    Vector3&              sdfAnimOffset;    ///< [out] Animacni posuv pro SDF
    Vector3&              vdbAnimOffset;    ///< [out] Animacni posuv pro VDB
    float                 lightAnimSpeed;   ///< Rychlost animace svetel
    SceneType             currentSceneType; ///< Typ aktualni sceny
};

/**
 * @brief Vlastni stav sceny a ridi animace entit a svetel.
 */
class SceneManager {
public:
    SceneManager()  = default;
    ~SceneManager() = default;

    SceneManager(const SceneManager&)            = delete;
    SceneManager& operator=(const SceneManager&) = delete;

    // =========================================================================
    // NACTENI POPISU SCEN
    // =========================================================================

    /// @brief Nacte seznam scen ze souboru .scn.
    int loadScenesFromFile(const std::string& filePath);

    [[nodiscard]] const std::vector<SceneDescription>& getScenes() const { return scnScenes_; }
    [[nodiscard]] bool hasScenes() const { return !scnScenes_.empty(); }
    [[nodiscard]] int  getSelectedIndex() const { return selectedSceneIdx_; }
    void               setSelectedIndex(const int idx) { selectedSceneIdx_ = idx; }

    [[nodiscard]] const SceneDescription& getSelectedScene() const {
        return scnScenes_[selectedSceneIdx_];
    }

    // =========================================================================
    // STAV ANIMACI
    // =========================================================================

    [[nodiscard]] std::vector<LightDesc>&        getLightDescs()        { return activeLightDescs_; }
    [[nodiscard]] const std::vector<LightDesc>&  getLightDescs()  const { return activeLightDescs_; }
    [[nodiscard]] std::vector<EntityAnimState>&       getEntityAnims()       { return activeEntityAnims_; }
    [[nodiscard]] const std::vector<EntityAnimState>& getEntityAnims() const { return activeEntityAnims_; }

    /// @brief Resetuje vsechny aktivni animace (vola se pri clearScene).
    void clearAnimations();

    [[nodiscard]] float getSceneTime() const { return sceneTime_; }

    // =========================================================================
    // AKTUALIZACE ANIMACI
    // =========================================================================

    /// @brief Aktualizuje pozice svetel a entit pro dany cas sceny.
    void update(float time, SceneAnimResources& res);

    /// @brief Aktualizuje transformace animovanych entit (MESH/SDF/VDB).
    void updateEntityTransforms(float time, SceneAnimResources& res);

private:
    std::vector<SceneDescription> scnScenes_;
    int                           selectedSceneIdx_{ 0 };
    std::vector<LightDesc>        activeLightDescs_;
    std::vector<EntityAnimState>  activeEntityAnims_;
    float                         sceneTime_{ 0.0f };
};