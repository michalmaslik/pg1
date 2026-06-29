#include "stdafx.h"
#include "SceneManager.h"
#include "SceneLoader.h"
#include <iostream>
#include <cmath>
#include <algorithm>

//=============================================================================
// NACTENI POPISU SCEN
//=============================================================================

int SceneManager::loadScenesFromFile(const std::string& filePath)
{
    scnScenes_ = SceneLoader::LoadFromFile(filePath);
    return static_cast<int>(scnScenes_.size());
}

//=============================================================================
// STAV ANIMACI
//=============================================================================

void SceneManager::clearAnimations()
{
    activeLightDescs_.clear();
    activeEntityAnims_.clear();
    // sceneTime_ se nenuluje zde -- volajici to dela explicitne v loadPredefinedScene
}

//=============================================================================
// AKTUALIZACE ANIMACI (Update + updateEntityTransforms)
//=============================================================================

void SceneManager::update(const float time, SceneAnimResources& res)
{
    sceneTime_ = time;

    // -------------------------------------------------------------------
    // DATA-DRIVEN CESTA -- naplnena loadSceneFromDescription
    // -------------------------------------------------------------------
    if (!activeLightDescs_.empty()) {
        const size_t n = std::min(res.lights.size(), activeLightDescs_.size());
        for (size_t i = 0; i < n; ++i) {
            const LightDesc& ld = activeLightDescs_[i];
            // Pouze POINT svetla s ORBIT animaci potrebuji aktualizaci pozice
            if (ld.type == "POINT" && ld.animType == "ORBIT") {
                const float theta      = sceneTime_ * ld.animSpeed + ld.animPhase;
                res.lights[i].position.x = ld.px + cosf(theta) * ld.animRadius;
                res.lights[i].position.z = ld.pz + sinf(theta) * ld.animRadius;
                res.lights[i].position.y = ld.py;  // orbita v XZ rovine, Y pevne
            }
        }
        updateEntityTransforms(sceneTime_, res);
        return;
    }

    // -------------------------------------------------------------------
    // LEGACY CESTA -- hardcoded ShaderToy SDF animace
    // -------------------------------------------------------------------
    if (res.currentSceneType != SceneType::SCENE_SHADERTOY_SDF) {
        updateEntityTransforms(sceneTime_, res);
        return;
    }
    if (res.lights.size() < 3) {
        updateEntityTransforms(sceneTime_, res);
        return;
    }

    constexpr float kTwoThirdsPi = static_cast<float>(M_PI) * 2.0f / 3.0f;
    constexpr float kRadius      = 18.5f;
    const float     speed        = 0.7f * res.lightAnimSpeed;

    const Vector3 colors[3] = {
        Vector3(1.0f, 0.0f, 1.0f) * 17.0f,
        Vector3(0.0f, 1.0f, 0.0f) * 17.0f,
        Vector3(0.0f, 0.0f, 1.0f) * 17.0f,
    };

    for (int i = 0; i < 3; ++i) {
        const float theta       = sceneTime_ * speed + float(i) * kTwoThirdsPi;
        res.lights[i].position  = Vector3(
            kRadius * cosf(theta),
            6.0f + sinf(theta * 2.0f) * 2.5f,
            kRadius * sinf(theta));
        res.lights[i].color     = colors[i];
        res.lights[i].intensity = 1.0f;
    }

    updateEntityTransforms(sceneTime_, res);
}

//=============================================================================
// TRANSFORMACE ANIMOVANYCH ENTIT
//=============================================================================

void SceneManager::updateEntityTransforms(const float time, SceneAnimResources& res)
{
    if (activeEntityAnims_.empty()) return;

    // Sdileny zamek: clearScene (unique_lock) musi cekat na dokonceni teto metody.
    std::shared_lock<std::shared_mutex> animLock(res.sceneMutex);

    // Dvojita kontrola po zamknutni: clearScene mohlo vymazat seznam
    if (activeEntityAnims_.empty()) return;

    bool anyMeshUpdated = false;

    for (auto& anim : activeEntityAnims_) {
        if (anim.animType.empty()) continue;

        // -------------------------------------------------------------------
        // Vypocti translacni offset ze zakladni pozice
        // -------------------------------------------------------------------
        float tx = 0.0f, ty = 0.0f, tz = 0.0f;

        if (anim.animType == "ORBIT") {
            // Kruhova orbita v XZ rovine kolem pocatku
            const float theta = time * anim.animSpeed;
            tx = cosf(theta) * anim.animParam1 - anim.baseX;
            tz = sinf(theta) * anim.animParam1 - anim.baseZ;
        } else if (anim.animType == "HOVER") {
            // Sinusoidalni Y-osa oscilace kolem zakladni vysky
            ty = sinf(time * anim.animSpeed) * anim.animParam1;
        } else if (anim.animType == "PINGPONG") {
            // Hladky zpetny chod mezi zakladem a cilem (sinus hodiny)
            const float t = (sinf(time * anim.animSpeed) + 1.0f) * 0.5f;
            tx = t * (anim.targetX - anim.baseX);
            ty = t * (anim.targetY - anim.baseY);
            tz = t * (anim.targetZ - anim.baseZ);
        } else if (anim.animType == "TOWARDS") {
            // Jednokrokovy prichod: zacina v (targetX,Y,Z), konci v (baseX,Y,Z)
            const float t = std::min(time * anim.animSpeed, 1.0f);
            tx = (1.0f - t) * (anim.targetX - anim.baseX);
            ty = (1.0f - t) * (anim.targetY - anim.baseY);
            tz = (1.0f - t) * (anim.targetZ - anim.baseZ);
        }

        // -------------------------------------------------------------------
        // Aplikuj offset dle typu entity
        // -------------------------------------------------------------------
        if (anim.entityKind == "MESH") {
            for (auto& g : anim.geoms) {
                const int n = static_cast<int>(g.animVerts.size());
                for (int i = 0; i < n; ++i) {
                    g.animVerts[i].x = g.baseVerts[i].x + tx;
                    g.animVerts[i].y = g.baseVerts[i].y + ty;
                    g.animVerts[i].z = g.baseVerts[i].z + tz;
                }
                // Pouzij thread-safe variantu rtcGetGeometry (dokumentovana pro non-callback kod)
                RTCGeometry geom = rtcGetGeometryThreadSafe(res.scene, g.geomID);
                if (geom) {
                    rtcUpdateGeometryBuffer(geom, RTC_BUFFER_TYPE_VERTEX, 0);
                    rtcCommitGeometry(geom);
                    rtcReleaseGeometry(geom);
                    anyMeshUpdated = true;
                }
            }
        } else if (anim.entityKind == "SDF") {
            res.sdfAnimOffset.x = anim.baseX + tx;
            res.sdfAnimOffset.y = anim.baseY + ty;
            res.sdfAnimOffset.z = anim.baseZ + tz;
        } else if (anim.entityKind == "VDB") {
            res.vdbAnimOffset.x = anim.baseX + tx;
            res.vdbAnimOffset.y = anim.baseY + ty;
            res.vdbAnimOffset.z = anim.baseZ + tz;
        }
    }

    // Recommitni BVH jednou po vsech updatech meshe (levnejsi nez per-geometrie)
    if (anyMeshUpdated)
        rtcCommitScene(res.scene);
}