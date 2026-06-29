#pragma once

/**
 * @file  RayTracerUI.h
 * @brief ImGui uzivatelske rozhrani pro RayTracer.
 *
 * RayTracerUI je deklarovana jako friend trida v RayTracer, takze ma
 * plny pristup ke vsem privatnim clenum. Tato trida oddeluje veskery
 * ImGui kod od jadra rendereru.
 *
 * RayTracer vlastni instanci pres unique_ptr<RayTracerUI>
 * a deleguje metodu Ui() na RayTracerUI::build().
 */

// Pouze dopredna deklarace -- plna definice je v raytracer.h (includovana v .cpp)
class RayTracer;

/**
 * @brief Implementuje ImGui ovladaci panel pro RayTracer.
 */
class RayTracerUI {
public:
    /// @brief Inicializuje UI pres odkaz na vlastnici RayTracer.
    explicit RayTracerUI(RayTracer& rt) : rt_(rt) {}

    ~RayTracerUI() = default;

    RayTracerUI(const RayTracerUI&)            = delete;
    RayTracerUI& operator=(const RayTracerUI&) = delete;

    /// @brief Sestavi cely ImGui panel a vraci 0 (kompatibilita s Ui() rozhranim).
    int build();

private:
    RayTracer& rt_; ///< Odkaz na vlastnici RayTracer (pristup pres friend deklaraci).
};