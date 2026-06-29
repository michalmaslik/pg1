#ifndef SMOOTH_UNION_H_
#define SMOOTH_UNION_H_

#include "shape.h"
#include <memory>
#include <vector>

/**
 * @brief Reprezentuje hladke sjednoceni vice 3D SDF teles.
 *
 * Implementuje SDF pro hladke sjednoceni pomoci C^1-spojite funkce
 * smoothMin. Faktor hladkosti k je ulozen v kazdem potomkovi.
 * Priznak sumu (noiseEnabled_) se pomoci pretizene metody setNoiseEnabled()
 * automaticky propaguje do vsech potomku, cimz nahrazuje puvodni
 * statickou globalnu promennou.
 *
 * Poznamky k zapouzdreni:
 *   - shapes_ je privatni; vlastnictvi pres unique_ptr<Shape>.
 *   - SetSmoothFactor() je pretizena pro propagaci k do vsech potomku.
 *   - setNoiseEnabled() je pretizena pro propagaci do vsech potomku.
 */
class SmoothUnion : public Shape {
public:
	/// @brief Prevzame vlastnictvi potomku pomoci move semantiky.
	/// @param shapes  Vektor unikatnich ukazatelu na potomky.
	/// @param noise   Konfigurace proceduralniho sumu pro tento uzel.
	SmoothUnion(std::vector<std::unique_ptr<Shape>> shapes, const Noise& noise)
		: Shape(1.0f, noise), shapes_(std::move(shapes)) {}

	/// @brief Vypocita SDF pro dany bod.
	/// Pokud je instancni priznak sumu zapnut, deleguje na SDFNoise().
	/// Jinak rekurzivne vola SDF() potomku a aplikuje hladke minimum.
	/// @param point  Testovany bod ve svetovych souradnicich.
	/// @return       SDF hodnota hladkeho sjednoceni.
	float SDF(const Vector3& point) const override {
		if (isNoiseEnabled()) {
			return SDFNoise(point);
		}
		if (shapes_.empty()) return FLT_MAX;

		float sdfValue = shapes_[0]->SDF(point);
		for (size_t i = 1; i < shapes_.size(); ++i) {
			const float d1 = sdfValue;
			const float d2 = shapes_[i]->SDF(point);
			sdfValue = Smooth(d1, d2, shapes_[i]->GetSmoothFactor());
		}
		return sdfValue;
	}

	/// @brief Vypocita SDF s aplikovanym sumem (pouziva vlastni noise_ uzlu).
	/// @param point  Testovany bod.
	/// @return       SDF hodnota s proceduralnim posunem.
	float SDFNoise(const Vector3& point) const override {
		return SDFNoise(point, noise_);
	}

	/// @brief Vypocita SDF s explicitne predanou konfiguraci sumu.
	/// Rekurzivne vola SDFNoise() vsech potomku a aplikuje hladke minimum.
	/// @param point  Testovany bod.
	/// @param noise  Konfigurace sumu, ktera ma byt pouzita.
	/// @return       SDF hodnota hladkeho sjednoceni s proceduralnim posunem.
	float SDFNoise(const Vector3& point, const Noise& noise) const override {
		if (shapes_.empty()) return FLT_MAX;

		float sdfValue = shapes_[0]->SDFNoise(point, noise);
		for (size_t i = 1; i < shapes_.size(); ++i) {
			const float d1 = sdfValue;
			const float d2 = shapes_[i]->SDFNoise(point, noise);
			sdfValue = Smooth(d1, d2, shapes_[i]->GetSmoothFactor());
		}
		return sdfValue;
	}

	/// @brief Nastavi faktor hladkeho spojeni k a propaguje ho do vsech potomku.
	/// @param k  Novy polomer hladkeho spojeni.
	void SetSmoothFactor(const float k) override {
		Shape::SetSmoothFactor(k);
		for (auto& child : shapes_) {
			child->SetSmoothFactor(k);
		}
	}

	/// @brief Nastavi priznak sumu a propaguje ho do vsech potomku.
	/// Tato metoda pretizuje Shape::setNoiseEnabled() a zajistuje,
	/// ze zmena se projevuje konzistentne v celem strome teles.
	/// @param flag  true = sum zapnut, false = sum vypnut.
	void setNoiseEnabled(const bool flag) override {
		Shape::setNoiseEnabled(flag);
		for (auto& child : shapes_) {
			child->setNoiseEnabled(flag);
		}
	}

	/// @brief C^1-spojita funkce hladkeho minima dvou SDF hodnot.
	/// Implementuje polynomial smooth minimum podle IQ (iquilezles.com).
	/// @param d1  Prvni SDF hodnota.
	/// @param d2  Druha SDF hodnota.
	/// @param k   Polomer hladkeho prechodu.
	/// @return    Hladke minimum hodnot d1 a d2.
	static float Smooth(const float d1, const float d2, const float k) {
		const float h = clamp(0.5f + 0.5f * (d2 - d1) / k, 0.0f, 1.0f);
		return (d2 * (1.0f - h) + d1 * h) - k * h * (1.0f - h);
	}

private:
	std::vector<std::unique_ptr<Shape>> shapes_; ///< Vlastnene potomky.
};

#endif
