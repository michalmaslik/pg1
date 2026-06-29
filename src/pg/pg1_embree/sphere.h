#ifndef SPHERE_H_
#define SPHERE_H_

#include "shape.h"
#include "mymath.h"

/**
 * @brief Reprezentuje 3D kouli definovanou stredovym bodem a polomerem.
 *
 * Implementuje Signed Distance Function (SDF) pro kouli a podporuje
 * aplikaci proceduralniho sumu pro efekty organickeho tvaru.
 * Priznak sumu (noiseEnabled_) je dedeny z Shape a lze jej nastavit
 * metodou setNoiseEnabled().
 */
class Sphere : public Shape {
public:
	/// @brief Inicializuje kouli se stredovym bodem, polomerem, faktorem hladkosti a sumem.
	/// @param center  Stredovy bod koule ve svetovych souradnicich.
	/// @param radius  Polomer koule.
	/// @param k       Faktor hladkeho spojeni pro smooth-union operace.
	/// @param noise   Konfigurace proceduralniho sumu.
	Sphere(const Vector3& center, const float radius, const float k = 1.0f, const Noise& noise = {})
		: Shape(k, noise) {
		center_ = center;
		radius_ = radius;
	}

	/// @brief Vypocita SDF pro dany bod.
	/// Pokud je instancni priznak sumu zapnut, deleguje na SDFNoise().
	/// @param point  Testovany bod ve svetovych souradnicich.
	/// @return       SDF hodnota (zaporna uvnitr koule).
	float SDF(const Vector3& point) const override {
		if (isNoiseEnabled()) {
			return SDFNoise(point);
		}
		return (point - center_).L2Norm() - radius_;
	}

	/// @brief Vypocita SDF s aplikovanym sumem (pouziva vlastni noise_ koule).
	/// @param point  Testovany bod.
	/// @return       SDF hodnota s proceduralnim posunem.
	float SDFNoise(const Vector3& point) const override {
		return SDFNoise(point, noise_);
	}

	/// @brief Vypocita SDF s explicitne predanou konfiguraci sumu.
	/// @param point  Testovany bod.
	/// @param noise  Konfigurace sumu, ktera ma byt pouzita.
	/// @return       SDF hodnota s proceduralnim posunem.
	float SDFNoise(const Vector3& point, const Noise& noise) const override {
		const float distance = (point - center_).L2Norm() - radius_;
		return distance + noise.Generate(point);
	}

private:
	Vector3 center_; ///< Stredovy bod koule ve svetovych souradnicich.
	float   radius_; ///< Polomer koule.
};

#endif
