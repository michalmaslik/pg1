#ifndef PLANE_H_
#define PLANE_H_

#include "shape.h"

/**
 * @brief Reprezentuje nekonecnou rovinu ve 3D prostoru.
 *
 * Implementuje Signed Distance Function (SDF) pro rovinu definovanou
 * normalovym vektorem a vzdalenosti od pocatku souradnic.
 *
 * Matematicky odkaz:
 *   SDF(p, n, d) = dot(p, n) + d
 *
 * Priznak sumu (noiseEnabled_) je dedeny z Shape a lze jej nastavit
 * metodou setNoiseEnabled().
 */
class Plane : public Shape {
public:
	/// @brief Inicializuje rovinu zarovnanou s osou Z a vzdalenosti od pocatku.
	/// @param d  Vzdalenost roviny od pocatku souradnic podél normaly.
	/// @param k  Faktor hladkeho spojeni pro smooth-union operace.
	Plane(const float d, const float k = 1.0f)
		: Shape(k, {}), d_(d), normal_(0.0f, 0.0f, 1.0f) {
	}

	/// @brief Inicializuje rovinu se zadanym normalovym vektorem a vzdalenosti.
	/// @param normal  Normala roviny (mel by byt normalizovany).
	/// @param d       Vzdalenost roviny od pocatku souradnic podél normaly.
	/// @param k       Faktor hladkeho spojeni pro smooth-union operace.
	Plane(const Vector3& normal, const float d, const float k = 1.0f)
		: Shape(k, {}), d_(d), normal_(normal) {
	}

	/// @brief Vypocita SDF pro dany bod.
	/// Pokud je instancni priznak sumu zapnut, deleguje na SDFNoise().
	/// @param point  Testovany bod ve svetovych souradnicich.
	/// @return       SDF hodnota (zaporna pod rovinou).
	float SDF(const Vector3& point) const override {
		if (isNoiseEnabled()) {
			return SDFNoise(point);
		}
		// Obecne SDF roviny: dot(point, normal) + vzdalenost
		return point.DotProduct(normal_) + d_;
	}

	/// @brief Vypocita SDF s aplikovanym sumem (pouziva vlastni noise_ roviny).
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
		const float baseSDF = point.DotProduct(normal_) + d_;
		return baseSDF + noise.Generate(point);
	}

private:
	float   d_;      ///< Vzdalenost roviny od pocatku souradnic podél normaly.
	Vector3 normal_; ///< Normala roviny (mel by byt normalizovany).
};

#endif
