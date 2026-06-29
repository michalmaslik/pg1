#ifndef BOX_H_
#define BOX_H_

#include "shape.h"
#include "mymath.h"

/**
 * @brief Reprezentuje osu-zarovnany kvádr (Axis-Aligned Box) ve 3D prostoru.
 *
 * Implementuje Signed Distance Function (SDF) pro kvádr definovany stredovym
 * bodem a poluvlikosti podél kazde osy. Podporuje aplikaci proceduralniho
 * sumu pres instancni priznak noiseEnabled_ zdedeeny z Shape.
 */
class Box : public Shape {
public:
	/// @brief Inicializuje kvadr se stredovym bodem, poluvlikosti, faktorem hladkosti a sumem.
	/// @param center    Stredovy bod kvadru ve svetovych souradnicich.
	/// @param halfSize  Poluvlikost kvadru podél každé osy.
	/// @param k         Faktor hladkeho spojeni pro smooth-union operace.
	/// @param noise     Konfigurace proceduralniho sumu.
	Box(const Vector3& center, const Vector3& halfSize, const float k = 1.0f, const Noise& noise = {})
		: Shape(k, noise) {
		center_   = center;
		halfSize_ = halfSize;
	}

	/// @brief Vypocita SDF pro dany bod.
	/// Pokud je instancni priznak sumu zapnut, deleguje na SDFNoise().
	/// @param point  Testovany bod ve svetovych souradnicich.
	/// @return       SDF hodnota (zaporna uvnitr kvadru).
	float SDF(const Vector3& point) const override {
		if (isNoiseEnabled()) {
			return SDFNoise(point);
		}
		const Vector3 q = (point - center_).Abs() - halfSize_;
		return q.Max(0.0f).L2Norm() + std::min(std::max(q.x, std::max(q.y, q.z)), 0.0f);
	}

	/// @brief Vypocita SDF s aplikovanym sumem (pouziva vlastni noise_ kvadru).
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
		const Vector3 q        = (point - center_).Abs() - halfSize_;
		const float   distance = q.Max(0.0f).L2Norm() + std::min(std::max(q.x, std::max(q.y, q.z)), 0.0f);
		return distance + noise.Generate(point);
	}

private:
	Vector3 center_;   ///< Stredovy bod kvadru ve svetovych souradnicich.
	Vector3 halfSize_; ///< Poluvlikost kvadru podél každé osy.
};

#endif
