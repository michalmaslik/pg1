#ifndef SHAPE_H_
#define SHAPE_H_

#include "mymath.h"
#include "noise.h"

/**
 * @brief Abstraktni zakladni trida pro vsechna 3D SDF telesa.
 *
 * Poskytuje spolecne rozhrani pro vypocet Signed Distance Function (SDF)
 * a jeji varianty s aplikovanym proceduralnim sumem (fBm noise).
 *
 * Poznamky k zapouzdreni:
 *   - k_            je privatni; pristup pres GetSmoothFactor() / SetSmoothFactor().
 *   - noise_        je chraneny, aby odvozene tridy mohly volat noise_.Generate() primo.
 *   - noiseEnabled_ je instancni promenna (nikoliv staticka globalni); pro zmenu
 *                   pouzij setNoiseEnabled(). SmoothUnion pretizuje tuto metodu
 *                   a propaguje priznak do vsech svych potomku.
 */
class Shape {
public:
	/// @brief Inicializuje teleso s faktorem hladkeho spojeni a konfiguraci sumu.
	/// @param k      Polomer hladkeho spojeni pro smooth-union operace.
	/// @param noise  Konfigurace proceduralniho sumu.
	Shape(const float k = 1.0f, const Noise& noise = {}) : k_(k), noise_(noise) {}

	/// @brief Virtualni destruktor zajistuje spravne uvolneni odvozene tridy.
	virtual ~Shape() = default;

	// --- Ciste virtualni SDF rozhrani ---

	/// @brief Vypocita Signed Distance Function pro dany bod ve 3D prostoru.
	/// @param point  Testovany bod ve svetovych souradnicich.
	/// @return       Vzdalena hodnota SDF (zaporna uvnitr telesa).
	virtual float SDF(const Vector3& point) const = 0;

	/// @brief Vypocita SDF s aplikovanym sumem (pouziva vlastni noise_ telesa).
	/// @param point  Testovany bod.
	/// @return       SDF hodnota s proceduralnim posunem.
	virtual float SDFNoise(const Vector3& point) const = 0;

	/// @brief Vypocita SDF s explicitne predanou konfiguraci sumu.
	/// @param point  Testovany bod.
	/// @param noise  Konfigurace sumu, ktera ma byt pouzita.
	/// @return       SDF hodnota s proceduralnim posunem.
	virtual float SDFNoise(const Vector3& point, const Noise& noise) const = 0;

	// --- Pristupove metody pro faktor hladkeho spojeni ---

	/// @brief Vraci polomer hladkeho spojeni k pouzivany operaci smooth-union.
	[[nodiscard]] float GetSmoothFactor() const { return k_; }

	/// @brief Nastavi polomer hladkeho spojeni k.
	/// SmoothUnion pretizuje tuto metodu a propaguje hodnotu do potomku.
	virtual void SetSmoothFactor(const float k) { k_ = k; }

	// --- Pristupove metody pro sum ---

	/// @brief Nastavi prostorovou frekvenci sumu.
	void SetNoiseScale(const float scale)       { noise_.SetScale(scale); }

	/// @brief Nastavi amplitudu sumu.
	void SetNoiseStrength(const float strength) { noise_.SetStrength(strength); }

	// --- Instancni priznak sumu (nahrazuje puvodni statickou promennou) ---

	/// @brief Vraci, zda je sum pro toto teleso povolen.
	[[nodiscard]] bool isNoiseEnabled() const { return noiseEnabled_; }

	/// @brief Povoli nebo zakaze sum pro toto teleso.
	/// SmoothUnion pretizuje tuto metodu a propaguje priznak do vsech potomku.
	/// @param flag  true = sum zapnut, false = sum vypnut.
	virtual void setNoiseEnabled(const bool flag) { noiseEnabled_ = flag; }

protected:
	Noise noise_; ///< Konfigurace sumu pristupna v odvozanych tridach.

private:
	float k_;                    ///< Faktor hladkeho spojeni pro SDF blending operace.
	bool  noiseEnabled_{ true }; ///< Instancni priznak: true = pouzij fBm-posunuty SDF.
};

#endif
