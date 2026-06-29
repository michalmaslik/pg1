#ifndef MATERIAL_H_
#define MATERIAL_H_

#include "vector3.h"
#include "texture.h"

/*! \def NO_TEXTURES
\brief Maximální počet textur přiřazených materiálu.
*/
#define NO_TEXTURES 4

/*! \def IOR_AIR
\brief Index lomu vzduchu za normálního tlaku.
*/
#define IOR_AIR 1.000293f

/*! \def IOR_WATER
\brief Index lomu vody.
*/
#define IOR_WATER 1.33f

/*! \def IOR_GLASS
\brief Index lomu skla.
*/
#define IOR_GLASS 1.5f

/*! \class Material
\brief A simple physically-based material.

All data members are private. Access is through [[nodiscard]] getters
and corresponding setters. The virtual destructor ensures safe
polymorphic deletion.

\author Tomáš Fabián (original), refactored for Modern C++20
\version 1.1
\date 2024
*/
class Material
{
public:
	// -------------------------------------------------------------------------
	// Construction / Destruction
	// -------------------------------------------------------------------------

	//! Default constructor: grey matte material.
	Material();

	//! Full constructor.
	/*!
	\param name       Material name.
	\param ambient    Ambient RGB colour.
	\param diffuse    Diffuse RGB colour.
	\param specular   Specular RGB colour.
	\param emission   Emissive RGB colour.
	\param reflectivity Reflectivity coefficient.
	\param shininess  Phong shininess exponent.
	\param ior        Index of refraction.
	\param textures   Array of texture pointers (optional).
	\param no_textures Length of the textures array.
	*/
	Material(std::string& name, const Vector3& ambient, const Vector3& diffuse,
		const Vector3& specular, const Vector3& emission, const float reflectivity,
		const float shininess, const float ior,
		Texture** textures = nullptr, const int no_textures = 0);

	//! Virtual destructor: releases owned texture objects.
	virtual ~Material();

	// -------------------------------------------------------------------------
	// Name & Texture API (legacy snake_case kept for compatibility)
	// -------------------------------------------------------------------------

	void        set_name(const char* name);
	[[nodiscard]] std::string get_name() const;

	void      set_texture(const int slot, Texture* texture);
	[[nodiscard]] Texture* get_texture(const int slot) const;

	// -------------------------------------------------------------------------
	// Texture slot indices (public static constants)
	// -------------------------------------------------------------------------

	static const char kDiffuseMapSlot;
	static const char kSpecularMapSlot;
	static const char kNormalMapSlot;
	static const char kOpacityMapSlot;

	// -------------------------------------------------------------------------
	// [[nodiscard]] Getters
	// -------------------------------------------------------------------------

	[[nodiscard]] const Vector3& GetAmbient()      const { return ambient_; }
	[[nodiscard]] const Vector3& GetDiffuse()      const { return diffuse_; }
	[[nodiscard]] const Vector3& GetSpecular()     const { return specular_; }
	[[nodiscard]] const Vector3& GetEmission()     const { return emission_; }
	[[nodiscard]] const Vector3& GetAttenuation()  const { return attenuation_; }
	[[nodiscard]] const Vector3& GetScattering()   const { return scattering_; }
	[[nodiscard]] const Vector3& GetAbsorption()   const { return absorption_; }

	[[nodiscard]] float GetShininess()    const { return shininess_; }
	[[nodiscard]] float GetReflectivity() const { return reflectivity_; }
	[[nodiscard]] float GetIor()          const { return ior_; }
	[[nodiscard]] float GetDensity()      const { return density_; }
	[[nodiscard]] float GetPhaseG()       const { return phase_g_; }
	[[nodiscard]] int   GetShader()       const { return shader_; }

	// -------------------------------------------------------------------------
	// Setters
	// -------------------------------------------------------------------------

	void SetAmbient(const Vector3& v)     { ambient_ = v; }
	void SetDiffuse(const Vector3& v)     { diffuse_ = v; }
	void SetSpecular(const Vector3& v)    { specular_ = v; }
	void SetEmission(const Vector3& v)    { emission_ = v; }
	void SetAttenuation(const Vector3& v) { attenuation_ = v; }
	void SetScattering(const Vector3& v)  { scattering_ = v; }
	void SetAbsorption(const Vector3& v)  { absorption_ = v; }

	void SetShininess(const float v)    { shininess_ = v; }
	void SetReflectivity(const float v) { reflectivity_ = v; }
	void SetIor(const float v)          { ior_ = v; }
	void SetDensity(const float v)      { density_ = v; }
	void SetPhaseG(const float v)       { phase_g_ = v; }
	void SetShader(const int v)         { shader_ = v; }

private:
	// -------------------------------------------------------------------------
	// Private data
	// -------------------------------------------------------------------------

	Vector3 ambient_;     /*!< RGB ambient colour. */
	Vector3 diffuse_;     /*!< RGB diffuse colour. */
	Vector3 specular_;    /*!< RGB specular colour. */
	Vector3 emission_;    /*!< RGB emissive colour. */
	Vector3 attenuation_; /*!< Per-channel Beer-Lambert attenuation. */
	Vector3 scattering_;  /*!< Volumetric scattering coefficient. */
	Vector3 absorption_;  /*!< Volumetric absorption coefficient. */

	float shininess_{ 1.0f };    /*!< Phong shininess exponent. */
	float reflectivity_{ 0.99f }; /*!< Reflectivity coefficient. */
	float ior_{ -1.0f };          /*!< Index of refraction (-1 = not set). */
	float density_{ 0.0f };       /*!< Volumetric density. */
	float phase_g_{ 0.0f };       /*!< Henyey-Greenstein asymmetry parameter. */
	int   shader_{ -1 };          /*!< Shader type selector. */

	Texture*    textures_[NO_TEXTURES]{}; /*!< Texture slots (0=diffuse,1=spec,2=normal,3=opacity). */
	std::string name_{ "default" };        /*!< Material name. */
};

#endif
