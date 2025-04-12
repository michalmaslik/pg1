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

/*! \def IOR_AIR
\brief Index lomu vody.
*/
#define IOR_WATER 1.33f

/*! \def IOR_GLASS
\brief Index lomu skla.
*/
#define IOR_GLASS 1.5f

/*! \class Material
\brief A simple material.

\author Tomáš Fabián
\version 0.9
\date 2011-2018
*/
class Material
{
public:
	//! Implicitní konstruktor.
	/*!
	Inicializuje všechny složky materiálu na výchozí matně šedý materiál.
	*/
	Material();

	//! Specializovaný konstruktor.
	/*!
	Inicializuje materiál podle zadaných hodnot parametrů.

	\param name název materiálu.
	\param ambient barva prostředí.
	\param diffuse barva rozptylu.
	\param specular barva odrazu.
	\param emission  barva emise.
	\param shininess lesklost.
	\param ior index lomu.
	\param textures pole ukazatelů na textury.
	\param no_textures délka pole \a textures. Maximálně \a NO_TEXTURES - 1.
	*/
	Material(std::string& name, const Vector3& ambient, const Vector3& diffuse,
		const Vector3& specular, const Vector3& emission, const float reflectivity,
		const float shininess, const float ior,
		Texture** textures = NULL, const int no_textures = 0);

	//! Destruktor.
	/*!
	Uvolní všechny alokované zdroje.
	*/
	~Material();

	//void Print();

	//! Nastaví název materiálu.
	/*!
	\param name název materiálu.
	*/
	void set_name(const char* name);

	//! Vrátí název materiálu.
	/*!
	\return Název materiálu.
	*/
	std::string get_name() const;

	//! Nastaví texturu.
	/*!
	\param slot číslo slotu, do kterého bude textura přiřazena. Maximálně \a NO_TEXTURES - 1.
	\param texture ukazatel na texturu.
	*/
	void set_texture(const int slot, Texture* texture);

	//! Vrátí texturu.
	/*!
	\param slot číslo slotu textury. Maximálně \a NO_TEXTURES - 1.
	\return Ukazatel na zvolenou texturu.
	*/
	Texture* get_texture(const int slot) const;

public:
	Vector3 ambient; /*!< RGB barva prostředí \f$\left<0, 1\right>^3\f$. */
	Vector3 diffuse; /*!< RGB barva rozptylu \f$\left<0, 1\right>^3\f$. */
	Vector3 specular; /*!< RGB barva odrazu \f$\left<0, 1\right>^3\f$. */

	Vector3 emission; /*!< RGB barva emise \f$\left<0, 1\right>^3\f$. */

	float shininess; /*!< Koeficient lesklosti (\f$\ge 0\f$). Čím je hodnota větší, tím se jeví povrch lesklejší. */

	float reflectivity; /*!< Koeficient odrazivosti. */
	float ior; /*!< Index lomu. */

	Vector3 attenuation; /*!<  */
	int shader; /*!<  */

	// Volumetric properties
	float density; /*!< Hustota volumetrického materiálu */
	Vector3 scattering; /*!< Koeficient rozptylu svetla */
	Vector3 absorption; /*!< Koeficient absorpcie svetla */
	float phase_g; /*!< Parameter Henyey-Greenstein fázy */

	static const char kDiffuseMapSlot; /*!< Číslo slotu difuzní textury. */
	static const char kSpecularMapSlot; /*!< Číslo slotu spekulární textury. */
	static const char kNormalMapSlot; /*!< Číslo slotu normálové textury. */
	static const char kOpacityMapSlot; /*!< Číslo slotu transparentní textury. */

private:
	Texture* textures_[NO_TEXTURES]; /*!< Pole ukazatelů na textury. */
	/*
	slot 0 - diffuse map + alpha
	slot 1 - specular map + opaque alpha
	slot 2 - normal map
	slot 3 - transparency map
	*/

	std::string name_; /*!< Název materiálu. */
};

#endif
