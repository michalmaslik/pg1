#include "stdafx.h"
#include "material.h"

const char Material::kDiffuseMapSlot = 0;
const char Material::kSpecularMapSlot = 1;
const char Material::kNormalMapSlot = 2;
const char Material::kOpacityMapSlot = 3;

Material::Material()
{
	ambient_     = Vector3(0.1f, 0.1f, 0.1f);
	diffuse_     = Vector3(0.4f, 0.4f, 0.4f);
	specular_    = Vector3(0.8f, 0.8f, 0.8f);
	emission_    = Vector3(0.0f, 0.0f, 0.0f);
	attenuation_ = Vector3(0.0f, 0.0f, 0.0f);
	scattering_  = Vector3(0.0f, 0.0f, 0.0f);
	absorption_  = Vector3(0.0f, 0.0f, 0.0f);

	reflectivity_ = 0.99f;
	shininess_    = 1.0f;
	ior_          = -1.0f;
	shader_       = -1;
	density_      = 0.0f;
	phase_g_      = 0.0f;

	memset(textures_, 0, sizeof(*textures_) * NO_TEXTURES);
	name_ = "default";
}

Material::Material(std::string& name, const Vector3& ambient, const Vector3& diffuse,
	const Vector3& specular, const Vector3& emission, const float reflectivity,
	const float shininess, const float ior, Texture** textures, const int no_textures)
{
	name_         = name;
	ambient_      = ambient;
	diffuse_      = diffuse;
	specular_     = specular;
	emission_     = emission;
	reflectivity_ = reflectivity;
	shininess_    = shininess;
	ior_          = ior;

	if (textures)
	{
		memcpy(textures_, textures, sizeof(textures) * no_textures);
	}
}

Material::~Material()
{
	for (int i = 0; i < NO_TEXTURES; ++i)
	{
		if (textures_[i])
		{
			delete[] textures_[i];
			textures_[i] = nullptr;
		}
	}
}

void Material::set_name(const char* name)
{
	name_ = std::string(name);
}

std::string Material::get_name() const
{
	return name_;
}

void Material::set_texture(const int slot, Texture* texture)
{
	textures_[slot] = texture;
}

Texture* Material::get_texture(const int slot) const
{
	return textures_[slot];
}
