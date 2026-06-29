#include "stdafx.h"
#include "ShadingUtils.h"

//=============================================================================
// Implementace pomocnych funkci pro senkovani a osvetleni
//=============================================================================

RTCRay makeSecondaryRay(const Vector3& origin, const Vector3& dir)
{
    // Inicializuj strukturu paprsku nulami
    RTCRay ray = RTCRay();

    // Nastav pocatecni bod paprsku
    ray.org_x = origin.x;
    ray.org_y = origin.y;
    ray.org_z = origin.z;

    // Maly posun od povrchu zabranuje sebeprusecikovani (shadow acne)
    ray.tnear = 0.001f;

    // Nastav smer paprsku
    ray.dir_x = dir.x;
    ray.dir_y = dir.y;
    ray.dir_z = dir.z;

    // Cas pro motion blur — nepouzivan
    ray.time = 0.0f;

    // Maximalni vzdalenost paprsku (dosahne konec sceny)
    ray.tfar = FLT_MAX;

    // Volitelne masky a identifikatory
    ray.mask  = 0;
    ray.id    = 0;
    ray.flags = 0;

    return ray;
}

float getLightAttenuation(const float distanceToLight, const float lightAttenuationFactor)
{
    // Attenuation = 1 / d^n; fyzikalni pokles intenzity se vzdalenosti
    return 1.0f / pow(distanceToLight, lightAttenuationFactor);
}

Vector3 getAmbientLight()
{
    // Konstantni ambientni osvetleni s mirnym cervenkastym nadechem.
    // Hodnota 1.2 * (0.03, 0.018, 0.018) odpovidá referencnimu ShaderToy sceni.
    return 1.2f * Vector3(0.03f, 0.018f, 0.018f);
}

Vector3 computeSdfNormal(const Vector3& p, const Shape& shape)
{
    // Numericky gradient SDF pomoci centralnych diferenci.
    // eps urcuje krok diferencovani; mensi eps = presnejsi, ale nestabilnejsi.
    constexpr float eps = 0.001f;

    // Parcialni derivace podle x, y, z
    const float dx = shape.SDF(p + Vector3(eps,  0.0f, 0.0f))
                   - shape.SDF(p - Vector3(eps,  0.0f, 0.0f));
    const float dy = shape.SDF(p + Vector3(0.0f, eps,  0.0f))
                   - shape.SDF(p - Vector3(0.0f, eps,  0.0f));
    const float dz = shape.SDF(p + Vector3(0.0f, 0.0f, eps))
                   - shape.SDF(p - Vector3(0.0f, 0.0f, eps));

    // Gradient normalizovany na jednotkovy vektor (normala povrchu)
    return Vector3(dx, dy, dz) / (2.0f * eps);
}
