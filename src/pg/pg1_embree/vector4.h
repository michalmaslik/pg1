#ifndef VECTOR4_H_
#define VECTOR4_H_

#include "structs.h"
#include "vector3.h"

/*! \struct Vector4
\brief Štvorrozmerný (4D) vektor.

Implementácia štvorsložkového reálneho vektoru podporujúca základné
matematické operácie.

\code{.cpp}
Vector4 v = Vector4(2.0f, 4.5f, 7.8f, 1.0f);
v.Normalize();
\endcode

*/
struct /*ALIGN*/ Vector4
{
public:
    union // anonymná únia
    {
        struct
        {
            float x; /*!< Prvá zložka vektoru. */
            float y; /*!< Druhá zložka vektoru. */
            float z; /*!< Tretia zložka vektoru. */
            float w; /*!< Štvrtá zložka vektoru. */
        };

        float data[4]; /*!< Pole zložiek vektoru. */
    };

    //! Výchozí konstruktor.
    /*!
    Inicializuje všetky zložky vektoru na hodnotu nula,
    \f$\mathbf{v}=\mathbf{0}\f$.
    */
    Vector4() : x(0), y(0), z(0), w(0) {}

    //! Obecný konstruktor.
    /*!
    Inicializuje zložky vektoru pod¾a zadaných hodnôt parametrov,
    \f$\mathbf{v}=(x,y,z,w)\f$.

    \param x prvá zložka vektoru.
    \param y druhá zložka vektoru.
    \param z tretia zložka vektoru.
    \param w štvrtá zložka vektoru.
    */
    Vector4(const float x, const float y, const float z, const float w) : x(x), y(y), z(z), w(w) {}

    //! Konstruktor z pole.
    /*!
    Inicializuje zložky vektoru pod¾a zadaných hodnôt po¾a,

    \param v ukazovate¾ na prvú zložku vektoru.
    */
    Vector4(const float* v);

    Vector4(const Vector3 v, const float w) : x(v.x), y(v.y), z(v.z), w(w) {}

    Vector4(const float xyzw) : x(xyzw), y(xyzw), z(xyzw), w(xyzw) {}


    //! L2-norma vektoru.
    /*!
    \return Hodnotu \f$\mathbf{||v||}=\sqrt{x^2+y^2+z^2+w^2}\f$.
    */
    float L2Norm() const;

    //! Druhá mocnina L2-normy vektoru.
    /*!
    \return Hodnotu \f$\mathbf{||v||^2}=x^2+y^2+z^2+w^2\f$.
    */
    float SqrL2Norm() const;

    //! Normalizácia vektoru.
    /*!
    Po vykonaní operácie bude ma vektor jednotkovú dåžku.
    */
    void Normalize();

    //! Skalárny súèin.
    /*!
    \return Hodnotu \f$\mathbf{u}_x \mathbf{v}_x + \mathbf{u}_y \mathbf{v}_y + \mathbf{u}_z \mathbf{v}_z + \mathbf{u}_w \mathbf{v}_w\f$.
    */
    float DotProduct(const Vector4& v) const;

    //! Index najväèšej zložky vektoru.
    /*!
    \param absolute_value index bude urèený pod¾a absolútnej hodnoty zložky

    \return Index najväèšej zložky vektoru.
    */
    char LargestComponent(const bool absolute_value = false) const;

    //! Tlaè vektoru.
    void Print();

    // --- operátory ------

    friend Vector4 operator-(const Vector4& v);

    friend Vector4 operator+(const Vector4& u, const Vector4& v);
    friend Vector4 operator-(const Vector4& u, const Vector4& v);

    friend Vector4 operator*(const Vector4& v, const float a);
    friend Vector4 operator*(const float a, const Vector4& v);
    friend Vector4 operator*(const Vector4& u, const Vector4& v);

    friend Vector4 operator/(const Vector4& v, const float a);

    friend void operator+=(Vector4& u, const Vector4& v);
    friend void operator-=(Vector4& u, const Vector4& v);
    friend void operator*=(Vector4& v, const float a);
    friend void operator/=(Vector4& v, const float a);

    friend bool operator==(const Vector4& u, const Vector4& v);
};

#endif
