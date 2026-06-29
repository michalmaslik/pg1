#pragma once

#include "stdafx.h"
#include "vector3.h"
#include "shape.h"
#include <embree3/rtcore.h>

/**
 * @file  ShadingUtils.h
 * @brief Pomocne funkce pro senkovani a vyzarovani pouzivane ve vsech rendererech.
 *
 * Tento soubor obsahuje bezne volne funkce (bez tridy), ktere jsou sdileny
 * mezi jednotlivymi renderovacimi moduly (SdfRenderer, VdbRenderer, PathTracer,
 * raytracer.cpp). Jsou oddeleny od tridy RayTracer, aby se zamezilo
 * duplikaci kodu pri extrakci novych trid.
 */

/**
 * @brief Vytvori sekundarni paprsek (odraz, lom, sekundarni paprsky sledovani cest).
 *
 * Nastavi origin, smer a hranicni hodnoty paprsku tak, aby se zamezilo
 * sebeprusecikovani (tnear = 0.001f).
 *
 * @param origin  Pocatecni bod paprsku ve svetovych souradnicich.
 * @param dir     Smer paprsku (mel by byt normalizovany).
 * @return        Inicializovana struktura RTCRay.
 */
RTCRay makeSecondaryRay(const Vector3& origin, const Vector3& dir);

/**
 * @brief Vypocita utlum svetla na zaklade vzdalenosti (inverze mocniny).
 *
 * Pouziva vzorec: attenuation = 1 / d^n, kde n je lightAttenuationFactor.
 *
 * @param distanceToLight        Vzdalenost od svetla ke vzorkovemu bodu.
 * @param lightAttenuationFactor Exponent n pro inverzi mocniny.
 * @return                       Hodnota utlumu (0.0 - 1.0+).
 */
float getLightAttenuation(float distanceToLight, float lightAttenuationFactor);

/**
 * @brief Vraci konstantni ambientni osvetleni pouzivane v cele scene.
 *
 * Hodnota je staticky definovana s mirnym cervenkastym nádechem pro
 * realistictejsi vzhled SDF mraku.
 *
 * @return  Vektor barvy ambientniho osvetleni (R, G, B).
 */
Vector3 getAmbientLight();

/**
 * @brief Vypocita normalu SDF povrchu pomoci centralnych diferenci.
 *
 * Numericky aproximuje gradient SDF v bode p pomoci centralnych diferenci
 * s krokem eps. Vraci normalizovany gradient (= normala povrchu).
 *
 * @param p      Bod na povrchu SDF telesa.
 * @param shape  Teleso, jehoz gradient se pocita.
 * @return       Normalizovany normalovy vektor v bode p.
 */
Vector3 computeSdfNormal(const Vector3& p, const Shape& shape);
