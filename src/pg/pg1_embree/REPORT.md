# Project Status Audit

## 1. Core Architecture
The solution implements a hybrid rendering engine combining **Surface Ray Tracing** (via Intel Embree) and **Volumetric Ray Marching** (Custom SDF + OpenVKL).

* **Data Flow**:
    1.  **Scene Setup**: The `RayTracer` class initializes the Embree device (`InitDeviceAndScene`) and OpenVKL (`InitializeOpenVKL`). It loads OBJ models into Embree scenes and VDB grids into OpenVKL volumes.
    2.  **Ray Generation**: Rays are generated in `Camera::GenerateRay()`. The camera uses a pinhole model (`f_y_` derived from FOV) and constructs a camera-to-world matrix (`m_c_w_`) based on `viewFrom`, `viewAt`, and `up` vectors.
    3.  **Integration Loop**: The main entry point for pixel calculation is `RayTracer::GetPixel()`. It supports supersampling (4x4 jittered) and dispatches execution to `RenderPixel()` based on `currentRenderingMode_`.

## 2. Rendering Modes Deep Dive

### A. Surface Rendering (Mesh)
* **Files**: `raytracer.cpp`, `raytracer.h`, `material.cpp`
* **Algorithm**: Recursive Whitted-style Ray Tracing.
    * Primary rays are traced using `rtcIntersect1` (Embree).
    * Secondary rays (reflection/refraction) are spawned recursively in `TraceRay` (max depth hardcoded to 10).
    * Shadow rays are used for direct lighting (`IsHitPointVisible`).
* **Acceleration**: **Intel Embree** is used for BVH construction and high-performance ray-triangle intersection.
* **Physics & Materials**:
    * **Blinn-Phong Model**: Implemented in `PhongShader`. Includes ambient, diffuse (Lambert), and specular terms.
    * **Transparency**: Implemented in `TransparentShader`. Uses **Fresnel equations** (Schlick approximation fallback commented out) and Snell's law for refraction.
    * **Absorption**: Beer-Lambert law (`t_b_l`) is applied for attenuation inside transparent materials (`exp(-attenuation * length)`).
* **Lighting**: Direct lighting from a single point light (`light_`). Shadows are binary (ray-traced).

### B. SDF Rendering (Procedural)
* **Files**: `raytracer.cpp` (`VolumetricEffect`, `SurfaceEffect`), `smooth_union.h`, `shape.h`
* **Algorithm**: Two sub-modes are implemented:
    1.  **Sphere Tracing** (`SurfaceEffect`): Variable step size based on SDF distance. Renders implicit surfaces with Phong shading.
    2.  **Volumetric Ray Marching** (`VolumetricEffect`): Constant step size marching (`stepSize_`) through the volume.
* **Math**:
    * **SDFs**: Sphere, Plane, and Box primitives.
    * **CSG**: `SmoothUnion` (`k_` factor) for blending shapes.
    * **Noise**: Fractal Brownian Motion (FBM) implemented in `Noise` class (`noiseScale_`, `noiseStrength_`) to perturb SDF surfaces.
    * **Normals**: Computed via finite differences (central difference method) in `ComputeNormal`.

### C. Volumetric Rendering (VDB/OpenVKL)
* **Files**: `raytracer.cpp` (`VdbVolumeRayMarching`), `vdb_loader.cpp`
* **Algorithm**: **Ray Marching** (Delta Tracking is NOT implemented).
    * The integrator calculates the intersection with the volume bounding box (AABB).
    * It marches through the volume with a fixed `stepSize_`.
* **Physics**:
    * **Beer-Lambert Law**: Explicitly implemented. Transmittance is calculated as `exp(-absorption)`.
    * **Absorption**: Derived from density: `normalizedDensity * absorptionCoefficient_ * stepSize`.
    * **Scattering**: Isotropic scattering model. The code effectively implements `(Ambient + Light * Attenuation) * Albedo * Density`.
    * **Phase Function**: **Missing**. No anisotropic phase function (like Henyey-Greenstein) is currently implemented; light is scattered equally in all directions.
* **Libraries**:
    * **OpenVDB**: Used in `VdbLoader` to read `.vdb` files and extract leaf nodes.
    * **OpenVKL**: Used to sample density values (`vklComputeSample`) from the VDB grid on the CPU.

## 3. Configuration & Limitations

### Parameters (Adjustable in UI)
* **Volumetric**: `stepSize_` (0.001 - 1.0), `maxDistance_`, `absorptionCoefficient_` (0.0 - 3.0).
* **Lighting**: `lightAttenuationFactor_` (controls falloff $1/d^n$).
* **Procedural**: `noiseScale_`, `noiseStrength_`, `smoothFactor_`.
* **Material**: `volumetricAlbedo_`.

### Hardcoded Limitations ("Magic Numbers")
* **Lights**: The engine supports **only 1 light source** (`const int numLights = 1` in `VolumetricEffect`).
* **Recursion Depth**: Hardcoded to `10` in `TraceRay` default arguments.
* **Shadow Bias**: Hardcoded `0.001f` in `MakeSecondaryRay` and `IsHitPointVisible`.
* **VDB Grid Name**: Defaults to `"density"`, manually changeable in UI but not auto-detected.
* **SDF Steps**: Sphere tracing loop limit hardcoded to `1000`.

## 4. File Mapping

| Logical Component | Core Implementation Files | Key Classes/Functions |
| :--- | :--- | :--- |
| **Main Entry** | `pg1_embree.cpp` | `main` |
| **Integrator / Pipeline** | `raytracer.cpp` | `RayTracer::GetPixel`, `RenderPixel` |
| **Surface Tracing** | `raytracer.cpp` | `TraceRay`, `PhongShader`, `TransparentShader` |
| **SDF Logic** | `raytracer.cpp`, `shape.h`, `smooth_union.h` | `VolumetricEffect`, `SurfaceEffect`, `ComputeNormal` |
| **VDB Logic** | `vdb_loader.cpp`, `raytracer.cpp` | `VdbLoader`, `VdbVolumeRayMarching` |
| **Camera** | `camera.cpp` | `GenerateRay`, `Update_m_c_w_` |
| **Embree Geometry** | `raytracer.cpp` | `LoadModel`, `InitDeviceAndScene` |