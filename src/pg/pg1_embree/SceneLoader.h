#pragma once
#include <string>
#include <vector>

/// Camera descriptor parsed from the "camera" keyword.
/// fov is stored for reference; SetFovY is not currently exposed by Camera.
struct CameraDesc {
	float px{ 0 }, py{ 0 }, pz{ 0 };  ///< Camera eye position (viewFrom)
	float tx{ 0 }, ty{ 0 }, tz{ 0 };  ///< Look-at target
	float fov{ 45.0f };                ///< Vertical FoV in degrees (informational)
};

/// One light source as described in the .scn file.
/// px/py/pz is the world position for POINT lights, or the direction for DIRECTIONAL lights.
/// r/g/b may be pre-multiplied by intensity (e.g., 17.0 for ShaderToy-style bright lights).
/// Optional ORBIT animation orbits the light in the XZ plane around its base position.
struct LightDesc {
	std::string type{ "POINT" };   ///< "POINT" or "DIRECTIONAL"
	float px{ 0 }, py{ 0 }, pz{ 0 };  ///< Base position / direction vector
	float r{ 1 }, g{ 1 }, b{ 1 };     ///< RGB colour (pre-multiplied intensity)
	// --- Optional animation fields (filled only when anim line is present) ---
	std::string animType;        ///< "ORBIT" for orbital animation; empty = static
	float animSpeed{ 0.0f };     ///< Angular velocity (rad/s) for ORBIT
	float animRadius{ 0.0f };    ///< Orbit radius from the base XZ position
	float animPhase{ 0.0f };     ///< Initial phase offset (radians), default 0
};

/// A single geometry or volume asset referenced by a scene entry.
/// - type  : "MESH", "VDB", or "SDF"
/// - value : absolute file path (MESH/VDB) or shape identifier (SDF: "ShaderToy", "Sphere", …)
/// Optional animation block (appended after VALUE on the same line):
///   ORBIT    speed radius          — circular XZ-plane orbit, rad/s speed, world-unit radius
///   HOVER    speed ampl            — sinusoidal Y hover: y = base_y + sin(t*speed)*ampl
///   PINGPONG speed tX tY tZ        — smooth back-and-forth between base and target (sin oscillation)
///   TOWARDS  speed tX tY tZ        — one-shot arrival: entity starts at (tX,tY,tZ) and arrives
///                                    at its natural base position; offset = (1-t)*(target-base)
struct EntityDesc {
	std::string type;
	std::string value;
	std::string animType;          ///< "ORBIT", "HOVER", "PINGPONG", "TOWARDS", or "" (static)
	float       animSpeed{0.0f};   ///< Angular velocity (ORBIT/HOVER) or lerp speed (PINGPONG/TOWARDS)
	float       animParam1{0.0f};  ///< Orbit radius (ORBIT) or hover amplitude (HOVER)
	// World-space position used by PINGPONG and TOWARDS.
	// PINGPONG: entity oscillates between its base position and this point.
	// TOWARDS : entity starts HERE and arrives at its natural base position.
	float       animTargetX{0.0f};
	float       animTargetY{0.0f};
	float       animTargetZ{0.0f};
};

/// Parsed representation of one complete scene block from a .scn file.
struct SceneDescription {
	std::string name;               ///< Human-readable display name
	bool        hasCamera{ false }; ///< true when a "camera" line was found
	CameraDesc  camera{};               ///< Parsed camera (valid only when hasCamera == true)
	std::vector<LightDesc>  lights;     ///< Ordered list of light sources
	std::vector<EntityDesc> entities;   ///< Ordered list of geometry/volume assets
};

/// Zero-dependency plain-text scene configuration parser.
///
/// File format (scenes.scn):
///   # comment line
///   scene "Name With Spaces"
///   camera px py pz  tx ty tz  fov
///   light  TYPE  px py pz  r g b  [ANIM_TYPE speed radius [phase]]
///   entity TYPE  VALUE  [ANIM_TYPE speed param1]
///   (blank line separates scenes)
class SceneLoader {
public:
	/// Parses @p filePath and returns all well-formed SceneDescription blocks.
	/// Returns an empty vector if the file cannot be opened.
	static std::vector<SceneDescription> LoadFromFile(const std::string& filePath);
};
