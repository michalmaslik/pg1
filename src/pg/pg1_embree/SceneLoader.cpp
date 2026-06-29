#include "stdafx.h"
#include "SceneLoader.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <cctype>

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

static std::string TrimWhitespace(const std::string& s)
{
	const auto first = s.find_first_not_of(" \t\r\n");
	if (first == std::string::npos) return {};
	const auto last = s.find_last_not_of(" \t\r\n");
	return s.substr(first, last - first + 1);
}

/// Removes surrounding double-quotes from a string, if present.
static std::string Unquote(const std::string& s)
{
	if (s.size() >= 2 && s.front() == '"' && s.back() == '"')
		return s.substr(1, s.size() - 2);
	return s;
}

/// Commits @p current to @p scenes if it has a valid name.
static void CommitScene(std::vector<SceneDescription>& scenes, SceneDescription& current, bool& hasScene)
{
	if (hasScene && !current.name.empty()) {
		scenes.push_back(current);
		current = SceneDescription{};
		hasScene = false;
	}
}

// ---------------------------------------------------------------------------
// SceneLoader::LoadFromFile
// ---------------------------------------------------------------------------

std::vector<SceneDescription> SceneLoader::LoadFromFile(const std::string& filePath)
{
	std::vector<SceneDescription> scenes;
	std::ifstream file(filePath);

	if (!file.is_open()) {
		std::cerr << "[SceneLoader] Cannot open file: " << filePath << "\n";
		return scenes;
	}

	SceneDescription current;
	bool hasScene = false;
	std::string line;

	while (std::getline(file, line)) {
		const std::string trimmed = TrimWhitespace(line);

		// Blank line — commit any in-progress scene
		if (trimmed.empty()) {
			CommitScene(scenes, current, hasScene);
			continue;
		}

		// Comment line
		if (trimmed[0] == '#') continue;

		std::istringstream iss(trimmed);
		std::string key;
		iss >> key;

		if (key == "scene") {
			// Commit previous block before starting a new one
			CommitScene(scenes, current, hasScene);

			// Read the rest of the line as the scene name (may be quoted)
			std::string rest;
			std::getline(iss, rest);
			current.name = Unquote(TrimWhitespace(rest));
			hasScene = true;
		}
		else if (key == "camera") {
			// Syntax: camera px py pz  tx ty tz  fov
			CameraDesc c;
			if (iss >> c.px >> c.py >> c.pz >> c.tx >> c.ty >> c.tz >> c.fov) {
				current.camera    = c;
				current.hasCamera = true;
			} else {
				std::cerr << "[SceneLoader] Malformed 'camera' line (expected 7 floats)\n";
			}
		}
		else if (key == "light") {
			// Syntax: light TYPE px py pz r g b [ANIM_TYPE speed radius [phase]]
			// For POINT lights px/py/pz is the world position.
			// For DIRECTIONAL lights px/py/pz is the direction vector.
			// Animation params (ANIM_TYPE speed radius [phase]) are optional.
			LightDesc l;
			if (iss >> l.type >> l.px >> l.py >> l.pz >> l.r >> l.g >> l.b) {
				// Try to read the optional animation block
				std::string animToken;
				if (iss >> animToken) {
					l.animType = animToken;
					if (iss >> l.animSpeed >> l.animRadius) {
						// Phase is optional — silently defaults to 0.0 if absent
						if (!(iss >> l.animPhase))
							l.animPhase = 0.0f;
					} else {
						std::cerr << "[SceneLoader] Malformed anim block for light '"
						          << l.type << "' (expected speed + radius)\n";
						l.animType.clear();
					}
				}
				current.lights.push_back(l);
			} else {
				std::cerr << "[SceneLoader] Malformed 'light' line (expected TYPE + 6 floats)\n";
			}
		}
		else if (key == "entity") {
			EntityDesc e;
			iss >> e.type;

			// Collect all remaining tokens; ORBIT/HOVER may appear after the file path
			std::vector<std::string> tokens;
			std::string tok;
			while (iss >> tok) tokens.push_back(tok);

			// Find the first animation keyword (always uppercase)
			int animIdx = -1;
			for (int i = 0; i < static_cast<int>(tokens.size()); ++i) {
				if (tokens[i] == "ORBIT"    || tokens[i] == "HOVER" ||
				    tokens[i] == "PINGPONG" || tokens[i] == "TOWARDS") {
					animIdx = i;
					break;
				}
			}

			if (animIdx < 0) {
				// No animation — join all tokens as the value (handles space-padded paths)
				for (int i = 0; i < static_cast<int>(tokens.size()); ++i) {
					if (i > 0) e.value += ' ';
					e.value += tokens[i];
				}
			} else {
				// Value = tokens before the animation keyword
				for (int i = 0; i < animIdx; ++i) {
					if (i > 0) e.value += ' ';
					e.value += tokens[i];
				}
				e.animType = tokens[animIdx];
				const bool isTwoParam  = (e.animType == "ORBIT" || e.animType == "HOVER");
				const bool isFourParam = (e.animType == "PINGPONG" || e.animType == "TOWARDS");

				if (isTwoParam) {
					// ORBIT speed radius  |  HOVER speed ampl
					if (animIdx + 2 < static_cast<int>(tokens.size())) {
						try {
							e.animSpeed  = std::stof(tokens[animIdx + 1]);
							e.animParam1 = std::stof(tokens[animIdx + 2]);
						} catch (...) {
							std::cerr << "[SceneLoader] Malformed ORBIT/HOVER block for entity '" << e.type << "'\n";
							e.animType.clear();
						}
					} else {
						std::cerr << "[SceneLoader] " << e.animType << " needs speed + param1 for entity '" << e.type << "'\n";
						e.animType.clear();
					}
				} else if (isFourParam) {
					// PINGPONG speed tX tY tZ  |  TOWARDS speed tX tY tZ
					if (animIdx + 4 < static_cast<int>(tokens.size())) {
						try {
							e.animSpeed   = std::stof(tokens[animIdx + 1]);
							e.animTargetX = std::stof(tokens[animIdx + 2]);
							e.animTargetY = std::stof(tokens[animIdx + 3]);
							e.animTargetZ = std::stof(tokens[animIdx + 4]);
						} catch (...) {
							std::cerr << "[SceneLoader] Malformed " << e.animType << " block for entity '" << e.type << "'\n";
							e.animType.clear();
						}
					} else {
						std::cerr << "[SceneLoader] " << e.animType << " needs speed + targetX + targetY + targetZ for entity '" << e.type << "'\n";
						e.animType.clear();
					}
				}
			}
			current.entities.push_back(e);
		}
		// Unknown keys are silently ignored for forward compatibility
	}

	// Commit the final scene block (no trailing blank line required)
	CommitScene(scenes, current, hasScene);

	std::cout << "[SceneLoader] Loaded " << scenes.size()
	          << " scene(s) from: " << filePath << "\n";
	return scenes;
}
