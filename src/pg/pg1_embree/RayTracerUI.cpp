#include "stdafx.h"
#include "raytracer.h"
#include "RayTracerUI.h"
#include <opencv2/opencv.hpp>
#include <sstream>
#include <iomanip>

//=============================================================================
// IMPLEMENTACE ImGui PANELU
//=============================================================================

int RayTracerUI::build()
{

	ImGui::Begin("Ray Tracer Control Panel");

	// === HEADER INFO ===
	ImGui::Text("Surfaces: %d | Materials: %d", rt_.surfaces_.size(), rt_.materials_.size());
	const float currentFPS = rt_.getCurrentFPS();
	if (currentFPS > 0.0f)
		ImGui::Text("FPS: %.1f (%.3f ms/frame)", currentFPS, 1000.0f / currentFPS);
	else
		ImGui::Text("FPS: 0.0 (PAUSED)");
	ImGui::Separator();

	// ==========================================================================
	// RENDER PROGRESS
	// ==========================================================================
	{
		const int   completed = rt_.completedRows_.load(std::memory_order_relaxed);
		const int   total     = (rt_.totalRows_ > 0) ? rt_.totalRows_ : 1;
		const float progress  = std::min(static_cast<float>(completed) / static_cast<float>(total), 1.0f);
		const bool  rendering = rt_.isRendering_.load(std::memory_order_relaxed);

		// Colour the bar: green while rendering, grey when idle/complete
		if (rendering)
			ImGui::PushStyleColor(ImGuiCol_PlotHistogram, ImVec4(0.22f, 0.78f, 0.35f, 1.0f));
		else
			ImGui::PushStyleColor(ImGuiCol_PlotHistogram, ImVec4(0.45f, 0.45f, 0.45f, 1.0f));

		ImGui::ProgressBar(progress, ImVec2(-1.0f, 18.0f), "");
		ImGui::PopStyleColor();

		if (rendering && progress > 0.0f) {
			// Raw ETA: elapsed / progress gives the linear extrapolation of total time.
			const auto  now      = std::chrono::high_resolution_clock::now();
			const float elapsed  = std::chrono::duration<float>(now - rt_.renderStartTime_).count();
			const float rawEta   = std::max(0.0f, (elapsed / progress) - elapsed);

			// Exponential moving average to suppress per-frame flicker.
			// Î± = 0.15 â†’ smooth but still tracks real changes within ~7 UI frames.
			// Reset (seed directly) when a new frame starts so warm-up lag is avoided.
			constexpr float kAlpha = 0.15f;
			if (rt_.smoothedEta_ < 0.0f)
				rt_.smoothedEta_ = rawEta;           // first sample this frame
			else
				rt_.smoothedEta_ = kAlpha * rawEta + (1.0f - kAlpha) * rt_.smoothedEta_;

			ImGui::SameLine();
			if (rt_.smoothedEta_ < 60.0f)
				ImGui::Text("ETA: %.1f s", rt_.smoothedEta_);
			else
				ImGui::Text("ETA: %dm %.0fs",
				            static_cast<int>(rt_.smoothedEta_) / 60,
				            fmodf(rt_.smoothedEta_, 60.0f));

			ImGui::SameLine();
			ImGui::TextDisabled("(%.0f%%  row %d/%d)", progress * 100.0f, completed, total);
		} else if (!rendering && rt_.lastFrameDuration_ > 0.0f) {
			rt_.smoothedEta_ = -1.0f;   // reset seed so next frame starts fresh
			ImGui::SameLine();
			ImGui::TextColored(ImVec4(0.6f, 0.9f, 0.6f, 1.0f),
			                   "Done  %.2f s", rt_.lastFrameDuration_);
		} else {
			ImGui::SameLine();
			ImGui::TextDisabled("Idle");
		}
	}
	ImGui::Separator();

	// ==========================================================================
	// SCENE SELECTION  â€” hot-swaps the scene immediately on combo change
	// ==========================================================================
	if (!rt_.sceneManager_->hasScenes()) {
		ImGui::TextColored(ImVec4(1.0f, 0.45f, 0.45f, 1.0f), "No scenes loaded!");
		ImGui::TextDisabled("Place scenes.scn in the project root and restart.");
	} else {
		const auto& scenes = rt_.sceneManager_->getScenes();
		std::vector<const char*> sceneNamePtrs;
		sceneNamePtrs.reserve(scenes.size());
		for (const auto& s : scenes) sceneNamePtrs.push_back(s.name.c_str());

		int selIdx = rt_.sceneManager_->getSelectedIndex();
		ImGui::PushItemWidth(-1.0f);
		if (ImGui::Combo("##scnscene", &selIdx,
		                 sceneNamePtrs.data(), static_cast<int>(sceneNamePtrs.size()))) {
			rt_.sceneManager_->setSelectedIndex(selIdx);
			rt_.loadSceneFromDescription(rt_.sceneManager_->getSelectedScene());
		}
		ImGui::PopItemWidth();

		const SceneDescription& sel = scenes[rt_.sceneManager_->getSelectedIndex()];
		if (!sel.entities.empty()) {
			for (const auto& e : sel.entities)
				ImGui::TextDisabled("  [%s] %s", e.type.c_str(), e.value.c_str());
		}
	}

	ImGui::Separator();

	// ==========================================================================
	// PLAYBACK CONTROLS
	// ==========================================================================
	ImGui::PushStyleColor(ImGuiCol_Button,
		rt_.renderingPaused_ ? ImVec4(0.0f, 0.7f, 0.0f, 1.0f) : ImVec4(0.7f, 0.0f, 0.0f, 1.0f));
	if (ImGui::Button(rt_.renderingPaused_ ? "RESUME" : "PAUSE", ImVec2(80, 28))) {
		rt_.renderingPaused_ = !rt_.renderingPaused_;
		rt_.renderingPaused_ ? rt_.PauseRendering() : rt_.ResumeRendering();
	}
	ImGui::PopStyleColor();
	if (rt_.renderingPaused_) {
		ImGui::SameLine();
		if (ImGui::Button("RENDER FRAME", ImVec2(110, 28))) rt_.RequestSingleFrame();
	}

	ImGui::Separator();

	// ==========================================================================
	// GLOBAL RENDER FILTER  â€” determines which pipeline(s) run each frame
	// ==========================================================================
	{
		static const char* kFilterLabels[] = {
			"Mesh Only",
			"VDB Volume Only",
			"SDF Volume Only",
			"Mesh + SDF",
			"Mesh + VDB",
			"Volume (Auto)",
			"Combined (Auto)",
		};
		int filterIdx = static_cast<int>(rt_.currentFilter_);
		ImGui::PushItemWidth(-1.0f);
		if (ImGui::Combo("##filter", &filterIdx, kFilterLabels, 7))
			rt_.currentFilter_ = static_cast<GlobalRenderFilter>(filterIdx);
		ImGui::PopItemWidth();

		// Show the resolved internal mode so the user always knows what's executing.
		const char* modeNames[] = {
			"Whitted (Embree)", "SDF Volume", "VDB Volume",
			"Whitted + SDF",   "Path Trace", "PT + SDF",  "PT + VDB"
		};
		const int modeIdx = static_cast<int>(rt_.currentRenderingMode_);
		ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.6f, 0.85f, 0.6f, 1.0f));
		ImGui::Text("Active: %s", (modeIdx >= 0 && modeIdx < 7) ? modeNames[modeIdx] : "?");
		ImGui::PopStyleColor();
	}
	ImGui::Separator();

	// ==========================================================================
	// GLOBAL SETTINGS
	// ==========================================================================
	ImGui::Checkbox("VSync", &rt_.vsync_);
	ImGui::SameLine();
	ImGui::Checkbox("4x4 AA", &rt_.sampling_);
	ImGui::SameLine();
	ImGui::Checkbox("Auto Cam", &rt_.autoRotateCamera_);
	rt_.cameraMovementEnabled_ = rt_.autoRotateCamera_ && !rt_.renderingPaused_;
	ImGui::SameLine();
	ImGui::Checkbox("Mouse Cam", &rt_.mouseCameraInput_);
	ImGui::SameLine();
	ImGui::Checkbox("Debug Axes", &rt_.showDebugAxes_);

	if (rt_.mouseCameraInput_) {
		ImVec2 mousePos   = ImGui::GetMousePos();
		ImVec2 mouseDelta = ImVec2(mousePos.x - rt_.lastMousePos_.x, mousePos.y - rt_.lastMousePos_.y);
		if (ImGui::IsMouseDown(0)) {
			if (!rt_.leftMouseDragging_) { rt_.leftMouseDragging_ = true; rt_.lastMousePos_ = mousePos; }
			else rt_.handleLeftMouseDrag(mouseDelta);
		} else { rt_.leftMouseDragging_ = false; }
		if (ImGui::IsMouseDown(1)) {
			if (!rt_.rightMouseDragging_) { rt_.rightMouseDragging_ = true; rt_.lastMousePos_ = mousePos; }
			else rt_.handleRightMouseDrag(mouseDelta);
		} else { rt_.rightMouseDragging_ = false; }
		rt_.lastMousePos_ = mousePos;
	}

	// ==========================================================================
	// CAMERA  â€” directly below global inputs for fast access
	// ==========================================================================
	if (ImGui::CollapsingHeader("Camera")) {
		ImGui::Text("Pos: (%.1f, %.1f, %.1f)  Dist: %.1f", rt_.cameraX_, rt_.cameraY_, rt_.cameraZ_, rt_.cameraDistance_);
		ImGui::Text("Az: %.1f  El: %.1f", rt_.cameraAzimuth_, rt_.cameraElevation_);
		if (ImGui::SliderFloat("Distance",  &rt_.cameraDistance_,  rt_.minCameraDistance_, rt_.maxCameraDistance_)) rt_.updateCameraPosition();
		if (ImGui::SliderFloat("Azimuth",   &rt_.cameraAzimuth_,   0.0f,    360.0f)) rt_.updateCameraPosition();
		if (ImGui::SliderFloat("Elevation", &rt_.cameraElevation_, -rt_.maxElevation_, rt_.maxElevation_)) rt_.updateCameraPosition();
		if (ImGui::Button("Reset Camera", ImVec2(110, 25))) {
			rt_.cameraDistance_ = 20.0f; rt_.cameraAzimuth_ = 0.0f; rt_.cameraElevation_ = 0.0f;
			rt_.updateCameraPosition();
		}
	}

	ImGui::Separator();

	// ==========================================================================
	// ENVIRONMENT & GLOBAL LIGHTING  â€” scene-independent sun light
	// ==========================================================================
	if (ImGui::CollapsingHeader("Environment & Global Lighting")) {
		ImGui::Checkbox("Global Sun", &rt_.enableGlobalSun_);

		if (rt_.enableGlobalSun_) {
			ImGui::Indent();

			float sunColorArr[3]{ rt_.sunColor_.x, rt_.sunColor_.y, rt_.sunColor_.z };
			if (ImGui::ColorEdit3("Sun Colour", sunColorArr))
				rt_.sunColor_ = Vector3(sunColorArr[0], sunColorArr[1], sunColorArr[2]);

			ImGui::DragFloat("Sun Intensity", &rt_.sunIntensity_, 0.1f, 0.0f, 50.0f, "%.2f");

			ImGui::SliderFloat3("Sun Direction", rt_.sunDirUI_, -1.0f, 1.0f, "%.2f");

			// Show normalised direction so the user sees the actual value used.
			Vector3 disp(rt_.sunDirUI_[0], rt_.sunDirUI_[1], rt_.sunDirUI_[2]);
			const float len = disp.L2Norm();
			if (len > 1e-4f) disp /= len;
			ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.65f, 0.65f, 0.65f, 1.0f));
			ImGui::Text("Normalised: (%.2f, %.2f, %.2f)", disp.x, disp.y, disp.z);
			ImGui::PopStyleColor();

			ImGui::Unindent();
		}
	}
	ImGui::Separator();

	// ==========================================================================
	// OBJ SETTINGS  â€” mesh pipeline and path-tracing controls
	// ==========================================================================
	if (ImGui::CollapsingHeader("OBJ Settings", ImGuiTreeNodeFlags_DefaultOpen)) {
		if (ImGui::RadioButton("Whitted Ray Tracer", !rt_.objUsePathTracing_))
			rt_.objUsePathTracing_ = false;
		ImGui::SameLine();
		if (ImGui::RadioButton("Path Tracer", rt_.objUsePathTracing_))
			rt_.objUsePathTracing_ = true;

		ImGui::SliderInt("Samples/Pixel", &rt_.ptSamplesPerPixel_, 1,  64);
		ImGui::SliderInt("Max Depth",      &rt_.ptMaxDepth_,        1,  16);
		ImGui::SliderInt("RR Start Depth", &rt_.ptRRMinDepth_,      1,   8);
		ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.6f, 0.6f, 0.6f, 1.0f));
		ImGui::TextWrapped("RR p=min(max(albedo),0.95) | NEE: point lights | BRDF: cosine");
		ImGui::PopStyleColor();
	}

	// ==========================================================================
	// VDB SETTINGS  â€” ray marching controls for VDB volumes
	// ==========================================================================
	const bool hasVdbActive =
		rt_.currentRenderingMode_ == RenderingMode::VOLUMETRIC_VDB ||
		rt_.currentRenderingMode_ == RenderingMode::COMBINED_PT_VDB;
	if (ImGui::CollapsingHeader("VDB Settings", hasVdbActive ? ImGuiTreeNodeFlags_DefaultOpen : 0)) {
		ImGui::SliderFloat("Step Size",    &rt_.stepSize_,              0.01f, 2.0f,  "%.3f");
		ImGui::SliderFloat("Max Distance", &rt_.maxDistance_,           10.0f, 500.0f);
		ImGui::SliderInt  ("Max Steps",    &rt_.maxSteps_,              10,    500);
		ImGui::SliderFloat("Absorption",   &rt_.absorptionCoefficient_, 0.0f,  3.0f);
		ImGui::SliderFloat("Phase G (H-G)",&rt_.phaseG_,               -0.99f, 0.99f);
		ImGui::SliderFloat("Light Atten.", &rt_.lightAttenuationFactor_,0.0f,  3.0f);
		if (ImGui::ColorEdit3("Volumetric Albedo", rt_.volumetricAlbedo_))
			rt_.volumetricAlbedoVec_ = Vector3(rt_.volumetricAlbedo_[0], rt_.volumetricAlbedo_[1], rt_.volumetricAlbedo_[2]);
	}

	// ==========================================================================
	// SDF SETTINGS  â€” volumetric shape and noise parameters
	// ==========================================================================
	const bool hasSdfActive =
		rt_.currentRenderingMode_ == RenderingMode::VOLUMETRIC_SDF  ||
		rt_.currentRenderingMode_ == RenderingMode::COMBINED_SDF    ||
		rt_.currentRenderingMode_ == RenderingMode::COMBINED_PT_SDF;
	if (ImGui::CollapsingHeader("SDF Settings", hasSdfActive ? ImGuiTreeNodeFlags_DefaultOpen : 0)) {
		ImGui::SliderFloat("Step Size##sdf",  &rt_.stepSize_,              0.01f, 2.0f,  "%.3f");
		ImGui::SliderFloat("Max Dist##sdf",   &rt_.maxDistance_,           10.0f, 500.0f);
		ImGui::SliderInt  ("Max Steps##sdf",  &rt_.maxSteps_,              10,    500);
		ImGui::SliderFloat("Absorption##sdf", &rt_.absorptionCoefficient_, 0.0f,  3.0f);
		ImGui::SliderFloat("Phase G##sdf",    &rt_.phaseG_,               -0.99f, 0.99f);
		ImGui::SliderFloat("Light Atten.##sdf",&rt_.lightAttenuationFactor_,0.0f, 3.0f);
		ImGui::SliderFloat("Emissive R",      &rt_.emissiveLightSphereRadius_,0.1f,3.0f);
		ImGui::SliderFloat("Anim Speed",      &rt_.lightAnimSpeed_,        0.0f,  5.0f);
		if (ImGui::ColorEdit3("SDF Albedo", rt_.volumetricAlbedo_))
			rt_.volumetricAlbedoVec_ = Vector3(rt_.volumetricAlbedo_[0], rt_.volumetricAlbedo_[1], rt_.volumetricAlbedo_[2]);
		ImGui::Separator();
		ImGui::Text("Noise:");
		if (ImGui::Checkbox("Enable", &rt_.useNoise_))
			for (Shape* s : rt_.volumetricShapes_) s->setNoiseEnabled(rt_.useNoise_);
		if (ImGui::SliderFloat("Scale",    &rt_.noiseScale_,    0.0f, 1.0f))
			for (Shape* s : rt_.volumetricShapes_) s->SetNoiseScale(rt_.noiseScale_);
		if (ImGui::SliderFloat("Strength", &rt_.noiseStrength_, 0.0f, 10.0f))
			for (Shape* s : rt_.volumetricShapes_) s->SetNoiseStrength(rt_.noiseStrength_);
		if (ImGui::SliderFloat("Smooth k", &rt_.smoothFactor_,  0.0f, 3.0f))
			for (Shape* s : rt_.volumetricShapes_) s->SetSmoothFactor(rt_.smoothFactor_);
	}

	// ==========================================================================
	// LIGHTING & BACKGROUND
	// ==========================================================================
	if (ImGui::CollapsingHeader("Lighting & Background")) {
		static int lx = (int)rt_.light_.position.x, ly = (int)rt_.light_.position.y, lz = (int)rt_.light_.position.z;
		if (ImGui::SliderInt("L.X", &lx, -100, 100)) rt_.light_.position = Vector3((float)lx,(float)ly,(float)lz);
		if (ImGui::SliderInt("L.Y", &ly, -100, 100)) rt_.light_.position = Vector3((float)lx,(float)ly,(float)lz);
		if (ImGui::SliderInt("L.Z", &lz, -100, 100)) rt_.light_.position = Vector3((float)lx,(float)ly,(float)lz);
		ImGui::Checkbox("Cubemap Background", &rt_.backgroundEnabled_);
		if (ImGui::ColorEdit3("BG Color", rt_.backgroundColor_))
			rt_.backgroundColorVec_ = Vector3(rt_.backgroundColor_[0], rt_.backgroundColor_[1], rt_.backgroundColor_[2]);
	}

	// === VIDEO EXPORT ===
	if (ImGui::CollapsingHeader("Video Export")) {
		ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.3f, 0.3f, 1.0f));
		ImGui::Text("đźŽ¬ Video Creation");
		ImGui::PopStyleColor();

		ImGui::Text("Frames captured: %d", rt_.frameCount_);

		if (ImGui::Button("đź’ľ Save Video and Exit", ImVec2(200, 30))) {
			cv::VideoWriter writer("frames/output.avi",
				cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
				60,
				cv::Size(width(), rt_.height()));

			if (writer.isOpened()) {
				for (int i = 0; i < rt_.frameCount_; ++i) {
					std::stringstream filename;
					filename << "frames/frame_" << std::setw(6) << std::setfill('0') << i << ".ppm";

					cv::Mat frame = cv::imread(filename.str());
					if (!frame.empty()) {
						writer.write(frame);
					}
				}
				writer.release();
			}

			std::cout << "Video successfully created: frames/output.avi" << std::endl;
			MessageBoxA(NULL, "Video saved successfully", "Success", MB_OK);
			PostQuitMessage(0);
		}
	}

	ImGui::End();
	return 0;
}
}
