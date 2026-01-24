#pragma once
#include "simpleguidx11.h"
#include "structs.h"

class SimpleGuiDX11
{
public:
	SimpleGuiDX11(const int width, const int height);
	~SimpleGuiDX11();

	int MainLoop();

protected:
	int Init();
	int Cleanup();

	void CreateRenderTarget();
	void CleanupRenderTarget();
	HRESULT CreateDeviceD3D(HWND hWnd);
	void CleanupDeviceD3D();

	HRESULT CreateTexture();
	LRESULT WndProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam);
	static LRESULT CALLBACK s_WndProc(HWND hwnd, UINT uMsg, WPARAM wParam, LPARAM lParam);

	virtual int Ui();
	virtual Color4f GetPixel(const int x, const int y, const float t = 0.0f);
	virtual void MoveCamera() {};

	void Producer();

	int width() const;
	int height() const;

	bool vsync_{ true };
	bool rendering_{ true };
	int frameCount_{ 0 };

	// Rendering control
	std::atomic<bool> rendering_paused_{ false };
	std::atomic<bool> single_frame_requested_{ false };

	// Metódy pre kontrolu renderovania
	void PauseRendering() { rendering_paused_.store(true, std::memory_order_release); }
	void ResumeRendering() { rendering_paused_.store(false, std::memory_order_release); }
	void RequestSingleFrame() { single_frame_requested_.store(true, std::memory_order_release); }
	bool IsRenderingPaused() const { return rendering_paused_.load(std::memory_order_acquire); }
	bool IsSingleFrameRequested() const { return single_frame_requested_.load(std::memory_order_acquire); }
	void ClearSingleFrameRequest() { single_frame_requested_.store(false, std::memory_order_release); }

	// FPS tracking
	std::chrono::high_resolution_clock::time_point lastFrameTime_;
	std::chrono::high_resolution_clock::time_point fpsUpdateTime_;
	float currentFPS_{ 0.0f };
	int framesSinceLastUpdate_{ 0 };
	bool isFPSInitialized_{ false };

	// FPS calculation methods
	void UpdateFPS();
	float GetCurrentFPS() const { return rendering_paused_.load() ? 0.0f : currentFPS_; }

private:
	WNDCLASSEX wc_;
	HWND hwnd_;

	ID3D11Device* g_pd3dDevice{ nullptr };
	ID3D11DeviceContext* g_pd3dDeviceContext{ nullptr };
	IDXGISwapChain* g_pSwapChain{ nullptr };
	ID3D11RenderTargetView* g_mainRenderTargetView{ nullptr };

	ID3D11Texture2D* tex_id_{ nullptr };
	ID3D11ShaderResourceView* tex_view_{ nullptr };
	int width_{ 640 };
	int height_{ 480 };
	float* tex_data_{ nullptr }; // DXGI_FORMAT_R32G32B32A32_FLOAT
	std::mutex tex_data_lock_;

	std::atomic<bool> finish_request_{ false };
};
