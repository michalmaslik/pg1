#include "stdafx.h"
#define _SILENCE_EXPERIMENTAL_FILESYSTEM_DEPRECATION_WARNING
#include "simpleguidx11.h"
#include <fstream>
#include <sstream>
#include <iomanip>
#include <experimental/filesystem>
#include <opencv2/opencv.hpp>


SimpleGuiDX11::SimpleGuiDX11(const int width, const int height)
{
	width_ = width;
	height_ = height;

	Init();
}

int SimpleGuiDX11::Init()
{
	// Create application window
	wc_ = { sizeof(WNDCLASSEX), CS_CLASSDC, s_WndProc, 0L, 0L,
		GetModuleHandle(NULL), NULL, NULL, NULL, NULL, _T("ImGui Example"), NULL };
	RegisterClassEx(&wc_);
	hwnd_ = CreateWindow(_T("ImGui Example"), _T("PG1 Ray Tracer"),
		WS_OVERLAPPEDWINDOW, 100, 100, 1280, 800, NULL, NULL, wc_.hInstance, this);

	// Initialize Direct3D
	if (CreateDeviceD3D(hwnd_) < 0)
	{
		CleanupDeviceD3D();
		UnregisterClass(_T("ImGui Example"), wc_.hInstance);
		return S_OK;
	}

	// Show the window
	ShowWindow(hwnd_, SW_SHOWDEFAULT);
	UpdateWindow(hwnd_);

	// Setup Dear ImGui binding
	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	//ImGuiIO & io = ImGui::GetIO(); ( void )io;
	//io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;  // Enable Keyboard Controls

	// Initialize helper Platform and Renderer bindings
	// (here we are using imgui_impl_win32 and imgui_impl_dx11)
	ImGui_ImplWin32_Init(hwnd_);
	ImGui_ImplDX11_Init(g_pd3dDevice, g_pd3dDeviceContext);

	// Setup style
	ImGui::StyleColorsDark();
	//ImGui::StyleColorsClassic();

	tex_data_ = new float[width_ * height_ * 4 * sizeof(float)];
	CreateTexture();

	return 0;
}

SimpleGuiDX11::~SimpleGuiDX11()
{
	Cleanup();

	delete[] tex_data_;
	tex_data_ = nullptr;
}

int SimpleGuiDX11::Cleanup()
{
	ImGui_ImplDX11_Shutdown();
	ImGui_ImplWin32_Shutdown();
	ImGui::DestroyContext();

	CleanupDeviceD3D();
	DestroyWindow(hwnd_);
	UnregisterClass(_T("ImGui Example"), wc_.hInstance);

	// Vytvoríme video z uložených snímkov
	std::string ffmpeg_cmd = "ffmpeg -framerate 60 -i frames/frame_%06d.ppm -c:v libx264 -crf 0 -preset veryslow -pix_fmt yuv420p frames/output.mp4";
	system(ffmpeg_cmd.c_str());

	return 0;
}

// abstract method reimplemented in the descendant
int SimpleGuiDX11::Ui()
{
	return 0;
}

Color4f SimpleGuiDX11::GetPixel(const int x, const int y, const float t)
{
	return Color4f{ 1.0f, 0.0f, 1.0f, 1.0f };
}

void SimpleGuiDX11::Producer()
{
	float* local_data = new float[width_ * height_ * 4];

	float t = 0.0f; // time
	auto t0 = std::chrono::high_resolution_clock::now();

	// Initialize FPS timing
	lastFrameTime_ = std::chrono::high_resolution_clock::now();
	fpsUpdateTime_ = lastFrameTime_;
	isFPSInitialized_ = true;

	while (!finish_request_.load(std::memory_order_acquire))
	{
		// Check if rendering is paused
		bool should_render = !IsRenderingPaused();

		// Check for single frame request when paused
		if (IsRenderingPaused() && IsSingleFrameRequested()) {
			should_render = true;
			ClearSingleFrameRequest();
		}

		if (should_render) {
			auto t1 = std::chrono::high_resolution_clock::now();
			std::chrono::duration<float> dt = t1 - t0;
			t += dt.count();
			t0 = t1;

			// Update FPS calculation
			UpdateFPS();

			// Only move camera if not paused
			if (!IsRenderingPaused()) {
				MoveCamera();
			}

			// compute rendering
#pragma omp parallel for
			for (int y = 0; y < height_; ++y)
			{
				for (int x = 0; x < width_; ++x)
				{
					const Color4f pixel = GetPixel(x, y, t);
					const int offset = (y * width_ + x) * 4;

					local_data[offset] = pixel.r;
					local_data[offset + 1] = pixel.g;
					local_data[offset + 2] = pixel.b;
					local_data[offset + 3] = pixel.a;
				}
			}

			// write rendering results
			{
				std::lock_guard<std::mutex> lock(tex_data_lock_);
				memcpy(tex_data_, local_data, width_ * height_ * 4 * sizeof(float));
			} // lock release

			// Save frame only when actively rendering
			if (!IsRenderingPaused()) {
				// Frame saving code (existing frame saving logic)
				std::experimental::filesystem::create_directory("frames");

				std::stringstream filename;
				filename << "frames/frame_" << std::setw(6) << std::setfill('0') << frameCount_ << ".ppm";

				std::ofstream file(filename.str(), std::ios::binary);
				if (file.is_open()) {
					file << "P6\n" << width_ << " " << height_ << "\n255\n";

					for (int y = 0; y < height_; ++y) {
						for (int x = 0; x < width_; ++x) {
							const int offset = (y * width_ + x) * 4;
							unsigned char r = static_cast<unsigned char>(local_data[offset] * 255.0f);
							unsigned char g = static_cast<unsigned char>(local_data[offset + 1] * 255.0f);
							unsigned char b = static_cast<unsigned char>(local_data[offset + 2] * 255.0f);
							file.write(reinterpret_cast<const char*>(&r), 1);
							file.write(reinterpret_cast<const char*>(&g), 1);
							file.write(reinterpret_cast<const char*>(&b), 1);
						}
					}
				}
				frameCount_++;
			}
		}
		else {
			// When paused, sleep briefly to avoid busy waiting
			std::this_thread::sleep_for(std::chrono::milliseconds(16)); // ~60 FPS check rate
		}
	}

	delete[] local_data;
}

void SimpleGuiDX11::UpdateFPS() {
	if (!isFPSInitialized_) return;

	auto currentTime = std::chrono::high_resolution_clock::now();
	framesSinceLastUpdate_++;

	// Update FPS every 0.5 seconds
	std::chrono::duration<float> timeSinceUpdate = currentTime - fpsUpdateTime_;
	if (timeSinceUpdate.count() >= 0.5f) {
		currentFPS_ = framesSinceLastUpdate_ / timeSinceUpdate.count();
		framesSinceLastUpdate_ = 0;
		fpsUpdateTime_ = currentTime;
	}

	lastFrameTime_ = currentTime;
}
int SimpleGuiDX11::width() const
{
	return width_;
}

int SimpleGuiDX11::height() const
{
	return height_;
}

int SimpleGuiDX11::MainLoop()
{
	// start image producing threads
	std::thread producer_thread(&SimpleGuiDX11::Producer, this);
	BOOL r = SetThreadPriority(producer_thread.native_handle(), THREAD_PRIORITY_BELOW_NORMAL);

	// and enter message loop
	MSG msg;
	ZeroMemory(&msg, sizeof(msg));
	while (msg.message != WM_QUIT)
	{
		// Poll and handle messages (inputs, window resize, etc.)
		// You can read the io.WantCaptureMouse, io.WantCaptureKeyboard flags to tell if dear imgui wants to use your inputs.
		// - When io.WantCaptureMouse is true, do not dispatch mouse input data to your main application.
		// - When io.WantCaptureKeyboard is true, do not dispatch keyboard input data to your main application.
		// Generally you may always pass all inputs to dear imgui, and hide them from your application based on those two flags.
		if (PeekMessage(&msg, NULL, 0U, 0U, PM_REMOVE))
		{
			TranslateMessage(&msg);
			DispatchMessage(&msg);
			continue;
		}

		// Start the Dear ImGui frame
		ImGui_ImplDX11_NewFrame();
		ImGui_ImplWin32_NewFrame();
		ImGui::NewFrame();

		Ui();

		{
			D3D11_MAPPED_SUBRESOURCE mapped;
			ZeroMemory(&mapped, sizeof(mapped));
			HRESULT hr = g_pd3dDeviceContext->Map(tex_id_, 0, D3D11_MAP_WRITE_DISCARD, 0, &mapped); // D3D11_MAP_WRITE, D3D11_MAP_WRITE_DISCARD

			{
				std::lock_guard<std::mutex> lock(tex_data_lock_);
				memcpy(mapped.pData, tex_data_, mapped.RowPitch * height_);
			}

			g_pd3dDeviceContext->Unmap(tex_id_, 0);
		}

		// Úplne fixované Image okno v ľavom hornom rohu
		ImGui::SetNextWindowPos(ImVec2(0, 0), ImGuiCond_Always); // Vždy v ľavom hornom rohu
		ImGui::SetNextWindowSize(ImVec2(float(width_ + 16), float(height_ + 35)), ImGuiCond_Always); // Vždy rovnaká veľkosť

		ImGui::Begin("Image", 0,
			ImGuiWindowFlags_NoResize |
			ImGuiWindowFlags_NoMove |
			ImGuiWindowFlags_NoTitleBar |       // Odstráni title bar
			ImGuiWindowFlags_NoScrollbar |
			ImGuiWindowFlags_NoScrollWithMouse |
			ImGuiWindowFlags_NoCollapse
		);

		ImGui::Image(ImTextureID(tex_view_), ImVec2(float(width_), float(height_)));
		ImGui::End();

		//ImGui_ImplDX11_RenderDrawData()
		// Rendering
		ImGui::Render();
		g_pd3dDeviceContext->OMSetRenderTargets(1, &g_mainRenderTargetView, NULL);
		const FLOAT clear_color[4] = { 0.45f, 0.55f, 0.60f, 1.00f };
		g_pd3dDeviceContext->ClearRenderTargetView(g_mainRenderTargetView, (float*)&clear_color);
		ImGui_ImplDX11_RenderDrawData(ImGui::GetDrawData());

		g_pSwapChain->Present(((vsync_) ? 1 : 0), 0); // present with or without vsync
	}

	finish_request_.store(true, std::memory_order_release);
	producer_thread.join();

	return 0;
}

void SimpleGuiDX11::CreateRenderTarget()
{
	ID3D11Texture2D* pBackBuffer = nullptr;
	g_pSwapChain->GetBuffer(0, __uuidof(ID3D11Texture2D), (LPVOID*)&pBackBuffer);
	g_pd3dDevice->CreateRenderTargetView(pBackBuffer, nullptr, &g_mainRenderTargetView);
	pBackBuffer->Release();
}

void SimpleGuiDX11::CleanupRenderTarget()
{
	if (g_mainRenderTargetView)
	{
		g_mainRenderTargetView->Release();
		g_mainRenderTargetView = nullptr;
	}
}

HRESULT SimpleGuiDX11::CreateDeviceD3D(HWND hWnd)
{
	// Setup swap chain
	DXGI_SWAP_CHAIN_DESC sd;
	ZeroMemory(&sd, sizeof(sd));
	sd.BufferCount = 2;
	sd.BufferDesc.Width = 0;
	sd.BufferDesc.Height = 0;
	sd.BufferDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
	sd.BufferDesc.RefreshRate.Numerator = 60;
	sd.BufferDesc.RefreshRate.Denominator = 1;
	sd.Flags = DXGI_SWAP_CHAIN_FLAG_ALLOW_MODE_SWITCH;
	sd.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
	sd.OutputWindow = hWnd;
	sd.SampleDesc.Count = 1;
	sd.SampleDesc.Quality = 0;
	sd.Windowed = TRUE;
	sd.SwapEffect = DXGI_SWAP_EFFECT_DISCARD;

	UINT createDeviceFlags = 0;
	//createDeviceFlags |= D3D11_CREATE_DEVICE_DEBUG;
	D3D_FEATURE_LEVEL featureLevel;
	const D3D_FEATURE_LEVEL featureLevelArray[2] = { D3D_FEATURE_LEVEL_11_0, D3D_FEATURE_LEVEL_10_0 };
	if (D3D11CreateDeviceAndSwapChain(nullptr, D3D_DRIVER_TYPE_HARDWARE,
		nullptr, createDeviceFlags, featureLevelArray, 2, D3D11_SDK_VERSION,
		&sd, &g_pSwapChain, &g_pd3dDevice, &featureLevel, &g_pd3dDeviceContext) != S_OK)
	{
		return E_FAIL;
	}

	CreateRenderTarget();

	return S_OK;
}

void SimpleGuiDX11::CleanupDeviceD3D()
{
	CleanupRenderTarget();

	if (g_pSwapChain)
	{
		g_pSwapChain->Release();
		g_pSwapChain = nullptr;
	}

	if (g_pd3dDeviceContext)
	{
		g_pd3dDeviceContext->Release();
		g_pd3dDeviceContext = nullptr;
	}

	if (g_pd3dDevice)
	{
		g_pd3dDevice->Release();
		g_pd3dDevice = nullptr;
	}
}

HRESULT SimpleGuiDX11::CreateTexture()
{
	if (tex_id_ == nullptr)
	{
		// set up texture description
		D3D11_TEXTURE2D_DESC desc;
		ZeroMemory(&desc, sizeof(desc));
		desc.Width = width_;
		desc.Height = height_;
		desc.MipLevels = 1;
		desc.ArraySize = 1;
		desc.Format = DXGI_FORMAT_R32G32B32A32_FLOAT;
		desc.SampleDesc.Count = 1;
		desc.SampleDesc.Quality = 0;
		desc.Usage = D3D11_USAGE_DYNAMIC;
		desc.BindFlags = D3D11_BIND_SHADER_RESOURCE;
		desc.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
		desc.MiscFlags = 0;

		// set up initial data description for the texture
		D3D11_SUBRESOURCE_DATA initData;
		ZeroMemory(&initData, sizeof(initData));
		initData.pSysMem = (void*)tex_data_;
		initData.SysMemPitch = width_ * (4 * sizeof(float));
		initData.SysMemSlicePitch = height_ * initData.SysMemPitch;

		// create the texture
		HRESULT hr = g_pd3dDevice->CreateTexture2D(&desc, &initData, &tex_id_);

		//https://github.com/ocornut/imgui/issues/1877
		//https://github.com/ocornut/imgui/issues/1265

		// create a view of the texture
		D3D11_SHADER_RESOURCE_VIEW_DESC srvDesc;
		ZeroMemory(&srvDesc, sizeof(srvDesc));
		srvDesc.ViewDimension = D3D11_SRV_DIMENSION_TEXTURE2D,
			srvDesc.Texture2D.MostDetailedMip = 0;
		srvDesc.Texture2D.MipLevels = desc.MipLevels;
		srvDesc.Format = desc.Format;
		hr = g_pd3dDevice->CreateShaderResourceView(tex_id_, &srvDesc, &tex_view_);

		return hr;
	}

	return S_OK;
}

extern LRESULT ImGui_ImplWin32_WndProcHandler(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam);

LRESULT SimpleGuiDX11::WndProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam)
{
	if (ImGui_ImplWin32_WndProcHandler(hWnd, msg, wParam, lParam))
		return true;

	switch (msg)
	{
	case WM_SIZE:
		if (g_pd3dDevice != NULL && wParam != SIZE_MINIMIZED)
		{
			ImGui_ImplDX11_InvalidateDeviceObjects();
			CleanupRenderTarget();
			g_pSwapChain->ResizeBuffers(0, (UINT)LOWORD(lParam), (UINT)HIWORD(lParam), DXGI_FORMAT_UNKNOWN, 0);
			CreateRenderTarget();
			ImGui_ImplDX11_CreateDeviceObjects();
		}
		return 0;
	case WM_SYSCOMMAND:
		if ((wParam & 0xfff0) == SC_KEYMENU) // Disable ALT application menu
			return 0;
		break;
	case WM_DESTROY:
		PostQuitMessage(0);
		return 0;
	}

	return DefWindowProc(hWnd, msg, wParam, lParam);
}

// https://blogs.msdn.microsoft.com/oldnewthing/20140203-00/?p=1893/
LRESULT CALLBACK SimpleGuiDX11::s_WndProc(
	HWND hwnd, UINT uMsg, WPARAM wParam, LPARAM lParam)
{
	SimpleGuiDX11* pThis = nullptr; // our "this" pointer will go here

	if (uMsg == WM_NCCREATE)
	{
		// Recover the "this" pointer which was passed as a parameter
		// to CreateWindow(Ex).
		LPCREATESTRUCT lpcs = reinterpret_cast<LPCREATESTRUCT>(lParam);
		pThis = static_cast<SimpleGuiDX11*>(lpcs->lpCreateParams);
		// Put the value in a safe place for future use
		SetWindowLongPtr(hwnd, GWLP_USERDATA,
			reinterpret_cast<LONG_PTR>(pThis));
	}
	else
	{
		// Recover the "this" pointer from where our WM_NCCREATE handler
		// stashed it.
		pThis = reinterpret_cast<SimpleGuiDX11*>(
			GetWindowLongPtr(hwnd, GWLP_USERDATA));
	}

	if (pThis)
	{
		// Now that we have recovered our "this" pointer, let the
		// member function finish the job.
		return pThis->WndProc(hwnd, uMsg, wParam, lParam);
	}

	// We don't know what our "this" pointer is, so just do the default
	// thing. Hopefully, we didn't need to customize the behavior yet.
	return DefWindowProc(hwnd, uMsg, wParam, lParam);
}
