#include "graphics/D3D11Renderer.hpp"
#include "capture/DesktopCpuFrame.hpp"

#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>

#include <d3d11_1.h>
#include <d3dcompiler.h>
#include <dxgi1_3.h>
#include <wrl/client.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <utility>
#include <vector>

namespace xreal::graphics
{
namespace
{

using Microsoft::WRL::ComPtr;

[[nodiscard]] capture::DesktopFit captureFit(
    rendering::PanelFitMode value) noexcept
{
    switch (value)
    {
    case rendering::PanelFitMode::contain: return capture::DesktopFit::contain;
    case rendering::PanelFitMode::cover: return capture::DesktopFit::cover;
    case rendering::PanelFitMode::stretch: return capture::DesktopFit::stretch;
    }
    return capture::DesktopFit::contain;
}

struct Vertex
{
    float x{};
    float y{};
    float z{};
    float red{};
    float green{};
    float blue{};
    float alpha{1.0F};
    float textureU{};
    float textureV{};
};

[[nodiscard]] std::string hresultText(HRESULT value)
{
    std::ostringstream output;
    output << "HRESULT 0x" << std::hex << std::uppercase
           << static_cast<unsigned long>(value);
    LPWSTR message = nullptr;
    const DWORD length = FormatMessageW(
        FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM
            | FORMAT_MESSAGE_IGNORE_INSERTS,
        nullptr, static_cast<DWORD>(value), 0,
        reinterpret_cast<LPWSTR>(&message), 0, nullptr);
    if (length > 0U && message != nullptr)
    {
        std::wstring wideMessage(message, length);
        LocalFree(message);
        const int size = WideCharToMultiByte(CP_UTF8, 0, wideMessage.data(),
            static_cast<int>(wideMessage.size()), nullptr, 0, nullptr, nullptr);
        std::string converted(static_cast<std::size_t>(size), '\0');
        (void)WideCharToMultiByte(CP_UTF8, 0, wideMessage.data(),
            static_cast<int>(wideMessage.size()), converted.data(), size, nullptr, nullptr);
        output << " (" << converted << ')';
    }
    return output.str();
}

[[nodiscard]] rendering::DesktopTextureDescriptorContract textureContract(
    const D3D11_TEXTURE2D_DESC& description) noexcept
{
    return {
        description.Width,
        description.Height,
        description.MipLevels,
        description.ArraySize,
        static_cast<std::uint32_t>(description.Format),
        description.SampleDesc.Count,
        description.SampleDesc.Quality,
        static_cast<std::uint32_t>(description.Usage),
        description.BindFlags,
        description.CPUAccessFlags,
        description.MiscFlags,
    };
}

[[nodiscard]] rendering::DesktopShaderResourceDescriptorContract shaderResourceContract(
    const D3D11_SHADER_RESOURCE_VIEW_DESC& description) noexcept
{
    return {
        static_cast<std::uint32_t>(description.Format),
        static_cast<std::uint32_t>(description.ViewDimension),
        description.Texture2D.MostDetailedMip,
        description.Texture2D.MipLevels,
    };
}

[[nodiscard]] std::string bgraPixelText(const capture::BgraPixel& pixel)
{
    std::ostringstream output;
    output << std::hex << std::uppercase << std::setfill('0')
           << std::setw(2) << static_cast<unsigned int>(pixel.blue) << ' '
           << std::setw(2) << static_cast<unsigned int>(pixel.green) << ' '
           << std::setw(2) << static_cast<unsigned int>(pixel.red) << ' '
           << std::setw(2) << static_cast<unsigned int>(pixel.alpha);
    return output.str();
}

[[nodiscard]] std::filesystem::path executableDirectory()
{
    std::wstring buffer(32768U, L'\0');
    const DWORD length = GetModuleFileNameW(nullptr, buffer.data(),
        static_cast<DWORD>(buffer.size()));
    buffer.resize(length);
    return std::filesystem::path(buffer).parent_path();
}

[[nodiscard]] std::string featureLevelText(D3D_FEATURE_LEVEL level)
{
    switch (level)
    {
    case D3D_FEATURE_LEVEL_11_1: return "11.1";
    case D3D_FEATURE_LEVEL_11_0: return "11.0";
    case D3D_FEATURE_LEVEL_10_1: return "10.1";
    case D3D_FEATURE_LEVEL_10_0: return "10.0";
    default: return "unknown";
    }
}

[[nodiscard]] std::string adapterName(const DXGI_ADAPTER_DESC& description)
{
    const int size = WideCharToMultiByte(CP_UTF8, 0, description.Description, -1,
        nullptr, 0, nullptr, nullptr);
    if (size <= 1)
    {
        return "unknown";
    }
    std::string result(static_cast<std::size_t>(size), '\0');
    (void)WideCharToMultiByte(CP_UTF8, 0, description.Description, -1,
        result.data(), size, nullptr, nullptr);
    result.pop_back();
    return result;
}

} // namespace

class D3D11Renderer::Implementation
{
public:
    ~Implementation()
    {
        if (swapChain_ && information_.exclusiveFullscreen)
        {
            (void)swapChain_->SetFullscreenState(FALSE, nullptr);
        }
    }

    [[nodiscard]] bool initialize(void* nativeWindow, const D3D11RendererConfig& config)
    {
        if (nativeWindow == nullptr || config.width == 0U || config.height == 0U)
        {
            error_ = "D3D11 initialization requires a valid window and dimensions.";
            return false;
        }
        window_ = static_cast<HWND>(nativeWindow);
        panelDistance_ = config.panelDistance;
        panelWidth_ = config.panelWidth;
        panelHeight_ = config.panelHeight;
        requestedAdapterLuid_ = config.selectedAdapterLuid;
        requestedOutputDeviceName_ = config.selectedOutputDeviceName;
        fullscreen_ = config.fullscreen;
        desktopShaderDebugMode_ = config.desktopShaderDebugMode;
        desktopDebugOpaqueBase_ = config.desktopDebugOpaqueBase;
        desktopDebugReadbackUploadPath_ = config.desktopDebugReadbackUploadPath;
        desktopDebugRenderTargetPath_ = config.desktopDebugRenderTargetPath;
        information_.selectedAdapterRequested = requestedAdapterLuid_.has_value();

        if (!createDevice(false) && (!config.allowWarpFallback || !createDevice(true)))
        {
            if (!config.allowWarpFallback)
            {
                error_ += " WARP fallback was not enabled.";
            }
            return false;
        }
        if (!selectOutput()
            || !createSwapChain(config.width, config.height)
            || !createRenderTargets(config.width, config.height)
            || !createPipeline() || !createScene())
        {
            return false;
        }
        // Core D3D resources tracked explicitly: swap chain, render/depth
        // targets, three shaders, input layout, constant/VB/IB/line buffers,
        // rasterizer, two blend states, depth state and two samplers.
        sceneStatistics_.resourcesCreatedAtStartup = 18U;
        return true;
    }

    [[nodiscard]] bool createDevice(bool warp)
    {
        device_.Reset();
        context_.Reset();
        constexpr std::array featureLevels{
            D3D_FEATURE_LEVEL_11_1,
            D3D_FEATURE_LEVEL_11_0,
            D3D_FEATURE_LEVEL_10_1,
            D3D_FEATURE_LEVEL_10_0,
        };
        UINT flags = D3D11_CREATE_DEVICE_BGRA_SUPPORT;
#ifndef NDEBUG
        flags |= D3D11_CREATE_DEVICE_DEBUG;
#endif
        ComPtr<IDXGIAdapter1> requestedAdapter;
        if (!warp && requestedAdapterLuid_.has_value())
        {
            ComPtr<IDXGIFactory1> factory;
            HRESULT enumerationResult = CreateDXGIFactory1(IID_PPV_ARGS(&factory));
            if (FAILED(enumerationResult))
            {
                error_ = "CreateDXGIFactory1 failed while locating the selected monitor adapter: "
                    + hresultText(enumerationResult);
                return false;
            }
            for (UINT index = 0U;; ++index)
            {
                ComPtr<IDXGIAdapter1> candidate;
                enumerationResult = factory->EnumAdapters1(index, &candidate);
                if (enumerationResult == DXGI_ERROR_NOT_FOUND)
                {
                    break;
                }
                if (FAILED(enumerationResult))
                {
                    error_ = "IDXGIFactory1::EnumAdapters1 failed while locating the selected "
                        "monitor adapter: " + hresultText(enumerationResult);
                    return false;
                }
                DXGI_ADAPTER_DESC1 candidateDescription{};
                enumerationResult = candidate->GetDesc1(&candidateDescription);
                if (FAILED(enumerationResult))
                {
                    error_ = "IDXGIAdapter1::GetDesc1 failed while locating the selected "
                        "monitor adapter: " + hresultText(enumerationResult);
                    return false;
                }
                const platform::windows::DxgiAdapterLuid candidateLuid{
                    candidateDescription.AdapterLuid.LowPart,
                    candidateDescription.AdapterLuid.HighPart,
                };
                if (candidateLuid == *requestedAdapterLuid_)
                {
                    requestedAdapter = std::move(candidate);
                    break;
                }
            }
            if (!requestedAdapter)
            {
                error_ = "No active DXGI adapter matches the selected monitor LUID "
                    + platform::windows::dxgiAdapterLuidText(*requestedAdapterLuid_) + ".";
                return false;
            }
        }
        const D3D_DRIVER_TYPE driver = warp ? D3D_DRIVER_TYPE_WARP
            : requestedAdapter ? D3D_DRIVER_TYPE_UNKNOWN : D3D_DRIVER_TYPE_HARDWARE;
        IDXGIAdapter* adapterArgument = requestedAdapter.Get();
        HRESULT result = D3D11CreateDevice(adapterArgument, driver, nullptr, flags,
            featureLevels.data(), static_cast<UINT>(featureLevels.size()), D3D11_SDK_VERSION,
            &device_, &featureLevel_, &context_);
#ifndef NDEBUG
        if (result == DXGI_ERROR_SDK_COMPONENT_MISSING)
        {
            debugLayerUnavailable_ = true;
            flags &= ~D3D11_CREATE_DEVICE_DEBUG;
            result = D3D11CreateDevice(adapterArgument, driver, nullptr, flags,
                featureLevels.data(), static_cast<UINT>(featureLevels.size()), D3D11_SDK_VERSION,
                &device_, &featureLevel_, &context_);
        }
#endif
        if (FAILED(result))
        {
            error_ = std::string(warp ? "D3D11 WARP device creation failed: "
                                      : "D3D11 hardware device creation failed: ")
                + hresultText(result);
            return false;
        }
        information_.warp = warp;
        information_.debugLayer = (flags & D3D11_CREATE_DEVICE_DEBUG) != 0U;
        information_.featureLevel = featureLevelText(featureLevel_);
        ComPtr<IDXGIDevice> dxgiDevice;
        ComPtr<IDXGIAdapter> adapter;
        DXGI_ADAPTER_DESC description{};
        if (FAILED(device_.As(&dxgiDevice)) || FAILED(dxgiDevice->GetAdapter(&adapter))
            || FAILED(adapter->GetDesc(&description)))
        {
            error_ = "D3D11 device was created but its DXGI adapter could not be queried.";
            return false;
        }
        adapter_ = std::move(adapter);
        information_.adapterName = adapterName(description);
        information_.adapterLuid = platform::windows::DxgiAdapterLuid{
            description.AdapterLuid.LowPart,
            description.AdapterLuid.HighPart,
        };
        information_.selectedAdapterMatched = !requestedAdapterLuid_.has_value()
            || information_.adapterLuid == requestedAdapterLuid_;
        if (!warp && !information_.selectedAdapterMatched)
        {
            error_ = "D3D11 created a device on adapter "
                + platform::windows::dxgiAdapterLuidText(*information_.adapterLuid)
                + " instead of selected monitor adapter "
                + platform::windows::dxgiAdapterLuidText(*requestedAdapterLuid_) + ".";
            return false;
        }
        return true;
    }

    [[nodiscard]] bool selectOutput()
    {
        selectedOutput_.Reset();
        information_.outputDeviceName.clear();
        information_.selectedOutputMatched = requestedOutputDeviceName_.empty();
        if (information_.warp || requestedOutputDeviceName_.empty())
        {
            return true;
        }
        for (UINT index = 0U;; ++index)
        {
            ComPtr<IDXGIOutput> output;
            const HRESULT result = adapter_->EnumOutputs(index, &output);
            if (result == DXGI_ERROR_NOT_FOUND)
            {
                break;
            }
            if (FAILED(result))
            {
                error_ = "IDXGIAdapter::EnumOutputs failed while locating selected output "
                    + requestedOutputDeviceName_ + ": " + hresultText(result);
                return false;
            }
            DXGI_OUTPUT_DESC description{};
            if (FAILED(output->GetDesc(&description)))
            {
                continue;
            }
            const int size = WideCharToMultiByte(CP_UTF8, 0, description.DeviceName, -1,
                nullptr, 0, nullptr, nullptr);
            std::string name;
            if (size > 1)
            {
                name.resize(static_cast<std::size_t>(size));
                (void)WideCharToMultiByte(CP_UTF8, 0, description.DeviceName, -1,
                    name.data(), size, nullptr, nullptr);
                name.pop_back();
            }
            if (name == requestedOutputDeviceName_)
            {
                selectedOutput_ = std::move(output);
                information_.outputDeviceName = std::move(name);
                information_.selectedOutputMatched = true;
                return true;
            }
        }
        error_ = "Selected monitor output " + requestedOutputDeviceName_
            + " is not owned by D3D11 adapter " + information_.adapterName + ".";
        return false;
    }

    [[nodiscard]] bool createSwapChain(unsigned int width, unsigned int height)
    {
        ComPtr<IDXGIFactory2> factory;
        HRESULT result = adapter_->GetParent(IID_PPV_ARGS(&factory));
        if (FAILED(result))
        {
            error_ = "IDXGIAdapter::GetParent(IDXGIFactory2) failed: " + hresultText(result);
            return false;
        }
        DXGI_SWAP_CHAIN_DESC1 description{};
        description.Width = width;
        description.Height = height;
        description.Format = DXGI_FORMAT_B8G8R8A8_UNORM;
        description.SampleDesc.Count = 1;
        description.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
        description.BufferCount = 2;
        description.Scaling = DXGI_SCALING_STRETCH;
        description.SwapEffect = DXGI_SWAP_EFFECT_FLIP_DISCARD;
        description.AlphaMode = DXGI_ALPHA_MODE_IGNORE;
        // D3D11 does not support FRAME_LATENCY_WAITABLE_OBJECT in exclusive fullscreen.
        // The fullscreen path instead sets maximum latency through IDXGIDevice1.
        description.Flags = fullscreen_
            ? 0U : DXGI_SWAP_CHAIN_FLAG_FRAME_LATENCY_WAITABLE_OBJECT;
        ComPtr<IDXGISwapChain1> swapChain1;
        result = factory->CreateSwapChainForHwnd(
            device_.Get(), window_, &description, nullptr, selectedOutput_.Get(), &swapChain1);
        if (FAILED(result))
        {
            error_ = "IDXGIFactory2::CreateSwapChainForHwnd failed: " + hresultText(result);
            return false;
        }
        result = swapChain1.As(&swapChain_);
        if (FAILED(result))
        {
            error_ = "Querying IDXGISwapChain2 failed: " + hresultText(result);
            return false;
        }
        result = factory->MakeWindowAssociation(window_, DXGI_MWA_NO_ALT_ENTER);
        if (FAILED(result))
        {
            error_ = "IDXGIFactory::MakeWindowAssociation failed: " + hresultText(result);
            return false;
        }
        if (!fullscreen_)
        {
            result = swapChain_->SetMaximumFrameLatency(1);
            if (FAILED(result))
            {
                error_ = "IDXGISwapChain2::SetMaximumFrameLatency(1) failed: "
                    + hresultText(result);
                return false;
            }
            frameLatencyWaitableObject_ = swapChain_->GetFrameLatencyWaitableObject();
            information_.frameLatencyWaitableObject = frameLatencyWaitableObject_ != nullptr;
        }
        if (fullscreen_)
        {
            if (!selectedOutput_)
            {
                error_ = "Exclusive fullscreen requires a DXGI output matching the selected monitor.";
                return false;
            }
            result = swapChain_->SetFullscreenState(TRUE, selectedOutput_.Get());
            if (FAILED(result))
            {
                error_ = "IDXGISwapChain::SetFullscreenState failed for selected output "
                    + requestedOutputDeviceName_ + ": " + hresultText(result);
                return false;
            }
            information_.exclusiveFullscreen = true;
            ComPtr<IDXGIDevice1> dxgiDevice;
            result = device_.As(&dxgiDevice);
            if (FAILED(result))
            {
                error_ = "Querying IDXGIDevice1 for fullscreen frame latency failed: "
                    + hresultText(result);
                return false;
            }
            result = dxgiDevice->SetMaximumFrameLatency(1);
            if (FAILED(result))
            {
                error_ = "IDXGIDevice1::SetMaximumFrameLatency(1) failed in fullscreen: "
                    + hresultText(result);
                return false;
            }
            result = swapChain_->ResizeBuffers(
                0U, width, height, DXGI_FORMAT_UNKNOWN, description.Flags);
            if (FAILED(result))
            {
                error_ = "IDXGISwapChain::ResizeBuffers failed after fullscreen transition: "
                    + hresultText(result);
                return false;
            }
        }
        return true;
    }

    [[nodiscard]] bool createRenderTargets(unsigned int width, unsigned int height)
    {
        ComPtr<ID3D11Texture2D> backBuffer;
        HRESULT result = swapChain_->GetBuffer(0, IID_PPV_ARGS(&backBuffer));
        if (FAILED(result))
        {
            error_ = "IDXGISwapChain::GetBuffer failed: " + hresultText(result);
            return false;
        }
        result = device_->CreateRenderTargetView(backBuffer.Get(), nullptr, &renderTarget_);
        if (FAILED(result))
        {
            error_ = "ID3D11Device::CreateRenderTargetView failed: " + hresultText(result);
            return false;
        }
        D3D11_TEXTURE2D_DESC depthDescription{};
        depthDescription.Width = width;
        depthDescription.Height = height;
        depthDescription.MipLevels = 1;
        depthDescription.ArraySize = 1;
        depthDescription.Format = DXGI_FORMAT_D24_UNORM_S8_UINT;
        depthDescription.SampleDesc.Count = 1;
        depthDescription.BindFlags = D3D11_BIND_DEPTH_STENCIL;
        result = device_->CreateTexture2D(&depthDescription, nullptr, &depthTexture_);
        if (FAILED(result))
        {
            error_ = "ID3D11Device::CreateTexture2D(depth) failed: " + hresultText(result);
            return false;
        }
        result = device_->CreateDepthStencilView(depthTexture_.Get(), nullptr, &depthView_);
        if (FAILED(result))
        {
            error_ = "ID3D11Device::CreateDepthStencilView failed: " + hresultText(result);
            return false;
        }
        width_ = width;
        height_ = height;
        D3D11_VIEWPORT viewport{};
        viewport.Width = static_cast<float>(width);
        viewport.Height = static_cast<float>(height);
        viewport.MinDepth = 0.0F;
        viewport.MaxDepth = 1.0F;
        context_->RSSetViewports(1, &viewport);
        return true;
    }

    [[nodiscard]] bool compileShader(
        const char* entry,
        const char* target,
        ComPtr<ID3DBlob>& bytecode)
    {
        const auto shaderPath = executableDirectory() / L"Panel.hlsl";
        ComPtr<ID3DBlob> errors;
        UINT flags = D3DCOMPILE_ENABLE_STRICTNESS;
#ifndef NDEBUG
        flags |= D3DCOMPILE_DEBUG | D3DCOMPILE_SKIP_OPTIMIZATION;
#else
        flags |= D3DCOMPILE_OPTIMIZATION_LEVEL3;
#endif
        const HRESULT result = D3DCompileFromFile(shaderPath.c_str(), nullptr,
            D3D_COMPILE_STANDARD_FILE_INCLUDE, entry, target, flags, 0, &bytecode, &errors);
        if (FAILED(result))
        {
            error_ = "Shader compilation failed for " + shaderPath.string() + " (" + entry
                + "): " + hresultText(result);
            if (errors)
            {
                error_ += "\n" + std::string(
                    static_cast<const char*>(errors->GetBufferPointer()), errors->GetBufferSize());
            }
            return false;
        }
        if (errors && errors->GetBufferSize() > 0U)
        {
            std::cerr << "Shader compiler diagnostics for " << entry << ":\n"
                      << std::string(static_cast<const char*>(errors->GetBufferPointer()),
                             errors->GetBufferSize()) << '\n';
        }
        return true;
    }

    [[nodiscard]] bool createPipeline()
    {
        ComPtr<ID3DBlob> vertexBytecode;
        ComPtr<ID3DBlob> pixelBytecode;
        ComPtr<ID3DBlob> overlayPixelBytecode;
        if (!compileShader("VSMain", "vs_5_0", vertexBytecode)
            || !compileShader("PSMain", "ps_5_0", pixelBytecode)
            || !compileShader("PSOverlay", "ps_5_0", overlayPixelBytecode))
        {
            return false;
        }
        HRESULT result = device_->CreateVertexShader(vertexBytecode->GetBufferPointer(),
            vertexBytecode->GetBufferSize(), nullptr, &vertexShader_);
        if (FAILED(result))
        {
            error_ = "CreateVertexShader failed: " + hresultText(result);
            return false;
        }
        result = device_->CreatePixelShader(pixelBytecode->GetBufferPointer(),
            pixelBytecode->GetBufferSize(), nullptr, &pixelShader_);
        if (FAILED(result))
        {
            error_ = "CreatePixelShader failed: " + hresultText(result);
            return false;
        }
        result = device_->CreatePixelShader(overlayPixelBytecode->GetBufferPointer(),
            overlayPixelBytecode->GetBufferSize(), nullptr, &overlayPixelShader_);
        if (FAILED(result))
        {
            error_ = "CreatePixelShader(overlay) failed: " + hresultText(result);
            return false;
        }
        constexpr std::array layout{
            D3D11_INPUT_ELEMENT_DESC{"POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0,
                D3D11_INPUT_PER_VERTEX_DATA, 0},
            D3D11_INPUT_ELEMENT_DESC{"COLOR", 0, DXGI_FORMAT_R32G32B32A32_FLOAT, 0, 12,
                D3D11_INPUT_PER_VERTEX_DATA, 0},
            D3D11_INPUT_ELEMENT_DESC{"TEXCOORD", 0, DXGI_FORMAT_R32G32_FLOAT, 0, 28,
                D3D11_INPUT_PER_VERTEX_DATA, 0},
        };
        result = device_->CreateInputLayout(layout.data(), static_cast<UINT>(layout.size()),
            vertexBytecode->GetBufferPointer(), vertexBytecode->GetBufferSize(), &inputLayout_);
        if (FAILED(result))
        {
            error_ = "CreateInputLayout failed: " + hresultText(result);
            return false;
        }
        D3D11_BUFFER_DESC constantDescription{};
        constantDescription.ByteWidth = sizeof(rendering::PanelShaderConstants);
        constantDescription.Usage = D3D11_USAGE_DYNAMIC;
        constantDescription.BindFlags = D3D11_BIND_CONSTANT_BUFFER;
        constantDescription.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
        result = device_->CreateBuffer(&constantDescription, nullptr, &constantBuffer_);
        if (FAILED(result))
        {
            error_ = "CreateBuffer(constant) failed: " + hresultText(result);
            return false;
        }
        D3D11_RASTERIZER_DESC rasterizer{};
        rasterizer.FillMode = D3D11_FILL_SOLID;
        rasterizer.CullMode = D3D11_CULL_NONE;
        rasterizer.DepthClipEnable = TRUE;
        result = device_->CreateRasterizerState(&rasterizer, &rasterizerState_);
        if (FAILED(result))
        {
            error_ = "CreateRasterizerState failed: " + hresultText(result);
            return false;
        }
        D3D11_BLEND_DESC blend{};
        blend.RenderTarget[0].BlendEnable = TRUE;
        blend.RenderTarget[0].SrcBlend = D3D11_BLEND_SRC_ALPHA;
        blend.RenderTarget[0].DestBlend = D3D11_BLEND_INV_SRC_ALPHA;
        blend.RenderTarget[0].BlendOp = D3D11_BLEND_OP_ADD;
        blend.RenderTarget[0].SrcBlendAlpha = D3D11_BLEND_ONE;
        blend.RenderTarget[0].DestBlendAlpha = D3D11_BLEND_ZERO;
        blend.RenderTarget[0].BlendOpAlpha = D3D11_BLEND_OP_ADD;
        blend.RenderTarget[0].RenderTargetWriteMask = D3D11_COLOR_WRITE_ENABLE_ALL;
        result = device_->CreateBlendState(&blend, &blendState_);
        if (FAILED(result))
        {
            error_ = "CreateBlendState failed: " + hresultText(result);
            return false;
        }
        D3D11_BLEND_DESC opaqueBlend{};
        opaqueBlend.RenderTarget[0].BlendEnable = FALSE;
        opaqueBlend.RenderTarget[0].RenderTargetWriteMask = D3D11_COLOR_WRITE_ENABLE_ALL;
        result = device_->CreateBlendState(&opaqueBlend, &opaqueBlendState_);
        if (FAILED(result))
        {
            error_ = "CreateBlendState(opaque desktop base) failed: " + hresultText(result);
            return false;
        }
        D3D11_DEPTH_STENCIL_DESC depth{};
        depth.DepthEnable = TRUE;
        depth.DepthWriteMask = D3D11_DEPTH_WRITE_MASK_ALL;
        depth.DepthFunc = D3D11_COMPARISON_LESS_EQUAL;
        result = device_->CreateDepthStencilState(&depth, &depthStencilState_);
        if (FAILED(result))
        {
            error_ = "CreateDepthStencilState failed: " + hresultText(result);
            return false;
        }
        D3D11_SAMPLER_DESC pointSampler{};
        pointSampler.Filter = D3D11_FILTER_MIN_MAG_MIP_POINT;
        pointSampler.AddressU = D3D11_TEXTURE_ADDRESS_CLAMP;
        pointSampler.AddressV = D3D11_TEXTURE_ADDRESS_CLAMP;
        pointSampler.AddressW = D3D11_TEXTURE_ADDRESS_CLAMP;
        pointSampler.MaxLOD = D3D11_FLOAT32_MAX;
        result = device_->CreateSamplerState(&pointSampler, &pointSampler_);
        if (FAILED(result))
        {
            error_ = "CreateSamplerState(point) failed: " + hresultText(result);
            return false;
        }
        D3D11_SAMPLER_DESC linearSampler = pointSampler;
        linearSampler.Filter = D3D11_FILTER_MIN_MAG_MIP_LINEAR;
        result = device_->CreateSamplerState(&linearSampler, &linearSampler_);
        if (FAILED(result))
        {
            error_ = "CreateSamplerState(linear) failed: " + hresultText(result);
            return false;
        }
        return true;
    }

    [[nodiscard]] bool createVertexBuffer(
        const std::vector<Vertex>& vertices,
        ComPtr<ID3D11Buffer>& buffer)
    {
        D3D11_BUFFER_DESC description{};
        description.ByteWidth = static_cast<UINT>(vertices.size() * sizeof(Vertex));
        description.Usage = D3D11_USAGE_IMMUTABLE;
        description.BindFlags = D3D11_BIND_VERTEX_BUFFER;
        D3D11_SUBRESOURCE_DATA data{};
        data.pSysMem = vertices.data();
        const HRESULT result = device_->CreateBuffer(&description, &data, &buffer);
        if (FAILED(result))
        {
            error_ = "CreateBuffer(scene vertices) failed: " + hresultText(result);
            return false;
        }
        return true;
    }

    [[nodiscard]] bool createScene()
    {
        // One immutable unit quad is shared by every panel. Panel dimensions,
        // placement and curvature are supplied by the per-panel matrix.
        constexpr float halfWidth = 0.5F;
        constexpr float halfHeight = 0.5F;
        constexpr float z = 0.0F;
        panelVertices_ = {
            {-halfWidth, -halfHeight, z, 0.04F, 0.08F, 0.16F, 0.88F, 0.0F, 1.0F},
            {-halfWidth, halfHeight, z, 0.04F, 0.08F, 0.16F, 0.88F, 0.0F, 0.0F},
            {halfWidth, -halfHeight, z, 0.04F, 0.08F, 0.16F, 0.88F, 1.0F, 1.0F},
            {halfWidth, halfHeight, z, 0.04F, 0.08F, 0.16F, 0.88F, 1.0F, 0.0F},
        };
        constexpr std::array panelUvs{
            rendering::PanelVertexUv{0.0F, 1.0F},
            rendering::PanelVertexUv{0.0F, 0.0F},
            rendering::PanelVertexUv{1.0F, 1.0F},
            rendering::PanelVertexUv{1.0F, 0.0F},
        };
        constexpr std::array<std::uint16_t, 6> panelIndices{0, 1, 2, 2, 1, 3};
        if (!rendering::validatePanelGeometry(panelUvs, panelIndices))
        {
            error_ = "Panel vertex UVs or index ranges are invalid.";
            return false;
        }
        if (!createVertexBuffer(panelVertices_, panelBuffer_))
        {
            return false;
        }
        D3D11_BUFFER_DESC indexDescription{};
        indexDescription.ByteWidth = sizeof(panelIndices);
        indexDescription.Usage = D3D11_USAGE_IMMUTABLE;
        indexDescription.BindFlags = D3D11_BIND_INDEX_BUFFER;
        D3D11_SUBRESOURCE_DATA indexData{};
        indexData.pSysMem = panelIndices.data();
        const HRESULT indexResult = device_->CreateBuffer(
            &indexDescription, &indexData, &panelIndexBuffer_);
        if (FAILED(indexResult))
        {
            error_ = "CreateBuffer(panel indices) failed: " + hresultText(indexResult);
            return false;
        }

        constexpr float front = 0.002F;
        const auto line = [this](float x1, float y1, float z1, float x2, float y2, float z2,
                                 float r, float g, float b) {
            lineVertices_.push_back({x1, y1, z1, r, g, b, 1.0F});
            lineVertices_.push_back({x2, y2, z2, r, g, b, 1.0F});
        };
        line(-halfWidth, halfHeight, front, halfWidth, halfHeight, front, 1.0F, 0.85F, 0.1F);
        line(-halfWidth, -halfHeight, front, halfWidth, -halfHeight, front, 0.1F, 0.4F, 1.0F);
        line(-halfWidth, -halfHeight, front, -halfWidth, halfHeight, front, 1.0F, 0.15F, 0.15F);
        line(halfWidth, -halfHeight, front, halfWidth, halfHeight, front, 0.15F, 1.0F, 0.2F);
        line(-0.08F, 0.0F, front, 0.08F, 0.0F, front, 1.0F, 1.0F, 1.0F);
        line(0.0F, -0.08F, front, 0.0F, 0.08F, front, 1.0F, 1.0F, 1.0F);
        baseLineVertexCount_ = static_cast<UINT>(lineVertices_.size());

        gridStartVertex_ = baseLineVertexCount_;
        constexpr int gridLines = 10;
        for (int index = -gridLines; index <= gridLines; ++index)
        {
            const float fraction = static_cast<float>(index) / static_cast<float>(gridLines);
            line(fraction * halfWidth, -halfHeight, z - 0.01F,
                 fraction * halfWidth, halfHeight, z - 0.01F, 0.15F, 0.2F, 0.3F);
            line(-halfWidth, fraction * halfHeight, z - 0.01F,
                 halfWidth, fraction * halfHeight, z - 0.01F, 0.15F, 0.2F, 0.3F);
        }
        gridVertexCount_ = static_cast<UINT>(lineVertices_.size()) - gridStartVertex_;
        axesStartVertex_ = static_cast<UINT>(lineVertices_.size());
        line(0.0F, 0.0F, front, 0.35F, 0.0F, front, 1.0F, 0.0F, 0.0F);
        line(0.0F, 0.0F, front, 0.0F, 0.35F, front, 0.0F, 1.0F, 0.0F);
        line(0.0F, 0.0F, front, 0.0F, 0.0F, front + 0.35F, 0.0F, 0.4F, 1.0F);
        axesVertexCount_ = static_cast<UINT>(lineVertices_.size()) - axesStartVertex_;
        return createVertexBuffer(lineVertices_, lineBuffer_);
    }

    [[nodiscard]] bool resize(unsigned int width, unsigned int height)
    {
        if (width == 0U || height == 0U)
        {
            error_ = "Cannot resize D3D11 render targets to zero dimensions.";
            return false;
        }
        context_->OMSetRenderTargets(0, nullptr, nullptr);
        renderTarget_.Reset();
        depthView_.Reset();
        depthTexture_.Reset();
        const UINT flags = fullscreen_
            ? 0U : DXGI_SWAP_CHAIN_FLAG_FRAME_LATENCY_WAITABLE_OBJECT;
        const HRESULT result = swapChain_->ResizeBuffers(
            0, width, height, DXGI_FORMAT_UNKNOWN, flags);
        if (FAILED(result))
        {
            error_ = "IDXGISwapChain::ResizeBuffers failed: " + hresultText(result);
            return false;
        }
        return createRenderTargets(width, height);
    }

    [[nodiscard]] std::optional<capture::DesktopCaptureFrame> readbackTexture(
        ID3D11Texture2D* texture,
        std::uint64_t sequence,
        std::string_view operation)
    {
        if (texture == nullptr)
        {
            error_ = std::string(operation) + " requires a valid D3D11 texture.";
            return std::nullopt;
        }
        D3D11_TEXTURE2D_DESC sourceDescription{};
        texture->GetDesc(&sourceDescription);
        D3D11_TEXTURE2D_DESC stagingDescription = sourceDescription;
        stagingDescription.MipLevels = 1U;
        stagingDescription.ArraySize = 1U;
        stagingDescription.SampleDesc.Count = 1U;
        stagingDescription.SampleDesc.Quality = 0U;
        stagingDescription.Usage = D3D11_USAGE_STAGING;
        stagingDescription.BindFlags = 0U;
        stagingDescription.CPUAccessFlags = D3D11_CPU_ACCESS_READ;
        stagingDescription.MiscFlags = 0U;
        ComPtr<ID3D11Texture2D> staging;
        HRESULT result = device_->CreateTexture2D(&stagingDescription, nullptr, &staging);
        if (FAILED(result))
        {
            error_ = std::string(operation) + " CreateTexture2D(staging) failed: "
                + hresultText(result);
            return std::nullopt;
        }
        context_->CopyResource(staging.Get(), texture);
        D3D11_MAPPED_SUBRESOURCE mapped{};
        result = context_->Map(staging.Get(), 0U, D3D11_MAP_READ, 0U, &mapped);
        if (FAILED(result))
        {
            error_ = std::string(operation) + " Map(staging) failed: " + hresultText(result);
            return std::nullopt;
        }
        const auto packed = capture::calculatePackedBgraLayout(
            sourceDescription.Width, sourceDescription.Height);
        auto pixels = packed.has_value()
            ? std::make_shared<std::vector<std::byte>>(packed->bufferSize) : nullptr;
        const bool copied = packed.has_value() && pixels
            && capture::repackBgraRows(static_cast<const std::byte*>(mapped.pData),
                static_cast<std::size_t>(mapped.RowPitch) * sourceDescription.Height,
                mapped.RowPitch, *pixels, sourceDescription.Width, sourceDescription.Height);
        context_->Unmap(staging.Get(), 0U);
        if (!copied)
        {
            error_ = std::string(operation)
                + " could not repack mapped rows using the returned RowPitch.";
            return std::nullopt;
        }
        capture::DesktopCaptureFrame frame;
        frame.sequence = sequence == 0U ? 1U : sequence;
        frame.captureHostTimestamp = std::chrono::steady_clock::now();
        frame.sourceWidth = sourceDescription.Width;
        frame.sourceHeight = sourceDescription.Height;
        frame.sourceFormat = static_cast<std::uint32_t>(sourceDescription.Format);
        frame.cpuPixels = std::move(pixels);
        frame.cpuRowPitch = packed->stride;
        frame.valid = true;
        frame.status = capture::DesktopCaptureStatus::active;
        frame.sourceMonitorDeviceName = std::string(operation);
        return frame;
    }

    [[nodiscard]] bool dumpTexture(
        ID3D11Texture2D* texture,
        const std::string& path,
        std::uint64_t sequence,
        std::string_view operation,
        std::uint64_t& checksum)
    {
        auto frame = readbackTexture(texture, sequence, operation);
        if (!frame.has_value())
        {
            return false;
        }
        const auto content = capture::analyzeDesktopFrameContent(*frame);
        if (!content.has_value())
        {
            error_ = std::string(operation) + " produced an invalid BGRA readback.";
            return false;
        }
        checksum = content->checksum;
        const auto result = capture::writeDesktopFrameBmp(*frame, path);
        if (!result.success)
        {
            error_ = std::string(operation) + " failed: " + result.error;
            return false;
        }
        std::cout << operation << " path=" << path
                  << " checksum=0x" << std::hex << std::uppercase << checksum
                  << std::dec << '\n';
        return true;
    }

    [[nodiscard]] bool traceDesktopDescriptors()
    {
        if (desktopLocalTexture_ == nullptr || desktopShaderResource_ == nullptr)
        {
            error_ = "Desktop texture/SRV descriptor tracing requires valid resources.";
            return false;
        }
        D3D11_TEXTURE2D_DESC textureDescription{};
        desktopLocalTexture_->GetDesc(&textureDescription);
        D3D11_SHADER_RESOURCE_VIEW_DESC resourceDescription{};
        desktopShaderResource_->GetDesc(&resourceDescription);
        const auto texture = textureContract(textureDescription);
        const auto resource = shaderResourceContract(resourceDescription);
        if (!rendering::isCompatibleDesktopShaderResource(texture, resource))
        {
            error_ = "Desktop texture or SRV descriptor violates the BGRA shader-resource contract.";
            return false;
        }
        std::cout << "Desktop Intel upload texture descriptor:"
                  << " Width=" << texture.width
                  << " Height=" << texture.height
                  << " MipLevels=" << texture.mipLevels
                  << " ArraySize=" << texture.arraySize
                  << " Format=" << texture.format
                  << " SampleDesc.Count=" << texture.sampleCount
                  << " SampleDesc.Quality=" << texture.sampleQuality
                  << " Usage=" << texture.usage
                  << " BindFlags=0x" << std::hex << texture.bindFlags
                  << " CPUAccessFlags=0x" << texture.cpuAccessFlags
                  << " MiscFlags=0x" << texture.miscFlags << std::dec << '\n'
                  << "Desktop SRV descriptor:"
                  << " Format=" << resource.format
                  << " ViewDimension=" << resource.viewDimension
                  << " MostDetailedMip=" << resource.mostDetailedMip
                  << " MipLevels=" << resource.mipLevels << '\n'
                  << "Desktop shader registers: texture=t"
                  << rendering::desktopTextureShaderRegister
                  << " sampler=s" << rendering::desktopSamplerShaderRegister
                  << " constants=b" << rendering::panelConstantBufferShaderRegister << '\n';
        return true;
    }

    [[nodiscard]] bool render(
        const rendering::Matrix4& viewProjection,
        bool backgroundGrid,
        bool worldAxes,
        bool ready,
        bool desktopContent,
        capture::DesktopBackground desktopBackground,
        bool desktopStale)
    {
        if (!viewProjection.finite())
        {
            error_ = "Refusing to render a non-finite view-projection matrix.";
            return false;
        }
        rendering::PanelDefinition legacyPanel;
        legacyPanel.transform.position = {0.0, 0.0, -panelDistance_};
        legacyPanel.dimensions = {panelWidth_, panelHeight_};
        const rendering::Matrix4 legacyWorldViewProjection = viewProjection
            * rendering::panelWorldMatrix(legacyPanel);
        D3D11_MAPPED_SUBRESOURCE mapped{};
        HRESULT result = context_->Map(constantBuffer_.Get(), 0, D3D11_MAP_WRITE_DISCARD, 0, &mapped);
        if (FAILED(result))
        {
            error_ = "ID3D11DeviceContext::Map(constant buffer) failed: " + hresultText(result);
            return false;
        }
        rendering::PanelShaderConstants constants;
        for (std::size_t index = 0; index < constants.viewProjection.size(); ++index)
        {
            constants.viewProjection[index] = static_cast<float>(
                legacyWorldViewProjection.values[index]);
        }
        constants.desktopContentBounds = {
            static_cast<float>(desktopLayout_.contentMinimum[0]),
            static_cast<float>(desktopLayout_.contentMinimum[1]),
            static_cast<float>(desktopLayout_.contentMaximum[0]),
            static_cast<float>(desktopLayout_.contentMaximum[1]),
        };
        constants.desktopCrop = {
            static_cast<float>(desktopLayout_.cropLeft),
            static_cast<float>(desktopLayout_.cropTop),
            static_cast<float>(desktopLayout_.cropWidth),
            static_cast<float>(desktopLayout_.cropHeight),
        };
        constants.desktopRotation = static_cast<std::uint32_t>(desktopLayout_.rotation);
        constants.desktopFlipY = desktopLayout_.flipY ? 1U : 0U;
        desktopStatistics_.panelContentRequested = desktopContent
            ? capture::PanelContent::desktop : capture::PanelContent::synthetic;
        desktopStatistics_.desktopTextureAvailable = desktopShaderResource_ != nullptr;
        desktopStatistics_.panelContentEffective = capture::effectiveDesktopPanelMode(
            desktopStatistics_.panelContentRequested,
            desktopStatistics_.desktopSrvValid, desktopStale);
        constants.desktopEnabled = desktopContent ? 1U : 0U;
        constants.desktopUnavailable = desktopStatistics_.panelContentEffective
            == capture::DesktopPanelEffectiveMode::unavailable ? 1U : 0U;
        constants.desktopBackgroundGrid = desktopBackground == capture::DesktopBackground::grid
            ? 1U : 0U;
        constants.desktopDebugMode = static_cast<std::uint32_t>(desktopShaderDebugMode_);
        if (!rendering::panelUvConstantsFinite(constants))
        {
            context_->Unmap(constantBuffer_.Get(), 0);
            error_ = "Panel desktop UV constants are non-finite or degenerate.";
            return false;
        }
        *static_cast<rendering::PanelShaderConstants*>(mapped.pData) = constants;
        context_->Unmap(constantBuffer_.Get(), 0);
        if (desktopContent && desktopDrawEvents_.size() == 2U)
        {
            desktopDrawEvents_.push_back(rendering::DesktopDrawEvent::constantsUpdated);
        }
        if (desktopContent && !desktopConstantsTraced_)
        {
            const float scaleX = constants.desktopCrop[2]
                / (constants.desktopContentBounds[2] - constants.desktopContentBounds[0]);
            const float scaleY = constants.desktopCrop[3]
                / (constants.desktopContentBounds[3] - constants.desktopContentBounds[1]);
            const float offsetX = constants.desktopCrop[0]
                - constants.desktopContentBounds[0] * scaleX;
            const float offsetY = constants.desktopCrop[1]
                - constants.desktopContentBounds[1] * scaleY;
            std::cout << "Desktop panel constants: content_mode=" << constants.desktopEnabled
                      << " texture_available=" << (desktopStatistics_.desktopTextureAvailable ? 1 : 0)
                      << " unavailable=" << constants.desktopUnavailable
                      << " uv_scale=[" << scaleX << ',' << scaleY << ']'
                      << " uv_offset=[" << offsetX << ',' << offsetY << ']'
                      << " source_rotation=" << constants.desktopRotation
                      << " flip_y=" << constants.desktopFlipY
                      << " debug_mode="
                      << rendering::desktopShaderDebugModeText(desktopShaderDebugMode_) << '\n';
            desktopConstantsTraced_ = true;
        }

        const std::array clearColor = ready
            ? std::array{0.005F, 0.008F, 0.018F, 1.0F}
            : std::array{0.16F, 0.07F, 0.01F, 1.0F};
        context_->ClearRenderTargetView(renderTarget_.Get(), clearColor.data());
        context_->ClearDepthStencilView(depthView_.Get(),
            D3D11_CLEAR_DEPTH | D3D11_CLEAR_STENCIL, 1.0F, 0);
        ID3D11RenderTargetView* target = renderTarget_.Get();
        context_->OMSetRenderTargets(1, &target, depthView_.Get());
        context_->IASetInputLayout(inputLayout_.Get());
        constexpr UINT stride = sizeof(Vertex);
        constexpr UINT offset = 0;
        ID3D11Buffer* panel = panelBuffer_.Get();
        context_->IASetVertexBuffers(0, 1, &panel, &stride, &offset);
        context_->IASetIndexBuffer(panelIndexBuffer_.Get(), DXGI_FORMAT_R16_UINT, 0);
        context_->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
        if (desktopContent && desktopDrawEvents_.size() == 3U)
        {
            desktopDrawEvents_.push_back(rendering::DesktopDrawEvent::geometryBound);
        }
        context_->VSSetShader(vertexShader_.Get(), nullptr, 0);
        context_->PSSetShader(pixelShader_.Get(), nullptr, 0);
        if (desktopContent && desktopDrawEvents_.size() == 4U)
        {
            desktopDrawEvents_.push_back(rendering::DesktopDrawEvent::shadersBound);
        }
        ID3D11ShaderResourceView* desktopView = desktopStatistics_.panelContentEffective
            == capture::DesktopPanelEffectiveMode::desktop
            ? desktopShaderResource_.Get() : nullptr;
        context_->PSSetShaderResources(rendering::desktopTextureShaderRegister, 1, &desktopView);
        ID3D11SamplerState* desktopSampler = desktopFilter_ == capture::DesktopFilter::point
            ? pointSampler_.Get() : linearSampler_.Get();
        context_->PSSetSamplers(rendering::desktopSamplerShaderRegister, 1, &desktopSampler);
        ID3D11Buffer* constant = constantBuffer_.Get();
        context_->VSSetConstantBuffers(rendering::panelConstantBufferShaderRegister, 1, &constant);
        context_->PSSetConstantBuffers(rendering::panelConstantBufferShaderRegister, 1, &constant);
        if (desktopContent && desktopDrawEvents_.size() == 5U)
        {
            desktopDrawEvents_.push_back(rendering::DesktopDrawEvent::shaderResourceBound);
            desktopDrawEvents_.push_back(rendering::DesktopDrawEvent::samplerBound);
            desktopDrawEvents_.push_back(rendering::DesktopDrawEvent::constantBuffersBound);
        }
        context_->RSSetState(rasterizerState_.Get());
        const std::array blendFactor{0.0F, 0.0F, 0.0F, 0.0F};
        const bool opaqueBase = rendering::desktopBaseBlendMode(
            desktopContent, desktopDebugOpaqueBase_) == rendering::DesktopPanelBlendMode::opaque;
        context_->OMSetBlendState(
            opaqueBase ? opaqueBlendState_.Get() : blendState_.Get(),
            blendFactor.data(), 0xFFFFFFFFU);
        context_->OMSetDepthStencilState(depthStencilState_.Get(), 0);
        if (desktopContent && desktopDrawEvents_.size() == 8U)
        {
            desktopDrawEvents_.push_back(rendering::DesktopDrawEvent::statesBound);
        }
        if (desktopStatistics_.panelContentEffective
            == capture::DesktopPanelEffectiveMode::desktop)
        {
            ++desktopStatistics_.desktopSrvBindCount;
            desktopStatistics_.latestBoundSequence = desktopFrameSequence_;
            if (!firstSrvBindTraced_)
            {
                std::cout << "desktop-first-frame stage=desktop_srv_bound sequence="
                          << desktopFrameSequence_ << '\n';
                firstSrvBindTraced_ = true;
            }
            if (!firstShaderPathTraced_)
            {
                std::cout << "desktop-first-frame stage=desktop_pixel_shader_selected sequence="
                          << desktopFrameSequence_ << '\n';
                firstShaderPathTraced_ = true;
            }
        }
        context_->DrawIndexed(6, 0, 0);
        if (desktopContent && desktopDrawEvents_.size() == 9U)
        {
            desktopDrawEvents_.push_back(rendering::DesktopDrawEvent::baseDrawIndexed);
        }
        if (desktopStatistics_.panelContentEffective
            == capture::DesktopPanelEffectiveMode::desktop)
        {
            ++desktopStatistics_.renderedDesktopFrames;
            if (!firstDesktopRenderedTraced_)
            {
                std::cout << "desktop-first-frame stage=desktop_frame_rendered sequence="
                          << desktopFrameSequence_ << '\n';
                firstDesktopRenderedTraced_ = true;
            }
        }
        else if (desktopStatistics_.panelContentEffective
            == capture::DesktopPanelEffectiveMode::unavailable)
        {
            ++desktopStatistics_.renderedUnavailableFrames;
        }
        else
        {
            ++desktopStatistics_.renderedSyntheticFrames;
        }
        const bool drawPanelOverlay = rendering::desktopOverlayEnabled(desktopShaderDebugMode_);
        ID3D11Buffer* lines = lineBuffer_.Get();
        context_->IASetVertexBuffers(0, 1, &lines, &stride, &offset);
        context_->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_LINELIST);
        context_->PSSetShader(overlayPixelShader_.Get(), nullptr, 0);
        context_->OMSetBlendState(blendState_.Get(), blendFactor.data(), 0xFFFFFFFFU);
        if (drawPanelOverlay)
        {
            context_->Draw(baseLineVertexCount_, 0);
            if (desktopContent && desktopDrawEvents_.size() == 10U)
            {
                desktopDrawEvents_.push_back(rendering::DesktopDrawEvent::overlayDraw);
            }
        }
        if (backgroundGrid)
        {
            context_->Draw(gridVertexCount_, gridStartVertex_);
        }
        if (worldAxes)
        {
            context_->Draw(axesVertexCount_, axesStartVertex_);
        }
        ID3D11ShaderResourceView* nullView{};
        context_->PSSetShaderResources(rendering::desktopTextureShaderRegister, 1, &nullView);
        if (desktopContent
            && desktopDrawEvents_.size() == (drawPanelOverlay ? 11U : 10U))
        {
            desktopDrawEvents_.push_back(rendering::DesktopDrawEvent::shaderResourceUnbound);
        }
        if (desktopContent && !desktopBlendStateTraced_)
        {
            std::cout << "Desktop blend states: base="
                      << (opaqueBase ? "opaque_disabled_blending" : "alpha_blending")
                      << " overlay=alpha_blending"
                      << " overlay_enabled=" << (drawPanelOverlay ? "yes" : "no") << '\n'
                      << "Desktop bound slots: texture=t"
                      << rendering::desktopTextureShaderRegister
                      << " sampler=s" << rendering::desktopSamplerShaderRegister
                      << " VS_constants=b" << rendering::panelConstantBufferShaderRegister
                      << " PS_constants=b" << rendering::panelConstantBufferShaderRegister << '\n';
            desktopBlendStateTraced_ = true;
        }
        if (desktopDebugRenderTargetPath_.has_value() && !renderTargetDumpAttempted_)
        {
            renderTargetDumpAttempted_ = true;
            ComPtr<ID3D11Resource> targetResource;
            renderTarget_->GetResource(&targetResource);
            ComPtr<ID3D11Texture2D> targetTexture;
            result = targetResource.As(&targetTexture);
            if (FAILED(result))
            {
                error_ = "Render-target resource is not an ID3D11Texture2D: "
                    + hresultText(result);
                return false;
            }
            if (!dumpTexture(targetTexture.Get(), *desktopDebugRenderTargetPath_,
                    desktopFrameSequence_, "desktop_render_target_readback",
                    renderTargetReadbackChecksum_))
            {
                return false;
            }
        }
        return true;
    }

    [[nodiscard]] bool renderScene(
        std::span<const rendering::PanelRenderInstance> panels,
        bool backgroundGrid,
        bool worldAxes,
        bool ready,
        capture::DesktopBackground desktopBackground)
    {
        if (panels.empty() || panels.size() > rendering::maximumPanelCount)
        {
            error_ = "D3D11 multi-panel rendering requires between one and three panels.";
            return false;
        }
        for (const auto& panel : panels)
        {
            if (panel.visible && !panel.worldViewProjection.finite())
            {
                error_ = "Refusing to render a non-finite panel matrix.";
                return false;
            }
        }

        const std::array clearColor = ready
            ? std::array{0.005F, 0.008F, 0.018F, 1.0F}
            : std::array{0.16F, 0.07F, 0.01F, 1.0F};
        context_->ClearRenderTargetView(renderTarget_.Get(), clearColor.data());
        context_->ClearDepthStencilView(depthView_.Get(),
            D3D11_CLEAR_DEPTH | D3D11_CLEAR_STENCIL, 1.0F, 0);
        ID3D11RenderTargetView* target = renderTarget_.Get();
        context_->OMSetRenderTargets(1, &target, depthView_.Get());
        context_->IASetInputLayout(inputLayout_.Get());
        constexpr UINT stride = sizeof(Vertex);
        constexpr UINT offset = 0U;
        ID3D11Buffer* panelBuffer = panelBuffer_.Get();
        context_->IASetVertexBuffers(0, 1, &panelBuffer, &stride, &offset);
        context_->IASetIndexBuffer(panelIndexBuffer_.Get(), DXGI_FORMAT_R16_UINT, 0);
        context_->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
        context_->VSSetShader(vertexShader_.Get(), nullptr, 0);
        context_->PSSetShader(pixelShader_.Get(), nullptr, 0);
        ID3D11Buffer* constant = constantBuffer_.Get();
        context_->VSSetConstantBuffers(rendering::panelConstantBufferShaderRegister, 1, &constant);
        context_->PSSetConstantBuffers(rendering::panelConstantBufferShaderRegister, 1, &constant);
        context_->RSSetState(rasterizerState_.Get());
        context_->OMSetDepthStencilState(depthStencilState_.Get(), 0);
        sceneStatistics_.stateSetCalls += 11U;
        const std::array blendFactor{0.0F, 0.0F, 0.0F, 0.0F};

        ++sceneStatistics_.frames;
        bool desktopRequestedByScene{};
        bool desktopStaleInScene{};
        for (const auto& panel : panels)
        {
            desktopRequestedByScene = desktopRequestedByScene
                || (panel.visible && panel.content == rendering::PanelContentKind::desktop);
            desktopStaleInScene = desktopStaleInScene
                || (panel.visible && panel.content == rendering::PanelContentKind::desktop
                    && panel.stale);
        }
        desktopStatistics_.panelContentRequested = desktopRequestedByScene
            ? capture::PanelContent::desktop : capture::PanelContent::synthetic;
        desktopStatistics_.desktopTextureAvailable = desktopShaderResource_ != nullptr;
        desktopStatistics_.panelContentEffective = capture::effectiveDesktopPanelMode(
            desktopStatistics_.panelContentRequested,
            desktopStatistics_.desktopSrvValid, desktopStaleInScene);
        for (const auto& panel : panels)
        {
            if (!panel.visible)
            {
                ++sceneStatistics_.culledPanels;
                continue;
            }
            ++sceneStatistics_.visiblePanels;
            if (panel.sourceSlot >= rendering::maximumPanelCount)
            {
                error_ = "Panel references a desktop source slot outside fixed capacity.";
                return false;
            }
            const bool primarySource = panel.sourceSlot == 0U;
            auto& additionalSource = additionalDesktopSources_[panel.sourceSlot];
            ID3D11ShaderResourceView* sourceShaderResource = primarySource
                ? desktopShaderResource_.Get() : additionalSource.shaderResource.Get();
            const std::uint32_t sourceWidth = primarySource
                ? desktopWidth_ : additionalSource.width;
            const std::uint32_t sourceHeight = primarySource
                ? desktopHeight_ : additionalSource.height;
            const std::uint64_t sourceSequence = primarySource
                ? desktopFrameSequence_ : additionalSource.sequence;
            const capture::DesktopTextureLayout sourceLayout = primarySource
                ? desktopLayout_ : additionalSource.layout;
            const bool sourceValid = primarySource
                ? desktopStatistics_.desktopSrvValid
                : additionalSource.statistics.desktopSrvValid;
            const capture::DesktopTextureLayout panelDesktopLayout =
                sourceWidth > 0U && sourceHeight > 0U
                ? capture::calculateDesktopTextureLayout(
                    sourceWidth, sourceHeight,
                    panel.dimensions.width / panel.dimensions.height,
                    captureFit(panel.fit), sourceLayout.rotation,
                    sourceLayout.flipY)
                : sourceLayout;
            rendering::PanelShaderConstants constants;
            for (std::size_t index = 0; index < constants.viewProjection.size(); ++index)
            {
                constants.viewProjection[index] = static_cast<float>(
                    panel.worldViewProjection.values[index]);
            }
            constants.desktopContentBounds = {
                static_cast<float>(panelDesktopLayout.contentMinimum[0]),
                static_cast<float>(panelDesktopLayout.contentMinimum[1]),
                static_cast<float>(panelDesktopLayout.contentMaximum[0]),
                static_cast<float>(panelDesktopLayout.contentMaximum[1]),
            };
            constants.desktopCrop = {
                static_cast<float>(panelDesktopLayout.cropLeft),
                static_cast<float>(panelDesktopLayout.cropTop),
                static_cast<float>(panelDesktopLayout.cropWidth),
                static_cast<float>(panelDesktopLayout.cropHeight),
            };
            constants.desktopRotation = static_cast<std::uint32_t>(
                panelDesktopLayout.rotation);
            constants.desktopFlipY = panelDesktopLayout.flipY ? 1U : 0U;
            const bool desktopRequested = panel.content == rendering::PanelContentKind::desktop;
            // Stale is diagnostic metadata. A WAIT_TIMEOUT must retain the last valid SRV.
            const bool desktopAvailable = desktopRequested
                && sourceShaderResource != nullptr && sourceValid;
            auto& sourceStatistics = primarySource
                ? desktopStatistics_ : additionalSource.statistics;
            if (desktopRequested)
            {
                sourceStatistics.panelContentRequested = capture::PanelContent::desktop;
                sourceStatistics.desktopTextureAvailable =
                    sourceShaderResource != nullptr;
                sourceStatistics.panelContentEffective =
                    capture::effectiveDesktopPanelMode(capture::PanelContent::desktop,
                        sourceValid, panel.stale);
            }
            constants.desktopEnabled = desktopRequested ? 1U : 0U;
            constants.desktopUnavailable = desktopRequested && !desktopAvailable ? 1U : 0U;
            constants.desktopBackgroundGrid = desktopBackground
                == capture::DesktopBackground::grid ? 1U : 0U;
            constants.desktopPadding[0] = static_cast<std::uint32_t>(panel.content);
            constants.desktopPadding[1] = panel.id.value;
            constants.desktopPadding[2] = panel.selected ? 1U : 0U;
            constants.desktopDebugMode = static_cast<std::uint32_t>(desktopShaderDebugMode_);
            if (!rendering::panelUvConstantsFinite(constants))
            {
                error_ = "Panel desktop UV constants are non-finite or degenerate.";
                return false;
            }
            D3D11_MAPPED_SUBRESOURCE mapped{};
            const HRESULT mapResult = context_->Map(
                constantBuffer_.Get(), 0, D3D11_MAP_WRITE_DISCARD, 0, &mapped);
            if (FAILED(mapResult))
            {
                error_ = "ID3D11DeviceContext::Map(per-panel constants) failed: "
                    + hresultText(mapResult);
                return false;
            }
            *static_cast<rendering::PanelShaderConstants*>(mapped.pData) = constants;
            context_->Unmap(constantBuffer_.Get(), 0);
            ++sceneStatistics_.constantBufferUpdates;

            ID3D11ShaderResourceView* desktopView = desktopAvailable
                ? sourceShaderResource : nullptr;
            context_->PSSetShaderResources(
                rendering::desktopTextureShaderRegister, 1, &desktopView);
            ++sceneStatistics_.shaderResourceBindCalls;
            ID3D11SamplerState* panelSampler = panel.filter == rendering::PanelFilterMode::point
                ? pointSampler_.Get() : linearSampler_.Get();
            context_->PSSetSamplers(
                rendering::desktopSamplerShaderRegister, 1, &panelSampler);
            ++sceneStatistics_.samplerBindCalls;
            const bool opaqueBase = rendering::desktopBaseBlendMode(
                desktopRequested, desktopDebugOpaqueBase_)
                == rendering::DesktopPanelBlendMode::opaque;
            context_->OMSetBlendState(
                opaqueBase ? opaqueBlendState_.Get() : blendState_.Get(),
                blendFactor.data(), 0xFFFFFFFFU);
            context_->DrawIndexed(6, 0, 0);
            ++sceneStatistics_.drawCalls;
            ++sceneStatistics_.baseDrawCalls;
            sceneStatistics_.stateSetCalls += 3U;
            if (desktopAvailable)
            {
                ++desktopStatistics_.desktopSrvBindCount;
                ++desktopStatistics_.renderedDesktopFrames;
                desktopStatistics_.latestBoundSequence = sourceSequence;
                if (!primarySource)
                {
                    ++additionalSource.statistics.desktopSrvBindCount;
                    ++additionalSource.statistics.renderedDesktopFrames;
                    additionalSource.statistics.latestBoundSequence = sourceSequence;
                }
                if (!firstSrvBindTraced_)
                {
                    std::cout << "desktop-first-frame stage=desktop_srv_bound sequence="
                              << sourceSequence << '\n';
                    firstSrvBindTraced_ = true;
                }
                if (!firstDesktopRenderedTraced_)
                {
                    std::cout << "desktop-first-frame stage=desktop_frame_rendered sequence="
                              << sourceSequence << '\n';
                    firstDesktopRenderedTraced_ = true;
                }
            }
            else if (desktopRequested)
            {
                ++desktopStatistics_.renderedUnavailableFrames;
                if (!primarySource)
                {
                    ++additionalSource.statistics.renderedUnavailableFrames;
                }
            }
            else
            {
                ++desktopStatistics_.renderedSyntheticFrames;
            }

            if (panel.overlay.enabled
                && rendering::desktopOverlayEnabled(desktopShaderDebugMode_))
            {
                ID3D11Buffer* lines = lineBuffer_.Get();
                context_->IASetVertexBuffers(0, 1, &lines, &stride, &offset);
                context_->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_LINELIST);
                context_->PSSetShader(overlayPixelShader_.Get(), nullptr, 0);
                context_->OMSetBlendState(blendState_.Get(), blendFactor.data(), 0xFFFFFFFFU);
                context_->Draw(baseLineVertexCount_, 0);
                ++sceneStatistics_.drawCalls;
                ++sceneStatistics_.overlayDrawCalls;
                sceneStatistics_.stateSetCalls += 4U;
                if (backgroundGrid)
                {
                    context_->Draw(gridVertexCount_, gridStartVertex_);
                    ++sceneStatistics_.drawCalls;
                    ++sceneStatistics_.auxiliaryDrawCalls;
                }
                if (worldAxes)
                {
                    context_->Draw(axesVertexCount_, axesStartVertex_);
                    ++sceneStatistics_.drawCalls;
                    ++sceneStatistics_.auxiliaryDrawCalls;
                }
                context_->IASetVertexBuffers(0, 1, &panelBuffer, &stride, &offset);
                context_->IASetIndexBuffer(panelIndexBuffer_.Get(), DXGI_FORMAT_R16_UINT, 0);
                context_->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
                context_->PSSetShader(pixelShader_.Get(), nullptr, 0);
                sceneStatistics_.stateSetCalls += 4U;
            }
            ID3D11ShaderResourceView* nullView{};
            context_->PSSetShaderResources(
                rendering::desktopTextureShaderRegister, 1, &nullView);
            ++sceneStatistics_.shaderResourceBindCalls;
        }
        return true;
    }

    [[nodiscard]] bool updateDesktopFrame(
        const capture::DesktopCaptureFrame& frame,
        capture::DesktopFit fit,
        capture::DesktopFilter filter,
        bool flipY)
    {
        if (!frame.valid || (!frame.cpuPixels
                && (!frame.surface || frame.surface->sharedHandle() == nullptr)))
        {
            error_ = "Refusing to consume an invalid desktop capture frame.";
            return false;
        }
        if (frame.cpuPixels)
        {
            ++desktopStatistics_.cpuFramesSeen;
            const auto packed = capture::calculatePackedBgraLayout(
                frame.sourceWidth, frame.sourceHeight);
            const bool bufferValid = packed.has_value()
                && frame.cpuRowPitch >= packed->stride
                && frame.cpuPixels->size() >= static_cast<std::size_t>(frame.cpuRowPitch)
                    * frame.sourceHeight;
            capture::DesktopUploadState uploadState{
                desktopFrameSequence_, desktopWidth_, desktopHeight_, desktopFormat_,
                desktopLocalTexture_ != nullptr && desktopSurfaceIdentity_ == nullptr};
            const auto action = capture::decideDesktopUpload(uploadState, frame.sequence,
                frame.sourceWidth, frame.sourceHeight, frame.sourceFormat,
                frame.valid && bufferValid);
            if (action == capture::DesktopUploadAction::skipSameSequence)
            {
                ++desktopStatistics_.cpuFramesSkippedSameSequence;
                return true;
            }
            if (action == capture::DesktopUploadAction::invalid)
            {
                ++desktopStatistics_.updateSubresourceFailures;
                error_ = "CPU desktop frame has invalid dimensions, format, sequence, stride, or buffer size.";
                return false;
            }
            ++desktopStatistics_.cpuFramesConsumed;
            desktopStatistics_.latestConsumedSequence = frame.sequence;
            if (!firstCpuConsumedTraced_)
            {
                std::cout << "desktop-first-frame stage=cpu_frame_consumed sequence="
                          << frame.sequence << '\n';
                firstCpuConsumedTraced_ = true;
            }
            const bool dimensionsChanged = action == capture::DesktopUploadAction::create
                || action == capture::DesktopUploadAction::recreate;
            if (dimensionsChanged)
            {
                D3D11_TEXTURE2D_DESC description{};
                description.Width = frame.sourceWidth;
                description.Height = frame.sourceHeight;
                description.MipLevels = 1;
                description.ArraySize = 1;
                description.Format = static_cast<DXGI_FORMAT>(frame.sourceFormat);
                description.SampleDesc.Count = 1;
                description.Usage = D3D11_USAGE_DEFAULT;
                description.BindFlags = D3D11_BIND_SHADER_RESOURCE;
                const auto textureDescriptionContract = textureContract(description);
                if (!rendering::isShaderReadableDesktopTexture(textureDescriptionContract))
                {
                    error_ = "CPU-upload texture descriptor is not a shader-readable BGRA texture.";
                    return false;
                }
                desktopShaderResource_.Reset();
                desktopLocalTexture_.Reset();
                HRESULT result = device_->CreateTexture2D(
                    &description, nullptr, &desktopLocalTexture_);
                if (FAILED(result))
                {
                    error_ = "Creating CPU-upload desktop texture failed: " + hresultText(result);
                    return false;
                }
                if (action == capture::DesktopUploadAction::create)
                {
                    ++desktopStatistics_.uploadTextureCreations;
                    sceneStatistics_.resourcesCreatedAtStartup += 2U;
                }
                else
                {
                    ++desktopStatistics_.uploadTextureRecreations;
                    sceneStatistics_.resourcesCreatedSteadyState += 2U;
                }
                if (!firstUploadTextureTraced_)
                {
                    std::cout << "desktop-first-frame stage=upload_texture_created sequence="
                              << frame.sequence << '\n';
                    firstUploadTextureTraced_ = true;
                }
                D3D11_SHADER_RESOURCE_VIEW_DESC resourceDescription{};
                resourceDescription.Format = description.Format;
                resourceDescription.ViewDimension = D3D11_SRV_DIMENSION_TEXTURE2D;
                resourceDescription.Texture2D.MostDetailedMip = 0U;
                resourceDescription.Texture2D.MipLevels = 1U;
                result = device_->CreateShaderResourceView(desktopLocalTexture_.Get(),
                    &resourceDescription, &desktopShaderResource_);
                if (FAILED(result))
                {
                    ++desktopStatistics_.desktopSrvFailures;
                    error_ = "Creating CPU-upload desktop SRV failed: " + hresultText(result);
                    return false;
                }
                ++desktopStatistics_.desktopSrvCreations;
                desktopStatistics_.desktopSrvValid = true;
                desktopSharedTexture_.Reset();
                desktopKeyedMutex_.Reset();
                desktopSurfaceIdentity_ = nullptr;
                desktopSharedHandle_ = nullptr;
                desktopWidth_ = frame.sourceWidth;
                desktopHeight_ = frame.sourceHeight;
                desktopFormat_ = frame.sourceFormat;
                if (!desktopDescriptorsTraced_)
                {
                    if (!traceDesktopDescriptors())
                    {
                        return false;
                    }
                    desktopDescriptorsTraced_ = true;
                }
            }
            const auto updateStart = std::chrono::steady_clock::now();
            context_->UpdateSubresource(desktopLocalTexture_.Get(), 0U, nullptr,
                frame.cpuPixels->data(), frame.cpuRowPitch, 0U);
            const auto updateEnd = std::chrono::steady_clock::now();
            const double updateMilliseconds = std::chrono::duration<double, std::milli>(
                updateEnd - updateStart).count();
            ++desktopStatistics_.updateSubresourceCalls;
            desktopStatistics_.uploadBytesSubmitted += packed->bufferSize;
            desktopStatistics_.latestUploadBytes = packed->bufferSize;
            desktopStatistics_.averageUpdateSubresourceMilliseconds +=
                (updateMilliseconds
                    - desktopStatistics_.averageUpdateSubresourceMilliseconds)
                / static_cast<double>(desktopStatistics_.updateSubresourceCalls);
            desktopStatistics_.maximumUpdateSubresourceMilliseconds = std::max(
                desktopStatistics_.maximumUpdateSubresourceMilliseconds,
                updateMilliseconds);
            if (desktopFirstUploadTimestamp_
                == std::chrono::steady_clock::time_point{})
            {
                desktopFirstUploadTimestamp_ = updateEnd;
            }
            const double uploadSeconds = std::chrono::duration<double>(
                updateEnd - desktopFirstUploadTimestamp_).count();
            desktopStatistics_.uploadFramesPerSecond = uploadSeconds > 0.0
                ? static_cast<double>(desktopStatistics_.updateSubresourceCalls - 1U)
                    / uploadSeconds : 0.0;
            ++sceneStatistics_.textureUploads;
            if (!firstUpdateTraced_)
            {
                std::cout << "desktop-first-frame stage=update_subresource_completed sequence="
                          << frame.sequence << '\n';
                firstUpdateTraced_ = true;
            }
            if (!firstSrvCreatedTraced_ && desktopStatistics_.desktopSrvValid)
            {
                std::cout << "desktop-first-frame stage=desktop_srv_created sequence="
                          << frame.sequence << '\n';
                firstSrvCreatedTraced_ = true;
            }
            if (desktopDrawEvents_.empty())
            {
                desktopDrawEvents_.push_back(rendering::DesktopDrawEvent::textureUpdated);
                desktopDrawEvents_.push_back(rendering::DesktopDrawEvent::shaderResourceReady);
            }
            if (desktopDebugReadbackUploadPath_.has_value() && !uploadReadbackAttempted_)
            {
                uploadReadbackAttempted_ = true;
                const auto sourceContent = capture::analyzeDesktopFrameContent(frame);
                if (!sourceContent.has_value())
                {
                    ++desktopStatistics_.updateSubresourceFailures;
                    error_ = "CPU desktop frame could not be analyzed for debug readback.";
                    return false;
                }
                desktopUploadSourceChecksum_ = sourceContent->checksum;
                if (!dumpTexture(desktopLocalTexture_.Get(), *desktopDebugReadbackUploadPath_,
                        frame.sequence, "desktop_upload_readback", uploadReadbackChecksum_))
                {
                    return false;
                }
                std::cout << "Desktop upload checksum comparison: source=0x"
                          << std::hex << std::uppercase << desktopUploadSourceChecksum_
                          << " readback=0x" << uploadReadbackChecksum_ << std::dec
                          << " match="
                          << (desktopUploadSourceChecksum_ == uploadReadbackChecksum_
                                  ? "yes" : "no") << '\n';
                if (desktopUploadSourceChecksum_ != uploadReadbackChecksum_)
                {
                    error_ = "Desktop upload readback checksum does not match the CPU source.";
                    return false;
                }
            }
            desktopFrameSequence_ = frame.sequence;
            desktopStatistics_.latestUploadedSequence = frame.sequence;
            desktopStatistics_.latestUploadWidth = frame.sourceWidth;
            desktopStatistics_.latestUploadHeight = frame.sourceHeight;
            desktopStatistics_.latestUploadFormat = frame.sourceFormat;
            desktopStatistics_.uploadTextureValid = desktopLocalTexture_ != nullptr;
            desktopFilter_ = filter;
            desktopLayout_ = capture::calculateDesktopTextureLayout(
                frame.sourceWidth, frame.sourceHeight, panelWidth_ / panelHeight_,
                fit, frame.rotation, flipY);
            return true;
        }
        if (frame.sequence == desktopFrameSequence_)
        {
            return true;
        }
        const bool resourceChanged = frame.surface->sharedHandle() != desktopSharedHandle_
            || frame.recoveryGeneration != desktopRecoveryGeneration_
            || frame.sourceWidth != desktopWidth_ || frame.sourceHeight != desktopHeight_
            || frame.sourceFormat != desktopFormat_;
        if (resourceChanged)
        {
            ComPtr<ID3D11Device1> device1;
            HRESULT result = device_.As(&device1);
            if (FAILED(result))
            {
                error_ = "Render device does not expose ID3D11Device1 for NT shared handles: "
                    + hresultText(result);
                return false;
            }
            ComPtr<ID3D11Texture2D> sharedTexture;
            result = device1->OpenSharedResource1(
                static_cast<HANDLE>(frame.surface->sharedHandle()), IID_PPV_ARGS(&sharedTexture));
            if (FAILED(result))
            {
                error_ = "ID3D11Device1::OpenSharedResource1 failed for the desktop texture: "
                    + hresultText(result)
                    + ". Cross-adapter sharing was not claimed and no CPU fallback was used.";
                return false;
            }
            ComPtr<IDXGIKeyedMutex> keyedMutex;
            result = sharedTexture.As(&keyedMutex);
            if (FAILED(result))
            {
                error_ = "Shared desktop texture does not expose IDXGIKeyedMutex: "
                    + hresultText(result);
                return false;
            }
            D3D11_TEXTURE2D_DESC description{};
            sharedTexture->GetDesc(&description);
            description.MiscFlags = 0U;
            description.BindFlags = D3D11_BIND_SHADER_RESOURCE;
            description.CPUAccessFlags = 0U;
            description.Usage = D3D11_USAGE_DEFAULT;
            ComPtr<ID3D11Texture2D> localTexture;
            result = device_->CreateTexture2D(&description, nullptr, &localTexture);
            if (FAILED(result))
            {
                error_ = "Creating render-owned desktop texture failed: " + hresultText(result);
                return false;
            }
            ComPtr<ID3D11ShaderResourceView> shaderResource;
            D3D11_SHADER_RESOURCE_VIEW_DESC resourceDescription{};
            resourceDescription.Format = description.Format;
            resourceDescription.ViewDimension = D3D11_SRV_DIMENSION_TEXTURE2D;
            resourceDescription.Texture2D.MostDetailedMip = 0U;
            resourceDescription.Texture2D.MipLevels = 1U;
            result = device_->CreateShaderResourceView(
                localTexture.Get(), &resourceDescription, &shaderResource);
            if (FAILED(result))
            {
                error_ = "Creating desktop shader-resource view failed: " + hresultText(result);
                return false;
            }
            desktopSharedTexture_ = std::move(sharedTexture);
            desktopKeyedMutex_ = std::move(keyedMutex);
            desktopLocalTexture_ = std::move(localTexture);
            desktopShaderResource_ = std::move(shaderResource);
            desktopStatistics_.desktopSrvValid = true;
            desktopSurfaceIdentity_ = frame.surface.get();
            desktopSharedHandle_ = frame.surface->sharedHandle();
            desktopRecoveryGeneration_ = frame.recoveryGeneration;
            desktopWidth_ = frame.sourceWidth;
            desktopHeight_ = frame.sourceHeight;
            desktopFormat_ = frame.sourceFormat;
            if (!desktopDescriptorsTraced_)
            {
                if (!traceDesktopDescriptors())
                {
                    return false;
                }
                desktopDescriptorsTraced_ = true;
            }
        }
        const HRESULT acquired = desktopKeyedMutex_->AcquireSync(1U, 0U);
        if (acquired == WAIT_TIMEOUT)
        {
            return true;
        }
        if (FAILED(acquired))
        {
            error_ = "Render keyed-mutex AcquireSync failed: " + hresultText(acquired);
            return false;
        }
        context_->CopyResource(desktopLocalTexture_.Get(), desktopSharedTexture_.Get());
        ++sceneStatistics_.textureUploads;
        if (desktopDrawEvents_.empty())
        {
            desktopDrawEvents_.push_back(rendering::DesktopDrawEvent::textureUpdated);
            desktopDrawEvents_.push_back(rendering::DesktopDrawEvent::shaderResourceReady);
        }
        const HRESULT released = desktopKeyedMutex_->ReleaseSync(0U);
        if (FAILED(released))
        {
            error_ = "Render keyed-mutex ReleaseSync failed: " + hresultText(released);
            return false;
        }
        desktopFrameSequence_ = frame.sequence;
        desktopFilter_ = filter;
        desktopLayout_ = capture::calculateDesktopTextureLayout(
            frame.sourceWidth, frame.sourceHeight, panelWidth_ / panelHeight_,
            fit, frame.rotation, flipY);
        return true;
    }

    [[nodiscard]] bool updateDesktopFrame(
        std::size_t sourceSlot,
        const capture::DesktopCaptureFrame& frame,
        capture::DesktopFit fit,
        capture::DesktopFilter filter,
        bool flipY)
    {
        if (sourceSlot >= rendering::maximumPanelCount)
        {
            error_ = "Desktop upload source slot exceeds the fixed capacity of three.";
            return false;
        }
        if (sourceSlot == 0U)
        {
            return updateDesktopFrame(frame, fit, filter, flipY);
        }
        auto& resource = additionalDesktopSources_[sourceSlot];
        if (!frame.valid || (!frame.cpuPixels
                && (!frame.surface || frame.surface->sharedHandle() == nullptr)))
        {
            ++resource.statistics.updateSubresourceFailures;
            error_ = "Desktop source frame has neither a CPU buffer nor a shared surface.";
            return false;
        }
        if (!frame.cpuPixels)
        {
            if (frame.sequence == resource.sequence)
            {
                return true;
            }
            const bool resourceChanged =
                frame.surface->sharedHandle() != resource.sharedHandle
                || frame.recoveryGeneration != resource.recoveryGeneration
                || frame.sourceWidth != resource.width
                || frame.sourceHeight != resource.height
                || frame.sourceFormat != resource.format;
            if (resourceChanged)
            {
                const bool replacingResource = resource.texture != nullptr;
                ComPtr<ID3D11Device1> device1;
                HRESULT result = device_.As(&device1);
                if (FAILED(result))
                {
                    error_ = "Render device does not expose ID3D11Device1 for a per-source shared handle: "
                        + hresultText(result);
                    return false;
                }
                ComPtr<ID3D11Texture2D> sharedTexture;
                result = device1->OpenSharedResource1(
                    static_cast<HANDLE>(frame.surface->sharedHandle()),
                    IID_PPV_ARGS(&sharedTexture));
                if (FAILED(result))
                {
                    error_ = "Opening per-source shared desktop texture failed: "
                        + hresultText(result);
                    return false;
                }
                ComPtr<IDXGIKeyedMutex> keyedMutex;
                result = sharedTexture.As(&keyedMutex);
                if (FAILED(result))
                {
                    error_ = "Per-source shared desktop texture has no keyed mutex: "
                        + hresultText(result);
                    return false;
                }
                D3D11_TEXTURE2D_DESC description{};
                sharedTexture->GetDesc(&description);
                description.MiscFlags = 0U;
                description.BindFlags = D3D11_BIND_SHADER_RESOURCE;
                description.CPUAccessFlags = 0U;
                description.Usage = D3D11_USAGE_DEFAULT;
                ComPtr<ID3D11Texture2D> localTexture;
                result = device_->CreateTexture2D(
                    &description, nullptr, &localTexture);
                if (FAILED(result))
                {
                    error_ = "Creating per-source render-owned shared texture failed: "
                        + hresultText(result);
                    return false;
                }
                ComPtr<ID3D11ShaderResourceView> shaderResource;
                result = device_->CreateShaderResourceView(
                    localTexture.Get(), nullptr, &shaderResource);
                if (FAILED(result))
                {
                    ++resource.statistics.desktopSrvFailures;
                    error_ = "Creating per-source shared desktop SRV failed: "
                        + hresultText(result);
                    return false;
                }
                resource.sharedTexture = std::move(sharedTexture);
                resource.keyedMutex = std::move(keyedMutex);
                resource.texture = std::move(localTexture);
                resource.shaderResource = std::move(shaderResource);
                resource.surfaceIdentity = frame.surface.get();
                resource.sharedHandle = frame.surface->sharedHandle();
                resource.recoveryGeneration = frame.recoveryGeneration;
                resource.width = frame.sourceWidth;
                resource.height = frame.sourceHeight;
                resource.format = frame.sourceFormat;
                if (replacingResource)
                {
                    ++resource.statistics.uploadTextureRecreations;
                    sceneStatistics_.resourcesCreatedSteadyState += 2U;
                }
                else
                {
                    ++resource.statistics.uploadTextureCreations;
                    sceneStatistics_.resourcesCreatedAtStartup += 2U;
                }
                ++resource.statistics.desktopSrvCreations;
                resource.statistics.desktopSrvValid = true;
            }
            const HRESULT acquired = resource.keyedMutex->AcquireSync(1U, 0U);
            if (acquired == WAIT_TIMEOUT)
            {
                return true;
            }
            if (FAILED(acquired))
            {
                error_ = "Per-source render keyed-mutex AcquireSync failed: "
                    + hresultText(acquired);
                return false;
            }
            context_->CopyResource(resource.texture.Get(), resource.sharedTexture.Get());
            const HRESULT released = resource.keyedMutex->ReleaseSync(0U);
            if (FAILED(released))
            {
                error_ = "Per-source render keyed-mutex ReleaseSync failed: "
                    + hresultText(released);
                return false;
            }
            ++sceneStatistics_.textureUploads;
            resource.sequence = frame.sequence;
            resource.filter = filter;
            resource.layout = capture::calculateDesktopTextureLayout(
                frame.sourceWidth, frame.sourceHeight, panelWidth_ / panelHeight_,
                fit, frame.rotation, flipY);
            resource.statistics.latestUploadedSequence = frame.sequence;
            resource.statistics.latestUploadWidth = frame.sourceWidth;
            resource.statistics.latestUploadHeight = frame.sourceHeight;
            resource.statistics.latestUploadFormat = frame.sourceFormat;
            resource.statistics.uploadTextureValid = resource.texture != nullptr;
            return true;
        }
        ++resource.statistics.cpuFramesSeen;
        const auto packed = capture::calculatePackedBgraLayout(
            frame.sourceWidth, frame.sourceHeight);
        const bool bufferValid = packed.has_value()
            && frame.cpuRowPitch >= packed->stride
            && frame.cpuPixels->size() >= static_cast<std::size_t>(frame.cpuRowPitch)
                * frame.sourceHeight;
        capture::DesktopUploadState state{
            resource.sequence, resource.width, resource.height, resource.format,
            resource.texture != nullptr};
        const auto action = capture::decideDesktopUpload(state, frame.sequence,
            frame.sourceWidth, frame.sourceHeight, frame.sourceFormat,
            frame.valid && bufferValid);
        if (action == capture::DesktopUploadAction::skipSameSequence)
        {
            ++resource.statistics.cpuFramesSkippedSameSequence;
            return true;
        }
        if (action == capture::DesktopUploadAction::invalid)
        {
            ++resource.statistics.updateSubresourceFailures;
            error_ = "Desktop source frame violates the upload texture contract.";
            return false;
        }
        ++resource.statistics.cpuFramesConsumed;
        resource.statistics.latestConsumedSequence = frame.sequence;
        if (action == capture::DesktopUploadAction::create
            || action == capture::DesktopUploadAction::recreate)
        {
            resource.sharedTexture.Reset();
            resource.keyedMutex.Reset();
            resource.surfaceIdentity = nullptr;
            resource.sharedHandle = nullptr;
            resource.recoveryGeneration = 0U;
            resource.shaderResource.Reset();
            resource.texture.Reset();
            D3D11_TEXTURE2D_DESC description{};
            description.Width = frame.sourceWidth;
            description.Height = frame.sourceHeight;
            description.MipLevels = 1U;
            description.ArraySize = 1U;
            description.Format = static_cast<DXGI_FORMAT>(frame.sourceFormat);
            description.SampleDesc.Count = 1U;
            description.Usage = D3D11_USAGE_DEFAULT;
            description.BindFlags = D3D11_BIND_SHADER_RESOURCE;
            HRESULT result = device_->CreateTexture2D(
                &description, nullptr, &resource.texture);
            if (FAILED(result))
            {
                error_ = "Creating per-source render upload texture failed: "
                    + hresultText(result);
                return false;
            }
            result = device_->CreateShaderResourceView(
                resource.texture.Get(), nullptr, &resource.shaderResource);
            if (FAILED(result))
            {
                ++resource.statistics.desktopSrvFailures;
                error_ = "Creating per-source desktop SRV failed: " + hresultText(result);
                return false;
            }
            if (action == capture::DesktopUploadAction::create)
            {
                ++resource.statistics.uploadTextureCreations;
                sceneStatistics_.resourcesCreatedAtStartup += 2U;
            }
            else
            {
                ++resource.statistics.uploadTextureRecreations;
                sceneStatistics_.resourcesCreatedSteadyState += 2U;
            }
            ++resource.statistics.desktopSrvCreations;
            resource.statistics.desktopSrvValid = true;
            resource.width = frame.sourceWidth;
            resource.height = frame.sourceHeight;
            resource.format = frame.sourceFormat;
        }
        const auto updateStart = std::chrono::steady_clock::now();
        context_->UpdateSubresource(resource.texture.Get(), 0U, nullptr,
            frame.cpuPixels->data(), frame.cpuRowPitch, 0U);
        resource.lastUploadMilliseconds = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - updateStart).count();
        ++resource.statistics.updateSubresourceCalls;
        resource.statistics.uploadBytesSubmitted += packed->bufferSize;
        resource.statistics.latestUploadBytes = packed->bufferSize;
        resource.statistics.averageUpdateSubresourceMilliseconds +=
            (resource.lastUploadMilliseconds
                - resource.statistics.averageUpdateSubresourceMilliseconds)
            / static_cast<double>(resource.statistics.updateSubresourceCalls);
        resource.statistics.maximumUpdateSubresourceMilliseconds = std::max(
            resource.statistics.maximumUpdateSubresourceMilliseconds,
            resource.lastUploadMilliseconds);
        const auto uploadEnd = std::chrono::steady_clock::now();
        if (resource.firstUploadTimestamp == std::chrono::steady_clock::time_point{})
        {
            resource.firstUploadTimestamp = uploadEnd;
        }
        const double uploadSeconds = std::chrono::duration<double>(
            uploadEnd - resource.firstUploadTimestamp).count();
        resource.statistics.uploadFramesPerSecond = uploadSeconds > 0.0
            ? static_cast<double>(resource.statistics.updateSubresourceCalls - 1U)
                / uploadSeconds : 0.0;
        ++sceneStatistics_.textureUploads;
        resource.sequence = frame.sequence;
        resource.filter = filter;
        resource.layout = capture::calculateDesktopTextureLayout(
            frame.sourceWidth, frame.sourceHeight, panelWidth_ / panelHeight_,
            fit, frame.rotation, flipY);
        resource.statistics.latestUploadedSequence = frame.sequence;
        resource.statistics.latestUploadWidth = frame.sourceWidth;
        resource.statistics.latestUploadHeight = frame.sourceHeight;
        resource.statistics.latestUploadFormat = frame.sourceFormat;
        resource.statistics.uploadTextureValid = resource.texture != nullptr;
        return true;
    }

    [[nodiscard]] bool present(bool vsync)
    {
        if (!vsync && frameLatencyWaitableObject_ != nullptr)
        {
            const DWORD wait = WaitForSingleObjectEx(frameLatencyWaitableObject_, 1000, FALSE);
            if (wait == WAIT_FAILED)
            {
                error_ = "Waiting for the DXGI frame-latency object failed with Win32 error "
                    + std::to_string(GetLastError()) + ".";
                return false;
            }
        }
        const HRESULT result = swapChain_->Present(vsync ? 1U : 0U, 0);
        if (FAILED(result))
        {
            error_ = "IDXGISwapChain::Present failed: " + hresultText(result);
            return false;
        }
        ++sceneStatistics_.presents;
        if (!desktopDrawEventLogPrinted_ && !desktopDrawEvents_.empty()
            && desktopDrawEvents_.back() == rendering::DesktopDrawEvent::shaderResourceUnbound)
        {
            desktopDrawEvents_.push_back(rendering::DesktopDrawEvent::presented);
            const bool overlayExpected = rendering::desktopOverlayEnabled(desktopShaderDebugMode_);
            if (!rendering::validateDesktopDrawOrdering(desktopDrawEvents_, overlayExpected))
            {
                error_ = "First desktop frame violated the required draw-call ordering.";
                return false;
            }
            std::cout << "Desktop first-frame draw ordering:\n";
            for (const auto event : desktopDrawEvents_)
            {
                std::cout << "  desktop-draw-event="
                          << rendering::desktopDrawEventText(event) << '\n';
            }
            desktopDrawEventLogPrinted_ = true;
        }
        return true;
    }

    HWND window_{};
    unsigned int width_{};
    unsigned int height_{};
    double panelDistance_{};
    double panelWidth_{};
    double panelHeight_{};
    D3D_FEATURE_LEVEL featureLevel_{};
    bool debugLayerUnavailable_{};
    bool fullscreen_{};
    rendering::DesktopShaderDebugMode desktopShaderDebugMode_{
        rendering::DesktopShaderDebugMode::normal};
    bool desktopDebugOpaqueBase_{};
    std::optional<std::string> desktopDebugReadbackUploadPath_;
    std::optional<std::string> desktopDebugRenderTargetPath_;
    HANDLE frameLatencyWaitableObject_{};
    D3D11RendererInformation information_;
    std::string error_;
    std::optional<platform::windows::DxgiAdapterLuid> requestedAdapterLuid_;
    std::string requestedOutputDeviceName_;
    ComPtr<ID3D11Device> device_;
    ComPtr<ID3D11DeviceContext> context_;
    ComPtr<IDXGIAdapter> adapter_;
    ComPtr<IDXGIOutput> selectedOutput_;
    ComPtr<IDXGISwapChain2> swapChain_;
    ComPtr<ID3D11RenderTargetView> renderTarget_;
    ComPtr<ID3D11Texture2D> depthTexture_;
    ComPtr<ID3D11DepthStencilView> depthView_;
    ComPtr<ID3D11VertexShader> vertexShader_;
    ComPtr<ID3D11PixelShader> pixelShader_;
    ComPtr<ID3D11PixelShader> overlayPixelShader_;
    ComPtr<ID3D11InputLayout> inputLayout_;
    ComPtr<ID3D11Buffer> constantBuffer_;
    ComPtr<ID3D11RasterizerState> rasterizerState_;
    ComPtr<ID3D11BlendState> blendState_;
    ComPtr<ID3D11BlendState> opaqueBlendState_;
    ComPtr<ID3D11DepthStencilState> depthStencilState_;
    ComPtr<ID3D11SamplerState> pointSampler_;
    ComPtr<ID3D11SamplerState> linearSampler_;
    ComPtr<ID3D11Texture2D> desktopSharedTexture_;
    ComPtr<IDXGIKeyedMutex> desktopKeyedMutex_;
    ComPtr<ID3D11Texture2D> desktopLocalTexture_;
    ComPtr<ID3D11ShaderResourceView> desktopShaderResource_;
    const capture::DesktopCaptureSurface* desktopSurfaceIdentity_{};
    void* desktopSharedHandle_{};
    std::uint64_t desktopRecoveryGeneration_{};
    std::uint64_t desktopFrameSequence_{};
    std::uint32_t desktopWidth_{};
    std::uint32_t desktopHeight_{};
    std::uint32_t desktopFormat_{};
    capture::DesktopFilter desktopFilter_{capture::DesktopFilter::linear};
    capture::DesktopTextureLayout desktopLayout_;
    capture::DesktopRenderStageStatistics desktopStatistics_;
    std::chrono::steady_clock::time_point desktopFirstUploadTimestamp_{};
    struct AdditionalDesktopSource
    {
        ComPtr<ID3D11Texture2D> texture;
        ComPtr<ID3D11ShaderResourceView> shaderResource;
        ComPtr<ID3D11Texture2D> sharedTexture;
        ComPtr<IDXGIKeyedMutex> keyedMutex;
        const capture::DesktopCaptureSurface* surfaceIdentity{};
        void* sharedHandle{};
        std::uint64_t recoveryGeneration{};
        std::uint64_t sequence{};
        std::uint32_t width{};
        std::uint32_t height{};
        std::uint32_t format{};
        capture::DesktopFilter filter{capture::DesktopFilter::linear};
        capture::DesktopTextureLayout layout;
        capture::DesktopRenderStageStatistics statistics;
        double lastUploadMilliseconds{};
        std::chrono::steady_clock::time_point firstUploadTimestamp{};
    };
    std::array<AdditionalDesktopSource, rendering::maximumPanelCount>
        additionalDesktopSources_{};
    D3D11SceneStatistics sceneStatistics_;
    std::uint64_t desktopUploadSourceChecksum_{};
    std::uint64_t uploadReadbackChecksum_{};
    std::uint64_t renderTargetReadbackChecksum_{};
    std::vector<rendering::DesktopDrawEvent> desktopDrawEvents_;
    bool desktopDescriptorsTraced_{};
    bool desktopConstantsTraced_{};
    bool desktopBlendStateTraced_{};
    bool uploadReadbackAttempted_{};
    bool renderTargetDumpAttempted_{};
    bool desktopDrawEventLogPrinted_{};
    bool firstCpuConsumedTraced_{};
    bool firstUploadTextureTraced_{};
    bool firstUpdateTraced_{};
    bool firstSrvCreatedTraced_{};
    bool firstSrvBindTraced_{};
    bool firstShaderPathTraced_{};
    bool firstDesktopRenderedTraced_{};
    ComPtr<ID3D11Buffer> panelBuffer_;
    ComPtr<ID3D11Buffer> panelIndexBuffer_;
    ComPtr<ID3D11Buffer> lineBuffer_;
    std::vector<Vertex> panelVertices_;
    std::vector<Vertex> lineVertices_;
    UINT baseLineVertexCount_{};
    UINT gridStartVertex_{};
    UINT gridVertexCount_{};
    UINT axesStartVertex_{};
    UINT axesVertexCount_{};
};

D3D11Renderer::D3D11Renderer() : implementation_(std::make_unique<Implementation>()) {}
D3D11Renderer::~D3D11Renderer() = default;
D3D11Renderer::D3D11Renderer(D3D11Renderer&&) noexcept = default;
D3D11Renderer& D3D11Renderer::operator=(D3D11Renderer&&) noexcept = default;

bool D3D11Renderer::initialize(void* window, const D3D11RendererConfig& config)
{
    return implementation_->initialize(window, config);
}

bool D3D11Renderer::resize(unsigned int width, unsigned int height)
{
    return implementation_->resize(width, height);
}

bool D3D11Renderer::updateDesktopFrame(
    const capture::DesktopCaptureFrame& frame,
    capture::DesktopFit fit,
    capture::DesktopFilter filter,
    bool flipY)
{
    return implementation_->updateDesktopFrame(frame, fit, filter, flipY);
}

bool D3D11Renderer::updateDesktopFrame(
    std::size_t sourceSlot,
    const capture::DesktopCaptureFrame& frame,
    capture::DesktopFit fit,
    capture::DesktopFilter filter,
    bool flipY)
{
    return implementation_->updateDesktopFrame(sourceSlot, frame, fit, filter, flipY);
}

bool D3D11Renderer::render(
    const rendering::Matrix4& matrix,
    bool grid,
    bool axes,
    bool ready,
    bool desktopContent,
    capture::DesktopBackground desktopBackground,
    bool desktopStale)
{
    return implementation_->render(
        matrix, grid, axes, ready, desktopContent, desktopBackground, desktopStale);
}

bool D3D11Renderer::renderScene(
    std::span<const rendering::PanelRenderInstance> panels,
    bool grid,
    bool axes,
    bool ready,
    capture::DesktopBackground desktopBackground)
{
    return implementation_->renderScene(panels, grid, axes, ready, desktopBackground);
}

bool D3D11Renderer::present(bool vsync) { return implementation_->present(vsync); }

const D3D11RendererInformation& D3D11Renderer::information() const noexcept
{
    return implementation_->information_;
}

capture::DesktopRenderStageStatistics D3D11Renderer::desktopStatistics() const noexcept
{
    return implementation_->desktopStatistics_;
}

capture::DesktopRenderStageStatistics D3D11Renderer::desktopStatistics(
    std::size_t sourceSlot) const noexcept
{
    if (sourceSlot == 0U)
    {
        return implementation_->desktopStatistics_;
    }
    if (sourceSlot < rendering::maximumPanelCount)
    {
        return implementation_->additionalDesktopSources_[sourceSlot].statistics;
    }
    return {};
}

D3D11SceneStatistics D3D11Renderer::sceneStatistics() const noexcept
{
    return implementation_->sceneStatistics_;
}

const std::string& D3D11Renderer::error() const noexcept { return implementation_->error_; }

} // namespace xreal::graphics
