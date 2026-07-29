#include "graphics/D3D11Renderer.hpp"

#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>

#include <d3d11_1.h>
#include <d3dcompiler.h>
#include <dxgi1_3.h>
#include <wrl/client.h>

#include <array>
#include <cstdint>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <utility>
#include <vector>

namespace xreal::graphics
{
namespace
{

using Microsoft::WRL::ComPtr;

struct Vertex
{
    float x{};
    float y{};
    float z{};
    float red{};
    float green{};
    float blue{};
    float alpha{1.0F};
};

struct ConstantBuffer
{
    std::array<float, 16> viewProjection{};
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
        return true;
    }

    [[nodiscard]] bool createPipeline()
    {
        ComPtr<ID3DBlob> vertexBytecode;
        ComPtr<ID3DBlob> pixelBytecode;
        if (!compileShader("VSMain", "vs_5_0", vertexBytecode)
            || !compileShader("PSMain", "ps_5_0", pixelBytecode))
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
        constexpr std::array layout{
            D3D11_INPUT_ELEMENT_DESC{"POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0,
                D3D11_INPUT_PER_VERTEX_DATA, 0},
            D3D11_INPUT_ELEMENT_DESC{"COLOR", 0, DXGI_FORMAT_R32G32B32A32_FLOAT, 0, 12,
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
        constantDescription.ByteWidth = sizeof(ConstantBuffer);
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
        const float halfWidth = static_cast<float>(panelWidth_ * 0.5);
        const float halfHeight = static_cast<float>(panelHeight_ * 0.5);
        const float z = static_cast<float>(-panelDistance_);
        panelVertices_ = {
            {-halfWidth, -halfHeight, z, 0.04F, 0.08F, 0.16F, 0.88F},
            {-halfWidth, halfHeight, z, 0.04F, 0.08F, 0.16F, 0.88F},
            {halfWidth, -halfHeight, z, 0.04F, 0.08F, 0.16F, 0.88F},
            {halfWidth, halfHeight, z, 0.04F, 0.08F, 0.16F, 0.88F},
        };
        if (!createVertexBuffer(panelVertices_, panelBuffer_))
        {
            return false;
        }
        constexpr std::array<std::uint16_t, 6> panelIndices{0, 1, 2, 2, 1, 3};
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

        const float front = z + 0.002F;
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

    [[nodiscard]] bool render(
        const rendering::Matrix4& viewProjection,
        bool backgroundGrid,
        bool worldAxes,
        bool ready)
    {
        if (!viewProjection.finite())
        {
            error_ = "Refusing to render a non-finite view-projection matrix.";
            return false;
        }
        D3D11_MAPPED_SUBRESOURCE mapped{};
        HRESULT result = context_->Map(constantBuffer_.Get(), 0, D3D11_MAP_WRITE_DISCARD, 0, &mapped);
        if (FAILED(result))
        {
            error_ = "ID3D11DeviceContext::Map(constant buffer) failed: " + hresultText(result);
            return false;
        }
        ConstantBuffer constants;
        for (std::size_t index = 0; index < constants.viewProjection.size(); ++index)
        {
            constants.viewProjection[index] = static_cast<float>(viewProjection.values[index]);
        }
        *static_cast<ConstantBuffer*>(mapped.pData) = constants;
        context_->Unmap(constantBuffer_.Get(), 0);

        const std::array clearColor = ready
            ? std::array{0.005F, 0.008F, 0.018F, 1.0F}
            : std::array{0.16F, 0.07F, 0.01F, 1.0F};
        context_->ClearRenderTargetView(renderTarget_.Get(), clearColor.data());
        context_->ClearDepthStencilView(depthView_.Get(),
            D3D11_CLEAR_DEPTH | D3D11_CLEAR_STENCIL, 1.0F, 0);
        ID3D11RenderTargetView* target = renderTarget_.Get();
        context_->OMSetRenderTargets(1, &target, depthView_.Get());
        context_->IASetInputLayout(inputLayout_.Get());
        context_->VSSetShader(vertexShader_.Get(), nullptr, 0);
        context_->PSSetShader(pixelShader_.Get(), nullptr, 0);
        ID3D11Buffer* constant = constantBuffer_.Get();
        context_->VSSetConstantBuffers(0, 1, &constant);
        context_->RSSetState(rasterizerState_.Get());
        const std::array blendFactor{0.0F, 0.0F, 0.0F, 0.0F};
        context_->OMSetBlendState(blendState_.Get(), blendFactor.data(), 0xFFFFFFFFU);
        context_->OMSetDepthStencilState(depthStencilState_.Get(), 0);
        constexpr UINT stride = sizeof(Vertex);
        constexpr UINT offset = 0;
        ID3D11Buffer* panel = panelBuffer_.Get();
        context_->IASetVertexBuffers(0, 1, &panel, &stride, &offset);
        context_->IASetIndexBuffer(panelIndexBuffer_.Get(), DXGI_FORMAT_R16_UINT, 0);
        context_->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
        context_->DrawIndexed(6, 0, 0);
        ID3D11Buffer* lines = lineBuffer_.Get();
        context_->IASetVertexBuffers(0, 1, &lines, &stride, &offset);
        context_->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_LINELIST);
        context_->Draw(baseLineVertexCount_, 0);
        if (backgroundGrid)
        {
            context_->Draw(gridVertexCount_, gridStartVertex_);
        }
        if (worldAxes)
        {
            context_->Draw(axesVertexCount_, axesStartVertex_);
        }
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
    ComPtr<ID3D11InputLayout> inputLayout_;
    ComPtr<ID3D11Buffer> constantBuffer_;
    ComPtr<ID3D11RasterizerState> rasterizerState_;
    ComPtr<ID3D11BlendState> blendState_;
    ComPtr<ID3D11DepthStencilState> depthStencilState_;
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

bool D3D11Renderer::render(
    const rendering::Matrix4& matrix,
    bool grid,
    bool axes,
    bool ready)
{
    return implementation_->render(matrix, grid, axes, ready);
}

bool D3D11Renderer::present(bool vsync) { return implementation_->present(vsync); }

const D3D11RendererInformation& D3D11Renderer::information() const noexcept
{
    return implementation_->information_;
}

const std::string& D3D11Renderer::error() const noexcept { return implementation_->error_; }

} // namespace xreal::graphics
