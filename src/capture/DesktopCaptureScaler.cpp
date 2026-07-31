#include "capture/DesktopCaptureScaler.hpp"

#include "capture/DesktopCpuFrame.hpp"

#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>

#include <d3d11.h>
#include <d3dcompiler.h>
#include <wrl/client.h>

#include <array>
#include <chrono>
#include <cstring>
#include <mutex>
#include <sstream>
#include <utility>

namespace xreal::capture
{
namespace
{

using Microsoft::WRL::ComPtr;

constexpr const char* desktopScaleShaderSource = R"(
Texture2D SourceTexture : register(t0);
SamplerState SourceSampler : register(s0);

cbuffer ScaleConstants : register(b0)
{
    float4 SourceRectangle;
};

struct ScaleVertexOutput
{
    float4 position : SV_Position;
    float2 uv : TEXCOORD0;
};

ScaleVertexOutput ScaleVertex(uint vertexId : SV_VertexID)
{
    ScaleVertexOutput output;
    output.uv = float2((vertexId << 1) & 2, vertexId & 2);
    output.position = float4(output.uv.x * 2.0 - 1.0,
        1.0 - output.uv.y * 2.0, 0.0, 1.0);
    return output;
}

float4 ScalePixel(ScaleVertexOutput input) : SV_Target
{
    const float2 sourceUv = SourceRectangle.xy + input.uv * SourceRectangle.zw;
    return SourceTexture.SampleLevel(SourceSampler, sourceUv, 0.0);
}
)";

[[nodiscard]] std::string hresultText(HRESULT value)
{
    std::ostringstream output;
    output << "HRESULT 0x" << std::hex << std::uppercase
           << static_cast<unsigned long>(value);
    return output.str();
}

struct CompiledScaleShaders
{
    CompiledScaleShaders()
    {
        constexpr UINT flags = D3DCOMPILE_ENABLE_STRICTNESS
#ifndef NDEBUG
            | D3DCOMPILE_DEBUG | D3DCOMPILE_SKIP_OPTIMIZATION
#else
            | D3DCOMPILE_OPTIMIZATION_LEVEL3
#endif
            ;
        ComPtr<ID3DBlob> errors;
        HRESULT result = D3DCompile(desktopScaleShaderSource,
            std::strlen(desktopScaleShaderSource), "DesktopScale.hlsl", nullptr, nullptr,
            "ScaleVertex", "vs_5_0", flags, 0U, &vertex, &errors);
        if (FAILED(result))
        {
            error = "Compiling DesktopScale ScaleVertex failed: " + hresultText(result);
            if (errors)
            {
                error += " ";
                error.append(static_cast<const char*>(errors->GetBufferPointer()),
                    errors->GetBufferSize());
            }
            return;
        }
        errors.Reset();
        result = D3DCompile(desktopScaleShaderSource,
            std::strlen(desktopScaleShaderSource), "DesktopScale.hlsl", nullptr, nullptr,
            "ScalePixel", "ps_5_0", flags, 0U, &pixel, &errors);
        if (FAILED(result))
        {
            error = "Compiling DesktopScale ScalePixel failed: " + hresultText(result);
            if (errors)
            {
                error += " ";
                error.append(static_cast<const char*>(errors->GetBufferPointer()),
                    errors->GetBufferSize());
            }
        }
    }

    ComPtr<ID3DBlob> vertex;
    ComPtr<ID3DBlob> pixel;
    std::string error;
};

[[nodiscard]] const CompiledScaleShaders& compiledScaleShaders()
{
    // Function-local immutable state guarantees one shader compilation per process.
    static const CompiledScaleShaders shaders;
    return shaders;
}

struct ScaleConstants
{
    std::array<float, 4> sourceRectangle{};
};

} // namespace

class DesktopCaptureScaler::Implementation
{
public:
    [[nodiscard]] bool initialize(void* d3dDevice, void* immediateContext)
    {
        if (d3dDevice == nullptr || immediateContext == nullptr)
        {
            error_ = "DesktopCaptureScaler requires a device and its owning immediate context.";
            return false;
        }
        device_ = static_cast<ID3D11Device*>(d3dDevice);
        context_ = static_cast<ID3D11DeviceContext*>(immediateContext);
        const auto& bytecode = compiledScaleShaders();
        if (!bytecode.error.empty())
        {
            error_ = bytecode.error;
            return false;
        }
        HRESULT result = device_->CreateVertexShader(bytecode.vertex->GetBufferPointer(),
            bytecode.vertex->GetBufferSize(), nullptr, &vertexShader_);
        if (FAILED(result))
        {
            error_ = "Creating capture scale vertex shader failed: " + hresultText(result);
            return false;
        }
        result = device_->CreatePixelShader(bytecode.pixel->GetBufferPointer(),
            bytecode.pixel->GetBufferSize(), nullptr, &pixelShader_);
        if (FAILED(result))
        {
            error_ = "Creating capture scale pixel shader failed: " + hresultText(result);
            return false;
        }
        D3D11_SAMPLER_DESC sampler{};
        sampler.Filter = D3D11_FILTER_MIN_MAG_MIP_POINT;
        sampler.AddressU = D3D11_TEXTURE_ADDRESS_CLAMP;
        sampler.AddressV = D3D11_TEXTURE_ADDRESS_CLAMP;
        sampler.AddressW = D3D11_TEXTURE_ADDRESS_CLAMP;
        sampler.MaxLOD = D3D11_FLOAT32_MAX;
        result = device_->CreateSamplerState(&sampler, &pointSampler_);
        if (FAILED(result))
        {
            error_ = "Creating capture point sampler failed: " + hresultText(result);
            return false;
        }
        sampler.Filter = D3D11_FILTER_MIN_MAG_MIP_LINEAR;
        result = device_->CreateSamplerState(&sampler, &linearSampler_);
        if (FAILED(result))
        {
            error_ = "Creating capture linear sampler failed: " + hresultText(result);
            return false;
        }
        statistics_.shaderCompilations = 1U;
        initialized_ = true;
        return true;
    }

    void reset()
    {
        releaseFrameResources();
        pointSampler_.Reset();
        linearSampler_.Reset();
        vertexShader_.Reset();
        pixelShader_.Reset();
        context_.Reset();
        device_.Reset();
        initialized_ = false;
    }

    [[nodiscard]] bool ensureResources(
        const D3D11_TEXTURE2D_DESC& source,
        const DesktopScalePlan& plan)
    {
        if (ownedSource_ && source.Width == sourceWidth_ && source.Height == sourceHeight_
            && source.Format == format_ && plan.targetWidth == targetWidth_
            && plan.targetHeight == targetHeight_ && plan.crop == crop_
            && plan.gpuScaleRequired == gpuScaleRequired_)
        {
            return true;
        }
        releaseFrameResources();
        if (source.Format != DXGI_FORMAT_B8G8R8A8_UNORM)
        {
            error_ = "Capture scaling supports DXGI_FORMAT_B8G8R8A8_UNORM only; "
                "implicit sRGB or HDR conversion is forbidden.";
            return false;
        }
        D3D11_TEXTURE2D_DESC owned{};
        owned.Width = source.Width;
        owned.Height = source.Height;
        owned.MipLevels = 1U;
        owned.ArraySize = 1U;
        owned.Format = source.Format;
        owned.SampleDesc.Count = 1U;
        owned.Usage = D3D11_USAGE_DEFAULT;
        owned.BindFlags = D3D11_BIND_SHADER_RESOURCE;
        HRESULT result = device_->CreateTexture2D(&owned, nullptr, &ownedSource_);
        if (FAILED(result))
        {
            error_ = "Creating capture-owned source texture failed: " + hresultText(result);
            return false;
        }
        result = device_->CreateShaderResourceView(ownedSource_.Get(), nullptr, &sourceView_);
        if (FAILED(result))
        {
            error_ = "Creating capture-owned source SRV failed: " + hresultText(result);
            return false;
        }

        ID3D11Texture2D* readbackSource = ownedSource_.Get();
        if (plan.gpuScaleRequired)
        {
            D3D11_TEXTURE2D_DESC scaled = owned;
            scaled.Width = plan.targetWidth;
            scaled.Height = plan.targetHeight;
            scaled.BindFlags = D3D11_BIND_RENDER_TARGET;
            result = device_->CreateTexture2D(&scaled, nullptr, &scaledTarget_);
            if (FAILED(result))
            {
                error_ = "Creating capture scale render target failed: " + hresultText(result);
                return false;
            }
            result = device_->CreateRenderTargetView(scaledTarget_.Get(), nullptr, &scaledView_);
            if (FAILED(result))
            {
                error_ = "Creating capture scale RTV failed: " + hresultText(result);
                return false;
            }
            readbackSource = scaledTarget_.Get();
            ScaleConstants constants{{
                static_cast<float>(plan.crop.x) / source.Width,
                static_cast<float>(plan.crop.y) / source.Height,
                static_cast<float>(plan.crop.width) / source.Width,
                static_cast<float>(plan.crop.height) / source.Height,
            }};
            D3D11_BUFFER_DESC constantDescription{};
            constantDescription.ByteWidth = sizeof(ScaleConstants);
            constantDescription.Usage = D3D11_USAGE_IMMUTABLE;
            constantDescription.BindFlags = D3D11_BIND_CONSTANT_BUFFER;
            D3D11_SUBRESOURCE_DATA constantData{&constants, 0U, 0U};
            result = device_->CreateBuffer(&constantDescription, &constantData, &constants_);
            if (FAILED(result))
            {
                error_ = "Creating capture scale constants failed: " + hresultText(result);
                return false;
            }
        }

        D3D11_TEXTURE2D_DESC staging{};
        readbackSource->GetDesc(&staging);
        staging.Usage = D3D11_USAGE_STAGING;
        staging.BindFlags = 0U;
        staging.CPUAccessFlags = D3D11_CPU_ACCESS_READ;
        staging.MiscFlags = 0U;
        for (auto& texture : stagingTextures_)
        {
            result = device_->CreateTexture2D(&staging, nullptr, &texture);
            if (FAILED(result))
            {
                error_ = "Creating fixed capture staging ring failed: " + hresultText(result);
                return false;
            }
        }
        const auto packed = calculatePackedBgraLayout(plan.targetWidth, plan.targetHeight);
        if (!packed.has_value())
        {
            error_ = "Capture scale target overflowed the bounded CPU buffer.";
            return false;
        }
        for (auto& buffer : cpuBuffers_)
        {
            buffer = std::make_shared<std::vector<std::byte>>(packed->bufferSize);
        }
        sourceWidth_ = source.Width;
        sourceHeight_ = source.Height;
        targetWidth_ = plan.targetWidth;
        targetHeight_ = plan.targetHeight;
        format_ = source.Format;
        crop_ = plan.crop;
        gpuScaleRequired_ = plan.gpuScaleRequired;
        nextSlot_ = 0U;
        ++statistics_.resourceRecreations;
        return true;
    }

    [[nodiscard]] DesktopScaleReadback process(
        void* sourceTexture,
        const DesktopScalePlan& plan,
        DesktopFilter filter)
    {
        DesktopScaleReadback output;
        if (!initialized_ || sourceTexture == nullptr)
        {
            output.error = "Desktop capture scaler is not initialized or has no source texture.";
            return output;
        }
        auto* source = static_cast<ID3D11Texture2D*>(sourceTexture);
        D3D11_TEXTURE2D_DESC sourceDescription{};
        source->GetDesc(&sourceDescription);
        if (!ensureResources(sourceDescription, plan))
        {
            output.error = error_;
            return output;
        }

        std::size_t selectedSlot = desktopStagingRingCapacity;
        for (std::size_t attempt = 0U; attempt < desktopStagingRingCapacity; ++attempt)
        {
            const std::size_t slot = (nextSlot_ + attempt) % desktopStagingRingCapacity;
            if (cpuBuffers_[slot].use_count() == 1)
            {
                selectedSlot = slot;
                break;
            }
        }
        if (selectedSlot == desktopStagingRingCapacity)
        {
            ++statistics_.ringContentions;
            output.success = true;
            return output;
        }
        nextSlot_ = (selectedSlot + 1U) % desktopStagingRingCapacity;
        const auto submissionStart = std::chrono::steady_clock::now();
        context_->CopyResource(ownedSource_.Get(), source);
        ID3D11Texture2D* readbackSource = ownedSource_.Get();
        if (gpuScaleRequired_)
        {
            D3D11_VIEWPORT viewport{};
            viewport.Width = static_cast<float>(targetWidth_);
            viewport.Height = static_cast<float>(targetHeight_);
            viewport.MaxDepth = 1.0F;
            context_->RSSetViewports(1U, &viewport);
            ID3D11RenderTargetView* target = scaledView_.Get();
            context_->OMSetRenderTargets(1U, &target, nullptr);
            context_->IASetInputLayout(nullptr);
            context_->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
            context_->VSSetShader(vertexShader_.Get(), nullptr, 0U);
            context_->PSSetShader(pixelShader_.Get(), nullptr, 0U);
            ID3D11ShaderResourceView* sourceResource = sourceView_.Get();
            context_->PSSetShaderResources(0U, 1U, &sourceResource);
            ID3D11SamplerState* sampler = filter == DesktopFilter::point
                ? pointSampler_.Get() : linearSampler_.Get();
            context_->PSSetSamplers(0U, 1U, &sampler);
            ID3D11Buffer* constant = constants_.Get();
            context_->PSSetConstantBuffers(0U, 1U, &constant);
            context_->Draw(3U, 0U);
            ID3D11ShaderResourceView* nullResource{};
            context_->PSSetShaderResources(0U, 1U, &nullResource);
            ID3D11RenderTargetView* nullTarget{};
            context_->OMSetRenderTargets(1U, &nullTarget, nullptr);
            readbackSource = scaledTarget_.Get();
            ++statistics_.gpuScaleDraws;
        }
        context_->CopyResource(stagingTextures_[selectedSlot].Get(), readbackSource);
        const auto submissionEnd = std::chrono::steady_clock::now();
        ++statistics_.submissions;
        ++statistics_.stagingCopies;

        const auto mapStart = std::chrono::steady_clock::now();
        D3D11_MAPPED_SUBRESOURCE mapped{};
        const HRESULT mappedResult = context_->Map(
            stagingTextures_[selectedSlot].Get(), 0U, D3D11_MAP_READ, 0U, &mapped);
        const auto mapEnd = std::chrono::steady_clock::now();
        ++statistics_.stagingMaps;
        if (FAILED(mappedResult))
        {
            output.error = "Mapping capture scale staging slot failed: "
                + hresultText(mappedResult);
            error_ = output.error;
            return output;
        }
        const auto repackStart = std::chrono::steady_clock::now();
        const auto packed = calculatePackedBgraLayout(targetWidth_, targetHeight_);
        const bool copied = packed.has_value() && repackBgraRows(
            static_cast<const std::byte*>(mapped.pData),
            static_cast<std::size_t>(mapped.RowPitch) * targetHeight_, mapped.RowPitch,
            *cpuBuffers_[selectedSlot], targetWidth_, targetHeight_);
        context_->Unmap(stagingTextures_[selectedSlot].Get(), 0U);
        const auto repackEnd = std::chrono::steady_clock::now();
        if (!copied)
        {
            output.error = "Repacking capture scale BGRA rows failed.";
            error_ = output.error;
            return output;
        }
        output.success = true;
        output.frameReady = true;
        output.pixels = cpuBuffers_[selectedSlot];
        output.width = targetWidth_;
        output.height = targetHeight_;
        output.rowPitch = packed->stride;
        output.format = static_cast<std::uint32_t>(format_);
        output.stagingSlot = selectedSlot;
        output.mappedBytes = packed->bufferSize;
        output.gpuSubmissionMilliseconds = std::chrono::duration<double, std::milli>(
            submissionEnd - submissionStart).count();
        output.mapWaitMilliseconds = std::chrono::duration<double, std::milli>(
            mapEnd - mapStart).count();
        output.cpuRepackMilliseconds = std::chrono::duration<double, std::milli>(
            repackEnd - repackStart).count();
        return output;
    }

    [[nodiscard]] DesktopCaptureScalerStatistics statistics() const noexcept
    {
        return statistics_;
    }

    [[nodiscard]] const std::string& error() const noexcept { return error_; }

private:
    void releaseFrameResources()
    {
        for (auto& texture : stagingTextures_) { texture.Reset(); }
        for (auto& buffer : cpuBuffers_) { buffer.reset(); }
        constants_.Reset();
        scaledView_.Reset();
        scaledTarget_.Reset();
        sourceView_.Reset();
        ownedSource_.Reset();
        sourceWidth_ = 0U;
        sourceHeight_ = 0U;
        targetWidth_ = 0U;
        targetHeight_ = 0U;
        format_ = DXGI_FORMAT_UNKNOWN;
        crop_ = {};
        gpuScaleRequired_ = false;
    }

    ComPtr<ID3D11Device> device_;
    ComPtr<ID3D11DeviceContext> context_;
    ComPtr<ID3D11VertexShader> vertexShader_;
    ComPtr<ID3D11PixelShader> pixelShader_;
    ComPtr<ID3D11SamplerState> pointSampler_;
    ComPtr<ID3D11SamplerState> linearSampler_;
    ComPtr<ID3D11Texture2D> ownedSource_;
    ComPtr<ID3D11ShaderResourceView> sourceView_;
    ComPtr<ID3D11Texture2D> scaledTarget_;
    ComPtr<ID3D11RenderTargetView> scaledView_;
    ComPtr<ID3D11Buffer> constants_;
    std::array<ComPtr<ID3D11Texture2D>, desktopStagingRingCapacity> stagingTextures_;
    std::array<std::shared_ptr<std::vector<std::byte>>, desktopStagingRingCapacity> cpuBuffers_;
    std::uint32_t sourceWidth_{};
    std::uint32_t sourceHeight_{};
    std::uint32_t targetWidth_{};
    std::uint32_t targetHeight_{};
    DXGI_FORMAT format_{DXGI_FORMAT_UNKNOWN};
    DesktopCaptureRegion crop_;
    bool gpuScaleRequired_{};
    std::size_t nextSlot_{};
    bool initialized_{};
    DesktopCaptureScalerStatistics statistics_;
    std::string error_;
};

DesktopCaptureScaler::DesktopCaptureScaler()
    : implementation_(std::make_unique<Implementation>())
{
}

DesktopCaptureScaler::~DesktopCaptureScaler() = default;
DesktopCaptureScaler::DesktopCaptureScaler(DesktopCaptureScaler&&) noexcept = default;
DesktopCaptureScaler& DesktopCaptureScaler::operator=(DesktopCaptureScaler&&) noexcept = default;

bool DesktopCaptureScaler::initialize(void* d3dDevice, void* immediateContext)
{
    return implementation_->initialize(d3dDevice, immediateContext);
}

DesktopScaleReadback DesktopCaptureScaler::process(
    void* sourceTexture,
    const DesktopScalePlan& plan,
    DesktopFilter filter)
{
    return implementation_->process(sourceTexture, plan, filter);
}

DesktopCaptureScalerStatistics DesktopCaptureScaler::statistics() const noexcept
{
    return implementation_->statistics();
}

const std::string& DesktopCaptureScaler::error() const noexcept
{
    return implementation_->error();
}

void DesktopCaptureScaler::reset() { implementation_->reset(); }

} // namespace xreal::capture
