#pragma once

#include "platform/windows/DisplayTopology.hpp"
#include "rendering/RenderMath.hpp"

#include <memory>
#include <optional>
#include <string>

namespace xreal::graphics
{

struct D3D11RendererConfig
{
    unsigned int width{};
    unsigned int height{};
    bool allowWarpFallback{};
    bool fullscreen{};
    std::optional<platform::windows::DxgiAdapterLuid> selectedAdapterLuid;
    std::string selectedOutputDeviceName;
    double panelDistance{2.0};
    double panelWidth{1.6};
    double panelHeight{0.9};
};

struct D3D11RendererInformation
{
    std::string adapterName;
    std::optional<platform::windows::DxgiAdapterLuid> adapterLuid;
    std::string outputDeviceName;
    std::string featureLevel;
    std::string swapChainFormat{"DXGI_FORMAT_B8G8R8A8_UNORM"};
    bool warp{};
    bool debugLayer{};
    bool frameLatencyWaitableObject{};
    bool selectedAdapterRequested{};
    bool selectedAdapterMatched{};
    bool selectedOutputMatched{};
    bool exclusiveFullscreen{};
};

class D3D11Renderer
{
public:
    D3D11Renderer();
    ~D3D11Renderer();
    D3D11Renderer(const D3D11Renderer&) = delete;
    D3D11Renderer& operator=(const D3D11Renderer&) = delete;
    D3D11Renderer(D3D11Renderer&&) noexcept;
    D3D11Renderer& operator=(D3D11Renderer&&) noexcept;

    [[nodiscard]] bool initialize(void* nativeWindow, const D3D11RendererConfig& config);
    [[nodiscard]] bool resize(unsigned int width, unsigned int height);
    [[nodiscard]] bool render(
        const rendering::Matrix4& viewProjection,
        bool backgroundGrid,
        bool worldAxes,
        bool ready);
    [[nodiscard]] bool present(bool vsync);
    [[nodiscard]] const D3D11RendererInformation& information() const noexcept;
    [[nodiscard]] const std::string& error() const noexcept;

private:
    class Implementation;
    std::unique_ptr<Implementation> implementation_;
};

} // namespace xreal::graphics
