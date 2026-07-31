#pragma once

#include "capture/DesktopCaptureFrame.hpp"
#include "capture/DesktopCaptureOptions.hpp"
#include "capture/DesktopRenderStages.hpp"
#include "platform/windows/DisplayTopology.hpp"
#include "rendering/DesktopPanelDiagnostics.hpp"
#include "rendering/RenderMath.hpp"
#include "rendering/PanelScene.hpp"

#include <memory>
#include <optional>
#include <span>
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
    rendering::DesktopShaderDebugMode desktopShaderDebugMode{
        rendering::DesktopShaderDebugMode::normal};
    bool desktopDebugOpaqueBase{};
    std::optional<std::string> desktopDebugReadbackUploadPath;
    std::optional<std::string> desktopDebugRenderTargetPath;
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

struct D3D11SceneStatistics
{
    std::uint64_t frames{};
    std::uint64_t presents{};
    std::uint64_t drawCalls{};
    std::uint64_t baseDrawCalls{};
    std::uint64_t overlayDrawCalls{};
    std::uint64_t auxiliaryDrawCalls{};
    std::uint64_t stateSetCalls{};
    std::uint64_t shaderResourceBindCalls{};
    std::uint64_t samplerBindCalls{};
    std::uint64_t constantBufferUpdates{};
    std::uint64_t instanceBufferUpdates{};
    std::uint64_t visiblePanels{};
    std::uint64_t culledPanels{};
    std::uint64_t textureUploads{};
    std::uint64_t resourcesCreatedAtStartup{};
    std::uint64_t resourcesCreatedSteadyState{};
    std::uint64_t flushCalls{};
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
    [[nodiscard]] bool updateDesktopFrame(
        const capture::DesktopCaptureFrame& frame,
        capture::DesktopFit fit,
        capture::DesktopFilter filter,
        bool flipY);
    [[nodiscard]] bool render(
        const rendering::Matrix4& viewProjection,
        bool backgroundGrid,
        bool worldAxes,
        bool ready,
        bool desktopContent = false,
        capture::DesktopBackground desktopBackground = capture::DesktopBackground::black,
        bool desktopStale = false);
    [[nodiscard]] bool renderScene(
        std::span<const rendering::PanelRenderInstance> panels,
        bool backgroundGrid,
        bool worldAxes,
        bool ready,
        capture::DesktopBackground desktopBackground = capture::DesktopBackground::black);
    [[nodiscard]] bool present(bool vsync);
    [[nodiscard]] const D3D11RendererInformation& information() const noexcept;
    [[nodiscard]] capture::DesktopRenderStageStatistics desktopStatistics() const noexcept;
    [[nodiscard]] D3D11SceneStatistics sceneStatistics() const noexcept;
    [[nodiscard]] const std::string& error() const noexcept;

private:
    class Implementation;
    std::unique_ptr<Implementation> implementation_;
};

} // namespace xreal::graphics
