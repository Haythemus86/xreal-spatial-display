#pragma once

#include "rendering/RenderMath.hpp"
#include "capture/DesktopCaptureScaling.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>

namespace xreal::rendering
{

inline constexpr std::size_t maximumPanelCount = 3U;

struct PanelId
{
    std::uint8_t value{};

    [[nodiscard]] friend constexpr bool operator==(PanelId, PanelId) noexcept = default;
};

enum class PanelContentKind
{
    synthetic,
    checkerboard,
    desktop,
    unavailable,
};

enum class PanelLayoutPreset
{
    single,
    dualFlat,
    tripleFlat,
    tripleAngled,
    custom,
};

enum class PerformanceProfile
{
    quality,
    balanced,
    performance,
};

enum class PanelFitMode
{
    contain,
    cover,
    stretch,
};

enum class PanelFilterMode
{
    point,
    linear,
};

enum class PanelTransferPolicy
{
    automatic,
    sharedHandle,
    cpuFallback,
    reject,
};

struct PanelOverlaySettings
{
    bool enabled{true};
};

struct PanelTransform
{
    Vector3 position{0.0, 0.0, -2.0};
    double yawDegrees{};
    double pitchDegrees{};
};

struct PanelDimensions
{
    double width{1.6};
    double height{0.9};
};

struct PanelContentSource
{
    PanelContentKind kind{PanelContentKind::synthetic};
    std::optional<unsigned int> captureMonitorIndex;
    std::optional<std::string> captureMonitorDeviceName;
    unsigned int requestedWidth{};
    unsigned int requestedHeight{};
    double requestedScale{1.0};
    capture::DesktopScaleRequest scaling;
    double requestedUploadFramesPerSecond{30.0};
    bool requestedUploadFramesPerSecondExplicit{};
    std::uint8_t captureBenchmarkInstance{};
    PanelTransferPolicy transferPolicy{PanelTransferPolicy::automatic};
};

struct PanelDefinition
{
    PanelId id{};
    std::string displayName{"Panel"};
    bool enabled{true};
    PanelTransform transform;
    PanelDimensions dimensions;
    PanelContentSource content;
    PanelFitMode fit{PanelFitMode::contain};
    PanelFilterMode filter{PanelFilterMode::linear};
    PanelOverlaySettings overlay;
    double targetFramesPerSecond{30.0};
    bool targetFramesPerSecondExplicit{};
};

struct PanelRuntime
{
    bool visible{true};
    bool culled{};
    bool contentAvailable{true};
    bool stale{};
    PanelContentKind effectiveContent{PanelContentKind::synthetic};
    std::size_t sourceSlot{};
    std::uint64_t latestContentSequence{};
    std::uint64_t latestUploadSequence{};
    std::uint64_t renderedFrames{};
    double frameAgeMilliseconds{};
    Matrix4 worldTransform{Matrix4::identity()};
};

struct PanelRenderInstance
{
    PanelId id{};
    Matrix4 worldViewProjection{Matrix4::identity()};
    PanelDimensions dimensions;
    PanelContentKind content{PanelContentKind::synthetic};
    PanelFitMode fit{PanelFitMode::contain};
    PanelFilterMode filter{PanelFilterMode::linear};
    PanelOverlaySettings overlay;
    std::size_t sourceSlot{};
    bool visible{true};
    bool stale{};
    bool selected{};
};

struct PanelScene
{
    std::array<PanelDefinition, maximumPanelCount> panels{};
    std::array<PanelRuntime, maximumPanelCount> runtime{};
    std::size_t panelCount{1U};
    std::size_t selectedPanel{};
    PanelLayoutPreset layout{PanelLayoutPreset::single};
    PerformanceProfile performanceProfile{PerformanceProfile::balanced};
    double defaultWidth{1.6};
    double defaultHeight{0.9};
    double defaultDistance{2.0};
    double gap{0.12};
    double curvatureDegrees{18.0};
};

struct PanelSceneValidation
{
    bool valid{};
    std::string error;
};

class PanelSceneController
{
public:
    explicit PanelSceneController(PanelScene& scene) noexcept;

    [[nodiscard]] bool select(std::size_t index) noexcept;
    [[nodiscard]] bool resizeSelected(double widthDelta, double heightDelta) noexcept;
    [[nodiscard]] bool moveSelected(Vector3 delta) noexcept;
    [[nodiscard]] bool rotateSelected(double yawDeltaDegrees, double pitchDeltaDegrees) noexcept;
    [[nodiscard]] bool toggleSelected() noexcept;
    [[nodiscard]] bool applyPreset(PanelLayoutPreset preset) noexcept;
    [[nodiscard]] bool resetSelected();
    void reset();

private:
    PanelScene& scene_;
};

[[nodiscard]] PanelScene makeDefaultPanelScene(std::size_t panelCount = 1U);
[[nodiscard]] bool applyPanelLayout(PanelScene& scene, PanelLayoutPreset preset) noexcept;
[[nodiscard]] PanelSceneValidation validatePanelScene(const PanelScene& scene);
[[nodiscard]] Matrix4 panelWorldMatrix(const PanelDefinition& panel) noexcept;
void refreshPanelWorldTransforms(PanelScene& scene) noexcept;
[[nodiscard]] bool panelPotentiallyVisible(const PanelDefinition& panel) noexcept;
[[nodiscard]] std::string_view panelContentKindText(PanelContentKind value) noexcept;
[[nodiscard]] std::optional<PanelContentKind> parsePanelContentKind(std::string_view value) noexcept;
[[nodiscard]] std::string_view panelLayoutPresetText(PanelLayoutPreset value) noexcept;
[[nodiscard]] std::optional<PanelLayoutPreset> parsePanelLayoutPreset(std::string_view value) noexcept;
[[nodiscard]] std::string_view panelFitModeText(PanelFitMode value) noexcept;
[[nodiscard]] std::optional<PanelFitMode> parsePanelFitMode(std::string_view value) noexcept;
[[nodiscard]] std::string_view panelFilterModeText(PanelFilterMode value) noexcept;
[[nodiscard]] std::optional<PanelFilterMode> parsePanelFilterMode(std::string_view value) noexcept;
[[nodiscard]] std::string_view panelTransferPolicyText(PanelTransferPolicy value) noexcept;
[[nodiscard]] std::optional<PanelTransferPolicy> parsePanelTransferPolicy(
    std::string_view value) noexcept;
[[nodiscard]] std::string_view performanceProfileText(PerformanceProfile value) noexcept;
[[nodiscard]] std::optional<PerformanceProfile> parsePerformanceProfile(std::string_view value) noexcept;

} // namespace xreal::rendering
