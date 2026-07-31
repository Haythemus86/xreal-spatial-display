#include "rendering/PanelScene.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace xreal::rendering
{
namespace
{

[[nodiscard]] Matrix4 translation(Vector3 value) noexcept
{
    Matrix4 result = Matrix4::identity();
    result.at(0, 3) = value.x;
    result.at(1, 3) = value.y;
    result.at(2, 3) = value.z;
    return result;
}

[[nodiscard]] Matrix4 scale(double width, double height) noexcept
{
    Matrix4 result = Matrix4::identity();
    result.at(0, 0) = width;
    result.at(1, 1) = height;
    return result;
}

[[nodiscard]] Matrix4 yawPitch(double yawDegrees, double pitchDegrees) noexcept
{
    const double yaw = yawDegrees * std::numbers::pi / 180.0;
    const double pitch = pitchDegrees * std::numbers::pi / 180.0;
    const double cosineYaw = std::cos(yaw);
    const double sineYaw = std::sin(yaw);
    const double cosinePitch = std::cos(pitch);
    const double sinePitch = std::sin(pitch);

    Matrix4 yawMatrix = Matrix4::identity();
    yawMatrix.at(0, 0) = cosineYaw;
    yawMatrix.at(0, 2) = sineYaw;
    yawMatrix.at(2, 0) = -sineYaw;
    yawMatrix.at(2, 2) = cosineYaw;

    Matrix4 pitchMatrix = Matrix4::identity();
    pitchMatrix.at(1, 1) = cosinePitch;
    pitchMatrix.at(1, 2) = -sinePitch;
    pitchMatrix.at(2, 1) = sinePitch;
    pitchMatrix.at(2, 2) = cosinePitch;
    return yawMatrix * pitchMatrix;
}

void initializeIds(PanelScene& scene) noexcept
{
    for (std::size_t index = 0; index < maximumPanelCount; ++index)
    {
        scene.panels[index].id = PanelId{static_cast<std::uint8_t>(index)};
    }
}

} // namespace

PanelSceneController::PanelSceneController(PanelScene& scene) noexcept : scene_(scene) {}

bool PanelSceneController::select(std::size_t index) noexcept
{
    if (index >= scene_.panelCount)
    {
        return false;
    }
    scene_.selectedPanel = index;
    return true;
}

bool PanelSceneController::resizeSelected(double widthDelta, double heightDelta) noexcept
{
    auto& dimensions = scene_.panels[scene_.selectedPanel].dimensions;
    const double width = dimensions.width + widthDelta;
    const double height = dimensions.height + heightDelta;
    if (!std::isfinite(width) || !std::isfinite(height) || width <= 0.05 || height <= 0.05)
    {
        return false;
    }
    dimensions = {width, height};
    scene_.layout = PanelLayoutPreset::custom;
    scene_.runtime[scene_.selectedPanel].worldTransform =
        panelWorldMatrix(scene_.panels[scene_.selectedPanel]);
    return true;
}

bool PanelSceneController::moveSelected(Vector3 delta) noexcept
{
    auto& position = scene_.panels[scene_.selectedPanel].transform.position;
    const Vector3 moved{position.x + delta.x, position.y + delta.y, position.z + delta.z};
    if (!moved.finite() || moved.z >= -0.05)
    {
        return false;
    }
    position = moved;
    scene_.layout = PanelLayoutPreset::custom;
    scene_.runtime[scene_.selectedPanel].worldTransform =
        panelWorldMatrix(scene_.panels[scene_.selectedPanel]);
    return true;
}

bool PanelSceneController::rotateSelected(
    double yawDeltaDegrees,
    double pitchDeltaDegrees) noexcept
{
    auto& transform = scene_.panels[scene_.selectedPanel].transform;
    const double yaw = transform.yawDegrees + yawDeltaDegrees;
    const double pitch = transform.pitchDegrees + pitchDeltaDegrees;
    if (!std::isfinite(yaw) || !std::isfinite(pitch))
    {
        return false;
    }
    transform.yawDegrees = std::remainder(yaw, 360.0);
    transform.pitchDegrees = std::clamp(pitch, -89.0, 89.0);
    scene_.layout = PanelLayoutPreset::custom;
    scene_.runtime[scene_.selectedPanel].worldTransform =
        panelWorldMatrix(scene_.panels[scene_.selectedPanel]);
    return true;
}

bool PanelSceneController::toggleSelected() noexcept
{
    auto& enabled = scene_.panels[scene_.selectedPanel].enabled;
    enabled = !enabled;
    return enabled;
}

bool PanelSceneController::applyPreset(PanelLayoutPreset preset) noexcept
{
    return applyPanelLayout(scene_, preset);
}

bool PanelSceneController::resetSelected()
{
    PanelScene reference = scene_;
    const PanelLayoutPreset referencePreset = scene_.layout != PanelLayoutPreset::custom
        ? scene_.layout
        : scene_.panelCount == 1U ? PanelLayoutPreset::single
        : scene_.panelCount == 2U ? PanelLayoutPreset::dualFlat
                                  : PanelLayoutPreset::tripleAngled;
    if (!applyPanelLayout(reference, referencePreset))
    {
        return false;
    }
    auto& panel = scene_.panels[scene_.selectedPanel];
    panel.transform = reference.panels[scene_.selectedPanel].transform;
    panel.dimensions = reference.panels[scene_.selectedPanel].dimensions;
    panel.enabled = true;
    scene_.runtime[scene_.selectedPanel].worldTransform = panelWorldMatrix(panel);
    scene_.layout = PanelLayoutPreset::custom;
    return true;
}

void PanelSceneController::reset()
{
    const PanelLayoutPreset preset = scene_.panelCount == 1U
        ? PanelLayoutPreset::single
        : scene_.panelCount == 2U ? PanelLayoutPreset::dualFlat
                                  : PanelLayoutPreset::tripleAngled;
    (void)applyPanelLayout(scene_, preset);
    scene_.selectedPanel = 0U;
}

PanelScene makeDefaultPanelScene(std::size_t panelCount)
{
    PanelScene result;
    result.panelCount = std::clamp(panelCount, std::size_t{1U}, maximumPanelCount);
    initializeIds(result);
    constexpr std::array<std::string_view, maximumPanelCount> names{
        "Panel 1", "Panel 2", "Panel 3"};
    for (std::size_t index = 0; index < maximumPanelCount; ++index)
    {
        result.panels[index].displayName = names[index];
    }
    const PanelLayoutPreset preset = result.panelCount == 1U
        ? PanelLayoutPreset::single
        : result.panelCount == 2U ? PanelLayoutPreset::dualFlat
                                  : PanelLayoutPreset::tripleAngled;
    (void)applyPanelLayout(result, preset);
    return result;
}

bool applyPanelLayout(PanelScene& scene, PanelLayoutPreset preset) noexcept
{
    if (scene.panelCount == 0U || scene.panelCount > maximumPanelCount
        || !std::isfinite(scene.defaultWidth) || scene.defaultWidth <= 0.05
        || !std::isfinite(scene.defaultHeight) || scene.defaultHeight <= 0.05
        || !std::isfinite(scene.defaultDistance) || scene.defaultDistance <= 0.05
        || !std::isfinite(scene.gap) || scene.gap < 0.0
        || !std::isfinite(scene.curvatureDegrees))
    {
        return false;
    }
    if (preset == PanelLayoutPreset::custom)
    {
        scene.layout = preset;
        return true;
    }
    initializeIds(scene);
    const double spacing = scene.defaultWidth + scene.gap;
    for (std::size_t index = 0; index < maximumPanelCount; ++index)
    {
        auto& panel = scene.panels[index];
        panel.enabled = index < scene.panelCount;
        panel.dimensions = {scene.defaultWidth, scene.defaultHeight};
        panel.transform = {{0.0, 0.0, -scene.defaultDistance}, 0.0, 0.0};
    }
    if (preset == PanelLayoutPreset::single || scene.panelCount == 1U)
    {
        scene.panels[0].transform.position.x = 0.0;
    }
    else if (preset == PanelLayoutPreset::dualFlat || scene.panelCount == 2U)
    {
        scene.panels[0].transform.position.x = -spacing * 0.5;
        scene.panels[1].transform.position.x = spacing * 0.5;
    }
    else
    {
        scene.panels[0].transform.position.x = -spacing;
        scene.panels[1].transform.position.x = 0.0;
        scene.panels[2].transform.position.x = spacing;
        if (preset == PanelLayoutPreset::tripleAngled)
        {
            scene.panels[0].transform.yawDegrees = scene.curvatureDegrees;
            scene.panels[2].transform.yawDegrees = -scene.curvatureDegrees;
            const double sideDistance = scene.defaultDistance
                + std::sin(std::abs(scene.curvatureDegrees) * std::numbers::pi / 180.0)
                    * scene.defaultWidth * 0.25;
            scene.panels[0].transform.position.z = -sideDistance;
            scene.panels[2].transform.position.z = -sideDistance;
        }
    }
    scene.layout = preset;
    scene.selectedPanel = std::min(scene.selectedPanel, scene.panelCount - 1U);
    refreshPanelWorldTransforms(scene);
    return true;
}

PanelSceneValidation validatePanelScene(const PanelScene& scene)
{
    if (scene.panelCount == 0U || scene.panelCount > maximumPanelCount)
    {
        return {false, "Panel count must be between 1 and 3."};
    }
    if (scene.selectedPanel >= scene.panelCount)
    {
        return {false, "Selected panel index is outside the active panel range."};
    }
    if (!std::isfinite(scene.defaultWidth) || scene.defaultWidth <= 0.05
        || scene.defaultWidth > 20.0
        || !std::isfinite(scene.defaultHeight) || scene.defaultHeight <= 0.05
        || scene.defaultHeight > 20.0
        || !std::isfinite(scene.defaultDistance) || scene.defaultDistance <= 0.05
        || scene.defaultDistance > 100.0
        || !std::isfinite(scene.gap) || scene.gap < 0.0 || scene.gap > 20.0
        || !std::isfinite(scene.curvatureDegrees)
        || std::abs(scene.curvatureDegrees) > 90.0)
    {
        return {false, "Panel layout defaults are outside the supported finite bounds."};
    }
    for (std::size_t index = 0; index < scene.panelCount; ++index)
    {
        const auto& panel = scene.panels[index];
        const bool displayNameValid = !panel.displayName.empty()
            && panel.displayName.size() <= 64U
            && std::ranges::all_of(panel.displayName,
                [](unsigned char character) { return character >= 0x20U; });
        if (panel.id.value != index)
        {
            return {false, "Panel identifiers must be stable and contiguous."};
        }
        if (!displayNameValid
            || !panel.transform.position.finite() || !std::isfinite(panel.transform.yawDegrees)
            || !std::isfinite(panel.transform.pitchDegrees)
            || !std::isfinite(panel.dimensions.width) || panel.dimensions.width <= 0.05
            || !std::isfinite(panel.dimensions.height) || panel.dimensions.height <= 0.05
            || panel.dimensions.width > 20.0 || panel.dimensions.height > 20.0
            || std::abs(panel.transform.position.x) > 100.0
            || std::abs(panel.transform.position.y) > 100.0
            || panel.transform.position.z >= -0.05
            || panel.transform.position.z < -100.0
            || !std::isfinite(panel.targetFramesPerSecond)
            || panel.targetFramesPerSecond < 0.0
            || panel.targetFramesPerSecond > 1000.0
            || !std::isfinite(panel.content.requestedScale)
            || panel.content.requestedScale <= 0.0
            || panel.content.requestedScale > 4.0
            || panel.content.requestedWidth > 16384U
            || panel.content.requestedHeight > 16384U)
        {
            return {false, "Panel geometry and rate values must be finite and valid."};
        }
    }
    return {true, {}};
}

Matrix4 panelWorldMatrix(const PanelDefinition& panel) noexcept
{
    return translation(panel.transform.position)
        * yawPitch(panel.transform.yawDegrees, panel.transform.pitchDegrees)
        * scale(panel.dimensions.width, panel.dimensions.height);
}

void refreshPanelWorldTransforms(PanelScene& scene) noexcept
{
    for (std::size_t index = 0; index < maximumPanelCount; ++index)
    {
        scene.runtime[index].worldTransform = panelWorldMatrix(scene.panels[index]);
    }
}

bool panelPotentiallyVisible(const PanelDefinition& panel) noexcept
{
    return panel.enabled && panel.dimensions.width > 0.05 && panel.dimensions.height > 0.05
        && std::isfinite(panel.transform.yawDegrees)
        && std::isfinite(panel.transform.pitchDegrees)
        && panel.transform.position.finite() && panel.transform.position.z < -0.05;
}

std::string_view panelContentKindText(PanelContentKind value) noexcept
{
    switch (value)
    {
    case PanelContentKind::synthetic: return "synthetic";
    case PanelContentKind::checkerboard: return "checkerboard";
    case PanelContentKind::desktop: return "desktop";
    case PanelContentKind::unavailable: return "unavailable";
    }
    return "unavailable";
}

std::optional<PanelContentKind> parsePanelContentKind(std::string_view value) noexcept
{
    if (value == "synthetic") { return PanelContentKind::synthetic; }
    if (value == "checkerboard") { return PanelContentKind::checkerboard; }
    if (value == "desktop") { return PanelContentKind::desktop; }
    if (value == "unavailable") { return PanelContentKind::unavailable; }
    return std::nullopt;
}

std::string_view panelLayoutPresetText(PanelLayoutPreset value) noexcept
{
    switch (value)
    {
    case PanelLayoutPreset::single: return "single";
    case PanelLayoutPreset::dualFlat: return "dual-flat";
    case PanelLayoutPreset::tripleFlat: return "triple-flat";
    case PanelLayoutPreset::tripleAngled: return "triple-angled";
    case PanelLayoutPreset::custom: return "custom";
    }
    return "custom";
}

std::optional<PanelLayoutPreset> parsePanelLayoutPreset(std::string_view value) noexcept
{
    if (value == "single") { return PanelLayoutPreset::single; }
    if (value == "dual-flat") { return PanelLayoutPreset::dualFlat; }
    if (value == "triple-flat") { return PanelLayoutPreset::tripleFlat; }
    if (value == "triple-angled") { return PanelLayoutPreset::tripleAngled; }
    if (value == "custom") { return PanelLayoutPreset::custom; }
    return std::nullopt;
}

std::string_view panelFitModeText(PanelFitMode value) noexcept
{
    switch (value)
    {
    case PanelFitMode::contain: return "contain";
    case PanelFitMode::cover: return "cover";
    case PanelFitMode::stretch: return "stretch";
    }
    return "contain";
}

std::optional<PanelFitMode> parsePanelFitMode(std::string_view value) noexcept
{
    if (value == "contain") { return PanelFitMode::contain; }
    if (value == "cover") { return PanelFitMode::cover; }
    if (value == "stretch") { return PanelFitMode::stretch; }
    return std::nullopt;
}

std::string_view panelFilterModeText(PanelFilterMode value) noexcept
{
    switch (value)
    {
    case PanelFilterMode::point: return "point";
    case PanelFilterMode::linear: return "linear";
    }
    return "linear";
}

std::optional<PanelFilterMode> parsePanelFilterMode(std::string_view value) noexcept
{
    if (value == "point") { return PanelFilterMode::point; }
    if (value == "linear") { return PanelFilterMode::linear; }
    return std::nullopt;
}

std::string_view panelTransferPolicyText(PanelTransferPolicy value) noexcept
{
    switch (value)
    {
    case PanelTransferPolicy::automatic: return "auto";
    case PanelTransferPolicy::sharedHandle: return "shared-handle";
    case PanelTransferPolicy::cpuFallback: return "cpu-fallback";
    case PanelTransferPolicy::reject: return "reject";
    }
    return "auto";
}

std::optional<PanelTransferPolicy> parsePanelTransferPolicy(
    std::string_view value) noexcept
{
    if (value == "auto") { return PanelTransferPolicy::automatic; }
    if (value == "shared-handle") { return PanelTransferPolicy::sharedHandle; }
    if (value == "cpu-fallback") { return PanelTransferPolicy::cpuFallback; }
    if (value == "reject") { return PanelTransferPolicy::reject; }
    return std::nullopt;
}

std::string_view performanceProfileText(PerformanceProfile value) noexcept
{
    switch (value)
    {
    case PerformanceProfile::quality: return "quality";
    case PerformanceProfile::balanced: return "balanced";
    case PerformanceProfile::performance: return "performance";
    }
    return "balanced";
}

std::optional<PerformanceProfile> parsePerformanceProfile(std::string_view value) noexcept
{
    if (value == "quality") { return PerformanceProfile::quality; }
    if (value == "balanced") { return PerformanceProfile::balanced; }
    if (value == "performance") { return PerformanceProfile::performance; }
    return std::nullopt;
}

} // namespace xreal::rendering
