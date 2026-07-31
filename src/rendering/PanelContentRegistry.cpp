#include "rendering/PanelContentRegistry.hpp"

#include <algorithm>
#include <cmath>

namespace xreal::rendering
{

PanelSourceRegistryResult PanelContentRegistry::rebuild(PanelScene& scene)
{
    clear();
    for (std::size_t panelIndex = 0; panelIndex < scene.panelCount; ++panelIndex)
    {
        const auto& panel = scene.panels[panelIndex];
        if (!panelPotentiallyVisible(panel))
        {
            scene.runtime[panelIndex].visible = false;
            continue;
        }
        scene.runtime[panelIndex].visible = true;
        const auto key = makePanelSourceKey(panel);
        auto slot = find(key);
        if (!slot.has_value())
        {
            if (sourceCount_ >= sources_.size())
            {
                return {false, "The fixed-capacity panel source registry is full."};
            }
            slot = sourceCount_++;
            sources_[*slot].key = key;
            sources_[*slot].active = true;
        }
        auto& sourceEntry = sources_[*slot];
        sourceEntry.consumerMask |= static_cast<std::uint8_t>(1U << panelIndex);
        sourceEntry.requestedFramesPerSecond = std::max(
            sourceEntry.requestedFramesPerSecond,
            defaultPanelSourceRate(scene.performanceProfile,
                panelIndex == scene.selectedPanel, true,
                panel.targetFramesPerSecond,
                panel.targetFramesPerSecondExplicit));
        scene.runtime[panelIndex].sourceSlot = *slot;
    }
    return {true, {}};
}

std::size_t PanelContentRegistry::sourceCount() const noexcept
{
    return sourceCount_;
}

const PanelSourceEntry& PanelContentRegistry::source(std::size_t slot) const noexcept
{
    return sources_[slot];
}

PanelSourceEntry& PanelContentRegistry::source(std::size_t slot) noexcept
{
    return sources_[slot];
}

std::optional<std::size_t> PanelContentRegistry::find(const PanelSourceKey& key) const noexcept
{
    for (std::size_t index = 0; index < sourceCount_; ++index)
    {
        if (sources_[index].active && sources_[index].key == key)
        {
            return index;
        }
    }
    return std::nullopt;
}

void PanelContentRegistry::clear() noexcept
{
    sources_ = {};
    sourceCount_ = 0U;
}

PanelSourceKey makePanelSourceKey(const PanelDefinition& panel)
{
    PanelSourceKey result;
    result.kind = panel.content.kind;
    result.monitorIndex = panel.content.captureMonitorIndex;
    result.requestedWidth = panel.content.requestedWidth;
    result.requestedHeight = panel.content.requestedHeight;
    result.requestedScale = panel.content.requestedScale;
    result.transferPolicy = panel.content.transferPolicy;
    if (panel.content.captureMonitorDeviceName.has_value())
    {
        result.monitorDeviceName = *panel.content.captureMonitorDeviceName;
    }
    // Synthetic and checkerboard content is generated once per content kind.
    if (result.kind != PanelContentKind::desktop)
    {
        result.monitorIndex.reset();
        result.monitorDeviceName.clear();
        result.requestedWidth = 0U;
        result.requestedHeight = 0U;
        result.requestedScale = 1.0;
        result.transferPolicy = PanelTransferPolicy::automatic;
    }
    return result;
}

double defaultPanelSourceRate(
    PerformanceProfile profile,
    bool selected,
    bool visible,
    double configuredRate,
    bool explicitRate) noexcept
{
    if (!visible || !std::isfinite(configuredRate) || configuredRate <= 0.0)
    {
        return 0.0;
    }
    if (explicitRate)
    {
        return configuredRate;
    }
    const double policyRate = selected ? 30.0
        : profile == PerformanceProfile::quality ? 30.0
        : profile == PerformanceProfile::performance ? 15.0 : 20.0;
    return std::min(configuredRate, policyRate);
}

double defaultPanelSourceRate(
    bool selected,
    bool visible,
    double configuredRate) noexcept
{
    return defaultPanelSourceRate(
        PerformanceProfile::balanced, selected, visible, configuredRate, false);
}

} // namespace xreal::rendering
