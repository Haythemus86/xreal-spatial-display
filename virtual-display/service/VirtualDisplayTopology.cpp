#include "VirtualDisplayTopology.hpp"

#include "rendering/PanelContentRegistry.hpp"

#include <utility>

namespace xreal::virtual_display
{

VirtualDisplayTopologyResult routeVirtualDisplaysToPanels(
    const VirtualDisplayStatus& status,
    rendering::PanelScene& scene)
{
    VirtualDisplayTopologyResult result;
    if (!status.requested.enabled || status.state == VirtualDisplayState::disabled)
    {
        for (auto& panel : scene.panels)
        {
            panel.enabled = false;
        }
        result.success = true;
        return result;
    }
    if (status.state != VirtualDisplayState::active
        && status.state != VirtualDisplayState::partial)
    {
        result.error = "Virtual display topology is not active.";
        return result;
    }
    const std::size_t panelCount = virtualDisplayCountValue(status.requested.count);
    const std::size_t requiredMonitors = status.requested.mode == VirtualDisplayMode::mirrored
        ? 1U : panelCount;
    if (status.monitorCount < requiredMonitors)
    {
        result.error = "The active virtual monitor set is incomplete.";
        return result;
    }

    rendering::PanelScene candidate = scene;
    candidate.panelCount = panelCount;
    if (candidate.selectedPanel >= panelCount)
    {
        candidate.selectedPanel = 0U;
    }
    for (std::size_t index = 0U; index < candidate.panels.size(); ++index)
    {
        auto& panel = candidate.panels[index];
        panel.enabled = index < panelCount;
        if (!panel.enabled)
        {
            continue;
        }
        const std::size_t monitorIndex = status.requested.mode == VirtualDisplayMode::mirrored
            ? 0U : index;
        panel.content.kind = rendering::PanelContentKind::desktop;
        panel.content.captureMonitorIndex.reset();
        const auto& monitor = status.monitors[monitorIndex];
        if (!monitor.windowsMonitorIdentity.empty())
        {
            panel.content.captureMonitorStableId = monitor.windowsMonitorIdentity;
            panel.content.captureMonitorDeviceName.reset();
        }
        else if (!monitor.windowsDeviceName.empty())
        {
            panel.content.captureMonitorStableId.reset();
            panel.content.captureMonitorDeviceName = monitor.windowsDeviceName;
        }
        else
        {
            result.error = "Windows has not assigned a capture identity to virtual monitor "
                + monitor.friendlyName + ". Re-enumerate topology before routing it.";
            return result;
        }
    }
    rendering::PanelContentRegistry registry;
    const auto registryResult = registry.rebuild(candidate);
    if (!registryResult.success)
    {
        result.error = registryResult.error;
        return result;
    }
    scene = std::move(candidate);
    result.success = true;
    result.panelCount = panelCount;
    result.sourceCount = registry.sourceCount();
    for (std::size_t index = 0U; index < panelCount; ++index)
    {
        result.bindings[index] = {
            scene.panels[index].id,
            scene.panels[index].content.captureMonitorStableId.value_or(
                scene.panels[index].content.captureMonitorDeviceName.value_or("")),
            scene.runtime[index].sourceSlot,
        };
    }
    return result;
}

} // namespace xreal::virtual_display
