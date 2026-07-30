#include "capture/DesktopCaptureOptions.hpp"

namespace xreal::capture
{

MonitorResolution resolveMonitor(
    std::span<const platform::windows::MonitorInformation> monitors,
    const MonitorSelector& selector,
    std::string_view role)
{
    if (selector.deviceName.has_value())
    {
        for (const auto& monitor : monitors)
        {
            if (monitor.deviceName == *selector.deviceName)
            {
                if (!monitor.dxgiOutput.has_value())
                {
                    return {nullptr, std::string(role) + " monitor " + monitor.deviceName
                        + " has no matching active DXGI output."};
                }
                return {&monitor, {}};
            }
        }
        return {nullptr, std::string(role) + " monitor device name "
            + *selector.deviceName + " was not found; no fallback index was selected."};
    }
    if (!selector.index.has_value())
    {
        return {nullptr, std::string(role) + " monitor selection is required."};
    }
    if (*selector.index >= monitors.size())
    {
        return {nullptr, std::string(role) + " monitor index "
            + std::to_string(*selector.index) + " does not exist."};
    }
    const auto& monitor = monitors[*selector.index];
    if (!monitor.dxgiOutput.has_value())
    {
        return {nullptr, std::string(role) + " monitor " + monitor.deviceName
            + " has no matching active DXGI output."};
    }
    return {&monitor, {}};
}

bool sameAdapter(
    const platform::windows::MonitorInformation& first,
    const platform::windows::MonitorInformation& second) noexcept
{
    return first.dxgiOutput.has_value() && second.dxgiOutput.has_value()
        && first.dxgiOutput->adapterLuid == second.dxgiOutput->adapterLuid;
}

std::string panelContentText(PanelContent value)
{
    return value == PanelContent::desktop ? "desktop" : "synthetic";
}

std::string desktopFitText(DesktopFit value)
{
    switch (value)
    {
    case DesktopFit::contain: return "contain";
    case DesktopFit::cover: return "cover";
    case DesktopFit::stretch: return "stretch";
    }
    return "unknown";
}

std::string desktopFilterText(DesktopFilter value)
{
    return value == DesktopFilter::point ? "point" : "linear";
}

std::string crossAdapterPolicyText(CrossAdapterPolicy value)
{
    switch (value)
    {
    case CrossAdapterPolicy::automatic: return "auto";
    case CrossAdapterPolicy::sharedHandle: return "shared-handle";
    case CrossAdapterPolicy::cpuFallback: return "cpu-fallback";
    case CrossAdapterPolicy::reject: return "reject";
    }
    return "unknown";
}

} // namespace xreal::capture
