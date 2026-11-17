#include "VirtualDisplayTypes.hpp"

#include <charconv>

namespace xreal::virtual_display
{

VirtualDisplayValidation validateVirtualDisplayConfiguration(
    const VirtualDisplayConfiguration& configuration)
{
    const auto count = virtualDisplayCountValue(configuration.count);
    if (count < 1U || count > maximumVirtualDisplayCount)
    {
        return {false, VirtualDisplayErrorCode::invalidCount,
            "Virtual display count must be between one and three."};
    }
    if (configuration.mode != VirtualDisplayMode::extended
        && configuration.mode != VirtualDisplayMode::mirrored)
    {
        return {false, VirtualDisplayErrorCode::invalidMode,
            "Virtual display mode must be extended or mirrored."};
    }
    if (configuration.resolution.width != defaultVirtualDisplayWidth
        || configuration.resolution.height != defaultVirtualDisplayHeight)
    {
        return {false, VirtualDisplayErrorCode::unsupportedResolution,
            "The prototype supports only 1920x1080 virtual displays."};
    }
    if (configuration.resolution.refreshHertz != defaultVirtualDisplayRefreshHertz)
    {
        return {false, VirtualDisplayErrorCode::unsupportedRefreshRate,
            "The prototype supports only 60 Hz virtual displays."};
    }
    return {true, VirtualDisplayErrorCode::none, {}};
}

std::string stableVirtualMonitorId(std::size_t logicalIndex)
{
    static constexpr std::array<std::string_view, maximumVirtualDisplayCount> ids{
        "7d5529c1-9b47-4fc4-a201-000000000001",
        "7d5529c1-9b47-4fc4-a201-000000000002",
        "7d5529c1-9b47-4fc4-a201-000000000003",
    };
    if (logicalIndex >= ids.size())
    {
        return {};
    }
    return std::string(ids[logicalIndex]);
}

std::string virtualMonitorFriendlyName(std::size_t logicalIndex)
{
    if (logicalIndex >= maximumVirtualDisplayCount)
    {
        return {};
    }
    return "XREAL Virtual Display " + std::to_string(logicalIndex + 1U);
}

std::array<VirtualMonitorDescriptor, maximumVirtualDisplayCount>
buildVirtualMonitorDescriptors(
    const VirtualDisplayConfiguration& configuration,
    std::size_t& descriptorCount)
{
    std::array<VirtualMonitorDescriptor, maximumVirtualDisplayCount> result{};
    descriptorCount = 0U;
    if (!configuration.enabled)
    {
        return result;
    }
    const auto validation = validateVirtualDisplayConfiguration(configuration);
    if (!validation.valid)
    {
        return result;
    }
    descriptorCount = configuration.mode == VirtualDisplayMode::mirrored
        ? 1U : virtualDisplayCountValue(configuration.count);
    for (std::size_t index = 0U; index < descriptorCount; ++index)
    {
        result[index].logicalIndex = static_cast<std::uint8_t>(index);
        result[index].connectorIndex = static_cast<std::uint8_t>(index);
        result[index].stableId = stableVirtualMonitorId(index);
        result[index].friendlyName = virtualMonitorFriendlyName(index);
        result[index].resolution = configuration.resolution;
        result[index].active = true;
        result[index].topologyMode = configuration.mode;
    }
    return result;
}

std::optional<VirtualDisplayCount> parseVirtualDisplayCount(std::string_view value) noexcept
{
    unsigned int parsed{};
    const auto conversion = std::from_chars(value.data(), value.data() + value.size(), parsed);
    if (conversion.ec != std::errc{} || conversion.ptr != value.data() + value.size())
    {
        return std::nullopt;
    }
    switch (parsed)
    {
    case 1U: return VirtualDisplayCount::one;
    case 2U: return VirtualDisplayCount::two;
    case 3U: return VirtualDisplayCount::three;
    default: return std::nullopt;
    }
}

std::optional<VirtualDisplayMode> parseVirtualDisplayMode(std::string_view value) noexcept
{
    if (value == "extended")
    {
        return VirtualDisplayMode::extended;
    }
    if (value == "mirrored")
    {
        return VirtualDisplayMode::mirrored;
    }
    return std::nullopt;
}

std::string_view virtualDisplayModeText(VirtualDisplayMode value) noexcept
{
    switch (value)
    {
    case VirtualDisplayMode::extended: return "extended";
    case VirtualDisplayMode::mirrored: return "mirrored";
    }
    return "invalid";
}

std::string_view virtualDisplayStateText(VirtualDisplayState value) noexcept
{
    switch (value)
    {
    case VirtualDisplayState::disabled: return "disabled";
    case VirtualDisplayState::applying: return "applying";
    case VirtualDisplayState::active: return "active";
    case VirtualDisplayState::partial: return "partial";
    case VirtualDisplayState::failed: return "failed";
    case VirtualDisplayState::restartRequired: return "restart_required";
    }
    return "invalid";
}

bool sameRequestedTopology(
    const VirtualDisplayConfiguration& left,
    const VirtualDisplayConfiguration& right) noexcept
{
    return left.enabled == right.enabled
        && left.count == right.count
        && left.mode == right.mode
        && left.resolution == right.resolution;
}

} // namespace xreal::virtual_display
