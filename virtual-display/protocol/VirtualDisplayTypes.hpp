#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>

namespace xreal::virtual_display
{

inline constexpr std::size_t maximumVirtualDisplayCount = 3U;
inline constexpr std::uint32_t defaultVirtualDisplayWidth = 1920U;
inline constexpr std::uint32_t defaultVirtualDisplayHeight = 1080U;
inline constexpr std::uint32_t defaultVirtualDisplayRefreshHertz = 60U;

enum class VirtualDisplayCount : std::uint8_t
{
    one = 1U,
    two = 2U,
    three = 3U,
};

enum class VirtualDisplayMode : std::uint8_t
{
    extended = 1U,
    mirrored = 2U,
};

enum class VirtualDisplayState : std::uint8_t
{
    disabled = 0U,
    applying = 1U,
    active = 2U,
    partial = 3U,
    failed = 4U,
    restartRequired = 5U,
};

enum class VirtualDisplayErrorCode : std::uint32_t
{
    none = 0U,
    invalidProtocol = 1U,
    invalidCount = 2U,
    invalidMode = 3U,
    unsupportedResolution = 4U,
    unsupportedRefreshRate = 5U,
    malformedBuffer = 6U,
    driverUnavailable = 7U,
    driverRejected = 8U,
    platformFailure = 9U,
};

struct VirtualDisplayResolution
{
    std::uint32_t width{defaultVirtualDisplayWidth};
    std::uint32_t height{defaultVirtualDisplayHeight};
    std::uint32_t refreshHertz{defaultVirtualDisplayRefreshHertz};

    [[nodiscard]] friend constexpr bool operator==(
        const VirtualDisplayResolution&,
        const VirtualDisplayResolution&) noexcept = default;
};

struct VirtualDisplayConfiguration
{
    bool enabled{};
    VirtualDisplayCount count{VirtualDisplayCount::one};
    VirtualDisplayMode mode{VirtualDisplayMode::extended};
    VirtualDisplayResolution resolution;
    std::uint64_t applyGeneration{};

    [[nodiscard]] friend constexpr bool operator==(
        const VirtualDisplayConfiguration&,
        const VirtualDisplayConfiguration&) noexcept = default;
};

struct VirtualMonitorDescriptor
{
    std::uint8_t logicalIndex{};
    std::uint8_t connectorIndex{};
    std::string stableId;
    std::string friendlyName;
    std::string windowsDeviceName;
    // DISPLAYCONFIG_TARGET_DEVICE_NAME::monitorDevicePath, populated by the
    // user-mode discovery layer after Windows activates the target.
    std::string windowsMonitorIdentity;
    VirtualDisplayResolution resolution;
    bool active{};
    VirtualDisplayMode topologyMode{VirtualDisplayMode::extended};

    [[nodiscard]] friend bool operator==(
        const VirtualMonitorDescriptor&,
        const VirtualMonitorDescriptor&) noexcept = default;
};

struct VirtualDisplayStatus
{
    VirtualDisplayConfiguration requested;
    VirtualDisplayConfiguration actual;
    VirtualDisplayState state{VirtualDisplayState::disabled};
    bool restartRequired{};
    VirtualDisplayErrorCode errorCode{VirtualDisplayErrorCode::none};
    std::string lastError;
    std::array<VirtualMonitorDescriptor, maximumVirtualDisplayCount> monitors{};
    std::size_t monitorCount{};
};

struct VirtualDisplayValidation
{
    bool valid{};
    VirtualDisplayErrorCode errorCode{VirtualDisplayErrorCode::none};
    std::string error;
};

[[nodiscard]] constexpr std::size_t virtualDisplayCountValue(
    VirtualDisplayCount value) noexcept
{
    return static_cast<std::size_t>(value);
}

[[nodiscard]] VirtualDisplayValidation validateVirtualDisplayConfiguration(
    const VirtualDisplayConfiguration& configuration);
[[nodiscard]] std::string stableVirtualMonitorId(std::size_t logicalIndex);
[[nodiscard]] std::string virtualMonitorFriendlyName(std::size_t logicalIndex);
[[nodiscard]] std::array<VirtualMonitorDescriptor, maximumVirtualDisplayCount>
    buildVirtualMonitorDescriptors(
        const VirtualDisplayConfiguration& configuration,
        std::size_t& descriptorCount);
[[nodiscard]] std::optional<VirtualDisplayCount> parseVirtualDisplayCount(
    std::string_view value) noexcept;
[[nodiscard]] std::optional<VirtualDisplayMode> parseVirtualDisplayMode(
    std::string_view value) noexcept;
[[nodiscard]] std::string_view virtualDisplayModeText(VirtualDisplayMode value) noexcept;
[[nodiscard]] std::string_view virtualDisplayStateText(VirtualDisplayState value) noexcept;
[[nodiscard]] bool sameRequestedTopology(
    const VirtualDisplayConfiguration& left,
    const VirtualDisplayConfiguration& right) noexcept;

} // namespace xreal::virtual_display
