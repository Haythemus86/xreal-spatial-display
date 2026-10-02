#pragma once

#include "VirtualDisplayTypes.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>

namespace xreal::virtual_display
{

inline constexpr std::uint32_t virtualDisplayProtocolMagic = 0x58445650U; // XDVP
inline constexpr std::uint16_t virtualDisplayProtocolVersion = 1U;
// CTL_CODE(FILE_DEVICE_UNKNOWN, 0x800, METHOD_BUFFERED, FILE_ANY_ACCESS).
inline constexpr std::uint32_t virtualDisplayIoControlCode = 0x00222000U;

enum class VirtualDisplayCommand : std::uint16_t
{
    getCapabilities = 1U,
    getCurrentConfiguration = 2U,
    applyConfiguration = 3U,
    disableVirtualDisplays = 4U,
    getVirtualMonitors = 5U,
    getStatus = 6U,
};

struct VirtualDisplayWireConfiguration
{
    std::uint32_t enabled{};
    std::uint32_t count{1U};
    std::uint32_t mode{1U};
    std::uint32_t width{defaultVirtualDisplayWidth};
    std::uint32_t height{defaultVirtualDisplayHeight};
    std::uint32_t refreshHertz{defaultVirtualDisplayRefreshHertz};
};

struct VirtualDisplayProtocolRequest
{
    std::uint32_t magic{virtualDisplayProtocolMagic};
    std::uint16_t version{virtualDisplayProtocolVersion};
    std::uint16_t command{};
    std::uint32_t byteSize{};
    std::uint64_t clientGeneration{};
    VirtualDisplayWireConfiguration configuration;
};

struct VirtualDisplayProtocolMonitor
{
    std::uint32_t logicalIndex{};
    std::uint32_t connectorIndex{};
    std::uint32_t active{};
    std::uint32_t width{};
    std::uint32_t height{};
    std::uint32_t refreshHertz{};
    std::array<char, 40U> stableId{};
    std::array<char, 64U> friendlyName{};
    std::array<char, 128U> windowsDeviceName{};
    std::array<char, 256U> windowsMonitorIdentity{};
};

struct VirtualDisplayProtocolResponse
{
    std::uint32_t magic{virtualDisplayProtocolMagic};
    std::uint16_t version{virtualDisplayProtocolVersion};
    std::uint16_t byteSize{};
    std::uint32_t accepted{};
    std::uint32_t state{};
    std::uint32_t errorCode{};
    std::uint32_t restartRequired{};
    std::uint64_t generation{};
    VirtualDisplayWireConfiguration requested;
    VirtualDisplayWireConfiguration actual;
    std::uint32_t monitorCount{};
    std::uint32_t supportedCountMask{0x7U};
    std::uint32_t supportedModeMask{0x3U};
    std::array<VirtualDisplayProtocolMonitor, maximumVirtualDisplayCount> monitors{};
    std::array<char, 192U> errorText{};
};

struct VirtualDisplayProtocolValidation
{
    bool valid{};
    VirtualDisplayErrorCode errorCode{VirtualDisplayErrorCode::none};
    std::string error;
};

[[nodiscard]] VirtualDisplayProtocolRequest makeProtocolRequest(
    VirtualDisplayCommand command,
    const VirtualDisplayConfiguration& configuration = {});
[[nodiscard]] VirtualDisplayProtocolValidation validateProtocolRequest(
    std::span<const std::byte> bytes);
[[nodiscard]] std::optional<VirtualDisplayProtocolRequest> decodeProtocolRequest(
    std::span<const std::byte> bytes);
[[nodiscard]] VirtualDisplayConfiguration configurationFromWire(
    const VirtualDisplayWireConfiguration& wire) noexcept;
[[nodiscard]] VirtualDisplayWireConfiguration configurationToWire(
    const VirtualDisplayConfiguration& configuration) noexcept;
[[nodiscard]] VirtualDisplayStatus statusFromProtocolResponse(
    const VirtualDisplayProtocolResponse& response);
void populateProtocolResponse(
    VirtualDisplayProtocolResponse& response,
    const VirtualDisplayStatus& status,
    bool accepted);
[[nodiscard]] VirtualDisplayProtocolValidation validateProtocolResponse(
    std::span<const std::byte> bytes);

} // namespace xreal::virtual_display
