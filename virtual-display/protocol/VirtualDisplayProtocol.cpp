#include "VirtualDisplayProtocol.hpp"

#include <algorithm>
#include <cstring>
#include <type_traits>

namespace xreal::virtual_display
{
namespace
{

template <std::size_t Size>
void copyText(std::array<char, Size>& destination, std::string_view source) noexcept
{
    destination.fill('\0');
    const std::size_t count = std::min(source.size(), Size - 1U);
    std::copy_n(source.data(), count, destination.data());
}

template <std::size_t Size>
[[nodiscard]] std::string readText(const std::array<char, Size>& source)
{
    const auto end = std::find(source.begin(), source.end(), '\0');
    return std::string(source.begin(), end);
}

[[nodiscard]] bool knownCommand(std::uint16_t raw) noexcept
{
    return raw >= static_cast<std::uint16_t>(VirtualDisplayCommand::getCapabilities)
        && raw <= static_cast<std::uint16_t>(VirtualDisplayCommand::getStatus);
}

} // namespace

static_assert(std::is_trivially_copyable_v<VirtualDisplayProtocolRequest>);
static_assert(std::is_trivially_copyable_v<VirtualDisplayProtocolResponse>);
static_assert(sizeof(VirtualDisplayWireConfiguration) == 24U);
static_assert(sizeof(VirtualDisplayProtocolRequest) == 48U);
static_assert(sizeof(VirtualDisplayProtocolMonitor) == 512U);
static_assert(sizeof(VirtualDisplayProtocolResponse) == 1824U);

VirtualDisplayProtocolRequest makeProtocolRequest(
    VirtualDisplayCommand command,
    const VirtualDisplayConfiguration& configuration)
{
    VirtualDisplayProtocolRequest result;
    result.command = static_cast<std::uint16_t>(command);
    result.byteSize = sizeof(VirtualDisplayProtocolRequest);
    result.clientGeneration = configuration.applyGeneration;
    result.configuration = configurationToWire(configuration);
    return result;
}

VirtualDisplayProtocolValidation validateProtocolRequest(std::span<const std::byte> bytes)
{
    if (bytes.size() != sizeof(VirtualDisplayProtocolRequest))
    {
        return {false, VirtualDisplayErrorCode::malformedBuffer,
            "Protocol request size is invalid."};
    }
    VirtualDisplayProtocolRequest request;
    std::memcpy(&request, bytes.data(), sizeof(request));
    if (request.magic != virtualDisplayProtocolMagic
        || request.version != virtualDisplayProtocolVersion)
    {
        return {false, VirtualDisplayErrorCode::invalidProtocol,
            "Protocol magic or version is invalid."};
    }
    if (request.byteSize != sizeof(VirtualDisplayProtocolRequest))
    {
        return {false, VirtualDisplayErrorCode::malformedBuffer,
            "Declared protocol request size is invalid."};
    }
    if (!knownCommand(request.command))
    {
        return {false, VirtualDisplayErrorCode::invalidProtocol,
            "Protocol command is invalid."};
    }
    if (request.command == static_cast<std::uint16_t>(
            VirtualDisplayCommand::applyConfiguration))
    {
        if (request.configuration.enabled > 1U)
        {
            return {false, VirtualDisplayErrorCode::malformedBuffer,
                "Protocol enabled state must be zero or one."};
        }
        const auto validation = validateVirtualDisplayConfiguration(
            configurationFromWire(request.configuration));
        if (!validation.valid)
        {
            return {false, validation.errorCode, validation.error};
        }
    }
    return {true, VirtualDisplayErrorCode::none, {}};
}

std::optional<VirtualDisplayProtocolRequest> decodeProtocolRequest(
    std::span<const std::byte> bytes)
{
    if (!validateProtocolRequest(bytes).valid)
    {
        return std::nullopt;
    }
    VirtualDisplayProtocolRequest result;
    std::memcpy(&result, bytes.data(), sizeof(result));
    return result;
}

VirtualDisplayConfiguration configurationFromWire(
    const VirtualDisplayWireConfiguration& wire) noexcept
{
    VirtualDisplayConfiguration result;
    result.enabled = wire.enabled != 0U;
    result.count = static_cast<VirtualDisplayCount>(wire.count);
    result.mode = static_cast<VirtualDisplayMode>(wire.mode);
    result.resolution = {wire.width, wire.height, wire.refreshHertz};
    return result;
}

VirtualDisplayWireConfiguration configurationToWire(
    const VirtualDisplayConfiguration& configuration) noexcept
{
    return {
        configuration.enabled ? 1U : 0U,
        static_cast<std::uint32_t>(configuration.count),
        static_cast<std::uint32_t>(configuration.mode),
        configuration.resolution.width,
        configuration.resolution.height,
        configuration.resolution.refreshHertz,
    };
}

VirtualDisplayStatus statusFromProtocolResponse(
    const VirtualDisplayProtocolResponse& response)
{
    VirtualDisplayStatus result;
    result.requested = configurationFromWire(response.requested);
    result.actual = configurationFromWire(response.actual);
    result.requested.applyGeneration = response.generation;
    result.actual.applyGeneration = response.generation;
    result.state = static_cast<VirtualDisplayState>(response.state);
    result.restartRequired = response.restartRequired != 0U;
    result.errorCode = static_cast<VirtualDisplayErrorCode>(response.errorCode);
    result.lastError = readText(response.errorText);
    result.monitorCount = std::min<std::size_t>(response.monitorCount, result.monitors.size());
    for (std::size_t index = 0U; index < result.monitorCount; ++index)
    {
        const auto& source = response.monitors[index];
        auto& destination = result.monitors[index];
        destination.logicalIndex = static_cast<std::uint8_t>(source.logicalIndex);
        destination.connectorIndex = static_cast<std::uint8_t>(source.connectorIndex);
        destination.active = source.active != 0U;
        destination.resolution = {source.width, source.height, source.refreshHertz};
        destination.stableId = readText(source.stableId);
        destination.friendlyName = readText(source.friendlyName);
        destination.windowsDeviceName = readText(source.windowsDeviceName);
        destination.windowsMonitorIdentity = readText(source.windowsMonitorIdentity);
        destination.topologyMode = result.actual.mode;
    }
    return result;
}

void populateProtocolResponse(
    VirtualDisplayProtocolResponse& response,
    const VirtualDisplayStatus& status,
    bool accepted)
{
    response = {};
    response.byteSize = static_cast<std::uint16_t>(sizeof(response));
    response.accepted = accepted ? 1U : 0U;
    response.state = static_cast<std::uint32_t>(status.state);
    response.errorCode = static_cast<std::uint32_t>(status.errorCode);
    response.restartRequired = status.restartRequired ? 1U : 0U;
    response.generation = status.actual.applyGeneration;
    response.requested = configurationToWire(status.requested);
    response.actual = configurationToWire(status.actual);
    response.monitorCount = static_cast<std::uint32_t>(
        std::min(status.monitorCount, status.monitors.size()));
    for (std::size_t index = 0U; index < response.monitorCount; ++index)
    {
        const auto& source = status.monitors[index];
        auto& destination = response.monitors[index];
        destination.logicalIndex = source.logicalIndex;
        destination.connectorIndex = source.connectorIndex;
        destination.active = source.active ? 1U : 0U;
        destination.width = source.resolution.width;
        destination.height = source.resolution.height;
        destination.refreshHertz = source.resolution.refreshHertz;
        copyText(destination.stableId, source.stableId);
        copyText(destination.friendlyName, source.friendlyName);
        copyText(destination.windowsDeviceName, source.windowsDeviceName);
        copyText(destination.windowsMonitorIdentity, source.windowsMonitorIdentity);
    }
    copyText(response.errorText, status.lastError);
}

VirtualDisplayProtocolValidation validateProtocolResponse(std::span<const std::byte> bytes)
{
    if (bytes.size() != sizeof(VirtualDisplayProtocolResponse))
    {
        return {false, VirtualDisplayErrorCode::malformedBuffer,
            "Protocol response size is invalid."};
    }
    VirtualDisplayProtocolResponse response;
    std::memcpy(&response, bytes.data(), sizeof(response));
    if (response.magic != virtualDisplayProtocolMagic
        || response.version != virtualDisplayProtocolVersion)
    {
        return {false, VirtualDisplayErrorCode::invalidProtocol,
            "Protocol response magic or version is invalid."};
    }
    if (response.byteSize != sizeof(VirtualDisplayProtocolResponse)
        || response.monitorCount > maximumVirtualDisplayCount)
    {
        return {false, VirtualDisplayErrorCode::malformedBuffer,
            "Protocol response metadata is invalid."};
    }
    if (response.accepted > 1U || response.restartRequired > 1U
        || response.state > static_cast<std::uint32_t>(
            VirtualDisplayState::restartRequired)
        || response.errorCode > static_cast<std::uint32_t>(
            VirtualDisplayErrorCode::platformFailure))
    {
        return {false, VirtualDisplayErrorCode::malformedBuffer,
            "Protocol response state metadata is invalid."};
    }
    const auto requestedValidation = validateVirtualDisplayConfiguration(
        configurationFromWire(response.requested));
    const auto actualValidation = validateVirtualDisplayConfiguration(
        configurationFromWire(response.actual));
    if (!requestedValidation.valid || !actualValidation.valid)
    {
        return {false, VirtualDisplayErrorCode::malformedBuffer,
            "Protocol response contains an invalid topology."};
    }
    for (std::size_t index = 0U; index < response.monitorCount; ++index)
    {
        const auto& monitor = response.monitors[index];
        if (monitor.logicalIndex >= maximumVirtualDisplayCount
            || monitor.connectorIndex >= maximumVirtualDisplayCount
            || monitor.active > 1U
            || monitor.width != defaultVirtualDisplayWidth
            || monitor.height != defaultVirtualDisplayHeight
            || monitor.refreshHertz != defaultVirtualDisplayRefreshHertz)
        {
            return {false, VirtualDisplayErrorCode::malformedBuffer,
                "Protocol response contains an invalid monitor descriptor."};
        }
    }
    return {true, VirtualDisplayErrorCode::none, {}};
}

} // namespace xreal::virtual_display
