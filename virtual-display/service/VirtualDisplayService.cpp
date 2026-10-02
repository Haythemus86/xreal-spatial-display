#include "VirtualDisplayService.hpp"

#include <cstddef>
#include <span>
#include <utility>

namespace xreal::virtual_display
{

VirtualDisplayClient::VirtualDisplayClient(IVirtualDisplayTransport& transport) noexcept
    : transport_(transport)
{
}

VirtualDisplayClientResult VirtualDisplayClient::capabilities()
{
    return execute(VirtualDisplayCommand::getCapabilities);
}

VirtualDisplayClientResult VirtualDisplayClient::currentConfiguration()
{
    return execute(VirtualDisplayCommand::getCurrentConfiguration);
}

VirtualDisplayClientResult VirtualDisplayClient::apply(
    const VirtualDisplayConfiguration& configuration)
{
    const auto validation = validateVirtualDisplayConfiguration(configuration);
    if (!validation.valid)
    {
        VirtualDisplayClientResult result;
        result.status.state = VirtualDisplayState::failed;
        result.status.errorCode = validation.errorCode;
        result.status.lastError = validation.error;
        result.error = validation.error;
        return result;
    }
    return execute(VirtualDisplayCommand::applyConfiguration, configuration);
}

VirtualDisplayClientResult VirtualDisplayClient::disable()
{
    return execute(VirtualDisplayCommand::disableVirtualDisplays);
}

VirtualDisplayClientResult VirtualDisplayClient::monitors()
{
    return execute(VirtualDisplayCommand::getVirtualMonitors);
}

VirtualDisplayClientResult VirtualDisplayClient::status()
{
    return execute(VirtualDisplayCommand::getStatus);
}

VirtualDisplayClientResult VirtualDisplayClient::execute(
    VirtualDisplayCommand command,
    const VirtualDisplayConfiguration& configuration)
{
    const auto response = transport_.transact(makeProtocolRequest(command, configuration));
    if (!response.has_value())
    {
        return {false, {}, transport_.error()};
    }
    const auto bytes = std::as_bytes(std::span{&*response, 1U});
    const auto validation = validateProtocolResponse(bytes);
    if (!validation.valid)
    {
        return {false, {}, validation.error};
    }
    VirtualDisplayClientResult result;
    result.accepted = response->accepted != 0U;
    result.status = statusFromProtocolResponse(*response);
    result.error = result.accepted ? std::string{} : result.status.lastError;
    return result;
}

VirtualDisplayStateMachine::VirtualDisplayStateMachine()
{
    status_.requested.enabled = false;
    status_.actual.enabled = false;
    status_.state = VirtualDisplayState::disabled;
}

VirtualDisplayProtocolResponse VirtualDisplayStateMachine::process(
    const VirtualDisplayProtocolRequest& request)
{
    const auto bytes = std::as_bytes(std::span{&request, 1U});
    const auto protocolValidation = validateProtocolRequest(bytes);
    if (!protocolValidation.valid)
    {
        return rejectedResponse(protocolValidation.errorCode, protocolValidation.error);
    }
    const auto command = static_cast<VirtualDisplayCommand>(request.command);
    switch (command)
    {
    case VirtualDisplayCommand::getCapabilities:
    case VirtualDisplayCommand::getCurrentConfiguration:
    case VirtualDisplayCommand::getVirtualMonitors:
    case VirtualDisplayCommand::getStatus:
        return currentResponse(true);
    case VirtualDisplayCommand::applyConfiguration:
    {
        auto configuration = configurationFromWire(request.configuration);
        const auto validation = validateVirtualDisplayConfiguration(configuration);
        if (!validation.valid)
        {
            return rejectedResponse(validation.errorCode, validation.error);
        }
        if (!configuration.enabled)
        {
            configuration.count = VirtualDisplayCount::one;
        }
        if (sameRequestedTopology(status_.requested, configuration)
            && ((configuration.enabled && status_.state == VirtualDisplayState::active)
                || (!configuration.enabled && status_.state == VirtualDisplayState::disabled)))
        {
            return currentResponse(true);
        }
        applyConfiguration(configuration);
        return currentResponse(true);
    }
    case VirtualDisplayCommand::disableVirtualDisplays:
    {
        auto disabled = status_.requested;
        disabled.enabled = false;
        disabled.count = VirtualDisplayCount::one;
        if (!status_.actual.enabled && status_.state == VirtualDisplayState::disabled)
        {
            return currentResponse(true);
        }
        applyConfiguration(disabled);
        return currentResponse(true);
    }
    }
    return rejectedResponse(
        VirtualDisplayErrorCode::invalidProtocol, "Protocol command is invalid.");
}

const VirtualDisplayStatus& VirtualDisplayStateMachine::status() const noexcept
{
    return status_;
}

VirtualDisplayProtocolResponse VirtualDisplayStateMachine::currentResponse(bool accepted) const
{
    VirtualDisplayProtocolResponse result;
    populateProtocolResponse(result, status_, accepted);
    return result;
}

VirtualDisplayProtocolResponse VirtualDisplayStateMachine::rejectedResponse(
    VirtualDisplayErrorCode errorCode,
    std::string error) const
{
    auto rejected = status_;
    rejected.errorCode = errorCode;
    rejected.lastError = std::move(error);
    VirtualDisplayProtocolResponse result;
    populateProtocolResponse(result, rejected, false);
    return result;
}

void VirtualDisplayStateMachine::applyConfiguration(
    VirtualDisplayConfiguration configuration)
{
    const std::uint64_t generation = status_.actual.applyGeneration + 1U;
    configuration.applyGeneration = generation;
    status_.requested = configuration;
    status_.actual = configuration;
    if (configuration.enabled && configuration.mode == VirtualDisplayMode::mirrored)
    {
        status_.actual.count = VirtualDisplayCount::one;
    }
    status_.requested.applyGeneration = generation;
    status_.actual.applyGeneration = generation;
    status_.restartRequired = false;
    status_.errorCode = VirtualDisplayErrorCode::none;
    status_.lastError.clear();
    status_.monitors = buildVirtualMonitorDescriptors(configuration, status_.monitorCount);
    status_.state = configuration.enabled
        ? VirtualDisplayState::active : VirtualDisplayState::disabled;
}

std::optional<VirtualDisplayProtocolResponse> InMemoryVirtualDisplayTransport::transact(
    const VirtualDisplayProtocolRequest& request)
{
    error_.clear();
    return stateMachine_.process(request);
}

const std::string& InMemoryVirtualDisplayTransport::error() const noexcept
{
    return error_;
}

const VirtualDisplayStatus& InMemoryVirtualDisplayTransport::status() const noexcept
{
    return stateMachine_.status();
}

} // namespace xreal::virtual_display
