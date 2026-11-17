#pragma once

#include "protocol/VirtualDisplayProtocol.hpp"

#include <optional>
#include <string>

namespace xreal::virtual_display
{

class IVirtualDisplayTransport
{
public:
    virtual ~IVirtualDisplayTransport() = default;

    [[nodiscard]] virtual std::optional<VirtualDisplayProtocolResponse> transact(
        const VirtualDisplayProtocolRequest& request) = 0;
    [[nodiscard]] virtual const std::string& error() const noexcept = 0;
};

struct VirtualDisplayClientResult
{
    bool accepted{};
    VirtualDisplayStatus status;
    std::string error;
};

class VirtualDisplayClient
{
public:
    explicit VirtualDisplayClient(IVirtualDisplayTransport& transport) noexcept;

    [[nodiscard]] VirtualDisplayClientResult capabilities();
    [[nodiscard]] VirtualDisplayClientResult currentConfiguration();
    [[nodiscard]] VirtualDisplayClientResult apply(
        const VirtualDisplayConfiguration& configuration);
    [[nodiscard]] VirtualDisplayClientResult disable();
    [[nodiscard]] VirtualDisplayClientResult monitors();
    [[nodiscard]] VirtualDisplayClientResult status();

private:
    [[nodiscard]] VirtualDisplayClientResult execute(
        VirtualDisplayCommand command,
        const VirtualDisplayConfiguration& configuration = {});

    IVirtualDisplayTransport& transport_;
};

class VirtualDisplayStateMachine
{
public:
    VirtualDisplayStateMachine();

    [[nodiscard]] VirtualDisplayProtocolResponse process(
        const VirtualDisplayProtocolRequest& request);
    [[nodiscard]] const VirtualDisplayStatus& status() const noexcept;

private:
    [[nodiscard]] VirtualDisplayProtocolResponse currentResponse(bool accepted) const;
    [[nodiscard]] VirtualDisplayProtocolResponse rejectedResponse(
        VirtualDisplayErrorCode errorCode,
        std::string error) const;
    void applyConfiguration(VirtualDisplayConfiguration configuration);

    VirtualDisplayStatus status_;
};

class InMemoryVirtualDisplayTransport final : public IVirtualDisplayTransport
{
public:
    [[nodiscard]] std::optional<VirtualDisplayProtocolResponse> transact(
        const VirtualDisplayProtocolRequest& request) override;
    [[nodiscard]] const std::string& error() const noexcept override;
    [[nodiscard]] const VirtualDisplayStatus& status() const noexcept;

private:
    VirtualDisplayStateMachine stateMachine_;
    std::string error_;
};

} // namespace xreal::virtual_display
