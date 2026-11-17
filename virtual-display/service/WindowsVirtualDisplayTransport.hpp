#pragma once

#include "VirtualDisplayService.hpp"

#include <string>

namespace xreal::virtual_display
{

class WindowsVirtualDisplayTransport final : public IVirtualDisplayTransport
{
public:
    [[nodiscard]] std::optional<VirtualDisplayProtocolResponse> transact(
        const VirtualDisplayProtocolRequest& request) override;
    [[nodiscard]] const std::string& error() const noexcept override;

private:
    std::string error_;
};

} // namespace xreal::virtual_display
