#pragma once

#include <cstddef>
#include <string>
#include <string_view>

namespace xreal::platform::windows
{

struct HidDeviceCapabilities
{
    bool available{};
    std::size_t inputReportByteLength{};
    std::size_t outputReportByteLength{};
    std::size_t featureReportByteLength{};
    std::string errorMessage;
};

[[nodiscard]] HidDeviceCapabilities queryHidDeviceCapabilities(std::string_view path);

} // namespace xreal::platform::windows
