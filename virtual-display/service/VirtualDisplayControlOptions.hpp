#pragma once

#include "protocol/VirtualDisplayTypes.hpp"

#include <span>
#include <string>
#include <string_view>

namespace xreal::virtual_display
{

struct VirtualDisplayControlOptions
{
    VirtualDisplayConfiguration configuration;
    bool countSpecified{};
    bool apply{};
    bool disable{};
    bool showStatus{};
    bool help{};
};

struct VirtualDisplayControlParseResult
{
    bool success{};
    VirtualDisplayControlOptions options;
    std::string error;
};

[[nodiscard]] VirtualDisplayControlParseResult parseVirtualDisplayControlOptions(
    std::span<const std::string_view> arguments);
[[nodiscard]] std::string virtualDisplayControlUsage();

} // namespace xreal::virtual_display
