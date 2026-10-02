#pragma once

#include "protocol/VirtualDisplayTypes.hpp"

#include <array>
#include <string>
#include <vector>

namespace xreal::virtual_display
{

struct VirtualDisplayViewModel
{
    std::array<VirtualDisplayCount, 3U> availableCounts{
        VirtualDisplayCount::one,
        VirtualDisplayCount::two,
        VirtualDisplayCount::three,
    };
    std::array<VirtualDisplayMode, 2U> availableModes{
        VirtualDisplayMode::extended,
        VirtualDisplayMode::mirrored,
    };
    VirtualDisplayCount selectedCount{VirtualDisplayCount::one};
    VirtualDisplayMode selectedMode{VirtualDisplayMode::extended};
    VirtualDisplayResolution resolution;
    bool enabled{};
    bool applyEnabled{};
    bool applyInProgress{};
    bool restartRequired{};
    VirtualDisplayStatus actualTopology;
    std::vector<std::string> validationMessages;
};

[[nodiscard]] VirtualDisplayViewModel makeVirtualDisplayViewModel(
    const VirtualDisplayConfiguration& requested,
    const VirtualDisplayStatus& actual,
    bool applyInProgress = false);

} // namespace xreal::virtual_display
