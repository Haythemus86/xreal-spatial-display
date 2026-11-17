#pragma once

#include "protocol/VirtualDisplayTypes.hpp"
#include "rendering/PanelScene.hpp"

#include <array>
#include <cstddef>
#include <string>

namespace xreal::virtual_display
{

struct VirtualDisplayPanelBinding
{
    rendering::PanelId panelId{};
    std::string monitorStableId;
    std::size_t sourceSlot{};
};

struct VirtualDisplayTopologyResult
{
    bool success{};
    std::string error;
    std::size_t panelCount{};
    std::size_t sourceCount{};
    std::array<VirtualDisplayPanelBinding, maximumVirtualDisplayCount> bindings{};
};

[[nodiscard]] VirtualDisplayTopologyResult routeVirtualDisplaysToPanels(
    const VirtualDisplayStatus& status,
    rendering::PanelScene& scene);

} // namespace xreal::virtual_display
