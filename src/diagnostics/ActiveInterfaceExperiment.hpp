#pragma once

#include "diagnostics/InterfaceProbeSupport.hpp"
#include "sensors/XrealDevice.hpp"

#include <vector>

namespace xreal::diagnostics
{

[[nodiscard]] int runActiveInterfaceExperiment(
    const std::vector<sensors::XrealDeviceInfo>& devices,
    const ProbeOptions& options);

} // namespace xreal::diagnostics
