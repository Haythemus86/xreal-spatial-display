#pragma once

#include "diagnostics/GyroScaleCalibrationOptions.hpp"
#include "sensors/XrealDevice.hpp"

namespace xreal::diagnostics
{

[[nodiscard]] int runGyroscopeRecordOnlyMode(
    const sensors::XrealDeviceInfo& device,
    const GyroScaleCalibrationOptions& options);

} // namespace xreal::diagnostics
