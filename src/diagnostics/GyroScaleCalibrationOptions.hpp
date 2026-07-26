#pragma once

#include "sensors/GyroscopeBiasCalibration.hpp"
#include "sensors/GyroscopeScaleCalibration.hpp"

#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <string_view>

namespace xreal::diagnostics
{

struct GyroScaleCalibrationOptions
{
    sensors::GyroscopeScaleCalibrationConfig scaleConfiguration;
    sensors::GyroscopeBiasCalibrationConfig biasConfiguration;
    std::uint64_t trialCount{6};
    std::uint64_t countdownSeconds{3};
    std::string outputPath;
    std::optional<std::string> csvPrefix;
};

struct GyroScaleCalibrationOptionResult
{
    std::optional<GyroScaleCalibrationOptions> options;
    std::string error;
    bool showHelp{};
};

[[nodiscard]] GyroScaleCalibrationOptionResult parseGyroScaleCalibrationOptions(
    std::span<const std::string_view> arguments);

[[nodiscard]] std::string gyroScaleCalibrationUsage();

} // namespace xreal::diagnostics
