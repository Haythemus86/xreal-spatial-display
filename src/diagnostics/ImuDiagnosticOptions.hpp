#pragma once

#include "sensors/GyroscopeBiasCalibration.hpp"

#include <chrono>
#include <optional>
#include <span>
#include <string>
#include <string_view>

namespace xreal::diagnostics
{

struct ImuDiagnosticOptions
{
    std::chrono::seconds duration{10};
    bool durationExplicit{};
    unsigned int printRateHz{10};
    std::optional<std::string> csvPath;
    std::optional<std::string> calibrationName;
    std::chrono::seconds stationaryDuration{5};
    bool stationaryDurationExplicit{};
    std::optional<std::string> calibrationOutputPath;
    std::optional<sensors::GyroscopeBiasCalibrationConfig> gyroscopeCalibration;
    bool applyGyroscopeBias{};
    std::optional<std::string> gyroscopeCalibrationOutputPath;
    bool verbose{};
};

struct ImuDiagnosticOptionResult
{
    std::optional<ImuDiagnosticOptions> options;
    std::string error;
    bool showHelp{};
};

[[nodiscard]] ImuDiagnosticOptionResult parseImuDiagnosticOptions(
    std::span<const std::string_view> arguments);

[[nodiscard]] std::string imuDiagnosticUsage();

} // namespace xreal::diagnostics
