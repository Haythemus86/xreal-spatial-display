#pragma once

#include "sensors/GyroscopeBiasCalibration.hpp"

#include <chrono>
#include <optional>
#include <span>
#include <string>
#include <string_view>

namespace xreal::diagnostics
{

enum class OrientationOutputMode
{
    quaternion,
    euler,
    both,
};

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
    std::optional<std::string> gyroscopeScaleProfilePath;
    std::optional<double> gyroscopeScaleRawPerDegreePerSecond;
    bool printGyroscopeDegrees{};
    bool printGyroscopeRadians{};
    bool compareQ12Scale{};
    bool integrateGyroscopeOrientation{};
    OrientationOutputMode orientationOutput{OrientationOutputMode::both};
    bool orientationOutputExplicit{};
    std::optional<double> recenterAfterSeconds;
    std::chrono::nanoseconds orientationMaximumDelta{std::chrono::milliseconds(20)};
    bool orientationMaximumDeltaExplicit{};
    std::optional<std::string> orientationProfileOutputPath;
    unsigned int orientationPrintRateHz{10};
    bool orientationPrintRateExplicit{};
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
