#pragma once

#include "sensors/ImuCalibration.hpp"

#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace xreal::sensors
{

struct StaticAccelerometerCapture
{
    std::string name;
    std::string source;
    bool stationaryAccepted = false;
    RawVector3d accelMeanRaw;
};

struct AccelerometerCaptureResidual
{
    std::string name;
    std::string source;
    RawVector3d accelMeanRaw;
    double residual = 0.0;
    double correctedGravityMagnitude = 0.0;
};

struct AxisAlignedAccelerometerCalibration
{
    RawVector3d offsetRaw;
    RawVector3d rawUnitsPerG;
    std::vector<AccelerometerCaptureResidual> captures;
    double rmsResidual = 0.0;
    double maximumResidual = 0.0;
    double normalEquationPivotRatio = 0.0;
    bool poorQuality = false;
    std::vector<std::string> qualityWarnings;
};

struct AccelerometerCalibrationFitResult
{
    std::optional<AxisAlignedAccelerometerCalibration> calibration;
    std::string errorMessage;
};

struct CalibrationCaptureParseResult
{
    std::optional<StaticAccelerometerCapture> capture;
    std::string errorMessage;
};

[[nodiscard]] AccelerometerCalibrationFitResult fitAxisAlignedAccelerometerEllipsoid(
    std::span<const StaticAccelerometerCapture> captures);

[[nodiscard]] CalibrationCaptureParseResult parseCalibrationCaptureJson(
    std::string_view json,
    std::string source);

[[nodiscard]] std::string serializeAccelerometerCalibrationProfileJson(
    const AxisAlignedAccelerometerCalibration& calibration);

} // namespace xreal::sensors
