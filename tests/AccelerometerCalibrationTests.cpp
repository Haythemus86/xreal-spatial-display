#include "sensors/AccelerometerCalibration.hpp"

#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

namespace
{

void require(bool condition, const char* message)
{
    if (!condition)
    {
        std::cerr << "FAILED: " << message << '\n';
        std::exit(EXIT_FAILURE);
    }
}

[[nodiscard]] bool nearlyEqual(double actual, double expected, double tolerance)
{
    return std::abs(actual - expected) <= tolerance;
}

[[nodiscard]] std::vector<xreal::sensors::StaticAccelerometerCapture> makeSyntheticCaptures()
{
    constexpr xreal::sensors::RawVector3d offset{1200.0, -3400.0, 7800.0};
    constexpr xreal::sensors::RawVector3d scale{510000.0, 525000.0, 495000.0};
    constexpr std::array<xreal::sensors::RawVector3d, 12> directions{{
        {1.0, 2.0, 3.0},
        {-2.0, 1.0, 3.0},
        {3.0, -2.0, 1.0},
        {-3.0, -1.0, 2.0},
        {2.0, 3.0, -1.0},
        {-1.0, 3.0, -2.0},
        {1.0, -3.0, -2.0},
        {-2.0, -3.0, -1.0},
        {4.0, 1.0, -2.0},
        {-4.0, 2.0, 1.0},
        {1.0, -2.0, 4.0},
        {-1.0, -4.0, 2.0},
    }};

    std::vector<xreal::sensors::StaticAccelerometerCapture> captures;
    captures.reserve(directions.size());
    for (std::size_t index = 0; index < directions.size(); ++index)
    {
        const auto& direction = directions[index];
        const double magnitude = std::sqrt(
            direction.x * direction.x + direction.y * direction.y + direction.z * direction.z);
        captures.push_back({
            "synthetic-" + std::to_string(index),
            "generated",
            true,
            {
                offset.x + scale.x * direction.x / magnitude,
                offset.y + scale.y * direction.y / magnitude,
                offset.z + scale.z * direction.z / magnitude,
            }});
    }
    return captures;
}

void testKnownEllipsoidFit()
{
    auto captures = makeSyntheticCaptures();
    captures.push_back({"rejected-outlier", "generated", false, {9000000.0, -8000000.0, 7000000.0}});

    const auto result = xreal::sensors::fitAxisAlignedAccelerometerEllipsoid(captures);
    require(result.calibration.has_value(), "synthetic ellipsoid fit should succeed");
    const auto& calibration = *result.calibration;

    require(nearlyEqual(calibration.offsetRaw.x, 1200.0, 1.0e-6), "known X offset");
    require(nearlyEqual(calibration.offsetRaw.y, -3400.0, 1.0e-6), "known Y offset");
    require(nearlyEqual(calibration.offsetRaw.z, 7800.0, 1.0e-6), "known Z offset");
    require(nearlyEqual(calibration.rawUnitsPerG.x, 510000.0, 1.0e-6), "known X scale");
    require(nearlyEqual(calibration.rawUnitsPerG.y, 525000.0, 1.0e-6), "known Y scale");
    require(nearlyEqual(calibration.rawUnitsPerG.z, 495000.0, 1.0e-6), "known Z scale");
    require(calibration.captures.size() == 12, "rejected captures must not be fitted");
    require(calibration.rmsResidual < 1.0e-12, "synthetic RMS residual");
    require(calibration.maximumResidual < 1.0e-12, "synthetic maximum residual");
}

void testRequiresSixAcceptedCaptures()
{
    auto captures = makeSyntheticCaptures();
    captures.resize(6);
    captures.back().stationaryAccepted = false;
    const auto result = xreal::sensors::fitAxisAlignedAccelerometerEllipsoid(captures);
    require(!result.calibration.has_value(), "fewer than six accepted captures must fail");
}

void testPoorFitQualityWarning()
{
    auto captures = makeSyntheticCaptures();
    captures.front().accelMeanRaw.x += 175000.0;
    const auto result = xreal::sensors::fitAxisAlignedAccelerometerEllipsoid(captures);
    require(result.calibration.has_value(), "noisy ellipsoid should still produce diagnostic results");
    require(result.calibration->poorQuality, "large residuals should mark the fit as poor");
    require(!result.calibration->qualityWarnings.empty(), "poor fit should include a quality warning");
}

void testCaptureJsonParsing()
{
    constexpr std::string_view json = R"json({
        "schema_version": 1,
        "units": "raw",
        "calibration": "right-side-retry",
        "stationary": {
            "accepted": true,
            "accel_mean_raw": {"x": 12.5, "y": -34.25, "z": 56}
        }
    })json";
    const auto parsed = xreal::sensors::parseCalibrationCaptureJson(json, "retry.json");
    require(parsed.capture.has_value(), "capture JSON should parse");
    require(parsed.capture->name == "right-side-retry", "capture name");
    require(parsed.capture->stationaryAccepted, "accepted state");
    require(parsed.capture->source == "retry.json", "capture source");
    require(nearlyEqual(parsed.capture->accelMeanRaw.x, 12.5, 0.0), "parsed X mean");
    require(nearlyEqual(parsed.capture->accelMeanRaw.y, -34.25, 0.0), "parsed Y mean");
    require(nearlyEqual(parsed.capture->accelMeanRaw.z, 56.0, 0.0), "parsed Z mean");
}

void testProfileSerialization()
{
    const auto result = xreal::sensors::fitAxisAlignedAccelerometerEllipsoid(makeSyntheticCaptures());
    require(result.calibration.has_value(), "serialization fixture fit");
    const std::string json = xreal::sensors::serializeAccelerometerCalibrationProfileJson(*result.calibration);
    require(json.find("\"model\": \"axis_aligned_ellipsoid\"") != std::string::npos, "profile model");
    require(json.find("\"raw_units_per_g\"") != std::string::npos, "profile scales");
    require(json.find("\"corrected_gravity_magnitude\"") != std::string::npos, "profile residual details");
}

} // namespace

int main()
{
    testKnownEllipsoidFit();
    testRequiresSixAcceptedCaptures();
    testPoorFitQualityWarning();
    testCaptureJsonParsing();
    testProfileSerialization();
    std::cout << "All accelerometer calibration tests passed.\n";
    return EXIT_SUCCESS;
}
