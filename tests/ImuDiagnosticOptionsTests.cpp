#include "diagnostics/ImuDiagnosticOptions.hpp"

#include <array>
#include <chrono>
#include <iostream>
#include <string_view>

namespace
{

int failureCount{};

void expect(bool condition, std::string_view description)
{
    if (!condition)
    {
        ++failureCount;
        std::cerr << "FAILED: " << description << '\n';
    }
}

template <std::size_t Size>
[[nodiscard]] auto parse(const std::array<std::string_view, Size>& arguments)
{
    return xreal::diagnostics::parseImuDiagnosticOptions(arguments);
}

void testValidGyroscopeOptions()
{
    constexpr std::array arguments{
        std::string_view("--gyro-calibrate-seconds"), std::string_view("2"),
        std::string_view("--gyro-warmup-seconds"), std::string_view("1"),
        std::string_view("--apply-gyro-bias"),
        std::string_view("--duration"), std::string_view("10"),
        std::string_view("--print-rate"), std::string_view("10"),
        std::string_view("--gyro-calibration-output"), std::string_view("gyro.json"),
        std::string_view("--gyro-max-stddev"), std::string_view("650.5"),
        std::string_view("--gyro-max-range"), std::string_view("4500"),
        std::string_view("--gyro-min-samples"), std::string_view("1600"),
    };
    const auto result = parse(arguments);
    expect(result.options.has_value(), "valid gyroscope calibration options are accepted");
    expect(result.options->gyroscopeCalibration.has_value(), "gyroscope configuration is created");
    expect(result.options->gyroscopeCalibration->calibrationDuration == std::chrono::seconds(2),
           "gyroscope duration is parsed");
    expect(result.options->gyroscopeCalibration->warmupDuration == std::chrono::seconds(1),
           "gyroscope warm-up duration is parsed");
    expect(result.options->gyroscopeCalibration->minimumSampleCount == 1600,
           "minimum sample count is parsed");
    expect(result.options->gyroscopeCalibration->maximumStandardDeviationRaw == 650.5,
           "standard-deviation threshold is parsed");
    expect(result.options->gyroscopeCalibration->maximumRangeRaw == 4500.0,
           "range threshold is parsed");
    expect(result.options->applyGyroscopeBias, "bias application is enabled");
}

void testInvalidGyroscopeOptions()
{
    constexpr std::array zeroDuration{
        std::string_view("--gyro-calibrate-seconds"), std::string_view("0")};
    expect(!parse(zeroDuration).options.has_value(), "zero calibration duration is rejected");

    constexpr std::array negativeThreshold{
        std::string_view("--gyro-calibrate-seconds"), std::string_view("2"),
        std::string_view("--gyro-max-stddev"), std::string_view("-1")};
    expect(!parse(negativeThreshold).options.has_value(), "negative threshold is rejected");

    constexpr std::array zeroWarmup{
        std::string_view("--gyro-calibrate-seconds"), std::string_view("2"),
        std::string_view("--gyro-warmup-seconds"), std::string_view("0")};
    expect(!parse(zeroWarmup).options.has_value(), "zero warm-up duration is rejected");

    constexpr std::array negativeWarmup{
        std::string_view("--gyro-calibrate-seconds"), std::string_view("2"),
        std::string_view("--gyro-warmup-seconds"), std::string_view("-1")};
    expect(!parse(negativeWarmup).options.has_value(), "negative warm-up duration is rejected");

    constexpr std::array thresholdWithoutCalibration{
        std::string_view("--gyro-max-range"), std::string_view("5000")};
    expect(!parse(thresholdWithoutCalibration).options.has_value(),
           "threshold without calibration mode is rejected");

    constexpr std::array applyWithoutCalibration{std::string_view("--apply-gyro-bias")};
    expect(!parse(applyWithoutCalibration).options.has_value(),
           "bias application without same-run calibration is rejected");

    constexpr std::array outputWithoutCalibration{
        std::string_view("--gyro-calibration-output"), std::string_view("gyro.json")};
    expect(!parse(outputWithoutCalibration).options.has_value(),
           "gyro JSON output without calibration is rejected");
}

void testExistingAccelerometerOptionsRemainValid()
{
    constexpr std::array arguments{
        std::string_view("--calibration"), std::string_view("stationary-flat"),
        std::string_view("--stationary-seconds"), std::string_view("5"),
        std::string_view("--calibration-output"), std::string_view("stationary-flat.json")};
    const auto result = parse(arguments);
    expect(result.options.has_value(), "existing accelerometer calibration options remain valid");
    expect(result.options->duration == std::chrono::seconds(5),
           "stationary capture duration behavior is preserved");
    expect(result.options->csvPath == "calibration-stationary-flat.csv",
           "stationary capture default CSV behavior is preserved");
}

void testPhysicalUnitOptions()
{
    constexpr std::array valid{
        std::string_view("--gyro-calibrate-seconds"), std::string_view("2"),
        std::string_view("--apply-gyro-bias"),
        std::string_view("--gyro-scale-raw-per-dps"), std::string_view("4090"),
        std::string_view("--print-gyro-degrees"),
        std::string_view("--print-gyro-radians"),
        std::string_view("--compare-q12-scale")};
    const auto result = parse(valid);
    expect(result.options.has_value()
               && result.options->gyroscopeScaleRawPerDegreePerSecond == 4090.0
               && result.options->printGyroscopeDegrees
               && result.options->printGyroscopeRadians,
           "explicit 4090 physical-unit options are accepted");

    constexpr std::array invalidScale{
        std::string_view("--gyro-scale-raw-per-dps"), std::string_view("0")};
    expect(!parse(invalidScale).options.has_value(), "zero scale is rejected by CLI");

    constexpr std::array conflicting{
        std::string_view("--gyro-scale-profile"), std::string_view("profile.json"),
        std::string_view("--gyro-scale-raw-per-dps"), std::string_view("4090")};
    expect(!parse(conflicting).options.has_value(), "conflicting scale inputs are rejected");

    constexpr std::array missingScale{std::string_view("--print-gyro-degrees")};
    expect(!parse(missingScale).options.has_value(),
           "physical-unit printing requires an explicit scale");
    expect(xreal::diagnostics::imuDiagnosticUsage().find("--print-gyro-degrees")
               != std::string::npos,
           "diagnostic usage exposes explicit physical-unit labels");
}

void testOrientationOptions()
{
    constexpr std::array valid{
        std::string_view("--gyro-calibrate-seconds"), std::string_view("2"),
        std::string_view("--apply-gyro-bias"),
        std::string_view("--gyro-scale-profile"), std::string_view("scale.json"),
        std::string_view("--integrate-gyro-orientation"),
        std::string_view("--orientation-output"), std::string_view("both"),
        std::string_view("--recenter-after-seconds"), std::string_view("3.5"),
        std::string_view("--orientation-max-delta-ms"), std::string_view("12.5"),
        std::string_view("--orientation-profile-output"), std::string_view("orientation.json"),
        std::string_view("--orientation-print-rate"), std::string_view("20"),
    };
    const auto result = parse(valid);
    expect(result.options.has_value(), "complete orientation options are accepted");
    expect(result.options->integrateGyroscopeOrientation
               && result.options->orientationOutput
                    == xreal::diagnostics::OrientationOutputMode::both
               && result.options->orientationMaximumDelta == std::chrono::microseconds(12'500)
               && result.options->orientationPrintRateHz == 20U,
           "orientation values are parsed exactly");

    constexpr std::array noScale{
        std::string_view("--gyro-calibrate-seconds"), std::string_view("2"),
        std::string_view("--apply-gyro-bias"),
        std::string_view("--integrate-gyro-orientation")};
    expect(!parse(noScale).options.has_value(), "orientation without a scale is rejected");

    constexpr std::array noBias{
        std::string_view("--gyro-calibrate-seconds"), std::string_view("2"),
        std::string_view("--gyro-scale-raw-per-dps"), std::string_view("4090"),
        std::string_view("--integrate-gyro-orientation")};
    expect(!parse(noBias).options.has_value(), "orientation without applied bias is rejected");

    constexpr std::array invalidOutput{
        std::string_view("--integrate-gyro-orientation"),
        std::string_view("--orientation-output"), std::string_view("matrix")};
    expect(!parse(invalidOutput).options.has_value(), "unknown orientation output mode is rejected");

    constexpr std::array zeroDelta{
        std::string_view("--integrate-gyro-orientation"),
        std::string_view("--orientation-max-delta-ms"), std::string_view("0")};
    expect(!parse(zeroDelta).options.has_value(), "non-positive maximum delta is rejected");

    constexpr std::array optionWithoutMode{
        std::string_view("--orientation-print-rate"), std::string_view("10")};
    expect(!parse(optionWithoutMode).options.has_value(),
           "orientation-specific option without integration mode is rejected");

    constexpr std::array zeroPrintRate{
        std::string_view("--gyro-calibrate-seconds"), std::string_view("2"),
        std::string_view("--apply-gyro-bias"),
        std::string_view("--gyro-scale-raw-per-dps"), std::string_view("4090"),
        std::string_view("--integrate-gyro-orientation"),
        std::string_view("--orientation-print-rate"), std::string_view("0")};
    expect(!parse(zeroPrintRate).options.has_value(),
           "non-positive orientation print rate is rejected");

    constexpr std::array existingBehavior{std::string_view("--duration"), std::string_view("5")};
    const auto existingResult = parse(existingBehavior);
    expect(existingResult.options.has_value()
               && !existingResult.options->integrateGyroscopeOrientation,
           "orientation remains disabled for existing diagnostics");
}

} // namespace

int main()
{
    testValidGyroscopeOptions();
    testInvalidGyroscopeOptions();
    testExistingAccelerometerOptionsRemainValid();
    testPhysicalUnitOptions();
    testOrientationOptions();

    if (failureCount != 0)
    {
        std::cerr << failureCount << " IMU diagnostic option test(s) failed.\n";
        return 1;
    }
    std::cout << "All IMU diagnostic option tests passed.\n";
    return 0;
}
