#include "diagnostics/ImuDiagnosticOptions.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <limits>
#include <vector>

namespace xreal::diagnostics
{
namespace
{

[[nodiscard]] std::optional<unsigned int> parsePositiveInteger(std::string_view value)
{
    unsigned int parsed{};
    const auto result = std::from_chars(value.data(), value.data() + value.size(), parsed);
    if (result.ec != std::errc{} || result.ptr != value.data() + value.size() || parsed == 0U)
    {
        return std::nullopt;
    }
    return parsed;
}

[[nodiscard]] std::optional<double> parsePositiveDouble(std::string_view value)
{
    double parsed{};
    const auto result = std::from_chars(value.data(), value.data() + value.size(), parsed);
    if (result.ec != std::errc{} || result.ptr != value.data() + value.size()
        || !std::isfinite(parsed) || parsed <= 0.0)
    {
        return std::nullopt;
    }
    return parsed;
}

[[nodiscard]] std::optional<double> parseNonNegativeDouble(std::string_view value)
{
    double parsed{};
    const auto result = std::from_chars(value.data(), value.data() + value.size(), parsed);
    if (result.ec != std::errc{} || result.ptr != value.data() + value.size()
        || !std::isfinite(parsed) || parsed < 0.0)
    {
        return std::nullopt;
    }
    return parsed;
}

[[nodiscard]] bool isValueOption(std::string_view argument)
{
    return argument == "--duration"
        || argument == "--print-rate"
        || argument == "--csv"
        || argument == "--calibration"
        || argument == "--stationary-seconds"
        || argument == "--calibration-output"
        || argument == "--gyro-calibrate-seconds"
        || argument == "--gyro-warmup-seconds"
        || argument == "--gyro-calibration-output"
        || argument == "--gyro-scale-profile"
        || argument == "--gyro-scale-raw-per-dps"
        || argument == "--orientation-output"
        || argument == "--recenter-after-seconds"
        || argument == "--orientation-max-delta-ms"
        || argument == "--orientation-profile-output"
        || argument == "--orientation-print-rate"
        || argument == "--fusion-mode"
        || argument == "--accelerometer-profile"
        || argument == "--accelerometer-correction-time-constant"
        || argument == "--accelerometer-max-correction-dps"
        || argument == "--accelerometer-confidence-full-deviation-g"
        || argument == "--accelerometer-confidence-zero-deviation-g"
        || argument == "--accelerometer-confidence-smoothing-seconds"
        || argument == "--fusion-startup"
        || argument == "--fusion-output"
        || argument == "--fusion-print-rate"
        || argument == "--fusion-json-output"
        || argument == "--experiment-stationary-before-seconds"
        || argument == "--experiment-motion-seconds"
        || argument == "--experiment-stationary-after-seconds"
        || argument == "--experiment-recenter-seconds"
        || argument == "--stationary-gyro-threshold-dps"
        || argument == "--stationary-accel-deviation-g"
        || argument == "--stationary-min-duration-seconds"
        || argument == "--convergence-thresholds-degrees"
        || argument == "--convergence-sustain-seconds"
        || argument == "--comparison-output"
        || argument == "--comparison-print-rate"
        || argument == "--comparison-json-output"
        || argument == "--comparison-csv-output"
        || argument == "--gyro-max-stddev"
        || argument == "--gyro-max-range"
        || argument == "--gyro-min-samples"
        || argument == "--prediction-mode"
        || argument == "--prediction-horizon-ms"
        || argument == "--prediction-max-horizon-ms"
        || argument == "--prediction-angular-velocity-smoothing-seconds"
        || argument == "--prediction-angular-acceleration-smoothing-seconds"
        || argument == "--prediction-max-angular-speed-dps"
        || argument == "--prediction-max-angular-acceleration-dps2"
        || argument == "--prediction-max-angle-degrees"
        || argument == "--prediction-limit-behavior"
        || argument == "--prediction-evaluation-tolerance-ms"
        || argument == "--prediction-output"
        || argument == "--prediction-print-rate"
        || argument == "--prediction-json-output"
        || argument == "--prediction-csv-output";
}

} // namespace

ImuDiagnosticOptionResult parseImuDiagnosticOptions(
    std::span<const std::string_view> arguments)
{
    ImuDiagnosticOptions options;
    sensors::GyroscopeBiasCalibrationConfig gyroscopeConfiguration;
    bool gyroscopeConfigurationCustomized{};

    for (std::size_t index = 0; index < arguments.size(); ++index)
    {
        const std::string_view argument = arguments[index];
        if (argument == "--help" || argument == "-h")
        {
            return {options, {}, true};
        }
        if (argument == "--verbose")
        {
            options.verbose = true;
            continue;
        }
        if (argument == "--apply-gyro-bias")
        {
            options.applyGyroscopeBias = true;
            continue;
        }
        if (argument == "--print-gyro-degrees")
        {
            options.printGyroscopeDegrees = true;
            continue;
        }
        if (argument == "--print-gyro-radians")
        {
            options.printGyroscopeRadians = true;
            continue;
        }
        if (argument == "--compare-q12-scale")
        {
            options.compareQ12Scale = true;
            continue;
        }
        if (argument == "--integrate-gyro-orientation")
        {
            options.integrateGyroscopeOrientation = true;
            continue;
        }
        if (argument == "--fuse-gyro-accelerometer")
        {
            options.fuseGyroscopeAccelerometer = true;
            continue;
        }
        if (argument == "--compare-gyro-and-fusion")
        {
            options.compareGyroscopeAndFusion = true;
            continue;
        }
        if (argument == "--orientation-comparison-experiment")
        {
            options.orientationComparisonExperiment = true;
            options.comparisonOptionExplicit = true;
            continue;
        }
        if (argument == "--predict-orientation")
        {
            options.predictOrientation = true;
            continue;
        }
        if (argument == "--prediction-evaluate-delayed")
        {
            options.predictionEvaluateDelayed = true;
            options.predictionOptionExplicit = true;
            continue;
        }
        if (argument == "--print-accelerometer-physical")
        {
            options.printAccelerometerPhysical = true;
            options.fusionOptionExplicit = true;
            continue;
        }
        if (argument == "--print-fusion-diagnostics")
        {
            options.printFusionDiagnostics = true;
            options.fusionOptionExplicit = true;
            continue;
        }
        if (!isValueOption(argument))
        {
            return {std::nullopt, "Unknown option: " + std::string(argument), false};
        }
        if (++index >= arguments.size())
        {
            return {std::nullopt, "Missing value for " + std::string(argument), false};
        }

        const std::string_view value = arguments[index];
        if (argument == "--csv"
            || argument == "--calibration-output"
            || argument == "--gyro-calibration-output"
            || argument == "--gyro-scale-profile"
            || argument == "--orientation-profile-output"
            || argument == "--accelerometer-profile"
            || argument == "--fusion-json-output"
            || argument == "--comparison-json-output"
            || argument == "--comparison-csv-output"
            || argument == "--prediction-json-output"
            || argument == "--prediction-csv-output")
        {
            if (value.empty())
            {
                return {std::nullopt, std::string(argument) + " requires a non-empty file path.", false};
            }
            if (argument == "--csv")
            {
                options.csvPath = value;
            }
            else if (argument == "--calibration-output")
            {
                options.calibrationOutputPath = value;
            }
            else
            {
                if (argument == "--gyro-calibration-output")
                {
                    options.gyroscopeCalibrationOutputPath = value;
                }
                else
                {
                    if (argument == "--gyro-scale-profile")
                    {
                        options.gyroscopeScaleProfilePath = value;
                    }
                    else if (argument == "--orientation-profile-output")
                    {
                        options.orientationProfileOutputPath = value;
                    }
                    else if (argument == "--accelerometer-profile")
                    {
                        options.accelerometerProfilePath = value;
                        options.fusionOptionExplicit = true;
                    }
                    else if (argument == "--comparison-json-output")
                    {
                        options.comparisonJsonOutputPath = value;
                        options.comparisonOptionExplicit = true;
                    }
                    else if (argument == "--comparison-csv-output")
                    {
                        options.comparisonCsvOutputPath = value;
                        options.comparisonOptionExplicit = true;
                    }
                    else if (argument == "--prediction-json-output")
                    {
                        options.predictionJsonOutputPath = value;
                        options.predictionOptionExplicit = true;
                    }
                    else if (argument == "--prediction-csv-output")
                    {
                        options.predictionCsvOutputPath = value;
                        options.predictionOptionExplicit = true;
                    }
                    else
                    {
                        options.fusionJsonOutputPath = value;
                        options.fusionOptionExplicit = true;
                    }
                }
            }
            continue;
        }

        if (argument == "--convergence-thresholds-degrees")
        {
            std::vector<double> thresholds;
            std::size_t begin{};
            while (begin < value.size())
            {
                const std::size_t comma = value.find(',', begin);
                const std::string_view token = value.substr(
                    begin, comma == std::string_view::npos ? value.size() - begin : comma - begin);
                const auto parsed = parsePositiveDouble(token);
                if (!parsed.has_value()
                    || (!thresholds.empty() && *parsed >= thresholds.back()))
                {
                    return {std::nullopt,
                            "--convergence-thresholds-degrees requires a strictly decreasing CSV of positive values.",
                            false};
                }
                thresholds.push_back(*parsed);
                if (comma == std::string_view::npos)
                {
                    break;
                }
                begin = comma + 1U;
            }
            if (thresholds.empty())
            {
                return {std::nullopt, "--convergence-thresholds-degrees cannot be empty.", false};
            }
            options.convergenceThresholdsDegrees = std::move(thresholds);
            options.comparisonOptionExplicit = true;
            continue;
        }

        if (argument == "--comparison-output")
        {
            if (value == "quaternion")
            {
                options.comparisonOutput = OrientationOutputMode::quaternion;
            }
            else if (value == "euler")
            {
                options.comparisonOutput = OrientationOutputMode::euler;
            }
            else if (value == "both")
            {
                options.comparisonOutput = OrientationOutputMode::both;
            }
            else
            {
                return {std::nullopt, "--comparison-output requires quaternion, euler or both.", false};
            }
            options.comparisonOptionExplicit = true;
            continue;
        }

        if (argument == "--prediction-mode")
        {
            if (value == "constant-velocity")
            {
                options.predictionMode = sensors::PredictionMode::constantVelocity;
            }
            else if (value == "constant-acceleration")
            {
                options.predictionMode = sensors::PredictionMode::constantAcceleration;
            }
            else
            {
                return {std::nullopt,
                        "--prediction-mode requires constant-velocity or constant-acceleration.", false};
            }
            options.predictionOptionExplicit = true;
            continue;
        }
        if (argument == "--prediction-limit-behavior")
        {
            if (value == "reject")
            {
                options.predictionLimitBehavior = sensors::PredictionLimitBehavior::reject;
            }
            else if (value == "clamp")
            {
                options.predictionLimitBehavior = sensors::PredictionLimitBehavior::clamp;
            }
            else
            {
                return {std::nullopt, "--prediction-limit-behavior requires reject or clamp.", false};
            }
            options.predictionOptionExplicit = true;
            continue;
        }
        if (argument == "--prediction-output")
        {
            if (value == "quaternion") { options.predictionOutput = OrientationOutputMode::quaternion; }
            else if (value == "euler") { options.predictionOutput = OrientationOutputMode::euler; }
            else if (value == "both") { options.predictionOutput = OrientationOutputMode::both; }
            else
            {
                return {std::nullopt, "--prediction-output requires quaternion, euler or both.", false};
            }
            options.predictionOptionExplicit = true;
            continue;
        }
        if (argument == "--prediction-horizon-ms"
            || argument == "--prediction-max-horizon-ms"
            || argument == "--prediction-angular-velocity-smoothing-seconds"
            || argument == "--prediction-angular-acceleration-smoothing-seconds"
            || argument == "--prediction-max-angular-speed-dps"
            || argument == "--prediction-max-angular-acceleration-dps2"
            || argument == "--prediction-max-angle-degrees"
            || argument == "--prediction-evaluation-tolerance-ms")
        {
            const bool permitsZero = argument == "--prediction-horizon-ms"
                || argument == "--prediction-angular-velocity-smoothing-seconds"
                || argument == "--prediction-angular-acceleration-smoothing-seconds";
            const auto parsed = permitsZero ? parseNonNegativeDouble(value) : parsePositiveDouble(value);
            if (!parsed.has_value())
            {
                return {std::nullopt, std::string(argument) + " requires a finite "
                        + (permitsZero ? "non-negative" : "positive") + " value.", false};
            }
            if (argument == "--prediction-horizon-ms") { options.predictionHorizonMilliseconds = *parsed; }
            else if (argument == "--prediction-max-horizon-ms") { options.predictionMaximumHorizonMilliseconds = *parsed; }
            else if (argument == "--prediction-angular-velocity-smoothing-seconds") { options.predictionAngularVelocitySmoothingSeconds = *parsed; }
            else if (argument == "--prediction-angular-acceleration-smoothing-seconds") { options.predictionAngularAccelerationSmoothingSeconds = *parsed; }
            else if (argument == "--prediction-max-angular-speed-dps") { options.predictionMaximumAngularSpeedDegreesPerSecond = *parsed; }
            else if (argument == "--prediction-max-angular-acceleration-dps2") { options.predictionMaximumAngularAccelerationDegreesPerSecondSquared = *parsed; }
            else if (argument == "--prediction-max-angle-degrees") { options.predictionMaximumAngleDegrees = *parsed; }
            else { options.predictionEvaluationToleranceMilliseconds = *parsed; }
            options.predictionOptionExplicit = true;
            continue;
        }

        if (argument == "--experiment-stationary-before-seconds"
            || argument == "--experiment-motion-seconds"
            || argument == "--experiment-stationary-after-seconds"
            || argument == "--experiment-recenter-seconds"
            || argument == "--stationary-gyro-threshold-dps"
            || argument == "--stationary-accel-deviation-g"
            || argument == "--stationary-min-duration-seconds"
            || argument == "--convergence-sustain-seconds")
        {
            const auto parsed = parsePositiveDouble(value);
            if (!parsed.has_value())
            {
                return {std::nullopt, std::string(argument) + " requires a finite positive value.", false};
            }
            if (argument == "--experiment-stationary-before-seconds")
            {
                options.experimentStationaryBeforeSeconds = *parsed;
            }
            else if (argument == "--experiment-motion-seconds")
            {
                options.experimentMotionSeconds = *parsed;
            }
            else if (argument == "--experiment-stationary-after-seconds")
            {
                options.experimentStationaryAfterSeconds = *parsed;
            }
            else if (argument == "--experiment-recenter-seconds")
            {
                options.experimentRecenterSeconds = *parsed;
            }
            else if (argument == "--stationary-gyro-threshold-dps")
            {
                options.stationaryGyroscopeThresholdDegreesPerSecond = *parsed;
            }
            else if (argument == "--stationary-accel-deviation-g")
            {
                options.stationaryAccelerometerDeviationG = *parsed;
            }
            else if (argument == "--stationary-min-duration-seconds")
            {
                options.stationaryMinimumDurationSeconds = *parsed;
            }
            else
            {
                options.convergenceSustainSeconds = *parsed;
            }
            options.comparisonOptionExplicit = true;
            continue;
        }

        if (argument == "--gyro-scale-raw-per-dps")
        {
            const auto parsed = parsePositiveDouble(value);
            if (!parsed.has_value())
            {
                return {std::nullopt, "--gyro-scale-raw-per-dps requires a finite positive value.", false};
            }
            options.gyroscopeScaleRawPerDegreePerSecond = *parsed;
            continue;
        }

        if (argument == "--orientation-output")
        {
            if (value == "quaternion")
            {
                options.orientationOutput = OrientationOutputMode::quaternion;
            }
            else if (value == "euler")
            {
                options.orientationOutput = OrientationOutputMode::euler;
            }
            else if (value == "both")
            {
                options.orientationOutput = OrientationOutputMode::both;
            }
            else
            {
                return {std::nullopt,
                        "--orientation-output requires quaternion, euler or both.",
                        false};
            }
            options.orientationOutputExplicit = true;
            continue;
        }

        if (argument == "--fusion-mode")
        {
            if (value == "gyro-only")
            {
                options.fusionMode = FusionMode::gyroOnly;
            }
            else if (value == "complementary")
            {
                options.fusionMode = FusionMode::complementary;
            }
            else
            {
                return {std::nullopt, "--fusion-mode requires gyro-only or complementary.", false};
            }
            options.fusionModeExplicit = true;
            options.fusionOptionExplicit = true;
            continue;
        }

        if (argument == "--fusion-startup")
        {
            if (value == "identity")
            {
                options.fusionStartup = FusionStartupMode::identity;
            }
            else if (value == "gravity")
            {
                options.fusionStartup = FusionStartupMode::gravity;
            }
            else
            {
                return {std::nullopt, "--fusion-startup requires identity or gravity.", false};
            }
            options.fusionOptionExplicit = true;
            continue;
        }

        if (argument == "--fusion-output")
        {
            if (value == "quaternion")
            {
                options.fusionOutput = OrientationOutputMode::quaternion;
            }
            else if (value == "euler")
            {
                options.fusionOutput = OrientationOutputMode::euler;
            }
            else if (value == "both")
            {
                options.fusionOutput = OrientationOutputMode::both;
            }
            else
            {
                return {std::nullopt, "--fusion-output requires quaternion, euler or both.", false};
            }
            options.fusionOptionExplicit = true;
            continue;
        }

        if (argument == "--accelerometer-correction-time-constant"
            || argument == "--accelerometer-max-correction-dps"
            || argument == "--accelerometer-confidence-full-deviation-g"
            || argument == "--accelerometer-confidence-zero-deviation-g"
            || argument == "--accelerometer-confidence-smoothing-seconds")
        {
            const auto parsed = parsePositiveDouble(value);
            if (!parsed.has_value())
            {
                return {std::nullopt, std::string(argument) + " requires a finite positive value.", false};
            }
            if (argument == "--accelerometer-correction-time-constant")
            {
                options.accelerometerCorrectionTimeConstantSeconds = *parsed;
            }
            else if (argument == "--accelerometer-max-correction-dps")
            {
                options.accelerometerMaximumCorrectionDegreesPerSecond = *parsed;
            }
            else if (argument == "--accelerometer-confidence-full-deviation-g")
            {
                options.accelerometerFullConfidenceDeviationG = *parsed;
            }
            else if (argument == "--accelerometer-confidence-zero-deviation-g")
            {
                options.accelerometerZeroConfidenceDeviationG = *parsed;
            }
            else
            {
                options.accelerometerConfidenceSmoothingSeconds = *parsed;
            }
            options.fusionOptionExplicit = true;
            continue;
        }

        if (argument == "--recenter-after-seconds" || argument == "--orientation-max-delta-ms")
        {
            const auto parsed = parsePositiveDouble(value);
            if (!parsed.has_value())
            {
                return {std::nullopt, std::string(argument) + " requires a finite positive value.", false};
            }
            if (argument == "--recenter-after-seconds")
            {
                options.recenterAfterSeconds = *parsed;
            }
            else
            {
                const double nanoseconds = *parsed * 1'000'000.0;
                if (nanoseconds > static_cast<double>(std::numeric_limits<std::int64_t>::max()))
                {
                    return {std::nullopt, "--orientation-max-delta-ms is too large.", false};
                }
                options.orientationMaximumDelta = std::chrono::nanoseconds(
                    static_cast<std::int64_t>(nanoseconds));
                options.orientationMaximumDeltaExplicit = true;
            }
            continue;
        }

        if (argument == "--calibration")
        {
            const bool validName = !value.empty()
                && std::all_of(value.begin(), value.end(), [](char character) {
                       return (character >= 'a' && character <= 'z')
                           || (character >= 'A' && character <= 'Z')
                           || (character >= '0' && character <= '9')
                           || character == '-'
                           || character == '_';
                   });
            if (!validName)
            {
                return {std::nullopt,
                        "--calibration must contain only letters, digits, '-' or '_'.",
                        false};
            }
            options.calibrationName = value;
            continue;
        }

        if (argument == "--gyro-max-stddev" || argument == "--gyro-max-range")
        {
            const auto parsed = parsePositiveDouble(value);
            if (!parsed.has_value())
            {
                return {std::nullopt, "Invalid positive number for " + std::string(argument), false};
            }
            if (argument == "--gyro-max-stddev")
            {
                gyroscopeConfiguration.maximumStandardDeviationRaw = *parsed;
            }
            else
            {
                gyroscopeConfiguration.maximumRangeRaw = *parsed;
            }
            gyroscopeConfigurationCustomized = true;
            continue;
        }

        const auto parsed = parsePositiveInteger(value);
        if (!parsed.has_value())
        {
            return {std::nullopt, "Invalid positive integer for " + std::string(argument), false};
        }

        if (argument == "--duration")
        {
            options.duration = std::chrono::seconds(*parsed);
            options.durationExplicit = true;
        }
        else if (argument == "--stationary-seconds")
        {
            options.stationaryDuration = std::chrono::seconds(*parsed);
            options.stationaryDurationExplicit = true;
        }
        else if (argument == "--gyro-calibrate-seconds")
        {
            if (*parsed > 3600U)
            {
                return {std::nullopt, "--gyro-calibrate-seconds must not exceed 3600 seconds.", false};
            }
            gyroscopeConfiguration.calibrationDuration = std::chrono::seconds(*parsed);
            options.gyroscopeCalibration = gyroscopeConfiguration;
        }
        else if (argument == "--gyro-warmup-seconds")
        {
            if (*parsed > 3600U)
            {
                return {std::nullopt, "--gyro-warmup-seconds must not exceed 3600 seconds.", false};
            }
            gyroscopeConfiguration.warmupDuration = std::chrono::seconds(*parsed);
            gyroscopeConfigurationCustomized = true;
        }
        else if (argument == "--gyro-min-samples")
        {
            gyroscopeConfiguration.minimumSampleCount = *parsed;
            gyroscopeConfigurationCustomized = true;
        }
        else if (argument == "--orientation-print-rate")
        {
            if (*parsed > 100U)
            {
                return {std::nullopt, "--orientation-print-rate must be between 1 and 100 Hz.", false};
            }
            options.orientationPrintRateHz = *parsed;
            options.orientationPrintRateExplicit = true;
        }
        else if (argument == "--fusion-print-rate")
        {
            if (*parsed > 100U)
            {
                return {std::nullopt, "--fusion-print-rate must be between 1 and 100 Hz.", false};
            }
            options.fusionPrintRateHz = *parsed;
            options.fusionOptionExplicit = true;
        }
        else if (argument == "--comparison-print-rate")
        {
            if (*parsed > 100U)
            {
                return {std::nullopt, "--comparison-print-rate must be between 1 and 100 Hz.", false};
            }
            options.comparisonPrintRateHz = *parsed;
            options.comparisonOptionExplicit = true;
        }
        else if (argument == "--prediction-print-rate")
        {
            if (*parsed > 100U)
            {
                return {std::nullopt, "--prediction-print-rate must be between 1 and 100 Hz.", false};
            }
            options.predictionPrintRateHz = *parsed;
            options.predictionOptionExplicit = true;
        }
        else if (*parsed > 10U)
        {
            return {std::nullopt, "--print-rate must be between 1 and 10 Hz.", false};
        }
        else
        {
            options.printRateHz = *parsed;
        }
    }

    if (options.calibrationOutputPath.has_value() && !options.calibrationName.has_value())
    {
        return {std::nullopt, "--calibration-output requires --calibration.", false};
    }
    if (options.stationaryDurationExplicit && !options.calibrationName.has_value())
    {
        return {std::nullopt, "--stationary-seconds requires --calibration.", false};
    }
    if (options.calibrationName.has_value())
    {
        const bool rotational = options.calibrationName->ends_with("-rotation");
        if (!rotational && !options.durationExplicit)
        {
            options.duration = options.stationaryDuration;
        }
        if (!options.csvPath.has_value())
        {
            options.csvPath = "calibration-" + *options.calibrationName + ".csv";
        }
    }

    if (!options.gyroscopeCalibration.has_value() && gyroscopeConfigurationCustomized)
    {
        return {std::nullopt, "Gyroscope thresholds require --gyro-calibrate-seconds.", false};
    }
    if (options.gyroscopeCalibration.has_value())
    {
        gyroscopeConfiguration.calibrationDuration = options.gyroscopeCalibration->calibrationDuration;
        options.gyroscopeCalibration = gyroscopeConfiguration;
    }
    if (options.applyGyroscopeBias && !options.gyroscopeCalibration.has_value())
    {
        return {std::nullopt, "--apply-gyro-bias requires --gyro-calibrate-seconds.", false};
    }
    if (options.gyroscopeCalibrationOutputPath.has_value() && !options.gyroscopeCalibration.has_value())
    {
        return {std::nullopt, "--gyro-calibration-output requires --gyro-calibrate-seconds.", false};
    }
    if (options.gyroscopeScaleProfilePath.has_value()
        && options.gyroscopeScaleRawPerDegreePerSecond.has_value())
    {
        return {std::nullopt, "Use either --gyro-scale-profile or --gyro-scale-raw-per-dps, not both.", false};
    }
    const bool physicalOutput = options.printGyroscopeDegrees || options.printGyroscopeRadians;
    const bool hasScale = options.gyroscopeScaleProfilePath.has_value()
        || options.gyroscopeScaleRawPerDegreePerSecond.has_value();
    if (physicalOutput && !hasScale)
    {
        return {std::nullopt, "Physical gyroscope output requires an explicit scale profile or scalar.", false};
    }
    if (physicalOutput && (!options.applyGyroscopeBias
        || !options.gyroscopeCalibration.has_value()))
    {
        return {std::nullopt, "Physical gyroscope output requires runtime bias calibration and --apply-gyro-bias.", false};
    }
    if (options.compareQ12Scale && !hasScale)
    {
        return {std::nullopt, "--compare-q12-scale requires an explicit gyroscope scale.", false};
    }
    const bool hasOrientationOption = options.orientationOutputExplicit
        || options.orientationMaximumDeltaExplicit
        || options.orientationProfileOutputPath.has_value()
        || options.orientationPrintRateExplicit;
    if (hasOrientationOption && !options.integrateGyroscopeOrientation)
    {
        return {std::nullopt,
                "Orientation-specific options require --integrate-gyro-orientation.",
                false};
    }
    if (options.integrateGyroscopeOrientation && !hasScale)
    {
        return {std::nullopt,
                "Gyroscope orientation integration requires an explicit scale profile or scalar.",
                false};
    }
    if (options.integrateGyroscopeOrientation
        && (!options.applyGyroscopeBias || !options.gyroscopeCalibration.has_value()))
    {
        return {std::nullopt,
                "Gyroscope orientation integration requires same-run bias calibration and --apply-gyro-bias.",
                false};
    }
    if (options.comparisonOptionExplicit && !options.compareGyroscopeAndFusion)
    {
        return {std::nullopt, "Comparison-specific options require --compare-gyro-and-fusion.", false};
    }
    if (options.fusionOptionExplicit
        && !options.fuseGyroscopeAccelerometer && !options.compareGyroscopeAndFusion)
    {
        return {std::nullopt, "Fusion-specific options require --fuse-gyro-accelerometer.", false};
    }
    if (options.integrateGyroscopeOrientation && options.fuseGyroscopeAccelerometer)
    {
        return {std::nullopt,
                "Choose either --integrate-gyro-orientation or --fuse-gyro-accelerometer.",
                false};
    }
    if (options.compareGyroscopeAndFusion
        && (options.integrateGyroscopeOrientation || options.fuseGyroscopeAccelerometer))
    {
        return {std::nullopt,
                "--compare-gyro-and-fusion cannot be combined with either independent orientation mode.",
                false};
    }
    if (options.compareGyroscopeAndFusion && options.recenterAfterSeconds.has_value())
    {
        return {std::nullopt,
                "Comparison mode uses --experiment-recenter-seconds, not --recenter-after-seconds.",
                false};
    }
    if (options.recenterAfterSeconds.has_value()
        && !options.integrateGyroscopeOrientation && !options.fuseGyroscopeAccelerometer
        && !options.compareGyroscopeAndFusion)
    {
        return {std::nullopt,
                "--recenter-after-seconds requires an orientation or fusion mode.",
                false};
    }
    if (options.fuseGyroscopeAccelerometer && !hasScale)
    {
        return {std::nullopt, "Fusion requires an explicit gyroscope scale.", false};
    }
    if (options.fuseGyroscopeAccelerometer
        && (!options.applyGyroscopeBias || !options.gyroscopeCalibration.has_value()))
    {
        return {std::nullopt,
                "Fusion requires same-run gyroscope bias calibration and --apply-gyro-bias.",
                false};
    }
    if (options.compareGyroscopeAndFusion && !hasScale)
    {
        return {std::nullopt, "Comparison mode requires an explicit gyroscope scale.", false};
    }
    if (options.compareGyroscopeAndFusion
        && (!options.applyGyroscopeBias || !options.gyroscopeCalibration.has_value()))
    {
        return {std::nullopt,
                "Comparison mode requires same-run gyroscope bias calibration and --apply-gyro-bias.",
                false};
    }
    if (options.compareGyroscopeAndFusion && !options.accelerometerProfilePath.has_value())
    {
        return {std::nullopt,
                "Comparison mode requires --accelerometer-profile; no fallback is used.",
                false};
    }
    const double experimentDurationSeconds = options.experimentStationaryBeforeSeconds
        + options.experimentMotionSeconds + options.experimentStationaryAfterSeconds;
    if (options.orientationComparisonExperiment
        && options.experimentRecenterSeconds > experimentDurationSeconds)
    {
        return {std::nullopt,
                "--experiment-recenter-seconds must be within the configured experiment duration.",
                false};
    }
    if (options.fuseGyroscopeAccelerometer
        && options.fusionMode == FusionMode::complementary
        && !options.accelerometerProfilePath.has_value())
    {
        return {std::nullopt,
                "Complementary fusion requires --accelerometer-profile; no fallback is used.",
                false};
    }
    if (options.accelerometerFullConfidenceDeviationG
        >= options.accelerometerZeroConfidenceDeviationG)
    {
        return {std::nullopt,
                "Full-confidence acceleration deviation must be smaller than zero-confidence deviation.",
                false};
    }
    if (options.predictionOptionExplicit && !options.predictOrientation)
    {
        return {std::nullopt, "Prediction-specific options require --predict-orientation.", false};
    }
    if (options.predictOrientation
        && !options.fuseGyroscopeAccelerometer && !options.compareGyroscopeAndFusion)
    {
        return {std::nullopt,
                "--predict-orientation requires complementary fusion or comparison mode.", false};
    }
    if (options.predictOrientation && options.fuseGyroscopeAccelerometer
        && options.fusionMode != FusionMode::complementary)
    {
        return {std::nullopt, "Orientation prediction requires complementary fusion.", false};
    }
    if (options.predictionHorizonMilliseconds > options.predictionMaximumHorizonMilliseconds
        && options.predictionLimitBehavior == sensors::PredictionLimitBehavior::reject)
    {
        return {std::nullopt,
                "Prediction horizon exceeds the maximum; select clamp explicitly to bound it.", false};
    }

    return {options, {}, false};
}

std::string imuDiagnosticUsage()
{
    return "Usage: xreal-imu-diagnostic [--duration <seconds>] [--print-rate <hz>]"
           " [--csv <file>] [--calibration <name>] [--stationary-seconds <seconds>]"
           " [--calibration-output <file.json>] [--gyro-calibrate-seconds <seconds>]"
           " [--gyro-warmup-seconds <seconds>]"
           " [--apply-gyro-bias] [--gyro-calibration-output <file.json>]"
           " [--gyro-max-stddev <raw-units>] [--gyro-max-range <raw-units>]"
           " [--gyro-min-samples <count>]"
           " [--gyro-scale-profile <file.json> | --gyro-scale-raw-per-dps <value>]"
           " [--print-gyro-degrees] [--print-gyro-radians] [--compare-q12-scale]"
           " [--integrate-gyro-orientation]"
           " [--orientation-output <quaternion|euler|both>]"
           " [--recenter-after-seconds <seconds>] [--orientation-max-delta-ms <value>]"
           " [--orientation-profile-output <file.json>] [--orientation-print-rate <hz>]"
           " [--fuse-gyro-accelerometer] [--fusion-mode <gyro-only|complementary>]"
           " [--accelerometer-profile <file.json>]"
           " [--accelerometer-correction-time-constant <seconds>]"
           " [--accelerometer-max-correction-dps <value>]"
           " [--accelerometer-confidence-full-deviation-g <value>]"
           " [--accelerometer-confidence-zero-deviation-g <value>]"
           " [--accelerometer-confidence-smoothing-seconds <value>]"
           " [--fusion-startup <identity|gravity>] [--print-accelerometer-physical]"
           " [--print-fusion-diagnostics] [--fusion-output <quaternion|euler|both>]"
           " [--fusion-print-rate <hz>] [--fusion-json-output <file.json>]"
           " [--compare-gyro-and-fusion] [--orientation-comparison-experiment]"
           " [--experiment-stationary-before-seconds <value>]"
           " [--experiment-motion-seconds <value>]"
           " [--experiment-stationary-after-seconds <value>]"
           " [--experiment-recenter-seconds <value>]"
           " [--stationary-gyro-threshold-dps <value>]"
           " [--stationary-accel-deviation-g <value>]"
           " [--stationary-min-duration-seconds <value>]"
           " [--convergence-thresholds-degrees <csv>]"
           " [--convergence-sustain-seconds <value>]"
           " [--comparison-output <quaternion|euler|both>]"
           " [--comparison-print-rate <hz>] [--comparison-json-output <file.json>]"
           " [--comparison-csv-output <file.csv>]"
           " [--predict-orientation]"
           " [--prediction-mode <constant-velocity|constant-acceleration>]"
           " [--prediction-horizon-ms <value>] [--prediction-max-horizon-ms <value>]"
           " [--prediction-angular-velocity-smoothing-seconds <value>]"
           " [--prediction-angular-acceleration-smoothing-seconds <value>]"
           " [--prediction-max-angular-speed-dps <value>]"
           " [--prediction-max-angular-acceleration-dps2 <value>]"
           " [--prediction-max-angle-degrees <value>]"
           " [--prediction-limit-behavior <reject|clamp>] [--prediction-evaluate-delayed]"
           " [--prediction-evaluation-tolerance-ms <value>]"
           " [--prediction-output <quaternion|euler|both>] [--prediction-print-rate <hz>]"
           " [--prediction-json-output <file.json>] [--prediction-csv-output <file.csv>]"
           " [--verbose]\n";
}

} // namespace xreal::diagnostics
