#include "diagnostics/ImuDiagnosticOptions.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <limits>

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
        || argument == "--gyro-max-stddev"
        || argument == "--gyro-max-range"
        || argument == "--gyro-min-samples";
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
            || argument == "--orientation-profile-output")
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
                    else
                    {
                        options.orientationProfileOutputPath = value;
                    }
                }
            }
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
        || options.recenterAfterSeconds.has_value()
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
           " [--verbose]\n";
}

} // namespace xreal::diagnostics
