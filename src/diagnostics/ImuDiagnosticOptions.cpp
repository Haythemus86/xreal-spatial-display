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
        || argument == "--gyro-calibration-output"
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
            || argument == "--gyro-calibration-output")
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
                options.gyroscopeCalibrationOutputPath = value;
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
        else if (argument == "--gyro-min-samples")
        {
            gyroscopeConfiguration.minimumSampleCount = *parsed;
            gyroscopeConfigurationCustomized = true;
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

    return {options, {}, false};
}

std::string imuDiagnosticUsage()
{
    return "Usage: xreal-imu-diagnostic [--duration <seconds>] [--print-rate <hz>]"
           " [--csv <file>] [--calibration <name>] [--stationary-seconds <seconds>]"
           " [--calibration-output <file.json>] [--gyro-calibrate-seconds <seconds>]"
           " [--apply-gyro-bias] [--gyro-calibration-output <file.json>]"
           " [--gyro-max-stddev <raw-units>] [--gyro-max-range <raw-units>]"
           " [--gyro-min-samples <count>] [--verbose]\n";
}

} // namespace xreal::diagnostics
