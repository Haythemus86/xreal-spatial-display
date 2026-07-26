#include "diagnostics/GyroScaleCalibrationOptions.hpp"

#include <charconv>
#include <chrono>
#include <cmath>
#include <utility>

namespace xreal::diagnostics
{
namespace
{

[[nodiscard]] std::optional<double> positiveDouble(std::string_view value)
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

[[nodiscard]] std::optional<std::uint64_t> positiveInteger(std::string_view value)
{
    std::uint64_t parsed{};
    const auto result = std::from_chars(value.data(), value.data() + value.size(), parsed);
    if (result.ec != std::errc{} || result.ptr != value.data() + value.size() || parsed == 0U)
    {
        return std::nullopt;
    }
    return parsed;
}

[[nodiscard]] std::optional<std::uint64_t> nonNegativeInteger(std::string_view value)
{
    std::uint64_t parsed{};
    const auto result = std::from_chars(value.data(), value.data() + value.size(), parsed);
    if (result.ec != std::errc{} || result.ptr != value.data() + value.size())
    {
        return std::nullopt;
    }
    return parsed;
}

[[nodiscard]] std::chrono::nanoseconds seconds(double value)
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(value));
}

[[nodiscard]] GyroScaleCalibrationOptionResult error(std::string message)
{
    return {std::nullopt, std::move(message), false};
}

} // namespace

GyroScaleCalibrationOptionResult parseGyroScaleCalibrationOptions(
    std::span<const std::string_view> arguments)
{
    GyroScaleCalibrationOptions options;
    options.biasConfiguration.warmupDuration = std::chrono::seconds(1);
    bool axisProvided{};
    bool expectedAngleProvided{};
    bool recordDurationProvided{};

    for (std::size_t index = 0; index < arguments.size(); ++index)
    {
        const std::string_view argument = arguments[index];
        if (argument == "--help" || argument == "-h")
        {
            return {options, {}, true};
        }
        if (argument == "--record-only")
        {
            options.recordOnly = true;
            continue;
        }
        if (argument == "--include-accelerometer")
        {
            options.includeAccelerometer = true;
            continue;
        }
        if (argument == "--include-host-timestamps")
        {
            options.includeHostTimestamps = true;
            continue;
        }
        if (++index >= arguments.size())
        {
            return error("Missing value for " + std::string(argument));
        }
        const std::string_view value = arguments[index];

        if (argument == "--axis")
        {
            if (value == "x")
            {
                options.scaleConfiguration.axis = sensors::GyroscopeAxis::x;
                options.recordingAnalysisConfiguration.axisSelection =
                    sensors::GyroscopeAxisSelection::x;
            }
            else if (value == "y")
            {
                options.scaleConfiguration.axis = sensors::GyroscopeAxis::y;
                options.recordingAnalysisConfiguration.axisSelection =
                    sensors::GyroscopeAxisSelection::y;
            }
            else if (value == "z")
            {
                options.scaleConfiguration.axis = sensors::GyroscopeAxis::z;
                options.recordingAnalysisConfiguration.axisSelection =
                    sensors::GyroscopeAxisSelection::z;
            }
            else if (value == "auto")
            {
                options.recordingAnalysisConfiguration.axisSelection =
                    sensors::GyroscopeAxisSelection::automatic;
            }
            else
            {
                return error("--axis must be x, y, z or auto in record-only mode.");
            }
            axisProvided = true;
        }
        else if (argument == "--direction")
        {
            if (value == "positive")
            {
                options.scaleConfiguration.direction = sensors::RotationDirection::positive;
            }
            else if (value == "negative")
            {
                options.scaleConfiguration.direction = sensors::RotationDirection::negative;
            }
            else if (value == "auto")
            {
                options.scaleConfiguration.direction = sensors::RotationDirection::automatic;
            }
            else
            {
                return error("--direction must be positive, negative or auto.");
            }
            options.recordingAnalysisConfiguration.requestedDirection =
                options.scaleConfiguration.direction;
        }
        else if (argument == "--output" || argument == "--csv-prefix"
                 || argument == "--csv-output" || argument == "--analysis-output")
        {
            if (value.empty())
            {
                return error(std::string(argument) + " requires a non-empty value.");
            }
            if (argument == "--output") options.outputPath = value;
            else if (argument == "--csv-prefix") options.csvPrefix = value;
            else if (argument == "--csv-output") options.csvOutputPath = value;
            else options.analysisOutputPath = value;
        }
        else if (argument == "--trials" || argument == "--min-accepted-trials"
                 || argument == "--print-rate")
        {
            const auto parsed = positiveInteger(value);
            if (!parsed.has_value())
            {
                return error("Invalid positive count for " + std::string(argument));
            }
            if (argument == "--trials") options.trialCount = *parsed;
            else if (argument == "--min-accepted-trials")
            {
                options.scaleConfiguration.minimumAcceptedTrials = *parsed;
            }
            else if (*parsed > 1000U)
            {
                return error("--print-rate must not exceed 1000 Hz.");
            }
            else
            {
                options.printRateHz = static_cast<std::uint32_t>(*parsed);
            }
        }
        else if (argument == "--countdown-seconds")
        {
            const auto parsed = nonNegativeInteger(value);
            if (!parsed.has_value())
            {
                return error("Invalid countdown duration.");
            }
            options.countdownSeconds = *parsed;
        }
        else
        {
            const auto parsed = positiveDouble(value);
            if (!parsed.has_value())
            {
                return error("Invalid positive value for " + std::string(argument));
            }
            if (argument == "--expected-degrees")
            {
                options.scaleConfiguration.expectedAngleDegrees = *parsed;
                options.recordingAnalysisConfiguration.expectedAngleDegrees = *parsed;
                expectedAngleProvided = true;
            }
            else if (argument == "--record-seconds")
            {
                options.recordDuration = seconds(*parsed);
                recordDurationProvided = true;
            }
            else if (argument == "--gyro-calibrate-seconds")
            {
                options.biasConfiguration.calibrationDuration = seconds(*parsed);
            }
            else if (argument == "--gyro-warmup-seconds")
            {
                options.biasConfiguration.warmupDuration = seconds(*parsed);
            }
            else if (argument == "--start-threshold")
            {
                options.scaleConfiguration.startThresholdRaw = *parsed;
            }
            else if (argument == "--stop-threshold")
            {
                options.scaleConfiguration.stopThresholdRaw = *parsed;
            }
            else if (argument == "--stillness-seconds")
            {
                options.scaleConfiguration.stillnessDuration = seconds(*parsed);
            }
            else if (argument == "--min-rotation-seconds")
            {
                options.scaleConfiguration.minimumRotationDuration = seconds(*parsed);
            }
            else if (argument == "--max-rotation-seconds")
            {
                options.scaleConfiguration.maximumRotationDuration = seconds(*parsed);
            }
            else if (argument == "--max-cross-axis-ratio")
            {
                options.scaleConfiguration.maximumCrossAxisRatio = *parsed;
            }
            else if (argument == "--max-trial-variation-percent")
            {
                options.scaleConfiguration.maximumTrialVariationPercent = *parsed;
            }
            else if (argument == "--analysis-start-threshold")
            {
                options.recordingAnalysisConfiguration.startThresholdRaw = *parsed;
            }
            else if (argument == "--analysis-stop-threshold")
            {
                options.recordingAnalysisConfiguration.stopThresholdRaw = *parsed;
            }
            else if (argument == "--analysis-stillness-seconds")
            {
                options.recordingAnalysisConfiguration.stillnessDuration = seconds(*parsed);
            }
            else if (argument == "--max-device-delta-ms")
            {
                options.recordingAnalysisConfiguration.maximumDeviceDelta =
                    std::chrono::duration_cast<std::chrono::nanoseconds>(
                        std::chrono::duration<double, std::milli>(*parsed));
            }
            else
            {
                return error("Unknown option: " + std::string(argument));
            }
        }
    }

    if (!axisProvided) return error("--axis is required.");
    if (options.recordOnly)
    {
        if (!recordDurationProvided) return error("--record-seconds is required in record-only mode.");
        if (options.csvOutputPath.empty()) return error("--csv-output is required in record-only mode.");
        if (options.analysisOutputPath.empty())
        {
            return error("--analysis-output is required in record-only mode.");
        }
        if (options.recordingAnalysisConfiguration.stopThresholdRaw
            >= options.recordingAnalysisConfiguration.startThresholdRaw)
        {
            return error("--analysis-stop-threshold must be lower than --analysis-start-threshold.");
        }
        return {options, {}, false};
    }

    if (options.recordingAnalysisConfiguration.axisSelection
        == sensors::GyroscopeAxisSelection::automatic)
    {
        return error("--axis auto is only valid with --record-only.");
    }
    if (!expectedAngleProvided) return error("--expected-degrees is required.");
    if (options.outputPath.empty()) return error("--output is required.");
    if (options.scaleConfiguration.stopThresholdRaw >= options.scaleConfiguration.startThresholdRaw)
    {
        return error("--stop-threshold must be lower than --start-threshold.");
    }
    if (options.scaleConfiguration.minimumRotationDuration
        >= options.scaleConfiguration.maximumRotationDuration)
    {
        return error("Minimum rotation duration must be lower than maximum duration.");
    }
    if (options.scaleConfiguration.maximumCrossAxisRatio > 1.0)
    {
        return error("--max-cross-axis-ratio must not exceed 1.");
    }
    if (options.scaleConfiguration.minimumAcceptedTrials > options.trialCount)
    {
        return error("Minimum accepted trials cannot exceed requested trials.");
    }
    return {options, {}, false};
}

std::string gyroScaleCalibrationUsage()
{
    return "Interactive:\n"
           "  xreal-gyro-scale-calibration --axis <x|y|z> --expected-degrees <value>"
           " --output <file.json> [interactive options]\n"
           "Record-only:\n"
           "  xreal-gyro-scale-calibration --record-only --record-seconds <seconds>"
           " --axis <x|y|z|auto> --csv-output <file.csv>"
           " --analysis-output <file.json> [--expected-degrees <value>]"
           " [--direction <positive|negative|auto>] [--gyro-calibrate-seconds <seconds>]"
           " [--gyro-warmup-seconds <seconds>] [--countdown-seconds <seconds>]"
           " [--print-rate <hz>] [--analysis-start-threshold <raw>]"
           " [--analysis-stop-threshold <raw>] [--analysis-stillness-seconds <seconds>]"
           " [--max-device-delta-ms <ms>] [--include-accelerometer]"
           " [--include-host-timestamps]\n"
           "Interactive options: [--trials <count>] [--start-threshold <raw>]"
           " [--stop-threshold <raw>] [--stillness-seconds <seconds>]"
           " [--min-rotation-seconds <seconds>] [--max-rotation-seconds <seconds>]"
           " [--max-cross-axis-ratio <ratio>] [--max-trial-variation-percent <percent>]"
           " [--min-accepted-trials <count>] [--csv-prefix <prefix>]\n";
}

} // namespace xreal::diagnostics
