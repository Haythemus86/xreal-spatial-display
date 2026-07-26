#include "diagnostics/GyroScaleCalibrationOptions.hpp"

#include <charconv>
#include <chrono>
#include <cmath>

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

} // namespace

GyroScaleCalibrationOptionResult parseGyroScaleCalibrationOptions(
    std::span<const std::string_view> arguments)
{
    GyroScaleCalibrationOptions options;
    options.biasConfiguration.warmupDuration = std::chrono::seconds(1);
    bool axisProvided{};
    bool expectedAngleProvided{};

    for (std::size_t index = 0; index < arguments.size(); ++index)
    {
        const std::string_view argument = arguments[index];
        if (argument == "--help" || argument == "-h")
        {
            return {options, {}, true};
        }
        if (++index >= arguments.size())
        {
            return {std::nullopt, "Missing value for " + std::string(argument), false};
        }
        const std::string_view value = arguments[index];

        if (argument == "--axis")
        {
            if (value == "x") options.scaleConfiguration.axis = sensors::GyroscopeAxis::x;
            else if (value == "y") options.scaleConfiguration.axis = sensors::GyroscopeAxis::y;
            else if (value == "z") options.scaleConfiguration.axis = sensors::GyroscopeAxis::z;
            else return {std::nullopt, "--axis must be x, y or z.", false};
            axisProvided = true;
        }
        else if (argument == "--direction")
        {
            if (value == "positive") options.scaleConfiguration.direction = sensors::RotationDirection::positive;
            else if (value == "negative") options.scaleConfiguration.direction = sensors::RotationDirection::negative;
            else if (value == "auto") options.scaleConfiguration.direction = sensors::RotationDirection::automatic;
            else return {std::nullopt, "--direction must be positive, negative or auto.", false};
        }
        else if (argument == "--output" || argument == "--csv-prefix")
        {
            if (value.empty())
            {
                return {std::nullopt, std::string(argument) + " requires a non-empty value.", false};
            }
            if (argument == "--output") options.outputPath = value;
            else options.csvPrefix = value;
        }
        else if (argument == "--trials" || argument == "--min-accepted-trials")
        {
            const auto parsed = positiveInteger(value);
            if (!parsed.has_value())
            {
                return {std::nullopt, "Invalid positive count for " + std::string(argument), false};
            }
            if (argument == "--trials") options.trialCount = *parsed;
            else options.scaleConfiguration.minimumAcceptedTrials = *parsed;
        }
        else if (argument == "--countdown-seconds")
        {
            const auto parsed = nonNegativeInteger(value);
            if (!parsed.has_value())
            {
                return {std::nullopt, "Invalid countdown duration.", false};
            }
            options.countdownSeconds = *parsed;
        }
        else
        {
            const auto parsed = positiveDouble(value);
            if (!parsed.has_value())
            {
                return {std::nullopt, "Invalid positive value for " + std::string(argument), false};
            }
            if (argument == "--expected-degrees")
            {
                options.scaleConfiguration.expectedAngleDegrees = *parsed;
                expectedAngleProvided = true;
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
            else
            {
                return {std::nullopt, "Unknown option: " + std::string(argument), false};
            }
        }
    }

    if (!axisProvided) return {std::nullopt, "--axis is required.", false};
    if (!expectedAngleProvided) return {std::nullopt, "--expected-degrees is required.", false};
    if (options.outputPath.empty()) return {std::nullopt, "--output is required.", false};
    if (options.scaleConfiguration.stopThresholdRaw >= options.scaleConfiguration.startThresholdRaw)
    {
        return {std::nullopt, "--stop-threshold must be lower than --start-threshold.", false};
    }
    if (options.scaleConfiguration.minimumRotationDuration
        >= options.scaleConfiguration.maximumRotationDuration)
    {
        return {std::nullopt, "Minimum rotation duration must be lower than maximum duration.", false};
    }
    if (options.scaleConfiguration.maximumCrossAxisRatio > 1.0)
    {
        return {std::nullopt, "--max-cross-axis-ratio must not exceed 1.", false};
    }
    if (options.scaleConfiguration.minimumAcceptedTrials > options.trialCount)
    {
        return {std::nullopt, "Minimum accepted trials cannot exceed requested trials.", false};
    }
    return {options, {}, false};
}

std::string gyroScaleCalibrationUsage()
{
    return "Usage: xreal-gyro-scale-calibration --axis <x|y|z> --expected-degrees <value>"
           " --output <file.json> [--direction <positive|negative|auto>] [--trials <count>]"
           " [--gyro-calibrate-seconds <seconds>] [--gyro-warmup-seconds <seconds>]"
           " [--start-threshold <raw>] [--stop-threshold <raw>]"
           " [--stillness-seconds <seconds>] [--min-rotation-seconds <seconds>]"
           " [--max-rotation-seconds <seconds>] [--countdown-seconds <seconds>]"
           " [--max-cross-axis-ratio <ratio>] [--max-trial-variation-percent <percent>]"
           " [--min-accepted-trials <count>] [--csv-prefix <prefix>]\n";
}

} // namespace xreal::diagnostics
