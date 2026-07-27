#include "diagnostics/GyroRecordingAnalyzerOptions.hpp"

#include <charconv>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <utility>

namespace xreal::diagnostics
{
namespace
{

[[nodiscard]] std::optional<double> number(std::string_view text)
{
    double value{};
    const auto parsed = std::from_chars(text.data(), text.data() + text.size(), value);
    if (parsed.ec != std::errc{} || parsed.ptr != text.data() + text.size()
        || !std::isfinite(value))
    {
        return std::nullopt;
    }
    return value;
}

[[nodiscard]] std::optional<std::uint64_t> positiveInteger(std::string_view text)
{
    std::uint64_t value{};
    const auto parsed = std::from_chars(text.data(), text.data() + text.size(), value);
    if (parsed.ec != std::errc{} || parsed.ptr != text.data() + text.size() || value == 0U)
    {
        return std::nullopt;
    }
    return value;
}

[[nodiscard]] std::chrono::nanoseconds seconds(double value)
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(value));
}

[[nodiscard]] GyroRecordingAnalyzerOptionResult error(std::string message)
{
    return {std::nullopt, std::move(message), false};
}

} // namespace

GyroRecordingAnalyzerOptionResult parseGyroRecordingAnalyzerOptions(
    std::span<const std::string_view> arguments)
{
    GyroRecordingAnalyzerOptions options;
    bool axisProvided{};
    for (std::size_t index = 0; index < arguments.size(); ++index)
    {
        const std::string_view argument = arguments[index];
        if (argument == "--help" || argument == "-h")
        {
            return {options, {}, true};
        }
        if (argument == "--no-trim")
        {
            options.analysis.trimCandidates = false;
            continue;
        }
        if (++index >= arguments.size())
        {
            return error("Missing value for " + std::string(argument));
        }
        const std::string_view value = arguments[index];
        if (argument == "--input")
        {
            if (value.empty())
            {
                return error("--input requires a non-empty path.");
            }
            options.inputPaths.emplace_back(value);
        }
        else if (argument == "--output")
        {
            if (value.empty())
            {
                return error("--output requires a non-empty path.");
            }
            options.outputPath = value;
        }
        else if (argument == "--axis")
        {
            if (value == "x")
            {
                options.analysis.base.axisSelection = sensors::GyroscopeAxisSelection::x;
            }
            else if (value == "y")
            {
                options.analysis.base.axisSelection = sensors::GyroscopeAxisSelection::y;
            }
            else if (value == "z")
            {
                options.analysis.base.axisSelection = sensors::GyroscopeAxisSelection::z;
            }
            else if (value == "auto")
            {
                options.analysis.base.axisSelection = sensors::GyroscopeAxisSelection::automatic;
            }
            else
            {
                return error("--axis must be x, y, z or auto.");
            }
            axisProvided = true;
        }
        else if (argument == "--direction")
        {
            if (value == "positive")
            {
                options.analysis.base.requestedDirection = sensors::RotationDirection::positive;
            }
            else if (value == "negative")
            {
                options.analysis.base.requestedDirection = sensors::RotationDirection::negative;
            }
            else if (value == "auto")
            {
                options.analysis.base.requestedDirection = sensors::RotationDirection::automatic;
            }
            else
            {
                return error("--direction must be positive, negative or auto.");
            }
        }
        else if (argument == "--deadband-mode")
        {
            if (value == "none")
            {
                options.analysis.deadbandMode = sensors::GyroscopeDeadbandMode::none;
            }
            else if (value == "hard")
            {
                options.analysis.deadbandMode = sensors::GyroscopeDeadbandMode::hard;
            }
            else if (value == "soft")
            {
                options.analysis.deadbandMode = sensors::GyroscopeDeadbandMode::soft;
            }
            else
            {
                return error("--deadband-mode must be none, hard or soft.");
            }
        }
        else if (argument == "--residual-bias-mode")
        {
            if (value == "none")
            {
                options.analysis.residualBiasMode = sensors::GyroscopeResidualBiasMode::none;
            }
            else if (value == "pre")
            {
                options.analysis.residualBiasMode = sensors::GyroscopeResidualBiasMode::preStillness;
            }
            else if (value == "linear")
            {
                options.analysis.residualBiasMode = sensors::GyroscopeResidualBiasMode::linear;
            }
            else
            {
                return error("--residual-bias-mode must be none, pre or linear.");
            }
        }
        else if (argument == "--batch-min-recordings")
        {
            const auto parsed = positiveInteger(value);
            if (!parsed.has_value())
            {
                return error("Invalid positive batch recording count.");
            }
            options.batch.minimumAcceptedRecordings = *parsed;
        }
        else
        {
            const auto parsed = number(value);
            if (!parsed.has_value())
            {
                return error("Invalid numeric value for " + std::string(argument));
            }
            const double numeric = *parsed;
            const bool biasOption = argument == "--bias-x" || argument == "--bias-y"
                || argument == "--bias-z";
            if (!biasOption && numeric <= 0.0)
            {
                return error(std::string(argument) + " must be strictly positive.");
            }
            if (argument == "--bias-x")
            {
                options.biasX = numeric;
            }
            else if (argument == "--bias-y")
            {
                options.biasY = numeric;
            }
            else if (argument == "--bias-z")
            {
                options.biasZ = numeric;
            }
            else if (argument == "--expected-degrees")
            {
                options.analysis.base.expectedAngleDegrees = numeric;
            }
            else if (argument == "--start-threshold")
            {
                options.analysis.base.startThresholdRaw = numeric;
            }
            else if (argument == "--stop-threshold")
            {
                options.analysis.base.stopThresholdRaw = numeric;
            }
            else if (argument == "--pre-stillness-seconds")
            {
                options.analysis.preStillnessDuration = seconds(numeric);
            }
            else if (argument == "--pre-stillness-max-rms")
            {
                options.analysis.preStillnessMaximumRmsRaw = numeric;
            }
            else if (argument == "--pre-stillness-max-peak")
            {
                options.analysis.preStillnessMaximumPeakRaw = numeric;
            }
            else if (argument == "--post-stillness-seconds")
            {
                options.analysis.postStillnessDuration = seconds(numeric);
            }
            else if (argument == "--post-stillness-max-rms")
            {
                options.analysis.postStillnessMaximumRmsRaw = numeric;
            }
            else if (argument == "--post-stillness-max-peak")
            {
                options.analysis.postStillnessMaximumPeakRaw = numeric;
            }
            else if (argument == "--noise-multiplier")
            {
                options.analysis.noiseStandardDeviationMultiplier = numeric;
            }
            else if (argument == "--minimum-deadband")
            {
                options.analysis.minimumDeadbandRaw = numeric;
            }
            else if (argument == "--envelope-window-ms")
            {
                options.analysis.envelopeWindow = std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::duration<double, std::milli>(numeric));
            }
            else if (argument == "--start-sustain-seconds")
            {
                options.analysis.startSustainDuration = seconds(numeric);
            }
            else if (argument == "--stop-sustain-seconds")
            {
                options.analysis.stopSustainDuration = seconds(numeric);
            }
            else if (argument == "--trim-envelope-fraction")
            {
                options.analysis.trimEnvelopeFraction = numeric;
            }
            else if (argument == "--residual-disagreement-warning")
            {
                options.analysis.residualMeanDisagreementWarningRaw = numeric;
            }
            else if (argument == "--max-device-delta-ms")
            {
                options.analysis.base.maximumDeviceDelta =
                    std::chrono::duration_cast<std::chrono::nanoseconds>(
                        std::chrono::duration<double, std::milli>(numeric));
            }
            else if (argument == "--batch-outlier-percent")
            {
                options.batch.outlierPercent = numeric;
            }
            else if (argument == "--batch-max-cv-percent")
            {
                options.batch.maximumCoefficientOfVariationPercent = numeric;
            }
            else if (argument == "--batch-max-direction-disagreement-percent")
            {
                options.batch.maximumDirectionDisagreementPercent = numeric;
            }
            else if (argument == "--batch-min-confidence")
            {
                options.batch.minimumConfidence = numeric;
            }
            else
            {
                return error("Unknown option: " + std::string(argument));
            }
        }
    }
    if (options.inputPaths.empty())
    {
        return error("At least one --input file is required.");
    }
    if (options.outputPath.empty())
    {
        return error("--output is required.");
    }
    if (!axisProvided)
    {
        return error("--axis is required.");
    }
    if (options.analysis.base.stopThresholdRaw >= options.analysis.base.startThresholdRaw)
    {
        return error("--stop-threshold must be lower than --start-threshold.");
    }
    const bool anyBias = options.biasX.has_value() || options.biasY.has_value() || options.biasZ.has_value();
    const bool allBias = options.biasX.has_value() && options.biasY.has_value() && options.biasZ.has_value();
    if (anyBias && !allBias)
    {
        return error("Explicit bias requires --bias-x, --bias-y and --bias-z together.");
    }
    if (options.analysis.trimEnvelopeFraction > 1.0)
    {
        return error("--trim-envelope-fraction must not exceed 1.");
    }
    if (options.batch.minimumConfidence > 1.0)
    {
        return error("--batch-min-confidence must not exceed 1.");
    }
    return {options, {}, false};
}

std::string gyroRecordingAnalyzerUsage()
{
    return "Usage: xreal-gyro-recording-analyzer --input <file.csv> [--input <file.csv> ...]"
           " --axis <x|y|z|auto> --output <file.json> [--expected-degrees <value>]"
           " [--direction <positive|negative|auto>] [--deadband-mode <none|hard|soft>]"
           " [--residual-bias-mode <none|pre|linear>] [--pre-stillness-seconds <seconds>]"
           " [--pre-stillness-max-rms <raw>] [--pre-stillness-max-peak <raw>]"
           " [--post-stillness-seconds <seconds>] [--post-stillness-max-rms <raw>]"
           " [--post-stillness-max-peak <raw>] [--noise-multiplier <value>]"
           " [--minimum-deadband <raw>] [--envelope-window-ms <ms>]"
           " [--start-sustain-seconds <seconds>] [--stop-sustain-seconds <seconds>]"
           " [--trim-envelope-fraction <0..1>] [--no-trim]"
           " [--bias-x <raw> --bias-y <raw> --bias-z <raw>]"
           " [--batch-outlier-percent <percent>] [--batch-min-recordings <count>]"
           " [--batch-max-cv-percent <percent>] [--batch-min-confidence <0..1>]\n";
}

} // namespace xreal::diagnostics
