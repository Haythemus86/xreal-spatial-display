#include "diagnostics/GyroRecordingAnalyzerOptions.hpp"
#include "sensors/GyroscopeOfflineAnalysis.hpp"
#include "sensors/GyroscopePhysicalUnits.hpp"
#include "sensors/GyroscopeRecordingCsv.hpp"

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <exception>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <sstream>
#include <cmath>
#include <string_view>
#include <vector>

namespace
{

[[nodiscard]] double axisValue(
    const xreal::sensors::RawVector3d& value,
    xreal::sensors::GyroscopeAxis axis) noexcept
{
    if (axis == xreal::sensors::GyroscopeAxis::x)
    {
        return value.x;
    }
    if (axis == xreal::sensors::GyroscopeAxis::y)
    {
        return value.y;
    }
    return value.z;
}

[[nodiscard]] std::optional<xreal::sensors::GyroscopeAngleComparison> angleComparison(
    const xreal::sensors::GyroscopeOfflineAnalysisResult& analysis,
    double fixedScale,
    double comparisonScale)
{
    if (!analysis.bestSegmentIndex.has_value()
        || !analysis.configuration.base.expectedAngleDegrees.has_value())
    {
        return std::nullopt;
    }
    const auto& segment = analysis.segments[*analysis.bestSegmentIndex];
    double expected = *analysis.configuration.base.expectedAngleDegrees;
    if (segment.direction == xreal::sensors::RotationDirection::negative)
    {
        expected = -expected;
    }
    return xreal::sensors::compareGyroscopeIntegratedAngle(
        axisValue(segment.integratedRaw, segment.dominantAxis),
        expected,
        fixedScale,
        comparisonScale);
}

void appendPhysicalComparison(
    std::string& json,
    const std::vector<std::string>& names,
    const std::vector<xreal::sensors::GyroscopeOfflineAnalysisResult>& analyses,
    double fixedScale,
    double comparisonScale)
{
    const std::size_t closing = json.rfind("\n}\n");
    if (closing == std::string::npos)
    {
        return;
    }
    json.erase(closing);
    std::ostringstream output;
    output << std::setprecision(17)
           << ",\n  \"fixed_scale_comparison\":{\"experimental\":true,"
           << "\"verified\":false,\"fixed_raw_per_dps\":" << fixedScale
           << ",\"comparison_raw_per_dps\":" << comparisonScale
           << ",\"recordings\":[";
    std::vector<double> fixedErrors;
    std::vector<double> comparisonErrors;
    double positiveErrorSum{};
    double negativeErrorSum{};
    std::size_t positiveCount{};
    std::size_t negativeCount{};
    for (std::size_t index = 0; index < analyses.size(); ++index)
    {
        const auto comparison = angleComparison(analyses[index], fixedScale, comparisonScale);
        if (index != 0U)
        {
            output << ',';
        }
        output << "{\"input\":\"" << names[index] << "\",\"available\":"
               << (comparison.has_value() ? "true" : "false");
        if (comparison.has_value())
        {
            output << ",\"integrated_raw\":" << comparison->integratedRaw
                   << ",\"fixed_degrees\":" << comparison->selectedDegrees
                   << ",\"fixed_radians\":" << comparison->selectedRadians
                   << ",\"expected_degrees\":" << comparison->expectedDegrees
                   << ",\"fixed_signed_error_degrees\":" << comparison->selectedSignedErrorDegrees
                   << ",\"fixed_absolute_error_degrees\":" << comparison->selectedAbsoluteErrorDegrees
                   << ",\"fixed_percentage_error\":" << comparison->selectedPercentageError
                   << ",\"comparison_degrees\":" << comparison->comparisonDegrees
                   << ",\"comparison_absolute_error_degrees\":"
                   << comparison->comparisonAbsoluteErrorDegrees
                   << ",\"relative_result_difference_percent\":"
                   << comparison->relativeResultDifferencePercent
                   << ",\"fixed_fits_better\":"
                   << (comparison->selectedFitsBetter ? "true" : "false");
            fixedErrors.push_back(comparison->selectedAbsoluteErrorDegrees);
            comparisonErrors.push_back(comparison->comparisonAbsoluteErrorDegrees);
            if (comparison->expectedDegrees >= 0.0)
            {
                positiveErrorSum += comparison->selectedSignedErrorDegrees;
                ++positiveCount;
            }
            else
            {
                negativeErrorSum += comparison->selectedSignedErrorDegrees;
                ++negativeCount;
            }
        }
        output << '}';
    }
    const auto mean = [](const std::vector<double>& values) {
        return values.empty() ? 0.0
            : std::accumulate(values.begin(), values.end(), 0.0)
                / static_cast<double>(values.size());
    };
    const auto median = [](std::vector<double> values) {
        if (values.empty())
        {
            return 0.0;
        }
        std::sort(values.begin(), values.end());
        const std::size_t middle = values.size() / 2U;
        return values.size() % 2U == 0U
            ? (values[middle - 1U] + values[middle]) * 0.5 : values[middle];
    };
    double squared{};
    for (const double error : fixedErrors)
    {
        squared += error * error;
    }
    output << "],\"batch\":{\"mean_absolute_error_degrees\":" << mean(fixedErrors)
           << ",\"median_absolute_error_degrees\":" << median(fixedErrors)
           << ",\"rms_angle_error_degrees\":"
           << (fixedErrors.empty() ? 0.0 : std::sqrt(squared / fixedErrors.size()))
           << ",\"positive_mean_signed_error_degrees\":"
           << (positiveCount == 0U ? 0.0 : positiveErrorSum / positiveCount)
           << ",\"negative_mean_signed_error_degrees\":"
           << (negativeCount == 0U ? 0.0 : negativeErrorSum / negativeCount)
           << ",\"better_fit\":\"" << (mean(fixedErrors) <= mean(comparisonErrors)
                ? "fixed" : "comparison")
           << "\",\"officially_verified\":false}}\n}\n";
    json += output.str();
}

[[nodiscard]] xreal::sensors::GyroscopeRecordingMetadata metadata(
    const xreal::sensors::GyroscopeRecordingCsvResult& recording,
    const xreal::sensors::GyroscopeBias& bias)
{
    xreal::sensors::GyroscopeRecordingMetadata result;
    result.device = {0x3318, 0x0426, 2, "XREAL Air 2 Ultra (CSV recording)"};
    result.biasCalibration.accepted = true;
    result.biasCalibration.biasRaw = bias;
    if (recording.samples.size() > 1U)
    {
        result.measuredDurationSeconds = static_cast<double>(
            recording.samples.back().deviceTimestamp.nanoseconds
                - recording.samples.front().deviceTimestamp.nanoseconds) / 1'000'000'000.0;
        result.requestedDurationSeconds = result.measuredDurationSeconds;
    }
    return result;
}

void printSingleSummary(
    std::string_view input,
    const xreal::sensors::GyroscopeOfflineAnalysisResult& analysis)
{
    std::cout << "\nOffline gyroscope analysis: " << input << '\n'
              << "  Samples: " << analysis.backwardCompatible.samples.size() << '\n'
              << "  Pre-stillness: " << (analysis.preStillness.accepted ? "accepted" : "missing") << '\n'
              << "  Post-stillness: " << (analysis.postStillness.accepted ? "accepted" : "missing") << '\n'
              << "  Refined segments: " << analysis.segments.size() << '\n';
    if (analysis.bestSegmentIndex.has_value())
    {
        const auto& segment = analysis.segments[*analysis.bestSegmentIndex];
        std::cout << "  Selected segment: " << *analysis.bestSegmentIndex
                  << " (score=" << segment.score.total << ")\n"
                  << "  Refined duration: "
                  << std::chrono::duration<double>(segment.refinedDuration).count() << " seconds\n"
                  << "  Dominant axis: "
                  << xreal::sensors::gyroscopeAxisText(segment.dominantAxis) << '\n'
                  << "  Direction: "
                  << xreal::sensors::rotationDirectionText(segment.direction) << '\n'
                  << "  Cross-axis ratio: " << segment.crossAxisRatio << '\n';
    }
    if (analysis.scaleEstimates.recommended.available)
    {
        std::cout << "  Recommended experimental method: "
                  << analysis.scaleEstimates.recommendedMethod << '\n'
                  << "  Scale: "
                  << analysis.scaleEstimates.recommended.rawUnitsPerDegreePerSecond
                  << " raw units/(degree/s)\n"
                  << "  Confidence: " << analysis.scaleEstimates.confidence << '\n';
    }
    for (const auto& warning : analysis.warnings)
    {
        std::cout << "  Warning: " << warning << '\n';
    }
}

} // namespace

int main(int argc, char* argv[])
{
    std::vector<std::string_view> arguments;
    arguments.reserve(static_cast<std::size_t>(argc > 0 ? argc - 1 : 0));
    for (int index = 1; index < argc; ++index)
    {
        arguments.emplace_back(argv[index]);
    }
    const auto parsed = xreal::diagnostics::parseGyroRecordingAnalyzerOptions(arguments);
    if (parsed.showHelp)
    {
        std::cout << xreal::diagnostics::gyroRecordingAnalyzerUsage();
        return 0;
    }
    if (!parsed.options.has_value())
    {
        std::cerr << "Error: " << parsed.error << '\n'
                  << xreal::diagnostics::gyroRecordingAnalyzerUsage();
        return 2;
    }

    try
    {
        const auto& options = *parsed.options;
        if (options.comparisonScaleRawPerDegreePerSecond.has_value()
            && !options.fixedScaleRawPerDegreePerSecond.has_value())
        {
            std::cerr << "--compare-scale-raw-per-dps requires --fixed-scale-raw-per-dps.\n";
            return 2;
        }
        std::vector<xreal::sensors::GyroscopeOfflineAnalysisResult> analyses;
        std::vector<xreal::sensors::GyroscopeRecordingMetadata> metadataItems;
        analyses.reserve(options.inputPaths.size());
        metadataItems.reserve(options.inputPaths.size());
        for (const auto& path : options.inputPaths)
        {
            std::ifstream input(path);
            if (!input)
            {
                std::cerr << "Failed to open input CSV: " << path << '\n';
                return 1;
            }
            const auto recording = xreal::sensors::parseGyroscopeRecordingCsv(input);
            if (!recording.valid() || !recording.bias.has_value())
            {
                std::cerr << "Invalid record-only CSV " << path << " at line "
                          << recording.lineNumber << ": " << recording.error << '\n';
                return 1;
            }
            const xreal::sensors::GyroscopeBias bias = options.biasX.has_value()
                ? xreal::sensors::GyroscopeBias{*options.biasX, *options.biasY, *options.biasZ}
                : *recording.bias;
            analyses.push_back(xreal::sensors::analyzeGyroscopeRecordingOffline(
                recording.samples, bias, options.analysis));
            metadataItems.push_back(metadata(recording, bias));
            printSingleSummary(path, analyses.back());
        }

        std::string json;
        bool accepted{};
        if (analyses.size() == 1U)
        {
            json = xreal::sensors::serializeGyroscopeOfflineAnalysisJson(
                metadataItems.front(), analyses.front());
            accepted = analyses.front().bestSegmentIndex.has_value();
        }
        else
        {
            const auto batch = xreal::sensors::aggregateGyroscopeRecordings(
                options.inputPaths, analyses, options.batch);
            json = xreal::sensors::serializeGyroscopeBatchAnalysisJson(batch, analyses);
            accepted = batch.accepted;
            std::cout << "\nBatch result: " << (batch.accepted ? "accepted" : "rejected")
                      << "\n  Reason: " << batch.rejectionReason
                      << "\n  Accepted recordings: " << batch.acceptedCount
                      << "\n  Rejected recordings: " << batch.rejectedCount
                      << "\n  Median scale: " << batch.median
                      << "\n  Mean scale: " << batch.mean
                      << "\n  Standard deviation: " << batch.standardDeviation
                      << "\n  Coefficient of variation: " << batch.coefficientOfVariation << '\n';
        }
        if (options.fixedScaleRawPerDegreePerSecond.has_value())
        {
            const double comparisonScale = options.comparisonScaleRawPerDegreePerSecond
                .value_or(*options.fixedScaleRawPerDegreePerSecond);
            appendPhysicalComparison(
                json,
                options.inputPaths,
                analyses,
                *options.fixedScaleRawPerDegreePerSecond,
                comparisonScale);
            std::cout << "  Fixed experimental scale comparison: "
                      << *options.fixedScaleRawPerDegreePerSecond << " versus "
                      << comparisonScale << " raw/(degree/s)\n";
        }
        std::ofstream output(options.outputPath, std::ios::out | std::ios::trunc);
        if (!output)
        {
            std::cerr << "Failed to open JSON output: " << options.outputPath << '\n';
            return 1;
        }
        output << json;
        if (!output)
        {
            std::cerr << "Failed while writing JSON output: " << options.outputPath << '\n';
            return 1;
        }
        std::cout << "  JSON output: " << options.outputPath << '\n';
        return accepted ? 0 : 1;
    }
    catch (const std::exception& exception)
    {
        std::cerr << "Offline gyroscope analyzer failed: " << exception.what() << '\n';
        return 1;
    }
}
