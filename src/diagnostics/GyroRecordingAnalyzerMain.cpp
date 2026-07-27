#include "diagnostics/GyroRecordingAnalyzerOptions.hpp"
#include "sensors/GyroscopeOfflineAnalysis.hpp"
#include "sensors/GyroscopeRecordingCsv.hpp"

#include <chrono>
#include <cstddef>
#include <exception>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string_view>
#include <vector>

namespace
{

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
