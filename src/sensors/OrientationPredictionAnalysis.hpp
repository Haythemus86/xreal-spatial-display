#pragma once

#include "sensors/OrientationPrediction.hpp"

#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace xreal::sensors
{

struct OrientationPredictionAnalysisSample
{
    std::uint64_t deviceTimestampNanoseconds{};
    AbsoluteOrientation measuredAbsolute;
    AngularVelocityRadians bodyAngularVelocity;
    std::uint64_t recenterGeneration{};
    std::string phase;
};

struct OrientationPredictionCsvLoadResult
{
    std::vector<OrientationPredictionAnalysisSample> samples;
    std::string error;
};

struct PredictionErrorStatistics
{
    double mean{};
    double rms{};
    double maximum{};
    double median{};
    double percentile95{};
};

struct OrientationPredictionHorizonAnalysis
{
    double horizonMilliseconds{};
    PredictionMode mode{PredictionMode::constantVelocity};
    std::uint64_t inputSampleCount{};
    std::uint64_t eligiblePredictionCount{};
    std::uint64_t matchedPredictionCount{};
    std::uint64_t unmatchedPredictionCount{};
    std::uint64_t rejectedPredictionCount{};
    std::uint64_t clampedPredictionCount{};
    PredictionErrorStatistics unpredictedTotal;
    PredictionErrorStatistics predictedTotal;
    PredictionErrorStatistics unpredictedTilt;
    PredictionErrorStatistics predictedTilt;
    double meanImprovementDegrees{};
    double meanImprovementPercent{};
    double improvedRatio{};
};

struct OrientationPredictionMultiHorizonAnalysis
{
    std::vector<OrientationPredictionHorizonAnalysis> horizons;
    std::optional<double> bestTotalErrorHorizonMilliseconds;
    std::optional<double> bestTiltErrorHorizonMilliseconds;
    std::optional<double> bestImprovementHorizonMilliseconds;
};

[[nodiscard]] OrientationPredictionCsvLoadResult loadOrientationPredictionCsv(
    std::string_view csv);
[[nodiscard]] OrientationPredictionMultiHorizonAnalysis analyzeOrientationPredictionHorizons(
    std::span<const OrientationPredictionAnalysisSample> samples,
    std::span<const double> horizonsMilliseconds,
    const OrientationPredictorConfig& baseConfiguration,
    std::chrono::nanoseconds evaluationTolerance);
[[nodiscard]] std::string orientationPredictionAnalysisCsvHeader();
[[nodiscard]] std::string serializeOrientationPredictionAnalysisCsv(
    const OrientationPredictionMultiHorizonAnalysis& analysis);
[[nodiscard]] std::string serializeOrientationPredictionAnalysisJson(
    const OrientationPredictionMultiHorizonAnalysis& analysis,
    const OrientationPredictorConfig& configuration,
    std::string_view inputPath,
    std::uint64_t inputSampleCount);

} // namespace xreal::sensors
