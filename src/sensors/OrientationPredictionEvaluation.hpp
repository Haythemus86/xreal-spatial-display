#pragma once

#include "sensors/OrientationPrediction.hpp"

#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>
#include <string>

namespace xreal::sensors
{

struct OrientationPredictionEvaluation
{
    std::uint64_t sourceDeviceTimestampNanoseconds{};
    std::uint64_t targetDeviceTimestampNanoseconds{};
    std::uint64_t matchedDeviceTimestampNanoseconds{};
    std::int64_t targetTimestampErrorNanoseconds{};
    double predictionTotalErrorDegrees{};
    double predictionTiltErrorDegrees{};
    double baselineTotalErrorDegrees{};
    double baselineTiltErrorDegrees{};
    double totalImprovementDegrees{};
    double tiltImprovementDegrees{};
    std::optional<double> totalImprovementPercent;
};

struct OrientationPredictionEvaluationStatistics
{
    std::uint64_t queued{};
    std::uint64_t matched{};
    std::uint64_t unmatched{};
    std::uint64_t expired{};
    std::uint64_t incompatibleRecenter{};
    double meanPredictionTotalErrorDegrees{};
    double rmsPredictionTotalErrorDegrees{};
    double maximumPredictionTotalErrorDegrees{};
    double meanPredictionTiltErrorDegrees{};
    double rmsPredictionTiltErrorDegrees{};
    double maximumPredictionTiltErrorDegrees{};
    double meanBaselineTotalErrorDegrees{};
    double rmsBaselineTotalErrorDegrees{};
    double maximumBaselineTotalErrorDegrees{};
    double meanBaselineTiltErrorDegrees{};
    double rmsBaselineTiltErrorDegrees{};
    double meanTotalImprovementDegrees{};
    double meanTiltImprovementDegrees{};
    std::uint64_t improved{};
    double improvedRatio{};
};

struct DelayedOrientationEvaluatorConfig
{
    std::chrono::nanoseconds tolerance{std::chrono::milliseconds(2)};
    std::size_t maximumPendingPredictions{4096};
};

class DelayedOrientationEvaluator
{
public:
    explicit DelayedOrientationEvaluator(DelayedOrientationEvaluatorConfig configuration = {});
    void enqueue(
        std::uint64_t sourceDeviceTimestampNanoseconds,
        std::chrono::nanoseconds horizon,
        const AbsoluteOrientation& measured,
        const PredictedAbsoluteOrientation& predicted,
        std::uint64_t recenterGeneration);
    [[nodiscard]] std::optional<OrientationPredictionEvaluation> consumeMeasured(
        std::uint64_t deviceTimestampNanoseconds,
        const AbsoluteOrientation& measured,
        std::uint64_t recenterGeneration) noexcept;
    void finish() noexcept;
    [[nodiscard]] const OrientationPredictionEvaluationStatistics& statistics() const noexcept;
    [[nodiscard]] std::size_t pendingCount() const noexcept;

private:
    struct Pending
    {
        std::uint64_t source{};
        std::uint64_t target{};
        AbsoluteOrientation measured;
        PredictedAbsoluteOrientation predicted;
        std::uint64_t recenterGeneration{};
    };
    DelayedOrientationEvaluatorConfig configuration_;
    std::deque<Pending> pending_;
    OrientationPredictionEvaluationStatistics statistics_;
    double predictionTotalSum_{};
    double predictionTotalSquaredSum_{};
    double predictionTiltSum_{};
    double predictionTiltSquaredSum_{};
    double baselineTotalSum_{};
    double baselineTotalSquaredSum_{};
    double baselineTiltSum_{};
    double baselineTiltSquaredSum_{};
};

struct OrientationPredictionRecord
{
    std::uint64_t deviceTimestampNanoseconds{};
    std::uint64_t hostTimestampNanoseconds{};
    double elapsedSeconds{};
    std::uint8_t sequence{};
    std::uint64_t recenterGeneration{};
    std::string phase{"live"};
    PredictionMode mode{PredictionMode::constantVelocity};
    OrientationPredictionResult prediction;
    AbsoluteOrientation measuredAbsolute;
    RelativeOrientation measuredRelative;
    std::optional<OrientationPredictionEvaluation> delayedEvaluation;
};

struct OrientationPredictionMetadata
{
    std::string gyroscopeScaleSource{"unavailable"};
    std::string accelerometerProfileSource{"unavailable"};
    std::string gyroscopeAxisMappingSource{"unavailable"};
    std::string accelerometerAxisMappingSource{"unavailable"};
    bool recenterActive{};
    std::uint64_t recenterGeneration{};
};

[[nodiscard]] std::string orientationPredictionCsvHeader();
[[nodiscard]] std::string serializeOrientationPredictionCsvRow(
    const OrientationPredictionRecord& record);
[[nodiscard]] std::string serializeOrientationPredictionJson(
    const OrientationPredictorConfig& configuration,
    const OrientationPredictorStatistics& predictor,
    const OrientationPredictionEvaluationStatistics& evaluation,
    const std::optional<OrientationPredictionRecord>& finalRecord,
    const OrientationPredictionMetadata& metadata = {});

} // namespace xreal::sensors
