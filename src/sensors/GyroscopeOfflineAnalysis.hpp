#pragma once

#include "sensors/GyroscopeRecordingAnalysis.hpp"

#include <chrono>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace xreal::sensors
{

enum class GyroscopeDeadbandMode
{
    none,
    hard,
    soft,
};

enum class GyroscopeResidualBiasMode
{
    none,
    preStillness,
    linear,
};

struct GyroscopeOfflineAnalysisConfig
{
    GyroscopeRecordingAnalysisConfig base;
    std::chrono::nanoseconds preStillnessDuration{std::chrono::seconds(1)};
    double preStillnessMaximumRmsRaw{1000.0};
    double preStillnessMaximumPeakRaw{2500.0};
    std::chrono::nanoseconds postStillnessDuration{std::chrono::seconds(1)};
    double postStillnessMaximumRmsRaw{1000.0};
    double postStillnessMaximumPeakRaw{2500.0};
    double noiseStandardDeviationMultiplier{3.0};
    double minimumDeadbandRaw{500.0};
    GyroscopeDeadbandMode deadbandMode{GyroscopeDeadbandMode::soft};
    std::chrono::nanoseconds envelopeWindow{std::chrono::milliseconds(50)};
    std::chrono::nanoseconds startSustainDuration{std::chrono::milliseconds(100)};
    std::chrono::nanoseconds stopSustainDuration{std::chrono::milliseconds(300)};
    bool trimCandidates{true};
    double trimEnvelopeFraction{0.02};
    GyroscopeResidualBiasMode residualBiasMode{GyroscopeResidualBiasMode::linear};
    double residualMeanDisagreementWarningRaw{1500.0};
};

struct GyroscopeStillnessAxisStatistics
{
    double mean{};
    double standardDeviation{};
    double rootMeanSquare{};
    double peakAbsolute{};
    double medianAbsoluteDeviation{};
};

struct GyroscopeStillnessWindow
{
    bool found{};
    bool accepted{};
    std::size_t startSampleIndex{};
    std::size_t endSampleIndex{};
    std::uint64_t startDeviceTimestamp{};
    std::uint64_t endDeviceTimestamp{};
    std::chrono::nanoseconds duration{};
    GyroscopeStillnessAxisStatistics x;
    GyroscopeStillnessAxisStatistics y;
    GyroscopeStillnessAxisStatistics z;
};

struct GyroscopeNoiseFloor
{
    RawVector3d meanRaw;
    RawVector3d standardDeviationRaw;
    RawVector3d rootMeanSquareRaw;
    RawVector3d peakAbsoluteRaw;
    RawVector3d medianAbsoluteDeviationRaw;
    RawVector3d deadbandRaw;
    GyroscopeDeadbandMode mode{GyroscopeDeadbandMode::soft};
};

struct GyroscopeSegmentScoreBreakdown
{
    double preStillness{};
    double postStillness{};
    double dominantAxis{};
    double crossAxis{};
    double duration{};
    double sustainedMovement{};
    double timestampValidity{};
    double energyContainment{};
    double thresholdQuality{};
    double residualStability{};
    double total{};
};

struct GyroscopeRefinedSegment
{
    std::size_t roughStartSampleIndex{};
    std::size_t roughEndSampleIndex{};
    std::size_t refinedStartSampleIndex{};
    std::size_t refinedEndSampleIndex{};
    std::uint64_t roughStartDeviceTimestamp{};
    std::uint64_t roughEndDeviceTimestamp{};
    std::uint64_t refinedStartDeviceTimestamp{};
    std::uint64_t refinedEndDeviceTimestamp{};
    std::chrono::nanoseconds roughDuration{};
    std::chrono::nanoseconds refinedDuration{};
    std::uint64_t leadingSamplesTrimmed{};
    std::uint64_t trailingSamplesTrimmed{};
    std::chrono::nanoseconds leadingDurationRemoved{};
    std::chrono::nanoseconds trailingDurationRemoved{};
    GyroscopeAxis dominantAxis{GyroscopeAxis::z};
    RotationDirection direction{RotationDirection::automatic};
    RawVector3d integratedRaw;
    RawVector3d integratedDeadbanded;
    RawVector3d integratedResidualCorrected;
    RawVector3d peakAbsoluteRaw;
    double dominantAxisRatio{};
    double crossAxisRatio{};
    double rotationalEnergyFraction{};
    bool preStillnessVerified{};
    bool postStillnessVerified{};
    double startEnvelope{};
    double endEnvelope{};
    GyroscopeSegmentScoreBreakdown score;
};

struct GyroscopeScaleMethodEstimate
{
    bool available{};
    double rawUnitsPerDegreePerSecond{};
    double rawUnitsPerRadianPerSecond{};
    double degreesPerSecondPerRawUnit{};
    double radiansPerSecondPerRawUnit{};
};

struct GyroscopeRefinedScaleEstimates
{
    bool experimental{true};
    GyroscopeAxis axis{GyroscopeAxis::z};
    double expectedAngleDegrees{};
    GyroscopeScaleMethodEstimate raw;
    GyroscopeScaleMethodEstimate deadbanded;
    GyroscopeScaleMethodEstimate residualCorrected;
    std::string recommendedMethod;
    GyroscopeScaleMethodEstimate recommended;
    double maximumRelativeMethodDifference{};
    bool directionMatchesRequest{true};
    double confidence{};
};

struct GyroscopeOfflineAnalysisResult
{
    GyroscopeRecordingAnalysisResult backwardCompatible;
    GyroscopeOfflineAnalysisConfig configuration;
    GyroscopeStillnessWindow preStillness;
    GyroscopeStillnessWindow postStillness;
    GyroscopeNoiseFloor noiseFloor;
    RawVector3d postStillnessMeanRaw;
    std::vector<double> movementEnvelope;
    std::vector<GyroscopeRefinedSegment> segments;
    std::optional<std::size_t> bestSegmentIndex;
    GyroscopeRefinedScaleEstimates scaleEstimates;
    std::vector<std::string> warnings;
};

struct GyroscopeBatchConfig
{
    double outlierPercent{20.0};
    std::uint64_t minimumAcceptedRecordings{3};
    double maximumCoefficientOfVariationPercent{10.0};
    double maximumDirectionDisagreementPercent{10.0};
    double minimumConfidence{0.3};
};

struct GyroscopeBatchRecordingResult
{
    std::string inputName;
    bool accepted{};
    std::string rejectionReason;
    double recommendedScale{};
    RotationDirection direction{RotationDirection::automatic};
    GyroscopeAxis dominantAxis{GyroscopeAxis::z};
    double selectedDurationSeconds{};
    double crossAxisRatio{};
    double confidence{};
};

struct GyroscopeBatchAnalysisResult
{
    bool accepted{};
    std::string rejectionReason;
    std::vector<GyroscopeBatchRecordingResult> recordings;
    std::uint64_t acceptedCount{};
    std::uint64_t rejectedCount{};
    double median{};
    double mean{};
    double standardDeviation{};
    double coefficientOfVariation{};
    double minimum{};
    double maximum{};
    double directionDisagreementPercent{};
};

[[nodiscard]] GyroscopeOfflineAnalysisResult analyzeGyroscopeRecordingOffline(
    std::span<const ImuSample> samples,
    const GyroscopeBias& bias,
    GyroscopeOfflineAnalysisConfig configuration);

[[nodiscard]] double applyGyroscopeDeadband(
    double value,
    double deadband,
    GyroscopeDeadbandMode mode) noexcept;

[[nodiscard]] GyroscopeBatchAnalysisResult aggregateGyroscopeRecordings(
    std::span<const std::string> names,
    std::span<const GyroscopeOfflineAnalysisResult> analyses,
    GyroscopeBatchConfig configuration);

[[nodiscard]] std::string serializeGyroscopeOfflineAnalysisJson(
    const GyroscopeRecordingMetadata& metadata,
    const GyroscopeOfflineAnalysisResult& analysis);

[[nodiscard]] std::string serializeGyroscopeBatchAnalysisJson(
    const GyroscopeBatchAnalysisResult& batch,
    std::span<const GyroscopeOfflineAnalysisResult> analyses);

[[nodiscard]] std::string gyroscopeDeadbandModeText(GyroscopeDeadbandMode mode);
[[nodiscard]] std::string gyroscopeResidualBiasModeText(GyroscopeResidualBiasMode mode);

} // namespace xreal::sensors
