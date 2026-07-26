#pragma once

#include "sensors/GyroscopeBiasCalibration.hpp"
#include "sensors/GyroscopeScaleCalibration.hpp"
#include "sensors/ImuSample.hpp"

#include <chrono>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace xreal::sensors
{

enum class GyroscopeAxisSelection
{
    x,
    y,
    z,
    automatic,
};

enum class RecordingTimestampStatus
{
    firstSample,
    valid,
    duplicate,
    decreasing,
    deltaTooLarge,
};

struct GyroscopeRecordingAnalysisConfig
{
    GyroscopeAxisSelection axisSelection{GyroscopeAxisSelection::automatic};
    RotationDirection requestedDirection{RotationDirection::automatic};
    std::optional<double> expectedAngleDegrees;
    double startThresholdRaw{5000.0};
    double stopThresholdRaw{1500.0};
    std::chrono::nanoseconds stillnessDuration{std::chrono::seconds(1)};
    std::chrono::nanoseconds maximumDeviceDelta{std::chrono::milliseconds(20)};
};

struct GyroscopeRecordingSample
{
    ImuSample original;
    CorrectedGyroscopeRaw corrected;
    RecordingTimestampStatus timestampStatus{RecordingTimestampStatus::firstSample};
    std::optional<std::uint64_t> deviceDeltaNanoseconds;
    std::uint64_t elapsedDeviceNanoseconds{};
    std::uint64_t sequenceGap{};
    bool sampleValid{true};
};

struct GyroscopeRecordingAxisAnalysis
{
    std::uint64_t sampleCount{};
    std::uint64_t validSampleCount{};
    std::chrono::nanoseconds captureDuration{};
    double minimumCorrectedRaw{};
    double maximumCorrectedRaw{};
    double meanCorrectedRaw{};
    double standardDeviationCorrectedRaw{};
    double rootMeanSquareCorrectedRaw{};
    double peakAbsoluteCorrectedRaw{};
    double positiveIntegratedRawAngle{};
    double negativeIntegratedRawAngle{};
    double signedIntegratedRawAngle{};
    double absoluteIntegratedRawAngle{};
    std::chrono::nanoseconds durationAboveStartThreshold{};
    std::chrono::nanoseconds durationAboveStopThreshold{};
    double rotationalEnergy{};
    double rotationalEnergyPercent{};
    double dominantAxisScore{};
};

struct GyroscopeRotationSegment
{
    std::uint64_t startDeviceTimestamp{};
    std::uint64_t endDeviceTimestamp{};
    std::chrono::nanoseconds duration{};
    GyroscopeAxis dominantAxis{GyroscopeAxis::z};
    RotationDirection direction{RotationDirection::automatic};
    RawVector3d integratedRawAngle;
    RawVector3d peakAbsoluteCorrectedRaw;
    double crossAxisRatio{};
    double dominanceScore{};
    bool stillnessBefore{};
    bool stillnessAfter{};
    std::uint64_t sampleCount{};
    double selectionScore{};
};

struct ExperimentalGyroscopeScaleEstimate
{
    bool available{};
    bool experimental{true};
    GyroscopeAxis axis{GyroscopeAxis::z};
    double expectedAngleDegrees{};
    double rawUnitsPerDegreePerSecond{};
    double rawUnitsPerRadianPerSecond{};
    double degreesPerSecondPerRawUnit{};
    double radiansPerSecondPerRawUnit{};
    bool directionMatchesRequest{true};
    double confidence{};
};

enum class GyroscopeRecordingAnalysisFailure
{
    none,
    noSamples,
    noUsableSegment,
};

struct GyroscopeRecordingAnalysisResult
{
    bool usable{};
    GyroscopeRecordingAnalysisFailure failure{GyroscopeRecordingAnalysisFailure::none};
    GyroscopeRecordingAnalysisConfig configuration;
    GyroscopeBias bias;
    std::vector<GyroscopeRecordingSample> samples;
    GyroscopeRecordingAxisAnalysis x;
    GyroscopeRecordingAxisAnalysis y;
    GyroscopeRecordingAxisAnalysis z;
    std::vector<GyroscopeRotationSegment> segments;
    std::optional<std::size_t> bestSegmentIndex;
    ExperimentalGyroscopeScaleEstimate scaleEstimate;
    GyroscopeAxis dominantAxis{GyroscopeAxis::z};
    std::uint64_t validTimestampSamples{};
    std::uint64_t invalidTimestampSamples{};
    std::uint64_t sequenceGaps{};
    std::vector<std::string> warnings;
};

struct GyroscopeRecordingMetadata
{
    GyroscopeBiasCalibrationDevice device;
    GyroscopeBiasCalibrationResult biasCalibration;
    double requestedDurationSeconds{};
    double measuredDurationSeconds{};
    std::uint64_t malformedPackets{};
    std::uint64_t outOfSequenceEvents{};
    std::uint64_t diagnosticQueueDrops{};
};

[[nodiscard]] GyroscopeRecordingAnalysisResult analyzeGyroscopeRecording(
    std::span<const ImuSample> samples,
    const GyroscopeBias& bias,
    GyroscopeRecordingAnalysisConfig configuration);

[[nodiscard]] std::string gyroscopeRecordingCsvHeader(
    bool includeAccelerometer,
    bool includeHostTimestamps);

[[nodiscard]] std::string serializeGyroscopeRecordingCsvRow(
    const GyroscopeRecordingSample& sample,
    const GyroscopeBias& bias,
    bool includeAccelerometer,
    bool includeHostTimestamps);

[[nodiscard]] std::string serializeGyroscopeRecordingAnalysisJson(
    const GyroscopeRecordingMetadata& metadata,
    const GyroscopeRecordingAnalysisResult& analysis);

[[nodiscard]] std::string recordingTimestampStatusText(RecordingTimestampStatus status);
[[nodiscard]] std::string gyroscopeRecordingAnalysisFailureText(
    GyroscopeRecordingAnalysisFailure failure);
[[nodiscard]] std::string gyroscopeAxisSelectionText(GyroscopeAxisSelection selection);

} // namespace xreal::sensors
