#pragma once

#include "sensors/GyroscopeBiasCalibration.hpp"
#include "sensors/ImuSample.hpp"

#include <chrono>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace xreal::sensors
{

enum class GyroscopeAxis
{
    x,
    y,
    z,
};

enum class RotationDirection
{
    positive,
    negative,
    automatic,
};

enum class RawAngularIntegrationRejectionReason
{
    none,
    invalidTimestamp,
    timestampDeltaTooLarge,
    insufficientSamples,
};

struct RawAngularIntegrationResult
{
    bool valid{};
    RawAngularIntegrationRejectionReason rejectionReason{
        RawAngularIntegrationRejectionReason::none};
    double integratedRawAngle{};
    double positiveContribution{};
    double negativeContribution{};
    double peakCorrectedRawRate{};
    double meanCorrectedRawRate{};
    double packetRate{};
    std::chrono::nanoseconds captureDuration{};
    std::uint64_t sampleCount{};
};

class GyroscopeRawAngularIntegrator
{
public:
    GyroscopeRawAngularIntegrator(
        GyroscopeAxis axis,
        GyroscopeBias bias,
        std::chrono::nanoseconds maximumTimestampDelta);

    void consume(const ImuSample& sample) noexcept;
    [[nodiscard]] bool failed() const noexcept;
    [[nodiscard]] RawAngularIntegrationResult finish() const noexcept;

private:
    GyroscopeAxis axis_;
    GyroscopeBias bias_;
    std::chrono::nanoseconds maximumTimestampDelta_;
    std::optional<std::uint64_t> previousTimestamp_;
    double previousRate_{};
    double integratedRawAngle_{};
    double positiveContribution_{};
    double negativeContribution_{};
    double peakCorrectedRawRate_{};
    std::uint64_t accumulatedNanoseconds_{};
    std::uint64_t sampleCount_{};
    RawAngularIntegrationRejectionReason rejectionReason_{
        RawAngularIntegrationRejectionReason::none};
};

enum class GyroscopeScaleTrialPhase
{
    waitingForRotation,
    rotating,
    waitingForStillnessAfterRotation,
    completed,
    rejected,
};

enum class GyroscopeScaleTrialRejectionReason
{
    none,
    missingBias,
    invalidTimestamp,
    timestampDeltaTooLarge,
    wrongDominantAxis,
    excessiveCrossAxisMotion,
    rotationTooShort,
    rotationTooLong,
    directionMismatch,
    unstableStop,
    insufficientSamples,
    packetRateOutOfRange,
    streamDataLoss,
    scaleOutlier,
};

struct GyroscopeScaleCalibrationConfig
{
    GyroscopeAxis axis{GyroscopeAxis::z};
    double expectedAngleDegrees{360.0};
    RotationDirection direction{RotationDirection::automatic};
    double startThresholdRaw{5000.0};
    double stopThresholdRaw{1500.0};
    std::chrono::nanoseconds stillnessDuration{std::chrono::seconds(1)};
    std::chrono::nanoseconds minimumRotationDuration{std::chrono::milliseconds(500)};
    std::chrono::nanoseconds maximumRotationDuration{std::chrono::seconds(20)};
    std::chrono::nanoseconds maximumTimestampDelta{std::chrono::milliseconds(20)};
    std::optional<PacketRateRange> acceptablePacketRate{PacketRateRange{800.0, 1200.0}};
    double maximumCrossAxisRatio{0.35};
    double maximumTrialVariationPercent{10.0};
    double maximumDirectionDisagreementPercent{10.0};
    double outlierDeviationPercent{20.0};
    std::uint64_t minimumAcceptedTrials{4};
};

struct GyroscopeScaleCalibrationTrial
{
    bool accepted{};
    GyroscopeScaleTrialRejectionReason rejectionReason{
        GyroscopeScaleTrialRejectionReason::none};
    RotationDirection measuredDirection{RotationDirection::automatic};
    GyroscopeAxis dominantAxis{GyroscopeAxis::z};
    double expectedAngleDegrees{};
    double scaleRawPerDegreePerSecond{};
    RawAngularIntegrationResult integration;
    RawVector3d peakCorrectedRawByAxis;
};

class GyroscopeScaleTrialCalibrator
{
public:
    GyroscopeScaleTrialCalibrator(
        GyroscopeScaleCalibrationConfig configuration,
        std::optional<GyroscopeBias> bias);

    void consume(const ImuSample& sample) noexcept;
    [[nodiscard]] GyroscopeScaleTrialPhase phase() const noexcept;
    [[nodiscard]] bool isComplete() const noexcept;
    [[nodiscard]] std::optional<GyroscopeScaleCalibrationTrial> result() const noexcept;
    [[nodiscard]] GyroscopeScaleCalibrationTrial finish() noexcept;

private:
    void reject(GyroscopeScaleTrialRejectionReason reason) noexcept;
    void complete() noexcept;

    GyroscopeScaleCalibrationConfig configuration_;
    std::optional<GyroscopeBias> bias_;
    GyroscopeScaleTrialPhase phase_{GyroscopeScaleTrialPhase::waitingForRotation};
    std::optional<GyroscopeRawAngularIntegrator> integrator_;
    std::optional<ImuSample> previousSample_;
    std::optional<std::uint64_t> previousTimestamp_;
    std::uint64_t rotationElapsedNanoseconds_{};
    std::uint64_t stillnessElapsedNanoseconds_{};
    RawVector3d peakCorrectedRawByAxis_;
    std::optional<GyroscopeScaleCalibrationTrial> result_;
};

enum class GyroscopeScaleCalibrationRejectionReason
{
    none,
    tooFewAcceptedTrials,
    excessiveTrialVariation,
    directionDisagreement,
    invalidScale,
};

struct GyroscopeScaleStatistics
{
    double mean{};
    double median{};
    double standardDeviation{};
    double coefficientOfVariation{};
    double minimum{};
    double maximum{};
    double relativeSpread{};
};

struct GyroscopeAxisScale
{
    bool valid{};
    GyroscopeAxis axis{GyroscopeAxis::z};
    double rawUnitsPerDegreePerSecond{};
    double rawUnitsPerRadianPerSecond{};
    double degreesPerSecondPerRawUnit{};
    double radiansPerSecondPerRawUnit{};
    std::uint64_t acceptedTrialCount{};
    double trialStandardDeviation{};
};

struct GyroscopeScaleCalibrationResult
{
    bool accepted{};
    GyroscopeScaleCalibrationRejectionReason rejectionReason{
        GyroscopeScaleCalibrationRejectionReason::none};
    GyroscopeScaleCalibrationConfig configuration;
    GyroscopeAxisScale scale;
    GyroscopeScaleStatistics statistics;
    std::uint64_t acceptedTrialCount{};
    std::uint64_t rejectedTrialCount{};
    std::vector<GyroscopeScaleCalibrationTrial> trials;
};

struct GyroscopeAngularVelocity
{
    double degreesPerSecond{};
    double radiansPerSecond{};
};

[[nodiscard]] GyroscopeScaleCalibrationResult calculateGyroscopeScaleCalibration(
    GyroscopeScaleCalibrationConfig configuration,
    std::span<const GyroscopeScaleCalibrationTrial> trials);

[[nodiscard]] std::optional<GyroscopeAngularVelocity> applyGyroscopeScale(
    GyroscopeAxis axis,
    double biasCorrectedRaw,
    const GyroscopeAxisScale& scale) noexcept;

[[nodiscard]] std::string gyroscopeAxisText(GyroscopeAxis axis);
[[nodiscard]] std::string rotationDirectionText(RotationDirection direction);
[[nodiscard]] std::string gyroscopeScaleTrialRejectionReasonText(
    GyroscopeScaleTrialRejectionReason reason);
[[nodiscard]] std::string gyroscopeScaleCalibrationRejectionReasonText(
    GyroscopeScaleCalibrationRejectionReason reason);

[[nodiscard]] std::string serializeGyroscopeScaleCalibrationJson(
    const GyroscopeBiasCalibrationDevice& device,
    const GyroscopeScaleCalibrationResult& result);

} // namespace xreal::sensors
