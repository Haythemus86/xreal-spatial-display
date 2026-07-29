#pragma once

#include "sensors/OrientationComparison.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>

namespace xreal::sensors
{

enum class PredictionMode { constantVelocity, constantAcceleration };
enum class PredictionLimitBehavior { reject, clamp };
enum class PredictionValidity { valid, rejected };
enum class PredictionRejectionReason
{
    none,
    invalidOrientation,
    invalidAngularVelocity,
    decreasingTimestamp,
    horizonExceedsMaximum,
    angularSpeedExceedsMaximum,
    predictedAngleExceedsMaximum,
};
enum class PredictionFallbackReason
{
    none,
    firstSample,
    duplicateTimestamp,
    excessiveTimestampDelta,
    angularAccelerationExceedsMaximum,
};

struct PredictionHorizon
{
    std::chrono::duration<double> requested{0.010};
    std::chrono::duration<double> applied{0.010};
    bool clamped{};
};

struct AngularVelocityEstimate
{
    AngularVelocityRadians raw;
    AngularVelocityRadians filtered;
    bool smoothingApplied{};
};

struct AngularAccelerationRadians
{
    double xRadiansPerSecondSquared{};
    double yRadiansPerSecondSquared{};
    double zRadiansPerSecondSquared{};

    [[nodiscard]] bool finite() const noexcept;
};

struct AngularAccelerationEstimate
{
    AngularAccelerationRadians raw;
    AngularAccelerationRadians filtered;
    bool available{};
    bool used{};
};

struct PredictedAbsoluteOrientation { Quaternion value; };
struct PredictedRelativeOrientation { Quaternion value; };

struct OrientationPredictorConfig
{
    PredictionMode mode{PredictionMode::constantVelocity};
    std::chrono::duration<double> horizon{0.010};
    std::chrono::duration<double> maximumHorizon{0.050};
    std::optional<std::chrono::duration<double>> angularVelocitySmoothingTimeConstant;
    std::optional<std::chrono::duration<double>> angularAccelerationSmoothingTimeConstant;
    std::chrono::nanoseconds maximumTimestampDelta{std::chrono::milliseconds(20)};
    double maximumAngularSpeedRadiansPerSecond{17.453292519943295};
    double maximumAngularAccelerationRadiansPerSecondSquared{349.0658503988659};
    double maximumPredictionAngleRadians{0.5235987755982988};
    double maximumAccelerationContributionRadians{0.08726646259971647};
    double smallAngleThresholdRadians{1.0e-8};
    PredictionLimitBehavior limitBehavior{PredictionLimitBehavior::reject};
};

struct OrientationPredictorInput
{
    AbsoluteOrientation measuredAbsolute;
    RecenterReference recenterReference;
    AngularVelocityRadians bodyAngularVelocity;
    std::uint64_t deviceTimestampNanoseconds{};
};

struct OrientationPredictionDiagnostics
{
    double rawAngularSpeedRadiansPerSecond{};
    double filteredAngularSpeedRadiansPerSecond{};
    double angularAccelerationRadiansPerSecondSquared{};
    double predictedAngleRadians{};
    double accelerationContributionRadians{};
    bool speedClamped{};
    bool accelerationClamped{};
    bool angleClamped{};
    PredictionFallbackReason fallbackReason{PredictionFallbackReason::none};
};

struct OrientationPredictionResult
{
    PredictionValidity validity{PredictionValidity::rejected};
    PredictionRejectionReason rejectionReason{PredictionRejectionReason::none};
    PredictionHorizon horizon;
    AngularVelocityEstimate angularVelocity;
    AngularAccelerationEstimate angularAcceleration;
    PredictedAbsoluteOrientation absolute;
    PredictedRelativeOrientation relative;
    OrientationPredictionDiagnostics diagnostics;
};

struct OrientationPredictorStatistics
{
    std::uint64_t requested{};
    std::uint64_t produced{};
    std::uint64_t rejected{};
    std::uint64_t clampedHorizons{};
    std::uint64_t clampedSpeeds{};
    std::uint64_t clampedAccelerations{};
    std::uint64_t clampedAngles{};
    std::uint64_t constantAccelerationPredictions{};
    std::uint64_t constantVelocityFallbacks{};
    std::uint64_t duplicateTimestamps{};
    std::uint64_t decreasingTimestamps{};
    std::uint64_t excessiveTimestampDeltas{};
    std::uint64_t zeroHorizonPredictions{};
    std::uint64_t constantVelocityPredictions{};
    std::uint64_t invalidInputRejections{};
    double requestedHorizonSecondsSum{};
    double appliedHorizonSecondsSum{};
    double maximumObservedAngularSpeedRadiansPerSecond{};
    double maximumObservedPredictionAngleRadians{};
};

struct OrientationPredictorState
{
    std::optional<std::uint64_t> previousDeviceTimestampNanoseconds;
    std::optional<AngularVelocityRadians> filteredAngularVelocity;
    std::optional<AngularAccelerationRadians> filteredAngularAcceleration;
    OrientationPredictorStatistics statistics;
};

class OrientationPredictor
{
public:
    explicit OrientationPredictor(OrientationPredictorConfig configuration = {});
    [[nodiscard]] OrientationPredictionResult predict(
        const OrientationPredictorInput& input) noexcept;
    void reset() noexcept;
    [[nodiscard]] const OrientationPredictorConfig& configuration() const noexcept;
    [[nodiscard]] const OrientationPredictorState& state() const noexcept;

private:
    OrientationPredictorConfig configuration_;
    OrientationPredictorState state_;
};

[[nodiscard]] std::optional<Quaternion> quaternionFromBodyRotationVector(
    const AngularVelocityRadians& rotationVectorRadians,
    double smallAngleThresholdRadians = 1.0e-8) noexcept;
[[nodiscard]] std::string predictionModeText(PredictionMode mode);
[[nodiscard]] std::string predictionValidityText(PredictionValidity validity);
[[nodiscard]] std::string predictionRejectionReasonText(PredictionRejectionReason reason);
[[nodiscard]] std::string predictionFallbackReasonText(PredictionFallbackReason reason);

} // namespace xreal::sensors
