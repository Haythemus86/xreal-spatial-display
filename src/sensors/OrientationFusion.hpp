#pragma once

#include "sensors/AccelerometerPhysicalUnits.hpp"
#include "sensors/GyroscopeOrientation.hpp"

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>

namespace xreal::sensors
{

enum class FusionStartupMode
{
    identity,
    gravity,
};

enum class FusionCorrectionStatus
{
    applied,
    skipped,
    rejected,
};

enum class FusionStartupStatus
{
    identity,
    gravityAligned,
    waitingForConfidence,
    rejected,
};

enum class FusionReason
{
    none,
    firstTimestamp,
    duplicateTimestamp,
    decreasingTimestamp,
    excessiveTimestampDelta,
    invalidGyroscope,
    invalidAccelerometer,
    accelerationNearZero,
    accelerationNormOutsideRange,
    jerkExceeded,
    confidenceHysteresis,
    startupWaitingForGravity,
    antiParallelGravity,
    correctionDisabled,
    invalidQuaternion,
};

struct GravityConfidenceConfig
{
    double fullConfidenceDeviationG{0.05};
    double zeroConfidenceDeviationG{0.20};
    double minimumAccelerationNormG{0.05};
    std::chrono::duration<double> smoothingTimeConstant{0.25};
    std::optional<double> maximumJerkGPerSecond;
    bool hysteresisEnabled{};
    double hysteresisEnterConfidence{0.6};
    double hysteresisExitConfidence{0.2};
};

struct OrientationFusionConfig
{
    bool enabled{true};
    bool accelerometerCorrectionEnabled{true};
    std::chrono::duration<double> correctionTimeConstant{2.0};
    double correctionGain{1.0};
    double maximumCorrectionDegreesPerSecond{10.0};
    std::chrono::nanoseconds maximumDeviceTimestampDelta{std::chrono::milliseconds(20)};
    GravityConfidenceConfig confidence;
    FusionStartupMode startupMode{FusionStartupMode::identity};
    GyroscopeAxisMapping gyroscopeAxisMapping;
    bool experimental{true};
    bool verified{};
};

struct AccelerometerConfidence
{
    double normG{};
    double deviationG{};
    double rawConfidence{};
    double smoothedConfidence{};
    double correctionConfidence{};
    std::optional<double> jerkGPerSecond;
    bool valid{};
    FusionReason reason{FusionReason::none};
};

struct GravityObservation
{
    PhysicalVector3d measuredBodyUp;
    PhysicalVector3d predictedBodyUp;
    AccelerometerConfidence confidence;
};

struct OrientationFusionState
{
    Quaternion fusedOrientation;
    Quaternion gyroscopePredictedOrientation;
    Quaternion recenterReference;
    bool initialized{};
    bool valid{true};
    bool gravityStartupApplied{};
    std::optional<std::uint64_t> lastDeviceTimestamp;
    std::uint64_t appliedGyroscopeSampleCount{};
    std::uint64_t appliedAccelerometerCorrectionCount{};
    std::uint64_t skippedAccelerometerCorrectionCount{};
    std::uint64_t rejectedSampleCount{};
    double currentAccelerationNormG{};
    double currentAccelerometerConfidence{};
    double currentCorrectionAngleRadians{};
    double accumulatedCorrectionAngleRadians{};
    std::chrono::nanoseconds integrationDuration{};
    std::chrono::nanoseconds fusionDuration{};
    FusionStartupStatus startupStatus{FusionStartupStatus::identity};
    bool experimental{true};
    bool verified{};
    FusionReason lastReason{FusionReason::none};
};

struct OrientationFusionResult
{
    OrientationSampleStatus gyroscopeStatus{OrientationSampleStatus::skipped};
    FusionCorrectionStatus correctionStatus{FusionCorrectionStatus::skipped};
    FusionReason reason{FusionReason::none};
    Quaternion gyroscopePrediction;
    Quaternion fusedOrientation;
    Quaternion relativeOrientation;
    GravityObservation gravity;
    std::chrono::nanoseconds deltaTime{};
    double appliedCorrectionAngleRadians{};
};

class AccelerometerConfidenceEstimator
{
public:
    explicit AccelerometerConfidenceEstimator(GravityConfidenceConfig configuration = {});

    [[nodiscard]] AccelerometerConfidence update(
        const AccelerometerPhysicalSample& sample,
        std::uint64_t deviceTimestamp) noexcept;
    void reset() noexcept;

private:
    GravityConfidenceConfig configuration_;
    std::optional<std::uint64_t> previousTimestamp_;
    std::optional<PhysicalVector3d> previousAccelerationG_;
    double smoothedConfidence_{};
    bool initialized_{};
    bool hysteresisActive_{};
};

class OrientationFusionFilter
{
public:
    explicit OrientationFusionFilter(OrientationFusionConfig configuration = {});

    [[nodiscard]] OrientationFusionResult update(
        const AngularVelocityRadians& angularVelocity,
        const AccelerometerPhysicalSample& acceleration,
        std::uint64_t deviceTimestamp) noexcept;
    void reset() noexcept;
    [[nodiscard]] bool recenter() noexcept;
    void clearRecenter() noexcept;
    [[nodiscard]] bool setOrientation(const Quaternion& orientation) noexcept;
    [[nodiscard]] Quaternion orientation() const noexcept;
    [[nodiscard]] Quaternion relativeOrientation() const noexcept;
    [[nodiscard]] const OrientationFusionState& state() const noexcept;
    [[nodiscard]] const OrientationFusionConfig& configuration() const noexcept;

private:
    [[nodiscard]] OrientationFusionResult makeResult(
        OrientationSampleStatus gyroscopeStatus,
        FusionCorrectionStatus correctionStatus,
        FusionReason reason,
        const GravityObservation& gravity,
        std::chrono::nanoseconds delta,
        double correctionAngle) const noexcept;

    OrientationFusionConfig configuration_;
    GyroscopeOrientationIntegrator gyroscopeIntegrator_;
    AccelerometerConfidenceEstimator confidenceEstimator_;
    OrientationFusionState state_;
};

[[nodiscard]] PhysicalVector3d predictBodyFrameAccelerometerUp(
    const Quaternion& bodyToWorldOrientation) noexcept;
[[nodiscard]] std::optional<Quaternion> gravityAlignedStartupOrientation(
    const PhysicalVector3d& measuredBodyUp) noexcept;
[[nodiscard]] std::string serializeOrientationFusionJson(
    const OrientationFusionFilter& filter,
    const GyroscopeScaleProfile& gyroscopeScale,
    const AccelerometerCalibrationProfile& accelerometerProfile,
    bool includeEuler);
[[nodiscard]] std::string fusionReasonText(FusionReason reason);

} // namespace xreal::sensors
