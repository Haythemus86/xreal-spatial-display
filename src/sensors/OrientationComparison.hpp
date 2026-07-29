#pragma once

#include "sensors/OrientationFusion.hpp"

#include <chrono>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace xreal::sensors
{

enum class OrientationFrame { absolute, relative };
enum class OrientationSource { gyroscope, fused };

struct AbsoluteOrientation { Quaternion value; };
struct RelativeOrientation { Quaternion value; };
struct GyroPredictionAbsolute { Quaternion value; };
struct GyroPredictionRelative { Quaternion value; };
struct FusedOrientationAbsolute { Quaternion value; };
struct FusedOrientationRelative { Quaternion value; };
struct RecenterReference { Quaternion value; bool active{}; };

struct OrientationComparisonSnapshot
{
    GyroPredictionAbsolute gyroAbsolute;
    GyroPredictionRelative gyroRelative;
    FusedOrientationAbsolute fusedAbsolute;
    FusedOrientationRelative fusedRelative;
    RecenterReference gyroRecenterReference;
    RecenterReference fusedRecenterReference;
    EulerAnglesDiagnostic gyroAbsoluteEuler;
    EulerAnglesDiagnostic gyroRelativeEuler;
    EulerAnglesDiagnostic fusedAbsoluteEuler;
    EulerAnglesDiagnostic fusedRelativeEuler;
};

struct AngularDistance
{
    double radians{};
    double degrees{};
};

struct OrientationDifference
{
    AngularDistance total;
    AngularDistance tilt;
    double yawDegrees{};
    double pitchDegrees{};
    double rollDegrees{};
};

[[nodiscard]] std::optional<RelativeOrientation> makeRelativeOrientation(
    const AbsoluteOrientation& absolute,
    const RecenterReference& reference) noexcept;
[[nodiscard]] std::optional<AngularDistance> quaternionAngularDistance(
    const Quaternion& first,
    const Quaternion& second) noexcept;
[[nodiscard]] std::optional<AngularDistance> quaternionTiltDistance(
    const Quaternion& first,
    const Quaternion& second) noexcept;
[[nodiscard]] std::optional<OrientationDifference> compareOrientations(
    const Quaternion& first,
    const Quaternion& second) noexcept;

struct OrientationComparisonConfig
{
    OrientationIntegratorConfig gyroscope;
    OrientationFusionConfig fusion;
};

struct OrientationComparisonUpdate
{
    OrientationIntegratorResult gyroscope;
    OrientationFusionResult fusion;
    OrientationComparisonSnapshot orientations;
    std::optional<OrientationDifference> absoluteDifference;
    std::optional<OrientationDifference> relativeDifference;
};

class OrientationComparisonEngine
{
public:
    explicit OrientationComparisonEngine(OrientationComparisonConfig configuration = {});
    [[nodiscard]] OrientationComparisonUpdate update(
        const AngularVelocityRadians& angularVelocity,
        const AccelerometerPhysicalSample& acceleration,
        std::uint64_t deviceTimestamp) noexcept;
    [[nodiscard]] bool recenter() noexcept;
    void clearRecenter() noexcept;
    [[nodiscard]] OrientationComparisonSnapshot snapshot() const noexcept;
    [[nodiscard]] const GyroscopeOrientationIntegrator& gyroscope() const noexcept;
    [[nodiscard]] const OrientationFusionFilter& fusion() const noexcept;

private:
    GyroscopeOrientationIntegrator gyroscope_;
    OrientationFusionFilter fusion_;
};

enum class StationaryReason
{
    stationary,
    invalidTimestamp,
    angularSpeedTooHigh,
    accelerationNormOutsideRange,
    accelerometerConfidenceTooLow,
    sustaining,
};

struct StationaryDetectorConfig
{
    double maximumGyroscopeDegreesPerSecond{1.0};
    double maximumAccelerationDeviationG{0.05};
    double minimumAccelerometerConfidence{0.8};
    std::chrono::duration<double> minimumDuration{0.5};
};

struct StationaryResult
{
    bool stationary{};
    StationaryReason reason{StationaryReason::sustaining};
    double angularSpeedDegreesPerSecond{};
    double accelerationDeviationG{};
    std::chrono::nanoseconds sustainedDuration{};
};

class StationaryDetector
{
public:
    explicit StationaryDetector(StationaryDetectorConfig configuration = {});
    [[nodiscard]] StationaryResult update(
        const AngularVelocityRadians& angularVelocity,
        const AccelerometerPhysicalSample& acceleration,
        double accelerometerConfidence,
        std::uint64_t deviceTimestamp) noexcept;
    void reset() noexcept;

private:
    StationaryDetectorConfig configuration_;
    std::optional<std::uint64_t> candidateStart_;
    std::optional<std::uint64_t> previousTimestamp_;
};

enum class ExperimentPhase
{
    calibration,
    startup,
    stationaryBefore,
    motion,
    stationaryAfter,
    complete,
};

struct OrientationExperimentConfig
{
    std::chrono::duration<double> stationaryBefore{6.0};
    std::chrono::duration<double> motion{12.0};
    std::chrono::duration<double> stationaryAfter{15.0};
    std::chrono::duration<double> recenterAt{10.0};
    std::vector<double> convergenceThresholdsDegrees{5.0, 2.0, 1.0};
    std::chrono::duration<double> convergenceSustain{0.5};
};

[[nodiscard]] ExperimentPhase experimentPhaseAt(
    std::chrono::nanoseconds elapsed,
    const OrientationExperimentConfig& configuration) noexcept;
[[nodiscard]] std::string experimentPhaseText(ExperimentPhase phase);
[[nodiscard]] std::string stationaryReasonText(StationaryReason reason);

struct DriftMetrics
{
    bool available{};
    std::uint64_t sampleCount{};
    std::uint64_t stationarySampleCount{};
    double stationarySampleRatio{};
    double durationSeconds{};
    double totalDegrees{};
    double tiltDegrees{};
    double yawDegrees{};
    double pitchDegrees{};
    double rollDegrees{};
    double totalDegreesPerSecond{};
    double tiltDegreesPerSecond{};
    double yawDegreesPerSecond{};
    double pitchDegreesPerSecond{};
    double rollDegreesPerSecond{};
};

class DriftAccumulator
{
public:
    void consume(
        const Quaternion& orientation,
        bool stationary,
        std::uint64_t deviceTimestamp) noexcept;
    [[nodiscard]] DriftMetrics metrics() const noexcept;

private:
    std::uint64_t sampleCount_{};
    std::uint64_t stationaryCount_{};
    std::optional<Quaternion> first_;
    std::optional<Quaternion> last_;
    std::optional<std::uint64_t> firstTimestamp_;
    std::optional<std::uint64_t> lastTimestamp_;
};

struct ConvergenceThresholdResult
{
    double thresholdDegrees{};
    std::optional<double> reachedAfterSeconds;
};

struct OrientationComparisonRecord
{
    std::uint64_t deviceTimestampNanoseconds{};
    double elapsedSeconds{};
    ExperimentPhase phase{ExperimentPhase::startup};
    std::uint8_t sequence{};
    StationaryResult stationary;
    AngularVelocityRadians gyroscopeRadiansPerSecond;
    AccelerometerPhysicalSample acceleration;
    double accelerometerConfidence{};
    FusionCorrectionStatus correctionStatus{FusionCorrectionStatus::skipped};
    FusionReason correctionReason{FusionReason::none};
    double correctionAngleDegrees{};
    OrientationComparisonSnapshot orientations;
    std::optional<OrientationDifference> absoluteDifference;
    std::optional<OrientationDifference> relativeDifference;
};

[[nodiscard]] std::string orientationComparisonCsvHeader();
[[nodiscard]] std::string serializeOrientationComparisonCsvRow(
    const OrientationComparisonRecord& record);

struct PhaseComparisonMetrics
{
    DriftMetrics gyroAbsolute;
    DriftMetrics gyroRelative;
    DriftMetrics fusedAbsolute;
    DriftMetrics fusedRelative;
};

struct PhaseTimeRange
{
    std::optional<std::uint64_t> startDeviceTimestampNanoseconds;
    std::optional<std::uint64_t> endDeviceTimestampNanoseconds;
};

struct TiltRecoveryMetrics
{
    bool available{};
    double maximumErrorDegrees{};
    double meanErrorDegrees{};
    double rmsErrorDegrees{};
    double finalErrorDegrees{};
};

struct OrientationComparisonSummary
{
    OrientationExperimentConfig experiment;
    StationaryDetectorConfig stationary;
    OrientationComparisonSnapshot finalOrientations;
    PhaseComparisonMetrics stationaryBefore;
    PhaseComparisonMetrics stationaryAfter;
    PhaseTimeRange stationaryBeforeTime;
    PhaseTimeRange motionTime;
    PhaseTimeRange stationaryAfterTime;
    std::vector<ConvergenceThresholdResult> gyroConvergence;
    std::vector<ConvergenceThresholdResult> fusedConvergence;
    TiltRecoveryMetrics gyroRecovery;
    TiltRecoveryMetrics fusedRecovery;
    std::optional<OrientationDifference> finalAbsoluteDifference;
    std::optional<OrientationDifference> finalRelativeDifference;
    std::string gyroscopeProfileSource;
    std::string accelerometerProfileSource;
    std::uint64_t receivedPackets{};
    std::uint64_t gyroscopeSamplesApplied{};
    std::uint64_t accelerometerCorrectionsApplied{};
    std::uint64_t accelerometerCorrectionsSkipped{};
    std::uint64_t rejectedSamples{};
};

[[nodiscard]] std::string serializeOrientationComparisonJson(
    const OrientationComparisonSummary& summary);

class TiltConvergenceTracker
{
public:
    TiltConvergenceTracker(
        Quaternion target,
        std::span<const double> thresholdsDegrees,
        std::chrono::duration<double> sustainDuration);
    void consume(const Quaternion& orientation, std::uint64_t deviceTimestamp) noexcept;
    [[nodiscard]] const std::vector<ConvergenceThresholdResult>& results() const noexcept;
    [[nodiscard]] double maximumErrorDegrees() const noexcept;
    [[nodiscard]] double meanErrorDegrees() const noexcept;
    [[nodiscard]] double rmsErrorDegrees() const noexcept;
    [[nodiscard]] std::optional<double> finalErrorDegrees() const noexcept;

private:
    Quaternion target_;
    std::chrono::duration<double> sustainDuration_;
    std::vector<ConvergenceThresholdResult> results_;
    std::vector<std::optional<std::uint64_t>> candidateStarts_;
    std::optional<std::uint64_t> firstTimestamp_;
    double maximumError_{};
    double sumError_{};
    double sumSquaredError_{};
    std::uint64_t count_{};
    std::optional<double> finalError_;
};

} // namespace xreal::sensors
