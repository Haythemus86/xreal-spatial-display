#include "sensors/OrientationComparison.hpp"

#include "sensors/DeviceTimestampDelta.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <locale>
#include <numbers>
#include <sstream>
#include <iomanip>
#include <string_view>
#include <utility>

namespace xreal::sensors
{
namespace
{

constexpr double radiansToDegrees = 180.0 / std::numbers::pi;

[[nodiscard]] std::string escapeJson(std::string_view value)
{
    std::string result;
    for (const char character : value)
    {
        switch (character)
        {
        case '\\': result += "\\\\"; break;
        case '"': result += "\\\""; break;
        case '\n': result += "\\n"; break;
        case '\r': result += "\\r"; break;
        case '\t': result += "\\t"; break;
        default: result += character; break;
        }
    }
    return result;
}

[[nodiscard]] std::string quoteCsv(std::string_view value)
{
    std::string result{"\""};
    for (const char character : value)
    {
        if (character == '"')
        {
            result += "\"\"";
        }
        else
        {
            result += character;
        }
    }
    result += '"';
    return result;
}

void appendQuaternion(std::ostringstream& output, const Quaternion& value)
{
    output << value.w << ',' << value.x << ',' << value.y << ',' << value.z;
}

void appendEuler(std::ostringstream& output, const EulerAnglesDiagnostic& value)
{
    output << value.yawDegrees << ',' << value.pitchDegrees << ',' << value.rollDegrees;
}

void appendJsonQuaternion(std::ostringstream& output, const Quaternion& value)
{
    output << "{\"w\":" << value.w << ",\"x\":" << value.x
           << ",\"y\":" << value.y << ",\"z\":" << value.z << '}';
}

void appendJsonMetric(std::ostringstream& output, const DriftMetrics& metric)
{
    output << "{\"available\":" << (metric.available ? "true" : "false")
           << ",\"sample_count\":" << metric.sampleCount
           << ",\"stationary_sample_count\":" << metric.stationarySampleCount
           << ",\"duration_seconds\":" << metric.durationSeconds
           << ",\"stationary_sample_ratio\":" << metric.stationarySampleRatio
           << ",\"total_drift_degrees\":" << metric.totalDegrees
           << ",\"tilt_drift_degrees\":" << metric.tiltDegrees
           << ",\"yaw_drift_degrees\":" << metric.yawDegrees
           << ",\"pitch_drift_degrees\":" << metric.pitchDegrees
           << ",\"roll_drift_degrees\":" << metric.rollDegrees
           << ",\"total_drift_degrees_per_second\":" << metric.totalDegreesPerSecond
           << ",\"tilt_drift_degrees_per_second\":" << metric.tiltDegreesPerSecond
           << ",\"yaw_drift_degrees_per_second\":" << metric.yawDegreesPerSecond
           << ",\"pitch_drift_degrees_per_second\":" << metric.pitchDegreesPerSecond
           << ",\"roll_drift_degrees_per_second\":" << metric.rollDegreesPerSecond << '}';
}

[[nodiscard]] double wrappedDegrees(double value) noexcept
{
    double wrapped = std::remainder(value, 360.0);
    if (wrapped <= -180.0)
    {
        wrapped += 360.0;
    }
    return wrapped;
}

} // namespace

std::optional<RelativeOrientation> makeRelativeOrientation(
    const AbsoluteOrientation& absolute,
    const RecenterReference& reference) noexcept
{
    const auto normalizedAbsolute = absolute.value.normalized();
    const auto inverseReference = reference.value.inverseNormalized();
    if (!normalizedAbsolute.has_value() || !inverseReference.has_value())
    {
        return std::nullopt;
    }
    const auto relative = (*inverseReference * *normalizedAbsolute).normalized();
    if (!relative.has_value())
    {
        return std::nullopt;
    }
    return RelativeOrientation{*relative};
}

std::optional<AngularDistance> quaternionAngularDistance(
    const Quaternion& first,
    const Quaternion& second) noexcept
{
    const auto firstNormalized = first.normalized();
    const auto secondNormalized = second.normalized();
    if (!firstNormalized.has_value() || !secondNormalized.has_value())
    {
        return std::nullopt;
    }
    const double dot = std::abs(firstNormalized->w * secondNormalized->w
        + firstNormalized->x * secondNormalized->x
        + firstNormalized->y * secondNormalized->y
        + firstNormalized->z * secondNormalized->z);
    const double radians = 2.0 * std::acos(std::clamp(dot, 0.0, 1.0));
    return AngularDistance{radians, radians * radiansToDegrees};
}

std::optional<AngularDistance> quaternionTiltDistance(
    const Quaternion& first,
    const Quaternion& second) noexcept
{
    const PhysicalVector3d firstUp = predictBodyFrameAccelerometerUp(first);
    const PhysicalVector3d secondUp = predictBodyFrameAccelerometerUp(second);
    if (!firstUp.finite() || !secondUp.finite()
        || firstUp.norm() < 0.5 || secondUp.norm() < 0.5)
    {
        return std::nullopt;
    }
    const double dot = std::clamp(
        firstUp.x * secondUp.x + firstUp.y * secondUp.y + firstUp.z * secondUp.z,
        -1.0,
        1.0);
    const double radians = std::acos(dot);
    return AngularDistance{radians, radians * radiansToDegrees};
}

std::optional<OrientationDifference> compareOrientations(
    const Quaternion& first,
    const Quaternion& second) noexcept
{
    const auto total = quaternionAngularDistance(first, second);
    const auto tilt = quaternionTiltDistance(first, second);
    if (!total.has_value() || !tilt.has_value())
    {
        return std::nullopt;
    }
    const auto firstEuler = quaternionToEulerDiagnostic(first);
    const auto secondEuler = quaternionToEulerDiagnostic(second);
    return OrientationDifference{
        *total,
        *tilt,
        wrappedDegrees(secondEuler.yawDegrees - firstEuler.yawDegrees),
        wrappedDegrees(secondEuler.pitchDegrees - firstEuler.pitchDegrees),
        wrappedDegrees(secondEuler.rollDegrees - firstEuler.rollDegrees),
    };
}

OrientationComparisonEngine::OrientationComparisonEngine(OrientationComparisonConfig configuration)
    : gyroscope_(configuration.gyroscope), fusion_(configuration.fusion)
{
}

OrientationComparisonUpdate OrientationComparisonEngine::update(
    const AngularVelocityRadians& angularVelocity,
    const AccelerometerPhysicalSample& acceleration,
    std::uint64_t deviceTimestamp) noexcept
{
    const auto gyroscopeResult = gyroscope_.update(angularVelocity, deviceTimestamp);
    const auto fusionResult = fusion_.update(angularVelocity, acceleration, deviceTimestamp);
    const auto current = snapshot();
    return {
        gyroscopeResult,
        fusionResult,
        current,
        compareOrientations(current.gyroAbsolute.value, current.fusedAbsolute.value),
        compareOrientations(current.gyroRelative.value, current.fusedRelative.value),
    };
}

bool OrientationComparisonEngine::recenter() noexcept
{
    const Quaternion gyroBefore = gyroscope_.orientation();
    const Quaternion fusionBefore = fusion_.orientation();
    if (!gyroBefore.normalized().has_value() || !fusionBefore.normalized().has_value())
    {
        return false;
    }
    const bool gyroSuccess = gyroscope_.recenter();
    const bool fusionSuccess = fusion_.recenter();
    const auto gyroDifference = quaternionAngularDistance(
        gyroBefore, gyroscope_.orientation());
    const auto fusionDifference = quaternionAngularDistance(
        fusionBefore, fusion_.orientation());
    return gyroSuccess && fusionSuccess && gyroDifference.has_value()
        && fusionDifference.has_value() && gyroDifference->degrees < 1.0e-9
        && fusionDifference->degrees < 1.0e-9;
}

void OrientationComparisonEngine::clearRecenter() noexcept
{
    gyroscope_.clearRecenter();
    fusion_.clearRecenter();
}

OrientationComparisonSnapshot OrientationComparisonEngine::snapshot() const noexcept
{
    const Quaternion gyroAbsolute = gyroscope_.orientation();
    const Quaternion gyroRelative = gyroscope_.relativeOrientation();
    const Quaternion fusedAbsolute = fusion_.orientation();
    const Quaternion fusedRelative = fusion_.relativeOrientation();
    const Quaternion gyroReference = gyroscope_.recenterReference();
    const Quaternion fusedReference = fusion_.state().recenterReference;
    return {
        {gyroAbsolute}, {gyroRelative}, {fusedAbsolute}, {fusedRelative},
        {gyroReference, gyroscope_.recenterActive()},
        {fusedReference, fusion_.state().recenterActive},
        quaternionToEulerDiagnostic(gyroAbsolute),
        quaternionToEulerDiagnostic(gyroRelative),
        quaternionToEulerDiagnostic(fusedAbsolute),
        quaternionToEulerDiagnostic(fusedRelative),
    };
}

const GyroscopeOrientationIntegrator& OrientationComparisonEngine::gyroscope() const noexcept
{
    return gyroscope_;
}

const OrientationFusionFilter& OrientationComparisonEngine::fusion() const noexcept
{
    return fusion_;
}

StationaryDetector::StationaryDetector(StationaryDetectorConfig configuration)
    : configuration_(std::move(configuration))
{
}

void StationaryDetector::reset() noexcept
{
    candidateStart_.reset();
    previousTimestamp_.reset();
}

StationaryResult StationaryDetector::update(
    const AngularVelocityRadians& angularVelocity,
    const AccelerometerPhysicalSample& acceleration,
    double confidence,
    std::uint64_t timestamp) noexcept
{
    StationaryResult result;
    result.angularSpeedDegreesPerSecond = std::sqrt(
        angularVelocity.xRadiansPerSecond * angularVelocity.xRadiansPerSecond
        + angularVelocity.yRadiansPerSecond * angularVelocity.yRadiansPerSecond
        + angularVelocity.zRadiansPerSecond * angularVelocity.zRadiansPerSecond) * radiansToDegrees;
    result.accelerationDeviationG = std::abs(acceleration.normG - 1.0);
    if (previousTimestamp_.has_value()
        && !forwardDeviceTimestampDelta(timestamp, *previousTimestamp_).has_value())
    {
        candidateStart_.reset();
        result.reason = StationaryReason::invalidTimestamp;
        return result;
    }
    previousTimestamp_ = timestamp;
    if (!std::isfinite(result.angularSpeedDegreesPerSecond)
        || result.angularSpeedDegreesPerSecond > configuration_.maximumGyroscopeDegreesPerSecond)
    {
        candidateStart_.reset();
        result.reason = StationaryReason::angularSpeedTooHigh;
        return result;
    }
    if (!acceleration.valid || !std::isfinite(result.accelerationDeviationG)
        || result.accelerationDeviationG > configuration_.maximumAccelerationDeviationG)
    {
        candidateStart_.reset();
        result.reason = StationaryReason::accelerationNormOutsideRange;
        return result;
    }
    if (!std::isfinite(confidence) || confidence < configuration_.minimumAccelerometerConfidence)
    {
        candidateStart_.reset();
        result.reason = StationaryReason::accelerometerConfidenceTooLow;
        return result;
    }
    if (!candidateStart_.has_value())
    {
        candidateStart_ = timestamp;
    }
    const auto duration = forwardDeviceTimestampDelta(timestamp, *candidateStart_).value_or(0U);
    result.sustainedDuration = std::chrono::nanoseconds(duration);
    result.stationary = std::chrono::duration<double>(result.sustainedDuration)
        >= configuration_.minimumDuration;
    result.reason = result.stationary ? StationaryReason::stationary : StationaryReason::sustaining;
    return result;
}

ExperimentPhase experimentPhaseAt(
    std::chrono::nanoseconds elapsed,
    const OrientationExperimentConfig& configuration) noexcept
{
    const double seconds = std::chrono::duration<double>(elapsed).count();
    if (seconds < configuration.stationaryBefore.count())
    {
        return ExperimentPhase::stationaryBefore;
    }
    if (seconds < configuration.stationaryBefore.count() + configuration.motion.count())
    {
        return ExperimentPhase::motion;
    }
    if (seconds < configuration.stationaryBefore.count() + configuration.motion.count()
        + configuration.stationaryAfter.count())
    {
        return ExperimentPhase::stationaryAfter;
    }
    return ExperimentPhase::complete;
}

std::string experimentPhaseText(ExperimentPhase phase)
{
    switch (phase)
    {
    case ExperimentPhase::calibration: return "calibration";
    case ExperimentPhase::startup: return "startup";
    case ExperimentPhase::stationaryBefore: return "stationary_before";
    case ExperimentPhase::motion: return "motion";
    case ExperimentPhase::stationaryAfter: return "stationary_after";
    case ExperimentPhase::complete: return "complete";
    }
    return "unknown";
}

std::string stationaryReasonText(StationaryReason reason)
{
    switch (reason)
    {
    case StationaryReason::stationary: return "stationary";
    case StationaryReason::invalidTimestamp: return "invalid_timestamp";
    case StationaryReason::angularSpeedTooHigh: return "angular_speed_too_high";
    case StationaryReason::accelerationNormOutsideRange: return "acceleration_norm_outside_range";
    case StationaryReason::accelerometerConfidenceTooLow: return "accelerometer_confidence_too_low";
    case StationaryReason::sustaining: return "sustaining";
    }
    return "unknown";
}

void DriftAccumulator::consume(
    const Quaternion& orientation,
    bool stationary,
    std::uint64_t timestamp) noexcept
{
    ++sampleCount_;
    if (!stationary || !orientation.normalized().has_value())
    {
        return;
    }
    ++stationaryCount_;
    if (!first_.has_value())
    {
        first_ = orientation;
        firstTimestamp_ = timestamp;
    }
    last_ = orientation;
    lastTimestamp_ = timestamp;
}

DriftMetrics DriftAccumulator::metrics() const noexcept
{
    DriftMetrics result;
    result.sampleCount = sampleCount_;
    result.stationarySampleCount = stationaryCount_;
    result.stationarySampleRatio = sampleCount_ == 0U ? 0.0
        : static_cast<double>(stationaryCount_) / static_cast<double>(sampleCount_);
    if (!first_.has_value() || !last_.has_value() || !firstTimestamp_.has_value()
        || !lastTimestamp_.has_value() || stationaryCount_ < 2U)
    {
        return result;
    }
    const auto difference = compareOrientations(*first_, *last_);
    const auto duration = forwardDeviceTimestampDelta(*lastTimestamp_, *firstTimestamp_);
    if (!difference.has_value() || !duration.has_value() || *duration == 0U)
    {
        return result;
    }
    result.available = true;
    result.durationSeconds = static_cast<double>(*duration) * 1.0e-9;
    result.totalDegrees = difference->total.degrees;
    result.tiltDegrees = difference->tilt.degrees;
    result.yawDegrees = difference->yawDegrees;
    result.pitchDegrees = difference->pitchDegrees;
    result.rollDegrees = difference->rollDegrees;
    result.totalDegreesPerSecond = result.totalDegrees / result.durationSeconds;
    result.tiltDegreesPerSecond = result.tiltDegrees / result.durationSeconds;
    result.yawDegreesPerSecond = result.yawDegrees / result.durationSeconds;
    result.pitchDegreesPerSecond = result.pitchDegrees / result.durationSeconds;
    result.rollDegreesPerSecond = result.rollDegrees / result.durationSeconds;
    return result;
}

TiltConvergenceTracker::TiltConvergenceTracker(
    Quaternion target,
    std::span<const double> thresholds,
    std::chrono::duration<double> sustain)
    : target_(std::move(target)), sustainDuration_(sustain)
{
    results_.reserve(thresholds.size());
    candidateStarts_.resize(thresholds.size());
    for (const double threshold : thresholds)
    {
        results_.push_back({threshold, std::nullopt});
    }
}

void TiltConvergenceTracker::consume(const Quaternion& orientation, std::uint64_t timestamp) noexcept
{
    const auto error = quaternionTiltDistance(target_, orientation);
    if (!error.has_value())
    {
        return;
    }
    if (!firstTimestamp_.has_value())
    {
        firstTimestamp_ = timestamp;
    }
    maximumError_ = std::max(maximumError_, error->degrees);
    sumError_ += error->degrees;
    sumSquaredError_ += error->degrees * error->degrees;
    ++count_;
    finalError_ = error->degrees;
    for (std::size_t index = 0; index < results_.size(); ++index)
    {
        if (results_[index].reachedAfterSeconds.has_value())
        {
            continue;
        }
        if (error->degrees <= results_[index].thresholdDegrees)
        {
            if (!candidateStarts_[index].has_value())
            {
                candidateStarts_[index] = timestamp;
            }
            const auto sustained = forwardDeviceTimestampDelta(timestamp, *candidateStarts_[index]);
            if (sustained.has_value()
                && std::chrono::duration<double>(std::chrono::nanoseconds(*sustained)) >= sustainDuration_)
            {
                if (*candidateStarts_[index] == *firstTimestamp_)
                {
                    results_[index].reachedAfterSeconds = 0.0;
                }
                else
                {
                    const auto elapsed = forwardDeviceTimestampDelta(
                        *candidateStarts_[index], *firstTimestamp_);
                    if (elapsed.has_value())
                    {
                        results_[index].reachedAfterSeconds = *elapsed * 1.0e-9;
                    }
                }
            }
        }
        else
        {
            candidateStarts_[index].reset();
        }
    }
}

const std::vector<ConvergenceThresholdResult>& TiltConvergenceTracker::results() const noexcept
{
    return results_;
}
double TiltConvergenceTracker::maximumErrorDegrees() const noexcept { return maximumError_; }
double TiltConvergenceTracker::meanErrorDegrees() const noexcept
{ return count_ == 0U ? 0.0 : sumError_ / static_cast<double>(count_); }
double TiltConvergenceTracker::rmsErrorDegrees() const noexcept
{ return count_ == 0U ? 0.0 : std::sqrt(sumSquaredError_ / static_cast<double>(count_)); }
std::optional<double> TiltConvergenceTracker::finalErrorDegrees() const noexcept { return finalError_; }

std::string orientationComparisonCsvHeader()
{
    return "device_timestamp_ns,elapsed_seconds,phase,sequence,stationary,stationary_reason,"
        "gyro_corrected_dps_x,gyro_corrected_dps_y,gyro_corrected_dps_z,"
        "gyro_angular_speed_norm_dps,accel_g_x,accel_g_y,accel_g_z,accel_norm_g,"
        "accel_confidence,correction_status,correction_reason,correction_angle_degrees,"
        "gyro_absolute_w,gyro_absolute_x,gyro_absolute_y,gyro_absolute_z,"
        "gyro_relative_w,gyro_relative_x,gyro_relative_y,gyro_relative_z,"
        "fused_absolute_w,fused_absolute_x,fused_absolute_y,fused_absolute_z,"
        "fused_relative_w,fused_relative_x,fused_relative_y,fused_relative_z,"
        "gyro_absolute_yaw_degrees,gyro_absolute_pitch_degrees,gyro_absolute_roll_degrees,"
        "gyro_relative_yaw_degrees,gyro_relative_pitch_degrees,gyro_relative_roll_degrees,"
        "fused_absolute_yaw_degrees,fused_absolute_pitch_degrees,fused_absolute_roll_degrees,"
        "fused_relative_yaw_degrees,fused_relative_pitch_degrees,fused_relative_roll_degrees,"
        "absolute_orientation_difference_degrees,relative_orientation_difference_degrees,"
        "tilt_difference_degrees";
}

std::string serializeOrientationComparisonCsvRow(const OrientationComparisonRecord& record)
{
    std::ostringstream output;
    output.imbue(std::locale::classic());
    output << std::setprecision(std::numeric_limits<double>::max_digits10)
           << record.deviceTimestampNanoseconds << ',' << record.elapsedSeconds << ','
           << quoteCsv(experimentPhaseText(record.phase)) << ','
           << static_cast<unsigned int>(record.sequence) << ','
           << (record.stationary.stationary ? "true" : "false") << ','
           << quoteCsv(stationaryReasonText(record.stationary.reason)) << ','
           << record.gyroscopeRadiansPerSecond.xRadiansPerSecond * radiansToDegrees << ','
           << record.gyroscopeRadiansPerSecond.yRadiansPerSecond * radiansToDegrees << ','
           << record.gyroscopeRadiansPerSecond.zRadiansPerSecond * radiansToDegrees << ','
           << record.stationary.angularSpeedDegreesPerSecond << ','
           << record.acceleration.accelerationG.x << ',' << record.acceleration.accelerationG.y << ','
           << record.acceleration.accelerationG.z << ',' << record.acceleration.normG << ','
           << record.accelerometerConfidence << ','
           << quoteCsv(record.correctionStatus == FusionCorrectionStatus::applied ? "applied"
               : record.correctionStatus == FusionCorrectionStatus::rejected ? "rejected" : "skipped")
           << ',' << quoteCsv(fusionReasonText(record.correctionReason))
           << ',' << record.correctionAngleDegrees << ',';
    appendQuaternion(output, record.orientations.gyroAbsolute.value); output << ',';
    appendQuaternion(output, record.orientations.gyroRelative.value); output << ',';
    appendQuaternion(output, record.orientations.fusedAbsolute.value); output << ',';
    appendQuaternion(output, record.orientations.fusedRelative.value); output << ',';
    appendEuler(output, record.orientations.gyroAbsoluteEuler); output << ',';
    appendEuler(output, record.orientations.gyroRelativeEuler); output << ',';
    appendEuler(output, record.orientations.fusedAbsoluteEuler); output << ',';
    appendEuler(output, record.orientations.fusedRelativeEuler); output << ',';
    output << (record.absoluteDifference ? record.absoluteDifference->total.degrees : 0.0) << ','
           << (record.relativeDifference ? record.relativeDifference->total.degrees : 0.0) << ','
           << (record.absoluteDifference ? record.absoluteDifference->tilt.degrees : 0.0);
    return output.str();
}

std::string serializeOrientationComparisonJson(const OrientationComparisonSummary& summary)
{
    std::ostringstream output;
    output.imbue(std::locale::classic());
    output << std::setprecision(std::numeric_limits<double>::max_digits10)
           << "{\"schema_version\":1,\"type\":\"gyro-vs-fusion-orientation-comparison\","
           << "\"experimental\":true,\"configuration\":{"
           << "\"stationary_before_seconds\":" << summary.experiment.stationaryBefore.count()
           << ",\"motion_seconds\":" << summary.experiment.motion.count()
           << ",\"stationary_after_seconds\":" << summary.experiment.stationaryAfter.count()
           << ",\"recenter_seconds\":" << summary.experiment.recenterAt.count()
           << ",\"stationary_gyro_threshold_dps\":"
           << summary.stationary.maximumGyroscopeDegreesPerSecond
           << ",\"stationary_accel_deviation_g\":"
           << summary.stationary.maximumAccelerationDeviationG
           << ",\"stationary_min_duration_seconds\":"
           << summary.stationary.minimumDuration.count() << "},\"quaternion_convention\":{"
           << "\"component_order\":\"wxyz\",\"frame\":\"body-to-world\","
           << "\"handedness\":\"right-handed\",\"relative_formula\":"
           << "\"inverse(recenter_reference)*absolute\"},\"profiles\":{"
           << "\"gyroscope_source\":\"" << escapeJson(summary.gyroscopeProfileSource)
           << "\",\"accelerometer_source\":\""
           << escapeJson(summary.accelerometerProfileSource) << "\"},\"recenter\":{"
           << "\"gyro_active\":" << (summary.finalOrientations.gyroRecenterReference.active ? "true" : "false")
           << ",\"fused_active\":" << (summary.finalOrientations.fusedRecenterReference.active ? "true" : "false")
           << ",\"gyro_reference_quaternion_wxyz\":";
    appendJsonQuaternion(output, summary.finalOrientations.gyroRecenterReference.value);
    output << ",\"fused_reference_quaternion_wxyz\":";
    appendJsonQuaternion(output, summary.finalOrientations.fusedRecenterReference.value);
    const auto appendTimeRange = [&output](const PhaseTimeRange& time) {
        output << "{\"start_device_timestamp_ns\":";
        if (time.startDeviceTimestampNanoseconds)
        {
            output << *time.startDeviceTimestampNanoseconds;
        }
        else
        {
            output << "null";
        }
        output << ",\"end_device_timestamp_ns\":";
        if (time.endDeviceTimestampNanoseconds)
        {
            output << *time.endDeviceTimestampNanoseconds;
        }
        else
        {
            output << "null";
        }
        output << '}';
    };
    output << "},\"phases\":{\"stationary_before\":{\"time\":";
    appendTimeRange(summary.stationaryBeforeTime);
    output << ",\"gyro_absolute\":";
    appendJsonMetric(output, summary.stationaryBefore.gyroAbsolute);
    output << ",\"gyro_relative\":"; appendJsonMetric(output, summary.stationaryBefore.gyroRelative);
    output << ",\"fused_absolute\":"; appendJsonMetric(output, summary.stationaryBefore.fusedAbsolute);
    output << ",\"fused_relative\":"; appendJsonMetric(output, summary.stationaryBefore.fusedRelative);
    output << "},\"motion\":{\"time\":";
    appendTimeRange(summary.motionTime);
    output << "},\"stationary_after\":{\"time\":";
    appendTimeRange(summary.stationaryAfterTime);
    output << ",\"gyro_absolute\":";
    appendJsonMetric(output, summary.stationaryAfter.gyroAbsolute);
    output << ",\"gyro_relative\":"; appendJsonMetric(output, summary.stationaryAfter.gyroRelative);
    output << ",\"fused_absolute\":"; appendJsonMetric(output, summary.stationaryAfter.fusedAbsolute);
    output << ",\"fused_relative\":"; appendJsonMetric(output, summary.stationaryAfter.fusedRelative);
    output << ",\"convergence\":{\"gyro\":[";
    for (std::size_t index = 0; index < summary.gyroConvergence.size(); ++index)
    {
        if (index != 0U)
        {
            output << ',';
        }
        const auto& value = summary.gyroConvergence[index];
        output << "{\"threshold_degrees\":" << value.thresholdDegrees << ",\"reached_after_seconds\":";
        if (value.reachedAfterSeconds)
        {
            output << *value.reachedAfterSeconds;
        }
        else
        {
            output << "null";
        }
        output << '}';
    }
    output << "],\"fused\":[";
    for (std::size_t index = 0; index < summary.fusedConvergence.size(); ++index)
    {
        if (index != 0U)
        {
            output << ',';
        }
        const auto& value = summary.fusedConvergence[index];
        output << "{\"threshold_degrees\":" << value.thresholdDegrees << ",\"reached_after_seconds\":";
        if (value.reachedAfterSeconds)
        {
            output << *value.reachedAfterSeconds;
        }
        else
        {
            output << "null";
        }
        output << '}';
    }
    output << "],\"gyro_recovery\":{\"available\":"
           << (summary.gyroRecovery.available ? "true" : "false")
           << ",\"maximum_tilt_error_degrees\":" << summary.gyroRecovery.maximumErrorDegrees
           << ",\"mean_tilt_error_degrees\":" << summary.gyroRecovery.meanErrorDegrees
           << ",\"rms_tilt_error_degrees\":" << summary.gyroRecovery.rmsErrorDegrees
           << ",\"final_tilt_error_degrees\":" << summary.gyroRecovery.finalErrorDegrees
           << "},\"fused_recovery\":{\"available\":"
           << (summary.fusedRecovery.available ? "true" : "false")
           << ",\"maximum_tilt_error_degrees\":" << summary.fusedRecovery.maximumErrorDegrees
           << ",\"mean_tilt_error_degrees\":" << summary.fusedRecovery.meanErrorDegrees
           << ",\"rms_tilt_error_degrees\":" << summary.fusedRecovery.rmsErrorDegrees
           << ",\"final_tilt_error_degrees\":" << summary.fusedRecovery.finalErrorDegrees
           << "}}}},\"final\":{\"gyro_absolute_quaternion_wxyz\":";
    appendJsonQuaternion(output, summary.finalOrientations.gyroAbsolute.value);
    output << ",\"gyro_relative_quaternion_wxyz\":";
    appendJsonQuaternion(output, summary.finalOrientations.gyroRelative.value);
    output << ",\"fused_absolute_quaternion_wxyz\":";
    appendJsonQuaternion(output, summary.finalOrientations.fusedAbsolute.value);
    output << ",\"fused_relative_quaternion_wxyz\":";
    appendJsonQuaternion(output, summary.finalOrientations.fusedRelative.value);
    output << ",\"absolute_difference_degrees\":"
           << (summary.finalAbsoluteDifference ? summary.finalAbsoluteDifference->total.degrees : 0.0)
           << ",\"relative_difference_degrees\":"
           << (summary.finalRelativeDifference ? summary.finalRelativeDifference->total.degrees : 0.0)
           << ",\"tilt_difference_degrees\":"
           << (summary.finalAbsoluteDifference ? summary.finalAbsoluteDifference->tilt.degrees : 0.0)
           << "},\"statistics\":{\"received_packets\":" << summary.receivedPackets
           << ",\"gyro_samples_applied\":" << summary.gyroscopeSamplesApplied
           << ",\"accelerometer_corrections_applied\":" << summary.accelerometerCorrectionsApplied
           << ",\"accelerometer_corrections_skipped\":" << summary.accelerometerCorrectionsSkipped
           << ",\"rejected_samples\":" << summary.rejectedSamples << "}}\n";
    return output.str();
}

} // namespace xreal::sensors
