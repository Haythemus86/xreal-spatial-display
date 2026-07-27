#include "sensors/OrientationFusion.hpp"

#include "sensors/DeviceTimestampDelta.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <numbers>
#include <sstream>
#include <string_view>
#include <utility>

namespace xreal::sensors
{
namespace
{

[[nodiscard]] std::string escapeJson(std::string_view value)
{
    std::string escaped;
    escaped.reserve(value.size());
    for (const char character : value)
    {
        switch (character)
        {
        case '\\': escaped += "\\\\"; break;
        case '"': escaped += "\\\""; break;
        case '\n': escaped += "\\n"; break;
        case '\r': escaped += "\\r"; break;
        case '\t': escaped += "\\t"; break;
        default: escaped += character; break;
        }
    }
    return escaped;
}

[[nodiscard]] PhysicalVector3d normalized(const PhysicalVector3d& value) noexcept
{
    const double length = value.norm();
    if (!value.finite() || !std::isfinite(length)
        || length <= std::numeric_limits<double>::epsilon())
    {
        return {};
    }
    return {value.x / length, value.y / length, value.z / length};
}

[[nodiscard]] PhysicalVector3d cross(
    const PhysicalVector3d& left,
    const PhysicalVector3d& right) noexcept
{
    return {
        left.y * right.z - left.z * right.y,
        left.z * right.x - left.x * right.z,
        left.x * right.y - left.y * right.x,
    };
}

[[nodiscard]] double dot(const PhysicalVector3d& left, const PhysicalVector3d& right) noexcept
{
    return left.x * right.x + left.y * right.y + left.z * right.z;
}

[[nodiscard]] Quaternion axisAngle(const PhysicalVector3d& axis, double angle) noexcept
{
    const double halfAngle = angle * 0.5;
    const double sine = std::sin(halfAngle);
    return {std::cos(halfAngle), axis.x * sine, axis.y * sine, axis.z * sine};
}

[[nodiscard]] Quaternion eulerZeroYaw(double pitch, double roll) noexcept
{
    const double halfPitch = pitch * 0.5;
    const double halfRoll = roll * 0.5;
    return {
        std::cos(halfPitch) * std::cos(halfRoll),
        std::cos(halfPitch) * std::sin(halfRoll),
        std::sin(halfPitch) * std::cos(halfRoll),
        -std::sin(halfPitch) * std::sin(halfRoll),
    };
}

[[nodiscard]] FusionReason orientationReason(OrientationRejectionReason reason) noexcept
{
    switch (reason)
    {
    case OrientationRejectionReason::none: return FusionReason::none;
    case OrientationRejectionReason::firstTimestamp: return FusionReason::firstTimestamp;
    case OrientationRejectionReason::duplicateTimestamp: return FusionReason::duplicateTimestamp;
    case OrientationRejectionReason::decreasingTimestamp: return FusionReason::decreasingTimestamp;
    case OrientationRejectionReason::excessiveTimestampDelta: return FusionReason::excessiveTimestampDelta;
    case OrientationRejectionReason::invalidAngularVelocity:
    case OrientationRejectionReason::invalidAxisMapping: return FusionReason::invalidGyroscope;
    case OrientationRejectionReason::invalidOrientation: return FusionReason::invalidQuaternion;
    }
    return FusionReason::invalidQuaternion;
}

} // namespace

AccelerometerConfidenceEstimator::AccelerometerConfidenceEstimator(
    GravityConfidenceConfig configuration)
    : configuration_(std::move(configuration))
{
}

void AccelerometerConfidenceEstimator::reset() noexcept
{
    previousTimestamp_.reset();
    previousAccelerationG_.reset();
    smoothedConfidence_ = 0.0;
    initialized_ = false;
    hysteresisActive_ = false;
}

AccelerometerConfidence AccelerometerConfidenceEstimator::update(
    const AccelerometerPhysicalSample& sample,
    std::uint64_t timestamp) noexcept
{
    AccelerometerConfidence result;
    result.normG = sample.normG;
    if (!sample.valid || !sample.accelerationG.finite() || !std::isfinite(sample.normG))
    {
        result.reason = FusionReason::invalidAccelerometer;
        return result;
    }
    if (sample.normG < configuration_.minimumAccelerationNormG)
    {
        result.reason = FusionReason::accelerationNearZero;
        return result;
    }
    result.deviationG = std::abs(sample.normG - 1.0);
    if (result.deviationG <= configuration_.fullConfidenceDeviationG)
    {
        result.rawConfidence = 1.0;
    }
    else if (result.deviationG < configuration_.zeroConfidenceDeviationG)
    {
        const double span = configuration_.zeroConfidenceDeviationG
            - configuration_.fullConfidenceDeviationG;
        result.rawConfidence = 1.0
            - (result.deviationG - configuration_.fullConfidenceDeviationG) / span;
    }
    else
    {
        result.reason = FusionReason::accelerationNormOutsideRange;
    }

    std::optional<double> deltaSeconds;
    if (previousTimestamp_.has_value())
    {
        const auto delta = forwardDeviceTimestampDelta(timestamp, *previousTimestamp_);
        if (delta.has_value() && *delta > 0U)
        {
            deltaSeconds = static_cast<double>(*delta) * 1.0e-9;
        }
    }
    if (configuration_.maximumJerkGPerSecond.has_value() && deltaSeconds.has_value()
        && previousAccelerationG_.has_value())
    {
        const PhysicalVector3d difference{
            sample.accelerationG.x - previousAccelerationG_->x,
            sample.accelerationG.y - previousAccelerationG_->y,
            sample.accelerationG.z - previousAccelerationG_->z,
        };
        result.jerkGPerSecond = difference.norm() / *deltaSeconds;
        if (*result.jerkGPerSecond > *configuration_.maximumJerkGPerSecond)
        {
            result.rawConfidence = 0.0;
            result.reason = FusionReason::jerkExceeded;
        }
    }

    result.rawConfidence = std::clamp(result.rawConfidence, 0.0, 1.0);
    if (!initialized_ || configuration_.smoothingTimeConstant.count() <= 0.0
        || !deltaSeconds.has_value())
    {
        smoothedConfidence_ = result.rawConfidence;
        initialized_ = true;
    }
    else
    {
        const double alpha = 1.0 - std::exp(
            -*deltaSeconds / configuration_.smoothingTimeConstant.count());
        smoothedConfidence_ += alpha * (result.rawConfidence - smoothedConfidence_);
    }
    smoothedConfidence_ = std::clamp(smoothedConfidence_, 0.0, 1.0);
    result.smoothedConfidence = smoothedConfidence_;
    if (configuration_.hysteresisEnabled)
    {
        if (!hysteresisActive_ && smoothedConfidence_ >= configuration_.hysteresisEnterConfidence)
        {
            hysteresisActive_ = true;
        }
        else if (hysteresisActive_
                 && smoothedConfidence_ <= configuration_.hysteresisExitConfidence)
        {
            hysteresisActive_ = false;
        }
        if (!hysteresisActive_)
        {
            result.reason = FusionReason::confidenceHysteresis;
        }
    }
    else
    {
        hysteresisActive_ = true;
    }
    result.correctionConfidence = hysteresisActive_
        ? std::min(result.rawConfidence, smoothedConfidence_) : 0.0;
    result.valid = result.correctionConfidence > 0.0;
    previousTimestamp_ = timestamp;
    previousAccelerationG_ = sample.accelerationG;
    return result;
}

PhysicalVector3d predictBodyFrameAccelerometerUp(const Quaternion& orientation) noexcept
{
    const auto inverse = orientation.inverseNormalized();
    if (!inverse.has_value())
    {
        return {};
    }
    const Quaternion worldUp{0.0, 0.0, 0.0, 1.0};
    const Quaternion bodyUp = *inverse * worldUp * orientation.normalized().value();
    return normalized({bodyUp.x, bodyUp.y, bodyUp.z});
}

std::optional<Quaternion> gravityAlignedStartupOrientation(
    const PhysicalVector3d& measuredBodyUp) noexcept
{
    const PhysicalVector3d up = normalized(measuredBodyUp);
    if (!up.finite() || up.norm() < 0.5)
    {
        return std::nullopt;
    }
    const double pitch = -std::asin(std::clamp(up.x, -1.0, 1.0));
    const double roll = std::atan2(up.y, up.z);
    return eulerZeroYaw(pitch, roll).normalized();
}

OrientationFusionFilter::OrientationFusionFilter(OrientationFusionConfig configuration)
    : configuration_(std::move(configuration)),
      gyroscopeIntegrator_(OrientationIntegratorConfig{
          configuration_.maximumDeviceTimestampDelta,
          1.0e-8,
          configuration_.gyroscopeAxisMapping}),
      confidenceEstimator_(configuration_.confidence)
{
    reset();
}

void OrientationFusionFilter::reset() noexcept
{
    gyroscopeIntegrator_.reset();
    confidenceEstimator_.reset();
    state_ = {};
    state_.fusedOrientation = Quaternion::identity();
    state_.gyroscopePredictedOrientation = Quaternion::identity();
    state_.recenterReference = Quaternion::identity();
    state_.valid = true;
    state_.startupStatus = configuration_.startupMode == FusionStartupMode::gravity
        ? FusionStartupStatus::waitingForConfidence
        : FusionStartupStatus::identity;
    state_.experimental = configuration_.experimental;
    state_.verified = configuration_.verified;
}

bool OrientationFusionFilter::setOrientation(const Quaternion& orientationValue) noexcept
{
    const auto normalizedValue = orientationValue.normalized();
    if (!normalizedValue.has_value() || !gyroscopeIntegrator_.setOrientation(*normalizedValue))
    {
        return false;
    }
    state_.fusedOrientation = *normalizedValue;
    state_.gyroscopePredictedOrientation = *normalizedValue;
    state_.valid = true;
    return true;
}

bool OrientationFusionFilter::recenter() noexcept
{
    const auto normalizedValue = state_.fusedOrientation.normalized();
    if (!normalizedValue.has_value())
    {
        return false;
    }
    state_.recenterReference = *normalizedValue;
    return true;
}

void OrientationFusionFilter::clearRecenter() noexcept
{
    state_.recenterReference = Quaternion::identity();
}

Quaternion OrientationFusionFilter::orientation() const noexcept
{
    return state_.fusedOrientation;
}

Quaternion OrientationFusionFilter::relativeOrientation() const noexcept
{
    const auto inverse = state_.recenterReference.inverseNormalized();
    if (!inverse.has_value())
    {
        return state_.fusedOrientation;
    }
    return (*inverse * state_.fusedOrientation).normalized().value_or(state_.fusedOrientation);
}

const OrientationFusionState& OrientationFusionFilter::state() const noexcept
{
    return state_;
}

const OrientationFusionConfig& OrientationFusionFilter::configuration() const noexcept
{
    return configuration_;
}

OrientationFusionResult OrientationFusionFilter::makeResult(
    OrientationSampleStatus gyroStatus,
    FusionCorrectionStatus correctionStatus,
    FusionReason reason,
    const GravityObservation& gravity,
    std::chrono::nanoseconds delta,
    double correctionAngle) const noexcept
{
    return {gyroStatus, correctionStatus, reason, state_.gyroscopePredictedOrientation,
            state_.fusedOrientation, relativeOrientation(), gravity, delta, correctionAngle};
}

OrientationFusionResult OrientationFusionFilter::update(
    const AngularVelocityRadians& angularVelocity,
    const AccelerometerPhysicalSample& acceleration,
    std::uint64_t timestamp) noexcept
{
    GravityObservation gravity;
    gravity.confidence = confidenceEstimator_.update(acceleration, timestamp);
    if (acceleration.valid && acceleration.normG > 0.0)
    {
        gravity.measuredBodyUp = normalized(acceleration.accelerationG);
    }

    const auto gyroResult = gyroscopeIntegrator_.update(angularVelocity, timestamp);
    state_.lastDeviceTimestamp = timestamp;
    if (gyroResult.status == OrientationSampleStatus::rejected)
    {
        ++state_.rejectedSampleCount;
        state_.lastReason = orientationReason(gyroResult.reason);
        return makeResult(gyroResult.status, FusionCorrectionStatus::rejected,
                          state_.lastReason, gravity, gyroResult.deltaTime, 0.0);
    }

    if (!state_.initialized)
    {
        if (configuration_.startupMode == FusionStartupMode::gravity)
        {
            if (gravity.confidence.correctionConfidence <= 0.0)
            {
                state_.lastReason = FusionReason::startupWaitingForGravity;
                state_.startupStatus = FusionStartupStatus::waitingForConfidence;
                return makeResult(gyroResult.status, FusionCorrectionStatus::skipped,
                                  state_.lastReason, gravity, gyroResult.deltaTime, 0.0);
            }
            const auto startup = gravityAlignedStartupOrientation(gravity.measuredBodyUp);
            if (!startup.has_value() || !setOrientation(*startup))
            {
                ++state_.rejectedSampleCount;
                state_.lastReason = FusionReason::invalidQuaternion;
                state_.startupStatus = FusionStartupStatus::rejected;
                return makeResult(gyroResult.status, FusionCorrectionStatus::rejected,
                                  state_.lastReason, gravity, gyroResult.deltaTime, 0.0);
            }
            state_.gravityStartupApplied = true;
            state_.startupStatus = FusionStartupStatus::gravityAligned;
        }
        state_.initialized = true;
    }

    if (gyroResult.status != OrientationSampleStatus::applied)
    {
        state_.lastReason = orientationReason(gyroResult.reason);
        return makeResult(gyroResult.status, FusionCorrectionStatus::skipped,
                          state_.lastReason, gravity, gyroResult.deltaTime, 0.0);
    }

    ++state_.appliedGyroscopeSampleCount;
    state_.integrationDuration += gyroResult.deltaTime;
    state_.fusionDuration += gyroResult.deltaTime;
    state_.gyroscopePredictedOrientation = gyroResult.absoluteOrientation;
    state_.fusedOrientation = gyroResult.absoluteOrientation;
    state_.currentAccelerationNormG = gravity.confidence.normG;
    state_.currentAccelerometerConfidence = gravity.confidence.correctionConfidence;
    state_.currentCorrectionAngleRadians = 0.0;
    gravity.predictedBodyUp = predictBodyFrameAccelerometerUp(state_.fusedOrientation);

    if (!configuration_.enabled || !configuration_.accelerometerCorrectionEnabled)
    {
        ++state_.skippedAccelerometerCorrectionCount;
        state_.lastReason = FusionReason::correctionDisabled;
        return makeResult(gyroResult.status, FusionCorrectionStatus::skipped,
                          state_.lastReason, gravity, gyroResult.deltaTime, 0.0);
    }
    if (gravity.confidence.correctionConfidence <= 0.0)
    {
        ++state_.skippedAccelerometerCorrectionCount;
        state_.lastReason = gravity.confidence.reason;
        return makeResult(gyroResult.status, FusionCorrectionStatus::skipped,
                          state_.lastReason, gravity, gyroResult.deltaTime, 0.0);
    }

    const PhysicalVector3d correctionCross = cross(
        gravity.measuredBodyUp, gravity.predictedBodyUp);
    const double crossLength = correctionCross.norm();
    const double gravityDot = std::clamp(
        dot(gravity.measuredBodyUp, gravity.predictedBodyUp), -1.0, 1.0);
    if (crossLength <= 1.0e-12)
    {
        ++state_.skippedAccelerometerCorrectionCount;
        state_.lastReason = gravityDot < 0.0 ? FusionReason::antiParallelGravity : FusionReason::none;
        return makeResult(gyroResult.status, FusionCorrectionStatus::skipped,
                          state_.lastReason, gravity, gyroResult.deltaTime, 0.0);
    }
    const double errorAngle = std::atan2(crossLength, gravityDot);
    const double seconds = std::chrono::duration<double>(gyroResult.deltaTime).count();
    const double alpha = configuration_.correctionTimeConstant.count() > 0.0
        ? 1.0 - std::exp(-seconds / configuration_.correctionTimeConstant.count()) : 1.0;
    double correctionAngle = errorAngle * alpha * configuration_.correctionGain
        * gravity.confidence.correctionConfidence;
    const double maximumCorrection = configuration_.maximumCorrectionDegreesPerSecond
        * std::numbers::pi / 180.0 * seconds;
    correctionAngle = std::clamp(correctionAngle, 0.0, maximumCorrection);
    const PhysicalVector3d axis{
        correctionCross.x / crossLength,
        correctionCross.y / crossLength,
        correctionCross.z / crossLength,
    };
    const auto corrected = (state_.fusedOrientation * axisAngle(axis, correctionAngle)).normalized();
    if (!corrected.has_value() || !gyroscopeIntegrator_.setOrientation(*corrected))
    {
        ++state_.rejectedSampleCount;
        state_.lastReason = FusionReason::invalidQuaternion;
        return makeResult(gyroResult.status, FusionCorrectionStatus::rejected,
                          state_.lastReason, gravity, gyroResult.deltaTime, 0.0);
    }
    state_.fusedOrientation = *corrected;
    state_.currentCorrectionAngleRadians = correctionAngle;
    state_.accumulatedCorrectionAngleRadians += correctionAngle;
    ++state_.appliedAccelerometerCorrectionCount;
    state_.lastReason = FusionReason::none;
    return makeResult(gyroResult.status, FusionCorrectionStatus::applied,
                      FusionReason::none, gravity, gyroResult.deltaTime, correctionAngle);
}

std::string serializeOrientationFusionJson(
    const OrientationFusionFilter& filter,
    const GyroscopeScaleProfile& gyroScale,
    const AccelerometerCalibrationProfile& accelProfile,
    bool includeEuler)
{
    const auto& state = filter.state();
    const auto& config = filter.configuration();
    const Quaternion fused = filter.relativeOrientation();
    const auto euler = quaternionToEulerDiagnostic(fused);
    constexpr double radiansToDegrees = 180.0 / std::numbers::pi;
    std::ostringstream output;
    output << std::setprecision(std::numeric_limits<double>::max_digits10)
           << "{\n  \"schema_version\":1,\n  \"orientation\":{"
           << "\"type\":\"gyro-accelerometer-complementary\",\"experimental\":"
           << (state.experimental ? "true" : "false")
           << ",\"verified\":" << (state.verified ? "true" : "false") << ','
           << "\"valid\":" << (state.valid ? "true" : "false")
           << ",\"quaternion_convention\":{\"component_order\":\"wxyz\","
           << "\"frame\":\"body-to-world\",\"handedness\":\"right-handed\","
           << "\"multiplication_order\":\"current_times_body_delta\"},"
           << "\"gyro_scale_profile\":{\"source\":\"" << escapeJson(gyroScale.source)
           << "\",\"experimental\":" << (gyroScale.experimental ? "true" : "false")
           << ",\"verified\":" << (gyroScale.verified ? "true" : "false") << "},"
           << "\"accelerometer_profile\":{\"source\":\"" << escapeJson(accelProfile.source)
           << "\",\"experimental\":" << (accelProfile.experimental ? "true" : "false")
           << ",\"verified\":" << (accelProfile.verified ? "true" : "false") << "},"
           << "\"axis_mapping\":{\"gyro_source\":\""
           << escapeJson(config.gyroscopeAxisMapping.source)
           << "\",\"accelerometer_source\":\""
           << escapeJson(accelProfile.axisMapping.source) << "\"},\"configuration\":{"
           << "\"correction_time_constant_seconds\":" << config.correctionTimeConstant.count()
           << ",\"maximum_correction_degrees_per_second\":"
           << config.maximumCorrectionDegreesPerSecond << ",\"startup_mode\":\""
           << (config.startupMode == FusionStartupMode::gravity ? "gravity" : "identity")
           << "\",\"startup_status\":\""
           << (state.startupStatus == FusionStartupStatus::gravityAligned
                   ? "gravity-aligned"
                   : state.startupStatus == FusionStartupStatus::waitingForConfidence
                       ? "waiting-for-confidence"
                       : state.startupStatus == FusionStartupStatus::rejected
                           ? "rejected" : "identity") << "\"},"
           << "\"final_gyro_prediction\":{\"w\":" << state.gyroscopePredictedOrientation.w
           << ",\"x\":" << state.gyroscopePredictedOrientation.x << ",\"y\":"
           << state.gyroscopePredictedOrientation.y << ",\"z\":"
           << state.gyroscopePredictedOrientation.z << "},\"final_fused_quaternion\":{"
           << "\"w\":" << fused.w << ",\"x\":" << fused.x << ",\"y\":"
           << fused.y << ",\"z\":" << fused.z << '}';
    if (includeEuler)
    {
        output << ",\"final_euler_degrees\":{\"yaw\":" << euler.yawDegrees
               << ",\"pitch\":" << euler.pitchDegrees << ",\"roll\":"
               << euler.rollDegrees << '}';
    }
    output << ",\"accelerometer\":{\"final_norm_g\":" << state.currentAccelerationNormG
           << ",\"final_confidence\":" << state.currentAccelerometerConfidence
           << "},\"correction\":{\"final_angle_degrees\":"
           << state.currentCorrectionAngleRadians * radiansToDegrees
           << ",\"accumulated_angle_degrees\":"
           << state.accumulatedCorrectionAngleRadians * radiansToDegrees
           << "},\"statistics\":{\"gyro_samples_applied\":"
           << state.appliedGyroscopeSampleCount << ",\"accelerometer_corrections_applied\":"
           << state.appliedAccelerometerCorrectionCount
           << ",\"accelerometer_corrections_skipped\":"
           << state.skippedAccelerometerCorrectionCount << ",\"samples_rejected\":"
           << state.rejectedSampleCount << ",\"duration_seconds\":"
           << std::chrono::duration<double>(state.fusionDuration).count() << "}}\n}\n";
    return output.str();
}

std::string fusionReasonText(FusionReason reason)
{
    switch (reason)
    {
    case FusionReason::none: return "none";
    case FusionReason::firstTimestamp: return "first timestamp";
    case FusionReason::duplicateTimestamp: return "duplicate timestamp";
    case FusionReason::decreasingTimestamp: return "decreasing timestamp";
    case FusionReason::excessiveTimestampDelta: return "excessive timestamp delta";
    case FusionReason::invalidGyroscope: return "invalid gyroscope";
    case FusionReason::invalidAccelerometer: return "invalid accelerometer";
    case FusionReason::accelerationNearZero: return "acceleration near zero";
    case FusionReason::accelerationNormOutsideRange: return "acceleration norm outside range";
    case FusionReason::jerkExceeded: return "jerk threshold exceeded";
    case FusionReason::confidenceHysteresis: return "confidence hysteresis inactive";
    case FusionReason::startupWaitingForGravity: return "startup waiting for gravity";
    case FusionReason::antiParallelGravity: return "anti-parallel gravity ambiguity";
    case FusionReason::correctionDisabled: return "accelerometer correction disabled";
    case FusionReason::invalidQuaternion: return "invalid quaternion";
    }
    return "unknown";
}

} // namespace xreal::sensors
