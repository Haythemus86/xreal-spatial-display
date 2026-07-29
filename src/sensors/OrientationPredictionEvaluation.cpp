#include "sensors/OrientationPredictionEvaluation.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
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
} // namespace

DelayedOrientationEvaluator::DelayedOrientationEvaluator(
    DelayedOrientationEvaluatorConfig configuration)
    : configuration_(std::move(configuration))
{
}

void DelayedOrientationEvaluator::enqueue(
    std::uint64_t source,
    std::chrono::nanoseconds horizon,
    const AbsoluteOrientation& measured,
    const PredictedAbsoluteOrientation& predicted,
    std::uint64_t recenterGeneration)
{
    if (pending_.size() >= configuration_.maximumPendingPredictions)
    {
        pending_.pop_front();
        ++statistics_.expired;
        ++statistics_.unmatched;
    }
    const auto horizonCount = static_cast<std::uint64_t>(std::max<std::int64_t>(0, horizon.count()));
    const std::uint64_t target = source > std::numeric_limits<std::uint64_t>::max() - horizonCount
        ? std::numeric_limits<std::uint64_t>::max() : source + horizonCount;
    pending_.push_back({source, target, measured, predicted, recenterGeneration});
    ++statistics_.queued;
}

std::optional<OrientationPredictionEvaluation> DelayedOrientationEvaluator::consumeMeasured(
    std::uint64_t timestamp,
    const AbsoluteOrientation& measured,
    std::uint64_t recenterGeneration) noexcept
{
    std::optional<OrientationPredictionEvaluation> latest;
    const auto tolerance = static_cast<std::uint64_t>(
        std::max<std::int64_t>(0, configuration_.tolerance.count()));
    while (!pending_.empty())
    {
        const Pending& candidate = pending_.front();
        // Do not consume a measurement merely because it is inside the early
        // side of the tolerance window. A closer (or exact) target-time sample
        // may still arrive. The first sample at or after the target is used.
        if (timestamp < candidate.target)
        {
            break;
        }
        if (candidate.recenterGeneration != recenterGeneration)
        {
            ++statistics_.incompatibleRecenter;
            ++statistics_.unmatched;
            pending_.pop_front();
            continue;
        }
        const std::uint64_t difference = timestamp - candidate.target;
        if (difference > tolerance)
        {
            ++statistics_.expired;
            ++statistics_.unmatched;
            pending_.pop_front();
            continue;
        }
        const auto predictionTotal = quaternionAngularDistance(candidate.predicted.value, measured.value);
        const auto predictionTilt = quaternionTiltDistance(candidate.predicted.value, measured.value);
        const auto baselineTotal = quaternionAngularDistance(candidate.measured.value, measured.value);
        const auto baselineTilt = quaternionTiltDistance(candidate.measured.value, measured.value);
        if (predictionTotal && predictionTilt && baselineTotal && baselineTilt)
        {
            OrientationPredictionEvaluation value;
            value.sourceDeviceTimestampNanoseconds = candidate.source;
            value.targetDeviceTimestampNanoseconds = candidate.target;
            value.matchedDeviceTimestampNanoseconds = timestamp;
            value.targetTimestampErrorNanoseconds = timestamp >= candidate.target
                ? static_cast<std::int64_t>(timestamp - candidate.target)
                : -static_cast<std::int64_t>(candidate.target - timestamp);
            value.predictionTotalErrorDegrees = predictionTotal->degrees;
            value.predictionTiltErrorDegrees = predictionTilt->degrees;
            value.baselineTotalErrorDegrees = baselineTotal->degrees;
            value.baselineTiltErrorDegrees = baselineTilt->degrees;
            value.totalImprovementDegrees = baselineTotal->degrees - predictionTotal->degrees;
            value.tiltImprovementDegrees = baselineTilt->degrees - predictionTilt->degrees;
            if (baselineTotal->degrees > std::numeric_limits<double>::epsilon())
            {
                value.totalImprovementPercent = value.totalImprovementDegrees
                    * 100.0 / baselineTotal->degrees;
            }
            predictionTotalSum_ += value.predictionTotalErrorDegrees;
            predictionTotalSquaredSum_ += value.predictionTotalErrorDegrees * value.predictionTotalErrorDegrees;
            predictionTiltSum_ += value.predictionTiltErrorDegrees;
            predictionTiltSquaredSum_ += value.predictionTiltErrorDegrees * value.predictionTiltErrorDegrees;
            baselineTotalSum_ += value.baselineTotalErrorDegrees;
            baselineTotalSquaredSum_ += value.baselineTotalErrorDegrees
                * value.baselineTotalErrorDegrees;
            baselineTiltSum_ += value.baselineTiltErrorDegrees;
            baselineTiltSquaredSum_ += value.baselineTiltErrorDegrees
                * value.baselineTiltErrorDegrees;
            if (value.totalImprovementDegrees > 0.0)
            {
                ++statistics_.improved;
            }
            ++statistics_.matched;
            statistics_.maximumPredictionTotalErrorDegrees = std::max(
                statistics_.maximumPredictionTotalErrorDegrees, value.predictionTotalErrorDegrees);
            statistics_.maximumPredictionTiltErrorDegrees = std::max(
                statistics_.maximumPredictionTiltErrorDegrees, value.predictionTiltErrorDegrees);
            const double count = static_cast<double>(statistics_.matched);
            statistics_.meanPredictionTotalErrorDegrees = predictionTotalSum_ / count;
            statistics_.rmsPredictionTotalErrorDegrees = std::sqrt(predictionTotalSquaredSum_ / count);
            statistics_.meanPredictionTiltErrorDegrees = predictionTiltSum_ / count;
            statistics_.rmsPredictionTiltErrorDegrees = std::sqrt(predictionTiltSquaredSum_ / count);
            statistics_.meanBaselineTotalErrorDegrees = baselineTotalSum_ / count;
            statistics_.rmsBaselineTotalErrorDegrees = std::sqrt(
                baselineTotalSquaredSum_ / count);
            statistics_.maximumBaselineTotalErrorDegrees = std::max(
                statistics_.maximumBaselineTotalErrorDegrees,
                value.baselineTotalErrorDegrees);
            statistics_.meanBaselineTiltErrorDegrees = baselineTiltSum_ / count;
            statistics_.rmsBaselineTiltErrorDegrees = std::sqrt(
                baselineTiltSquaredSum_ / count);
            statistics_.meanTotalImprovementDegrees = baselineTotalSum_ / count
                - statistics_.meanPredictionTotalErrorDegrees;
            statistics_.meanTiltImprovementDegrees = baselineTiltSum_ / count
                - statistics_.meanPredictionTiltErrorDegrees;
            statistics_.improvedRatio = static_cast<double>(statistics_.improved) / count;
            latest = value;
        }
        else
        {
            ++statistics_.unmatched;
        }
        pending_.pop_front();
    }
    return latest;
}

void DelayedOrientationEvaluator::finish() noexcept
{
    statistics_.unmatched += pending_.size();
    statistics_.expired += pending_.size();
    pending_.clear();
}

const OrientationPredictionEvaluationStatistics&
DelayedOrientationEvaluator::statistics() const noexcept
{
    return statistics_;
}

std::size_t DelayedOrientationEvaluator::pendingCount() const noexcept
{
    return pending_.size();
}

std::string orientationPredictionCsvHeader()
{
    return "device_timestamp_ns,host_timestamp_ns,elapsed_seconds,sequence,recenter_generation,phase,"
           "prediction_mode,prediction_requested_horizon_ms,prediction_applied_horizon_ms,"
           "prediction_valid,prediction_reason,prediction_fallback_reason,prediction_clamped,"
           "angular_velocity_raw_rad_s_x,angular_velocity_raw_rad_s_y,angular_velocity_raw_rad_s_z,"
           "angular_velocity_filtered_rad_s_x,angular_velocity_filtered_rad_s_y,"
           "angular_velocity_filtered_rad_s_z,angular_speed_norm_dps,"
           "angular_acceleration_raw_rad_s2_x,angular_acceleration_raw_rad_s2_y,"
           "angular_acceleration_raw_rad_s2_z,angular_acceleration_filtered_rad_s2_x,"
           "angular_acceleration_filtered_rad_s2_y,angular_acceleration_filtered_rad_s2_z,"
           "angular_acceleration_used,measured_absolute_w,measured_absolute_x,measured_absolute_y,"
           "measured_absolute_z,predicted_absolute_w,predicted_absolute_x,predicted_absolute_y,"
           "predicted_absolute_z,measured_relative_w,measured_relative_x,measured_relative_y,"
           "measured_relative_z,predicted_relative_w,predicted_relative_x,predicted_relative_y,"
           "predicted_relative_z,measured_absolute_yaw_degrees,measured_absolute_pitch_degrees,"
           "measured_absolute_roll_degrees,predicted_absolute_yaw_degrees,"
           "predicted_absolute_pitch_degrees,predicted_absolute_roll_degrees,"
           "measured_relative_yaw_degrees,measured_relative_pitch_degrees,"
           "measured_relative_roll_degrees,predicted_relative_yaw_degrees,"
           "predicted_relative_pitch_degrees,predicted_relative_roll_degrees,"
           "predicted_delta_angle_degrees,delayed_evaluation_available,"
           "evaluation_target_device_timestamp_ns,evaluation_target_timestamp_error_ns,"
           "unpredicted_error_degrees,predicted_error_degrees,unpredicted_tilt_error_degrees,"
           "predicted_tilt_error_degrees,prediction_improvement_degrees,"
           "prediction_improvement_percent";
}

std::string serializeOrientationPredictionCsvRow(const OrientationPredictionRecord& record)
{
    const auto& p = record.prediction;
    const auto number = [](double value) -> std::string {
        if (!std::isfinite(value)) { return {}; }
        std::ostringstream output;
        output.imbue(std::locale::classic());
        output << std::setprecision(std::numeric_limits<double>::max_digits10) << value;
        return output.str();
    };
    const auto measuredAbsoluteEuler = quaternionToEulerDiagnostic(record.measuredAbsolute.value);
    const auto predictedAbsoluteEuler = quaternionToEulerDiagnostic(p.absolute.value);
    const auto measuredRelativeEuler = quaternionToEulerDiagnostic(record.measuredRelative.value);
    const auto predictedRelativeEuler = quaternionToEulerDiagnostic(p.relative.value);
    const bool clamped = p.horizon.clamped || p.diagnostics.speedClamped
        || p.diagnostics.accelerationClamped || p.diagnostics.angleClamped;
    const auto& evaluation = record.delayedEvaluation;
    std::ostringstream output;
    output.imbue(std::locale::classic());
    output << record.deviceTimestampNanoseconds << ',' << record.hostTimestampNanoseconds << ','
           << number(record.elapsedSeconds) << ',' << static_cast<unsigned int>(record.sequence)
           << ',' << record.recenterGeneration << ",\"" << record.phase << "\",\""
           << predictionModeText(record.mode) << "\"," << number(p.horizon.requested.count() * 1000.0)
           << ',' << number(p.horizon.applied.count() * 1000.0) << ','
           << (p.validity == PredictionValidity::valid ? "true" : "false") << ",\""
           << predictionRejectionReasonText(p.rejectionReason) << "\",\""
           << predictionFallbackReasonText(p.diagnostics.fallbackReason) << "\","
           << (clamped ? "true" : "false") << ','
           << number(p.angularVelocity.raw.xRadiansPerSecond) << ','
           << number(p.angularVelocity.raw.yRadiansPerSecond) << ','
           << number(p.angularVelocity.raw.zRadiansPerSecond) << ','
           << number(p.angularVelocity.filtered.xRadiansPerSecond) << ','
           << number(p.angularVelocity.filtered.yRadiansPerSecond) << ','
           << number(p.angularVelocity.filtered.zRadiansPerSecond) << ','
           << number(p.diagnostics.filteredAngularSpeedRadiansPerSecond * 57.29577951308232) << ',';
    if (p.angularAcceleration.available)
    {
        output << number(p.angularAcceleration.raw.xRadiansPerSecondSquared) << ','
               << number(p.angularAcceleration.raw.yRadiansPerSecondSquared) << ','
               << number(p.angularAcceleration.raw.zRadiansPerSecondSquared) << ','
               << number(p.angularAcceleration.filtered.xRadiansPerSecondSquared) << ','
               << number(p.angularAcceleration.filtered.yRadiansPerSecondSquared) << ','
               << number(p.angularAcceleration.filtered.zRadiansPerSecondSquared);
    }
    else
    {
        output << ",,,,,";
    }
    output << ',' << (p.angularAcceleration.used ? "true" : "false")
           << ',' << number(record.measuredAbsolute.value.w) << ',' << number(record.measuredAbsolute.value.x)
           << ',' << number(record.measuredAbsolute.value.y) << ',' << number(record.measuredAbsolute.value.z)
           << ',' << number(p.absolute.value.w) << ',' << number(p.absolute.value.x)
           << ',' << number(p.absolute.value.y) << ',' << number(p.absolute.value.z)
           << ',' << number(record.measuredRelative.value.w) << ',' << number(record.measuredRelative.value.x)
           << ',' << number(record.measuredRelative.value.y) << ',' << number(record.measuredRelative.value.z)
           << ',' << number(p.relative.value.w) << ',' << number(p.relative.value.x)
           << ',' << number(p.relative.value.y) << ',' << number(p.relative.value.z)
           << ',' << number(measuredAbsoluteEuler.yawDegrees) << ',' << number(measuredAbsoluteEuler.pitchDegrees)
           << ',' << number(measuredAbsoluteEuler.rollDegrees) << ',' << number(predictedAbsoluteEuler.yawDegrees)
           << ',' << number(predictedAbsoluteEuler.pitchDegrees) << ',' << number(predictedAbsoluteEuler.rollDegrees)
           << ',' << number(measuredRelativeEuler.yawDegrees) << ',' << number(measuredRelativeEuler.pitchDegrees)
           << ',' << number(measuredRelativeEuler.rollDegrees) << ',' << number(predictedRelativeEuler.yawDegrees)
           << ',' << number(predictedRelativeEuler.pitchDegrees) << ',' << number(predictedRelativeEuler.rollDegrees)
           << ',' << number(p.diagnostics.predictedAngleRadians * 57.29577951308232) << ','
           << (evaluation.has_value() ? "true" : "false") << ',';
    if (evaluation.has_value())
    {
        output << evaluation->targetDeviceTimestampNanoseconds << ','
               << evaluation->targetTimestampErrorNanoseconds << ','
               << number(evaluation->baselineTotalErrorDegrees) << ','
               << number(evaluation->predictionTotalErrorDegrees) << ','
               << number(evaluation->baselineTiltErrorDegrees) << ','
               << number(evaluation->predictionTiltErrorDegrees) << ','
               << number(evaluation->totalImprovementDegrees) << ',';
        if (evaluation->totalImprovementPercent.has_value())
        {
            output << number(*evaluation->totalImprovementPercent);
        }
    }
    else
    {
        output << ",,,,,,,";
    }
    return output.str();
}

std::string serializeOrientationPredictionJson(
    const OrientationPredictorConfig& configuration,
    const OrientationPredictorStatistics& predictor,
    const OrientationPredictionEvaluationStatistics& evaluation,
    const std::optional<OrientationPredictionRecord>& finalRecord,
    const OrientationPredictionMetadata& metadata)
{
    std::ostringstream output;
    output.imbue(std::locale::classic());
    output << std::setprecision(std::numeric_limits<double>::max_digits10)
           << "{\n  \"schema_version\":1,\n  \"type\":\"orientation-pose-prediction\","
           << "\n  \"experimental\":true,\n  \"quaternion_convention\":{\"component_order\":\"wxyz\","
           << "\"frame\":\"body-to-world\",\"multiplication_order\":\"current_times_body_delta\","
           << "\"handedness\":\"right-handed\"},\n  \"configuration\":{\"mode\":\""
           << predictionModeText(configuration.mode) << "\",\"horizon_ms\":"
           << configuration.horizon.count() * 1000.0 << ",\"maximum_horizon_ms\":"
           << configuration.maximumHorizon.count() * 1000.0 << ",\"limit_behavior\":\""
           << (configuration.limitBehavior == PredictionLimitBehavior::clamp ? "clamp" : "reject")
           << "\",\"angular_velocity_smoothing_seconds\":";
    if (configuration.angularVelocitySmoothingTimeConstant.has_value())
    {
        output << configuration.angularVelocitySmoothingTimeConstant->count();
    }
    else
    {
        output << "null";
    }
    output << ",\"angular_acceleration_smoothing_seconds\":";
    if (configuration.angularAccelerationSmoothingTimeConstant.has_value())
    {
        output << configuration.angularAccelerationSmoothingTimeConstant->count();
    }
    else
    {
        output << "null";
    }
    output << "},\n  \"provenance\":{\"gyroscope_scale_source\":\""
           << escapeJson(metadata.gyroscopeScaleSource) << "\",\"accelerometer_profile_source\":\""
           << escapeJson(metadata.accelerometerProfileSource) << "\",\"gyroscope_axis_mapping_source\":\""
           << escapeJson(metadata.gyroscopeAxisMappingSource) << "\",\"accelerometer_axis_mapping_source\":\""
           << escapeJson(metadata.accelerometerAxisMappingSource)
           << "\",\"scale_verified\":false,\"mappings_verified\":false},"
           << "\n  \"recenter\":{\"active\":" << (metadata.recenterActive ? "true" : "false")
           << ",\"generation\":" << metadata.recenterGeneration << "},"
           << "\n  \"predictor_statistics\":{\"requested\":" << predictor.requested
           << ",\"produced\":" << predictor.produced << ",\"rejected\":" << predictor.rejected
           << ",\"clamped_horizons\":" << predictor.clampedHorizons
           << ",\"clamped_speeds\":" << predictor.clampedSpeeds
           << ",\"clamped_accelerations\":" << predictor.clampedAccelerations
           << ",\"clamped_angles\":" << predictor.clampedAngles
           << ",\"constant_velocity_fallbacks\":" << predictor.constantVelocityFallbacks
           << ",\"zero_horizon\":" << predictor.zeroHorizonPredictions
           << ",\"constant_velocity\":" << predictor.constantVelocityPredictions
           << ",\"constant_acceleration\":" << predictor.constantAccelerationPredictions
           << ",\"invalid_input_rejections\":" << predictor.invalidInputRejections
           << ",\"average_requested_horizon_ms\":"
           << (predictor.requested == 0U ? 0.0
               : predictor.requestedHorizonSecondsSum * 1000.0 / static_cast<double>(predictor.requested))
           << ",\"average_applied_horizon_ms\":"
           << (predictor.produced == 0U ? 0.0
               : predictor.appliedHorizonSecondsSum * 1000.0 / static_cast<double>(predictor.produced))
           << ",\"maximum_angular_speed_dps\":"
           << predictor.maximumObservedAngularSpeedRadiansPerSecond * 57.29577951308232
           << ",\"maximum_predicted_angle_degrees\":"
           << predictor.maximumObservedPredictionAngleRadians * 57.29577951308232
           << "},\n  \"delayed_evaluation\":{\"matched\":" << evaluation.matched
           << ",\"unmatched\":" << evaluation.unmatched << ",\"expired\":" << evaluation.expired
           << ",\"mean_prediction_total_error_degrees\":" << evaluation.meanPredictionTotalErrorDegrees
           << ",\"rms_prediction_total_error_degrees\":" << evaluation.rmsPredictionTotalErrorDegrees
           << ",\"maximum_prediction_total_error_degrees\":" << evaluation.maximumPredictionTotalErrorDegrees
           << ",\"mean_prediction_tilt_error_degrees\":" << evaluation.meanPredictionTiltErrorDegrees
           << ",\"rms_prediction_tilt_error_degrees\":" << evaluation.rmsPredictionTiltErrorDegrees
           << ",\"maximum_prediction_tilt_error_degrees\":" << evaluation.maximumPredictionTiltErrorDegrees
           << ",\"mean_baseline_total_error_degrees\":" << evaluation.meanBaselineTotalErrorDegrees
           << ",\"rms_baseline_total_error_degrees\":" << evaluation.rmsBaselineTotalErrorDegrees
           << ",\"mean_baseline_tilt_error_degrees\":" << evaluation.meanBaselineTiltErrorDegrees
           << ",\"mean_total_improvement_degrees\":" << evaluation.meanTotalImprovementDegrees
           << ",\"mean_tilt_improvement_degrees\":" << evaluation.meanTiltImprovementDegrees
           << ",\"improved_ratio\":" << evaluation.improvedRatio << '}';
    if (finalRecord.has_value())
    {
        const auto& measuredAbsolute = finalRecord->measuredAbsolute.value;
        const auto& measuredRelative = finalRecord->measuredRelative.value;
        const auto& predictedAbsolute = finalRecord->prediction.absolute.value;
        const auto& predictedRelative = finalRecord->prediction.relative.value;
        const auto measuredEuler = quaternionToEulerDiagnostic(measuredAbsolute);
        const auto predictedEuler = quaternionToEulerDiagnostic(predictedAbsolute);
        output << ",\n  \"final\":{\"measured_absolute_quaternion_wxyz\":["
               << measuredAbsolute.w << ',' << measuredAbsolute.x << ',' << measuredAbsolute.y << ','
               << measuredAbsolute.z << "],\"predicted_absolute_quaternion_wxyz\":["
               << predictedAbsolute.w << ',' << predictedAbsolute.x << ',' << predictedAbsolute.y << ','
               << predictedAbsolute.z << "],\"measured_relative_quaternion_wxyz\":["
               << measuredRelative.w << ',' << measuredRelative.x << ',' << measuredRelative.y << ','
               << measuredRelative.z << "],\"predicted_relative_quaternion_wxyz\":["
               << predictedRelative.w << ',' << predictedRelative.x << ',' << predictedRelative.y << ','
               << predictedRelative.z << "],\"measured_absolute_euler_zyx_degrees\":["
               << measuredEuler.yawDegrees << ',' << measuredEuler.pitchDegrees << ','
               << measuredEuler.rollDegrees << "],\"predicted_absolute_euler_zyx_degrees\":["
               << predictedEuler.yawDegrees << ',' << predictedEuler.pitchDegrees << ','
               << predictedEuler.rollDegrees << "],\"predicted_delta_angle_degrees\":"
               << finalRecord->prediction.diagnostics.predictedAngleRadians * 57.29577951308232
               << ",\"validity\":\"" << predictionValidityText(finalRecord->prediction.validity)
               << "\",\"rejection_reason\":\""
               << predictionRejectionReasonText(finalRecord->prediction.rejectionReason)
               << "\",\"fallback_reason\":\""
               << predictionFallbackReasonText(finalRecord->prediction.diagnostics.fallbackReason)
               << "\"}";
    }
    output << "\n}\n";
    return output.str();
}

} // namespace xreal::sensors
