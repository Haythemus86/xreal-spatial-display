#include "sensors/GyroscopeOrientation.hpp"

#include "sensors/DeviceTimestampDelta.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <limits>
#include <numbers>
#include <sstream>
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

[[nodiscard]] std::string_view axisName(GyroscopeLogicalAxis axis) noexcept
{
    switch (axis)
    {
    case GyroscopeLogicalAxis::sensorX: return "x";
    case GyroscopeLogicalAxis::sensorY: return "y";
    case GyroscopeLogicalAxis::sensorZ: return "z";
    }
    return "invalid";
}

} // namespace

double Quaternion::norm() const noexcept
{
    return std::sqrt(w * w + x * x + y * y + z * z);
}

bool Quaternion::finite() const noexcept
{
    return std::isfinite(w) && std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
}

std::optional<Quaternion> Quaternion::normalized() const noexcept
{
    const double length = norm();
    if (!finite() || !std::isfinite(length) || length <= std::numeric_limits<double>::epsilon())
    {
        return std::nullopt;
    }
    return Quaternion{w / length, x / length, y / length, z / length};
}

Quaternion Quaternion::conjugate() const noexcept
{
    return {w, -x, -y, -z};
}

std::optional<Quaternion> Quaternion::inverseNormalized() const noexcept
{
    const auto unit = normalized();
    return unit.has_value() ? std::optional<Quaternion>(unit->conjugate()) : std::nullopt;
}

Quaternion operator*(const Quaternion& left, const Quaternion& right) noexcept
{
    return {
        left.w * right.w - left.x * right.x - left.y * right.y - left.z * right.z,
        left.w * right.x + left.x * right.w + left.y * right.z - left.z * right.y,
        left.w * right.y - left.x * right.z + left.y * right.w + left.z * right.x,
        left.w * right.z + left.x * right.y - left.y * right.x + left.z * right.w,
    };
}

bool AngularVelocityRadians::finite() const noexcept
{
    return std::isfinite(xRadiansPerSecond) && std::isfinite(yRadiansPerSecond)
        && std::isfinite(zRadiansPerSecond);
}

AngularVelocityRadians mapAngularVelocity(
    const AngularVelocityRadians& sensor,
    const GyroscopeAxisMapping& mapping) noexcept
{
    AngularVelocityRadians result;
    const auto assign = [&](const GyroscopeAxisMappingEntry& entry, double value) {
        const double mapped = value * static_cast<double>(entry.sign);
        switch (entry.logicalAxis)
        {
        case GyroscopeLogicalAxis::sensorX: result.xRadiansPerSecond = mapped; break;
        case GyroscopeLogicalAxis::sensorY: result.yRadiansPerSecond = mapped; break;
        case GyroscopeLogicalAxis::sensorZ: result.zRadiansPerSecond = mapped; break;
        }
    };
    assign(mapping.sensorX, sensor.xRadiansPerSecond);
    assign(mapping.sensorY, sensor.yRadiansPerSecond);
    assign(mapping.sensorZ, sensor.zRadiansPerSecond);
    return result;
}

GyroscopeOrientationIntegrator::GyroscopeOrientationIntegrator(
    OrientationIntegratorConfig configuration)
    : configuration_(std::move(configuration))
{
    reset();
}

void GyroscopeOrientationIntegrator::reset() noexcept
{
    state_ = {};
    state_.orientation = Quaternion::identity();
    state_.valid = true;
    recenterReference_ = Quaternion::identity();
}

void GyroscopeOrientationIntegrator::clearTimestamp() noexcept
{
    state_.lastValidDeviceTimestamp.reset();
    state_.initialized = false;
    state_.lastDeltaTime = {};
}

bool GyroscopeOrientationIntegrator::setOrientation(const Quaternion& orientationValue) noexcept
{
    const auto normalized = orientationValue.normalized();
    if (!normalized.has_value())
    {
        state_.lastRejectionReason = OrientationRejectionReason::invalidOrientation;
        return false;
    }
    state_.orientation = *normalized;
    state_.valid = true;
    return true;
}

bool GyroscopeOrientationIntegrator::recenter() noexcept
{
    const auto normalized = state_.orientation.normalized();
    if (!normalized.has_value())
    {
        return false;
    }
    recenterReference_ = *normalized;
    return true;
}

void GyroscopeOrientationIntegrator::clearRecenter() noexcept
{
    recenterReference_ = Quaternion::identity();
}

Quaternion GyroscopeOrientationIntegrator::orientation() const noexcept
{
    return state_.orientation;
}

Quaternion GyroscopeOrientationIntegrator::relativeOrientation() const noexcept
{
    const auto inverse = recenterReference_.inverseNormalized();
    if (!inverse.has_value())
    {
        return state_.orientation;
    }
    const auto relative = (*inverse * state_.orientation).normalized();
    return relative.value_or(state_.orientation);
}

const OrientationState& GyroscopeOrientationIntegrator::state() const noexcept
{
    return state_;
}

const OrientationIntegratorConfig& GyroscopeOrientationIntegrator::configuration() const noexcept
{
    return configuration_;
}

OrientationIntegratorResult GyroscopeOrientationIntegrator::result(
    OrientationSampleStatus status,
    OrientationRejectionReason reason,
    std::chrono::nanoseconds delta) const noexcept
{
    return {status, reason, state_.orientation, relativeOrientation(), delta};
}

OrientationIntegratorResult GyroscopeOrientationIntegrator::update(
    const AngularVelocityRadians& sensorAngularVelocity,
    std::uint64_t deviceTimestamp) noexcept
{
    if (!validateGyroscopeAxisMapping(configuration_.axisMapping))
    {
        ++state_.rejectedSampleCount;
        state_.lastRejectionReason = OrientationRejectionReason::invalidAxisMapping;
        return result(OrientationSampleStatus::rejected, state_.lastRejectionReason, {});
    }
    if (!sensorAngularVelocity.finite())
    {
        ++state_.rejectedSampleCount;
        state_.lastRejectionReason = OrientationRejectionReason::invalidAngularVelocity;
        return result(OrientationSampleStatus::rejected, state_.lastRejectionReason, {});
    }
    if (!state_.lastValidDeviceTimestamp.has_value())
    {
        state_.lastValidDeviceTimestamp = deviceTimestamp;
        state_.initialized = true;
        ++state_.skippedSampleCount;
        state_.lastRejectionReason = OrientationRejectionReason::firstTimestamp;
        return result(OrientationSampleStatus::skipped, state_.lastRejectionReason, {});
    }
    if (deviceTimestamp == *state_.lastValidDeviceTimestamp)
    {
        ++state_.skippedSampleCount;
        state_.lastRejectionReason = OrientationRejectionReason::duplicateTimestamp;
        return result(OrientationSampleStatus::skipped, state_.lastRejectionReason, {});
    }
    const auto deltaValue = forwardDeviceTimestampDelta(
        deviceTimestamp, *state_.lastValidDeviceTimestamp);
    if (!deltaValue.has_value())
    {
        ++state_.rejectedSampleCount;
        state_.lastRejectionReason = OrientationRejectionReason::decreasingTimestamp;
        return result(OrientationSampleStatus::rejected, state_.lastRejectionReason, {});
    }
    const std::chrono::nanoseconds delta(*deltaValue);
    if (delta <= std::chrono::nanoseconds::zero()
        || delta > configuration_.maximumDeviceTimestampDelta)
    {
        ++state_.rejectedSampleCount;
        state_.lastRejectionReason = OrientationRejectionReason::excessiveTimestampDelta;
        state_.lastValidDeviceTimestamp = deviceTimestamp;
        return result(OrientationSampleStatus::rejected, state_.lastRejectionReason, delta);
    }

    const AngularVelocityRadians angularVelocity = mapAngularVelocity(
        sensorAngularVelocity, configuration_.axisMapping);
    const double seconds = std::chrono::duration<double>(delta).count();
    const double magnitude = std::sqrt(
        angularVelocity.xRadiansPerSecond * angularVelocity.xRadiansPerSecond
        + angularVelocity.yRadiansPerSecond * angularVelocity.yRadiansPerSecond
        + angularVelocity.zRadiansPerSecond * angularVelocity.zRadiansPerSecond);
    const double theta = magnitude * seconds;
    Quaternion deltaOrientation;
    if (theta < configuration_.smallAngleThresholdRadians)
    {
        deltaOrientation = {1.0,
                            angularVelocity.xRadiansPerSecond * seconds * 0.5,
                            angularVelocity.yRadiansPerSecond * seconds * 0.5,
                            angularVelocity.zRadiansPerSecond * seconds * 0.5};
    }
    else
    {
        const double halfTheta = theta * 0.5;
        const double factor = std::sin(halfTheta) / magnitude;
        deltaOrientation = {std::cos(halfTheta),
                            angularVelocity.xRadiansPerSecond * factor,
                            angularVelocity.yRadiansPerSecond * factor,
                            angularVelocity.zRadiansPerSecond * factor};
    }
    const auto next = (state_.orientation * deltaOrientation).normalized();
    if (!next.has_value())
    {
        ++state_.rejectedSampleCount;
        state_.lastRejectionReason = OrientationRejectionReason::invalidOrientation;
        return result(OrientationSampleStatus::rejected, state_.lastRejectionReason, delta);
    }
    state_.orientation = *next;
    state_.lastValidDeviceTimestamp = deviceTimestamp;
    state_.lastDeltaTime = delta;
    state_.accumulatedIntegrationDuration += delta;
    state_.lastRejectionReason = OrientationRejectionReason::none;
    ++state_.appliedSampleCount;
    return result(OrientationSampleStatus::applied, OrientationRejectionReason::none, delta);
}

EulerAnglesDiagnostic quaternionToEulerDiagnostic(const Quaternion& orientation)
    noexcept
{
    const Quaternion q = orientation.normalized().value_or(Quaternion::identity());
    const double sinRollCosPitch = 2.0 * (q.w * q.x + q.y * q.z);
    const double cosRollCosPitch = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
    const double sinPitch = std::clamp(2.0 * (q.w * q.y - q.z * q.x), -1.0, 1.0);
    const double sinYawCosPitch = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosYawCosPitch = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    EulerAnglesDiagnostic result;
    result.rollRadians = std::atan2(sinRollCosPitch, cosRollCosPitch);
    result.pitchRadians = std::asin(sinPitch);
    result.yawRadians = std::atan2(sinYawCosPitch, cosYawCosPitch);
    constexpr double radiansToDegrees = 180.0 / std::numbers::pi;
    result.rollDegrees = result.rollRadians * radiansToDegrees;
    result.pitchDegrees = result.pitchRadians * radiansToDegrees;
    result.yawDegrees = result.yawRadians * radiansToDegrees;
    return result;
}

std::string serializeGyroscopeOrientationJson(
    const GyroscopeOrientationIntegrator& integrator,
    const GyroscopeScaleProfile& scaleProfile,
    bool includeEuler)
{
    const auto& state = integrator.state();
    const Quaternion q = integrator.relativeOrientation();
    const auto euler = quaternionToEulerDiagnostic(q);
    const auto& mapping = integrator.configuration().axisMapping;
    std::ostringstream output;
    output << std::setprecision(std::numeric_limits<double>::max_digits10)
           << "{\n  \"schema_version\":1,\n  \"orientation\":{\"type\":\"gyro-only-quaternion\","
           << "\"valid\":" << (state.valid ? "true" : "false")
           << ",\"experimental\":true,\"quaternion_convention\":{"
           << "\"component_order\":\"wxyz\",\"frame\":\"body-to-world\","
           << "\"multiplication_order\":\"current_times_body_delta\","
           << "\"handedness\":\"right-handed\"},\"axis_mapping\":{"
           << "\"source\":\"" << escapeJson(mapping.source) << "\",\"notes\":\""
           << escapeJson(mapping.notes) << "\",\"experimental\":"
           << (mapping.experimental ? "true" : "false") << ",\"verified\":"
           << (mapping.verified ? "true" : "false") << ",\"sensor_x_target\":\""
           << axisName(mapping.sensorX.logicalAxis) << "\",\"sensor_x_sign\":"
           << mapping.sensorX.sign << ",\"sensor_y_target\":\""
           << axisName(mapping.sensorY.logicalAxis) << "\",\"sensor_y_sign\":"
           << mapping.sensorY.sign << ",\"sensor_z_target\":\""
           << axisName(mapping.sensorZ.logicalAxis) << "\",\"sensor_z_sign\":"
           << mapping.sensorZ.sign << "},\"scale_profile\":{\"source\":\""
           << escapeJson(scaleProfile.source) << "\",\"experimental\":"
           << (scaleProfile.experimental ? "true" : "false") << ",\"verified\":"
           << (scaleProfile.verified ? "true" : "false")
           << "},\"final_quaternion\":{\"w\":" << q.w << ",\"x\":" << q.x
           << ",\"y\":" << q.y << ",\"z\":" << q.z << '}';
    if (includeEuler)
    {
        output << ",\"final_euler_degrees\":{\"yaw\":" << euler.yawDegrees
               << ",\"pitch\":" << euler.pitchDegrees << ",\"roll\":"
               << euler.rollDegrees << '}';
    }
    output << ",\"statistics\":{\"applied_samples\":" << state.appliedSampleCount
           << ",\"skipped_samples\":" << state.skippedSampleCount
           << ",\"rejected_samples\":" << state.rejectedSampleCount
           << ",\"integration_duration_seconds\":"
           << std::chrono::duration<double>(state.accumulatedIntegrationDuration).count()
           << "}}\n}\n";
    return output.str();
}

std::string orientationRejectionReasonText(OrientationRejectionReason reason)
{
    switch (reason)
    {
    case OrientationRejectionReason::none: return "none";
    case OrientationRejectionReason::firstTimestamp: return "first timestamp";
    case OrientationRejectionReason::duplicateTimestamp: return "duplicate timestamp";
    case OrientationRejectionReason::decreasingTimestamp: return "decreasing timestamp";
    case OrientationRejectionReason::excessiveTimestampDelta: return "excessive timestamp delta";
    case OrientationRejectionReason::invalidAngularVelocity: return "invalid angular velocity";
    case OrientationRejectionReason::invalidAxisMapping: return "invalid axis mapping";
    case OrientationRejectionReason::invalidOrientation: return "invalid orientation";
    }
    return "unknown";
}

} // namespace xreal::sensors
