#include "rendering/DemoOrientationSource.hpp"

#include <chrono>
#include <cmath>

namespace xreal::rendering
{
namespace
{

[[nodiscard]] sensors::Quaternion axisAngle(Vector3 axis, double angle) noexcept
{
    const double half = angle * 0.5;
    const double sine = std::sin(half);
    return {std::cos(half), axis.x * sine, axis.y * sine, axis.z * sine};
}

[[nodiscard]] sensors::Quaternion relativeOrientation(
    const sensors::Quaternion& absolute,
    const sensors::Quaternion& reference,
    bool active) noexcept
{
    if (!active)
    {
        return absolute;
    }
    const auto inverse = reference.inverseNormalized();
    return inverse.has_value()
        ? (*inverse * absolute).normalized().value_or(absolute) : absolute;
}

[[nodiscard]] std::uint64_t steadyNowNanoseconds() noexcept
{
    return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
}

} // namespace

DemoOrientationSource::DemoOrientationSource(
    OrientationRenderBridge& bridge,
    double predictionHorizonMilliseconds)
    : bridge_(bridge),
      predictionHorizonSeconds_(predictionHorizonMilliseconds * 0.001),
      recenterReference_(sensors::Quaternion::identity())
{
}

sensors::Quaternion DemoOrientationSource::orientationAt(double elapsedSeconds) const noexcept
{
    const double yaw = yawOffset_ + 0.28 * std::sin(elapsedSeconds * 0.45);
    const double pitch = pitchOffset_ + 0.12 * std::sin(elapsedSeconds * 0.31);
    const double roll = rollOffset_ + 0.08 * std::sin(elapsedSeconds * 0.23);
    const auto composed = axisAngle({0.0, 0.0, 1.0}, yaw)
        * axisAngle({0.0, 1.0, 0.0}, pitch)
        * axisAngle({1.0, 0.0, 0.0}, roll);
    return composed.normalized().value_or(sensors::Quaternion::identity());
}

void DemoOrientationSource::update(double elapsedSeconds) noexcept
{
    const auto measured = orientationAt(elapsedSeconds);
    if (bridge_.consumeClearRecenterRequest())
    {
        recenterReference_ = sensors::Quaternion::identity();
        recenterActive_ = false;
        ++recenterGeneration_;
    }
    if (bridge_.consumeRecenterRequest())
    {
        recenterReference_ = measured;
        recenterActive_ = true;
        ++recenterGeneration_;
    }
    const auto predicted = orientationAt(elapsedSeconds + predictionHorizonSeconds_);
    RenderOrientationSnapshot snapshot;
    snapshot.measuredAbsolute = measured;
    snapshot.measuredRelative = relativeOrientation(measured, recenterReference_, recenterActive_);
    snapshot.predictedAbsolute = predicted;
    snapshot.predictedRelative = relativeOrientation(predicted, recenterReference_, recenterActive_);
    snapshot.deviceTimestampNanoseconds = static_cast<std::uint64_t>(elapsedSeconds * 1.0e9);
    snapshot.hostPublishTimestampNanoseconds = steadyNowNanoseconds();
    snapshot.predictionHorizonMilliseconds = predictionHorizonSeconds_ * 1000.0;
    snapshot.measuredValid = true;
    snapshot.predictionValid = true;
    snapshot.recenterGeneration = recenterGeneration_;
    (void)bridge_.publish(snapshot);
}

void DemoOrientationSource::adjustYaw(double radians) noexcept { yawOffset_ += radians; }
void DemoOrientationSource::adjustPitch(double radians) noexcept { pitchOffset_ += radians; }
void DemoOrientationSource::adjustRoll(double radians) noexcept { rollOffset_ += radians; }

void DemoOrientationSource::reset() noexcept
{
    yawOffset_ = 0.0;
    pitchOffset_ = 0.0;
    rollOffset_ = 0.0;
    recenterReference_ = sensors::Quaternion::identity();
    recenterActive_ = false;
    ++recenterGeneration_;
}

} // namespace xreal::rendering
