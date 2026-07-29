#include "sensors/OrientationPrediction.hpp"

#include "sensors/DeviceTimestampDelta.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace xreal::sensors
{
namespace
{

[[nodiscard]] double magnitude(const AngularVelocityRadians& value) noexcept
{
    return std::sqrt(value.xRadiansPerSecond * value.xRadiansPerSecond
        + value.yRadiansPerSecond * value.yRadiansPerSecond
        + value.zRadiansPerSecond * value.zRadiansPerSecond);
}

[[nodiscard]] double magnitude(const AngularAccelerationRadians& value) noexcept
{
    return std::sqrt(value.xRadiansPerSecondSquared * value.xRadiansPerSecondSquared
        + value.yRadiansPerSecondSquared * value.yRadiansPerSecondSquared
        + value.zRadiansPerSecondSquared * value.zRadiansPerSecondSquared);
}

[[nodiscard]] double smoothingAlpha(
    std::chrono::duration<double> delta,
    const std::optional<std::chrono::duration<double>>& timeConstant) noexcept
{
    if (!timeConstant.has_value() || timeConstant->count() <= 0.0)
    {
        return 1.0;
    }
    return 1.0 - std::exp(-delta.count() / timeConstant->count());
}

[[nodiscard]] AngularVelocityRadians blend(
    const AngularVelocityRadians& previous,
    const AngularVelocityRadians& current,
    double alpha) noexcept
{
    return {
        previous.xRadiansPerSecond + alpha * (current.xRadiansPerSecond - previous.xRadiansPerSecond),
        previous.yRadiansPerSecond + alpha * (current.yRadiansPerSecond - previous.yRadiansPerSecond),
        previous.zRadiansPerSecond + alpha * (current.zRadiansPerSecond - previous.zRadiansPerSecond),
    };
}

[[nodiscard]] AngularAccelerationRadians blend(
    const AngularAccelerationRadians& previous,
    const AngularAccelerationRadians& current,
    double alpha) noexcept
{
    return {
        previous.xRadiansPerSecondSquared + alpha * (current.xRadiansPerSecondSquared - previous.xRadiansPerSecondSquared),
        previous.yRadiansPerSecondSquared + alpha * (current.yRadiansPerSecondSquared - previous.yRadiansPerSecondSquared),
        previous.zRadiansPerSecondSquared + alpha * (current.zRadiansPerSecondSquared - previous.zRadiansPerSecondSquared),
    };
}

template <typename Value>
void scaleToMagnitude(Value& value, double currentMagnitude, double maximum) noexcept;

template <>
void scaleToMagnitude(AngularVelocityRadians& value, double currentMagnitude, double maximum) noexcept
{
    const double factor = maximum / currentMagnitude;
    value.xRadiansPerSecond *= factor;
    value.yRadiansPerSecond *= factor;
    value.zRadiansPerSecond *= factor;
}

template <>
void scaleToMagnitude(AngularAccelerationRadians& value, double currentMagnitude, double maximum) noexcept
{
    const double factor = maximum / currentMagnitude;
    value.xRadiansPerSecondSquared *= factor;
    value.yRadiansPerSecondSquared *= factor;
    value.zRadiansPerSecondSquared *= factor;
}

} // namespace

bool AngularAccelerationRadians::finite() const noexcept
{
    return std::isfinite(xRadiansPerSecondSquared)
        && std::isfinite(yRadiansPerSecondSquared)
        && std::isfinite(zRadiansPerSecondSquared);
}

std::optional<Quaternion> quaternionFromBodyRotationVector(
    const AngularVelocityRadians& rotationVector,
    double smallAngleThresholdRadians) noexcept
{
    if (!rotationVector.finite() || !std::isfinite(smallAngleThresholdRadians)
        || smallAngleThresholdRadians < 0.0)
    {
        return std::nullopt;
    }
    const double angle = magnitude(rotationVector);
    Quaternion delta;
    if (angle < smallAngleThresholdRadians)
    {
        delta = {1.0,
                 rotationVector.xRadiansPerSecond * 0.5,
                 rotationVector.yRadiansPerSecond * 0.5,
                 rotationVector.zRadiansPerSecond * 0.5};
    }
    else
    {
        const double halfAngle = angle * 0.5;
        const double factor = std::sin(halfAngle) / angle;
        delta = {std::cos(halfAngle),
                 rotationVector.xRadiansPerSecond * factor,
                 rotationVector.yRadiansPerSecond * factor,
                 rotationVector.zRadiansPerSecond * factor};
    }
    return delta.normalized();
}

OrientationPredictor::OrientationPredictor(OrientationPredictorConfig configuration)
    : configuration_(std::move(configuration))
{
}

void OrientationPredictor::reset() noexcept
{
    state_ = {};
}

const OrientationPredictorConfig& OrientationPredictor::configuration() const noexcept
{
    return configuration_;
}

const OrientationPredictorState& OrientationPredictor::state() const noexcept
{
    return state_;
}

OrientationPredictionResult OrientationPredictor::predict(
    const OrientationPredictorInput& input) noexcept
{
    ++state_.statistics.requested;
    state_.statistics.requestedHorizonSecondsSum += configuration_.horizon.count();
    OrientationPredictionResult result;
    result.horizon.requested = configuration_.horizon;
    result.horizon.applied = configuration_.horizon;
    result.angularVelocity.raw = input.bodyAngularVelocity;
    result.absolute.value = input.measuredAbsolute.value;
    result.relative.value = makeRelativeOrientation(
        input.measuredAbsolute, input.recenterReference).value_or(
            RelativeOrientation{input.measuredAbsolute.value}).value;

    const auto reject = [&](PredictionRejectionReason reason) {
        result.validity = PredictionValidity::rejected;
        result.rejectionReason = reason;
        ++state_.statistics.rejected;
        return result;
    };

    const auto measured = input.measuredAbsolute.value.normalized();
    if (!measured.has_value())
    {
        ++state_.statistics.invalidInputRejections;
        return reject(PredictionRejectionReason::invalidOrientation);
    }
    if (!input.bodyAngularVelocity.finite())
    {
        ++state_.statistics.invalidInputRejections;
        return reject(PredictionRejectionReason::invalidAngularVelocity);
    }
    if (!std::isfinite(configuration_.horizon.count()) || configuration_.horizon.count() < 0.0
        || !std::isfinite(configuration_.maximumHorizon.count())
        || configuration_.maximumHorizon.count() < 0.0)
    {
        return reject(PredictionRejectionReason::horizonExceedsMaximum);
    }
    if (result.horizon.applied > configuration_.maximumHorizon)
    {
        if (configuration_.limitBehavior == PredictionLimitBehavior::reject)
        {
            return reject(PredictionRejectionReason::horizonExceedsMaximum);
        }
        result.horizon.applied = configuration_.maximumHorizon;
        result.horizon.clamped = true;
        ++state_.statistics.clampedHorizons;
    }

    std::optional<std::chrono::nanoseconds> delta;
    if (state_.previousDeviceTimestampNanoseconds.has_value())
    {
        if (input.deviceTimestampNanoseconds == *state_.previousDeviceTimestampNanoseconds)
        {
            ++state_.statistics.duplicateTimestamps;
            result.diagnostics.fallbackReason = PredictionFallbackReason::duplicateTimestamp;
        }
        else
        {
            const auto forward = forwardDeviceTimestampDelta(
                input.deviceTimestampNanoseconds, *state_.previousDeviceTimestampNanoseconds);
            if (!forward.has_value())
            {
                ++state_.statistics.decreasingTimestamps;
                state_.previousDeviceTimestampNanoseconds = input.deviceTimestampNanoseconds;
                state_.filteredAngularVelocity = input.bodyAngularVelocity;
                state_.filteredAngularAcceleration.reset();
                return reject(PredictionRejectionReason::decreasingTimestamp);
            }
            delta = std::chrono::nanoseconds(*forward);
            if (*delta > configuration_.maximumTimestampDelta)
            {
                ++state_.statistics.excessiveTimestampDeltas;
                result.diagnostics.fallbackReason = PredictionFallbackReason::excessiveTimestampDelta;
                state_.filteredAngularVelocity = input.bodyAngularVelocity;
                state_.filteredAngularAcceleration.reset();
                delta.reset();
            }
        }
    }
    else
    {
        result.diagnostics.fallbackReason = PredictionFallbackReason::firstSample;
    }

    AngularVelocityRadians filtered = input.bodyAngularVelocity;
    if (delta.has_value() && state_.filteredAngularVelocity.has_value())
    {
        const double alpha = smoothingAlpha(*delta,
            configuration_.angularVelocitySmoothingTimeConstant);
        filtered = blend(*state_.filteredAngularVelocity, input.bodyAngularVelocity, alpha);
        result.angularVelocity.smoothingApplied = alpha < 1.0;
    }
    else if (!delta.has_value() && state_.filteredAngularVelocity.has_value()
             && result.diagnostics.fallbackReason == PredictionFallbackReason::duplicateTimestamp)
    {
        filtered = *state_.filteredAngularVelocity;
    }
    result.angularVelocity.filtered = filtered;
    result.diagnostics.rawAngularSpeedRadiansPerSecond = magnitude(input.bodyAngularVelocity);
    result.diagnostics.filteredAngularSpeedRadiansPerSecond = magnitude(filtered);
    state_.statistics.maximumObservedAngularSpeedRadiansPerSecond = std::max(
        state_.statistics.maximumObservedAngularSpeedRadiansPerSecond,
        result.diagnostics.rawAngularSpeedRadiansPerSecond);

    if (result.diagnostics.filteredAngularSpeedRadiansPerSecond
        > configuration_.maximumAngularSpeedRadiansPerSecond)
    {
        if (configuration_.limitBehavior == PredictionLimitBehavior::reject)
        {
            return reject(PredictionRejectionReason::angularSpeedExceedsMaximum);
        }
        scaleToMagnitude(filtered, result.diagnostics.filteredAngularSpeedRadiansPerSecond,
            configuration_.maximumAngularSpeedRadiansPerSecond);
        result.angularVelocity.filtered = filtered;
        result.diagnostics.filteredAngularSpeedRadiansPerSecond = magnitude(filtered);
        result.diagnostics.speedClamped = true;
        ++state_.statistics.clampedSpeeds;
    }

    AngularAccelerationRadians acceleration;
    if (delta.has_value() && state_.filteredAngularVelocity.has_value()
        && delta->count() > 0)
    {
        const double seconds = std::chrono::duration<double>(*delta).count();
        acceleration = {
            (filtered.xRadiansPerSecond - state_.filteredAngularVelocity->xRadiansPerSecond) / seconds,
            (filtered.yRadiansPerSecond - state_.filteredAngularVelocity->yRadiansPerSecond) / seconds,
            (filtered.zRadiansPerSecond - state_.filteredAngularVelocity->zRadiansPerSecond) / seconds,
        };
        result.angularAcceleration.raw = acceleration;
        if (state_.filteredAngularAcceleration.has_value())
        {
            const double alpha = smoothingAlpha(*delta,
                configuration_.angularAccelerationSmoothingTimeConstant);
            acceleration = blend(*state_.filteredAngularAcceleration, acceleration, alpha);
        }
        result.angularAcceleration.filtered = acceleration;
        result.angularAcceleration.available = acceleration.finite();
    }

    const double accelerationMagnitude = result.angularAcceleration.available
        ? magnitude(acceleration) : 0.0;
    result.diagnostics.angularAccelerationRadiansPerSecondSquared = accelerationMagnitude;
    if (result.angularAcceleration.available
        && accelerationMagnitude > configuration_.maximumAngularAccelerationRadiansPerSecondSquared)
    {
        if (configuration_.limitBehavior == PredictionLimitBehavior::clamp)
        {
            scaleToMagnitude(acceleration, accelerationMagnitude,
                configuration_.maximumAngularAccelerationRadiansPerSecondSquared);
            result.angularAcceleration.filtered = acceleration;
            result.diagnostics.accelerationClamped = true;
            ++state_.statistics.clampedAccelerations;
        }
        else
        {
            result.angularAcceleration.available = false;
            result.diagnostics.fallbackReason = PredictionFallbackReason::angularAccelerationExceedsMaximum;
        }
    }

    const double horizonSeconds = result.horizon.applied.count();
    AngularVelocityRadians rotationVector{
        filtered.xRadiansPerSecond * horizonSeconds,
        filtered.yRadiansPerSecond * horizonSeconds,
        filtered.zRadiansPerSecond * horizonSeconds,
    };
    if (configuration_.mode == PredictionMode::constantAcceleration
        && result.angularAcceleration.available)
    {
        AngularVelocityRadians accelerationContribution{
            0.5 * acceleration.xRadiansPerSecondSquared * horizonSeconds * horizonSeconds,
            0.5 * acceleration.yRadiansPerSecondSquared * horizonSeconds * horizonSeconds,
            0.5 * acceleration.zRadiansPerSecondSquared * horizonSeconds * horizonSeconds,
        };
        double contributionMagnitude = magnitude(accelerationContribution);
        if (contributionMagnitude > configuration_.maximumAccelerationContributionRadians)
        {
            scaleToMagnitude(accelerationContribution, contributionMagnitude,
                configuration_.maximumAccelerationContributionRadians);
            contributionMagnitude = configuration_.maximumAccelerationContributionRadians;
            result.diagnostics.accelerationClamped = true;
            ++state_.statistics.clampedAccelerations;
        }
        rotationVector.xRadiansPerSecond += accelerationContribution.xRadiansPerSecond;
        rotationVector.yRadiansPerSecond += accelerationContribution.yRadiansPerSecond;
        rotationVector.zRadiansPerSecond += accelerationContribution.zRadiansPerSecond;
        result.diagnostics.accelerationContributionRadians = contributionMagnitude;
        result.angularAcceleration.used = true;
        ++state_.statistics.constantAccelerationPredictions;
    }
    else if (configuration_.mode == PredictionMode::constantAcceleration)
    {
        ++state_.statistics.constantVelocityFallbacks;
    }

    double predictedAngle = magnitude(rotationVector);
    if (predictedAngle > configuration_.maximumPredictionAngleRadians)
    {
        if (configuration_.limitBehavior == PredictionLimitBehavior::reject)
        {
            return reject(PredictionRejectionReason::predictedAngleExceedsMaximum);
        }
        scaleToMagnitude(rotationVector, predictedAngle, configuration_.maximumPredictionAngleRadians);
        predictedAngle = configuration_.maximumPredictionAngleRadians;
        result.diagnostics.angleClamped = true;
        ++state_.statistics.clampedAngles;
    }
    result.diagnostics.predictedAngleRadians = predictedAngle;
    state_.statistics.maximumObservedPredictionAngleRadians = std::max(
        state_.statistics.maximumObservedPredictionAngleRadians, predictedAngle);
    const auto deltaOrientation = quaternionFromBodyRotationVector(
        rotationVector, configuration_.smallAngleThresholdRadians);
    if (!deltaOrientation.has_value())
    {
        return reject(PredictionRejectionReason::invalidAngularVelocity);
    }
    const auto predicted = (*measured * *deltaOrientation).normalized();
    if (!predicted.has_value())
    {
        return reject(PredictionRejectionReason::invalidOrientation);
    }
    result.absolute.value = *predicted;
    const auto relative = makeRelativeOrientation({*predicted}, input.recenterReference);
    if (!relative.has_value())
    {
        return reject(PredictionRejectionReason::invalidOrientation);
    }
    result.relative.value = relative->value;
    result.validity = PredictionValidity::valid;
    result.rejectionReason = PredictionRejectionReason::none;
    ++state_.statistics.produced;
    state_.statistics.appliedHorizonSecondsSum += result.horizon.applied.count();
    if (result.horizon.applied.count() == 0.0)
    {
        ++state_.statistics.zeroHorizonPredictions;
    }
    if (!result.angularAcceleration.used)
    {
        ++state_.statistics.constantVelocityPredictions;
    }

    if (input.deviceTimestampNanoseconds != state_.previousDeviceTimestampNanoseconds)
    {
        state_.previousDeviceTimestampNanoseconds = input.deviceTimestampNanoseconds;
        state_.filteredAngularVelocity = filtered;
        if (result.angularAcceleration.available)
        {
            state_.filteredAngularAcceleration = acceleration;
        }
    }
    return result;
}

std::string predictionModeText(PredictionMode mode)
{
    return mode == PredictionMode::constantAcceleration
        ? "constant-acceleration" : "constant-velocity";
}

std::string predictionValidityText(PredictionValidity validity)
{
    return validity == PredictionValidity::valid ? "valid" : "rejected";
}

std::string predictionRejectionReasonText(PredictionRejectionReason reason)
{
    switch (reason)
    {
    case PredictionRejectionReason::none: return "none";
    case PredictionRejectionReason::invalidOrientation: return "invalid-orientation";
    case PredictionRejectionReason::invalidAngularVelocity: return "invalid-angular-velocity";
    case PredictionRejectionReason::decreasingTimestamp: return "decreasing-timestamp";
    case PredictionRejectionReason::horizonExceedsMaximum: return "horizon-exceeds-maximum";
    case PredictionRejectionReason::angularSpeedExceedsMaximum: return "angular-speed-exceeds-maximum";
    case PredictionRejectionReason::predictedAngleExceedsMaximum: return "predicted-angle-exceeds-maximum";
    }
    return "unknown";
}

std::string predictionFallbackReasonText(PredictionFallbackReason reason)
{
    switch (reason)
    {
    case PredictionFallbackReason::none: return "none";
    case PredictionFallbackReason::firstSample: return "first-sample";
    case PredictionFallbackReason::duplicateTimestamp: return "duplicate-timestamp";
    case PredictionFallbackReason::excessiveTimestampDelta: return "excessive-timestamp-delta";
    case PredictionFallbackReason::angularAccelerationExceedsMaximum:
        return "angular-acceleration-exceeds-maximum";
    }
    return "unknown";
}

} // namespace xreal::sensors
