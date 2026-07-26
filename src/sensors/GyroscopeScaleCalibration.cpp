#include "sensors/GyroscopeScaleCalibration.hpp"

#include "sensors/DeviceTimestampDelta.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <numbers>
#include <sstream>
#include <stdexcept>

namespace xreal::sensors
{
namespace
{

[[nodiscard]] double rawAxisValue(const RawImuVector3& value, GyroscopeAxis axis) noexcept
{
    switch (axis)
    {
    case GyroscopeAxis::x:
        return static_cast<double>(value.x);
    case GyroscopeAxis::y:
        return static_cast<double>(value.y);
    case GyroscopeAxis::z:
        return static_cast<double>(value.z);
    }
    return 0.0;
}

[[nodiscard]] double biasAxisValue(const GyroscopeBias& value, GyroscopeAxis axis) noexcept
{
    switch (axis)
    {
    case GyroscopeAxis::x:
        return value.x;
    case GyroscopeAxis::y:
        return value.y;
    case GyroscopeAxis::z:
        return value.z;
    }
    return 0.0;
}

[[nodiscard]] RawVector3d correctedRawVector(
    const RawImuVector3& raw,
    const GyroscopeBias& bias) noexcept
{
    return {
        static_cast<double>(raw.x) - bias.x,
        static_cast<double>(raw.y) - bias.y,
        static_cast<double>(raw.z) - bias.z,
    };
}

[[nodiscard]] double vectorAxisValue(const RawVector3d& value, GyroscopeAxis axis) noexcept
{
    switch (axis)
    {
    case GyroscopeAxis::x:
        return value.x;
    case GyroscopeAxis::y:
        return value.y;
    case GyroscopeAxis::z:
        return value.z;
    }
    return 0.0;
}

[[nodiscard]] double maximumCrossAxisMagnitude(
    const RawVector3d& value,
    GyroscopeAxis selectedAxis) noexcept
{
    double maximum = 0.0;
    if (selectedAxis != GyroscopeAxis::x)
    {
        maximum = std::max(maximum, std::abs(value.x));
    }
    if (selectedAxis != GyroscopeAxis::y)
    {
        maximum = std::max(maximum, std::abs(value.y));
    }
    if (selectedAxis != GyroscopeAxis::z)
    {
        maximum = std::max(maximum, std::abs(value.z));
    }
    return maximum;
}

[[nodiscard]] GyroscopeAxis dominantAxis(const RawVector3d& peak) noexcept
{
    if (peak.x >= peak.y && peak.x >= peak.z)
    {
        return GyroscopeAxis::x;
    }
    if (peak.y >= peak.z)
    {
        return GyroscopeAxis::y;
    }
    return GyroscopeAxis::z;
}

void accumulateSignedTrapezoid(
    double previousRate,
    double currentRate,
    double deltaSeconds,
    double& positive,
    double& negative) noexcept
{
    if (previousRate >= 0.0 && currentRate >= 0.0)
    {
        positive += (previousRate + currentRate) * 0.5 * deltaSeconds;
        return;
    }
    if (previousRate <= 0.0 && currentRate <= 0.0)
    {
        negative += (previousRate + currentRate) * 0.5 * deltaSeconds;
        return;
    }

    const double magnitudeSum = std::abs(previousRate) + std::abs(currentRate);
    if (magnitudeSum == 0.0)
    {
        return;
    }
    const double crossingFraction = std::abs(previousRate) / magnitudeSum;
    const double firstDuration = deltaSeconds * crossingFraction;
    const double secondDuration = deltaSeconds - firstDuration;
    const double firstArea = previousRate * 0.5 * firstDuration;
    const double secondArea = currentRate * 0.5 * secondDuration;
    if (firstArea >= 0.0)
    {
        positive += firstArea;
    }
    else
    {
        negative += firstArea;
    }
    if (secondArea >= 0.0)
    {
        positive += secondArea;
    }
    else
    {
        negative += secondArea;
    }
}

[[nodiscard]] double median(std::vector<double> values)
{
    std::sort(values.begin(), values.end());
    const std::size_t middle = values.size() / 2U;
    if ((values.size() % 2U) == 0U)
    {
        return (values[middle - 1U] + values[middle]) * 0.5;
    }
    return values[middle];
}

[[nodiscard]] GyroscopeScaleStatistics calculateStatistics(const std::vector<double>& values)
{
    GyroscopeScaleStatistics result;
    if (values.empty())
    {
        return result;
    }
    result.minimum = *std::min_element(values.begin(), values.end());
    result.maximum = *std::max_element(values.begin(), values.end());
    result.median = median(values);
    for (const double value : values)
    {
        result.mean += value;
    }
    result.mean /= static_cast<double>(values.size());
    double sumSquared = 0.0;
    for (const double value : values)
    {
        const double difference = value - result.mean;
        sumSquared += difference * difference;
    }
    result.standardDeviation = std::sqrt(sumSquared / static_cast<double>(values.size()));
    result.coefficientOfVariation = result.mean > 0.0
        ? result.standardDeviation / result.mean
        : 0.0;
    result.relativeSpread = result.median > 0.0
        ? (result.maximum - result.minimum) / result.median
        : 0.0;
    return result;
}

[[nodiscard]] std::string escapeJson(const std::string& value)
{
    std::ostringstream output;
    for (const char character : value)
    {
        if (character == '"')
        {
            output << "\\\"";
        }
        else if (character == '\\')
        {
            output << "\\\\";
        }
        else if (character == '\n')
        {
            output << "\\n";
        }
        else
        {
            output << character;
        }
    }
    return output.str();
}

} // namespace

GyroscopeRawAngularIntegrator::GyroscopeRawAngularIntegrator(
    GyroscopeAxis axis,
    GyroscopeBias bias,
    std::chrono::nanoseconds maximumTimestampDelta)
    : axis_(axis), bias_(bias), maximumTimestampDelta_(maximumTimestampDelta)
{
    if (maximumTimestampDelta <= std::chrono::nanoseconds::zero())
    {
        throw std::invalid_argument("Maximum timestamp delta must be positive.");
    }
}

void GyroscopeRawAngularIntegrator::consume(const ImuSample& sample) noexcept
{
    if (failed())
    {
        return;
    }
    const double rate = rawAxisValue(sample.gyroscopeRaw, axis_) - biasAxisValue(bias_, axis_);
    peakCorrectedRawRate_ = std::max(peakCorrectedRawRate_, std::abs(rate));
    ++sampleCount_;
    if (!previousTimestamp_.has_value())
    {
        previousTimestamp_ = sample.deviceTimestamp.nanoseconds;
        previousRate_ = rate;
        return;
    }

    const auto delta = forwardDeviceTimestampDelta(
        sample.deviceTimestamp.nanoseconds,
        *previousTimestamp_);
    if (!delta.has_value())
    {
        rejectionReason_ = RawAngularIntegrationRejectionReason::invalidTimestamp;
        return;
    }
    if (*delta > static_cast<std::uint64_t>(maximumTimestampDelta_.count()))
    {
        rejectionReason_ = RawAngularIntegrationRejectionReason::timestampDeltaTooLarge;
        return;
    }

    const double deltaSeconds = static_cast<double>(*delta) / 1'000'000'000.0;
    const double area = (previousRate_ + rate) * 0.5 * deltaSeconds;
    integratedRawAngle_ += area;
    accumulateSignedTrapezoid(
        previousRate_, rate, deltaSeconds, positiveContribution_, negativeContribution_);
    accumulatedNanoseconds_ += *delta;
    previousTimestamp_ = sample.deviceTimestamp.nanoseconds;
    previousRate_ = rate;
}

bool GyroscopeRawAngularIntegrator::failed() const noexcept
{
    return rejectionReason_ != RawAngularIntegrationRejectionReason::none;
}

RawAngularIntegrationResult GyroscopeRawAngularIntegrator::finish() const noexcept
{
    RawAngularIntegrationRejectionReason reason = rejectionReason_;
    if (reason == RawAngularIntegrationRejectionReason::none && sampleCount_ < 2U)
    {
        reason = RawAngularIntegrationRejectionReason::insufficientSamples;
    }
    const double seconds = static_cast<double>(accumulatedNanoseconds_) / 1'000'000'000.0;
    return {
        reason == RawAngularIntegrationRejectionReason::none,
        reason,
        integratedRawAngle_,
        positiveContribution_,
        negativeContribution_,
        peakCorrectedRawRate_,
        seconds > 0.0 ? integratedRawAngle_ / seconds : 0.0,
        seconds > 0.0 && sampleCount_ > 1U
            ? static_cast<double>(sampleCount_ - 1U) / seconds
            : 0.0,
        std::chrono::nanoseconds(accumulatedNanoseconds_),
        sampleCount_,
    };
}

GyroscopeScaleTrialCalibrator::GyroscopeScaleTrialCalibrator(
    GyroscopeScaleCalibrationConfig configuration,
    std::optional<GyroscopeBias> bias)
    : configuration_(std::move(configuration)), bias_(bias)
{
    if (!std::isfinite(configuration_.expectedAngleDegrees)
        || configuration_.expectedAngleDegrees <= 0.0
        || !std::isfinite(configuration_.startThresholdRaw)
        || !std::isfinite(configuration_.stopThresholdRaw)
        || configuration_.startThresholdRaw <= configuration_.stopThresholdRaw
        || configuration_.stopThresholdRaw <= 0.0
        || configuration_.stillnessDuration <= std::chrono::nanoseconds::zero()
        || configuration_.minimumRotationDuration <= std::chrono::nanoseconds::zero()
        || configuration_.maximumRotationDuration <= configuration_.minimumRotationDuration
        || configuration_.maximumTimestampDelta <= std::chrono::nanoseconds::zero()
        || !std::isfinite(configuration_.maximumCrossAxisRatio)
        || configuration_.maximumCrossAxisRatio <= 0.0
        || configuration_.maximumCrossAxisRatio > 1.0)
    {
        throw std::invalid_argument("Gyroscope scale trial configuration is invalid.");
    }
    if (configuration_.acceptablePacketRate.has_value()
        && (configuration_.acceptablePacketRate->minimumPacketsPerSecond <= 0.0
            || configuration_.acceptablePacketRate->maximumPacketsPerSecond
                < configuration_.acceptablePacketRate->minimumPacketsPerSecond))
    {
        throw std::invalid_argument("Gyroscope scale trial packet-rate range is invalid.");
    }
    if (!bias_.has_value())
    {
        reject(GyroscopeScaleTrialRejectionReason::missingBias);
    }
}

void GyroscopeScaleTrialCalibrator::consume(const ImuSample& sample) noexcept
{
    if (isComplete())
    {
        return;
    }

    std::optional<std::uint64_t> delta;
    if (previousTimestamp_.has_value())
    {
        delta = forwardDeviceTimestampDelta(sample.deviceTimestamp.nanoseconds, *previousTimestamp_);
        if (!delta.has_value())
        {
            reject(GyroscopeScaleTrialRejectionReason::invalidTimestamp);
            return;
        }
        if (*delta > static_cast<std::uint64_t>(configuration_.maximumTimestampDelta.count()))
        {
            reject(GyroscopeScaleTrialRejectionReason::timestampDeltaTooLarge);
            return;
        }
    }
    previousTimestamp_ = sample.deviceTimestamp.nanoseconds;

    const RawVector3d corrected = correctedRawVector(sample.gyroscopeRaw, *bias_);
    peakCorrectedRawByAxis_.x = std::max(peakCorrectedRawByAxis_.x, std::abs(corrected.x));
    peakCorrectedRawByAxis_.y = std::max(peakCorrectedRawByAxis_.y, std::abs(corrected.y));
    peakCorrectedRawByAxis_.z = std::max(peakCorrectedRawByAxis_.z, std::abs(corrected.z));
    const double selected = vectorAxisValue(corrected, configuration_.axis);
    const double selectedMagnitude = std::abs(selected);
    const double crossMagnitude = maximumCrossAxisMagnitude(corrected, configuration_.axis);

    if (phase_ == GyroscopeScaleTrialPhase::waitingForRotation)
    {
        if (crossMagnitude >= configuration_.startThresholdRaw
            && selectedMagnitude < configuration_.startThresholdRaw)
        {
            reject(GyroscopeScaleTrialRejectionReason::wrongDominantAxis);
            return;
        }
        if (selectedMagnitude >= configuration_.startThresholdRaw)
        {
            if (crossMagnitude > selectedMagnitude * configuration_.maximumCrossAxisRatio)
            {
                reject(GyroscopeScaleTrialRejectionReason::excessiveCrossAxisMotion);
                return;
            }
            integrator_.emplace(
                configuration_.axis,
                *bias_,
                configuration_.maximumTimestampDelta);
            if (previousSample_.has_value())
            {
                integrator_->consume(*previousSample_);
            }
            integrator_->consume(sample);
            phase_ = GyroscopeScaleTrialPhase::rotating;
            rotationElapsedNanoseconds_ = 0U;
        }
        previousSample_ = sample;
        return;
    }

    integrator_->consume(sample);
    if (integrator_->failed())
    {
        const auto reason = integrator_->finish().rejectionReason;
        reject(reason == RawAngularIntegrationRejectionReason::timestampDeltaTooLarge
            ? GyroscopeScaleTrialRejectionReason::timestampDeltaTooLarge
            : GyroscopeScaleTrialRejectionReason::invalidTimestamp);
        return;
    }

    if (phase_ == GyroscopeScaleTrialPhase::rotating && delta.has_value())
    {
        rotationElapsedNanoseconds_ += *delta;
        if (rotationElapsedNanoseconds_
            > static_cast<std::uint64_t>(configuration_.maximumRotationDuration.count()))
        {
            reject(GyroscopeScaleTrialRejectionReason::rotationTooLong);
            return;
        }
        if (selectedMagnitude >= configuration_.startThresholdRaw
            && crossMagnitude > selectedMagnitude * configuration_.maximumCrossAxisRatio)
        {
            reject(GyroscopeScaleTrialRejectionReason::excessiveCrossAxisMotion);
            return;
        }
        if (crossMagnitude >= configuration_.startThresholdRaw
            && selectedMagnitude < configuration_.startThresholdRaw)
        {
            reject(GyroscopeScaleTrialRejectionReason::wrongDominantAxis);
            return;
        }
        if (selectedMagnitude <= configuration_.stopThresholdRaw
            && crossMagnitude <= configuration_.stopThresholdRaw)
        {
            phase_ = GyroscopeScaleTrialPhase::waitingForStillnessAfterRotation;
            stillnessElapsedNanoseconds_ = 0U;
        }
    }
    else if (phase_ == GyroscopeScaleTrialPhase::waitingForStillnessAfterRotation
             && delta.has_value())
    {
        if (selectedMagnitude <= configuration_.stopThresholdRaw
            && crossMagnitude <= configuration_.stopThresholdRaw)
        {
            stillnessElapsedNanoseconds_ += *delta;
            if (stillnessElapsedNanoseconds_
                >= static_cast<std::uint64_t>(configuration_.stillnessDuration.count()))
            {
                complete();
            }
        }
        else if (selectedMagnitude >= configuration_.startThresholdRaw
                 && crossMagnitude <= selectedMagnitude * configuration_.maximumCrossAxisRatio)
        {
            phase_ = GyroscopeScaleTrialPhase::rotating;
            stillnessElapsedNanoseconds_ = 0U;
        }
        else
        {
            reject(GyroscopeScaleTrialRejectionReason::unstableStop);
        }
    }
    previousSample_ = sample;
}

GyroscopeScaleTrialPhase GyroscopeScaleTrialCalibrator::phase() const noexcept
{
    return phase_;
}

bool GyroscopeScaleTrialCalibrator::isComplete() const noexcept
{
    return phase_ == GyroscopeScaleTrialPhase::completed
        || phase_ == GyroscopeScaleTrialPhase::rejected;
}

std::optional<GyroscopeScaleCalibrationTrial> GyroscopeScaleTrialCalibrator::result() const noexcept
{
    return result_;
}

GyroscopeScaleCalibrationTrial GyroscopeScaleTrialCalibrator::finish() noexcept
{
    if (!result_.has_value())
    {
        reject(GyroscopeScaleTrialRejectionReason::insufficientSamples);
    }
    return *result_;
}

void GyroscopeScaleTrialCalibrator::reject(GyroscopeScaleTrialRejectionReason reason) noexcept
{
    phase_ = GyroscopeScaleTrialPhase::rejected;
    result_ = GyroscopeScaleCalibrationTrial{
        false,
        reason,
        RotationDirection::automatic,
        dominantAxis(peakCorrectedRawByAxis_),
        configuration_.expectedAngleDegrees,
        0.0,
        integrator_.has_value() ? integrator_->finish() : RawAngularIntegrationResult{},
        peakCorrectedRawByAxis_,
    };
}

void GyroscopeScaleTrialCalibrator::complete() noexcept
{
    const auto integration = integrator_->finish();
    if (!integration.valid)
    {
        reject(GyroscopeScaleTrialRejectionReason::insufficientSamples);
        return;
    }
    if (rotationElapsedNanoseconds_
        < static_cast<std::uint64_t>(configuration_.minimumRotationDuration.count()))
    {
        reject(GyroscopeScaleTrialRejectionReason::rotationTooShort);
        return;
    }
    if (configuration_.acceptablePacketRate.has_value()
        && (integration.packetRate
                < configuration_.acceptablePacketRate->minimumPacketsPerSecond
            || integration.packetRate
                > configuration_.acceptablePacketRate->maximumPacketsPerSecond))
    {
        reject(GyroscopeScaleTrialRejectionReason::packetRateOutOfRange);
        return;
    }
    const RotationDirection measuredDirection = integration.integratedRawAngle >= 0.0
        ? RotationDirection::positive
        : RotationDirection::negative;
    if (configuration_.direction != RotationDirection::automatic
        && configuration_.direction != measuredDirection)
    {
        reject(GyroscopeScaleTrialRejectionReason::directionMismatch);
        return;
    }
    const GyroscopeAxis measuredDominantAxis = dominantAxis(peakCorrectedRawByAxis_);
    if (measuredDominantAxis != configuration_.axis)
    {
        reject(GyroscopeScaleTrialRejectionReason::wrongDominantAxis);
        return;
    }
    const double scale = std::abs(integration.integratedRawAngle)
        / configuration_.expectedAngleDegrees;
    if (!std::isfinite(scale) || scale <= 0.0)
    {
        reject(GyroscopeScaleTrialRejectionReason::insufficientSamples);
        return;
    }

    phase_ = GyroscopeScaleTrialPhase::completed;
    result_ = GyroscopeScaleCalibrationTrial{
        true,
        GyroscopeScaleTrialRejectionReason::none,
        measuredDirection,
        measuredDominantAxis,
        configuration_.expectedAngleDegrees,
        scale,
        integration,
        peakCorrectedRawByAxis_,
    };
}

GyroscopeScaleCalibrationResult calculateGyroscopeScaleCalibration(
    GyroscopeScaleCalibrationConfig configuration,
    std::span<const GyroscopeScaleCalibrationTrial> trials)
{
    GyroscopeScaleCalibrationResult result;
    result.configuration = configuration;
    result.trials.assign(trials.begin(), trials.end());

    std::vector<double> candidates;
    for (const auto& trial : trials)
    {
        if (trial.accepted && std::isfinite(trial.scaleRawPerDegreePerSecond)
            && trial.scaleRawPerDegreePerSecond > 0.0)
        {
            candidates.push_back(trial.scaleRawPerDegreePerSecond);
        }
    }
    if (candidates.empty())
    {
        result.rejectionReason = GyroscopeScaleCalibrationRejectionReason::tooFewAcceptedTrials;
        result.rejectedTrialCount = trials.size();
        return result;
    }

    const double initialMedian = median(candidates);
    std::vector<double> retained;
    std::vector<double> positive;
    std::vector<double> negative;
    for (std::size_t index = 0; index < trials.size(); ++index)
    {
        const auto& trial = trials[index];
        if (!trial.accepted || !std::isfinite(trial.scaleRawPerDegreePerSecond)
            || trial.scaleRawPerDegreePerSecond <= 0.0)
        {
            continue;
        }
        const double deviationPercent = std::abs(trial.scaleRawPerDegreePerSecond - initialMedian)
            / initialMedian * 100.0;
        if (deviationPercent <= configuration.outlierDeviationPercent)
        {
            retained.push_back(trial.scaleRawPerDegreePerSecond);
            if (trial.measuredDirection == RotationDirection::positive)
            {
                positive.push_back(trial.scaleRawPerDegreePerSecond);
            }
            else if (trial.measuredDirection == RotationDirection::negative)
            {
                negative.push_back(trial.scaleRawPerDegreePerSecond);
            }
        }
        else
        {
            result.trials[index].accepted = false;
            result.trials[index].rejectionReason =
                GyroscopeScaleTrialRejectionReason::scaleOutlier;
        }
    }

    result.acceptedTrialCount = retained.size();
    result.rejectedTrialCount = trials.size() - retained.size();
    if (retained.size() < configuration.minimumAcceptedTrials)
    {
        result.rejectionReason = GyroscopeScaleCalibrationRejectionReason::tooFewAcceptedTrials;
        return result;
    }

    result.statistics = calculateStatistics(retained);
    if (result.statistics.coefficientOfVariation * 100.0
        > configuration.maximumTrialVariationPercent)
    {
        result.rejectionReason = GyroscopeScaleCalibrationRejectionReason::excessiveTrialVariation;
        return result;
    }
    if (!positive.empty() && !negative.empty())
    {
        const double positiveMedian = median(positive);
        const double negativeMedian = median(negative);
        const double disagreement = std::abs(positiveMedian - negativeMedian)
            / result.statistics.median * 100.0;
        if (disagreement > configuration.maximumDirectionDisagreementPercent)
        {
            result.rejectionReason = GyroscopeScaleCalibrationRejectionReason::directionDisagreement;
            return result;
        }
    }

    const double rawPerDegree = result.statistics.median;
    const double rawPerRadian = rawPerDegree * 180.0 / std::numbers::pi;
    if (!std::isfinite(rawPerDegree) || !std::isfinite(rawPerRadian)
        || rawPerDegree <= 0.0 || rawPerRadian <= 0.0)
    {
        result.rejectionReason = GyroscopeScaleCalibrationRejectionReason::invalidScale;
        return result;
    }
    result.scale = {
        true,
        configuration.axis,
        rawPerDegree,
        rawPerRadian,
        1.0 / rawPerDegree,
        1.0 / rawPerRadian,
        retained.size(),
        result.statistics.standardDeviation,
    };
    result.accepted = true;
    result.rejectionReason = GyroscopeScaleCalibrationRejectionReason::none;
    return result;
}

std::optional<GyroscopeAngularVelocity> applyGyroscopeScale(
    GyroscopeAxis axis,
    double biasCorrectedRaw,
    const GyroscopeAxisScale& scale) noexcept
{
    if (!scale.valid || scale.axis != axis || !std::isfinite(biasCorrectedRaw)
        || !std::isfinite(scale.rawUnitsPerDegreePerSecond)
        || !std::isfinite(scale.rawUnitsPerRadianPerSecond)
        || scale.rawUnitsPerDegreePerSecond <= 0.0
        || scale.rawUnitsPerRadianPerSecond <= 0.0)
    {
        return std::nullopt;
    }
    return GyroscopeAngularVelocity{
        biasCorrectedRaw / scale.rawUnitsPerDegreePerSecond,
        biasCorrectedRaw / scale.rawUnitsPerRadianPerSecond,
    };
}

std::string gyroscopeAxisText(GyroscopeAxis axis)
{
    switch (axis)
    {
    case GyroscopeAxis::x: return "x";
    case GyroscopeAxis::y: return "y";
    case GyroscopeAxis::z: return "z";
    }
    return "unknown";
}

std::string rotationDirectionText(RotationDirection direction)
{
    switch (direction)
    {
    case RotationDirection::positive: return "positive";
    case RotationDirection::negative: return "negative";
    case RotationDirection::automatic: return "auto";
    }
    return "unknown";
}

std::string gyroscopeScaleTrialRejectionReasonText(GyroscopeScaleTrialRejectionReason reason)
{
    switch (reason)
    {
    case GyroscopeScaleTrialRejectionReason::none: return {};
    case GyroscopeScaleTrialRejectionReason::missingBias: return "runtime gyroscope bias was not accepted";
    case GyroscopeScaleTrialRejectionReason::invalidTimestamp: return "device timestamp was duplicate or decreasing";
    case GyroscopeScaleTrialRejectionReason::timestampDeltaTooLarge: return "device timestamp delta was unreasonably large";
    case GyroscopeScaleTrialRejectionReason::wrongDominantAxis: return "motion began on the wrong dominant axis";
    case GyroscopeScaleTrialRejectionReason::excessiveCrossAxisMotion: return "cross-axis motion exceeded the configured ratio";
    case GyroscopeScaleTrialRejectionReason::rotationTooShort: return "rotation duration was too short";
    case GyroscopeScaleTrialRejectionReason::rotationTooLong: return "rotation duration was too long";
    case GyroscopeScaleTrialRejectionReason::directionMismatch: return "integrated rotation direction did not match the request";
    case GyroscopeScaleTrialRejectionReason::unstableStop: return "motion did not return cleanly to stillness";
    case GyroscopeScaleTrialRejectionReason::insufficientSamples: return "rotation contained insufficient valid samples";
    case GyroscopeScaleTrialRejectionReason::packetRateOutOfRange: return "rotation packet rate was outside the configured range";
    case GyroscopeScaleTrialRejectionReason::streamDataLoss: return "the acquisition stream lost data";
    case GyroscopeScaleTrialRejectionReason::scaleOutlier: return "trial scale was rejected as a median-relative outlier";
    }
    return "unknown trial rejection";
}

std::string gyroscopeScaleCalibrationRejectionReasonText(
    GyroscopeScaleCalibrationRejectionReason reason)
{
    switch (reason)
    {
    case GyroscopeScaleCalibrationRejectionReason::none: return {};
    case GyroscopeScaleCalibrationRejectionReason::tooFewAcceptedTrials: return "too few trials remained after rejection";
    case GyroscopeScaleCalibrationRejectionReason::excessiveTrialVariation: return "trial-to-trial coefficient of variation was excessive";
    case GyroscopeScaleCalibrationRejectionReason::directionDisagreement: return "positive and negative direction estimates disagreed";
    case GyroscopeScaleCalibrationRejectionReason::invalidScale: return "the aggregated scale was invalid";
    }
    return "unknown calibration rejection";
}

std::string serializeGyroscopeScaleCalibrationJson(
    const GyroscopeBiasCalibrationDevice& device,
    const GyroscopeScaleCalibrationResult& result)
{
    std::ostringstream output;
    output << std::setprecision(std::numeric_limits<double>::max_digits10)
           << "{\n  \"schema_version\": 1,\n  \"units\": \"raw_and_angular_rate\",\n"
           << "  \"device\": {\"vendor_id\":\"0x" << std::hex << std::uppercase
           << std::setw(4) << std::setfill('0') << device.vendorId
           << "\",\"product_id\":\"0x" << std::setw(4) << device.productId
           << std::dec << std::nouppercase << std::setfill(' ')
           << "\",\"interface_number\":" << device.interfaceNumber
           << ",\"product\":\"" << escapeJson(device.productName) << "\"},\n"
           << "  \"calibration\": {\"type\":\"gyroscope-scale\",\"axis\":\""
           << gyroscopeAxisText(result.configuration.axis)
           << "\",\"accepted\":" << (result.accepted ? "true" : "false")
           << ",\"rejection_reason\":\""
           << escapeJson(gyroscopeScaleCalibrationRejectionReasonText(result.rejectionReason))
           << "\",\"expected_angle_degrees\":" << result.configuration.expectedAngleDegrees
           << ",\"accepted_trials\":" << result.acceptedTrialCount
           << ",\"rejected_trials\":" << result.rejectedTrialCount
           << ",\"scale\":{\"valid\":" << (result.scale.valid ? "true" : "false")
           << ",\"raw_units_per_degree_per_second\":" << result.scale.rawUnitsPerDegreePerSecond
           << ",\"raw_units_per_radian_per_second\":" << result.scale.rawUnitsPerRadianPerSecond
           << ",\"degrees_per_second_per_raw_unit\":" << result.scale.degreesPerSecondPerRawUnit
           << ",\"radians_per_second_per_raw_unit\":" << result.scale.radiansPerSecondPerRawUnit
           << "}},\n  \"statistics\": {\"mean\":" << result.statistics.mean
           << ",\"median\":" << result.statistics.median
           << ",\"stddev\":" << result.statistics.standardDeviation
           << ",\"coefficient_of_variation\":" << result.statistics.coefficientOfVariation
           << ",\"minimum\":" << result.statistics.minimum
           << ",\"maximum\":" << result.statistics.maximum
           << ",\"relative_spread\":" << result.statistics.relativeSpread << "},\n"
           << "  \"trials\": [\n";
    for (std::size_t index = 0; index < result.trials.size(); ++index)
    {
        const auto& trial = result.trials[index];
        output << "    {\"accepted\":" << (trial.accepted ? "true" : "false")
               << ",\"rejection_reason\":\""
               << escapeJson(gyroscopeScaleTrialRejectionReasonText(trial.rejectionReason))
               << "\",\"direction\":\"" << rotationDirectionText(trial.measuredDirection)
               << "\",\"duration_seconds\":"
               << std::chrono::duration<double>(trial.integration.captureDuration).count()
               << ",\"integrated_raw_angle\":" << trial.integration.integratedRawAngle
               << ",\"positive_contribution\":" << trial.integration.positiveContribution
               << ",\"negative_contribution\":" << trial.integration.negativeContribution
               << ",\"packet_rate\":" << trial.integration.packetRate
               << ",\"scale_raw_per_degree_per_second\":" << trial.scaleRawPerDegreePerSecond
               << ",\"dominant_axis\":\"" << gyroscopeAxisText(trial.dominantAxis)
               << "\",\"peak_corrected_raw\":" << trial.integration.peakCorrectedRawRate << '}';
        if (index + 1U != result.trials.size())
        {
            output << ',';
        }
        output << '\n';
    }
    output << "  ],\n  \"configuration\": {\"start_threshold_raw\":"
           << result.configuration.startThresholdRaw
           << ",\"stop_threshold_raw\":" << result.configuration.stopThresholdRaw
           << ",\"stillness_duration_ns\":" << result.configuration.stillnessDuration.count()
           << ",\"minimum_rotation_duration_ns\":"
           << result.configuration.minimumRotationDuration.count()
           << ",\"maximum_rotation_duration_ns\":"
           << result.configuration.maximumRotationDuration.count()
           << ",\"maximum_timestamp_delta_ns\":"
           << result.configuration.maximumTimestampDelta.count()
           << ",\"maximum_cross_axis_ratio\":" << result.configuration.maximumCrossAxisRatio
           << ",\"maximum_trial_variation_percent\":"
           << result.configuration.maximumTrialVariationPercent
           << ",\"minimum_accepted_trials\":" << result.configuration.minimumAcceptedTrials
           << "}\n}\n";
    return output.str();
}

} // namespace xreal::sensors
