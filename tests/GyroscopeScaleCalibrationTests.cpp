#include "sensors/GyroscopeScaleCalibration.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <numbers>
#include <string>
#include <string_view>
#include <vector>

namespace
{

int failureCount{};

void expect(bool condition, std::string_view description)
{
    if (!condition)
    {
        ++failureCount;
        std::cerr << "FAILED: " << description << '\n';
    }
}

[[nodiscard]] bool near(double actual, double expected, double tolerance = 1.0e-9)
{
    return std::abs(actual - expected) <= tolerance;
}

[[nodiscard]] xreal::sensors::ImuSample sample(
    std::uint64_t timestamp,
    std::int32_t x,
    std::int32_t y,
    std::int32_t z)
{
    xreal::sensors::ImuSample result;
    result.deviceTimestamp.nanoseconds = timestamp;
    result.gyroscopeRaw = {x, y, z};
    return result;
}

void testRawIntegration()
{
    using namespace std::chrono_literals;
    xreal::sensors::GyroscopeRawAngularIntegrator constant(
        xreal::sensors::GyroscopeAxis::z, {0.0, 0.0, 100.0}, 2s);
    constant.consume(sample(0, 0, 0, 1100));
    constant.consume(sample(1'000'000'000, 0, 0, 1100));
    const auto constantResult = constant.finish();
    expect(constantResult.valid && near(constantResult.integratedRawAngle, 1000.0),
           "constant bias-corrected rate integrates by device time");
    expect(near(constantResult.meanCorrectedRawRate, 1000.0), "mean corrected rate is time based");

    xreal::sensors::GyroscopeRawAngularIntegrator changing(
        xreal::sensors::GyroscopeAxis::x, {}, 2s);
    changing.consume(sample(0, 0, 0, 0));
    changing.consume(sample(250'000'000, 100, 0, 0));
    changing.consume(sample(1'000'000'000, 200, 0, 0));
    const auto changingResult = changing.finish();
    expect(near(changingResult.integratedRawAngle, 125.0),
           "changing rates use trapezoids and irregular device intervals");
    expect(changingResult.positiveContribution > 0.0 && changingResult.negativeContribution == 0.0,
           "positive contribution remains signed and separate");

    xreal::sensors::GyroscopeRawAngularIntegrator negative(
        xreal::sensors::GyroscopeAxis::y, {}, 2s);
    negative.consume(sample(0, 0, -50, 0));
    negative.consume(sample(1'000'000'000, 0, -50, 0));
    expect(near(negative.finish().integratedRawAngle, -50.0),
           "negative rotation produces a negative integrated raw angle");
}

void testIntegrationTimestampValidationAndPreservation()
{
    using namespace std::chrono_literals;
    const auto original = sample(100, 1, 2, 3);
    xreal::sensors::GyroscopeRawAngularIntegrator duplicate(
        xreal::sensors::GyroscopeAxis::x, {1.0, 0.0, 0.0}, 20ms);
    duplicate.consume(original);
    duplicate.consume(sample(100, 1, 2, 3));
    expect(!duplicate.finish().valid, "duplicate timestamp is rejected");
    expect(original.gyroscopeRaw.x == 1, "integration does not mutate the original sample");

    xreal::sensors::GyroscopeRawAngularIntegrator decreasing(
        xreal::sensors::GyroscopeAxis::x, {}, 20ms);
    decreasing.consume(sample(200, 1, 0, 0));
    decreasing.consume(sample(100, 1, 0, 0));
    expect(!decreasing.finish().valid, "decreasing timestamp is rejected");

    xreal::sensors::GyroscopeRawAngularIntegrator largeDelta(
        xreal::sensors::GyroscopeAxis::x, {}, 20ms);
    largeDelta.consume(sample(0, 1, 0, 0));
    largeDelta.consume(sample(21'000'000, 1, 0, 0));
    expect(largeDelta.finish().rejectionReason
               == xreal::sensors::RawAngularIntegrationRejectionReason::timestampDeltaTooLarge,
           "unreasonable timestamp delta is rejected");

    xreal::sensors::GyroscopeRawAngularIntegrator wrapped(
        xreal::sensors::GyroscopeAxis::x, {}, 2ms);
    wrapped.consume(sample(std::numeric_limits<std::uint64_t>::max() - 499'999U, 100, 0, 0));
    wrapped.consume(sample(500'000U, 100, 0, 0));
    expect(wrapped.finish().valid, "uint64 timestamp wraparound follows the shared policy");
}

[[nodiscard]] xreal::sensors::GyroscopeScaleCalibrationConfig trialConfiguration(double degrees)
{
    using namespace std::chrono_literals;
    xreal::sensors::GyroscopeScaleCalibrationConfig configuration;
    configuration.axis = xreal::sensors::GyroscopeAxis::z;
    configuration.expectedAngleDegrees = degrees;
    configuration.startThresholdRaw = 100.0;
    configuration.stopThresholdRaw = 20.0;
    configuration.stillnessDuration = 20ms;
    configuration.minimumRotationDuration = 100ms;
    configuration.maximumRotationDuration = 5s;
    configuration.maximumTimestampDelta = 20ms;
    configuration.acceptablePacketRate = xreal::sensors::PacketRateRange{80.0, 120.0};
    configuration.maximumCrossAxisRatio = 0.3;
    configuration.minimumAcceptedTrials = 3;
    return configuration;
}

[[nodiscard]] xreal::sensors::GyroscopeScaleCalibrationTrial syntheticRotation(
    double degrees,
    int sign,
    xreal::sensors::RotationDirection requested = xreal::sensors::RotationDirection::automatic)
{
    auto configuration = trialConfiguration(degrees);
    configuration.direction = requested;
    const std::uint64_t activeMilliseconds = degrees == 90.0 ? 1000U : 2000U;
    const std::int32_t rate = static_cast<std::int32_t>(10.0 * degrees
        / (static_cast<double>(activeMilliseconds) / 1000.0)) * sign;
    xreal::sensors::GyroscopeScaleTrialCalibrator calibrator(configuration, xreal::sensors::GyroscopeBias{});
    calibrator.consume(sample(0, 0, 0, 0));
    for (std::uint64_t milliseconds = 10; milliseconds <= activeMilliseconds; milliseconds += 10)
    {
        calibrator.consume(sample(milliseconds * 1'000'000U, 0, 0, rate));
    }
    calibrator.consume(sample((activeMilliseconds + 10U) * 1'000'000U, 0, 0, 0));
    calibrator.consume(sample((activeMilliseconds + 20U) * 1'000'000U, 0, 0, 0));
    calibrator.consume(sample((activeMilliseconds + 30U) * 1'000'000U, 0, 0, 0));
    return calibrator.finish();
}

void testTrialDetectionAndKnownScales()
{
    const auto fullPositive = syntheticRotation(360.0, 1);
    expect(fullPositive.accepted, "start and stop thresholds detect a complete positive rotation");
    expect(fullPositive.measuredDirection == xreal::sensors::RotationDirection::positive,
           "positive direction is retained");
    expect(near(fullPositive.scaleRawPerDegreePerSecond, 10.0, 0.01),
           "synthetic 360-degree rotation recovers its scale");

    const auto quarterNegative = syntheticRotation(90.0, -1);
    expect(quarterNegative.accepted
               && quarterNegative.measuredDirection == xreal::sensors::RotationDirection::negative,
           "negative 90-degree rotation is accepted");
    expect(near(quarterNegative.scaleRawPerDegreePerSecond, 10.0, 0.01),
           "synthetic 90-degree rotation recovers the same scale magnitude");

    const auto mismatch = syntheticRotation(
        90.0, -1, xreal::sensors::RotationDirection::positive);
    expect(!mismatch.accepted
               && mismatch.rejectionReason
                   == xreal::sensors::GyroscopeScaleTrialRejectionReason::directionMismatch,
           "requested direction mismatch is rejected");
}

void testTrialRejections()
{
    using namespace std::chrono_literals;
    auto configuration = trialConfiguration(360.0);
    xreal::sensors::GyroscopeScaleTrialCalibrator wrongAxis(configuration, xreal::sensors::GyroscopeBias{});
    wrongAxis.consume(sample(0, 0, 0, 0));
    wrongAxis.consume(sample(10'000'000, 200, 0, 10));
    expect(wrongAxis.finish().rejectionReason
               == xreal::sensors::GyroscopeScaleTrialRejectionReason::wrongDominantAxis,
           "wrong dominant axis is rejected");

    xreal::sensors::GyroscopeScaleTrialCalibrator crossAxis(configuration, xreal::sensors::GyroscopeBias{});
    crossAxis.consume(sample(0, 0, 0, 0));
    crossAxis.consume(sample(10'000'000, 40, 0, 100));
    expect(crossAxis.finish().rejectionReason
               == xreal::sensors::GyroscopeScaleTrialRejectionReason::excessiveCrossAxisMotion,
           "excessive cross-axis ratio is rejected");

    configuration.minimumRotationDuration = 2s;
    const auto shortTrial = [&]() {
        xreal::sensors::GyroscopeScaleTrialCalibrator trial(configuration, xreal::sensors::GyroscopeBias{});
        trial.consume(sample(0, 0, 0, 0));
        for (std::uint64_t index = 1; index <= 50; ++index)
        {
            trial.consume(sample(index * 10'000'000U, 0, 0, 1000));
        }
        trial.consume(sample(510'000'000, 0, 0, 0));
        trial.consume(sample(520'000'000, 0, 0, 0));
        trial.consume(sample(530'000'000, 0, 0, 0));
        return trial.finish();
    }();
    expect(shortTrial.rejectionReason
               == xreal::sensors::GyroscopeScaleTrialRejectionReason::rotationTooShort,
           "too-short rotation is rejected after stillness is enforced");

    configuration = trialConfiguration(360.0);
    configuration.minimumRotationDuration = 50ms;
    configuration.maximumRotationDuration = 100ms;
    xreal::sensors::GyroscopeScaleTrialCalibrator longTrial(configuration, xreal::sensors::GyroscopeBias{});
    longTrial.consume(sample(0, 0, 0, 0));
    for (std::uint64_t index = 1; index <= 12; ++index)
    {
        longTrial.consume(sample(index * 10'000'000U, 0, 0, 1000));
    }
    expect(longTrial.finish().rejectionReason
               == xreal::sensors::GyroscopeScaleTrialRejectionReason::rotationTooLong,
           "too-long rotation is rejected");
}

[[nodiscard]] xreal::sensors::GyroscopeScaleCalibrationTrial trialScale(
    double scale,
    xreal::sensors::RotationDirection direction = xreal::sensors::RotationDirection::positive)
{
    xreal::sensors::GyroscopeScaleCalibrationTrial trial;
    trial.accepted = true;
    trial.measuredDirection = direction;
    trial.dominantAxis = xreal::sensors::GyroscopeAxis::z;
    trial.expectedAngleDegrees = 360.0;
    trial.scaleRawPerDegreePerSecond = scale;
    trial.integration.valid = true;
    return trial;
}

void testRobustAggregationAndConversion()
{
    auto configuration = trialConfiguration(360.0);
    configuration.minimumAcceptedTrials = 4;
    const std::vector trials{
        trialScale(10.0), trialScale(10.1), trialScale(9.9),
        trialScale(10.05, xreal::sensors::RotationDirection::negative),
        trialScale(100.0)};
    const auto result = xreal::sensors::calculateGyroscopeScaleCalibration(configuration, trials);
    expect(result.accepted && result.acceptedTrialCount == 4 && result.rejectedTrialCount == 1,
           "median-relative outlier rejection retains consistent trials");
    expect(!result.trials.back().accepted
               && result.trials.back().rejectionReason
                   == xreal::sensors::GyroscopeScaleTrialRejectionReason::scaleOutlier,
           "outlier trials retain an explicit serialized rejection reason");
    expect(near(result.scale.rawUnitsPerDegreePerSecond, 10.025, 0.001),
           "final scale uses the retained median");
    const auto velocity = xreal::sensors::applyGyroscopeScale(
        xreal::sensors::GyroscopeAxis::z,
        100.25,
        result.scale);
    expect(velocity.has_value() && near(velocity->degreesPerSecond, 10.0, 0.001),
           "valid scale converts corrected raw values to degrees per second");
    expect(near(velocity->radiansPerSecond, 10.0 * std::numbers::pi / 180.0, 0.001),
           "valid scale converts corrected raw values to radians per second");

    auto invalidScale = result.scale;
    invalidScale.rawUnitsPerDegreePerSecond = 0.0;
    expect(!xreal::sensors::applyGyroscopeScale(
                xreal::sensors::GyroscopeAxis::z, 1.0, invalidScale).has_value(),
           "zero scale is rejected");
    invalidScale.rawUnitsPerDegreePerSecond = -1.0;
    expect(!xreal::sensors::applyGyroscopeScale(
                xreal::sensors::GyroscopeAxis::z, 1.0, invalidScale).has_value(),
           "negative scale is rejected");
    invalidScale.rawUnitsPerDegreePerSecond = std::numeric_limits<double>::infinity();
    expect(!xreal::sensors::applyGyroscopeScale(
                xreal::sensors::GyroscopeAxis::z, 1.0, invalidScale).has_value(),
           "non-finite scale is rejected");
    invalidScale = result.scale;
    expect(!xreal::sensors::applyGyroscopeScale(
                xreal::sensors::GyroscopeAxis::x, 1.0, invalidScale).has_value(),
           "a scale is not applied to an uncalibrated axis");
}

void testFinalCalibrationRejectionsAndJson()
{
    auto configuration = trialConfiguration(360.0);
    configuration.minimumAcceptedTrials = 4;
    configuration.outlierDeviationPercent = 100.0;
    const std::vector variable{trialScale(8.0), trialScale(9.0), trialScale(11.0), trialScale(12.0)};
    const auto variableResult = xreal::sensors::calculateGyroscopeScaleCalibration(configuration, variable);
    expect(variableResult.rejectionReason
               == xreal::sensors::GyroscopeScaleCalibrationRejectionReason::excessiveTrialVariation,
           "excessive repeated-trial variation rejects final calibration");

    const std::vector tooFew{trialScale(10.0), trialScale(10.1)};
    expect(xreal::sensors::calculateGyroscopeScaleCalibration(configuration, tooFew).rejectionReason
               == xreal::sensors::GyroscopeScaleCalibrationRejectionReason::tooFewAcceptedTrials,
           "too few accepted trials reject final calibration");

    configuration.maximumTrialVariationPercent = 100.0;
    configuration.maximumDirectionDisagreementPercent = 10.0;
    const std::vector directions{
        trialScale(10.0), trialScale(10.0),
        trialScale(14.0, xreal::sensors::RotationDirection::negative),
        trialScale(14.0, xreal::sensors::RotationDirection::negative)};
    const auto directionResult = xreal::sensors::calculateGyroscopeScaleCalibration(configuration, directions);
    expect(directionResult.rejectionReason
               == xreal::sensors::GyroscopeScaleCalibrationRejectionReason::directionDisagreement,
           "positive and negative direction disagreement is rejected");

    const xreal::sensors::GyroscopeBiasCalibrationDevice device{0x3318, 0x0426, 2, "XREAL Air 2 Ultra"};
    const std::string rejectedJson = xreal::sensors::serializeGyroscopeScaleCalibrationJson(device, variableResult);
    expect(rejectedJson.starts_with("{") && rejectedJson.ends_with("}\n")
               && rejectedJson.find("\"accepted\":false") != std::string::npos
               && rejectedJson.find("trial-to-trial") != std::string::npos,
           "rejected JSON is complete and contains its reason");

    configuration = trialConfiguration(360.0);
    configuration.minimumAcceptedTrials = 3;
    const std::vector accepted{trialScale(10.0), trialScale(10.1), trialScale(9.9)};
    const auto acceptedResult = xreal::sensors::calculateGyroscopeScaleCalibration(configuration, accepted);
    const std::string acceptedJson = xreal::sensors::serializeGyroscopeScaleCalibrationJson(device, acceptedResult);
    expect(acceptedJson.find("\"accepted\":true") != std::string::npos
               && acceptedJson.find("\"raw_units_per_radian_per_second\":") != std::string::npos
               && acceptedJson.find("\"trials\": [") != std::string::npos,
           "accepted JSON contains a valid scale and formatted trials array");
}

} // namespace

int main()
{
    testRawIntegration();
    testIntegrationTimestampValidationAndPreservation();
    testTrialDetectionAndKnownScales();
    testTrialRejections();
    testRobustAggregationAndConversion();
    testFinalCalibrationRejectionsAndJson();

    if (failureCount != 0)
    {
        std::cerr << failureCount << " gyroscope scale calibration test(s) failed.\n";
        return 1;
    }
    std::cout << "All gyroscope scale calibration tests passed.\n";
    return 0;
}
