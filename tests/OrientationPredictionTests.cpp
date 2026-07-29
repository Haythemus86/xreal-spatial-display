#include "sensors/OrientationPrediction.hpp"
#include "sensors/OrientationPredictionEvaluation.hpp"
#include "JsonSyntaxParser.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <numbers>
#include <string>

namespace
{
int failures{};
void expect(bool condition, const char* message)
{
    if (!condition) { std::cerr << "FAILED: " << message << '\n'; ++failures; }
}
[[nodiscard]] bool near(double a, double b, double tolerance = 1.0e-8)
{
    return std::abs(a - b) <= tolerance;
}
[[nodiscard]] xreal::sensors::Quaternion rotation(double x, double y, double z, double degrees)
{
    const double radians = degrees * std::numbers::pi / 180.0;
    const double sine = std::sin(radians * 0.5);
    return {std::cos(radians * 0.5), x * sine, y * sine, z * sine};
}
[[nodiscard]] xreal::sensors::OrientationPredictorInput input(
    std::uint64_t timestamp, double radiansPerSecond,
    xreal::sensors::Quaternion orientation = {})
{
    return {{orientation}, {{}, false}, {0.0, 0.0, radiansPerSecond}, timestamp};
}

void testExponentialMapAndConstantVelocity()
{
    using namespace xreal::sensors;
    const auto identityDelta = quaternionFromBodyRotationVector({});
    expect(identityDelta && near(identityDelta->w, 1.0), "zero rotation maps to identity");
    const auto tiny = quaternionFromBodyRotationVector({1.0e-12, 0, 0});
    expect(tiny && tiny->finite() && near(tiny->norm(), 1.0), "small-angle map stays normalized");

    OrientationPredictorConfig config;
    config.horizon = std::chrono::milliseconds(10);
    OrientationPredictor predictor(config);
    const auto zero = predictor.predict(input(0, 0.0));
    expect(zero.validity == PredictionValidity::valid
               && near(quaternionAngularDistance({}, zero.absolute.value)->degrees, 0.0),
           "zero angular velocity preserves orientation");
    const auto result = predictor.predict(input(1'000'000, std::numbers::pi));
    expect(result.validity == PredictionValidity::valid
               && near(quaternionAngularDistance({}, result.absolute.value)->degrees, 1.8, 1.0e-6),
           "constant velocity predicts omega times horizon");
    OrientationPredictorConfig zeroConfig;
    zeroConfig.horizon = std::chrono::duration<double>::zero();
    OrientationPredictor zeroPredictor(zeroConfig);
    const auto measured = rotation(1, 0, 0, 37.0);
    const auto zeroHorizon = zeroPredictor.predict(input(0, 5.0, measured));
    expect(near(quaternionAngularDistance(measured, zeroHorizon.absolute.value)->degrees, 0.0),
           "zero horizon preserves measured orientation");
    expect(near(result.absolute.value.norm(), 1.0), "prediction output is normalized");
}

void testFramesAndRecenter()
{
    using namespace xreal::sensors;
    OrientationPredictor predictor;
    auto value = input(0, std::numbers::pi);
    value.measuredAbsolute = {rotation(1, 0, 0, 30)};
    value.recenterReference = {rotation(1, 0, 0, 30), true};
    const auto result = predictor.predict(value);
    const auto expected = (value.measuredAbsolute.value * rotation(0, 0, 1, 1.8)).normalized();
    expect(expected && near(quaternionAngularDistance(*expected, result.absolute.value)->degrees, 0.0),
           "body delta is multiplied on the right");
    const auto independentlyRelative = makeRelativeOrientation({result.absolute.value}, value.recenterReference);
    expect(independentlyRelative
               && near(quaternionAngularDistance(independentlyRelative->value,
                       result.relative.value)->degrees, 0.0),
           "relative prediction derives from predicted absolute and same reference");
}

void testTimeFilteringAndAcceleration()
{
    using namespace xreal::sensors;
    OrientationPredictorConfig smooth;
    smooth.angularVelocitySmoothingTimeConstant = std::chrono::duration<double>(0.010);
    OrientationPredictor predictor(smooth);
    static_cast<void>(predictor.predict(input(0, 0)));
    const auto filtered = predictor.predict(input(10'000'000, 10));
    expect(filtered.angularVelocity.smoothingApplied
               && filtered.angularVelocity.filtered.zRadiansPerSecond > 6.0
               && filtered.angularVelocity.filtered.zRadiansPerSecond < 7.0,
           "velocity smoothing is time-based exponential smoothing");
    const auto duplicate = predictor.predict(input(10'000'000, 20));
    expect(near(duplicate.angularVelocity.filtered.zRadiansPerSecond,
                    filtered.angularVelocity.filtered.zRadiansPerSecond)
               && predictor.state().statistics.duplicateTimestamps == 1,
           "duplicate timestamp does not advance smoothing state");
    const auto decreasing = predictor.predict(input(9'000'000, 20));
    expect(decreasing.rejectionReason == PredictionRejectionReason::decreasingTimestamp,
           "decreasing timestamp is explicitly rejected");

    OrientationPredictorConfig acceleration;
    acceleration.mode = PredictionMode::constantAcceleration;
    acceleration.horizon = std::chrono::milliseconds(10);
    acceleration.maximumAngularAccelerationRadiansPerSecondSquared = 10'000.0;
    OrientationPredictor accelerationPredictor(acceleration);
    const auto first = accelerationPredictor.predict(input(0, 1));
    expect(first.validity == PredictionValidity::valid && !first.angularAcceleration.used,
           "first constant-acceleration sample falls back to constant velocity");
    const auto second = accelerationPredictor.predict(input(10'000'000, 2));
    expect(second.angularAcceleration.available && second.angularAcceleration.used
               && near(second.angularAcceleration.filtered.zRadiansPerSecondSquared, 100.0),
           "constant acceleration uses velocity change divided by device delta");
}

void testValidationAndLimits()
{
    using namespace xreal::sensors;
    OrientationPredictorConfig reject;
    reject.horizon = std::chrono::milliseconds(60);
    OrientationPredictor horizonReject(reject);
    expect(horizonReject.predict(input(0, 1)).rejectionReason
               == PredictionRejectionReason::horizonExceedsMaximum,
           "excessive horizon rejects by default");
    reject.limitBehavior = PredictionLimitBehavior::clamp;
    OrientationPredictor horizonClamp(reject);
    const auto clamped = horizonClamp.predict(input(0, 1));
    expect(clamped.validity == PredictionValidity::valid && clamped.horizon.clamped
               && near(clamped.horizon.applied.count(), 0.05),
           "explicit clamp policy bounds horizon");
    OrientationPredictorConfig speed;
    speed.maximumAngularSpeedRadiansPerSecond = 1.0;
    OrientationPredictor speedReject(speed);
    expect(speedReject.predict(input(0, 2)).rejectionReason
               == PredictionRejectionReason::angularSpeedExceedsMaximum,
           "excessive speed rejects safely");
    speed.limitBehavior = PredictionLimitBehavior::clamp;
    OrientationPredictor speedClamp(speed);
    expect(speedClamp.predict(input(0, 2)).diagnostics.speedClamped,
           "speed clamp is reported");
    OrientationPredictor invalid;
    expect(invalid.predict(input(0, std::numeric_limits<double>::quiet_NaN())).rejectionReason
               == PredictionRejectionReason::invalidAngularVelocity,
           "NaN velocity is rejected");
    auto invalidQuaternion = input(0, 0);
    invalidQuaternion.measuredAbsolute.value = {0, 0, 0, 0};
    expect(invalid.predict(invalidQuaternion).rejectionReason
               == PredictionRejectionReason::invalidOrientation,
           "invalid quaternion is rejected");
}

void testDelayedEvaluationAndSerialization()
{
    using namespace xreal::sensors;
    DelayedOrientationEvaluator evaluator({std::chrono::milliseconds(1), 2});
    evaluator.enqueue(0, std::chrono::milliseconds(10), {{}}, {rotation(0, 0, 1, 10)}, 0);
    const auto early = evaluator.consumeMeasured(
        9'500'000, {rotation(0, 0, 1, 9.5)}, 0);
    expect(!early.has_value() && evaluator.pendingCount() == 1U,
           "early measurement inside tolerance waits for the target timestamp");
    const auto matched = evaluator.consumeMeasured(10'000'000, {rotation(0, 0, 1, 10)}, 0);
    expect(matched && near(matched->predictionTotalErrorDegrees, 0.0)
               && near(matched->baselineTotalErrorDegrees, 10.0)
               && matched->totalImprovementDegrees > 9.9,
           "delayed evaluator compares prediction and no-prediction baseline");
    evaluator.enqueue(20'000'000, std::chrono::milliseconds(10), {{}}, {{}}, 0);
    static_cast<void>(evaluator.consumeMeasured(30'000'000, {{}}, 1));
    expect(evaluator.statistics().incompatibleRecenter == 1,
           "delayed evaluator never crosses recenter generations");
    evaluator.enqueue(40'000'000, std::chrono::milliseconds(10), {{}}, {{}}, 1);
    evaluator.finish();
    expect(evaluator.statistics().unmatched == 2, "finish accounts for unmatched predictions");

    OrientationPredictionRecord record;
    record.prediction.validity = PredictionValidity::valid;
    const auto header = orientationPredictionCsvHeader();
    const auto row = serializeOrientationPredictionCsvRow(record);
    expect(std::count(header.begin(), header.end(), ',') == std::count(row.begin(), row.end(), ','),
           "prediction CSV header and row have stable column count");
    const auto json = serializeOrientationPredictionJson({}, {}, evaluator.statistics(), record);
    expect(JsonSyntaxParser(json).valid()
               && json.find("orientation-pose-prediction") != std::string::npos
               && json.find("current_times_body_delta") != std::string::npos
               && json.find("mean_total_improvement_degrees") != std::string::npos,
           "prediction JSON records schema, convention, and evaluation");
}
} // namespace

int main()
{
    testExponentialMapAndConstantVelocity();
    testFramesAndRecenter();
    testTimeFilteringAndAcceleration();
    testValidationAndLimits();
    testDelayedEvaluationAndSerialization();
    if (failures != 0) { return 1; }
    std::cout << "All orientation prediction tests passed.\n";
    return 0;
}
