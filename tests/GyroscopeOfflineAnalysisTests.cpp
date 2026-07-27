#include "sensors/GyroscopeOfflineAnalysis.hpp"
#include "sensors/GyroscopeRecordingCsv.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

namespace
{

using namespace std::chrono_literals;

int failureCount{};

void expect(bool condition, std::string_view description)
{
    if (!condition)
    {
        ++failureCount;
        std::cerr << "FAILED: " << description << '\n';
    }
}

[[nodiscard]] bool near(double actual, double expected, double tolerance = 1.0e-6)
{
    return std::abs(actual - expected) <= tolerance;
}

[[nodiscard]] xreal::sensors::GyroscopeOfflineAnalysisConfig config()
{
    xreal::sensors::GyroscopeOfflineAnalysisConfig value;
    value.base.startThresholdRaw = 100.0;
    value.base.stopThresholdRaw = 20.0;
    value.base.stillnessDuration = 50ms;
    value.base.maximumDeviceDelta = 20ms;
    value.preStillnessDuration = 100ms;
    value.preStillnessMaximumRmsRaw = 20.0;
    value.preStillnessMaximumPeakRaw = 30.0;
    value.postStillnessDuration = 100ms;
    value.postStillnessMaximumRmsRaw = 20.0;
    value.postStillnessMaximumPeakRaw = 30.0;
    value.minimumDeadbandRaw = 5.0;
    value.noiseStandardDeviationMultiplier = 3.0;
    value.envelopeWindow = 20ms;
    value.startSustainDuration = 30ms;
    value.stopSustainDuration = 40ms;
    value.trimEnvelopeFraction = 0.02;
    value.residualMeanDisagreementWarningRaw = 15.0;
    return value;
}

void append(
    std::vector<xreal::sensors::ImuSample>& samples,
    std::int32_t x,
    std::int32_t y,
    std::int32_t z)
{
    xreal::sensors::ImuSample sample;
    sample.deviceTimestamp.nanoseconds = samples.size() * 10'000'000ULL;
    sample.packetSequence = static_cast<std::uint8_t>(samples.size());
    sample.gyroscopeRaw = {x, y, z};
    samples.push_back(sample);
}

void repeat(
    std::vector<xreal::sensors::ImuSample>& samples,
    std::int32_t x,
    std::int32_t y,
    std::int32_t z,
    std::size_t count)
{
    for (std::size_t index = 0; index < count; ++index)
    {
        append(samples, x, y, z);
    }
}

[[nodiscard]] std::vector<xreal::sensors::ImuSample> controlledRotation(
    std::int32_t rate = 3600,
    bool negative = false,
    bool includePost = true)
{
    std::vector<xreal::sensors::ImuSample> samples;
    for (std::size_t index = 0; index < 20U; ++index)
    {
        append(samples, index % 2U == 0U ? 2 : -2, 1, -1);
    }
    repeat(samples, 0, 0, 40, 3U);
    repeat(samples, 0, 0, 80, 3U);
    repeat(samples, 0, 0, negative ? -rate : rate, 100U);
    repeat(samples, 0, 0, 80, 3U);
    repeat(samples, 0, 0, 40, 3U);
    if (includePost)
    {
        repeat(samples, 1, -1, 2, 20U);
    }
    return samples;
}

void testCsvParser()
{
    std::istringstream valid(
        "sequence,device_timestamp,gyro_raw_x,gyro_raw_y,gyro_raw_z,bias_raw_x,bias_raw_y,bias_raw_z\n"
        "2,100,10,20,30,-1.5,2.5,-3.5\n"
        "3,200,11,21,31,-1.5,2.5,-3.5\n");
    const auto parsed = xreal::sensors::parseGyroscopeRecordingCsv(valid);
    expect(parsed.valid() && parsed.samples.size() == 2U && parsed.bias.has_value(),
           "CSV parser accepts a valid record-only CSV");
    expect(parsed.samples[0].deviceTimestamp.nanoseconds == 100U
               && parsed.samples[1].deviceTimestamp.nanoseconds == 200U,
           "CSV parser preserves sample order");

    std::istringstream missing(
        "sequence,device_timestamp,gyro_raw_x,gyro_raw_y,bias_raw_x,bias_raw_y,bias_raw_z\n"
        "2,100,10,20,-1,2,-3\n");
    expect(!xreal::sensors::parseGyroscopeRecordingCsv(missing).valid(),
           "CSV parser rejects missing required columns");

    std::istringstream invalid(
        "sequence,device_timestamp,gyro_raw_x,gyro_raw_y,gyro_raw_z,bias_raw_x,bias_raw_y,bias_raw_z\n"
        "2,not-a-number,10,20,30,-1,2,-3\n");
    expect(!xreal::sensors::parseGyroscopeRecordingCsv(invalid).valid(),
           "CSV parser rejects invalid numeric values");
}

void testStillnessAndInitialPlacementExclusion()
{
    auto confidenceConfiguration = config();
    confidenceConfiguration.base.expectedAngleDegrees = 360.0;
    const auto analysis = xreal::sensors::analyzeGyroscopeRecordingOffline(
        controlledRotation(), {}, confidenceConfiguration);
    expect(analysis.preStillness.accepted,
           "valid pre-motion stillness is detected");
    expect(analysis.postStillness.accepted,
           "valid post-motion stillness is detected");
    expect(analysis.bestSegmentIndex.has_value()
               && analysis.segments[*analysis.bestSegmentIndex].refinedStartSampleIndex
                   > analysis.preStillness.endSampleIndex,
           "candidate starts only after verified pre-stillness");

    std::vector<xreal::sensors::ImuSample> placement;
    repeat(placement, 500, 300, 100, 20U);
    repeat(placement, 0, 0, 0, 20U);
    repeat(placement, 0, 0, 1000, 50U);
    repeat(placement, 0, 0, 0, 20U);
    const auto excluded = xreal::sensors::analyzeGyroscopeRecordingOffline(
        placement, {}, config());
    expect(excluded.preStillness.accepted && excluded.preStillness.startSampleIndex >= 20U,
           "initial placement motion is excluded before the accepted stillness window");

    std::vector<xreal::sensors::ImuSample> moving;
    repeat(moving, 200, 200, 200, 100U);
    const auto noStillness = xreal::sensors::analyzeGyroscopeRecordingOffline(
        moving, {}, config());
    expect(!noStillness.preStillness.accepted && noStillness.segments.empty()
               && !noStillness.warnings.empty(),
           "invalid pre-motion stillness prevents silent segmentation");

    const auto noPost = xreal::sensors::analyzeGyroscopeRecordingOffline(
        controlledRotation(1000, false, false), {}, confidenceConfiguration);
    expect(noPost.bestSegmentIndex.has_value()
               && !noPost.segments[*noPost.bestSegmentIndex].postStillnessVerified
               && noPost.scaleEstimates.confidence < analysis.scaleEstimates.confidence,
           "missing post-motion stillness is retained but lowers confidence");
}

void testNoiseFloorAndDeadband()
{
    const auto samples = controlledRotation(1000);
    const auto analysis = xreal::sensors::analyzeGyroscopeRecordingOffline(samples, {}, config());
    expect(near(analysis.noiseFloor.standardDeviationRaw.x, 2.0, 0.1),
           "stationary noise-floor standard deviation is calculated");
    expect(near(analysis.noiseFloor.deadbandRaw.x, 6.0, 0.1)
               && near(analysis.noiseFloor.deadbandRaw.y, 5.0),
           "dynamic deadband uses max(minimum, multiplier times standard deviation)");
    expect(xreal::sensors::applyGyroscopeDeadband(4.0, 5.0,
               xreal::sensors::GyroscopeDeadbandMode::hard) == 0.0
               && xreal::sensors::applyGyroscopeDeadband(8.0, 5.0,
                  xreal::sensors::GyroscopeDeadbandMode::hard) == 8.0,
           "hard deadband zeroes only values inside the threshold");
    expect(xreal::sensors::applyGyroscopeDeadband(-8.0, 5.0,
               xreal::sensors::GyroscopeDeadbandMode::soft) == -3.0,
           "soft deadband preserves only magnitude above the threshold");
    expect(samples.front().gyroscopeRaw.x == 2,
           "deadband processing does not modify original samples");
}

void testEnvelopeSustainAndTrimming()
{
    std::vector<xreal::sensors::ImuSample> spike;
    repeat(spike, 0, 0, 0, 20U);
    append(spike, 0, 0, 1000);
    repeat(spike, 0, 0, 0, 30U);
    const auto ignored = xreal::sensors::analyzeGyroscopeRecordingOffline(spike, {}, config());
    expect(ignored.segments.empty(), "moving RMS envelope ignores an isolated spike");

    const auto sustained = xreal::sensors::analyzeGyroscopeRecordingOffline(
        controlledRotation(1000), {}, config());
    expect(sustained.bestSegmentIndex.has_value(),
           "sustained motion triggers segment start and sustained stillness triggers end");
    const auto& segment = sustained.segments[*sustained.bestSegmentIndex];
    expect(segment.roughDuration >= 30ms && segment.postStillnessVerified,
           "start and stop sustain durations are enforced");

    auto trimmingConfig = config();
    trimmingConfig.base.startThresholdRaw = 20.0;
    trimmingConfig.base.stopThresholdRaw = 10.0;
    trimmingConfig.trimEnvelopeFraction = 0.2;
    const auto trimmed = xreal::sensors::analyzeGyroscopeRecordingOffline(
        controlledRotation(1000), {}, trimmingConfig);
    const auto& trimmedSegment = trimmed.segments[*trimmed.bestSegmentIndex];
    expect(trimmedSegment.leadingSamplesTrimmed != 0U
               || trimmedSegment.trailingSamplesTrimmed != 0U,
           "candidate trimming removes low-energy tails");
    expect(trimmedSegment.refinedDuration > 800ms,
           "candidate trimming preserves the main acceleration and rotation interval");
}

void testResidualCorrectionAndScaleMethods()
{
    std::vector<xreal::sensors::ImuSample> samples;
    repeat(samples, 0, 0, 10, 20U);
    repeat(samples, 0, 0, 1010, 100U);
    repeat(samples, 0, 0, 10, 20U);
    auto configuration = config();
    configuration.base.expectedAngleDegrees = 100.0;
    configuration.residualBiasMode = xreal::sensors::GyroscopeResidualBiasMode::preStillness;
    const auto constant = xreal::sensors::analyzeGyroscopeRecordingOffline(
        samples, {}, configuration);
    const auto& constantSegment = constant.segments[*constant.bestSegmentIndex];
    expect(constantSegment.integratedResidualCorrected.z
               < constantSegment.integratedRaw.z,
           "constant pre-stillness residual bias is subtracted from integration");
    expect(constant.scaleEstimates.recommendedMethod == "residual-corrected",
           "configured residual correction becomes the recommended scale method");

    samples.clear();
    repeat(samples, 0, 0, 10, 20U);
    for (std::size_t index = 0; index < 100U; ++index)
    {
        append(samples, 0, 0, static_cast<std::int32_t>(1010.0 + index * 0.1));
    }
    repeat(samples, 0, 0, 20, 20U);
    configuration.residualBiasMode = xreal::sensors::GyroscopeResidualBiasMode::linear;
    const auto linear = xreal::sensors::analyzeGyroscopeRecordingOffline(samples, {}, configuration);
    expect(linear.segments[*linear.bestSegmentIndex].integratedResidualCorrected.z
               < linear.segments[*linear.bestSegmentIndex].integratedRaw.z,
           "linear pre/post residual correction is applied across the segment");
    expect(linear.scaleEstimates.raw.available && linear.scaleEstimates.deadbanded.available
               && linear.scaleEstimates.residualCorrected.available,
           "raw, deadbanded and residual-corrected scale estimates are all reported");

    samples.clear();
    repeat(samples, 0, 0, 10, 20U);
    repeat(samples, 0, 0, 1010, 100U);
    repeat(samples, 0, 0, 50, 20U);
    configuration.postStillnessMaximumRmsRaw = 100.0;
    configuration.postStillnessMaximumPeakRaw = 100.0;
    const auto disagreement = xreal::sensors::analyzeGyroscopeRecordingOffline(
        samples, {}, configuration);
    expect(!disagreement.warnings.empty(),
           "pre/post residual disagreement creates a warning");
    expect(disagreement.scaleEstimates.maximumRelativeMethodDifference >= 0.0
               && disagreement.scaleEstimates.confidence
                   <= disagreement.segments[*disagreement.bestSegmentIndex].score.total,
           "method disagreement cannot increase confidence");
}

void testScaleRecoveryAndScoringJson()
{
    auto configuration = config();
    configuration.base.expectedAngleDegrees = 360.0;
    configuration.deadbandMode = xreal::sensors::GyroscopeDeadbandMode::none;
    configuration.residualBiasMode = xreal::sensors::GyroscopeResidualBiasMode::none;
    const auto positive = xreal::sensors::analyzeGyroscopeRecordingOffline(
        controlledRotation(3600), {}, configuration);
    const auto negative = xreal::sensors::analyzeGyroscopeRecordingOffline(
        controlledRotation(3600, true), {}, configuration);
    expect(near(positive.scaleEstimates.recommended.rawUnitsPerDegreePerSecond, 10.0, 0.2)
               && near(negative.scaleEstimates.recommended.rawUnitsPerDegreePerSecond, 10.0, 0.2),
           "synthetic positive and negative 360-degree rotations recover equal scale magnitude");
    expect(positive.bestSegmentIndex.has_value()
               && positive.segments[*positive.bestSegmentIndex].rotationalEnergyFraction > 0.9,
           "best segment contains most rotational energy");

    xreal::sensors::GyroscopeRecordingMetadata metadata;
    metadata.device = {0x3318, 0x0426, 2, "test"};
    metadata.biasCalibration.accepted = true;
    const std::string json = xreal::sensors::serializeGyroscopeOfflineAnalysisJson(
        metadata, positive);
    expect(json.starts_with("{") && json.ends_with("}\n")
               && json.find("\"score_breakdown\"") != std::string::npos
               && json.find("\"scale_estimate\"") != std::string::npos
               && json.find("\"scale_estimates\"") != std::string::npos,
           "valid refined JSON serializes score breakdown and backward-compatible fields");
}

[[nodiscard]] xreal::sensors::GyroscopeOfflineAnalysisResult batchAnalysis(
    double scale,
    xreal::sensors::RotationDirection direction,
    double confidence)
{
    xreal::sensors::GyroscopeOfflineAnalysisResult result;
    xreal::sensors::GyroscopeRefinedSegment segment;
    segment.direction = direction;
    segment.dominantAxis = xreal::sensors::GyroscopeAxis::z;
    segment.refinedDuration = 1s;
    result.segments.push_back(segment);
    result.bestSegmentIndex = 0U;
    result.scaleEstimates.recommended.available = true;
    result.scaleEstimates.recommended.rawUnitsPerDegreePerSecond = scale;
    result.scaleEstimates.confidence = confidence;
    return result;
}

void testBatchAggregation()
{
    const std::vector names{std::string("a"), std::string("b"), std::string("c"), std::string("out")};
    const std::vector consistent{
        batchAnalysis(10.0, xreal::sensors::RotationDirection::positive, 0.9),
        batchAnalysis(10.2, xreal::sensors::RotationDirection::positive, 0.9),
        batchAnalysis(9.8, xreal::sensors::RotationDirection::negative, 0.9),
        batchAnalysis(100.0, xreal::sensors::RotationDirection::negative, 0.9)};
    const auto batch = xreal::sensors::aggregateGyroscopeRecordings(
        names, consistent, {});
    expect(batch.accepted && batch.acceptedCount == 3U && batch.rejectedCount == 1U,
           "consistent batch is accepted after robust outlier rejection");
    expect(near(batch.median, 10.0) && near(batch.mean, 10.0)
               && batch.standardDeviation > 0.0 && batch.coefficientOfVariation > 0.0,
           "batch median, mean, standard deviation and coefficient of variation are correct");
    expect(!batch.recordings.back().accepted
               && batch.recordings.back().rejectionReason.find("outlier") != std::string::npos,
           "batch median-relative outlier has an explicit reason");

    auto tooFewConfig = xreal::sensors::GyroscopeBatchConfig{};
    tooFewConfig.minimumAcceptedRecordings = 4U;
    expect(!xreal::sensors::aggregateGyroscopeRecordings(names, consistent, tooFewConfig).accepted,
           "too few accepted recordings reject the batch");

    std::vector low = consistent;
    for (auto& value : low)
    {
        value.scaleEstimates.confidence = 0.1;
    }
    expect(!xreal::sensors::aggregateGyroscopeRecordings(names, low, {}).accepted,
           "low-confidence recordings reject the batch");

    const std::vector directionNames{std::string("p1"), std::string("p2"),
                                     std::string("n1"), std::string("n2")};
    const std::vector directions{
        batchAnalysis(10.0, xreal::sensors::RotationDirection::positive, 0.9),
        batchAnalysis(10.0, xreal::sensors::RotationDirection::positive, 0.9),
        batchAnalysis(14.0, xreal::sensors::RotationDirection::negative, 0.9),
        batchAnalysis(14.0, xreal::sensors::RotationDirection::negative, 0.9)};
    auto directionConfig = xreal::sensors::GyroscopeBatchConfig{};
    directionConfig.outlierPercent = 100.0;
    directionConfig.maximumCoefficientOfVariationPercent = 100.0;
    expect(!xreal::sensors::aggregateGyroscopeRecordings(
                directionNames, directions, directionConfig).accepted,
           "opposite-direction disagreement is detected");
}

} // namespace

int main()
{
    testCsvParser();
    testStillnessAndInitialPlacementExclusion();
    testNoiseFloorAndDeadband();
    testEnvelopeSustainAndTrimming();
    testResidualCorrectionAndScaleMethods();
    testScaleRecoveryAndScoringJson();
    testBatchAggregation();
    if (failureCount != 0)
    {
        std::cerr << failureCount << " offline gyroscope analysis test(s) failed.\n";
        return 1;
    }
    std::cout << "All offline gyroscope analysis tests passed.\n";
    return 0;
}
