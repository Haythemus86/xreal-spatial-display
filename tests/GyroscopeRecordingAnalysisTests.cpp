#include "sensors/GyroscopeRecordingAnalysis.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
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

[[nodiscard]] xreal::sensors::GyroscopeRecordingAnalysisConfig configuration()
{
    xreal::sensors::GyroscopeRecordingAnalysisConfig result;
    result.startThresholdRaw = 100.0;
    result.stopThresholdRaw = 20.0;
    result.stillnessDuration = 30ms;
    result.maximumDeviceDelta = 20ms;
    return result;
}

void append(
    std::vector<xreal::sensors::ImuSample>& samples,
    std::uint64_t timestamp,
    std::int32_t x,
    std::int32_t y,
    std::int32_t z,
    std::uint8_t sequence)
{
    xreal::sensors::ImuSample sample;
    sample.deviceTimestamp.nanoseconds = timestamp;
    sample.gyroscopeRaw = {x, y, z};
    sample.accelerometerRaw = {11, 22, 33};
    sample.packetSequence = sequence;
    sample.hostReceiveTimestamp = std::chrono::steady_clock::time_point(
        std::chrono::nanoseconds(timestamp + 123U));
    samples.push_back(sample);
}

void appendStill(
    std::vector<xreal::sensors::ImuSample>& samples,
    std::uint64_t& timestamp,
    std::uint8_t& sequence,
    std::size_t count)
{
    for (std::size_t index = 0; index < count; ++index)
    {
        append(samples, timestamp, 0, 0, 0, sequence++);
        timestamp += 10'000'000U;
    }
}

void appendMotion(
    std::vector<xreal::sensors::ImuSample>& samples,
    std::uint64_t& timestamp,
    std::uint8_t& sequence,
    std::int32_t x,
    std::int32_t y,
    std::int32_t z,
    std::size_t count)
{
    for (std::size_t index = 0; index < count; ++index)
    {
        append(samples, timestamp, x, y, z, sequence++);
        timestamp += 10'000'000U;
    }
}

[[nodiscard]] std::vector<xreal::sensors::ImuSample> oneSegment(
    std::int32_t x,
    std::int32_t y,
    std::int32_t z,
    std::size_t motionSamples = 100U)
{
    std::vector<xreal::sensors::ImuSample> samples;
    std::uint64_t timestamp{};
    std::uint8_t sequence{};
    appendStill(samples, timestamp, sequence, 5U);
    appendMotion(samples, timestamp, sequence, x, y, z, motionSamples);
    appendStill(samples, timestamp, sequence, 5U);
    return samples;
}

void testCaptureNeverRejectsOrdinaryMotion()
{
    const auto samples = oneSegment(-300, 200, -500);
    auto config = configuration();
    config.requestedDirection = xreal::sensors::RotationDirection::positive;
    const auto result = xreal::sensors::analyzeGyroscopeRecording(samples, {}, config);
    expect(result.samples.size() == samples.size(),
           "cross-axis motion does not truncate record-only samples");
    expect(result.samples.size() == samples.size() && result.segments.size() == 1U,
           "opposite direction and exceeded thresholds do not stop capture");
    expect(result.samples.back().elapsedDeviceNanoseconds == 1'090'000'000U,
           "the complete requested synthetic recording duration is retained");
}

void testBiasIntegrationAndOriginalPreservation()
{
    std::vector<xreal::sensors::ImuSample> samples;
    append(samples, 0U, 110, 0, 0, 0U);
    append(samples, 250'000'000U, 210, 0, 0, 1U);
    append(samples, 1'000'000'000U, 310, 0, 0, 2U);
    auto config = configuration();
    config.maximumDeviceDelta = 1s;
    const auto result = xreal::sensors::analyzeGyroscopeRecording(
        samples, {10.0, 0.0, 0.0}, config);
    expect(near(result.x.signedIntegratedRawAngle, 225.0),
           "variable-rate trapezoidal integration uses irregular device timestamps");
    expect(result.samples[0].corrected.x == 100.0, "bias is subtracted before analysis");
    expect(samples[0].gyroscopeRaw.x == 110 && result.samples[0].original.gyroscopeRaw.x == 110,
           "original raw samples remain unchanged");

    std::vector<xreal::sensors::ImuSample> constant;
    append(constant, 0U, 100, 0, 0, 0U);
    append(constant, 1'000'000'000U, 100, 0, 0, 1U);
    expect(near(xreal::sensors::analyzeGyroscopeRecording(
                    constant, {}, config).x.signedIntegratedRawAngle, 100.0),
           "constant-rate integration is exact");
}

void testTimestampAndSequenceDiagnostics()
{
    std::vector<xreal::sensors::ImuSample> samples;
    append(samples, 100U, 0, 0, 0, 1U);
    append(samples, 100U, 0, 0, 0, 2U);
    append(samples, 90U, 0, 0, 0, 5U);
    append(samples, 30'000'090U, 0, 0, 0, 6U);
    const auto result = xreal::sensors::analyzeGyroscopeRecording(samples, {}, configuration());
    expect(result.samples[1].timestampStatus
               == xreal::sensors::RecordingTimestampStatus::duplicate,
           "duplicate timestamps are flagged");
    expect(result.samples[2].timestampStatus
               == xreal::sensors::RecordingTimestampStatus::decreasing,
           "decreasing timestamps are flagged");
    expect(result.samples[3].timestampStatus
               == xreal::sensors::RecordingTimestampStatus::deltaTooLarge,
           "excessive timestamp deltas are flagged");
    expect(result.sequenceGaps == 2U && result.samples[2].sequenceGap == 2U,
           "sequence gaps are reported");
}

void testSegmentationAndDominantAxes()
{
    const auto xResult = xreal::sensors::analyzeGyroscopeRecording(
        oneSegment(300, 20, 10), {}, configuration());
    expect(xResult.segments.size() == 1U
               && xResult.segments[0].dominantAxis == xreal::sensors::GyroscopeAxis::x,
           "a single dominant X segment is detected");
    expect(xResult.segments[0].stillnessBefore && xResult.segments[0].stillnessAfter,
           "stillness before and after a segment is detected");
    expect(near(xResult.segments[0].crossAxisRatio, 20.0 / 300.0),
           "cross-axis ratio uses peak corrected values");
    expect(xResult.segments[0].direction == xreal::sensors::RotationDirection::positive,
           "positive direction is identified");

    const auto yResult = xreal::sensors::analyzeGyroscopeRecording(
        oneSegment(10, -400, 20), {}, configuration());
    expect(yResult.dominantAxis == xreal::sensors::GyroscopeAxis::y
               && yResult.segments[0].direction == xreal::sensors::RotationDirection::negative,
           "dominant Y and negative direction are identified");

    const auto zResult = xreal::sensors::analyzeGyroscopeRecording(
        oneSegment(10, 20, 500), {}, configuration());
    expect(zResult.dominantAxis == xreal::sensors::GyroscopeAxis::z,
           "dominant Z movement is identified");

    std::vector<xreal::sensors::ImuSample> multiple;
    std::uint64_t timestamp{};
    std::uint8_t sequence{};
    appendStill(multiple, timestamp, sequence, 5U);
    appendMotion(multiple, timestamp, sequence, 300, 0, 0, 50U);
    appendStill(multiple, timestamp, sequence, 6U);
    appendMotion(multiple, timestamp, sequence, 0, 0, 500, 100U);
    appendStill(multiple, timestamp, sequence, 5U);
    const auto first = xreal::sensors::analyzeGyroscopeRecording(multiple, {}, configuration());
    const auto second = xreal::sensors::analyzeGyroscopeRecording(multiple, {}, configuration());
    expect(first.segments.size() == 2U, "multiple motion segments are preserved");
    expect(first.bestSegmentIndex == second.bestSegmentIndex
               && first.bestSegmentIndex == 1U,
           "best-segment selection is deterministic and favors the cleaner longer segment");
}

void testNoMotionAndNoiseWarnings()
{
    std::vector<xreal::sensors::ImuSample> still;
    std::uint64_t timestamp{};
    std::uint8_t sequence{};
    appendStill(still, timestamp, sequence, 100U);
    const auto noMotion = xreal::sensors::analyzeGyroscopeRecording(still, {}, configuration());
    expect(!noMotion.usable && noMotion.segments.empty()
               && noMotion.failure
                   == xreal::sensors::GyroscopeRecordingAnalysisFailure::noUsableSegment,
           "no-motion recording returns an explicit no-segment result");

    const auto noisy = xreal::sensors::analyzeGyroscopeRecording(
        oneSegment(300, 250, 200), {}, configuration());
    expect(!noisy.warnings.empty(), "substantial cross-axis noise produces a warning");
}

void testExperimentalScaleEstimation()
{
    auto config = configuration();
    config.expectedAngleDegrees = 90.0;
    auto samples = oneSegment(0, 0, 900, 100U);
    const auto quarter = xreal::sensors::analyzeGyroscopeRecording(samples, {}, config);
    expect(quarter.scaleEstimate.available && quarter.scaleEstimate.experimental,
           "known-angle estimate is available and explicitly experimental");
    expect(quarter.scaleEstimate.axis == xreal::sensors::GyroscopeAxis::z,
           "axis auto selects the dominant axis");
    expect(near(quarter.scaleEstimate.rawUnitsPerDegreePerSecond, 10.0, 0.01),
           "known synthetic 90-degree rotation recovers the scale");

    config.expectedAngleDegrees = 360.0;
    samples = oneSegment(0, 0, 3600, 100U);
    const auto full = xreal::sensors::analyzeGyroscopeRecording(samples, {}, config);
    expect(near(full.scaleEstimate.rawUnitsPerDegreePerSecond, 10.0, 0.01),
           "known synthetic 360-degree rotation recovers the scale");

    config.expectedAngleDegrees.reset();
    expect(!xreal::sensors::analyzeGyroscopeRecording(samples, {}, config)
                .scaleEstimate.available,
           "scale remains unavailable without expected degrees");

    config.expectedAngleDegrees = 360.0;
    config.axisSelection = xreal::sensors::GyroscopeAxisSelection::x;
    const auto explicitAxis = xreal::sensors::analyzeGyroscopeRecording(
        oneSegment(400, 0, 1000, 100U), {}, config);
    expect(explicitAxis.scaleEstimate.axis == xreal::sensors::GyroscopeAxis::x,
           "an explicit analysis axis overrides dominant-axis selection");
}

void testSerialization()
{
    auto config = configuration();
    config.expectedAngleDegrees = 90.0;
    const auto analysis = xreal::sensors::analyzeGyroscopeRecording(
        oneSegment(0, 0, 900), {}, config);
    xreal::sensors::GyroscopeRecordingMetadata metadata;
    metadata.device = {0x3318, 0x0426, 2, "XREAL Air 2 Ultra"};
    metadata.biasCalibration.accepted = true;
    const std::string json = xreal::sensors::serializeGyroscopeRecordingAnalysisJson(
        metadata, analysis);
    expect(json.starts_with("{") && json.ends_with("}\n")
               && json.find("\"experimental\":true") != std::string::npos,
           "accepted recording JSON is structurally complete and marks estimates experimental");
    expect(json.find("\"segments\":[") != std::string::npos
               && json.find("\"warnings\":[") != std::string::npos,
           "all segments and warnings are serialized");

    std::vector<xreal::sensors::ImuSample> still;
    std::uint64_t timestamp{};
    std::uint8_t sequence{};
    appendStill(still, timestamp, sequence, 10U);
    const std::string noSegmentJson = xreal::sensors::serializeGyroscopeRecordingAnalysisJson(
        metadata, xreal::sensors::analyzeGyroscopeRecording(still, {}, config));
    expect(noSegmentJson.find("\"usable\":false") != std::string::npos
               && noSegmentJson.find("no usable motion segment") != std::string::npos,
           "no-segment JSON remains complete with an explicit reason");

    const std::string header = xreal::sensors::gyroscopeRecordingCsvHeader(true, true);
    expect(header.find("device_delta_ns") != std::string::npos
               && header.find("timestamp_valid") != std::string::npos
               && header.find("gyro_corrected_z") != std::string::npos
               && header.find("host_timestamp_ns") != std::string::npos
               && header.find("accel_raw_z") != std::string::npos,
           "CSV header contains every required field");
    const std::string first = xreal::sensors::serializeGyroscopeRecordingCsvRow(
        analysis.samples.front(), {}, true, true);
    const std::string second = xreal::sensors::serializeGyroscopeRecordingCsvRow(
        analysis.samples.front(), {}, true, true);
    expect(first == second && first.ends_with("\n"), "CSV row output is deterministic");
}

} // namespace

int main()
{
    testCaptureNeverRejectsOrdinaryMotion();
    testBiasIntegrationAndOriginalPreservation();
    testTimestampAndSequenceDiagnostics();
    testSegmentationAndDominantAxes();
    testNoMotionAndNoiseWarnings();
    testExperimentalScaleEstimation();
    testSerialization();
    if (failureCount != 0)
    {
        std::cerr << failureCount << " gyroscope recording analysis test(s) failed.\n";
        return 1;
    }
    std::cout << "All gyroscope recording analysis tests passed.\n";
    return 0;
}
