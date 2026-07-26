#include "sensors/GyroscopeBiasCalibration.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <string>
#include <string_view>

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

[[nodiscard]] xreal::sensors::ImuSample makeSample(
    std::uint64_t timestampNanoseconds,
    xreal::sensors::RawImuVector3 gyroscope)
{
    xreal::sensors::ImuSample sample;
    sample.deviceTimestamp.nanoseconds = timestampNanoseconds;
    sample.gyroscopeRaw = gyroscope;
    return sample;
}

[[nodiscard]] xreal::sensors::GyroscopeBiasCalibrationConfig testConfiguration()
{
    xreal::sensors::GyroscopeBiasCalibrationConfig configuration;
    configuration.calibrationDuration = std::chrono::milliseconds(4);
    configuration.minimumSampleCount = 5;
    configuration.maximumStandardDeviationRaw = 10.0;
    configuration.maximumRangeRaw = 20.0;
    configuration.acceptablePacketRate = xreal::sensors::PacketRateRange{900.0, 1100.0};
    configuration.warmupSampleCount = 0;
    return configuration;
}

[[nodiscard]] xreal::sensors::GyroscopeBiasCalibrationResult acceptedFixture()
{
    xreal::sensors::GyroscopeBiasCalibrator calibrator(testConfiguration());
    constexpr std::array<std::int32_t, 5> xValues{8, 9, 10, 11, 12};
    constexpr std::array<std::int32_t, 5> zValues{28, 29, 30, 31, 32};
    for (std::size_t index = 0; index < xValues.size(); ++index)
    {
        calibrator.consume(makeSample(
            static_cast<std::uint64_t>(index) * 1'000'000U,
            {xValues[index], -20, zValues[index]}));
    }
    return calibrator.finish();
}

void testAcceptedStatisticsAndKnownBias()
{
    const auto result = acceptedFixture();
    expect(result.accepted, "stationary synthetic samples are accepted");
    expect(result.biasRaw.has_value(), "accepted calibration exposes a bias");
    expect(near(result.biasRaw->x, 10.0)
               && near(result.biasRaw->y, -20.0)
               && near(result.biasRaw->z, 30.0),
           "per-axis means match the known synthetic bias");
    expect(near(result.statistics.gyroscopeRaw.x.standardDeviation, std::sqrt(2.0)),
           "population standard deviation is correct");
    expect(near(result.statistics.gyroscopeRaw.x.minimum, 8.0)
               && near(result.statistics.gyroscopeRaw.x.maximum, 12.0)
               && near(result.statistics.rangeRaw.x, 4.0),
           "minimum, maximum and range are correct");
    expect(result.statistics.sampleCount == 5, "all calibration samples are counted");
    expect(result.statistics.captureDuration == std::chrono::milliseconds(4),
           "capture duration uses device timestamps");
    expect(near(result.statistics.packetRate, 1000.0), "packet rate uses device timestamps");
}

void testDefaultsAcceptRepresentativeStationaryNoise()
{
    xreal::sensors::GyroscopeBiasCalibrator calibrator(
        xreal::sensors::GyroscopeBiasCalibrationConfig{});
    constexpr std::array<std::int32_t, 3> noise{-300, 0, 300};
    for (std::uint64_t index = 0; index < 2'101; ++index)
    {
        const std::int32_t variation = noise[index % noise.size()];
        calibrator.consume(makeSample(
            index * 1'000'000U,
            {1234 + variation, -2345 - variation, 3456 + variation / 2}));
    }
    const auto result = calibrator.finish();
    expect(result.accepted, "default thresholds accept representative stationary raw noise");
    expect(result.statistics.sampleCount == 2001,
           "the default 100 warm-up samples are excluded from calibration statistics");
    expect(near(result.biasRaw->x, 1234.0)
               && near(result.biasRaw->y, -2345.0)
               && near(result.biasRaw->z, 3456.0),
           "warm-up exclusion preserves the deterministic stationary mean");
}

void testStandardDeviationRejection()
{
    auto configuration = testConfiguration();
    configuration.maximumStandardDeviationRaw = 1.0;
    configuration.maximumRangeRaw = 100.0;
    xreal::sensors::GyroscopeBiasCalibrator calibrator(configuration);
    for (std::uint64_t index = 0; index < 5; ++index)
    {
        calibrator.consume(makeSample(index * 1'000'000U, {static_cast<std::int32_t>(index * 4), 0, 0}));
    }
    const auto result = calibrator.finish();
    expect(!result.accepted
               && result.rejectionReason
                   == xreal::sensors::GyroscopeBiasCalibrationRejectionReason::excessiveStandardDeviation,
           "excessive standard deviation is rejected explicitly");
}

void testRangeRejection()
{
    auto configuration = testConfiguration();
    configuration.maximumStandardDeviationRaw = 100.0;
    configuration.maximumRangeRaw = 3.0;
    xreal::sensors::GyroscopeBiasCalibrator calibrator(configuration);
    for (std::uint64_t index = 0; index < 5; ++index)
    {
        calibrator.consume(makeSample(index * 1'000'000U, {static_cast<std::int32_t>(index), 0, 0}));
    }
    const auto result = calibrator.finish();
    expect(!result.accepted
               && result.rejectionReason
                   == xreal::sensors::GyroscopeBiasCalibrationRejectionReason::excessiveRange,
           "excessive raw range is rejected explicitly");
}

void testInsufficientSamplesAndDuration()
{
    auto configuration = testConfiguration();
    configuration.calibrationDuration = std::chrono::milliseconds(10);
    configuration.minimumSampleCount = 5;
    configuration.acceptablePacketRate.reset();
    xreal::sensors::GyroscopeBiasCalibrator tooFew(configuration);
    tooFew.consume(makeSample(0, {1, 2, 3}));
    tooFew.consume(makeSample(1'000'000, {1, 2, 3}));
    const auto tooFewResult = tooFew.finish();
    expect(tooFewResult.rejectionReason
               == xreal::sensors::GyroscopeBiasCalibrationRejectionReason::insufficientSamples,
           "too few samples are rejected");

    configuration.minimumSampleCount = 3;
    xreal::sensors::GyroscopeBiasCalibrator tooShort(configuration);
    tooShort.consume(makeSample(0, {1, 2, 3}));
    tooShort.consume(makeSample(1'000'000, {1, 2, 3}));
    tooShort.consume(makeSample(2'000'000, {1, 2, 3}));
    const auto tooShortResult = tooShort.finish();
    expect(tooShortResult.rejectionReason
               == xreal::sensors::GyroscopeBiasCalibrationRejectionReason::insufficientDuration,
           "insufficient device timestamp duration is rejected");
}

void testInvalidTimestampAndWraparound()
{
    auto configuration = testConfiguration();
    configuration.minimumSampleCount = 2;
    configuration.calibrationDuration = std::chrono::milliseconds(1);
    xreal::sensors::GyroscopeBiasCalibrator invalid(configuration);
    invalid.consume(makeSample(5'000'000, {1, 2, 3}));
    invalid.consume(makeSample(5'000'000, {1, 2, 3}));
    expect(invalid.finish().rejectionReason
               == xreal::sensors::GyroscopeBiasCalibrationRejectionReason::invalidDeviceTimestamp,
           "non-monotonic timestamps are rejected");

    xreal::sensors::GyroscopeBiasCalibrator wrapped(configuration);
    wrapped.consume(makeSample(std::numeric_limits<std::uint64_t>::max() - 499'999U, {1, 2, 3}));
    wrapped.consume(makeSample(500'000U, {1, 2, 3}));
    expect(wrapped.finish().accepted, "uint64 device timestamp wraparound is handled");
}

void testPacketRateRejection()
{
    auto configuration = testConfiguration();
    configuration.minimumSampleCount = 3;
    configuration.calibrationDuration = std::chrono::milliseconds(4);
    xreal::sensors::GyroscopeBiasCalibrator calibrator(configuration);
    calibrator.consume(makeSample(0, {1, 2, 3}));
    calibrator.consume(makeSample(2'000'000, {1, 2, 3}));
    calibrator.consume(makeSample(4'000'000, {1, 2, 3}));
    expect(calibrator.finish().rejectionReason
               == xreal::sensors::GyroscopeBiasCalibrationRejectionReason::packetRateOutOfRange,
           "unexpected device-timestamp packet rate is rejected");
}

void testBiasCorrectionPreservesOriginal()
{
    const xreal::sensors::RawImuVector3 original{-10, 20, -30};
    const xreal::sensors::GyroscopeBias bias{5.5, -2.0, 10.0};
    const auto corrected = xreal::sensors::applyGyroscopeBias(original, bias);
    expect(near(corrected.x, -15.5) && near(corrected.y, 22.0) && near(corrected.z, -40.0),
           "bias subtraction preserves negative corrected raw values on all axes");
    expect(original.x == -10 && original.y == 20 && original.z == -30,
           "bias subtraction does not modify the original raw sample");
}

void testAcceptedAndRejectedJson()
{
    const xreal::sensors::GyroscopeBiasCalibrationDevice device{0x3318, 0x0426, 2, "XREAL Air 2 Ultra"};
    const std::string acceptedJson = xreal::sensors::serializeGyroscopeBiasCalibrationJson(
        device,
        acceptedFixture());
    expect(acceptedJson.starts_with("{") && acceptedJson.ends_with("}\n"), "accepted JSON is a complete object");
    expect(acceptedJson.find("\"accepted\":true") != std::string::npos
               && acceptedJson.find("\"bias_valid\":true") != std::string::npos
               && acceptedJson.find("\"bias_raw\":{") != std::string::npos
               && acceptedJson.find("\"statistics\":") != std::string::npos
               && acceptedJson.find("\"configuration\":") != std::string::npos
               && acceptedJson.find("\"device_timestamp_delta_ns\":") != std::string::npos,
           "accepted JSON contains bias, statistics and configuration");

    auto configuration = testConfiguration();
    configuration.minimumSampleCount = 10;
    xreal::sensors::GyroscopeBiasCalibrator rejected(configuration);
    rejected.consume(makeSample(0, {1, 2, 3}));
    const std::string rejectedJson = xreal::sensors::serializeGyroscopeBiasCalibrationJson(
        device,
        rejected.finish());
    expect(rejectedJson.find("\"accepted\":false") != std::string::npos
               && rejectedJson.find("\"bias_valid\":false") != std::string::npos
               && rejectedJson.find("\"bias_raw\":null") != std::string::npos
               && rejectedJson.find("too few stationary samples") != std::string::npos,
           "rejected JSON contains its reason and no valid bias");
}

} // namespace

int main()
{
    testAcceptedStatisticsAndKnownBias();
    testDefaultsAcceptRepresentativeStationaryNoise();
    testStandardDeviationRejection();
    testRangeRejection();
    testInsufficientSamplesAndDuration();
    testInvalidTimestampAndWraparound();
    testPacketRateRejection();
    testBiasCorrectionPreservesOriginal();
    testAcceptedAndRejectedJson();

    if (failureCount != 0)
    {
        std::cerr << failureCount << " gyroscope bias calibration test(s) failed.\n";
        return 1;
    }
    std::cout << "All gyroscope bias calibration tests passed.\n";
    return 0;
}
