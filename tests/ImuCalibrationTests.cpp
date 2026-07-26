#include "sensors/ImuCalibration.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
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

bool near(double actual, double expected, double tolerance = 1.0e-9)
{
    return std::abs(actual - expected) <= tolerance;
}

xreal::sensors::ImuSample makeSample(
    std::uint64_t deviceTimestampNanoseconds,
    std::int64_t hostTimestampNanoseconds,
    xreal::sensors::RawImuVector3 gyroscope,
    xreal::sensors::RawImuVector3 accelerometer)
{
    xreal::sensors::ImuSample sample;
    sample.deviceTimestamp.nanoseconds = deviceTimestampNanoseconds;
    sample.hostReceiveTimestamp = std::chrono::steady_clock::time_point(
        std::chrono::nanoseconds(hostTimestampNanoseconds));
    sample.gyroscopeRaw = gyroscope;
    sample.accelerometerRaw = accelerometer;
    return sample;
}

void testMeanAndStandardDeviation()
{
    xreal::sensors::RunningStatistics statistics;
    statistics.consume(1.0);
    statistics.consume(2.0);
    statistics.consume(3.0);
    statistics.consume(4.0);

    const auto result = statistics.result();
    expect(result.sampleCount == 4, "running statistics count every value");
    expect(near(result.mean, 2.5), "mean is calculated in double precision");
    expect(near(result.minimum, 1.0), "minimum is retained");
    expect(near(result.maximum, 4.0), "maximum is retained");
    expect(near(result.standardDeviation, std::sqrt(1.25)),
           "population standard deviation is calculated with Welford accumulation");
}

void testGyroBiasAndStationaryRejection()
{
    xreal::sensors::GyroBiasEstimator stationaryEstimator(5.0);
    stationaryEstimator.consume({10, -20, 30});
    stationaryEstimator.consume({12, -18, 28});
    stationaryEstimator.consume({8, -22, 32});

    const auto stationary = stationaryEstimator.result();
    expect(stationary.has_value(), "stationary gyro estimate is available");
    expect(stationary->accepted, "small gyro variation is accepted");
    expect(near(stationary->mean.x, 10.0)
               && near(stationary->mean.y, -20.0)
               && near(stationary->mean.z, 30.0),
           "gyro bias is the per-axis raw mean");

    xreal::sensors::GyroBiasEstimator movingEstimator(5.0);
    movingEstimator.consume({-10, 0, 0});
    movingEstimator.consume({10, 0, 0});
    const auto moving = movingEstimator.result();
    expect(moving.has_value() && !moving->accepted,
           "gyro standard deviation above the threshold rejects stationary calibration");
}

void testTimestampDeltaStatistics()
{
    xreal::sensors::ImuCalibrationAccumulator accumulator(100.0);
    accumulator.consume(makeSample(1'000, 100, {1, 2, 3}, {3, 4, 0}));
    accumulator.consume(makeSample(2'000, 110, {1, 2, 3}, {3, 4, 0}));
    accumulator.consume(makeSample(4'000, 130, {1, 2, 3}, {3, 4, 0}));

    const auto result = accumulator.statistics();
    expect(result.timestampDeltas.deviceNanoseconds.sampleCount == 2,
           "the first device timestamp does not create a delta");
    expect(near(result.timestampDeltas.deviceNanoseconds.mean, 1500.0),
           "device timestamp delta mean is calculated");
    expect(near(result.timestampDeltas.deviceNanoseconds.standardDeviation, 500.0),
           "device timestamp delta standard deviation is calculated");
    expect(near(result.timestampDeltas.hostNanoseconds.mean, 15.0),
           "host timestamp delta mean is calculated");
    expect(near(result.timestampDeltas.hostNanoseconds.standardDeviation, 5.0),
           "host timestamp delta standard deviation is calculated");
    expect(result.captureDuration == std::chrono::nanoseconds(30),
           "capture duration spans the first through last host timestamps");

    const auto stationary = accumulator.stationaryResult();
    expect(stationary.has_value(), "stationary calibration result is produced");
    expect(near(stationary->accelVectorMagnitudeRaw, 5.0),
           "raw acceleration mean magnitude is calculated without an SI scale");
}

void testJsonSerialization()
{
    xreal::sensors::ImuCalibrationAccumulator accumulator(100.0);
    accumulator.consume(makeSample(1'000, 100, {10, -20, 30}, {3, 4, 0}));
    accumulator.consume(makeSample(2'000, 1'100, {10, -20, 30}, {3, 4, 0}));
    const auto stationary = accumulator.stationaryResult();

    const xreal::sensors::ImuCalibrationReport report{
        "stationary-flat",
        accumulator.statistics(),
        *stationary,
        1000.0,
        0,
        0,
        0,
        0,
    };
    const std::string json = xreal::sensors::serializeCalibrationReportJson(report);

    expect(json.find("\"calibration\": \"stationary-flat\"") != std::string::npos,
           "JSON contains the calibration name");
    expect(json.find("\"units\": \"raw\"") != std::string::npos,
           "JSON explicitly identifies raw units");
    expect(json.find("\"gyro_bias_raw\":") != std::string::npos,
           "JSON contains the raw gyro bias");
    expect(json.find("\"device_timestamp_delta_ns\":") != std::string::npos,
           "JSON contains timestamp delta statistics");
    expect(json.find("gyro_rad_s") == std::string::npos
               && json.find("meters_per_second") == std::string::npos,
           "JSON does not serialize unconfirmed SI values");
}

} // namespace

int main()
{
    testMeanAndStandardDeviation();
    testGyroBiasAndStationaryRejection();
    testTimestampDeltaStatistics();
    testJsonSerialization();

    if (failureCount != 0)
    {
        std::cerr << failureCount << " calibration test(s) failed.\n";
        return 1;
    }

    std::cout << "All IMU calibration tests passed.\n";
    return 0;
}
