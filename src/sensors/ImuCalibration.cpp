#include "sensors/ImuCalibration.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace xreal::sensors
{
namespace
{

RawVector3d means(const VectorStatistics& statistics) noexcept
{
    return {statistics.x.mean, statistics.y.mean, statistics.z.mean};
}

double signedTimestampDelta(std::uint64_t current, std::uint64_t previous) noexcept
{
    if (current >= previous)
    {
        return static_cast<double>(current - previous);
    }

    return -static_cast<double>(previous - current);
}

std::string escapeJson(std::string_view value)
{
    std::ostringstream output;
    for (const unsigned char character : value)
    {
        switch (character)
        {
        case '"':
            output << "\\\"";
            break;
        case '\\':
            output << "\\\\";
            break;
        case '\b':
            output << "\\b";
            break;
        case '\f':
            output << "\\f";
            break;
        case '\n':
            output << "\\n";
            break;
        case '\r':
            output << "\\r";
            break;
        case '\t':
            output << "\\t";
            break;
        default:
            if (character < 0x20U)
            {
                output << "\\u" << std::hex << std::setw(4) << std::setfill('0')
                       << static_cast<unsigned int>(character) << std::dec << std::setfill(' ');
            }
            else
            {
                output << static_cast<char>(character);
            }
            break;
        }
    }
    return output.str();
}

void writeScalarStatistics(std::ostringstream& output, const ScalarStatistics& statistics)
{
    output << "{\"count\":" << statistics.sampleCount
           << ",\"mean\":" << statistics.mean
           << ",\"min\":" << statistics.minimum
           << ",\"max\":" << statistics.maximum
           << ",\"stddev\":" << statistics.standardDeviation << '}';
}

void writeVectorStatistics(std::ostringstream& output, const VectorStatistics& statistics)
{
    output << "{\"x\":";
    writeScalarStatistics(output, statistics.x);
    output << ",\"y\":";
    writeScalarStatistics(output, statistics.y);
    output << ",\"z\":";
    writeScalarStatistics(output, statistics.z);
    output << '}';
}

void writeRawVector(std::ostringstream& output, const RawVector3d& vector)
{
    output << "{\"x\":" << vector.x << ",\"y\":" << vector.y << ",\"z\":" << vector.z << '}';
}

} // namespace

void RunningStatistics::consume(double value) noexcept
{
    ++sampleCount_;
    if (sampleCount_ == 1U)
    {
        mean_ = value;
        minimum_ = value;
        maximum_ = value;
        return;
    }

    minimum_ = std::min(minimum_, value);
    maximum_ = std::max(maximum_, value);
    const double difference = value - mean_;
    mean_ += difference / static_cast<double>(sampleCount_);
    const double differenceFromNewMean = value - mean_;
    sumSquaredDifferences_ += difference * differenceFromNewMean;
}

ScalarStatistics RunningStatistics::result() const noexcept
{
    if (sampleCount_ == 0U)
    {
        return {};
    }

    return {
        sampleCount_,
        mean_,
        minimum_,
        maximum_,
        std::sqrt(sumSquaredDifferences_ / static_cast<double>(sampleCount_)),
    };
}

GyroBiasEstimator::GyroBiasEstimator(double movementThresholdRaw)
    : movementThresholdRaw_(movementThresholdRaw)
{
    if (!std::isfinite(movementThresholdRaw) || movementThresholdRaw < 0.0)
    {
        throw std::invalid_argument("Gyroscope movement threshold must be finite and non-negative.");
    }
}

void GyroBiasEstimator::consume(const RawImuVector3& gyroscopeRaw) noexcept
{
    x_.consume(static_cast<double>(gyroscopeRaw.x));
    y_.consume(static_cast<double>(gyroscopeRaw.y));
    z_.consume(static_cast<double>(gyroscopeRaw.z));
}

std::optional<GyroBiasEstimate> GyroBiasEstimator::result() const noexcept
{
    const ScalarStatistics x = x_.result();
    const ScalarStatistics y = y_.result();
    const ScalarStatistics z = z_.result();
    if (x.sampleCount == 0U)
    {
        return std::nullopt;
    }

    const RawVector3d standardDeviation{x.standardDeviation, y.standardDeviation, z.standardDeviation};
    const bool accepted = standardDeviation.x <= movementThresholdRaw_
        && standardDeviation.y <= movementThresholdRaw_
        && standardDeviation.z <= movementThresholdRaw_;

    return GyroBiasEstimate{
        accepted,
        {x.mean, y.mean, z.mean},
        standardDeviation,
        x.sampleCount,
        movementThresholdRaw_,
    };
}

ImuCalibrationAccumulator::ImuCalibrationAccumulator(double gyroMovementThresholdRaw)
    : gyroBiasEstimator_(gyroMovementThresholdRaw)
{
}

void ImuCalibrationAccumulator::consume(const ImuSample& sample) noexcept
{
    gyroX_.consume(static_cast<double>(sample.gyroscopeRaw.x));
    gyroY_.consume(static_cast<double>(sample.gyroscopeRaw.y));
    gyroZ_.consume(static_cast<double>(sample.gyroscopeRaw.z));
    accelX_.consume(static_cast<double>(sample.accelerometerRaw.x));
    accelY_.consume(static_cast<double>(sample.accelerometerRaw.y));
    accelZ_.consume(static_cast<double>(sample.accelerometerRaw.z));
    gyroBiasEstimator_.consume(sample.gyroscopeRaw);

    if (previousDeviceTimestamp_.has_value())
    {
        deviceTimestampDeltas_.consume(signedTimestampDelta(
            sample.deviceTimestamp.nanoseconds,
            *previousDeviceTimestamp_));
    }
    if (previousHostTimestamp_.has_value())
    {
        hostTimestampDeltas_.consume(static_cast<double>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                sample.hostReceiveTimestamp - *previousHostTimestamp_).count()));
    }

    previousDeviceTimestamp_ = sample.deviceTimestamp.nanoseconds;
    previousHostTimestamp_ = sample.hostReceiveTimestamp;
    if (!firstHostTimestamp_.has_value())
    {
        firstHostTimestamp_ = sample.hostReceiveTimestamp;
    }
    lastHostTimestamp_ = sample.hostReceiveTimestamp;
}

ImuCalibrationStatistics ImuCalibrationAccumulator::statistics() const noexcept
{
    const VectorStatistics gyroscope{gyroX_.result(), gyroY_.result(), gyroZ_.result()};
    const VectorStatistics accelerometer{accelX_.result(), accelY_.result(), accelZ_.result()};
    std::chrono::nanoseconds duration{};
    if (firstHostTimestamp_.has_value() && lastHostTimestamp_.has_value())
    {
        duration = std::chrono::duration_cast<std::chrono::nanoseconds>(
            *lastHostTimestamp_ - *firstHostTimestamp_);
    }

    return {
        gyroscope,
        accelerometer,
        {deviceTimestampDeltas_.result(), hostTimestampDeltas_.result()},
        gyroscope.x.sampleCount,
        duration,
    };
}

std::optional<StationaryCalibrationResult> ImuCalibrationAccumulator::stationaryResult() const noexcept
{
    const auto bias = gyroBiasEstimator_.result();
    if (!bias.has_value())
    {
        return std::nullopt;
    }

    const ImuCalibrationStatistics calibrationStatistics = statistics();
    const RawVector3d accelerationMean = means(calibrationStatistics.accelerometerRaw);
    return StationaryCalibrationResult{
        bias->accepted,
        bias->mean,
        bias->standardDeviation,
        accelerationMean,
        std::sqrt((accelerationMean.x * accelerationMean.x)
                  + (accelerationMean.y * accelerationMean.y)
                  + (accelerationMean.z * accelerationMean.z)),
        bias->sampleCount,
        calibrationStatistics.captureDuration,
    };
}

std::string serializeCalibrationReportJson(const ImuCalibrationReport& report)
{
    std::ostringstream output;
    output << std::setprecision(std::numeric_limits<double>::max_digits10)
           << "{\n  \"schema_version\": 1,\n"
           << "  \"units\": \"raw\",\n"
           << "  \"calibration\": \"" << escapeJson(report.name) << "\",\n"
           << "  \"sample_count\": " << report.statistics.sampleCount << ",\n"
           << "  \"capture_duration_ns\": " << report.statistics.captureDuration.count() << ",\n"
           << "  \"packet_rate\": " << report.packetRate << ",\n"
           << "  \"sequence_gaps\": " << report.sequenceGaps << ",\n"
           << "  \"out_of_sequence_events\": " << report.outOfSequenceEvents << ",\n"
           << "  \"malformed_packets\": " << report.malformedPackets << ",\n"
           << "  \"queue_drops\": " << report.queueDrops << ",\n"
           << "  \"gyro_raw\": ";
    writeVectorStatistics(output, report.statistics.gyroscopeRaw);
    output << ",\n  \"accel_raw\": ";
    writeVectorStatistics(output, report.statistics.accelerometerRaw);
    output << ",\n  \"device_timestamp_delta_ns\": ";
    writeScalarStatistics(output, report.statistics.timestampDeltas.deviceNanoseconds);
    output << ",\n  \"host_timestamp_delta_ns\": ";
    writeScalarStatistics(output, report.statistics.timestampDeltas.hostNanoseconds);
    output << ",\n  \"stationary\": {\"accepted\":"
           << (report.stationary.accepted ? "true" : "false")
           << ",\"gyro_bias_raw\":";
    writeRawVector(output, report.stationary.gyroBias);
    output << ",\"gyro_stddev_raw\":";
    writeRawVector(output, report.stationary.gyroStandardDeviation);
    output << ",\"accel_mean_raw\":";
    writeRawVector(output, report.stationary.accelMean);
    output << ",\"accel_vector_magnitude_raw\":" << report.stationary.accelVectorMagnitudeRaw
           << ",\"sample_count\":" << report.stationary.sampleCount
           << ",\"capture_duration_ns\":" << report.stationary.captureDuration.count()
           << "}\n}\n";
    return output.str();
}

} // namespace xreal::sensors
