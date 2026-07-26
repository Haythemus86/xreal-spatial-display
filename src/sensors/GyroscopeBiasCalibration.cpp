#include "sensors/GyroscopeBiasCalibration.hpp"
#include "sensors/DeviceTimestampDelta.hpp"

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

[[nodiscard]] bool isFinitePositive(double value) noexcept
{
    return std::isfinite(value) && value > 0.0;
}

void validateConfiguration(const GyroscopeBiasCalibrationConfig& configuration)
{
    if (configuration.calibrationDuration <= std::chrono::nanoseconds::zero())
    {
        throw std::invalid_argument("Gyroscope calibration duration must be positive.");
    }
    if (configuration.minimumSampleCount == 0U)
    {
        throw std::invalid_argument("Gyroscope calibration minimum sample count must be positive.");
    }
    if (!isFinitePositive(configuration.maximumStandardDeviationRaw)
        || !isFinitePositive(configuration.maximumRangeRaw))
    {
        throw std::invalid_argument("Gyroscope calibration movement thresholds must be finite and positive.");
    }
    if (configuration.warmupDuration.has_value()
        && *configuration.warmupDuration < std::chrono::nanoseconds::zero())
    {
        throw std::invalid_argument("Gyroscope calibration warm-up duration cannot be negative.");
    }
    if (configuration.acceptablePacketRate.has_value())
    {
        const auto& range = *configuration.acceptablePacketRate;
        if (!isFinitePositive(range.minimumPacketsPerSecond)
            || !isFinitePositive(range.maximumPacketsPerSecond)
            || range.minimumPacketsPerSecond > range.maximumPacketsPerSecond)
        {
            throw std::invalid_argument("Gyroscope calibration packet-rate range is invalid.");
        }
    }
}

[[nodiscard]] RawVector3d means(const VectorStatistics& statistics) noexcept
{
    return {statistics.x.mean, statistics.y.mean, statistics.z.mean};
}

[[nodiscard]] RawVector3d ranges(const VectorStatistics& statistics) noexcept
{
    return {
        statistics.x.maximum - statistics.x.minimum,
        statistics.y.maximum - statistics.y.minimum,
        statistics.z.maximum - statistics.z.minimum,
    };
}

void writeRawVector(std::ostringstream& output, const RawVector3d& vector)
{
    output << "{\"x\":" << vector.x << ",\"y\":" << vector.y << ",\"z\":" << vector.z << '}';
}

void writeBias(std::ostringstream& output, const GyroscopeBias& bias)
{
    output << "{\"x\":" << bias.x << ",\"y\":" << bias.y << ",\"z\":" << bias.z << '}';
}

[[nodiscard]] std::string escapeJson(const std::string& value)
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
            output << static_cast<char>(character);
            break;
        }
    }
    return output.str();
}

} // namespace

GyroscopeBiasCalibrator::GyroscopeBiasCalibrator(GyroscopeBiasCalibrationConfig configuration)
    : configuration_(std::move(configuration))
{
    validateConfiguration(configuration_);
}

void GyroscopeBiasCalibrator::consume(const ImuSample& sample) noexcept
{
    if (result_.has_value())
    {
        return;
    }

    ++observedSampleCount_;
    std::optional<std::uint64_t> delta;
    if (previousDeviceTimestamp_.has_value())
    {
        delta = forwardDeviceTimestampDelta(sample.deviceTimestamp.nanoseconds, *previousDeviceTimestamp_);
        if (!delta.has_value())
        {
            reject(GyroscopeBiasCalibrationRejectionReason::invalidDeviceTimestamp);
            return;
        }
    }
    previousDeviceTimestamp_ = sample.deviceTimestamp.nanoseconds;

    const bool warmupSamplesComplete = observedSampleCount_ > configuration_.warmupSampleCount;
    if (!warmupSamplesComplete)
    {
        if (delta.has_value())
        {
            accumulatedWarmupNanoseconds_ += *delta;
        }
        return;
    }

    if (configuration_.warmupDuration.has_value()
        && accumulatedWarmupNanoseconds_ < static_cast<std::uint64_t>(configuration_.warmupDuration->count()))
    {
        if (delta.has_value())
        {
            accumulatedWarmupNanoseconds_ += *delta;
        }
        return;
    }

    x_.consume(static_cast<double>(sample.gyroscopeRaw.x));
    y_.consume(static_cast<double>(sample.gyroscopeRaw.y));
    z_.consume(static_cast<double>(sample.gyroscopeRaw.z));
    ++captureSampleCount_;

    if (captureSampleCount_ > 1U && delta.has_value())
    {
        if (*delta > std::numeric_limits<std::uint64_t>::max() - accumulatedCaptureNanoseconds_)
        {
            reject(GyroscopeBiasCalibrationRejectionReason::invalidDeviceTimestamp);
            return;
        }
        accumulatedCaptureNanoseconds_ += *delta;
        deviceTimestampDeltas_.consume(static_cast<double>(*delta));
    }

    if (accumulatedCaptureNanoseconds_
        >= static_cast<std::uint64_t>(configuration_.calibrationDuration.count()))
    {
        evaluateCompletedCapture();
    }
}

bool GyroscopeBiasCalibrator::isComplete() const noexcept
{
    return result_.has_value();
}

std::optional<GyroscopeBiasCalibrationResult> GyroscopeBiasCalibrator::result() const noexcept
{
    return result_;
}

GyroscopeBiasCalibrationResult GyroscopeBiasCalibrator::finish() noexcept
{
    if (!result_.has_value())
    {
        if (captureSampleCount_ < configuration_.minimumSampleCount)
        {
            reject(GyroscopeBiasCalibrationRejectionReason::insufficientSamples);
        }
        else
        {
            reject(GyroscopeBiasCalibrationRejectionReason::insufficientDuration);
        }
    }
    return *result_;
}

void GyroscopeBiasCalibrator::reject(GyroscopeBiasCalibrationRejectionReason reason) noexcept
{
    result_ = GyroscopeBiasCalibrationResult{
        false,
        reason,
        configuration_.calibrationDuration,
        configuration_,
        statistics(),
        std::nullopt,
    };
}

void GyroscopeBiasCalibrator::evaluateCompletedCapture() noexcept
{
    const auto currentStatistics = statistics();
    if (currentStatistics.sampleCount < configuration_.minimumSampleCount)
    {
        reject(GyroscopeBiasCalibrationRejectionReason::insufficientSamples);
        return;
    }
    if (currentStatistics.captureDuration < configuration_.calibrationDuration)
    {
        reject(GyroscopeBiasCalibrationRejectionReason::insufficientDuration);
        return;
    }
    if (configuration_.acceptablePacketRate.has_value())
    {
        const auto& range = *configuration_.acceptablePacketRate;
        if (currentStatistics.packetRate < range.minimumPacketsPerSecond
            || currentStatistics.packetRate > range.maximumPacketsPerSecond)
        {
            reject(GyroscopeBiasCalibrationRejectionReason::packetRateOutOfRange);
            return;
        }
    }

    const auto& gyro = currentStatistics.gyroscopeRaw;
    if (gyro.x.standardDeviation > configuration_.maximumStandardDeviationRaw
        || gyro.y.standardDeviation > configuration_.maximumStandardDeviationRaw
        || gyro.z.standardDeviation > configuration_.maximumStandardDeviationRaw)
    {
        reject(GyroscopeBiasCalibrationRejectionReason::excessiveStandardDeviation);
        return;
    }
    const auto& range = currentStatistics.rangeRaw;
    if (range.x > configuration_.maximumRangeRaw
        || range.y > configuration_.maximumRangeRaw
        || range.z > configuration_.maximumRangeRaw)
    {
        reject(GyroscopeBiasCalibrationRejectionReason::excessiveRange);
        return;
    }

    const RawVector3d mean = means(gyro);
    result_ = GyroscopeBiasCalibrationResult{
        true,
        GyroscopeBiasCalibrationRejectionReason::none,
        configuration_.calibrationDuration,
        configuration_,
        currentStatistics,
        GyroscopeBias{mean.x, mean.y, mean.z},
    };
}

GyroscopeBiasCalibrationStatistics GyroscopeBiasCalibrator::statistics() const noexcept
{
    const VectorStatistics gyro{x_.result(), y_.result(), z_.result()};
    const double durationSeconds = static_cast<double>(accumulatedCaptureNanoseconds_) / 1'000'000'000.0;
    const double packetRate = durationSeconds > 0.0 && captureSampleCount_ > 1U
        ? static_cast<double>(captureSampleCount_ - 1U) / durationSeconds
        : 0.0;
    return {
        gyro,
        captureSampleCount_ == 0U ? RawVector3d{} : ranges(gyro),
        deviceTimestampDeltas_.result(),
        captureSampleCount_,
        std::chrono::nanoseconds(accumulatedCaptureNanoseconds_),
        packetRate,
    };
}

CorrectedGyroscopeRaw applyGyroscopeBias(
    const RawImuVector3& raw,
    const GyroscopeBias& bias) noexcept
{
    return {
        static_cast<double>(raw.x) - bias.x,
        static_cast<double>(raw.y) - bias.y,
        static_cast<double>(raw.z) - bias.z,
    };
}

std::string gyroscopeBiasCalibrationRejectionReasonText(
    GyroscopeBiasCalibrationRejectionReason reason)
{
    switch (reason)
    {
    case GyroscopeBiasCalibrationRejectionReason::none:
        return {};
    case GyroscopeBiasCalibrationRejectionReason::insufficientSamples:
        return "too few stationary samples";
    case GyroscopeBiasCalibrationRejectionReason::insufficientDuration:
        return "device timestamp capture duration was insufficient";
    case GyroscopeBiasCalibrationRejectionReason::invalidDeviceTimestamp:
        return "device timestamps were invalid or non-monotonic";
    case GyroscopeBiasCalibrationRejectionReason::excessiveStandardDeviation:
        return "gyroscope standard deviation exceeded the configured threshold";
    case GyroscopeBiasCalibrationRejectionReason::excessiveRange:
        return "gyroscope raw range exceeded the configured threshold";
    case GyroscopeBiasCalibrationRejectionReason::packetRateOutOfRange:
        return "device timestamp packet rate was outside the configured range";
    }
    return "unknown rejection reason";
}

std::string serializeGyroscopeBiasCalibrationJson(
    const GyroscopeBiasCalibrationDevice& device,
    const GyroscopeBiasCalibrationResult& result)
{
    std::ostringstream output;
    output << std::setprecision(std::numeric_limits<double>::max_digits10)
           << "{\n  \"schema_version\": 1,\n"
           << "  \"units\": \"raw\",\n"
           << "  \"device\": {\"vendor_id\":\"0x" << std::hex << std::uppercase
           << std::setw(4) << std::setfill('0') << device.vendorId
           << "\",\"product_id\":\"0x" << std::setw(4) << device.productId
           << std::dec << std::nouppercase << std::setfill(' ')
           << "\",\"interface_number\":" << device.interfaceNumber
           << ",\"product\":\"" << escapeJson(device.productName) << "\"},\n"
           << "  \"calibration\": {\"type\":\"gyroscope-bias\",\"accepted\":"
           << (result.accepted ? "true" : "false")
           << ",\"rejection_reason\":\""
           << escapeJson(gyroscopeBiasCalibrationRejectionReasonText(result.rejectionReason))
           << "\",\"requested_duration_seconds\":"
           << std::chrono::duration<double>(result.requestedDuration).count()
           << ",\"measured_duration_seconds\":"
           << std::chrono::duration<double>(result.statistics.captureDuration).count()
           << ",\"sample_count\":" << result.statistics.sampleCount
           << ",\"bias_valid\":" << (result.biasRaw.has_value() ? "true" : "false")
           << ",\"bias_raw\":";
    if (result.biasRaw.has_value())
    {
        writeBias(output, *result.biasRaw);
    }
    else
    {
        output << "null";
    }

    output << "},\n  \"statistics\": {\"mean_raw\":";
    writeRawVector(output, means(result.statistics.gyroscopeRaw));
    output << ",\"stddev_raw\":{"
           << "\"x\":" << result.statistics.gyroscopeRaw.x.standardDeviation
           << ",\"y\":" << result.statistics.gyroscopeRaw.y.standardDeviation
           << ",\"z\":" << result.statistics.gyroscopeRaw.z.standardDeviation << "}"
           << ",\"min_raw\":{"
           << "\"x\":" << result.statistics.gyroscopeRaw.x.minimum
           << ",\"y\":" << result.statistics.gyroscopeRaw.y.minimum
           << ",\"z\":" << result.statistics.gyroscopeRaw.z.minimum << "}"
           << ",\"max_raw\":{"
           << "\"x\":" << result.statistics.gyroscopeRaw.x.maximum
           << ",\"y\":" << result.statistics.gyroscopeRaw.y.maximum
           << ",\"z\":" << result.statistics.gyroscopeRaw.z.maximum << "}"
           << ",\"range_raw\":";
    writeRawVector(output, result.statistics.rangeRaw);
    output << ",\"device_timestamp_delta_ns\":{"
           << "\"count\":" << result.statistics.deviceTimestampDeltaNanoseconds.sampleCount
           << ",\"mean\":" << result.statistics.deviceTimestampDeltaNanoseconds.mean
           << ",\"min\":" << result.statistics.deviceTimestampDeltaNanoseconds.minimum
           << ",\"max\":" << result.statistics.deviceTimestampDeltaNanoseconds.maximum
           << ",\"stddev\":" << result.statistics.deviceTimestampDeltaNanoseconds.standardDeviation
           << "},\"packet_rate\":" << result.statistics.packetRate << "},\n"
           << "  \"configuration\": {\"calibration_duration_ns\":"
           << result.configuration.calibrationDuration.count()
           << ",\"minimum_sample_count\":" << result.configuration.minimumSampleCount
           << ",\"maximum_standard_deviation_raw\":"
           << result.configuration.maximumStandardDeviationRaw
           << ",\"maximum_range_raw\":" << result.configuration.maximumRangeRaw
           << ",\"warmup_sample_count\":" << result.configuration.warmupSampleCount
           << ",\"warmup_duration_ns\":";
    if (result.configuration.warmupDuration.has_value())
    {
        output << result.configuration.warmupDuration->count();
    }
    else
    {
        output << "null";
    }
    output << ",\"acceptable_packet_rate\":";
    if (result.configuration.acceptablePacketRate.has_value())
    {
        output << "{\"minimum\":"
               << result.configuration.acceptablePacketRate->minimumPacketsPerSecond
               << ",\"maximum\":"
               << result.configuration.acceptablePacketRate->maximumPacketsPerSecond << '}';
    }
    else
    {
        output << "null";
    }
    output << "}\n}\n";
    return output.str();
}

} // namespace xreal::sensors
