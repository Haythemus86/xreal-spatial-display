#include "diagnostics/ImuDiagnosticOptions.hpp"
#include "sensors/XrealDevice.hpp"
#include "sensors/GyroscopeBiasCalibration.hpp"
#include "sensors/ImuCalibration.hpp"
#include "sensors/XrealImuStream.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <exception>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

namespace
{

template <typename Value, std::size_t Capacity>
class SampleQueue
{
public:
    [[nodiscard]] bool tryPush(const Value& value) noexcept
    {
        const std::size_t write = writeIndex_.load(std::memory_order_relaxed);
        const std::size_t next = (write + 1U) % Capacity;
        if (next == readIndex_.load(std::memory_order_acquire))
        {
            return false;
        }

        values_[write] = value;
        writeIndex_.store(next, std::memory_order_release);
        return true;
    }

    [[nodiscard]] bool tryPop(Value& value) noexcept
    {
        const std::size_t read = readIndex_.load(std::memory_order_relaxed);
        if (read == writeIndex_.load(std::memory_order_acquire))
        {
            return false;
        }

        value = values_[read];
        readIndex_.store((read + 1U) % Capacity, std::memory_order_release);
        return true;
    }

private:
    static_assert(Capacity > 1);
    std::array<Value, Capacity> values_{};
    std::atomic_size_t readIndex_{};
    std::atomic_size_t writeIndex_{};
};

void printUsage()
{
    std::cout << xreal::diagnostics::imuDiagnosticUsage();
}

std::wstring maskedSerial(const std::wstring& serial)
{
    if (serial.empty())
    {
        return L"unavailable";
    }

    return L"********";
}

void writeCsvHeader(std::ofstream& output)
{
    output << "host_timestamp_ns,device_timestamp,sequence,gyro_raw_x,gyro_raw_y,gyro_raw_z,"
              "accel_raw_x,accel_raw_y,accel_raw_z,gyro_x,gyro_y,gyro_z,accel_x,accel_y,accel_z\n";
}

void writeCsvSample(std::ofstream& output, const xreal::sensors::ImuSample& sample)
{
    const auto hostNanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(
        sample.hostReceiveTimestamp.time_since_epoch()).count();

    output << hostNanoseconds << ','
           << sample.deviceTimestamp.nanoseconds << ','
           << static_cast<unsigned int>(sample.packetSequence) << ','
           << sample.gyroscopeRaw.x << ',' << sample.gyroscopeRaw.y << ',' << sample.gyroscopeRaw.z << ','
           << sample.accelerometerRaw.x << ',' << sample.accelerometerRaw.y << ','
           << sample.accelerometerRaw.z;

    if (sample.gyroscopeRadiansPerSecond.has_value())
    {
        const auto& gyro = *sample.gyroscopeRadiansPerSecond;
        output << ',' << gyro.x << ',' << gyro.y << ',' << gyro.z;
    }
    else
    {
        output << ",,,";
    }

    if (sample.accelerometerMetersPerSecondSquared.has_value())
    {
        const auto& acceleration = *sample.accelerometerMetersPerSecondSquared;
        output << ',' << acceleration.x << ',' << acceleration.y << ',' << acceleration.z;
    }
    else
    {
        output << ",,,";
    }

    output << '\n';
}

void printSample(
    const xreal::sensors::ImuSample& sample,
    std::optional<std::uint64_t> deviceTimestampDelta,
    std::optional<std::int64_t> hostTimestampDelta,
    const std::optional<xreal::sensors::GyroscopeBias>& gyroscopeBias)
{
    std::cout << "seq=" << static_cast<unsigned int>(sample.packetSequence)
              << " device_ns=" << sample.deviceTimestamp.nanoseconds
              << " gyro_raw=[" << sample.gyroscopeRaw.x << ", " << sample.gyroscopeRaw.y
              << ", " << sample.gyroscopeRaw.z << ']'
              << " accel_raw=[" << sample.accelerometerRaw.x << ", " << sample.accelerometerRaw.y
              << ", " << sample.accelerometerRaw.z << ']';

    if (gyroscopeBias.has_value())
    {
        const auto corrected = xreal::sensors::applyGyroscopeBias(
            sample.gyroscopeRaw,
            *gyroscopeBias);
        std::cout << " gyro_corrected_raw_units=[" << corrected.x << ", "
                  << corrected.y << ", " << corrected.z << ']';
    }

    if (deviceTimestampDelta.has_value() && hostTimestampDelta.has_value())
    {
        std::cout << " delta_device_ns=" << *deviceTimestampDelta
                  << " delta_host_ns=" << *hostTimestampDelta;
    }

    if (sample.gyroscopeRadiansPerSecond.has_value()
        && sample.accelerometerMetersPerSecondSquared.has_value())
    {
        const auto& gyro = *sample.gyroscopeRadiansPerSecond;
        const auto& acceleration = *sample.accelerometerMetersPerSecondSquared;
        std::cout << " gyro_rad_s=[" << gyro.x << ", " << gyro.y << ", " << gyro.z << ']'
                  << " accel_m_s2=[" << acceleration.x << ", " << acceleration.y << ", "
                  << acceleration.z << ']';
    }

    std::cout << '\n';
}

void printScalarStatistics(std::string_view label, const xreal::sensors::ScalarStatistics& statistics)
{
    std::cout << "  " << label
              << ": mean=" << statistics.mean
              << " min=" << statistics.minimum
              << " max=" << statistics.maximum
              << " stddev=" << statistics.standardDeviation
              << " count=" << statistics.sampleCount << '\n';
}

void printVectorStatistics(
    std::string_view label,
    const xreal::sensors::VectorStatistics& statistics)
{
    printScalarStatistics(std::string(label) + "_x", statistics.x);
    printScalarStatistics(std::string(label) + "_y", statistics.y);
    printScalarStatistics(std::string(label) + "_z", statistics.z);
}

constexpr double stationaryGyroMovementThresholdRaw = 5000.0;

[[nodiscard]] std::string narrowAscii(std::wstring_view value)
{
    std::string result;
    result.reserve(value.size());
    for (const wchar_t character : value)
    {
        result.push_back(character >= 0 && character <= 0x7F
            ? static_cast<char>(character)
            : '?');
    }
    return result;
}

} // namespace

int main(int argc, char* argv[])
{
    std::vector<std::string_view> arguments;
    arguments.reserve(static_cast<std::size_t>(argc > 0 ? argc - 1 : 0));
    for (int index = 1; index < argc; ++index)
    {
        arguments.emplace_back(argv[index]);
    }

    const auto optionResult = xreal::diagnostics::parseImuDiagnosticOptions(arguments);
    if (optionResult.showHelp)
    {
        printUsage();
        return 0;
    }
    if (!optionResult.options.has_value())
    {
        std::cerr << "Error: " << optionResult.error << '\n';
        printUsage();
        return 2;
    }
    const xreal::diagnostics::ImuDiagnosticOptions& options = *optionResult.options;

    try
    {
        const xreal::sensors::XrealDevice deviceEnumerator;
        const auto device = xreal::sensors::XrealImuStream::findInterface(deviceEnumerator.enumerate());
        if (!device.has_value())
        {
            std::cerr << "No exact XREAL Air 2 Ultra VID 0x3318/PID 0x0426/interface 2 was found.\n";
            return 1;
        }

        std::cout << "XREAL Air 2 Ultra IMU on VID 0x3318/PID 0x0426/interface 2\n";
        std::wcout << L"  Product: " << device->productName << L'\n'
                   << L"  Serial:  " << (options.verbose ? device->serialNumber : maskedSerial(device->serialNumber))
                   << L'\n';
        if (options.verbose)
        {
            std::cout << "  HID path: " << device->path << '\n';
        }
        std::cout << "  SI conversion: unavailable (upstream scale factors are explicitly unverified)\n";
        if (options.calibrationName.has_value())
        {
            std::cout << "  Calibration: " << *options.calibrationName << '\n'
                      << "  Raw CSV: " << *options.csvPath << '\n'
                      << "  Stationary gyro stddev threshold: "
                      << stationaryGyroMovementThresholdRaw << " raw units per axis\n";
        }
        if (options.gyroscopeCalibration.has_value())
        {
            const auto& configuration = *options.gyroscopeCalibration;
            std::cout << "\nGyroscope startup bias calibration requested.\n"
                      << "  Keep the glasses completely still until calibration completes.\n"
                      << "  Calibration duration: "
                      << std::chrono::duration<double>(configuration.calibrationDuration).count()
                      << " seconds (device time)\n"
                      << "  Warm-up samples: " << configuration.warmupSampleCount << '\n'
                      << "  Warm-up duration: ";
            if (configuration.warmupDuration.has_value())
            {
                std::cout << std::chrono::duration<double>(*configuration.warmupDuration).count()
                          << " seconds (device time)\n";
            }
            else
            {
                std::cout << "not configured\n";
            }
            std::cout << "  Maximum stddev: " << configuration.maximumStandardDeviationRaw
                      << " raw units per axis\n"
                      << "  Maximum range: " << configuration.maximumRangeRaw
                      << " raw units per axis\n";
        }

        std::ofstream csv;
        if (options.csvPath.has_value())
        {
            csv.open(*options.csvPath, std::ios::out | std::ios::trunc);
            if (!csv)
            {
                std::cerr << "Failed to open CSV output file: " << *options.csvPath << '\n';
                return 1;
            }
            writeCsvHeader(csv);
        }

        SampleQueue<xreal::sensors::ImuSample, 8192> samples;
        std::atomic_uint64_t diagnosticQueueDrops{};
        std::optional<xreal::sensors::ImuCalibrationAccumulator> calibrationAccumulator;
        if (options.calibrationName.has_value())
        {
            calibrationAccumulator.emplace(stationaryGyroMovementThresholdRaw);
        }
        std::optional<xreal::sensors::GyroscopeBiasCalibrator> gyroscopeCalibrator;
        if (options.gyroscopeCalibration.has_value())
        {
            gyroscopeCalibrator.emplace(*options.gyroscopeCalibration);
        }
        std::optional<xreal::sensors::GyroscopeBias> acceptedGyroscopeBias;

        const auto consumeGyroscopeCalibration = [&](const xreal::sensors::ImuSample& sample) {
            if (!gyroscopeCalibrator.has_value() || gyroscopeCalibrator->isComplete())
            {
                return;
            }
            gyroscopeCalibrator->consume(sample);
            const auto currentResult = gyroscopeCalibrator->result();
            if (currentResult.has_value() && currentResult->accepted)
            {
                acceptedGyroscopeBias = currentResult->biasRaw;
            }
        };
        xreal::sensors::XrealImuStream stream(*device);
        if (!stream.start([&](std::span<const std::uint8_t, 64>, const xreal::sensors::ImuSample& sample) {
                if (!samples.tryPush(sample))
                {
                    ++diagnosticQueueDrops;
                }
            }))
        {
            std::wcerr << L"Failed to start IMU acquisition: " << stream.errorMessage() << L'\n';
            return 1;
        }

        const auto start = std::chrono::steady_clock::now();
        const auto deadline = start + options.duration;
        const auto printInterval = std::chrono::milliseconds(1000U / options.printRateHz);
        constexpr std::chrono::milliseconds processingInterval{20};
        auto nextPrint = start + printInterval;
        auto nextProcessing = start + processingInterval;
        std::optional<xreal::sensors::ImuSample> latestSample;
        std::optional<xreal::sensors::ImuSample> previousSample;
        std::optional<std::uint64_t> deviceDelta;
        std::optional<std::int64_t> hostDelta;
        std::mutex timerMutex;
        std::condition_variable timer;

        while (std::chrono::steady_clock::now() < deadline && stream.isRunning())
        {
            xreal::sensors::ImuSample sample;
            while (samples.tryPop(sample))
            {
                if (csv)
                {
                    writeCsvSample(csv, sample);
                }
                if (calibrationAccumulator.has_value())
                {
                    calibrationAccumulator->consume(sample);
                }
                consumeGyroscopeCalibration(sample);

                if (previousSample.has_value())
                {
                    deviceDelta = sample.deviceTimestamp.nanoseconds
                        - previousSample->deviceTimestamp.nanoseconds;
                    hostDelta = std::chrono::duration_cast<std::chrono::nanoseconds>(
                        sample.hostReceiveTimestamp - previousSample->hostReceiveTimestamp).count();
                }
                previousSample = sample;
                latestSample = sample;
            }

            const auto now = std::chrono::steady_clock::now();
            if (now >= nextPrint)
            {
                if (latestSample.has_value())
                {
                    printSample(
                        *latestSample,
                        deviceDelta,
                        hostDelta,
                        options.applyGyroscopeBias ? acceptedGyroscopeBias : std::nullopt);
                }
                nextPrint = now + printInterval;
            }
            nextProcessing = now + processingInterval;

            std::unique_lock lock(timerMutex);
            timer.wait_until(lock, std::min({nextPrint, nextProcessing, deadline}));
        }

        stream.stop();
        xreal::sensors::ImuSample remainingSample;
        while (samples.tryPop(remainingSample))
        {
            if (csv)
            {
                writeCsvSample(csv, remainingSample);
            }
            if (calibrationAccumulator.has_value())
            {
                calibrationAccumulator->consume(remainingSample);
            }
            consumeGyroscopeCalibration(remainingSample);
        }
        if (csv)
        {
            csv.flush();
            if (!csv)
            {
                std::cerr << "Failed while writing CSV output file: " << *options.csvPath << '\n';
                return 1;
            }
        }

        const auto elapsed = std::chrono::steady_clock::now() - start;
        const auto statistics = stream.statistics();
        const double seconds = std::chrono::duration<double>(elapsed).count();
        std::cout << "\nIMU acquisition summary\n"
                  << "  Effective packet rate: " << std::fixed << std::setprecision(2)
                  << (seconds > 0.0 ? static_cast<double>(statistics.received) / seconds : 0.0) << " packets/s\n"
                  << "  Received sensor packets: " << statistics.received << '\n'
                  << "  Activation responses: " << statistics.activationResponses << '\n'
                  << "  Estimated sequence gaps: " << statistics.dropped << '\n'
                  << "  Out-of-sequence events: " << statistics.outOfSequence << '\n'
                  << "  Malformed packets: " << statistics.invalid << '\n'
                  << "  Diagnostic queue drops: " << diagnosticQueueDrops.load() << '\n';

        bool calibrationRejected{};
        if (gyroscopeCalibrator.has_value())
        {
            const auto result = gyroscopeCalibrator->finish();
            const auto& gyro = result.statistics.gyroscopeRaw;
            std::cout << "\nGyroscope bias calibration (raw units)\n"
                      << "  Status: " << (result.accepted ? "accepted" : "rejected") << '\n'
                      << "  Samples: " << result.statistics.sampleCount << '\n'
                      << "  Device duration: "
                      << std::chrono::duration<double>(result.statistics.captureDuration).count()
                      << " seconds\n"
                      << "  Packet rate: " << result.statistics.packetRate << " packets/s\n";
            printVectorStatistics("gyro_raw", gyro);
            std::cout << "  gyro_range_raw: [" << result.statistics.rangeRaw.x << ", "
                      << result.statistics.rangeRaw.y << ", "
                      << result.statistics.rangeRaw.z << "]\n";

            if (result.accepted && result.biasRaw.has_value())
            {
                std::cout << "  Bias raw units: [" << result.biasRaw->x << ", "
                          << result.biasRaw->y << ", " << result.biasRaw->z << "]\n";
            }
            else
            {
                std::cerr << "Gyroscope bias calibration rejected: "
                          << xreal::sensors::gyroscopeBiasCalibrationRejectionReasonText(
                                 result.rejectionReason)
                          << ".\n";
                calibrationRejected = true;
            }

            if (options.gyroscopeCalibrationOutputPath.has_value())
            {
                const xreal::sensors::GyroscopeBiasCalibrationDevice calibrationDevice{
                    xreal::sensors::XrealImuStream::vendorId,
                    xreal::sensors::XrealImuStream::productId,
                    xreal::sensors::XrealImuStream::interfaceNumber,
                    narrowAscii(device->productName),
                };
                std::ofstream json(
                    *options.gyroscopeCalibrationOutputPath,
                    std::ios::out | std::ios::trunc);
                if (!json)
                {
                    std::cerr << "Failed to open gyroscope calibration JSON output file: "
                              << *options.gyroscopeCalibrationOutputPath << '\n';
                    return 1;
                }
                json << xreal::sensors::serializeGyroscopeBiasCalibrationJson(
                    calibrationDevice,
                    result);
                if (!json)
                {
                    std::cerr << "Failed while writing gyroscope calibration JSON output file: "
                              << *options.gyroscopeCalibrationOutputPath << '\n';
                    return 1;
                }
                std::cout << "  Gyroscope calibration JSON: "
                          << *options.gyroscopeCalibrationOutputPath << '\n';
            }
        }

        if (calibrationAccumulator.has_value())
        {
            const auto calibrationStatistics = calibrationAccumulator->statistics();
            const auto stationary = calibrationAccumulator->stationaryResult();
            if (!stationary.has_value())
            {
                std::cerr << "Calibration failed: no decoded sensor samples were recorded.\n";
                return 1;
            }

            std::cout << "\nCalibration analysis (raw units)\n"
                      << "  Name: " << *options.calibrationName << '\n'
                      << "  Samples: " << calibrationStatistics.sampleCount << '\n'
                      << "  Capture duration: "
                      << std::chrono::duration<double>(calibrationStatistics.captureDuration).count()
                      << " seconds\n";
            printVectorStatistics("gyro_raw", calibrationStatistics.gyroscopeRaw);
            printVectorStatistics("accel_raw", calibrationStatistics.accelerometerRaw);
            printScalarStatistics(
                "device_timestamp_delta_ns",
                calibrationStatistics.timestampDeltas.deviceNanoseconds);
            printScalarStatistics(
                "host_timestamp_delta_ns",
                calibrationStatistics.timestampDeltas.hostNanoseconds);
            std::cout << "  Gyro stationary estimate: " << (stationary->accepted ? "accepted" : "rejected")
                      << '\n'
                      << "  Gyro bias raw: [" << stationary->gyroBias.x << ", "
                      << stationary->gyroBias.y << ", " << stationary->gyroBias.z << "]\n"
                      << "  Accel mean raw: [" << stationary->accelMean.x << ", "
                      << stationary->accelMean.y << ", " << stationary->accelMean.z << "]\n"
                      << "  Accel vector magnitude raw: " << stationary->accelVectorMagnitudeRaw << '\n';

            const xreal::sensors::ImuCalibrationReport report{
                *options.calibrationName,
                calibrationStatistics,
                *stationary,
                seconds > 0.0 ? static_cast<double>(statistics.received) / seconds : 0.0,
                statistics.dropped,
                statistics.outOfSequence,
                statistics.invalid,
                diagnosticQueueDrops.load(),
            };

            if (options.calibrationOutputPath.has_value())
            {
                std::ofstream json(*options.calibrationOutputPath, std::ios::out | std::ios::trunc);
                if (!json)
                {
                    std::cerr << "Failed to open calibration JSON output file: "
                              << *options.calibrationOutputPath << '\n';
                    return 1;
                }
                json << xreal::sensors::serializeCalibrationReportJson(report);
                if (!json)
                {
                    std::cerr << "Failed while writing calibration JSON output file: "
                              << *options.calibrationOutputPath << '\n';
                    return 1;
                }
                std::cout << "  Calibration JSON: " << *options.calibrationOutputPath << '\n';
            }

            if (diagnosticQueueDrops.load() != 0U)
            {
                std::cerr << "Calibration rejected because decoded samples were dropped by the diagnostic queue.\n";
                calibrationRejected = true;
            }
            if (calibrationStatistics.sampleCount != statistics.received)
            {
                std::cerr << "Calibration rejected because the analyzed sample count does not match"
                             " the decoded packet count.\n";
                calibrationRejected = true;
            }
            const bool rotationalCalibration = options.calibrationName->ends_with("-rotation");
            if (!rotationalCalibration && !stationary->accepted)
            {
                std::cerr << "Stationary calibration rejected because gyroscope movement exceeded the raw threshold.\n";
                calibrationRejected = true;
            }
        }

        const std::wstring streamError = stream.errorMessage();
        if (!streamError.empty())
        {
            std::wcerr << L"IMU acquisition stopped with an error: " << streamError << L'\n';
            return 1;
        }

        return calibrationRejected ? 1 : 0;
    }
    catch (const std::exception& exception)
    {
        std::cerr << "IMU diagnostic startup failed: " << exception.what() << '\n';
        return 1;
    }
}
