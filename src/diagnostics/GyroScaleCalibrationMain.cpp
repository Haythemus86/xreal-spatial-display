#include "diagnostics/GyroScaleCalibrationOptions.hpp"
#include "diagnostics/GyroRecordOnlyMode.hpp"
#include "sensors/GyroscopeBiasCalibration.hpp"
#include "sensors/GyroscopeScaleCalibration.hpp"
#include "sensors/XrealDevice.hpp"
#include "sensors/XrealImuStream.hpp"

#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

namespace
{

using namespace std::chrono_literals;

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
    static_assert(Capacity > 1U);
    std::array<Value, Capacity> values_{};
    std::atomic_size_t readIndex_{};
    std::atomic_size_t writeIndex_{};
};

[[nodiscard]] std::string narrowAscii(std::wstring_view value)
{
    std::string result;
    result.reserve(value.size());
    for (const wchar_t character : value)
    {
        result.push_back(character >= 0 && character <= 0x7f
            ? static_cast<char>(character)
            : '?');
    }
    return result;
}

void writeCsvHeader(std::ofstream& output)
{
    output << "host_timestamp_ns,device_timestamp,sequence,phase,gyro_raw_x,gyro_raw_y,"
              "gyro_raw_z,gyro_corrected_x,gyro_corrected_y,gyro_corrected_z\n";
}

void writeCsvSample(
    std::ofstream& output,
    const xreal::sensors::ImuSample& sample,
    std::string_view phase,
    const std::optional<xreal::sensors::GyroscopeBias>& bias)
{
    const auto hostNanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(
        sample.hostReceiveTimestamp.time_since_epoch()).count();
    output << hostNanoseconds << ',' << sample.deviceTimestamp.nanoseconds << ','
           << static_cast<unsigned int>(sample.packetSequence) << ',' << phase << ','
           << sample.gyroscopeRaw.x << ',' << sample.gyroscopeRaw.y << ','
           << sample.gyroscopeRaw.z;
    if (bias.has_value())
    {
        const auto corrected = xreal::sensors::applyGyroscopeBias(sample.gyroscopeRaw, *bias);
        output << ',' << corrected.x << ',' << corrected.y << ',' << corrected.z;
    }
    else
    {
        output << ",,,";
    }
    output << '\n';
}

void printBias(const xreal::sensors::GyroscopeBiasCalibrationResult& result)
{
    std::cout << "  Bias calibration: " << (result.accepted ? "accepted" : "rejected")
              << " (" << xreal::sensors::gyroscopeBiasCalibrationRejectionReasonText(
                     result.rejectionReason)
              << ")\n";
    if (result.biasRaw.has_value())
    {
        std::cout << "  Bias raw: [" << result.biasRaw->x << ", " << result.biasRaw->y
                  << ", " << result.biasRaw->z << "]\n";
    }
}

void printTrial(
    std::uint64_t index,
    const xreal::sensors::GyroscopeScaleCalibrationTrial& trial)
{
    std::cout << "\nTrial " << index << ": " << (trial.accepted ? "accepted" : "rejected")
              << '\n'
              << "  Rejection reason: "
              << xreal::sensors::gyroscopeScaleTrialRejectionReasonText(trial.rejectionReason)
              << '\n'
              << "  Duration: "
              << std::chrono::duration<double>(trial.integration.captureDuration).count()
              << " seconds\n"
              << "  Integrated raw angle: " << trial.integration.integratedRawAngle << '\n'
              << "  Expected angle: " << trial.expectedAngleDegrees << " degrees\n"
              << "  Trial scale: " << trial.scaleRawPerDegreePerSecond
              << " raw units/(degree/s)\n"
              << "  Direction: "
              << xreal::sensors::rotationDirectionText(trial.measuredDirection) << '\n'
              << "  Dominant axis: " << xreal::sensors::gyroscopeAxisText(trial.dominantAxis)
              << '\n'
              << "  Peak corrected raw rate: " << trial.integration.peakCorrectedRawRate
              << '\n'
              << "  Rotation packet rate: " << trial.integration.packetRate << " packets/s\n";
}

[[nodiscard]] xreal::sensors::GyroscopeScaleCalibrationTrial runTrial(
    const xreal::sensors::XrealDeviceInfo& device,
    const xreal::diagnostics::GyroScaleCalibrationOptions& options,
    std::uint64_t trialIndex)
{
    std::cout << "\n=== Trial " << trialIndex << " of " << options.trialCount << " ===\n"
              << "Place the glasses on a stable surface and keep them completely still.\n";

    std::ofstream csv;
    if (options.csvPrefix.has_value())
    {
        const std::string path = *options.csvPrefix + "-trial-" + std::to_string(trialIndex) + ".csv";
        csv.open(path, std::ios::out | std::ios::trunc);
        if (!csv)
        {
            throw std::runtime_error("Failed to open CSV output: " + path);
        }
        writeCsvHeader(csv);
        std::cout << "  Trial CSV: " << path << '\n';
    }

    SampleQueue<xreal::sensors::ImuSample, 8192> queue;
    std::atomic_uint64_t queueDrops{};
    xreal::sensors::XrealImuStream stream(device);
    if (!stream.start([&](std::span<const std::uint8_t, 64>, const xreal::sensors::ImuSample& sample) {
            if (!queue.tryPush(sample))
            {
                ++queueDrops;
            }
        }))
    {
        throw std::runtime_error("Failed to start the IMU stream.");
    }

    xreal::sensors::GyroscopeBiasCalibrator biasCalibrator(options.biasConfiguration);
    const auto biasDeadline = std::chrono::steady_clock::now()
        + options.biasConfiguration.calibrationDuration
        + options.biasConfiguration.warmupDuration.value_or(0ns) + 10s;
    while (!biasCalibrator.isComplete() && std::chrono::steady_clock::now() < biasDeadline
           && stream.isRunning())
    {
        xreal::sensors::ImuSample sample;
        bool consumed{};
        while (queue.tryPop(sample))
        {
            consumed = true;
            biasCalibrator.consume(sample);
            if (csv)
            {
                writeCsvSample(csv, sample, "bias-calibration", std::nullopt);
            }
        }
        if (!consumed)
        {
            std::this_thread::sleep_for(2ms);
        }
    }

    const auto biasResult = biasCalibrator.finish();
    printBias(biasResult);
    if (!biasResult.accepted || !biasResult.biasRaw.has_value())
    {
        stream.stop();
        return xreal::sensors::GyroscopeScaleTrialCalibrator(
            options.scaleConfiguration,
            std::nullopt).finish();
    }

    std::cout << "Prepare a controlled " << options.scaleConfiguration.expectedAngleDegrees
              << " degree rotation around axis "
              << xreal::sensors::gyroscopeAxisText(options.scaleConfiguration.axis)
              << ". Avoid tilting around the other axes.\n";
    for (std::uint64_t remaining = options.countdownSeconds; remaining > 0U; --remaining)
    {
        std::cout << "  Starting in " << remaining << "...\n";
        const auto countdownDeadline = std::chrono::steady_clock::now() + 1s;
        while (std::chrono::steady_clock::now() < countdownDeadline)
        {
            xreal::sensors::ImuSample sample;
            while (queue.tryPop(sample))
            {
                if (csv)
                {
                    writeCsvSample(csv, sample, "countdown", biasResult.biasRaw);
                }
            }
            std::this_thread::sleep_for(2ms);
        }
    }

    std::cout << "START ROTATION NOW. Stop after the known angle, then hold still.\n";
    xreal::sensors::GyroscopeScaleTrialCalibrator calibrator(
        options.scaleConfiguration,
        biasResult.biasRaw);
    const auto trialDeadline = std::chrono::steady_clock::now()
        + options.scaleConfiguration.maximumRotationDuration
        + options.scaleConfiguration.stillnessDuration + 30s;
    auto previousPhase = calibrator.phase();
    while (!calibrator.isComplete() && std::chrono::steady_clock::now() < trialDeadline
           && stream.isRunning())
    {
        xreal::sensors::ImuSample sample;
        bool consumed{};
        while (queue.tryPop(sample))
        {
            consumed = true;
            calibrator.consume(sample);
            if (csv)
            {
                writeCsvSample(csv, sample, "rotation-trial", biasResult.biasRaw);
            }
            if (calibrator.phase() != previousPhase)
            {
                previousPhase = calibrator.phase();
                if (previousPhase == xreal::sensors::GyroscopeScaleTrialPhase::rotating)
                {
                    std::cout << "  Rotation detected.\n";
                }
                else if (previousPhase
                    == xreal::sensors::GyroscopeScaleTrialPhase::waitingForStillnessAfterRotation)
                {
                    std::cout << "  Stop detected; keep the glasses still.\n";
                }
            }
            if (calibrator.isComplete())
            {
                break;
            }
        }
        if (!consumed)
        {
            std::this_thread::sleep_for(2ms);
        }
    }

    stream.stop();
    auto trial = calibrator.finish();
    const auto statistics = stream.statistics();
    if (queueDrops.load() != 0U || statistics.dropped != 0U || statistics.invalid != 0U
        || statistics.outOfSequence != 0U || !stream.errorMessage().empty())
    {
        trial.accepted = false;
        trial.rejectionReason = xreal::sensors::GyroscopeScaleTrialRejectionReason::streamDataLoss;
        trial.scaleRawPerDegreePerSecond = 0.0;
    }
    if (csv)
    {
        csv.flush();
        if (!csv)
        {
            throw std::runtime_error("Failed while writing the trial CSV.");
        }
    }
    return trial;
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

    const auto parsed = xreal::diagnostics::parseGyroScaleCalibrationOptions(arguments);
    if (parsed.showHelp)
    {
        std::cout << xreal::diagnostics::gyroScaleCalibrationUsage();
        return 0;
    }
    if (!parsed.options.has_value())
    {
        std::cerr << "Error: " << parsed.error << '\n'
                  << xreal::diagnostics::gyroScaleCalibrationUsage();
        return 2;
    }

    try
    {
        const auto& options = *parsed.options;
        const xreal::sensors::XrealDevice enumerator;
        const auto device = xreal::sensors::XrealImuStream::findInterface(enumerator.enumerate());
        if (!device.has_value())
        {
            std::cerr << "No exact XREAL Air 2 Ultra VID 0x3318/PID 0x0426/interface 2 was found.\n";
            return 1;
        }

        if (options.recordOnly)
        {
            return xreal::diagnostics::runGyroscopeRecordOnlyMode(*device, options);
        }

        std::cout << "Experimental gyroscope scale calibration\n"
                  << "  Axis: " << xreal::sensors::gyroscopeAxisText(options.scaleConfiguration.axis)
                  << "\n  Expected angle: " << options.scaleConfiguration.expectedAngleDegrees
                  << " degrees\n  Direction: "
                  << xreal::sensors::rotationDirectionText(options.scaleConfiguration.direction)
                  << "\n  Trials: " << options.trialCount << "\n\n"
                  << "Use slow, smooth rotations and a marked turntable or printed reference if possible.\n"
                  << "Include clockwise and counter-clockwise trials when direction is auto.\n"
                  << "A hand-performed rotation is not laboratory-grade calibration.\n";

        std::vector<xreal::sensors::GyroscopeScaleCalibrationTrial> trials;
        trials.reserve(static_cast<std::size_t>(options.trialCount));
        for (std::uint64_t index = 1U; index <= options.trialCount; ++index)
        {
            auto trial = runTrial(*device, options, index);
            printTrial(index, trial);
            trials.push_back(std::move(trial));
        }

        const auto result = xreal::sensors::calculateGyroscopeScaleCalibration(
            options.scaleConfiguration,
            trials);
        std::cout << "\nFinal calibration: " << (result.accepted ? "accepted" : "rejected")
                  << "\n  Rejection reason: "
                  << xreal::sensors::gyroscopeScaleCalibrationRejectionReasonText(
                         result.rejectionReason)
                  << "\n  Accepted trials: " << result.acceptedTrialCount
                  << "\n  Rejected trials: " << result.rejectedTrialCount
                  << "\n  Median: " << result.statistics.median
                  << "\n  Mean: " << result.statistics.mean
                  << "\n  Standard deviation: " << result.statistics.standardDeviation
                  << "\n  Coefficient of variation: " << result.statistics.coefficientOfVariation
                  << "\n  Raw units/(degree/s): "
                  << result.scale.rawUnitsPerDegreePerSecond
                  << "\n  Raw units/(radian/s): "
                  << result.scale.rawUnitsPerRadianPerSecond << '\n';

        const xreal::sensors::GyroscopeBiasCalibrationDevice calibrationDevice{
            xreal::sensors::XrealImuStream::vendorId,
            xreal::sensors::XrealImuStream::productId,
            xreal::sensors::XrealImuStream::interfaceNumber,
            narrowAscii(device->productName),
        };
        std::ofstream output(options.outputPath, std::ios::out | std::ios::trunc);
        if (!output)
        {
            std::cerr << "Failed to open JSON output: " << options.outputPath << '\n';
            return 1;
        }
        output << xreal::sensors::serializeGyroscopeScaleCalibrationJson(
            calibrationDevice,
            result);
        if (!output)
        {
            std::cerr << "Failed while writing JSON output: " << options.outputPath << '\n';
            return 1;
        }
        std::cout << "  JSON profile: " << options.outputPath << '\n';
        return result.accepted ? 0 : 1;
    }
    catch (const std::exception& exception)
    {
        std::cerr << "Gyroscope scale calibration failed: " << exception.what() << '\n';
        return 1;
    }
}
