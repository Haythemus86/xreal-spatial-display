#include "diagnostics/GyroRecordOnlyMode.hpp"

#include "sensors/GyroscopeBiasCalibration.hpp"
#include "sensors/GyroscopeOfflineAnalysis.hpp"
#include "sensors/XrealImuStream.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

namespace xreal::diagnostics
{
namespace
{

using namespace std::chrono_literals;

volatile std::sig_atomic_t interruptionRequested{};

extern "C" void handleInterrupt(int) noexcept
{
    interruptionRequested = 1;
}

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

void printBiasResult(const sensors::GyroscopeBiasCalibrationResult& result)
{
    std::cout << "Bias calibration: " << (result.accepted ? "accepted" : "rejected")
              << '\n';
    if (result.biasRaw.has_value())
    {
        std::cout << "  Bias raw: [" << result.biasRaw->x << ", " << result.biasRaw->y
                  << ", " << result.biasRaw->z << "]\n";
    }
    else
    {
        std::cout << "  Reason: "
                  << sensors::gyroscopeBiasCalibrationRejectionReasonText(result.rejectionReason)
                  << '\n';
    }
}

void writeOutputs(
    const GyroScaleCalibrationOptions& options,
    const sensors::GyroscopeRecordingMetadata& metadata,
    const sensors::GyroscopeOfflineAnalysisResult& offlineAnalysis)
{
    const auto& analysis = offlineAnalysis.backwardCompatible;
    std::ofstream csv(options.csvOutputPath, std::ios::out | std::ios::trunc);
    if (!csv)
    {
        throw std::runtime_error("Failed to open CSV output: " + options.csvOutputPath);
    }
    csv << sensors::gyroscopeRecordingCsvHeader(
        options.includeAccelerometer,
        options.includeHostTimestamps);
    for (const auto& sample : analysis.samples)
    {
        csv << sensors::serializeGyroscopeRecordingCsvRow(
            sample,
            analysis.bias,
            options.includeAccelerometer,
            options.includeHostTimestamps);
    }
    if (!csv)
    {
        throw std::runtime_error("Failed while writing CSV output: " + options.csvOutputPath);
    }

    std::ofstream json(options.analysisOutputPath, std::ios::out | std::ios::trunc);
    if (!json)
    {
        throw std::runtime_error("Failed to open analysis output: " + options.analysisOutputPath);
    }
    json << sensors::serializeGyroscopeOfflineAnalysisJson(metadata, offlineAnalysis);
    if (!json)
    {
        throw std::runtime_error("Failed while writing analysis output: " + options.analysisOutputPath);
    }
}

void printSummary(
    const GyroScaleCalibrationOptions& options,
    const sensors::GyroscopeRecordingMetadata& metadata,
    const sensors::GyroscopeOfflineAnalysisResult& offlineAnalysis)
{
    const auto& analysis = offlineAnalysis.backwardCompatible;
    std::cout << "\nRecord-only analysis summary\n"
              << "  Bias calibration: "
              << (metadata.biasCalibration.accepted ? "accepted" : "rejected") << '\n'
              << "  Actual recording duration: " << metadata.measuredDurationSeconds << " seconds\n"
              << "  Samples: " << analysis.samples.size() << '\n'
              << "  Sequence gaps: " << analysis.sequenceGaps << '\n'
              << "  Malformed packets: " << metadata.malformedPackets << '\n'
              << "  Valid timestamps: " << analysis.validTimestampSamples << '\n'
              << "  Invalid timestamps: " << analysis.invalidTimestampSamples << '\n'
              << "  Dominant axis: " << sensors::gyroscopeAxisText(analysis.dominantAxis) << '\n'
              << "  Detected segments: " << analysis.segments.size() << '\n';
    if (analysis.bestSegmentIndex.has_value())
    {
        const auto& segment = analysis.segments[*analysis.bestSegmentIndex];
        std::cout << "  Selected segment: " << *analysis.bestSegmentIndex << '\n'
                  << "  Selected duration: "
                  << std::chrono::duration<double>(segment.duration).count() << " seconds\n"
                  << "  Selected direction: "
                  << sensors::rotationDirectionText(segment.direction) << '\n'
                  << "  Cross-axis ratio: " << segment.crossAxisRatio << '\n'
                  << "  Signed integrated raw angle: [" << segment.integratedRawAngle.x
                  << ", " << segment.integratedRawAngle.y << ", "
                  << segment.integratedRawAngle.z << "]\n";
    }
    else
    {
        std::cout << "  No usable segment: "
                  << sensors::gyroscopeRecordingAnalysisFailureText(analysis.failure) << '\n';
    }
    if (analysis.scaleEstimate.available)
    {
        std::cout << "  Experimental scale axis: "
                  << sensors::gyroscopeAxisText(analysis.scaleEstimate.axis) << '\n'
                  << "  Raw units/(degree/s): "
                  << analysis.scaleEstimate.rawUnitsPerDegreePerSecond << '\n'
                  << "  Raw units/(radian/s): "
                  << analysis.scaleEstimate.rawUnitsPerRadianPerSecond << '\n'
                  << "  Confidence: " << analysis.scaleEstimate.confidence << '\n';
    }
    for (const auto& warning : analysis.warnings)
    {
        std::cout << "  Warning: " << warning << '\n';
    }
    for (const auto& warning : offlineAnalysis.warnings)
    {
        if (std::find(analysis.warnings.begin(), analysis.warnings.end(), warning)
            == analysis.warnings.end())
        {
            std::cout << "  Refined analysis warning: " << warning << '\n';
        }
    }
    std::cout << "  CSV: " << options.csvOutputPath << '\n'
              << "  Analysis JSON: " << options.analysisOutputPath << '\n';
}

} // namespace

int runGyroscopeRecordOnlyMode(
    const sensors::XrealDeviceInfo& device,
    const GyroScaleCalibrationOptions& options)
{
    interruptionRequested = 0;
    const auto previousHandler = std::signal(SIGINT, handleInterrupt);
    auto queue = std::make_unique<SampleQueue<sensors::ImuSample, 65536>>();
    std::atomic_uint64_t queueDrops{};
    sensors::XrealImuStream stream(device);
    if (!stream.start([&](std::span<const std::uint8_t, 64>, const sensors::ImuSample& sample) {
            if (!queue->tryPush(sample))
            {
                ++queueDrops;
            }
        }))
    {
        std::signal(SIGINT, previousHandler);
        std::wcerr << L"Failed to start IMU acquisition: " << stream.errorMessage() << L'\n';
        return 1;
    }

    std::cout << "Record-only mode: ordinary movement never aborts capture or analysis.\n"
              << "Place the glasses completely still for runtime bias calibration.\n";
    sensors::GyroscopeBiasCalibrator biasCalibrator(options.biasConfiguration);
    const auto biasDeadline = std::chrono::steady_clock::now()
        + options.biasConfiguration.calibrationDuration
        + options.biasConfiguration.warmupDuration.value_or(0ns) + 10s;
    while (!biasCalibrator.isComplete() && std::chrono::steady_clock::now() < biasDeadline
           && stream.isRunning() && interruptionRequested == 0)
    {
        sensors::ImuSample sample;
        bool consumed{};
        while (queue->tryPop(sample))
        {
            consumed = true;
            biasCalibrator.consume(sample);
        }
        if (!consumed) std::this_thread::sleep_for(2ms);
    }
    const auto biasResult = biasCalibrator.finish();
    printBiasResult(biasResult);
    if (!biasResult.accepted || !biasResult.biasRaw.has_value())
    {
        stream.stop();
        std::signal(SIGINT, previousHandler);
        return 1;
    }

    std::cout << "Prepare the controlled motion. Capture continues for the full configured duration.\n";
    for (std::uint64_t remaining = options.countdownSeconds;
         remaining > 0U && interruptionRequested == 0;
         --remaining)
    {
        std::cout << "  Recording starts in " << remaining << "...\n";
        const auto deadline = std::chrono::steady_clock::now() + 1s;
        while (std::chrono::steady_clock::now() < deadline && interruptionRequested == 0)
        {
            sensors::ImuSample discarded;
            while (queue->tryPop(discarded)) {}
            std::this_thread::sleep_for(2ms);
        }
    }

    std::vector<sensors::ImuSample> captured;
    const double requestedSeconds = std::chrono::duration<double>(options.recordDuration).count();
    captured.reserve(static_cast<std::size_t>(requestedSeconds * 1100.0) + 1024U);
    const auto start = std::chrono::steady_clock::now();
    const auto deadline = start + options.recordDuration;
    const auto printInterval = std::chrono::duration<double>(1.0 / options.printRateHz);
    auto nextPrint = start + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        printInterval);
    std::cout << "RECORDING NOW for " << requestedSeconds << " seconds. Ctrl+C stops early.\n";
    while (std::chrono::steady_clock::now() < deadline && stream.isRunning()
           && interruptionRequested == 0)
    {
        sensors::ImuSample sample;
        bool consumed{};
        while (queue->tryPop(sample))
        {
            consumed = true;
            if (sample.hostReceiveTimestamp <= deadline)
            {
                captured.push_back(sample);
            }
        }
        const auto now = std::chrono::steady_clock::now();
        if (now >= nextPrint)
        {
            std::cout << "  captured=" << captured.size() << " elapsed=" << std::fixed
                      << std::setprecision(1) << std::chrono::duration<double>(now - start).count()
                      << "s\n";
            nextPrint = now + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                printInterval);
        }
        if (!consumed) std::this_thread::sleep_for(1ms);
    }
    stream.stop();
    const auto stop = std::chrono::steady_clock::now();
    sensors::ImuSample remainingSample;
    while (queue->tryPop(remainingSample))
    {
        if (remainingSample.hostReceiveTimestamp <= deadline)
        {
            captured.push_back(remainingSample);
        }
    }
    std::signal(SIGINT, previousHandler);

    const auto statistics = stream.statistics();
    sensors::GyroscopeOfflineAnalysisConfig offlineConfiguration;
    offlineConfiguration.base = options.recordingAnalysisConfiguration;
    auto analysis = sensors::analyzeGyroscopeRecordingOffline(
        captured,
        *biasResult.biasRaw,
        offlineConfiguration);
    auto addWarning = [&](std::string warning) {
        analysis.backwardCompatible.warnings.push_back(warning);
        analysis.warnings.push_back(std::move(warning));
    };
    if (queueDrops.load() != 0U)
    {
        addWarning("The diagnostic queue dropped decoded samples during capture.");
    }
    if (statistics.invalid != 0U)
    {
        addWarning("The IMU stream reported malformed packets during capture.");
    }
    if (interruptionRequested != 0)
    {
        addWarning("The user interrupted capture before the requested duration elapsed.");
    }
    const std::wstring streamError = stream.errorMessage();
    if (!streamError.empty())
    {
        addWarning("The HID acquisition stream ended with an I/O error.");
    }

    const auto& backwardCompatible = analysis.backwardCompatible;
    const double measuredSeconds = backwardCompatible.samples.empty()
        ? std::chrono::duration<double>(stop - start).count()
        : std::chrono::duration<double>(backwardCompatible.x.captureDuration).count();
    const sensors::GyroscopeRecordingMetadata metadata{
        {sensors::XrealImuStream::vendorId,
         sensors::XrealImuStream::productId,
         sensors::XrealImuStream::interfaceNumber,
         narrowAscii(device.productName)},
        biasResult,
        requestedSeconds,
        measuredSeconds,
        statistics.invalid,
        statistics.outOfSequence,
        queueDrops.load(),
    };
    writeOutputs(options, metadata, analysis);
    printSummary(options, metadata, analysis);
    if (!streamError.empty())
    {
        std::wcerr << L"IMU acquisition error: " << streamError << L'\n';
        return 1;
    }
    if (!backwardCompatible.usable || !analysis.bestSegmentIndex.has_value())
    {
        std::cerr << "Recording completed, but no usable motion segment was found.\n";
        return 1;
    }
    return 0;
}

} // namespace xreal::diagnostics
