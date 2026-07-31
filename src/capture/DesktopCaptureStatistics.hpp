#pragma once

#include "capture/DesktopCaptureFrame.hpp"
#include "capture/DesktopFrameRetention.hpp"

#include <cstdint>
#include <array>
#include <cstddef>
#include <string>

namespace xreal::capture
{

inline constexpr std::size_t desktopCaptureTimingHistoryCapacity = 256U;

struct DesktopCaptureTimingPercentiles
{
    std::size_t sampleCount{};
    double p50Milliseconds{};
    double p95Milliseconds{};
    double p99Milliseconds{};
};

class DesktopCaptureTimingHistory
{
public:
    void add(double milliseconds) noexcept;
    [[nodiscard]] DesktopCaptureTimingPercentiles statistics() const noexcept;
    [[nodiscard]] std::size_t size() const noexcept;

private:
    std::array<double, desktopCaptureTimingHistoryCapacity> values_{};
    std::size_t next_{};
    std::size_t size_{};
};

struct DesktopCaptureStatistics
{
    DesktopCaptureStatus state{DesktopCaptureStatus::disabled};
    std::uint64_t captureAttempts{};
    std::uint64_t acquiredFrames{};
    std::uint64_t stagingCopies{};
    std::uint64_t stagingMaps{};
    std::uint64_t stagingMapSuccesses{};
    std::uint64_t cpuBuffersCreated{};
    std::uint64_t cpuFramesPublished{};
    std::uint64_t latestPublishedSequence{};
    std::uint64_t waitTimeouts{};
    std::uint64_t accessLossEvents{};
    std::uint64_t recreationAttempts{};
    std::uint64_t successfulRecreations{};
    std::uint64_t captureErrors{};
    std::uint64_t gpuCopies{};
    std::uint64_t cpuFallbackCopies{};
    std::uint64_t cpuFallbackBytes{};
    std::uint64_t originalSourceBytes{};
    std::uint64_t cropBytes{};
    std::uint64_t scaledTargetBytes{};
    std::uint64_t stagingBytesMapped{};
    std::uint64_t cpuBytesCopied{};
    std::uint64_t gpuScaleDraws{};
    std::uint64_t scalerResourceRecreations{};
    std::uint64_t stagingRingContentions{};
    std::uint64_t recoveryGeneration{};
    std::uint64_t dirtyRectCount{};
    std::uint64_t moveRectCount{};
    std::uint64_t pointerMetadataCount{};
    double capturedFramesPerSecond{};
    double averageAcquisitionIntervalMilliseconds{};
    double maximumAcquisitionIntervalMilliseconds{};
    std::uint32_t sourceWidth{};
    std::uint32_t sourceHeight{};
    std::uint32_t cropX{};
    std::uint32_t cropY{};
    std::uint32_t cropWidth{};
    std::uint32_t cropHeight{};
    std::uint32_t transferWidth{};
    std::uint32_t transferHeight{};
    std::uint32_t sourceFormat{};
    DesktopRotation sourceRotation{DesktopRotation::identity};
    DesktopFrameAvailability availability{DesktopFrameAvailability::noFrameEver};
    double averageAcquireMilliseconds{};
    double maximumAcquireMilliseconds{};
    double averageGpuScaleSubmissionMilliseconds{};
    double maximumGpuScaleSubmissionMilliseconds{};
    double averageMapWaitMilliseconds{};
    double maximumMapWaitMilliseconds{};
    double averageCpuRepackMilliseconds{};
    double maximumCpuRepackMilliseconds{};
    double acquireP50Milliseconds{};
    double acquireP95Milliseconds{};
    double acquireP99Milliseconds{};
    double gpuScaleSubmissionP50Milliseconds{};
    double gpuScaleSubmissionP95Milliseconds{};
    double gpuScaleSubmissionP99Milliseconds{};
    double mapWaitP50Milliseconds{};
    double mapWaitP95Milliseconds{};
    double mapWaitP99Milliseconds{};
    double cpuRepackP50Milliseconds{};
    double cpuRepackP95Milliseconds{};
    double cpuRepackP99Milliseconds{};
    std::string transferMode{"unavailable"};
    std::string lastError;
};

class DesktopCaptureStatisticsTracker
{
public:
    void recordAcquired(std::chrono::steady_clock::time_point timestamp) noexcept;
    void recordAcquireDuration(double milliseconds) noexcept;
    void recordScaleSubmission(double milliseconds) noexcept;
    void recordMapWait(double milliseconds) noexcept;
    void recordCpuRepack(double milliseconds) noexcept;
    [[nodiscard]] DesktopCaptureStatistics snapshot() const noexcept;
    [[nodiscard]] DesktopCaptureStatistics detailedSnapshot() const noexcept;
    DesktopCaptureStatistics values;

private:
    std::chrono::steady_clock::time_point firstAcquisition_{};
    std::chrono::steady_clock::time_point previousAcquisition_{};
    double totalIntervalMilliseconds_{};
    double totalAcquireMilliseconds_{};
    double totalScaleSubmissionMilliseconds_{};
    double totalMapWaitMilliseconds_{};
    double totalCpuRepackMilliseconds_{};
    std::uint64_t acquireSamples_{};
    std::uint64_t scaleSamples_{};
    std::uint64_t mapSamples_{};
    std::uint64_t repackSamples_{};
    DesktopCaptureTimingHistory acquireHistory_;
    DesktopCaptureTimingHistory scaleHistory_;
    DesktopCaptureTimingHistory mapHistory_;
    DesktopCaptureTimingHistory repackHistory_;
};

} // namespace xreal::capture
