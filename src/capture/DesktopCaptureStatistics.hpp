#pragma once

#include "capture/DesktopCaptureFrame.hpp"

#include <cstdint>
#include <string>

namespace xreal::capture
{

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
    std::uint64_t recoveryGeneration{};
    std::uint64_t dirtyRectCount{};
    std::uint64_t moveRectCount{};
    std::uint64_t pointerMetadataCount{};
    double capturedFramesPerSecond{};
    double averageAcquisitionIntervalMilliseconds{};
    double maximumAcquisitionIntervalMilliseconds{};
    std::uint32_t sourceWidth{};
    std::uint32_t sourceHeight{};
    std::uint32_t sourceFormat{};
    DesktopRotation sourceRotation{DesktopRotation::identity};
    std::string transferMode{"unavailable"};
    std::string lastError;
};

class DesktopCaptureStatisticsTracker
{
public:
    void recordAcquired(std::chrono::steady_clock::time_point timestamp) noexcept;
    [[nodiscard]] DesktopCaptureStatistics snapshot() const noexcept;
    DesktopCaptureStatistics values;

private:
    std::chrono::steady_clock::time_point firstAcquisition_{};
    std::chrono::steady_clock::time_point previousAcquisition_{};
    double totalIntervalMilliseconds_{};
};

} // namespace xreal::capture
