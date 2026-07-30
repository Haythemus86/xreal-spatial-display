#pragma once

#include "capture/DesktopCaptureBridge.hpp"
#include "capture/DesktopCaptureOptions.hpp"
#include "capture/DesktopCaptureStatistics.hpp"
#include "capture/DesktopRenderStages.hpp"

#include <chrono>
#include <cstdint>
#include <string>

namespace xreal::capture
{

struct DesktopRenderCaptureStatistics
{
    std::uint64_t renderedCaptureFrames{};
    std::uint64_t repeatedCaptureFrames{};
    double averageSourceFrameAgeMilliseconds{};
    double maximumSourceFrameAgeMilliseconds{};
};

class DesktopCaptureDiagnostics
{
public:
    void recordRendered(
        const DesktopCaptureFrame& frame,
        std::chrono::steady_clock::time_point now) noexcept;
    [[nodiscard]] DesktopRenderCaptureStatistics snapshot() const noexcept;

private:
    std::uint64_t previousSequence_{};
    DesktopRenderCaptureStatistics values_;
    double totalAgeMilliseconds_{};
};

struct DesktopCaptureSummary
{
    platform::windows::MonitorInformation renderMonitor;
    platform::windows::MonitorInformation captureMonitor;
    DesktopCaptureOptions options;
    DesktopCaptureStatistics capture;
    DesktopCaptureBridgeStatistics bridge;
    DesktopRenderCaptureStatistics render;
    DesktopRenderStageStatistics stages;
    bool sameAdapter{};
    bool sharedHandleSupported{};
    bool cpuFallbackUsed{};
    std::string finalError;
};

[[nodiscard]] std::string serializeDesktopCaptureSummaryJson(
    const DesktopCaptureSummary& summary);

} // namespace xreal::capture
