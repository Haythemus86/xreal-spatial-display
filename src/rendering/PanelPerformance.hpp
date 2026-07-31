#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>

namespace xreal::rendering
{

inline constexpr std::size_t frameHistoryCapacity = 1024U;

struct FramePercentiles
{
    std::size_t sampleCount{};
    double averageMilliseconds{};
    double p50Milliseconds{};
    double p95Milliseconds{};
    double p99Milliseconds{};
    double maximumMilliseconds{};
    std::size_t overBudget33Milliseconds{};
};

class FrameTimeHistory
{
public:
    void add(double milliseconds) noexcept;
    void clear() noexcept;
    [[nodiscard]] std::size_t size() const noexcept;
    [[nodiscard]] FramePercentiles statistics() const noexcept;

private:
    std::array<double, frameHistoryCapacity> values_{};
    std::size_t next_{};
    std::size_t size_{};
};

struct BandwidthEstimate
{
    bool valid{};
    std::uint64_t bytesPerFrame{};
    double mebibytesPerSecond{};
    bool warning{};
};

struct PanelDrawPlan
{
    std::size_t visiblePanels{};
    std::size_t baseDrawCalls{};
    std::size_t overlayDrawCalls{};
    std::size_t auxiliaryDrawCalls{};
    std::size_t totalDrawCalls{};
    std::size_t presentCalls{};
};

struct MultiPanelPerformanceCounters
{
    FrameTimeHistory frameTimes;
    FrameTimeHistory presentTimes;
    FrameTimeHistory cpuUpdateTimes;
    FrameTimeHistory drawSubmissionTimes;
    std::uint64_t frames{};
    std::uint64_t presents{};
    std::uint64_t drawCalls{};
    std::uint64_t baseDrawCalls{};
    std::uint64_t overlayDrawCalls{};
    std::uint64_t auxiliaryDrawCalls{};
    std::uint64_t stateSetCalls{};
    std::uint64_t shaderResourceBindCalls{};
    std::uint64_t samplerBindCalls{};
    std::uint64_t constantBufferUpdates{};
    std::uint64_t instanceBufferUpdates{};
    std::uint64_t textureUploads{};
    std::uint64_t captureFrames{};
    std::uint64_t repeatedFrames{};
    std::uint64_t droppedFrames{};
    std::uint64_t resourcesCreatedAtStartup{};
    std::uint64_t resourcesCreatedSteadyState{};
    std::uint64_t flushCalls{};
    std::uint64_t workingSetBytes{};
    std::uint64_t privateBytes{};
    std::uint64_t initialWorkingSetBytes{};
    std::uint64_t peakWorkingSetBytes{};
    std::uint64_t initialPrivateBytes{};
    std::uint64_t peakPrivateBytes{};
};

[[nodiscard]] BandwidthEstimate estimateBgraBandwidth(
    std::uint32_t width,
    std::uint32_t height,
    double framesPerSecond,
    std::size_t sourceCount) noexcept;
[[nodiscard]] PanelDrawPlan makePanelDrawPlan(
    std::size_t visiblePanels,
    bool overlayEnabled,
    bool backgroundGrid,
    bool worldAxes) noexcept;
[[nodiscard]] std::string serializeMultiPanelPerformanceJson(
    const MultiPanelPerformanceCounters& counters,
    double elapsedSeconds,
    std::size_t panelCount,
    std::size_t sourceCount,
    std::string_view adapter,
    std::string_view output,
    std::string_view buildConfiguration);

} // namespace xreal::rendering
