#pragma once

#include "rendering/OrientationRenderBridge.hpp"

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>

namespace xreal::rendering
{

enum class RendererStartupState
{
    initializingRenderer,
    waitingForXrealSdkPose,
    openingImu,
    warmingUpGyro,
    calibratingGyro,
    initializingFusion,
    ready,
    error,
    shuttingDown,
};

struct FrameTimingStatistics
{
    std::uint64_t renderFrameCount{};
    std::uint64_t presentCount{};
    double elapsedSeconds{};
    double averageFramesPerSecond{};
    double minimumFramesPerSecond{};
    double maximumFramesPerSecond{};
    double averageFrameTimeMilliseconds{};
    double maximumFrameTimeMilliseconds{};
    std::uint64_t slowFrameCount{};
    std::uint64_t repeatedOrientationSnapshots{};
    std::uint64_t invalidOrientationSnapshots{};
    std::uint64_t predictionFallbacks{};
    double averageSnapshotAgeMilliseconds{};
    double maximumSnapshotAgeMilliseconds{};
    double averageApproximateEffectiveLeadMilliseconds{};
    double sensorPublishRate{};
    std::uint64_t latestDeviceTimestamp{};
};

class FrameTimingTracker
{
public:
    explicit FrameTimingTracker(std::optional<double> targetFramesPerSecond = std::nullopt);
    void beginFrame() noexcept;
    void recordSnapshot(
        const RenderOrientationSnapshot* snapshot,
        bool repeated,
        bool invalid,
        bool predictionFallback) noexcept;
    void endFrame(bool presented) noexcept;
    [[nodiscard]] FrameTimingStatistics statistics() const noexcept;

private:
    std::optional<double> targetFramesPerSecond_;
    std::chrono::steady_clock::time_point start_{std::chrono::steady_clock::now()};
    std::chrono::steady_clock::time_point frameStart_{start_};
    std::chrono::steady_clock::time_point previousFrameStart_{};
    std::uint64_t frameCount_{};
    std::uint64_t presentCount_{};
    std::uint64_t slowFrames_{};
    std::uint64_t repeatedSnapshots_{};
    std::uint64_t invalidSnapshots_{};
    std::uint64_t predictionFallbacks_{};
    std::uint64_t snapshotCount_{};
    std::uint64_t firstSnapshotSequence_{};
    std::uint64_t lastSnapshotSequence_{};
    std::uint64_t latestDeviceTimestamp_{};
    double frameTimeSumMilliseconds_{};
    double maximumFrameTimeMilliseconds_{};
    double minimumFramesPerSecond_{};
    double maximumFramesPerSecond_{};
    double snapshotAgeSumMilliseconds_{};
    double maximumSnapshotAgeMilliseconds_{};
    double effectiveLeadSumMilliseconds_{};
};

[[nodiscard]] std::string rendererStartupStateText(RendererStartupState state);

} // namespace xreal::rendering
