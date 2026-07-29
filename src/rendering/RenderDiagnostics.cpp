#include "rendering/RenderDiagnostics.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace xreal::rendering
{

FrameTimingTracker::FrameTimingTracker(std::optional<double> targetFramesPerSecond)
    : targetFramesPerSecond_(targetFramesPerSecond)
{
}

void FrameTimingTracker::beginFrame() noexcept
{
    frameStart_ = std::chrono::steady_clock::now();
    if (previousFrameStart_ != std::chrono::steady_clock::time_point{})
    {
        const double seconds = std::chrono::duration<double>(frameStart_ - previousFrameStart_).count();
        if (seconds > 0.0)
        {
            const double fps = 1.0 / seconds;
            if (minimumFramesPerSecond_ == 0.0 || fps < minimumFramesPerSecond_)
            {
                minimumFramesPerSecond_ = fps;
            }
            maximumFramesPerSecond_ = std::max(maximumFramesPerSecond_, fps);
        }
    }
    previousFrameStart_ = frameStart_;
}

void FrameTimingTracker::recordSnapshot(
    const RenderOrientationSnapshot* snapshot,
    bool repeated,
    bool invalid,
    bool predictionFallback) noexcept
{
    repeatedSnapshots_ += repeated ? 1U : 0U;
    invalidSnapshots_ += invalid ? 1U : 0U;
    predictionFallbacks_ += predictionFallback ? 1U : 0U;
    if (snapshot == nullptr)
    {
        return;
    }
    if (firstSnapshotSequence_ == 0U)
    {
        firstSnapshotSequence_ = snapshot->sequence;
    }
    lastSnapshotSequence_ = snapshot->sequence;
    latestDeviceTimestamp_ = snapshot->deviceTimestampNanoseconds;
    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    const auto nowNanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();
    const double ageMilliseconds = std::max(
        0.0,
        static_cast<double>(nowNanoseconds - static_cast<std::int64_t>(
            snapshot->hostPublishTimestampNanoseconds)) * 1.0e-6);
    ++snapshotCount_;
    snapshotAgeSumMilliseconds_ += ageMilliseconds;
    maximumSnapshotAgeMilliseconds_ = std::max(maximumSnapshotAgeMilliseconds_, ageMilliseconds);
    effectiveLeadSumMilliseconds_ += snapshot->predictionHorizonMilliseconds - ageMilliseconds;
}

void FrameTimingTracker::endFrame(bool presented) noexcept
{
    const auto now = std::chrono::steady_clock::now();
    const double milliseconds = std::chrono::duration<double, std::milli>(now - frameStart_).count();
    ++frameCount_;
    presentCount_ += presented ? 1U : 0U;
    frameTimeSumMilliseconds_ += milliseconds;
    maximumFrameTimeMilliseconds_ = std::max(maximumFrameTimeMilliseconds_, milliseconds);
    const double slowThreshold = targetFramesPerSecond_.has_value()
        ? 1500.0 / *targetFramesPerSecond_ : 25.0;
    slowFrames_ += milliseconds > slowThreshold ? 1U : 0U;
}

FrameTimingStatistics FrameTimingTracker::statistics() const noexcept
{
    FrameTimingStatistics result;
    result.renderFrameCount = frameCount_;
    result.presentCount = presentCount_;
    result.elapsedSeconds = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - start_).count();
    result.averageFramesPerSecond = result.elapsedSeconds > 0.0
        ? static_cast<double>(frameCount_) / result.elapsedSeconds : 0.0;
    result.minimumFramesPerSecond = minimumFramesPerSecond_;
    result.maximumFramesPerSecond = maximumFramesPerSecond_;
    result.averageFrameTimeMilliseconds = frameCount_ > 0U
        ? frameTimeSumMilliseconds_ / static_cast<double>(frameCount_) : 0.0;
    result.maximumFrameTimeMilliseconds = maximumFrameTimeMilliseconds_;
    result.slowFrameCount = slowFrames_;
    result.repeatedOrientationSnapshots = repeatedSnapshots_;
    result.invalidOrientationSnapshots = invalidSnapshots_;
    result.predictionFallbacks = predictionFallbacks_;
    result.averageSnapshotAgeMilliseconds = snapshotCount_ > 0U
        ? snapshotAgeSumMilliseconds_ / static_cast<double>(snapshotCount_) : 0.0;
    result.maximumSnapshotAgeMilliseconds = maximumSnapshotAgeMilliseconds_;
    result.averageApproximateEffectiveLeadMilliseconds = snapshotCount_ > 0U
        ? effectiveLeadSumMilliseconds_ / static_cast<double>(snapshotCount_) : 0.0;
    result.sensorPublishRate = result.elapsedSeconds > 0.0 && lastSnapshotSequence_ >= firstSnapshotSequence_
        ? static_cast<double>(lastSnapshotSequence_ - firstSnapshotSequence_ + 1U)
            / result.elapsedSeconds : 0.0;
    result.latestDeviceTimestamp = latestDeviceTimestamp_;
    return result;
}

std::string rendererStartupStateText(RendererStartupState state)
{
    switch (state)
    {
    case RendererStartupState::initializingRenderer: return "initializing_renderer";
    case RendererStartupState::openingImu: return "opening_imu";
    case RendererStartupState::warmingUpGyro: return "warming_up_gyro";
    case RendererStartupState::calibratingGyro: return "calibrating_gyro";
    case RendererStartupState::initializingFusion: return "initializing_fusion";
    case RendererStartupState::ready: return "ready";
    case RendererStartupState::error: return "error";
    case RendererStartupState::shuttingDown: return "shutting_down";
    }
    return "error";
}

} // namespace xreal::rendering
