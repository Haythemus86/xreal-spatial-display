#include "capture/DesktopCaptureStatistics.hpp"

#include <algorithm>
#include <cmath>

namespace xreal::capture
{
namespace
{

[[nodiscard]] double percentile(
    const std::array<double, desktopCaptureTimingHistoryCapacity>& sorted,
    std::size_t count,
    double fraction) noexcept
{
    if (count == 0U)
    {
        return 0.0;
    }
    const double position = fraction * static_cast<double>(count - 1U);
    const auto lower = static_cast<std::size_t>(std::floor(position));
    const auto upper = static_cast<std::size_t>(std::ceil(position));
    const double weight = position - static_cast<double>(lower);
    return sorted[lower] + (sorted[upper] - sorted[lower]) * weight;
}

} // namespace

void DesktopCaptureTimingHistory::add(double milliseconds) noexcept
{
    if (!std::isfinite(milliseconds) || milliseconds < 0.0)
    {
        return;
    }
    values_[next_] = milliseconds;
    next_ = (next_ + 1U) % values_.size();
    size_ = std::min(size_ + 1U, values_.size());
}

DesktopCaptureTimingPercentiles DesktopCaptureTimingHistory::statistics() const noexcept
{
    DesktopCaptureTimingPercentiles result;
    result.sampleCount = size_;
    if (size_ == 0U)
    {
        return result;
    }
    std::array<double, desktopCaptureTimingHistoryCapacity> sorted{};
    for (std::size_t index = 0U; index < size_; ++index)
    {
        const std::size_t source = size_ == values_.size()
            ? (next_ + index) % values_.size() : index;
        sorted[index] = values_[source];
    }
    std::sort(sorted.begin(), sorted.begin() + static_cast<std::ptrdiff_t>(size_));
    result.p50Milliseconds = percentile(sorted, size_, 0.50);
    result.p95Milliseconds = percentile(sorted, size_, 0.95);
    result.p99Milliseconds = percentile(sorted, size_, 0.99);
    return result;
}

std::size_t DesktopCaptureTimingHistory::size() const noexcept { return size_; }

void DesktopCaptureStatisticsTracker::recordAcquired(
    std::chrono::steady_clock::time_point timestamp) noexcept
{
    if (values.acquiredFrames == 0U)
    {
        firstAcquisition_ = timestamp;
    }
    else
    {
        const double interval = std::chrono::duration<double, std::milli>(
            timestamp - previousAcquisition_).count();
        totalIntervalMilliseconds_ += interval;
        values.maximumAcquisitionIntervalMilliseconds = std::max(
            values.maximumAcquisitionIntervalMilliseconds, interval);
        values.averageAcquisitionIntervalMilliseconds = totalIntervalMilliseconds_
            / static_cast<double>(values.acquiredFrames);
    }
    previousAcquisition_ = timestamp;
    ++values.acquiredFrames;
    const double elapsed = std::chrono::duration<double>(timestamp - firstAcquisition_).count();
    if (elapsed > 0.0)
    {
        values.capturedFramesPerSecond = static_cast<double>(values.acquiredFrames - 1U) / elapsed;
    }
}

DesktopCaptureStatistics DesktopCaptureStatisticsTracker::snapshot() const noexcept
{
    return values;
}

DesktopCaptureStatistics DesktopCaptureStatisticsTracker::detailedSnapshot() const noexcept
{
    auto result = values;
    const auto acquire = acquireHistory_.statistics();
    const auto scale = scaleHistory_.statistics();
    const auto map = mapHistory_.statistics();
    const auto repack = repackHistory_.statistics();
    result.acquireP50Milliseconds = acquire.p50Milliseconds;
    result.acquireP95Milliseconds = acquire.p95Milliseconds;
    result.acquireP99Milliseconds = acquire.p99Milliseconds;
    result.gpuScaleSubmissionP50Milliseconds = scale.p50Milliseconds;
    result.gpuScaleSubmissionP95Milliseconds = scale.p95Milliseconds;
    result.gpuScaleSubmissionP99Milliseconds = scale.p99Milliseconds;
    result.mapWaitP50Milliseconds = map.p50Milliseconds;
    result.mapWaitP95Milliseconds = map.p95Milliseconds;
    result.mapWaitP99Milliseconds = map.p99Milliseconds;
    result.cpuRepackP50Milliseconds = repack.p50Milliseconds;
    result.cpuRepackP95Milliseconds = repack.p95Milliseconds;
    result.cpuRepackP99Milliseconds = repack.p99Milliseconds;
    return result;
}

void DesktopCaptureStatisticsTracker::recordAcquireDuration(double milliseconds) noexcept
{
    if (!std::isfinite(milliseconds) || milliseconds < 0.0) { return; }
    totalAcquireMilliseconds_ += milliseconds;
    ++acquireSamples_;
    values.averageAcquireMilliseconds = totalAcquireMilliseconds_ / acquireSamples_;
    values.maximumAcquireMilliseconds = std::max(values.maximumAcquireMilliseconds, milliseconds);
    acquireHistory_.add(milliseconds);
}

void DesktopCaptureStatisticsTracker::recordScaleSubmission(double milliseconds) noexcept
{
    if (!std::isfinite(milliseconds) || milliseconds < 0.0) { return; }
    totalScaleSubmissionMilliseconds_ += milliseconds;
    ++scaleSamples_;
    values.averageGpuScaleSubmissionMilliseconds =
        totalScaleSubmissionMilliseconds_ / scaleSamples_;
    values.maximumGpuScaleSubmissionMilliseconds = std::max(
        values.maximumGpuScaleSubmissionMilliseconds, milliseconds);
    scaleHistory_.add(milliseconds);
}

void DesktopCaptureStatisticsTracker::recordMapWait(double milliseconds) noexcept
{
    if (!std::isfinite(milliseconds) || milliseconds < 0.0) { return; }
    totalMapWaitMilliseconds_ += milliseconds;
    ++mapSamples_;
    values.averageMapWaitMilliseconds = totalMapWaitMilliseconds_ / mapSamples_;
    values.maximumMapWaitMilliseconds = std::max(values.maximumMapWaitMilliseconds, milliseconds);
    mapHistory_.add(milliseconds);
}

void DesktopCaptureStatisticsTracker::recordCpuRepack(double milliseconds) noexcept
{
    if (!std::isfinite(milliseconds) || milliseconds < 0.0) { return; }
    totalCpuRepackMilliseconds_ += milliseconds;
    ++repackSamples_;
    values.averageCpuRepackMilliseconds = totalCpuRepackMilliseconds_ / repackSamples_;
    values.maximumCpuRepackMilliseconds = std::max(
        values.maximumCpuRepackMilliseconds, milliseconds);
    repackHistory_.add(milliseconds);
}

} // namespace xreal::capture
