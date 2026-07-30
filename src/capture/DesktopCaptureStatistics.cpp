#include "capture/DesktopCaptureStatistics.hpp"

#include <algorithm>

namespace xreal::capture
{

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

} // namespace xreal::capture
