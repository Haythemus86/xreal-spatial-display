#include "capture/DesktopCaptureManagerFrameSource.hpp"

#include <utility>

namespace xreal::capture
{

DesktopCaptureManagerFrameSource::DesktopCaptureManagerFrameSource(
    DesktopCaptureManager& manager,
    std::size_t sourceSlot,
    std::string stableIdentity)
    : manager_(&manager),
      sourceSlot_(sourceSlot),
      stableIdentity_(std::move(stableIdentity))
{
}

std::string_view DesktopCaptureManagerFrameSource::stableIdentity() const noexcept
{
    return stableIdentity_;
}

std::optional<DesktopCaptureFrame> DesktopCaptureManagerFrameSource::tryLatest()
{
    return manager_->tryLatest(sourceSlot_);
}

DesktopCaptureStatistics DesktopCaptureManagerFrameSource::statistics() const
{
    return manager_->statistics(sourceSlot_);
}

void DesktopCaptureManagerFrameSource::setMaximumFramesPerSecond(
    double framesPerSecond) noexcept
{
    manager_->setMaximumFramesPerSecond(sourceSlot_, framesPerSecond);
}

} // namespace xreal::capture
