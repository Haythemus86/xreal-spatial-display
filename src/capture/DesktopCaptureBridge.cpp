#include "capture/DesktopCaptureBridge.hpp"

#include <utility>

namespace xreal::capture
{

std::uint64_t DesktopCaptureBridge::publish(DesktopCaptureFrame frame)
{
    if (!frame.valid || frame.sourceWidth == 0U || frame.sourceHeight == 0U)
    {
        return 0U;
    }
    std::scoped_lock lock(mutex_);
    if (current_.has_value() && current_->sequence != lastReadSequence_)
    {
        ++statistics_.droppedPublications;
    }
    frame.sequence = nextSequence_++;
    const std::uint64_t publishedSequence = frame.sequence;
    current_ = std::move(frame);
    ++statistics_.publications;
    return publishedSequence;
}

std::optional<DesktopCaptureFrame> DesktopCaptureBridge::latest()
{
    std::scoped_lock lock(mutex_);
    if (!current_.has_value())
    {
        return std::nullopt;
    }
    if (current_->sequence == lastReadSequence_)
    {
        ++statistics_.repeatedReads;
    }
    lastReadSequence_ = current_->sequence;
    return current_;
}

std::optional<DesktopCaptureFrame> DesktopCaptureBridge::tryLatest()
{
    std::unique_lock lock(mutex_, std::try_to_lock);
    if (!lock.owns_lock())
    {
        contendedReads_.fetch_add(1U, std::memory_order_relaxed);
        return std::nullopt;
    }
    if (!current_.has_value())
    {
        return std::nullopt;
    }
    if (current_->sequence == lastReadSequence_)
    {
        ++statistics_.repeatedReads;
    }
    lastReadSequence_ = current_->sequence;
    return current_;
}

DesktopCaptureBridgeStatistics DesktopCaptureBridge::statistics() const
{
    std::scoped_lock lock(mutex_);
    auto result = statistics_;
    result.contendedReads = contendedReads_.load(std::memory_order_relaxed);
    return result;
}

} // namespace xreal::capture
