#include "capture/DesktopFrameRetention.hpp"

namespace xreal::capture
{

void DesktopFrameRetention::recordCaptureAttempt(
    std::chrono::steady_clock::time_point now) noexcept
{
    lastCaptureAttemptTimestamp_ = now;
}

void DesktopFrameRetention::recordValidFrame(
    std::chrono::steady_clock::time_point now) noexcept
{
    hasValidFrame_ = true;
    lastValidFrameTimestamp_ = now;
    lastChangeTimestamp_ = now;
    availability_ = DesktopFrameAvailability::activeUnchanged;
}

void DesktopFrameRetention::recordWaitTimeout(
    std::chrono::steady_clock::time_point now,
    std::chrono::milliseconds staleThreshold) noexcept
{
    lastCaptureAttemptTimestamp_ = now;
    if (!hasValidFrame_)
    {
        availability_ = DesktopFrameAvailability::noFrameEver;
        return;
    }
    availability_ = now - lastValidFrameTimestamp_ > staleThreshold
        ? DesktopFrameAvailability::staleButValid
        : DesktopFrameAvailability::activeUnchanged;
}

void DesktopFrameRetention::recordUnavailable() noexcept
{
    if (!hasValidFrame_)
    {
        availability_ = DesktopFrameAvailability::unavailable;
    }
}

void DesktopFrameRetention::recordAccessLost() noexcept
{
    availability_ = DesktopFrameAvailability::accessLost;
}

DesktopFrameAvailability DesktopFrameRetention::availability() const noexcept
{
    return availability_;
}

bool DesktopFrameRetention::hasValidFrame() const noexcept { return hasValidFrame_; }

std::chrono::steady_clock::time_point
DesktopFrameRetention::lastValidFrameTimestamp() const noexcept
{
    return lastValidFrameTimestamp_;
}

std::chrono::steady_clock::time_point
DesktopFrameRetention::lastChangeTimestamp() const noexcept
{
    return lastChangeTimestamp_;
}

std::chrono::steady_clock::time_point
DesktopFrameRetention::lastCaptureAttemptTimestamp() const noexcept
{
    return lastCaptureAttemptTimestamp_;
}

std::string_view desktopFrameAvailabilityText(DesktopFrameAvailability value) noexcept
{
    switch (value)
    {
    case DesktopFrameAvailability::noFrameEver: return "no_frame_ever";
    case DesktopFrameAvailability::activeUnchanged: return "active_unchanged";
    case DesktopFrameAvailability::staleButValid: return "stale_but_valid";
    case DesktopFrameAvailability::unavailable: return "unavailable";
    case DesktopFrameAvailability::accessLost: return "access_lost";
    }
    return "unknown";
}

} // namespace xreal::capture
