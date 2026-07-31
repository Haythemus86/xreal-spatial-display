#pragma once

#include <chrono>
#include <string_view>

namespace xreal::capture
{

enum class DesktopFrameAvailability
{
    noFrameEver,
    activeUnchanged,
    staleButValid,
    unavailable,
    accessLost,
};

class DesktopFrameRetention
{
public:
    void recordCaptureAttempt(std::chrono::steady_clock::time_point now) noexcept;
    void recordValidFrame(std::chrono::steady_clock::time_point now) noexcept;
    void recordWaitTimeout(
        std::chrono::steady_clock::time_point now,
        std::chrono::milliseconds staleThreshold) noexcept;
    void recordUnavailable() noexcept;
    void recordAccessLost() noexcept;

    [[nodiscard]] DesktopFrameAvailability availability() const noexcept;
    [[nodiscard]] bool hasValidFrame() const noexcept;
    [[nodiscard]] std::chrono::steady_clock::time_point lastValidFrameTimestamp() const noexcept;
    [[nodiscard]] std::chrono::steady_clock::time_point lastChangeTimestamp() const noexcept;
    [[nodiscard]] std::chrono::steady_clock::time_point lastCaptureAttemptTimestamp() const noexcept;

private:
    DesktopFrameAvailability availability_{DesktopFrameAvailability::noFrameEver};
    bool hasValidFrame_{};
    std::chrono::steady_clock::time_point lastValidFrameTimestamp_{};
    std::chrono::steady_clock::time_point lastChangeTimestamp_{};
    std::chrono::steady_clock::time_point lastCaptureAttemptTimestamp_{};
};

[[nodiscard]] std::string_view desktopFrameAvailabilityText(
    DesktopFrameAvailability value) noexcept;

} // namespace xreal::capture
