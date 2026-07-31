#pragma once

#include "capture/DesktopCaptureFrame.hpp"

#include <cstdint>
#include <atomic>
#include <mutex>
#include <optional>

namespace xreal::capture
{

struct DesktopCaptureBridgeStatistics
{
    std::uint64_t publications{};
    std::uint64_t droppedPublications{};
    std::uint64_t repeatedReads{};
    std::uint64_t contendedReads{};
};

class DesktopCaptureBridge
{
public:
    [[nodiscard]] std::uint64_t publish(DesktopCaptureFrame frame);
    [[nodiscard]] std::optional<DesktopCaptureFrame> latest();
    [[nodiscard]] std::optional<DesktopCaptureFrame> tryLatest();
    [[nodiscard]] DesktopCaptureBridgeStatistics statistics() const;

private:
    mutable std::mutex mutex_;
    std::optional<DesktopCaptureFrame> current_;
    std::uint64_t nextSequence_{1};
    std::uint64_t lastReadSequence_{};
    DesktopCaptureBridgeStatistics statistics_;
    std::atomic<std::uint64_t> contendedReads_{};
};

} // namespace xreal::capture
