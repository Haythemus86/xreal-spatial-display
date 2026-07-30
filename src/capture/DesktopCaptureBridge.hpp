#pragma once

#include "capture/DesktopCaptureFrame.hpp"

#include <cstdint>
#include <mutex>
#include <optional>

namespace xreal::capture
{

struct DesktopCaptureBridgeStatistics
{
    std::uint64_t publications{};
    std::uint64_t droppedPublications{};
    std::uint64_t repeatedReads{};
};

class DesktopCaptureBridge
{
public:
    [[nodiscard]] std::uint64_t publish(DesktopCaptureFrame frame);
    [[nodiscard]] std::optional<DesktopCaptureFrame> latest();
    [[nodiscard]] DesktopCaptureBridgeStatistics statistics() const;

private:
    mutable std::mutex mutex_;
    std::optional<DesktopCaptureFrame> current_;
    std::uint64_t nextSequence_{1};
    std::uint64_t lastReadSequence_{};
    DesktopCaptureBridgeStatistics statistics_;
};

} // namespace xreal::capture
