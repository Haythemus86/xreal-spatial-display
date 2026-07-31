#pragma once

#include "capture/DesktopDuplicationCapture.hpp"
#include "capture/DesktopSyntheticCapture.hpp"

#include <array>
#include <cstddef>
#include <memory>
#include <optional>
#include <string>

namespace xreal::capture
{

inline constexpr std::size_t maximumDesktopCaptureSources = 3U;

struct DesktopCaptureSourceConfig
{
    std::size_t sourceSlot{};
    bool synthetic{};
    DesktopDuplicationConfig duplication;
    DesktopSyntheticCaptureConfig syntheticConfig;
};

class DesktopCaptureManager
{
public:
    DesktopCaptureManager();
    ~DesktopCaptureManager();
    DesktopCaptureManager(const DesktopCaptureManager&) = delete;
    DesktopCaptureManager& operator=(const DesktopCaptureManager&) = delete;

    [[nodiscard]] bool startSource(DesktopCaptureSourceConfig config);
    void setMaximumFramesPerSecond(std::size_t sourceSlot, double value) noexcept;
    [[nodiscard]] std::optional<DesktopCaptureFrame> tryLatest(std::size_t sourceSlot);
    [[nodiscard]] DesktopCaptureStatistics statistics(std::size_t sourceSlot) const;
    [[nodiscard]] DesktopCaptureStatistics detailedStatistics(
        std::size_t sourceSlot) const;
    [[nodiscard]] DesktopCaptureBridgeStatistics bridgeStatistics(
        std::size_t sourceSlot) const;
    [[nodiscard]] std::string error(std::size_t sourceSlot) const;
    [[nodiscard]] bool active(std::size_t sourceSlot) const noexcept;
    [[nodiscard]] std::size_t activeSourceCount() const noexcept;
    void stopSource(std::size_t sourceSlot);
    void stop();

private:
    struct Runtime;
    std::array<std::unique_ptr<Runtime>, maximumDesktopCaptureSources> sources_;
    std::array<std::string, maximumDesktopCaptureSources> lastErrors_;
};

} // namespace xreal::capture
