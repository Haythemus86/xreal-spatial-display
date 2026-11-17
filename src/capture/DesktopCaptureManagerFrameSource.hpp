#pragma once

#include "capture/DesktopCaptureManager.hpp"
#include "capture/IPanelFrameSource.hpp"

#include <cstddef>
#include <string>

namespace xreal::capture
{

class DesktopCaptureManagerFrameSource final : public IPanelFrameSource
{
public:
    DesktopCaptureManagerFrameSource(
        DesktopCaptureManager& manager,
        std::size_t sourceSlot,
        std::string stableIdentity);

    [[nodiscard]] std::string_view stableIdentity() const noexcept override;
    [[nodiscard]] std::optional<DesktopCaptureFrame> tryLatest() override;
    [[nodiscard]] DesktopCaptureStatistics statistics() const override;
    void setMaximumFramesPerSecond(double framesPerSecond) noexcept override;

private:
    DesktopCaptureManager* manager_{};
    std::size_t sourceSlot_{};
    std::string stableIdentity_;
};

} // namespace xreal::capture
