#pragma once

#include "capture/DesktopCaptureFrame.hpp"
#include "capture/DesktopCaptureStatistics.hpp"

#include <optional>
#include <string_view>

namespace xreal::capture
{

// Generic boundary consumed by panel-content routing. A future direct IddCx
// frame source can implement this interface without changing the D3D11 panel
// renderer or its upload registry.
class IPanelFrameSource
{
public:
    virtual ~IPanelFrameSource() = default;

    [[nodiscard]] virtual std::string_view stableIdentity() const noexcept = 0;
    [[nodiscard]] virtual std::optional<DesktopCaptureFrame> tryLatest() = 0;
    [[nodiscard]] virtual DesktopCaptureStatistics statistics() const = 0;
    virtual void setMaximumFramesPerSecond(double framesPerSecond) noexcept = 0;
};

} // namespace xreal::capture
