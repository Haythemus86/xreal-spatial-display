#pragma once

#include "capture/DesktopCaptureBridge.hpp"
#include "capture/DesktopCaptureScaling.hpp"
#include "capture/DesktopCaptureStatistics.hpp"

#include <memory>
#include <string>

namespace xreal::capture
{

struct DesktopSyntheticCaptureConfig
{
    std::uint32_t sourceWidth{3840U};
    std::uint32_t sourceHeight{2160U};
    std::size_t sourceOrdinal{};
    double maximumFramesPerSecond{30.0};
    DesktopScaleRequest scaling;
};

class DesktopSyntheticCapture
{
public:
    explicit DesktopSyntheticCapture(DesktopCaptureBridge& bridge);
    ~DesktopSyntheticCapture();
    DesktopSyntheticCapture(const DesktopSyntheticCapture&) = delete;
    DesktopSyntheticCapture& operator=(const DesktopSyntheticCapture&) = delete;

    [[nodiscard]] bool start(DesktopSyntheticCaptureConfig config);
    void setMaximumFramesPerSecond(double value) noexcept;
    void stop();
    [[nodiscard]] DesktopCaptureStatistics statistics() const;
    [[nodiscard]] DesktopCaptureStatistics detailedStatistics() const;
    [[nodiscard]] std::string error() const;

private:
    class Implementation;
    std::unique_ptr<Implementation> implementation_;
};

} // namespace xreal::capture
