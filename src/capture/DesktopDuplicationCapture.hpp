#pragma once

#include "capture/DesktopCaptureBridge.hpp"
#include "capture/DesktopCaptureOptions.hpp"
#include "capture/DesktopCaptureStatistics.hpp"
#include "capture/DesktopCaptureScaling.hpp"

#include <memory>
#include <mutex>
#include <string>
#include <thread>

namespace xreal::capture
{

struct DesktopDuplicationConfig
{
    platform::windows::MonitorInformation captureMonitor;
    platform::windows::DxgiAdapterLuid renderAdapterLuid;
    DesktopCaptureOptions options;
    DesktopScaleRequest scaling;
    double maximumFramesPerSecond{30.0};
};

class DesktopDuplicationCapture
{
public:
    explicit DesktopDuplicationCapture(DesktopCaptureBridge& bridge);
    ~DesktopDuplicationCapture();
    DesktopDuplicationCapture(const DesktopDuplicationCapture&) = delete;
    DesktopDuplicationCapture& operator=(const DesktopDuplicationCapture&) = delete;

    [[nodiscard]] bool start(DesktopDuplicationConfig config);
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
