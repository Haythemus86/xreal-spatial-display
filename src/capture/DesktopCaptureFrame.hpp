#pragma once

#include "capture/DesktopTextureLayout.hpp"
#include "platform/windows/DisplayTopology.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace xreal::capture
{

enum class DesktopCaptureStatus
{
    disabled,
    initializing,
    active,
    waitTimeout,
    accessLost,
    recreating,
    outputMissing,
    unsupported,
    fatalError,
    shuttingDown,
};

class DesktopCaptureSurface
{
public:
    ~DesktopCaptureSurface();
    DesktopCaptureSurface(const DesktopCaptureSurface&) = delete;
    DesktopCaptureSurface& operator=(const DesktopCaptureSurface&) = delete;

    [[nodiscard]] void* sharedHandle() const noexcept;

private:
    friend class DesktopDuplicationCapture;
    DesktopCaptureSurface(void* nativeTexture, void* sharedHandle);
    [[nodiscard]] void* nativeTexture() const noexcept;
    class Implementation;
    std::unique_ptr<Implementation> implementation_;
};

struct DesktopCaptureFrame
{
    std::uint64_t sequence{};
    std::chrono::steady_clock::time_point captureHostTimestamp{};
    std::int64_t lastPresentTimestamp{};
    std::uint32_t accumulatedFrames{};
    std::uint32_t sourceWidth{};
    std::uint32_t sourceHeight{};
    std::uint32_t originalSourceWidth{};
    std::uint32_t originalSourceHeight{};
    std::uint32_t cropX{};
    std::uint32_t cropY{};
    std::uint32_t cropWidth{};
    std::uint32_t cropHeight{};
    std::uint32_t sourceFormat{};
    DesktopRotation rotation{DesktopRotation::identity};
    std::uint32_t dirtyRectCount{};
    std::uint32_t moveRectCount{};
    std::uint32_t metadataBytes{};
    bool pointerVisible{};
    int pointerX{};
    int pointerY{};
    std::string sourceMonitorDeviceName;
    platform::windows::DxgiAdapterLuid sourceAdapterLuid;
    std::shared_ptr<DesktopCaptureSurface> surface;
    std::shared_ptr<const std::vector<std::byte>> cpuPixels;
    std::uint32_t cpuRowPitch{};
    bool valid{};
    DesktopCaptureStatus status{DesktopCaptureStatus::disabled};
    std::uint64_t recoveryGeneration{};
};

[[nodiscard]] std::string desktopCaptureStatusText(DesktopCaptureStatus status);
[[nodiscard]] DesktopCaptureStatus desktopCaptureLoopExitStatus(
    DesktopCaptureStatus current,
    bool stopRequested) noexcept;

} // namespace xreal::capture
