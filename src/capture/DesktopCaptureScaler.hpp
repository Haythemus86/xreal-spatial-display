#pragma once

#include "capture/DesktopCaptureScaling.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace xreal::capture
{

inline constexpr std::size_t desktopStagingRingCapacity = 3U;

struct DesktopScaleReadback
{
    bool success{};
    bool frameReady{};
    std::shared_ptr<const std::vector<std::byte>> pixels;
    std::uint32_t width{};
    std::uint32_t height{};
    std::uint32_t rowPitch{};
    std::uint32_t format{};
    std::size_t stagingSlot{};
    std::uint64_t mappedBytes{};
    double gpuSubmissionMilliseconds{};
    double mapWaitMilliseconds{};
    double cpuRepackMilliseconds{};
    std::string error;
};

struct DesktopCaptureScalerStatistics
{
    std::uint64_t submissions{};
    std::uint64_t gpuScaleDraws{};
    std::uint64_t stagingCopies{};
    std::uint64_t stagingMaps{};
    std::uint64_t resourceRecreations{};
    std::uint64_t ringContentions{};
    std::uint64_t shaderCompilations{};
};

class DesktopCaptureScaler
{
public:
    DesktopCaptureScaler();
    ~DesktopCaptureScaler();
    DesktopCaptureScaler(const DesktopCaptureScaler&) = delete;
    DesktopCaptureScaler& operator=(const DesktopCaptureScaler&) = delete;
    DesktopCaptureScaler(DesktopCaptureScaler&&) noexcept;
    DesktopCaptureScaler& operator=(DesktopCaptureScaler&&) noexcept;

    [[nodiscard]] bool initialize(void* d3dDevice, void* immediateContext);
    [[nodiscard]] DesktopScaleReadback process(
        void* sourceTexture,
        const DesktopScalePlan& plan,
        DesktopFilter filter);
    [[nodiscard]] DesktopCaptureScalerStatistics statistics() const noexcept;
    [[nodiscard]] const std::string& error() const noexcept;
    void reset();

private:
    class Implementation;
    std::unique_ptr<Implementation> implementation_;
};

} // namespace xreal::capture
