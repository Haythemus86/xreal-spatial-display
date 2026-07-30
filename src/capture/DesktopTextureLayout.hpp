#pragma once

#include "capture/DesktopCaptureOptions.hpp"

#include <array>
#include <cstdint>

namespace xreal::capture
{

enum class DesktopRotation : std::uint16_t { identity = 0, rotate90 = 90, rotate180 = 180, rotate270 = 270 };

struct DesktopTextureLayout
{
    std::array<double, 2> contentMinimum{0.0, 0.0};
    std::array<double, 2> contentMaximum{1.0, 1.0};
    double cropLeft{};
    double cropTop{};
    double cropWidth{1.0};
    double cropHeight{1.0};
    DesktopRotation rotation{DesktopRotation::identity};
    bool flipY{};
};

[[nodiscard]] DesktopTextureLayout calculateDesktopTextureLayout(
    std::uint32_t sourceWidth,
    std::uint32_t sourceHeight,
    double panelAspectRatio,
    DesktopFit fit,
    DesktopRotation rotation,
    bool flipY) noexcept;
[[nodiscard]] std::array<double, 2> mapPanelToSource(
    const DesktopTextureLayout& layout,
    std::array<double, 2> panelCoordinates) noexcept;
[[nodiscard]] std::array<double, 2> mapSourceToPanel(
    const DesktopTextureLayout& layout,
    std::array<double, 2> sourceCoordinates) noexcept;

} // namespace xreal::capture
