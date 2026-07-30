#include "capture/DesktopTextureLayout.hpp"

#include <algorithm>

namespace xreal::capture
{
namespace
{

[[nodiscard]] std::array<double, 2> rotateToSource(
    std::array<double, 2> coordinates,
    DesktopRotation rotation) noexcept
{
    switch (rotation)
    {
    case DesktopRotation::identity: return coordinates;
    case DesktopRotation::rotate90: return {coordinates[1], 1.0 - coordinates[0]};
    case DesktopRotation::rotate180: return {1.0 - coordinates[0], 1.0 - coordinates[1]};
    case DesktopRotation::rotate270: return {1.0 - coordinates[1], coordinates[0]};
    }
    return coordinates;
}

[[nodiscard]] std::array<double, 2> rotateFromSource(
    std::array<double, 2> coordinates,
    DesktopRotation rotation) noexcept
{
    switch (rotation)
    {
    case DesktopRotation::identity: return coordinates;
    case DesktopRotation::rotate90: return {1.0 - coordinates[1], coordinates[0]};
    case DesktopRotation::rotate180: return {1.0 - coordinates[0], 1.0 - coordinates[1]};
    case DesktopRotation::rotate270: return {coordinates[1], 1.0 - coordinates[0]};
    }
    return coordinates;
}

} // namespace

DesktopTextureLayout calculateDesktopTextureLayout(
    std::uint32_t sourceWidth,
    std::uint32_t sourceHeight,
    double panelAspectRatio,
    DesktopFit fit,
    DesktopRotation rotation,
    bool flipY) noexcept
{
    DesktopTextureLayout result;
    result.rotation = rotation;
    result.flipY = flipY;
    if (sourceWidth == 0U || sourceHeight == 0U || panelAspectRatio <= 0.0)
    {
        result.contentMaximum = {0.0, 0.0};
        return result;
    }
    double sourceAspect = static_cast<double>(sourceWidth) / sourceHeight;
    if (rotation == DesktopRotation::rotate90 || rotation == DesktopRotation::rotate270)
    {
        sourceAspect = 1.0 / sourceAspect;
    }
    if (fit == DesktopFit::contain)
    {
        if (sourceAspect > panelAspectRatio)
        {
            const double height = panelAspectRatio / sourceAspect;
            result.contentMinimum[1] = (1.0 - height) * 0.5;
            result.contentMaximum[1] = result.contentMinimum[1] + height;
        }
        else
        {
            const double width = sourceAspect / panelAspectRatio;
            result.contentMinimum[0] = (1.0 - width) * 0.5;
            result.contentMaximum[0] = result.contentMinimum[0] + width;
        }
    }
    else if (fit == DesktopFit::cover)
    {
        if (sourceAspect > panelAspectRatio)
        {
            result.cropWidth = panelAspectRatio / sourceAspect;
            result.cropLeft = (1.0 - result.cropWidth) * 0.5;
        }
        else
        {
            result.cropHeight = sourceAspect / panelAspectRatio;
            result.cropTop = (1.0 - result.cropHeight) * 0.5;
        }
    }
    return result;
}

std::array<double, 2> mapPanelToSource(
    const DesktopTextureLayout& layout,
    std::array<double, 2> panelCoordinates) noexcept
{
    const double width = layout.contentMaximum[0] - layout.contentMinimum[0];
    const double height = layout.contentMaximum[1] - layout.contentMinimum[1];
    if (width <= 0.0 || height <= 0.0)
    {
        return {-1.0, -1.0};
    }
    std::array<double, 2> oriented{
        (panelCoordinates[0] - layout.contentMinimum[0]) / width,
        (panelCoordinates[1] - layout.contentMinimum[1]) / height,
    };
    oriented[0] = layout.cropLeft + oriented[0] * layout.cropWidth;
    oriented[1] = layout.cropTop + oriented[1] * layout.cropHeight;
    if (layout.flipY)
    {
        oriented[1] = 1.0 - oriented[1];
    }
    return rotateToSource(oriented, layout.rotation);
}

std::array<double, 2> mapSourceToPanel(
    const DesktopTextureLayout& layout,
    std::array<double, 2> sourceCoordinates) noexcept
{
    auto oriented = rotateFromSource(sourceCoordinates, layout.rotation);
    if (layout.flipY)
    {
        oriented[1] = 1.0 - oriented[1];
    }
    oriented[0] = (oriented[0] - layout.cropLeft) / layout.cropWidth;
    oriented[1] = (oriented[1] - layout.cropTop) / layout.cropHeight;
    return {
        layout.contentMinimum[0] + oriented[0]
            * (layout.contentMaximum[0] - layout.contentMinimum[0]),
        layout.contentMinimum[1] + oriented[1]
            * (layout.contentMaximum[1] - layout.contentMinimum[1]),
    };
}

} // namespace xreal::capture
