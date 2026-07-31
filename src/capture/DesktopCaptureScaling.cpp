#include "capture/DesktopCaptureScaling.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace xreal::capture
{
namespace
{

[[nodiscard]] std::optional<std::uint64_t> bgraBytes(
    std::uint32_t width,
    std::uint32_t height) noexcept
{
    if (width == 0U || height == 0U
        || static_cast<std::uint64_t>(width)
            > std::numeric_limits<std::uint64_t>::max() / 4U / height)
    {
        return std::nullopt;
    }
    return static_cast<std::uint64_t>(width) * height * 4U;
}

[[nodiscard]] std::uint32_t roundedDimension(double value) noexcept
{
    if (!std::isfinite(value) || value <= 0.0
        || value > static_cast<double>(desktopScaleMaximumDimension))
    {
        return 0U;
    }
    return std::max(1U, static_cast<std::uint32_t>(std::llround(value)));
}

[[nodiscard]] DesktopCaptureRegion centerAspectCrop(
    std::uint32_t sourceWidth,
    std::uint32_t sourceHeight,
    double targetAspect) noexcept
{
    const double sourceAspect = static_cast<double>(sourceWidth) / sourceHeight;
    if (sourceAspect > targetAspect)
    {
        const auto width = std::min(sourceWidth,
            roundedDimension(static_cast<double>(sourceHeight) * targetAspect));
        return {(sourceWidth - width) / 2U, 0U, width, sourceHeight};
    }
    const auto height = std::min(sourceHeight,
        roundedDimension(static_cast<double>(sourceWidth) / targetAspect));
    return {0U, (sourceHeight - height) / 2U, sourceWidth, height};
}

[[nodiscard]] bool regionInside(
    DesktopCaptureRegion region,
    std::uint32_t sourceWidth,
    std::uint32_t sourceHeight) noexcept
{
    return region.width > 0U && region.height > 0U
        && region.x <= sourceWidth && region.y <= sourceHeight
        && region.width <= sourceWidth - region.x
        && region.height <= sourceHeight - region.y;
}

void fitInside(
    std::uint32_t sourceWidth,
    std::uint32_t sourceHeight,
    std::uint32_t boundingWidth,
    std::uint32_t boundingHeight,
    std::uint32_t& resultWidth,
    std::uint32_t& resultHeight) noexcept
{
    const double scale = std::min(
        static_cast<double>(boundingWidth) / sourceWidth,
        static_cast<double>(boundingHeight) / sourceHeight);
    resultWidth = roundedDimension(sourceWidth * scale);
    resultHeight = roundedDimension(sourceHeight * scale);
}

} // namespace

DesktopScalePlanResult resolveDesktopScalePlan(
    std::uint32_t sourceWidth,
    std::uint32_t sourceHeight,
    const DesktopScaleRequest& request) noexcept
{
    if (sourceWidth == 0U || sourceHeight == 0U
        || sourceWidth > desktopScaleMaximumDimension
        || sourceHeight > desktopScaleMaximumDimension)
    {
        return {std::nullopt, "Desktop source dimensions are outside D3D11 limits."};
    }
    if (!std::isfinite(request.scale) || request.scale <= 0.0
        || !std::isfinite(request.panelAwareSafetyFactor)
        || request.panelAwareSafetyFactor <= 0.0)
    {
        return {std::nullopt, "Desktop scale and safety factor must be finite and positive."};
    }

    DesktopCaptureRegion crop{0U, 0U, sourceWidth, sourceHeight};
    if (request.cropMode == DesktopCropMode::center16x9)
    {
        crop = centerAspectCrop(sourceWidth, sourceHeight, 16.0 / 9.0);
    }
    else if (request.cropMode == DesktopCropMode::custom)
    {
        crop = request.customRegion;
    }
    if (!regionInside(crop, sourceWidth, sourceHeight))
    {
        return {std::nullopt, "Desktop crop rectangle is empty or outside the source."};
    }

    const bool explicitDimensions = request.targetWidth != 0U || request.targetHeight != 0U;
    const bool explicitScale = std::abs(request.scale - 1.0) > 1.0e-9;
    const DesktopResolutionPolicy effectivePolicy = explicitDimensions || explicitScale
        ? DesktopResolutionPolicy::fixed : request.resolutionPolicy;
    std::uint32_t targetWidth = crop.width;
    std::uint32_t targetHeight = crop.height;

    if (effectivePolicy == DesktopResolutionPolicy::panelAware)
    {
        if (request.projectedWidth == 0U || request.projectedHeight == 0U)
        {
            return {std::nullopt,
                "Panel-aware resolution requires non-zero projected dimensions."};
        }
        const std::uint32_t boundingWidth = std::max(desktopPanelAwareMinimumWidth,
            roundedDimension(request.projectedWidth * request.panelAwareSafetyFactor));
        const std::uint32_t boundingHeight = std::max(desktopPanelAwareMinimumHeight,
            roundedDimension(request.projectedHeight * request.panelAwareSafetyFactor));
        fitInside(crop.width, crop.height, boundingWidth, boundingHeight,
            targetWidth, targetHeight);
    }
    else if (effectivePolicy == DesktopResolutionPolicy::fixed)
    {
        if (explicitDimensions)
        {
            if (request.targetWidth != 0U && request.targetHeight != 0U)
            {
                targetWidth = request.targetWidth;
                targetHeight = request.targetHeight;
                if (request.fit == DesktopFit::contain)
                {
                    fitInside(crop.width, crop.height, request.targetWidth,
                        request.targetHeight, targetWidth, targetHeight);
                }
                else if (request.fit == DesktopFit::cover)
                {
                    auto coverCrop = centerAspectCrop(crop.width, crop.height,
                        static_cast<double>(request.targetWidth) / request.targetHeight);
                    coverCrop.x += crop.x;
                    coverCrop.y += crop.y;
                    crop = coverCrop;
                }
            }
            else if (request.targetWidth != 0U)
            {
                targetWidth = request.targetWidth;
                targetHeight = roundedDimension(static_cast<double>(crop.height)
                    * request.targetWidth / crop.width);
            }
            else
            {
                targetHeight = request.targetHeight;
                targetWidth = roundedDimension(static_cast<double>(crop.width)
                    * request.targetHeight / crop.height);
            }
        }
        else
        {
            targetWidth = roundedDimension(crop.width * request.scale);
            targetHeight = roundedDimension(crop.height * request.scale);
        }
    }

    if (targetWidth == 0U || targetHeight == 0U
        || targetWidth > desktopScaleMaximumDimension
        || targetHeight > desktopScaleMaximumDimension)
    {
        return {std::nullopt, "Resolved desktop target dimensions are invalid."};
    }
    if (!request.allowUpscale
        && (targetWidth > crop.width || targetHeight > crop.height))
    {
        return {std::nullopt,
            "Desktop target would upscale the crop; enable upscale explicitly to allow it."};
    }
    const auto sourceBytes = bgraBytes(sourceWidth, sourceHeight);
    const auto cropBytes = bgraBytes(crop.width, crop.height);
    const auto targetBytes = bgraBytes(targetWidth, targetHeight);
    if (!sourceBytes.has_value() || !cropBytes.has_value() || !targetBytes.has_value())
    {
        return {std::nullopt, "Desktop BGRA byte count overflowed."};
    }
    DesktopScalePlan plan;
    plan.sourceWidth = sourceWidth;
    plan.sourceHeight = sourceHeight;
    plan.crop = crop;
    plan.targetWidth = targetWidth;
    plan.targetHeight = targetHeight;
    plan.sourceBytesPerFrame = *sourceBytes;
    plan.cropBytesPerFrame = *cropBytes;
    plan.targetBytesPerFrame = *targetBytes;
    plan.gpuScaleRequired = crop != DesktopCaptureRegion{0U, 0U, sourceWidth, sourceHeight}
        || targetWidth != sourceWidth || targetHeight != sourceHeight;
    return {plan, {}};
}

bool shouldTransitionDesktopScalePlan(
    const DesktopScalePlan& current,
    const DesktopScalePlan& candidate,
    std::chrono::steady_clock::time_point now,
    std::chrono::steady_clock::time_point previousTransition,
    std::chrono::milliseconds minimumInterval,
    double relativeHysteresis) noexcept
{
    if (candidate == current)
    {
        return false;
    }
    if (!std::isfinite(relativeHysteresis) || relativeHysteresis < 0.0
        || now - previousTransition < minimumInterval)
    {
        return false;
    }
    if (candidate.crop != current.crop)
    {
        return true;
    }
    const double widthChange = std::abs(static_cast<double>(candidate.targetWidth)
        - current.targetWidth) / std::max(1U, current.targetWidth);
    const double heightChange = std::abs(static_cast<double>(candidate.targetHeight)
        - current.targetHeight) / std::max(1U, current.targetHeight);
    return widthChange >= relativeHysteresis || heightChange >= relativeHysteresis;
}

std::string_view desktopResolutionPolicyText(DesktopResolutionPolicy value) noexcept
{
    switch (value)
    {
    case DesktopResolutionPolicy::native: return "native";
    case DesktopResolutionPolicy::panelAware: return "panel-aware";
    case DesktopResolutionPolicy::fixed: return "fixed";
    }
    return "unknown";
}

std::optional<DesktopResolutionPolicy> parseDesktopResolutionPolicy(
    std::string_view value) noexcept
{
    if (value == "native") { return DesktopResolutionPolicy::native; }
    if (value == "panel-aware") { return DesktopResolutionPolicy::panelAware; }
    if (value == "fixed") { return DesktopResolutionPolicy::fixed; }
    return std::nullopt;
}

std::string_view desktopCropModeText(DesktopCropMode value) noexcept
{
    switch (value)
    {
    case DesktopCropMode::full: return "full";
    case DesktopCropMode::center16x9: return "center-16x9";
    case DesktopCropMode::custom: return "custom";
    }
    return "unknown";
}

std::optional<DesktopCropMode> parseDesktopCropMode(std::string_view value) noexcept
{
    if (value == "full") { return DesktopCropMode::full; }
    if (value == "center-16x9") { return DesktopCropMode::center16x9; }
    if (value == "custom") { return DesktopCropMode::custom; }
    return std::nullopt;
}

} // namespace xreal::capture
