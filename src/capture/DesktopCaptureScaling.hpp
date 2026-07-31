#pragma once

#include "capture/DesktopCaptureOptions.hpp"

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>

namespace xreal::capture
{

inline constexpr std::uint32_t desktopScaleMaximumDimension = 16384U;
inline constexpr std::uint32_t desktopPanelAwareMinimumWidth = 320U;
inline constexpr std::uint32_t desktopPanelAwareMinimumHeight = 180U;

enum class DesktopResolutionPolicy
{
    native,
    panelAware,
    fixed,
};

enum class DesktopCropMode
{
    full,
    center16x9,
    custom,
};

struct DesktopCaptureRegion
{
    std::uint32_t x{};
    std::uint32_t y{};
    std::uint32_t width{};
    std::uint32_t height{};

    [[nodiscard]] friend constexpr bool operator==(
        DesktopCaptureRegion,
        DesktopCaptureRegion) noexcept = default;
};

struct DesktopScaleRequest
{
    DesktopResolutionPolicy resolutionPolicy{DesktopResolutionPolicy::native};
    DesktopCropMode cropMode{DesktopCropMode::full};
    DesktopCaptureRegion customRegion;
    std::uint32_t targetWidth{};
    std::uint32_t targetHeight{};
    double scale{1.0};
    DesktopFit fit{DesktopFit::contain};
    DesktopFilter filter{DesktopFilter::linear};
    bool allowUpscale{};
    double panelAwareSafetyFactor{1.25};
    std::uint32_t projectedWidth{};
    std::uint32_t projectedHeight{};
};

struct DesktopScalePlan
{
    std::uint32_t sourceWidth{};
    std::uint32_t sourceHeight{};
    DesktopCaptureRegion crop;
    std::uint32_t targetWidth{};
    std::uint32_t targetHeight{};
    std::uint64_t sourceBytesPerFrame{};
    std::uint64_t cropBytesPerFrame{};
    std::uint64_t targetBytesPerFrame{};
    bool gpuScaleRequired{};

    [[nodiscard]] friend constexpr bool operator==(
        const DesktopScalePlan&,
        const DesktopScalePlan&) noexcept = default;
};

struct DesktopScalePlanResult
{
    std::optional<DesktopScalePlan> plan;
    std::string error;
};

[[nodiscard]] DesktopScalePlanResult resolveDesktopScalePlan(
    std::uint32_t sourceWidth,
    std::uint32_t sourceHeight,
    const DesktopScaleRequest& request) noexcept;
[[nodiscard]] bool shouldTransitionDesktopScalePlan(
    const DesktopScalePlan& current,
    const DesktopScalePlan& candidate,
    std::chrono::steady_clock::time_point now,
    std::chrono::steady_clock::time_point previousTransition,
    std::chrono::milliseconds minimumInterval = std::chrono::milliseconds(1000),
    double relativeHysteresis = 0.15) noexcept;
[[nodiscard]] std::string_view desktopResolutionPolicyText(
    DesktopResolutionPolicy value) noexcept;
[[nodiscard]] std::optional<DesktopResolutionPolicy> parseDesktopResolutionPolicy(
    std::string_view value) noexcept;
[[nodiscard]] std::string_view desktopCropModeText(DesktopCropMode value) noexcept;
[[nodiscard]] std::optional<DesktopCropMode> parseDesktopCropMode(
    std::string_view value) noexcept;

} // namespace xreal::capture
