#pragma once

#include "platform/windows/DisplayTopology.hpp"

#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <string_view>

namespace xreal::capture
{

enum class PanelContent { synthetic, desktop };
enum class DesktopFit { contain, cover, stretch };
enum class DesktopFilter { point, linear };
enum class DesktopBackground { black, grid };
enum class CrossAdapterPolicy { automatic, sharedHandle, cpuFallback, reject };

struct MonitorSelector
{
    std::optional<unsigned int> index;
    std::optional<std::string> deviceName;
    std::optional<std::string> stableIdentity;
};

struct DesktopCaptureOptions
{
    PanelContent panelContent{PanelContent::synthetic};
    MonitorSelector captureMonitor;
    DesktopFit fit{DesktopFit::contain};
    DesktopFilter filter{DesktopFilter::linear};
    DesktopBackground background{DesktopBackground::black};
    bool flipY{};
    bool showCursor{true};
    std::uint32_t staleThresholdMilliseconds{250};
    std::uint32_t timeoutMilliseconds{100};
    std::uint32_t retryMilliseconds{500};
    CrossAdapterPolicy crossAdapterPolicy{CrossAdapterPolicy::automatic};
    bool allowCpuFallback{};
    bool diagnostics{};
    std::optional<std::string> jsonOutputPath;
    std::optional<std::string> firstFrameBmpPath;
};

struct MonitorResolution
{
    const platform::windows::MonitorInformation* monitor{};
    std::string error;
};

[[nodiscard]] MonitorResolution resolveMonitor(
    std::span<const platform::windows::MonitorInformation> monitors,
    const MonitorSelector& selector,
    std::string_view role);
[[nodiscard]] bool sameAdapter(
    const platform::windows::MonitorInformation& first,
    const platform::windows::MonitorInformation& second) noexcept;
[[nodiscard]] std::string panelContentText(PanelContent value);
[[nodiscard]] std::string desktopFitText(DesktopFit value);
[[nodiscard]] std::string desktopFilterText(DesktopFilter value);
[[nodiscard]] std::string crossAdapterPolicyText(CrossAdapterPolicy value);

} // namespace xreal::capture
