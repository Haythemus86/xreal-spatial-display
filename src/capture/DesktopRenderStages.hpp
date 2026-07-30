#pragma once

#include "capture/DesktopCaptureOptions.hpp"

#include <cstdint>
#include <string>

namespace xreal::capture
{

enum class DesktopPanelEffectiveMode { synthetic, unavailable, desktop };
enum class DesktopUploadAction { invalid, skipSameSequence, create, recreate, update };

struct DesktopUploadState
{
    std::uint64_t sequence{};
    std::uint32_t width{};
    std::uint32_t height{};
    std::uint32_t format{};
    bool textureValid{};
};

struct DesktopRenderStageStatistics
{
    std::uint64_t cpuFramesSeen{};
    std::uint64_t cpuFramesConsumed{};
    std::uint64_t cpuFramesSkippedSameSequence{};
    std::uint64_t latestConsumedSequence{};
    std::uint64_t uploadTextureCreations{};
    std::uint64_t uploadTextureRecreations{};
    std::uint64_t updateSubresourceCalls{};
    std::uint64_t updateSubresourceFailures{};
    std::uint64_t latestUploadedSequence{};
    std::uint32_t latestUploadWidth{};
    std::uint32_t latestUploadHeight{};
    std::uint32_t latestUploadFormat{};
    bool uploadTextureValid{};
    std::uint64_t desktopSrvCreations{};
    std::uint64_t desktopSrvFailures{};
    std::uint64_t desktopSrvBindCount{};
    std::uint64_t latestBoundSequence{};
    bool desktopSrvValid{};
    PanelContent panelContentRequested{PanelContent::synthetic};
    DesktopPanelEffectiveMode panelContentEffective{DesktopPanelEffectiveMode::synthetic};
    bool desktopTextureAvailable{};
    std::uint64_t renderedDesktopFrames{};
    std::uint64_t renderedUnavailableFrames{};
    std::uint64_t renderedSyntheticFrames{};
};

[[nodiscard]] DesktopUploadAction decideDesktopUpload(
    const DesktopUploadState& state,
    std::uint64_t sequence,
    std::uint32_t width,
    std::uint32_t height,
    std::uint32_t format,
    bool frameValid) noexcept;
[[nodiscard]] DesktopPanelEffectiveMode effectiveDesktopPanelMode(
    PanelContent requested,
    bool desktopSrvValid,
    bool stale) noexcept;
[[nodiscard]] std::string desktopPanelEffectiveModeText(DesktopPanelEffectiveMode mode);

} // namespace xreal::capture
