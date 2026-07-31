#include "capture/DesktopRenderStages.hpp"

namespace xreal::capture
{

DesktopUploadAction decideDesktopUpload(
    const DesktopUploadState& state,
    std::uint64_t sequence,
    std::uint32_t width,
    std::uint32_t height,
    std::uint32_t format,
    bool frameValid) noexcept
{
    if (!frameValid || sequence == 0U || width == 0U || height == 0U || format == 0U)
    {
        return DesktopUploadAction::invalid;
    }
    if (state.sequence == sequence)
    {
        return DesktopUploadAction::skipSameSequence;
    }
    if (!state.textureValid)
    {
        return DesktopUploadAction::create;
    }
    if (state.width != width || state.height != height || state.format != format)
    {
        return DesktopUploadAction::recreate;
    }
    return DesktopUploadAction::update;
}

DesktopPanelEffectiveMode effectiveDesktopPanelMode(
    PanelContent requested,
    bool desktopSrvValid,
    bool /*stale*/) noexcept
{
    if (requested == PanelContent::synthetic)
    {
        return DesktopPanelEffectiveMode::synthetic;
    }
    // Staleness is diagnostic metadata. A WAIT_TIMEOUT or a static desktop must
    // never invalidate the last successfully uploaded texture.
    return desktopSrvValid
        ? DesktopPanelEffectiveMode::desktop : DesktopPanelEffectiveMode::unavailable;
}

std::string desktopPanelEffectiveModeText(DesktopPanelEffectiveMode mode)
{
    switch (mode)
    {
    case DesktopPanelEffectiveMode::synthetic: return "synthetic";
    case DesktopPanelEffectiveMode::unavailable: return "unavailable";
    case DesktopPanelEffectiveMode::desktop: return "desktop";
    }
    return "unknown";
}

} // namespace xreal::capture
