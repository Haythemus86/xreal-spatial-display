#include "rendering/DesktopPanelDiagnostics.hpp"

#include <algorithm>
#include <cmath>

namespace xreal::rendering
{

std::string_view desktopShaderDebugModeText(DesktopShaderDebugMode mode) noexcept
{
    switch (mode)
    {
    case DesktopShaderDebugMode::normal: return "normal";
    case DesktopShaderDebugMode::solidRed: return "solid_red";
    case DesktopShaderDebugMode::ultraviolet: return "uv";
    case DesktopShaderDebugMode::sample: return "sample";
    case DesktopShaderDebugMode::sampleNoOverlay: return "sample_no_overlay";
    }
    return "unknown";
}

bool desktopShaderDebugModeUsesTexture(DesktopShaderDebugMode mode) noexcept
{
    return mode == DesktopShaderDebugMode::normal
        || mode == DesktopShaderDebugMode::sample
        || mode == DesktopShaderDebugMode::sampleNoOverlay;
}

bool desktopOverlayEnabled(DesktopShaderDebugMode mode) noexcept
{
    return mode != DesktopShaderDebugMode::sampleNoOverlay;
}

DesktopPanelBlendMode desktopBaseBlendMode(bool desktopBase, bool forceOpaque) noexcept
{
    return desktopBase && forceOpaque
        ? DesktopPanelBlendMode::opaque : DesktopPanelBlendMode::alpha;
}

DesktopPanelBlendMode desktopOverlayBlendMode() noexcept
{
    return DesktopPanelBlendMode::alpha;
}

bool isShaderReadableDesktopTexture(
    const DesktopTextureDescriptorContract& descriptor) noexcept
{
    return descriptor.width > 0U && descriptor.height > 0U
        && descriptor.mipLevels == 1U && descriptor.arraySize == 1U
        && (descriptor.format == bgra8UnormFormat
            || descriptor.format == bgra8UnormSrgbFormat)
        && descriptor.sampleCount == 1U && descriptor.sampleQuality == 0U
        && descriptor.usage == defaultTextureUsage
        && (descriptor.bindFlags & shaderResourceBindFlag) != 0U
        && descriptor.cpuAccessFlags == 0U && descriptor.miscFlags == 0U;
}

bool isCompatibleDesktopShaderResource(
    const DesktopTextureDescriptorContract& texture,
    const DesktopShaderResourceDescriptorContract& resource) noexcept
{
    return isShaderReadableDesktopTexture(texture)
        && resource.format == texture.format
        && resource.viewDimension == texture2dSrvDimension
        && resource.mostDetailedMip == 0U && resource.mipLevels == 1U;
}

bool panelUvConstantsFinite(const PanelShaderConstants& constants) noexcept
{
    return std::ranges::all_of(constants.desktopContentBounds,
               [](float value) { return std::isfinite(value); })
        && std::ranges::all_of(constants.desktopCrop,
               [](float value) { return std::isfinite(value); })
        && constants.desktopContentBounds[2] > constants.desktopContentBounds[0]
        && constants.desktopContentBounds[3] > constants.desktopContentBounds[1]
        && constants.desktopCrop[2] > 0.0F && constants.desktopCrop[3] > 0.0F;
}

bool validatePanelGeometry(
    std::span<const PanelVertexUv> vertices,
    std::span<const std::uint16_t> indices) noexcept
{
    if (vertices.size() != 4U || indices.size() != 6U)
    {
        return false;
    }
    for (const auto& vertex : vertices)
    {
        if (!std::isfinite(vertex.u) || !std::isfinite(vertex.v)
            || vertex.u < 0.0F || vertex.u > 1.0F
            || vertex.v < 0.0F || vertex.v > 1.0F)
        {
            return false;
        }
    }
    return std::ranges::all_of(indices,
        [vertices](std::uint16_t index) { return index < vertices.size(); });
}

bool validateDesktopDrawOrdering(
    std::span<const DesktopDrawEvent> events,
    bool overlayExpected) noexcept
{
    constexpr std::array required{
        DesktopDrawEvent::textureUpdated,
        DesktopDrawEvent::shaderResourceReady,
        DesktopDrawEvent::constantsUpdated,
        DesktopDrawEvent::geometryBound,
        DesktopDrawEvent::shadersBound,
        DesktopDrawEvent::shaderResourceBound,
        DesktopDrawEvent::samplerBound,
        DesktopDrawEvent::constantBuffersBound,
        DesktopDrawEvent::statesBound,
        DesktopDrawEvent::baseDrawIndexed,
    };
    if (events.size() < required.size() + 2U
        || !std::equal(required.begin(), required.end(), events.begin()))
    {
        return false;
    }
    std::size_t index = required.size();
    if (overlayExpected)
    {
        if (events[index++] != DesktopDrawEvent::overlayDraw)
        {
            return false;
        }
    }
    if (events[index++] != DesktopDrawEvent::shaderResourceUnbound
        || index >= events.size() || events[index++] != DesktopDrawEvent::presented)
    {
        return false;
    }
    return index == events.size();
}

std::string_view desktopDrawEventText(DesktopDrawEvent event) noexcept
{
    switch (event)
    {
    case DesktopDrawEvent::textureUpdated: return "texture_updated";
    case DesktopDrawEvent::shaderResourceReady: return "shader_resource_ready";
    case DesktopDrawEvent::constantsUpdated: return "constants_updated";
    case DesktopDrawEvent::geometryBound: return "geometry_bound";
    case DesktopDrawEvent::shadersBound: return "shaders_bound";
    case DesktopDrawEvent::shaderResourceBound: return "shader_resource_bound";
    case DesktopDrawEvent::samplerBound: return "sampler_bound";
    case DesktopDrawEvent::constantBuffersBound: return "constant_buffers_bound";
    case DesktopDrawEvent::statesBound: return "states_bound";
    case DesktopDrawEvent::baseDrawIndexed: return "base_draw_indexed";
    case DesktopDrawEvent::overlayDraw: return "overlay_draw";
    case DesktopDrawEvent::shaderResourceUnbound: return "shader_resource_unbound";
    case DesktopDrawEvent::presented: return "presented";
    }
    return "unknown";
}

} // namespace xreal::rendering
