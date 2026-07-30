#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

namespace xreal::rendering
{

inline constexpr std::uint32_t desktopTextureShaderRegister = 0U;
inline constexpr std::uint32_t desktopSamplerShaderRegister = 0U;
inline constexpr std::uint32_t panelConstantBufferShaderRegister = 0U;
inline constexpr std::uint32_t bgra8UnormFormat = 87U;
inline constexpr std::uint32_t bgra8UnormSrgbFormat = 91U;
inline constexpr std::uint32_t defaultTextureUsage = 0U;
inline constexpr std::uint32_t shaderResourceBindFlag = 0x8U;
inline constexpr std::uint32_t texture2dSrvDimension = 4U;

enum class DesktopShaderDebugMode : std::uint32_t
{
    normal = 0U,
    solidRed = 1U,
    ultraviolet = 2U,
    sample = 3U,
    sampleNoOverlay = 4U,
};

enum class DesktopPanelBlendMode
{
    alpha,
    opaque,
};

enum class DesktopDrawEvent
{
    textureUpdated,
    shaderResourceReady,
    constantsUpdated,
    geometryBound,
    shadersBound,
    shaderResourceBound,
    samplerBound,
    constantBuffersBound,
    statesBound,
    baseDrawIndexed,
    overlayDraw,
    shaderResourceUnbound,
    presented,
};

struct alignas(16) PanelShaderConstants
{
    std::array<float, 16> viewProjection{};
    std::array<float, 4> desktopContentBounds{};
    std::array<float, 4> desktopCrop{};
    std::uint32_t desktopRotation{};
    std::uint32_t desktopFlipY{};
    std::uint32_t desktopEnabled{};
    std::uint32_t desktopUnavailable{};
    std::uint32_t desktopBackgroundGrid{};
    std::array<std::uint32_t, 3> desktopPadding{};
    std::uint32_t desktopDebugMode{};
    std::array<std::uint32_t, 3> desktopDebugPadding{};
};

static_assert(sizeof(PanelShaderConstants) == 144U);
static_assert(sizeof(PanelShaderConstants) % 16U == 0U);
static_assert(offsetof(PanelShaderConstants, desktopContentBounds) == 64U);
static_assert(offsetof(PanelShaderConstants, desktopCrop) == 80U);
static_assert(offsetof(PanelShaderConstants, desktopEnabled) == 104U);
static_assert(offsetof(PanelShaderConstants, desktopUnavailable) == 108U);
static_assert(offsetof(PanelShaderConstants, desktopDebugMode) == 128U);

struct DesktopTextureDescriptorContract
{
    std::uint32_t width{};
    std::uint32_t height{};
    std::uint32_t mipLevels{};
    std::uint32_t arraySize{};
    std::uint32_t format{};
    std::uint32_t sampleCount{};
    std::uint32_t sampleQuality{};
    std::uint32_t usage{};
    std::uint32_t bindFlags{};
    std::uint32_t cpuAccessFlags{};
    std::uint32_t miscFlags{};
};

struct DesktopShaderResourceDescriptorContract
{
    std::uint32_t format{};
    std::uint32_t viewDimension{};
    std::uint32_t mostDetailedMip{};
    std::uint32_t mipLevels{};
};

struct PanelVertexUv
{
    float u{};
    float v{};
};

[[nodiscard]] std::string_view desktopShaderDebugModeText(
    DesktopShaderDebugMode mode) noexcept;
[[nodiscard]] bool desktopShaderDebugModeUsesTexture(
    DesktopShaderDebugMode mode) noexcept;
[[nodiscard]] bool desktopOverlayEnabled(DesktopShaderDebugMode mode) noexcept;
[[nodiscard]] DesktopPanelBlendMode desktopBaseBlendMode(
    bool desktopBase,
    bool forceOpaque) noexcept;
[[nodiscard]] DesktopPanelBlendMode desktopOverlayBlendMode() noexcept;
[[nodiscard]] bool isShaderReadableDesktopTexture(
    const DesktopTextureDescriptorContract& descriptor) noexcept;
[[nodiscard]] bool isCompatibleDesktopShaderResource(
    const DesktopTextureDescriptorContract& texture,
    const DesktopShaderResourceDescriptorContract& resource) noexcept;
[[nodiscard]] bool panelUvConstantsFinite(const PanelShaderConstants& constants) noexcept;
[[nodiscard]] bool validatePanelGeometry(
    std::span<const PanelVertexUv> vertices,
    std::span<const std::uint16_t> indices) noexcept;
[[nodiscard]] bool validateDesktopDrawOrdering(
    std::span<const DesktopDrawEvent> events,
    bool overlayExpected) noexcept;
[[nodiscard]] std::string_view desktopDrawEventText(DesktopDrawEvent event) noexcept;

} // namespace xreal::rendering
