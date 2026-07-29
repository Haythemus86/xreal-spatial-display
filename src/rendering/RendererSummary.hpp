#pragma once

#include "graphics/D3D11Renderer.hpp"
#include "platform/windows/RenderWindow.hpp"
#include "rendering/OrientationRenderBridge.hpp"
#include "rendering/RenderDiagnostics.hpp"
#include "rendering/RendererOptions.hpp"

#include <string>

namespace xreal::rendering
{

struct RendererSummary
{
    RendererOptions options;
    graphics::D3D11RendererInformation graphics;
    platform::windows::MonitorInformation monitor;
    OrientationToRenderMapping mapping;
    FrameTimingStatistics timing;
    ImuHealthCounters imu;
    RendererStartupState finalState{RendererStartupState::shuttingDown};
    std::uint64_t recenterGeneration{};
    bool calibrationAccepted{};
    std::string shutdownReason{"normal"};
    std::string error;
};

[[nodiscard]] std::string serializeRendererSummaryJson(const RendererSummary& summary);

} // namespace xreal::rendering
