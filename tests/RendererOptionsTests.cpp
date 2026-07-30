#include "rendering/RendererOptions.hpp"

#include <array>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace
{
int failures{};
void expect(bool condition, const char* message)
{
    if (!condition) { std::cerr << "FAILED: " << message << '\n'; ++failures; }
}

xreal::rendering::RendererOptionResult parse(std::initializer_list<const char*> values)
{
    std::vector<std::string> storage{"renderer"};
    for (const char* value : values) { storage.emplace_back(value); }
    std::vector<char*> arguments;
    for (auto& value : storage) { arguments.push_back(value.data()); }
    return xreal::rendering::parseRendererOptions(
        static_cast<int>(arguments.size()), arguments.data());
}
}

int main()
{
    using namespace xreal::rendering;
    const auto demo = parse({"--orientation-demo-mode"});
    expect(demo.options.has_value() && demo.options->windowWidth == 1280U
               && demo.options->windowHeight == 720U && demo.options->vsync,
           "demo defaults are valid");
    expect(demo.options.has_value()
               && demo.options->desktopCapture.panelContent
                    == xreal::capture::PanelContent::synthetic,
           "synthetic panel content is the default");
    expect(!parse({"--orientation-demo-mode", "--panel-content", "desktop"})
                .options.has_value(),
           "desktop mode requires explicit capture monitor selection");
    const auto checkerboard = parse({"--orientation-demo-mode",
        "--desktop-debug-checkerboard"});
    expect(checkerboard.options.has_value() && checkerboard.options->desktopDebugCheckerboard
            && checkerboard.options->desktopCapture.panelContent
                == xreal::capture::PanelContent::desktop,
        "desktop debug checkerboard selects desktop mode without capture monitor");
    const auto staticDemo = parse({"--orientation-demo-static"});
    expect(staticDemo.options.has_value() && staticDemo.options->orientationDemoMode
            && staticDemo.options->orientationDemoStatic,
        "static demo mode keeps the demo orientation at identity");
    const auto shaderModes = parse({"--orientation-demo-static",
        "--desktop-debug-checkerboard", "--desktop-debug-shader-sample-no-overlay",
        "--desktop-debug-opaque-base", "--desktop-debug-readback-upload", "upload.bmp",
        "--desktop-debug-dump-render-target", "target.bmp"});
    expect(shaderModes.options.has_value()
            && shaderModes.options->desktopShaderDebugMode
                == DesktopShaderDebugMode::sampleNoOverlay
            && shaderModes.options->desktopDebugOpaqueBase
            && shaderModes.options->desktopDebugReadbackUploadPath == "upload.bmp"
            && shaderModes.options->desktopDebugRenderTargetPath == "target.bmp",
        "desktop renderer diagnostic options preserve explicit modes and paths");
    expect(!parse({"--orientation-demo-mode", "--desktop-debug-checkerboard",
            "--desktop-debug-shader-solid-red", "--desktop-debug-shader-uv"})
            .options.has_value(),
        "forced desktop shader modes are mutually exclusive");

    PanelShaderConstants constants;
    constants.desktopContentBounds = {0.0F, 0.0F, 1.0F, 1.0F};
    constants.desktopCrop = {0.0F, 0.0F, 1.0F, 1.0F};
    expect(sizeof(constants) == 144U && sizeof(constants) % 16U == 0U
            && offsetof(PanelShaderConstants, desktopEnabled) == 104U
            && offsetof(PanelShaderConstants, desktopUnavailable) == 108U,
        "panel constant-buffer layout matches the 16-byte HLSL packing contract");
    expect(panelUvConstantsFinite(constants),
        "valid UV scale and offset constants are finite");
    constants.desktopCrop[2] = std::numeric_limits<float>::infinity();
    expect(!panelUvConstantsFinite(constants),
        "non-finite UV constants are rejected");

    const DesktopTextureDescriptorContract texture{
        640U, 360U, 1U, 1U, bgra8UnormFormat, 1U, 0U,
        defaultTextureUsage, shaderResourceBindFlag, 0U, 0U};
    const DesktopShaderResourceDescriptorContract resource{
        bgra8UnormFormat, texture2dSrvDimension, 0U, 1U};
    expect(isShaderReadableDesktopTexture(texture),
        "upload texture descriptor is shader-readable BGRA");
    expect(isCompatibleDesktopShaderResource(texture, resource),
        "SRV descriptor format and mip contract match the upload texture");
    auto invalidTexture = texture;
    invalidTexture.cpuAccessFlags = 1U;
    expect(!isShaderReadableDesktopTexture(invalidTexture),
        "default upload textures reject CPU access flags");

    expect(desktopTextureShaderRegister == 0U
            && desktopSamplerShaderRegister == 0U
            && panelConstantBufferShaderRegister == 0U,
        "C++ binding slots enforce t0, s0 and b0");
    std::ifstream shader(XREAL_PANEL_SHADER_PATH, std::ios::binary);
    std::ostringstream shaderText;
    shaderText << shader.rdbuf();
    expect(shader && shaderText.str().find("CameraConstants : register(b0)")
                != std::string::npos
            && shaderText.str().find("desktopTexture : register(t0)")
                != std::string::npos
            && shaderText.str().find("desktopSampler : register(s0)")
                != std::string::npos,
        "HLSL register declarations match the explicit C++ binding slots");

    expect(desktopShaderDebugModeText(DesktopShaderDebugMode::solidRed) == "solid_red"
            && !desktopShaderDebugModeUsesTexture(DesktopShaderDebugMode::solidRed),
        "solid-red mode selects the no-sampling branch");
    expect(desktopShaderDebugModeText(DesktopShaderDebugMode::ultraviolet) == "uv"
            && !desktopShaderDebugModeUsesTexture(DesktopShaderDebugMode::ultraviolet),
        "UV mode selects the interpolation branch");
    expect(desktopShaderDebugModeUsesTexture(DesktopShaderDebugMode::sample)
            && desktopShaderDebugModeUsesTexture(DesktopShaderDebugMode::sampleNoOverlay)
            && !desktopOverlayEnabled(DesktopShaderDebugMode::sampleNoOverlay),
        "sample modes select texture-only sampling and no-overlay suppresses overlays");
    expect(desktopBaseBlendMode(true, true) == DesktopPanelBlendMode::opaque
            && desktopBaseBlendMode(true, false) == DesktopPanelBlendMode::alpha
            && desktopOverlayBlendMode() == DesktopPanelBlendMode::alpha,
        "opaque diagnostic base disables blending and the overlay remains alpha blended");

    constexpr std::array vertices{
        PanelVertexUv{0.0F, 1.0F}, PanelVertexUv{0.0F, 0.0F},
        PanelVertexUv{1.0F, 1.0F}, PanelVertexUv{1.0F, 0.0F}};
    constexpr std::array<std::uint16_t, 6> indices{0U, 1U, 2U, 2U, 1U, 3U};
    expect(validatePanelGeometry(vertices, indices),
        "panel vertex UVs and index ranges are valid");
    constexpr std::array orderedEvents{
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
        DesktopDrawEvent::overlayDraw,
        DesktopDrawEvent::shaderResourceUnbound,
        DesktopDrawEvent::presented};
    expect(validateDesktopDrawOrdering(orderedEvents, true),
        "SRV is bound before DrawIndexed and unbound only after the overlay draw");
    auto invalidEvents = orderedEvents;
    std::swap(invalidEvents[5], invalidEvents[11]);
    expect(!validateDesktopDrawOrdering(invalidEvents, true),
        "draw ordering rejects an SRV unbound before DrawIndexed");
    const auto dump = parse({"--orientation-demo-mode", "--panel-content", "desktop",
        "--capture-monitor-index", "0", "--desktop-capture-dump-first-frame", "frame.bmp"});
    expect(dump.options.has_value()
            && dump.options->desktopCapture.firstFrameBmpPath == "frame.bmp",
        "first-frame BMP path is parsed without changing panel mode");
    const auto desktop = parse({"--orientation-demo-mode", "--panel-content", "desktop",
        "--render-monitor-index", "1", "--capture-monitor-index", "0",
        "--desktop-fit", "contain", "--desktop-filter", "linear"});
    expect(desktop.options.has_value() && desktop.options->monitorIndex == 1U
               && desktop.options->desktopCapture.captureMonitor.index == 0U,
           "render and capture monitor options remain independent");
    expect(!parse({"--orientation-demo-mode", "--panel-content", "desktop",
                  "--capture-monitor-index", "0", "--desktop-fit", "distort"})
                .options.has_value(),
           "invalid desktop fit is rejected");
    expect(!parse({"--orientation-demo-mode", "--panel-content", "desktop",
                  "--capture-monitor-index", "0", "--desktop-filter", "cubic"})
                .options.has_value(),
           "invalid desktop filter is rejected");
    expect(!parse({"--orientation-demo-mode", "--panel-content", "desktop",
                  "--capture-monitor-index", "0", "--desktop-capture-timeout-ms", "0"})
                .options.has_value(),
           "zero desktop capture timeout is rejected");
    expect(!parse({"--orientation-demo-mode", "--panel-content", "desktop",
                  "--capture-monitor-index", "0", "--desktop-capture-retry-ms", "0"})
                .options.has_value(),
           "zero desktop capture retry is rejected");
    expect(!parse({"--orientation-demo-mode", "--panel-content", "desktop",
                  "--capture-monitor-index", "0", "--desktop-capture-cross-adapter", "magic"})
                .options.has_value(),
           "invalid cross-adapter policy is rejected");
    expect(!parse({"--orientation-demo-mode", "--panel-content", "desktop",
                  "--capture-monitor-index", "0", "--desktop-capture-cross-adapter", "cpu-fallback"})
                .options.has_value(),
           "CPU fallback requires explicit opt-in");
    expect(!parse({"--orientation-demo-mode", "--window-width", "0"}).options.has_value(),
           "invalid window width is rejected");
    expect(!parse({"--orientation-demo-mode", "--monitor-index", "-1"}).options.has_value(),
           "invalid monitor index syntax is rejected");
    expect(!parse({"--orientation-demo-mode", "--field-of-view-degrees", "180"}).options.has_value(),
           "invalid FOV is rejected");
    expect(!parse({"--orientation-demo-mode", "--panel-width", "0"}).options.has_value(),
           "invalid panel size is rejected");
    expect(!parse({"--orientation-demo-mode", "--render-orientation-source", "future"})
                .options.has_value(),
           "invalid orientation source is rejected");
    expect(!parse({"--orientation-demo-mode", "--render-orientation-frame", "camera"})
                .options.has_value(),
           "invalid orientation frame is rejected");
    const auto relative = parse({"--orientation-demo-mode", "--render-orientation-frame",
        "relative", "--recenter-on-start"});
    expect(relative.options.has_value() && relative.options->recenterOnStart,
           "relative frame supports recenter");
    expect(!parse({}).options.has_value(),
           "hardware mode requires explicit calibration inputs");
    expect(!parse({"--apply-gyro-bias", "--accelerometer-profile", "a.json"})
                .options.has_value(),
           "hardware mode requires explicit gyro scale");
    expect(!parse({"--apply-gyro-bias", "--gyro-scale-raw-per-dps", "4090"})
                .options.has_value(),
           "hardware mode requires accelerometer profile");
    const auto measuredHardware = parse({"--apply-gyro-bias", "--gyro-scale-raw-per-dps",
        "4090", "--accelerometer-profile", "a.json", "--render-orientation-source", "measured"});
    expect(measuredHardware.options.has_value() && !measuredHardware.options->predictOrientation,
           "measured hardware mode works without prediction");
    expect(!parse({"--apply-gyro-bias", "--gyro-scale-raw-per-dps", "4090",
                  "--accelerometer-profile", "a.json", "--render-orientation-source", "predicted"})
                .options.has_value(),
           "predicted hardware mode requires explicit prediction");
    expect(!parse({"--orientation-demo-mode", "--render-orientation-source", "measured",
                  "--prediction-horizon-ms", "15"}).options.has_value(),
           "prediction options require predicted source");
    const auto smoke = parse({"--smoke-test", "--smoke-test-frames", "3"});
    expect(smoke.options.has_value() && smoke.options->orientationDemoMode,
           "explicit smoke test uses demo mode without HID");
    if (failures != 0) { return 1; }
    std::cout << "All renderer option tests passed.\n";
    return 0;
}
