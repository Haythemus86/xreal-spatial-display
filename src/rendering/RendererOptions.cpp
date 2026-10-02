#include "rendering/RendererOptions.hpp"

#include <charconv>
#include <array>
#include <cmath>
#include <limits>
#include <string_view>

namespace xreal::rendering
{
namespace
{

struct PanelOptionOverrides
{
    std::optional<PanelContentKind> content;
    std::optional<unsigned int> captureMonitorIndex;
    std::optional<std::string> captureMonitorDeviceName;
    std::optional<std::string> captureMonitorStableId;
    std::optional<double> width;
    std::optional<double> height;
    std::optional<double> positionX;
    std::optional<double> positionY;
    std::optional<double> positionZ;
    std::optional<double> yawDegrees;
    std::optional<double> pitchDegrees;
    std::optional<double> targetFramesPerSecond;
    std::optional<unsigned int> sourceWidth;
    std::optional<unsigned int> sourceHeight;
    std::optional<double> sourceScale;
    std::optional<capture::DesktopResolutionPolicy> resolutionPolicy;
    std::optional<capture::DesktopCropMode> cropMode;
    std::optional<capture::DesktopCaptureRegion> cropRegion;
    std::optional<capture::DesktopFit> sourceFit;
    std::optional<capture::DesktopFilter> sourceFilter;
    std::optional<bool> allowUpscale;
    std::optional<double> uploadFramesPerSecond;
};

struct IndexedPanelOption
{
    std::size_t panelIndex{};
    std::string_view property;
};

[[nodiscard]] std::optional<IndexedPanelOption> indexedPanelOption(
    std::string_view option) noexcept
{
    constexpr std::string_view prefix = "--panel-";
    if (!option.starts_with(prefix) || option.size() <= prefix.size() + 2U)
    {
        return std::nullopt;
    }
    const char panelNumber = option[prefix.size()];
    if (panelNumber < '1' || panelNumber > '3'
        || option[prefix.size() + 1U] != '-')
    {
        return std::nullopt;
    }
    return IndexedPanelOption{
        static_cast<std::size_t>(panelNumber - '1'),
        option.substr(prefix.size() + 2U),
    };
}

[[nodiscard]] std::optional<IndexedPanelOption> indexedDesktopSourceOption(
    std::string_view option) noexcept
{
    constexpr std::string_view prefix = "--desktop-source-";
    if (!option.starts_with(prefix) || option.size() <= prefix.size() + 2U)
    {
        return std::nullopt;
    }
    const char sourceNumber = option[prefix.size()];
    if (sourceNumber < '1' || sourceNumber > '3'
        || option[prefix.size() + 1U] != '-')
    {
        return std::nullopt;
    }
    return IndexedPanelOption{
        static_cast<std::size_t>(sourceNumber - '1'),
        option.substr(prefix.size() + 2U),
    };
}

[[nodiscard]] std::optional<double> parseDouble(std::string_view value)
{
    double parsed{};
    const auto result = std::from_chars(value.data(), value.data() + value.size(), parsed);
    if (result.ec != std::errc{} || result.ptr != value.data() + value.size()
        || !std::isfinite(parsed))
    {
        return std::nullopt;
    }
    return parsed;
}

template<typename Integer>
[[nodiscard]] std::optional<Integer> parseInteger(std::string_view value)
{
    Integer parsed{};
    const auto result = std::from_chars(value.data(), value.data() + value.size(), parsed);
    if (result.ec != std::errc{} || result.ptr != value.data() + value.size())
    {
        return std::nullopt;
    }
    return parsed;
}

[[nodiscard]] bool requiresValue(std::string_view option)
{
    return option == "--window-width" || option == "--window-height"
        || option == "--window-x" || option == "--window-y"
        || option == "--monitor-index" || option == "--render-monitor-index"
        || option == "--capture-monitor-index"
        || option == "--render-monitor-device-name"
        || option == "--capture-monitor-device-name"
        || option == "--panel-content" || option == "--desktop-fit"
        || option == "--desktop-filter" || option == "--desktop-background"
        || option == "--desktop-stale-threshold-ms"
        || option == "--desktop-capture-timeout-ms"
        || option == "--desktop-capture-retry-ms"
        || option == "--desktop-capture-cross-adapter"
        || option == "--desktop-capture-json-output"
        || option == "--desktop-capture-dump-first-frame"
        || option == "--desktop-debug-readback-upload"
        || option == "--desktop-debug-dump-render-target"
        || option == "--desktop-capture-smoke-test-frames"
        || option == "--render-duration"
        || option == "--panel-distance" || option == "--panel-width"
        || option == "--panel-height" || option == "--field-of-view-degrees"
        || option == "--near-plane" || option == "--far-plane"
        || option == "--render-orientation-source"
        || option == "--render-orientation-frame"
        || option == "--recenter-key" || option == "--reset-recenter-key"
        || option == "--gyro-calibrate-seconds" || option == "--gyro-warmup-seconds"
        || option == "--gyro-scale-raw-per-dps" || option == "--accelerometer-profile"
        || option == "--fusion-mode" || option == "--fusion-startup"
        || option == "--accelerometer-correction-time-constant"
        || option == "--accelerometer-max-correction-dps"
        || option == "--prediction-mode" || option == "--prediction-horizon-ms"
        || option == "--prediction-max-horizon-ms"
        || option == "--prediction-angular-velocity-smoothing-seconds"
        || option == "--prediction-max-angular-speed-dps"
        || option == "--prediction-max-angle-degrees"
        || option == "--prediction-limit-behavior"
        || option == "--target-fps" || option == "--render-diagnostics-rate"
        || option == "--render-json-output" || option == "--smoke-test-frames";
}

[[nodiscard]] RendererOptionResult fail(std::string error)
{
    return {std::nullopt, std::move(error)};
}

} // namespace

RendererOptionResult parseRendererOptions(int argc, char* argv[])
{
    RendererOptions options;
    std::array<PanelOptionOverrides, maximumPanelCount> panelOverrides{};
    bool panelLayoutExplicit{};
    bool panelConfigurationExplicit{};
    bool predictionOptionExplicit{};
    unsigned int desktopShaderDebugModeCount{};
    for (int index = 1; index < argc; ++index)
    {
        const std::string_view argument(argv[index]);
        if (argument == "--help" || argument == "-h")
        {
            options.help = true;
            continue;
        }
        if (argument == "--fullscreen") { options.fullscreen = true; continue; }
        if (argument == "--borderless") { options.borderless = true; continue; }
        if (argument == "--vsync") { options.vsync = true; continue; }
        if (argument == "--no-vsync") { options.vsync = false; continue; }
        if (argument == "--allow-warp-fallback") { options.allowWarpFallback = true; continue; }
        if (argument == "--show-render-diagnostics") { options.showRenderDiagnostics = true; continue; }
        if (argument == "--background-grid") { options.backgroundGrid = true; continue; }
        if (argument == "--world-axes") { options.worldAxes = true; continue; }
        if (argument == "--recenter-on-start") { options.recenterOnStart = true; continue; }
        if (argument == "--xreal-sdk-pose") { options.xrealSdkPose = true; continue; }
        if (argument == "--orientation-demo-mode") { options.orientationDemoMode = true; continue; }
        if (argument == "--orientation-demo-static")
        {
            options.orientationDemoMode = true;
            options.orientationDemoStatic = true;
            continue;
        }
        if (argument == "--smoke-test") { options.smokeTest = true; continue; }
        if (argument == "--desktop-capture-smoke-test") { options.desktopCaptureSmokeTest = true; continue; }
        if (argument == "--desktop-debug-checkerboard") { options.desktopDebugCheckerboard = true; continue; }
        if (argument == "--desktop-debug-shader-solid-red")
        {
            options.desktopShaderDebugMode = DesktopShaderDebugMode::solidRed;
            ++desktopShaderDebugModeCount;
            continue;
        }
        if (argument == "--desktop-debug-shader-uv")
        {
            options.desktopShaderDebugMode = DesktopShaderDebugMode::ultraviolet;
            ++desktopShaderDebugModeCount;
            continue;
        }
        if (argument == "--desktop-debug-shader-sample")
        {
            options.desktopShaderDebugMode = DesktopShaderDebugMode::sample;
            ++desktopShaderDebugModeCount;
            continue;
        }
        if (argument == "--desktop-debug-shader-sample-no-overlay")
        {
            options.desktopShaderDebugMode = DesktopShaderDebugMode::sampleNoOverlay;
            ++desktopShaderDebugModeCount;
            continue;
        }
        if (argument == "--desktop-debug-opaque-base")
        {
            options.desktopDebugOpaqueBase = true;
            continue;
        }
        if (argument == "--desktop-show-cursor") { options.desktopCapture.showCursor = true; continue; }
        if (argument == "--desktop-hide-cursor") { options.desktopCapture.showCursor = false; continue; }
        if (argument == "--desktop-flip-y") { options.desktopCapture.flipY = true; continue; }
        if (argument == "--allow-desktop-capture-cpu-fallback") { options.desktopCapture.allowCpuFallback = true; continue; }
        if (argument == "--desktop-capture-diagnostics") { options.desktopCapture.diagnostics = true; continue; }
        if (argument == "--apply-gyro-bias") { options.applyGyroscopeBias = true; continue; }
        if (argument == "--predict-orientation")
        {
            options.predictOrientation = true;
            predictionOptionExplicit = true;
            continue;
        }
        if (argument == "--save-panel-layout-on-exit")
        {
            options.savePanelLayoutOnExit = true;
            continue;
        }
        if (argument == "--overwrite-panel-layout")
        {
            options.overwritePanelLayout = true;
            continue;
        }
        if (argument == "--multi-panel-benchmark")
        {
            options.multiPanelBenchmark = true;
            continue;
        }
        if (argument == "--capture-benchmark")
        {
            options.captureBenchmark = true;
            options.multiPanelBenchmark = true;
            continue;
        }

        if (const auto sourceOption = indexedDesktopSourceOption(argument);
            sourceOption.has_value())
        {
            panelConfigurationExplicit = true;
            auto& override = panelOverrides[sourceOption->panelIndex];
            if (sourceOption->property == "allow-upscale")
            {
                override.allowUpscale = true;
                continue;
            }
            if (sourceOption->property == "region")
            {
                if (index + 4 >= argc)
                {
                    return fail(std::string(argument) + " requires x y width height.");
                }
                const auto x = parseInteger<unsigned int>(argv[++index]);
                const auto y = parseInteger<unsigned int>(argv[++index]);
                const auto width = parseInteger<unsigned int>(argv[++index]);
                const auto height = parseInteger<unsigned int>(argv[++index]);
                if (!x.has_value() || !y.has_value() || !width.has_value()
                    || !height.has_value() || *width == 0U || *height == 0U)
                {
                    return fail(std::string(argument)
                        + " requires non-negative x/y and positive width/height.");
                }
                override.cropRegion = capture::DesktopCaptureRegion{
                    *x, *y, *width, *height};
                override.cropMode = capture::DesktopCropMode::custom;
                continue;
            }
            if (index + 1 >= argc)
            {
                return fail(std::string(argument) + " requires a value.");
            }
            const std::string_view value(argv[++index]);
            if (sourceOption->property == "target-width"
                || sourceOption->property == "target-height")
            {
                const auto parsed = parseInteger<unsigned int>(value);
                if (!parsed.has_value() || *parsed == 0U)
                {
                    return fail(std::string(argument) + " requires a positive integer.");
                }
                if (sourceOption->property == "target-width")
                {
                    override.sourceWidth = *parsed;
                }
                else
                {
                    override.sourceHeight = *parsed;
                }
                override.resolutionPolicy = capture::DesktopResolutionPolicy::fixed;
                continue;
            }
            if (sourceOption->property == "crop")
            {
                override.cropMode = capture::parseDesktopCropMode(value);
                if (!override.cropMode.has_value())
                {
                    return fail(std::string(argument)
                        + " requires full, center-16x9 or custom.");
                }
                continue;
            }
            if (sourceOption->property == "fit")
            {
                if (value == "contain") { override.sourceFit = capture::DesktopFit::contain; }
                else if (value == "cover") { override.sourceFit = capture::DesktopFit::cover; }
                else if (value == "stretch") { override.sourceFit = capture::DesktopFit::stretch; }
                else { return fail(std::string(argument) + " requires contain, cover or stretch."); }
                continue;
            }
            if (sourceOption->property == "filter")
            {
                if (value == "point") { override.sourceFilter = capture::DesktopFilter::point; }
                else if (value == "linear") { override.sourceFilter = capture::DesktopFilter::linear; }
                else { return fail(std::string(argument) + " requires point or linear."); }
                continue;
            }
            if (sourceOption->property == "resolution-policy")
            {
                override.resolutionPolicy = capture::parseDesktopResolutionPolicy(value);
                if (!override.resolutionPolicy.has_value())
                {
                    return fail(std::string(argument)
                        + " requires native, panel-aware or fixed.");
                }
                continue;
            }
            const auto parsed = parseDouble(value);
            if (!parsed.has_value() || *parsed <= 0.0)
            {
                return fail(std::string(argument) + " requires a positive finite number.");
            }
            if (sourceOption->property == "scale")
            {
                override.sourceScale = *parsed;
                override.resolutionPolicy = capture::DesktopResolutionPolicy::fixed;
            }
            else if (sourceOption->property == "capture-fps")
            {
                override.targetFramesPerSecond = *parsed;
            }
            else if (sourceOption->property == "upload-fps")
            {
                override.uploadFramesPerSecond = *parsed;
            }
            else
            {
                return fail("Unknown renderer option: " + std::string(argument));
            }
            continue;
        }

        if (const auto panelOption = indexedPanelOption(argument); panelOption.has_value())
        {
            panelConfigurationExplicit = true;
            if (index + 1 >= argc)
            {
                return fail(std::string(argument) + " requires a value.");
            }
            const std::string_view value(argv[++index]);
            auto& override = panelOverrides[panelOption->panelIndex];
            if (panelOption->property == "content")
            {
                override.content = parsePanelContentKind(value);
                if (!override.content.has_value())
                {
                    return fail(std::string(argument)
                        + " requires synthetic, checkerboard, desktop or unavailable.");
                }
                continue;
            }
            if (panelOption->property == "capture-monitor-device-name"
                || panelOption->property == "capture-monitor-stable-id")
            {
                if (value.empty())
                {
                    return fail(std::string(argument) + " requires a non-empty value.");
                }
                if (panelOption->property == "capture-monitor-device-name")
                {
                    override.captureMonitorDeviceName = std::string(value);
                }
                else
                {
                    override.captureMonitorStableId = std::string(value);
                }
                continue;
            }
            if (panelOption->property == "capture-monitor-index"
                || panelOption->property == "source-target-width"
                || panelOption->property == "source-target-height")
            {
                const auto parsed = parseInteger<unsigned int>(value);
                if (!parsed.has_value())
                {
                    return fail(std::string(argument) + " requires an unsigned integer.");
                }
                if (panelOption->property == "capture-monitor-index")
                {
                    override.captureMonitorIndex = *parsed;
                }
                else if (panelOption->property == "source-target-width")
                {
                    override.sourceWidth = *parsed;
                }
                else
                {
                    override.sourceHeight = *parsed;
                }
                continue;
            }
            const auto parsed = parseDouble(value);
            if (!parsed.has_value())
            {
                return fail(std::string(argument) + " requires a finite number.");
            }
            if (panelOption->property == "width-m") { override.width = *parsed; }
            else if (panelOption->property == "height-m") { override.height = *parsed; }
            else if (panelOption->property == "position-x-m") { override.positionX = *parsed; }
            else if (panelOption->property == "position-y-m") { override.positionY = *parsed; }
            else if (panelOption->property == "position-z-m") { override.positionZ = *parsed; }
            else if (panelOption->property == "yaw-degrees") { override.yawDegrees = *parsed; }
            else if (panelOption->property == "pitch-degrees") { override.pitchDegrees = *parsed; }
            else if (panelOption->property == "target-fps") { override.targetFramesPerSecond = *parsed; }
            else if (panelOption->property == "source-scale") { override.sourceScale = *parsed; }
            else { return fail("Unknown renderer option: " + std::string(argument)); }
            continue;
        }

        if (!requiresValue(argument))
        {
            const bool multiPanelValue = argument == "--panel-count"
                || argument == "--panel-width-m" || argument == "--panel-height-m"
                || argument == "--panel-distance-m" || argument == "--panel-gap-m"
                || argument == "--panel-curvature-degrees" || argument == "--panel-layout"
                || argument == "--panel-layout-file"
                || argument == "--panel-layout-save-file"
                || argument == "--performance-profile"
                || argument == "--multi-panel-benchmark-seconds"
                || argument == "--multi-panel-benchmark-warmup-seconds"
                || argument == "--multi-panel-benchmark-panels"
                || argument == "--multi-panel-benchmark-content"
                || argument == "--multi-panel-benchmark-json"
                || argument == "--performance-json-output"
                || argument == "--desktop-resolution-policy"
                || argument == "--desktop-resolution-safety-factor"
                || argument == "--desktop-bandwidth-warning-mib-s"
                || argument == "--capture-benchmark-sources"
                || argument == "--capture-benchmark-seconds"
                || argument == "--capture-benchmark-target-width"
                || argument == "--capture-benchmark-target-height"
                || argument == "--capture-benchmark-fps"
                || argument == "--capture-benchmark-json";
            if (!multiPanelValue)
            {
                return fail("Unknown renderer option: " + std::string(argument));
            }
        }
        if (index + 1 >= argc)
        {
            return fail(std::string(argument) + " requires a value.");
        }
        const std::string_view value(argv[++index]);
        if (argument == "--panel-count" || argument == "--multi-panel-benchmark-panels")
        {
            const auto parsed = parseInteger<std::size_t>(value);
            if (!parsed.has_value() || *parsed == 0U || *parsed > maximumPanelCount)
            {
                return fail(std::string(argument) + " requires 1, 2 or 3.");
            }
            if (argument == "--panel-count")
            {
                options.panelScene.panelCount = *parsed;
                panelConfigurationExplicit = true;
            }
            else { options.multiPanelBenchmarkPanels = *parsed; }
            continue;
        }
        if (argument == "--capture-benchmark-sources")
        {
            const auto parsed = parseInteger<std::size_t>(value);
            if (!parsed.has_value() || *parsed == 0U || *parsed > maximumPanelCount)
            {
                return fail("--capture-benchmark-sources requires 1, 2 or 3.");
            }
            options.captureBenchmark = true;
            options.multiPanelBenchmark = true;
            options.captureBenchmarkSources = *parsed;
            continue;
        }
        if (argument == "--capture-benchmark-target-width"
            || argument == "--capture-benchmark-target-height")
        {
            const auto parsed = parseInteger<unsigned int>(value);
            if (!parsed.has_value() || *parsed == 0U)
            {
                return fail(std::string(argument) + " requires a positive integer.");
            }
            options.captureBenchmark = true;
            options.multiPanelBenchmark = true;
            if (argument == "--capture-benchmark-target-width")
            {
                options.captureBenchmarkTargetWidth = *parsed;
            }
            else
            {
                options.captureBenchmarkTargetHeight = *parsed;
            }
            continue;
        }
        if (argument == "--panel-layout")
        {
            const auto parsed = parsePanelLayoutPreset(value);
            if (!parsed.has_value())
            {
                return fail("--panel-layout requires single, dual-flat, triple-flat, triple-angled or custom.");
            }
            options.panelScene.layout = *parsed;
            panelLayoutExplicit = true;
            panelConfigurationExplicit = true;
            continue;
        }
        if (argument == "--performance-profile")
        {
            const auto parsed = parsePerformanceProfile(value);
            if (!parsed.has_value())
            {
                return fail("--performance-profile requires quality, balanced or performance.");
            }
            options.panelScene.performanceProfile = *parsed;
            continue;
        }
        if (argument == "--desktop-resolution-policy")
        {
            const auto parsed = capture::parseDesktopResolutionPolicy(value);
            if (!parsed.has_value())
            {
                return fail("--desktop-resolution-policy requires native, panel-aware or fixed.");
            }
            options.desktopResolutionPolicy = *parsed;
            continue;
        }
        if (argument == "--multi-panel-benchmark-content")
        {
            const auto parsed = parsePanelContentKind(value);
            if (!parsed.has_value() || *parsed == PanelContentKind::unavailable)
            {
                return fail("--multi-panel-benchmark-content requires synthetic, checkerboard or desktop.");
            }
            options.multiPanelBenchmarkContent = *parsed;
            continue;
        }
        if (argument == "--panel-layout-file" || argument == "--panel-layout-save-file"
            || argument == "--multi-panel-benchmark-json"
            || argument == "--performance-json-output"
            || argument == "--capture-benchmark-json")
        {
            if (value.empty())
            {
                return fail(std::string(argument) + " requires a non-empty path.");
            }
            if (argument == "--panel-layout-file")
            {
                options.panelLayoutFilePath = std::string(value);
            }
            else if (argument == "--panel-layout-save-file")
            {
                options.panelLayoutSavePath = std::string(value);
            }
            else if (argument == "--capture-benchmark-json")
            {
                options.captureBenchmark = true;
                options.multiPanelBenchmark = true;
                options.captureBenchmarkJsonPath = std::string(value);
                options.multiPanelBenchmarkJsonPath = std::string(value);
            }
            else
            {
                options.multiPanelBenchmarkJsonPath = std::string(value);
            }
            continue;
        }
        if (argument.starts_with("--prediction-"))
        {
            predictionOptionExplicit = true;
        }

        if (argument == "--window-width" || argument == "--window-height"
            || argument == "--monitor-index" || argument == "--render-monitor-index"
            || argument == "--capture-monitor-index"
            || argument == "--render-diagnostics-rate"
            || argument == "--smoke-test-frames"
            || argument == "--desktop-capture-smoke-test-frames"
            || argument == "--desktop-stale-threshold-ms"
            || argument == "--desktop-capture-timeout-ms"
            || argument == "--desktop-capture-retry-ms")
        {
            const auto parsed = parseInteger<unsigned int>(value);
            if (!parsed.has_value()) { return fail(std::string(argument) + " requires an unsigned integer."); }
            if (argument == "--window-width") { options.windowWidth = *parsed; }
            else if (argument == "--window-height") { options.windowHeight = *parsed; }
            else if (argument == "--monitor-index" || argument == "--render-monitor-index") { options.monitorIndex = *parsed; }
            else if (argument == "--capture-monitor-index") { options.desktopCapture.captureMonitor.index = *parsed; }
            else if (argument == "--render-diagnostics-rate") { options.renderDiagnosticsRateHz = *parsed; }
            else if (argument == "--smoke-test-frames") { options.smokeTestFrames = *parsed; }
            else if (argument == "--desktop-capture-smoke-test-frames") { options.desktopCaptureSmokeTestFrames = *parsed; }
            else if (argument == "--desktop-stale-threshold-ms") { options.desktopCapture.staleThresholdMilliseconds = *parsed; }
            else if (argument == "--desktop-capture-timeout-ms") { options.desktopCapture.timeoutMilliseconds = *parsed; }
            else { options.desktopCapture.retryMilliseconds = *parsed; }
            continue;
        }
        if (argument == "--window-x" || argument == "--window-y")
        {
            const auto parsed = parseInteger<int>(value);
            if (!parsed.has_value()) { return fail(std::string(argument) + " requires an integer."); }
            if (argument == "--window-x") { options.windowX = *parsed; }
            else { options.windowY = *parsed; }
            continue;
        }
        if (argument == "--render-orientation-source")
        {
            if (value == "measured") { options.orientationSource = RenderOrientationSource::measured; }
            else if (value == "predicted") { options.orientationSource = RenderOrientationSource::predicted; }
            else { return fail("--render-orientation-source requires measured or predicted."); }
            continue;
        }
        if (argument == "--panel-content")
        {
            if (value == "synthetic") { options.desktopCapture.panelContent = capture::PanelContent::synthetic; }
            else if (value == "desktop") { options.desktopCapture.panelContent = capture::PanelContent::desktop; }
            else { return fail("--panel-content requires synthetic or desktop."); }
            continue;
        }
        if (argument == "--desktop-debug-readback-upload")
        {
            options.desktopDebugReadbackUploadPath = std::string(value);
            continue;
        }
        if (argument == "--desktop-debug-dump-render-target")
        {
            options.desktopDebugRenderTargetPath = std::string(value);
            continue;
        }
        if (argument == "--desktop-fit")
        {
            if (value == "contain") { options.desktopCapture.fit = capture::DesktopFit::contain; }
            else if (value == "cover") { options.desktopCapture.fit = capture::DesktopFit::cover; }
            else if (value == "stretch") { options.desktopCapture.fit = capture::DesktopFit::stretch; }
            else { return fail("--desktop-fit requires contain, cover or stretch."); }
            continue;
        }
        if (argument == "--desktop-filter")
        {
            if (value == "point") { options.desktopCapture.filter = capture::DesktopFilter::point; }
            else if (value == "linear") { options.desktopCapture.filter = capture::DesktopFilter::linear; }
            else { return fail("--desktop-filter requires point or linear."); }
            continue;
        }
        if (argument == "--desktop-background")
        {
            if (value == "black") { options.desktopCapture.background = capture::DesktopBackground::black; }
            else if (value == "grid") { options.desktopCapture.background = capture::DesktopBackground::grid; }
            else { return fail("--desktop-background requires black or grid."); }
            continue;
        }
        if (argument == "--desktop-capture-cross-adapter")
        {
            if (value == "auto") { options.desktopCapture.crossAdapterPolicy = capture::CrossAdapterPolicy::automatic; }
            else if (value == "shared-handle") { options.desktopCapture.crossAdapterPolicy = capture::CrossAdapterPolicy::sharedHandle; }
            else if (value == "cpu-fallback") { options.desktopCapture.crossAdapterPolicy = capture::CrossAdapterPolicy::cpuFallback; }
            else if (value == "reject") { options.desktopCapture.crossAdapterPolicy = capture::CrossAdapterPolicy::reject; }
            else { return fail("--desktop-capture-cross-adapter requires auto, shared-handle, cpu-fallback or reject."); }
            continue;
        }
        if (argument == "--render-orientation-frame")
        {
            if (value == "absolute") { options.orientationFrame = RenderOrientationFrame::absolute; }
            else if (value == "relative") { options.orientationFrame = RenderOrientationFrame::relative; }
            else { return fail("--render-orientation-frame requires absolute or relative."); }
            continue;
        }
        if (argument == "--prediction-mode")
        {
            if (value == "constant-velocity") { options.predictionMode = sensors::PredictionMode::constantVelocity; }
            else if (value == "constant-acceleration") { options.predictionMode = sensors::PredictionMode::constantAcceleration; }
            else { return fail("--prediction-mode requires constant-velocity or constant-acceleration."); }
            continue;
        }
        if (argument == "--prediction-limit-behavior")
        {
            if (value == "reject") { options.predictionLimitBehavior = sensors::PredictionLimitBehavior::reject; }
            else if (value == "clamp") { options.predictionLimitBehavior = sensors::PredictionLimitBehavior::clamp; }
            else { return fail("--prediction-limit-behavior requires reject or clamp."); }
            continue;
        }
        if (argument == "--fusion-mode")
        {
            if (value != "complementary") { return fail("Only --fusion-mode complementary is supported by the renderer."); }
            continue;
        }
        if (argument == "--fusion-startup")
        {
            if (value == "gravity") { options.fusionStartupGravity = true; }
            else if (value == "identity") { options.fusionStartupGravity = false; }
            else { return fail("--fusion-startup requires gravity or identity."); }
            continue;
        }
        if (argument == "--recenter-key" || argument == "--reset-recenter-key")
        {
            if (value.size() != 1U) { return fail(std::string(argument) + " requires one character."); }
            const char key = static_cast<char>(std::toupper(static_cast<unsigned char>(value.front())));
            if (argument == "--recenter-key") { options.recenterKey = key; }
            else { options.resetRecenterKey = key; }
            continue;
        }
        if (argument == "--accelerometer-profile" || argument == "--render-json-output"
            || argument == "--render-monitor-device-name"
            || argument == "--capture-monitor-device-name"
            || argument == "--desktop-capture-json-output"
            || argument == "--desktop-capture-dump-first-frame")
        {
            if (value.empty()) { return fail(std::string(argument) + " requires a non-empty path."); }
            if (argument == "--accelerometer-profile") { options.accelerometerProfilePath = std::string(value); }
            else if (argument == "--render-json-output") { options.renderJsonOutputPath = std::string(value); }
            else if (argument == "--render-monitor-device-name") { options.renderMonitorDeviceName = std::string(value); }
            else if (argument == "--capture-monitor-device-name") { options.desktopCapture.captureMonitor.deviceName = std::string(value); }
            else if (argument == "--desktop-capture-json-output") { options.desktopCapture.jsonOutputPath = std::string(value); }
            else { options.desktopCapture.firstFrameBmpPath = std::string(value); }
            continue;
        }

        const auto parsed = parseDouble(value);
        if (!parsed.has_value()) { return fail(std::string(argument) + " requires a finite number."); }
        if (argument == "--render-duration") { options.renderDurationSeconds = *parsed; }
        else if (argument == "--panel-distance" || argument == "--panel-distance-m") { options.panelDistance = *parsed; }
        else if (argument == "--panel-width" || argument == "--panel-width-m") { options.panelWidth = *parsed; }
        else if (argument == "--panel-height" || argument == "--panel-height-m") { options.panelHeight = *parsed; }
        else if (argument == "--panel-gap-m") { options.panelScene.gap = *parsed; }
        else if (argument == "--panel-curvature-degrees") { options.panelScene.curvatureDegrees = *parsed; }
        else if (argument == "--multi-panel-benchmark-seconds") { options.multiPanelBenchmarkSeconds = *parsed; }
        else if (argument == "--multi-panel-benchmark-warmup-seconds") { options.multiPanelBenchmarkWarmupSeconds = *parsed; }
        else if (argument == "--capture-benchmark-seconds")
        {
            options.captureBenchmark = true;
            options.multiPanelBenchmark = true;
            options.captureBenchmarkSeconds = *parsed;
        }
        else if (argument == "--capture-benchmark-fps")
        {
            options.captureBenchmark = true;
            options.multiPanelBenchmark = true;
            options.captureBenchmarkFramesPerSecond = *parsed;
        }
        else if (argument == "--desktop-resolution-safety-factor")
        {
            options.desktopResolutionSafetyFactor = *parsed;
        }
        else if (argument == "--desktop-bandwidth-warning-mib-s")
        {
            options.desktopBandwidthWarningMebibytesPerSecond = *parsed;
        }
        else if (argument == "--field-of-view-degrees") { options.fieldOfViewDegrees = *parsed; }
        else if (argument == "--near-plane") { options.nearPlane = *parsed; }
        else if (argument == "--far-plane") { options.farPlane = *parsed; }
        else if (argument == "--gyro-calibrate-seconds") { options.gyroscopeCalibrationSeconds = *parsed; }
        else if (argument == "--gyro-warmup-seconds") { options.gyroscopeWarmupSeconds = *parsed; }
        else if (argument == "--gyro-scale-raw-per-dps") { options.gyroscopeScaleRawPerDegreePerSecond = *parsed; }
        else if (argument == "--accelerometer-correction-time-constant") { options.accelerometerCorrectionTimeConstantSeconds = *parsed; }
        else if (argument == "--accelerometer-max-correction-dps") { options.accelerometerMaximumCorrectionDegreesPerSecond = *parsed; }
        else if (argument == "--prediction-horizon-ms") { options.predictionHorizonMilliseconds = *parsed; }
        else if (argument == "--prediction-max-horizon-ms") { options.predictionMaximumHorizonMilliseconds = *parsed; }
        else if (argument == "--prediction-angular-velocity-smoothing-seconds") { options.predictionAngularVelocitySmoothingSeconds = *parsed; }
        else if (argument == "--prediction-max-angular-speed-dps") { options.predictionMaximumAngularSpeedDegreesPerSecond = *parsed; }
        else if (argument == "--prediction-max-angle-degrees") { options.predictionMaximumAngleDegrees = *parsed; }
        else if (argument == "--target-fps") { options.targetFramesPerSecond = *parsed; }
    }

    if (options.windowWidth == 0U || options.windowHeight == 0U)
    {
        return fail("Window width and height must be positive.");
    }
    if (options.fullscreen && options.borderless)
    {
        return fail("--fullscreen and --borderless are mutually exclusive.");
    }
    if (options.renderDurationSeconds.has_value() && *options.renderDurationSeconds <= 0.0)
    {
        return fail("--render-duration must be positive.");
    }
    if (options.panelDistance <= 0.0 || options.panelWidth <= 0.0 || options.panelHeight <= 0.0)
    {
        return fail("Panel distance, width and height must be positive.");
    }
    options.panelScene.defaultDistance = options.panelDistance;
    options.panelScene.defaultWidth = options.panelWidth;
    options.panelScene.defaultHeight = options.panelHeight;
    if (!panelLayoutExplicit)
    {
        options.panelScene.layout = options.panelScene.panelCount == 1U
            ? PanelLayoutPreset::single
            : options.panelScene.panelCount == 2U
                ? PanelLayoutPreset::dualFlat : PanelLayoutPreset::tripleAngled;
    }
    if (!applyPanelLayout(options.panelScene, options.panelScene.layout))
    {
        return fail("Multi-panel layout defaults are invalid.");
    }
    for (std::size_t panelIndex = 0; panelIndex < maximumPanelCount; ++panelIndex)
    {
        const auto& override = panelOverrides[panelIndex];
        auto& panel = options.panelScene.panels[panelIndex];
        panel.content.transferPolicy = options.desktopCapture.crossAdapterPolicy
            == capture::CrossAdapterPolicy::sharedHandle
            ? PanelTransferPolicy::sharedHandle
            : options.desktopCapture.crossAdapterPolicy
                == capture::CrossAdapterPolicy::cpuFallback
                ? PanelTransferPolicy::cpuFallback
                : options.desktopCapture.crossAdapterPolicy
                    == capture::CrossAdapterPolicy::reject
                    ? PanelTransferPolicy::reject : PanelTransferPolicy::automatic;
        panel.fit = options.desktopCapture.fit == capture::DesktopFit::cover
            ? PanelFitMode::cover
            : options.desktopCapture.fit == capture::DesktopFit::stretch
                ? PanelFitMode::stretch : PanelFitMode::contain;
        panel.filter = options.desktopCapture.filter == capture::DesktopFilter::point
            ? PanelFilterMode::point : PanelFilterMode::linear;
        panel.content.scaling.resolutionPolicy = options.desktopResolutionPolicy;
        panel.content.scaling.panelAwareSafetyFactor =
            options.desktopResolutionSafetyFactor;
        panel.content.scaling.fit = options.desktopCapture.fit;
        panel.content.scaling.filter = options.desktopCapture.filter;
        if (override.content.has_value()) { panel.content.kind = *override.content; }
        if (override.captureMonitorIndex.has_value())
        {
            panel.content.captureMonitorIndex = *override.captureMonitorIndex;
        }
        if (override.captureMonitorDeviceName.has_value())
        {
            panel.content.captureMonitorDeviceName = *override.captureMonitorDeviceName;
        }
        if (override.captureMonitorStableId.has_value())
        {
            panel.content.captureMonitorStableId = *override.captureMonitorStableId;
        }
        if (override.width.has_value()) { panel.dimensions.width = *override.width; }
        if (override.height.has_value()) { panel.dimensions.height = *override.height; }
        if (override.positionX.has_value()) { panel.transform.position.x = *override.positionX; }
        if (override.positionY.has_value()) { panel.transform.position.y = *override.positionY; }
        if (override.positionZ.has_value()) { panel.transform.position.z = *override.positionZ; }
        if (override.yawDegrees.has_value()) { panel.transform.yawDegrees = *override.yawDegrees; }
        if (override.pitchDegrees.has_value()) { panel.transform.pitchDegrees = *override.pitchDegrees; }
        if (override.targetFramesPerSecond.has_value())
        {
            panel.targetFramesPerSecond = *override.targetFramesPerSecond;
            panel.targetFramesPerSecondExplicit = true;
        }
        if (override.sourceWidth.has_value())
        {
            panel.content.requestedWidth = *override.sourceWidth;
            panel.content.scaling.targetWidth = *override.sourceWidth;
            panel.content.scaling.resolutionPolicy = capture::DesktopResolutionPolicy::fixed;
        }
        if (override.sourceHeight.has_value())
        {
            panel.content.requestedHeight = *override.sourceHeight;
            panel.content.scaling.targetHeight = *override.sourceHeight;
            panel.content.scaling.resolutionPolicy = capture::DesktopResolutionPolicy::fixed;
        }
        if (override.sourceScale.has_value())
        {
            panel.content.requestedScale = *override.sourceScale;
            panel.content.scaling.scale = *override.sourceScale;
            panel.content.scaling.resolutionPolicy = capture::DesktopResolutionPolicy::fixed;
        }
        if (override.resolutionPolicy.has_value())
        {
            panel.content.scaling.resolutionPolicy = *override.resolutionPolicy;
        }
        if (override.cropMode.has_value())
        {
            panel.content.scaling.cropMode = *override.cropMode;
        }
        if (override.cropRegion.has_value())
        {
            panel.content.scaling.customRegion = *override.cropRegion;
        }
        if (override.sourceFit.has_value())
        {
            panel.content.scaling.fit = *override.sourceFit;
        }
        if (override.sourceFilter.has_value())
        {
            panel.content.scaling.filter = *override.sourceFilter;
        }
        if (override.allowUpscale.has_value())
        {
            panel.content.scaling.allowUpscale = *override.allowUpscale;
        }
        if (override.uploadFramesPerSecond.has_value())
        {
            panel.content.requestedUploadFramesPerSecond = *override.uploadFramesPerSecond;
            panel.content.requestedUploadFramesPerSecondExplicit = true;
        }
    }
    // Preserve every legacy single-panel command without changing its behavior.
    if (!panelOverrides[0].content.has_value())
    {
        options.panelScene.panels[0].content.kind = options.desktopDebugCheckerboard
            ? PanelContentKind::checkerboard
            : options.desktopCapture.panelContent == capture::PanelContent::desktop
                ? PanelContentKind::desktop : PanelContentKind::synthetic;
    }
    if (!panelOverrides[0].captureMonitorIndex.has_value())
    {
        options.panelScene.panels[0].content.captureMonitorIndex =
            options.desktopCapture.captureMonitor.index;
    }
    if (!panelOverrides[0].captureMonitorDeviceName.has_value())
    {
        options.panelScene.panels[0].content.captureMonitorDeviceName =
            options.desktopCapture.captureMonitor.deviceName;
    }
    const auto sceneValidation = validatePanelScene(options.panelScene);
    if (!sceneValidation.valid)
    {
        return fail(sceneValidation.error);
    }
    refreshPanelWorldTransforms(options.panelScene);
    if (options.multiPanelBenchmarkSeconds <= 0.0
        || options.multiPanelBenchmarkWarmupSeconds < 0.0)
    {
        return fail("Multi-panel benchmark durations are invalid.");
    }
    if (options.savePanelLayoutOnExit && !options.panelLayoutSavePath.has_value())
    {
        return fail("--save-panel-layout-on-exit requires --panel-layout-save-file.");
    }
    if (options.fieldOfViewDegrees <= 0.0 || options.fieldOfViewDegrees >= 180.0
        || options.nearPlane <= 0.0 || options.farPlane <= options.nearPlane)
    {
        return fail("Projection requires 0 < FOV < 180 and 0 < near < far.");
    }
    if (options.renderDiagnosticsRateHz == 0U || options.renderDiagnosticsRateHz > 60U)
    {
        return fail("--render-diagnostics-rate must be between 1 and 60 Hz.");
    }
    if (options.targetFramesPerSecond.has_value() && *options.targetFramesPerSecond <= 0.0)
    {
        return fail("--target-fps must be positive.");
    }
    if (options.desktopResolutionSafetyFactor <= 0.0
        || options.desktopBandwidthWarningMebibytesPerSecond <= 0.0
        || options.captureBenchmarkSeconds <= 0.0
        || options.captureBenchmarkFramesPerSecond <= 0.0)
    {
        return fail("Desktop resolution, bandwidth and capture benchmark values must be positive.");
    }
    if (options.smokeTestFrames == 0U)
    {
        return fail("--smoke-test-frames must be positive.");
    }
    if (desktopShaderDebugModeCount > 1U)
    {
        return fail("Desktop pixel-shader diagnostic modes are mutually exclusive.");
    }
    if ((options.desktopDebugReadbackUploadPath.has_value()
            && options.desktopDebugReadbackUploadPath->empty())
        || (options.desktopDebugRenderTargetPath.has_value()
            && options.desktopDebugRenderTargetPath->empty()))
    {
        return fail("Desktop debug BMP output paths must not be empty.");
    }
    if (options.desktopCapture.timeoutMilliseconds == 0U
        || options.desktopCapture.retryMilliseconds == 0U
        || options.desktopCapture.staleThresholdMilliseconds == 0U)
    {
        return fail("Desktop capture timeout, retry and stale threshold must be positive.");
    }
    if (options.desktopCaptureSmokeTestFrames == 0U)
    {
        return fail("--desktop-capture-smoke-test-frames must be positive.");
    }
    if (options.desktopCapture.crossAdapterPolicy == capture::CrossAdapterPolicy::cpuFallback
        && !options.desktopCapture.allowCpuFallback)
    {
        return fail("CPU desktop-capture fallback requires --allow-desktop-capture-cpu-fallback.");
    }
    if (options.desktopCapture.panelContent == capture::PanelContent::desktop
        && !options.desktopDebugCheckerboard
        && !options.desktopCapture.captureMonitor.index.has_value()
        && !options.desktopCapture.captureMonitor.deviceName.has_value())
    {
        return fail("Desktop panel mode requires --capture-monitor-index or --capture-monitor-device-name.");
    }
    for (std::size_t panelIndex = 0; panelIndex < options.panelScene.panelCount; ++panelIndex)
    {
        const auto& panel = options.panelScene.panels[panelIndex];
        if (panel.content.kind == PanelContentKind::desktop
            && !panel.content.captureMonitorIndex.has_value()
            && !panel.content.captureMonitorDeviceName.has_value()
            && !panel.content.captureMonitorStableId.has_value())
        {
            return fail("Each desktop panel requires its own capture monitor selector.");
        }
    }
    if (options.desktopCaptureSmokeTest)
    {
        options.desktopCapture.panelContent = capture::PanelContent::desktop;
        options.orientationDemoMode = true;
        options.smokeTest = true;
        options.smokeTestFrames = options.desktopCaptureSmokeTestFrames;
    }
    if (options.desktopDebugCheckerboard)
    {
        options.desktopCapture.panelContent = capture::PanelContent::desktop;
    }
    if (options.predictionHorizonMilliseconds < 0.0
        || options.predictionMaximumHorizonMilliseconds <= 0.0
        || options.predictionMaximumAngularSpeedDegreesPerSecond <= 0.0
        || options.predictionMaximumAngleDegrees <= 0.0)
    {
        return fail("Prediction horizons and limits are invalid.");
    }
    if (options.predictionHorizonMilliseconds > options.predictionMaximumHorizonMilliseconds
        && options.predictionLimitBehavior == sensors::PredictionLimitBehavior::reject)
    {
        return fail("Prediction horizon exceeds its maximum; use clamp explicitly to bound it.");
    }
    if (predictionOptionExplicit
        && options.orientationSource != RenderOrientationSource::predicted)
    {
        return fail("Prediction options require --render-orientation-source predicted.");
    }
    if (options.smokeTest)
    {
        options.orientationDemoMode = true;
    }
    if (options.xrealSdkPose && options.orientationDemoMode)
    {
        return fail("--xreal-sdk-pose cannot be combined with orientation demo or smoke-test modes.");
    }
    if (options.xrealSdkPose && options.predictOrientation)
    {
        return fail("--predict-orientation is not used with the XREAL PC SDK pose source.");
    }
    if (options.captureBenchmark)
    {
        options.multiPanelBenchmark = true;
        options.multiPanelBenchmarkSeconds = options.captureBenchmarkSeconds;
        if (options.captureBenchmarkJsonPath.has_value())
        {
            options.multiPanelBenchmarkJsonPath = options.captureBenchmarkJsonPath;
        }
        if (!panelConfigurationExplicit)
        {
            options.panelScene = makeDefaultPanelScene(options.captureBenchmarkSources);
            options.panelScene.performanceProfile = PerformanceProfile::performance;
            for (std::size_t panelIndex = 0U;
                 panelIndex < options.panelScene.panelCount; ++panelIndex)
            {
                auto& panel = options.panelScene.panels[panelIndex];
                panel.content.kind = PanelContentKind::desktop;
                panel.content.captureMonitorIndex = options.desktopCapture.captureMonitor.index;
                panel.content.captureMonitorDeviceName =
                    options.desktopCapture.captureMonitor.deviceName;
                panel.content.transferPolicy = options.desktopCapture.crossAdapterPolicy
                    == capture::CrossAdapterPolicy::sharedHandle
                    ? PanelTransferPolicy::sharedHandle
                    : options.desktopCapture.crossAdapterPolicy
                        == capture::CrossAdapterPolicy::cpuFallback
                        ? PanelTransferPolicy::cpuFallback
                        : options.desktopCapture.crossAdapterPolicy
                            == capture::CrossAdapterPolicy::reject
                            ? PanelTransferPolicy::reject
                            : PanelTransferPolicy::automatic;
                panel.content.scaling.resolutionPolicy =
                    capture::DesktopResolutionPolicy::fixed;
                panel.content.scaling.targetWidth = options.captureBenchmarkTargetWidth;
                panel.content.scaling.targetHeight = options.captureBenchmarkTargetHeight;
                panel.content.requestedWidth = options.captureBenchmarkTargetWidth;
                panel.content.requestedHeight = options.captureBenchmarkTargetHeight;
                panel.targetFramesPerSecond = options.captureBenchmarkFramesPerSecond;
                panel.targetFramesPerSecondExplicit = true;
                panel.content.captureBenchmarkInstance =
                    static_cast<std::uint8_t>(panelIndex);
            }
            refreshPanelWorldTransforms(options.panelScene);
        }
    }
    if (options.multiPanelBenchmark)
    {
        if (options.multiPanelBenchmarkContent == PanelContentKind::desktop
            && !options.desktopCapture.captureMonitor.index.has_value()
            && !options.desktopCapture.captureMonitor.deviceName.has_value())
        {
            return fail("Desktop benchmark content requires --capture-monitor-index or --capture-monitor-device-name.");
        }
        options.orientationDemoMode = true;
        options.orientationDemoStatic = true;
        if (!panelConfigurationExplicit && !options.captureBenchmark)
        {
            options.panelScene = makeDefaultPanelScene(options.multiPanelBenchmarkPanels);
        }
        options.panelScene.performanceProfile = PerformanceProfile::performance;
        if (!panelConfigurationExplicit && !options.captureBenchmark)
        {
            for (std::size_t index = 0; index < options.panelScene.panelCount; ++index)
            {
                auto& panel = options.panelScene.panels[index];
                panel.content.kind = options.multiPanelBenchmarkContent;
                panel.content.transferPolicy = options.desktopCapture.crossAdapterPolicy
                    == capture::CrossAdapterPolicy::sharedHandle
                    ? PanelTransferPolicy::sharedHandle
                    : options.desktopCapture.crossAdapterPolicy
                        == capture::CrossAdapterPolicy::cpuFallback
                        ? PanelTransferPolicy::cpuFallback
                        : options.desktopCapture.crossAdapterPolicy
                            == capture::CrossAdapterPolicy::reject
                            ? PanelTransferPolicy::reject
                            : PanelTransferPolicy::automatic;
                panel.fit = options.desktopCapture.fit == capture::DesktopFit::cover
                    ? PanelFitMode::cover
                    : options.desktopCapture.fit == capture::DesktopFit::stretch
                        ? PanelFitMode::stretch : PanelFitMode::contain;
                panel.filter = options.desktopCapture.filter
                    == capture::DesktopFilter::point
                    ? PanelFilterMode::point : PanelFilterMode::linear;
                if (options.multiPanelBenchmarkContent == PanelContentKind::desktop)
                {
                    panel.content.captureMonitorIndex =
                        options.desktopCapture.captureMonitor.index;
                    panel.content.captureMonitorDeviceName =
                        options.desktopCapture.captureMonitor.deviceName;
                }
            }
        }
        options.renderDurationSeconds = options.multiPanelBenchmarkWarmupSeconds
            + options.multiPanelBenchmarkSeconds;
    }
    if (!options.orientationDemoMode)
    {
        if (!options.applyGyroscopeBias || !options.gyroscopeScaleRawPerDegreePerSecond.has_value()
            || !options.accelerometerProfilePath.has_value())
        {
            return fail("Hardware mode requires --apply-gyro-bias, --gyro-scale-raw-per-dps and --accelerometer-profile.");
        }
        if (options.orientationSource == RenderOrientationSource::predicted
            && !options.predictOrientation)
        {
            return fail("Predicted hardware rendering requires --predict-orientation.");
        }
    }
    if (options.gyroscopeCalibrationSeconds <= 0.0 || options.gyroscopeWarmupSeconds < 0.0
        || options.accelerometerCorrectionTimeConstantSeconds <= 0.0
        || options.accelerometerMaximumCorrectionDegreesPerSecond <= 0.0)
    {
        return fail("Calibration and fusion timing values are invalid.");
    }
    return {options, {}};
}

std::string rendererUsage()
{
    return "xreal-spatial-renderer [--orientation-demo-mode|--xreal-sdk-pose] [--window-width <pixels>] "
           "[--window-height <pixels>] [--window-x <pixels>] [--window-y <pixels>] "
           "[--fullscreen|--borderless] [--monitor-index <index>|--render-monitor-index <index>] "
           "[--render-monitor-device-name <name>] [--vsync|--no-vsync] "
           "[--allow-warp-fallback] [--render-duration <seconds>] [--target-fps <fps>] "
           "[--show-render-diagnostics] [--render-diagnostics-rate <hz>] "
           "[--panel-count <1|2|3>] "
           "[--panel-layout <single|dual-flat|triple-flat|triple-angled|custom>] "
           "[--panel-width-m <value>] [--panel-height-m <value>] "
           "[--panel-distance-m <value>] [--panel-gap-m <value>] "
           "[--panel-curvature-degrees <value>] "
           "[--panel-1-content <synthetic|checkerboard|desktop|unavailable>] "
           "[--panel-N-capture-monitor-index <index>] "
           "[--panel-N-capture-monitor-stable-id <identity>] "
           "[--panel-N-width-m <value>] [--panel-N-position-x-m <value>] "
           "[--panel-N-yaw-degrees <value>] [--panel-N-target-fps <value>] "
           "[--panel-layout-file <file.json>] [--panel-layout-save-file <file.json>] "
           "[--save-panel-layout-on-exit] [--overwrite-panel-layout] "
           "[--performance-profile <quality|balanced|performance>] "
           "[--multi-panel-benchmark] [--multi-panel-benchmark-seconds <value>] "
           "[--multi-panel-benchmark-warmup-seconds <value>] "
           "[--multi-panel-benchmark-panels <1|2|3>] "
           "[--multi-panel-benchmark-content <synthetic|checkerboard|desktop>] "
           "[--multi-panel-benchmark-json <file.json>] "
           "[--performance-json-output <file.json>] "
           "[--desktop-resolution-policy <native|panel-aware|fixed>] "
           "[--desktop-resolution-safety-factor <value>] "
           "[--desktop-bandwidth-warning-mib-s <value>] "
           "[--desktop-source-N-target-width <pixels>] "
           "[--desktop-source-N-target-height <pixels>] "
           "[--desktop-source-N-scale <value>] "
           "[--desktop-source-N-crop <full|center-16x9|custom>] "
           "[--desktop-source-N-region <x> <y> <width> <height>] "
           "[--desktop-source-N-fit <contain|cover|stretch>] "
           "[--desktop-source-N-filter <point|linear>] "
           "[--desktop-source-N-allow-upscale] "
           "[--desktop-source-N-capture-fps <value>] "
           "[--desktop-source-N-upload-fps <value>] "
           "[--capture-benchmark] [--capture-benchmark-sources <1|2|3>] "
           "[--capture-benchmark-seconds <value>] "
           "[--capture-benchmark-target-width <pixels>] "
           "[--capture-benchmark-target-height <pixels>] "
           "[--capture-benchmark-fps <value>] "
           "[--capture-benchmark-json <file.json>] "
           "[--panel-distance <value>] [--panel-width <value>] [--panel-height <value>] "
           "[--field-of-view-degrees <value>] [--near-plane <value>] [--far-plane <value>] "
           "[--background-grid] [--world-axes] "
           "[--orientation-demo-static] "
           "[--xreal-sdk-pose] "
           "[--render-orientation-source <measured|predicted>] "
           "[--render-orientation-frame <absolute|relative>] [--recenter-on-start] "
           "[--predict-orientation] [--prediction-mode <constant-velocity|constant-acceleration>] "
           "[--prediction-horizon-ms <value>] [--render-json-output <file.json>] "
           "[--panel-content <synthetic|desktop>] [--capture-monitor-index <index>] "
           "[--capture-monitor-device-name <name>] [--desktop-fit <contain|cover|stretch>] "
           "[--desktop-filter <point|linear>] [--desktop-show-cursor|--desktop-hide-cursor] "
           "[--desktop-capture-cross-adapter <auto|shared-handle|cpu-fallback|reject>] "
           "[--desktop-capture-diagnostics] [--desktop-capture-json-output <file.json>] "
           "[--desktop-debug-checkerboard] "
           "[--desktop-debug-shader-solid-red|--desktop-debug-shader-uv|"
           "--desktop-debug-shader-sample|--desktop-debug-shader-sample-no-overlay] "
           "[--desktop-debug-opaque-base] "
           "[--desktop-debug-readback-upload <file.bmp>] "
           "[--desktop-debug-dump-render-target <file.bmp>] "
           "[--desktop-capture-dump-first-frame <file.bmp>] "
           "[--smoke-test] [--smoke-test-frames <count>]\n";
}

} // namespace xreal::rendering
